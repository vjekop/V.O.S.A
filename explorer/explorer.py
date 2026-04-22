"""
VOSA Explorer v4 — Search-and-Rescue

Two modes selectable via --mode:

  frontier  (default from v3) — probabilistic occupancy + A* frontier exploration
  search    — lawnmower/spiral sweep optimised for finding a person

Search-and-rescue pipeline (--mode search):
  1. Fly a systematic lawnmower pattern covering --search-width x --search-length metres
  2. On each cycle, analyse the downward camera frame for a person
  3. When detected: stop, loiter, and alert operator

Person detection:
  - Subscribes to the RGB camera topic
  - Looks for a bright-red target in the frame (works with a red Gazebo actor/sphere)
  - Threshold tunable with --detection-threshold (0–1, default 0.003)
  - Detection saved to person_detected.jpg when found

Sensor fusion (both modes):
  - Probabilistic occupancy grid (log-odds, depth + LiDAR)
  - A* path planning through grid (frontier mode only)
  - Pose from Gazebo ground-truth

Usage:
    python3 explorer/explorer.py --alt 30.0 --mode search
    python3 explorer/explorer.py --alt 30.0 --mode frontier

Run with:
    Terminal 1: make px4_sitl gz_x500_depth_lidar   (in PX4-Autopilot/)
    Terminal 2: vosa serve mission.vosa --mavlink udpin:0.0.0.0:14550 --port 7777
    Terminal 3: python3 explorer/explorer.py --alt 30.0 --mode search
"""

import argparse
import math
import socket
import struct
import threading
import time
from collections import defaultdict
from heapq import heappush, heappop
from typing import List, Optional, Tuple

import numpy as np

import gz.transport13
import gz.msgs10.pointcloud_packed_pb2  as pc_msgs
import gz.msgs10.pose_v_pb2            as pose_msgs
import gz.msgs10.laserscan_pb2         as lidar_msgs
import gz.msgs10.image_pb2             as img_msgs

# ══════════════════════════════════════════════════════════════════════════════
# Grid configuration
# ══════════════════════════════════════════════════════════════════════════════

GRID_RESOLUTION = 1.0    # metres per cell
GRID_SIZE       = 300    # 300 x 300 = 300 m x 300 m
GRID_ORIGIN     = GRID_SIZE // 2

# ── Log-odds occupancy model ───────────────────────────────────────────────

L_OCC_DEPTH  =  0.7
L_FREE_DEPTH = -0.4
L_OCC_LIDAR  =  0.9
L_FREE_LIDAR = -0.5
L_MIN        = -3.0
L_MAX        =  3.0
L_DECAY      = -0.02

P_OCCUPIED_THRESHOLD = 0.65
P_FREE_THRESHOLD     = 0.30
P_FRONTIER_MAX       = 0.55

# ══════════════════════════════════════════════════════════════════════════════
# Sensor configuration
# ══════════════════════════════════════════════════════════════════════════════

DEPTH_TOPIC  = "/depth_camera/points"
LIDAR_TOPIC  = "/world/default/model/x500_depth_lidar_0/link/lidar_link/sensor/lidar_2d/scan"
POSE_TOPIC   = "/world/default/dynamic_pose/info"
CAMERA_TOPIC = "/world/default/model/x500_depth_lidar_0/link/camera_link/sensor/IMX214/image"
DRONE_MODEL  = "x500_depth_lidar_0"

DEPTH_MIN_RANGE  = 0.3
DEPTH_MAX_RANGE  = 8.0
DEPTH_GROUND_Z   = -1.5
DEPTH_FOV_HALF   = 35.0
MAX_DEPTH_POINTS = 200

LIDAR_MIN_RANGE = 0.15
LIDAR_MAX_RANGE = 28.0

# ══════════════════════════════════════════════════════════════════════════════
# Exploration configuration
# ══════════════════════════════════════════════════════════════════════════════

WAYPOINT_STEP = 3.0
ARRIVAL_TOL   = 4.0


# ══════════════════════════════════════════════════════════════════════════════
# Probabilistic Occupancy Grid
# ══════════════════════════════════════════════════════════════════════════════

class OccupancyGrid:
    def __init__(self):
        self._logodds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        dgx, dgy = GRID_ORIGIN, GRID_ORIGIN
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                if dx*dx + dy*dy <= 9:
                    self._logodds[dgx+dx, dgy+dy] = L_MIN

    def world_to_grid(self, north: float, east: float) -> Tuple[int, int]:
        gx = int(round(GRID_ORIGIN + north / GRID_RESOLUTION))
        gy = int(round(GRID_ORIGIN + east  / GRID_RESOLUTION))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (gx - GRID_ORIGIN) * GRID_RESOLUTION, \
               (gy - GRID_ORIGIN) * GRID_RESOLUTION

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE

    def prob(self, gx: int, gy: int) -> float:
        l = float(self._logodds[gx, gy])
        return 1.0 / (1.0 + math.exp(-l))

    def _update(self, gx: int, gy: int, delta: float):
        if self.in_bounds(gx, gy):
            self._logodds[gx, gy] = np.clip(
                self._logodds[gx, gy] + delta, L_MIN, L_MAX
            )

    def _raycast(self, x0, y0, x1, y1, l_free, l_occ):
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        x, y = x0, y0
        while (x, y) != (x1, y1):
            self._update(x, y, l_free)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
        self._update(x1, y1, l_occ)

    def update_depth(self, drone_n: float, drone_e: float,
                     obstacles: List[Tuple[float, float]]):
        dgx, dgy = self.world_to_grid(drone_n, drone_e)
        for dx in range(-2, 3):
            for dy in range(-2, 3):
                self._update(dgx+dx, dgy+dy, L_FREE_DEPTH * 2)
        for obs_n, obs_e in obstacles:
            ogx, ogy = self.world_to_grid(obs_n, obs_e)
            if self.in_bounds(ogx, ogy):
                self._raycast(dgx, dgy, ogx, ogy, L_FREE_DEPTH, L_OCC_DEPTH)

    def update_lidar(self, drone_n: float, drone_e: float,
                     scan_ranges: List[float], angle_min: float,
                     angle_increment: float):
        dgx, dgy = self.world_to_grid(drone_n, drone_e)
        with _state.lock:
            yaw = _state.yaw
        for i, r in enumerate(scan_ranges):
            angle = angle_min + i * angle_increment + yaw
            if not math.isfinite(r) or r > LIDAR_MAX_RANGE:
                end_n = drone_n + LIDAR_MAX_RANGE * math.cos(angle)
                end_e = drone_e + LIDAR_MAX_RANGE * math.sin(angle)
                egx, egy = self.world_to_grid(end_n, end_e)
                if self.in_bounds(egx, egy):
                    self._raycast(dgx, dgy, egx, egy, L_FREE_LIDAR, L_FREE_LIDAR)
                continue
            if r < LIDAR_MIN_RANGE:
                continue
            obs_n = drone_n + r * math.cos(angle)
            obs_e = drone_e + r * math.sin(angle)
            ogx, ogy = self.world_to_grid(obs_n, obs_e)
            if self.in_bounds(ogx, ogy):
                self._raycast(dgx, dgy, ogx, ogy, L_FREE_LIDAR, L_OCC_LIDAR)

    def decay(self):
        self._logodds *= (1.0 + L_DECAY)

    def frontiers(self) -> List[Tuple[float, float]]:
        result = []
        dirs4 = [(0,1),(0,-1),(1,0),(-1,0)]
        for gx in range(1, GRID_SIZE - 1):
            for gy in range(1, GRID_SIZE - 1):
                p = self.prob(gx, gy)
                if p > P_FRONTIER_MAX or p < P_FREE_THRESHOLD:
                    continue
                for dx, dy in dirs4:
                    if self.prob(gx+dx, gy+dy) < P_FREE_THRESHOLD:
                        result.append(self.grid_to_world(gx, gy))
                        break
        return result

    def astar(self, sn: float, se: float,
              gn: float, ge: float) -> Optional[List[Tuple[float, float]]]:
        start = self.world_to_grid(sn, se)
        goal  = self.world_to_grid(gn, ge)
        if not self.in_bounds(*goal):
            return None
        if self.prob(goal[0], goal[1]) > P_OCCUPIED_THRESHOLD:
            return None
        counter = 0
        open_set = []
        heappush(open_set, (0.0, counter, start))
        came_from: dict = {}
        g: dict = defaultdict(lambda: float("inf"))
        g[start] = 0.0
        dirs8 = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        while open_set:
            _, _, current = heappop(open_set)
            if current == goal:
                path = []
                node = current
                while node in came_from:
                    path.append(self.grid_to_world(*node))
                    node = came_from[node]
                path.reverse()
                return path
            for dx, dy in dirs8:
                nb = (current[0]+dx, current[1]+dy)
                if not self.in_bounds(*nb):
                    continue
                p = self.prob(nb[0], nb[1])
                if p > P_OCCUPIED_THRESHOLD:
                    continue
                cell_cost = 1.0 if p < P_FREE_THRESHOLD else 1.5
                step = math.sqrt(dx*dx + dy*dy) * cell_cost
                new_g = g[current] + step
                if new_g < g[nb]:
                    came_from[nb] = current
                    g[nb] = new_g
                    h = math.sqrt((nb[0]-goal[0])**2 + (nb[1]-goal[1])**2)
                    counter += 1
                    heappush(open_set, (new_g + h, counter, nb))
        return None


# ══════════════════════════════════════════════════════════════════════════════
# Shared sensor state
# ══════════════════════════════════════════════════════════════════════════════

class SensorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.north      = 0.0
        self.east       = 0.0
        self.yaw        = 0.0
        self.pose_time  = 0.0
        self.depth_obstacles: List[Tuple[float, float]] = []
        self.depth_time = 0.0
        self.lidar_ranges:   List[float] = []
        self.lidar_angle_min = 0.0
        self.lidar_angle_inc = 0.0
        self.lidar_time      = 0.0
        # Camera
        self.camera_frame: Optional[bytes] = None
        self.camera_width  = 0
        self.camera_height = 0
        self.camera_time   = 0.0

_state = SensorState()


# ══════════════════════════════════════════════════════════════════════════════
# Gazebo subscribers
# ══════════════════════════════════════════════════════════════════════════════

def _on_depth(msg: pc_msgs.PointCloudPacked):
    try:
        offsets = {f.name: f.offset for f in msg.field}
        if "x" not in offsets:
            return
        step  = msg.point_step
        data  = msg.data
        x_off = offsets.get("x", 0)
        y_off = offsets.get("y", 4)
        z_off = offsets.get("z", 8)
        fov   = math.tan(math.radians(DEPTH_FOV_HALF))
        with _state.lock:
            n, e, yaw = _state.north, _state.east, _state.yaw
        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        pts: List[Tuple[float, float]] = []
        for i in range(0, len(data) - step + 1, step):
            x = struct.unpack_from("<f", data, i + x_off)[0]
            y = struct.unpack_from("<f", data, i + y_off)[0]
            z = struct.unpack_from("<f", data, i + z_off)[0]
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < DEPTH_GROUND_Z:
                continue
            dist = math.sqrt(x*x + y*y + z*z)
            if dist < DEPTH_MIN_RANGE or dist > DEPTH_MAX_RANGE:
                continue
            if x <= 0 or abs(y) > x * fov:
                continue
            pts.append((n + cos_y*x - sin_y*y, e + sin_y*x + cos_y*y))
        if len(pts) > MAX_DEPTH_POINTS:
            stride = len(pts) // MAX_DEPTH_POINTS
            pts = pts[::stride]
        with _state.lock:
            _state.depth_obstacles = pts
            _state.depth_time      = time.time()
    except Exception as exc:
        print(f"[explorer] depth error: {exc}")


def _on_lidar(msg: lidar_msgs.LaserScan):
    try:
        ranges = list(msg.ranges)
        with _state.lock:
            _state.lidar_ranges    = ranges
            _state.lidar_angle_min = msg.angle_min
            _state.lidar_angle_inc = msg.angle_step
            _state.lidar_time      = time.time()
    except Exception as exc:
        print(f"[explorer] lidar error: {exc}")


def _on_pose(msg: pose_msgs.Pose_V):
    try:
        names = [pose.name for pose in msg.pose]
        if not any(n == DRONE_MODEL for n in names):
            if not hasattr(_on_pose, '_logged'):
                print(f"[explorer] pose models: {names[:5]}")
                _on_pose._logged = True
        for pose in msg.pose:
            if pose.name != DRONE_MODEL:
                continue
            east  = pose.position.x
            north = pose.position.y
            qx, qy, qz, qw = (pose.orientation.x, pose.orientation.y,
                               pose.orientation.z, pose.orientation.w)
            yaw = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
            with _state.lock:
                _state.north     = north
                _state.east      = east
                _state.yaw       = yaw
                _state.pose_time = time.time()
            break
    except Exception as exc:
        print(f"[explorer] pose error: {exc}")


def _on_camera(msg: img_msgs.Image):
    """Capture raw RGB camera frame for person detection."""
    try:
        with _state.lock:
            _state.camera_frame  = bytes(msg.data)
            _state.camera_width  = msg.width
            _state.camera_height = msg.height
            _state.camera_time   = time.time()
    except Exception as exc:
        print(f"[explorer] camera error: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
# Person detection
# ══════════════════════════════════════════════════════════════════════════════

def detect_person(frame: bytes, width: int, height: int,
                  threshold: float) -> bool:
    """
    Detect a person (or red proxy target) in a raw RGB frame.

    Strategy: count pixels where R channel dominates strongly over G and B.
    A Gazebo human actor wearing default red clothes (or a red sphere proxy)
    will have a cluster of such pixels in the downward-facing camera view.

    threshold: fraction of frame pixels that must match (default 0.003 = 0.3%)
    Returns True if detected, False otherwise.
    """
    if not frame or width == 0 or height == 0:
        return False
    try:
        data = np.frombuffer(frame, dtype=np.uint8)
        # Determine number of channels from data length
        expected_rgb  = width * height * 3
        expected_rgba = width * height * 4
        if len(data) == expected_rgba:
            data = data.reshape((height, width, 4))
            r, g, b = data[:,:,0].astype(int), data[:,:,1].astype(int), data[:,:,2].astype(int)
        elif len(data) == expected_rgb:
            data = data.reshape((height, width, 3))
            r, g, b = data[:,:,0].astype(int), data[:,:,1].astype(int), data[:,:,2].astype(int)
        else:
            return False

        # Bright red: R > 150, R - G > 80, R - B > 80
        red_mask = (r > 150) & ((r - g) > 80) & ((r - b) > 80)
        ratio = red_mask.sum() / (width * height)
        return float(ratio) >= threshold
    except Exception as exc:
        print(f"[explorer] detection error: {exc}")
        return False


def save_detection_frame(frame: bytes, width: int, height: int,
                         north: float, east: float):
    """Save detected-person frame as PPM (no dependencies)."""
    try:
        data = np.frombuffer(frame, dtype=np.uint8)
        expected_rgb = width * height * 3
        if len(data) == width * height * 4:
            data = data.reshape((height, width, 4))[:,:,:3]
        elif len(data) == expected_rgb:
            data = data.reshape((height, width, 3))
        else:
            return
        path = f"person_detected_N{north:+.1f}_E{east:+.1f}.ppm"
        with open(path, "wb") as f:
            f.write(f"P6\n{width} {height}\n255\n".encode())
            f.write(data.tobytes())
        print(f"[explorer] Frame saved → {path}")
    except Exception as exc:
        print(f"[explorer] save error: {exc}")


# ══════════════════════════════════════════════════════════════════════════════
# Lawnmower search pattern
# ══════════════════════════════════════════════════════════════════════════════

def lawnmower_waypoints(start_n: float, start_e: float,
                        width: float, length: float,
                        strip_spacing: float) -> List[Tuple[float, float]]:
    """
    Generate a lawnmower (boustrophedon) search pattern.

    Covers a rectangle of `width` (N-S) x `length` (E-W) metres,
    starting at (start_n, start_e).  Strips run E-W, separated by
    `strip_spacing` metres (set to ~2x LiDAR range for coverage overlap).

    Returns ordered list of (north, east) waypoints.
    """
    wps: List[Tuple[float, float]] = []
    n = start_n
    east_fwd = True
    while n <= start_n + width:
        if east_fwd:
            wps.append((n, start_e))
            wps.append((n, start_e + length))
        else:
            wps.append((n, start_e + length))
            wps.append((n, start_e))
        n += strip_spacing
        east_fwd = not east_fwd
    return wps


# ══════════════════════════════════════════════════════════════════════════════
# VOSA gateway
# ══════════════════════════════════════════════════════════════════════════════

class VosaGateway:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self._sock: Optional[socket.socket] = None

    def connect(self):
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.connect((self.host, self.port))
        self._sock.settimeout(5.0)
        print(f"[explorer] Connected to VOSA at {self.host}:{self.port}")

    def send(self, north: float, east: float, alt: float) -> bool:
        cmd = f"waypoint north={north:.2f} east={east:.2f} alt={alt:.2f}\n"
        try:
            self._sock.sendall(cmd.encode())
            resp = self._sock.recv(256).decode().strip()
            accepted = resp == "ok"
            symbol = "✓" if accepted else "✗"
            print(f"[explorer] {symbol}  N={north:+6.1f}m  E={east:+6.1f}m  "
                  f"alt={alt:.1f}m  {'→ PX4' if accepted else resp}")
            return accepted
        except Exception as exc:
            print(f"[explorer] VOSA error: {exc}")
            return False


# ══════════════════════════════════════════════════════════════════════════════
# Helpers
# ══════════════════════════════════════════════════════════════════════════════

def _spaced(path: List[Tuple[float,float]], start_n: float, start_e: float,
            step: float) -> List[Tuple[float,float]]:
    result = []
    pn, pe = start_n, start_e
    for n, e in path:
        if math.sqrt((n-pn)**2 + (e-pe)**2) >= step:
            result.append((n, e))
            pn, pe = n, e
    if not result and path:
        result = [path[-1]]
    return result


# ══════════════════════════════════════════════════════════════════════════════
# Main
# ══════════════════════════════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(description="VOSA explorer v4 — search & rescue")
    parser.add_argument("--host",   default="127.0.0.1")
    parser.add_argument("--port",   type=int,   default=7777)
    parser.add_argument("--alt",    type=float, default=30.0,
                        help="Cruise altitude in metres (relative)")
    parser.add_argument("--step",   type=float, default=WAYPOINT_STEP,
                        help="Metres between waypoints")
    parser.add_argument("--mode",   choices=["frontier", "search"], default="search",
                        help="frontier=A*/occupancy, search=lawnmower SAR")
    # Search pattern parameters
    parser.add_argument("--search-width",   type=float, default=80.0,
                        help="N-S extent of search area in metres")
    parser.add_argument("--search-length",  type=float, default=80.0,
                        help="E-W extent of search area in metres")
    parser.add_argument("--strip-spacing",  type=float, default=15.0,
                        help="Spacing between lawnmower strips (metres)")
    # Detection
    parser.add_argument("--detection-threshold", type=float, default=0.003,
                        help="Fraction of red pixels to trigger person detection")
    args = parser.parse_args()

    gateway = VosaGateway(args.host, args.port)
    gateway.connect()

    node = gz.transport13.Node()
    node.subscribe(pc_msgs.PointCloudPacked, DEPTH_TOPIC,  _on_depth)
    node.subscribe(lidar_msgs.LaserScan,     LIDAR_TOPIC,  _on_lidar)
    node.subscribe(pose_msgs.Pose_V,         POSE_TOPIC,   _on_pose)
    node.subscribe(img_msgs.Image,           CAMERA_TOPIC, _on_camera)
    print("[explorer] Subscribed: depth camera + LiDAR + pose + RGB camera")
    print(f"[explorer] Mode: {args.mode}  altitude: {args.alt}m")

    time.sleep(3.0)

    grid = OccupancyGrid()
    path: List[Tuple[float, float]] = []
    cycle = 0
    person_found = False
    search_wps: List[Tuple[float, float]] = []
    search_started = False

    while True:
        time.sleep(1.5)
        cycle += 1

        # ── Snapshot sensor state ──────────────────────────────────────────
        with _state.lock:
            north       = _state.north
            east        = _state.east
            depth_obs   = list(_state.depth_obstacles)
            lidar_r     = list(_state.lidar_ranges)
            lidar_amin  = _state.lidar_angle_min
            lidar_ainc  = _state.lidar_angle_inc
            depth_age   = time.time() - _state.depth_time
            lidar_age   = time.time() - _state.lidar_time
            pose_age    = time.time() - _state.pose_time
            cam_frame   = _state.camera_frame
            cam_w       = _state.camera_width
            cam_h       = _state.camera_height
            cam_age     = time.time() - _state.camera_time

        if pose_age > 5.0:
            print("[explorer] WARNING: no pose — is Gazebo running?")
        else:
            print(f"[explorer] pose N={north:+.1f} E={east:+.1f} age={pose_age:.1f}s")
        if depth_age > 5.0:
            print("[explorer] WARNING: depth camera silent")
        if lidar_age > 5.0:
            print("[explorer] WARNING: LiDAR silent")

        # ── Person detection ───────────────────────────────────────────────
        if not person_found and cam_age < 3.0:
            if detect_person(cam_frame, cam_w, cam_h, args.detection_threshold):
                person_found = True
                print("\n" + "="*60)
                print("[explorer] *** PERSON DETECTED ***")
                print(f"[explorer]     Position: N={north:+.1f}m  E={east:+.1f}m")
                print("="*60 + "\n")
                save_detection_frame(cam_frame, cam_w, cam_h, north, east)
                # Stop exploration — hold position
                gateway.send(north, east, args.alt)
                break

        # ── Sensor fusion (both modes) ─────────────────────────────────────
        if depth_obs:
            grid.update_depth(north, east, depth_obs)
        if lidar_r:
            grid.update_lidar(north, east, lidar_r, lidar_amin, lidar_ainc)
        if cycle % 10 == 0:
            grid.decay()

        # ══════════════════════════════════════════════════════════════════
        # SEARCH MODE — lawnmower pattern
        # ══════════════════════════════════════════════════════════════════
        if args.mode == "search":
            # Build pattern once (starting from drone's current position)
            if not search_started:
                search_wps = lawnmower_waypoints(
                    north, east,
                    args.search_width,
                    args.search_length,
                    args.strip_spacing,
                )
                search_started = True
                print(f"[explorer] Lawnmower pattern: {len(search_wps)} waypoints  "
                      f"area={args.search_width}x{args.search_length}m  "
                      f"strips every {args.strip_spacing}m")

            # Pop reached waypoints
            while search_wps:
                wn, we = search_wps[0]
                dist = math.sqrt((north-wn)**2 + (east-we)**2)
                if dist < ARRIVAL_TOL:
                    search_wps.pop(0)
                    print(f"[explorer] Search waypoint reached — "
                          f"{len(search_wps)} remaining")
                else:
                    break

            if not search_wps:
                print("[explorer] Search pattern complete — no person found")
                break

            wn, we = search_wps[0]
            gateway.send(wn, we, args.alt)
            continue

        # ══════════════════════════════════════════════════════════════════
        # FRONTIER MODE — probabilistic A* exploration (original v3 logic)
        # ══════════════════════════════════════════════════════════════════
        while path:
            wn, we = path[0]
            if math.sqrt((north-wn)**2 + (east-we)**2) < ARRIVAL_TOL:
                path.pop(0)
                print(f"[explorer] Waypoint reached — {len(path)} remaining")
            else:
                break

        if not path:
            all_frontiers = grid.frontiers()
            if not all_frontiers:
                print("[explorer] Exploration complete — no frontiers remain")
                break
            all_frontiers.sort(
                key=lambda f: (f[0]-north)**2 + (f[1]-east)**2
            )
            planned = False
            for candidate in all_frontiers[:10]:
                route = grid.astar(north, east, candidate[0], candidate[1])
                if route:
                    path = _spaced(route, north, east, args.step)
                    print(
                        f"[explorer] Frontier N={candidate[0]:+.1f}m "
                        f"E={candidate[1]:+.1f}m  "
                        f"path={len(path)} wps  "
                        f"frontiers={len(all_frontiers)}"
                    )
                    planned = True
                    break
            if not planned:
                print("[explorer] No A* path — probing outward")
                for deg in range(0, 360, 45):
                    rad = math.radians(deg)
                    pn = north + args.step * math.cos(rad)
                    pe = east  + args.step * math.sin(rad)
                    pgx, pgy = grid.world_to_grid(pn, pe)
                    if grid.in_bounds(pgx, pgy) and \
                       grid.prob(pgx, pgy) < P_OCCUPIED_THRESHOLD:
                        path = [(pn, pe)]
                        break

        if path:
            wn, we = path[0]
            if not gateway.send(wn, we, args.alt):
                ogx, ogy = grid.world_to_grid(wn, we)
                grid._logodds[ogx, ogy] = L_MAX
                path.clear()
                print("[explorer] Waypoint rejected — penalised cell, replanning")


if __name__ == "__main__":
    main()
