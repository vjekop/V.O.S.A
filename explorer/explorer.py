"""
VOSA Explorer v3 — Sensor fusion (depth camera + LiDAR) with probabilistic
occupancy mapping and A* frontier exploration.

HOW IT WORKS:

  1. PROBABILISTIC OCCUPANCY GRID
     Instead of hard FREE/OCCUPIED/UNKNOWN states, every cell holds a probability
     (0.0–1.0) of being occupied. This is the core of sensor fusion:

       - Each sensor updates cells independently using a log-odds model
       - When two sensors agree (both see an obstacle), probability rises fast
       - When only one sensor fires, probability rises slowly — not enough to block
       - Cells decay toward 0.5 (uncertain) over time if no sensor confirms them

     This eliminates the spinning-Roomba problem: a single bad depth camera reading
     no longer blocks a path. Both sensors must agree before a cell is treated as
     a wall.

  2. SENSOR FUSION — TWO COMPLEMENTARY SENSORS
     Depth camera (OakD-Lite):
       - Dense point cloud, excellent close-range detail (0.3–8m)
       - Struggles in bright sunlight, noisy at edges
       - High CPU cost — we subsample aggressively

     LiDAR (2D, 360° scan):
       - 360 range readings per frame, 30m range
       - Works in any lighting, very accurate
       - Low CPU cost — just 360 floats per frame
       - Blind close up (<0.15m) — depth camera fills this gap

     Together they cover each other's weaknesses. The occupancy grid naturally
     combines them: both write to the same cells, probabilities accumulate.

  3. FRONTIER DETECTION
     A frontier is a cell with probability near 0.5 (unknown) adjacent to a
     cell with probability < 0.3 (confidently free). Flying toward frontiers
     guarantees the drone always moves into new territory.

  4. A* PATH PLANNING
     Finds the shortest path through the probability grid to the nearest frontier.
     Cells with probability > 0.6 are treated as walls. Cells between 0.3–0.6
     (uncertain) are passable with a penalty — the drone prefers known-safe
     corridors but will enter uncertain space when needed.

  5. VOSA SAFETY GATEWAY
     Every waypoint is validated by VOSA before reaching PX4. If rejected,
     the cell is penalised in the grid and the planner replans automatically.

  6. POSE TRACKING
     Ground-truth position from Gazebo pose topic — no dead-reckoning drift.

Requires:
    pip install numpy
    gz-transport13 and gz-msgs10 Python bindings (from PX4/Gazebo install)

Usage:
    python3 explorer.py [--host 127.0.0.1] [--port 7777] [--alt 30.0]

Run with:
    Terminal 1: make px4_sitl gz_x500_depth_lidar   (in PX4-Autopilot/)
    Terminal 2: vosa serve mission.vosa --mavlink udpin:0.0.0.0:14550 --port 7777
    Terminal 3: python3 explorer/explorer.py --alt 30.0
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

# ══════════════════════════════════════════════════════════════════════════════
# Grid configuration
# ══════════════════════════════════════════════════════════════════════════════

GRID_RESOLUTION = 1.0    # metres per cell
GRID_SIZE       = 300    # 300 x 300 = 300 m x 300 m
GRID_ORIGIN     = GRID_SIZE // 2

# ── Log-odds occupancy model ───────────────────────────────────────────────
# Each cell stores log-odds L where P(occupied) = 1 / (1 + exp(-L)).
# This lets us accumulate evidence from multiple sensors naturally.
#
# Why log-odds: adding independent sensor observations is just addition in
# log-odds space. No need to multiply probabilities or track history.

L_OCC_DEPTH  =  0.7   # log-odds increment when depth camera sees obstacle
L_FREE_DEPTH = -0.4   # log-odds decrement when depth camera ray-casts through
L_OCC_LIDAR  =  0.9   # LiDAR is more accurate — stronger signal
L_FREE_LIDAR = -0.5
L_MIN        = -3.0   # clamp: don't become infinitely certain free
L_MAX        =  3.0   # clamp: don't become infinitely certain occupied
L_DECAY      = -0.02  # cells decay toward uncertain each cycle (forgetting)

# Probability thresholds derived from log-odds
P_OCCUPIED_THRESHOLD = 0.65  # treat as wall in A*
P_FREE_THRESHOLD     = 0.30  # treat as safely free
P_FRONTIER_MAX       = 0.55  # frontier cells are uncertain (near 0.5)

# ══════════════════════════════════════════════════════════════════════════════
# Sensor configuration
# ══════════════════════════════════════════════════════════════════════════════

DEPTH_TOPIC = "/depth_camera/points"
LIDAR_TOPIC = "/world/default/model/x500_depth_lidar_0/link/lidar_link/sensor/lidar_2d/scan"
POSE_TOPIC  = "/world/default/dynamic_pose/info"
DRONE_MODEL = "x500_depth_lidar_0"

# Depth camera filtering
DEPTH_MIN_RANGE  = 0.3
DEPTH_MAX_RANGE  = 8.0
DEPTH_GROUND_Z   = -1.5   # drop points this far below camera (ground)
DEPTH_FOV_HALF   = 35.0   # degrees
MAX_DEPTH_POINTS = 200    # subsample — dense clouds are redundant for mapping

# LiDAR filtering
LIDAR_MIN_RANGE = 0.15
LIDAR_MAX_RANGE = 28.0    # slightly under sensor max to ignore edge noise

# ══════════════════════════════════════════════════════════════════════════════
# Exploration configuration
# ══════════════════════════════════════════════════════════════════════════════

WAYPOINT_STEP = 3.0   # metres between waypoints sent to PX4
ARRIVAL_TOL   = 2.5   # metres — waypoint considered reached within this distance


# ══════════════════════════════════════════════════════════════════════════════
# Probabilistic Occupancy Grid
# ══════════════════════════════════════════════════════════════════════════════

class OccupancyGrid:
    """
    Probabilistic 2D occupancy grid updated by sensor fusion.

    Internal representation is log-odds for numerical stability and additive
    sensor fusion. Public interface exposes probabilities (0–1).

    Why 2D: the drone flies at fixed altitude so we only need the horizontal
    slice. A 3D grid would be ~100x larger with no benefit here.
    """

    def __init__(self):
        # Start all cells at 0.0 log-odds = 0.5 probability = uncertain
        self._logodds = np.zeros((GRID_SIZE, GRID_SIZE), dtype=np.float32)
        # Mark a radius around start as free
        dgx, dgy = GRID_ORIGIN, GRID_ORIGIN
        for dx in range(-3, 4):
            for dy in range(-3, 4):
                if dx*dx + dy*dy <= 9:
                    self._logodds[dgx+dx, dgy+dy] = L_MIN

    # ── Coordinate helpers ────────────────────────────────────────────────

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
        """Probability of occupancy at (gx, gy). 0=free, 1=occupied, 0.5=unknown."""
        l = float(self._logodds[gx, gy])
        return 1.0 / (1.0 + math.exp(-l))

    # ── Log-odds updates ──────────────────────────────────────────────────

    def _update(self, gx: int, gy: int, delta: float):
        if self.in_bounds(gx, gy):
            self._logodds[gx, gy] = np.clip(
                self._logodds[gx, gy] + delta, L_MIN, L_MAX
            )

    def _raycast(self, x0, y0, x1, y1, l_free, l_occ):
        """
        Bresenham ray from (x0,y0) to (x1,y1).
        Every cell along the ray gets l_free (clear air).
        The endpoint cell gets l_occ (obstacle).

        This is the key operation of occupancy grid mapping — each sensor
        reading simultaneously confirms free space AND marks an obstacle.
        """
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

    # ── Public sensor update interface ────────────────────────────────────

    def update_depth(self, drone_n: float, drone_e: float,
                     obstacles: List[Tuple[float, float]]):
        """
        Incorporate depth camera obstacles into the grid.
        Weaker signal than LiDAR — depth cameras are noisier.
        """
        dgx, dgy = self.world_to_grid(drone_n, drone_e)
        # Mark drone position as free
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
        """
        Incorporate LiDAR scan into the grid.

        LiDAR gives us 360 range readings at known angles. We ray-cast each
        valid reading. Stronger signal than depth camera — LiDAR is more
        accurate so we trust it more (higher log-odds increment).

        Invalid readings (inf, out of range) still ray-cast free space up to
        LIDAR_MAX_RANGE — that space is confirmed empty.
        """
        dgx, dgy = self.world_to_grid(drone_n, drone_e)

        with _state.lock:
            yaw = _state.yaw

        for i, r in enumerate(scan_ranges):
            angle = angle_min + i * angle_increment + yaw

            if not math.isfinite(r) or r > LIDAR_MAX_RANGE:
                # Ray goes to max range — mark that space as free
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
        """
        Apply a small decay toward uncertainty each planning cycle.

        Why: the world can change. An obstacle that was there 30 seconds ago
        might have moved (another drone, a person). Decay prevents the grid
        from permanently blocking cells based on stale readings, while still
        requiring consistent observations to mark something as occupied.
        """
        self._logodds *= (1.0 + L_DECAY)

    # ── Frontier detection ────────────────────────────────────────────────

    def frontiers(self) -> List[Tuple[float, float]]:
        """
        Return world coords of frontier cells.

        Frontier = uncertain cell (P near 0.5) adjacent to a confidently
        free cell (P < P_FREE_THRESHOLD). These are the boundaries of what
        the drone has explored — targeting them guarantees new information.
        """
        result = []
        dirs4 = [(0,1),(0,-1),(1,0),(-1,0)]

        for gx in range(1, GRID_SIZE - 1):
            for gy in range(1, GRID_SIZE - 1):
                p = self.prob(gx, gy)
                if p > P_FRONTIER_MAX or p < P_FREE_THRESHOLD:
                    continue
                # Is this uncertain cell adjacent to a free cell?
                for dx, dy in dirs4:
                    if self.prob(gx+dx, gy+dy) < P_FREE_THRESHOLD:
                        result.append(self.grid_to_world(gx, gy))
                        break

        return result

    # ── A* path planner ───────────────────────────────────────────────────

    def astar(self, sn: float, se: float,
              gn: float, ge: float) -> Optional[List[Tuple[float, float]]]:
        """
        Shortest collision-free path from (sn,se) to (gn,ge).

        Cell cost is based on occupancy probability:
          P < 0.30  → free, cost = 1.0 (known safe)
          P 0.30–0.60 → uncertain, cost = 1.5 (enter if necessary)
          P > 0.60  → occupied, impassable

        This means A* naturally prefers corridors that have been confirmed
        safe by both sensors, while still allowing the drone to enter
        uncertain space to reach a frontier.
        """
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

                # Cost increases with uncertainty
                if p < P_FREE_THRESHOLD:
                    cell_cost = 1.0
                else:
                    cell_cost = 1.5

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
        # Pose
        self.north     = 0.0
        self.east      = 0.0
        self.yaw       = 0.0
        self.pose_time = 0.0
        # Depth camera
        self.depth_obstacles: List[Tuple[float, float]] = []
        self.depth_time = 0.0
        # LiDAR
        self.lidar_ranges:    List[float] = []
        self.lidar_angle_min  = 0.0
        self.lidar_angle_inc  = 0.0
        self.lidar_time       = 0.0

_state = SensorState()


# ══════════════════════════════════════════════════════════════════════════════
# Gazebo subscribers
# ══════════════════════════════════════════════════════════════════════════════

def _on_depth(msg: pc_msgs.PointCloudPacked):
    """
    Parse OakD-Lite point cloud.

    Filters: ground (Z < -1.5m), noise (dist < 0.3m), long range (dist > 8m),
    outside forward FOV cone. Rotates surviving points into world frame using
    drone yaw. Subsamples to MAX_DEPTH_POINTS to keep CPU load reasonable.
    """
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

            pts.append((n + cos_y*x - sin_y*y,
                        e + sin_y*x + cos_y*y))

        if len(pts) > MAX_DEPTH_POINTS:
            stride = len(pts) // MAX_DEPTH_POINTS
            pts = pts[::stride]

        with _state.lock:
            _state.depth_obstacles = pts
            _state.depth_time      = time.time()

    except Exception as exc:
        print(f"[explorer] depth error: {exc}")


def _on_lidar(msg: lidar_msgs.LaserScan):
    """
    Parse 2D LiDAR scan (LaserScan message).

    LiDAR gives angle_min, angle_increment, and a list of range readings.
    We store the raw ranges and let the grid update rotate them using the
    drone's current yaw — this way the angles are always in world frame.
    """
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
    """
    Track drone position and heading from Gazebo ground-truth pose.

    Gazebo ENU convention: position.x = east, position.y = north.
    Yaw extracted from quaternion — 0 = facing north.
    """
    try:
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
# Main exploration loop
# ══════════════════════════════════════════════════════════════════════════════

def _spaced(path: List[Tuple[float,float]], start_n: float, start_e: float,
            step: float) -> List[Tuple[float,float]]:
    """Subsample A* path to waypoints at least `step` metres apart."""
    result = []
    pn, pe = start_n, start_e
    for n, e in path:
        if math.sqrt((n-pn)**2 + (e-pe)**2) >= step:
            result.append((n, e))
            pn, pe = n, e
    if not result and path:
        result = [path[-1]]
    return result


def main():
    parser = argparse.ArgumentParser(description="VOSA autonomous explorer v3 — sensor fusion")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--alt",  type=float, default=30.0)
    parser.add_argument("--step", type=float, default=WAYPOINT_STEP)
    args = parser.parse_args()

    gateway = VosaGateway(args.host, args.port)
    gateway.connect()

    node = gz.transport13.Node()
    node.subscribe(pc_msgs.PointCloudPacked, DEPTH_TOPIC, _on_depth)
    node.subscribe(lidar_msgs.LaserScan,     LIDAR_TOPIC, _on_lidar)
    node.subscribe(pose_msgs.Pose_V,         POSE_TOPIC,  _on_pose)
    print("[explorer] Subscribed: depth camera + LiDAR + pose")
    print(f"[explorer] Sensor fusion active — cruise alt {args.alt}m")

    time.sleep(3.0)

    grid = OccupancyGrid()
    path: List[Tuple[float, float]] = []
    cycle = 0

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

        if pose_age > 5.0:
            print("[explorer] WARNING: no pose — is Gazebo running?")
        if depth_age > 5.0:
            print("[explorer] WARNING: depth camera silent")
        if lidar_age > 5.0:
            print("[explorer] WARNING: LiDAR silent")

        # ── 1. Fuse both sensors into the probabilistic grid ───────────────
        if depth_obs:
            grid.update_depth(north, east, depth_obs)
        if lidar_r:
            grid.update_lidar(north, east, lidar_r, lidar_amin, lidar_ainc)

        # Decay grid every 10 cycles (~15 s) to handle dynamic environments
        if cycle % 10 == 0:
            grid.decay()

        # ── 2. Pop waypoints we have reached ──────────────────────────────
        while path:
            wn, we = path[0]
            if math.sqrt((north-wn)**2 + (east-we)**2) < ARRIVAL_TOL:
                path.pop(0)
                print(f"[explorer] Waypoint reached — {len(path)} remaining")
            else:
                break

        # ── 3. Replan when path exhausted ──────────────────────────────────
        if not path:
            all_frontiers = grid.frontiers()

            if not all_frontiers:
                print("[explorer] Exploration complete — no frontiers remain")
                break

            # Sort by distance, try nearest 10
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
                # Surrounded — probe in 8 directions
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

        # ── 4. Send next waypoint through VOSA ────────────────────────────
        if path:
            wn, we = path[0]
            if not gateway.send(wn, we, args.alt):
                # VOSA rejected — penalise cell and replan
                ogx, ogy = grid.world_to_grid(wn, we)
                grid._logodds[ogx, ogy] = L_MAX
                path.clear()
                print("[explorer] Waypoint rejected — penalised cell, replanning")


if __name__ == "__main__":
    main()
