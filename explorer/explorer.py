"""
VOSA Explorer v2 — Autonomous frontier exploration with occupancy mapping and A* path planning.

HOW IT WORKS (read this):

  1. MAPPING — OccupancyGrid
     The depth camera produces a point cloud every frame. We filter out ground
     points, noise, and out-of-range readings, then ray-cast each surviving point
     into a 2D grid. Every cell between the drone and an obstacle gets marked FREE
     (we can see through it), and the obstacle cell itself gets marked OCCUPIED.
     This is called inverse sensor modelling — the grid becomes a memory of what
     the drone has seen.

  2. FRONTIER DETECTION
     A "frontier" is any UNKNOWN cell with at least one FREE neighbour — the
     boundary between explored and unexplored space. Flying toward frontiers
     guarantees the drone systematically discovers new territory instead of
     retracing ground it has already covered.

  3. PATH PLANNING — A*
     Rather than flying straight at the frontier (which may pass through walls),
     A* searches the occupancy grid for the shortest collision-free path. UNKNOWN
     cells are passable but slightly penalised so the drone prefers known-safe
     corridors while still being willing to enter new space.

  4. SAFETY GATEWAY — VOSA
     Every waypoint is sent to VOSA over TCP before reaching PX4. VOSA checks
     geofence, altitude limits, and battery constraints. If a waypoint is rejected,
     the explorer marks that cell occupied and replans — the safety layer is never
     bypassed.

  5. POSE TRACKING
     Rather than dead-reckoning from sent waypoints, the explorer subscribes to
     Gazebo's pose topic for ground-truth position. This means replanning uses the
     drone's actual location, not where we hoped it would be.

Usage:
    python3 explorer.py [--host 127.0.0.1] [--port 7777] [--alt 30.0] [--step 3.0]

Requirements:
    pip install numpy gz-transport13 gz-msgs10
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
import gz.msgs10.pointcloud_packed_pb2 as pc_msgs
import gz.msgs10.pose_v_pb2 as pose_msgs

# ── Grid configuration ────────────────────────────────────────────────────────

# 1 metre per cell is coarse enough to plan quickly but fine enough to thread
# corridors between obstacles that are a few metres apart.
GRID_RESOLUTION = 1.0
GRID_SIZE       = 300    # 300 x 300 cells = 300 m x 300 m exploration area
GRID_ORIGIN     = GRID_SIZE // 2  # home position maps to the centre of the grid

UNKNOWN  = -1
FREE     =  0
OCCUPIED =  1

# ── Sensor configuration ──────────────────────────────────────────────────────

DEPTH_TOPIC = "/depth_camera/points"
POSE_TOPIC  = "/world/default/dynamic_pose/info"
DRONE_MODEL = "x500_depth_0"

# Depth camera filtering thresholds
MIN_RANGE    = 0.3    # metres — below this is sensor noise / self-reflection
MAX_RANGE    = 8.0    # metres — depth cameras lose accuracy beyond ~8 m
GROUND_Z_MIN = -1.5   # metres — points this far below the camera are ground
FOV_HALF_DEG = 35.0   # half-angle of the forward cone we trust

# How many obstacle points to keep per frame — depth cameras produce thousands
# of points but we only need a representative sample for mapping
MAX_POINTS_PER_FRAME = 300

# ── Exploration configuration ─────────────────────────────────────────────────

DEFAULT_STEP = 3.0   # metres between waypoints sent to PX4
ARRIVAL_TOL  = 2.5   # metres — consider a waypoint "reached" within this distance


# ══════════════════════════════════════════════════════════════════════════════
# OccupancyGrid
# ══════════════════════════════════════════════════════════════════════════════

class OccupancyGrid:
    """
    2D grid of FREE / OCCUPIED / UNKNOWN cells.

    We use 2D (not 3D) because the drone flies at a fixed cruise altitude, so
    we only care about the horizontal slice at that altitude. A 3D grid would be
    ~100x larger in memory and slower to plan through without meaningful benefit
    for a fixed-altitude vehicle.

    Coordinate convention: (north, east) in metres from home.
    North is +row, East is +column. This matches VOSA's waypoint convention.
    """

    def __init__(self):
        self.grid = np.full((GRID_SIZE, GRID_SIZE), UNKNOWN, dtype=np.int8)
        # Start position is known-free
        self._free_radius(GRID_ORIGIN, GRID_ORIGIN, 3)

    # ── Coordinate helpers ────────────────────────────────────────────────

    def world_to_grid(self, north: float, east: float) -> Tuple[int, int]:
        gx = int(round(GRID_ORIGIN + north / GRID_RESOLUTION))
        gy = int(round(GRID_ORIGIN + east  / GRID_RESOLUTION))
        return gx, gy

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        north = (gx - GRID_ORIGIN) * GRID_RESOLUTION
        east  = (gy - GRID_ORIGIN) * GRID_RESOLUTION
        return north, east

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < GRID_SIZE and 0 <= gy < GRID_SIZE

    # ── Cell updates ──────────────────────────────────────────────────────

    def _free(self, gx: int, gy: int):
        """Mark a cell free only if it is not already occupied."""
        if self.in_bounds(gx, gy) and self.grid[gx, gy] != OCCUPIED:
            self.grid[gx, gy] = FREE

    def _free_radius(self, gx: int, gy: int, r: int):
        for dx in range(-r, r + 1):
            for dy in range(-r, r + 1):
                if dx * dx + dy * dy <= r * r:
                    self._free(gx + dx, gy + dy)

    def _occupy(self, gx: int, gy: int):
        if self.in_bounds(gx, gy):
            self.grid[gx, gy] = OCCUPIED

    def _raycast(self, x0: int, y0: int, x1: int, y1: int):
        """
        Bresenham line from (x0,y0) to (x1,y1) — marks every cell along the
        ray as FREE, stopping one step before the endpoint.

        Why: the ray up to an obstacle is clear air; only the endpoint itself
        is the obstacle. Marking the endpoint free would erase wall information.
        """
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        x, y = x0, y0

        while (x, y) != (x1, y1):
            self._free(x, y)
            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy

    # ── Map update ────────────────────────────────────────────────────────

    def update(self, drone_north: float, drone_east: float,
               obstacle_world: List[Tuple[float, float]]):
        """
        Incorporate a new sensor scan into the map.

        For each obstacle point:
          - Ray-cast from drone → obstacle, marking free space
          - Mark obstacle cell occupied

        Also clears a small radius around the drone itself — we know we are
        here, so the immediate surroundings are free.
        """
        dgx, dgy = self.world_to_grid(drone_north, drone_east)
        self._free_radius(dgx, dgy, 3)

        for obs_n, obs_e in obstacle_world:
            ogx, ogy = self.world_to_grid(obs_n, obs_e)
            if not self.in_bounds(ogx, ogy):
                continue
            self._raycast(dgx, dgy, ogx, ogy)
            self._occupy(ogx, ogy)

    # ── Frontier detection ────────────────────────────────────────────────

    def frontiers(self) -> List[Tuple[float, float]]:
        """
        Return world coordinates of all frontier cells.

        A frontier is UNKNOWN with at least one FREE neighbour. These are the
        edges of what the drone knows — flying toward them always yields new
        information, guaranteeing the drone does not retrace explored ground.
        """
        result = []
        dirs4 = [(0, 1), (0, -1), (1, 0), (-1, 0)]

        for gx in range(1, GRID_SIZE - 1):
            for gy in range(1, GRID_SIZE - 1):
                if self.grid[gx, gy] != UNKNOWN:
                    continue
                for dx, dy in dirs4:
                    if self.grid[gx + dx, gy + dy] == FREE:
                        result.append(self.grid_to_world(gx, gy))
                        break

        return result

    # ── A* path planner ───────────────────────────────────────────────────

    def astar(self, sn: float, se: float,
              gn: float, ge: float) -> Optional[List[Tuple[float, float]]]:
        """
        Shortest collision-free path from (sn, se) to (gn, ge).

        8-connected grid search. OCCUPIED cells are walls. UNKNOWN cells are
        passable with a small penalty — the drone prefers corridors it knows
        are safe but will enter unknown space when that is the only way forward.
        This is what makes it an explorer rather than a navigator.

        Returns a list of (north, east) world coordinates, or None if no path
        exists (e.g. the goal is completely surrounded by obstacles).
        """
        start = self.world_to_grid(sn, se)
        goal  = self.world_to_grid(gn, ge)

        if not self.in_bounds(*goal):
            return None
        if self.grid[goal[0], goal[1]] == OCCUPIED:
            return None

        # tie-breaker counter prevents Python from comparing tuple elements
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
                nb = (current[0] + dx, current[1] + dy)
                if not self.in_bounds(*nb):
                    continue
                cell = self.grid[nb[0], nb[1]]
                if cell == OCCUPIED:
                    continue

                cost = math.sqrt(dx * dx + dy * dy)
                if cell == UNKNOWN:
                    cost += 0.5   # slight penalty for uncharted territory

                new_g = g[current] + cost
                if new_g < g[nb]:
                    came_from[nb] = current
                    g[nb] = new_g
                    h = math.sqrt((nb[0] - goal[0]) ** 2 + (nb[1] - goal[1]) ** 2)
                    counter += 1
                    heappush(open_set, (new_g + h, counter, nb))

        return None


# ══════════════════════════════════════════════════════════════════════════════
# Shared sensor state (written by subscriber threads, read by main loop)
# ══════════════════════════════════════════════════════════════════════════════

class SensorState:
    def __init__(self):
        self.lock = threading.Lock()
        self.north      = 0.0
        self.east       = 0.0
        self.yaw        = 0.0   # radians, 0 = facing north
        self.pose_time  = 0.0
        self.obstacles: List[Tuple[float, float]] = []
        self.cloud_time = 0.0

_state = SensorState()


# ══════════════════════════════════════════════════════════════════════════════
# Gazebo subscribers
# ══════════════════════════════════════════════════════════════════════════════

def _on_point_cloud(msg_data: bytes):
    """
    Parse depth camera point cloud and convert surviving points into world-frame
    obstacle coordinates.

    Filtering pipeline (in order):
      1. Drop NaN / Inf — these are no-return rays (open sky)
      2. Drop Z < GROUND_Z_MIN — these points are below the drone (ground)
      3. Drop dist < MIN_RANGE — sensor noise and self-reflections
      4. Drop dist > MAX_RANGE — depth cameras lose accuracy at long range
      5. Drop points outside the forward FOV cone — side and rear returns are
         unreliable and cause false obstacle detections

    Coordinate transform:
      Camera frame: X=forward, Y=left, Z=up
      World frame:  north=forward-component, east=right-component

      We rotate the (X, Y) camera vector by the drone's current yaw to get the
      world-frame displacement, then add the drone's world position.
    """
    try:
        msg = pc_msgs.PointCloudPacked()
        msg.ParseFromString(msg_data)

        offsets = {f.name: f.offset for f in msg.field}
        if "x" not in offsets:
            return

        step  = msg.point_step
        data  = msg.data
        x_off = offsets.get("x", 0)
        y_off = offsets.get("y", 4)
        z_off = offsets.get("z", 8)

        fov_tan = math.tan(math.radians(FOV_HALF_DEG))

        with _state.lock:
            drone_n = _state.north
            drone_e = _state.east
            yaw     = _state.yaw

        cos_y = math.cos(yaw)
        sin_y = math.sin(yaw)

        raw: List[Tuple[float, float]] = []

        for i in range(0, len(data) - step + 1, step):
            x = struct.unpack_from("<f", data, i + x_off)[0]
            y = struct.unpack_from("<f", data, i + y_off)[0]
            z = struct.unpack_from("<f", data, i + z_off)[0]

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            if z < GROUND_Z_MIN:
                continue

            dist = math.sqrt(x * x + y * y + z * z)
            if dist < MIN_RANGE or dist > MAX_RANGE:
                continue

            # Must be in the forward half-space and within the horizontal FOV
            if x <= 0 or abs(y) > x * fov_tan:
                continue

            # Rotate camera-frame (x=forward, y=left) into world (north, east)
            obs_n = drone_n + cos_y * x - sin_y * y
            obs_e = drone_e + sin_y * x + cos_y * y
            raw.append((obs_n, obs_e))

        # Subsample so we don't flood the map with redundant points each frame
        if len(raw) > MAX_POINTS_PER_FRAME:
            stride = len(raw) // MAX_POINTS_PER_FRAME
            raw = raw[::stride]

        with _state.lock:
            _state.obstacles  = raw
            _state.cloud_time = time.time()

    except Exception as exc:
        print(f"[explorer] point cloud error: {exc}")


def _on_pose(msg_data: bytes):
    """
    Update drone position and heading from Gazebo world pose.

    Gazebo uses ENU (East-North-Up): pose.position.x = east, pose.position.y = north.
    We convert to the (north, east) convention used everywhere else in VOSA.

    Yaw is extracted from the orientation quaternion using the standard formula.
    Yaw=0 means the drone faces north (positive X in camera frame = north in world frame).
    """
    try:
        msg = pose_msgs.Pose_V()
        msg.ParseFromString(msg_data)

        for pose in msg.pose:
            if pose.name != DRONE_MODEL:
                continue

            # Gazebo ENU → VOSA (north, east)
            east  = pose.position.x
            north = pose.position.y

            qx = pose.orientation.x
            qy = pose.orientation.y
            qz = pose.orientation.z
            qw = pose.orientation.w
            yaw = math.atan2(2 * (qw * qz + qx * qy),
                             1 - 2 * (qy * qy + qz * qz))

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
            if resp == "ok":
                print(f"[explorer] ✓  N={north:+6.1f}m  E={east:+6.1f}m  alt={alt:.1f}m")
                return True
            print(f"[explorer] ✗  VOSA rejected: {resp}")
            return False
        except Exception as exc:
            print(f"[explorer] VOSA error: {exc}")
            return False


# ══════════════════════════════════════════════════════════════════════════════
# Main exploration loop
# ══════════════════════════════════════════════════════════════════════════════

def _nearest_frontier(frontiers, north, east):
    return min(frontiers, key=lambda f: (f[0] - north) ** 2 + (f[1] - east) ** 2)


def _spaced_waypoints(path: List[Tuple[float, float]],
                      start_n: float, start_e: float,
                      step: float) -> List[Tuple[float, float]]:
    """
    Subsample A* path to waypoints spaced at least `step` metres apart.
    Sending every grid cell would spam PX4 with micro-waypoints; this keeps
    the waypoint stream at a rate the autopilot can execute smoothly.
    """
    result = []
    prev_n, prev_e = start_n, start_e
    for n, e in path:
        if math.sqrt((n - prev_n) ** 2 + (e - prev_e) ** 2) >= step:
            result.append((n, e))
            prev_n, prev_e = n, e
    if not result and path:
        result = [path[-1]]
    return result


def main():
    parser = argparse.ArgumentParser(description="VOSA autonomous explorer v2")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--alt",  type=float, default=30.0,
                        help="cruise altitude in metres")
    parser.add_argument("--step", type=float, default=DEFAULT_STEP,
                        help="metres between waypoints")
    args = parser.parse_args()

    gateway = VosaGateway(args.host, args.port)
    gateway.connect()

    node = gz.transport13.Node()
    node.subscribe(DEPTH_TOPIC, _on_point_cloud)
    node.subscribe(POSE_TOPIC,  _on_pose)
    print(f"[explorer] Subscribed to depth camera + pose topics")
    print(f"[explorer] Cruising at {args.alt}m — building occupancy map")

    time.sleep(3.0)  # let sensors warm up

    grid    = OccupancyGrid()
    path:   List[Tuple[float, float]] = []
    target: Optional[Tuple[float, float]] = None

    while True:
        time.sleep(1.5)

        # ── Snapshot shared state ──────────────────────────────────────────
        with _state.lock:
            north      = _state.north
            east       = _state.east
            obstacles  = list(_state.obstacles)
            cloud_age  = time.time() - _state.cloud_time
            pose_age   = time.time() - _state.pose_time

        if pose_age > 5.0:
            print("[explorer] WARNING: no pose data — is Gazebo running?")
        if cloud_age > 5.0:
            print("[explorer] WARNING: no depth camera data for 5 s")

        # ── 1. Update occupancy map ────────────────────────────────────────
        grid.update(north, east, obstacles)

        # ── 2. Pop waypoints we have already reached ───────────────────────
        while path:
            wp_n, wp_e = path[0]
            dist = math.sqrt((north - wp_n) ** 2 + (east - wp_e) ** 2)
            if dist < ARRIVAL_TOL:
                path.pop(0)
                print(f"[explorer] Waypoint reached — {len(path)} remaining on path")
            else:
                break

        # ── 3. Replan when path is exhausted ──────────────────────────────
        if not path:
            all_frontiers = grid.frontiers()

            if not all_frontiers:
                print("[explorer] No frontiers remaining — area fully explored!")
                break

            # Sort frontiers by distance so we try nearest first
            all_frontiers.sort(
                key=lambda f: (f[0] - north) ** 2 + (f[1] - east) ** 2
            )

            planned = False
            for candidate in all_frontiers[:10]:
                route = grid.astar(north, east, candidate[0], candidate[1])
                if route:
                    path = _spaced_waypoints(route, north, east, args.step)
                    target = candidate
                    print(
                        f"[explorer] → Frontier N={candidate[0]:+.1f}m "
                        f"E={candidate[1]:+.1f}m  "
                        f"path={len(path)} waypoints  "
                        f"frontiers={len(all_frontiers)}"
                    )
                    planned = True
                    break

            if not planned:
                # Every reachable frontier is surrounded by occupied cells.
                # Probe outward in 8 directions to escape.
                print("[explorer] No A* path found — probing outward")
                for angle_deg in range(0, 360, 45):
                    rad = math.radians(angle_deg)
                    pn = north + args.step * math.cos(rad)
                    pe = east  + args.step * math.sin(rad)
                    pgx, pgy = grid.world_to_grid(pn, pe)
                    if grid.in_bounds(pgx, pgy) and grid.grid[pgx, pgy] != OCCUPIED:
                        path = [(pn, pe)]
                        break

        # ── 4. Send next waypoint to VOSA ──────────────────────────────────
        if path:
            wp_n, wp_e = path[0]
            if not gateway.send(wp_n, wp_e, args.alt):
                # VOSA rejected — treat destination as blocked and replan
                ogx, ogy = grid.world_to_grid(wp_n, wp_e)
                grid._occupy(ogx, ogy)
                path.clear()
                print("[explorer] Waypoint rejected — cell blocked, replanning")


if __name__ == "__main__":
    main()
