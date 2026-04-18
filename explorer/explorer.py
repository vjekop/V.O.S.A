"""
VOSA Explorer — Autonomous frontier exploration using depth camera.

Reads point cloud data from Gazebo via gz-transport, detects free space
ahead of the drone, and sends safe waypoints to the VOSA serve API for
validation before forwarding to PX4.

Usage:
    python3 explorer.py [--host 127.0.0.1] [--port 7777] [--alt 30.0]
"""

import argparse
import math
import socket
import threading
import time

import gz.transport13
import gz.msgs10.pointcloud_packed_pb2 as pc_msgs
import gz.msgs10.pose_v_pb2 as pose_msgs

# ── Configuration ─────────────────────────────────────────────────────────────

DEPTH_TOPIC = "/depth_camera/points"
POSE_TOPIC  = "/world/default/dynamic_pose/info"
DRONE_MODEL = "x500_depth_0"

# How far ahead to command the next waypoint (metres)
STEP_SIZE = 5.0

# Minimum clear distance ahead before we consider it obstacle-free (metres)
CLEAR_THRESHOLD = 3.0

# ── State ─────────────────────────────────────────────────────────────────────

class ExplorerState:
    def __init__(self):
        self.lock = threading.Lock()
        self.heading_deg = 0.0       # current yaw in degrees
        self.position = (0.0, 0.0)   # (north, east) from home in metres
        self.min_distance_ahead = float("inf")  # closest obstacle ahead (m)
        self.last_point_cloud_time = 0.0

state = ExplorerState()

# ── VOSA gateway ──────────────────────────────────────────────────────────────

class VosaGateway:
    def __init__(self, host: str, port: int):
        self.host = host
        self.port = port
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(5.0)
        print(f"[explorer] Connected to VOSA at {self.host}:{self.port}")

    def send_waypoint(self, north: float, east: float, alt: float) -> bool:
        """Send waypoint to VOSA. Returns True if accepted, False if rejected."""
        cmd = f"waypoint north={north:.2f} east={east:.2f} alt={alt:.2f}\n"
        try:
            self.sock.sendall(cmd.encode())
            response = self.sock.recv(256).decode().strip()
            if response == "ok":
                print(f"[explorer] ✓ VOSA accepted: N+{north:.1f}m E+{east:.1f}m alt {alt:.1f}m")
                return True
            else:
                print(f"[explorer] ✗ VOSA rejected: {response}")
                return False
        except Exception as e:
            print(f"[explorer] VOSA connection error: {e}")
            return False

# ── Depth camera subscriber ───────────────────────────────────────────────────

def on_point_cloud(msg_data: bytes):
    """Parse point cloud and find minimum distance to obstacles directly ahead."""
    try:
        msg = pc_msgs.PointCloudPacked()
        msg.ParseFromString(msg_data)

        # Find the point field offsets (x, y, z are standard PointXYZ fields)
        field_offsets = {}
        for field in msg.field:
            field_offsets[field.name] = field.offset

        if "x" not in field_offsets or "z" not in field_offsets:
            return

        point_step = msg.point_step
        data = msg.data

        min_dist = float("inf")
        import struct

        for i in range(0, len(data) - point_step + 1, point_step):
            x_off = field_offsets.get("x", 0)
            y_off = field_offsets.get("y", 4)
            z_off = field_offsets.get("z", 8)

            x = struct.unpack_from("<f", data, i + x_off)[0]
            y = struct.unpack_from("<f", data, i + y_off)[0]
            z = struct.unpack_from("<f", data, i + z_off)[0]

            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue

            # Only consider points in the forward cone (±30° in y, any z)
            dist = math.sqrt(x * x + y * y + z * z)
            if dist < 0.1:
                continue
            if abs(y) < x * math.tan(math.radians(30)) and x > 0:
                min_dist = min(min_dist, dist)

        with state.lock:
            state.min_distance_ahead = min_dist
            state.last_point_cloud_time = time.time()

    except Exception as e:
        print(f"[explorer] point cloud parse error: {e}")

# ── Frontier exploration algorithm ────────────────────────────────────────────

def frontier_step(current_north: float, current_east: float, heading_deg: float, step: float):
    """Calculate next waypoint in the current heading direction."""
    heading_rad = math.radians(heading_deg)
    next_north = current_north + step * math.cos(heading_rad)
    next_east  = current_east  + step * math.sin(heading_rad)
    return next_north, next_east

def choose_heading(current_heading: float, obstacle_ahead: bool) -> float:
    """Simple obstacle avoidance: turn right 45° if blocked, else continue."""
    if obstacle_ahead:
        new_heading = (current_heading + 45.0) % 360.0
        print(f"[explorer] Obstacle detected — turning to {new_heading:.0f}°")
        return new_heading
    return current_heading

# ── Main loop ─────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(description="VOSA autonomous explorer")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=7777)
    parser.add_argument("--alt",  type=float, default=30.0, help="cruise altitude in metres")
    parser.add_argument("--step", type=float, default=STEP_SIZE, help="waypoint step size in metres")
    args = parser.parse_args()

    # Connect to VOSA
    gateway = VosaGateway(args.host, args.port)
    gateway.connect()

    # Subscribe to depth camera
    node = gz.transport13.Node()
    node.subscribe(DEPTH_TOPIC, on_point_cloud)
    print(f"[explorer] Subscribed to {DEPTH_TOPIC}")
    print(f"[explorer] Starting frontier exploration at {args.alt}m altitude")

    # Give sensors a moment to start publishing
    time.sleep(2.0)

    north, east = 0.0, 0.0
    heading = 0.0  # start heading north

    while True:
        time.sleep(1.5)  # one waypoint per 1.5 seconds — conservative

        with state.lock:
            min_dist = state.min_distance_ahead
            last_time = state.last_point_cloud_time

        # Check if depth camera is publishing
        if time.time() - last_time > 5.0:
            print("[explorer] Warning: no depth camera data for 5s — continuing blind")

        obstacle_ahead = min_dist < CLEAR_THRESHOLD
        heading = choose_heading(heading, obstacle_ahead)

        next_north, next_east = frontier_step(north, east, heading, args.step)

        if gateway.send_waypoint(next_north, next_east, args.alt):
            north, east = next_north, next_east
        else:
            # VOSA rejected — try a different direction
            heading = (heading + 90.0) % 360.0
            print(f"[explorer] Waypoint rejected — rotating to {heading:.0f}°")

if __name__ == "__main__":
    main()
