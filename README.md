# V.O.S.A. — Vectorized Operational Safety Autonomy

> **The open-source DSL for autonomous drone missions.**  
> Write a mission. V.O.S.A. validates it, reacts to conditions mid-flight, and drives real hardware — PX4, MAVLink, ROS 2, Gazebo.

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Built with Rust](https://img.shields.io/badge/Built%20with-Rust-orange.svg)](https://www.rust-lang.org)

---

## What Is V.O.S.A.?

Drone autonomy today is fragile. Missions are rigid scripts — they execute line by line and have no idea what's happening around them. If battery drops, wind picks up, or an obstacle appears, there's no language-level response. You get a crash, a fly-away, or a manually aborted flight.

V.O.S.A. is built around a different idea: **missions should react, not just execute.**

It is a typed, safety-first DSL where:
- Safety constraints are **enforced at compile time** — before the motors spin
- Missions respond **autonomously to live conditions** via reactive triggers
- The language compiles directly to **real MAVLink packets** — no middleware, no abstraction tax
- It connects natively to **PX4 SITL, Gazebo, and ROS 2 / MAVROS**

---

## Reactive Triggers

The core feature. Drones respond to conditions mid-flight without human input:

```vosa
mission "autonomous_survey" {
  vehicle: Quadcopter

  safety {
    max_altitude:    120m
    battery_reserve: 10%
    geofence:        circle(center: home, radius: 2000m)
    failsafe:        return_home
  }

  sequence {

    // Register reactive triggers — active for the entire mission
    on battery < 20% {
      return_home()
      land()
    }

    on wind > 12m/s {
      hover(30s)
    }

    on obstacle_detected {
      hover(5s)
      return_home()
      land()
    }

    // Flight plan
    takeoff(60m)
    waypoint(lat: 37.7750, lon: -122.4195, alt: 60m)
    waypoint(lat: 37.7780, lon: -122.4195, alt: 60m)
    camera(action: photo, resolution: "4K")
    return_home()
    land()
  }
}
```

Triggers use **rising-edge semantics** — they fire once when a condition becomes true, reset when it clears, and can re-fire if the situation recurs. On real hardware they are driven by live MAVLink telemetry (`SYS_STATUS`, `WIND_COV`), not simulation.

---

## Quick Start

### Prerequisites
- [Rust toolchain](https://rustup.rs/) (1.75+)

### Install
```bash
git clone https://github.com/vjekop/V.O.S.A.git
cd V.O.S.A
cargo build --release
```

### Simulate a mission
```bash
./target/release/vosa run examples/reactive_mission.vosa
./target/release/vosa run examples/perimeter_scan.vosa
```

### Validate without executing
```bash
./target/release/vosa check examples/reactive_mission.vosa
```

### Language reference
```bash
./target/release/vosa docs
```

---

## Gazebo + PX4 SITL

V.O.S.A. connects directly to PX4 SITL running inside Gazebo. No middleware required.

### Setup (Ubuntu / WSL2)

```bash
# 1. Clone PX4 and install dependencies
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot
bash ./Tools/setup/ubuntu.sh

# 2. Start PX4 SITL with Gazebo
make px4_sitl gazebo
```

### Run a VOSA mission against it

```bash
# In a second terminal
./target/release/vosa run examples/reactive_mission.vosa --mavlink udpin:0.0.0.0:14550
```

### What happens

V.O.S.A. handles the full startup sequence automatically:

1. GCS heartbeat handshake
2. Waits for 3D GPS lock
3. Waits for home position confirmation
4. Switches PX4 to `AUTO.MISSION` mode
5. Uploads mission via `MISSION_ITEM_INT` protocol
6. Arms vehicle, starts mission
7. Monitors live telemetry — fires reactive triggers in real time

---

## ROS 2 / MAVROS

```bash
# Start MAVROS
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557

# Arm and set OFFBOARD mode first (service calls)
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"

# Run mission
./target/release/vosa run examples/reactive_mission.vosa --ros2 0
```

Waypoints are published to `/mavros/setpoint_raw/global` as `mavros_msgs/GlobalPositionTarget`.

---

## Language Reference

### Full structure

```vosa
mission "name" {
    vehicle: Quadcopter | FixedWing | Hexacopter

    safety {
        max_altitude:    <n>m
        min_altitude:    <n>m
        max_speed:       <n>m/s
        battery_reserve: <n>%
        geofence:        circle(center: home, radius: <n>m)
        failsafe:        return_home | land | hover
    }

    flight {
        cruise_altitude: <n>m
        cruise_speed:    <n>m/s
    }

    sequence {
        // Commands
        takeoff(<n>m)
        waypoint(lat: <f64>, lon: <f64>, alt: <n>m)
        hover(<n>s)
        camera(action: record | photo | stop, resolution: "4K")
        return_home()
        land()

        // Control flow
        repeat <n> { ... }
        if battery < <n>% { ... }
        parallel { ... }

        // Reactive triggers
        on battery < <n>% { ... }
        on battery > <n>% { ... }
        on wind    > <n>m/s { ... }
        on wind    < <n>m/s { ... }
        on obstacle_detected { ... }
    }
}
```

### Units

| Suffix | Meaning |
|--------|---------|
| `m` | metres |
| `m/s` | metres per second |
| `s` | seconds |
| `%` | percent |
| `deg` | degrees |

---

## Architecture

```
Source (.vosa)
    │
    ▼
Lexer  →  Vec<Token>
    │
    ▼
Parser →  Mission AST
    │
    ▼
SafetySandbox  →  pre-flight validation (altitude, speed, geofence, hover duration)
    │
    ▼
Runtime (sim) or Hardware Bridge
    ├── MAVLink  →  PX4 / ArduPilot (MISSION_ITEM_INT protocol, live telemetry)
    └── ROS 2    →  MAVROS (/mavros/setpoint_raw/global)
```

```
src/
├── lexer/         — tokeniser
├── parser/        — recursive-descent parser + AST
├── safety/        — pre-flight safety sandbox
├── runtime/       — simulator + reactive trigger engine
└── hw_bridge/     — MAVLink and ROS 2 bridges
```

---

## Examples

| File | Description |
|------|-------------|
| `examples/hello_world.vosa` | Minimal takeoff and land |
| `examples/perimeter_scan.vosa` | Geofenced perimeter survey |
| `examples/photo_survey.vosa` | Multi-waypoint photographic transect |
| `examples/parallel_survey.vosa` | Parallel multi-drone blocks |
| `examples/reactive_mission.vosa` | Full reactive mission — battery, wind, obstacle triggers |

---

## Roadmap

- [x] Typed DSL with safety sandbox
- [x] Geofence boundary checking
- [x] Reactive triggers (`on battery`, `on wind`, `on obstacle_detected`)
- [x] Real MAVLink bridge — `MISSION_ITEM_INT` protocol, GPS lock, mode switching
- [x] Live telemetry loop feeding reactive triggers
- [x] ROS 2 / MAVROS topic bridge
- [x] Gazebo + PX4 SITL integration
- [ ] Parallel execution (concurrent multi-drone scheduling)
- [ ] LSP / editor support

---

## License

Apache 2.0 — see [LICENSE](LICENSE).
