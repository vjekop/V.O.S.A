# VOSA — Drone Mission Safety, Before You Leave the Ground

> **Write a mission. Catch every unsafe flight before motors spin.**  
> VOSA is an open-source mission language for autonomous drones — with reactive safety built in, not bolted on.

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Built with Rust](https://img.shields.io/badge/Built%20with-Rust-orange.svg)](https://www.rust-lang.org)

---

## Demo

VOSA connecting to PX4 SITL, arming, taking off, and running a reactive mission in Gazebo:


https://github.com/user-attachments/assets/4e91609f-ae2c-45b1-94f3-7d91aeb2c8e7



---

## The Problem

Drone crashes are expensive. Fly-aways are embarrassing. Manually aborted flights cost time and money.

Most drone missions are rigid scripts — they execute line by line and have no awareness of what's happening around them. Battery drops? Wind picks up? Obstacle appears? The vehicle has no language-level response. You find out when something goes wrong.

**VOSA is built around a different idea: missions should react, not just execute.**

---

## What VOSA Does

VOSA is a typed mission language that compiles directly to real drone hardware (PX4, ArduPilot via MAVLink). You describe your mission — including what should happen when conditions change mid-flight — and VOSA enforces your safety rules at compile time, before anything moves.

- **Pre-flight safety validation** — altitude, speed, geofence, battery reserves checked before arming
- **Reactive triggers** — missions respond autonomously to live telemetry: battery, wind, obstacles
- **Real hardware output** — compiles to MAVLink packets, no middleware, no abstraction tax
- **Simulator-ready** — connects natively to PX4 SITL, Gazebo, and ROS 2 / MAVROS

---

## What a Mission Looks Like

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

    // If battery drops below 20%, abort and come home — automatically
    on battery < 20% {
      return_home()
      land()
    }

    // Wait out high wind, then continue
    on wind > 12m/s {
      hover(30s)
    }

    // Obstacle detected: pause, then return safely
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

Triggers fire once when a condition becomes true, reset when it clears, and re-fire if conditions recur. On real hardware they are driven by live MAVLink telemetry (`SYS_STATUS`, `WIND_COV`) — not simulation.

---

## Quick Start

**Prerequisites:** [Rust 1.75+](https://rustup.rs/)

```bash
git clone https://github.com/vjekop/V.O.S.A.git
cd V.O.S.A
cargo build --release
```

**Validate a mission (no hardware needed):**
```bash
./target/release/vosa check examples/reactive_mission.vosa
```

**Run in simulation:**
```bash
./target/release/vosa run examples/reactive_mission.vosa
./target/release/vosa run examples/perimeter_scan.vosa
```

**Browse the language reference:**
```bash
./target/release/vosa docs
```

---

## Connecting to Real Hardware

### PX4 SITL + Gazebo

```bash
# 1. Start PX4 SITL (Ubuntu / WSL2)
git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd PX4-Autopilot && bash ./Tools/setup/ubuntu.sh
make px4_sitl gazebo

# 2. Run your VOSA mission against it
./target/release/vosa run examples/reactive_mission.vosa --mavlink udpin:0.0.0.0:14550
```

VOSA handles the full startup sequence: GCS heartbeat → GPS lock → home position → `AUTO.MISSION` mode → mission upload → arm → live telemetry monitoring.

### ROS 2 / MAVROS

```bash
ros2 run mavros mavros_node --ros-args -p fcu_url:=udp://:14540@127.0.0.1:14557
ros2 service call /mavros/cmd/arming mavros_msgs/srv/CommandBool "{value: true}"
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'OFFBOARD'}"
./target/release/vosa run examples/reactive_mission.vosa --ros2 0
```

---

## Language Reference

```vosa
mission "name" {
    vehicle: Quadcopter | FixedWing | Hexacopter

    // Declare named sensors from any supported MAVLink field
    sensor <name> from <MESSAGE>.<field>

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
        takeoff(<n>m)
        waypoint(lat: <f64>, lon: <f64>, alt: <n>m)
        hover(<n>s)
        camera(action: record | photo | stop, resolution: "4K")
        return_home()
        land()

        repeat <n> { ... }
        if battery < <n>% { ... }
        parallel { ... }

        on battery < <n>% { ... }
        on battery > <n>% { ... }
        on wind    > <n>m/s { ... }
        on wind    < <n>m/s { ... }
        on obstacle_detected { ... }

        // Custom sensor triggers (requires sensor declaration above)
        on <sensor_name> < <value> { ... }
        on <sensor_name> > <value> { ... }
    }
}
```

| Suffix | Meaning |
|--------|---------|
| `m` | metres |
| `m/s` | metres per second |
| `s` | seconds |
| `%` | percent |
| `deg` | degrees |

### Sensor Bindings

Bind any supported MAVLink telemetry field to a named sensor, then use it in reactive triggers:

```vosa
sensor roll_angle   from ATTITUDE.roll
sensor gps_hdop     from GPS_RAW_INT.eph
sensor climb_rate   from VFR_HUD.climb
sensor ground_speed from VFR_HUD.groundspeed

on roll_angle > 0.45 { hover(10s)       }
on gps_hdop   > 500  { hover(20s)       }
on climb_rate < -3.0 { return_home()    }
```

Sensor names are validated at compile time (`vosa check`). On real hardware, values come from live MAVLink telemetry. In simulation, custom sensors read as `0.0`.

**Supported sources:**

| Message | Fields |
|---------|--------|
| `ATTITUDE` | `roll`, `pitch`, `yaw`, `rollspeed`, `pitchspeed`, `yawspeed` |
| `VFR_HUD` | `airspeed`, `groundspeed`, `alt`, `climb` |
| `WIND_COV` | `wind_x`, `wind_y` |
| `GPS_RAW_INT` | `eph`, `epv`, `satellites_visible` |
| `SYS_STATUS` | `battery_remaining` |
| `DISTANCE_SENSOR` | `current_distance` |

---

## Example Missions

| File | What it demonstrates |
|------|-------------|
| `examples/hello_world.vosa` | Minimal takeoff and land |
| `examples/perimeter_scan.vosa` | Geofenced perimeter survey |
| `examples/photo_survey.vosa` | Multi-waypoint photographic transect |
| `examples/parallel_survey.vosa` | Parallel multi-drone blocks |
| `examples/reactive_mission.vosa` | Full reactive mission — battery, wind, obstacle triggers |
| `examples/sensor_mission.vosa` | Generic sensor bindings from MAVLink telemetry |

---

## How It Works

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
Safety Sandbox  →  pre-flight validation (altitude, speed, geofence, battery)
    │
    ▼
Runtime (sim) or Hardware Bridge
    ├── MAVLink  →  PX4 / ArduPilot
    └── ROS 2    →  MAVROS
```

---

## Roadmap

- [x] Typed DSL with safety sandbox
- [x] Geofence boundary checking
- [x] Reactive triggers (`on battery`, `on wind`, `on obstacle_detected`)
- [x] Real MAVLink bridge — `MISSION_ITEM_INT` protocol, GPS lock, mode switching
- [x] Live telemetry loop feeding reactive triggers
- [x] ROS 2 / MAVROS topic bridge
- [x] Gazebo + PX4 SITL integration
- [x] Parallel execution (`parallel { ... }` blocks)
- [x] Generic sensor bindings (`sensor X from MESSAGE.field`)
- [x] Temporal trigger conditions (`on X for <duration>`)
- [x] Sensor injection for simulation testing (`--inject battery=18,wind=15`)
- [ ] LSP / editor support (syntax highlighting, inline validation)

---

## License

Apache 2.0 — see [LICENSE](LICENSE).
