# V.O.S.A. — Vectorized Operational Safety Autonomy

> **The open-source Domain-Specific Language for autonomous drones.**  
> V.O.S.A. acts as the *nervous system* between high-level mission intent and low-level machine execution — with safety enforced at the language level before the motors ever spin.

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache_2.0-blue.svg)](LICENSE)
[![Built with Rust](https://img.shields.io/badge/Built%20with-Rust-orange.svg)](https://www.rust-lang.org)

---

## What Is V.O.S.A.?

Most drone programming forces you to choose between:
- **High-level flight apps** (DJI, Autel) — easy but closed, unsafe, inflexible
- **Low-level SDKs / MAVLink** — powerful but extremely complex and error-prone

V.O.S.A. fills the gap. It is a **typed, safety-first DSL** designed specifically for 3D spatial autonomy. You write a mission in plain, readable V.O.S.A. syntax. The compiler validates every constraint before execution. The runtime (or hardware bridge) carries it out.

```
Write mission → Safety Sandbox validates → Runtime executes
```

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

The binary will be at `target/release/vosa`.

### Run an example mission
```bash
cargo run -- run examples/hello_world.vosa
cargo run -- run examples/perimeter_scan.vosa
cargo run -- run examples/photo_survey.vosa
```

### Validate without executing
```bash
cargo run -- check examples/perimeter_scan.vosa
```

### Print language reference
```bash
cargo run -- docs
```

---

## Language Syntax

A V.O.S.A. file contains exactly one `mission` block.

### Full Structure

```vosa
mission "mission_name" {

    vehicle: Quadcopter          // optional

    safety {                     // optional — validated before any motor runs
        max_altitude:    120m
        min_altitude:    5m
        max_speed:       15m/s
        battery_reserve: 25%
        geofence:        circle(center: home, radius: 500m)
        failsafe:        return_home
    }

    flight {                     // optional defaults
        cruise_altitude: 50m
        cruise_speed:    8m/s
    }

    sequence {                   // required — ordered list of commands
        takeoff(10m)
        waypoint(lat: 38.897, lon: -77.036, alt: 50m)
        hover(5s)
        camera(action: record, resolution: "4K")
        return_home()
        land()
    }
}
```

### Commands

| Command | Parameters | Description |
|---|---|---|
| `takeoff(Xm)` | altitude in metres | Arm and ascend |
| `land()` | — | Descend and disarm |
| `hover(Xs)` | duration in seconds | Hold current position |
| `waypoint(lat, lon, alt)` | GPS coordinate + altitude | Fly to location |
| `return_home()` | — | Return to launch point |
| `camera(action, resolution?)` | `record`, `photo`, or `stop` | Control onboard camera |

### Vehicle Types

`Quadcopter` · `FixedWing` · `Hexacopter`

### Units

| Suffix | Meaning |
|---|---|
| `m` | metres |
| `m/s` | metres per second |
| `s` | seconds |
| `%` | percent |
| `deg` | degrees |

### Comments
```vosa
// This is a line comment
```

---

## Architecture

```
src/
├── lib.rs            — Public API: parse() and run()
├── error.rs          — Unified VosaError type
├── lexer/
│   ├── mod.rs        — Lexer: source text → token stream
│   └── token.rs      — Token and Unit definitions
├── parser/
│   ├── mod.rs        — Recursive-descent parser → AST
│   └── ast.rs        — All AST node types
├── safety/
│   └── mod.rs        — SafetySandbox: pre-flight constraint validation
└── runtime/
    └── mod.rs        — Simulator / execution engine
```

### Data Flow

```
Source (.vosa)
    │
    ▼
Lexer  →  Vec<Token>
    │
    ▼
Parser →  Mission (AST)
    │
    ▼
SafetySandbox  →  validates all constraints (altitude, geofence, etc.)
    │
    ▼
Runtime  →  ExecutionReport (simulation log, distance, max altitude)
```

### Safety Sandbox

Every command in the sequence is checked **before** the first motor fires:
- Takeoff and waypoint altitudes are checked against `max_altitude` / `min_altitude`
- Hover duration must be > 0
- (Planned) Speed validated against `max_speed`
- (Planned) Waypoints checked against geofence boundary

A `SafetyViolation` error aborts execution with a clear message.

---

## CLI Reference

```
vosa run   <file.vosa>          Parse, validate, and simulate a mission
vosa run   <file.vosa> --ast    Also print the full parsed AST
vosa check <file.vosa>          Validate only — no execution
vosa docs                       Print the inline language reference
vosa --help                     Show all commands
vosa --version                  Show version
```

---

## Examples

See the [`examples/`](examples/) directory:

| File | Description |
|---|---|
| `hello_world.vosa` | Simplest valid mission — takeoff, hover, land |
| `perimeter_scan.vosa` | Full 4-corner perimeter survey with camera |
| `photo_survey.vosa` | Multi-waypoint transect with aerial photography |

---

## Roadmap

- [x] Speed validation in safety sandbox
- [x] Geofence boundary checking for each waypoint
- [x] `loop` and `if battery <` control flow
- [ ] MAVLink output bridge (real hardware)
- [ ] ROS 2 node publisher
- [ ] WASM build (browser mission planner)
- [ ] Language Server Protocol (LSP) support

---

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md).

---

## License

Apache 2.0 — see [LICENSE](LICENSE).
