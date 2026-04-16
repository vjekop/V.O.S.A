# Changelog

All significant changes to V.O.S.A. are documented here.  
Format follows [Keep a Changelog](https://keepachangelog.com/en/1.0.0/).  
Versions follow [Semantic Versioning](https://semver.org/).

---

## [Unreleased]

### Added
- **Speed Enforcement**: The `SafetySandbox` now statically verifies that `flight.cruise_speed` does not exceed `safety.max_speed`.
- **Geofence Enforcement**: The `Runtime` loop now validates every waypoint dynamically. If a waypoint exceeds the geofence radius from the center, the mission aborts via a `SafetyViolation`.
- **Coordinate-based Geofence**: The parser now supports `circle(lat: X, lon: Y, radius: Z)` in addition to `center: home`. This is required so the simulator can run geofences without "home" being dynamically locked in by hardware.
- **Unit & Integration Tests**: Added test suites for the `SafetySandbox`, `haversine` distance math, and `Runtime` geofence module.

### Fixed
- Fixed an unmatched `expect_quantity` warning in the parser component.

---

## [0.1.0] — 2026-04-15

### Commit: `7910b09` — `feat: initial V.O.S.A. DSL implementation`

This is the **first working version** of the V.O.S.A. language.  
The full pipeline — source text → tokens → AST → safety check → simulation — is functional end-to-end.

---

#### What was built

**`src/error.rs` — Unified error type**

Defines `VosaError`, the single error enum used across every compiler phase:
- `LexError { col, msg }` — bad character or unterminated string
- `ParseError { line, msg }` — unexpected token or missing required field
- `SafetyViolation(msg)` — mission violates a declared safety constraint
- `RuntimeError(msg)` — execution failure
- `IoError` — file read failure (wraps `std::io::Error`)

Uses [`thiserror`](https://docs.rs/thiserror) for clean `Display` formatting.

---

**`src/lexer/token.rs` — Token definitions**

Defines the full token vocabulary for the VOSA language:
- **`Unit`** enum: `Meters`, `MetersPerSecond`, `Seconds`, `Percent`, `Degrees`
- **`TokenKind`** enum: every keyword, literal, command, parameter key, and punctuation symbol
- **`Token`** struct: wraps a `TokenKind` with its source `line` and `col` for error reporting

Key design decision: physical units are baked into the token type as `Quantity(f64, Unit)` rather than being post-processed strings. This means unit mismatches are caught at lex time.

---

**`src/lexer/mod.rs` — The Lexer**

A hand-written, character-by-character lexer (`struct Lexer`) that converts raw source text into a `Vec<Token>`.

How it works:
1. `.tokenize()` is called on the `Lexer` — it loops calling `.next_token()` until `Eof`
2. Whitespace and `//` line comments are skipped before every token
3. Numbers are parsed greedily, then a unit suffix (`m`, `m/s`, `s`, `%`, `deg`) is consumed if present — producing `Quantity` or bare `Number`
4. Identifiers are read and looked up in a keyword table — producing the correct `TokenKind` for each VOSA keyword, or `Ident(String)` as a fallback
5. Strings are delimited by `"..."` and reject unterminated literals
6. Single-character punctuation (`{}(),:`) is matched directly

---

**`src/parser/ast.rs` — Abstract Syntax Tree**

Defines all typed AST nodes that a parsed `.vosa` file produces:

| Type | Purpose |
|---|---|
| `Mission` | Root node. Contains name, optional vehicle/safety/flight, and a required sequence |
| `VehicleKind` | `Quadcopter`, `FixedWing`, `Hexacopter`, `Custom(String)` |
| `SafetyBlock` | All safety constraints: altitudes, speed, geofence, battery reserve, failsafe |
| `Geofence` | Currently: `Circle { center: GeoCenter, radius: f64 }` |
| `GeoCenter` | `Home` (launch point) or `Coord { lat, lon }` |
| `FailsafeAction` | `ReturnHome`, `Land`, `Hover` |
| `FlightConfig` | Default `cruise_altitude` and `cruise_speed` |
| `Sequence` | Ordered `Vec<Command>` |
| `Command` | `Takeoff`, `Land`, `Hover`, `Waypoint`, `ReturnHome`, `Camera` |
| `CameraAction` | `Record`, `Photo`, `Stop` |

All fields that are optional in the syntax are `Option<T>` in the AST.

---

**`src/parser/mod.rs` — The Parser**

A recursive-descent parser (`struct Parser`) that consumes a `Vec<Token>` and produces a `Mission` AST node.

Structure mirrors the grammar:
- `parse()` → expects `mission "name" { ... }`
- `parse_vehicle()` → expects `: <VehicleKind>`
- `parse_safety()` → parses key-value pairs inside `safety { }`
- `parse_geofence()` → parses `circle(center: home, radius: 500m)`
- `parse_flight()` → parses `flight { cruise_altitude, cruise_speed }`
- `parse_sequence()` → loops parsing `Command`s until `}`
- `parse_command()` → dispatches on the command keyword token
- Individual command parsers handle their parameter lists with named or positional args

Commas in parameter lists are optional — the parser skips them if present. This makes the language more forgiving to write.

---

**`src/safety/mod.rs` — The Safety Sandbox**

`struct SafetySandbox` runs a validation pass over the parsed `Mission` **before any execution**.

Current checks:
- `Takeoff { altitude }` — checked against `max_altitude` and `min_altitude`
- `Waypoint { alt }` — checked against `max_altitude` and `min_altitude`
- `Hover { duration }` — must be > 0 seconds

If any check fails, a `VosaError::SafetyViolation` is returned with a precise message naming the command and the violated limit. Execution never begins.

---

**`src/runtime/mod.rs` — The Runtime Simulator**

`struct Runtime` simulates mission execution and produces an `ExecutionReport`.

For each command:
- `Takeoff` — updates `current_alt`, logs ascent
- `Land` — resets altitude to 0, logs descent
- `Hover` — logs hold duration
- `Waypoint` — computes distance from current position using the **haversine formula**, accumulates `total_distance`, updates position and altitude
- `ReturnHome` — computes distance back to origin (0,0), accumulates it
- `Camera` — logs action and resolution

`ExecutionReport` contains the full step log, total distance flown (metres), and max altitude reached (metres).

The haversine function uses Earth radius R = 6,371,000 m for accurate great-circle distance.

In a future hardware bridge, this layer would emit MAVLink packets or ROS 2 messages instead of log strings.

---

**`src/lib.rs` — Public API**

Exposes two top-level functions:
- `vosa::parse(source: &str) → Result<Mission>` — lex + parse only
- `vosa::run(source: &str) → Result<ExecutionReport>` — full pipeline including safety check

---

**`src/main.rs` — CLI**

Built with [`clap`](https://docs.rs/clap) derive macros. Three subcommands:

| Subcommand | What it does |
|---|---|
| `vosa run <file>` | Runs the full pipeline and prints the execution log |
| `vosa run <file> --ast` | Same, but also prints the parsed AST |
| `vosa check <file>` | Parse + safety validation only — no execution |
| `vosa docs` | Prints the inline language reference |

Uses [`colored`](https://docs.rs/colored) for terminal output: green for success, red for errors, yellow for safety violations.

---

**`examples/`**

| File | What it demonstrates |
|---|---|
| `hello_world.vosa` | Minimal valid mission: takeoff → hover → land |
| `perimeter_scan.vosa` | Full safety block, 4-corner waypoint path, camera control |
| `photo_survey.vosa` | Hexacopter, photo-at-waypoint pattern, multi-transect |

---

**`Cargo.toml`**

| Dependency | Version | Why |
|---|---|---|
| `thiserror` | 1.0 | Clean error type derivation |
| `clap` | 4.5 (derive) | CLI argument parsing |
| `colored` | 2.1 | Terminal color output |

---

## Commit: `docs: add README, CHANGELOG, CONTRIBUTING`

### Added

- `README.md` — Project overview, quick start, full syntax reference, architecture diagram, CLI reference, example table, and roadmap
- `CHANGELOG.md` — This file. Every commit will be documented here with what changed and why
- `CONTRIBUTING.md` — How to contribute, commit conventions, and branch strategy

---

[Unreleased]: https://github.com/vjekop/V.O.S.A/compare/v0.1.0...HEAD
[0.1.0]: https://github.com/vjekop/V.O.S.A/releases/tag/v0.1.0
