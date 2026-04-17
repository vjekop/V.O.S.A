# Contributing to V.O.S.A.

VOSA is a safety-first DSL for writing autonomous drone missions. Instead of
wiring MAVLink packets by hand, you write a `.vosa` file and let the compiler
validate it before anything flies. This document gets you from zero to running
your first mission in under two minutes.

---

## Quickstart (no drone required)

```bash
git clone https://github.com/vjekop/V.O.S.A.git
cd V.O.S.A
cargo run -- run examples/hello_world.vosa
```

That's it. The simulator runs the mission, logs every step, and prints a
summary. No hardware, no PX4, no setup.

To validate without running:

```bash
cargo run -- check examples/reactive_mission.vosa
```

To see the parsed AST:

```bash
cargo run -- run examples/hello_world.vosa --ast
```

---

## Testing trigger conditions with `--inject`

The simulator's sensors are static by default (battery drains normally, wind
escalates with waypoints, obstacle is always false). Use `--inject` to freeze
any sensor at a fixed value so you can verify your `on` trigger fires correctly:

```bash
# Test that your battery < 20% trigger fires
cargo run -- run examples/reactive_mission.vosa --inject battery=18

# Test wind trigger
cargo run -- run examples/reactive_mission.vosa --inject wind=15

# Force obstacle detection
cargo run -- run examples/reactive_mission.vosa --inject obstacle=1

# Multiple overrides
cargo run -- run examples/reactive_mission.vosa --inject battery=18,wind=15

# Custom sensor declared in the mission file
cargo run -- run examples/reactive_mission.vosa --inject roll_angle=0.5
```

Injected values stay constant — no drain, no escalation — for the entire run.

---

## Run the test suite

```bash
cargo test
```

All tests should pass. If any fail on your machine, open an issue with your
OS and Rust version (`rustc --version`).

---

## Good first issues

These are scoped, self-contained tasks that don't require deep knowledge of the
codebase. Each one has a clear acceptance criterion.

| Area | Task | Where to look |
|------|------|---------------|
| Examples | Write a delivery-drone mission with geofence + battery trigger | `examples/` |
| Examples | Write a search-and-rescue pattern (grid + obstacle reaction) | `examples/` |
| Parser | Add `speed(<n>m/s)` command to set cruise speed mid-mission | `src/parser/`, `src/runtime/` |
| Docs | Add estimated flight-time to the mission summary output | `src/runtime/mod.rs` |
| CLI | Add `--verbose` flag that prints sensor state after every command | `src/main.rs`, `src/runtime/` |
| Safety | Warn (not error) when no failsafe is declared | `src/safety/mod.rs` |
| Tests | Add a test for `--inject battery=5` holding battery constant | `src/runtime/mod.rs` tests |

To claim one: open an issue saying "I'll take [task name]" so two people don't
work on the same thing.

---

## Project layout

```
src/
  lexer/       Tokenizer — text → Vec<Token>
  parser/      Parser + AST — tokens → Mission struct
  safety/      Static validator — checks AST before any execution
  runtime/     Simulator — executes the AST, logs every step
  hw_bridge/   MAVLink + ROS 2 hardware bridges
  error.rs     Unified VosaError enum
  lib.rs       Public API surface
  main.rs      CLI entry point (clap)
examples/      Sample .vosa missions
```

The pipeline is strictly linear: **lex → parse → safety check → execute**.
Each stage only sees the output of the previous one. If you're adding a new
language feature, you'll touch all four stages in order.

---

## Making a change

```bash
# 1. Fork and clone
git clone https://github.com/YOUR_FORK/V.O.S.A.git
cd V.O.S.A

# 2. Create a branch
git checkout -b feat/my-feature

# 3. Make changes, then format and lint
cargo fmt
cargo clippy

# 4. Run tests
cargo test

# 5. Open a PR against main
```

PRs without passing tests won't be merged. CI runs automatically on every PR.

---

## Commit format

```
<type>(<scope>): <short description>
```

| Type | Use for |
|------|---------|
| `feat` | New language feature or CLI flag |
| `fix` | Bug fix |
| `docs` | Documentation, comments, examples |
| `test` | Adding or fixing tests |
| `refactor` | Restructuring with no behavior change |
| `chore` | Deps, CI, build system |

**Scope** = the module you touched: `lexer`, `parser`, `safety`, `runtime`,
`cli`, `examples`.

Examples:

```
feat(parser): add speed() command
fix(safety): correctly reject waypoints below min_altitude
docs(examples): add delivery drone example
test(runtime): verify --inject holds battery constant
```

---

## Changelog rule

Every PR that changes behavior must add an entry to `CHANGELOG.md` under
`## [Unreleased]`. Include what changed and why.

---

## Reporting bugs

Open a GitHub Issue with:

1. The `.vosa` source that triggered the problem (paste it in full)
2. The exact error message / output
3. Your OS and `rustc --version`

A minimal reproducing `.vosa` file is the fastest path to a fix.

---

## Hardware testing

If you have a drone and PX4/ArduPilot SITL set up:

```bash
# MAVLink over TCP (SITL default)
cargo run -- run examples/hello_world.vosa --mavlink tcp:127.0.0.1:5760

# ROS 2 (requires --features ros2 build)
cargo build --features ros2
cargo run --features ros2 -- run examples/hello_world.vosa --ros2 0
```

Hardware test reports (even "it connected but X went wrong") are extremely
valuable — open an issue with the output.
