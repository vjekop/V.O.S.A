# Contributing to V.O.S.A.

Thank you for your interest in V.O.S.A.! This document explains how to contribute and how the project is organized.

---

## Commit Convention

Every commit **must** follow this format so the changelog stays accurate:

```
<type>(<scope>): <short description>

<body — explain what changed and WHY, not just what>
```

### Types

| Type | When to use |
|---|---|
| `feat` | A new feature or language capability |
| `fix` | A bug fix |
| `docs` | Documentation only (README, CHANGELOG, comments) |
| `refactor` | Code restructuring with no behavior change |
| `test` | Adding or updating tests |
| `chore` | Build system, CI, dependency updates |
| `perf` | Performance improvement |

### Scopes

| Scope | What it covers |
|---|---|
| `lexer` | `src/lexer/` |
| `parser` | `src/parser/` |
| `safety` | `src/safety/` |
| `runtime` | `src/runtime/` |
| `cli` | `src/main.rs` |
| `examples` | `examples/` |
| `deps` | `Cargo.toml` changes |

### Examples

```
feat(lexer): add support for hex color literals
fix(safety): correctly check waypoint altitude with min_altitude
docs: update CHANGELOG for v0.2.0
refactor(parser): extract param list parsing into helper
```

---

## Changelog Rule

**Every commit that changes behaviour must update `CHANGELOG.md`.**

Add your entry under `## [Unreleased]` before opening a PR. Include:
- What file(s) changed
- What the change does
- Why it was necessary

---

## Branch Strategy

| Branch | Purpose |
|---|---|
| `main` | Stable, always builds |
| `dev` | Active development — PRs merge here first |
| `feat/<name>` | Feature branches |
| `fix/<name>` | Bug fix branches |

---

## Development Setup

```bash
git clone https://github.com/vjekop/V.O.S.A.git
cd V.O.S.A
cargo build
cargo test
```

To run a VOSA file during development:
```bash
cargo run -- run examples/hello_world.vosa
```

---

## Code Style

- Run `cargo fmt` before every commit
- Run `cargo clippy` and address all warnings
- All public items must have doc comments (`///`)

---

## Reporting Issues

Open a GitHub Issue with:
1. The `.vosa` source that triggered the problem
2. The exact error message / output
3. Your OS and Rust version (`rustc --version`)
