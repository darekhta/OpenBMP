# OpenBMP Phase 1 Plan — Deterministic Core

This document plans Phase 1 of OpenBMP — the first implementation phase
following the Phase-0 documentation set. Phase 1's goal is a working
Rust workspace with a deterministic, byte-stable simulation kernel and
the CI gates the rest of the project will rely on.

The plan pins versions to **April 2026** state of the Rust ecosystem
(Rust 1.95 stable, Edition 2024). Pins are conservative where a major
crate bump is fresh (rand, approx). All policy is rooted in
`design-concept.md`, `software-architecture.md`, `safety-boundaries.md`,
`scenario-format.md`, `verification.md`, `frames-time.md`,
`data-provenance.md`, and `supply-chain.md`.

## Goal

When Phase 1 closes, a developer can run:

```bash
git clone … && cd openbmp
cargo nextest run                                  # all tests pass
openbmp run scenarios/analytic-toy/constant-acceleration-drop.toml \
    --output out/run.parquet
openbmp diff tests/golden/constant-acceleration-drop.parquet out/run.parquet
                                                   # byte-identical
openbmp check scenarios/analytic-toy/constant-acceleration-drop.toml
                                                   # passes lint
```

…and the canonical scenario set runs **byte-identically** twice on the
reference platform profile (`x86_64-unknown-linux-gnu`, pinned MSRV,
default simulation profile) — the determinism CI gate from
`verification.md`.

Phase 1 does **not** include physics fidelity. The only physics is
constant gravity in an analytic-toy scenario — enough to validate the
kernel, integrators, frames, units, RNG, telemetry, scenario parser,
golden-test infrastructure, and CI. Real environment / vehicle / aero /
propulsion / sensors / FC / hypersonic crates ship in **Phase 2 and
beyond**.

## Toolchain and Pinning Decisions

### Rust toolchain

| Setting | Value | Rationale |
|---|---|---|
| Channel | `1.95` (stable) | Latest stable as of 2026-04-16 |
| Edition | `2024` | Stable since Rust 1.85, ecosystem is caught up |
| MSRV | `1.93` (= stable − 2) | Bumps quarterly; verified by `cargo-msrv` in CI |
| Components | `rustfmt`, `clippy` | Required in CI |
| Targets | `x86_64-unknown-linux-gnu` (reference) | Determinism gate runs here; macOS / Windows / aarch64-Linux are state-stable, not bit-stable |

`rust-toolchain.toml`:

```toml
[toolchain]
channel    = "1.95"
components = ["rustfmt", "clippy"]
targets    = ["x86_64-unknown-linux-gnu"]
profile    = "default"
```

### Workspace `[workspace.dependencies]`

Pinned for Phase 1. See `software-architecture.md § Crate Selection and
Rationale` for the design rationale; pins below are the version-current
mapping.

```toml
[workspace.package]
edition      = "2024"
rust-version = "1.93"
license      = "Apache-2.0 OR MIT"
repository   = "https://github.com/<org>/openbmp"
authors      = ["OpenBMP contributors"]

[workspace.dependencies]
# --- Math / numerics
nalgebra    = { version = "0.34.2", default-features = false, features = ["std"] }
num-traits  = { version = "0.2.19", default-features = false }
num-complex = { version = "0.4.6",  default-features = false }
uom         = { version = "0.38.0", default-features = false, features = ["f64", "si", "std", "autoconvert"] }
approx      = { version = "0.5.1",  default-features = false }   # 0.6 still rc; hold
ndarray     = { version = "0.17.2", default-features = false }   # 0.17.0 yanked

# --- Determinism
rand        = { version = "0.9.4",  default-features = false, features = ["std", "std_rng"] }
rand_chacha = { version = "0.9.0",  default-features = false }   # ChaCha8Rng for kernel

# --- Statistics (sensor noise later)
statrs      = { version = "0.18.0", default-features = false }

# --- Serialization & config
serde       = { version = "1.0.228", features = ["derive"] }
serde_with  = { version = "3.18.0" }
serde_norway = { version = "0.9.42" }                            # serde_yaml replacement
toml        = { version = "1.1.2"  }
toml_edit   = { version = "0.25.11" }
postcard    = { version = "1.1.3", default-features = false, features = ["alloc"] }

# --- Telemetry archive
arrow       = { version = "58.1.0", default-features = false, features = ["ipc"] }
parquet     = { version = "58.1.0", default-features = false, features = ["arrow", "snap"] }

# --- CLI / errors / logging
clap                = { version = "4.6.1", features = ["derive", "env", "wrap_help"] }
miette              = { version = "7.6.0", features = ["fancy"] }
thiserror           = { version = "2.0.18" }
anyhow              = { version = "1.0.102" }                    # binary crates only
tracing             = { version = "0.1.44", default-features = false, features = ["std"] }
tracing-subscriber  = { version = "0.3.23", default-features = false, features = ["fmt", "env-filter"] }

# --- Containers
indexmap    = { version = "2.14.0" }
bytemuck    = { version = "1.25.0", features = ["derive"] }

# --- Builders
bon         = { version = "3.9.1" }

# --- Time (scenario boundaries; NOT in kernel)
jiff        = { version = "0.2.24" }
hifitime    = { version = "4.2.6", default-features = false, features = ["std"] }

# --- Async (bridge crate ONLY)
tokio       = { version = "1.52.1", default-features = false, features = ["rt", "macros", "io-util"] }

# --- Test / bench / fuzz
proptest        = { version = "1.11.0", default-features = false, features = ["std"] }
proptest-derive = { version = "0.8.0" }
insta           = { version = "1.47.2", features = ["yaml"] }
criterion       = { version = "0.8.2", default-features = false, features = ["plotters"] }

# --- Build helpers
duct        = { version = "1.1.1" }
```

### CI tooling pins

Installed once in CI, version-locked:

```bash
cargo install \
    cargo-deny@0.19.4 \
    cargo-vet@0.10.2 \
    cargo-cyclonedx@0.5.9 \
    cargo-audit@0.22.1 \
    cargo-machete@0.9.2 \
    cargo-msrv@0.19.3 \
    cargo-hack@0.6.44 \
    cargo-nextest@0.9.133 \
    typos-cli@1.45.1 \
    --locked
```

GitHub Action: `actions-rust-lang/setup-rust-toolchain@v1.16.0`.

### Why we hold back on some bumps

- **`rand 0.9` not 0.10.** 0.10 (2026-02-08) swapped its ChaCha
  implementation and changed several APIs (`Uniform::new` returns
  `Result`, `from_entropy` → `from_os_rng`, `thread_rng()` → `rng()`).
  Wait one quarter for ecosystem consolidation.
- **`approx 0.5.1` not 0.6.** 0.6.0-rc1 was yanked; rc2 is recent.
- **`ndarray 0.17.2`** explicitly — 0.17.0 was yanked.
- **`tracing 0.1.44` / `tracing-subscriber 0.3.23`** — earlier 0.1.42 /
  0.3.21 were yanked.

These are encoded as bans in `deny.toml` so a stray `cargo update`
can't reintroduce a yanked version.

## Sub-phase Plan

Phase 1 is decomposed into eleven sub-phases. Each declares an exit
criterion that CI checks. Sub-phases are listed in dependency order;
some can proceed in parallel (noted).

### 1.0 — Repo bootstrap

**Scope.** Create the workspace, toolchain pin, repo-level files
(`README.md`, `LICENSE-APACHE`, `LICENSE-MIT`, `CONTRIBUTING.md`,
`SECURITY.md`, `CODE_OF_CONDUCT.md`), `.gitignore`, `.editorconfig`,
empty crate stubs, baseline `Cargo.toml` workspace declaration.

**Tasks.**

- `cargo new --workspace`.
- Create the 15 crate directories per
  `software-architecture.md § Workspace Layout`. Each starts with a
  `Cargo.toml`, `src/lib.rs` (or `main.rs` for CLI / bridge), and a
  short `README.md` per the model README template.
- Add `rust-toolchain.toml`, root `Cargo.toml`, dev-only `.cargo/config.toml`
  with the FMA-disable target features for x86_64-unknown-linux-gnu
  (see `software-architecture.md § Determinism Profile`).
- Add `.github/workflows/ci.yml` skeleton (lint, format, build, test,
  empty determinism-gate placeholder).
- Add `deny.toml` (see § Configuration Files below).
- Add `dependabot.yml` for monthly `cargo` updates with grouping.
- Add top-level `README.md` linking into `docs/`.
- Add `CONTRIBUTING.md` with the safety-boundary review checklist.
- Add `SECURITY.md` with the disclosure process.

**Exit criteria.**

- `cargo build --workspace` succeeds.
- `cargo nextest run` runs zero tests (no implementations yet) and
  exits clean.
- `cargo deny check` passes against the empty workspace.
- `cargo clippy --workspace --all-targets -- -D warnings` passes.
- `cargo fmt --check` passes.

**Effort.** Small (~1–2 days).

### 1.1 — `openbmp-core`

**Scope.** L0 foundation: math, units, frames, time, deterministic RNG,
error types, validation labels, channel IDs, frame IDs.

**Tasks.**

- `SimTime`, `Duration`, `StepIndex` newtypes with documented monotonicity
  invariants and `proptest` strategies in `openbmp-testkit`.
- Frame tag types (`ECI`, `ECEF`, `NED`, `ENU`, `Body`) and the `Frame`
  trait. `Position3<F>`, `Velocity3<F>`, `AngularVelocity3<F>`,
  `Quaternion<From, To>` parametric types backed by nalgebra.
- `FrameContext` and `FrameTransform` traits. Phase 1 implements only
  `toy-fixed-earth` (per `frames-time.md` § Frame Profiles).
- `uom`-typed quantity re-exports for the public API surface.
- `MassProperties`, basic state types are Phase 1.2's job — but the
  minimum compile-time scaffolding lives here.
- `DeterministicRng` wrapping `rand_chacha::ChaCha8Rng` with
  `for_channel(scenario_seed, step, channel)` constructor.
- `ChannelId`, `ModelId`, `ScenarioId` newtypes.
- Project-wide error types: `CoreError`, `FrameError`, `TimeError`,
  `ValidationStatus` enum.
- Documentation: every public item has a `///` doc comment; module-level
  docs explain the determinism contract.

**Tests.**

- Unit tests for math primitives and frame round-trips.
- Property tests: monotonic time, normalised quaternion under
  composition, frame transform round-trip identity, RNG determinism
  across reruns and seeds, finite-value invariants.
- Doc tests for every public example.

**Exit criteria.**

- All tests green.
- 100 % of public items have `///` docs (clippy `missing_docs` deny).
- `openbmp-core` crate has zero dependencies on other openbmp crates.
- No `std::time` use anywhere in `openbmp-core` (lint enforced).

**Effort.** Medium (~1 week).

### 1.2 — `openbmp-state`

**Scope.** L1 state types: `PointMassState`, `RigidBodyState`,
`MassProperties`. Pure data types with conversion helpers. Sits between
`openbmp-core` and `openbmp-sim`.

**Tasks.**

- `PointMassState` and `RigidBodyState` per
  `software-architecture.md § State`.
- Builder types via `bon`.
- `From` / `TryFrom` between point-mass and rigid-body states (with
  documented invariants for promotion / projection).
- Validation: `state.is_finite()`, `state.is_normalised()`, etc.

**Tests.**

- Property tests: state validation invariants, builder correctness.
- Snapshot tests on `Debug` representation (insta).

**Exit criteria.** State types compile, tests pass, no `unsafe` code.

**Effort.** Small (~2 days).

**Parallelisable with.** 1.4 telemetry, after 1.1 closes.

### 1.3 — `openbmp-sim`

**Scope.** L1 lockstep kernel: `SimulationKernel`, scheduler, RK4
fixed-step `Integrator`, event handling, step pseudocode per
`software-architecture.md § Simulation Kernel`.

**Tasks.**

- `Integrator` trait with `IntegratorDeterminism` enum.
- `RK4FixedStep` implementation. Documented FMA policy
  (no `mul_add` on f64 in the integrator hot path; reduction order
  locked).
- `SimulationKernel` struct with the canonical step loop, parameterised
  over an `Integrator`, an environment, a force/moment provider, a mass
  model, sensors, and a virtual flight controller — but Phase 1 ships
  only **no-op** implementations for everything except the integrator
  and `ConstantGravityForce` (a pure scaffold for the analytic-toy
  scenario, in `openbmp-sim` for Phase 1, to be moved to `openbmp-env`
  in Phase 2).
- `EventQueue` for stop conditions (end time, validation failure,
  scripted abort).
- `SimulationError`, `IntegratorError`, `StopReason` enum types.
- Tracing instrumentation behind `tracing` feature flag (default off in
  release) so the determinism gate can run with the subscriber
  redirected.

**Tests.**

- Analytic-toy validation: constant-acceleration 1-D drop. Closed-form
  comparison to `<1e-12` relative error after 1000 steps at `dt = 0.01`.
- Property tests: monotonic step index, monotonic time, finite state
  invariant, deterministic replay.
- Microbenchmark (criterion) on the kernel hot path; baseline numbers
  recorded in the bench report.

**Exit criteria.**

- `RK4FixedStep` deterministic at the integrator level (same input →
  same f64 bits across reruns on the reference platform).
- Constant-acceleration drop validates analytically.
- Kernel step has no allocations on the hot path
  (`alloc_counter` or manual `#[no_alloc]` macro at test time).

**Effort.** Medium (~1 week).

### 1.4 — `openbmp-telemetry`

**Scope.** L5 typed channels, ring buffer, exporters (CSV, JSON,
Parquet), schema metadata.

**Tasks.**

- `TelemetrySink` trait, `TelemetryValue` typed-channel pattern.
- In-memory ring buffer for current-value access.
- CSV exporter (deterministic field ordering, no float formatting
  ambiguity — fixed-precision `{:.17e}`).
- JSON exporter (deterministic key ordering via `serde_json` + sorted
  maps; or `IndexMap` everywhere).
- Parquet exporter via `arrow`/`parquet`. Column schema includes
  unit/frame metadata; schema version embedded in file metadata.
- Golden-mode helpers for byte comparison.

**Tests.**

- Round-trip unit tests: write → read → equal.
- Snapshot tests on the JSON output of canonical channels.
- Property tests: schema version is always present, monotonic sample
  time, channel metadata is complete.
- Determinism property test: same input → byte-identical CSV / JSON /
  Parquet across reruns.

**Exit criteria.**

- Three exporters produce byte-stable output for canonical fixtures.
- `parquet`'s default features are off; we enable only `arrow` + `snap`.
- Schema metadata includes unit, frame, and version per column.

**Effort.** Medium (~1 week).

**Parallelisable with.** 1.3 sim, after 1.1 / 1.2 close.

### 1.5 — `openbmp-scenario`

**Scope.** L6 in-house TOML scenario parser, model registry, validation
rules, fault-injector hooks (deferred to Phase 3 internals; the
**hooks** ship in Phase 1 so the parser layout is stable).

**Tasks.**

- TOML parser with **unknown-field rejection** (per
  `scenario-format.md § Format Rules`).
- Required tables: `[meta]`, `[time]`, `[vehicle]`, `[environment]`,
  `[forces]`, `[telemetry]`, `[validation]`. Optional: `[epoch]`,
  `[frames]`, `[sensors]`, `[fc]`, `[faults]`, `[batch]`.
- Model registry with compile-time-known model names. Phase 1 registers
  only `point_mass` (vehicle), `constant` (gravity), `none` (atmosphere
  / wind), and `noop` (controller).
- Frame-suffix and unit-suffix linting (e.g., `mass_kg` recognised,
  `mass` rejected).
- **Safety-name lint** — reject `target`, `seeker`, `warhead`, `strike`,
  `interceptor`, `kill`, `threat`, `engagement`, terminal-homing
  variants per `safety-boundaries.md § Naming Rules`.
- File-resolution helper that turns relative paths into absolute paths
  rooted at the scenario file.

**Tests.**

- Unit tests on every required field (presence, type, range).
- Snapshot tests on parser error messages (using `miette` for
  source-span pointing).
- Fuzz target on the parser (`cargo-fuzz`, lazy initialisation but
  available from Phase 1; promoted to nightly CI).
- Property tests: round-trip for canonical scenario; rejection of
  unknown fields.

**Exit criteria.**

- All canonical scenarios parse.
- Unknown-field cases produce structured `miette` diagnostics.
- Safety-name lint catches synthetic test cases.
- Fuzz harness builds and produces a corpus.

**Effort.** Medium (~1 week).

**Parallelisable with.** 1.3 sim, 1.4 telemetry.

### 1.6 — `openbmp-testkit`

**Scope.** L6 helpers for the entire test corpus.

**Tasks.**

- `proptest::Strategy` constructors for: `SimTime`, `Duration`,
  `StepIndex`, `Position3`, `Velocity3`, `Quaternion`,
  `MassProperties`, `PointMassState`, `RigidBodyState`.
- Analytic-toy scenario generators (constant-acceleration drop,
  torque-free Euler dynamics, two-body Keplerian).
- Tolerance-table parser: read `expected.toml` per
  `verification.md § Tolerance Tables`.
- `compare_filters` helper (placeholder Phase 1 — full implementation
  in Phase 4 with real estimators).
- Determinism oracle: run the same scenario twice and compare byte by
  byte, returning a structured diff if not equal.

**Tests.** Self-tests of every helper.

**Exit criteria.** Helpers are dependency-injected into 1.3, 1.4, 1.5
tests.

**Effort.** Small-medium (~3 days).

### 1.7 — `openbmp-cli`

**Scope.** L7 user entry point: `openbmp run`, `openbmp diff`,
`openbmp check`. CLI uses `clap` derive.

**Tasks.**

- `openbmp run <scenario.toml> --output <path>` runs the scenario
  through the kernel and writes telemetry.
- `openbmp diff <golden.parquet> <actual.parquet>` reports the first
  divergent row with channel + field + values + scenario hash.
- `openbmp check <scenario.toml>` runs the lint per
  `scenario-format.md § Scenario Lint`: schema, provenance, units /
  frames, safety names, deterministic schedule.
- `openbmp check-provenance <data/>` walks the data tree and verifies
  provenance records (placeholder Phase 1, full machine-readable
  parser in Phase 2 once data lands).
- All errors via `miette`. All structured output behind `--json`.
- Tracing subscriber configurable from environment
  (`RUST_LOG` / `--trace`).

**Tests.**

- `assert_cmd` + `insta-cmd` snapshot tests on every subcommand,
  golden path and error path.
- One end-to-end test that runs the constant-acceleration scenario,
  exports Parquet, and diffs against a committed golden — covered
  jointly with sub-phase 1.8.

**Exit criteria.**

- All three subcommands work on the canonical scenario.
- CLI builds with `default-features = false` for tokio (it does not
  pull tokio at all unless the optional `bridge` feature is enabled).

**Effort.** Medium (~1 week).

### 1.8 — First end-to-end golden test

**Scope.** First analytic-toy scenario: vertical drop under constant
gravity, point-mass vehicle, no environment, no controller, no sensors.
Closed-form solution. Byte-stable telemetry.

**Tasks.**

- `scenarios/analytic-toy/constant-acceleration-drop.toml` — full
  scenario file.
- Generate the golden Parquet on a reference machine, commit to
  `tests/golden/`.
- Generate `tests/validation/constant-acceleration-drop/expected.toml`
  with peak velocity, peak position, and final state tolerances.
- End-to-end test in workspace `tests/` that runs the scenario and
  asserts byte equality plus tolerance compliance.

**Exit criteria.**

- Scenario runs in <50 ms on the reference platform.
- Golden test is byte-identical.
- Tolerance table assertions pass at `1e-12` relative.

**Effort.** Small (~2 days).

### 1.9 — Determinism CI gate

**Scope.** The headline CI job that proves byte stability per
`verification.md § Determinism Gate`.

**Tasks.**

- `.github/workflows/determinism.yml` — runs on the reference platform.
- Job runs the canonical scenario set twice (sequential, two separate
  `cargo run` invocations) and asserts byte equality of all Parquet
  artefacts.
- Job re-runs once with `RUST_LOG=info,trace` redirected to `/dev/null`
  to verify tracing subscriber side-effects do not leak.
- Cross-platform jobs (macOS, Windows, aarch64-Linux) run in matrix
  but produce **state-stable, not bit-stable** comparisons via the
  tolerance-table mode of `openbmp diff`. Phase 1 may keep these as
  warning-only and promote to blocking later.

**Exit criteria.**

- Determinism gate green on `x86_64-unknown-linux-gnu`.
- Cross-platform gates produce structured diff reports.

**Effort.** Small (~2 days).

### 1.10 — Provenance + supply-chain CI gates

**Scope.** All policy CI gates from `supply-chain.md` and
`data-provenance.md`.

**Tasks.**

- `.github/workflows/ci.yml` adds: `cargo deny check`,
  `cargo vet check`, `cargo audit`, `cargo machete`,
  `cargo msrv verify`, `cargo hack check --feature-powerset`,
  `cargo nextest run`, `typos`, `cargo fmt --check`,
  `cargo clippy -- -D warnings`.
- `.github/workflows/release.yml` produces release artefacts:
  source archive, CLI binary, `Cargo.lock`, CycloneDX SBOM,
  dependency audit report, scenario / golden manifest, non-suitability
  disclaimer, SLSA Build Level 2 provenance via
  `slsa-framework/slsa-github-generator`.
- `deny.toml` populated per § Configuration Files below.
- `vet.toml` skeleton with Mozilla shared audits as initial trust
  source.
- Trusted Publishing wired up for `cargo publish` (post-Phase-1
  releases).

**Exit criteria.**

- All gates green on the empty / stub workspace at sub-phase close.
- Release dry-run produces all required artefacts.

**Effort.** Medium (~3 days).

### 1.11 — Crate READMEs and modeling-guide compliance

**Scope.** Each of the 15 crates carries a `README.md` per
`modeling-guide.md` template. Two crates per day for a week.

**Tasks.**

- Crate READMEs: Purpose / Inputs and Outputs / Units and Frames /
  Assumptions / Validity Range / Determinism / Validation / Data
  Provenance / Safety Boundary.
- Bibliography per crate naming the public references it draws on
  (e.g., `openbmp-core/README.md` cites Vallado, IEEE 1139, and the
  uom / nalgebra version pins).

**Exit criteria.** Every crate `README.md` exists and links into the
`docs/` set.

**Effort.** Small-medium (~3 days).

## Configuration Files

The exact contents of the policy files. Each becomes part of repo
bootstrap.

### `rust-toolchain.toml`

```toml
[toolchain]
channel    = "1.95"
components = ["rustfmt", "clippy"]
targets    = ["x86_64-unknown-linux-gnu"]
profile    = "default"
```

### Workspace `Cargo.toml` (root, fragment)

```toml
[workspace]
resolver = "2"
members  = [
    "crates/openbmp-core",
    "crates/openbmp-state",
    "crates/openbmp-sim",
    "crates/openbmp-env",
    "crates/openbmp-vehicle",
    "crates/openbmp-aero",
    "crates/openbmp-aerothermal",
    "crates/openbmp-propulsion",
    "crates/openbmp-sensors",
    "crates/openbmp-fc",
    "crates/openbmp-telemetry",
    "crates/openbmp-scenario",
    "crates/openbmp-testkit",
    "crates/openbmp-cli",
    "crates/openbmp-bridge",
]

[workspace.package]
edition      = "2024"
rust-version = "1.93"
license      = "Apache-2.0 OR MIT"
repository   = "https://github.com/<org>/openbmp"
authors      = ["OpenBMP contributors"]

[workspace.lints.rust]
missing_docs                  = "deny"
missing_debug_implementations = "warn"
unsafe_code                   = "deny"   # rare unsafe must be opt-in per crate
unused_must_use               = "deny"

[workspace.lints.clippy]
pedantic                      = { level = "warn", priority = -1 }
all                           = { level = "warn", priority = -1 }
unwrap_used                   = "warn"
expect_used                   = "warn"
panic                         = "warn"
float_cmp                     = "deny"
```

The full `[workspace.dependencies]` block from § Toolchain and Pinning
Decisions slots in after these.

### `.cargo/config.toml`

```toml
[build]
incremental = false                       # determinism > rebuild speed in CI

[target.x86_64-unknown-linux-gnu]
rustflags = [
    "-C", "target-feature=-fma",          # disable FMA codegen on the
                                          # reference platform to keep
                                          # bit-stable output. See
                                          # software-architecture.md
                                          # § Determinism Profile.
    "-C", "force-frame-pointers=yes",     # easier profiling
    "-C", "strip=none",                   # release strip handled separately
]

```

Release profile settings live in the root `Cargo.toml`. `trim-paths` is not
used until it is stable on the pinned Rust toolchain.

### `deny.toml`

```toml
[graph]
all-features = true

[advisories]
db-urls    = ["https://github.com/rustsec/advisory-db"]
yanked     = "deny"
ignore     = []                           # populated as we encounter exceptions

[licenses]
allow = [
    "Apache-2.0",
    "Apache-2.0 WITH LLVM-exception",
    "MIT",
    "BSD-2-Clause",
    "BSD-3-Clause",
    "ISC",
    "Unicode-DFS-2016",
    "Unicode-3.0",
    "Zlib",
    "CC0-1.0",
    "MPL-2.0",
]
confidence-threshold = 0.93

[bans]
multiple-versions = "warn"
wildcards         = "deny"
deny = [
    # Yanked-version landmines from the April 2026 ecosystem survey
    { crate = "ndarray@=0.17.0",            reason = "yanked" },
    { crate = "tracing@=0.1.42",            reason = "yanked" },
    { crate = "tracing-subscriber@=0.3.21", reason = "yanked" },
    { crate = "color-eyre@=0.6.4",          reason = "yanked" },
    { crate = "approx@=0.6.0-rc1",          reason = "yanked" },
    { crate = "cargo-deny@=0.19.3",         reason = "yanked" },
    # Deprecated upstream
    { crate = "serde_yaml",                 reason = "deprecated; use serde_norway" },
    { crate = "chrono",                     reason = "prefer jiff for new code" },
]

[bans.workspace-dependencies]
duplicates    = "deny"
unused        = "deny"

[sources]
unknown-registry = "deny"
unknown-git      = "deny"
allow-org        = []
```

`tokio` is **not** banned globally because the bridge crate needs it,
but a per-crate `cargo deny` rule (or a `forbid_dependencies` lint
integration) restricts it to `openbmp-bridge` only.

### `.github/workflows/ci.yml` (skeleton)

Key elements (full file written during sub-phase 1.0):

- One job per gate, fail-fast off so all gates surface their findings.
- `actions-rust-lang/setup-rust-toolchain@v1.16.0` everywhere.
- Cache via `Swatinem/rust-cache@v3` (the de facto standard; pin major).
- `cargo nextest run --workspace --all-features` as the test runner.
- Determinism gate runs on `ubuntu-latest`, x86_64.
- Cross-platform matrix runs on `macos-latest`, `windows-latest`,
  `ubuntu-latest` (aarch64 emulation if available).
- Nightly schedule: fuzz corpus run, criterion benchmark trend, full
  golden corpus.

### `dependabot.yml`

```yaml
version: 2
updates:
  - package-ecosystem: cargo
    directory: /
    schedule:
      interval: monthly
    open-pull-requests-limit: 5
    groups:
      arrow-stack:
        patterns: ["arrow", "parquet"]
      tracing-stack:
        patterns: ["tracing", "tracing-subscriber"]
      serde-stack:
        patterns: ["serde", "serde_*"]
      proptest-stack:
        patterns: ["proptest", "proptest-derive"]
  - package-ecosystem: github-actions
    directory: /
    schedule:
      interval: monthly
```

## Risk Register

| Risk | Mitigation |
|---|---|
| FMA codegen variance breaks byte stability cross-CPU | `target-feature=-fma` on the reference platform; cross-platform gates are state-stable not bit-stable |
| Edition-2024 never-type fallback affects a generic helper | Lint denies bare divergent closures; helpers annotated explicitly |
| Yanked dependency reintroduced via transitive update | `deny.toml` ban list with explicit version pins |
| `rand 0.10` adopted accidentally | workspace pin to 0.9.4 (not `0.9`); `cargo deny` warns on `multiple-versions` |
| Tracing subscriber side-effect leaks into deterministic outputs | Determinism gate runs once with subscriber redirected to `/dev/null` |
| `parquet`/`arrow` minor bumps break schema | Pin major; nightly job tests against latest minor; release-time pin lock |
| TOML parser permissive on unknown fields | Use `serde::Deserialize` with `deny_unknown_fields`; explicit unit tests |
| `cargo deny` advisories block CI on a transient ecosystem fix | Advisories warning-only during early Phase 1; blocking from sub-phase 1.10 onward |
| FFI to nalgebra-glm silently pulls f32 paths | Avoid nalgebra-glm; only nalgebra (f64 default) |

## Out-of-Scope (Phase 1)

Explicit reminders. Anything below is scheduled for Phase 2+, or
`out-of-roadmap` per `safety-boundaries.md`.

- Real environment models (atmosphere, gravity, wind, magnetic) beyond
  the constant-gravity scaffold needed for the analytic-toy scenario.
- Real vehicle / aero / propulsion / sensors / FC implementations.
- Hypersonic everything (Phase 6).
- Real device drivers, real bus protocols, real flight-computer ports.
- Targeting, terminal-homing, intercept, payload-delivery.
- Real fielded-vehicle parameter sets.
- Adaptive integrators (DOPRI5/8) — Phase 5 behind a profile flag.
- Many-body / multi-vehicle scenarios.
- The optional `openbmp-bridge` socket bridge (compiles as an empty
  crate in Phase 1; real implementation in Phase 5).
- Visualization and plotting tools.

## Acceptance Gate for Phase 1 Closure

Phase 1 is complete when **all** of the following are simultaneously
true on a clean clone of the repository:

- `cargo build --workspace --all-features` succeeds.
- `cargo nextest run --workspace --all-features` passes.
- `cargo deny check` passes.
- `cargo vet check` passes.
- `cargo audit` reports zero blocking advisories.
- `cargo machete` reports no unused dependencies.
- `cargo msrv verify` passes against `rust-version = "1.93"`.
- `cargo clippy --workspace --all-targets -- -D warnings` passes.
- `cargo fmt --check` passes.
- `typos` passes.
- The determinism gate runs the canonical scenario set twice and
  produces byte-identical Parquet output.
- `openbmp run`, `openbmp diff`, and `openbmp check` work on every
  shipped scenario.
- A release dry-run produces source archive, CLI binary, `Cargo.lock`,
  CycloneDX SBOM, dependency audit report, scenario / golden
  manifest, non-suitability disclaimer, and SLSA L2 provenance.
- Each of the 15 crates has a `README.md` following the model template.
- `docs/` is updated where any architecture detail shifted during
  implementation.

## Suggested Calendar (rough)

| Week | Sub-phases | Notes |
|---|---|---|
| 1 | 1.0 | Bootstrap, all CI scaffold, deny.toml |
| 2–3 | 1.1 | Core types, frames, time, RNG |
| 4 | 1.2, 1.4 (start) | State + telemetry exporters |
| 5 | 1.3, 1.4 (finish) | Kernel + RK4 + telemetry done |
| 6 | 1.5, 1.6 | Scenario parser + testkit |
| 7 | 1.7 | CLI |
| 8 | 1.8, 1.9 | First golden + determinism gate |
| 9 | 1.10, 1.11 | Supply-chain gates + crate READMEs |

Total: ~9 weeks of focused single-developer work, or ~5–6 weeks if 1.4
and 1.5 truly proceed in parallel with two contributors. The
calendar is suggestive, not a commitment.

## What Phase 2 Looks Like

Once Phase 1 closes, Phase 2 begins with the first real physics:

- `openbmp-env`: US Standard Atmosphere 1976, J2 gravity,
  constant/layered/gust wind, magnetic-field stub.
- `openbmp-vehicle`: rigid-body trait, constant-mass and linear-burn
  mass models, analytic-toy force/moment providers.
- `openbmp-aero`: stub aero deck with synthetic CD(Mach, alpha).
- `openbmp-propulsion`: synthetic solid-motor model, in-house TOML
  thrust-curve format.
- `openbmp-sensors`: ideal-state and synthetic IMU.
- Updated golden corpus: ballistic toy with drag, torque-free Euler,
  two-body Keplerian.

Phase 2 is the validation milestone where OpenBMP starts to *do
physics* — but it's only possible because Phase 1 nailed determinism,
testing, scenario parsing, telemetry, and CI gates first.

## References

- `design-concept.md` — high-level Phase 1 plan and MVP scope.
- `software-architecture.md` — workspace layout, crate selection
  rationale, kernel design, determinism profile.
- `safety-boundaries.md` — accept/reject rules and naming policy.
- `scenario-format.md` — Phase-1 scenario contract.
- `verification.md` — validation labels, test classes, determinism
  gate.
- `frames-time.md` — frame/time conventions for the scenario layer.
- `data-provenance.md` — data record requirements.
- `supply-chain.md` — CI-tool policy and SBOM requirements.

External (verified at planning time, 2026-04-25):

- Rust 1.95.0 release notes — <https://blog.rust-lang.org/>
- Edition 2024 reference — <https://doc.rust-lang.org/edition-guide/rust-2024/>
- crates.io API — <https://crates.io/api/v1>
- SLSA v1.1 spec — <https://slsa.dev/spec/v1.1/>
- `slsa-framework/slsa-github-generator` —
  <https://github.com/slsa-framework/slsa-github-generator>
- `actions-rust-lang/setup-rust-toolchain` —
  <https://github.com/actions-rust-lang/setup-rust-toolchain>
- `cargo-deny` config docs —
  <https://embarkstudios.github.io/cargo-deny/>
