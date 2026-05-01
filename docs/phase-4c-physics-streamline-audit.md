# `openbmp-physics` post-consolidation streamline audit handover

> **Audience.** An external auditor / senior reviewer. Independent
> of the prior author. No prior context with this codebase. Mandated
> to **streamline-review**, not deep-bug-hunt.
>
> **Scope.** This is a sweep over the full `openbmp-physics` crate
> after Phase 4.C consolidation lands. Specifically, the crate as it
> stands at HEAD (commit `71edfdf` "Phase 4.C: move FrameContext +
> WGS84 from openbmp-core to openbmp-physics::frames"). Two prior
> external audits have covered the major moves
> (`docs/phase-4c-external-audit-findings.md`,
> `docs/phase-4c-physics-followup-external-audit-findings.md`) and a
> third covers the most recent FrameContext consolidation
> (`docs/phase-4c-frames-consolidation-external-audit.md`). Run **after**
> that third audit closes; this one assumes the consolidation itself
> is correct and asks a different question: **is the resulting
> `openbmp-physics` crate well-shaped?**
>
> **Stance.** Pragmatic-skeptical. The crate has grown by
> accumulation: env retired into it, kinematics + statistics + frames
> migrated in, dynamic-pressure helper added, `MagneticFieldEci` /
> `MagneticModel` dual-trait surface left from a partial unification.
> The risk is not "math is wrong" — that's been audited. The risk is
> "the crate is now a junk drawer." Find the junk-drawer signs.
>
> **Read-only.** Authorized to read every file, run any deterministic
> command, and write a findings + recommendations report. **Not**
> authorized to modify code, push commits, or refactor while
> auditing. If a finding suggests a refactor, name it; the project
> owner decides what to do.

## Why this audit exists

The `openbmp-physics` crate has accumulated:

- 18 source files, 4 914 lines, 9 modules (`atmosphere`, `error`,
  `frames`, `gravity`, `kinematics`, `magnetic`, `statistics`,
  `validity`, `wind`) plus an `earth` constants sub-module.
- Two trait families that overlap (`MagneticModel` and
  `MagneticFieldEci`) for historical FC / sim split reasons.
- Multiple validity-envelope conventions (closed-form clamp in
  `IsothermalAtmosphere`, layered range in `UsStandard1976` with an
  `ExoatmosphericPolicy`, ad-hoc `is_finite` checks in `gravity`).
- Multiple constant-naming conventions (`WGS84_*`, `USSA76_*`,
  `EARTH_DIPOLE_EQUATORIAL_FIELD_NT`, `STANDARD_GRAVITY_M_S2`,
  `MEAN_RADIUS_M` in a sub-module).
- Two separate quaternion-construction surfaces: nalgebra's
  `UnitQuaternion::from_axis_angle` is used in `frames` tests, and
  the workspace's own `quaternion_from_axis_angle` lives in
  `kinematics`.

Each of these is defensible in isolation, but the question is
whether the crate as a whole reads as a coherent library or as a
collection of imported drawers. The answer dictates whether
near-term phases need a streamline pass or whether the current
shape is load-bearing.

## What you're reviewing

Nothing is being changed during this audit. You're producing a
**recommendations document** with three buckets:

1. **Definite cleanups** — strictly cosmetic / non-functional, would
   have caught at code review if a reviewer were familiar with the
   whole crate. Examples: redundant doc references, drift between
   `lib.rs` re-exports and module visibility, naming inconsistencies
   that don't break anything.

2. **Architectural smells** — patterns that are not bugs but suggest
   the wrong abstraction. Examples: two traits that should be one,
   one trait that should be two, a sub-module that should be its
   own top-level module, a top-level module that should fold into
   another.

3. **Out of scope but noted** — observations about the crate's
   relationship with the rest of the workspace that don't fit in
   #1 or #2 but are worth surfacing for the project owner. Examples:
   `openbmp-physics::frames::FrameError` vs
   `openbmp-core::FrameError` (cross-crate error types), trait-vs-
   free-function consistency between `physics::statistics` and
   `physics::kinematics`.

## Review dimensions

### A. Module shape

For each top-level module:

- Is the module's public surface minimal? Are private helpers
  exposed unnecessarily?
- Does the module's docstring (`//!`) accurately describe what's in
  it, or is it stale framing prose from a prior phase?
- Is the module's *name* consistent with the workspace conventions?
  E.g. plural vs singular (`frames` vs `gravity`), method vs concept
  (`statistics` vs `chi_square`).
- Does the module re-export from sub-modules in a consistent
  pattern? (E.g. `wind` does `pub use constant::*;` style; does
  `magnetic`?)
- If the module has more than 500 lines, is it well-organized into
  named sections?

The current modules and their line counts (after consolidation):

| Module | Lines | Notes |
|---|---|---|
| `atmosphere/mod.rs` | 231 | trait + sample type + helpers |
| `atmosphere/isothermal.rs` | 136 | toy model |
| `atmosphere/us_standard_1976.rs` | 648 | full 7-layer |
| `error.rs` | 34 | single error type |
| `frames.rs` | 724 | newly consolidated |
| `gravity.rs` | 517 | three models + constants |
| `kinematics.rs` | 131 | four primitives |
| `lib.rs` | 93 | crate root |
| `magnetic/mod.rs` | 91 | two traits |
| `magnetic/dipole.rs` | 128 | toy model |
| `magnetic/wmm2025.rs` | 633 | NOAA model |
| `statistics.rs` | 140 | two functions |
| `validity.rs` | 56 | one type |
| `wind/mod.rs` | 87 | trait |
| `wind/constant.rs` | 245 | NoWind + ConstantWind |
| `wind/gust.rs` | 628 | Dryden |
| `wind/layered.rs` | 392 | step-table |

Read each `mod.rs` (or top-level file for flat modules) in order
and note shape inconsistencies.

### B. Public surface coherence

Open `crates/openbmp-physics/src/lib.rs`. Every `pub use` line is
a public-API commitment. Verify:

- Each re-exported item is named once. No duplicate re-exports.
- Each re-exported item is from the most appropriate module path.
  (E.g. `WGS84_J2` is re-exported from `gravity::WGS84_J2`; that's
  fine because J2 is a gravity coefficient, not an ellipsoid
  constant. Check for cases where the re-export source feels
  arbitrary.)
- The `pub use frames::{...}` block is comprehensive — anything a
  consumer needs from `frames` should be re-exported, so consumers
  can write `use openbmp_physics::FrameContext;` not
  `use openbmp_physics::frames::FrameContext;`. Or, if the
  convention is to require the sub-path, then **none** of `frames`
  should be re-exported. Pick one; flag inconsistency.
- Re-exports of types from sub-modules vs top-level constants: is
  the distinction principled? (E.g. `WindModel` is a top-level
  re-export but `wind::ConstantWind` could also be re-exported as
  `openbmp_physics::ConstantWind` — and is, per current `lib.rs`.
  Verify the rule applies symmetrically across `magnetic`, `wind`,
  `atmosphere`, `gravity`.)
- The `earth` sub-module: is the only constant in it
  (`MEAN_RADIUS_M`) better placed in `gravity` or as a top-level
  re-export? An entire module for one constant is a smell unless
  there's a planned expansion.

### C. Trait-family consistency

The `MagneticModel` (NED-output) and `MagneticFieldEci` (ECI-output)
traits coexist by design — the prior physics-followup audit
documented that the simulator-side adapter consumes `MagneticModel`
with full envelope error handling, while the FC-side estimator
consumes `MagneticFieldEci` without dragging in geodetic-conversion
machinery.

Test that defense:

- Is the dual-trait surface still load-bearing, or is one trait now
  a thin wrapper around the other?
- Specifically: does any sim-side consumer use `MagneticFieldEci`?
  Does any FC-side consumer use `MagneticModel`? `grep -rn` the
  whole workspace.
- If the trait-split is justified, is the same split applied
  symmetrically to gravity? `GravityModel` is single-trait. Why
  doesn't gravity need a `GravityFieldEci`-equivalent split? (The
  honest answer is probably "gravity output is already ECI-only" —
  but if so, the asymmetry is a documentation gap, not a code
  problem.)
- Atmosphere has `AtmosphereModel` only; it produces an
  `AtmosphereSample` (density, pressure, temperature, speed of
  sound) — frame-agnostic by physics. Wind has `WindModel` only;
  it produces NED. Is the wind/atmosphere asymmetry deliberate
  (NED makes physical sense; atmosphere quantities are scalar /
  frame-agnostic) or accidental?

### D. Error type consistency

Errors in this crate:

- `openbmp_physics::PhysicsError` (in `physics/error.rs`) — runtime
  evaluation failures: out-of-envelope, non-finite, invalid
  parameter.
- `openbmp_core::FrameError` (in `core/error.rs`) — frame
  operations: not-finite, not-normalised, invalid-tolerance,
  transform-not-available, local-origin-required, invalid-geodetic-
  coordinate.

The `openbmp-physics::frames` module returns `FrameError` (e.g.
`LocalOriginRequired`). The other physics modules return
`PhysicsError`. Worth flagging:

- Is the split `FrameError` / `PhysicsError` principled (frame
  ops vs model evaluation)? Or is it historical (frame ops were in
  core, model ops in physics, the split survived the move)?
- If physics is going to return *two* error types, is there a
  uniform top-level alias or `From` impls so consumers don't have
  to plumb both? `PhysicsError` does **not** currently `#[from]`
  `FrameError`. Should it?
- Variant naming: `PhysicsError::OutOfEnvelope` vs
  `FrameError::InvalidGeodeticCoordinate` — the first is a generic
  envelope-violation, the second is a specific kind of envelope-
  violation. Consistent? Could `FrameError::InvalidGeodeticCoordinate`
  be unified into a generic out-of-envelope variant?

### E. Constant-naming consistency

Constants in the crate:

- `WGS84_A_M`, `WGS84_INV_FLATTENING`, `WGS84_FLATTENING`,
  `WGS84_ECCENTRICITY_SQUARED`, `WGS84_MU_M3_S2`,
  `WGS84_OMEGA_RAD_S` (in `frames`).
- `WGS84_J2` (in `gravity`).
- `STANDARD_GRAVITY_M_S2` (in `gravity`).
- `USSA76_*` constants (in `atmosphere/mod.rs` and
  `atmosphere/us_standard_1976.rs`).
- `EARTH_DIPOLE_EQUATORIAL_FIELD_NT` (in `magnetic`).
- `MEAN_RADIUS_M` (in `earth`).

Audit the naming:

- The `WGS84_*` family lives in `frames` except `WGS84_J2`. Is
  splitting `J2` from the rest principled? (Yes: J2 is a gravity-
  *model* coefficient, not an ellipsoid primitive — multiple
  gravity models can use different J2 values.) But document the
  rule or callers may guess wrong.
- `STANDARD_GRAVITY_M_S2` — the `_M_S2` suffix matches the
  workspace convention (units in suffix). The `STANDARD_` prefix
  is unusual; would `EARTH_SURFACE_GRAVITY_M_S2` or
  `WGS84_NOMINAL_G_M_S2` be more discoverable? (Don't push for a
  rename if the current name has external citations; flag as
  observation.)
- `USSA76_` prefix lengths: some constants are long
  (`USSA76_SEA_LEVEL_DENSITY_KG_M3`,
  `USSA76_MAX_GEOPOTENTIAL_M`); is the prefix earning its keep, or
  could the constants be inside an `atmosphere::ussa76` namespace
  to drop the prefix?

### F. Test-suite shape

Run `cargo test -p openbmp-physics --all-features` and look at the
output:

- Each module has tests; do the tests follow a uniform pattern
  (common helpers, common epsilon conventions)?
- Are the tests organized into named sub-modules (e.g.
  `tests::wgs84` for the WGS84-specific block in `frames`), or
  flat?
- Are property-based tests (`proptest!`) used consistently? The
  `frames` consolidation moved several tests; were any
  `proptest!` blocks dropped on the way?
- Are doc-tests present where they would help users (e.g.
  `dynamic_pressure_pa` example)?
- Run `cargo test -p openbmp-physics --no-default-features` to
  confirm the `synthetic`-feature-gated tests work both ways.

### G. Documentation flow

Read these in order:

1. `crates/openbmp-physics/src/lib.rs` doc comment.
2. `crates/openbmp-physics/README.md`.
3. Each module's `//!` header.

Verify:
- The lib doc, README, and module headers tell the same story
  about what's in the crate.
- The "Module map" in the README is a grep'able truth source: each
  named module is in the file system, each file in the file system
  is named.
- The "Layering with `openbmp-core`" section is current (post-
  consolidation).
- No section still says "outstanding follow-up" / "deferred to" /
  "Phase 5 will" with respect to anything that already shipped.
- No section names a removed item (e.g. `openbmp-env`) without
  explicitly noting it's retired.

### H. Determinism contract clarity

The crate-level doc says:

> Pure `f64` arithmetic with locked operand order on every model;
> no FMA, no wall-clock time, no system RNG, no network, no file
> I/O.

Verify:
- Every public function with arithmetic preserves operand order
  (no `mul_add`, no associativity-changing rewrites). Spot-check:
  - `frames::FrameContext::eci_to_ecef_velocity` — locked-order?
  - `gravity::J2Gravity::gravity_eci_m_s2` — locked-order with
    explicit `let` bindings, no FMA?
  - `atmosphere::dynamic_pressure_pa` — verify body is
    `0.5 * density * velocity * velocity` not
    `density.mul_add(velocity * velocity, 0.0) * 0.5` or similar.
  - `wind::gust::*` — Dryden filter is a delicate IIR; locked order?
  - `magnetic::wmm2025::*` — spherical-harmonic sum; locked order
    matters.
- No `std::time::*` import, no `rand::*` (only
  `openbmp_core::DeterministicRng`).
- The `synthetic` feature is the only feature gate; verify it gates
  only the `GustWind` Dryden filter and nothing else.

### I. Cross-module coupling

For each pair of modules, ask: "is module A's hot path coupled to
module B in a way that suggests they should be co-located, or in a
way that suggests B's interface needs improvement?"

- `frames` ↔ `gravity`: `gravity` consumes `WGS84_A_M` and
  `WGS84_MU_M3_S2` from `frames`. Tight coupling, principled.
- `frames` ↔ `magnetic`: `wmm2025` consumes `WGS84_A_M`,
  `WGS84_ECCENTRICITY_SQUARED`, `FrameContext` from `frames`.
  Wider surface; check that the consumer doesn't reach into
  `frames` internals.
- `frames` ↔ `wind`: `wind/{constant,gust,layered,mod}` consume
  `FrameContext`. Is the consumer surface clean (one type), or
  does each wind variant pull in a different subset?
- `frames` ↔ `atmosphere`: should be **no** coupling; atmosphere
  is altitude-driven, not frame-aware. Verify by `grep`.
- `kinematics` ↔ anything: should be **standalone**; primitives
  consumed by FC and sim, no internal physics dependencies.
  Verify.
- `statistics` ↔ anything: should be **standalone**; same as
  kinematics.
- `validity` ↔ anything: utility module; check whether `gravity`
  / `atmosphere` / `wind` use it consistently or each rolls its
  own envelope-clamping.

### J. Out-of-scope but worth flagging

Watch for these as you read:

- **Cross-crate FrameError.** `physics::frames` returns
  `core::error::FrameError`. Architecturally clean (FrameError is
  a primitive that callers may already handle), but it means
  `openbmp-physics` doesn't fully own its own error surface.
  Flag as observation; the project owner already weighed the
  tradeoff.
- **`MEAN_RADIUS_M` in `earth`.** A whole sub-module for one
  constant. If no one's planning to add more, it should fold into
  `frames` or `gravity`. Flag as observation.
- **`openbmp-core::DeterministicRng`** lives in core but
  `physics::wind::gust` is its primary user. Is the determinism
  primitive in the right crate? (Probably yes — multiple workspace
  crates use it — but worth confirming.)

## Reproduction commands

```bash
# Mechanical gates
cargo +1.95 fmt --all -- --check
cargo +1.95 clippy --workspace --all-targets --all-features --locked -- -D warnings
cargo +1.95 build --workspace --all-targets --all-features --locked
cargo +1.95 test --workspace --all-features --locked

# Physics-only
cargo +1.95 build -p openbmp-physics --all-features --locked
cargo +1.95 test  -p openbmp-physics --all-features --locked
cargo +1.95 build -p openbmp-physics --no-default-features --locked
cargo +1.95 test  -p openbmp-physics --no-default-features --locked

# Deps + dead code
cargo deny check
cargo machete

# Module-shape inventory
find crates/openbmp-physics/src -name '*.rs' | xargs wc -l | sort -n

# Public-surface enumeration
grep -n "^pub " crates/openbmp-physics/src/lib.rs

# Cross-module coupling map
for mod in atmosphere error frames gravity kinematics magnetic statistics validity wind; do
  echo "=== $mod uses ==="
  grep -h "^use " crates/openbmp-physics/src/$mod.rs \
    crates/openbmp-physics/src/$mod/*.rs 2>/dev/null | \
    grep -v "^use std::\|^use nalgebra\|^use openbmp_core" | sort -u
done

# Determinism red flags
grep -rn "mul_add\|f64::mul_add\|std::time\|SystemTime\|rand::thread_rng\|rand::random" \
  crates/openbmp-physics/src/

# Trait-overlap check
grep -rn "impl MagneticModel\|impl MagneticFieldEci" crates/

# Constant-naming consistency
grep -rn "^pub const " crates/openbmp-physics/src/

# Operand-order spot-check on the helper that consumed the
# inline formula in the prior physics-followup audit:
grep -A 3 "pub fn dynamic_pressure_pa" crates/openbmp-physics/src/atmosphere/mod.rs
```

## Deliverables

`docs/phase-4c-physics-streamline-audit-findings.md` (you write
it). Sections:

```markdown
# `openbmp-physics` post-consolidation streamline audit findings

Auditor: <your name>
Date: <ISO 8601>
Working tree at: 71edfdf
Toolchain: <rustc --version>

## Executive summary

3-5 sentences. Is the post-consolidation `openbmp-physics` crate
well-shaped? Major recommendations: rename / split / merge / fold?

## Module-shape table

| Module | Lines | Public surface clean? | Doc up to date? | Notes |
|---|---|---|---|---|
| `atmosphere` | ... | ... | ... | ... |
| ... | ... | ... | ... | ... |

## Bucket 1: definite cleanups

| # | File:line | Issue | Suggested fix | Effort |
|---|---|---|---|---|
| 1 | ... | ... | ... | trivial / small / medium |

## Bucket 2: architectural smells

| # | What | Why a smell | Suggested direction | Effort |
|---|---|---|---|---|
| 1 | ... | ... | ... | small / medium / workspace-shape |

## Bucket 3: out of scope but noted

For each: a sentence describing the observation and why the
auditor felt it didn't fit in #1 or #2.

## Trait-family deep-dive (MagneticModel vs MagneticFieldEci)

A focused read of whether the two-trait surface is justified.
Verdict: keep / merge / refactor / unclear.

## Error-type deep-dive (PhysicsError vs FrameError)

A focused read of the cross-crate FrameError choice.
Verdict: keep / unify / unclear.

## Determinism contract verification

For each public function with arithmetic that participates in
hot paths: confirmed-locked-order / suspected-FMA-risk / unclear.

## Sign-off

Whether the crate is ready for the next phase, or whether a
streamline pass should land first. If a streamline pass is
recommended, give a tractable scope (e.g. "fold `earth` into
`gravity`; merge bucket-1 cleanups; that's it") not a wishlist.
```

## Authorization scope

You may:
- Read every file in the repo.
- Run any `cargo` command (build, test, clippy, fmt, deny,
  machete, run, doc, tree).
- Run any deterministic shell command for inspection (grep, find,
  diff, sha256sum, cmp, git log / show / blame).
- Write your findings to
  `docs/phase-4c-physics-streamline-audit-findings.md`.
- Write throwaway files to `target/audit-scratch/` to verify
  shape / trait claims (e.g. a small bin that exercises the
  trait family in both modes).

You may NOT:
- Modify production code, scenarios, data files, or the prior
  author's documentation.
- Stage or commit changes (other than the findings doc).
- Push, force-push, branch-delete, rebase, or reset.
- Run anything that requires network access.
- Modify dependencies or `Cargo.lock`.
- Disable any test, lint, or feature gate.

If a recommendation requires code modification to verify, file the
recommendation and stop. Don't fix.

## Non-goals

- Re-running the consolidation audit
  (`docs/phase-4c-frames-consolidation-external-audit.md`). That
  work is independent and should land first.
- Re-running the prior physics-followup audit. Findings closed in
  commit `bb9da62`.
- Math-correctness audits of moved / migrated algorithms. Those
  were verified in prior rounds.
- Speculative future-phase work. Phase 5 (LSP-equivalent FC
  surface, real-time bridge) is its own scope.
- Rewrites of the FC-side estimator surface. The FC consumes
  `physics::statistics`, `physics::kinematics`,
  `physics::MagneticFieldEci`; that consumer relationship is in
  scope for FC audits, not this one.

## Tone expectations

- **Recommend, don't dictate.** "Fold `earth::MEAN_RADIUS_M` into
  `gravity` — there's no other earth-scale constant" is a
  recommendation. "The `earth` module must be removed" is not.
- **Estimate effort.** For each cleanup recommendation, give a
  rough size: trivial (one-line edit), small (single-file
  refactor, < 50 lines), medium (multi-file refactor, > 50 lines
  but < 500), workspace-shape (multi-crate, requires byte-stable
  scenario re-run). The project owner uses these to prioritize.
- **Be opinionated about traits.** "Two traits is fine" is not
  useful. "Two traits is fine because X-side caller needs envelope
  errors and Y-side caller doesn't" is. If you can't justify the
  split, recommend merging.
- **Don't accept "it works" as defense for shape problems.** A
  crate can compile and pass tests while still being a junk
  drawer.
- **Distinguish observation from recommendation.** Observations go
  in bucket 3; recommendations go in 1 or 2. Don't pad bucket 1.
- **Cite specific lines.** `magnetic/mod.rs:56` not "somewhere in
  the magnetic module."

## Sign-off

The audit is complete when:

1. Every top-level module has been read end-to-end and represented
   in the module-shape table.
2. The trait-family deep-dive is opinionated (keep / merge /
   refactor) with reasons.
3. The error-type deep-dive is opinionated.
4. The determinism contract has been spot-checked on at least four
   hot-path functions across different modules.
5. Each bucket-1 cleanup has a file:line citation and an effort
   estimate.
6. Each bucket-2 smell has a suggested direction (not just a
   complaint) and an effort estimate.
7. The findings doc has a clear sign-off recommendation: ready for
   next phase / needs streamline pass first / unclear.

Project owner reads the findings and decides whether to act on
recommendations now (likely a small commit batch for bucket-1) or
defer to a future phase.
