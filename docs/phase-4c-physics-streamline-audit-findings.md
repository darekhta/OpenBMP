# openbmp-physics post-consolidation streamline audit findings

Auditor: OpenAI Codex
Date: 2026-05-01
Base HEAD: `0c6c56a`
Working tree: intentionally dirty with this audit's targeted cleanups and findings document
Toolchain: `rustc 1.95.0 (59807616e 2026-04-14)`, `cargo 1.95.0 (f2d3ce0bd 2026-03-21)`

## Executive summary

`openbmp-physics` is not a junk drawer after the frame/WGS84 consolidation. The crate has a clear foundation role: shared HAL-portable physics formulas, constants, typed-frame context, and deterministic model traits consumed by both simulator-side and FC-side crates. The post-consolidation import graph is shallow: `atmosphere` depends only on `PhysicsError`, `gravity` depends on WGS84 symbols from `frames`, `magnetic` depends on `frames` plus `earth` for the toy dipole, and `wind` depends on `FrameContext`.

The main architectural smell is not broad module sprawl; it is trait-family asymmetry. `MagneticModel` is the rich/error-returning NED trait, while `MagneticFieldEci` is the total FC hot-path ECI trait. That split is defensible, but the rich magnetic trait does not carry `FrameContext`, unlike `WindModel`, so WGS84 rotation is not expressible at the trait boundary. Today that is mostly contained because production consumers use `MagneticFieldEci`, but it should be resolved before the rich magnetic path becomes a simulator contract.

I applied the low-risk definite cleanups found during the audit:

- Corrected the physics README trait-signature summary for `AtmosphereModel` and `WindModel`.
- Corrected the README gust determinism tuple from a generic channel term to wind axis.
- Updated root and magnetic module docs to name both magnetic trait families.
- Removed stale `openbmp-env`/env-crate wording from atmosphere docs.
- Added a real summary line for `AtmosphereSample`.
- Removed an empty duplicate USSA76 section heading.
- Removed a no-op `FrameContext::toy_fixed_earth()` reference from WMM 2025.

## Module-shape table

Line counts are from `find crates/openbmp-physics/src -name '*.rs' | xargs wc -l | sort -n` after the targeted cleanups. Total source is 5,668 lines including the generated WMM coefficient table, or 4,919 lines excluding that table.

| Area | Files / lines | Public surface | Coupling verdict |
|---|---:|---|---|
| `lib.rs` / `earth` | 93 | crate docs, module exports, `earth::MEAN_RADIUS_M` | Coherent root. `earth` is intentionally tiny but should be rechecked before API freeze. |
| `error` | 34 | `PhysicsError` with `FrameError` conversion | Coherent. The prior audit premise that frame errors were not wrapped is stale. |
| `frames` | 731 | frame profiles, WGS84 constants, `FrameContext`, transforms | Large but cohesive. This is the WGS84 and typed-frame home. |
| `gravity` | 517 | `GravityModel`, constant/point-mass/J2 models, gravity constants | Coherent. Uses WGS84 frame constants instead of duplicating them. |
| `atmosphere` | 1,012 | `AtmosphereModel`, sample type, USSA76 constants/helpers, two models | Coherent. Public helper set is larger than the trait, but all are atmosphere-specific. |
| `kinematics` | 131 | quaternion and skew helpers | Coherent shared math primitives. No physics-model coupling. |
| `magnetic` | 1,602 | two magnetic traits, dipole, WMM 2025, data-pin surface | Coherent domain, but trait split needs an explicit contract decision. |
| `statistics` | 140 | inverse normal and chi-square approximations | Coherent small numerical helper module. |
| `validity` | 56 | `HalfOpenRange` | Small but currently unused outside its own tests. Candidate for hiding or real adoption. |
| `wind` | 1,352 | `WindModel`, no/constant/layered/gust wind | Coherent domain. `GustWind` statefulness is documented but depends on runner discipline. |

## Bucket 1: definite cleanups

Status legend: `done` means fixed in this audit; `recommend` means the cleanup is definite but changes API shape or requires a follow-on decision.

| Status | Cleanup | Evidence / rationale |
|---|---|---|
| done | README trait signatures | The README grouped `AtmosphereModel` and `WindModel` as `Position3<Eci> + SimTime`, but `AtmosphereModel` takes altitude and `WindModel` also takes `&FrameContext`. |
| done | README gust seed wording | `GustWind` calls `DeterministicRng::for_wind_component(scenario_seed, step, axis)`, not a generic channel-id contract. |
| done | Magnetic docs should name both traits | `lib.rs` and `magnetic/mod.rs` reexport and document both `MagneticModel` and `MagneticFieldEci`; the old wording made `MagneticModel` sound like the only trait surface. |
| done | Atmosphere docs still referenced the retired env crate | The module now describes the shared controller/simulator surface rather than the former `openbmp-env` layering. |
| done | `AtmosphereSample` needed a real summary sentence | `missing_docs` accepted the existing field prose, but rustdoc flow was poor. |
| done | Duplicate empty USSA76 heading | `us_standard_1976.rs` had an empty "defining constants" section immediately before the layer table. |
| done | WMM 2025 no-op `FrameContext` reference | `field_ned_nt` constructed and discarded a toy context only to support prose. The import and no-op call are gone. |
| recommend | Decide whether `validity::HalfOpenRange` is public API | `rg HalfOpenRange` finds only its definition, tests, and root reexport. Either use it in WMM/USSA76 validity checks or keep it private until there is an external consumer. |
| recommend | De-phase current module docs over time | Several modules still lead with historical phase labels. They are not incorrect, but current contract should remain first, history second. |

## Bucket 2: architectural smells

| Smell | Severity | Recommendation |
|---|---|---|
| `MagneticModel` lacks `FrameContext` while describing ECI input converted to geodetic NED | Medium | Before any simulator-side WMM consumer depends on `MagneticModel`, choose one contract: either add a context-bearing rich magnetic trait/method, or state that callers must pre-rotate into fixed Earth before calling it. |
| `MagneticModel` returns errors while `MagneticFieldEci` is total with fallback | Low/medium | Keep the split if it is intentional, but document fallback policy at the crate root and FC bridge. Totality is useful for FC hot paths; silent fallback is not suitable for simulator validation. |
| `earth::MEAN_RADIUS_M` is a top-level micro-namespace | Low | Either keep it explicitly as the toy-model radius namespace or move it under `magnetic::dipole` if no other toy model adopts it. Current FC tests consume it, so do not remove casually. |
| `GustWind` is stateful behind an immutable `WindModel` method | Low/medium | The design is documented and tested locally. The real contract is runner-side: `advance(step)` exactly once per base tick, then read during RK stages. Keep or add runner-level tests that pin this usage. |
| Public constants are domain-specific but numerous | Low | Current naming is acceptable: `WGS84_*` in `frames`, `USSA76_*` in `atmosphere`, WMM-specific constants in `magnetic::wmm2025`, and J2 in `gravity`. Avoid moving constants merely for symmetry. |

## Bucket 3: out of scope but noted

- Full physics validation beyond the pinned WMM, USSA76, and J2 reference data is out of scope for this shape audit.
- Scenario parser and CLI magnetic-selection behavior are out of scope except where they reveal physics trait consumers.
- Dependency-policy changes such as moving test-only dependencies are out of scope unless `cargo machete` or CI flags them.
- Cross-platform last-bit determinism for libm calls remains a project-wide posture issue. This audit only checked code shape and local Linux/Apple toolchain gates available in the workspace.
- Generated WMM coefficient provenance is covered by the existing data-pin tests and was not re-reviewed line by line.

## Trait-family deep dive

The main model traits are not uniform by accident:

- `GravityModel`: `Position3<Eci> + SimTime -> Result<Vector3<f64>, PhysicsError>`.
- `AtmosphereModel`: geometric altitude + `SimTime -> Result<AtmosphereSample, PhysicsError>`.
- `WindModel`: `Position3<Eci> + &FrameContext + SimTime -> Result<Velocity3<Ned>, PhysicsError>`.
- `MagneticModel`: `Position3<Eci> + SimTime -> Result<Vector3<f64>, PhysicsError>` in geodetic NED.
- `MagneticFieldEci`: `Vector3<f64> + SimTime -> Vector3<f64>`.

`MagneticFieldEci` exists for the FC estimator path and is intentionally total. `EarthDipoleField` only implements this simple trait. `Wmm2025` implements both, using the rich trait for fallible NED evaluation and the FC trait for total ECI-vector evaluation with dipole fallback.

The smell is that `WindModel` already learned that frame context belongs at a rich environment-model boundary, while `MagneticModel` did not. `Wmm2025::field_ned_nt` currently treats the input vector as fixed Earth for geodetic conversion. That is acceptable only if the caller has already chosen the toy fixed-earth interpretation or pre-rotated the vector. The trait documentation should become explicit before external consumers grow around it.

## Error-type deep dive

`PhysicsError` is now the shared runtime error for physics-model evaluation and includes:

- `OutOfEnvelope`
- `NonFinite`
- `InvalidParameter`
- transparent `Frame(#[from] FrameError)`

`FrameError` remains the direct error type for frame primitives in `frames.rs`, which is appropriate because those functions are frame operations, not model evaluations. A physics model that calls a fallible frame operation can now use `?` into `PhysicsError`. No extra cross-crate conversion patch is needed.

Recommendation: keep the rule simple in docs and review comments. Frame helper APIs return `FrameError`; model APIs return `PhysicsError`; model APIs wrap frame failures only when they perform frame work internally.

## Determinism contract verification

Code search found no direct `std::time`, `SystemTime`, `Instant`, `rand::thread_rng`, `rand::random`, or explicit `mul_add` use in `crates/openbmp-physics/src`. The only RNG in physics source is `DeterministicRng::for_wind_component` inside the `synthetic` `GustWind` model.

Determinism tests exist at several levels:

- Bit-stable frame transform checks in `frames.rs`.
- Locked-order J2 and gravity checks in `gravity.rs` plus regression tests.
- USSA76 data-pin and per-kilometre reference checks.
- WMM 2025 reference-row and data-pin checks.
- Layered wind interpolation bit checks.
- Gust wind replay/reset checks over fixed step sequences.

Residual risk: the crate documents no FMA and locked operand order, but there is no compiler-level global FMA guard in this audit. The current code avoids explicit `mul_add` and writes sensitive arithmetic in expanded form. The project should keep deterministic golden tests in CI rather than relying on prose.

## Commands executed

Audit discovery:

```text
git rev-parse --short HEAD
git status --short
rustc -Vv
cargo -V
find crates/openbmp-physics/src -name '*.rs' | xargs wc -l | sort -n
rg -n "pub (mod|use|trait|struct|enum|const|fn)|pub\\(crate\\)" crates/openbmp-physics/src --glob '*.rs'
rg -n "MagneticModel|MagneticFieldEci|GravityModel|AtmosphereModel|WindModel" crates --glob '*.rs'
rg -n "HalfOpenRange" crates/openbmp-physics crates/openbmp-fc crates/openbmp-cli crates/openbmp-sim --glob '*.rs'
rg -n "openbmp_physics::earth|earth::|MEAN_RADIUS_M" crates --glob '*.rs'
rg -n "std::time|SystemTime|Instant|rand::|thread_rng|random\\(|DeterministicRng|synthetic" crates/openbmp-physics/src --glob '*.rs'
rg -n "FrameError|PhysicsError|InvalidParameter|OutOfEnvelope|NonFinite" crates/openbmp-physics/src crates/openbmp-physics/tests --glob '*.rs'
```

Final verification:

```text
cargo fmt --all --check
cargo build -p openbmp-physics --all-features --locked
cargo build -p openbmp-physics --no-default-features --locked
cargo test -p openbmp-physics --all-features --locked
cargo test -p openbmp-physics --no-default-features --locked
cargo clippy -p openbmp-physics --all-features --all-targets --locked -- -D warnings
cargo clippy -p openbmp-physics --no-default-features --all-targets --locked -- -D warnings
cargo deny check
cargo machete
cargo build --workspace --all-features --locked
cargo clippy --workspace --all-features --all-targets --locked -- -D warnings
cargo test --workspace --all-features --locked
```

All commands exited 0. `cargo deny check` retained the pre-existing
warning output for unmatched license allowances and duplicate transitive
crate versions; it still reported `advisories ok, bans ok, licenses ok,
sources ok`. `cargo machete` reported no unused dependencies.

## Sign-off

Sign-off: conditionally passes as a coherent foundation crate after the definite cleanups above. The crate is not currently a junk drawer. The main follow-up is to resolve the rich magnetic trait's frame-context story before simulator-side WMM use becomes a stable public contract.
