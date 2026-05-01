# Plan — consolidate physics into `openbmp-physics`, retire `openbmp-env`

> **Status.** Architectural prerequisite that lands **before** the
> Phase 4.C kernel↔FC bridge (workstream A in
> `docs/phase-4c-audit.md`). Closes the audit's
> thing-in-itself finding: `openbmp-env` currently locks ~2 000 lines
> of HAL-portable physics behind a sim-layer crate, forcing
> `openbmp-fc` to ship placeholder duplicates (degree-1 dipole vs
> the workspace's already-shipped WMM 2025; flat-Earth gravity vs
> the workspace's already-shipped J2; etc.). The split between
> `openbmp-physics` and `openbmp-env` was made under time pressure
> in the audit and only moved the smallest constants the FC was
> directly duplicating.

## Type / math layering rule (post-2026-05-01)

A follow-up scan surfaced finer-grained inline-physics in the FC:

- **Math primitives that the FC was reinventing inline** —
  dynamic-pressure formula (`q = ½ ρ v²`), rigid-body kinematics
  (quaternion construction from axis-angle / body rate, quaternion
  renormalisation, skew-symmetric cross-product matrix), and the
  chi-square inverse CDF used to derive innovation gates from a
  stated false-alarm rate. These all moved into `openbmp-physics`
  (`atmosphere::dynamic_pressure_pa`, `kinematics`, `statistics`)
  so any future consumer (sim-side property tests, FDIR threshold
  tuning, scenario validation gates) shares the same operand
  ordering and rational-approximation coefficients.

## Outstanding follow-up — frames + WGS84 constants

`openbmp-core::frames` currently owns `FrameContext`,
`LocalGeodeticOrigin`, the WGS84 constants (`WGS84_A_M`,
`WGS84_INV_FLATTENING`, `WGS84_FLATTENING`,
`WGS84_ECCENTRICITY_SQUARED`, `WGS84_MU_M3_S2`,
`WGS84_OMEGA_RAD_S`), and all the time-aware ECI ↔ ECEF and
ECEF ↔ NED transformations. Two of those constants (`WGS84_A_M`,
`WGS84_MU_M3_S2`) are also defined inline in
`openbmp-physics::gravity`; the values match bit-for-bit but are
maintained in two places.

These are physics primitives. The principled fix is to move
`FrameContext`, `LocalGeodeticOrigin`, and the WGS84 constants
into `openbmp-physics::frames`, and trim `openbmp-core::frames` to
the foundation types only (`Frame` trait, frame markers, value
types `Position3<F>` / `Velocity3<F>` / `Acceleration3<F>` with
their non-physics math methods, `Quaternion<From, To>` rotation
methods, `FrameProfile` enum, `FrameError`).

The consumer surface is small:
`openbmp-physics::magnetic::wmm2025`, `openbmp-physics::wind::*`,
`openbmp-cli::runner::wind`,
`openbmp-cli::runner::phase2_*`, plus `core` itself. No state,
sensors, mission, scenario, vehicle, fc, or telemetry consumer
uses `FrameContext` directly. This keeps the migration tractable
in a single workspace-wide commit, similar in shape to the
`openbmp-env` retirement but smaller. The work is tracked as a
follow-up in this document and is **not** yet executed; running it
is gated on user authorisation since the move is workspace-shape
in nature.

## Why retire `openbmp-env`

| Today | What's wrong |
|---|---|
| `openbmp-env` ships full WMM 2025 (594 lines), full 7-layer USSA76 (~915 lines combined), `J2Gravity` (512 lines), and the full wind family (1 345 lines). | All of it is pure math with HAL-portable dependencies (`openbmp-core` + `nalgebra`). None of it needs a sim crate. |
| `openbmp-fc` is forbidden from depending on `openbmp-env` (HAL portability). | The FC ships a degree-1 dipole placeholder, a flat-Earth gravity placeholder, and an inline closed-form pressure-altitude inverse — all reinventions of code 50 cm away in `openbmp-env`. |
| Phase 4.C plan defers "real WMM 2025" to Phase 5. | The defer is self-inflicted. The model already exists; it's just behind the wrong crate boundary. |
| `openbmp-physics` (147 lines) holds only the smallest shared constants. | The principle was "shared physics goes to physics" — but the principle was applied to constants, not to the formulas and stateful models that use them. |

The simplest fix: stop pretending `env` is structurally different. Move
its physics into `physics`. Move its sim-only adapters into `openbmp-sim`
and `openbmp-cli/runner/`. Delete the `env` crate.

## Target architecture

```
openbmp-physics (HAL-portable, expanded)
├── earth       — Earth geometric / WGS84 constants (re-exports from openbmp-core)
├── gravity     — GravityModel trait, ConstantGravity, PointMassGravity, J2Gravity (WGS84)
├── atmosphere  — AtmosphereModel trait, AtmosphereSample, ExoatmosphericPolicy,
│                  IsothermalAtmosphere, UsStandard1976 (full 7-layer)
├── magnetic    — MagneticModel trait, EarthDipoleField, Wmm2025 (full 12-degree)
├── wind        — WindModel trait, NoWind, ConstantWind, LayeredWind, GustWind
├── error       — PhysicsError (was EnvError), out-of-envelope and validity errors
└── validity    — ValidityRange struct used by every model

openbmp-sim (gains the kernel-side bridges)
└── env_bridge  — GravityForceAdapter, AtmosphereSampleCache, env-tick scheduling
                  (anything that adapts the simplified physics traits into the
                   kernel's force-evaluation pipeline)

openbmp-cli/src/runner/ (existing, gains the model-instantiator role)
├── wind.rs     — already lives here; consumes physics directly
└── fc_bridge   — already wired in Phase 4.C; consumes physics directly

openbmp-fc (decommissions parallel surface)
├── estimator   — consumes openbmp_physics::gravity directly; no FC GravityModel trait
├── magnetic    — DELETED (or kept as a one-line re-export of openbmp_physics::magnetic
│                  for ergonomics)
└── (rest unchanged)

openbmp-env  (deleted)
```

Crate dependency edges after the move:

```
openbmp-core ← openbmp-physics ← openbmp-fc, openbmp-sim, openbmp-aero,
                                  openbmp-vehicle, openbmp-cli
                                          ↑
                       openbmp-sim provides env_bridge for kernel-side code
```

## Architectural decisions (committed — execute, do not litigate)

### D-PC-1. Single trait per model class, simplified shape

The current env trait surface returns `Result<_, EnvError>`, takes
`Position3 + FrameContext + SimTime`, and surfaces a sim-shaped
error type. After the move, every physics model exposes one trait
with the shape:

```rust
pub trait GravityModel: Send + Sync + std::fmt::Debug {
    fn at_eci_m_s2(&self, position_eci_m: Vector3<f64>, time: SimTime) -> Vector3<f64>;
    fn validity_range(&self) -> ValidityRange;
}
```

**No `Result`.** Out-of-envelope inputs **clamp to the nearest
validity boundary** and return a finite vector. Callers that need
fail-noisy semantics check `validity_range().contains(position,
time)` themselves before calling. Reasoning: physics models on the
FC side cannot panic and cannot bail mid-tick; they must be
deterministic and total. Sim-side callers that surfaced
`EnvError::OutOfEnvelope` switch to an explicit pre-check.

**No `FrameContext`.** Models accept ECI position and `SimTime`.
Geodetic conversion (lat/lon/alt → ECI) happens **inside** the
model, using WGS84 constants from `openbmp-core`. Callers that
have non-ECI position do the conversion upstream.

**`SimTime` is fine.** It's a `f64`-backed value type in
`openbmp-core`, HAL-portable, no sim-side coupling.

Same shape for `AtmosphereModel`, `MagneticModel`, `WindModel`.

### D-PC-2. `ValidityRange` is the universal shape

```rust
pub struct ValidityRange {
    pub altitude_min_m: f64,
    pub altitude_max_m: f64,
    pub time_start: SimTime,
    pub time_end: SimTime,
}
impl ValidityRange {
    pub fn contains(&self, position_eci_m: Vector3<f64>, time: SimTime) -> bool { ... }
    pub fn unbounded() -> Self { ... }
}
```

`unbounded()` for time-independent and altitude-independent models
(`ConstantGravity`, `PointMassGravity`). `Wmm2025` returns
`time_start = 2025.0 decimal years`, `time_end = 2030.0 decimal
years`. `UsStandard1976` returns `altitude_max_m = 86 000`.

### D-PC-3. Bridges live in `openbmp-sim`, not `openbmp-physics`

The kernel-side adapter that turns a `physics::GravityModel` into
the kernel's `ForceModel` consumer (e.g. `GravityForceAdapter` in
`openbmp-sim::env_bridge`) **stays in `openbmp-sim`**. It is not
physics; it is sim-side glue.

The scenario-side instantiator that picks "constant" vs "j2" vs
"wmm_2025" from `[environment.gravity]` config **stays in
`openbmp-cli/src/runner/`**. It is not physics; it is scenario
glue.

Neither is moved to `openbmp-physics`.

### D-PC-4. FC parallel surface — delete

`openbmp-fc` has its own `GravityModel` trait
(`crates/openbmp-fc/src/estimator.rs`) and its own
`MagneticFieldModel` trait (`crates/openbmp-fc/src/magnetic.rs`)
plus their reference impls. After the move:

- The traits are **deleted**. FC consumes
  `openbmp_physics::gravity::GravityModel` and
  `openbmp_physics::magnetic::MagneticModel` directly.
- `openbmp-fc::estimator::ConstantGravityZ` is **deleted**. Replaced
  by `openbmp_physics::gravity::ConstantGravity` configured with
  the ECI `(0, 0, -STANDARD_GRAVITY_M_S2)` vector.
- `openbmp-fc::magnetic::EarthDipoleField` is **deleted**. The
  evaluator moves to `openbmp_physics::magnetic::EarthDipoleField`
  unchanged. FC consumes the physics version.
- The `openbmp-fc::magnetic` module either disappears or shrinks to
  a pub re-export. Decide at implementation time based on whether
  any FC-internal callers still need a stable path; my recommendation
  is **delete the module** and update FC consumers to import from
  `openbmp_physics::magnetic`.

After this, the FC's `Ekf::with_gravity_model<G>` becomes:

```rust
pub fn with_gravity_model<G>(mut self, gravity: G) -> Self
where
    G: openbmp_physics::gravity::GravityModel + 'static,
{
    self.gravity = Box::new(gravity);
    self
}
```

Same shape, different trait — caller change is mechanical.

### D-PC-5. The FC gets WMM 2025 today

Drop the old full-WMM deferral line from
`openbmp-fc/README.md`, `magnetic.rs` docs, and
`docs/phase-4c-plan.md`. The model exists; the FC consumes it.
The dipole stays as a simpler academic-tier alternative; both are
selectable via `mag_field` in `[fc.ekf]` or `[fc.mekf]`.

### D-PC-6. `EnvError` → `PhysicsError`, lives in `openbmp-physics`

The error variants in `openbmp-env::error::EnvError` are physics
concerns: `OutOfEnvelope`, `InvalidConfiguration`, etc. Rename to
`PhysicsError`, move to `openbmp-physics::error::PhysicsError`.
Sim-side wrappers in `openbmp-cli` map this into `CliError` as they
already do for env errors.

But: most call sites no longer return `Result` per D-PC-1. The
remaining `PhysicsError` users are construction-time validation
(e.g. `LayeredWind::new` failing on overlapping altitude bands) and
the explicit `validity_range().contains()` workflow. The new
`PhysicsError` is much smaller than the old `EnvError`.

### D-PC-7. Wind models move

All four (`NoWind`, `ConstantWind`, `LayeredWind`, `GustWind`) are
pure physics:
- `NoWind` returns zero. Trivial.
- `ConstantWind` returns a constant. Trivial.
- `LayeredWind` is altitude lookup. Pure formula.
- `GustWind` is seeded turbulence using `rand_chacha`, fully
  deterministic given a seed. Pure physics in the same way that
  any stochastic-but-seeded model is physics.

The `synthetic` feature flag on `openbmp-env` (which gates
`GustWind`) moves to `openbmp-physics` unchanged.

### D-PC-8. Coefficient tables stay where they live

`data/magnetic/WMM.COF` (and `provenance.md`) stay at their current
path. The generated `coefficients.rs` (which encodes the COF table
as a Rust `const`) moves with the WMM 2025 source from
`crates/openbmp-env/src/magnetic/wmm2025/` to
`crates/openbmp-physics/src/magnetic/wmm2025/`. No data is moved.

Same for `data/atmosphere/us_standard_1976.toml` — stays put;
provenance entries get re-targeted at `openbmp-physics`.

### D-PC-9. WGS84 / Earth constants are in interim dual residence

The old "WGS84 stays in `openbmp-core` and physics re-exports it"
plan is superseded by the follow-up at the top of this document.
`openbmp-core::frames` still owns the frame-conversion constants
for now, while `openbmp-physics::gravity` owns the gravity-model
copies of `WGS84_A_M` and `WGS84_MU_M3_S2`. Normal `openbmp-physics`
builds const-check the duplicate values against `openbmp-core` so
drift is caught before the full `FrameContext` + WGS84 move lands.

The final target remains `openbmp-physics::frames`: move
`FrameContext`, `LocalGeodeticOrigin`, the time-aware transforms,
and all WGS84 constants there, then leave only frame marker/value
types and non-physics arithmetic in `openbmp-core`.

### D-PC-10. Single workspace test pass per model class

Move every test from `openbmp-env/tests/` into `openbmp-physics/tests/`.
Notably: `wmm_data_pin.rs` (verifies the .COF SHA-256 pin),
`regression.rs` (model-output regression). No tests are dropped.

### D-PC-11. No new external dependencies

After the move, `openbmp-physics` will gain:
- `nalgebra` — already in workspace
- `rand_chacha` — already in workspace (used by `GustWind`)

No `cargo deny` impact. No license review needed.

### D-PC-12. Lockstep tripwire — extend coverage

`openbmp-testkit::fc_lints` already greps `openbmp-fc/` for wall-
clock APIs. **Add a parallel grep for `openbmp-physics/`** to
guarantee the consolidated crate stays HAL-portable. The bigger
the crate gets, the easier it is for someone to quietly slip a
`std::time::Instant::now()` into a benchmark helper.

### D-PC-13. The `openbmp-physics` crate becomes a workspace centerpiece

Post-move size: ~3 500 lines (current physics 147 + env's 3 500 −
trait bookkeeping consolidation). This is fine. `openbmp-core` is
similar size. `openbmp-physics` becomes the workspace's
"foundation physics" crate, paralleling `openbmp-core`'s
"foundation types" role.

### D-PC-14. Migration is one commit, not many

The migration touches every consumer crate. Splitting into many
commits leaves the workspace half-broken between commits, which
defeats `git bisect`. Land it as a single commit with a long
descriptive message (sub-headed by the workstreams below). The
prior author's commit `3474b2f` for Phase 4.A+B is the precedent.

### D-PC-15. Phase 4.C ordering — physics consolidation first

This work lands **before** the Phase 4.C kernel↔FC bridge
(workstream A). Reason: the bridge will use `physics::*` trait
types as its public API. If we ship the bridge first, then the
consolidation, the bridge has to migrate twice.

Update `docs/phase-4c-audit.md` to mark this consolidation as
"Phase 4.C P0 prerequisite" before workstream A.

## Migration steps (ordered)

### S1 — Inventory and freeze

1. `grep -rln "openbmp_env\|openbmp-env" --include='*.rs'
   --include='*.toml' --include='*.md'` → consumer list.
2. Catalogue every:
   - `pub use openbmp_env::*` re-export.
   - `use openbmp_env::*` direct import.
   - `openbmp-env =` Cargo.toml dependency.
   - Documentation reference (`docs/`, `README.md`).
3. Lock the inventory in a scratch file
   (`docs/scratch/env-consumers.md` — purgeable after the move).

### S2 — Extend `openbmp-physics`

1. Create the module skeleton:
   `crates/openbmp-physics/src/{gravity,atmosphere,magnetic,wind,error,validity}.rs`
   (or `.../mod.rs` for sub-trees).
2. Add traits per D-PC-1 / D-PC-2.
3. Add `PhysicsError` per D-PC-6.
4. Re-export from `crates/openbmp-physics/src/lib.rs`.

### S3 — Port models into `openbmp-physics`

For each model class (gravity, atmosphere, magnetic, wind):

1. Move the source files from
   `crates/openbmp-env/src/<class>/` to
   `crates/openbmp-physics/src/<class>/`.
2. Adapt to the simplified trait surface:
   - Replace `Result<_, EnvError>` returns with clamped finite values.
   - Replace `Position3 + FrameContext` parameters with
     `Vector3<f64> ECI`. Move geodetic conversion inside the model.
   - Add `validity_range()` impl returning the model's envelope.
3. Move associated tests. Update import paths.
4. Update `openbmp-physics::lib.rs` re-exports.
5. **Verify:** `cargo build -p openbmp-physics --all-features`
   clean; tests green.

Suggested order within S3:
- Wind first (smallest, simplest, fewest consumers).
- Atmosphere second (USSA76 + isothermal).
- Magnetic third (WMM 2025 has the largest test suite).
- Gravity last (J2 has the most call sites in `openbmp-sim`).

### S4 — Add the sim-side bridge

1. New module `crates/openbmp-sim/src/env_bridge/{mod,gravity,atmosphere,magnetic,wind}.rs`.
2. Move kernel-side adapters from where they currently live (some
   in `openbmp-sim` already, some in `openbmp-cli/runner/`) into
   `env_bridge`.
3. Adapter shape:
   ```rust
   pub struct GravityForceAdapter<G: openbmp_physics::gravity::GravityModel> { ... }
   impl<G: GravityModel> openbmp_models::ForceModel for GravityForceAdapter<G> { ... }
   ```
4. The adapter handles validity-range pre-checks and surfaces
   `CliError::Env` when a kernel-side caller wanted noisy errors.

### S5 — Migrate `openbmp-fc`

Per D-PC-4 / D-PC-5:

1. Delete `openbmp-fc::estimator::GravityModel` and
   `openbmp-fc::estimator::ConstantGravityZ`.
2. Delete `openbmp-fc::magnetic` (the entire module).
3. Update `Ekf` and `Mekf` to use
   `openbmp_physics::gravity::GravityModel` and
   `openbmp_physics::magnetic::MagneticModel`.
4. Update `EkfParams::default` to seed with
   `openbmp_physics::gravity::ConstantGravity` set to ECI z = −9.806 65.
5. Update `MekfParams::default` to seed with
   `openbmp_physics::magnetic::EarthDipoleField` (academic baseline)
   or `Wmm2025` (default for new scenarios).
6. Update `crates/openbmp-fc/src/lib.rs` module list.
7. Update `crates/openbmp-fc/README.md` module map; remove the old
   full-WMM deferral line.
8. Update FC tests that constructed the FC's now-deleted types.

### S6 — Migrate other consumers

For each crate in S1's inventory:

1. Replace `use openbmp_env::*` with `use openbmp_physics::*`.
2. Replace `openbmp-env =` with `openbmp-physics =` in `Cargo.toml`.
3. If the crate also uses `openbmp-sim::env_bridge::*`, add that
   dependency.
4. Verify.

Specific crates to migrate (from the inventory):
- `openbmp-cli` (heavy — runner glue, error mapping, scenario-side
  instantiation).
- `openbmp-vehicle` (uses atmosphere samples for sensor noise).
- `openbmp-aero` (atmosphere consumer).
- `openbmp-aerothermal` (atmosphere + wind consumer).
- `openbmp-propulsion` (atmosphere consumer).
- `openbmp-core::frames` — the inventory hit suggests there's a use
  of env types in core. **This is suspicious** (core should not
  depend on env). Verify in S1; resolve by either deleting the
  dependency or moving the consumer to a higher layer.
- `openbmp-testkit::tests::inline_data_tripwire` — likely a string
  match for "openbmp-env"; update the tripwire's expected-list.
- `openbmp-scenario` — verify it doesn't depend on env (likely
  doesn't; scenario only uses model NAMES via the registry).

### S7 — Retire `openbmp-env`

1. Verify no remaining workspace consumers reference `openbmp-env`:
   `cargo tree -p openbmp-env --invert` returns only the env crate
   itself.
2. Remove `openbmp-env =` from `Cargo.toml` workspace dependencies.
3. Remove `"crates/openbmp-env"` from `Cargo.toml` workspace members.
4. `git rm -r crates/openbmp-env/`.
5. Verify: `cargo build --workspace --all-features` clean.

### S8 — Documentation

1. Update `docs/design-concept.md`:
   - Phase 2 description: replace "openbmp-env" with "openbmp-physics".
   - Crate map: drop env, expand physics description.
2. Update `docs/software-architecture.md`:
   - Crate selection table: remove env row, expand physics row.
   - Workspace layout diagram.
3. Update `docs/phase-4-plan.md`:
   - Phase 4.C section: remove the old full-WMM deferral text — it's
     now a Phase 4.C deliverable via consolidation.
   - Phase 2/3 closure references that mention env: rewrite to
     reference physics.
4. Update `docs/phase-4c-plan.md`:
   - Mark this consolidation as the new P0 prerequisite.
   - Update P5 (Environment Models) — WMM 2025 is no longer a
     deferral; it's a Phase 4.C deliverable.
5. Update `docs/phase-4c-audit.md`:
   - Insert a new `D-PC-*` section referencing this plan.
   - Update workstream order: consolidation goes before workstream A.
6. Update `crates/openbmp-physics/README.md` (create if absent):
   - Module map.
   - Validity ranges per model.
   - Test data provenance.
7. Update `crates/openbmp-fc/README.md`:
   - Module map: drop `magnetic` module.
   - Drop the old full-WMM deferral line.
   - Add: "Estimator now consumes `openbmp_physics::gravity` and
     `::magnetic` directly."
8. Update data provenance:
   - `data/magnetic/provenance.md`: re-target the consumer list
     (was `openbmp-env`, now `openbmp-physics`).
   - `data/atmosphere/provenance.md`: same.
   - `data/gravity/wgs84-j2.toml`: same.

### S9 — Tripwire and CI

1. Extend `crates/openbmp-testkit/src/fc_lints.rs` to grep
   `crates/openbmp-physics/src/` for wall-clock APIs.
2. Update `inline_data_tripwire.rs` if it has expected-crate lists.
3. Verify CI workflow (`.github/workflows/ci.yml`) does not mention
   `openbmp-env` explicitly. If it does, replace with
   `openbmp-physics`.

### S10 — Verify

Acceptance gates per the Phase-4.B / 4.C pattern:
1. `cargo fmt --all -- --check` clean.
2. `cargo clippy --workspace --all-targets --all-features -- -D warnings` clean.
3. `cargo test --workspace --all-features` green.
4. `cargo test -p openbmp-fc --no-default-features` green.
5. `cargo build -p openbmp-fc --no-default-features` clean.
6. `cargo build -p openbmp-physics --no-default-features` clean
   (new gate; `openbmp-physics` must be HAL-portable).
7. `cargo deny check` clean.
8. `cargo machete` clean.
9. Phase-1 / Phase-2 / Phase-3 byte-stable scenarios still pass
   (regression — this is the load-bearing acceptance gate; if a
   scenario's Parquet output changes, the migration broke
   determinism).
10. The lockstep-clock tripwire tests pass for both `openbmp-fc/`,
    `openbmp-physics/`, and `openbmp-cli/src/runner/fc_bridge.rs`.
11. `cargo tree -p openbmp-env` returns "package not found"
    (positive confirmation env is gone).
12. `cargo tree -p openbmp-fc | grep openbmp-physics` shows the
    physics dependency is wired.

## Risks and mitigations

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Trait simplification (no `Result`) breaks a sim-side caller that depended on noisy errors. | Medium | Medium | Sim-side bridge in `openbmp-sim::env_bridge` re-introduces explicit pre-checks via `validity_range()`. Document the migration pattern in `env_bridge::README`. |
| Geodetic conversion inside a model produces different bit patterns than the previous "FrameContext-driven" path. | Medium | High | Determinism gate (S10.9) catches this. If a Phase-3 byte-stable scenario's Parquet changes, freeze the operand order around the old path's exact expression and ship a determinism comment explaining why. |
| Some `openbmp-core::frames` reference to env hides a circular dependency. | Low | High | S1 inventory + S6.6 explicit verification step. If circular, factor the offending util to a third location (probably `openbmp-physics::frames` or fold into `openbmp-core`). |
| Half-migrated workspace at intermediate commits. | High | Low | D-PC-14: ship as a single commit. |
| Test data `.COF` SHA-256 pin breaks because the consumer crate path changed in `provenance.md`. | Low | Low | S8.8 explicit re-targeting; the SHA-256 of the data file itself is invariant. |
| The FC's existing closed-loop test depends on `EarthDipoleField`'s exact dipole magnitude (30 000 nT). | Low | Low | The constant moves verbatim; the evaluator moves verbatim; no behavioral change. |
| Wind models' RNG seeding changes byte-stability when the type moves crates. | Low | High | The seed-to-state derivation is determined by the model code, not by the crate name. Move the source verbatim. |
| Two retirements at once (env crate + FC magnetic module + FC GravityModel trait) compounds risk. | Medium | Medium | The plan splits S3–S5 into independently verifiable steps. Each step's tree must build green before moving on. |

## Out of scope

- Adding new physics models (NRLMSISE-00 upper atmosphere, EGM2008
  spherical-harmonic gravity). Those remain Phase 5.
- Splitting the wind family further (e.g. extracting Gaussian
  noise). The current `GustWind` is a single deterministic model;
  splitting it is its own architectural conversation.
- Touching the trait surfaces of unrelated workspace crates
  (`openbmp-aero` deck families, `openbmp-propulsion` motor types,
  `openbmp-vehicle` assembly). They consume physics; they don't get
  refactored.
- Performance optimization. Locked operand order stays locked; no
  FMA, no SIMD, no vectorization. This migration is structural.
- Renaming `openbmp-physics` to something else
  (`openbmp-physics-models`, `openbmp-environment`). The name is
  fine.

## Definition of done

The consolidation is complete when **all** of S10's gates pass
**and**:

13. `crates/openbmp-env/` does not exist.
14. `openbmp-fc::magnetic` module does not exist (or is a
    one-line re-export — choose one path and document it in S5).
15. `openbmp-fc::estimator::GravityModel` trait does not exist.
16. `openbmp-fc::estimator::ConstantGravityZ` type does not exist.
17. `openbmp-fc::magnetic::EarthDipoleField` type does not exist
    (replaced by `openbmp_physics::magnetic::EarthDipoleField`).
18. `openbmp-physics` exports `gravity`, `atmosphere`, `magnetic`,
    `wind`, `error`, `validity`, and `earth` modules.
19. The old full-WMM deferral wording appears in zero current-status
    docs.
20. `docs/phase-4c-plan.md`'s P5 section is rewritten to reflect
    that WMM 2025 has landed.
21. The closed-loop scenario fixture at
    `scenarios/closed-loop-attitude-hold/scenario.toml` runs with
    `mag_field = "wmm_2025"` in `[fc.ekf]` / `[fc.mekf]` and passes its
    tolerance table. (This is the proof point that the migration
    actually delivered the promised capability.)

## Where this lands relative to Phase 4.C

This consolidation **replaces** the Phase 4.C plan's P5 (Environment
Models) section and **prepends** workstream A (kernel↔FC bridge).
Update both `docs/phase-4c-plan.md` and `docs/phase-4c-audit.md` to
reference this plan as the prerequisite.

After this lands, the Phase 4.C ordering becomes:

1. **Physics consolidation (this plan)** — landed.
2. **Workstream A** — kernel↔FC bridge using the new physics surface.
3. **Workstream E (1–9)** — FC-only test depth.
4. **Workstream B** — Estimator SOTA.
5. (rest unchanged from `docs/phase-4c-audit.md`)

## Anti-patterns to avoid during execution

- **Do not** ship a placeholder `openbmp-env` shell that pub-uses
  from `openbmp-physics`. The user explicitly said retire it.
  Half-retirement is worse than no retirement.
- **Do not** introduce a third crate (`openbmp-physics-models`,
  `openbmp-environment-bridge`). Two crates suffice: physics +
  sim-bridge in `openbmp-sim`.
- **Do not** keep the FC's `GravityModel` and `MagneticFieldModel`
  traits "for ergonomics." They duplicate physics; delete them.
- **Do not** preserve `Result<_, EnvError>` returns through the
  whole sim-side stack just because the sim used to expect them.
  Move the validity check to the sim-side bridge layer once.
- **Do not** move the `.COF` data file out of `data/magnetic/`.
  Data lives where data lives; only the consumer code moves.
- **Do not** carry over `// Phase 4.B`, `// Phase 4.C` markers
  from the old code. The new code is the new code; the prior
  phase's struggles aren't useful annotations.
