# OpenBMP Phase 2 Plan

> **Status:** Authored after Phase 1 closure (commit `0a3a694`). The
> Phase-1 plan (`docs/phase-1-plan.md`, removed in commit `085fd31`)
> is the precedent for this document's shape.

## Goal

Phase 1 shipped a deterministic, bit-stable point-mass kernel that
runs a constant-gravity drop through a closed-form analytic-toy
scenario and produces byte-stable Parquet telemetry. **Phase 2 lifts
the kernel from that toy to a single-stage sounding rocket** flying
through a real-coefficient atmosphere with realistic gravity, a
tabulated aerodynamic deck, a synthetic solid motor, and synthetic
IMU / barometer / ideal-state sensors. The Phase-2 closing milestone
is one public-benchmark validation case (the Niskanen 2009 thesis
worked example) producing a trajectory that matches the published
apogee within the documented tolerance, with byte-stable replay
across reruns.

Phase 2 stays **single-stage, single-motor, no actuators, no
controller**. Multi-engine clusters, control effectors, tank slosh,
the `VehicleAssembly` tree, the event/phase timeline, the mission
state machine, autopilot/FDIR, and any adaptive integrator are all
**out of scope** — they belong to Phase 3 / Phase 4.

## Scope Summary

### What Phase 2 ships

- **6-DOF rigid-body integration** in `openbmp-sim`: a `SimState` impl
  for `RigidBodyState`, a `RigidBodyDerivative`, and an
  `Integrator<RigidBodyState>` based on the existing `Rk4FixedStep`
  with a post-step quaternion renormalisation in `project()`.
- **Environment models** in `openbmp-env`:
  - `ConstantGravity` (same behaviour as `openbmp-sim`'s Phase-1
    scaffold, implemented independently to preserve crate layering).
  - `J2Gravity` using the WGS84 `J2 = 1.082626683 × 10⁻³` zonal
    coefficient.
  - `UsStandard1976` atmosphere, in-house Rust port of NOAA-S/T
    76-1562, valid 0–86 km geopotential.
  - `NoWind` and `ConstantWind` only; layered / gust profiles deferred
    to Phase 3.
- **Minimal Earth-frame support** in `openbmp-core`:
  `wgs84-uniform-rotation` plus a scenario-declared local geodetic
  origin, enough for altitude, vertical launch initialisation, and NED
  wind. IERS/EOP/leap-second profiles remain deferred.
- **Aerodynamic deck** in `openbmp-aero`: schema-1 deck format
  (`(mach, alpha, beta) → (CN, CD, CM)`) with trilinear interpolation
  at locked axis order, fail-closed extrapolation. Ship one synthetic
  textbook deck for a finned cylinder.
- **Synthetic solid motor** in `openbmp-propulsion`: in-house
  RASP-shaped TOML thrust-curve format with piecewise-linear
  interpolation. Ship one synthetic motor for unit tests and one
  public-citation `.eng` import for the integration scenario.
- **Synthetic sensors** in `openbmp-sensors`: `IdealStateSensor`,
  `SyntheticImu` (IEEE 952 five-component Allan-variance noise),
  `SyntheticBarometer`. GNSS + magnetometer + star-tracker deferred to
  Phase 3.
- **Force / moment / mass composition** in `openbmp-vehicle`: a flat
  `Vehicle` trait holding ordered force, moment, and mass model lists;
  the kernel sums them in declared order. The flat trait is the
  trivial-case degenerate of the Phase-3 `VehicleAssembly` tree.
- **Scenario format extensions** in `openbmp-scenario`: model-name
  extensions for the new env / aero / propulsion / sensor models, plus
  scenario-side references to aero deck and motor files (with
  scenario-relative path resolution and content-hash recording).
- **CLI extensions**: `openbmp run` accepts the new scenario shapes;
  the determinism CI gate runs the sounding-rocket scenario alongside
  the analytic-toy drop.
- **Public-benchmark validation case**: Niskanen 2009 example rocket
  (geometry + Estes D12 motor) → published apogee ± documented
  tolerance, plus a cross-tool sanity check against the RocketPy
  Calisto example.

### What Phase 2 explicitly does NOT ship

| Deferred | Phase | Reason |
|---|---|---|
| `VehicleAssembly` tree (Bodies / Propulsion / Effectors / Tanks) | 3 | Single-stage Phase-2 vehicle has no need for it; flat `Vehicle` trait is the degenerate case |
| `EngineModel` + `EngineCluster` (multi-engine) | 3 | Single-motor sounding rockets only |
| `ControlEffector` (rate-limit / saturation / latency / fault) | 3 | No actuators in the Phase-2 model |
| `TankModel` + `MovingMassModel` (slosh) | 3 | No liquid propellant in the synthetic solid motor |
| `EventTrigger` / `MissionPhaseGraph` | 3 | Phase-2 stop conditions stay `EndTime` / `MaxSteps` from Phase 1 |
| `LayeredWind`, `GustWind` (academic profiles) | 3 | `NoWind` + `ConstantWind` is enough to exercise wind-relative airspeed |
| `Recovery / DragDevice / ParachuteDrag` | 3 | Sounding-rocket apogee ≠ deployment in Phase 2 |
| `SyntheticGnss`, `SyntheticMagnetometer`, `SyntheticStarTracker` | 3 | IMU + barometer + ideal cover the validation case |
| Mission state machine, estimator, autopilot, FDIR | 4 | Virtual flight controller is its own phase |
| `DOPRI5` / `DOPRI8` adaptive integrators | 5 | Bit-stable RK4 is sufficient for Phase-2 scope |
| Multi-body / staging events | 5 | Multi-`VehicleAssembly` simultaneous flight |
| Multi-rate scheduling (`RatePlan`) | 5 | Single-rate kernel is enough |
| HIL socket bridge | 5 | Off-by-default tooling |
| Hypersonic methods, real-gas thermo, aerothermal | 6 | Earth-atmosphere only; sub-Mach 5 |
| Real fielded-vehicle parameter sets | never | Hard project boundary; see `safety-boundaries.md` |

## Architectural Decisions Already Settled

The trait surfaces and config shapes Phase 2 implements were
specified during Phase 0. The Phase-2 implementation **must not
re-design these** — the architecture references below are the
contract.

| Surface | Reference |
|---|---|
| `AtmosphereModel`, `AtmosphereSample` | [`software-architecture.md § Atmosphere`](software-architecture.md#atmosphere) |
| `ConstantGravity`, `J2Gravity`, gravity vector in `EnvironmentSample` | [`software-architecture.md § Gravity`](software-architecture.md#gravity) |
| `EnvironmentSample` wind block | [`software-architecture.md § Wind and Magnetic Field`](software-architecture.md#wind-and-magnetic-field) |
| `AeroMethod` trait, `AeroDeck` schema-1 TOML format | [`software-architecture.md § Aerodynamics`](software-architecture.md#aerodynamics) and [`§ Deck Format (in-house TOML)`](software-architecture.md#deck-format-in-house-toml) |
| `Motor` trait + RASP-shaped motor TOML | [`software-architecture.md § Propulsion`](software-architecture.md#propulsion) |
| `SyntheticSensor` trait, IMU / barometer noise spec | [`software-architecture.md § Synthetic Sensors`](software-architecture.md#synthetic-sensors) |
| Flat `Vehicle` trait (force / moment / mass model lists) | [`software-architecture.md § Vehicle and Mass Models`](software-architecture.md#vehicle-and-mass-models) |
| Determinism contract additions for Phase 2 | [`software-architecture.md § Determinism Profile`](software-architecture.md#determinism-profile) |
| Validation labels + tolerance-table format | [`verification.md`](verification.md) |
| Provenance contract for shipped data files | [`data-provenance.md`](data-provenance.md) |

## Toolchain and Dependency Decisions

### Inherited from Phase 1

- Rust 1.95 pinned in `rust-toolchain.toml`.
- All workspace pins in `[workspace.dependencies]` (`nalgebra`, `uom`,
  `serde`, `toml`, `arrow`, `parquet`, `clap`, `miette`,
  `tracing-subscriber`, `proptest`, `insta`, `criterion`,
  `assert_cmd`, `insta-cmd`, `predicates`, `tempfile`, …).
- `clippy::pedantic + clippy::all` warn-by-default, `unwrap_used /
  expect_used / panic` warn, `float_cmp` deny, `missing_docs` deny.
- The MXCSR guard at kernel construction.
- The `-C target-feature=-fma` rustflag for `x86_64-unknown-linux-gnu`.
- The deterministic ChaCha8 RNG seeded by `(scenario_seed, step_index,
  channel_id)`.

### New external dependencies

**No new physics/model dependencies.** The atmosphere model, gravity
coefficients, aero deck loader, motor parser, and noise generators are
all in-house Rust against public sources. The decision is grounded in
the project's "in-house re-implementation from public coefficients"
policy ([`safety-boundaries.md § Public-Data Provenance`](safety-boundaries.md#public-data-provenance))
and in the Phase-2 research brief's confirmation that no audit-quality
Rust crate currently exists for USSA76 or for five-component
Allan-variance IMU noise generation.

**One infrastructure dependency is allowed:** add a pinned SHA-256
implementation (`sha2` from the RustCrypto family) for
content-hash/provenance checks. Hashing is replay infrastructure, not a
physics model; implementing SHA-256 in-house would add avoidable
cryptographic-maintenance risk.

Toolchain note: `rust-toolchain.toml` pins the Phase-2 working
toolchain to Rust 1.95; the workspace package `rust-version` remains
the lower MSRV and is checked separately.

### New internal seam

`openbmp-core::DeterministicRng` keyed by `(scenario_seed,
step_index, channel_id)` is sufficient for telemetry and per-step
faults. Phase-2 sensor noise needs an extra dimension so each
component (ARW vs. RRW vs. bias-instability) gets its own bit-stable
sub-stream that doesn't shift when a sibling component is added or
removed. **Sub-phase 2.7 extends the keying to `(scenario_seed,
step_index, sensor_id, component_id)`** while preserving the
existing `for_channel` API.

## Implementation Seams Locked Before Coding

The following questions are closed here so implementation does not
reopen the Phase-0 architecture by accident:

- **Fallible model evaluation.** Environment, force, moment, mass, aero,
  and propulsion model calls that can leave their validity envelope
  return typed errors at the kernel boundary. Scenario validation catches
  obvious configuration mistakes, but runtime out-of-envelope samples
  still halt fail-closed with a model id and step index. No model reports
  failure by clamping silently, returning `NaN`, or panicking.
- **Crate layering.** `openbmp-sim` remains an L1 kernel crate and does
  not depend on `openbmp-env`, `openbmp-aero`, `openbmp-propulsion`, or
  `openbmp-vehicle`. L2 crates implement or adapt the L1 model traits;
  the Phase-1 `ConstantGravityForce` stays in `openbmp-sim` until a
  higher-layer compatibility adapter replaces it without a reverse
  dependency.
- **Frames.** Phase 2 implements `wgs84-uniform-rotation` and a
  scenario-declared local origin. The Niskanen scenario declares a local
  geodetic origin, initial position is derived from WGS84 height, and
  body `+x` points local up at ignition. `toy-fixed-earth` remains only
  for analytic toys.
- **Reduced aero deck semantics.** Schema 1 is an axisymmetric
  sounding-rocket deck, not a full six-coefficient aircraft-style deck.
  `CN` is the normal-force magnitude in the wind/body longitudinal
  plane, `CD` opposes relative wind, `CM` is the restoring moment in the
  same plane, and roll moment is zero. A full `CX/CY/CZ/Cl/Cm/Cn` deck
  is a Phase-3 schema extension.
- **Sensor IDs and RNG domain separation.** Phase 2 adds a stable
  `SensorId` newtype derived from the canonical scenario sensor path,
  not from list order. `for_sensor_component` uses an explicit domain
  tag in the seed material so it cannot collide with `for_channel`.
- **IMU standards.** IEEE Std 952-2020 is the inertial-sensor noise
  terminology source for Phase 2. Allan-deviation tests validate slopes
  statistically; byte-stability is required within the reference
  `x86_64-unknown-linux-gnu` platform profile, not across libm
  implementations.
- **Rigid-body inertia.** The Euler equation includes the
  `-I_dot * omega` term when a mass model returns a non-zero inertia
  derivative. Phase-2 shipped motors may use fixed inertia if the
  provenance and scenario docs state that simplification explicitly.

## Sub-phase Plan

The 12 sub-phases below mirror the Phase-1 cadence: small focused
commits per sub-phase, each one closing a verifiable deliverable.
Effort estimates assume one engineer working effectively full-time on
the sub-phase; multiply by your own discount factor.

### 2.1 — 6-DOF rigid-body kernel

**Scope.** Extend the kernel from `PointMassState`-only to a kernel
that also integrates `RigidBodyState` (already defined in
`openbmp-state`). The integrator architecture is generic over `S:
SimState`, but the current kernel and model contexts are still
`PointMassState`-shaped. Sub-phase 2.1 therefore includes the API
generalisation work: define `RigidBodyDerivative`, implement `SimState`
for `RigidBodyState`, make derivative evaluation fallible, and provide
`Phase1Kernel` / `RigidBodyKernel` aliases over one generic kernel type.

**Decision: RK4 + post-step renormalisation, not RKMK4.** The
research brief shows that at dt = 0.01 s over 100 s of sounding-rocket
flight, RK4 + projection produces sub-1e-9 attitude error after
renorm and is essentially indistinguishable from RKMK4 at small
step sizes. RK4 + projection is also simpler to bit-lock (only
quaternion add / scale + one final `sqrt`; no `exp` Taylor
truncation depth to lock). RKMK4 stays as a Phase-3 swap-in; the
trait surface is the same.

**Tasks.**

- `crates/openbmp-sim/src/derivative.rs` — add `RigidBodyDerivative`:
  - `velocity_m_s_eci: Vector3<f64>`
  - `acceleration_m_s2_eci: Vector3<f64>`
  - `quaternion_rate: Vector4<f64>` (the four-component time
    derivative of the body-to-ECI unit quaternion, locked as
    `q_dot = 0.5 * q_body_to_eci ⊗ [0, omega_body]`)
  - `angular_acceleration_rad_s2_body: Vector3<f64>` (from
    `I⁻¹ (M − ω × Iω − I_dot ω)` Euler equation)
  - `mass_rate_kg_s: f64`, `inertia_rate_body: Matrix3<f64>` (zero
    for `ConstantMass` / Phase-2 `LinearBurnMass` extended to rigid
    body)
  - `rk4_weighted_sum` with locked left-to-right parenthesisation,
    one final divide-by-six, no FMA.
- `crates/openbmp-sim/src/integrator.rs` — `impl SimState for
  RigidBodyState`:
  - `Derivative = RigidBodyDerivative`.
  - `advance_by(h, deriv)` updates time, position, velocity,
    quaternion, angular velocity, mass / inertia (locked order).
  - `project()` renormalises the quaternion via division by its
    magnitude (single `sqrt` and four divisions; no rotation matrix
    construction).
- `crates/openbmp-sim/src/kernel.rs` — generalise `SimulationKernel`
  to `SimulationKernel<S: SimState, …>` and expose `Phase1Kernel` /
  `RigidBodyKernel` aliases. Keep the Phase-1 analytic-toy CLI path on
  the alias so external behaviour stays stable.
- `crates/openbmp-sim/src/integrator.rs` — allow the derivative closure
  to return a typed model-evaluation error; RK stages short-circuit
  fail-closed and the kernel records `(step_index, model_id)`.
- New `MomentModel` trait in `openbmp-sim/src/models.rs`:
  - `fn moment_n_m_body(&self, ctx: MomentContext<'_>) -> Result<Vector3<f64>, ModelEvalError>`.
- `ConstantMassRigid`, `LinearBurnMassRigid` mass models that return
  full `MassProperties` derivatives (mass, inertia, CG offset).
- Two analytic-toy scenarios under `scenarios/analytic-toy/`:
  - `torque-free-precession.toml` — Euler torque-free rigid body,
    closed-form precession solution.
  - `gravity-gradient-stable-attitude.toml` — for Phase-3 forward
    compatibility (Phase 2 may stub this).

**Tests.**

- Unit tests on `RigidBodyDerivative::rk4_weighted_sum` for locked
  reduction order and finiteness propagation.
- Property test (proptest): random valid `RigidBodyState` + zero
  moment + `EndTime(1 s)` → quaternion stays unit-norm within
  `quaternion_tolerance` after 1000 steps.
- Property test: torque-free rigid body conserves `Iω` magnitude
  within tolerance.
- Analytic-toy validation: torque-free precession period matches
  closed-form solution.
- Quaternion kinematics sign test: integrate a constant positive
  body-axis rate for one small step and compare against the expected
  finite rotation.
- Determinism: byte-stable replay across two reruns.

**Exit criteria.**

- `RigidBodyKernel` runs to completion on a 100-second torque-free
  scenario at dt = 0.01 s.
- Quaternion magnitude stays inside `1.0 ± 1e-12` after the
  `project()` step at every step.
- `cargo bench` measures rigid-body step at < 5× the point-mass
  step (sanity check).

**Effort.** Large (~2 weeks).

### 2.2 — Gravity models and minimal frame profile

**Scope.** Add the minimal WGS84 frame profile in `openbmp-core`, add
gravity models to the dedicated env crate where the architecture put
them, and add J2. Do not make `openbmp-sim` depend on `openbmp-env`;
the sim crate keeps its Phase-1 constant-gravity toy model until
higher-layer adapters replace it.

**Decision sources.**

- J2 closed-form formulation: Vallado 4th ed. §8.6 (alternatively
  Montenbruck & Gill, *Satellite Orbits*, §3.2).
- WGS84 J2 coefficient: NIMA TR 8350.2 → `J2 = 1.082626683 × 10⁻³`,
  `µ = 3.986004418 × 10¹⁴ m³/s²`, `R_e = 6378137.0 m`. NGA WGS84
  portal as the live citation source.

**Tasks.**

- `crates/openbmp-core/src/frames.rs`:
  - Add `FrameProfile::Wgs84UniformRotation`.
  - Add a scenario-local-origin struct with geodetic latitude,
    longitude, and WGS84 height.
  - Add deterministic ECI/ECEF position and velocity transforms using a
    fixed WGS84 Earth rotation rate; no EOP, leap seconds, or polar
    motion in Phase 2.
  - Add ECEF/NED helpers for the declared local origin.
- `crates/openbmp-env/src/gravity.rs` with traits and three impls:
  - `GravityModel::gravity_eci_m_s2(&self, position_eci: Position3<Eci>, t: SimTime) -> Vector3<f64>`.
  - `ConstantGravity { g_eci_m_s2: Vector3<f64> }` — replicates the
    Phase-1 scaffold behaviour without depending on
    `openbmp-sim::models`.
  - `PointMassGravity { mu_m3_s2: f64 }`.
  - `J2Gravity { mu, r_e, j2 }` with the WGS84 default constructor.
- Gravity force adapter lives above `openbmp-sim`:
  - `openbmp-env` owns `GravityModel`.
  - `openbmp-sim::ConstantGravityForce` stays byte-identical for the
    Phase-1 CLI runner and is not deprecated in Phase 2.
  - The Phase-2 vehicle/environment assembly path adapts a
    `GravityModel` sample into a force contribution without introducing
    an L1 -> L2 dependency.
- `data/gravity/wgs84-j2.toml` — single-line file pinning
  `J2 = 1.082626683e-3`, with `provenance.md` next to it citing NIMA
  TR 8350.2 and the NGA portal URL.

**Tests.**

- Unit: `J2Gravity` reduces to `PointMassGravity` when `j2 = 0`.
- Unit: `wgs84-uniform-rotation` round-trips ECI ↔ ECEF for position
  and velocity at deterministic times.
- Unit: the declared local origin maps WGS84 height 0 m to NED origin.
- Property: `gravity_eci_m_s2(r) = -gravity_eci_m_s2(-r)` for the
  point-mass case.
- Analytic-toy: pure `PointMassGravity` Kepler orbit conserves
  energy and angular momentum within tolerance over one period.
- Provenance check passes on the WGS84 coefficient file.

**Exit criteria.**

- All three gravity models compile and unit-test green.
- The Phase-1 `constant-acceleration-drop` scenario still runs
  byte-identical after the env gravity additions.

**Effort.** Medium (~1 week).

### 2.3 — US Standard Atmosphere 1976

**Scope.** In-house Rust port of NOAA-S/T 76-1562 / NASA-TM-X-74335,
covering geopotential altitude **0–86 km only** (the seven lower
atmospheric layers). Above 86 km is treated as exoatmospheric for
Phase 2 (ρ ≈ 0) only when a scenario explicitly selects an
exoatmospheric policy; the default model returns a typed `RangeError`
when queried above the documented ceiling.

**Decision sources.**

- Primary citation: NOAA-S/T 76-1562 / NASA-TM-X-74335, NTRS
  19770009539. Tables 4 and 5 (geopotential layers, base
  temperatures, lapse rates, base pressures).
- Cross-check reference: Jacob Williams' MIT-licensed Modern Fortran
  reference (informational only — we reimplement, do not import).

**Tasks.**

- `crates/openbmp-env/src/atmosphere/us_standard_1976.rs`:
  - Layer table as a `const [Layer; 7]` literal (base altitude,
    base temperature, lapse rate, base pressure).
  - `geopotential_from_geometric(z_geometric_m, r_e_m) -> z_geo_m`
    closed-form conversion (no iteration).
  - `sample(altitude_geometric_m) -> Result<AtmosphereSample,
    AtmosphereError>` with locked barometric integration order per
    layer.
  - `speed_of_sound = sqrt(γ R T / M)` with γ, R, M as `const f64`s
    matching NOAA-S/T 76-1562 §1.3.
- `crates/openbmp-env/src/atmosphere/isothermal.rs` — toy
  `IsothermalAtmosphere { density, pressure, temperature }` for
  unit-test fixtures; no provenance file needed.
- `data/atmosphere/us_standard_1976.toml` — derived layer table for
  cross-checking, with `provenance.md` citing NOAA-S/T 76-1562.
- Update `EnvironmentSample` in `openbmp-sim/src/models.rs` to
  carry the `AtmosphereSample` (today it's gravity-only).

**Tests.**

- Regression test: at every kilometre between 0 and 86 km, the
  computed temperature, pressure, and density agree with the
  published NOAA tables within 1e-6 relative.
- Layer-boundary continuity: `T(11 km - ε)` and `T(11 km + ε)` agree
  to within `1 ULP` (the standard defines piecewise-continuous T).
- Property: `density(z) > 0` for `z ∈ [0, 86 km]`.
- Range check: `sample(86_001 m)` returns `RangeError` by default, and
  the kernel maps that error to a fail-closed simulation halt.

**Exit criteria.**

- Regression test passes within 1e-6 relative error at every
  altitude between 0 and 86 km in 1 km increments.
- `provenance.md` is present and `openbmp check-provenance` passes.

**Effort.** Medium (~1 week).

### 2.4 — Wind models (toy)

**Scope.** `NoWind` and `ConstantWind` only. The full
layered / gust profiles described in
`software-architecture.md § Wind and Magnetic Field` are deferred to
Phase 3 alongside the multi-rate scheduler that makes time-varying
gusts cheap to evaluate.

**Tasks.**

- `crates/openbmp-env/src/wind.rs`:
  - `WindModel::wind_ned_m_s(&self, position_eci: Position3<Eci>, frame: &FrameContext, t: SimTime) -> Result<Velocity3<Ned>, WindError>`.
  - `NoWind`.
  - `ConstantWind { wind_ned_m_s: [f64; 3] }`.
- Extend `EnvironmentSample` to carry the wind sample.
  `ConstantWind` requires a scenario local origin when any consumer asks
  for NED-to-body or NED-to-ECI transforms; `NoWind` does not.

**Tests.**

- Unit: `NoWind` returns zero everywhere and at every time.
- Unit: `ConstantWind` returns the constant.
- Property test on the `ConstantWind` constructor's NaN rejection.

**Exit criteria.** Both models compile and unit-test green; the
`EnvironmentSample` now carries gravity + atmosphere + wind in a
locked order documented in the type's doc comment.

**Effort.** Small (~0.4 week).

### 2.5 — Aerodynamic deck (tabulated)

**Scope.** Schema-1 reduced sounding-rocket deck only:
`(mach, alpha_deg, beta_deg) → (CN, CD, CM)`. The coefficients are
axisymmetric reduced coefficients as locked in § Implementation Seams,
not a full six-force/six-moment table. Trilinear interpolation uses an
**explicit reduction order** matching the axis order. Effector axes and
full six-coefficient decks are deferred to Phase 3.

**Decision sources.**

- Format inspired by OpenRocket's CSV decks (Niskanen 2009 thesis
  §5) and RocketPy's GenericSurface but in OpenBMP-native TOML to
  match the existing scenario / motor parser conventions.
- Trilinear reduction order locked per Demmel & Nguyen 2020
  (*Algorithms for Efficient Reproducible Floating Point Summation*,
  ACM TOMS 46:3): `((1-a)·((1-b)·((1-c)·c000 + c·c001) + b·…) + a·…)`,
  FMA disabled.

**Tasks.**

- `crates/openbmp-aero/src/deck.rs`:
  - `AeroDeck::load_from_toml(path)` parser with
    `serde(deny_unknown_fields)`.
  - `AeroDeck::lookup(mach, alpha, beta) -> Result<AeroCoefficients,
    AeroError>` with locked-order trilinear interpolation; out-of-grid
    produces `AeroError::OutOfEnvelope` (fail-closed).
  - Optional `extrapolation = "clamp"` toggle in the deck file
    documented as opt-in only.
- `crates/openbmp-aero/src/method.rs`:
  - `AeroMethod` trait.
  - `DeckLookup { deck, reference_area_m2, reference_length_m }`.
  - `aero_force_moment_body(&self, ctx) ->
    Result<AeroForceMomentBody, AeroError>` where
    `AeroForceMomentBody` carries `{ force_n_body, moment_n_m_body }`.
- One synthetic textbook deck under
  `data/aero/synthetic-finned-cylinder.toml` with `provenance.md`
  citing Niskanen 2009 Barrowman build-up as the derivation
  methodology.

**Tests.**

- Unit: `lookup` at every grid corner returns the corresponding cell
  exactly.
- Unit: `lookup` at the centroid of a cube returns the average of
  the eight corners.
- Property test: `lookup` along the grid boundary equals the
  one-dimensional sub-grid lookup.
- Determinism test: `lookup` produces bit-identical f64 results
  across two reruns and across two `AeroDeck` clones.
- Provenance check passes on the synthetic deck.

**Exit criteria.**

- All three coefficient channels (CN, CD, CM) interpolate correctly.
- The `synthetic-finned-cylinder` deck produces sane values
  (`CD > 0` everywhere; `CN(α=0) ≈ 0`).

**Effort.** Medium (~1 week).

### 2.6 — Synthetic solid motor

**Scope.** Solid motor with thrust-curve interpolation. The motor
TOML format follows the architecture's
[§ Motor Format (in-house TOML)](software-architecture.md#motor-format-in-house-toml)
spec verbatim — RASP-shaped, in-house re-implementation. Liquid /
hybrid / cold-gas variants deferred to Phase 3.

**Decision sources.**

- Format reference: ThrustCurve.org RASP `.eng` format.
- Public-citation reference motor: an Estes D12 thrust curve from
  ThrustCurve.org's public corpus (provenance file pins source URL,
  retrieval date, and SHA-256).
- Interpolation: piecewise-linear in `(t, F)` per OpenRocket / RASP
  convention.

**Tasks.**

- `crates/openbmp-propulsion/src/motor.rs`:
  - `Motor` trait per architecture spec.
  - `MotorVariant::Solid` only in Phase 2.
  - `SolidMotor { meta, burn, thrust_curve, geometry }` with
    piecewise-linear `thrust_n_at(t_since_ignition_s) -> f64`.
  - `MassModel` impl: `mass_kg(t)` and `mass_rate_kg_s(t)` from the
    declared `total_impulse_n_s` and `propellant_mass_kg`.
  - `ForceModel` impl: thrust along body `+x` (Phase-2 convention;
    documented in the deck).
  - If no inertia table is provided, the motor contributes mass loss
    only and declares fixed inertia as a Phase-2 simplification in its
    provenance record.
- `crates/openbmp-propulsion/src/parser.rs`:
  - `Motor::load_from_toml(path)` with `serde(deny_unknown_fields)`.
- `data/motors/synthetic-solid-textbook.toml` for unit tests.
- `data/motors/estes-d12.eng-derived.toml` for the integration
  scenario, with `provenance.md` citing ThrustCurve.org URL +
  retrieval date + SHA-256 of the source `.eng`.

**Tests.**

- Unit: `thrust_n_at(0)` returns 0; `thrust_n_at(burn_duration)`
  returns 0; `thrust_n_at(midpoint)` returns the interpolated value.
- Unit: integrating `thrust_n_at(t) dt` from 0 to `burn_duration`
  recovers the declared `total_impulse_n_s` within 1e-6 relative.
- Unit: `mass_rate_kg_s` is non-positive everywhere.
- Provenance check passes on both data files.

**Exit criteria.**

- Both motor files load, interpolate correctly, and pass mass-flow
  conservation checks.

**Effort.** Small-medium (~0.7 week).

### 2.7 — Synthetic sensors (IdealStateSensor, IMU, Barometer)

**Scope.** Three sensors. GNSS / magnetometer / star-tracker deferred
to Phase 3. The IMU's noise model is the headline complexity here —
it's the first place the determinism contract hits a noisy signal,
so the RNG seeding scheme has to be locked carefully.

**Decision sources.**

- Inertial-sensor noise terminology and Allan-variance decomposition:
  IEEE Std 952-2020.
- Noise-budget reference values: El-Sheimy / Hou / Niu 2008 IEEE
  T.I.M. (open-access via ResearchGate / U. Calgary).
- Counter-based deterministic Gaussian generator: Box-Muller (NOT
  Ziggurat — Ziggurat's rejection step makes per-step RNG draw count
  data-dependent and breaks bit-stable replay across small initial
  perturbations). Reference: Salmon et al. SC11 *Random123*.

**Tasks.**

- `crates/openbmp-core/src/rng.rs` — extend `DeterministicRng` with a
  new constructor:
  - `DeterministicRng::for_sensor_component(scenario_seed: u64,
    step: StepIndex, sensor_id: SensorId, component_id: u32)`.
  - The existing `for_channel(scenario_seed, step, channel_id)`
    stays unchanged; the new constructor is additive.
- `crates/openbmp-core/src/ids.rs` — add `SensorId(u64)`, derived from
  a stable canonical scenario path such as `sensors.imu`, never from
  insertion order.
- `crates/openbmp-sensors/src/sensor.rs`:
  - `SyntheticSensor` trait per architecture.
  - `IdealStateSensor` returns full `RigidBodyState` truth.
  - `SyntheticBarometer` with additive Gaussian noise + bias drift.
  - `SyntheticImu` with the IEEE 952 five-component model:
    quantization, ARW/VRW, bias instability (Ornstein-Uhlenbeck),
    RRW/ARW (random-walk integrator), scale-factor.
- `crates/openbmp-sensors/src/noise.rs`:
  - `BoxMullerGaussian` struct that draws two `f64` from a
    `DeterministicRng` and returns one Gaussian sample. Locked
    operation order. No transcendental beyond `ln`, `sqrt`,
    `cos`/`sin`; FMA disabled. Exact bytes are guaranteed within the
    reference platform profile; cross-libm bit equality is not claimed.
  - `OrnsteinUhlenbeck` for bias instability.
  - `IntegratedWhiteNoise` for RRW/ARW.
- Two reference noise budgets cited from El-Sheimy 2008 and Hou
  2004:
  - `noise-budgets/imu-tactical.toml`
  - `noise-budgets/imu-consumer-mems.toml`
  - Both with `provenance.md` citing the IEEE T.I.M. paper and the
    U. Calgary thesis. Documented as **academic example budgets**;
    not lifted from any vendor data sheet.

**Tests.**

- Unit: `IdealStateSensor` round-trips state to measurement bit-equal.
- Unit: `BoxMullerGaussian` matches the closed-form mean and variance
  to 4 decimal places over 10⁵ samples.
- Determinism: each sensor's measurement stream is bit-stable across
  two kernel reruns of the same scenario.
- Determinism: per-component noise streams (ARW vs. RRW vs. bias
  instability) don't shift if a sibling component is added or removed
  — i.e., adding a new noise component to the IMU model does not
  change the bytes of an existing run.
- Determinism: reordering `[sensors]` entries does not change any
  sensor's stream because `SensorId` is path-derived.
- Allan-variance regression: a long synthetic run of pure ARW noise
  produces an Allan-deviation log–log slope of −½ over the expected
  averaging-time decade.

**Exit criteria.**

- Three sensors compile, deterministic-replay correctly, and the
  Allan-variance regression matches the IEEE 952 slope.

**Effort.** Medium-large (~1.6 weeks).

### 2.8 — Force / moment / mass composition

**Scope.** Wire multiple force / moment / mass models into one
`Vehicle`. The kernel sums their outputs in **declared order** every
step. The `Vehicle` trait is the trivial-case degenerate of the
Phase-3 `VehicleAssembly` tree — designed so Phase 3 can subsume it
without breaking changes.

**Tasks.**

- `crates/openbmp-vehicle/src/vehicle.rs`:
  - `Vehicle` trait per architecture
    [§ Vehicle and Mass Models](software-architecture.md#vehicle-and-mass-models).
  - `BasicVehicle { force_models, moment_models, mass_model }` —
    ordered `Vec<Box<dyn _Model>>` lists.
- `crates/openbmp-sim/src/kernel.rs`:
  - Step pseudocode evaluates the fallible force / moment / mass lists
    in declared order, sums successful outputs in declared order, no FMA,
    no parallelism.
  - The first model error stops the step before any state mutation and
    reports `(step_index, model_id, error)`.
  - Telemetry channel for the per-model contribution
    (`force.<name>.x`, `force.<name>.y`, `force.<name>.z`) so a
    consumer can debug "which force is dominating".

**Tests.**

- Unit: `BasicVehicle::force_eci(...)` with two force models returns
  the ordered sum.
- Unit: the first failing force model short-circuits later models and
  leaves the state unchanged.
- Property: reordering the force list changes the byte output (the
  determinism contract is *order matters*; verify it does).
- Determinism: byte-stable replay of a multi-force scenario.

**Exit criteria.**

- A scenario with `forces = ["gravity", "aero", "thrust"]` runs and
  produces telemetry with per-model breakdowns.

**Effort.** Small-medium (~0.7 week).

### 2.9 — Sounding-rocket validation case

**Scope.** The headline Phase-2 outcome. Run the Niskanen 2009 thesis
example rocket (geometry + Estes D12 motor) through the assembled
stack, compare the predicted apogee against the published value, and
lock the result in a tolerance table. This sub-phase is executed after
the minimal scenario-format work in 2.10, even though the document keeps
the validation case first as the user-facing milestone.

**Decision sources.**

- Primary reference: Niskanen 2009 thesis §6 worked example
  (Helsinki UT / Aalto, CC-BY-NC-ND).
- Secondary cross-tool check: RocketPy "Calisto" example (Cesaroni
  Pro75 M1670). Run the same scenario in OpenBMP and report apogee /
  max-Q / max-Mach side-by-side.
- Sanity-envelope reference: NASA Sounding Rockets User Handbook
  NASA/TP-20230006855 (2023) for Terrier-class apogee envelopes —
  used as a sanity range, NOT as a reference trajectory.

**Tasks.**

- `scenarios/public-benchmark/niskanen-2009-example.toml`:
  - Vehicle geometry from thesis §6 (length, diameter, fin area).
  - Aero deck: synthetic Barrowman build-up matching the thesis
    parameters, in `data/aero/niskanen-2009-example.toml`.
  - Motor: Estes D12 from Phase-2.6.
  - Atmosphere: USSA76.
  - Gravity: J2.
  - Wind: NoWind.
  - Frames: `wgs84-uniform-rotation`, local geodetic origin declared in
    `[frames.local_origin]`.
  - Initial state: WGS84 height 0 m, body `+x` aligned with local up.
  - Stop: `EndTime(60 s)` (covers ascent + apogee).
- `crates/openbmp-cli/tests/expected/niskanen-2009-example.toml` —
  tolerance table:
  - `apogee_altitude_m` — Niskanen's published value ± 5%.
  - `apogee_time_s` — Niskanen's published value ± 5%.
  - `max_velocity_m_s` — sanity envelope.
  - `max_acceleration_m_s2` — sanity envelope.
- Integration test in `crates/openbmp-cli/tests/sounding_rocket.rs`:
  - Run the scenario.
  - Compute apogee / max-Q / max-Mach / max-acceleration from the
    Parquet output.
  - Assert against the tolerance table.
  - Assert byte-stability of two reruns.
- Documentation: `docs/verification.md` gains a "sounding-rocket
  reference case" section pointing at Niskanen 2009 with provenance.

**Tests.**

- The integration test above is the test.

**Exit criteria.**

- The integration test passes on the dev machine.
- The CI determinism gate runs the sounding-rocket scenario
  alongside the analytic-toy drop and asserts byte stability of
  both.

**Effort.** Medium-large (~1.5 weeks; the long pole is wiring the
deck and getting tolerances honest).

### 2.10 — Scenario format extensions

**Scope.** The Phase-1 scenario schema covered the analytic-toy drop
only. Phase 2 adds enough to express a sounding-rocket scenario:
new model-name registry entries, new top-level blocks for aero deck
and motor references, and the relative-path resolution + content-hash
recording for those references.

**Tasks.**

- `crates/openbmp-scenario/src/registry.rs` — extend the Phase-1
  registry:
  - `gravity = "constant" | "point_mass" | "j2"`.
  - `atmosphere = "none" | "isothermal" | "us_standard_1976"`.
  - `wind = "none" | "constant"`.
  - `forces = ["gravity", "aero", "thrust"]` permutations.
  - `vehicle.kind = "point_mass" | "rigid_body"`.
  - `sensors.<name>.kind = "ideal_state" | "imu" | "barometer"`.
- `crates/openbmp-scenario/src/document.rs` — extensions:
  - `[aero]` block: `deck = "path/to/deck.toml"`.
  - `[propulsion.motor]` block: `file = "path/to/motor.toml"`,
    `ignite_at_s = 0.0`.
  - `[wind]` block: `kind`, `wind_ned_m_s`.
  - `[atmosphere]` block: `kind = "us_standard_1976"`.
  - `[frames]` block: `profile = "wgs84-uniform-rotation"`.
  - `[frames.local_origin]` block: `latitude_deg`, `longitude_deg`,
    `height_m`, `source`.
- Hash recording: every external file referenced from the scenario
  has its SHA-256 computed at load time and recorded in the run's
  telemetry header for replay verification.
- `docs/scenario-format.md` updated to document every new block,
  with worked examples.

**Tests.**

- Unit: scenario referencing a missing aero-deck file fails closed
  with `ScenarioError::ReferencedFileMissing`.
- Unit: scenario referencing an aero deck whose SHA-256 doesn't
  match a pinned hash fails closed.
- Snapshot: `openbmp check` on the Niskanen scenario produces a
  stable structured report.

**Exit criteria.**

- Niskanen scenario parses, validates, and resolves all external
  file references.

**Effort.** Medium (~1 week).

### 2.11 — CLI / e2e extensions

**Scope.** Make `openbmp run` work end-to-end on the new scenario
shape, extend the determinism CI gate to cover it, and update
snapshot baselines.

**Tasks.**

- `crates/openbmp-cli/src/runner.rs` — generalise from the Phase-1
  `Phase1Kernel = SimulationKernel<…, point-mass, …>` to a
  `Phase2Kernel = SimulationKernel<…, rigid-body, …>` selector
  driven by `vehicle.kind`. The Phase-1 `point_mass` path stays
  intact for backward compatibility.
- `crates/openbmp-cli/src/commands/run.rs` — telemetry channels
  extended to include attitude (quaternion + Euler angles), angular
  velocity, atmosphere sample, sensor measurements.
- `.github/workflows/ci.yml` — extend the determinism job to also
  run the sounding-rocket scenario twice and diff on
  `x86_64-unknown-linux-gnu`. Cross-platform matrix jobs run the same
  scenario as state-stable smoke tests, not byte-equality gates.
- Snapshot baselines refreshed for `--help` (no change expected) and
  `openbmp check` on the new scenario.

**Tests.**

- The Phase-1.8 e2e test still passes (regression guard).
- The new Phase-2.9 sounding-rocket integration test still passes.
- `cargo test --workspace --all-features` green.

**Exit criteria.**

- The full workspace test suite is green.
- The Phase-1 analytic-toy scenario still produces byte-identical
  Parquet to the Phase-1 baseline (regression guard for the env gravity
  additions and higher-layer adapters).

**Effort.** Small-medium (~0.7 week).

### 2.12 — Phase 2 closure

**Scope.** Documentation cleanup + Phase-2 plan removal, mirroring
the Phase-1 closure pattern.

**Tasks.**

- `docs/design-concept.md § Phase Roadmap` — Phase 2 in past tense
  with the actual deliverables.
- `docs/README.md` — drop the link to `phase-2-plan.md`.
- `crates/*/README.md` — refresh status lines for the five crates
  that moved from "Phase 2 stub" to "Phase 2 implemented".
- `crates/openbmp-sim/README.md` — note the rigid-body extension and
  the continued Phase-1 `ConstantGravityForce` compatibility path.
- `git rm docs/phase-2-plan.md` — the per-sub-phase commit history is
  the source of truth.

**Tests.** N/A — docs only.

**Exit criteria.** Working tree clean; full suite still green.

**Effort.** Small (~0.3 week).

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| Fallible model evaluation spills wider than expected through integrator / kernel generics | Medium | High | Lock `ModelEvalError` in 2.1, make RK stages short-circuit before mutation, and keep Phase-1 aliases byte-stable |
| Frame/local-origin implementation contaminates altitude, wind, and launch-vertical semantics | Medium | High | Implement `wgs84-uniform-rotation` + local origin before validation; add round-trip frame tests and scenario metadata assertions |
| RK4 + renormalisation accumulates attitude error fast enough that 2.9 fails its tolerance | Low | Medium | Pre-design RKMK4 swap-in point; keep `Integrator<RigidBodyState>` open to alternative impls |
| USSA76 transcription bug at a layer boundary | Medium | Medium | Per-km regression test against the published table; layer-boundary continuity test to ULP |
| Trilinear FMA bug breaks bit-stability across rustc upgrades | Low | High | Locked reduction order test that runs in CI; FMA disabled at workspace level |
| IMU Allan-variance generator's RNG keying breaks bit-stable replay when sensors are reordered | Medium | Medium | Per-component RNG sub-streams keyed by `(seed, step, sensor_id, component_id)`; explicit reordering test |
| Niskanen 2009 thesis lacks enough detail to build the aero deck cleanly | Medium | Medium | RocketPy Calisto as fallback validation case; cross-tool sanity check |
| SHA-256 provenance dependency is rejected by supply-chain review | Low | Medium | Treat `sha2` as the only non-physics dependency; if rejected, insert an in-house SHA-256 task before 2.10 and extend schedule |
| Phase-2 `Vehicle` trait shape clashes with Phase-3 `VehicleAssembly` tree | Low | Medium | Design Phase-2 trait as Phase-3 tree's degenerate case; auditor sign-off before 2.8 closes |
| Adding `openbmp-env::ConstantGravity` or higher-layer gravity adapters breaks the Phase-1 analytic-toy byte-stability | Low | Low | Regression test on the Phase-1 scenario in 2.11; keep `openbmp-sim::ConstantGravityForce` byte-identical |
| `EnvironmentSample` extension breaks Phase-1 telemetry byte-stability | Low | Medium | Phase-1 scenarios continue to use a backward-compatible "atmosphere = none" code path that produces identical bytes |
| Sounding-rocket scenario takes hours to run, blocking iteration | Low | Low | Profile early; the kernel hot path is RK4 + LUT lookups, well within ms/scenario at dt = 0.01 s |

## Acceptance Gate for Phase 2 Closure

Phase 2 closes when **all** of the following are true:

1. Every sub-phase exit criterion is met.
2. `cargo fmt --all -- --check` clean.
3. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
4. `cargo test --workspace --all-features` green (entire workspace
   test suite).
5. The Phase-1 `constant-acceleration-drop` analytic-toy scenario
   continues to produce byte-identical Parquet vs. the Phase-1
   baseline (regression guard for env gravity additions and adapters).
6. The Phase-2 Niskanen sounding-rocket scenario passes its
   tolerance table on the dev machine.
7. The CI determinism gate runs both scenarios twice and asserts
   byte-stability of both on `x86_64-unknown-linux-gnu`.
8. Every new dataset file (gravity coefficient, atmosphere table,
   aero deck, motor curve, IMU noise budget) has a sibling
   `provenance.md` and `openbmp check-provenance` passes.
9. Every scenario-referenced external file has its SHA-256 recorded in
   telemetry metadata, and a pinned-hash mismatch fails closed.
10. The Niskanen scenario records `wgs84-uniform-rotation`, local
    origin, and transform metadata in the telemetry header.
11. `docs/design-concept.md § Phase Roadmap` lists Phase 2 in past
   tense with the actual deliverables.
12. `docs/phase-2-plan.md` is removed and `docs/README.md` no longer
    links to it.

## Suggested Calendar

Order is the dependency-respecting implementation order (see
§ Sub-phase Plan). The scenario-format work in 2.10 runs before the
validation case in 2.9. Effort is the per-sub-phase estimate; total is
**~11–12 weeks of focused effort** with a ~30% slip buffer for audit
follow-ups, which puts a realistic Phase-2 closure at **3–4 calendar
months**.

| # | Sub-phase | Weeks |
|---|---|---|
| 2.1 | 6-DOF rigid-body kernel | 2.0 |
| 2.2 | Gravity + minimal frames | 1.0 |
| 2.3 | USSA76 atmosphere | 1.0 |
| 2.4 | Wind (toy) | 0.4 |
| 2.5 | Aero deck (tabulated) | 1.0 |
| 2.6 | Solid motor | 0.7 |
| 2.7 | Synthetic sensors | 1.6 |
| 2.8 | Force / moment / mass composition | 0.7 |
| 2.10 | Scenario format extensions | 1.0 |
| 2.9 | Sounding-rocket validation case | 1.5 |
| 2.11 | CLI / e2e extensions | 0.7 |
| 2.12 | Phase 2 closure | 0.3 |
| **Total** | | **11.9** |

## References

The Phase-2 implementation must cite these sources in the
appropriate `provenance.md` files and crate `README.md`s. URLs
captured here so they're easy to copy.

### Numerical methods

- Iserles, Munthe-Kaas, Nørsett, Zanna. *Lie-group methods*. Acta
  Numerica 9 (2000), 215–365.
  <http://www.damtp.cam.ac.uk/user/na/NA_papers/NA2000_03.pdf>
- Hairer, Lubich, Wanner. *Geometric Numerical Integration*. 2nd ed.,
  Springer (2006).
  <https://link.springer.com/book/10.1007/3-540-30666-8>
- Andrle, Crassidis. *Applied Runge-Kutta-Munthe-Kaas Integration
  for the Quaternion Kinematics*. JGCD (2019).
  <https://arc.aiaa.org/doi/10.2514/1.G004578>
- Markley. *Attitude Estimation or Quaternion Estimation?*. NASA
  NTRS 20030093641.
  <https://ntrs.nasa.gov/api/citations/20030093641/downloads/20030093641.pdf>
- Demmel, Nguyen. *Algorithms for Efficient Reproducible Floating
  Point Summation*. ACM TOMS 46:3 (2020).
  <https://dl.acm.org/doi/10.1145/3389360>
- Salmon, Moraes, Dror, Shaw. *Parallel Random Numbers: As Easy as 1,
  2, 3*. SC11 (Random123).
  <https://www.thesalmons.org/john/random123/>

### Atmosphere and gravity

- *U.S. Standard Atmosphere, 1976*. NOAA-S/T 76-1562 / NASA-TM-X-74335.
  <https://ntrs.nasa.gov/citations/19770009539>
  <https://www.ngdc.noaa.gov/stp/space-weather/online-publications/miscellaneous/us-standard-atmosphere-1976/us-standard-atmosphere_st76-1562_noaa.pdf>
- *Standard Atmosphere Above 86-km Altitude* (Phase-3 reference,
  cited for completeness). NTRS 19770003812.
  <https://ntrs.nasa.gov/api/citations/19770003812/downloads/19770003812.pdf>
- NIMA TR 8350.2, *Department of Defense World Geodetic System 1984*,
  3rd ed. (2000).
  <https://gis-lab.info/docs/nima-tr8350.2-wgs84fin.pdf>
- NGA WGS84 portal.
  <https://earth-info.nga.mil/index.php?dir=wgs84&action=wgs84>
- Vallado. *Fundamentals of Astrodynamics and Applications*. 4th ed.

### Sensors

- IEEE Std 952-2020. *Single-Axis Interferometric Fiber Optic Gyros*.
  <https://standards.ieee.org/ieee/952/7496/>
- El-Sheimy, Hou, Niu. *Analysis and Modeling of Inertial Sensors
  Using Allan Variance*. IEEE T.I.M. 57:1 (2008).
  <https://www.researchgate.net/publication/3094132>
- Hou. *Modeling Inertial Sensor Errors Using Allan Variance*. U.
  Calgary MSc thesis (2004).
  <https://www.ucalgary.ca/engo_webdocs/NES/04.20201.HaiyingHou.pdf>

### Aero and propulsion

- Niskanen. *Development of an Open Source Model Rocket Simulation
  Software*. Helsinki UT / Aalto MSc thesis (2009).
  <https://openrocket.sourceforge.net/thesis.pdf>
  <https://aaltodoc.aalto.fi/items/c167b2b3-e51d-4410-aefe-0fc6690a883a>
- OpenRocket Advanced Flight Simulation 23.09 documentation.
  <https://openrocket.readthedocs.io/en/latest/user_guide/advanced_flight_simulation.html>
- RocketPy GenericSurface documentation.
  <https://docs.rocketpy.org/en/latest/user/rocket/generic_surface.html>
- ThrustCurve.org RASP File Format.
  <https://www.thrustcurve.org/info/raspformat.html>
- NASA Sounding Rockets User Handbook NASA/TP-20230006855 (2023).
  <https://sites.wff.nasa.gov/code810/files/SRHB.pdf>

### Project standards

- NASA-STD-7009B (March 2024). M&S credibility framework. See
  [`verification.md § External V&V Reference Frames`](verification.md#external-vv-reference-frames).
- AIAA G-077-1998. CFD V&V terminology. Same reference.
- RustCrypto `sha2` crate documentation and repository, used only for
  provenance/content hashing infrastructure.
  <https://github.com/RustCrypto/hashes>
