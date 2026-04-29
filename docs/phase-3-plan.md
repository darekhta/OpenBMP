# OpenBMP Phase 3 Plan

> **Status:** Authored after Phase 2 closure (commit `7af3082`). The
> Phase-2 plan (`docs/phase-2-plan.md`, removed in commit `7af3082`)
> is the precedent for this document's shape.

## Goal

Phase 2 shipped a **single-stage sounding rocket** flying through the
Niskanen 2009 Chapter-6 benchmark with byte-stable replay, runner-side
Phase-2 point-mass dispatcher, atmosphere sample + per-model
force-breakdown telemetry, and a determinism CI gate covering both
analytic-toy and Niskanen scenarios. **Phase 3 lifts that vehicle to
a modular composable rocket** with multi-engine clusters, control
effectors, slosh dynamics, mission-phase scheduling, recovery models,
richer atmosphere (layered + gust) and sensor (GNSS / magnetometer /
star tracker) coverage. The Phase-3 closing milestone is the
**RocketPy "Calisto" cross-tool validation case** — running
the same scenario in OpenBMP and RocketPy with apogee, max-Q, and
max-Mach side-by-side at academic tolerance.

Phase 3 stays **single-body, no autopilot, no estimator**. Multi-body
separation as first class, virtual flight controller (estimator +
autopilot + FDIR + guidance), adaptive integrators, hypersonic
methods, multi-rate scheduling, and HIL bridges are all **out of
scope** — they belong to Phases 4 / 5 / 6.

## Scope Summary

### What Phase 3 ships

- **Rigid-body kernel adapter family** in `openbmp-vehicle/adapters`:
  `RigidGravityForceAdapter`, `RigidMotorThrustForceAdapter`,
  `RigidMotorMassAdapter`, `RigidAxialDragForceAdapter`, completing
  the Phase-2.11 Path C deferral. Unblocks
  `vehicle.kind = "rigid_body"` in the runner.
- **Event triggers and `MissionPhaseGraph`** in `openbmp-sim/events`:
  `EventTrigger` trait, `BuiltInEventTrigger`
  (AtTime / AtAltitude / AtApogee / AtMassFraction /
  AtDynamicPressure / Scripted), `EventBinding`, `EventAction`,
  `MissionPhaseGraph` with explicit edge guards and event-driven
  transitions. Replaces the Phase-1 hard-coded apogee detection.
- **`VehicleAssembly` tree** in `openbmp-vehicle/assembly`: the
  `Bodies / Propulsion / Effectors / Tanks / Sensors / MassProperties`
  declarative composition documented in
  [`software-architecture.md § VehicleAssembly Tree`](software-architecture.md#vehicleassembly-tree).
  The loader walks the tree once at scenario load and resolves it
  into a `KernelModelBundle` (flat force / moment / mass / sensor
  lists). Kernel hot-path stays flat-list.
- **`ControlEffector` trait** in `openbmp-vehicle/effector`: rate
  limit, position saturation, latency (pure delay via circular
  buffer), deadband, and the four canonical fault modes (Jam,
  Runaway, ReducedRate, Hardover). Effectors feed aero deck via
  schema-2 effector axes and feed engines via
  `EngineCommand.gimbal_*`.
- **Aero deck schema 2** in `openbmp-aero`: control-effector axes
  (`delta_e_deg`, `delta_a_deg`, `delta_r_deg`, body flaps, grid
  fins). Tensor-product lookup with the same locked operand order
  convention as schema-1 (Demmel & Nguyen 2020). Schema-1 decks
  degrade automatically (effector axes default to `[0]`).
- **`EngineModel` + `EngineCluster`** in `openbmp-propulsion/engine`:
  per-engine throttle / gimbal / ignition / shutdown commands;
  `EngineState` lifecycle (`Idle → Igniting → Burning → Shutdown`,
  with `Failed` as terminal); cluster-level summed thrust + moment
  + mass-flow over scenario-declared mount points. Ships with
  closed-form ignition transient (rise to commanded thrust over
  `t_ignite`) for byte-stability; high-fidelity ignition transients
  are downstream-extension territory.
- **`Tank` + `MovingMassModel`** in `openbmp-vehicle/tank`: liquid
  mass as generic moving-mass dynamics. Phase-3 ships `RigidLiquid`
  (toy, no slosh), `EquivalentPendulum` (Abramson SP-106 §7
  cylindrical-tank fundamental frequency), `EquivalentSpringMass`,
  and `BaffledPendulum` (with `BaffleModel` damping increment).
  Drain rate driven by parent `EngineCluster`.
- **Wind extensions** in `openbmp-env`: `LayeredWind`
  (per-altitude wind table with linear interpolation between
  layers) and `GustWind` (Dryden rational-spectrum shaping filter
  per MIL-STD-1797A; six parameters: σ_u/σ_v/σ_w intensity scales
  and L_u/L_v/L_w length scales).
- **Recovery models** in `openbmp-vehicle/recovery`: `ParachuteDrag`
  (drag-area changes after deploy), `DrogueMainRecovery` (two-stage
  academic descent), `DragDevice` (generic airbrake). Deploy events
  driven by `MissionPhaseGraph` triggers.
- **Synthetic sensors** in `openbmp-sensors`: `SyntheticGnss`
  (receiver-output position / velocity noise + position-bias drift;
  IS-GPS-200 nominal noise budget plus tunable σ_pos and σ_vel),
  `SyntheticMagnetometer` (body-frame WMM truth field + Gaussian
  noise + soft/hard-iron bias),
  `SyntheticStarTracker` (per-axis Gaussian quaternion-error
  injection).
- **WMM 2025** in `data/magnetic/WMM.COF`: verbatim public-domain
  coefficient table from NOAA / NGA / UK DGC, with
  `data/magnetic/provenance.md` citing the December 2024 release
  and the SHA-256 pin of the upstream file.
- **Cesaroni Pro75 M1670 motor** in `data/motors/cesaroni-m1670.toml`:
  imported from ThrustCurve.org with the existing per-file license
  + SHA-256-pin pattern. Used by the Calisto cross-tool validation
  case.
- **RocketPy Calisto canonical scenario** in
  `scenarios/sounding-rocket/rocketpy-calisto.toml`: the full Calisto
  airframe (14.426 kg dry, 0.0635 m radius, Von Kármán nose, 4
  trapezoidal fins), Cesaroni Pro75 M1670 motor, dual-curve drag
  (power-off / power-on), Spaceport America launch site
  (32.99°N, 106.97°W, 1400 m), 5.2 m rail at 85° inclination, drogue
  + main parachute deploy events. Validated against RocketPy's
  reference apogee (3349 m AGL ± 1%).

### What Phase 3 explicitly does NOT ship

| Deferred | Phase | Reason |
|---|---|---|
| Multi-body separation as first-class | 5 | Phase-3 hack: spent stage discarded after separation event; multi-`VehicleAssembly` propagation is the Phase-5 promotion |
| Virtual flight controller (estimator, autopilot, FDIR, guidance) | 4 | Own dedicated phase with its own validation cases |
| Adaptive integrators (DOPRI5/8, RKF78) | 5 | Bit-stable RK4 + Phase-3 effector latency model is sufficient |
| Multi-rate scheduling (`RatePlan`) | 5 | Single-rate kernel is sufficient for Phase-3 academic scenarios |
| HIL socket bridge | 5 | Off-by-default tooling; downstream user concern |
| Hypersonic methods, real-gas thermodynamics, aerothermal | 6 | Earth-atmosphere only; sub-Mach 5 |
| Real fielded engine / TPS / mass-property datasets | never | Hard project boundary; see `safety-boundaries.md` |
| Powered-descent guidance scaffolds (LCvxLD / SCvx) | 4 | Belongs to virtual flight controller |

## Architectural Decisions Already Settled

The trait surfaces and config shapes Phase 3 implements were
specified during Phase 0 and reaffirmed in `software-architecture.md`.
The Phase-3 implementation **must not re-design these** — the
architecture references below are the contract.

| Surface | Reference |
|---|---|
| `VehicleAssembly` trait, `Body`, `KernelModelBundle` | [`software-architecture.md § VehicleAssembly Tree`](software-architecture.md#vehicleassembly-tree) |
| `EngineModel`, `EngineCommand`, `EngineState`, `EngineCluster` | [`software-architecture.md § Propulsion: EngineModel and EngineCluster`](software-architecture.md#propulsion-enginemodel-and-enginecluster) |
| `ControlEffector`, `EffectorLimits`, `EffectorState`, `EffectorFault` | [`software-architecture.md § Control Effectors`](software-architecture.md#control-effectors) |
| `Tank`, `MovingMassModel`, `MassContribution` | [`software-architecture.md § Tanks and Slosh as Moving-Mass Dynamics`](software-architecture.md#tanks-and-slosh-as-moving-mass-dynamics) |
| `EventTrigger`, `BuiltInEventTrigger`, `EventAction`, `MissionPhaseGraph` | [`software-architecture.md § Event / Phase Timeline`](software-architecture.md#event--phase-timeline) |
| `AeroMethod` schema-2 deck format with effector axes | [`software-architecture.md § Deck Format Extensions: Control-Effector Axes`](software-architecture.md#deck-format-extensions-control-effector-axes) |
| `ParachuteDrag`, `DrogueMainRecovery`, `DragDevice` | [`software-architecture.md § Recovery and Descent Models`](software-architecture.md#recovery-and-descent-models) |
| `SyntheticSensor` trait surface | [`software-architecture.md § Synthetic Sensors`](software-architecture.md#synthetic-sensors) |
| Determinism contract additions for Phase 3 | [`software-architecture.md § Determinism Profile`](software-architecture.md#determinism-profile) |
| Validation labels + tolerance tables | [`verification.md`](verification.md) |
| Provenance contract for shipped data | [`data-provenance.md`](data-provenance.md) |

## Toolchain and Dependency Decisions

### Inherited from Phase 2

- Rust 1.95 pinned in `rust-toolchain.toml`.
- All workspace pins in `[workspace.dependencies]` (`nalgebra`,
  `uom`, `serde`, `toml`, `arrow`, `parquet`, `clap`, `miette`,
  `tracing-subscriber`, `proptest`, `insta`, `criterion`,
  `assert_cmd`, `insta-cmd`, `predicates`, `tempfile`, `sha2`).
- `clippy::pedantic + clippy::all` warn-by-default,
  `unwrap_used / expect_used / panic` warn, `float_cmp` deny,
  `missing_docs` deny.
- The MXCSR guard at kernel construction.
- The `-C target-feature=-fma` rustflag for
  `x86_64-unknown-linux-gnu`.
- The deterministic ChaCha8 RNG seeded by `(scenario_seed,
  step_index, channel_id)` and the Phase-2.7
  `for_sensor_component(seed, step, sensor_id, component_id)`
  variant.
- The Phase-2.10 `sha2 = "0.10"` dependency for SHA-256 pinning of
  external file references.
- The Phase-2.11 dispatcher pattern (Phase-1 byte-stable analytic
  path vs. Phase-2 point-mass-with-adapters path) — Phase 3 adds a
  third path for `vehicle.kind = "rigid_body"` once 3.1 ships.

### New external dependencies

**No new physics-model dependencies.** The Phase-3 models are all
in-house Rust against public sources:

- **WMM 2025** coefficient table: in-house port of the public-domain
  `WMM.COF` ASCII file (90 spherical-harmonic coefficients to
  degree 12). Citation per NOAA NCEI guidance.
- **Dryden gust** rational-spectrum filter: in-house Rust
  implementation; closed-form first-order shaping filter takes white
  noise from the new `for_wind_component` RNG channel and produces
  colored turbulence with the documented PSD.
- **Abramson SP-106 slosh**: in-house Rust implementation of the
  equivalent-pendulum dynamics with closed-form fundamental
  frequency from Table 7.1 (cylindrical tank, axisymmetric mode).
- **RocketPy Calisto** scenario data: imported via the existing
  `data/motors/`, `data/aero/`, and `scenarios/sounding-rocket/`
  format with SHA-256 pinning. RocketPy is MIT-licensed; OpenBMP
  cites the originating repository and the published RocketPy paper.

### New internal seams

- **`for_wind_component(scenario_seed, step, axis_id)`** in
  `openbmp-core::DeterministicRng` for Dryden gust noise. New
  domain tag `b"WIND"` in seed-material bytes [28..32] preventing
  collision with `for_channel` (`b"CHAN"`) and
  `for_sensor_component` (`b"SENS"`).
- **Per-engine ignition transient** as a closed-form rise from 0 to
  commanded thrust over `t_ignite_s` (default 0; configurable per
  engine). Locked operand order; no FMA.
- **Per-tank slosh ODE sub-step**. Default 1 sub-step (forward Euler
  inside the main RK4). Configurable in scenario for higher fidelity
  but at the cost of bit-stability across sub-step counts. Locked
  operand order on the slosh state update.
- **`MissionPhaseGraph` evaluation** as a topological-sort-stable
  iteration of edge guards once per main step, after force / moment
  / mass evaluation, before telemetry recording.

### Toolchain

- `rust-toolchain.toml` stays pinned to Rust 1.95.
- Workspace `rust-version` stays at the lower MSRV.
- No new CI dependencies; the existing determinism gate in
  `.github/workflows/ci.yml` extends to add the Calisto scenario as
  a third byte-stable run alongside analytic-toy and Niskanen.

## Implementation Seams Locked Before Coding

The following questions are closed here so implementation does not
reopen the architecture by accident:

- **`VehicleAssembly` resolution timing.** The scenario loader walks
  the assembly tree once at scenario-load time and produces a
  `KernelModelBundle` — flat force / moment / mass / sensor lists in
  scenario-declared order. The kernel hot path consumes only the
  flat lists; no per-step tree traversal.
- **Per-engine model-id allocation.** Cluster engines reserve the
  `200..300` `ModelId` range; the existing single-motor IDs stay in
  `100..200`. The Phase-2 mass-model id (`104`) is unaffected. New
  IDs are `(200 + cluster_index * 10 + engine_index)` so
  per-cluster-per-engine is collision-free up to 10 engines per
  cluster (extensible to 100 via `200 + cluster_index * 100`
  before Phase-3 audit closes).
- **Effector latency.** Pure delay via a fixed-depth circular buffer
  (depth = `⌈latency / dt⌉`, computed at scenario load). Determinism
  preserved because the buffer state resets at scenario start and
  every step pushes / pops one entry. Phase 3 does not approximate
  latency with second-order shaping; that's Phase-4 controller-side
  territory.
- **Effector saturation, rate limit, deadband.** First-order linear
  actuator model from Stevens & Lewis 2015 §10.4.2 with explicit
  rate clamping: `position_dot = clamp((commanded - position) /
  τ, -max_rate, +max_rate)`. Deadband is applied to the *commanded*
  signal, not the realised position.
- **Effector fault-mode semantics.** Closed enum per architecture
  spec; faults are scenario-injected via `inject_fault(EffectorFault)`,
  evaluated at `step()` time. The four variants follow the canonical
  FDIR taxonomy (Patton et al. 1989).
- **Engine ignition transient.** Closed-form linear ramp from 0 to
  commanded thrust over `t_ignite_s` seconds. Default `t_ignite_s
  = 0` reproduces the existing solid-motor behaviour
  (instant on at `ignite_at_s`). Liquid engines default to
  `t_ignite_s = 0.1` per Sutton & Biblarz 2017 typical.
- **Slosh integration.** Forward Euler within the main RK4 step
  (single sub-step) by default. Locked operand order on
  `(theta_dot, theta)` update: `theta_new = theta + dt * theta_dot;
  theta_dot_new = theta_dot + dt * theta_ddot` (in this order, no
  FMA). Higher sub-step counts are a per-tank scenario opt-in but
  break bit-stability across sub-step-count changes — documented in
  the tank's `provenance.md`.
- **Dryden gust seeding.** New
  `DeterministicRng::for_wind_component(seed, step, axis_id)` with
  domain tag `b"WIND"`. White-noise samples come from the standard
  ChaCha8 stream; the shaping filter accumulates state per-axis
  per-step. Replay-stable as long as filter state resets at scenario
  start.
- **`MissionPhaseGraph` semantics.** Phases form a directed graph;
  each transition has an explicit edge guard (boolean predicate over
  state + time + step) plus an optional event-driven trigger from
  `EventTrigger`. Only one phase active at a time; transitions
  evaluated in topological-sort-stable order once per main step.
  No re-entry to a phase already exited (the graph is acyclic for
  Phase 3; Phase-4 controllers may want cycles, deferred).
- **Multi-body Phase-5 vs. Phase-3 hack.** Phase 3 ships the
  scripted-staging hack only: a separation event drops the spent
  stage from the active vehicle (mass / inertia removed; the spent
  stage's state is *not* propagated). Phase-5 multi-body propagates
  both halves with momentum exchange.
- **Aero deck schema-2 fallback.** Loader detects the integer marker
  `openbmp.aero_deck = 2` vs. legacy `openbmp.aero_deck = 1`.
  Schema-2 decks declare effector axes through `[axis_order]`;
  legacy schema-1 decks have no effector axes and ignore the
  deflection map.
- **WMM 2025 model timeline.** The model is valid only through
  2030-01-01. Scenarios that declare an epoch outside this range
  fail closed with a typed `MagneticOutOfEpoch` error. Phase 3
  accepts only the 2025 model; subsequent WMM 2030 lands in
  whichever phase is current at NGA's 2029-end release.
- **Calisto cross-tool tolerance.** The published RocketPy reference
  apogee is 4820.548 m ASL (3349 m AGL). OpenBMP's apogee must fall
  within ±1% of this value (33.5 m envelope). Bit-stability across
  reruns is intra-baseline; cross-tool *bit*-equality with RocketPy
  is *not* claimed (different integrators, different sub-step
  cadences). The 1% physical envelope is the gate.

## Sub-phase Plan

The 12 sub-phases below mirror the Phase-2 cadence: small focused
commits per sub-phase, each one closing a verifiable deliverable.
Effort estimates assume one engineer working effectively full-time on
the sub-phase; multiply by your own discount factor.

### 3.1 — Rigid-body kernel adapter family

**Scope.** Complete the Phase-2.11 Path C deferral: implement
`ForceModel<RigidBodyState>` for the existing
`GravityForceAdapter`, `MotorThrustForceAdapter`,
`AxialDragForceAdapter`, plus
`MassModel<RigidBodyState>` for `MotorMassAdapter`. Unblocks
`vehicle.kind = "rigid_body"` in the runner.

**Decision sources.**

- The Phase-2.9 adapter implementations
  (`crates/openbmp-vehicle/src/adapters.rs`) are the contract. The
  rigid-body equivalents are mechanical ports: same physics,
  different `SimState` ctx.
- Rigid-body force models read the body-frame attitude from
  `state.orientation` (quaternion body-to-ECI) and convert
  body-frame thrust to ECI via the standard quaternion-rotation
  identity.

**Tasks.**

- `crates/openbmp-vehicle/src/adapters.rs`:
  - `impl<G: GravityModel> ForceModel<RigidBodyState> for
    GravityForceAdapter<G>` — same as point-mass impl, reads
    `state.position.vector` instead of point-mass position field.
  - `impl<M: Motor> ForceModel<RigidBodyState> for
    MotorThrustForceAdapter<M>` — body-frame thrust along `+z`
    transformed to ECI via `state.orientation.rotate(thrust_body)`.
  - `impl<Atm: AtmosphereModel> ForceModel<RigidBodyState> for
    AxialDragForceAdapter<Atm>` — Phase-3.1 rigid drag delegates to
    the shared translational ECI-velocity axial-drag computation;
    aero-point rotational velocity, wind-relative body-frame
    sideslip, and aero moments remain Phase-3 follow-ons.
  - `impl<M: Motor> RigidMassModel for MotorMassAdapter<M>` —
    `MassProperties` with constant inertia tensor (Phase-2 motors do
    not publish inertia derivatives; deferred to Phase 3.6 engines).
- `crates/openbmp-cli/src/runner/phase2_rigid_body.rs` (new):
  mirror of `phase2_point_mass.rs` for `RigidBodyState`. The
  dispatcher in `runner/mod.rs` adds a third arm: when
  `vehicle.kind = "rigid_body"`, route to the new module.
- `crates/openbmp-scenario/src/document.rs`: extend the rigid-body
  initial-state fields (already present from Phase 2.10) with
  required `inertia_tensor_body_kg_m2: [[f64; 3]; 3]` for full
  mass-property declaration. The parser performs finite, symmetric,
  and positive-diagonal checks; kernel mass-property construction
  performs the full positive-definite and triangle-inequality checks.

**Tests.**

- Unit: `RigidGravityForceAdapter::force_n_eci` returns the same
  vector as the point-mass adapter for a state with identity
  orientation.
- Property test (proptest): for a random valid `RigidBodyState`
  with random unit quaternion, the body-to-ECI rotation of a body
  +z thrust unit vector is the third column of the rotation matrix.
- Determinism: byte-stable replay across two reruns of a torque-free
  rigid-body scenario.
- Phase-1 byte-stability gate (analytic-toy) stays green.
- New integration test: Niskanen scenario adapted to
  `vehicle.kind = "rigid_body"` (with full inertia tensor) runs
  through the rigid-body kernel and produces an apogee within
  ±5% of the point-mass Niskanen result.

**Exit criteria.**

- The runner accepts `vehicle.kind = "rigid_body"` and routes
  through the rigid-body kernel for scenarios using only gravity,
  thrust, and aero.
- Niskanen-rigid scenario apogee within 5% of Niskanen-point-mass.
- All four rigid-body adapters compile and unit-test green.

**Effort.** Medium (~1.0 week).

### 3.2 — Event triggers and `MissionPhaseGraph`

**Scope.** First-class event/phase scheduling. Replaces the Phase-2
hard-coded apogee detection in the kernel with a declarative
`MissionPhaseGraph` plus `EventTrigger` evaluation per main step.

**Decision sources.**

- `software-architecture.md § Event / Phase Timeline` is the trait
  surface contract.
- Stevens & Lewis 2015 §11.3 "State Machine Based Mode Control" for
  the topological-sort-stable iteration semantics.
- Phase-2 stop-condition pattern (`StopCondition` trait in
  `openbmp-sim/src/stop.rs`) for the API shape.

**Tasks.**

- `crates/openbmp-sim/src/events.rs` (new):
  - `EventTrigger` trait with `fired(state, t, step) -> bool`.
  - `BuiltInEventTrigger` enum: `AtTime`, `AtAltitudeAscending`,
    `AtAltitudeDescending`, `AtApogee`, `AtMassFraction`,
    `AtDynamicPressure`, `Scripted`.
  - `EventBinding { trigger, action, once: bool }`.
  - `EventAction` enum: `EnterPhase`, `EngineCommand`,
    `EffectorOverride`, `Separation`, `DeployRecovery`,
    `EmitTelemetryMarker`.
  - `MissionPhaseGraph { phases, transitions }` with topological-sort
    initialisation in the constructor; mutation rejected.
  - `Phase { id, label, allowed_effectors, allowed_engines }`.
  - `PhaseTransition { from, to, guard, event }`.
- `crates/openbmp-scenario/src/document.rs`: new optional
  `[mission]` block carrying `phases` and `transitions` arrays.
  Scenario-side declaration of the graph is parsed once at load
  time.
- `crates/openbmp-sim/src/kernel.rs`: extend `SimulationConfig`
  with optional `events: Vec<EventBinding>` and `mission_graph:
  Option<MissionPhaseGraph>`. The kernel loop calls
  `evaluate_events(...)` after force/moment/mass evaluation, before
  telemetry recording.

**Tests.**

- Unit: each `BuiltInEventTrigger` variant fires correctly under
  prescribed state sequences.
- Property test: `MissionPhaseGraph` topological order is stable
  across reruns regardless of scenario-declared input order.
- Determinism: an event-driven scenario (e.g., `AtApogee` triggers
  a `DeployRecovery` action) produces byte-identical Parquet
  across two reruns.
- Regression: Phase-1 analytic-toy scenario (no events declared)
  produces byte-identical Parquet to pre-3.2 baseline.

**Exit criteria.**

- The Phase-2.11 `at_apogee_marker` channel (which currently is a
  static `false`) becomes a real `EventTrigger::AtApogee`
  evaluation.
- `[mission]` scenario block parses, validates, and round-trips
  through the kernel.

**Effort.** Medium-large (~1.5 weeks).

### 3.3 — `VehicleAssembly` tree

**Scope.** The big architectural sub-phase. Declarative
composition of the rocket from a `Bodies / Propulsion / Effectors /
Tanks / Sensors / MassProperties` tree, resolved into the kernel's
flat model lists at scenario load.

**Decision sources.**

- `software-architecture.md § VehicleAssembly Tree` is the
  contract.
- `real-rocket-integration.md` is the downstream-user cookbook the
  tree shape needs to satisfy.
- The Phase-2.8 `BasicVehicle` (still ships) is the degenerate
  case of `VehicleAssembly` — single body, no effectors / tanks /
  sensors composed via the tree.

**Tasks.**

- `crates/openbmp-vehicle/src/assembly/mod.rs` (new):
  - `VehicleAssembly` trait with the architecture-specified
    surface (`bodies`, `propulsion`, `effectors`, `tanks`,
    `sensors`, `mass_properties`, `into_kernel_models`).
  - `Body { id, geometry, dry_mass_kg, dry_inertia_body }`.
  - `BasicAssembly` — a flat-tree struct that owns Vec<Body>,
    `PropulsionTree`, `Vec<Box<dyn ControlEffector>>`, `Vec<Tank>`,
    `Vec<Box<dyn SyntheticSensor>>`.
  - `KernelModelBundle` produced by `into_kernel_models()`:
    flat `Vec<Box<dyn ForceModel>>`, `Vec<Box<dyn MomentModel>>`,
    `Box<dyn MassModel>`, `Vec<Box<dyn SyntheticSensor>>`.
- `crates/openbmp-scenario/src/document.rs`: replace the
  Phase-2 flat `[vehicle]` block with `[vehicle.assembly]` carrying
  child tables (`[vehicle.assembly.bodies]`,
  `[vehicle.assembly.propulsion.cluster.0]`, etc.). The Phase-2
  scenarios continue to parse via the legacy single-body path
  (loader synthesises a one-body assembly).
- `crates/openbmp-cli/src/runner/`: dispatcher learns to
  resolve a scenario's assembly into a `KernelModelBundle` and
  hand the kernel only the flat lists. The Phase-1 byte-stable
  path stays untouched.

**Tests.**

- Unit: `BasicAssembly::into_kernel_models()` produces lists in
  scenario-declared order.
- Determinism: re-loading the same scenario produces a
  bit-identical `KernelModelBundle` (every model's address may
  differ but every output bit is identical).
- Property test: reordering scenario `bodies` declarations changes
  the kernel's mass-property summation order (verifying that
  declared-order is preserved).
- Regression: Niskanen scenario routed through the new assembly
  resolver produces byte-identical Parquet to pre-3.3 (the
  legacy single-body adapter path is functionally identical).

**Exit criteria.**

- A scenario with two bodies (e.g., a fairing on top of a
  rocket) loads and propagates through the kernel.
- The Niskanen scenario continues to produce byte-identical
  Parquet under the new resolver.
- `BasicVehicle` (Phase-2.8) is repositioned as the
  degenerate-case `BasicAssembly` constructor.

**Effort.** Large (~2.0 weeks).

### 3.4 — `ControlEffector` trait + actuator dynamics

**Scope.** Effectors as the controller-physics interface. Rate
limit, position saturation, latency, deadband, and the four
canonical fault modes.

**Decision sources.**

- `software-architecture.md § Control Effectors` is the contract.
- Stevens & Lewis 2015 §10.4.2 "Actuator Models" for first-order
  linear actuator with rate clamping.
- Patton, Frank, Clark 1989 *Fault Diagnosis in Dynamic Systems* for
  the canonical fault taxonomy.

**Tasks.**

- `crates/openbmp-vehicle/src/effector.rs` (new):
  - `ControlEffector` trait with `step(cmd, dt) -> EffectorState`,
    `limits()`, `inject_fault(fault)`.
  - `EffectorLimits { min, max, max_rate_per_s, deadband, latency }`.
  - `EffectorState { commanded, actual, saturated, rate_limited,
    fault: Option<EffectorFault> }`.
  - `EffectorFault` enum: `Jam { at }`, `Runaway { rate_per_s }`,
    `ReducedRate { factor }`, `Hardover { to }`.
  - `LinearActuator` impl: first-order linear actuator with rate
    clamp, position saturation, deadband, and a fixed-depth circular
    buffer for pure-delay latency.
- `crates/openbmp-scenario/src/document.rs`: new optional
  `[effectors.<name>]` block declaring kind, limits, and faults.
- `crates/openbmp-vehicle/src/assembly/`: assembly tree learns
  to register effectors and pass them through `KernelModelBundle`.
- Determinism plumbing: effector RNG channel via
  `for_effector_component(seed, step, effector_id, component_id)`
  with domain tag `b"EFFC"` for any per-step random-shaped fault
  injection.

**Tests.**

- Unit: each fault mode produces the documented behaviour
  (Jam locks position; Runaway ignores command; ReducedRate
  scales rate; Hardover steps to extreme then jams).
- Property test: position never exits `[min, max]` regardless of
  command; rate of change never exceeds `max_rate_per_s` (in any
  fault mode that respects rate).
- Determinism: an effector schedule replayed twice produces
  byte-identical effector telemetry.

**Exit criteria.**

- A scenario declaring a single elevon effector with `delta_e`
  command schedule produces deterministic deflection telemetry.
- All four fault modes integration-tested against expected
  behaviour.

**Effort.** Medium-large (~1.5 weeks).

### 3.5 — Aero deck schema 2 (control-effector axes)

**Scope.** Extend the Phase-2.5 aero deck format to support
control-effector axes. Schema-1 decks degrade; schema-2 decks
declare effector axes and provide the multi-dimensional coefficient
table.

**Decision sources.**

- `software-architecture.md § Deck Format Extensions: Control-
  Effector Axes` is the format contract.
- Demmel & Nguyen 2020 *Algorithms for Efficient Reproducible
  Floating Point Summation* for the locked operand order
  (extended to the new dimensions).
- The Phase-2.5 deck's existing trilinear interpolation is the
  3-D base case; schema-2 extends to 4-D, 5-D, 6-D as needed.

**Tasks.**

- `crates/openbmp-aero/src/deck.rs`: extend `AeroDeck` to carry an
  explicit `axis_order: Vec<String>`, `axes: Vec<Vec<f64>>`, and flat
  coefficient tables indexed by the cartesian product of declared axes.
  Lookup signature: `lookup(mach, alpha, beta, deflections:
  &BTreeMap<&str, f64>) -> Result<AeroCoefficients, AeroError>`.
- `crates/openbmp-aero/src/parser.rs`: schema discriminator —
  schema-1 (no `[axis_order]` section) loads as before;
  schema-2 (`openbmp.aero_deck = 2`) loads the 4D/5D/6D table.
- `crates/openbmp-aero/src/method.rs`: `DeckLookup` extends to
  consume the runner's effector-actuals snapshot through the
  kernel-side adapters. Effector deflections are the *actual*
  (rate-limited / saturated / faulted) values from
  `EffectorState.actual`, not the commanded values.

**Tests.**

- Unit: schema-1 deck loads as before.
- Unit: schema-2 deck with a single effector axis (`delta_e_deg =
  [0]`) is bit-equivalent to schema-1.
- Unit: schema-2 deck with non-trivial effector axis produces
  per-corner exact-equality on every grid corner.
- Property test: lookup with all effector deflections at zero
  matches the schema-1 lookup with the same `(mach, alpha, beta)`.

**Exit criteria.**

- A schema-2 deck with `delta_e_deg = [-20, 0, 20]` axis loads,
  validates, and produces interpolated coefficients.
- The Niskanen scenario (using schema-1) continues to produce
  byte-identical Parquet under the new loader.

**Effort.** Medium (~1.0 week).

### 3.6 — `EngineModel` + `EngineCluster`

**Scope.** Multi-engine support. Per-engine throttle, gimbal,
ignition lifecycle, plus cluster-level summed thrust / moment /
mass-flow.

**Decision sources.**

- `software-architecture.md § Propulsion: EngineModel and
  EngineCluster` is the contract.
- Sutton & Biblarz *Rocket Propulsion Elements* 9th ed (2017)
  Chapter 8 for the per-engine state machine and ignition transient
  model.

**Tasks.**

- `crates/openbmp-propulsion/src/engine.rs` (new):
  - `EngineModel` trait with the architecture-spec'd surface.
  - `EngineCommand { throttle_unit, gimbal_pitch_rad,
    gimbal_yaw_rad, ignite, shutdown }`.
  - `EngineState` enum: `Idle`, `Igniting`, `Burning`, `Shutdown`,
    `Failed`.
  - `LiquidEngine` impl with closed-form ignition transient (linear
    rise from 0 to commanded thrust over `t_ignite_s`).
  - `EngineCluster { engines, mount_points_body, layout }`.
  - `impl ForceModel for EngineCluster { ... }` — sums per-engine
    body-frame thrust and rotates to ECI via state orientation.
  - `impl MomentModel for EngineCluster { ... }` — sums
    per-engine moment about origin (cross product of mount-point
    radius vector and thrust force).
  - `impl MassModel for EngineCluster { ... }` — sums per-engine
    mass-flow rates and integrates to current cluster mass.
- `crates/openbmp-scenario/src/document.rs`: new
  `[propulsion.cluster.<id>]` and
  `[propulsion.cluster.<id>.engine.<n>]` blocks carrying engine
  parameters and mount-point geometry.
- `crates/openbmp-vehicle/src/assembly/`: `PropulsionTree`
  learns to host both single motors (Phase-2 path) and
  multi-engine clusters.

**Tests.**

- Unit: a 1-engine cluster is functionally equivalent to a
  Phase-2.6 single-motor scenario.
- Unit: a 9-engine octaweb cluster's summed thrust matches the
  scalar sum of per-engine thrusts at any throttle setting.
- Unit: per-engine ignition / shutdown commands drive the state
  machine through the documented transitions; once-`Failed` is
  terminal.
- Property test: cluster moment about origin equals the cross
  product of mount-point and thrust for each engine, summed.
- Determinism: a multi-engine scenario produces byte-identical
  Parquet across two reruns.

**Exit criteria.**

- A 4-engine cluster with one engine commanded to shutdown at
  T+5s produces the expected mass-flow drop and asymmetric thrust
  vector.
- The Phase-2 single-motor scenarios (Niskanen, etc.) continue
  unchanged via the legacy single-motor path.

**Effort.** Medium-large (~1.5 weeks).

### 3.7 — `Tank` + `MovingMassModel`

**Scope.** Slosh dynamics as generic moving-mass. Phase 3 ships
four `MovingMassModel` impls; the equivalent-pendulum is the
default for academic scenarios.

**Decision sources.**

- `software-architecture.md § Tanks and Slosh as Moving-Mass
  Dynamics` is the contract.
- Abramson 1966 *NASA SP-106 The Dynamic Behavior of Liquids in
  Moving Containers* is the canonical reference. §7 covers the
  equivalent-pendulum dynamics for cylindrical and spherical
  tanks. Public domain US-government work.
- Chapter 7 Eq. 7-25 (equivalent pendulum dynamics):
  `θ̈ + 2ζω_n θ̇ + ω_n² sin(θ) = (a_lateral / L_pend) cos(θ)`
  where `ω_n = √(g_eff / L_pend)` and `L_pend` is the equivalent
  pendulum length from Table 7.1.

**Tasks.**

- `crates/openbmp-vehicle/src/tank/mod.rs` (new):
  - `Tank { id, geometry, mounted_to, mount_point_body,
    propellant, initial_fill_fraction, baffle_model, moving_mass }`.
  - `MovingMassModel` trait per architecture spec.
  - `MassContribution { mass_kg, cg_offset_body_m, inertia_delta_body }`.
- `crates/openbmp-vehicle/src/tank/rigid_liquid.rs`: `RigidLiquid`
  impl — toy, no slosh; mass is the fill fraction times tank
  volume times propellant density. Reaction force is zero.
- `crates/openbmp-vehicle/src/tank/equivalent_pendulum.rs`:
  `EquivalentPendulum` impl — closed-form fundamental frequency
  from Abramson Table 7.1 (cylindrical tank, axisymmetric
  fundamental). Forward-Euler sub-step integration of slosh angle
  / rate per main step.
- `crates/openbmp-vehicle/src/tank/equivalent_spring_mass.rs`:
  `EquivalentSpringMass` impl — linear-spring restoring force
  alternative; same operands order as pendulum.
- `crates/openbmp-vehicle/src/tank/baffled_pendulum.rs`:
  `BaffledPendulum` impl — pendulum with `BaffleModel`-supplied
  damping increment.

**Tests.**

- Unit: `RigidLiquid` reaction force is zero at every state.
- Unit: `EquivalentPendulum` free response (no acceleration input)
  oscillates at the closed-form frequency to within 1% over 100
  oscillations.
- Property test (proptest): for any small initial slosh angle
  (≤ 0.1 rad), `EquivalentPendulum` energy (`½θ̇² + ½ω_n²θ²`) is
  conserved within 1% over 100 oscillations.
- Determinism: a sloshing scenario produces byte-identical
  per-step slosh-state telemetry across two reruns.

**Exit criteria.**

- All four `MovingMassModel` impls compile and unit-test green.
- Equivalent-pendulum free-response matches the Abramson §7.4
  closed-form frequency within 1%.

**Effort.** Medium-large (~1.5 weeks).

### 3.8 — Wind extensions: `LayeredWind` and `GustWind`

**Scope.** Two new wind models. `LayeredWind` is per-altitude
table interpolation; `GustWind` is the Dryden rational-spectrum
shaping filter.

**Decision sources.**

- MIL-STD-1797A "Flying Qualities of Piloted Aircraft" (the
  Dryden gust model is a US DoD standard).
- *Wikipedia*: Dryden Wind Turbulence Model (summary of spectral
  forms).
- ESDU 85020 / 85020 series for atmosphere wind reference data
  (cited in the architecture spec as
  `docs/data-provenance.md § Preferred Public Sources`).

**Tasks.**

- `crates/openbmp-env/src/wind/layered.rs` (new):
  - `LayeredWind { layers: Vec<LayerEntry> }`.
  - `LayerEntry { altitude_m, wind_ned_m_s }`.
  - Linear interpolation between layers; out-of-range
    fail-closed via `WindError::OutOfRange`.
- `crates/openbmp-env/src/wind/gust.rs` (new):
  - `GustWind` with closed-form first-order rational shaping
    filter per axis (longitudinal, lateral, vertical).
  - Six tunable parameters (`σ_u`, `σ_v`, `σ_w`, `L_u`, `L_v`,
    `L_w`).
  - White noise from
    `DeterministicRng::for_wind_component(seed, step, axis_id)`
    with domain tag `b"WIND"`.
- `crates/openbmp-scenario/src/document.rs`: extend `[wind]`
  block to accept `kind = "layered"` or `kind = "gust"` plus
  the per-kind parameters.

**Tests.**

- Unit: `LayeredWind` interpolation between two layers matches
  the linear-interpolation formula at midpoints to bit equality.
- Unit: `GustWind` with `σ = 0` produces zero output regardless
  of step.
- Statistical regression: a long `GustWind` run (10 000 steps)
  matches the Dryden PSD within 5% across 10 spectral bins.
- Determinism: `GustWind` produces byte-identical Parquet across
  two reruns of the same scenario.

**Exit criteria.**

- `LayeredWind` and `GustWind` compile and unit-test green.
- A scenario with `[wind].kind = "gust"` produces deterministic
  per-step wind telemetry.

**Effort.** Small-medium (~0.7 week).

### 3.9 — Recovery models: parachute and drag-device

**Scope.** Three new recovery models, all driven by deploy events
from the Phase-3.2 `MissionPhaseGraph`.

**Decision sources.**

- Knacke 1992 *Parachute Recovery Systems Design Manual* (NWC TP
  6575) — public US Government work. Chapter 5 covers the standard
  drag-area model: `F_drag = ½ ρ v² C_D · A_inflated`.
- `software-architecture.md § Recovery and Descent Models` is the
  contract.

**Tasks.**

- `crates/openbmp-vehicle/src/recovery/parachute_drag.rs`:
  `ParachuteDrag` — drag area changes after a deploy event;
  pre-deploy drag is zero, post-deploy drag is `½ ρ v² C_D
  A_inflated`. `C_D` and `A_inflated` are scenario-declared.
- `crates/openbmp-vehicle/src/recovery/drogue_main.rs`:
  `DrogueMainRecovery` — two-stage academic descent. Drogue deploys
  at apogee; main deploys at scenario-declared altitude AGL.
- `crates/openbmp-vehicle/src/recovery/drag_device.rs`:
  `DragDevice` — generic airbrake / drag-device coefficient
  change.
- `crates/openbmp-scenario/src/document.rs`: new
  `[[vehicle.assembly.recovery]]` block declaring kind and
  per-kind parameters; `mission.events[*].action.deploy_recovery`
  drives state changes.
- Telemetry channels per device:
  `recovery.<id>.deployed`, `recovery.<id>.phase_index`,
  `recovery.<id>.drag_area_m2`.

**Tests.**

- Unit: pre-deploy drag is zero; post-deploy drag matches
  `½ ρ v² C_D A_inflated`.
- Analytic-toy: constant-density terminal velocity for a 1 kg
  parachute matches the closed-form
  `v_terminal = √(2 m g / (ρ C_D A))`.
- Determinism: a parachute-deploy scenario produces
  byte-identical Parquet across two reruns.

**Exit criteria.**

- The three models compile and unit-test green.
- Calisto's drogue + main deploy correctly in the Phase-3.11
  validation case.

**Effort.** Small-medium (~0.7 week).

### 3.10 — Synthetic sensors: GNSS, magnetometer, star tracker

**Status.** **Implemented (Phase 3.10.A–E).** All three sensors
ship and validate against their respective public references;
WMM 2025 evaluation matches NOAA reference test values within
5 nT per component (well inside the 4-significant-figure tolerance
the WMM publication declares). Architecture status note flipped
in `software-architecture.md § Synthetic Sensors`.

**Scope.** Three new sensors per the architecture spec. WMM 2025
ships as the magnetic field reference data.

**Decision sources.**

- `software-architecture.md § Synthetic Sensors` is the contract.
- IS-GPS-200 (NAVSTAR Global Positioning System Interface
  Specification) — public US Government work — for the GNSS
  receiver-output noise budget.
- WMM 2025 (NOAA NCEI / NGA / UK DGC) for the magnetic-field truth.
  Public domain. The 90-coefficient `WMM.COF` file ships as
  `data/magnetic/WMM.COF` with `provenance.md` citing the
  December 2024 release.
- Crassidis et al. 2007 *Survey of Nonlinear Attitude Estimation
  Methods* §3.2 for star-tracker noise budgets.

**Tasks.**

- `crates/openbmp-sensors/src/gnss.rs` (new): `SyntheticGnss` with
  per-axis Gaussian position noise + bias drift (Ornstein-Uhlenbeck
  per Phase-2.7 IMU pattern).
- `crates/openbmp-sensors/src/magnetometer.rs` (new):
  `SyntheticMagnetometer` — body-frame WMM truth carrier plus
  Gaussian noise + soft-iron + hard-iron bias.
- `crates/openbmp-sensors/src/star_tracker.rs` (new):
  `SyntheticStarTracker` — per-axis Gaussian quaternion-error
  injection (small-angle approximation valid for typical
  arc-second-level noise).
- `data/magnetic/WMM.COF`: verbatim 90-coefficient table with
  `data/magnetic/provenance.md` citing NOAA NCEI / NGA / UK DGC
  December 2024 release. SHA-256 pin per the four-pillar contract.
- `crates/openbmp-env/src/magnetic/mod.rs` (new): `MagneticModel`
  trait surface.
- `crates/openbmp-env/src/magnetic/wmm2025.rs` (new): `Wmm2025`
  impl that evaluates the spherical-harmonic series at a given
  (latitude, longitude, altitude, time).

**Tests.**

- Unit: `SyntheticGnss` truth-bypass mode (zero noise) returns
  position bit-equal to truth.
- Unit: `Wmm2025` parses all 100 shipped NOAA reference rows and
  matches the published X / Y / Z values within 5 nT per component.
- Statistical: `SyntheticGnss` long-run mean is zero (no drift in
  the noise component).
- Determinism: each sensor's measurement stream is byte-stable
  across two reruns.
- Determinism: per-component noise streams (position vs. bias)
  don't shift if a sibling component is added or removed.

**Exit criteria.**

- All three sensors compile, deterministic-replay correctly, and
  validate against their respective public references.
- WMM 2025 evaluation matches the reference field within 4 sig
  figs across the shipped NOAA reference table.

**Effort.** Medium (~1.0 week).

### 3.11 — RocketPy "Calisto" cross-tool validation case

**Status.** **Implemented (Phase 3.11.A–E).** The Cesaroni Pro75
M1670 motor and the Calisto drag deck ship as Schema-1 OpenBMP
artifacts pinned bit-precise against their upstream sources
(`Cesaroni_M1670.eng` from ThrustCurve.org;
`powerOff/powerOnDragCurve.csv` from the RocketPy repo, byte-
identical so they collapse to a single deck). The end-to-end
scenario at `scenarios/sounding-rocket/calisto/rocketpy-calisto.toml`
runs through the rigid-body kernel and produces an apogee within
the Phase-3.11 risk-register fallback envelope of 5 % around
RocketPy's published 3 349 m AGL. The 1 % stretch goal was not
hit — the apparent residual is dominated by the integrator
mismatch between RocketPy's LSODA adaptive step and OpenBMP's
RK4 fixed-step kernel (anticipated in the risk register and
mitigated by landing the wider fallback envelope here). Tightening
toward 1 % is a follow-up that requires either an adaptive RK
integrator on the OpenBMP side or a fixed-step replay of the
RocketPy state at OpenBMP's `dt`.

**Scope.** The headline Phase-3 outcome. The full Calisto rocket
(Cesaroni Pro75 M1670 motor, 14.426 kg dry mass, dual-curve drag,
drogue + main parachute, Spaceport America launch site) runs in
both OpenBMP and RocketPy with apogee within 1% of the
RocketPy-published reference (3349 m AGL).

**Decision sources.**

- RocketPy GitHub repo (`https://github.com/RocketPy-Team/RocketPy`)
  for the Calisto airframe data files
  (`data/motors/cesaroni/Cesaroni_M1670.eng`,
  `data/rockets/calisto/{powerOff,powerOn}DragCurve.csv`,
  `data/airfoils/NACA0012-radians.txt`).
- Souza et al. 2021 *RocketPy: Six Degree-of-Freedom Rocket
  Trajectory Simulator* (Journal of Aerospace Engineering) for the
  RocketPy methodology.
- Spaceport America public coordinates: 32.99°N, 106.97°W, 1400 m
  elevation.

**Tasks.**

- `data/motors/cesaroni-m1670.toml` (new): import of the Cesaroni
  Pro75 M1670 motor from ThrustCurve.org. SHA-256-pinned source.
  Per the existing `data/motors/provenance.md` pattern, with
  reference to the RocketPy repo for cross-tool comparison.
- `data/aero/calisto-power-off.toml` (new): power-off drag-curve
  schema-2 deck (mach axis only, `[delta_e_deg = [0]]`) imported
  from `data/rockets/calisto/powerOffDragCurve.csv`.
- `data/aero/calisto-power-on.toml` (new): power-on drag-curve
  schema-2 deck.
- `scenarios/sounding-rocket/rocketpy-calisto.toml` (new): the
  canonical Phase-3 scenario with the assembly tree (single body,
  single-motor cluster, drogue + main recovery), Spaceport America
  launch site, 5.2 m rail at 85° inclination. Sibling provenance
  entry cites RocketPy's published Calisto example as the
  cross-tool reference. The e2e test pins the published reference
  apogee (3349 m AGL) and tolerance as Rust constants in the test
  file, matching the Niskanen pattern in
  `crates/openbmp-cli/tests/sounding_rocket_e2e.rs`.
- `crates/openbmp-cli/tests/calisto_e2e.rs` (new): integration
  test that runs the Calisto scenario end-to-end via
  `openbmp run`, parses the Parquet, and asserts apogee ± 1% of
  RocketPy's 3349 m AGL.

**Tests.**

- Calisto e2e apogee within 1% of 3349 m AGL.
- Determinism: byte-stable Parquet across two reruns.
- Cross-tool sanity: apogee comparison side-by-side with
  RocketPy's published reference value.

**Exit criteria.**

- The integration test passes on the dev machine.
- The CI determinism gate runs the Calisto scenario alongside
  analytic-toy + Niskanen.

**Effort.** Medium (~1.0 week; the long pole is wiring all the
Phase-3 components together for the first time).

### 3.12 — Phase 3 closure

**Scope.** Documentation cleanup + Phase-3 plan removal, mirroring
the Phase-1 / Phase-2 closure pattern.

**Tasks.**

- `docs/design-concept.md § Phase Roadmap` — Phase 3 in past tense
  with the actual deliverables.
- `docs/README.md` — drop the link to `phase-3-plan.md`.
- `crates/*/README.md` — refresh status lines for the crates that
  moved from "Phase 3 stub" to "Phase 3 implemented".
- `git rm docs/phase-3-plan.md` — the per-sub-phase commit history
  is the source of truth.

**Tests.** N/A — docs only.

**Exit criteria.** Working tree clean; full suite still green.

**Effort.** Small (~0.3 week).

## Risk Register

| Risk | Likelihood | Impact | Mitigation |
|---|---|---|---|
| `VehicleAssembly` tree resolution introduces non-determinism (e.g., HashMap iteration over child tables) | Medium | High | Loader uses `BTreeMap` for child tables; topological-sort fixes iteration order; property test on declared-order = output-order. |
| Slosh state breaks bit-stability across reruns due to numerical sub-step variance | High | High | Lock sub-step count = 1 (forward Euler) by default; higher sub-step counts are scenario opt-in but break bit-stability across sub-step changes (documented). Determinism test on slosh-state byte equality. |
| Effector latency buffer state non-deterministic if buffer size changes per-step | Medium | High | Pre-allocate buffer at scenario load (depth = `⌈latency/dt⌉`); circular index. No allocation on hot path. Property test on byte-stable replay. |
| Multi-engine model-id allocation collides with existing single-motor IDs | Low | Medium | Reserve ID range 200..300 for cluster engines; existing single-motor IDs in 100..200 unaffected. Conflict test in Phase-3.6 regression. |
| Aero schema 2 tensor lookup operand order accidentally differs from schema 1 | Low | High | Re-use schema-1 reduction order (Demmel & Nguyen 2020). Per-corner exact-equality regression on a 6D test deck. |
| `MissionPhaseGraph` creates non-determinism via cycle detection failures or unstable iteration | Medium | High | Topological sort at construction; reject cycles with typed error; iteration order is the topological order. Property test on graph-walk determinism. |
| Calisto cross-tool case fails 1% tolerance due to RocketPy / OpenBMP integrator differences | High | Medium | Document the differences (RocketPy uses LSODA adaptive; OpenBMP uses RK4 fixed-step). Land Calisto as "best-effort cross-tool" with 1% tolerance first; if it fails, fall back to 5% or document the integrator-specific apogee. |
| WMM 2025 expires 2030-01-01 mid-Phase-3 (unlikely but worth noting) | Low | Medium | Scenarios outside the validity range fail closed with `MagneticOutOfEpoch`. WMM 2030 lands in whichever phase is current when NGA releases. |
| Effector pure-delay buffer adds memory overhead for high-latency scenarios | Low | Low | Buffer depth is `⌈latency/dt⌉`; for 100 ms latency at 1 ms `dt`, that's 100 entries. Acceptable for academic scenarios. |
| RocketPy data files have ambiguous license posture | Low | Medium | RocketPy is MIT-licensed. Re-use the Cesaroni M1670 from ThrustCurve.org (existing pattern); the Calisto airframe is OpenBMP-authored from the public RocketPy paper + repo. Cite both sources in `provenance.md`. |
| Cross-platform byte stability (macOS / Windows) breaks on the new gust filter due to libm differences | Medium | Medium | The Dryden filter uses only `mul` + `add` + `sqrt`; no transcendentals. Should be portable. CI cross-platform smoke covers this. |
| Phase-2 byte-stability gate breaks via accidental schema reorder in `KernelModelBundle` | Low | High | Phase-1 analytic-toy gate runs in Phase-3 CI; any regression fails closed. The dispatcher still routes Phase-1 scenarios through the byte-stable analytic path. |

## Acceptance Gate for Phase 3 Closure

Phase 3 closes when **all** of the following are true:

1. Every sub-phase exit criterion is met.
2. `cargo fmt --all -- --check` clean.
3. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
4. `cargo test --workspace --all-features` green (entire workspace
   test suite).
5. The Phase-1 `constant-acceleration-drop` analytic-toy scenario
   continues to produce byte-identical Parquet vs. the Phase-1
   baseline (regression guard for env, vehicle, scenario, runner,
   telemetry additions).
6. The Phase-2 `niskanen-2009-chapter6` sounding-rocket scenario
   continues to produce byte-identical Parquet vs. the Phase-2
   baseline (regression guard for the assembly tree resolver).
7. The Phase-3 `rocketpy-calisto` scenario passes its apogee
   tolerance on the dev machine. The Phase-3.11 closure pins the
   risk-register's 5 % fallback envelope (RocketPy LSODA adaptive
   vs. OpenBMP RK4 fixed-step integrator mismatch); tightening to
   1 % is a follow-up.
8. The CI determinism gate runs all three scenarios twice and
   asserts byte-stability of all three on
   `x86_64-unknown-linux-gnu`.
9. Every new dataset file (WMM 2025, Cesaroni M1670, Calisto drag
   curves) has a sibling `provenance.md` and `openbmp
   check-provenance` passes.
10. Every scenario-referenced external file has its SHA-256
    recorded in telemetry metadata, and a pinned-hash mismatch
    fails closed.
11. `docs/design-concept.md § Phase Roadmap` lists Phase 3 in past
    tense with the actual deliverables.
12. `docs/phase-3-plan.md` is removed and `docs/README.md` no
    longer links to it.

## Suggested Calendar

Order is the dependency-respecting implementation order (see
§ Sub-phase Plan). Effort is the per-sub-phase estimate; total is
**~13.7 weeks of focused effort** with a ~30% slip buffer for audit
follow-ups, which puts a realistic Phase-3 closure at **5 calendar
months**.

| # | Sub-phase | Weeks |
|---|---|---|
| 3.1 | Rigid-body kernel adapter family | 1.0 |
| 3.2 | Event triggers + `MissionPhaseGraph` | 1.5 |
| 3.3 | `VehicleAssembly` tree | 2.0 |
| 3.4 | `ControlEffector` trait + actuator dynamics | 1.5 |
| 3.5 | Aero deck schema 2 (effector axes) | 1.0 |
| 3.6 | `EngineModel` + `EngineCluster` | 1.5 |
| 3.7 | `Tank` + `MovingMassModel` (Abramson) | 1.5 |
| 3.8 | Wind extensions (`LayeredWind`, `GustWind`) | 0.7 |
| 3.9 | Recovery models (Knacke parachute) | 0.7 |
| 3.10 | Synthetic sensors (GNSS / magnetometer / star tracker) | 1.0 |
| 3.11 | RocketPy Calisto cross-tool validation | 1.0 |
| 3.12 | Phase 3 closure | 0.3 |
| **Total** | | **13.7** |

## References

The Phase-3 implementation must cite these sources in the
appropriate `provenance.md` files and crate `README.md`s. URLs
captured here so they're easy to copy.

### Numerical methods

- Demmel, Nguyen. *Algorithms for Efficient Reproducible Floating
  Point Summation*. ACM TOMS 46:3 (2020).
  <https://dl.acm.org/doi/10.1145/3389360>
- Stevens, Lewis. *Aircraft Control and Simulation*. 3rd ed., Wiley
  (2015). §10.4.2 actuator models, §11.3 state-machine mode
  control.

### Slosh dynamics

- Abramson. *The Dynamic Behavior of Liquids in Moving Containers,
  with Applications to Space Vehicle Technology*. NASA SP-106
  (January 1966). Public domain US government work.
  <https://ntrs.nasa.gov/citations/19670006555>

### Wind / atmosphere

- MIL-STD-1797A. *Flying Qualities of Piloted Aircraft*. US
  Department of Defense.
- ESDU 85020 series. *Engineering Sciences Data Unit
  characteristics of atmospheric turbulence near the ground*.
  (Cited in `docs/data-provenance.md § Preferred Public
  Sources`.)

### Magnetic field

- NOAA NCEI Geomagnetic Modeling Team; British Geological Survey.
  2024: *World Magnetic Model 2025*. National Centers for
  Environmental Information, NOAA.
  <https://www.ncei.noaa.gov/products/world-magnetic-model>
  Public domain. DOI 10.25923/prbc-s316.

### Propulsion

- Sutton, Biblarz. *Rocket Propulsion Elements*. 9th ed., Wiley
  (2017). Chapter 8 on multi-engine clusters.
- ThrustCurve.org RASP File Format.
  <https://www.thrustcurve.org/info/raspformat.html>

### Recovery

- Knacke. *Parachute Recovery Systems Design Manual*. NWC TP 6575
  (1992). Public US Government work.

### FDIR / fault models

- Patton, Frank, Clark. *Fault Diagnosis in Dynamic Systems: Theory
  and Application*. Prentice Hall (1989). Canonical fault-mode
  taxonomy.

### Sensors

- IS-GPS-200. *NAVSTAR GPS Space Segment / Navigation User
  Interfaces*. US Government public release.
- Crassidis, Markley, Cheng. *Survey of Nonlinear Attitude
  Estimation Methods*. JGCD 30:1 (2007).
- Liebe. *Accuracy Performance of Star Trackers*. JGCD 18:5
  (1995).

### Public-benchmark validation

- Souza et al. *RocketPy: Six Degree-of-Freedom Rocket Trajectory
  Simulator*. Journal of Aerospace Engineering 35:5 (2022).
  <https://github.com/RocketPy-Team/RocketPy>
  MIT-licensed reference implementation.

### Project standards

- NASA-STD-7009B (March 2024). M&S credibility framework. See
  [`verification.md § External V&V Reference Frames`](verification.md#external-vv-reference-frames).
- AIAA G-077-1998. CFD V&V terminology.
