# OpenBMP Software Architecture

## Scope

This document is the technical architecture for **OpenBMP** — a Rust-first,
simulation-only research platform for rigid-body dynamics with a primary focus
on rocket-class and launch-vehicle-class flight simulation. The architecture
optimizes for **deterministic simulation, modular model replacement, strong
testability, and explicit safety boundaries**.

OpenBMP intentionally excludes hardware integration, deployable flight code,
real bus protocols, real device drivers, real-time deployment guarantees,
targeting, terminal guidance to real-world locations, real fielded-vehicle
parameter sets, and operational mission planning. See
[safety-boundaries.md](safety-boundaries.md) for the full acceptance/rejection
list and [design-concept.md](design-concept.md) for the project framing.

## Architecture at a Glance

| Concern | Decision |
|---|---|
| Language | Rust 2024 edition, MSRV pinned per release |
| Math | nalgebra (f64) |
| Units | uom (typed quantities) at API boundaries |
| Time | `SimTime` newtype; lockstep, no wall-clock |
| Frames | Type-tagged: `ECI`, `ECEF`, `NED`, `ENU`, `Body` |
| Frame/time policy | Explicit frame profiles; WGS84 default; optional pinned epoch data |
| Default integrator | RK4 fixed-step |
| Optional integrators | DOPRI5, DOPRI853, RKF78, implicit source-term profiles (behind explicit profile flags) |
| Concurrency in kernel | Synchronous, single-threaded |
| Async | Only for optional socket-bridge tooling, never in kernel |
| Scenario format | In-house, TOML-shaped, versioned, reject unknown fields |
| Scenario lint | `openbmp check` validates schema, provenance, safety names, determinism |
| Telemetry channels | Typed in-process bus (software bus pattern) |
| Telemetry archive | Parquet with explicit unit/frame metadata |
| Testing | proptest, insta, cargo-fuzz, criterion, golden CSV |
| Data provenance | Required `provenance.md` for every shipped data source |
| Supply chain | `cargo deny`, `cargo audit`, `cargo machete`, CycloneDX SBOM for releases; `cargo vet` once the audit ledger lands |
| Hardware path | None shipped (HIL pattern is a generic socket bridge) |

## Crate Selection and Rationale

The choice of crates is part of the architecture because it determines what
"in-house" means in practice and what we delegate to the wider Rust ecosystem.

| Concern | Crate | Rationale |
|---|---|---|
| Linear algebra | **nalgebra** | f64 default, full feature set, generic over scalar, integrates with `argmin` / `ndarray`. Compile-time slower than `glam` but precision and feature breadth matter more than SIMD speed for an academic simulator. |
| Units | **uom** | De-facto type-safe units crate. Quantity-based (length / mass / time, not unit-specific), zero-cost, supports many storage types. Used at public API boundaries; internal hot loops may use raw `f64`. |
| Quaternion / rotation | nalgebra (`UnitQuaternion`) | Standard aerospace double-precision quaternion. We do not use rotors (ultraviolet) — non-standard interop. |
| Serialization | **serde** | Universal. CSV, JSON, TOML, RON via `serde` derives. |
| Scenario file format | **toml** (via serde) | Human-friendly, deterministic ordering, supports comments, easy to version. |
| Telemetry archive | **parquet** + **arrow** (`apache/arrow-rs`) | Columnar, queryable from Python/duckdb/polars, embeds schema and metadata. Aligned with InfluxDB 3 Rust-on-Arrow direction. |
| Property testing | **proptest** | Explicit `Strategy` objects, per-value shrinking. Stronger than quickcheck for invariant-heavy code. |
| Snapshot tests | **insta** | Telemetry summaries, error messages, structured-output regressions; `cargo insta review` workflow used to accept or reject baseline shifts. |
| Fuzzing | **cargo-fuzz** + libFuzzer | Scenario parser, model config parser. |
| Microbenchmarks | **criterion** | Kernel hot path, integrator step, sensor packet generation. |
| Optional socket bridge wire format | **postcard** | `no_std`, compact, schema-friendly. Used only in optional HIL tooling, never in kernel. |
| Async (tooling only) | **tokio** | Reserved for optional socket bridge or CLI tooling. **Banned in `openbmp-core`, `openbmp-sim`, model crates, controller crates.** |
| CLI | **clap** | Standard, derive-based, self-documenting. |
| Logging | **tracing** | Structured logs distinct from telemetry. Telemetry is the simulation output; tracing is for diagnostic/dev logging. |
| Error handling | **thiserror** for libraries, **anyhow** for binaries | Explicit error types in libraries, ergonomic propagation in CLI. |

Development and release tooling:

| Concern | Tool | Rationale |
|---|---|---|
| Dependency policy | **cargo-deny** | License, advisory, banned-crate, duplicate, and source checks in CI. |
| Dependency audits | **cargo-vet** | Planned ledger for third-party Rust dependency audits by trusted reviewers. |
| SBOM | **cargo-cyclonedx** | Produces CycloneDX SBOMs for release artifacts. |
| Build provenance | **SLSA-style attestations** | Records builder, source, inputs, and artifact hashes for releases. |

**Banned patterns** in the deterministic kernel:
- `std::time::SystemTime`, `Instant`, or any wall-clock source.
- Threaded random number generation that reads from OS entropy.
- Iteration over `HashMap<K, V>` without explicit ordering — use `BTreeMap` or
  insertion-ordered structures (e.g., `IndexMap` with locked iteration).
- Floating-point reductions whose result depends on associativity choices —
  document and lock the reduction order.
- Threads, atomics, or anything with cross-thread non-determinism.

## System Layers

```text
+------------------------------------------------------------------------+
| L7  CLI, scenarios, reports, examples, documentation                   |
+------------------------------------------------------------------------+
| L6  Scenario parser, model registry, validation rules, fault injector  |
+------------------------------------------------------------------------+
| L5  Telemetry channels, ring buffer, archive writers (Parquet/CSV/JSON)|
+------------------------------------------------------------------------+
| L4  Virtual flight controller: estimator, autopilot, mission FSM,      |
|     FDIR, academic guidance                                            |
+------------------------------------------------------------------------+
| L3  Synthetic sensors, fault models, command/actuator stubs            |
+------------------------------------------------------------------------+
| L2  Environment models (atm/grav/wind/mag), force / moment / mass      |
|     providers, vehicle composition                                     |
+------------------------------------------------------------------------+
| L1  Simulation kernel: scheduler, propagator, integrator, event queue, |
|     state storage                                                      |
+------------------------------------------------------------------------+
| L0  Math, units, frames, time, deterministic RNG, error types          |
+------------------------------------------------------------------------+
```

Lower layers are pure and deterministic. Upper layers handle files,
reports, and user interaction. Cross-layer dependencies point downward only.

## Workspace Layout

```text
openbmp/
├── Cargo.toml                           # workspace
├── rust-toolchain.toml                  # pinned working toolchain
├── crates/
│   ├── openbmp-core/                    # L0: math, units, frames, time, RNG
│   ├── openbmp-sim/                     # L1: kernel, scheduler, integrators
│   ├── openbmp-state/                   # L1: state types (point-mass, rigid)
│   ├── openbmp-env/                     # L2: atmosphere, gravity, wind, mag
│   ├── openbmp-vehicle/                 # L2: rigid body, mass models,
│   │   ├── assembly/                    #     VehicleAssembly tree (Phase 3)
│   │   ├── effector/                    #     ControlEffector trait (Phase 3)
│   │   └── tank/                        #     TankModel + MovingMassModel (Phase 3)
│   ├── openbmp-aero/                    # L2: aero decks + hypersonic methods
│   ├── openbmp-aerothermal/             # L2: heat transfer, BL, thermal toy
│   ├── openbmp-propulsion/              # L2: motors, EngineModel + EngineCluster
│   │                                    #     (Phase 3)
│   ├── openbmp-sensors/                 # L3: synthetic sensors, fault models
│   ├── openbmp-fc/                      # L4: virtual flight controller
│   │   ├── estimator/                   #     EKF, MEKF
│   │   ├── autopilot/                   #     three-loop, gain-scheduled
│   │   ├── mission/                     #     phase state machine
│   │   ├── guidance/                    #     academic guidance laws
│   │   └── fdir/                        #     fault detection/isolation
│   ├── openbmp-telemetry/               # L5: channels, ring, exporters
│   ├── openbmp-scenario/                # L6: parser, validator, registry
│   ├── openbmp-testkit/                 # L6: helpers (proptest strategies,
│   │                                    #     analytic-toy fixtures, fuzzers)
│   ├── openbmp-cli/                     # L7: scenario runner, diff tool
│   └── openbmp-bridge/                  # L7 (optional): socket HIL bridge
├── docs/
│   ├── README.md
│   ├── design-concept.md
│   ├── software-architecture.md
│   ├── safety-boundaries.md
│   ├── data-provenance.md               # source records and data review
│   ├── frames-time.md                   # frames, epochs, EOP, WGS84 policy
│   ├── scenario-format.md               # (Phase 1)
│   ├── verification.md                  # (Phase 1)
│   ├── supply-chain.md                  # dependency and release policy
│   ├── modeling-guide.md                # (Phase 5)
│   ├── real-rocket-integration.md       # downstream-user assembly cookbook
│   └── glossary.md                      # (Phase 1)
├── scenarios/
│   ├── analytic-toy/                    # closed-form validation
│   ├── public-benchmark/                # academic reference cases
│   └── examples/                        # docs-test scenarios
├── data/
│   ├── motors/                          # synthetic motor curves
│   ├── atmosphere/                      # public-coefficient tables
│   └── gravity/                         # truncated harmonic coefficients
├── tests/
│   ├── golden/                          # byte-stable telemetry
│   └── validation/                      # analytic + benchmark validation
└── benches/                             # criterion benchmarks
```

Crates expose narrow APIs. Each has its own `tests/` directory for crate-local
integration tests; cross-crate scenario tests live in the workspace `tests/`
folder.

## Core Types

`openbmp-core` exports the foundational vocabulary used by every other crate.

### Time

```rust
/// Monotonic simulation time. Not a wall-clock value.
#[derive(Copy, Clone, PartialOrd, PartialEq, Debug)]
pub struct SimTime(f64);                  // seconds since scenario t=0

#[derive(Copy, Clone, PartialOrd, PartialEq, Debug)]
pub struct Duration(f64);                 // seconds

#[derive(Copy, Clone, PartialOrd, PartialEq, Debug)]
pub struct StepIndex(u64);                // monotonic step counter
```

`SimTime` is monotonic, advances only via the kernel's `step()`, and is the
only time source visible to any model. There is no `std::time::Instant` use
inside the kernel or any model crate (enforced by lint).

### Frames

```rust
/// Compile-time frame tag. Position/Velocity types are parametric in F.
pub trait Frame: 'static + Copy + Send + Sync {
    const ID: FrameId;
}

pub struct ECI;
pub struct ECEF;
pub struct NED;
pub struct ENU;
pub struct Body;

#[derive(Copy, Clone)]
pub struct Position3<F: Frame>(pub Vector3<f64>, PhantomData<F>);

#[derive(Copy, Clone)]
pub struct Velocity3<F: Frame>(pub Vector3<f64>, PhantomData<F>);

#[derive(Copy, Clone)]
pub struct Quaternion<From: Frame, To: Frame> {
    pub q: UnitQuaternion<f64>,
    _p: PhantomData<(From, To)>,
}
```

Frame mismatches are caught **at compile time**: you cannot add a `Position3<NED>`
to a `Position3<ECEF>`. Conversion is explicit:

```rust
pub trait FrameTransform<From: Frame, To: Frame> {
    fn transform_position(&self, t: SimTime, p: Position3<From>) -> Position3<To>;
    fn transform_velocity(
        &self,
        t: SimTime,
        v: Velocity3<From>,
        p: Position3<From>,
    ) -> Velocity3<To>;
}
```

A `FrameContext` (snapshotted at scenario time) holds the canonical
transforms (Earth rotation rate, geodetic origin, etc.) used by
`FrameTransform` instances at simulation time.

The detailed frame and time convention policy is maintained in
[frames-time.md](frames-time.md). The short rule is that scenarios either use
an explicit toy frame profile or pin every absolute-time table needed for
Earth-fixed transforms.

### Quantities

Public APIs use `uom`-typed quantities for all dimensional inputs and outputs:

```rust
use uom::si::f64::{Mass, Force, Length, Velocity, Time, Pressure};

pub struct ForceOutput<F: Frame> {
    pub force: Vector3<Force>,            // typed
    pub frame: PhantomData<F>,
}
```

Internal hot loops may use raw `f64` for performance, but conversion to/from
typed quantities happens at the trait boundary. This is the "Mars Climate
Orbiter" guard — unit mismatches surface at compile time.

### State

```rust
pub struct PointMassState {
    pub time: SimTime,
    pub position: Position3<ECI>,
    pub velocity: Velocity3<ECI>,
    pub mass: Mass,
}

pub struct RigidBodyState {
    pub time: SimTime,
    pub position: Position3<ECI>,
    pub velocity: Velocity3<ECI>,
    pub orientation: Quaternion<Body, ECI>,
    pub angular_velocity: AngularVelocity3<Body>,
    pub mass_props: MassProperties,
}

pub struct MassProperties {
    pub mass: Mass,
    pub center_of_mass_body: Position3<Body>,
    pub inertia_body: Matrix3<MomentOfInertia>,
}
```

ECI is the canonical inertial frame for state propagation. Body-frame is the
canonical frame for vehicle-fixed quantities. All other frames are derived via
time-aware `FrameTransform` calls or explicit `FrameContext` helpers such as
the WGS84/NED local-origin transforms.

### Determinism Primitives

```rust
/// Deterministic RNG seeded from (scenario_seed, step_index, channel_id).
pub struct DeterministicRng { /* ChaCha8 internally */ }

impl DeterministicRng {
    pub fn for_channel(scenario_seed: u64, step: StepIndex, channel: ChannelId) -> Self;
}
```

The same `(scenario_seed, step, channel)` always produces the same RNG stream.
This lets us generate noise per-sensor per-step deterministically and replay
byte-identical telemetry across runs.

## Simulation Kernel

`openbmp-sim` owns time progression and state propagation in **lockstep**:
each step, the kernel applies deterministic events, advances effectors
from held controller commands, evaluates models, integrates state, runs
sensors, calls the controller, and emits telemetry. The controller never
advances faster than the kernel, and the kernel never advances faster
than the controller.

### Step Pseudocode

```rust
pub fn step(&mut self) -> Result<(), SimulationError> {
    let active = self.schedule.active_groups(self.step_index);

    // 1. Apply deterministic events and advance effectors from the command
    // held over from the previous controller tick. This avoids an algebraic
    // loop from controller -> effector -> force -> state -> sensor -> controller
    // within one base tick.
    let t = self.state.time;
    self.events.apply_due(&mut self.state, t, self.step_index)?;
    self.effectors.step(&self.cmd, self.dt)?;

    // 2. Sample environment at current state.
    let env = self.env.sample(EnvironmentQuery::from(&self.state));

    // 3. Evaluate forces, moments, mass-property update.
    let force = self.force.force(ForceInput::new(&self.state, &env, &self.effectors));
    let moment = self.moment.moment(MomentInput::new(&self.state, &env, &self.effectors));
    let mass_dot = self.mass.derivative(self.state.time);

    // 4. Integrate (RK4 fixed-step or alternative).
    self.state = self.integrator.advance(&self.state, force, moment, mass_dot, self.dt)?;

    // 5. Generate synthetic sensor measurements from new truth when due.
    let measurements = self.sensors.sample_if_due(
        &self.state,
        self.rng.for_step(self.step_index),
        active.sensor_groups_due(),
    );

    // 6. Tick virtual flight controller when due. The resulting command is
    // consumed by effectors on the next base tick and is zero-order-held
    // between controller ticks.
    if active.controller_due() {
        self.cmd = self.fc.update(VirtualControlInput::new(&measurements, self.state.time))?;
    }

    // 7. Publish telemetry channels (truth, sensors, commands, mission phase).
    if active.telemetry_due() {
        self.telemetry.publish_step(&self.state, &measurements, &self.cmd, self.step_index);
    }

    // 8. Run validation rules; check stop conditions.
    self.validation.check(&self.state)?;
    self.step_index = self.step_index.next();
    self.state.time = self.state.time + self.dt;
    Ok(())
}
```

### Responsibilities

The kernel:
- Loads validated scenario configuration.
- Initializes deterministic model instances in a stable order.
- Advances the simulation schedule (fixed-step by default).
- Calls environment, force, moment, sensor, controller, and telemetry layers
  in a fixed order.
- Applies the selected propagator.
- Publishes state snapshots and sensor / command channels to telemetry.
- Enforces stop conditions: end time, invalid state (NaN/Inf, negative mass),
  scripted abort.

The kernel does **not**:
- Use wall-clock time.
- Make real-time guarantees.
- Generate real actuator packets.
- Open network sockets, files, or environment variables during a step.
- Touch the OS clock or RNG.

### Scheduling

The default schedule is a single fixed step `dt` for everything.
**Multi-rate scheduling** is supported via per-subsystem rate groups
expressed as integer divisors of the base step. The scheduler resolves
the rate plan once at scenario start and the kernel walks the same
fixed list every step:

```rust
// openbmp-sim::schedule
pub struct RatePlan {
    pub base_hz: NonZeroU32,                  // master tick (e.g., 1000 Hz)
    pub base_dt: Duration,                    // derived once from base_hz
    pub groups: Vec<RateGroup>,
}

pub struct RateGroup {
    pub label: &'static str,                  // "env" | "fc" | "telemetry" | ...
    pub hz: NonZeroU32,                       // must divide base_hz exactly
    pub divisor: NonZeroU32,                  // every N base ticks
    pub members: Vec<SubsystemId>,
}

impl RatePlan {
    /// Resolve into a flat per-base-tick `Vec<Vec<SubsystemId>>` so each
    /// `step()` is a constant-time list walk. No HashMap lookups and no
    /// rate-decision branching inside the hot loop.
    pub fn flatten(&self) -> Vec<Vec<SubsystemId>>;
}
```

Scenario syntax:

```toml
[schedule]
base_hz = 1000                                # 1 kHz master tick

[[schedule.group]]
label = "env"
hz = 100                                      # divisor = 10
members = ["atmosphere", "gravity", "wind"]

[[schedule.group]]
label = "fc"
hz = 50                                       # divisor = 20
members = ["estimator", "autopilot", "fdir", "mission_fsm"]

[[schedule.group]]
label = "effector"
hz = 200                                      # divisor = 5
members = ["effector_bank"]

[[schedule.group]]
label = "telemetry"
hz = 20                                       # divisor = 50
members = ["telemetry"]
```

Rules:

- Every group's `hz` must divide `base_hz` evenly (`base_hz % hz == 0`).
  The loader rejects non-integer divisors at scenario load time, not at
  runtime. Floating-point equality is never used to decide a schedule.
- The dynamics integrator runs every base tick — it is implicitly the
  highest rate. Anything below the integrator rate is a sub-rate.
- Rate-group ordering inside a single base tick is fixed and documented
  in [Step Pseudocode](#step-pseudocode): events and effectors first,
  then environment / forces / moments / mass, then integrator, then
  sensors, controller, telemetry. Controller output is consumed by the
  next effector tick and held between controller ticks. Same order on
  every tick that activates the group.
- Multi-rate output remains **bit-stable** because the schedule is
  resolved at scenario start and the kernel walks the same list every
  tick.

### Event / Phase Timeline

Events and phases are first-class scheduler inputs alongside rate
groups. They drive things like staging, engine start / shutdown,
parachute deploy, mission-phase transitions, and effector deflection
schedules:

```rust
// openbmp-sim::events
pub trait EventTrigger {
    /// Evaluated once per base tick (or per group tick if a divisor is set).
    fn fired(&self, state: &VehicleState, t: SimTime, step: StepIndex) -> bool;
}

pub enum BuiltInEventTrigger {
    AtTime(SimTime),
    AtAltitudeAscending { meters: f64 },
    AtAltitudeDescending { meters: f64 },
    AtApogee,
    AtMassFraction { remaining: f64 },
    AtDynamicPressure { pa: f64, falling: bool },
    Scripted { label: &'static str },         // fires when scenario sets the flag
}

pub struct EventBinding {
    pub trigger: BuiltInEventTrigger,
    pub action: EventAction,
    pub once: bool,                           // most events fire exactly once
}

pub enum EventAction {
    EnterPhase(PhaseId),
    EngineCommand { engine: EngineId, cmd: EngineCommand },
    EffectorOverride { id: EffectorId, command: f64 },
    Separation(SeparationEvent),
    DeployRecovery(RecoveryDeviceId),
    EmitTelemetryMarker { tag: &'static str },
}

pub struct MissionPhaseGraph {
    pub phases: Vec<Phase>,
    pub transitions: Vec<PhaseTransition>,
}

pub struct Phase {
    pub id: PhaseId,
    pub label: &'static str,                  // "ascent" | "coast" | "powered_descent" | ...
    pub allowed_effectors: Vec<EffectorId>,   // optional gate
    pub allowed_engines: Vec<EngineId>,
}
```

The graph and the event list together replace the Phase-1 ad-hoc
"hard-coded apogee detection in the kernel". The kernel keeps the same
fixed step shape and just consults the resolved event list each tick.

> **Phase-3.4 status note.** Phase 3.4 wires `ControlEffector` and
> the `EffectorOverride` action: a mission event whose action is
> `effector_override { id, command }` is recorded by the kernel,
> drained by the runner, and applied to the runner-side `EffectorRack`
> on the next rack tick before the kernel step. Override beats schedule
> for that rack tick only. Unknown effector ids are rejected during
> scenario validation. The `EventAction::EffectorOverride` enum
> variant carries `{ id: EffectorId, command: f64 }` (was a unit
> variant pre-3.4 — the runner consumes it, so the kernel never
> dispatches it itself).
>
> **Phase-3.2 status note (still current).** The Phase-3.2
> implementation in `openbmp-sim::events` ships every variant of
> `BuiltInEventTrigger` except `Scripted`, which is rejected at
> scenario parse time with a typed deferral error: scripted
> triggers are deferred to a later sub-phase, and the per-effector
> `command_schedule` covers the common scripted-command case.
> The remaining `EventAction` variants `EngineCommand`,
> `Separation`, and `DeployRecovery` exist in the enum (so 3.6 /
> 3.6 / 3.9 do not need to expand it) but are still
> parser-rejected. `AtDynamicPressure` is parser-rejected until a
> later sub-phase wires atmosphere into event evaluation.
> `EnterPhase`, `EmitTelemetryMarker`, `Stop`, and (Phase 3.4)
> `EffectorOverride` are wired end-to-end. `EventTrigger::fired`
> takes an `EventEvalState` snapshot rather than the full
> `VehicleState` shown above — the snapshot carries only the
> derived scalars triggers need (altitude, vertical velocity, mass
> fraction, dynamic pressure) plus the previous-step values for
> crossing detection.

## Numerical Integrators

`openbmp-sim` ships at minimum:

| Integrator | Order | Step | Use |
|---|---|---|---|
| **RK4** | 4 | fixed | Default. Byte-stable. Easy to reason about. |
| **DOPRI5** (Dormand-Prince 4(5)) | 5 | adaptive | Non-stiff scenarios needing local error control or event-gradient accuracy. Behind a `--profile=adaptive` flag. |
| **DOPRI853 / DOPRI8** | 8 | adaptive | High-accuracy trajectory and event-localization runs with dense output. |
| **RKF78** | 7/8 | adaptive | Independent high-order explicit RK cross-check for smooth hypersonic trajectory segments. |
| **Implicit Euler** | 1 | fixed sub-step | Deterministic stiff source-term baseline for chemistry / material-response submodels. |
| **Rosenbrock-Wanner** | varies | fixed/adaptive sub-step | Profile-gated stiff source-term solver for nonequilibrium chemistry. |
| **BDF** | 1-5 | adaptive | Research profile for stiff ODE systems; state-stable, never byte-stable. |
| **Analytic** | n/a | n/a | Toy validation: closed-form propagation for analytic-toy scenarios. |

RocketPy uses LSODA (auto-switching stiff/non-stiff) as default; OpenBMP
keeps RK4 fixed-step as canonical because LSODA-style adaptive switching
breaks bit-stable replay across runs that visit different stiffness
regimes. Adaptive integrators ship behind explicit profile flags only and
are tagged `state-stable, not bit-stable`.

DOPRI5/8, DOPRI853, and RKF78 are adaptive **explicit** Runge-Kutta methods.
They are useful for smooth trajectory propagation and event localization, but
they are not the stiff-chemistry answer for hypersonic nonequilibrium,
ablation chemistry, or tightly coupled aerothermal submodels. Those Phase-6
submodels use declared fixed sub-stepping with implicit Euler and, for harder
cases, profile-gated Rosenbrock-Wanner or BDF variants as described in
[hypersonic-extensions.md](hypersonic-extensions.md).

For hypersonic scenarios, the scenario selects a `SolverProfile` rather than
only an integrator. The profile declares the trajectory integrator, dense-output
event policy, stiff source-term solver, coupling policy, tolerances, fixed
sub-step counts, and determinism class. RK4 remains the canonical deterministic
baseline; it is not the upper bound of planned hypersonic fidelity.

```rust
pub trait Integrator {
    fn advance(
        &self,
        state: &RigidBodyState,
        force: ForceOutput<ECI>,
        moment: MomentOutput<Body>,
        mass_dot: MassDerivative,
        dt: Duration,
    ) -> Result<RigidBodyState, IntegratorError>;
}
```

The default profile is RK4 fixed-step for **deterministic byte-stable
replay**. Adaptive integrators are second-class: they enable profiles for
specific validation runs, but they break byte-stable telemetry across runs.
Each integrator declares this:

```rust
pub trait Integrator {
    fn determinism(&self) -> IntegratorDeterminism;
    // ...
}

pub enum IntegratorDeterminism {
    BitStable,    // RK4 fixed-step
    StateStable,  // adaptive: same input -> same final state, but step granularity may differ
}
```

## Model Interfaces

All physics is exposed through narrow Rust traits. Each implementation must
document assumptions, validity range, units, frames, and validation status.

### Environment

```rust
pub enum ModelEvalError {
    OutOfEnvelope { model: ModelId, reason: String },
    NonFinite { model: ModelId },
    InvalidState { model: ModelId, reason: String },
}

pub trait EnvironmentModel {
    fn sample(&self, q: EnvironmentQuery) -> Result<EnvironmentSample, ModelEvalError>;
    fn validation(&self) -> ValidationStatus;
}

pub struct EnvironmentQuery {
    pub time: SimTime,
    pub position: Position3<ECI>,        // converted internally as needed
}

pub struct EnvironmentSample {
    pub atmosphere: AtmosphereSample,    // density, pressure, temperature, speed of sound
    pub gravity: Vector3<Acceleration, ECI>,
    pub wind: Velocity3<NED>,
    pub magnetic_field: Vector3<MagneticFluxDensity, NED>,
}
```

Model evaluation is fallible at the kernel boundary. Envelope violations
(atmosphere ceiling, aero deck envelope, invalid motor time range, etc.) return
typed errors that the kernel records with the step and model id before halting;
they are not represented by `NaN`, silent clamping, or panics. Scenario
validation should catch obvious envelope mismatches up front, but runtime model
errors remain fail-closed.

### Force / Moment / Mass

```rust
pub trait ForceModel {
    fn force(&self, input: ForceInput) -> Result<ForceOutput<ECI>, ModelEvalError>;
    fn validation(&self) -> ValidationStatus;
}

pub trait MomentModel {
    fn moment(&self, input: MomentInput) -> Result<MomentOutput<Body>, ModelEvalError>;
    fn validation(&self) -> ValidationStatus;
}

pub trait MassModel {
    fn mass_properties(&self, time: SimTime) -> Result<MassProperties, ModelEvalError>;
    fn derivative(&self, time: SimTime) -> Result<MassDerivative, ModelEvalError>;
    fn validation(&self) -> ValidationStatus;
}
```

A vehicle is a **composition** of force/moment/mass models registered in the
scenario. The scenario file lists `force_models = ["aero", "gravity_force",
"thrust"]` and the kernel sums them in declared order each step.

## Environment Models

### Atmosphere

`openbmp-env::atmosphere`:
- `IsothermalAtmosphere` — `experimental` toy.
- `UsStandard1976` — implemented in-house from public coefficients;
  validity range 0–86 km; status `validated-toy` once cross-checked against
  published tables.
- `Nrlmsise00` (Phase 6) — public empirical model from Picone, Hedin, Drob
  (2002), *J. Geophys. Res.* 107(A12), 1468; ground to ~1000 km. The
  reference Fortran source is hosted at NASA's Community Coordinated
  Modeling Center (`ccmc.gsfc.nasa.gov`); OpenBMP re-implements in pure
  Rust from the public coefficients with a `provenance.md` entry. Each of
  the model's ~25 configuration flags is documented for its deterministic
  effect.
- `Nrlmsis2x` and `Hwm14` (Phase 6 follow-ons) — modern high-altitude
  atmosphere and horizontal-wind references. These are profile-gated until
  reference-table validation and provenance are complete.
- `EarthGramReference` (Phase 6 follow-on) — external-reference atmosphere
  profile for density / wind uncertainty envelopes when redistribution terms
  permit; not the default deterministic atmosphere.

```rust
pub struct AtmosphereSample {
    pub density: MassDensity,
    pub pressure: Pressure,
    pub temperature: ThermodynamicTemperature,
    pub speed_of_sound: Velocity,
}

pub trait AtmosphereModel {
    fn sample(&self, q: AtmQuery) -> AtmosphereSample;

    /// Optional real-gas / regime state for hypersonic / high-altitude flight.
    /// Default `None`; ideal-gas models do not implement.
    fn thermo_state(
        &self,
        q: AtmQuery,
        local_total_temperature: ThermodynamicTemperature,
        characteristic_length: Length,
    ) -> Option<RealGasState> { None }
}
```

The optional `thermo_state` method is the extension hook for hypersonic
real-gas thermodynamics, composition, effective gamma, and Knudsen-regime
classification. See [hypersonic-extensions.md](hypersonic-extensions.md).

### Gravity

- `ConstantGravity` — `g0` along NED `+Z`. Toy use only.
- `PointMassGravity` — `-µ/r² r̂`. Inertial, validated against analytic Kepler.
- `J2Gravity` — point-mass + J2 zonal harmonic. Public coefficients.
- `EgmTruncated` (deferred) — truncated EGM2008 to user-specified degree/order
  (max 36×36 in MVP). Provenance required.

### Wind and Magnetic Field

- `NoWind`, `ConstantWind`, `LayeredWind`, `GustWind` (academic profiles).
- Magnetic field models (deferred to Phase 5 unless a sensor needs them).

## Vehicle and Mass Models

```rust
pub trait Vehicle<S: SimState>: ForceModel<S> + MomentModel<S> {
    fn force_models(&self) -> &[Box<dyn ForceModel<S>>];
    fn moment_models(&self) -> &[Box<dyn MomentModel<S>>];
    fn mass_model(&self) -> &dyn MassModel;
    fn force_model_names(&self) -> Vec<String>;
    fn moment_model_names(&self) -> Vec<String>;
    fn evaluate_force_breakdown(
        &self,
        ctx: ForceContext<'_, S>,
    ) -> Result<ForceBreakdown, ModelEvalError>;
    fn evaluate_moment_breakdown(
        &self,
        ctx: MomentContext<'_, S>,
    ) -> Result<MomentBreakdown, ModelEvalError>;
}
```

Mass models:

- `ConstantMass` — fixed.
- `LinearBurn` — linear depletion from `m0` to `m_dry` over a burn time.
- `TableBurn` — interpolated mass / inertia from a synthetic table.
- `MultiStage` — composed of stages with separation events at scripted
  conditions; scenario-driven, not target-driven.

The flat `Vehicle` trait above is the single-stick MVP shape. Anything
larger than a textbook rocket — multi-engine boosters, multi-tank stages,
control-effector banks — composes through `VehicleAssembly` instead. The
flat trait remains supported as the trivial assembly degenerate case.

## VehicleAssembly Tree

Past a single rigid stick, vehicles are composed from typed sub-parts in a
tree: bodies, propulsion, effectors, tanks, sensors, mass properties.
`openbmp-vehicle::assembly` ships the trait surface; the scenario file
declares the tree and the loader resolves it into the kernel's flat
force / moment / mass / sensor lists at startup.

```rust
// openbmp-vehicle::assembly
pub trait VehicleAssembly {
    fn id(&self) -> VehicleId;

    /// Rigid bodies that make up this assembly.
    /// MVP: single body. Phase-5 multi-body: N bodies that may detach.
    fn bodies(&self) -> &[Body];

    /// Propulsion: motors, engines, engine clusters.
    fn propulsion(&self) -> &PropulsionTree;

    /// Control effectors: aero surfaces, gimbals, RCS thrusters,
    /// body flaps, grid fins. Each is rate/saturation/latency-bounded.
    fn effectors(&self) -> &[Box<dyn ControlEffector>];

    /// Tanks (and the moving-mass content inside them — slosh, baffles).
    fn tanks(&self) -> &[Tank];

    /// Sensor instances (truth-to-measurement converters).
    fn sensors(&self) -> &[Box<dyn SyntheticSensor>];

    /// Resolve into the flat MassProperties used by the kernel.
    fn mass_properties(&self, t: SimTime) -> MassProperties;

    /// Resolve into the kernel's force / moment / mass lists once.
    fn into_kernel_models(self) -> KernelModelBundle;
}

pub struct Body {
    pub id: BodyId,                   // referenced by tank/effector parents
    pub geometry: BodyGeometry,       // reference area, length, span, body axes
    pub dry_mass_kg: f64,
    pub dry_inertia_body: Matrix3<f64>,
}

pub struct KernelModelBundle {
    pub force_models: Vec<Box<dyn ForceModel>>,
    pub moment_models: Vec<Box<dyn MomentModel>>,
    pub mass_model: Box<dyn MassModel>,
    pub sensors: Vec<Box<dyn SyntheticSensor>>,
}
```

The tree is **declarative**: the scenario lists parts and parents, the
loader walks the tree once, and the kernel sees a flat model list. The
tree exists for authoring ergonomics and for the cookbook in
[real-rocket-integration.md](real-rocket-integration.md); the kernel hot
path stays flat-list and synchronous as today.

> **Phase-3.3 status note.** The Phase-3.3 implementation in
> `openbmp-vehicle::assembly` ships the trait surface, `Body`,
> `BodyGeometry`, `BasicAssembly`, `KernelModelBundle`,
> `KernelModelBundleRigid`, plus the path-derived stable ids
> (`BodyId`, `EffectorId`, `TankId`, `EngineId`, `VehicleId`) in
> `openbmp-core`. The `propulsion` / `effectors` / `tanks` /
> `sensors` accessors shown above are reserved for future phases
> (3.4 / 3.6 / 3.7 / 3.10) and are *not* part of the Phase-3.3
> trait surface. The `into_kernel_models` method shown above is
> also deferred — Phase-3.3 ships the resolver as a free-function
> bridge in `crates/openbmp-cli/src/runner/assembly.rs` rather
> than a trait method, avoiding an `openbmp-vehicle ->
> openbmp-scenario` dependency edge. Phase-3.3 runners consume the
> resolved assembly's dry mass properties during kernel mass
> construction; force / moment construction remains on the existing
> runner paths. Phase-3.4+ will land the full kernel-side bundle
> resolver as the assembly tree gains real propulsion / effector /
> tank content. The `[vehicle.assembly]` scenario block is documented in
> [`scenario-format.md § Vehicle assembly`](scenario-format.md#vehicle-assembly-phase-33).

### Propulsion: EngineModel and EngineCluster

Solid motors keep the existing `Motor: ForceModel + MassModel` shape from
the [Propulsion](#propulsion) section. Liquid engines and multi-engine
vehicles use the new `EngineModel` + `EngineCluster` traits:

```rust
// openbmp-propulsion::engine
pub trait EngineModel {
    fn id(&self) -> EngineId;

    /// Per-engine commanded throttle and gimbal axis angles.
    fn apply_command(&mut self, cmd: EngineCommand);

    /// Body-frame thrust force at the engine's mount point.
    fn thrust_body(&self, ctx: &EngineCtx) -> Vector3<Force>;

    /// Mass flow out of this engine right now (kg/s, ≥ 0).
    fn mass_flow_kg_per_s(&self, ctx: &EngineCtx) -> f64;

    /// Engine lifecycle: ignition allowed, shut down, hard-fail.
    fn state(&self) -> EngineState;

    fn validation(&self) -> ValidationStatus;
}

pub struct EngineCommand {
    pub throttle_unit: f64,           // 0.0..=1.0
    pub gimbal_pitch_rad: f64,
    pub gimbal_yaw_rad: f64,
    pub ignite: bool,
    pub shutdown: bool,
}

pub enum EngineState { Idle, Igniting, Burning, Shutdown, Failed }

/// N engines on a stage. The cluster owns mount geometry, summed thrust /
/// moment about the body origin, summed mass-flow, and per-engine fault
/// state. A Falcon-class 9-engine layout is one cluster, not nine
/// hand-summed force models.
pub struct EngineCluster {
    pub engines: Vec<Box<dyn EngineModel>>,
    pub mount_points_body: Vec<Position3<Body>>,
    pub layout: ClusterLayout,        // axial, ring, octaweb, etc.
}

impl ForceModel  for EngineCluster { /* sums per-engine thrust, body→ECI */ }
impl MomentModel for EngineCluster { /* sums per-engine moment about origin */ }
impl MassModel   for EngineCluster { /* d/dt(mass) = -Σ mdot_i */ }
```

`EngineCluster` is a force *and* moment *and* mass model — its body-frame
thrust at non-axial mount points contributes a moment about the body
origin without any extra wiring. Per-engine ignition and gimbal axes are
addressed by index in the controller's command bundle, so a 9-engine
shutdown after engine-out is a per-engine `shutdown = true`, not a
cluster-level rebuild.

> **Phase-3.6 status note.** Phase 3.6 ships:
>
> - `EngineModel` trait (`apply_command(cmd)`, `step(dt)`,
>   `limits()`, `inject_fault(fault)`, `current_state()`,
>   `current_snapshot()`, `id()`, `validation()`); `LiquidEngine`
>   reference impl with linear ignition / shutdown transients,
>   constant-throttle burn, mass flow `mdot = thrust / (g0 · Isp)`,
>   and locked-order pitch-then-yaw gimbal rotation. State machine
>   is one-shot: `Shutdown` and `Failed` are terminal.
> - Four canonical fault modes: `Stuck`, `HardOff`, `OverThrust`,
>   `GimbalLocked` (mirrors Phase-3.4 effector fault taxonomy).
>   Faults are scenario-loaded at construction; run-time injection
>   is deferred.
> - `EngineCommand` payload `{ throttle_unit, gimbal_pitch_rad,
>   gimbal_yaw_rad, ignite, shutdown }`. `EventAction::EngineCommand
>   { id, command }` is wired end-to-end: kernel records, runner
>   drains, `EngineRack::apply_commands` routes by id.
> - Architecture-spec'd "cluster as ForceModel + MomentModel +
>   MassModel" is split between propulsion and vehicle crates to
>   keep `openbmp-propulsion` L2 (no kernel dependency): the
>   propulsion-side `EngineCluster` is a pure container of
>   `Vec<Box<dyn EngineModel>>` + mount points; the kernel-side
>   adapter trio (`EngineClusterForceAdapter`,
>   `EngineClusterMomentAdapter`, `EngineClusterMassAdapter`) lives
>   in `openbmp-vehicle::adapters` and consumes a
>   per-step snapshot via `EngineSnapshotView` on
>   `ForceContext` / `MomentContext` / `MassContext`.
> - The runner-side `EngineRack` (in `openbmp-cli/src/runner/`)
>   owns the engines and pushes a fresh `BTreeMap<EngineId,
>   EngineSnapshot>` to the kernel before each `step()` so all four
>   RK4 stages see the same snapshot — same pattern as Phase-3.4
>   `EffectorRack` and Phase-3.5 `EffectorActualsView`.
> - `MassModel` gains `mass_kg_at(MassContext)` and
>   `mass_rate_kg_s_at(MassContext)` with default forwards to
>   `mass_kg(t)` / `mass_rate_kg_s(t)` so legacy models are
>   byte-identical; only `EngineClusterMassAdapter` overrides.
>   `BoxedMassModel` forwards both new methods through to its
>   inner box (a load-bearing fix caught by the e2e test's
>   mass-monotonicity assertion).
> - Single-motor and no-propulsion scenarios short-circuit every
>   rack-related operation on `engine_rack.is_empty()`; the
>   kernel's `engine_snapshot` field stays at the empty `BTreeMap`
>   set in `new()`, the cluster adapters are never instantiated,
>   and legacy scenarios produce byte-identical Parquet to pre-3.6.
> - Rigid-body engine-cluster moments wired in 3.6 via
>   `EngineClusterMomentAdapter`: per-engine `mount × thrust_body`
>   cross product, summed in scenario-declared order with locked
>   left-fold operand order. Rigid-body cluster mass-properties
>   evolution (inertia tensor as propellant is consumed) stays
>   deferred to Phase 3.7's tank-driven dynamics work.
> - Six-engine clusters (octaweb-style) work; the
>   `cluster_layout` enum carries through to telemetry but has no
>   behavioural effect in 3.6.
> - The Phase-3.6 exit-criterion scenario ships at
>   `scenarios/multi-engine-octaweb/four-engine-shutdown.toml`
>   and its e2e test at
>   `crates/openbmp-cli/tests/engine_cluster_e2e.rs`. The test
>   asserts mass strictly decreases (proves the kernel's RK4
>   integrator is consuming `mass_rate_kg_s_at` from the cluster
>   adapter), z-thrust drops by ~25 % after one of four engines
>   shuts down, and lateral x-thrust grows non-zero (proves
>   per-engine gimbal-applied `thrust_body` from the snapshot is
>   actually consumed by the cluster's force summation).

### Control Effectors

Control effectors are **how the controller talks to the physics**: aero
surfaces, gimbals, body flaps, grid fins, RCS thrusters, parachutes,
drogues. They are kept distinct from aero decks and engines so that rate
limits, saturation, latency, deadband, and fault modes live in one
place.

```rust
// openbmp-vehicle::effector
pub trait ControlEffector {
    fn id(&self) -> EffectorId;

    /// Per-step: take the controller's commanded position, apply rate
    /// limit / saturation / latency / deadband / fault model, advance
    /// internal effector state, and report the *actual* deflection /
    /// gimbal angle / valve position the physics sees.
    fn step(&mut self, cmd: f64, dt: Duration) -> EffectorState;

    /// Authority envelope for sanity checks (controller can ask).
    fn limits(&self) -> EffectorLimits;

    /// Inject a scenario-defined fault mode at runtime.
    fn inject_fault(&mut self, fault: EffectorFault) -> Result<(), EffectorError>;
}

pub struct EffectorLimits {
    pub min: f64,
    pub max: f64,
    pub max_rate_per_s: f64,
    pub deadband: f64,
    pub latency: Duration,
}

pub struct EffectorState {
    pub commanded: f64,               // what controller asked for
    pub actual: f64,                  // what physics gets
    pub saturated: bool,
    pub rate_limited: bool,
    pub fault: Option<EffectorFault>,
}

pub enum EffectorFault {
    Jam { at: f64 },                  // stuck at this value
    Runaway { rate_per_s: f64 },      // commanded ignored, drives at this rate
    ReducedRate { factor: f64 },      // 0..1
    Hardover { to: f64 },             // jumps to extreme then jams
}
```

Effectors feed the aerodynamics through the deck's effector axes (see
[Deck Format extensions](#deck-format-extensions-control-effector-axes))
and feed the propulsion through `EngineCommand.gimbal_pitch_rad /
gimbal_yaw_rad`. The controller does not see effector state directly; it
sees telemetry channels `effector.<id>.commanded`,
`effector.<id>.actual`, `effector.<id>.saturated`, etc.

> **Phase-3.4 + 3.5 status note.** Phase 3.4 shipped `ControlEffector`,
> the `LinearActuator` reference impl, the four canonical fault modes,
> the runner-side `EffectorRack`, and one `effector.<id>.actual`
> `f64` telemetry channel per declared effector. Phase 3.5 closed the
> deck-side consumption loop: schema-2 aero decks declare effector
> axes (e.g. `delta_e_deg`), the runner pushes the rack snapshot to a
> kernel-owned `BTreeMap<String, f64>` before each `step()`, and
> `ForceContext.effector_actuals: EffectorActualsView<'a>` exposes
> that snapshot to the deck adapter inside every RK4 stage. See
> [Phase-3.5 status note](#deck-format-extensions-control-effector-axes)
> in §Deck Format Extensions for the full schema-2 wire format and
> determinism contract.
>
> The implemented `step(cmd, dt)` returns
> `Result<EffectorState, EffectorError>` and `inject_fault(fault)`
> validates payloads and returns `Result<(), EffectorError>` (the
> design fragment above is simplified). Faults are scenario-loaded
> only in 3.4; run-time fault injection is deferred. The
> `EffectorRack` lives on the runner side (mirrors `mission.rs`),
> not on `BasicAssembly`, so the assembly stays `Clone` and legacy
> scenarios with no `[[vehicle.assembly.effectors]]` short-circuit
> every per-step rack operation on `is_empty()` and produce
> byte-identical Parquet to pre-3.4. See `scenarios/effector-elevon/`
> for the gravity-only exit-criterion case (3.4) and
> `scenarios/effector-elevon-aero/` for the schema-2-deck case (3.5).

### Tanks and Slosh as Moving-Mass Dynamics

Liquid propellant inside a tank is a moving mass: as the body
accelerates and rotates, the liquid sloshes, the CG shifts, and a
coupled-pendulum or equivalent moving-mass term loads the rigid body.
OpenBMP treats this as **generic moving-mass dynamics** rather than a
liquid-specific model, so the same machinery covers slosh, deployable
masses, and shifting payloads.

```rust
// openbmp-vehicle::tank
pub struct Tank {
    pub id: TankId,
    pub geometry: TankGeometry,       // cylinder / sphere / textbook ellipsoid
    pub mounted_to: BodyId,
    pub mount_point_body: Position3<Body>,
    pub propellant: PropellantSpec,   // toy / textbook only — no fielded data
    pub initial_fill_fraction: f64,
    pub baffle_model: Option<BaffleModel>,
    pub moving_mass: Box<dyn MovingMassModel>,
}

pub trait MovingMassModel {
    /// Update the moving-mass internal state (slosh angle/rate, etc.)
    /// given the current rigid-body acceleration and angular rate.
    fn step(&mut self, accel_body: Vector3<f64>, omega_body: Vector3<f64>, dt: Duration);

    /// Effective contribution to mass / CG / inertia at this moment.
    fn mass_contribution(&self) -> MassContribution;

    /// Reaction force / moment back on the parent body in body-frame.
    fn reaction_body(&self) -> ForceMomentBody;

    /// Drain rate (kg/s) commanded by the parent EngineCluster.
    fn drain(&mut self, kg_per_s: f64);
}

pub struct MassContribution {
    pub mass_kg: f64,
    pub cg_offset_body_m: Vector3<f64>,    // relative to tank mount point
    pub inertia_delta_body: Matrix3<f64>,  // additive to dry inertia
}
```

Available `MovingMassModel` implementations:

- `RigidLiquid` — toy, no slosh; the simplest baseline.
- `EquivalentPendulum` — Abramson-style equivalent pendulum; one or
  more pendulum modes per axis with textbook frequency / damping.
- `EquivalentSpringMass` — alternative equivalent representation for
  high-fill-fraction studies.
- `BaffledPendulum` — equivalent pendulum with `BaffleModel`-supplied
  damping increment.

OpenBMP-shipped reference implementations use **textbook propellant
ranges and textbook tank geometries only**. Upstream PRs that add real
fielded propellant data or real fielded tank geometry are rejected (see
[safety-boundaries.md](safety-boundaries.md)). Downstream users may bind
their own data through the real-data package path in their own
repositories.

> **Status — Phase 3.7 implemented.** The trait surface above ships
> in `crates/openbmp-vehicle/src/tank/`: `MovingMassModel`, `Tank`,
> `MassContribution`, `ForceMomentBody`, plus four implementations
> (`RigidLiquid`, `EquivalentPendulum`, `EquivalentSpringMass`,
> `BaffledPendulum`). The kernel-side `TankRackForceAdapter` /
> `TankRackMomentAdapter` / `TankRackMassAdapter` consume the
> per-step `TankSnapshotView` populated by the runner-side
> `TankRack` (mirror of the Phase-3.6 `EngineRack`). The
> Phase-3.7.E exit-criterion scenario at
> `scenarios/sloshing-tank/sloshing-tank.toml` exercises the full
> hot path on a rigid-body vehicle with an axial liquid engine and
> a cylindrical tank carrying an `EquivalentPendulum` slosh model.
>
> Phase-3.7 known limitations (each tracked for Phase-3.X follow-on):
> (1) tank drain is decoupled from the engine cluster — tanks
> declare a scalar `drain_rate_kg_per_s` rather than receiving the
> cluster's total mdot; (2) the `TankRackMassAdapter` adds each
> tank's `mass_kg` to the vehicle total without checking whether
> the cluster already accounts for the same propellant, so
> scenarios that declare both an engine cluster and a tank
> intended as its propellant store overcount mass; (3) the
> rigid-body cluster mass adapter (Phase-3.6 deferral) is still
> `ConstantMassRigid` — slosh inertia perturbations are published
> in the snapshot but are not consumed by the rigid mass-properties
> model; (4) slosh telemetry channels are deferred to a future
> phase, with determinism currently asserted via full-Parquet byte
> equality.
>
> Phase 3.7 shipped semi-implicit (symplectic) Euler rather than
> the explicit Euler that the original sub-phase planning called
> for: explicit Euler is unstable for the undamped harmonic
> oscillator (energy growth `(dt·ω)²` per step) and cannot meet
> the 1 %-over-100-cycles energy-conservation property test at any
> practical `dt`. The shipped implementation uses semi-implicit
> (symplectic) Euler with locked operand order — still single-step
> explicit, still bit-stable for fixed `dt`, and exact-conserves a
> modified Hamiltonian. The `EquivalentPendulum` and
> `EquivalentSpringMass` unit tests verify the 1 % gate at
> `dt = 1 ms` over 100 cycles. The per-sub-phase commit history
> (Phase 3.7.B / 3.7.C) is the source of truth for this decision.

### Multi-Body Separation Events

Phase-5 multi-body promotes separation to first class: after a separation
event, two or more `VehicleAssembly` instances fly simultaneously, each
with its own state vector, its own `ForceModel` / `MomentModel` /
`MassModel` lists, and its own controller (or no controller, for spent
stages). The kernel's environment sample is shared.

```rust
pub struct SeparationEvent {
    pub at: SeparationTrigger,        // time, altitude, mass-fraction, scripted
    pub split: SeparationSplit,
    pub momentum_exchange: Option<SeparationImpulse>,
}

pub enum SeparationTrigger {
    AtTime(SimTime),
    AtAltitude { meters: f64, ascending: bool },
    AtMassFraction { remaining: f64 },
    Scripted { label: &'static str },
}

pub struct SeparationSplit {
    pub upper: VehicleAssembly,       // continues with the controller
    pub lower: VehicleAssembly,       // becomes ballistic / spent stage
}

pub struct SeparationImpulse {
    pub upper_delta_v_body_m_s: Vector3<f64>,
    pub lower_delta_v_body_m_s: Vector3<f64>,
    /// Conservation check: m_u·Δv_u + m_l·Δv_l ≈ 0 (within tolerance).
    pub conserve_momentum: bool,
}
```

The Phase-3 scripted-staging hack remains supported for the single-body
"spent stage falls behind, ignored" case. The Phase-5 multi-body path
kicks in when the scenario declares more than one post-separation
assembly should be propagated.

## Aerodynamics

`openbmp-aero` handles aerodynamic force and moment computation. The crate
is structured around a pluggable **aero method** abstraction so that
tabulated decks, local-inclination methods, free-molecular methods, and
hybrids can all coexist:

```rust
pub trait AeroMethod {
    fn aero_force_moment(&self, ctx: &AeroContext) -> AeroForceMoment;
    fn validity(&self) -> AeroValidity;
}
```

The MVP ships `DeckLookup` (tabulated). Phase 6 adds hypersonic methods
(`ModifiedNewtonian`, `TangentCone`, `LocalInclinationPanels`,
`FreeMolecular`) and a `HybridAeroMethod` that dispatches between them
based on Mach and Knudsen number. See
[hypersonic-extensions.md](hypersonic-extensions.md) for details.

For the MVP deck path:

### Deck Format (in-house TOML)

```toml
openbmp.aero_deck = 1

reference.length_m = 0.5
reference.area_m2  = 0.196
provenance = "synthetic textbook example, Anderson Ch. 8"
validation = "experimental"

[grid]
mach  = [0.0, 0.5, 0.8, 1.2, 2.0, 3.0]
alpha_deg = [-10.0, -5.0, 0.0, 5.0, 10.0]
beta_deg  = [0.0]

[coefficients.cn]
# (mach, alpha, beta) -> CN
data = [ /* row-major 3D table */ ]

[coefficients.cd]
data = [ /* ... */ ]

[coefficients.cm]
data = [ /* ... */ ]
```

Trilinear interpolation is the default. Out-of-grid samples produce a
`ValidationError` (fail-closed) unless the deck explicitly opts into
extrapolation with a documented strategy (e.g., `extrapolation = "clamp"`).

Schema 1 is the reduced axisymmetric form. `beta_deg` may be present as a
grid axis for deck-family continuity, but there is no separate `CY`, yaw
moment, or body-`y` force channel in this schema: the body-frame mapping is
`F_body = q · S · (-CD, 0, -CN)` and `M_body = q · S · L · (0, CM, 0)`.
For non-zero beta grids, a deck author either encodes the intended reduced
normal-force behaviour into `CN(M, alpha, beta)` or accepts that Schema 1 does
not model side force/yaw moment. Full six-coefficient decks are the Phase 3
schema extension.

Decks must include provenance and a validation label. **Real fielded-vehicle
aero decks are explicitly rejected.** The MVP ships only synthetic textbook
decks for canonical shapes (sphere, cone, simple finned cylinder).

Users may pre-process aero coefficients into the deck format using public
semi-empirical methods (Barrowman 1967; the OpenRocket / Sampo Niskanen
2009 master's thesis, *Development of an Open-Source Model Rocket
Simulation Software*, Helsinki University of Technology, is the canonical
academic reference for the Barrowman extended component build-up applied
to model-rocket aerodynamics). OpenBMP itself does not ship a Barrowman
pre-processor; the deck is the boundary.

### Deck Format Extensions: Control-Effector Axes

The basic deck above is `(Mach, alpha, beta) → coefficients`. When the
dataset was built with control-effector deflections varied, the deck
adds extra axes:

```toml
openbmp.aero_deck = 2                 # axes-with-effectors schema

reference.length_m = 1.250
reference.area_m2  = 1.227
provenance = "synthetic ARV-Reference winged-second-stage example"
validation = "experimental"

[grid]
mach        = [0.3, 0.6, 0.9, 1.2, 1.6, 2.0, 3.0, 5.0]
alpha_deg   = [-5, 0, 5, 10, 15, 20]
beta_deg    = [-4, 0, 4]
delta_e_deg = [-20, -10, 0, 10, 20]   # body-flap pitch deflection
delta_a_deg = [-15, 0, 15]            # body-flap roll deflection
delta_r_deg = [0]                     # not used on this airframe

[axis_order]
# locked reduction order — part of determinism contract
order = ["mach", "alpha", "beta", "delta_e_deg", "delta_a_deg", "delta_r_deg"]

[coefficients.cn]
data = [ /* row-major 6D table */ ]

[coefficients.cm]
data = [ /* row-major 6D table */ ]

[interpolation]
method = "multilinear"
extrapolation = "error"               # fail-closed; control surfaces saturate
                                      # via the ControlEffector layer instead
```

The deck reports **influence coefficients** as a function of effector
position; the `ControlEffector` layer reports **what the effector is
actually doing** (rate-limited, saturated, possibly faulted). The
multiplication of the two gives the live aerodynamics. Decks built
without effector axes degrade to the schema-1 `(Mach, alpha, beta)` form
and the loader treats effector deflections as untracked.

Effector axes follow the `(positive deflection)` sign convention
declared in the deck provenance sidecar (see
[data-provenance.md § Real-Data Package Credibility Format](data-provenance.md#real-data-package-credibility-format)
for the full sidecar schema). Axis names are not prescribed beyond the
common `delta_e / delta_a / delta_r` and `body_flap_left /
body_flap_right / grid_fin_<n>`; the deck declares names and units, the
`ControlEffector` set declares names and limits, the loader matches.

> **Phase-3.5 status note.** Phase 3.5 wires the schema-2 deck format
> end-to-end:
>
> - The deck file's schema discriminator is the integer marker:
>   `openbmp.aero_deck = 1` (schema-1) or `= 2` (schema-2). The
>   earlier draft form `openbmp.aero_deck.schema = 2` was discarded:
>   actual TOML semantics prevent an integer + sub-table at the same
>   dotted key, so the shipping wire format bumps the integer instead.
> - The runtime `AeroDeck` is N-D internally: an `axis_order:
>   Vec<String>` (always starting with `["mach", "alpha", "beta"]`),
>   `axes: Vec<Vec<f64>>`, and three flat row-major coefficient
>   `Vec<f64>`s. At N = 3 the multilinear lookup is bit-identical to
>   the Phase-2.5 trilinear path — pinned by a 1024-case property test
>   in `openbmp-aero::deck`.
> - The lookup signature is
>   `lookup(mach, alpha, beta, deflections: &BTreeMap<&str, f64>) ->
>   Result<AeroCoefficients, AeroError>`. Schema-1 decks ignore
>   `deflections`. Schema-2 decks require an entry per declared
>   effector axis; missing-key returns `AeroError::InvalidParameter`.
> - The kernel owns a `BTreeMap<String, f64>` snapshot
>   (`SimulationKernel::set_effector_actuals` setter); the runner
>   refreshes it before every `step()` call so all four RK4 stages
>   see the same view. `ForceContext` and `MomentContext` carry an
>   `EffectorActualsView<'a>` (a `Copy` borrow of the kernel-owned map
>   wrapped in `Option`) that the deck adapter reads. Empty snapshot
>   for legacy schema-1 scenarios → empty view → schema-1 lookups
>   never touch the view → byte-stable legacy code paths.
> - Unit-matching contract: schema-2 deck axis names carry an explicit
>   `_deg` or `_rad` suffix. The runner's `aero_effector_match`
>   helper pairs each deck axis with a scenario effector by stripped
>   name and asserts the effector's declared `unit` matches the
>   suffix. Duplicate stripped deck-axis names and unit mismatches →
>   `CliError::AeroEffectorMismatch` at runner build time, before any
>   kernel step.
> - Schema-2 supports up to 3 effector axes (6 axes total) in 3.5.
>   The full six-coefficient `(CY, Cl, Cn-yaw)` deck is deferred past
>   3.5; schema-2 still ships only `(CN, CD, CM)`. The
>   `AxialDragForceAdapter` keeps its hard-coded `(alpha, beta) =
>   (0, 0)` query in 3.5; real alpha/beta consumption from
>   `RigidBodyState` is Phase-3.6's
>   `RigidAeroForceMomentAdapter` work.
> - The exit-criterion scenario lives at
>   `scenarios/effector-elevon-aero/single-elevon-aero-deflected.toml`
>   with the schema-2 deck at `data/aero/synthetic-elevon-1d.toml`.
>   The Phase-3.5.D e2e test in
>   `crates/openbmp-cli/tests/effector_aero_e2e.rs` runs both a
>   deflected scenario and a baseline (elevon held at 0°), asserts
>   the per-step `force.aero.z_n` telemetry differs by ≥ 5% between
>   the two — proof the schema-2 path is actually consuming the
>   live deflection.

### Aero Force / Moment Computation

```rust
pub struct AeroModel {
    deck: AeroDeck,
    reference: AeroReference,
}

impl ForceModel for AeroModel {
    fn force(&self, input: ForceInput) -> Result<ForceOutput<ECI>, ModelEvalError> {
        let q = dynamic_pressure(input.atmosphere, input.airspeed_body);
        let (mach, alpha, beta) = airdata(input.airspeed_body, input.atmosphere);
        let coeffs = self.deck.lookup(mach, alpha, beta)?;
        let f_body = aero_force_body(q, &self.reference, &coeffs);
        let f_eci = transform_to_eci(f_body, input.orientation);
        Ok(ForceOutput { force: f_eci, /* ... */ })
    }
}
```

## Propulsion

`openbmp-propulsion` models thrust and propellant mass flow.

### Motor Format (in-house TOML)

```toml
openbmp.motor = 1

[meta]
name = "synthetic-solid-A"
provenance = "synthetic, designed for OpenBMP analytic-toy validation"
validation = "validated-toy"          # experimental | checked | validated-toy | research

[burn]
duration_s = 4.0
total_impulse_n_s = 2444.5
specific_impulse_s = 226.60875296586778  # total_impulse = m_p · g0 · Isp
propellant_mass_kg = 1.10
dry_mass_kg = 0.40

[thrust_curve]
# (time_s, thrust_n) pairs
points = [
  [0.00, 0.0],
  [0.05, 750.0],
  [0.50, 720.0],
  [3.50, 580.0],
  [4.00, 0.0],
]

[geometry]
exit_area_m2 = 0.0019
ambient_pressure_correction = "constant"   # toy
```

The format is RASP-shaped (rocket motor format common in amateur rocketry)
but is an OpenBMP in-house re-implementation. Existing public `.eng` files
may be imported via a **pure data converter** with provenance preserved;
no real fielded-motor curves ship in the repository.

### Motor Variants

```rust
pub trait Motor: ForceModel + MassModel {
    fn ignite_at(&self) -> SimTime;
    fn burn_duration(&self) -> Duration;
    fn variant(&self) -> MotorVariant;
}

pub enum MotorVariant { Solid, Liquid, Hybrid, ColdGas }
```

Multi-stage support is a `MultiStageMotor` that owns child `Motor`s and a
sequence of `SeparationEvent`s in the mission state machine.

## Synthetic Sensors

`openbmp-sensors` converts simulated truth state into simulated noisy
measurements. **No device drivers, no real bus protocols, no real sensor
parameters.**

### Sensor Trait

```rust
pub trait SyntheticSensor {
    type Measurement;
    fn sample(&self, truth: &RigidBodyState, env: &EnvironmentSample, rng: &mut DeterministicRng)
        -> Self::Measurement;
    fn rate(&self) -> Frequency;
    fn validation(&self) -> ValidationStatus;
}
```

### Available Sensors

| Sensor | Measurement | Noise Model |
|---|---|---|
| `IdealStateSensor` | full truth (test only) | none |
| `SyntheticImu` | body accel + body angular rate | IEEE 952-style inertial-sensor noise decomposition: ARW / VRW, bias instability, RRW, quantization, and scale-factor error |
| `SyntheticBarometer` | pressure → altitude | additive Gaussian + bias drift |
| `SyntheticGnss` | ECI position + velocity | per-axis additive Gaussian on position + velocity, OU bias drift on position |
| `SyntheticMagnetometer` | body-frame magnetic field | additive Gaussian, constant 3×3 soft-iron, constant 3-vector hard-iron |
| `SyntheticStarTracker` | quaternion attitude | small-angle Gaussian rotation-vector perturbation, isotropic per axis |

Noise parameters come from **academic published budgets** (e.g., textbook
Allan-variance specs for "tactical-grade" or "consumer-grade" classes); they
are not lifted from any real fielded sensor's spec sheet. Sensor models
declare their noise class explicitly.

> **Implementation status (Phase 3.10).** WMM 2025 ships as the
> canonical magnetic-field truth at
> `data/magnetic/WMM.COF` (NOAA NCEI / NGA / UK DGC, December 2024
> release; SHA-256-pinned in `data/magnetic/provenance.md`). The
> [`Wmm2025`](../crates/openbmp-env/src/magnetic/wmm2025.rs)
> implementation is a direct port of the NOAA reference algorithm
> (Gauss-recursion with Schmidt-multiplied coefficients; all 100
> shipped NOAA reference rows match within 5 nT per component).
> All three Phase-3.10 synthetic sensors —
> [`SyntheticGnss`](../crates/openbmp-sensors/src/gnss.rs),
> [`SyntheticMagnetometer`](../crates/openbmp-sensors/src/magnetometer.rs),
> and [`SyntheticStarTracker`](../crates/openbmp-sensors/src/star_tracker.rs)
> — ship with truth-bypass exact equality, per-component RNG
> independence, byte-stable replay, and empirical-stddev
> convergence within 5 % of their declared budgets.
>
> Phase-3.10 known limitations (each tracked for Phase-3.X
> follow-on): (1) GNSS is a receiver-output noise model only, no
> pseudorange / satellite geometry / ionosphere; (2) magnetometer
> reads `SensorTruth.magnetic_field_body_nt` pre-rotated by the
> runner from WMM-truth NED via the truth attitude — the
> magnetometer itself is frame-agnostic; (3) star tracker uses
> the small-angle quaternion form (`σ ≤ 0.01 rad` enforced at
> construction); (4) no fault models on the new sensors (Phase-4
> controller-side concern).

### Fault Models

Faults are scenario-injected, not sensor-internal:

```rust
pub trait FaultModel {
    fn apply(&self, time: SimTime, channel: ChannelId, value: &mut Measurement);
}

pub struct StuckFault { /* freeze a channel after t */ }
pub struct DropoutFault { /* drop messages for a window */ }
pub struct BiasShiftFault { /* step bias at scripted t */ }
pub struct NoiseSpikeFault { /* increased variance for a window */ }
```

Faults are declared in the scenario and applied between the truth-to-sensor
and sensor-to-controller boundaries. They exercise the controller's FDIR
path.

## Recovery and Descent Models

Academic sounding-rocket and HPR scenarios need simple descent modeling, but
OpenBMP must avoid operational landing or payload-delivery optimization.
Recovery models live as simulator-local force/event components, not as
hardware outputs.

> **Implementation status (Phase 3.9).** Recovery is implemented as a
> force-only runner rack plus kernel snapshot. Devices do not
> contribute mass, moments, mount-point torques, or landing-target
> guidance. Drag is evaluated against ECI velocity, matching the
> existing axial-drag adapter; wind-relative canopy drag and inflation
> transients remain future work.

MVP-plus recovery models:

- `ParachuteDrag` - drag area changes after a scripted deployment event.
- `DrogueMainRecovery` - two-stage academic descent model with scenario
  deployment conditions.
- `DragDevice` - generic airbrake or drag-device coefficient change.

Deployment events are driven by simulator-observable state such as apogee,
altitude threshold, elapsed time, or mission phase. They are not driven by
targets, impact points, or terminal objectives.

Telemetry channels:

| Channel | Units | Notes |
|---|---|---|
| `recovery.<id>.deployed` | bool | Per recovery device |
| `recovery.<id>.phase_index` | 1 | `0 = Stowed`, `1 = Drogue`, `2 = Main` |
| `recovery.<id>.drag_area_m2` | m² | Effective drag area |

Validation cases include constant-density terminal-velocity checks,
deployment-event ordering, and fail-closed behavior for invalid deployment
conditions.

## Virtual Flight Controller

`openbmp-fc` is the controller framework. It is **simulator-local**: every
output is consumed by a simulator-internal model or by the optional generic
socket bridge. There are no real bus protocols, no real device drivers, and
no targeting / terminal-homing logic.

Top-level structure:

```rust
pub trait VirtualFlightController {
    fn update(&mut self, input: FcInput) -> Result<FcOutput, FcError>;
    fn validation(&self) -> ValidationStatus;
}

pub struct FcInput {
    pub time: SimTime,
    pub measurements: SensorBundle,
    pub mission_phase: MissionPhase,
    pub references: ReferenceState,    // scripted, not target-driven
}

pub struct FcOutput {
    pub commands: NormalizedCommands,  // abstract, consumed by sim actuators
    pub estimate: StateEstimate,
    pub mission_phase: MissionPhase,
    pub fdir_status: FdirStatus,
}
```

Internally, the controller is composed of four sub-modules.

### Estimator

Sensor fusion / state estimation. Available implementations:

- `IdealEstimator` (pass-through truth, test only).
- `Ekf` — standard Extended Kalman Filter for position/velocity.
- `Mekf` — Multiplicative EKF for quaternion attitude.
- `Ukf` — Unscented Kalman Filter (Phase 4+).

```rust
pub trait Estimator {
    fn predict(&mut self, dt: Duration, control_estimate: ControlEstimate);
    fn update(&mut self, measurements: &SensorBundle);
    fn estimate(&self) -> StateEstimate;
}
```

Each implementation documents the process model, measurement models, noise
covariances, and validation status. The `Ekf` implementation follows the
canonical 15-state error-state academic formulation (position, velocity,
attitude, accelerometer bias, gyroscope bias) used in NaveGo and the
NorthStarUAS `insgnss_tools` library; magnetometer-augmented variants are
a Phase-4 option. Filter validation borrows the side-by-side
filter-comparison harness pattern from those projects: two filters run on
the same scenario with different noise settings and the testkit emits a
compare report. References: NaveGo (Rodríguez et al., MATLAB/Octave),
INSTINCT (University of Stuttgart, C++), `insgnss_tools` (NorthStarUAS,
C++/Python) — re-implemented in Rust, not imported.

### Autopilot

Three-loop architecture (inner rate, outer attitude, command):

```rust
pub trait Autopilot {
    fn update(&mut self, estimate: &StateEstimate, reference: &ReferenceState, dt: Duration)
        -> NormalizedCommands;
}

pub struct ThreeLoopAutopilot {
    rate_loop: PidLoop,                 // body rate -> command
    attitude_loop: PidLoop,              // attitude error -> rate command
    outer_loop: TrajectoryFollower,      // reference state -> attitude command
    gain_schedule: GainSchedule,         // gains as f(Mach, q-bar, altitude)
}
```

The three-loop architecture (inner rate, outer attitude, outer command) is
the canonical academic formulation in Stevens, B. L. and Lewis, F. L.,
*Aircraft Control and Simulation* (Wiley, 3rd ed., 2015). Gains are
scenario-supplied. The OpenBMP repository ships only **academic** gain
sets for canonical toy vehicles; no real fielded tuning data is included.
The trait surface, however, accepts any scenario-supplied gain table —
downstream consumers may integrate production gain sets in their own
repositories under their own export-control posture (see
[Extensibility for Downstream Integration](#extensibility-for-downstream-integration)).
Gain scheduling tables are part of the scenario file with explicit
provenance.

### Mission State Machine

Phase-based academic mission FSM:

```rust
pub enum MissionPhase {
    PreLaunch,
    Ascent,
    Coast,
    Apogee,
    Descent,
    Recovery,
    Safed,
}

pub trait MissionStateMachine {
    fn transition(&mut self, input: MissionInput) -> Option<MissionPhase>;
    fn current(&self) -> MissionPhase;
}
```

Transitions are driven by **simulator-observable conditions** (apogee
detected from estimated vertical velocity, descent detected from negative
altitude rate, recovery from descent + altitude threshold). They are
**not** driven by target acquisition, terminal homing, or any real-world
location. Vocabulary is borrowed from BPS.space Signal flight computer
patterns; implementation is in-house and simulator-only.

### Academic Guidance

Guidance laws shipped:

- `AttitudeHold` — track a scripted attitude reference.
- `RateHold` — track a scripted body-rate reference.
- `WaypointTrack` — track a sequence of scripted waypoints in inertial space
  (waypoints are scenario-defined points, not real-world locations or
  targets).
- `GravityTurnReference` — pre-computed pitch profile for textbook ascent
  studies.

**Explicitly not shipped:** proportional navigation, augmented PN, sliding-mode
homing, target-tracking guidance, terminal-homing logic, intercept geometry,
or any guidance law whose stated purpose is to strike a real-world point.
This is a permanent restriction; any pull request adding such code is
rejected on safety grounds.

### FDIR

```rust
pub trait Fdir {
    fn evaluate(&mut self, sensors: &SensorBundle, estimate: &StateEstimate) -> FdirStatus;
}

pub struct FdirStatus {
    pub flags: FaultFlags,
    pub recommended_action: RecoveryAction,
}

pub enum RecoveryAction {
    None,
    SwitchToSecondary(SensorId),
    EnterSafeMode,
    AbortMission,
}
```

FDIR responds to faults that the **scenario injects** (see Sensor § Fault
Models). The MVP includes simple residual-based fault detection; later
phases may include parity-space or Kalman-innovation tests.

## Aerothermal (Phase 6)

`openbmp-aerothermal` is the heat-transfer, boundary-layer, and surface
thermal-state crate. It is **not** part of the MVP; it is a Phase-6 research
extension for hypersonic / re-entry studies.

```rust
pub trait HeatTransferModel {
    fn stagnation(&self, ctx: &AerothermalContext) -> StagnationHeating;
    fn distributed(
        &self,
        ctx: &AerothermalContext,
        station: BodyStation,
        bl_state: BoundaryLayerState,
    ) -> SurfaceHeating;
    fn validation(&self) -> ValidationStatus;
}

pub trait BoundaryLayer {
    fn state(&self, station: BodyStation, ctx: &AerothermalContext) -> BoundaryLayerState;
}
```

Implementations include Fay-Riddell stagnation heating, Sutton-Graves
engineering simplification, Tauber-Sutton radiative heating, reference-
enthalpy distributed-heating, and a 1-D explicit-FTCS thermal-conduction
toy. Sub-phases 6.10 and 6.11 add Park two-temperature nonequilibrium
thermochemistry (via the `NonequilibriumAir` trait, plugged in through the
`AtmosphereModel::thermo_state` extension hook) and a generic surface
ablation toy (`AblationModel` trait with steady-state and charring
variants, textbook materials only). Full design and validation suite in
[hypersonic-extensions.md](hypersonic-extensions.md).

Aerothermal output flows through dedicated telemetry channels
(`aerothermal.q_stag_conv`, `aerothermal.q_stag_rad`,
`aerothermal.surface_temp[i]`, etc.) defined in the telemetry schema.

## Telemetry

Telemetry is the primary output of the platform.

### Two-tier Architecture

1. **Hot path: typed in-process channels.** Each module declares the
   channels it publishes (truth state, raw sensor, filtered sensor,
   control command, mission phase, fault flag). Subscribers register
   compile-time-known interest.
2. **Archive path: ring buffer + Parquet exporter.** A bounded ring buffer
   captures published values; the exporter flushes them to a Parquet file
   with explicit unit/frame metadata in column descriptors.

```rust
pub trait TelemetrySink {
    fn publish<T: TelemetryValue>(&mut self, channel: ChannelId, t: SimTime, value: T);
}

pub struct ParquetExporter {
    schema: ParquetSchema,                // includes unit/frame metadata
    writer: ParquetWriter<File>,
}
```

CSV and JSON exporters remain available for human readability and
golden-test diffability. Parquet is the **canonical archive format** because
it preserves typed columns, units, frames, and version metadata; CSV/JSON are
**lossy exports** of the canonical archive.

### Schema Versioning

```toml
openbmp.telemetry = 1

[schema.truth_state]
fields = ["time_s", "px_m", "py_m", "pz_m", "vx_m_s", "vy_m_s", "vz_m_s",
          "qw", "qx", "qy", "qz", "wx_rad_s", "wy_rad_s", "wz_rad_s",
          "mass_kg"]
frame = "ECI for position/velocity, Body for angular_velocity"
```

Schema version is part of the exported file. Consumers that don't
understand the version refuse to load.

### Golden Tests

Golden tests assert byte-stable Parquet (or CSV) output for canonical
scenarios. The diff command (`openbmp-cli diff <golden> <actual>`)
reports the first divergent row with field-level context. This is the
primary regression line.

## Scenario Format

The scenario format is in-house, TOML-shaped, versioned, and rejects unknown
fields by default.

### Example

```toml
openbmp.scenario = 1

[meta]
name = "rk4-attitude-damping-toy"
description = "Torque-free rigid body with damping autopilot."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s  = 30.0
dt_s    = 0.01
seed    = 42

[vehicle]
kind = "rigid_body"
mass_kg = 10.0
inertia_diag_kg_m2 = [1.0, 2.0, 1.5]
initial_position_eci_m = [7000000.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]
initial_quaternion_body_eci = [1.0, 0.0, 0.0, 0.0]
initial_angular_velocity_body_rad_s = [0.5, 0.3, -0.2]

[environment]
gravity = "j2"
atmosphere = "us_standard_1976"
wind = "constant"
wind_velocity_ned_m_s = [5.0, 0.0, 0.0]

[forces]
models = ["gravity", "aero"]

[aero]
deck = "data/aero/cone-toy.toml"

[sensors.imu]
class = "synthetic_tactical"
allan_variance.arw_deg_per_sqrt_h = 0.05
allan_variance.bias_stability_deg_per_h = 5.0
rate_hz = 200

[fc]
estimator = "mekf"
autopilot = "three_loop"
mission = "academic_phase_fsm"
guidance = "attitude_hold"

[fc.gains]
attitude_kp = 4.0
attitude_kd = 1.0

[fc.references]
target_attitude_quat_body_eci = [1.0, 0.0, 0.0, 0.0]

[telemetry]
output.parquet = "out/run-001.parquet"
output.csv = "out/run-001.csv"
golden.path = "tests/golden/rk4-attitude-damping-toy.parquet"

[validation]
require_normalized_quaternion = true
require_finite_state = true
require_monotonic_time = true
```

### Parsing Rules

- Format version is mandatory and the parser rejects unknown major versions.
- Unknown fields produce a parse error (fail-closed). Allows future
  additions without breaking determinism silently.
- All units are explicit in field names (`mass_kg`, `dt_s`, `vx_m_s`).
- Frames are explicit in field names (`position_eci_m`, `attitude_quat_body_eci`).
- Files referenced from a scenario are resolved relative to the scenario
  file path.

A separate `scenario-format.md` document maintains the formal grammar; the
parser includes a fuzz target to ensure malformed scenarios produce
diagnostics, not panics.

## Real-Data Package Format

External datasets (aero decks, engine performance tables, mass / inertia
builds, sensor-noise budgets, controller gain schedules) ship as
**real-data packages**: the dataset file alongside a strict
credibility-metadata sidecar. The kernel-side loader:

- Reads the sidecar first, validates schema, and refuses unknown fields.
- Recomputes each shipped file's `sha256` and refuses on mismatch.
- Parses the dataset and binds it to the declared units, frames, and
  reference geometry — no implicit conversions, no axis re-ordering, no
  silent unit promotion.
- Records the package id / version / hash / credibility levels into the
  scenario telemetry header so a downstream consumer can reproduce the
  run from the archive alone.
- Hard-rejects scenario queries outside the package's `envelope`. The
  `extrapolation` policy (`error` default, `clamp` and `linear`
  opt-in) is consulted only inside the envelope.

The complete sidecar schema, field semantics, loader behaviour, and the
list of rejected packages live in
[data-provenance.md § Real-Data Package Credibility Format](data-provenance.md#real-data-package-credibility-format).
The architecture-level commitment here is just that this is the
**only** path for external data in OpenBMP-shipped models: ad-hoc CSV /
JSON / TOML loaded directly into a model is rejected in upstream code
review. Downstream users can write their own loaders, but they lose the
OpenBMP package-level credibility and replay metadata unless they follow
this contract.

The cookbook in
[real-rocket-integration.md](real-rocket-integration.md) shows the full
end-to-end pattern using the fictional ARV-Reference vehicle: scenario
file, package layout, sidecar, loader behaviour, telemetry header.

## SIL Test Harness

The SIL harness runs the kernel + models + sensors + virtual FC in-process,
single-threaded. Every scenario in `scenarios/` is runnable as both a
real-time-decoupled simulation and a regression test:

```bash
openbmp run scenarios/examples/attitude-damping-toy.toml \
  --output out/run-001.parquet
openbmp diff tests/golden/attitude-damping-toy.parquet out/run-001.parquet
```

CI runs the full SIL test corpus on every PR.

### Determinism CI gate

A dedicated CI job runs the canonical scenario set twice on the reference
platform profile and asserts byte-identical Parquet output. The job runs
in two configurations: with `tracing` at the default subscriber, and with
the subscriber redirected to `/dev/null`, to verify that diagnostic
logging side-effects do not leak into deterministic outputs. Failures
here block merging.

The reference platform profile is `x86_64-unknown-linux-gnu` with the
pinned working toolchain from `rust-toolchain.toml` and the `default` simulation
profile. Other platform profiles (macOS, ARM Linux, Windows) are
exercised in nightly CI as `state-stable, not bit-stable`: cross-platform
diffs are expected; intra-platform-profile bit identity is required.

## HIL Pattern (Optional, Generic)

The HIL pattern is **optional** and lives in `openbmp-bridge`. It is **not**
a hardware integration framework. It exposes a **generic socket bridge**
that emits simulated sensor packets and accepts abstract normalized command
packets, using an in-house `postcard`-encoded wire format.

```rust
// Wire types (postcard-encoded over UDP or TCP)
#[derive(serde::Serialize, serde::Deserialize)]
pub struct BridgeSensorPacket {
    pub schema_version: u32,
    pub time_s: f64,
    pub imu: ImuMeasurement,
    pub baro: BaroMeasurement,
    pub gnss: Option<GnssMeasurement>,
}

#[derive(serde::Serialize, serde::Deserialize)]
pub struct BridgeCommandPacket {
    pub schema_version: u32,
    pub time_s: f64,
    pub commands: NormalizedCommands,    // abstract values in [-1, 1] ranges
}
```

The bridge ships:
- An in-house wire format definition.
- A reference Rust client library that connects, sends commands, receives
  sensor packets.

The bridge does **not** ship:
- MAVLink, DDS, MIL-STD-1553, CAN, I2C, SPI, UART, or any real bus protocol.
- Real device drivers for any sensor or actuator.
- Pre-built integrations with PX4, ArduPilot, or any flight-software stack.
- Protocol adapters that translate the in-house bridge format into any real
  bus, flight-stack, or hardware command format.

Users who want to integrate with a specific external system are responsible
for writing that integration **outside the OpenBMP repository** under their
own license, governance, and export-control posture.

## Testing Taxonomy

| Class | Tool | Scope | Frequency |
|---|---|---|---|
| Unit | `cargo test` | Math, units, frames, parsers | Every PR |
| Property | proptest | Invariants (finite, monotonic, normalized, deterministic) | Every PR |
| Snapshot | insta | Telemetry summaries, error messages | Every PR |
| Golden scenario | custom diff | Byte-stable Parquet output | Every PR (small set), nightly (full) |
| Analytic-toy validation | custom | Closed-form comparison | Every PR |
| Public-benchmark validation | custom | Published academic cases | Nightly |
| Fuzz | cargo-fuzz | Scenario parser, motor parser, aero parser | Nightly |
| Doc tests | rustdoc | Examples in module docs | Every PR |
| Microbenchmarks | criterion | Kernel hot path, integrator step | Nightly, regression-tracked |

CI determinism budget: every PR runs the full unit + property + snapshot +
small golden + analytic-toy suites in under a fixed wall-clock budget
(target: under 5 minutes). Nightly runs the full corpus.

Following the RocketPy validation-table convention, every public-benchmark
validation case publishes its expected reference data alongside an
absolute-error column, a relative-error column, and a declared tolerance
envelope. The published table is the definition of "validation passed";
CI rejects regressions that move any error metric outside its declared
envelope. Each table is committed alongside the test in
`tests/validation/<case>/expected.toml` with a `provenance.md` entry
naming the public source.

## Determinism Profile

A *platform profile* is the tuple `(target_triple, rustc_version,
simulation_profile)` declared in CI. Byte-identical replay is guaranteed
within a platform profile; cross-platform-profile diffs are expected and
are documented in the determinism CI gate as `state-stable, not
bit-stable`. The reference platform profile is
`x86_64-unknown-linux-gnu`, the pinned working toolchain from `rust-toolchain.toml`,
and `default` simulation profile.

The default determinism profile guarantees:

- Fixed time step `dt`.
- Single-threaded kernel.
- Stable iteration order across all `BTreeMap` / `IndexMap` traversals.
- No `HashMap` iteration in the kernel.
- Seeded ChaCha8 RNG per `(scenario_seed, step_index, channel_id)`.
- No wall-clock or system RNG access in the kernel or any model.
- Stable floating-point reduction order (no Kahan-summation switching, no
  parallel reductions).
- Stable JSON / Parquet field ordering.
- No use of `f32::mul_add` / `f64::mul_add` (FMA) on hot deterministic
  paths unless the FMA emission is locked by build flags. Cross-CPU FMA
  pathway differences silently break bit-stable replay.

Performance optimizations must not change deterministic outputs unless they
introduce a new explicit profile flag (`--profile=adaptive`, etc.).

### Reproducible Numerics Profile

The reference bit-stable profile also pins:

- Rust toolchain version and target triple.
- `target-cpu` and `target-feature` settings for release artifacts.
- Floating-point precision (`f64` for physics unless a model documents
  otherwise).
- FMA policy: either forbidden on deterministic hot paths or explicitly
  enabled in a named profile.
- Math-function policy: deterministic kernels avoid platform-libm-dependent
  transcendental functions unless the profile pins the implementation.
- Reduction order for all vector sums, force sums, telemetry aggregates, and
  validation metrics.
- Serialization ordering for JSON, TOML-generated artifacts, and Parquet
  metadata.

Any change to this profile is a behavior change and must trigger golden-output
review.

## Error Handling

Errors are explicit `thiserror`-derived enums:

```rust
#[derive(thiserror::Error, Debug)]
pub enum ScenarioError { /* parse, validation, version */ }

#[derive(thiserror::Error, Debug)]
pub enum ModelConfigError { /* unknown model, bad params */ }

#[derive(thiserror::Error, Debug)]
pub enum SimulationError {
    #[error("invalid state: {0}")]
    InvalidState(StateInvalidReason),
    #[error("integrator failed: {0}")]
    Integrator(#[from] IntegratorError),
    #[error("validation rule failed: {0}")]
    Validation(#[from] ValidationError),
    #[error("scenario stop condition: {0}")]
    Stop(StopReason),
}

#[derive(thiserror::Error, Debug)]
pub enum TelemetryError { /* schema, IO, version mismatch */ }
```

The simulator **fails closed**: NaN/Inf state, negative mass, frame
mismatch, unknown scenario field, deck-extrapolation when not opted in,
sensor at invalid rate — all stop execution with actionable diagnostics.

## Documentation Structure

Required documentation set:

| File | Purpose |
|---|---|
| `README.md` | Top-level orientation, disclaimers, doc links |
| `design-concept.md` | Purpose, boundaries, principles, vehicle classes, MVP, roadmap |
| `software-architecture.md` | This document |
| `safety-boundaries.md` | Acceptance / rejection rules, review checklist |
| `hypersonic-extensions.md` | Phase-6 extensions: high-altitude atmosphere, real-gas, hypersonic aero, aerothermal, boundary layer, rarefied flow, re-entry trajectories, validation suite |
| `data-provenance.md` | Required source records, source classes, transformations, review and machine checks |
| `frames-time.md` | Frame profiles, Earth model, epoch metadata, time scales, telemetry frame metadata |
| `scenario-format.md` | Formal grammar of the scenario DSL (Phase 1) |
| `verification.md` | Test taxonomy, validation labels, golden process (Phase 1) |
| `supply-chain.md` | Rust dependency policy, release SBOM, dependency checks, build provenance |
| `modeling-guide.md` | How to write a new model, document assumptions (Phase 5) |
| `glossary.md` | Vocabulary (frames, time systems, validation labels) (Phase 1) |

Each crate carries a `README.md` describing its public API, its dependencies,
and its safety posture (which `safety-boundaries.md` rules it touches).

### Foundational textbook references

The architecture draws on, and modules cite as appropriate:

- Stevens, B. L. and Lewis, F. L. *Aircraft Control and Simulation*
  (Wiley, 3rd ed., 2015) — 6-DOF flight dynamics; three-loop autopilot
  formulation.
- Anderson, J. D. *Fundamentals of Aerodynamics* (McGraw-Hill, 6th ed.,
  2017) — synthetic deck examples and supersonic / hypersonic transition
  reference material.
- Vallado, D. A. *Fundamentals of Astrodynamics and Applications*
  (Microcosm Press, 4th ed., 2013) — orbital propagation, frame
  transforms, time systems.
- Niskanen, S. *Development of an Open-Source Model Rocket Simulation
  Software*, master's thesis, Helsinki University of Technology, 2009 —
  canonical academic rocket-simulator reference.
- Picone, J. M. et al. "NRLMSISE-00 empirical model of the atmosphere,"
  *J. Geophys. Res.* 107(A12), 2002.
- NACA Report 1135 (1953), *Equations, Tables, and Charts for Compressible
  Flow* — public foundational compressible-flow reference for Mach,
  dynamic pressure, oblique-shock, and Taylor-Maccoll relations.
- IEEE Std 952-2020 — *Single-Axis Interferometric Fiber Optic Gyros* —
  inertial-sensor noise terms and Allan-variance conventions used by the
  synthetic IMU model.

No restricted, ITAR-controlled, EAR-controlled, MTCR-controlled, or
operationally-classified document is consulted, cited, or implemented from.

## Roadmap

See [design-concept.md § Phase Roadmap](design-concept.md#phase-roadmap) for
the phased plan. The short version:

- **Phase 0** — Documentation and boundaries (this commit).
- **Phase 1** — Deterministic core, RK4 kernel, golden-test infrastructure.
- **Phase 2** — Toy physics, basic environment, synthetic sensors,
  telemetry exporters.
- **Phase 3** — Modular models (aero deck, motor format, wind, additional
  sensors), property + fuzz tests.
- **Phase 4** — Virtual flight controller (estimator, autopilot, mission
  FSM, academic guidance, FDIR).
- **Phase 5** — Adaptive integrators behind profile flags, public-benchmark
  validation, optional socket-bridge HIL pattern, optional many-body
  groundwork.
- **Phase 6** — Hypersonic extensions (Earth atmosphere only): solver
  profiles beyond RK4, high-altitude atmosphere (NRLMSISE-00 first, NRLMSIS
  2.x / HWM14 follow-ons), real-gas equilibrium thermodynamics, hypersonic
  aero methods, the `openbmp-aerothermal` crate, boundary-layer models,
  continuum-to-rarefied bridging, re-entry trajectory infrastructure,
  hypersonic validation suite, Park two-temperature nonequilibrium
  thermochemistry, generic surface ablation toy, offline high-fidelity
  reference packages, and UQ / credibility reporting. Detailed in
  [hypersonic-extensions.md](hypersonic-extensions.md).

**Out-of-roadmap (explicitly never):** real device drivers, real bus
protocols, deployable executive, real-time scheduling guarantees, real
fielded-vehicle parameter sets, targeting, terminal homing, intercept
logic, payload-delivery code, operational mission planning.

## Extensibility for Downstream Integration

OpenBMP's trait surfaces and crate boundaries are deliberately designed
to be **extensible by downstream consumers**. The OpenBMP repository
itself ships only academic, public, synthetic, or textbook content under
the safety boundaries; downstream consumers may build research,
engineering, or independently qualified applications in their own
repositories, with their own data and compliance posture.

### What downstream consumers may build on top of OpenBMP

- Additional implementations of any extension trait: `EnvironmentModel`,
  `AeroMethod`, `Motor`, `MassModel`, `SyntheticSensor` (replaced by real
  device drivers in their downstream stack), `Estimator`, `Autopilot`,
  `MissionStateMachine`, `Fdir`, `HeatTransferModel`, `BoundaryLayer`,
  `AblationModel`, `NonequilibriumAir`, `BridgeFunction`, `Integrator`,
  `FaultModel`.
- Lab-specific hardware adapters that translate between a downstream
  test rig and the optional generic socket bridge (`openbmp-bridge`), in
  their own repositories, with their own export-control and qualification
  posture. OpenBMP does not ship those adapters.
- Production gain sets, validated aerodynamic decks, real motor data,
  operationally-tuned sensor noise budgets, fielded-vehicle mass
  properties — all kept in downstream repositories with downstream
  provenance and compliance.
- Domain-specific scenario formats that compile down to OpenBMP's
  scenario DSL.
- Custom telemetry exporters, post-processing pipelines, or visualization
  layers that consume OpenBMP's Parquet output.

### The extensibility contract

- Trait method signatures use only types defined in `openbmp-core`;
  downstream extensions never depend on internal implementation types of
  other OpenBMP crates.
- Stable across minor versions within a major release. Breaking trait
  changes require a major version bump and a deprecation cycle.
- No "phantom" plugin loading or `cdylib` magic — extensions are static
  Rust dependencies linked at compile time. This is a determinism
  requirement, not just style.
- Determinism guarantees apply only when downstream extensions also obey
  the determinism profile (no wall-clock, seeded RNG, no `HashMap`
  iteration, etc.). Extensions that violate the profile produce
  `state-stable, not bit-stable` outputs and must be flagged as such.
- The OpenBMP core repository remains free of any downstream's restricted
  data; it is the downstream's responsibility to keep their integration
  in their own repository with their own license and export-control
  compliance.

### Where the line is

OpenBMP's safety boundaries (`safety-boundaries.md`) apply to the
**OpenBMP repository** — to what code, data, and documentation live in
this tree. They do **not** certify or endorse what downstream consumers
do in their own repositories. A downstream consumer integrating OpenBMP
into any qualified or operational stack is responsible for:

- Their own provenance and licensing compliance for any vehicle data
  they bring.
- Their own export-control posture (ITAR, EAR, MTCR, Wassenaar, national
  equivalents).
- Their own qualification regime (DO-178C, ISO 26262, IEC 61508, etc.) —
  OpenBMP claims none.
- Their own validation of any trait extensions they ship: OpenBMP's
  validation labels (`experimental`, `checked`, `validated-toy`,
  `research`) attach to OpenBMP-shipped models only.

The clean separation lets OpenBMP serve as a pristine academic core and
lets downstream integrators carry their own compliance burden without
contaminating the academic core.

Downstream extensions that add real protocols, real hardware drivers,
restricted data, production gain sets, or operational parameter sets are not
OpenBMP-supported features and must not be submitted back to this repository.
Compatibility with OpenBMP trait names does not make such extensions part of
OpenBMP's safety or validation posture.

## Open Architecture Decisions

Tracked here so the next contributor can see what hasn't been decided:

1. **Telemetry canonical format.** Parquet is recommended above; if the
   community preference is JSON-as-canonical with Parquet as export, decide
   before Phase 2.
2. **Many-body container shape.** Phase-5 staging now requires a
   `World`-style container for multiple `VehicleAssembly` instances. The
   open decision is whether the Phase-1/2 single-state kernel should grow
   a compatibility wrapper early or wait for the Phase-5 refactor.
3. **Frame-aware arithmetic strictness.** Type-tagged frames are
   compile-time-enforced above; if this becomes too friction-heavy in
   practice, evaluate a `WithFrame<T>` runtime-tagged alternative — but not
   before Phase 4.
4. **Scenario extensibility.** Recompile-to-add-model is the v1 stance.
   Plugin loading via `cdylib` is a Phase 5+ question with determinism
   risks.
5. **Synthetic-sensor noise classes.** Pick canonical published noise
   budgets (textbook tactical / consumer / aerospace classes) and lock
   them as named presets so scenarios don't drift across academic uses.
6. **Determinism budget.** Define the per-PR CI wall-clock target (current
   target: 5 minutes) and the nightly budget before tests proliferate.
7. **External viewer.** Whether `openbmp-cli` ships a viewer or only export
   files for users to plot in their tool of choice. Recommend the latter
   for v1 to avoid GUI scope creep. Any viewer that does ship must be
   read-only over telemetry archives or local playback, with no command path
   back into a running simulation.
8. **Recovery-model scope.** Resolved in Phase 3.9: recovery state
   machines live in `openbmp-vehicle`, runner orchestration lives in
   `openbmp-cli`, and `openbmp-sim` carries only flat snapshots/events
   to preserve layering.
