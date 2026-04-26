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
| Optional integrators | DOPRI5, DOPRI8 (behind profile flags) |
| Concurrency in kernel | Synchronous, single-threaded |
| Async | Only for optional socket-bridge tooling, never in kernel |
| Scenario format | In-house, TOML-shaped, versioned, reject unknown fields |
| Scenario lint | `openbmp check` validates schema, provenance, safety names, determinism |
| Telemetry channels | Typed in-process bus (software bus pattern) |
| Telemetry archive | Parquet with explicit unit/frame metadata |
| Testing | proptest, insta, cargo-fuzz, criterion, golden CSV |
| Data provenance | Required `provenance.md` for every shipped data source |
| Supply chain | `cargo deny`, `cargo vet`, CycloneDX SBOM for releases |
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
| Dependency audits | **cargo-vet** | Records third-party Rust dependency audits by trusted reviewers. |
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
├── rust-toolchain.toml                  # pinned MSRV
├── crates/
│   ├── openbmp-core/                    # L0: math, units, frames, time, RNG
│   ├── openbmp-sim/                     # L1: kernel, scheduler, integrators
│   ├── openbmp-state/                   # L1: state types (point-mass, rigid)
│   ├── openbmp-env/                     # L2: atmosphere, gravity, wind, mag
│   ├── openbmp-vehicle/                 # L2: rigid body, mass models
│   ├── openbmp-aero/                    # L2: aero decks + hypersonic methods
│   ├── openbmp-aerothermal/             # L2: heat transfer, BL, thermal toy
│   ├── openbmp-propulsion/              # L2: motors, thrust curves
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
    fn transform_position(&self, p: Position3<From>) -> Position3<To>;
    fn transform_velocity(&self, v: Velocity3<From>, p: Position3<From>) -> Velocity3<To>;
}
```

A `FrameContext` (snapshotted at scenario time) holds the canonical
transforms (Earth rotation rate, geodetic origin, etc.) used to construct
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
`FrameTransform`.

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
each step, the kernel evaluates models, integrates state, runs sensors,
calls the controller, applies commands, and emits telemetry. The
controller never advances faster than the kernel, and the kernel never
advances faster than the controller.

### Step Pseudocode

```rust
pub fn step(&mut self) -> Result<(), SimulationError> {
    // 1. Sample environment at current state.
    let env = self.env.sample(EnvironmentQuery::from(&self.state));

    // 2. Evaluate forces, moments, mass-property update.
    let force = self.force.force(ForceInput::new(&self.state, &env, &self.cmd));
    let moment = self.moment.moment(MomentInput::new(&self.state, &env, &self.cmd));
    let mass_dot = self.mass.derivative(self.state.time);

    // 3. Integrate (RK4 fixed-step or alternative).
    self.state = self.integrator.advance(&self.state, force, moment, mass_dot, self.dt)?;

    // 4. Generate synthetic sensor measurements from new truth.
    let measurements = self.sensors.sample(&self.state, self.rng.for_step(self.step_index));

    // 5. Tick virtual flight controller.
    let cmd = self.fc.update(VirtualControlInput::new(&measurements, self.state.time))?;
    self.cmd = cmd;

    // 6. Publish telemetry channels (truth, sensors, commands, mission phase).
    self.telemetry.publish_step(&self.state, &measurements, &cmd, self.step_index);

    // 7. Run validation rules; check stop conditions.
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

The default schedule is a single fixed step `dt` for everything. Multi-rate
scheduling is supported via per-subsystem **frame rates** that are integer
divisors of the base step (e.g., environment 100 Hz, controller 50 Hz, telemetry
20 Hz). Multi-rate output remains deterministic because the schedule is fixed
at scenario start.

## Numerical Integrators

`openbmp-sim` ships at minimum:

| Integrator | Order | Step | Use |
|---|---|---|---|
| **RK4** | 4 | fixed | Default. Byte-stable. Easy to reason about. |
| **DOPRI5** (Dormand-Prince 4(5)) | 5 | adaptive | Stiffer scenarios. Behind a `--profile=adaptive` flag. |
| **DOPRI8** (8(5,3)) | 8 | adaptive | High-accuracy validation runs. |
| **Analytic** | n/a | n/a | Toy validation: closed-form propagation for analytic-toy scenarios. |

RocketPy uses LSODA (auto-switching stiff/non-stiff) as default; OpenBMP
keeps RK4 fixed-step as canonical because LSODA-style adaptive switching
breaks bit-stable replay across runs that visit different stiffness
regimes. Adaptive integrators ship behind explicit profile flags only and
are tagged `state-stable, not bit-stable`.

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
pub trait EnvironmentModel {
    fn sample(&self, q: EnvironmentQuery) -> EnvironmentSample;
    fn validation(&self) -> ValidationStatus;
}

pub struct EnvironmentQuery {
    pub time: SimTime,
    pub position: Position3<ECI>,        // converted internally as needed
}

pub struct EnvironmentSample {
    pub atmosphere: AtmosphereSample,    // density, pressure, temperature, speed of sound
    pub gravity: Vector3<Acceleration, ECEF>,
    pub wind: Velocity3<NED>,
    pub magnetic_field: Vector3<MagneticFluxDensity, NED>,
}
```

### Force / Moment / Mass

```rust
pub trait ForceModel {
    fn force(&self, input: ForceInput) -> ForceOutput<ECI>;
    fn validation(&self) -> ValidationStatus;
}

pub trait MomentModel {
    fn moment(&self, input: MomentInput) -> MomentOutput<Body>;
    fn validation(&self) -> ValidationStatus;
}

pub trait MassModel {
    fn mass_properties(&self, time: SimTime) -> MassProperties;
    fn derivative(&self, time: SimTime) -> MassDerivative;
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
pub trait Vehicle {
    fn id(&self) -> VehicleId;
    fn force_models(&self) -> &[Box<dyn ForceModel>];
    fn moment_models(&self) -> &[Box<dyn MomentModel>];
    fn mass_model(&self) -> &dyn MassModel;
}
```

Mass models:

- `ConstantMass` — fixed.
- `LinearBurn` — linear depletion from `m0` to `m_dry` over a burn time.
- `TableBurn` — interpolated mass / inertia from a synthetic table.
- `MultiStage` — composed of stages with separation events at scripted
  conditions; scenario-driven, not target-driven.

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

### Aero Force / Moment Computation

```rust
pub struct AeroModel {
    deck: AeroDeck,
    reference: AeroReference,
}

impl ForceModel for AeroModel {
    fn force(&self, input: ForceInput) -> ForceOutput<ECI> {
        let q = dynamic_pressure(input.atmosphere, input.airspeed_body);
        let (mach, alpha, beta) = airdata(input.airspeed_body, input.atmosphere);
        let coeffs = self.deck.lookup(mach, alpha, beta);
        let f_body = aero_force_body(q, &self.reference, &coeffs);
        let f_eci = transform_to_eci(f_body, input.orientation);
        ForceOutput { force: f_eci, /* ... */ }
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
validation = "checked"

[burn]
duration_s = 4.0
total_impulse_n_s = 2400.0
specific_impulse_s = 220.0          # nominal, for ΔV consistency check
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
| `SyntheticImu` | body accel + body angular rate | Allan-variance per **IEEE 1139** overlapping-estimator convention: ARW (angle random walk), bias instability (flicker-frequency minimum), RRW (rate random walk), scale-factor error |
| `SyntheticBarometer` | pressure → altitude | additive Gaussian + bias drift |
| `SyntheticGnss` | position + velocity (with delay, dropout) | additive Gaussian + dropout windows |
| `SyntheticMagnetometer` | mag field in body | additive Gaussian + hard-iron offset |
| `SyntheticStarTracker` | quaternion attitude (optional) | von-Mises-Fisher rotation noise |

Noise parameters come from **academic published budgets** (e.g., textbook
Allan-variance specs for "tactical-grade" or "consumer-grade" classes); they
are not lifted from any real fielded sensor's spec sheet. Sensor models
declare their noise class explicitly.

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
Recovery models live as simulator-local force/mass/event components, not as
hardware outputs.

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
| `recovery.deployed` | bool | Per recovery device |
| `recovery.phase` | enum | Drogue, main, descent, recovered |
| `recovery.drag_area` | m² | Effective drag area |
| `recovery.descent_rate` | m/s | Derived from state |

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
pinned MSRV from `rust-toolchain.toml` and the `default` simulation
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
`x86_64-unknown-linux-gnu`, the pinned MSRV from `rust-toolchain.toml`,
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
| `supply-chain.md` | Rust dependency policy, release SBOM, audit/vet checks, build provenance |
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
- IEEE 1139-2008 — *Standard Definitions of Physical Quantities for
  Fundamental Frequency and Time Metrology — Random Instabilities* —
  Allan-variance overlapping-estimator standard.

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
- **Phase 6** — Hypersonic extensions (Earth atmosphere only): high-altitude
  atmosphere (NRLMSISE-00), real-gas equilibrium thermodynamics, hypersonic
  aero methods, the `openbmp-aerothermal` crate, boundary-layer models,
  continuum-to-rarefied bridging, re-entry trajectory infrastructure,
  hypersonic validation suite (sub-phases 6.1–6.9), Park two-temperature
  nonequilibrium thermochemistry (6.10), generic surface ablation toy
  (6.11). Detailed in [hypersonic-extensions.md](hypersonic-extensions.md).

**Out-of-roadmap (explicitly never):** real device drivers, real bus
protocols, deployable executive, real-time scheduling guarantees, real
fielded-vehicle parameter sets, targeting, terminal homing, intercept
logic, payload-delivery code, operational mission planning.

## Extensibility for Downstream Integration

OpenBMP's trait surfaces and crate boundaries are deliberately designed
to be **extensible by downstream consumers**. The OpenBMP repository
itself ships only academic, public, synthetic, or textbook content under
the safety boundaries; the architecture does not preclude production use
by downstream consumers in their own repositories.

### What downstream consumers may build on top of OpenBMP

- Additional implementations of any extension trait: `EnvironmentModel`,
  `AeroMethod`, `Motor`, `MassModel`, `SyntheticSensor` (replaced by real
  device drivers in their downstream stack), `Estimator`, `Autopilot`,
  `MissionStateMachine`, `Fdir`, `HeatTransferModel`, `BoundaryLayer`,
  `AblationModel`, `NonequilibriumAir`, `BridgeFunction`, `Integrator`,
  `FaultModel`.
- Real hardware drivers integrated via the optional generic socket bridge
  (`openbmp-bridge`), in their own repositories, with their own
  export-control posture.
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
this tree. They do **not** dictate what downstream consumers do in their
own repositories. A downstream consumer integrating OpenBMP into a
production flight stack is responsible for:

- Their own provenance and licensing compliance for any vehicle data
  they bring.
- Their own export-control posture (ITAR, EAR, MTCR, Wassenaar, national
  equivalents).
- Their own qualification regime (DO-178C, ISO 26262, IEC 61508, etc.) —
  OpenBMP claims none.
- Their own validation of any trait extensions they ship: OpenBMP's
  validation labels (`experimental`, `checked`, `validated-toy`,
  `research`) attach to OpenBMP-shipped models only.

The clean separation lets OpenBMP serve as a pristine academic core
without preventing downstream production use, and lets downstream
integrators carry their own compliance burden without contaminating the
academic core.

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
2. **Many-body roadmap.** If multi-vehicle scenarios become a goal, the
   kernel should adopt a `World`-style container in Phase 1 to avoid an ECS
   migration later.
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
8. **Recovery-model scope.** Decide whether recovery/descent models belong
   in `openbmp-vehicle` as force/event models or in a small
   `openbmp-recovery` crate once Phase 3 starts.
