# OpenBMP Design Concept

## Identity

**OpenBMP** stands for **Open Body Motion Platform**: a Rust-first,
simulation-only, academic research platform for rigid-body dynamics with a
primary focus on **rocket-class and launch-vehicle-class flight simulation**.

OpenBMP is intended for engineering education, simulation research, controller
prototyping, autotest-driven aerospace software experiments, and reproducible
trajectory studies. It is **not** a deployable flight stack, **not** a weapon
design tool, and **not** a hardware integration framework. See
[Safety Boundaries](safety-boundaries.md) for the full acceptance and
rejection list.

## Purpose

OpenBMP exists to make it easy to:

- Define a virtual rigid body or rocket-class vehicle in a small text scenario.
- Simulate its motion through atmosphere and gravity with deterministic 6-DOF
  dynamics.
- Wire in synthetic sensors, a virtual flight controller, and a launch-phase
  state machine.
- Run regression-tested experiments with byte-stable telemetry and
  property-tested invariants.
- Validate controllers against analytic-toy and public benchmark cases before
  considering real-world adaptation (which OpenBMP itself does not perform).

The audience is graduate aerospace students, controls and GNC researchers,
academic rocket teams, and software engineers who want a Rust-native sandbox
for studying flight dynamics without touching real hardware or real
fielded-vehicle data.

## Safety Boundary (Summary)

OpenBMP must remain **non-weapon, non-deployable, and non-hardware-integrating
by design**. Targeting, intercept, terminal guidance to real-world locations,
real fielded-vehicle parameters, real device drivers, real-time deployment,
operational mission planning, and counter-defense techniques are all
**out-of-scope and rejected**. Synthetic and public/educational physics
parameters are accepted. Detailed acceptance and rejection rules are
maintained in [safety-boundaries.md](safety-boundaries.md).

Every release artifact must include the non-suitability disclaimer:

> OpenBMP is an academic simulation platform. It is not validated for
> operational flight, not suitable for hardware deployment, not a weapon
> system, and not a substitute for any qualified flight-software stack.

## Vehicle Classes

OpenBMP is designed to simulate the following classes of virtual vehicles:

- **Toy rigid bodies** — abstract free or constrained rigid bodies for
  textbook problems (torque-free Euler dynamics, simple harmonic motion,
  constant-acceleration drops).
- **Sounding rockets** — single- or multi-stage research rockets with simple
  aerodynamic decks and synthetic motors.
- **Hobby high-power rockets (HPR)** — academic equivalents to OpenRocket /
  RocketPy use cases, with public/synthetic motor data.
- **Suborbital launch vehicles** — multi-stage academic launchers with
  separation events, coast phases, and atmosphere-to-vacuum transitions.
- **Orbital insertion analyses** — single-vehicle propagation in inertial
  frames with truncated spherical-harmonic gravity, used for educational
  trajectory studies.
- **Lifting re-entry research vehicles** (Phase 6) — academic equivalents to
  ESA IXV / NASA X-37B class. Lifting entry, blunt-cone capsules,
  aerocapture / aerobraking textbook problems. See
  [hypersonic-extensions.md](hypersonic-extensions.md).
- **Hypersonic research configurations** (Phase 6) — Mach 5–25+ flight at
  high altitude, with solver profiles beyond RK4, real-gas thermodynamics,
  aerothermal heating, boundary-layer transition, continuum-to-rarefied
  bridging, offline high-fidelity reference packages, and uncertainty
  reporting. Driven by scenario waypoints in inertial space, never by
  real-world targets.

OpenBMP does not simulate:

- Operational missile profiles, guided weapons, terminal-homing engagements.
- Multi-vehicle adversarial scenarios, intercept geometries, decoy logic.
- Re-entry vehicles for warhead delivery, MaRVs, or HGVs.
- Hypersonic skip-glide trajectories whose stated objective is to reach a
  real-world target location.
- Penetration aids, plasma-sheath blackout exploitation, defense penetration,
  or any counter-defense logic.

## Design Principles

### 1. Rust-first, in-house core

The deterministic simulation kernel, math primitives, units, frames,
state types, scenario parser, integrator suite, and test harness are all
written in pure Rust with no mandatory runtime dependencies outside
carefully chosen stable crates (see
[software-architecture.md](software-architecture.md) §Crate Selection).
External tools may be used for development, CI, documentation, and optional
visualization, but the core simulation runs from a single statically
linkable binary.

### 2. Modular by construction

Every physical model (environment, force, moment, mass, sensor, fault) is a
replaceable Rust trait implementation with explicit, typed inputs and
outputs. Cross-crate dependencies point downward only. Adding a new
atmosphere model, a new motor type, or a new estimator must not require
changes outside the relevant crate.

### 3. Determinism is a first-class property

The same scenario, seed, and version produce **byte-identical telemetry** on
the same platform profile. Time only advances on `step()`. There is no
wall-clock dependence, no network access, and no system-RNG access inside
the simulation kernel. RNG seeds are derived deterministically from
`(scenario_seed, step_index, channel_id)`. Performance optimizations must
not change deterministic outputs unless a new explicit profile is
introduced.

### 4. Lockstep simulation

The simulator and any virtual controller advance in lockstep: each kernel
step emits sensor packets, the controller emits commands, the kernel
applies them, and the loop repeats. Real-time chasing is not used. This is
borrowed directly from the PX4 SITL discipline.

### 5. Sim-first, autotest-driven development

Every feature begins as a scenario, golden output, property test, or
analytic-toy validation case **before** it becomes a public API. No module
is accepted without tests that define its expected behaviour. The test
corpus is part of the architecture, not an afterthought.

### 6. Academic traceability

Assumptions, model limitations, units, coordinate frames, validity ranges,
and validation status are documented next to the model. Validation claims
are conservative. Status labels per model:

- `experimental` — implemented but not validated.
- `checked` — internally consistent against unit/property tests.
- `validated-toy` — compared against analytic or simple public examples.
- `research` — suitable for academic experiments, not operational use.

No OpenBMP module ever claims operational suitability.

### 7. No hardware path shipped in the OpenBMP repository

The OpenBMP repository ships no supported path from its virtual-controller
code to physical sensors, real buses, real actuators, or real flight
computers. The optional HIL pattern is a generic socket bridge with no
real device drivers and no real bus protocols. Downstream consumers
building lab or independently qualified hardware integrations may add
such paths in their own repositories under their own export-control and
qualification posture
(see [software-architecture.md § Extensibility for Downstream
Integration](software-architecture.md#extensibility-for-downstream-integration));
the OpenBMP core repository itself will not.

### 8. Public-data provenance

Any imported public dataset (atmosphere coefficients, gravity coefficients,
synthetic motor curves, textbook aero examples) ships with a provenance
record naming the source, license, retrieval date, and verification method.
Datasets without acceptable provenance are removed. The required provenance
record is defined in [data-provenance.md](data-provenance.md).

The four-pillar contract is enforced at `cargo test` time by the
inline-data tripwires (see
[data-provenance.md § Inline Data Tripwires](data-provenance.md#inline-data-tripwires)).
The tripwires fail the build on multi-line Rust string TOML in `*.rs`
source, on high-precision benchmark constants outside their declared
source-of-truth files, and on TOML includes that do not resolve under
fixture/data directories, so benchmark-data smuggling cannot pass review.

### 9. Explicit frames, time, and numerics

Frame transforms, time scales, Earth models, and deterministic floating-point
assumptions are part of the scenario contract. OpenBMP scenarios must declare
their frame profile when they need Earth-fixed or local-frame semantics, and
telemetry must record enough metadata for a consumer to reject unknown frame
or time conventions. The detailed policy is in
[frames-time.md](frames-time.md).

## Conceptual Data Flow

```text
Scenario file
  -> parser + validator
  -> model registry (env, vehicle, sensors, controller, scheduler)
  -> initial state
  -> simulation kernel (lockstep, fixed-step or adaptive)
       |
       v
  +----> environment models (atmosphere, gravity, wind, magnetic)
  |
  +----> force / moment / mass-property models
  |        (aerodynamics, propulsion, gravity force, etc.)
  |
  +----> propagator (RK4 / DOPRI / analytic-toy)
  |
  +----> truth-state output
  |
  +----> synthetic sensors (truth -> noisy measurement)
  |
  +----> virtual flight controller
  |        (estimator -> autopilot -> mission state machine -> commands)
  |
  +----> commands consumed by simulator-local actuator models
  |
  +----> telemetry (typed channels -> ring buffer -> Parquet/CSV/JSON)
  |
  +----> validation rules / golden output / reports
```

The virtual flight controller is **inside the simulation loop only**. It does
not expose actuator packets to real hardware, real bus protocols, or
deployable runtime hooks. The optional socket bridge for HIL-style external
clients ships only an in-house wire format with no real protocol
implementation.

## Architectural Inspirations

OpenBMP is informed by — but does not import or copy from — the following
reference systems. Each has been studied for patterns; only the patterns are
borrowed, never the data, the deployment artifacts, or the operational
framing.

| System | What we borrow | What we skip |
|---|---|---|
| **NASA cFS / cFE** | Strict layering: pure deterministic core → runtime middle → mission-specific models on top. Centralized, version-controlled tables for parameters. Structured event types per module. | The deployable RTOS-targeted executive; CCSDS framing; void* message payloads. |
| **PX4 Autopilot** | Lockstep simulation discipline. Code-generated typed messages from a single schema source. Module concept maps to a Rust trait `Module { fn step(&mut self, ctx) }`. | NuttX/RTOS coupling; MAVLink as the simulation API; deployable bias of real-driver modules. |
| **ArduPilot** | The vehicle / library split as a Rust workspace organising principle. Massive scenario-driven autotest corpus. SITL-as-host-build pattern. | C++ inheritance hierarchies; runtime parameter blob; MAVLink ground link coupling. |
| **F Prime (NASA JPL)** | The components / ports / events / channels / commands vocabulary as Rust traits. Topology-as-deployment-artifact maps to OpenBMP's scenario file. | The full FPP/AC code-generation pipeline (defer until needed). |
| **RocketPy** | Four-class shape: `Environment`, `Motor`, `Vehicle`, `Flight`. Dispersion / Monte-Carlo as a first-class concept. Solid/Liquid/Hybrid motor variants as an enum. | Python performance ceiling; notebook-first culture as the primary surface. |
| **OpenRocket** | Motor / propulsion as a versioned external dataset with strong save-format discipline. Thrust-curve database as a curated, versioned resource. | Java/Swing GUI; embedded scripting languages (Lua/JS) — Rust traits only. |
| **NASA GMAT** | Resources + mission-sequence DSL shape. Two propagator families (numerical + analytic-toy). Force-model-as-composed-list pattern. | Operational mission-design commands (`Target`, `Maneuver`); large GUI; full EGM2008 to high degree/order. |
| **POST / POST2** | Point-mass-first mentality (3-DOF before 6-DOF). Discrete events (staging, separation, deploy) as first-class objects. | Fortran call conventions; real launch-vehicle parameter sets. |
| **BPS.space Signal** | Phase-based mission state machine vocabulary (pre-launch / ascent / coast / apogee / descent / recovery / safed). | Real microcontroller ports; real PWM / I2C / SPI; real motor and sensor parameters. |
| **MAPLEAF** (Stoldt 2021) | Key-value scenario DSL with parametric-uncertainty `_stdDev` suffix; separate batch-runner CLI (`mapleaf-batch`) for design-of-experiments runs and Monte Carlo dispersion. | Cython hot-path coupling; Python-first ecosystem assumptions. |
| **NaveGo / NorthStarUAS / INSTINCT** | 15-state error-state INS+GNSS Kalman formulation; side-by-side filter-comparison harness; multi-constellation patterns. | Native MATLAB / C++ build complexity; multi-receiver hardware coupling. |
| **OpenRocket / Niskanen 2009** | Barrowman extended component build-up as a public semi-empirical aero-prediction reference; RASP `.eng` motor-format heritage. | Java/Swing GUI; embedded scripting (Lua/JS). |

Each project in the table is referenced by URL in the bibliography of the
relevant module's `README.md` (per crate). The table reflects pattern
borrowing only; OpenBMP does not import code, data, or build artifacts
from any of these projects.

## Product Shape

OpenBMP is a workspace of small Rust crates organized in five layers:

```text
       openbmp-cli           (binary)        scenarios + reports + plots
            |
   openbmp-scenario     openbmp-telemetry    file IO, validation, archive
            |                  |
              openbmp-fc                     virtual flight controller
              openbmp-sensors                synthetic sensors
            |                  |
   openbmp-vehicle    openbmp-env            physics models
   openbmp-aero
   openbmp-propulsion
            |
        openbmp-sim                          lockstep kernel, integrators,
                                             scheduler, state, events
            |
        openbmp-core                         math, units, frames, time, RNG
```

Crates expose narrow APIs. The deterministic kernel sits below all model
crates. The CLI sits at the top and is the only entry point that produces
artifacts the user sees.

The first public release should feel like a research-grade simulator — a user
defines a toy virtual body or rocket, runs a scenario, inspects state history,
and verifies deterministic outputs via golden tests. The full architecture
is in [software-architecture.md](software-architecture.md).

## Verification Philosophy

OpenBMP is autotest-driven. Required test classes:

- **Unit tests** — math primitives, units, frames, interpolation, parsers.
- **Property tests** (proptest) — invariants such as finite values,
  normalized rotations, monotonic time, deterministic replay across reruns,
  positive mass, frame round-trips.
- **Snapshot tests** (insta) — telemetry summaries against a stored baseline.
- **Golden scenario tests** — byte-stable telemetry output for canonical
  scenarios.
- **Analytic-toy validation** — compared against closed-form solutions:
  constant-acceleration drop, torque-free rigid body Euler dynamics,
  two-body Keplerian orbit, harmonic-oscillator attitude damping.
- **Public benchmark validation** — selected published academic cases (e.g.,
  RocketPy/OpenRocket reference rockets with public parameters).
- **Fuzz tests** (cargo-fuzz / proptest) — scenario parsing, model config
  parsing.
- **Documentation tests** — every public example compiles and runs.
- **Microbenchmarks** (criterion) — kernel hot path, integrator step,
  sensor packet generation.
- **Determinism CI gate** — every PR runs the canonical scenario set twice
  on the reference platform profile and asserts byte-identical telemetry;
  regressions to `state-stable` (non-bit-stable) require an explicit
  profile flag. The same gate runs with the diagnostic `tracing`
  subscriber redirected to `/dev/null` to verify that logging side-effects
  do not leak into deterministic outputs.

Validation hierarchy:

1. **Analytic-toy** — closed-form comparison.
2. **Public benchmark** — published unclassified reference data.
3. **Peer-reviewed** — comparison against published academic papers.
4. **Flight-validated** — **explicitly not a goal**. OpenBMP does not claim
   flight validation and does not plan to add such claims.

## MVP

The safe MVP is intentionally narrow:

- Rust workspace scaffold with the crate layout above.
- Deterministic fixed-step simulation kernel with lockstep step semantics.
- `openbmp-core`: vector / matrix / quaternion / DCM primitives via
  nalgebra; `uom`-typed quantities at boundaries; `SimTime` newtype;
  `FrameId` tag types (ECI, ECEF, NED, ENU, Body); seeded deterministic RNG.
- `openbmp-sim`: RK4 fixed-step integrator; scheduler; event handling;
  state storage.
- `openbmp-state`: 3-DOF point-mass and 6-DOF rigid-body state.
- `openbmp-env`: constant gravity; J2 gravity; US Standard Atmosphere 1976;
  no-wind and constant-wind models.
- `openbmp-vehicle`: rigid-body trait; constant-mass and linear-burn mass
  models; analytic-toy force/moment providers.
- `openbmp-aero`: stub aero deck with synthetic CD(Mach, alpha) tables.
- `openbmp-propulsion`: synthetic solid-motor model with a thrust curve
  loaded from an in-house TOML format (RASP-shaped, in-house written).
- `openbmp-sensors`: ideal-state sensor; synthetic IMU with Allan-variance
  noise model; synthetic barometer.
- `openbmp-fc`: virtual-controller trait; no-op controller; toy attitude-
  damping controller; mission state machine with academic phases.
- `openbmp-scenario`: in-house TOML-shaped scenario format with versioning
  and unknown-field rejection.
- `openbmp-telemetry`: typed channels; CSV + JSON exporters; Parquet writer
  with explicit unit/frame metadata; golden-output regression mode.
- `openbmp-testkit`: property-test helpers, analytic-toy fixtures,
  scenario fuzzers, `compare_filters` helper for side-by-side estimator
  comparison (pattern borrowed from NorthStarUAS `insgnss_tools`).
- `openbmp-cli`: scenario runner, golden-test runner, diff command for
  regression review, `batch` subcommand for parametric-sweep / Monte-Carlo
  dispersion runs (pattern borrowed from MAPLEAF's `mapleaf-batch`).

The MVP **excludes** real device drivers, real bus protocols, real flight-
computer ports, real RTOS code, real fielded-vehicle parameter sets,
targeting, terminal guidance, payload-delivery logic, intercept geometry,
and any operational mission profile.

## Phase Roadmap

**Phase 0 — Documentation and boundaries** (complete)
- Published design concept and software architecture.
- Published safety boundary checklist.
- Published Phase-1 documentation set: scenario format, verification,
  glossary, data provenance, frames/time, modeling, and supply chain.
- Canonical telemetry archive format chosen: Parquet (with explicit
  unit/frame metadata in column descriptors).

**Phase 1 — Deterministic core** (complete)
- Rust workspace scaffold with the 15-crate layout.
- `openbmp-core` foundation: math, units, frames, time, deterministic
  RNG, validation labels.
- `openbmp-state`: 3-DOF `PointMassState` and 6-DOF `RigidBodyState`
  with structural validation (finiteness, positive mass, normalised
  quaternion, symmetric positive-definite inertia, triangle
  inequalities).
- `openbmp-sim`: RK4 fixed-step lockstep kernel for point-mass with
  the locked weighted-sum order, the canonical `start + step * dt`
  time advance, and the MXCSR floating-point-environment guard on
  x86_64.
- `openbmp-telemetry`: typed channels, ring buffer, deterministic CSV
  / JSON / Parquet exporters with locked archive column order.
- `openbmp-scenario`: strict TOML parser with
  `serde(deny_unknown_fields)`, model-name registry, safety-name lint,
  unit-suffix and frame-infix linting, source-rooted path resolution.
- `openbmp-testkit`: `proptest` strategies, analytic-toy reference
  solutions, tolerance-table parser, byte-stable replay helper.
- `openbmp-cli`: `openbmp run / diff / check / check-provenance`
  commands; binary + library shape with `assert_cmd` / `insta-cmd`
  snapshot tests on the user surface.
- First end-to-end golden test: the constant-acceleration drop
  scenario passes the closed-form tolerance table and is byte-stable
  across same-machine reruns.
- CI gates wired in `.github/workflows/ci.yml`: rustfmt, clippy
  `-D warnings`, build, `cargo nextest run`, doc-tests, `cargo deny`,
  `cargo audit`, `cargo machete`, feature-powerset, MSRV pin, typos,
  rustdoc warnings, the headline determinism gate (canonical scenario
  twice + tracing-no-leak verification), and a cross-platform
  state-stable matrix.
- Release artefacts in `.github/workflows/release.yml`: source tarball,
  Linux `openbmp` binary, Cargo.lock, CycloneDX SBOM bundle, audit
  report, build manifest, scenario/golden manifest, sha256 manifest,
  non-suitability disclaimer.
- Per-crate `README.md` files compliant with `docs/modeling-guide.md`
  (purpose / inputs / units / frames / assumptions / validity range /
  determinism / validation / data provenance / safety boundary).

**Phase 2 — Sounding-rocket physics** (complete)
- 6-DOF rigid-body kernel (`RigidBodyKernel`) added alongside the
  Phase-1 point-mass kernel; `Rk4FixedStep` extended over `SimState`
  with quaternion renormalisation; analytic-toy torque-free
  precession validation.
- Environment models in `openbmp-env`: `ConstantGravity`,
  `PointMassGravity`, `J2Gravity` (NIMA TR 8350.2 J2 coefficient),
  US Standard Atmosphere 1976 in-house port (0–86 km), `NoWind`,
  `ConstantWind`.
- Minimal Earth-frame support in `openbmp-core`:
  `wgs84-uniform-rotation` profile + scenario-declared local geodetic
  origin.
- Aerodynamic deck in `openbmp-aero`: schema-1 reduced sounding-rocket
  deck `(Mach, alpha, beta) → (CN, CD, CM)` with locked-order
  trilinear interpolation, fail-closed extrapolation; one shipped
  synthetic finned-cylinder deck.
- Synthetic solid motor in `openbmp-propulsion`: in-house RASP-shaped
  TOML thrust-curve format, piecewise-linear interpolation,
  impulse-weighted mass model. Shipped: synthetic textbook + D-class
  motors plus public Estes hobby motor files (B4, C6, D12) derived
  from ThrustCurve.org with SHA-256-pinned provenance.
- Synthetic sensors in `openbmp-sensors`: `IdealStateSensor`,
  `SyntheticImu` (IEEE 952 five-component noise model), and
  `SyntheticBarometer` (Gaussian + OU bias drift).
- Vehicle composition in `openbmp-vehicle`: flat `Vehicle` trait with
  `BasicVehicle` carrying ordered force / moment / mass model lists
  and per-step force-breakdown evaluation. Kernel-side adapter
  family (`GravityForceAdapter`, `MotorThrustForceAdapter`,
  `MotorMassAdapter`, `AxialDragForceAdapter`) wraps L2 physics into
  point-mass `ForceModel` / `MassModel` impls.
- Scenario format extensions in `openbmp-scenario`: structured
  `[aero]`, `[propulsion.motor]`, `[wind]`, `[atmosphere]`,
  `[frames.local_origin]` blocks plus typed `[sensors.<name>]`
  entries. Optional rigid-body initial-state fields. SHA-256 pin
  fields per external-file reference; pin verification fails closed
  before kernel construction.
- CLI integration in `openbmp-cli`: dispatcher selects between the
  Phase-1 byte-stable analytic-toy path and the Phase-2
  point-mass-with-adapters path by scenario shape; `vehicle.kind =
  "rigid_body"` returns a typed Phase-3 deferral message.
- Telemetry extensions: per-model force-breakdown channels
  (`force.<name>.x_n` etc.), atmosphere sample channels (density,
  pressure, temperature, speed of sound), and SHA-256 scenario-file
  digests recorded as Parquet schema metadata.
- Public-benchmark validation: Niskanen 2009 thesis Chapter-6
  sounding-rocket case, with the canonical scenario and Estes C6
  motor producing simulated apogee within ±5% of the published
  151.5 m experimental value.
- Inline-data prevention: workspace tripwires reject inline TOML in
  `*.rs` source, high-precision benchmark constants outside their
  declared source-of-truth files, and any `*.toml` under
  `data/`/`scenarios/` lacking a sibling `provenance.md`.
- Determinism CI gate runs both the analytic-toy and the Niskanen
  scenarios twice on `x86_64-unknown-linux-gnu` and asserts
  byte-identical Parquet across reruns; cross-platform matrix runs
  the same scenarios as state-stable smoke tests.

**Phase 3 — Modular composable rocket** (complete)
- Rigid-body kernel adapter family completing the Phase-2.11
  deferral: `RigidGravityForceAdapter`,
  `RigidMotorThrustForceAdapter`, `RigidMotorMassAdapter`,
  `RigidAxialDragForceAdapter`, `RigidAeroDeckForceAdapter`,
  `RigidEngineClusterAdapter`, `RecoveryRackForceAdapter`,
  `MovingMassRackAdapter`. Runner now accepts
  `vehicle.kind = "rigid_body"` end-to-end.
- `VehicleAssembly` tree (`Bodies / Propulsion / Effectors / Tanks
  / Sensors / Recovery`) composed in the scenario file and
  resolved into the kernel's flat `KernelModelBundle` at startup.
- `MissionPhaseGraph` with `EventTrigger` /
  `BuiltInEventTrigger` (AtTime / AtAltitude / AtApogee /
  AtMassFraction / AtDynamicPressure), `EventBinding`, and
  declarative phase transitions. Cycle-rejecting at construction
  time; iteration order is the topological order.
- `ControlEffector` trait with rate / position / latency /
  deadband limits and the four canonical fault modes (Jam,
  Runaway, ReducedRate, Hardover). Effectors feed aero deck via
  schema-2 effector axes and feed engines via `EngineCommand`.
- Aero deck schema-2 with optional control-effector axes
  (`delta_e_deg`, `delta_a_deg`, `delta_r_deg`, body flaps, grid
  fins). Schema-1 decks still parse byte-identically.
- `EngineModel` + `EngineCluster` for liquid / multi-engine
  vehicles: per-engine throttle, gimbal, ignition / shutdown
  state machine, cluster-summed thrust + moment + mass flow over
  scenario-declared mount points.
- `Tank` + `MovingMassModel` (rigid-liquid, equivalent pendulum
  per Abramson SP-106 §7.4, equivalent spring-mass, baffled
  pendulum with `BaffleModel` damping increment). Forward-Euler
  sub-step is the bit-stable default; higher sub-step counts are
  scenario opt-in but break bit-stability across changes (this
  deviated from the original implicit-step plan; the per-sub-phase
  commit history records the rationale).
- Wind extensions: `LayeredWind` (per-altitude table with linear
  interpolation between layers) and `GustWind` (Dryden rational-
  spectrum filter per MIL-STD-1797A; six parameters σ_u/σ_v/σ_w
  and L_u/L_v/L_w).
- Recovery models: `ParachuteDrag`, `DrogueMainRecovery`, and
  `DragDevice` deploy on `MissionPhaseGraph` triggers; recovery
  state and drag area surface as telemetry channels.
- Additional synthetic sensors: `SyntheticGnss` (IS-GPS-200
  nominal noise budget), `SyntheticMagnetometer` (WMM 2025 body-
  frame truth + Gaussian noise + soft / hard-iron biases), and
  `SyntheticStarTracker` (per-axis Gaussian quaternion-error
  injection).
- WMM 2025 in `data/magnetic/WMM.COF`: verbatim public-domain
  NOAA / NGA / UK DGC December 2024 release with sibling
  `provenance.md` and SHA-256 pin. Validity expires 2030-01-01;
  out-of-epoch queries fail closed.
- Public-benchmark cross-tool validation: RocketPy "Calisto"
  (Cesaroni Pro75 M1670 via the RocketPy-mass
  `rocketpy-calisto-m1670.toml` variant) runs end-to-end through
  the rigid-body kernel and reaches apogee within an audited
  ±2 % cross-tool envelope around RocketPy's published 3 349 m
  AGL. The original ±1 % stretch goal was not hit; the residual
  ~1.5 % is tracked as cross-tool model envelope (atmosphere,
  rail, RocketPy `SolidMotor` differences), not RK4 truncation
  error — the Phase-3.11 audit confirmed `dt = 0.001` and
  `dt = 0.0001` apogees are unchanged to sub-millimetre
  precision.
- Determinism CI gate runs the analytic-toy, Niskanen, and
  Calisto scenarios twice on `x86_64-unknown-linux-gnu` and
  asserts byte-stable Parquet across all three.
- Property tests, fuzz tests, and microbenchmarks across the new
  surface.

**Phase 4 — Virtual flight controller**
- Estimator framework: EKF, MEKF (quaternion attitude).
- Three-loop autopilot scaffold with academic gains.
- Mission state machine: pre-launch / ascent / coast / apogee / descent /
  recovery, plus configurable user-defined phases sourced from the
  Phase-3 `MissionPhaseGraph`.
- Academic guidance laws: attitude tracking, scripted reference state,
  waypoint navigation between scenario-defined points.
- Powered-descent guidance scaffold: lossless-convexification soft-landing
  (LCvxLD, Acikmese & Ploen 2007) and SCvx successive-convexification
  variant for academic powered-descent studies (e.g., reusable-vehicle
  return-to-pad textbook problem). Scenario-defined landing site, not a
  real-world target.
- Linear MPC framework reusing the same effector / engine models, for
  attitude tracking and trim hold; convex-QP solver via in-house Rust or
  a vetted permissive-licence crate.
- FDIR framework: scenario-injected fault models that exercise the
  Phase-3 effector and engine fault modes.

**Phase 5 — Test harness expansion**
- DOPRI5/8 adaptive integrators (behind explicit profile flags, not
  default).
- Public-benchmark validation cases (RocketPy/OpenRocket-equivalent
  reference rockets).
- Optional socket-bridge HIL pattern (in-house wire format) — the
  generic lab-HIL adapter pattern; downstream users wire concrete
  device adapters in their own repositories under their own
  export-control posture.
- **Multi-body / staging promoted to first-class**: simultaneous flight
  of multiple `VehicleAssembly` instances after a separation event, with
  momentum exchange at the separation moment, separate state vectors,
  and shared environment sampling. Replaces the Phase-3 single-vehicle
  staging hack.
- **Multi-rate scheduling** as a first-class scheduler concept: rate
  groups expressed as integer divisors of the base tick rate (`env_rate_hz`,
  `controller_rate_hz`, `telemetry_rate_hz`, `effector_rate_hz`), with
  determinism preserved because the schedule is fixed at scenario
  start. The kernel resolves rate groups into a static sub-step plan
  before the first `step()`.

**Phase 6 — Hypersonic extensions** (research-grade)
- 6.0 Hypersonic solver stack (fixed high-order explicit RK, adaptive
  DOPRI853/RKF78 with dense output, implicit source-term sub-steppers,
  coupling policies, solver telemetry).
- 6.1 NRLMSISE-00 high-altitude atmosphere (in-house Rust port, public
  coefficients), with NRLMSIS 2.x and HWM14 follow-ons.
- 6.2 Real-gas thermodynamics (Tannehill equilibrium air, gamma_eff,
  composition).
- 6.3 Hypersonic aero methods (Modified Newtonian, tangent-cone, hypersonic
  similarity, hybrid dispatch).
- 6.4 Aerothermal stagnation heating (`openbmp-aerothermal` crate;
  Fay-Riddell, Sutton-Graves, Tauber-Sutton).
- 6.5 Boundary layer state and distributed surface heating
  (reference-enthalpy method, transition models).
- 6.6 Continuum-to-rarefied bridging (Knudsen number, free-molecular limit).
- 6.7 Re-entry trajectory infrastructure (Allen-Eggers analytic-toy, Vinh
  lifting-entry, academic skip-glide reference profiles, mission-FSM
  additions).
- 6.8 Public-benchmark validation suite (Apollo-class blunt-cone, Stardust
  SRC, Tauber-Sutton radiative comparison).
- 6.9 1-D thermal-conduction toy (surface-temperature evolution under
  prescribed heat flux).
- 6.10 Park two-temperature nonequilibrium thermochemistry (public
  reaction sets only, sub-stepped implicit-Euler integration).
- 6.11 Generic surface-ablation toy (steady-state and charring variants;
  textbook materials only — no real fielded TPS materials).
- 6.12 Offline high-fidelity reference packages (CFD, DSMC, radiation,
  thermal-response, thermochemistry, trajectory references with provenance).
- 6.13 Hypersonic UQ and credibility reporting (uncertainty propagation,
  sensitivity reports, NASA-STD-7009B-style evidence summaries without
  operational suitability claims).

Phase 6 details are in [hypersonic-extensions.md](hypersonic-extensions.md).
All Phase 6 work is **Earth-atmosphere only**; non-Earth atmospheres
(Mars, Titan, Venus) are out of roadmap.

**Out-of-roadmap (explicitly never):**
- Hardware integration as a shipped product.
- Real device drivers.
- Real bus protocols.
- Targeting, terminal homing to real-world locations, intercept logic,
  payload-delivery code.
- Real fielded-vehicle parameter sets.
- Operational mission planning, deployment procedures, or weapon-employment
  logic.
- Real fielded HGV / MaRV / hypersonic-cruise-weapon parameter sets.
- Plasma-sheath blackout exploitation, penetration aids, decoys, or
  counter-defense logic at any phase of flight.
- Skip-glide trajectories with terminal evasion or defense-penetration
  optimization.
