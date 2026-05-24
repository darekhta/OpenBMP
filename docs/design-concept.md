# OpenBMP Design Concept

## Identity

**OpenBMP** stands for **Open Body Motion Platform**: a Rust-first,
simulation-only, academic research platform for rigid-body dynamics with a
primary focus on **rocket-class and launch-vehicle-class flight simulation**.

OpenBMP is intended for engineering education, simulation research, controller
prototyping, autotest-driven aerospace software experiments, and reproducible
trajectory studies. The project ships a simulator; **its abstractions are
designed to be re-implementable against real hardware** through a downstream
HAL, but the OpenBMP repository itself ships no HAL and does not validate or
support hardware deployment. See [Safety Boundaries](safety-boundaries.md)
for the project-side acceptance and rejection list.

## Purpose

OpenBMP exists to make it easy to:

- Define a virtual rigid body or rocket-class vehicle in a small text scenario.
- Simulate its motion through atmosphere and gravity with deterministic 6-DOF
  dynamics.
- Wire in synthetic sensors, a simulator-local flight controller, and a launch-phase
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

The OpenBMP repository must remain **non-weapon and non-deployable by
design**. Targeting, intercept, terminal guidance to real-world
locations, real fielded-vehicle parameters, operational mission
planning, and counter-defense techniques are all **out-of-scope and
rejected**. Real device drivers, board support packages, and real bus
protocols do not land in this repository — downstream HAL adopters who
build those on top of the controller-side trait surfaces ship them in
their own repositories under their own qualification posture.
Synthetic and public/educational physics parameters are accepted.
Detailed acceptance and rejection rules are maintained in
[safety-boundaries.md](safety-boundaries.md).

Every release artifact must include the non-suitability disclaimer:

> OpenBMP is an academic simulation platform. It is not validated for
> operational flight, not suitable for hardware deployment, and not a
> substitute for any qualified flight-software stack.

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
- **Lifting re-entry research vehicles** — academic equivalents to
  ESA IXV / NASA X-37B class. Lifting entry, blunt-cone capsules,
  aerocapture / aerobraking textbook problems. See
  [hypersonic-extensions.md](hypersonic-extensions.md).
- **Hypersonic research configurations** — Mach 5–25+ flight at
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
  +----> simulator-local flight controller
  |        (estimator -> autopilot -> mission state machine -> commands)
  |
  +----> commands consumed by simulator-local actuator models
  |
  +----> telemetry (typed channels -> ring buffer -> Parquet/CSV/JSON)
  |
  +----> validation rules / golden output / reports
```

The flight controller shipped in this repository is **inside the simulation loop only**. It does
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
              openbmp-fc                     simulator-local flight controller
              openbmp-sensors                synthetic sensors
            |                  |
   openbmp-vehicle    openbmp-physics        physics models + traits
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

## Capabilities and status

OpenBMP delivers a deterministic simulation kernel; point-mass and rigid-body
dynamics; a layered environment (gravity, atmosphere, magnetics, wind);
aerodynamics from tabulated decks through hypersonic engineering methods;
propulsion, tanks, and recovery devices; a full guidance-navigation-and-control
stack (EKF / MEKF / square-root UKF / IMM estimators; PID / LQR / INDI /
L1-adaptive / MPC autopilots; differential-flatness trajectory tracking;
control allocation; a voter / health / FDIR seam); high-order fixed-step and
adaptive integrators; re-entry trajectory tools; and CSV / JSON / Parquet
telemetry with a byte-stable determinism contract.

Each capability carries a validity envelope and a validation label
(`experimental`, `checked`, `validated-toy`, `research`) and fails closed
outside its envelope. For the precise map of what is shipped and validated,
shipped at research grade, or deferred, see [`roadmap.md`](roadmap.md). For the
binding list of what OpenBMP will never ship, see
[`safety-boundaries.md`](safety-boundaries.md).

