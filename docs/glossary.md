# OpenBMP Glossary

This glossary keeps project vocabulary consistent across documentation,
scenarios, telemetry, and code.

## Project Terms

**OpenBMP**
Open Body Motion Platform. A Rust-first, simulation-only academic research
platform for rigid-body dynamics and rocket-class flight simulation.

**Scenario**
A versioned TOML-shaped file that selects models, initial state, schedule,
telemetry outputs, and validation rules for one simulation run.

**Mission Package**
A versioned SIL manifest that names the scenario, optional plant/I-load,
mission, scenario-script, dictionary, test-case, provenance, and hash inputs
for one reviewable run family.

**Plant Config**
Simulator-owned vehicle, environment, sensor, actuator, and disturbance
configuration. It is not flight-controller mission data.

**Scenario Script**
Simulator-owned test stimulus under `[scenario_script]`, such as engine
commands, effector overrides, recovery deploys, and jettison actions.

**I-load**
Flight-software initialization load. OpenBMP HAL exposes an `OBIL` envelope
with schema version, payload length, CRC, and fallback selection.

**Test Case**
A named package entry selecting a scenario and objective for repeatable SIL
execution.

**Telemetry Dictionary**
Stable command/telemetry metadata generated from OpenBMP topic definitions and
exported as JSON, with experimental interchange exports kept separate.

**Model**
A deterministic Rust implementation of a physical, synthetic sensor,
controller, telemetry, or validation component.

**Kernel**
The lockstep simulation core that advances `SimTime`, evaluates models in a
fixed order, integrates state, and emits telemetry.

**Virtual Flight Controller**
A simulator-local controller framework. It is not deployable flight software
and does not expose real hardware protocols.

**Telemetry**
Typed simulation output channels, exported to Parquet/CSV/JSON with unit,
frame, schema, and version metadata.

## Frames

**ECI**
Earth-centered inertial frame. OpenBMP's canonical propagation frame.

**ECEF**
Earth-centered Earth-fixed frame. Used for Earth-relative models and geodetic
conversion.

**Body**
Vehicle-fixed frame.

**NED**
North-east-down local navigation frame.

**ENU**
East-north-up local navigation frame.

**FrameContext**
An immutable bundle of frame profile, Earth model, epoch, and pinned data
tables used to construct transforms.

## Time

**SimTime**
Monotonic seconds since scenario start. This is not wall-clock time.

**StepIndex**
Monotonic integer step counter used for deterministic scheduling and seeded
random-number generation.

**Platform Profile**
The tuple of target triple, Rust compiler version, and simulation profile used
to define bit-stable replay expectations.

## Determinism

**BitStable**
The same scenario, data, seed, and platform profile produce byte-identical
telemetry.

**StateStable**
The same run is physically repeatable within tolerances, but byte identity is
not guaranteed. Adaptive integrators and cross-platform runs usually fall here.

**Golden Test**
A regression test that compares generated telemetry against committed
reference output.

## Validation

**experimental**
Implemented or drafted, but not independently checked.

**checked**
Internally consistent against unit/property tests.

**validated-toy**
Compared against analytic or simple public examples.

**research**
Compared against public academic benchmark cases. Still not operationally
validated.

## Safety Vocabulary

**Accepted Data**
Synthetic, textbook, public standard, or public academic data with provenance.

**Out-of-Scope Data**
Restricted, unverifiable, or undocumented fielded-vehicle data outside the
repository scope.

**Scenario Waypoint**
A synthetic or academic reference point used for simulator-internal guidance.

**HIL Pattern**
An optional generic socket bridge using an in-house wire format. It is not a
hardware integration framework and ships no real device drivers or bus
protocols.

## Hypersonic Terms

**RealGasState**
Thermodynamic state that may include effective gamma, composition, speed of
sound, Knudsen number, and nonequilibrium information.

**Knudsen Number**
Ratio of mean free path to characteristic length, used to classify continuum,
slip, transition, and free-molecular regimes.

**Modified Newtonian**
A hypersonic local-inclination method that uses a stagnation-point pressure
coefficient rather than a constant Newtonian value.

**BoundaryLayerState**
Laminar, transitional, or turbulent surface-flow classification at a body
station.

**Ablation Toy**
A generic textbook surface-recession model. It is not a real TPS design tool.
