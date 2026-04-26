# openbmp-fc

L4 virtual flight controller.

**Status:** Phase 4 — stub.

## Purpose

Simulator-local controller framework. Outputs are abstract normalized
commands consumed by simulator-local actuator models or the optional
generic socket bridge. **No real hardware protocols, no targeting, no
terminal-homing.**

- `Estimator` — `Ekf` (15-state error-state per NaveGo / NorthStarUAS
  pattern), `Mekf`, `Ukf` (Phase 4+).
- `Autopilot` — `ThreeLoopAutopilot` (Stevens & Lewis 2015).
- `MissionStateMachine` — academic phases per `docs/glossary.md`.
- Academic guidance: `AttitudeHold`, `RateHold`, `WaypointTrack`
  (scenario waypoints, NOT real-world targets), `GravityTurnReference`.
- `Fdir` — scenario-injected faults; residual-based detection.

## Inputs and Outputs

`FcInput` (sensor measurements, mission phase, scripted references) →
`FcOutput` (normalized commands, state estimate, mission phase, FDIR
status).

## Units and Frames

Estimator state in `ECI` for position / velocity, body-frame for
attitude. Commands abstract normalized values in `[-1, 1]`.

## Assumptions

All controller code is simulator-local. Gains, references, and guidance modes
are scenario-defined academic data, not operational tuning or deployment
logic.

## Validity Range

Each estimator, autopilot, and guidance mode declares its own rate, state,
sensor, and reference validity ranges. Unsupported phases or references fail
closed.

## Determinism

Filter inner-loop sub-step counts declared in scenario. No wall-clock,
no system RNG, no allocation on hot path.

## Validation

`experimental` (stub). Phase 4 validation uses analytic attitude/rate
tracking and synthetic sensor scenarios only.

## Data Provenance

Any shipped gain sets or noise models are synthetic or textbook examples with
provenance. No real fielded controller gains ship in this crate.

## Safety Boundary

`AttitudeHold`, `RateHold`, `WaypointTrack` (scenario-defined inertial
waypoints), `GravityTurnReference` are the only guidance laws shipped.

**Explicitly NOT shipped:** proportional navigation, augmented PN,
sliding-mode terminal-homing, target-tracking guidance, intercept
geometry, terminal-mode autopilots (BTT/STT), TERCOM, DSMAC,
scene-matching. See `docs/safety-boundaries.md`.

## References

- Stevens & Lewis 2015.
- NaveGo (Rodríguez et al.); NorthStarUAS `insgnss_tools`; INSTINCT
  (U. Stuttgart).
