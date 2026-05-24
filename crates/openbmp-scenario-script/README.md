# openbmp-scenario-script

L3 simulator-only scenario-script actions.

**Status:** Implemented. Ships `ScenarioScriptAction` carrying the
four simulator-only physics-override variants (engine command,
effector override, scripted separation, recovery deploy) that the
runner's `EngineRack` / `EffectorRack` / `RecoveryRack` consume each
rack tick. The action taxonomy is separate from the HAL-portable
mission-action vocabulary in `openbmp-mission`.

## Purpose

- Define the `ScenarioScriptAction` enum for simulator-only physics
  overrides — the action vocabulary that **does not** appear in a
  HAL deployment.
- Keep the HAL-portable mission-action vocabulary in
  `openbmp-mission` free of actuator-domain shapes (engine /
  effector / recovery id types). A real-hardware FC adopter links
  `openbmp-mission` alone; this crate is not on their dependency
  graph.

## Inputs and Outputs

Consumed by `openbmp-sim`'s scenario-script binding evaluator;
produced by `openbmp-scenario`'s `[mission]` parser when a binding's
action targets a sim-only physics override.

## Units and Frames

Per-variant scalar payloads carry the same unit / frame conventions
as the actuator-rack interfaces: throttle in `[0, 1]`,
gimbal angles in radians, generic effector commands in deck-defined
units. The crate carries no dimensional types itself.

## Assumptions

The simulator owns the binding evaluator for this action type. A
HAL deployment does not link this crate; the FC commander never
sees `ScenarioScriptAction`.

## Validity Range

The action vocabulary is locked as part of the scenario format v4
contract.

## Determinism

The crate ships only data shapes; no behavior. Determinism is the
consumer's responsibility (the simulator kernel's event evaluator
preserves canonical id-sorted iteration order across both binding
lists).

## Validation

Per-variant docstring tests; cross-crate compile test in
`openbmp-sim` confirms the action type round-trips through the
binding generic. The determinism CI gate exercises every
shipped scenario that uses these variants.

## Data Provenance

Crate carries no external data. See
[`docs/data-provenance.md`](../../docs/data-provenance.md) for the
project-wide policy.

## Safety Boundary

OpenBMP is an academic simulation platform. It is **not** validated
for operational flight, **not** suitable for hardware deployment,
**not** a weapon system. The action variants in this crate are
scenario-script overrides for academic study; they do not encode
real-launch authorities or operational targeting logic. See
[`docs/safety-boundaries.md`](../../docs/safety-boundaries.md) and
[`docs/mission-graph-architecture.md`](../../docs/mission-graph-architecture.md)
for the full safety posture.
