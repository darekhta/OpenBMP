# openbmp-propulsion

L2 propulsion crate.

**Status:** Phase 2 / 3.6 — solid motors (synthetic + public Estes
B4/C6/D12 in `data/motors/*.toml`) plus liquid engines and engine
clusters (Phase 3.6). Hybrid / cold-gas / chamber-pressure engine
variants are deferred past 3.6.

## Purpose

- Phase-2 `Motor` trait + `SolidMotor` impl: time-driven thrust /
  mass / mass-rate lookups; impulse-weighted propellant depletion.
  Used for the legacy `[propulsion.motor]` scenario block.
- Phase-3.6 `EngineModel` trait + `LiquidEngine` reference impl:
  per-engine throttle / gimbal / ignition lifecycle. State machine
  is `Idle → Igniting → Burning → Shutdown`. Mass flow `mdot =
  thrust / (g0 · Isp)`. Gimbal applied as locked-order pitch-then-yaw
  rotation of nominal body-`+z` thrust.
- Phase-3.6 `EngineCluster` propulsion-side container: holds
  `Vec<Box<dyn EngineModel>>`, body-frame mount points, and a
  layout tag (`Axial | Ring | Octaweb | Custom`). Supports
  `apply_command(id, cmd)` and `step(dt)`.
- Four canonical fault modes for both `Motor` and `EngineModel`:
  load-time injection only.
- In-house TOML thrust-curve format (RASP `.eng`-shaped) with strict
  `serde(deny_unknown_fields)` schema-1 parser for solid motors.
- Kernel-side adapters (`MotorThrustForceAdapter`, `MotorMassAdapter`,
  `EngineClusterForceAdapter`, `EngineClusterMassAdapter`) live in
  `openbmp-vehicle`. The propulsion crate stays L2 — no kernel
  dependency.

## Inputs and Outputs

Time since ignition → thrust, motor mass, and mass-flow derivative.

## Units and Frames

Thrust in newtons. Kernel-side frame transformation lands with the Phase 2.10
adapter. Mass flow is in kg/s.

## Assumptions

Motor curves are synthetic, textbook, or allowed public educational data.
The crate models simulator-local thrust and mass flow only.

## Validity Range

Each motor declares burn time, thrust curve, and nozzle exit geometry. Thrust
and mass rate are zero outside the burn window; mass clamps to the pre-ignition
or post-burn boundary value.

## Determinism

Thrust curves are tabulated and interpolated linearly with locked operand order.

## Validation

Solid motors: `validated-toy` for the shipped synthetic motors
(`textbook`, `d-class`) and scenario-backed Estes C6/D12 cases;
`checked` for the standalone B4 import. The Phase-2.6 regression
suite asserts integrated impulse vs. declared total to 1e-12
relative, mass-at-burnout bit-equality with `dry_mass`, mass-rate
≤ 0 everywhere, monotone mass decrease, thrust-at-grid-corner
exact-equality, thrust-outside-window zero, and bit-stable lookups
across two evaluations.

Liquid engines: `Checked` per `LiquidEngine::validation()`.
Phase-3.6 in-crate tests cover the lifecycle state machine,
ignition / shutdown transient linearity, throttle clamping, gimbal
clamping + locked-order rotation, mass-flow derivation from thrust
/ Isp, all four fault modes, bit-stable replay, and cluster
summation. End-to-end exercise via
`crates/openbmp-cli/tests/engine_cluster_e2e.rs` against the
canonical 4-engine octaweb scenario.

## Data Provenance

Synthetic textbook motors plus public Estes hobby motor files (B4,
C6, D12) derived from ThrustCurve.org RASP data, each with a
SHA-256-pinned source digest in
`data/motors/provenance.md`. **Real fielded operational motor data
is categorically rejected.**

## Safety Boundary

No operational motor data, no real fielded engine curves, and no propulsion
configuration intended for payload delivery or weapon employment.

## References

- RASP `.eng` motor format — public amateur-rocketry standard,
  re-implemented in-house.
- OpenRocket / Niskanen 2009 — version-controlled motor database
  pattern.
