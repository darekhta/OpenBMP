# openbmp-sim

L1 lockstep simulation kernel.

**Status:** Phase 3 — Phase-2 point-mass + 6-DOF rigid-body
lockstep kernels plus the Phase-3.2 `EventTrigger` /
`MissionPhaseGraph` event-driven scheduling surface and the
Phase-3.7 slosh sub-step plumbing
(`SimulationKernel::sub_step_count`).

## Purpose

- `Integrator` trait + `IntegratorDeterminism` enum.
- `Rk4FixedStep` integrator with locked weighted-sum order; reused
  across both `PointMassState` and `RigidBodyState` (Phase 2.1
  generalisation; Phase-1 byte-stable analytic-toy path is preserved
  by the `Phase1Kernel` alias).
- `SimulationKernel` struct with the canonical step loop:
  stop check → overflow-checked step candidate → environment sample →
  force / moment model → mass-rate model → integrate →
  canonical time overwrite → validation.
- `RigidBodyKernel` alias plus `RigidModels` bundle for the 6-DOF
  path: `MomentModel`, `RigidMassModel`, post-step quaternion
  renormalisation in `project()`.
- `ConstantGravityForce` retained as the Phase-1 byte-stable
  analytic-toy gravity scaffold; the higher-layer `GravityForceAdapter`
  in `openbmp-vehicle` is the Phase-2 path.
- `StopCondition` trait and simple stop conditions
  (`AlwaysContinue`, `EndTime`, `MaxSteps`).
- Phase-3.2 mission scheduling: `EventTrigger` trait,
  `BuiltInEventTrigger` (AtTime / AtAltitudeAscending /
  AtAltitudeDescending / AtApogee / AtMassFraction /
  AtDynamicPressure), typed `EventBinding` lists, and
  `MissionPhaseGraph` with cycle-rejection at construction time and
  event-driven phase transitions.
- `ModelEvalError` typed error surface; RK stages short-circuit
  fail-closed and the kernel records `(step_index, model_id)`.
- `SimulationError`, `IntegratorError`, `StopReason` error types.

## Inputs and Outputs

- Inputs: a valid initial state (`PointMassState` or `RigidBodyState`),
  an environment provider, force / moment / mass models, an
  integrator, a stop condition, a fixed time step, and a scenario seed.
- Outputs: current state, current step/time, and stop reason on
  termination. Telemetry wiring is provided by the runner in
  `openbmp-cli`.

## Units and Frames

Translational propagation is in `ECI`. Rigid-body body-frame moments
land via `MomentModel<RigidBodyState>`; quaternion attitude
integration is body-to-ECI and renormalised at the end of each step.

## Assumptions

The kernel is synchronous, single-threaded, and lockstep. Models are evaluated
in a fixed order declared by the scenario.

## Validity Range

Valid for scenarios whose selected models satisfy the default determinism
profile. Adaptive or state-stable profiles are future explicit opt-ins.

## Determinism

Fixed-step lockstep. No wall-clock, no allocation on hot path.
FMA-disabled on the reference platform per `.cargo/config.toml`. See
`docs/software-architecture.md` § Determinism Profile.

## Validation

`checked` for the Phase-1 point-mass kernel surface and
`validated-toy` for the Phase-2.1 rigid-body extension and the
Phase-3.2 mission graph. Validated against the analytic-toy
constant-acceleration drop (`tests/analytic_toy.rs`), torque-free
precession (`tests/torque_free_precession.rs`), the
Phase-2.9/2.11 Niskanen sounding-rocket reduction, and the
Phase-3.11 RocketPy Calisto cross-tool case (via the
`openbmp-vehicle` and `openbmp-cli` integration tests).

## Data Provenance

This crate ships no data files. Analytic-toy fixtures cite their derivations
in validation metadata.

## Safety Boundary

No real-time guarantees, no actuator packets, no external command/
control links. See `docs/safety-boundaries.md`.
