# openbmp-sim

L1 lockstep simulation kernel.

**Status:** Implemented. Provides point-mass + 6-DOF rigid-body
lockstep kernels plus the `EventTrigger` /
`MissionPhaseGraph` event-driven scheduling surface and the
slosh sub-step plumbing
(`SimulationKernel::sub_step_count`).

## Purpose

- `Integrator` trait + `IntegratorDeterminism` enum.
- `Rk4FixedStep` integrator with locked weighted-sum order; reused
  across both `PointMassState` and `RigidBodyState`. The point-mass
  kernel shape is exposed by the `PointMassKernel` alias.
- `SimulationKernel` struct with the canonical step loop:
  stop check → overflow-checked step candidate → environment sample →
  force / moment model → mass-rate model → integrate →
  canonical time overwrite → validation.
- `RigidBodyKernel` alias plus `RigidModels` bundle for the 6-DOF
  path: `MomentModel`, `RigidMassModel`, post-step quaternion
  renormalisation in `project()`.
- `ConstantGravityForce` is the byte-stable
  analytic-toy gravity scaffold; the higher-layer `GravityForceAdapter`
  in `openbmp-vehicle` is the sounding-rocket path.
- `PhaseGatedForceModel` wraps one default force stack plus
  phase-specific overrides, selecting by the mission phase id the
  kernel passes through `ForceContext`.
- `StopCondition` trait and stop conditions
  (`AlwaysContinue`, `EndTime`, `MaxSteps`, `GroundImpact`) plus
  `AnyStop` for composing terminal predicates.
- Mission scheduling: `EventTrigger` trait,
  `BuiltInEventTrigger` (AtTime / AtAltitudeAscending /
  AtAltitudeDescending / AtApogee / AtMassFraction /
  AtVelocity / AtDynamicPressure), typed `EventBinding` lists, and
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

`checked` for the point-mass kernel surface and
`validated-toy` for the rigid-body extension and the
mission graph. Validated against the analytic-toy
constant-acceleration drop (`tests/analytic_toy.rs`), torque-free
precession (`tests/torque_free_precession.rs`), the
Niskanen sounding-rocket reduction, and the
RocketPy Calisto cross-tool case (via the
`openbmp-vehicle` and `openbmp-cli` integration tests).

## Data Provenance

This crate ships no data files. Analytic-toy fixtures cite their derivations
in validation metadata.

## Scope Boundary

No real-time guarantees, no actuator packets, no external command/
control links. See `docs/safety-boundaries.md`.
