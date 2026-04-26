# openbmp-sim

L1 lockstep simulation kernel.

**Status:** Phase 1.3 — implemented point-mass lockstep kernel.

## Purpose

- `Integrator` trait + `IntegratorDeterminism` enum.
- `Rk4FixedStep` integrator (Phase 1.3 default).
- `SimulationKernel` struct with the canonical step loop:
  stop check → overflow-checked step candidate → environment sample →
  force model → mass-rate model → integrate → canonical time overwrite
  → validation.
- `StopCondition` trait and simple stop conditions.
- `SimulationError`, `IntegratorError`, `StopReason` error types.

## Inputs and Outputs

- Inputs: a valid initial `PointMassState`, an environment provider, a
  force model, a mass-rate model, an integrator, a stop condition, a
  fixed time step, and a scenario seed.
- Outputs: current state, current step/time, and stop reason on
  termination. Telemetry wiring lands in a later phase.

## Units and Frames

Point-mass state propagation is in `ECI`. Rigid-body body-frame
moments and quaternion attitude integration are deferred to the
rigid-body extension.

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

`checked` for the Phase 1.3 point-mass kernel surface. The crate is
validated against the analytic-toy constant-acceleration drop.

## Data Provenance

This crate ships no data files. Analytic-toy fixtures cite their derivations
in validation metadata.

## Safety Boundary

No real-time guarantees, no actuator packets, no external command/
control links. See `docs/safety-boundaries.md`.
