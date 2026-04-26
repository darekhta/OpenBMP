# openbmp-sim

L1 lockstep simulation kernel.

**Status:** Phase 1.3 — stub. Compiles but provides no functionality.

## Purpose

- `Integrator` trait + `IntegratorDeterminism` enum.
- `RK4FixedStep` integrator (Phase 1.3 default).
- `SimulationKernel` struct with the canonical step loop:
  environment → forces / moments → integrate → sensors → controller →
  telemetry → validation → stop check.
- `EventQueue` for stop conditions.
- `SimulationError`, `IntegratorError`, `StopReason` error types.

## Inputs and Outputs

- Inputs: a validated scenario, an environment provider, force / moment
  providers, a mass model, sensors, a virtual flight controller.
- Outputs: telemetry samples per step, stop reason on termination.

## Units and Frames

State propagation in `ECI`. Body-frame inputs from the controller and
moments. See `docs/frames-time.md`.

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

`experimental` (stub). Validated against analytic-toy
constant-acceleration drop in Phase 1.8.

## Data Provenance

This crate ships no data files. Analytic-toy fixtures cite their derivations
in validation metadata.

## Safety Boundary

No real-time guarantees, no actuator packets, no external command/
control links. See `docs/safety-boundaries.md`.
