# openbmp-state

L1 state types crate. Sits between `openbmp-core` (foundation) and
`openbmp-sim` (kernel).

**Status:** Implemented state-type layer.

## Purpose

- `PointMassState` — 3-DOF position / velocity / mass.
- `RigidBodyState` — 6-DOF position / velocity / orientation /
  angular velocity / mass properties.
- `MassProperties` — mass, body-frame center of mass, body-frame
  inertia tensor.
- Projection from rigid-body to point-mass state and validating
  point-mass promotion to rigid-body state.
- Validation: literal finiteness checks, structural validity checks,
  quaternion normalisation, positive mass, symmetric positive-definite
  inertia, and rigid-body inertia triangle inequalities.

## Inputs and Outputs

Pure data containers; no IO. Explicit constructors are used today;
builders remain a future ergonomic addition.

## Units and Frames

`uom` quantities at the API. `Position3<ECI>` and `Velocity3<ECI>` are
the canonical inertial-state encoding. `Quaternion<Body, ECI>` is the
canonical attitude encoding.

## Assumptions

State values are finite, frame-tagged, and validated before use in the
kernel. Builders encode invariants where practical.

## Validity Range

Valid ranges are scenario/model dependent. This crate provides structural
validation helpers such as finite state, positive mass, and normalized
orientation.

## Determinism

No allocation on hot paths when the kernel consumes these
types. Pure data, no system access.

## Validation

`checked` for the structural validation helpers.

## Data Provenance

This crate ships no data files.

## Scope Boundary

State types are physics-neutral; no operational vocabulary. The
project's full accept/reject list lives in
[`docs/safety-boundaries.md`](../../docs/safety-boundaries.md).

## See Also

- [`docs/software-architecture.md`](../../docs/software-architecture.md)
  — frame conventions, state-type contracts, and the kernel's
  determinism profile that this crate's types satisfy.
- [`docs/frames-time.md`](../../docs/frames-time.md) — the canonical
  ECI / ECEF / NED / ENU / Body frame profile vocabulary used by the
  type-tagged `Position3` / `Velocity3` / `Quaternion` here.
- [`docs/glossary.md`](../../docs/glossary.md) — shared vocabulary
  for state, time, frames, validation labels, and determinism.
