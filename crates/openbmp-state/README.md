# openbmp-state

L1 state types crate. Sits between `openbmp-core` (foundation) and
`openbmp-sim` (kernel).

**Status:** Phase 1.2 — stub. Compiles but provides no functionality.

## Purpose

- `PointMassState` — 3-DOF position / velocity / mass.
- `RigidBodyState` — 6-DOF position / velocity / orientation /
  angular velocity / mass properties.
- `MassProperties` — mass, body-frame center of mass, body-frame
  inertia tensor.
- `From` / `TryFrom` between point-mass and rigid-body states.
- Validation: `is_finite`, `is_normalised`, etc.

## Inputs and Outputs

Pure data containers; no IO. Builders via `bon`.

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

No allocation on hot paths once Phase 1.3 begins consuming these
types. Pure data, no system access.

## Validation

`experimental` (stub).

## Data Provenance

This crate ships no data files.

## Safety Boundary

State types are physics-neutral; no operational vocabulary.
