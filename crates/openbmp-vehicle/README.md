# openbmp-vehicle

L2 vehicle composition crate.

**Status:** Phase 2 — stub.

## Purpose

- `Vehicle` trait per `docs/software-architecture.md`.
- Mass models: `ConstantMass`, `LinearBurn`, `TableBurn`, `MultiStage`.
- Vehicle composition logic: a vehicle is a list of force / moment /
  mass providers registered in the scenario.

## Inputs and Outputs

Composition only — vehicle behaviour is the sum of its registered
providers.

## Units and Frames

Forces in `ECI`, moments in `Body`, mass properties in `Body`.

## Assumptions

Vehicles are compositions of simulator-local models. The crate does not ship
real fielded-vehicle mass properties or operational staging data.

## Validity Range

Each vehicle model declares mass, inertia, geometry, staging, and composition
validity ranges. Invalid mass properties or unsupported stage events fail
closed.

## Determinism

Composition order is scenario-declared and stable. No allocation on
the kernel hot path once Phase 2 closes.

## Validation

`experimental` (stub). Phase 2 validation starts with constant-mass and
linear-burn consistency checks.

## Data Provenance

Toy vehicles are synthetic. Any public example vehicle data must include
provenance and must not be a real operational parameter set.

## Safety Boundary

Generic vehicle composition only. No real fielded-vehicle parameter
sets ship in this crate; toy academic vehicles only.
