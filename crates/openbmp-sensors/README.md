# openbmp-sensors

L3 synthetic sensors and fault models.

**Status:** Phase 2/3 — stub.

## Purpose

Synthesize noisy measurements from simulated truth state. **No device
drivers, no real bus protocols, no real sensor parameters.**

- `SyntheticSensor` trait.
- `IdealStateSensor` — full truth (test only).
- `SyntheticImu` — Allan-variance per IEEE 1139.
- `SyntheticBarometer`, `SyntheticGnss`, `SyntheticMagnetometer`,
  `SyntheticStarTracker`.
- `FaultModel` family for scenario-injected faults.

## Inputs and Outputs

Truth state + environment + deterministic RNG → noisy measurement.

## Units and Frames

IMU outputs in `Body`. GNSS in `ECI`. Barometer scalar (Pa).
Magnetometer in `Body`.

## Assumptions

Sensors synthesize measurements from simulated truth. They are not device
drivers and do not model any specific fielded sensor package.

## Validity Range

Each synthetic sensor declares rate, noise-class, dropout, and bias validity
ranges. Invalid rates and unsupported noise classes fail closed.

## Determinism

RNG seeded per `(scenario_seed, step_index, channel_id)` from
`openbmp-core::DeterministicRng`. Same seed → same measurement.

## Validation

`experimental` (stub). Phase 2/3 validation covers deterministic noise replay
and academic noise-budget checks.

## Data Provenance

Noise budgets drawn from published academic textbook noise classes
(tactical-grade / consumer-grade IMU, etc.). **No noise budgets lifted
from real fielded sensor datasheets** — academic ranges only.

## Safety Boundary

Synthetic sensors are not deployable, not real device drivers, not
hardware integrations.

## References

- IEEE 1139-2008.
- Niskanen 2009.
- Generic published academic noise budgets only.
