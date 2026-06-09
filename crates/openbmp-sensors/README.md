# openbmp-sensors

L2 synthetic sensors.

**Status:** Implemented. Ships `IdealStateSensor`, `SyntheticImu`,
and `SyntheticBarometer` plus the extended sensor set:
`SyntheticGnss` (per-axis Gaussian position / velocity noise +
position-bias OU drift, IS-GPS-200 nominal noise budget),
`SyntheticMagnetometer` (body-frame WMM 2025 truth + Gaussian
noise + soft-iron + hard-iron bias), and `SyntheticStarTracker`
(per-axis Gaussian quaternion-error injection). Multi-rate
scheduling and sensor fault models remain future work.

## Purpose

Synthesize noisy measurements from simulated truth state. **No device
drivers, no real bus protocols, no real sensor parameters.**

- `SyntheticSensor` trait.
- `SensorTruth` / `SensorMeasurement` typed surface.
- `IdealStateSensor` -- bit-equal truth echo.
- `SyntheticImu` -- IEEE 952 five-component noise model.
- `SyntheticBarometer` -- Gaussian pressure noise plus OU bias drift.
- `SyntheticGnss` -- IS-GPS-200 nominal noise budget plus tunable
  position / velocity sigmas and OU position-bias drift.
- `SyntheticMagnetometer` -- WMM 2025 body-frame truth plus
  Gaussian noise and soft-iron / hard-iron biases.
- `SyntheticStarTracker` -- per-axis Gaussian quaternion-error
  injection (small-angle approximation).

Multi-rate scheduling, per-axis IMU budgets, and sensor fault
models are future work.

## Inputs and Outputs

Truth state + environment + deterministic RNG → noisy measurement.

## Units and Frames

IMU outputs angular rate and specific force in `Body`. Barometer output
is scalar static pressure in Pa. `IdealStateSensor` echoes the whole
truth bag.

## Assumptions

Sensors synthesize measurements from simulated truth. They are not device
drivers and do not model any specific fielded sensor package.

## Validity Range

Noise budgets use SI units and validate finite, non-negative noise
parameters plus positive sample intervals. The
`SyntheticMagnetometer` requires a `MagneticModel` (the shipped
`Wmm2025` is in-epoch through 2030-01-01); out-of-epoch queries
fail closed. Unsupported sensor classes (multi-rate, fault
injection) remain future work.

## Determinism

RNG seeded per `(scenario_seed, step_index, sensor_id, component_id)`
through `openbmp-core::DeterministicRng::for_sensor_component`.
`SensorId` is path-derived and the sensor-component constructor is
domain-separated from telemetry `for_channel` streams.

## Validation

`validated-toy` for the shipped synthetic budgets. The suite
validates deterministic replay, schema parsing, and the ARW
Allan-deviation `-1/2` slope on a synthetic ARW-only IMU stream.
It also validates `SyntheticGnss` truth-bypass mode (zero
noise = bit-equal truth), `SyntheticMagnetometer` against the WMM
2025 body-frame field, and `SyntheticStarTracker` against the
small-angle quaternion-error budget.

## Data Provenance

Noise budgets are OpenBMP-authored synthetic envelopes shaped by
published academic methodology (tactical-grade / consumer-MEMS IMU
classes). **No noise budgets are lifted from real fielded sensor
datasheets** -- academic ranges only.

## Scope Boundary

Synthetic sensors are not deployable, not real device drivers, not
hardware integrations. The project-wide accept/reject list lives in
[`docs/safety-boundaries.md`](../../docs/safety-boundaries.md); the
sensor naming rules and the "no fielded data" policy are summarised
there.

## See Also

- [`docs/software-architecture.md § Synthetic Sensors`](../../docs/software-architecture.md#synthetic-sensors)
  — the trait surface, the noise-budget rule, and the fault-model
  framework this crate implements.
- [`docs/data-provenance.md`](../../docs/data-provenance.md) — the
  provenance contract noise-budget tables must satisfy when they
  ship.
- [`docs/modeling-guide.md`](../../docs/modeling-guide.md) — the
  per-model documentation contract.

## References

- IEEE Std 952-2020.
- El-Sheimy, Hou, and Niu 2008.
- Hou 2004.
