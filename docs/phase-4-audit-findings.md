# Phase 4 Audit Findings

Audit date: 2026-04-30. Baseline commit audited:
`3474b2f` (`Phase 4.A-B: openbmp-fc autopilot framework + close-out hardening`).

Scope: Phase 4.A/B claims, `openbmp-fc` workspace integration, constants
authority, tests, and state-of-the-art gaps. Resolutions are either
`fixed in this audit` or `Phase 4.C work item`.

## Claim Matrix

| Claim | Plan source | Code reference | Status | Severity |
|---|---|---|---|---|
| `EstimatorStatus` surfaces chi-square diagnostics and dead-reckoning | `docs/phase-4-plan.md`, `docs/phase-4b-plan.md` | `crates/openbmp-fc/src/estimator.rs:543` | Fixed: per-tick transient status clears in `begin_tick`; rejection now propagates through `innovation_rejected` | high |
| Actuator topic reports saturation | `docs/phase-4-plan.md` | `crates/openbmp-fc/src/autopilot.rs:301` | Fixed: trajectory, attitude, and rate-loop saturation now all feed `ActuatorCommand.saturated` | high |
| Health aggregates sensor staleness, estimator dead-reckoning, scheduler overruns | `docs/phase-4b-plan.md` C6 | `crates/openbmp-fc/src/health.rs:54` | Verified: staleness tracks bus sequence advance, not embedded sample timestamps | closed |
| Voter integration surfaces per-sensor divergence | `docs/phase-4b-plan.md` B3 | `crates/openbmp-fc/src/sensor_ingest.rs:384` | Fixed for voted IMU: `sensor.status` now carries lane health/divergence; baro/GNSS/mag extension deferred | medium |
| Runner constructs jobs around `SyntheticSensorAdapter` instances | `docs/phase-4b-plan.md` F2 | `crates/openbmp-fc/tests/sensor_ingest.rs:58` | Partly fixed by test coverage. Kernel runner bridge remains Phase 4.C by explicit scope | high, deferred |
| Scenario `[fc]` block drives real closed-loop runs | `docs/phase-4-plan.md` landed section | `crates/openbmp-cli/src/runner/fc.rs:63` | Overstated: `FcRunner` builds a controller but `phase2_*.rs` never invokes it | high, deferred |
| QP-solver vetting exists for MPC/LCvxLD/SCvx | `docs/phase-4-plan.md` Phase 4.C deferrals | no vetting file | Not present. Candidate list moved to `docs/phase-4c-plan.md` | medium |
| `[forces]` interacts with FC | `docs/phase-4-plan.md`, scenario fixture | `crates/openbmp-cli/src/runner/phase2_point_mass.rs`, `phase2_rigid_body.rs` | Decoupling is correct today: FC is not on the kernel path, so force evaluation is unchanged. Bridge spec added for Phase 4.C | medium |
| `openbmp-fc` remains HAL-portable | `docs/software-architecture.md` | `crates/openbmp-fc/Cargo.toml` | Preserved: no sim/env/cli dependency added; shared physics is HAL-portable | closed |

## Axis 1 - Plan Versus Code

| Finding | Severity | Resolution |
|---|---|---|
| The landed section claimed algorithm-fidelity closure, but baseline EKF used non-Joseph covariance updates, hard-coded gates, and stale transient status. | high | Fixed in `crates/openbmp-fc/src/estimator.rs:406`, `:426`, `:557`, `:611`. |
| `ActuatorCommand.saturated` only reflected rate-loop saturation. | high | Fixed in `crates/openbmp-fc/src/autopilot.rs:301` and `:318`. |
| `FcRunner` exists but is not called by `phase2_point_mass.rs` or `phase2_rigid_body.rs`; the closed-loop fixture parses but does not produce Parquet through the kernel runner. | high | Phase 4.C. This is explicitly out of scope for this audit's code changes. |
| Health staleness uses bus sequence counters and correctly catches publishers that stop after one publish. | low | Verified; no code change. |
| QP solver vetting list was empty. | medium | Added to `docs/phase-4c-plan.md`. |

## Axis 2 - Workspace Integration

| Finding | Severity | Resolution |
|---|---|---|
| FC duplicated USSA76, gravity, and magnetic constants already owned by the workspace. | high | Fixed by new HAL-portable `openbmp-physics` crate (`crates/openbmp-physics/src/lib.rs:10`) consumed by `openbmp-env` and `openbmp-fc`. |
| FC actuator commands were semantic channel names with no vehicle `EffectorId` mapping. | high | Fixed by `ActuatorChannelMap` and `EffectorCommandSet` in `crates/openbmp-fc/src/mixer.rs:26` and `crates/openbmp-fc/src/topics.rs:274`; `FcRunner` builds authority from mission phase allowed effectors at `crates/openbmp-cli/src/runner/fc.rs:441`. |
| Sensor sample types are intentionally wrapped at the FC bus boundary. | low | Kept as the right seam: `SensorMeasurement` remains the source shape; `ImuSample` adds bus timestamp/health fields. |
| Kernel-to-FC state translation is absent. | high | Phase 4.C bridge spec added. Translation belongs in `openbmp-cli/src/runner/fc.rs`, not in `openbmp-fc`. |

## Axis 3 - Constants Authority

| Constant / authority gap | Severity | Resolution |
|---|---|---|
| Standard gravity, sea-level pressure, molar mass, gas constants, tropopause constants duplicated in `estimator.rs`. | high | Deleted FC-local formula; `Ekf::update_baro` uses `openbmp_physics::atmosphere::pressure_altitude_troposphere_m` at `crates/openbmp-fc/src/estimator.rs:438`. |
| ECI constant gravity default used `-9.81`. | high | Uses `openbmp_physics::gravity::STANDARD_GRAVITY_M_S2` at `crates/openbmp-fc/src/estimator.rs:66`. |
| Mean Earth radius and dipole equatorial field duplicated in `magnetic.rs`. | high | Uses `openbmp_physics::{earth, magnetic}` at `crates/openbmp-fc/src/magnetic.rs`. |
| EKF chi-square gate default was `9.0` across 1-D/3-D/6-D updates. | high | Defaults now derive per DOF from false-alarm rate (`crates/openbmp-fc/src/estimator.rs:163`, `:622`). |
| Frame budget and 100 Hz job periods were hard-coded in `FcRunner`. | medium | Frame budget is required in `[fc]`; slow periods derive from `base_rate_hz` at `crates/openbmp-cli/src/runner/fc.rs:69` and `:75`. |
| Default gains and health staleness defaults remain FC-local fallbacks. | medium | Phase 4.C: move to scenario/params with provenance and validation. |

## Axis 4 - Test Coverage

| Finding | Severity | Resolution |
|---|---|---|
| Full synthetic ingest path was not exercised. | high | Fixed by `crates/openbmp-fc/tests/sensor_ingest.rs:58`. |
| HAL-portable tests were not proven under `--no-default-features`. | medium | Verified in this audit with `cargo test -p openbmp-fc --no-default-features`. |
| MEKF property tests, Monte-Carlo NEES, lags 2..10, bus/scheduler/voter adversarial suites, FDIR bias injection, GNSS dropout recovery, multi-phase safe state, full bus-history determinism, and 60 s stability are still incomplete. | medium | Scoped in `docs/phase-4c-plan.md`. |

## Axis 5 - State Of The Art

| Finding | Severity | Resolution |
|---|---|---|
| EKF covariance update used `P = (I-KH)P`. | high | Fixed with Joseph symmetric update at `crates/openbmp-fc/src/estimator.rs:611`. |
| Quaternion propagation/update lacked explicit renormalization. | high | Fixed at `crates/openbmp-fc/src/estimator.rs:599`, used after EKF/MEKF propagation and attitude updates. |
| Innovation gates were not derived from a stated false-alarm rate. | high | Fixed with `innovation_false_alarm_rate` in EKF/MEKF params and scenario schema. |
| Square-root/UD filtering, Markley MEKF reset, Gauss-Markov bias dynamics, iterated mag update, notch filters, L1 augmentation, differential-flatness trajectory tracking, covariance-weighted voter, GLRT/CUSUM, and WGS84-J2 adapter remain above the Phase 4.B authorization. | medium | Phase 4.C items with gates. |

## Fixed In This Audit

- Added `openbmp-physics` and moved shared USSA76/gravity/magnetic authority there.
- Reworked EKF/MEKF gates, Joseph covariance update, quaternion renormalization, and transient estimator status.
- Added saturation propagation across trajectory, attitude, and rate loops.
- Added `sensor.status` lane reporting for voted IMU ingest and a full `SyntheticSensorAdapter` ingest test.
- Added effector-id command mapping via `ActuatorChannelMap` and `EffectorCommandSet`.
- Made FC frame budget scenario-driven and scheduler slow periods derive from `base_rate_hz`.

## Phase 4.C Work Items

The Phase 4.C plan owns all medium/low items and out-of-scope high items:
`docs/phase-4c-plan.md`.
