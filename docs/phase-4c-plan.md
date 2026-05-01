# Phase 4.C Plan

Purpose: close the audit deferrals without expanding OpenBMP beyond its
simulator-local, non-deployable safety boundary.

## Execution State (2026-04-30)

This plan has an initial implementation pass in tree:

- P0 bridge is implemented in `crates/openbmp-cli/src/runner/fc_bridge.rs`
  and wired into the point-mass / rigid-body Phase-2 runners.
- P2 landed Markley-style MEKF reset, Gauss-Markov bias dynamics,
  iterated magnetometer updates, WGS84-J2 gravity through
  `openbmp-physics`, a feature-gated UD covariance-factor helper, and
  a real 6-state attitude + gyro-bias UKF.
- P3 landed gyro notch filters, flatness-inspired attitude reference
  generation, feature-gated L1-inspired rate-loop
  augmentation, and the anti-windup decision note keeping
  back-calculation as baseline.
- P4 landed per-kind sensor lane status, covariance-weighted scalar
  voting, and burst-counter / single-sample GLRT / CUSUM FDIR families.
- P5 landed WGS84-J2 and WMM 2025. NRLMSISE-00, EGM2008,
  multi-instance estimator routing, square-root UKF, and full
  external trajectory cross-validation are tracked in
  `docs/phase-5-plan.md`.

The solver decision is recorded in `docs/clarabel-vetting.md`.
Clarabel-backed QP / SOCP primitives are feature-gated; full
receding-horizon MPC and LCvxLD / SCvx trajectory reproduction are not
claimed by this implementation pass.

## Gates

1. Preserve `openbmp-fc` HAL portability: no `openbmp-sim`, `openbmp-cli`,
   `openbmp-scenario`, `openbmp-telemetry`, `openbmp-bridge`,
   `openbmp-aerothermal`, or `openbmp-env` dependency from `openbmp-fc`.
2. Every numeric table or physical constant has provenance or lives in
   `openbmp-physics`.
3. `cargo fmt --all -- --check`, clippy, workspace tests, no-default FC tests,
   `cargo deny check`, and `cargo machete` are clean.
4. New solver dependencies require an explicit vetting record before use.

## P0 - Kernel To FC Bridge

Implement the lockstep bridge in `crates/openbmp-cli/src/runner/phase2_point_mass.rs`
and `phase2_rigid_body.rs`.

Bridge shape:
- Build `FcRunner` when `[fc]` is present.
- Translate `openbmp-state::PointMassState` / `RigidBodyState` plus active
  environment samples into `openbmp_sensors::SensorTruth`.
- Prime `SyntheticSensorAdapter` instances once per kernel tick.
- Register voted ingest jobs from those adapters instead of publishing FC bus
  samples directly.
- Step FC after sensor publication and before force/moment evaluation consumes
  effector/engine commands.
- Convert `EffectorCommandSet` entries by `EffectorId` into `EffectorRack`
  commands; convert `EngineDemand` by `EngineId` into `EngineRack` commands.
- Emit Parquet/CSV telemetry proving `[fc]` scenarios produce controller
  output in normal kernel runs.

Translation details:
- `PointMassState.position` / `velocity` map to `SensorTruth.position_eci` /
  `velocity_eci`.
- `RigidBodyState.attitude` maps to `SensorTruth.attitude_eci_to_body`;
  body rates map to `angular_velocity_body_rad_s`.
- Specific force is non-gravitational acceleration in body axes; do not include
  gravity in the accelerometer truth.
- Barometer uses active atmosphere static pressure and geometric altitude.
- Magnetometer uses the active magnetic environment field rotated to body axes.

## P1 - Test Truthfulness

- MEKF property tests: deterministic reruns and long-run attitude-error
  whitening.
- Monte-Carlo NEES for the EKF state subspace with known truth.
- Innovation autocorrelation lags 2 through 10.
- Bus property tests: sequence monotonicity, single-writer/multi-reader,
  order-invariant read registration, order-defining iteration.
- Scheduler stress tests with adversarial periodic/topic-driven budgets.
- Voter adversarial tests: one stuck lane, one drifting lane, two divergent
  lanes, and expected median-select behavior.
- FDIR bias injection: 1 sigma, 3 sigma, 10 sigma burst-counter behavior.
- GNSS dropout/recovery: estimator, health, and commander state all flip and
  clear correctly.
- Multi-phase mission with safe-state/recovery phase and `safe_state_requested`.
- Full bus-history determinism across reruns.
- 60 s no-NaN/no-covariance-blowup/no-integrator-drift stability run.

## P2 - Estimator SOTA Upgrades

- Markley-form MEKF reset.
- First-order Gauss-Markov gyro/accel bias dynamics with scenario-configurable
  time constants.
- Iterated EKF magnetometer update.
- Square-root or UD-factorized EKF covariance path.
- Optional WGS84-J2 gravity adapter — `openbmp-fc` now consumes
  `openbmp_physics::gravity::J2Gravity` directly through the
  consolidated physics crate (the `openbmp-env` shim was retired
  and folded into `openbmp-physics` per
  `docs/physics-consolidation-plan.md`).
- Real sigma-point UKF. Keep deleted scaffold out of tree until this is real.

## P3 - Autopilot SOTA Upgrades

- Configurable rate-loop biquad/notch filter coefficients in `[fc.autopilot_params]`.
- Flatness-inspired trajectory loop for guided academic rocketry cases.
- L1-inspired augmentation behind a feature flag.
- Observer-form anti-windup investigation; keep back-calculation as baseline
  unless tests prove the observer implementation improves boundedness.
- Move health defaults and fallback gain sets into scenario-driven parameters
  with provenance.

## P4 - Voter / FDIR

- Extend `sensor.status` lane reporting from voted IMU to barometer, GNSS, and
  magnetometer ingest jobs.
- Add covariance/innovation-weighted voter mode with static fallback.
- Add GLRT or CUSUM residual detector.
- Populate fault-isolation metadata beyond the current bitmask.

## P5 - Environment Models

- **Full WMM 2025 spherical-harmonic geomagnetic model** — already in
  the workspace at `openbmp_physics::magnetic::Wmm2025` after the
  consolidation. The FC's degree-1 `EarthDipoleField` placeholder is
  superseded for scenarios that set `mag_field = "wmm_2025"` in
  `[fc.ekf]` or `[fc.mekf]`; the runner bridge uses the same
  selection for synthetic magnetometer truth.
- **NRLMSISE-00 upper atmosphere** — Phase 5 deferral. USSA76 7-layer
  (geopotential 0–86 km) covers every scenario shipped today.
- **EGM2008 / spherical-harmonic gravity** — Phase 5 deferral.
  WGS84-J2 (`J2Gravity`) is the SOTA-adjacent bridge for short-range
  scenarios and is now consumed directly by both kernel-side
  adapters and the FC's estimators through the consolidated
  `openbmp_physics::gravity` module.

## Solver Vetting List

Candidate permissive QP/SOCP crates to evaluate before MPC/LCvxLD/SCvx:

| Crate | Candidate use | Gate |
|---|---|---|
| `osqp` | Convex QP MPC | License, deterministic settings, allocation behavior, C/native dependency posture |
| `clarabel` | Conic MPC / powered-descent convex subproblems | License, no hidden threading/time, deterministic tolerances |
| `proxqp` | Dense/sparse QP | Rust binding maturity, license, native dependency review |
| `daqp` | Active-set QP | License, deterministic pivoting, no unsafe surprises |

No solver lands until `cargo deny check` passes and a short design note records
why the crate is acceptable under OpenBMP's no-operational-flight boundary.
