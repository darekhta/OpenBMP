# Phase 4.B — Close-out plan

> **Status:** This is the close-out punchlist for Phase 4. Phase 4.A
> shipped the architectural skeleton and a working closed-loop test;
> Phase 4.B finishes the work that 4.A stubbed, deletes the
> placeholders that misrepresent themselves, wires pre-existing types
> into the runtime, adds the quality gates the original plan required,
> and integrates the controller with the simulator runner so the FC
> is actually drivable from a scenario file.
>
> This plan is self-contained: every item names file paths, the
> reason, and an order of execution. It survives session compaction.

## Phase 4.A status (what's actually delivered)

Compiles and tests:
- `cargo build -p openbmp-fc --all-features` — clean
- `cargo build -p openbmp-fc --no-default-features` — clean (HAL-portability)
- `cargo test -p openbmp-fc` — 32 unit + 2 integration tests, all green
- Closed-loop integration test: full pipeline runs 1 000 ticks deterministically

Working subsystems:
- `bus.rs` — typed pub/sub, sequence counters
- `scheduler.rs` — cyclic dispatcher, declared-budget overrun events
- `clock.rs` — `Clock` trait + `SimulatedClock` + `FixedClock`
- `params.rs` — typed parameter sections
- `tables.rs` — validated-then-activated tables
- `dictionary.rs` — text + (feature-gated) JSON dictionary dump
- `commander.rs` — phase transitions, arming, liftoff detection
- `health.rs` — failsafe-flag aggregation
- `fdir.rs` — burst-counter detection (publishes status only)
- `guidance.rs` — `AttitudeHoldGuidance`, `WaypointGuidance`
- `voter.rs` — `PassThroughVoter`, `MidValueSelectScalar`, `WeightedMeanScalar`
- `sensor_ingest.rs` — per-sensor `Sensor::read` ingest jobs
- `replay.rs` — in-memory `BusRecorder` / `BusReplayer`
- `topics.rs` — canonical bus topics
- `controller.rs` — `FlightController` façade

EKF: GNSS update is real (covariance, innovation gate, chi-square).
Three-loop autopilot: real PID with anti-windup back-calculation.

## Phase 4.B work items

Items are grouped by category, ordered by dependency within each
category. The "Order of execution" section at the end gives the
single linear order for the implementation phase.

### A. Delete misrepresenting stubs

The Phase 4.A code shipped placeholder modules with names that
overstate what they do. Delete them. Real implementations belong in
a future Phase 4.C gated on a vetted SOCP / QP solver passing
`cargo deny` review.

- **A1.** Delete `crates/openbmp-fc/src/mpc.rs` and remove `pub mod
  mpc;` from `lib.rs`. *Why:* `LqrAttitudeMpc` is a constant-gain
  LQR-shaped feedback. There is no horizon, no QP solve, no receding
  window. The name overclaims; the trait surface is shaped around
  the stub and is not load-bearing for a future real MPC.
- **A2.** Delete `crates/openbmp-fc/src/landing.rs` and remove `pub
  mod landing;` from `lib.rs`. *Why:* `Lcvxld` is a single-step PD
  controller, not lossless-convexification soft-landing. `Scvx` just
  iterates the inner PD solve N times, not actually successive
  convexification. Both are placeholders wearing the algorithm names.
- **A3.** Delete `Ukf`, `UkfParams`, and the `ukf_*` tests from
  `crates/openbmp-fc/src/estimator.rs`. *Why:* `Ukf::predict`
  propagates only the mean and inflates the diagonal — that is an
  EKF, not a UKF. A real sigma-point UKF is ~150 lines + tests and
  is its own scope. Document the deferral in the estimator module
  docstring.
- **A4.** Update `crates/openbmp-fc/README.md` and the Phase-4 plan
  doc to reflect the deletions; mark MPC, LCvxLD/SCvx, UKF as
  Phase 4.C deferrals. *Why:* doc hygiene.

### B. Wire pre-existing types into the runtime

Several types were defined in Phase 4.A but never consulted by the
runtime that publishes through them. Wire them up.

- **B1.** Wire `autopilot::GainSchedule` into `ThreeLoopAutopilot`.
  *Where:* `crates/openbmp-fc/src/autopilot.rs`. Currently
  `find_gains_for_phase` returns hard-coded constants. Replace with:
  `ThreeLoopAutopilot::new(schedule: GainSchedule)` constructor that
  stores the schedule, then on each tick reads
  `vehicle_status.phase_id` and looks up the gains. Test: a 2-phase
  schedule with distinct gains shows different actuator commands as
  the commander transitions phases.
- **B2.** Wire `mixer::PhaseAuthorityTable` into `Mixer`. *Where:*
  `crates/openbmp-fc/src/mixer.rs`. Currently `Mixer::run` checks
  `armed && in_flight`. Replace with a per-phase mask check:
  republish only if the active phase's `PhaseAuthority` permits the
  channel and `autopilot_allowed` / `engines_allowed` is true. Pass
  the table in via constructor or read from a `Tables` registry.
  Test: a phase with `autopilot_allowed = false` zero-mixes the
  command even when armed and in flight.
- **B3.** Wire voter into the sensor ingest path. *Where:*
  `crates/openbmp-fc/src/sensor_ingest.rs`. Add `VotedImuIngest<S, V>`
  / `VotedGnssIngest<S, V>` / etc. that own `Vec<S: Sensor>` plus a
  `V: Voter`. On each tick, `read()` every source, hand the array to
  `voter.vote()`, publish the voted result + a per-sensor
  `sensor_status` divergence flag. Existing single-source
  `ImuIngest` etc. stay for the simplex case. Test: triplex IMU
  with one diverging source flags divergence and votes the median.
- **B4.** Wire `FdirStatus` into the commander's failsafe path.
  *Where:* `crates/openbmp-fc/src/commander.rs`. Currently the
  commander reads `FailsafeFlags` only. Add an `FdirStatus` read in
  `try_arm`: if `fdir.triggered`, refuse to arm. If already armed
  and `fdir.triggered`, mark a `safe_state_requested` flag in
  `VehicleStatus` so a future scenario can declare a safe-state
  phase to transition into. Test: an injected FDIR trip blocks
  arming.

### C. Implement missing measurement / process math

Phase 4.A shipped EKF with GNSS update only. Add the rest.

- **C1.** Add a `GravityModel` abstraction to the EKF process model.
  *Where:* `crates/openbmp-fc/src/estimator.rs`. The existing
  `Ekf::predict` integrates accel as if no gravity exists (comment:
  "minimal reference impl assumes gravity-as-zero"). Add a small
  trait `GravityModel { fn at(&self, position_eci: Vector3<f64>) ->
  Vector3<f64> }` and a reference impl `ConstantGravityZ` (default
  −9.81 m/s² in ECI z). Pass it into `Ekf::new(...)` as a
  constructor parameter. *Why:* without this the EKF produces
  meaningless integrals over any non-trivial flight. Test: a
  free-fall scenario reproduces analytic position-vs-time within
  1 cm over 10 s.
- **C2.** Implement `Ekf::update_baro` for real. *Where:* same file,
  line ~330. Currently `Ok(())`. Convert measured pressure to
  altitude via the US Standard Atmosphere 1976 (already in
  `openbmp-env::UsStandard1976` — but that's an `openbmp-env`
  dependency which is forbidden in `openbmp-fc`). *Resolution:*
  re-implement the inverse-isothermal pressure-altitude formula
  inline (it's ~20 lines: T0 = 288.15, lapse = -0.0065, etc.) so
  `openbmp-fc` doesn't pull in `openbmp-env`. Innovation: predicted
  altitude (z-component of position estimate) − measured altitude.
  Real chi-square gate. Test: barometric drift below the gate is
  absorbed; an out-of-gate measurement is rejected and chi^2 is
  recorded.
- **C3.** Add a `MagneticFieldModel` trait and an academic-tier
  `EarthDipoleField` reference impl. *Where:* new file
  `crates/openbmp-fc/src/magnetic.rs` plus wire-up in
  `estimator.rs`. *Why:* full WMM is a 12-degree spherical harmonic
  with an external dataset (`data/magnetic/WMM.COF`); pulling that
  into `openbmp-fc` is heavyweight. The earth-dipole truncation is
  one term of the WMM, ~30 lines, gives degree-1 accuracy
  acceptable for academic attitude-tracking scenarios. Mark
  `validated-toy`. Note in module docs that full WMM is Phase 4.C.
  Test: dipole field at the equator is ~horizontal, at the pole
  ~vertical.
- **C4.** Implement `Ekf::update_mag` and `Mekf::update_mag` for
  real. *Where:* same file, lines ~340 and ~590. Predicted body-
  frame field = `q.inverse() * gravity_model.field_at(position)`.
  Innovation = measured − predicted (after subtracting hard-iron
  bias). Real chi-square gate. Test: a noisy mag stream + GNSS
  improves attitude estimate vs IMU-only baseline.
- **C5.** Implement the trajectory loop in `ThreeLoopAutopilot`.
  *Where:* `crates/openbmp-fc/src/autopilot.rs`. Currently the
  `trajectory_loop_enabled` param is declared but unused; only the
  rate and attitude loops actually run. Add: when
  `trajectory_loop_enabled` is true and the reference contains a
  position (non-zero `position_eci_m`), compute a position-error PID
  to produce a body-frame thrust direction, fold it into the
  attitude reference. Test: a position-tracking reference holds the
  vehicle near the target.
- **C6.** Fix `HealthMonitor` staleness tracking. *Where:*
  `crates/openbmp-fc/src/health.rs`. Currently it tracks
  `last_*_time_s` from the *sample's embedded timestamp*, not from
  the bus publish time, so a misbehaving publisher with stale
  sample timestamps confuses the staleness check. Replace with
  bus-sequence-change detection: store the last seen sequence per
  topic; if the sequence has not advanced for >timeout ticks, mark
  the source unhealthy. Test: a stalled publisher trips
  `imu_unhealthy` after the configured timeout.

### D. Quality gates (the items the original plan required)

- **D1.** `cargo fmt --all -- --check` clean. Run `cargo fmt --all`
  and commit.
- **D2.** `cargo clippy --workspace --all-targets --all-features -- -D
  warnings` clean. Workspace lints set `unwrap_used = warn`,
  `expect_used = warn`, `panic = warn`, `pedantic = warn`,
  `float_cmp = deny`. Replace `.unwrap()` and `.expect()` in non-test
  code with proper error propagation. The Phase 4.A scheduler in
  particular has multiple `.expect("index in range by construction")`
  calls — refactor to remove them by using `let Some(...) = ... else {
  continue }` patterns.
- **D3.** `cargo deny check` passes. New deps added in 4.A:
  `indexmap`, `serde_json`, `toml`. Verify all licenses are
  permissive and pinned versions have no advisories.
- **D4.** `cargo machete` passes — no unused workspace deps. The
  estimator's `Matrix3` import is currently silenced by an
  `#[allow(dead_code)] _unused()` function; remove the trick after
  C3/C4 use `Matrix3` for real.

### E. Test depth

- **E1.** Property tests on EKF using `proptest`. *Where:* new file
  `crates/openbmp-fc/tests/ekf_properties.rs`. Required properties:
  - **Determinism:** same seed + same input stream → byte-identical
    state trajectory across 100 random scenarios.
  - **NEES consistency:** Normalized Estimation Error Squared over a
    Monte-Carlo ensemble lies within the 95 % chi-square bound for
    the 15-state filter (NIS analogue for measurements). This is the
    standard filter-consistency check.
  - **Innovation whitening:** the innovation sequence on a long run
    has near-zero mean and uncorrelated samples (lag-1 autocorrelation
    < 0.1 by Welch's test).
- **E2.** Lockstep clock tripwire. *Where:* new file
  `crates/openbmp-testkit/src/fc_lints.rs` plus a unit test that runs
  in CI. The tripwire greps `crates/openbmp-fc/src/` (and
  `tests/`) for `Instant::now`, `SystemTime::now`,
  `std::time::Instant`, `std::time::SystemTime` — fails CI if any
  match. *Why:* the lockstep clock contract is structural; without
  enforcement it rots silently.
- **E3.** CI workflow updates. *Where:* `.github/workflows/ci.yml`.
  Add steps:
  - `cargo build -p openbmp-fc --no-default-features` (HAL-portability)
  - `cargo test -p openbmp-fc --no-default-features` (HAL-portability)
  - The new lockstep-clock tripwire test (E2)
  - The scheduler-overrun test (already exists as `scheduler::tests::
    budget_overrun_emits_event_and_skips_job` — just verify
    `cargo nextest` runs it)

### F. Runner integration

The Phase 4.A FC is a library with a self-contained integration
test. It is not wired to the simulator's runner. F closes that gap
so a scenario file can declare an FC and the simulator drives it
in lockstep.

- **F1.** Add an `[fc]` block to the scenario format. *Where:*
  `crates/openbmp-scenario/src/lib.rs` and
  `docs/scenario-format.md`. Schema:
  ```toml
  [fc]
  estimator = "ekf"            # | "mekf"
  autopilot = "three_loop"
  guidance = "attitude_hold"   # | "waypoint"
  reference_q_xyzw = [0.0, 0.0, 0.0, 1.0]   # for attitude_hold
  base_rate_hz = 1000

  [fc.ekf]
  sigma_w_gyro = 0.01
  # ...

  [fc.gain_schedule.<phase_path>]
  rate_kp = [0.5, 0.5, 0.5]
  # ...
  ```
  Parser-side: `FcConfig` struct with `serde(deny_unknown_fields)`,
  validates against the registry of known estimator / autopilot /
  guidance kinds.
- **F2.** Add an `FcRunner` in `crates/openbmp-cli`. *Where:* new
  file `crates/openbmp-cli/src/fc_runner.rs`. Responsibilities:
  - Build a `FlightController` from `FcConfig`.
  - Register all canonical bus topics.
  - For each kernel-side `SyntheticSensor`, wrap with a
    `SyntheticSensorAdapter` and pass into the matching ingest job.
  - Drive `fc.step(time, tick)` on every kernel tick (after kernel
    integrator step, before kernel writes effector commands).
  - Read `actuator_cmd` / `engine_cmd` from the FC bus and convert
    into the `ControlEffector::step` / `EngineModel::apply_command`
    calls the kernel uses.
  - Surface `vehicle_status`, `failsafe_flags`, `estimator.attitude`,
    `actuator_cmd` as telemetry channels for the kernel's Parquet
    writer.
- **F3.** End-to-end closed-loop scenario.
  *Where:* `scenarios/closed-loop-attitude-hold/`. Files:
  `scenario.toml` (the scenario), `provenance.md` (synthetic data
  declaration), `expected.toml` (tolerance table). Demonstrates:
  synthetic IMU + barometer + GNSS + magnetometer, EKF with all
  measurement updates, attitude-hold guidance against a constant
  reference, three-loop autopilot, mixer with `PhaseAuthorityTable`,
  health monitor, FDIR detector — all driven from the CLI runner
  with deterministic Parquet output.
- **F4.** Determinism CI gate. *Where:*
  `.github/workflows/ci.yml`. Add the new scenario to the existing
  determinism job: run twice on x86_64-unknown-linux-gnu, assert
  byte-identical Parquet.

### G. Closure docs

- **G1.** `docs/phase-4-plan.md` — add a "Phase 4.A landed" section
  listing what's actually delivered after Phase 4.B. Mark MPC,
  LCvxLD/SCvx, real WMM, real UKF as Phase 4.C deferrals.
- **G2.** `docs/design-concept.md` Phase 4 description — keep
  consistent with the post-4.B reality.
- **G3.** `crates/openbmp-fc/README.md` — module list reflects the
  post-deletion state; remove references to MPC / landing / UKF.

## Out of scope (explicit deferrals)

These items are out of scope for Phase 4.B and are documented as
Phase 4.C work:

- Real UKF with sigma-point propagation (~150 lines + tests).
- Real MPC with a vetted permissive-licence convex-QP solver. Gated
  on `cargo deny` review of an acceptable solver crate or an
  in-house implementation.
- Real LCvxLD / SCvx powered-descent guidance with a vetted SOCP
  solver. Gated on the same review.
- Full WMM 2025 spherical-harmonic field (the dipole truncation in
  C3 is the academic-tier reference for Phase 4.B).
- Fuzz tests on the FC parser (no parser exists in the FC crate;
  parsing happens in `openbmp-scenario` via F1).
- Microbenchmarks (`criterion`) on the FC hot path.
- Real ULog format support in `replay.rs` (the in-memory recorder
  is sufficient for state-stable replay tests).
- Multi-instance estimator (PX4-style EKF lane voting). The
  `Voter` trait surface is shipped; instantiating two `Ekf`s
  alongside a voter is downstream-side.

## Order of execution

The dependency-respecting linear order:

1. **A1, A2, A3, A4** — deletions first; everything else builds on a
   clean base.
2. **D1, D2** — `cargo fmt`; clippy clean. Catches everything else
   I write thereafter.
3. **C1, C3** — gravity model + magnetic-field model. These are the
   abstractions C2/C4/C5 build on.
4. **C2, C4, C5, C6** — barometer, mag updates (EKF + MEKF),
   trajectory loop, health-monitor sequence-based staleness.
5. **B1, B2, B3, B4** — wire gain schedule, phase mask, voter,
   FDIR-recovery into the runtime.
6. **E1, E2** — proptest on EKF + lockstep tripwire. By this point
   the algorithms are real, so the property tests are meaningful.
7. **D3, D4** — `cargo deny`, `cargo machete`. Catch any deps the
   above introduced.
8. **F1, F2** — scenario format `[fc]` block, then runner
   integration. The largest single chunk.
9. **F3, F4** — closed-loop scenario + CI determinism gate.
10. **E3** — CI workflow updates (HAL-portability + tripwire +
    overrun test).
11. **G1, G2, G3** — closure docs.

## Definition of done

Phase 4.B is closed when **all** of the following are true:

1. Items A1–A4, B1–B4, C1–C6, D1–D4, E1–E3, F1–F4, G1–G3 are done.
2. `cargo fmt --all -- --check` clean.
3. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
4. `cargo test --workspace --all-features` green.
5. `cargo build -p openbmp-fc --no-default-features` clean.
6. `cargo deny check` clean.
7. `cargo machete` clean.
8. The lockstep-clock tripwire test in `openbmp-testkit` passes
   (no `Instant::now` / `SystemTime::now` references in `openbmp-fc`).
9. The new `closed-loop-attitude-hold` scenario produces
   byte-identical Parquet across two reruns.
10. `crates/openbmp-fc/src/` contains no `mpc.rs`, no `landing.rs`,
    no `Ukf` type.
11. The `crates/openbmp-fc/README.md` module list matches the
    post-4.B state.
12. The Phase-1, Phase-2, Phase-3 byte-stable scenarios still pass
    (regression).
