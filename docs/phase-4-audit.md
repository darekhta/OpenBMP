# Phase 4 audit + fix prompt

> **Target session.** An auditor — a senior software architect with a
> guidance-navigation-control background and a numerical-methods axe to
> grind — taking a fresh look at OpenBMP after commit
> `3474b2f` ("Phase 4.A–B: openbmp-fc autopilot framework + close-out
> hardening"). The author of that commit was conscientious but moved
> fast. Your job is to find where they settled for the textbook
> baseline when state-of-the-art was within reach, where they
> duplicated authority that already exists elsewhere in the workspace,
> and where the test surface lies about coverage. Then fix it.
>
> This prompt is self-contained. Use it verbatim. Don't ask for
> clarification on the project — the answers are in the repo.

## Mandatory reading before you start

1. `docs/design-concept.md` — workspace architecture, validation
   tiers, safety boundaries.
2. `docs/phase-4-plan.md` — Phase 4 plan with the "Phase 4.A + 4.B
   landed (2026-04-30)" closure section appended.
3. `docs/phase-4b-plan.md` — the close-out punchlist (categories
   A–G, 11-step order of execution, definition of done).
4. `docs/software-architecture.md § Flight Controller` — the
   architectural contract (HAL-portable seam, lockstep clock, no
   `openbmp-sim` dependency).
5. `docs/data-provenance.md` — how every numeric value is supposed
   to be sourced.
6. `crates/openbmp-fc/README.md` — module map of `openbmp-fc` and
   the Phase 4.C deferral list.
7. The commit itself: `git show 3474b2f` (45 files, +9 350 lines).

Do not trust the prior author's "completed" status. Verify against
the code.

## Mission

Five audit axes. For each, **find** the gap, **rate** it
(`high`/`medium`/`low` severity), and **fix** the high-severity ones
inside this same session. Out-of-scope work goes to a Phase 4.C list.

The bar for "state of the art" is explicit: this codebase is meant to
be a research-grade reference. A pedestrian implementation — vanilla
EKF without Joseph form, classical PID without anti-windup nuance,
hardcoded chi-square gates, single-instance estimator — is the
**baseline**, not the endpoint. Where Phase 4.A+B settled for the
baseline when SOTA was within reach without exotic dependencies, that
is a finding.

## Five audit axes

### Axis 1 — gap analysis (what the plan promised vs what the code delivers)

Read `docs/phase-4-plan.md` and `docs/phase-4b-plan.md` line by line.
For every concrete deliverable claimed there, prove the code does
what was claimed. Specific things to verify (this is not exhaustive):

- The plan claims "innovation chi-square ratio" and "dead-reckoning
  flag" are surfaced on `EstimatorStatus`. Are they reset / updated
  every tick or only on update? Check
  `crates/openbmp-fc/src/estimator.rs:status()`.
- The plan claims "saturation reporting on the actuator topic". The
  `ActuatorCommand.saturated` boolean is set only on rate-loop
  saturation. Is the trajectory loop's saturation counted? The
  attitude loop's? Check `crates/openbmp-fc/src/autopilot.rs`.
- The plan claims sensor-staleness, estimator dead-reckoning, and
  scheduler overruns are aggregated into `failsafe_flags`. Verify
  staleness is actually triggering against bus-sequence advances
  (Phase 4.B C6) and not against sample timestamps. Check
  `crates/openbmp-fc/src/health.rs`.
- The plan claims "voter integration" surfaces a per-sensor
  `sensor_status` divergence flag. The Phase 4.B implementation
  collapses divergence into `healthy = false` on the voted topic.
  Is that the seam the plan asked for? Check
  `crates/openbmp-fc/src/sensor_ingest.rs:VotedImuIngest::run`.
- The plan claims "the runner constructs jobs around
  `SyntheticSensorAdapter` instances" and the scenario `[fc]` block
  drives "real" runs. Phase 4.B ships an `FcRunner` that's never
  called by the kernel-side `phase2_*.rs` runners. The closed-loop
  scenario fixture parses but does not actually drive a Parquet
  output. This is the largest single gap; quantify it.
- The plan claims `cargo deny check` enforcement on a vetted QP
  solver. The vetting list is empty. List the candidate crates the
  next phase should evaluate (e.g. `osqp`, `clarabel`, `proxqp`,
  `daqp`).
- The plan claims `[forces]` interaction with the FC. The
  `[fc]` block is structurally independent of `[forces]`. Verify the
  kernel's force-evaluation path is unaffected and document why the
  decoupling is correct (or fix it if it's not).

Deliverable: a table of (claim, where it appears, where the code
lives, status, severity).

### Axis 2 — thing-in-itself risk: integration with the workspace

`openbmp-fc` is HAL-portable and must not import `openbmp-sim`,
`openbmp-cli`, `openbmp-scenario`, `openbmp-telemetry`,
`openbmp-bridge`, `openbmp-aerothermal`, or `openbmp-env`. Good. But
HAL portability is not the same as standalone-ness. Audit:

- **Sensor sample types.** `openbmp-sensors::SensorMeasurement::Imu
  { gyro_rad_s, accel_m_s2 }` and
  `openbmp-fc::topics::ImuSample { time, gyro_rad_s, accel_m_s2,
  healthy }` are parallel definitions of the same physical sample.
  The `ImuIngest` job adapter translates between them. Is the
  adapter the right seam, or should the FC's bus topic be a thin
  wrapper around `SensorMeasurement` so there's a single source of
  truth?
- **Mission types.** The commander correctly uses
  `openbmp-mission::{MissionPhaseGraph, EventBinding, PhaseId,
  EventId, BuiltInEventTrigger}`. Confirm there are no duplicates.
- **State types.** `openbmp-fc` does **not** depend on
  `openbmp-state`. That's the right call for HAL portability — the
  FC operates in its own state-vector space. But the kernel↔FC
  bridge needs to translate `openbmp-state::PointMassState` /
  `RigidBodyState` to FC sensor samples. Where will that translation
  live? `openbmp-cli/src/runner/fc.rs` is the right home, but the
  Phase 4.B `FcRunner` does not yet do this. Spec the translation.
- **Effector / engine types.** `openbmp-core` defines
  `EffectorId` / `EngineId`. The FC's `ActuatorCommand` uses
  hard-coded channel names (`aileron_rad`, `elevator_rad`,
  `rudder_rad`, `body_flap_rad`) that don't map to a vehicle's
  declared `[[vehicle.assembly.effectors]]` ids. There's an
  identity-mapping gap between FC actuator channels and the kernel's
  effector rack. Find it; fix it.
- **Magnetic / gravity / atmosphere.** `openbmp-env` already ships
  `gravity = "constant"` (`g_m_s2 = 9.80665`) and `UsStandard1976`.
  `openbmp-fc::estimator` re-implements US Standard Atmosphere 1976
  inline (`pressure_altitude_us_std_1976_m`, ~25 lines of constants
  duplicating `openbmp-env::UsStandard1976`). `openbmp-fc::magnetic`
  defines `R_EARTH_M = 6_371_000.0` from scratch. This is a
  high-severity authority duplication — the workspace has two
  parallel sets of physics constants and atmosphere math.
  Resolution options:
  1. Factor the constants into a new HAL-portable crate
     `openbmp-physics` (atmosphere, gravity, geomagnetic, Earth
     constants) that both `openbmp-env` and `openbmp-fc` depend on.
  2. Extend `openbmp-core` with a `physics_constants` module.
  3. Move the constants up into `openbmp-models` (already
     HAL-portable).
  Pick one. Implement it. Delete the duplicates.
- **Synthetic sensor seam.** The closed-loop integration test in
  `crates/openbmp-fc/tests/closed_loop.rs` publishes `ImuSample`
  directly to the bus rather than going through
  `SyntheticSensorAdapter` → `ImuIngest`. The seam exists but isn't
  exercised end-to-end. Add a test that uses the full ingest path.

### Axis 3 — constants and authority

For every numeric constant in `crates/openbmp-fc/src/`, ask: "is
some other workspace crate already the authority for this value?"
If yes, the FC code is wrong to hard-code it. Specific cases known
to violate this rule:

| Constant | Value | Where it appears | Authority that should own it |
|---|---|---|---|
| Standard gravity | `9.80665` | `estimator.rs::pressure_altitude_us_std_1976_m` | `openbmp-env::UsStandard1976` (or `openbmp-core` via the proposed `openbmp-physics` crate) |
| ECI-z gravity | `-9.81` | `estimator.rs::ConstantGravityZ::default` | same |
| Mean Earth radius | `6_371_000.0` | `magnetic.rs::R_EARTH_M` | shared physics crate |
| Sea-level pressure | `101_325.0` | `estimator.rs::P0` | `openbmp-env::UsStandard1976` |
| Air molar mass | `0.028_964_4` | `estimator.rs::M_AIR` | `openbmp-env::UsStandard1976` |
| Ideal gas constant | `8.314_462_618` | `estimator.rs::R` | physics crate |
| Tropopause | `22_632.06` Pa, `11_000` m | `estimator.rs::TROPOPAUSE_*` | `openbmp-env::UsStandard1976` |
| Equatorial mag field | `30_000.0` nT | `magnetic.rs::EarthDipoleField::default` | physics crate |
| EKF chi-square gate | `9.0` | `estimator.rs::EkfParams::default` | should be derived from a stated false-alarm rate (e.g. `chi2_inv(0.99, dof=6)` for GNSS), not a magic number |
| Frame budget | `2_000` µs | `controller.rs::FlightControllerBuilder::default` | scenario-driven via `[fc]`, not hard-coded |
| Health staleness defaults | `0.05`, `0.5`, `0.2`, `0.2`, `5` | `health.rs::HealthParams::default` | scenario-driven |
| Scheduler periods (1, 10) | hard-coded in `FcRunner` | `crates/openbmp-cli/src/runner/fc.rs` | should derive from `config.base_rate_hz` (already parsed but unused — `let _ = kernel_tick_per_fc_tick`) |
| Default gains | `0.5/0.05`, `2.0/2.0/1.0` | `autopilot.rs::default_gains` | scenario-driven via the gain schedule |

For each row: replace with a workspace authority or document why
the constant is intrinsically fc-local (e.g. process-noise sigmas
are filter-tuning, not physics).

### Axis 4 — test coverage

Phase 4.B claims property tests on the EKF and a closed-loop
integration test. Verify, then expand. Concrete gaps:

- **MEKF property tests.** Only the EKF has proptest coverage.
  Mirror it for MEKF: determinism across reruns, attitude-error
  whitening on long runs.
- **NEES filter-consistency test.** The plan called for "Normalized
  Estimation Error Squared over a Monte-Carlo ensemble lies within
  the 95 % chi-square bound." Phase 4.B substituted a single-run
  innovation whitening test. The Monte-Carlo NEES is the standard
  filter-consistency check (Bar-Shalom-Li-Kirubarajan §5.4); ship
  it.
- **Innovation autocorrelation lags 2…10.** Phase 4.B tests lag-1
  only. Welch's test on lags 2..10 catches more colored-noise
  pathologies.
- **Bus property tests.** Sequence monotonicity; single-writer-
  multi-reader correctness; topic registration is order-invariant
  for read but order-defining for iteration.
- **Scheduler stress tests.** Mix of periodic + topic-driven jobs
  with adversarial budget patterns; assert the overrun event fires
  exactly when budget was exceeded.
- **Voter adversarial tests.** Triplex with one stuck sensor; with
  one drifting sensor; with two divergent sensors; what does the
  median-select do under each? Document the expected behavior, then
  test it.
- **FDIR fault-injection tests.** Inject an IMU bias of 1 σ, 3 σ,
  10 σ; assert the burst counter trips at the right time. Today
  there are no direct FDIR tests beyond "runs without panicking."
- **GNSS-dropout dead-reckoning recovery.** Drop GNSS for 2 seconds
  and assert: (a) `EstimatorStatus::dead_reckoning` flips true at
  the right time, (b) `FailsafeFlags::estimator_dead_reckoning`
  flips true via the health monitor, (c) the commander refuses to
  arm during dead-reckoning, (d) recovery clears all three when
  GNSS returns.
- **Multi-phase mission test.** The closed_loop test exercises one
  phase transition (pad → ascent). Add a 3-phase or 4-phase test
  with a recovery / safe-state phase, exercising the FDIR
  `safe_state_requested` latch in `VehicleStatus`.
- **Determinism stress.** Today the closed_loop test asserts two
  runs of the actuator stream are identical. Strengthen: assert
  the **full bus history** (every published topic, every sequence
  number) is identical across reruns.
- **HAL-portable build.** `cargo build -p openbmp-fc
  --no-default-features` is in CI but `cargo test -p openbmp-fc
  --no-default-features` requires that no `#[cfg(feature =
  "sim-default")]` slips into a non-feature-gated test. Verify.
- **Long-duration stability.** A 60-second simulated run that
  asserts no NaN, no covariance blow-up, no integrator drift past
  declared envelope.

### Axis 5 — state of the art

The user's explicit instruction: "our autopilot should be state of
the art." Every claim below is a concrete SOTA upgrade that's
within reach without adding a vetted-solver dependency.

#### Estimator

- **Joseph form for the covariance update.** Phase 4.B ships
  `P = (I - K·H) · P`. This is numerically inferior to the Joseph
  symmetric form
  `P = (I - K·H) · P · (I - K·H)^T + K · R · K^T`
  which guarantees positive-definiteness even with off-by-rounding
  K. SOTA EKF implementations have used Joseph form since Bierman
  1977. Switch.
- **Quaternion renormalization.** After every
  `q_body_to_eci *= dq` and after every attitude-error reset,
  renormalize. nalgebra's `UnitQuaternion *= UnitQuaternion`
  technically returns a unit quaternion in exact arithmetic, but in
  finite precision the magnitude drifts. SOTA: explicit
  `UnitQuaternion::renormalize()` after every update.
- **Markley-form MEKF reset.** Markley 2003 ("Attitude error
  representations for Kalman filtering") prescribes a reset that
  preserves second-order accuracy. Phase 4.B uses
  `q_body_to_eci *= quaternion_from_axis_angle(δθ)` which is
  first-order. Upgrade.
- **Bias dynamics as a Gauss-Markov process.** Phase 4.B models
  gyro / accel biases as random walks (driven by white noise on the
  derivative). SOTA: 1st-order Gauss-Markov with a configurable
  time constant; the random-walk model is the τ → ∞ limit and
  diverges over long runs.
- **Square-root EKF / UD factorization.** The covariance matrix
  is propagated as full P. Square-root form (Cholesky) halves the
  conditioning number and is the SOTA reference for safety-critical
  filters. Bierman 1977.
- **Iterated EKF on nonlinear measurements.** Magnetic field
  prediction is nonlinear in attitude. A single-pass linearization
  is suboptimal; iterated relinearization around the posterior
  mean is the SOTA fix. ~20 lines.
- **Innovation gates derived from a stated false-alarm rate.** The
  hard-coded `innovation_gate = 9.0` is a chi-square-3 99.9 %
  threshold, but it's applied to the GNSS 6-D innovation
  (chi-square-6 99 % is 16.81). The gate should be `chi2_inv(1 -
  fa_rate, dof)` per measurement. Fix.

#### Autopilot

- **Notch filtering on gyro feedback.** Real airframes have
  body-mode resonances. PX4 / ArduPilot ship configurable notch
  filters on the rate-loop input. Without them, an autopilot can
  excite a structural mode. Add a `Biquad` filter struct to the
  rate loop with scenario-configurable coefficients.
- **Anti-windup observer form.** Phase 4.B uses back-calculation,
  which is fine. SOTA for safety-critical: observer-based anti-
  windup (Hippe 2006). Probably out of scope; document the
  decision.
- **Trajectory loop is a placeholder.** Phase 4.B's trajectory
  loop converts position error to body-axis attitude error via a
  per-axis PID. SOTA for guided rocketry: differential flatness
  (Mellinger & Kumar 2011) or model-predictive trajectory
  tracking. The latter is gated on Phase 4.C QP solver; the former
  is implementable today and is markedly better than per-axis PID.
- **L1 adaptive augmentation.** Cao & Hovakimyan 2010 L1 adaptive
  control is the standard "robustness booster" for hand-tuned
  PID baselines. Add as an optional augmentation enabled via the
  scenario `[fc.autopilot_params]`.
- **Saturation reporting on every loop.** Phase 4.B reports
  saturation only on the rate loop. Surface trajectory and attitude
  saturation too, so the commander can react.

#### Magnetic / gravity / atmosphere

- **WMM 2025** is Phase 4.C. Document the deferral. The
  `EarthDipoleField` is degree-1 only and is **not** SOTA — it's a
  placeholder. State this in the module docstring.
- **WGS84-J2 gravity** (already in `openbmp-env`) is SOTA-adjacent
  for short-range; full spherical-harmonic EGM2008 is the SOTA
  reference. The FC should accept either; today it ships
  `ConstantGravityZ` only.
- **NRLMSISE-00 atmosphere** above 11 km. USSA76 troposphere is
  fine for sounding-rocket apogee 3 km but breaks at the tropopause.
  Phase 4.B clamps to the tropopause altitude. Document; if Phase
  5 ramps up to LEO scenarios, NRLMSISE-00 lands.

#### Voter

- **In-flight covariance-based weighting.** Phase 4.B's
  `WeightedMeanScalar` uses a static `primary_weight`. SOTA: weight
  inversely proportional to the per-sensor innovation covariance,
  i.e. the Bayesian fusion of independent measurements. Configurable
  fall-back to static when innovation is unavailable.
- **Per-sensor `sensor_status` topic.** The plan asked for a
  per-lane divergence-flag stream. Phase 4.B published only the
  voted topic with a collapsed `healthy` boolean. Ship the per-lane
  topic.

#### FDIR

- **GLRT / CUSUM on innovation residuals.** Phase 4.B ships a
  burst-counter detector. SOTA for filter-based fault detection
  is the Generalized Likelihood Ratio Test (Willsky 1976) or
  CUSUM. Both fit in ~50 lines of code; both are statistically
  principled.
- **Fault isolation.** Phase 4.B reports `triggered: bool`. SOTA
  reports a fault tree (which sensor / which axis / which
  phase). Today the `tripped_mask` field is published but never
  populated.

#### Real-time discipline

- **Hot-path allocations.** `VotedImuIngest::run` does
  `Vec::with_capacity(self.sensors.len())` every tick. SOTA RT
  guarantees no allocation past startup. Switch to fixed-size
  arrays (`SmallVec` or stack arrays parameterized by const
  generic).
- **Locked operand order.** Phase 4.A claims locked operand order
  for determinism. Verify by inspecting every floating-point matrix
  multiplication: do they all use the same parenthesization? Use a
  determinism stress test (1 000 reruns, byte-equal state) to
  confirm.

#### Numerical

- **Kahan summation** on the integrator's accumulators. Probably
  unnecessary at 1 kHz over 1 minute; document the call.
- **Stable trigonometric reduction** in
  `quaternion_from_axis_angle`: switch to `sin(θ/2)/θ` Taylor
  expansion near zero to avoid 0/0 hazard. Phase 4.B has a
  `mag < 1e-12` cutoff; verify the cutoff matches `f64::EPSILON`
  reasoning.

## Specific code locations to investigate

These are the high-yield places to start:

- `crates/openbmp-fc/src/estimator.rs:330-510` — EKF / MEKF
  mathematics. Joseph form, quaternion renormalization, mag
  Jacobian.
- `crates/openbmp-fc/src/estimator.rs:580-635` — `pressure_altitude_us_std_1976_m`
  inline. Authority-duplication smoking gun.
- `crates/openbmp-fc/src/magnetic.rs` — `R_EARTH_M` and dipole
  truncation; verify that the singularity-tame at `r < R_e/2` is
  consistent with the simulator's initial-condition transient.
- `crates/openbmp-fc/src/autopilot.rs::run` — note the
  `#[allow(clippy::too_many_lines)]`; the function is doing five
  things and likely deserves decomposition.
- `crates/openbmp-fc/src/sensor_ingest.rs::vote_vec3` —
  hot-path allocation.
- `crates/openbmp-cli/src/runner/fc.rs:79-80` — the dead
  computation `let kernel_tick_per_fc_tick = ... ; let _ =
  kernel_tick_per_fc_tick;`. Either use it or delete it.
- `crates/openbmp-cli/src/runner/fc.rs:175-192` — hardcoded
  scheduler periods (1, 10) when the parsed `base_rate_hz` is
  ignored.
- `crates/openbmp-fc/src/health.rs:54-80` — the `TopicStaleness`
  abstraction; verify the `last_seq == 0` first-publish edge case
  handles the case where a publisher publishes once and then stops
  (last_seq advances to 1, time stalls).
- `crates/openbmp-fc/src/replay.rs:107-125` — `replay_until` uses
  `&&` chained `if-let`; verify the count is correct when the
  replay stream and the time threshold cross mid-stream.
- `crates/openbmp-fc/src/scheduler.rs:280-355` — `dispatch`
  function refactored away from `expect`; verify the new
  `let-else` flow doesn't change semantics under any path.
- `crates/openbmp-fc/src/commander.rs:194-204` — the
  `evaluate_fdir_safe_state` flag latches but no scenario binding
  shape consumes it; this is the seam for Phase 4.C.
- `crates/openbmp-cli/src/runner/phase2_*.rs` — verify these
  do **not** integrate with `FcRunner`. The kernel↔FC bridge is
  the largest architectural gap; the audit must spec it.

## Deliverables

1. **`docs/phase-4-audit-findings.md`** — a structured findings
   document organized by axis, every finding tagged with severity,
   linked to a file:line, and assigned to either "fixed in this
   audit" or "Phase 4.C work item."

2. **Code changes that fix every `high`-severity finding.** Stage
   them in dependency order (constants/authority → math → tests).
   Run `cargo fmt && cargo clippy --workspace --all-targets
   --all-features -- -D warnings && cargo test --workspace
   --all-features && cargo build -p openbmp-fc
   --no-default-features` after every meaningful change.

3. **`docs/phase-4c-plan.md`** — the next-phase punchlist with
   every `medium`/`low` finding scoped, ordered, and gated. Include
   the QP-solver vetting list, the kernel↔FC bridge spec, and the
   SOTA upgrades that exceed Phase 4.B's authorization (sigma-
   point UKF, real WMM, MPC).

4. **Update `docs/phase-4-plan.md`'s "landed" section** to reflect
   the post-audit reality. The current claim "every algorithm-
   fidelity gap that the plan listed is closed" is over-stated —
   the audit will surface concrete counter-examples.

5. **Update `crates/openbmp-fc/README.md`** module map and
   deferral list to match.

## Authorization & scope

Authorized:
- Read / edit any file in the workspace.
- Add unit / property / integration tests freely.
- Refactor crates (e.g. extract `openbmp-physics`).
- Run `cargo fmt`, `cargo clippy --fix`, `cargo test`, `cargo deny
  check`, `cargo machete`.
- Create commits in dependency order; the user will push.

Requires user approval first:
- Adding a new workspace dependency from crates.io. Justify against
  the `cargo deny` policy and the no-system-time / no-allocation /
  permissive-licence rules.
- Bumping MSRV.
- Touching `Cargo.lock` outside the natural consequence of
  workspace edits.

Out of scope (explicit Phase 4.C deferrals — do not implement):
- Real sigma-point UKF.
- Real receding-horizon MPC backed by a vetted convex-QP solver.
- Real LCvxLD / SCvx powered descent.
- Full WMM 2025 spherical-harmonic field with the COF dataset.
- NRLMSISE-00 upper atmosphere.
- The kernel↔FC bridge in `phase2_*.rs` (large enough to be its own
  phase; spec it in `phase-4c-plan.md` instead).
- ULog binary log format support in `replay.rs`.
- Multi-instance estimator (PX4 ekf2 lane voting).

Destructive operations are forbidden without explicit user
authorization: no `git push --force`, no `rm -rf`, no branch
deletion, no `--no-verify` commits, no `git reset --hard`.

## Definition of done

The audit is closed when **all** of:

1. `docs/phase-4-audit-findings.md` enumerates every finding with
   severity and resolution.
2. Every `high`-severity finding is fixed in the working tree.
3. `cargo fmt --all -- --check` clean.
4. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
5. `cargo test --workspace --all-features` green.
6. `cargo build -p openbmp-fc --no-default-features` clean.
7. `cargo deny check` clean.
8. `cargo machete` clean.
9. The lockstep-clock tripwire test still passes.
10. `docs/phase-4c-plan.md` exists with the deferred work scoped
    and prioritized.
11. `docs/phase-4-plan.md`'s "landed" section honestly reflects the
    post-audit state.
12. The Phase-1, Phase-2, Phase-3 byte-stable scenarios still pass
    (regression).

## A note on tone

The prior author's commit message implies confidence — "every
algorithm-fidelity gap closed", "1 043 tests passing", "all gates
clean". The audit must not respect those claims as load-bearing.
The whole point of an audit is to find what the prior author missed
or papered over. Be skeptical. Be thorough. Don't let "the tests
pass" stand in for "the code is right."

The audit will be reviewed against the same SOTA bar it applies to
the code. If the audit recommends "use Joseph form" without
implementing it, that's not an audit, it's a wishlist. Implement.
