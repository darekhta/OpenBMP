# Phase 4.C audit + implementation prompt

> **Target session.** A senior software architect with a guidance-
> navigation-control background and an opinionated stance on
> numerical methods, executing the Phase 4.C work tracked in
> `docs/phase-4c-plan.md`. The Phase 4 audit closed the high-severity
> findings; what remains are the deferred SOTA upgrades, the
> kernel↔FC bridge, and the test-truthfulness expansions.
>
> This prompt is **opinionated by design**. The architectural
> decisions below are decided, not options. The auditor's job is to
> execute them — push back only with concrete numerical or
> architectural reasoning, not aesthetic preference.
>
> "State of the art" remains the bar. A pedestrian implementation is
> a finding; a textbook reference (Bar-Shalom-Li-Kirubarajan, Markley
> 2003, Bierman 1977, Cao & Hovakimyan 2010, Mellinger & Kumar 2011)
> is the **floor**.

## Mandatory reading

1. `docs/phase-4-plan.md` § "Phase 4.A + Phase 4.B + Audit State"
2. `docs/phase-4-audit.md` — the audit prompt that produced the
   recent fixes; understand the constraints it imposed.
3. `docs/phase-4-audit-findings.md` — the audit's findings table.
   Verify the "fixed in this audit" rows are actually fixed in the
   working tree.
4. `docs/phase-4c-plan.md` — the priority-ordered punchlist (P0–P5).
   This prompt is the *implementation program* for that plan.
5. `docs/software-architecture.md § Flight Controller` — the
   architectural contract.
6. `docs/data-provenance.md` — every numeric value must have a
   citation; this is non-negotiable for new constants.
7. The current `Cargo.toml`, `Cargo.lock`, and the latest commit on
   `main`. (You may be auditing un-committed changes from the prior
   audit; treat the working tree as the baseline.)

## Architectural decisions (decided — execute, do not litigate)

### D1. Kernel↔FC bridge — separate module, optional, deterministic

The bridge lives in a **new module** `crates/openbmp-cli/src/runner/fc_bridge.rs`,
not inside `phase2_*.rs` and not inside `fc.rs`. `fc.rs` stays
pure-FC-construction; `fc_bridge.rs` owns the kernel-side adapter
logic. The bridge is invoked from `phase2_point_mass.rs` and
`phase2_rigid_body.rs` as a per-tick hook between integrator step and
force/moment evaluation, **after** sensor publication and **before**
the kernel applies effector commands. The bridge is a no-op when
`[fc]` is absent — existing `[mission.events]`-driven scenarios are
unaffected.

The bridge owns:
- `FcRunner` (already lives in `fc.rs`).
- `Vec<Box<dyn Sensor<Output = SensorMeasurement>>>` built from the
  scenario `[sensors]` block via the existing
  `SyntheticSensor` factories.
- A truth-state translator: `(PointMassState | RigidBodyState,
  EnvironmentSample) -> SensorTruth`. The exact `SensorTruth` shape
  is defined by `openbmp-sensors`; reuse it, do not duplicate.
- A backwards conversion: FC `EffectorCommandSet` keyed by
  `EffectorId` → kernel `EffectorRack::step` parameters. FC
  `EngineDemand` keyed by `EngineId` → kernel `EngineRack::apply`
  parameters.

**Specific force convention.** The accelerometer truth must be
**non-gravitational** body acceleration. The kernel computes the net
acceleration from `forces`; subtract gravity from it before handing
to the IMU sensor adapter. Read `crates/openbmp-state/src/`
`PointMassState` documentation; if `acceleration` includes gravity,
fix the conversion at the bridge layer.

**Determinism.** The bridge **must not** use `std::time`,
`SystemTime`, `Instant`, or any system RNG. The lockstep tripwire
(`crates/openbmp-testkit/src/fc_lints.rs`) covers `openbmp-fc/`; add
a parallel grep for `openbmp-cli/src/runner/fc_bridge.rs` so the
bridge can't quietly leak wall-clock APIs.

**Frame alignment.** FC ECI ≡ kernel ECI. Add a one-time
debug-assertion at bridge boot that the active `[frames]` profile is
ECI-shaped (e.g. `toy-fixed-earth`); the bridge fails closed
otherwise.

### D2. Solver choice for QP / SOCP — `clarabel`

Pre-approved (subject to `cargo deny check` on the locked version):
- License: Apache-2.0. Permissive, no copyleft.
- Pure Rust, no C bindings — `cargo deny` posture is straightforward.
- Conic backend: handles QP, SOCP, and SDP. **Kills three birds**:
  MPC's QP, LCvxLD's SOCP, and any future LMI-shaped subproblem.
- No `std::time` in the solve loop; deterministic when
  `Settings::eps_*` are pinned and `verbose = false`.
- Active maintenance (Stanford CVXGRP).

Reject:
- `osqp`: C dependency creates a build-system audit surface that
  doesn't pay for itself when `clarabel` covers QP too.
- `proxqp`: Inria, BSD-3, dense/sparse QP — fine, but no SOCP, so
  LCvxLD would need a second solver.
- `daqp`: active-set, MIT, but no SOCP and limited to small dense QPs.

Add `clarabel = { version = "0.9", default-features = false }` to
the workspace, gate solver-using code behind a `mpc` feature flag in
`openbmp-fc/Cargo.toml`. Default features stay the same;
`--features mpc` opts in.

**Vetting record:** create `docs/clarabel-vetting.md` documenting
the licence, the version pinned, the audit hash, the deterministic-
settings profile, and a note that the solver is used only inside
`openbmp-fc/mpc.rs` and `openbmp-fc/landing.rs` (both reintroduced
under `mpc` feature) and is invoked **only** during scheduled FC
ticks under the lockstep clock.

### D3. Estimator covariance representation

Default: full P with Joseph-form update (already shipped by the
audit). Add UD factorization (Bierman 1977) as a **feature-gated
alternative** `square-root-ekf` in `openbmp-fc`. The feature is
**off by default**; downstream HAL adopters running on
high-conditioning scenarios opt in. Reasoning: the academic-tier
default is the textbook reference; UD is for safety-critical use
cases where matrix conditioning matters. Both forms must produce
byte-identical state trajectories on well-conditioned scenarios —
add a determinism test.

### D4. UKF — sigma-point Julier-Uhlmann, classical (not square-root)

Implement a real sigma-point UKF in `openbmp-fc/src/estimator.rs`
following Wan & van der Merwe 2000 ("The Unscented Kalman Filter for
Nonlinear Estimation"). The classical form is sufficient for
academic-tier scenarios; square-root UKF (Merwe & Wan 2001) is
deferred to Phase 5. Same `Estimator` trait surface as
EKF / MEKF.

**Tunable parameters** (mirror Wan & van der Merwe):
- `alpha` (sigma spread) — typical `1e-3`.
- `beta` (distribution prior) — `2.0` for Gaussian.
- `kappa` (tertiary scaling) — `0.0` for state-vector-only filters.

**Process model.** Same rigid-body propagation as the EKF, with the
gravity model and quaternion renormalization plumbed through. **Do
not** propagate only the mean — the deleted scaffold did that and
was correctly removed. The UKF must propagate every sigma point and
recombine.

State dimension for the academic-tier UKF: 6 (attitude error +
gyro bias) for rapid validation; the full 15-state UKF is its own
scope and deferred.

### D5. MEKF Markley reset — direct upgrade, no flag

Replace the existing first-order reset with the Markley 2003
prescription. Specifically:
- After update, the multiplicative attitude error δθ is folded into
  the nominal quaternion via `q_nominal ⊗ exp(δθ/2)` where
  `exp(δθ/2)` uses the small-angle representation
  `[sin(|δθ|/2)·δθ̂, cos(|δθ|/2)]` (renormalize after).
- The state is then reset: `x[0..3] = 0`.
- The covariance reset is **not** identity in general; Markley §III
  prescribes a second-order correction. For attitude-error
  magnitudes < 0.05 rad the second-order correction is < 0.5 % and
  is omitted in many implementations; ship the corrected form
  (it's three lines), not the omission.

Reference: Markley, F. L. (2003). "Attitude error representations
for Kalman filtering." J. Guidance, Control, Dynamics 26(2):311–317,
§III.B equation (33).

### D6. Gauss-Markov bias dynamics

Replace the random-walk default with first-order Gauss-Markov:

    db/dt = -b/τ + w(t)

with `τ_gyro_bias_s` and `τ_accel_bias_s` as new fields in
`EkfParams` and `MekfParams`. Default `τ = f64::INFINITY` (the
random-walk limit) for backwards compatibility; scenarios that
specify finite `τ` get bounded bias estimates.

The discrete process noise per tick becomes:
`q_b = σ_b² · (1 - exp(-2·dt/τ))`

Reference: Bristow & Jacques 1999, "An overview of integrated
navigation systems for autonomous vehicles," § Bias modelling.

### D7. Iterated EKF on mag update — direct upgrade

Replace the single-pass mag linearization with an iterated relinearization
around the posterior mean (IEKF, see Bar-Shalom-Li-Kirubarajan §5.5.2).
- `max_iterations = 3` default.
- Convergence test: `‖x_k - x_{k-1}‖ < 1e-6 · ‖x_k‖`.
- Same chi-square gate at the *first* iteration; later iterations
  trust the earlier acceptance.

Same upgrade for the GNSS update is **out of scope** here (GNSS
update is linear in position/velocity; iteration provides nothing).

### D8. Notch filters — feature-gated, off by default

New module `crates/openbmp-fc/src/filters.rs` with a `Biquad` struct
implementing the standard Direct-Form-II Transposed biquad. Add a
`gyro_notch: Option<NotchConfig>` field to `AutopilotParams` keyed
per-axis. `NotchConfig` carries `(center_hz, bandwidth_hz, depth_db)`
per axis. Default: no filter (same as today).

Reference: PX4's `lib/mathlib/math/filter/NotchFilter.hpp` and
ArduPilot's `AP_HAL/utility/NotchFilter.h` for the canonical
biquad coefficients.

### D9. L1 adaptive augmentation — feature-gated, off by default

New module `crates/openbmp-fc/src/l1_adaptive.rs`. Feature flag
`l1-adaptive`. ~300 lines. Reference: Cao, C. & Hovakimyan, N.
(2010), "L1 Adaptive Control: Theory, Tools, and Applications."

The L1 augmentation wraps the rate loop: classical PID provides the
reference command; the L1 estimator + low-pass filter computes a
matched-uncertainty correction that's added to the PID output.
Saturation reporting must include the L1 contribution.

The feature must compile with `--no-default-features` (HAL
portability is non-negotiable).

### D10. Flatness-inspired trajectory tracking

New alternative trajectory loop in
`crates/openbmp-fc/src/autopilot.rs`. Selectable via
`[fc.autopilot.trajectory_kind = "pid" | "flatness_inspired"]`.
Default: `"pid"` (the existing per-axis loop).

Implementation: a flatness-inspired attitude-reference assignment
from desired acceleration and yaw. The full Mellinger & Kumar 2011
"Minimum snap trajectory generation and control for quadrotors"
tracker, including higher-derivative feed-forward from flat outputs,
is Phase-5 work.

Document the assumption: differential flatness presumes the vehicle
has full thrust-direction authority. Phase 4.C scenarios that don't
satisfy that assumption (fixed thrust direction, gimbal-only)
should keep `"pid"`.

### D11. Single-sample GLRT / CUSUM FDIR — alternative detectors via config

New detectors in `crates/openbmp-fc/src/fdir.rs`. Selectable via
`[fc.fdir.detector_kind = "burst_counter" | "single_sample_glrt" | "cusum"]`.
Default: `"burst_counter"` (existing). All detectors publish the
same `FdirStatus` topic shape but populate `tripped_mask` with
detector-specific bits.

References:
- GLRT: Willsky, A. S. (1976), "A survey of design methods for
  failure detection in dynamic systems."
- CUSUM: Page, E. S. (1954), "Continuous inspection schemes."

Surface the **fault tree**: the existing `FdirStatus.tripped_mask`
field is published but unpopulated. Define a bit assignment
(IMU = bit 0, baro = 1, GNSS = 2, mag = 3, scheduler-overrun = 4,
estimator-dead-reckoning = 5, autopilot-saturation = 6) and populate
it from each detector.

### D12. Covariance-weighted voter

New `CovarianceWeightedVoter` in `crates/openbmp-fc/src/voter.rs`.
Inputs: per-lane samples plus per-lane variance estimates.
Output: weighted mean with weights `w_i ∝ 1/σ_i²` (the optimal
Bayesian fusion of independent measurements with known covariance).

Fall-back: when no per-lane variance is supplied, decay to
`WeightedMeanScalar` with the static primary/others split.

Wire into `VotedImuIngest` etc. as an alternative voter; the
existing `MidValueSelectScalar` and `WeightedMeanScalar` stay.

### D13. WGS84-J2 gravity adapter

New `Wgs84J2Gravity` in `openbmp-physics::gravity` (the audit's
new `openbmp-physics` crate already exists). Expose it through a
HAL-portable interface that `openbmp-fc::estimator::GravityModel`
can wrap. Do **not** import `openbmp-env` from `openbmp-fc` — the
authority lives in `openbmp-physics`; both `openbmp-env` and
`openbmp-fc` consume it.

Reference: WGS84 Implementation Manual, NIMA Technical Report
TR8350.2, §3.

This is SOTA-adjacent for short-range scenarios; full EGM2008
spherical-harmonic gravity remains Phase 5.

### D14. WMM 2025 — defer, document

Do **not** implement the full 12-degree spherical-harmonic geomagnetic
field in Phase 4.C. The audit's `EarthDipoleField` is sufficient for
academic-tier attitude-tracking scenarios (degree-1 truncation). The
COF dataset shipping + the spherical-harmonic evaluator is its own
scope; document clearly in `magnetic.rs` that `EarthDipoleField`
is the academic baseline and full WMM is Phase 5.

### D15. NRLMSISE-00 atmosphere — defer

Same reasoning. USSA76 troposphere is sufficient for sounding-rocket
scenarios up to the tropopause. Phase 5 lands NRLMSISE-00 when
scenarios reach the upper atmosphere.

### D16. Multi-instance estimator — defer, trait surface is fine

PX4's `ekf2` runs N parallel filter instances and a lane voter
selects the active estimate. The `Voter` trait surface is the seam
that enables this; instantiating two `Ekf`s alongside a voter is
downstream-side (HAL adopter or scenario). Do not implement
multi-instance routing inside `openbmp-fc`.

### D17. Health & gain defaults — scenario-driven

Move the academic defaults out of FC code into scenario-driven
parameters with provenance. Specifically:
- `HealthParams::default()` → required in `[fc.health]` block.
- `default_gains()` → required in `[fc.gain_schedule.<phase>]`,
  no fall-through default.

This forces every scenario that uses the FC to declare its tunings
explicitly. Provenance is via the scenario's `provenance.md`.

### D18. Bus-history determinism

Extend `closed_loop_pipeline_runs_deterministically` in
`crates/openbmp-fc/tests/closed_loop.rs` to record **every published
topic** (sequence, time, payload) on every tick, run twice, assert
byte-equal across all topics. Today the test asserts only actuator-
stream identity, which is a subset.

Use a generic `BusHistoryRecorder` (~50 lines) that subscribes to all
registered topics via type erasure (`Box<dyn TopicSnapshot>` shape).

### D19. Long-duration stability test

New integration test
`crates/openbmp-fc/tests/long_duration_stability.rs`. 60-second
simulated run with all sensors active, asserting:
- No NaN in any state across all ticks.
- No covariance blow-up: max diagonal of P stays within 100× the
  initial value.
- Attitude estimate within 0.1 rad of truth (synthetic-truth
  comparison).
- No declared-budget overruns.

### D20. Cross-validation — Bar-Shalom textbook examples

Defer external-tool comparison (PX4 ekf2 published trajectories).
Instead, add unit tests that reproduce specific Bar-Shalom textbook
examples within published tolerances. The textbook is the immediate
SOTA reference for filter consistency.

## Workstreams

The Phase 4.C plan defines P0–P5. This program:

### Workstream A — Kernel↔FC bridge (P0)

**Goal:** `[fc]` scenarios actually drive a kernel run that emits
Parquet through the existing telemetry pipeline.

**Files:**
- `crates/openbmp-cli/src/runner/fc_bridge.rs` (new)
- `crates/openbmp-cli/src/runner/phase2_point_mass.rs` (modify —
  add bridge invocation hook)
- `crates/openbmp-cli/src/runner/phase2_rigid_body.rs` (modify)
- `crates/openbmp-cli/src/runner/mod.rs` (modify — `pub mod fc_bridge`)
- `crates/openbmp-testkit/src/fc_lints.rs` (modify — extend
  tripwire to cover `fc_bridge.rs`)

**Acceptance:**
1. `scenarios/closed-loop-attitude-hold/` produces a Parquet output
   under `out/closed-loop-attitude-hold.parquet` when run via the
   CLI.
2. Two reruns produce byte-identical Parquet (the existing
   determinism CI gate now applies).
3. `cargo build -p openbmp-fc --no-default-features` still clean
   (the bridge does not touch FC's no-default surface).
4. The closed-loop scenario's `expected.toml` tolerances are met.

### Workstream B — Estimator SOTA (P2)

**Goal:** Move EKF/MEKF beyond textbook-baseline.

Order:
1. **Markley MEKF reset** — `crates/openbmp-fc/src/estimator.rs::Mekf::update_*`.
2. **Gauss-Markov bias dynamics** — `EkfParams` / `MekfParams` add
   `tau_*_bias_s` fields; `predict` uses GM transition matrix.
3. **Iterated EKF mag update** — `Ekf::update_mag` and
   `Mekf::update_mag` add a 3-iteration relinearization loop.
4. **Square-root EKF (UD)** — feature-gated `square-root-ekf`. Reuse
   Bierman 1977 reference implementation. Determinism test that the
   feature-on and feature-off paths agree to within `1e-9` on a
   well-conditioned reference scenario.
5. **WGS84-J2 gravity adapter** — `openbmp-physics::gravity::Wgs84J2Gravity`
   with a `crate::estimator::GravityModel` wrapper.

**Acceptance:**
- Markley reset preserves second-order accuracy: a unit test that
  measures attitude-error growth rate vs the first-order reset shows
  the second-order correction reduces error by O(δθ²).
- Gauss-Markov bias estimate converges to truth within 5σ over a
  60-s run with a 100-s time constant.
- Iterated mag update reduces final attitude error by ≥ 10 % vs
  single-pass on a noisy long run.

### Workstream C — UKF (P2)

**Goal:** Real sigma-point UKF replacing the deleted scaffold.

**Files:**
- `crates/openbmp-fc/src/estimator.rs` — add `Ukf` and `UkfParams`.

**Acceptance:**
- Determinism test (same proptest harness as EKF).
- NEES test on a synthetic-truth scenario: ensemble NEES within the
  95 % chi-square bound for a 6-state filter.
- Comparison test vs MEKF on the same scenario: UKF NEES no worse
  than MEKF NEES (else the UKF is misconfigured).

### Workstream D — Solver-backed control (P0–P3 mixed)

**Goal:** Real MPC and LCvxLD/SCvx soft-landing backed by a vetted
solver.

**Files:**
- `Cargo.toml` (workspace) — add `clarabel`.
- `crates/openbmp-fc/Cargo.toml` — `mpc` feature flag.
- `crates/openbmp-fc/src/mpc.rs` (new under `mpc` feature) — receding-
  horizon convex-QP MPC for attitude tracking.
- `crates/openbmp-fc/src/landing.rs` (new under `mpc` feature) —
  LCvxLD soft-landing using `clarabel`'s SOCP backend.
- `docs/clarabel-vetting.md` (new).

**Acceptance:**
- `cargo deny check` passes with `clarabel` added.
- `cargo build -p openbmp-fc --no-default-features` still clean
  (MPC is feature-gated).
- Determinism test on the MPC: same input → same output across
  reruns. `Settings::eps_*` pinned in code.
- LCvxLD test reproduces the Acikmese & Ploen 2007 example trajectory
  within published tolerances.

### Workstream E — Test depth (P1)

**Goal:** Fill every test gap the audit identified.

Sub-tasks:
1. **MEKF property tests** — mirror EKF proptest; add to
   `crates/openbmp-fc/tests/mekf_properties.rs`.
2. **Monte-Carlo NEES** — new harness in
   `crates/openbmp-fc/tests/nees_consistency.rs`. 100-realization
   ensemble; 95 % chi-square bound; 3-, 6-, 15-DOF cases for
   MEKF / UKF / EKF.
3. **Innovation autocorrelation lags 2..10** — extend
   `ekf_properties.rs`. Welch's test bound 0.2 per lag.
4. **Bus property tests** — new
   `crates/openbmp-fc/tests/bus_properties.rs`. Sequence
   monotonicity, single-writer-multi-reader, registration order
   invariance for read but order-defining for iteration.
5. **Scheduler stress tests** — new
   `crates/openbmp-fc/tests/scheduler_stress.rs`. Adversarial
   periodic + topic-driven mixes that force the overrun event to
   fire on exactly the predicted ticks.
6. **Voter adversarial tests** — extend `voter::tests`. Stuck-lane,
   drifting-lane, two-divergent-lanes scenarios.
7. **FDIR fault-injection** — new
   `crates/openbmp-fc/tests/fdir_fault_injection.rs`. 1-, 3-, 10-σ
   bias injection on each sensor; assert detector trips at the
   right tick.
8. **GNSS-dropout dead-reckoning recovery** — new
   `crates/openbmp-fc/tests/gnss_dropout.rs`. 2-second dropout;
   assert `EstimatorStatus::dead_reckoning` ↔ `FailsafeFlags::
   estimator_dead_reckoning` ↔ commander-disarm flow.
9. **Multi-phase mission test** — new
   `crates/openbmp-fc/tests/multi_phase.rs`. 3-phase scenario with
   a safe-state phase; FDIR trip → `safe_state_requested` →
   commander transitions into safe-state phase.
10. **Full bus-history determinism** — extend
    `closed_loop.rs` per D18.
11. **60-s long-duration stability** — new
    `crates/openbmp-fc/tests/long_duration_stability.rs` per D19.
12. **Bar-Shalom textbook examples** — new
    `crates/openbmp-fc/tests/textbook_examples.rs` reproducing
    selected examples from §5.4 (consistency) and §5.5
    (innovation analysis).

### Workstream F — Voter / FDIR SOTA (P4)

1. **Voter:** extend `sensor.status` lane reporting from voted IMU
   to baro / GNSS / mag.
2. **CovarianceWeightedVoter** per D12.
3. **GLRT and CUSUM detectors** per D11.
4. **Fault tree:** populate `FdirStatus.tripped_mask` per the bit
   assignment in D11.

### Workstream G — Autopilot SOTA (P3)

1. **Notch filters** per D8.
2. **Differential-flatness trajectory** per D10.
3. **L1 adaptive augmentation** per D9.
4. **Observer-form anti-windup investigation:** read Hippe 2006;
   compare against current back-calculation in a side-by-side
   simulation; ship the observer form **only if** it demonstrably
   improves boundedness on a representative scenario. Otherwise
   keep back-calculation and document the decision in the autopilot
   docstring.

### Workstream H — Environment models (P5 — partial)

1. **WGS84-J2** per D13.
2. **WMM 2025** — **defer**, document per D14.
3. **NRLMSISE-00** — **defer**, document per D15.

### Workstream I — Documentation closure

1. Update `docs/phase-4-plan.md`'s "landed" section to reflect post-
   4.C state.
2. Update `crates/openbmp-fc/README.md` module map.
3. Update `docs/design-concept.md § Phase 4` for consistency.
4. Write `docs/phase-5-plan.md` skeleton with WMM, NRLMSISE,
   EGM2008, multi-instance estimator, square-root UKF, observer
   anti-windup as P0/P1.
5. Mark `docs/phase-4-plan.md` and `docs/phase-4b-plan.md` for
   removal at Phase 5 closure (mirror the Phase 1/2/3 closure
   pattern).

## Order of execution

The dependency-respecting linear order:

1. **A** — Kernel↔FC bridge. Unblocks every end-to-end test.
2. **E** (1–9) — Test depth that doesn't depend on the bridge:
   MEKF properties, NEES, lags, bus, scheduler, voter, FDIR
   injection, GNSS dropout, multi-phase. These exercise FC-only
   surface.
3. **B** — Estimator SOTA. Markley first (small, direct upgrade);
   Gauss-Markov second; iterated mag third; square-root EKF feature
   last (largest, gated).
4. **F** — Voter / FDIR SOTA. Independent of B and C.
5. **C** — UKF. Largest single algorithm addition; can land in
   parallel with D.
6. **G** — Autopilot SOTA. Notch first (simple); differential-
   flatness second; L1 third (largest).
7. **D** — Solver-backed MPC and LCvxLD. Requires `clarabel`
   workspace dep + vetting record + feature flag.
8. **H** — Environment: WGS84-J2; defer WMM / NRLMSISE.
9. **E (10–12)** — Full bus-history determinism, long-duration
   stability, textbook examples. These depend on the rest landing.
10. **I** — Documentation closure.

## Definition of done

Phase 4.C is closed when **all** of:

1. Every P0 and P1 item in `docs/phase-4c-plan.md` is implemented
   in working tree.
2. P2 estimator items (Markley, GM bias, iterated mag,
   square-root feature, WGS84-J2, UKF) implemented.
3. P3 autopilot items (notch, differential-flatness, L1) implemented;
   anti-windup decision documented.
4. P4 voter / FDIR items implemented.
5. P5 environment items: WGS84-J2 only; WMM / NRLMSISE deferred to
   Phase 5 with explicit module-docstring notes.
6. Solver vetting record `docs/clarabel-vetting.md` exists; MPC and
   LCvxLD reintroduced under `mpc` feature.
7. `cargo fmt --all -- --check` clean.
8. `cargo clippy --workspace --all-targets --all-features -- -D
   warnings` clean.
9. `cargo test --workspace --all-features` green.
10. `cargo test -p openbmp-fc --no-default-features` green.
11. `cargo test -p openbmp-fc --features mpc,square-root-ekf,l1-adaptive`
    green.
12. `cargo build -p openbmp-fc --no-default-features` clean.
13. `cargo deny check` clean (with `clarabel` added).
14. `cargo machete` clean.
15. Lockstep-clock tripwire passes for both `openbmp-fc/` and
    `openbmp-cli/src/runner/fc_bridge.rs`.
16. The closed-loop scenario produces byte-identical Parquet across
    reruns.
17. The 60-s long-duration test passes.
18. NEES test passes for EKF, MEKF, UKF.
19. Phase-1/2/3 byte-stable scenarios still pass (regression).
20. `docs/phase-4-plan.md`, `docs/phase-4-audit-findings.md`,
    `docs/phase-4c-plan.md`, `crates/openbmp-fc/README.md`, and
    `docs/design-concept.md` are consistent with the post-4.C
    reality.

## Out of scope

Explicit Phase 5 deferrals:
- Real WMM 2025 spherical-harmonic geomagnetic field with the COF
  dataset.
- NRLMSISE-00 upper atmosphere.
- EGM2008 spherical-harmonic gravity (WGS84-J2 is the Phase 4.C
  bridge).
- Multi-instance estimator routing (`Voter` trait surface stays;
  instantiation is downstream).
- Square-root UKF.
- Cross-validation against PX4 ekf2 / ArduPilot NavEKF3 published
  trajectories.
- Real ULog binary log format support in `replay.rs`.
- Observer-form anti-windup (gated on the empirical comparison —
  ships only if it wins).

## Authorization

Authorized:
- Read / edit any file in the workspace.
- Add the `clarabel` dependency (pre-approved per D2; still must
  pass `cargo deny check`).
- Refactor crates, add new features.
- Add unit / property / integration tests freely.
- Run `cargo fmt`, `cargo clippy --fix`, `cargo test`,
  `cargo deny check`, `cargo machete`.
- Create commits in dependency-ordered chunks; the user pushes.

Requires explicit user approval first:
- Adding any other workspace dependency from crates.io.
- Bumping MSRV.
- Removing files outside the natural consequence of refactor.
- Force-pushes, branch deletions, `--no-verify`, `git reset --hard`.

## Tone

The audit closed real findings; the prior author was conscientious.
But Phase 4.A+B's "all gaps closed" claim was already overstated
once. The same temptation will appear in 4.C — *"the tests pass,
ship it."* The tests passing is necessary, not sufficient.

Specifically:
- **Markley reset:** verify the second-order correction is actually
  applied by checking the resulting attitude-error norm decay rate
  against analytic prediction.
- **Gauss-Markov bias:** verify the steady-state variance matches
  the closed-form `σ² = q · τ / 2`, not just "the filter doesn't
  diverge."
- **Iterated EKF:** verify convergence is achieved within
  `max_iterations`, not just "the loop runs."
- **UKF:** verify the unscented transform produces the expected
  Gaussian moments on a known nonlinear function (e.g. `f(x) = x²`),
  not just "the filter doesn't NaN."
- **Notch filter:** verify the magnitude response at the notch
  frequency is `< -depth_db` against a chirp test, not just
  "compiles."
- **L1 adaptive:** verify the bounded matched-uncertainty estimate
  on a unit-step disturbance, not just "the demo runs."
- **Differential flatness:** verify the analytic attitude reference
  matches the observed reference on a known smooth trajectory, not
  just "the test asserts no NaN."
- **MPC / LCvxLD:** verify against published reference trajectories,
  not just "the solver returns a status."

The auditor's findings document should call out each "ship-it" risk
when it appears and either fix it in the same session or defer to
Phase 5 with a written reason.
