# Phase 5 Plan

Phase 5 starts from the closed Phase 4 work (lockstep flight controller,
kernel↔FC bridge, EKF / MEKF / classical UKF, three-loop autopilot,
phase-gated mixer, voter / health / FDIR seam, physics consolidation,
naming-honesty pass) and turns the simulator-local autopilot into a
research-grade GNC sandbox with credible external cross-validation.

The user-facing emphasis for Phase 5 is **autopilot coverage**: the
biggest deliverables in this phase are the SOTA autopilot algorithms
(Mellinger-Kumar minimum-snap, Cao-Hovakimyan L1 adaptive, observer-form
anti-windup, full receding-horizon MPC, control allocation) and the
estimator-side enablers that make those algorithms credible
(square-root UKF, multi-lane estimator routing, IMM, windowed GLRT).
Environment, kernel, HIL, and external-validation work is included so
the autopilot evidence rests on a richer envelope than Phase 4 had.

This plan replaces the Phase-5 skeleton in place. Items previously
labelled P0 / P1 / P2 are folded into the sub-phase ordering below.

## Scope and anti-scope

**In scope (Phase 5):**

- Autopilot SOTA: differential-flatness trajectory tracker; full L1
  adaptive architecture; observer-form anti-windup; LQR and INDI
  baselines; finite-horizon receding-horizon MPC; control allocation
  with axis priority and saturation reporting.
- Estimator / FDIR maturity: square-root UKF for the full 15-state
  vector; multi-instance estimator routing + active-lane selection;
  IMM (Bar-Shalom) maneuver-aware estimator; Willsky 1976 windowed
  mean-shift GLRT; parity-space residual generator.
- Environment: NRLMSISE-00 (in-house Rust port, public coefficients
  only); EGM2008 truncated spherical-harmonic gravity beyond
  WGS84-J2.
- Kernel and scenario: multi-rate scheduling promoted to first-class;
  multi-body simultaneous propagation (post-separation); DOPRI5/8
  adaptive integrator behind an explicit profile flag; API cleanup
  pass.
- HIL, replay, validation: optional socket-bridge HIL pattern
  (in-house postcard wire format); real ULog parser + PX4 ekf2
  cross-validation case; ArduPilot dataflash parser + NavEKF3
  cross-validation case; Bar-Shalom textbook §5.4 / §5.5
  reproducibility cases; full closed-loop long-duration soak on
  Calisto.

**Explicitly out of scope (deferred to Phase 6 or rejected outright):**

- NRLMSIS 2.x, HWM14, JB-2008 — Phase 6 hypersonic atmosphere
  follow-ons.
- Real-gas thermodynamics, hypersonic aero methods, aerothermal,
  boundary-layer state, continuum-to-rarefied bridging, re-entry
  trajectory infrastructure, Park two-temperature thermochemistry,
  surface ablation — all Phase 6.
- Proportional navigation, augmented PN, sliding-mode homing,
  bank-to-turn / skid-to-turn terminal-mode autopilot variants,
  TERCOM / DSMAC / scene-matching navigation — categorically
  rejected by `docs/safety-boundaries.md` and never appear in
  Phase 5 sub-phases.
- Real device drivers, real bus protocols (CAN, MAVLink, MIL-STD-1553,
  DDS, I²C, SPI, UART), real flight-computer firmware. Adopters who
  build a HAL ship those in their own repositories.
- Real fielded-vehicle parameter sets. PX4 / ArduPilot cross-validation
  uses public log captures of academic / community vehicles only;
  classified or proprietary captures are rejected on provenance
  grounds.

**Naming-honesty discipline (carried over from Phase 4.C):**

Until a sub-phase ships the full SOTA algorithm, the in-tree types keep
their `*Inspired` suffix. The corresponding sub-phases land the full
algorithms behind new types and retire the `*Inspired` types only
after the SOTA path is wired through the scenario, golden-tested, and
documented. Phase 5.A.1.D applied this discipline to retire
`TrajectoryKind::FlatnessInspired` once `TrajectoryKind::DifferentialFlatness`
landed scenario-tested via `[fc.trajectory]` and the
`diff-flatness-figure-eight` end-to-end run. Phase 5.A.2.C applied the
same rule to retire the interim L1-inspired rate-loop types after the
full `L1AdaptiveParams` / `L1AdaptiveChannel` path landed. No commit
may rename a type to drop "Inspired" without first shipping the
algorithm change.

## Success criteria

Phase 5 closes when:

1. Every sub-phase below has merged with its declared exit criterion
   met and its tests in CI.
2. The determinism CI gate runs at minimum the analytic-toy, Niskanen,
   Calisto, and the full closed-loop long-duration soak twice on
   `x86_64-unknown-linux-gnu` and asserts byte-identical Parquet.
3. Every external dataset introduced in Phase 5 (NRLMSISE-00
   coefficients, EGM2008 coefficients, PX4 / ArduPilot log captures)
   ships with a sibling `provenance.md` and a SHA-256 pin in the
   scenario or fixture file that references it.
4. No safety-boundary violation enters the repository: no proportional
   navigation, no terminal homing, no real fielded vehicle data,
   no real device drivers, no real bus protocols.
5. The non-suitability disclaimer remains on every release artifact.

## Delivery status

Sub-phase status as of the latest commit on `main`. Update this table
in lockstep with each sub-phase landing.

| Sub-phase | Status | Commit |
|---|---|---|
| 5.0 — pre-work | shipped | `f24aadc` (+ audit follow-ups in `c241ee8`) |
| 5.A.1.A — minimum-snap math + flat-output references + autopilot wiring | shipped | `8b5d0ab` |
| 5.A.1.B — `[fc.trajectory]` v3 scenario block + parser + runner integration | shipped | `27e079a` |
| 5.A.1.C — `diff-flatness-figure-eight` scenario + e2e test + tolerance table | shipped | `27e079a` |
| 5.A.1.D — retire `TrajectoryKind::FlatnessInspired` | shipped | `27e079a` |
| 5.A.2.A — `direct_torque` effector + rigid-body figure-eight integration | shipped | `16414b3` |
| 5.A.2.B — L1 adaptive math module | shipped | `041438c` |
| 5.A.2.C — L1 autopilot wiring + retire L1-inspired interim types | shipped | `bb17ea4` |
| 5.A.2.D — closed-loop L1 validation under roll-axis ReducedRate fault | shipped | `af943d1` |
| 5.A.3.A — observer-form anti-windup + back-calculation parameterisation | shipped | `327f2dc` |
| 5.A.3.B — per-axis LQR rate loop + structure-preserving DARE solver | shipped | `327f2dc` |
| 5.A.3.C onwards | pending | — |

## Vehicle-class scope

The Phase 5 autopilot algorithms ship for **thrust-along-body-z
vehicles**, which is the canonical assumption shared by:

- single-engine rockets pointing along their long axis,
- propulsive landers (e.g. SpaceX Falcon-9 stage 1, Blue Origin
  New Shepard),
- multirotors (the Mellinger & Kumar 2011 paper that motivated
  5.A.1 was a quadrotor paper; the math is identical).

The OpenBMP repository's *scenario corpus and validation evidence
ship for rocket-class vehicles only* — sounding rockets, multi-stage
launch vehicles, propulsive landers, and (Phase-6) lifting re-entry.
Multirotor sim is **downstream-extension territory**: the trajectory
math drops into the framework cleanly, but the project does not
ship a multirotor mixer (4-prop mixing matrix → body torque), a
multirotor force/moment model, or a multirotor scenario fixture.
The `docs/design-concept.md § Vehicle Classes` list is the
authoritative scope; UAVs / multicopters are absent by design, and
adding them requires the algorithm work above plus framework-side
mixer / model / scenario authoring that is out of Phase-5 scope.

Categorical out-of-scope (per `docs/safety-boundaries.md`):
fixed-wing aero-stability augmentation systems, missile autopilot
modes (proportional navigation, augmented PN, sliding-mode homing,
bank-to-turn / skid-to-turn terminal-mode variants), terrain-matching
navigation (TERCOM / DSMAC), real flight-deployable controllers
(no DO-178C compliance claims, no real bus protocols).

## Sub-phase roadmap

Sub-phases are grouped A (autopilot SOTA), B (estimator/FDIR), C
(environment), D (kernel/scenario), E (HIL/validation). Within a group
sub-phases must be completed in order. Across groups, A.1 may proceed
in parallel with B.1 / C.1 / D.1, and so on. Group E sub-phases depend
on at least the matching A/B sub-phase that they validate.

### 5.0 — Pre-work: API cleanup, scenario-format additions

**Scope.** Retire the simulator-crate re-export shims that Phase 4
left behind (e.g. `openbmp_sim::SimState`) now that downstream users
have had one phase to migrate to `openbmp-models` /
`openbmp-mission` / `openbmp-sensors`. Extend the scenario format
with the `[fc.estimator_lanes]`, `[fc.autopilot_allocation]`,
`[fc.fdir.detector]`, `environment.atmosphere = "nrlmsise00"` /
`[atmosphere].kind = "nrlmsise00"`,
`environment.gravity = "egm2008"`, `[schedule]` /
`[[schedule.group]]`, and `[multi_body]` /
`[[multi_body.separation]]` blocks consumed by the rest of Phase 5.
Each block parses with `serde(deny_unknown_fields)` and is gated on a
schema-version bump in `openbmp.scenario`.

**Exit criterion.** The Phase 4 deprecation shims are removed; the
new scenario blocks parse and reject unknown fields; existing Phase 4
scenarios still parse byte-identically without those blocks; doc
update lands in `docs/scenario-format.md`.

**Validation evidence.** Parser unit + property tests; scenario-fuzz
target updated; one round-trip golden parses every shipped scenario.

**Scope guardrail.** Schema-version bump; no behaviour change to the
Phase-4 closed-loop integration test until the next sub-phase
exercises a new field.

---

### Group A — Autopilot SOTA

#### 5.A.1 — Mellinger-Kumar minimum-snap differential-flatness tracker

**Scope.** Land the full Mellinger & Kumar 2011 differential-flatness
trajectory tracker for thrust-along-body-z vehicles:

- `MinimumSnapTrajectory` — piecewise-polynomial trajectory generator
  parameterised by waypoint sequence, segment durations, and
  derivative continuity (up to snap). Solver runs at scenario load,
  not in the FC tick.
- Analytical attitude / attitude-rate / angular-acceleration
  references derived from `(position, velocity, acceleration, jerk,
  snap)` of the polynomial. The reference quaternion is the body-z
  alignment with the desired specific force; the reference yaw is the
  scenario yaw spline; the reference body rates and angular
  accelerations follow the Mellinger-Kumar derivation.
- New `TrajectoryKind::DifferentialFlatness` variant. The Phase 4.C
  `TrajectoryKind::FlatnessInspired` is retired in 5.A.1.D once the
  closed-loop `diff-flatness-figure-eight` scenario lands; that is
  the full path the type was always meant to be.

**Exit criterion.** A new academic scenario flies a 3-D figure-eight
or slalom trajectory with the differential-flatness tracker active
across the full waypoint sequence, the trajectory generator's
polynomial coefficients are byte-stable across reruns, and the
closed-loop pipeline produces byte-identical telemetry across two
consecutive runs. A tight attitude-error tolerance envelope is
documented with the L1 adaptive and observer-form anti-windup
sub-phases (5.A.2 / 5.A.3), where the actuator-to-body torque path is
strengthened enough to make that envelope meaningful.

**Validation evidence.** Unit tests for polynomial moment minimisation;
property test that snap is finite at every interior knot; analytic-toy
case where a constant-acceleration segment matches the closed-form
attitude reference.

**References.** Mellinger, D. and Kumar, V., *Minimum snap trajectory
generation and control for quadrotors*, IEEE ICRA 2011. The reference
is a quadrotor paper but the differential-flatness derivation is
direct for any thrust-along-body-z vehicle (single rocket, lander).

**Scope guardrail.** Trajectory waypoints are scenario-defined points
in inertial space, not real-world locations. No proportional-navigation
fallback if the trajectory tracker saturates — the autopilot reverts
to the existing PID + back-calculation path, not to a homing law.

#### 5.A.2 — Cao-Hovakimyan L1 adaptive (full architecture)

5.A.2 ships in four sub-slices, each a separate commit (mirroring
the 5.A.1.A–D pattern):

- **5.A.2.A — rigid-body figure-eight integration.** Add a
  `direct_torque` effector kind to `openbmp-scenario`
  (`EffectorKindConfig::DirectTorque { axis: TorqueAxis,
  effectiveness_n_m_per_rad: f64 }` with `TorqueAxis ∈ {Roll, Pitch,
  Yaw}`). Add a `DirectTorqueMomentAdapter` in `openbmp-vehicle`
  that mirrors the `EngineClusterMomentAdapter` shape: reads
  declared effector deflections via `EffectorActualsView`, multiplies
  by per-effector effectiveness, sums per body axis. Convert
  `scenarios/diff-flatness-figure-eight/scenario.toml` from
  `point_mass` to `rigid_body` with three `direct_torque` effectors
  mapped through `[fc.actuator_channels]`. Replace the
  `kernel_steps` placeholder metric with an attitude-error-bound
  metric (loose tolerance documented as the Phase-5.A.2.A baseline,
  with the existing PID + DifferentialFlatness autopilot driving
  the rigid body). This slice closes the closed-loop tracking
  validation gap the user flagged across the 5.A.1 audit and
  unblocks 5.A.2.B–D.
- **5.A.2.B — L1 adaptive math module.** New
  `crates/openbmp-fc/src/l1_adaptive_full.rs` with
  `L1ReferenceModel`, `L1StatePredictor`,
  `L1PiecewiseConstantAdaptation`, `L1LowPassFilter`. Unit tests
  for each component (projection bound, LPF response, PCA
  estimator stability). Bandwidth-projection inequality
  `ω_c · L < 1` asserted at construction; violations fail closed.
- **5.A.2.C — autopilot wiring + retire L1-inspired interim types.** New
  `L1AdaptiveParams` / `L1AdaptiveChannel` types behind the
  existing `l1-adaptive` feature flag; wire into the rate loop
  replacing the Phase-4 L1-inspired interim channel. Retire the
  interim params/channel types in the same commit
  (naming-honesty discipline, mirrors 5.A.1.D). Scenario parameters
  live under `[fc.autopilot_params.l1_adaptive]` as a v3-only
  extension.
- **5.A.2.D — closed-loop validation under roll-axis ReducedRate
  fault.** Two sibling scenarios share the rigid-body figure-eight:
  `scenarios/diff-flatness-figure-eight-l1` activates the L1
  augmentation, `scenarios/diff-flatness-figure-eight-baseline`
  omits the `[fc.autopilot_params.l1_adaptive]` block so the rate
  loop runs as a pure PID. Both carry an
  `EffectorFault::ReducedRate { factor = 0.7 }` on the roll-torque
  effector, simulating a 30 % slew-rate loss on that axis as a
  matched, axis-local actuator disturbance. The
  `crates/openbmp-cli/tests/diff_flatness_l1_robustness_e2e.rs`
  e2e test runs both, computes the maximum body-frame
  angular-velocity magnitude `|ω|` for each, and asserts the L1
  run's peak is ≤ ½ × the baseline (observed ratio ≈ 0.09 on the
  reference platform — L1 reduces max `|ω|` from ~0.59 rad/s to
  ~0.05 rad/s and keeps the roll/pitch effectors out of the
  saturation limit). Each scenario is also asserted byte-stable
  across reruns.

**Scope.** Replace the Phase 4.C L1-inspired interim channel (scalar
projection + first-order LPF) with the full L1 adaptive controller
architecture:

- `L1ReferenceModel` — linear reference model with documented
  bandwidth.
- `L1StatePredictor` — predictor for the matched-uncertainty channel,
  propagated independently from the plant; the prediction error drives
  the piecewise-constant adaptation law.
- `L1PiecewiseConstantAdaptation` — piecewise-constant matched-
  uncertainty estimator, sampled at the scheduler's adaptation rate.
- `L1LowPassFilter` — first-order strictly-proper low-pass filter
  with bandwidth ω_c selected per Cao-Hovakimyan robustness margin
  bound (per-channel ω_c declared in the scenario, validated against
  the reference-model bandwidth).
- New `L1AdaptiveParams` / `L1AdaptiveChannel` types behind the
  existing `l1-adaptive` feature flag. The Phase 4.C L1-inspired
  interim params/channel types are retired in the same sub-phase once
  the full path lands.

**Exit criterion.** The 5.A.2.D sibling scenarios run the rigid-body
figure-eight with 3-axis direct-torque effectors, inject an
`EffectorFault::ReducedRate { factor = 0.7 }` on the roll-torque
effector, and compare PID-only against L1-augmented control under the
same deterministic seed. The e2e tolerance envelope lives in
`crates/openbmp-cli/tests/expected/diff-flatness-figure-eight-l1.toml`
and gates kernel steps, quaternion normalisation, attitude tracking
against the minimum-snap reference, visible baseline perturbation,
and the L1-vs-baseline max-body-rate ratio. The bandwidth-projection
inequality `ω_c · L < 1` is asserted at scenario load and violations
fail closed.

**Validation evidence.** Unit + property tests for the projection
operator (estimate stays inside bound); state-predictor / reference-
model agreement under nominal conditions; closed-loop figure-eight
attitude tracking under the roll-axis ReducedRate disturbance.

**References.** Cao, C. and Hovakimyan, N., *L1 Adaptive Control
Theory: Guaranteed Robustness with Fast Adaptation*, SIAM 2010 — the
authoritative architecture monograph. Modern piecewise-constant
adaptation refinements: Wang, Y. et al., *Robust flight control
based on a nonlinear-L1 adaptive control with modified
piecewise-constant and HIL experiments*, Nonlinear Dynamics 2024
(NDI-L1, modified PCA reduces computational burden without losing
estimation accuracy); MDPI Actuators 2024 *Design and Implementation
of an L1 Adaptive Proportional Output Feedback Controller*. The
Lund University thesis *Augmenting L1 Adaptive Control of
Piecewise Constant Type to Aerial Vehicles* documents the PCA path
applied to fighter-aircraft + mini-UAV envelopes.

**Scope guardrail.** The L1 controller is the rate-loop augmentation,
not a guidance law. No envelope-protection hooks tied to operational
limits; the bandwidth bound is academic.

#### 5.A.3 — Observer-form anti-windup; LQR baseline; INDI baseline

**Scope.** Three independent autopilot baselines that Phase 4.C
deliberately deferred. Phase 5.A.3.A and 5.A.3.B have shipped; the
remaining sub-phases (INDI baseline + comparison harness) are
slated as 5.A.3.C and 5.A.3.D and follow the same per-slice
review pattern as 5.A.1.A–D / 5.A.2.A–D.

1. **Observer-form anti-windup (Phase 5.A.3.A — shipped).** Adds an
   `AntiWindupKind` enum (`BackCalculation { gain }` / `ObserverForm
   { tracking_time_s }`) consumed by all three PID loops via
   `pid_step`. The two variants are mathematically equivalent on a
   SISO PID (with `gain = 1 / tracking_time_s`) but expose distinct
   design intents — empirical gain tuning vs Åström-Rundqwist 1989
   observer pole placement. Scenario block
   `[fc.autopilot_params.anti_windup]` is v3-only; legacy
   `anti_windup_gain` is preserved as a back-compat shim that maps
   to `BackCalculation` so Phase-1–4 scenarios stay byte-identical.
2. **LQR baseline (Phase 5.A.3.B — shipped).** Per-axis 2-state
   augmented LQR (state `[ω − ω_ref, ∫(ω − ω_ref) dt]`) gated behind
   the `lqr` Cargo feature. The runner solves the per-axis Discrete
   Algebraic Riccati Equation at scenario load for single-body
   assemblies with diagonal inertia, using the Anderson 1978 /
   Chu-Fan-Lin-Wang 2004 structure-preserving doubling algorithm
   (quadratic convergence even for stiff systems whose closed-loop
   poles approach the unit circle). New scenario fields:
   `[fc.autopilot_params.rate_loop_kind]` selects `pid` (default) or
   `lqr`; `[fc.autopilot_params.lqr]` declares per-axis cost weights
   `q_omega`, `q_int`, `r`. Anti-windup applies uniformly to both
   PID and LQR rate loops; L1 augmentation works on top transparently.
   Demonstration scenario:
   `scenarios/diff-flatness-figure-eight-lqr/scenario.toml`.
3. **INDI baseline (Phase 5.A.3.C — pending).** Incremental
   Nonlinear Dynamic Inversion for the rate loop, per Smeur, Chu,
   de Croon 2016 academic formulation. Inverts only the diagonal of
   the control-effectiveness matrix (no full plant inversion);
   per-axis filtered-derivative term for the gyro-rate signal.
   Useful as a reference baseline for the academic envelope where
   INDI is the standard comparison.
4. **Controller comparison harness (Phase 5.A.3.D — pending).**
   Runs the figure-eight scenario family across PID baseline + L1 +
   observer-form anti-windup + LQR + INDI under matched
   disturbances; emits a markdown table with per-axis tracking RMS
   and peak commanded torque. Same format as the Phase-3
   `compare_filters` harness.

**Exit criterion.** Each baseline runs in a dedicated scenario and
produces deterministic actuator output; the compare harness emits a
report with side-by-side metrics across PID + LQR + L1 + INDI for
the same disturbance.

**Validation evidence.** Unit tests for the DARE solver
(small algebraic residual, positive costs, closed-loop poles inside
the unit circle); scalar observer-form anti-windup tests cover the
SISO tracking-time/back-calculation reduction; INDI rate-loop
closure on a unit-inertia rigid body matches the analytic
angular-velocity step.

**References.** Åström, K. J. and Rundqwist, L., *Integrator
windup and how to avoid it*, ACC 1989 (observer-form / conditioning
technique). Stevens & Lewis 2015 (LQR formulation, discrete-time
DARE). Smeur, E., Chu, Q., and de Croon, G., *Adaptive Incremental
Nonlinear Dynamic Inversion for Attitude Control of Micro Air
Vehicles*, JGCD 2016 (foundational INDI). Modern INDI references:
Smeur et al., *From fundamentals to applications of incremental
nonlinear dynamic inversion: A survey on INDI – Part I*, Chinese
Journal of Aeronautics 2024 and *Part II*, ChinaXiv / CJA 2025
(comprehensive INDI surveys; INDI publication count grew from 186
in 2016 to ≈1530 by 2024). Robustness-augmented INDI for stretch
tracking: arXiv 2501.07223 *Improving Incremental Nonlinear Dynamic
Inversion Robustness Using Robust Control in Aerial Robotics*
(2025) — hybrid INDI + linear-structured H∞ achieves > 50 %
disturbance-rejection improvement in published gust simulations.

**Scope guardrail.** Baselines are educational comparisons; no
baseline ships as the default autopilot.

#### 5.A.4 — Receding-horizon MPC

**Scope.** Promote the Phase 4.C single-step attitude box-QP
(`solve_attitude_box_qp`) to a finite-horizon receding-horizon
MPC:

- `RecedingHorizonAttitudeMpc` — N-step QP over an attitude error
  state with attitude-rate input, body-torque limits, body-rate
  limits, and a quadratic terminal cost.
- `RecedingHorizonTranslationalMpc` — N-step SOCP over a position
  / velocity state with thrust-direction (cone) constraint and
  thrust-magnitude bound.
- Clarabel solver settings stay deterministic (`verbose = false`,
  pinned tolerances, no time-limit).
- Horizon length, weights, and limits live in `MpcParams` and a
  dedicated `Table` consulted on every tick.

**Exit criterion.** A scenario with the attitude MPC active tracks a
slew reference and respects torque + rate limits; a scenario with the
translational MPC active demonstrates a thrust-cone-constrained
hover-trim against a perturbed reference. Solver `Solved` status is
required on every tick or the controller falls back to PID with an
FDIR bit.

**Validation evidence.** Unit test that the N-step solution reduces
to the single-step QP when N=1; property test for positive-definite
Hessian; determinism test that two runs of the MPC produce
bit-identical actuator commands.

**References.** Boyd, S. and Vandenberghe, L., *Convex Optimization*,
Cambridge 2004 (QP / SOCP formulation). Açıkmeşe, B. and Ploen, S.,
*Convex Programming Approach to Powered Descent Guidance for Mars
Landing*, JGCD 2007 (SOCP for cone-constrained powered descent —
academic reference; the Phase-5 implementation does not include
any operational landing target). Modern successive-convexification
references (informational; not implemented in 5.A.4):
Szmuk, M., Reynolds, T. P., and Açıkmeşe, B., *Successive
Convexification for 6-DoF Mars Rocket Powered Landing with
Free-Final-Time*, AIAA SciTech 2017 / JGCD; Reynolds, T. P. et al.,
*Successive Convexification for Powered Descent Guidance with
Time-Varying Mass Properties*, AIAA SciTech 2024 (NASA Human
Landing System support); Sequential Convex Programming for 6-DoF
Powered Descent (arXiv 2510.09610, 2024). University of Washington
ACL ships a continuous-time SCvx with state-triggered constraints
(CT-cSTC) at <https://github.com/UW-ACL/CT-cSTC>; OpenBMP cites
this as the academic SOTA for powered-descent guidance and treats
it as a stretch goal for Phase 5 / Phase 6 follow-on work, not as a
5.A.4 deliverable.

**Scope guardrail.** No real-world landing-target reference data;
the SOCP is exercised against synthetic scenarios only. The full
LCvxLD / SCvx / CT-cSTC powered-descent reproduction is left as a
downstream-user worked example, not an OpenBMP shipped scenario.

#### 5.A.5 — Control allocation with axis priority + saturation reporting

**Scope.** Replace the Phase 4.C semantic-channel / effector-id
mapping with a proper control allocation framework:

- `ControlAllocator` trait — accepts a desired body torque (and
  optionally body force) and the current effector-state vector,
  returns a per-effector deflection vector that respects per-effector
  rate / position limits.
- Two reference implementations:
  1. **Pseudo-inverse** allocator (Stevens & Lewis 2015 §3.5
     formulation; redistributes saturation evenly across effectors).
  2. **Prioritised redistributed allocator** with axis priority
     (roll over yaw over pitch by default; scenario-overridable);
     allocates highest-priority axis first, redistributes residual
     to lower-priority axes within remaining headroom (per Härkegård
     2002 academic formulation).
- The mixer publishes per-axis saturation flags + per-effector
  saturation flags so FDIR can isolate "axis X cannot be commanded
  because effector Y is saturated".

**Exit criterion.** A scenario with a degraded effector (e.g. a
jammed elevon) demonstrates the prioritised allocator preserving
high-priority axis authority; per-axis + per-effector saturation
flags appear on the actuator topic and on the FDIR `tripped_mask`.

**Validation evidence.** Unit + property tests for the pseudo-inverse
formula; property test that the prioritised allocator never violates
per-effector limits; closed-loop scenario test for graceful
degradation under one-effector failure.

**References.** Härkegård, O., *Efficient active set algorithms for
solving constrained least squares problems in aircraft control
allocation*, CDC 2002 (foundational SLS-AS). Bodson, M.,
*Evaluation of optimization methods for control allocation*, JGCD
2002. Stevens & Lewis 2015 §3.5 (pseudo-inverse baseline). Modern
SLS-AS optimisations (informational): community work in 2023–2024
reports ≈50 % computational reduction in SLS-AS while retaining
solution quality, plus practical fault-tolerant control allocation
for over-actuated platforms (e.g. coaxial dodecacopters); these
extend Härkegård but do not change the algorithmic shape OpenBMP
ships.

**Scope guardrail.** Allocators are a redistribution layer; they
are not a guidance law and never perform target-driven actuator
selection.

---

### Group B — Estimator and FDIR

#### 5.B.1 — Square-root UKF (full 15-state)

**Scope.** Replace the Phase 4.C 6-state classical UKF with a full
15-state square-root UKF (Van der Merwe & Wan 2001) for the same
state vector as the EKF (`position, velocity, attitude error, gyro
bias, accel bias`):

- Square-root form: propagate Cholesky factor `S` of covariance,
  apply Givens / Householder rotations on each measurement update,
  use `cholupdate` for rank-one downdates. No Cholesky refactorisation
  on the hot path.
- Sigma-point set: scaled symmetric set (Julier-Uhlmann); weights
  derived from `(α, β, κ)` per the Wan-Van der Merwe formulation.
- Measurement updates for IMU (propagation), GNSS (6-D pos+vel),
  baro (1-D altitude via USSA76), magnetometer (3-D body-frame field
  via WMM 2025 or `EarthDipoleField`).
- The classical Phase-4.C `Ukf` is retired in the same commit and
  replaced by `SquareRootUkf`. The 6-state attitude-only path
  remains as `SquareRootUkfAttitude` for users who only need
  attitude estimation.

**Exit criterion.** SR-UKF passes all EKF property tests
(determinism, innovation whitening through lag 10, long-duration
boundedness, textbook Kalman algebra). On the closed-loop
integration test the SR-UKF and EKF agree on attitude / position /
velocity within a tolerance documented in
`tests/expected/ukf-vs-ekf-tolerance.toml`.

**Validation evidence.** Unit + property tests; tolerance-table
case; determinism CI gate exercises an SR-UKF scenario.

**References.** Van der Merwe, R. and Wan, E. A., *The Square-Root
Unscented Kalman Filter for State and Parameter-Estimation*, IEEE
ICASSP 2001 (foundational SR-UKF). Wan, E. A. and Van der Merwe, R.,
*The Unscented Kalman Filter for Nonlinear Estimation*, AS-SPCC
2000. Numerically-stable variants for sequential measurement
updates: arXiv 2203.06105 *A summary on the UD Kalman Filter* (UDU
factorization, free of square-root operations); recent comparative
work (2024) on UD-UKF reports stronger numerical stability and
covariance-stability than SR-UKF on highly nonlinear tracking.
OpenBMP ships SR-UKF as the Phase-5 deliverable; UD-UKF is tracked
as an optional alternative under the same trait surface and may
land later if the SR-UKF proves numerically fragile in CI.

**Scope guardrail.** Same measurement set as the EKF; no plant-
specific tuning that hides operational vehicle data.

#### 5.B.2 — Multi-instance estimator routing + active-lane selection

**Scope.** Land the parallel-estimator-lane architecture sketched in
the Phase-4.C voter docstring:

- `EstimatorLane` — a registered estimator instance (EKF, MEKF,
  UKF, SR-UKF, IMM) addressed by a `LaneId`. Each lane runs its own
  predict / update on every tick.
- `EstimatorVoter` — selects the *active* lane per tick from the
  set of healthy lanes. Decision policy is configurable:
  `SimplexPassThrough`, `MidValueSelectByInnovation`, or
  `BestByCovarianceTrace`. The active lane drives the public
  `AttitudeEstimate` / `PositionEstimate` topics; non-active lanes
  publish on debug topics for the compare harness.
- `EstimatorRouter` — orchestrates lane execution; declares which
  bus topics each lane reads (allowing per-lane sensor lockout for
  graceful-degradation experiments).

**Exit criterion.** A scenario registering EKF + SR-UKF + MEKF as
parallel lanes runs deterministically; the voter switches the
active lane on a synthetic GNSS dropout and the autopilot consumes
the new active-lane output without a tick discontinuity.

**Validation evidence.** Unit + property tests for voter policies;
closed-loop scenario test for mid-flight lane switch; determinism
test for the multi-lane case.

**References.** PX4 ekf2 multi-instance routing pattern (academic
reference, not imported). Bar-Shalom et al. 2001 §11 (multi-model
approaches).

**Scope guardrail.** Lane selection is sensor-fusion focused; no
lane represents a "target tracker" or "homing filter".

#### 5.B.3 — IMM (Bar-Shalom) maneuver-aware estimator

**Scope.** Add an Interacting Multiple Model estimator as one of the
lanes in 5.B.2:

- `ImmEstimator` — N parallel sub-filters (e.g. constant-velocity,
  constant-acceleration, coordinated-turn) with a Markov mode
  transition matrix; each tick runs all sub-filters and combines
  their outputs by mode probabilities.
- Mode probabilities published on a debug topic; the active mode
  feeds an `EstimatorMode` topic that the autopilot may read for
  gain-schedule selection (academic study only — not a target-
  tracking enabler).
- Default IMM bank ships three kinematic models suitable for
  rocket-class trajectories (boost, coast, descent).

**Exit criterion.** The IMM scenario tracks a maneuvering trajectory
with a documented mode-probability profile; mode probabilities are
deterministic across reruns.

**Validation evidence.** Unit tests for the mode-mixing equations;
property test for probability simplex (`Σpᵢ = 1`); reproducibility
of the §11 textbook IMM example from Bar-Shalom et al. 2001 (added
in 5.E.4).

**References.** Bar-Shalom, Y., Li, X. R., and Kirubarajan, T.,
*Estimation with Applications to Tracking and Navigation*,
Wiley 2001 §11 (IMM derivation and example sets). Blom, H. A. P.
and Bar-Shalom, Y., *The interacting multiple model algorithm for
systems with Markovian switching coefficients*, IEEE TAC 1988
(foundational). Modern variable-structure IMM literature
(informational; not implemented in 5.B.3): MDPI Aerospace 2023
*Adaptive IMM-UKF for Airborne Tracking* (adaptive transition
probabilities driven by a distance function); IET Radar, Sonar &
Navigation 2023 *A variable structure multi-model maneuvering
target tracking algorithm based on Monte Carlo learning*; IJAE
2024 *Improved Variable Structure Interacting Multimodels for
Target Trajectory Tracking and Extrapolation*. OpenBMP ships the
classical fixed-bank IMM as the Phase-5 deliverable; VSIMM and
adaptive-transition variants are tracked as follow-on work, not
5.B.3 scope.

**Scope guardrail.** IMM is a maneuvering-target-tracking technique
in the original literature; OpenBMP uses it strictly for
self-state estimation under regime change (boost vs coast vs
descent), not for tracking other vehicles. No multi-target
extension lands in Phase 5.

#### 5.B.4 — Willsky windowed-mean-shift GLRT + parity-space residual

**Scope.** Two FDIR detector additions:

1. **Windowed mean-shift GLRT.** Replace the Phase 4.C
   `DetectorKind::SingleSampleGlrt` chi-square threshold with a
   true windowed Generalized Likelihood Ratio test:
   - Maintain a sliding window of innovations per measurement.
   - For each candidate jump time within the window, compute the
     maximum-likelihood mean-shift estimate and the GLRT statistic.
   - Trip the detector when `max_τ GLRT(τ) > threshold`.
   - Emit the estimated jump time and magnitude on
     `fdir.glrt.diagnostic` for offline analysis.
2. **Parity-space residual generator.** Layered Patton-Frank
   parity-space residual generator over the IMU + GNSS + baro
   measurement set. Per-fault residual decomposition isolates the
   faulty sensor without retraining the estimator on the fault
   hypothesis.

**Exit criterion.** A scenario injecting a step-bias fault on the
GNSS lane is detected by the windowed GLRT with the correct jump
time within window resolution; the parity-space residual isolates
the GNSS lane vs an injected baro-bias fault.

**Validation evidence.** Unit + property tests for the GLRT
recursion; tolerance-table case for the textbook fault-detection
example from Willsky 1976.

**References.** Willsky, A. S. and Jones, H. L., *A generalized
likelihood ratio approach to the detection and estimation of jumps
in linear systems*, IEEE TAC 1976. Patton, R. J. and Frank, P. M.,
*Parity-space approach to model-based fault detection and
isolation*, in *Issues of Fault Diagnosis for Dynamic Systems*,
Springer 2000.

**Scope guardrail.** FDIR responds to faults the scenario injects;
no tuning is calibrated to a specific fielded sensor's failure
modes.

---

### Group C — Environment

#### 5.C.1 — NRLMSISE-00 atmosphere

**Scope.** In-house Rust port of the NRLMSISE-00 empirical
atmosphere model (Picone et al. 2002). The port:

- Parses the public coefficient set from `data/atmosphere/NRLMSISE-00/`
  with a sibling `provenance.md` and SHA-256-pinned files in the
  scenario.
- Implements the public mathematical formulation only; no derived
  coefficient sets, no operational tunings.
- Exposes `NrlMsise00 : AtmosphereModel`, returning density,
  temperature, and per-species number densities (N2, O2, O, He,
  Ar, H, N) over the documented validity envelope (0–1000 km).
- Fails closed outside the validity envelope (no extrapolation by
  default; clamp / linear opt-in is rejected for NRLMSISE-00).
- Switches in via `[atmosphere].kind = "nrlmsise00"` plus
  `f10_7`, `f10_7_avg`, `ap_index` (scalar Ap), and an absolute
  epoch in TAI seconds.

**Exit criterion.** NRLMSISE-00 reproduces the published reference
profile from the original Picone et al. 2002 paper at the documented
test points within a tolerance recorded in
`tests/expected/nrlmsise00.toml`. The scenario fails closed at
1001 km altitude.

**Validation evidence.** Unit tests against the public reference
profiles; analytic-toy scenario where the rocket ascends through a
documented density profile and the kernel sees the expected drag.

**References.** Picone, J. M., Hedin, A. E., Drob, D. P., and
Aikin, A. C., *NRLMSISE-00 empirical model of the atmosphere:
Statistical comparison and scientific issues*, J. Geophys. Res.
107(A12), 2002.

**Scope guardrail.** NRLMSIS 2.x and HWM14 are Phase 6 follow-ons.
JB-2008 is Phase 6. Phase 5 ships only the 2002 NRLMSISE-00
baseline.

#### 5.C.2 — EGM2008 truncated spherical-harmonic gravity

**Scope.** Truncated spherical-harmonic gravity beyond WGS84-J2:

- `Egm2008Gravity : GravityModel`, parametrised by `(degree, order)`
  with public coefficients in `data/gravity/EGM2008/` (Pavlis et al.
  2012 release; SHA-256-pinned in the scenario).
- Maximum degree / order in the shipped scenario is **20**, the
  documented orbit-determination working envelope. Higher truncations
  may be opted into per scenario but are not exercised in CI by
  default.
- Implements the standard Cunningham 1970 recursion for the
  associated Legendre functions, evaluated in ECEF and rotated into
  ECI on demand.
- Switches in via `environment.gravity = "egm2008"` plus `degree`,
  `order`, and `coefficients_path`.

**Exit criterion.** EGM2008-degree-20 gravity reproduces a published
GPS-orbit-class trajectory within a tolerance recorded in
`tests/expected/egm2008.toml`. Scenarios opting into degree > 20
parse but emit a `validation = "experimental"` label until their
own tolerance evidence lands.

**Validation evidence.** Unit tests for the Cunningham recursion;
property test for spherical symmetry at the equator under
zonal-only truncation; analytic-toy comparison against the
J2-only kernel for `degree = 2`.

**References.** Pavlis, N. K., Holmes, S. A., Kenyon, S. C., and
Factor, J. K., *The development and evaluation of the Earth
Gravitational Model 2008 (EGM2008)*, J. Geophys. Res. 117(B4),
2012. Cunningham, L. E., *On the computation of the spherical
harmonic terms needed during the numerical integration of the
orbital motion of an artificial satellite*, Cel. Mech. 2(2), 1970.

**Scope guardrail.** Public coefficients only. No derived datasets,
no operational tunings, no satellite-specific calibrations.

---

### Group D — Kernel and scenario

#### 5.D.1 — Multi-rate scheduling first-class

**Scope.** Promote multi-rate scheduling from the Phase-3 sketch in
`software-architecture.md` to first-class scenario syntax:

- `[schedule]` block: `base_hz`, plus `[[schedule.group]]` entries
  with `label`, `hz`, and `members`. Each `hz` must divide
  `base_hz` exactly (loader rejects non-integer divisors).
- `RatePlan` resolves into a flat `Vec<Vec<SubsystemId>>` at
  scenario load. Each base tick walks the resolved plan;
  rate-decision branches do not appear in the hot loop.
- The simulator-local FC is one of the rate groups; the integrator
  always runs on the base rate.

**Exit criterion.** Multi-rate scenarios run deterministically; the
analytic-toy and Calisto scenarios become two-group multi-rate
configurations (env at base rate, FC at sub-rate) without changing
their golden output.

**Validation evidence.** Property test that the resolved plan length
divides `base_hz`; determinism CI gate exercises a multi-rate
scenario; tolerance-table check that bit-stable output matches the
single-rate variant.

**Scope guardrail.** Multi-rate is a determinism-preserving
optimisation, not a real-time-scheduling claim.

#### 5.D.2 — Multi-body simultaneous propagation

**Scope.** Promote post-separation propagation from the Phase-3
single-body "spent stage falls behind, ignored" hack to first-class
multi-body simulation:

- `SeparationEvent` with `SeparationTrigger`, `SeparationSplit`,
  `SeparationImpulse` (per the architecture sketch in
  `software-architecture.md`).
- After separation, both `VehicleAssembly` instances propagate with
  their own state vectors, force / moment / mass-model lists, and
  optionally their own `FlightController`. The kernel's environment
  sample is shared.
- Momentum-conservation check: `m_u·Δv_u + m_l·Δv_l ≈ 0` within a
  documented tolerance; violation fails closed.

**Exit criterion.** The `scenarios/multi-body/` folder contains a
two-stage separation scenario where the upper stage flies under
controller authority and the spent stage tumbles ballistically;
both bodies appear in telemetry as separate streams; momentum
conservation holds within tolerance.

**Validation evidence.** Property test for momentum conservation;
analytic-toy case where two equal-mass bodies separate symmetrically
in vacuum; determinism CI gate exercises the multi-body scenario.

**Scope guardrail.** Multi-body propagation is for academic
multi-stage / drop-test studies. No multi-target tracking, no
intercept geometry, no engagement scenarios.

#### 5.D.3 — DOPRI5/8 adaptive integrator (profile-flagged)

**Scope.** Add an adaptive Dormand-Prince 5(4) and 8(7) integrator
behind an explicit profile flag. The default simulation profile
remains RK4 fixed-step; bit-stable output is preserved on the
default profile.

- `Dopri54Adaptive`, `Dopri87Adaptive` integrators in `openbmp-sim`
  with PI step controller; documented step-size policy.
- Activated via `--profile=adaptive` on the CLI or
  `[simulation] profile = "adaptive"` in the scenario; mutually
  exclusive with the default profile.
- The adaptive profile is `state-stable, not bit-stable` across
  reruns by design — the determinism CI gate runs the adaptive
  profile under a separate state-stable rule.

**Exit criterion.** Adaptive integrators reproduce the analytic-toy
constant-acceleration drop within their declared tolerance; the
fixed-step default profile remains byte-identical across this
sub-phase.

**Validation evidence.** Unit tests for embedded error estimator;
analytic-toy state-stable check; tolerance-table case for the
torque-free Euler precession with adaptive vs fixed-step.

**Scope guardrail.** Adaptive profile is opt-in and labelled
`state-stable, not bit-stable`. The bit-stable default profile is
the one any release artifact is benchmarked against.

---

### Group E — HIL, replay, validation

#### 5.E.1 — Optional socket-bridge HIL pattern

**Scope.** Land the optional `openbmp-bridge` crate per the
architecture sketch:

- In-house `postcard`-encoded wire format (`BridgeSensorPacket`,
  `BridgeCommandPacket`).
- TCP and UDP transport options; both are simulator-local in CI.
- Reference Rust client library (`openbmp-bridge-client`) that
  connects, sends commands, receives sensor packets — used for
  CI integration tests only.

**Exit criterion.** A CI integration test runs the simulator on
one process and the reference client on another, exchanges 1 000
ticks of synthetic sensor packets and abstract command packets,
and verifies the simulator state matches the equivalent in-process
run within tolerance.

**Validation evidence.** Wire-format round-trip property test;
in-process / over-socket comparison test; documented schema
versioning.

**References.** Pattern borrowed from PX4's SITL socket interface
shape; OpenBMP ships only its own in-house schema, never any real
flight-stack protocol.

**Scope guardrail.** No MAVLink, no DDS, no MIL-STD-1553, no CAN,
no I²C, no SPI, no UART. Real device-driver integrations live in
downstream HAL adopter repositories under their own export-control
posture.

#### 5.E.2 — Real ULog parser + PX4 ekf2 cross-validation

**Scope.** Public-log replay validation:

- `openbmp-replay-ulog` — pure-Rust ULog 2.x parser. Parses the
  self-describing ULog header and decodes the EKF-relevant uORB
  topics (`sensor_combined`, `vehicle_gps_position`,
  `vehicle_attitude`, `vehicle_local_position`).
- `OpenBmpEkfReplayCase` — replays the parsed topic stream into
  OpenBMP's EKF (or SR-UKF) and compares the published
  `AttitudeEstimate` / `PositionEstimate` against the reference
  PX4 ekf2 state from the same log, within a documented tolerance.
- Initial public log: a hobby-class fixed-wing or quadrotor flight
  from the PX4 community log archive, sourced from the public
  archive index, with provenance recorded in the case's
  `provenance.md` (URL, retrieval date, sha256, validity-range
  notes). The PX4 Flight Review archive at <https://review.px4.io/>
  hosted ≈ 123 000 publicly-uploaded logs in 2024 (a 17× growth
  from 2023, per the Roboto.ai analysis); pinning a specific log
  by file id + sha256 fixes the cross-validation reference.

**Exit criterion.** The PX4 ekf2 cross-validation case runs in CI;
attitude residual stays within the tolerance envelope declared in
`tests/expected/px4-ekf2-replay-case.toml`.

**Validation evidence.** ULog parser fuzz target (panic on
malformed payloads is forbidden); cross-validation tolerance table.

**References.** PX4 ULog file format documentation
(<https://docs.px4.io/main/en/dev_log/ulog_file_format.html>); PX4
ekf2 module documentation; PX4 Flight Review web app
(<https://github.com/PX4/flight_review>).

**Scope guardrail.** Only public hobby / academic / community logs.
No operational logs, no logs from restricted vehicles, no logs from
real-fielded missile / UAV programmes.

#### 5.E.3 — ArduPilot dataflash parser + NavEKF3 cross-validation

**Scope.** Same shape as 5.E.2 but for ArduPilot:

- `openbmp-replay-dataflash` — pure-Rust ArduPilot binary
  dataflash parser, decoding the EKF1 / EKF2 / EKF3 / EKF4 log
  messages plus the underlying sensor messages.
- A second cross-validation case feeding the same OpenBMP estimator
  the parsed log and comparing against the NavEKF3 reference state
  with a documented tolerance.
- Public log sourced from the ArduPilot community archive.

**Exit criterion.** The ArduPilot NavEKF3 cross-validation case
runs in CI within tolerance.

**Validation evidence.** Parser fuzz; tolerance-table case.

**References.** ArduPilot dataflash log format documentation;
ArduPilot Replay tool reference.

**Scope guardrail.** Same as 5.E.2.

#### 5.E.4 — Bar-Shalom textbook §5.4 / §5.5 reproducibility cases

**Scope.** Reproduce specific textbook examples with their original
parameter sets:

- **§5.4 — Innovation consistency (NIS) tests.** A scenario
  exercising the published 1-D and 2-D Kalman example with the
  textbook's `(F, H, Q, R, P_0, x_0)` parameters; assert the
  Normalised Innovation Squared (NIS) statistic is consistent
  with the chi-square distribution at the textbook's confidence
  level over a documented Monte-Carlo envelope.
- **§5.5 — Innovation autocorrelation.** Same scenario, with the
  innovation-autocorrelation property tested up to lag 10 against
  the textbook's whiteness criterion.
- These reproduce the published examples (not just generic Kalman
  algebra identities, which were Phase 4 scope).

**Exit criterion.** The textbook NIS and autocorrelation cases pass
within their declared tolerances; the cases run in CI as part of the
`local_kalman_algebra.rs` test family.

**Validation evidence.** Tolerance-table cases pinned to the
textbook parameter sets.

**References.** Bar-Shalom et al. 2001 §5.4 (NIS consistency
testing) and §5.5 (innovation analysis).

**Scope guardrail.** Textbook parameter sets only; no real-vehicle
calibration data.

#### 5.E.5 — Full closed-loop long-duration soak

**Scope.** Promote the Phase-4.C closed-loop integration test (1 000
ticks) and the EKF-only 60-second test to a full-pipeline
long-duration soak:

- `tests/full_closed_loop_long_duration.rs` runs the Calisto-class
  rocket end-to-end (sensor ingest → SR-UKF → guidance → commander →
  three-loop autopilot with differential-flatness trajectory tracker
  → control allocation → mixer → kernel) for the full ascent
  duration (multiple minutes of sim time at 1 kHz base rate).
- The soak asserts: (1) zero scheduler overruns, (2) finite state
  through every tick, (3) FDIR `triggered_mask = 0` in the nominal
  scenario, (4) Parquet bit-identical across reruns, (5) bus-history
  bit-identical across reruns.

**Exit criterion.** The long-duration soak runs in CI nightly
(not every PR — duration budget) and passes deterministically over
≥ 10 reruns. Apogee falls within the Phase-3 Calisto envelope (±2 %
of 3 349 m AGL).

**Validation evidence.** Bit-stable Parquet over reruns; bus-
history bit-stable over reruns; tolerance-table check on apogee.

**Scope guardrail.** The soak runs against the same Calisto
scenario already in CI for cross-tool validation. No new operational
parameter set is introduced.

## Sub-phase ordering

```text
5.0 (pre-work, blocks everything)
  ├── 5.A.1 (flatness tracker) ──► 5.A.2 (L1 adaptive) ──► 5.A.3 (anti-windup, LQR, INDI)
  │                                                           │
  │                                                           ▼
  │                                                       5.A.4 (MPC) ──► 5.A.5 (allocation)
  │
  ├── 5.B.1 (SR-UKF) ──► 5.B.2 (multi-lane) ──► 5.B.3 (IMM) ──► 5.B.4 (GLRT + parity)
  │
  ├── 5.C.1 (NRLMSISE-00) ──► 5.C.2 (EGM2008)
  │
  ├── 5.D.1 (multi-rate) ──► 5.D.2 (multi-body) ──► 5.D.3 (DOPRI)
  │
  └── 5.E.1 (HIL bridge) ──► 5.E.2 (ULog/PX4) ──► 5.E.3 (dataflash/ArduPilot)
                                                       │
                                                       ▼
                                                   5.E.4 (Bar-Shalom) ──► 5.E.5 (long-duration soak)
```

A → B → C → D run in parallel; E sub-phases are gated by their
prerequisite sub-phases (5.E.5 is gated by 5.A.1 + 5.A.5 + 5.B.1).

## Risks and contingencies

- **Determinism regressions.** SR-UKF, EGM2008 Cunningham recursion,
  and Clarabel SOCP are floating-point-heavy; the determinism CI
  gate is the canary. If any sub-phase produces non-bit-stable
  output, gate the work behind a `state-stable` profile flag and
  document the diff before merge.
- **External-log availability.** If a chosen public log is removed
  from the upstream archive during Phase 5, the case is paused, the
  provenance entry retired, and a substitute public log is sourced.
  No backup-private-log path lands in the repo.
- **NRLMSISE-00 numerical envelope.** The 2002 Fortran reference
  has known numerical quirks at boundaries. The Rust port matches
  the public reference profile within tolerance; quirks outside the
  documented validity envelope fail closed rather than degrade
  silently.
- **Scope creep.** Hypersonic-flavored asks (NRLMSIS 2.x, real-gas,
  aerothermal) are Phase 6. If a sub-phase reviewer thinks Phase 5
  needs them, the discussion happens in the Phase-6 plan, not in a
  Phase-5 sub-phase.

## Documentation deliverables

Each sub-phase ships:

- A per-sub-phase commit (or commits) following the existing
  Phase-4 / Phase-3 commit-message style.
- Updates to the relevant per-crate `README.md` (purpose, inputs,
  units, frames, assumptions, validity range, determinism,
  validation, data provenance, safety boundary).
- Updates to `docs/scenario-format.md` for any new scenario block.
- A tolerance-table TOML in `tests/expected/<case>.toml` for any
  validation case.
- A `provenance.md` for any external dataset.

## Closing condition

Phase 5 closes with:

- All sub-phases above merged.
- The `Phase 4 — Flight controller` and `Phase 5 — Test harness
  expansion` headings in `docs/design-concept.md` rewritten to the
  closed Phase 4 / Phase 5 status.
- This `phase-5-plan.md` retired (committed as "Retire Phase 5
  planning + audit docs"), mirroring the Phase 3.x / Phase 4 retire
  pattern.
