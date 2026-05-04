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
- Environment: piecewise-exponential layered atmosphere (Vallado
  Table 8-4 fit, 0-1000 km; full NRLMSISE-00 deferred);
  EGM2008 zonal-harmonic gravity (degrees 2-6) beyond WGS84-J2
  (tesseral / sectoral terms deferred to a follow-on slice).
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
| 5.A.3.C — per-axis INDI rate loop (Smeur-Chu-de Croon 2016) | shipped | `c06d881` |
| 5.A.3.D — controller comparison harness | shipped | `81d2407` |
| 5.A.4 — receding-horizon attitude MPC (Clarabel-backed) | shipped | _pending PR_ |
| 5.A.5 — prioritised redistributed control allocator | shipped | _pending PR_ |
| 5.C.2 — EGM2008 zonal-harmonic gravity (degrees 2-6) | shipped | _pending PR_ |
| 5.C.1 — piecewise-exponential atmosphere (0-1000 km) | shipped | _pending PR_ |
| 5.D.3 — DOPRI5 fixed-step integrator (5th-order solution) | shipped | _pending PR_ |
| 5.B.4 — Willsky windowed-mean-shift GLRT (vector-form) | shipped | _pending PR_ |
| 5.B.3 — IMM (Bar-Shalom) maneuver-aware estimator (2-mode bank) | shipped | _pending PR_ |
| 5.B.1.A — SR-UKF math primitives (sigma points, cholupdate, QR predict) | shipped | _pending PR_ |
| 5.B.1.B/C — SquareRootUkf 15-state filter + 6-state attitude variant + classical `Ukf` retirement | shipped | _pending PR_ |
| 5.B.2 — multi-instance estimator routing + active-lane selection | pending | — |
| 5.B.5 — Patton-Frank parity-space residual generator | pending — follow-on from 5.B.4 | — |
| 5.B.6 — IMM extensions (3-mode bank + lane integration + UKF/MEKF) | pending — follow-on from 5.B.3 | — |
| 5.C.3 — EGM2008 tesseral / sectoral expansion (Cunningham recursion) | pending — follow-on from 5.C.2 | — |
| 5.C.4 — NRLMSISE-00 full Rust port (solar-flux, per-species) | pending — follow-on from 5.C.1 | — |
| 5.D.1 — multi-rate scheduling first-class | pending | — |
| 5.D.2 — multi-body simultaneous propagation | pending | — |
| 5.D.4 — DOPRI5(4) adaptive integrator with PI step controller (point-mass runner) | shipped | _pending PR_ |
| 5.D.5 — adaptive integrator: rigid-body runner + per-component error norm + PI band test | shipped | _pending PR_ |
| 5.D.6 — DOP853 8(5,3) integrator (fixed-step + adaptive variants) | shipped | _pending PR_ |
| 5.E.1 — optional socket-bridge HIL pattern | pending | — |
| 5.E.2 — real ULog parser + PX4 ekf2 cross-validation | pending | — |
| 5.E.3 — ArduPilot dataflash parser + NavEKF3 cross-validation | pending | — |
| 5.E.4 — Bar-Shalom textbook §5.4 / §5.5 reproducibility cases | pending | — |
| 5.E.5 — full closed-loop long-duration soak | pending | — |

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
deliberately deferred plus the comparison harness that closes
Phase 5.A.3. Phase 5.A.3.A through 5.A.3.D have shipped and follow
the same per-slice review pattern as 5.A.1.A–D / 5.A.2.A–D.

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
3. **INDI baseline (Phase 5.A.3.C — shipped).** Per-axis INDI rate
   loop gated behind a new `indi` Cargo feature. Inverts the local
   incremental relationship `J · Δω̇ ≈ G_eff · Δu` per axis under
   the same single-body / diagonal-inertia precondition LQR ships
   with. The synchronised `ω` and `u` filters share a single cutoff
   exposed in the scenario API (first-order or second-order
   Butterworth biquad via the bilinear transform; Smeur 2016
   default is the second-order shape). Implicit anti-windup via the
   clamp `u[k+1] = clamp(u[k] + Δu)` — no separate integrator state.
   New scenario fields: `[fc.autopilot_params.rate_loop_kind] =
   "indi"` selects the loop; `[fc.autopilot_params.indi]` declares
   the working inertia / control-effectiveness / filter cutoff /
   outer-loop attitude P-gain. Composition with
   `[fc.autopilot_params.l1_adaptive]` is rejected at scenario load
   (filter-interaction concerns). Demonstration scenario
   `scenarios/diff-flatness-figure-eight-indi/scenario.toml`.
4. **Controller comparison harness (Phase 5.A.3.D — shipped).**
   Runs the figure-eight scenario family across PID baseline +
   PID + L1 + LQR + INDI under one common matched roll-axis
   `EffectorFault::ReducedRate { factor = 0.7 }` disturbance and
   one shared deterministic synthetic-sensor seed.
   `crates/openbmp-cli/tests/controller_comparison_harness.rs`
   computes per-scenario max `|ω|`, RMS `|ω|`, per-axis peak
   torque, and saturation fraction over the post-liftoff window;
   emits a markdown table to `tests/expected/controller-
   comparison.md`. Asserts byte-equality with that snapshot
   (regenerable via `UPDATE_EXPECT=1 cargo test ...`) plus a
   fixture-independent sanity gate that L1 beats PID baseline by
   ≥ 2× under the same fault and every rate loop keeps `|ω|`
   bounded. The table is a documented operating point, not a
   best-vs-best controller ranking. Two new sibling scenarios:
   `scenarios/diff-flatness-figure-eight-lqr-fault` and
   `scenarios/diff-flatness-figure-eight-indi-fault`.

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
MPC. **Phase 5.A.4 ships the attitude path** under the existing
`mpc` Cargo feature; the translational path stays scoped for a
future slice.

- `RecedingHorizonAttitudeMpc` (shipped) — condensed N-step QP per
  body axis (block-diagonal joint formulation across the three
  axes), small-angle Forward-Euler dynamics
  `x[k+1] = x[k] − dt · u[k]`, quadratic stage + terminal cost
  with per-axis weights, and a symmetric box constraint on the
  commanded body rate (`|u[k]| ≤ rate_limit_rad_s[axis]`).
  Decision variable layout `[u_x[0], u_y[0], u_z[0], u_x[1], …]`
  (3 · N elements). The Hessian and constraint matrices are
  pre-built at construction time; only the linear cost vector
  depends on the current attitude error and is rebuilt each
  `solve`.
- `RecedingHorizonTranslationalMpc` (deferred) — N-step SOCP over a
  position / velocity state with thrust-direction (cone) constraint
  and thrust-magnitude bound. Deferred to a follow-up slice; the
  Phase-4.C `solve_accel_norm_epigraph` cone backend is already in
  `landing.rs` and will be promoted there.
- Clarabel solver settings stay deterministic (`verbose = false`,
  pinned tolerances, no time-limit) — shared with Phase 4.C
  `solve_attitude_box_qp`. Two solves with the same inputs produce
  bit-identical outputs on the reference platform.
- The shipped attitude MPC is a command-level controller. Its prediction
  model is the small-angle attitude-error integrator into commanded body
  rate; it does not model the downstream PID/LQR/INDI rate-loop dynamics,
  actuator saturation, or reference-attitude motion across the horizon.
  The Phase-5.A.4 demo therefore proves deterministic solver wiring and
  bounded closed-loop simulation, not production cascaded-loop optimality.
  A later slice should augment the prediction model with the rate-loop /
  actuator dynamics or add explicit flatness body-rate feed-forward before
  using MPC tuning claims as control-performance evidence.
- New `AttitudeLoopKind { Pid, Mpc }` enum on `AutopilotParams`,
  selected by the v3 scenario block
  `[fc.autopilot_params.attitude_loop_kind]`. The MPC parameter
  block lives at `[fc.autopilot_params.attitude_mpc]` with v3-only
  gating, cross-validation, and rejection of non-positive
  parameters at scenario load.
- `attitude_loop_kind = "mpc"` is intentionally independent of
  `rate_loop_kind`: MPC selects the attitude-loop producer, while
  PID/LQR/INDI select the downstream rate-loop consumer. Existing
  parser restrictions still apply, including the Phase-5.A.3
  rejection of `rate_loop_kind = "indi"` combined with L1 adaptive
  augmentation.

**Exit criterion (5.A.4 attitude MPC).** A scenario with the attitude
MPC active runs the figure-eight reference end-to-end with bounded
`|ω|`, normalised quaternion, and byte-stable Parquet across reruns.
Demonstration scenario:
`scenarios/diff-flatness-figure-eight-mpc/scenario.toml`. E2E test:
`crates/openbmp-cli/tests/diff_flatness_mpc_e2e.rs`. Solver
`Solved` status is required on every tick; failure surfaces as an
`AutopilotError::Trajectory` and the controller refuses to step.
This strict refusal is deliberate for the deterministic simulator. A
real-time deployment profile should add an explicit fallback path, likely
with an FDIR bit, rather than silently continuing with stale MPC output.

**Runtime note.** The current implementation constructs a fresh Clarabel
solver for every attitude-MPC solve and updates only the linear vector
before construction. That is acceptable for the offline simulator and
determinism tests, but it should not be read as a 1 kHz real-time budget
claim. Future optimisation should investigate solver reuse / in-place
`q` updates or a slower scheduled MPC period if Clarabel's public API
allows it.

**Translational MPC exit criterion (deferred).** A scenario with
the translational MPC active demonstrates a thrust-cone-constrained
hover-trim against a perturbed reference. Solver `Solved` status
required on every tick or the controller falls back to PID with an
FDIR bit.

**Validation evidence.** Unit tests cover parameter validation, box-bound
enforcement, command sign under the documented error dynamics, zero-error
behaviour, closed-loop convergence of the ideal small-angle plant, and
bit-identical repeated solves. The e2e test covers bounded figure-eight
simulation and byte-identical Parquet across reruns. N=1 reduction and
positive-definite-Hessian property tests remain useful follow-up coverage.

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

**Scope.** Replace the Phase-4 1:1 semantic-channel / effector-id
mapping with a proper control-allocation framework. Phase 5.A.5
ships the **single-axis-effector case**:

- `PrioritisedRedistributedAllocator` (consumed) —
  `crates/openbmp-fc/src/allocation.rs`. Each `direct_torque`
  effector contributes to exactly one body axis; the allocator
  groups effectors by axis and distributes the autopilot's per-axis
  torque demand proportional to per-effector capacity. Phase
  authority is applied before the capacity calculation: disallowed
  effectors receive explicit zero commands and do not contribute to
  `Σ Lᵢ`. When `|τ_a| ≤ Σ Lᵢ` the split is exact and proportional;
  when demand exceeds the allowed axis capacity, every allowed
  effector pulls at its limit and the axis is reported as saturated.
- Pseudo-inverse allocator (parsed but **not** yet consumed) —
  `FcAutopilotAllocationKind::PseudoInverse` is rejected by the
  runner with an `UnsupportedScenario` error until a future slice
  ships the general `G_eff` path. The current scope stays
  rocket-class-honest: `direct_torque` effectors do not couple
  across axes, so the pseudo-inverse degenerates to the
  prioritised-redistributed result and shipping it now would be
  pure surface area.
- Mixer wiring — `Mixer::with_allocator(allocator)` supersedes the
  legacy channel-map dispatch when set. The `EffectorCommandSet`
  publish path uses the allocator's per-effector commands. The legacy
  channel map continues to gate the semantic `ActuatorCommand` topic
  for downstream consumers, but it does not gate the allocator's
  raw per-axis demand.
- Per-axis saturation flags propagate through `EffectorCommandSet.
  saturated`. Per-effector saturation flags are deferred to a
  future slice that adds the general G_eff matrix surface.
- Scenario surface — `[fc.autopilot_allocation]` (already declared
  in Phase 5.0, parser-only) is now consumed: `kind +
  axis_priority` plus the per-effector axis declarations from
  `vehicle.assembly.effectors` build the allocator at scenario
  load.

**Exit criterion (5.A.5 attitude-allocation slice).** An
over-actuated scenario (`scenarios/diff-flatness-figure-eight-allocator/`,
two ±0.2 N·m roll-torque effectors plus single pitch / yaw)
runs the figure-eight reference end-to-end with both roll
effectors receiving bit-identical commands every tick (exact
proportional split for equal capacities); byte-stable Parquet
across reruns. E2E test:
`crates/openbmp-cli/tests/diff_flatness_allocator_e2e.rs`.

**Deferred (future slice).** Coupled-effector G_eff matrix +
pseudo-inverse path; per-effector saturation flags; engine-cluster
TVC allocation (gimbal vectors per engine producing mixed body
moments).

**Validation evidence.** Unit tests cover constructor validation,
proportional split, unequal-capacity split, sign handling, saturation,
determinism, priority-order invariance for single-axis effectors, and
phase-authority capacity exclusion. The e2e allocator scenario asserts
run-to-completion, exact equal-capacity roll split, and byte-stable
Parquet across reruns. Pseudo-inverse and coupled-effector property tests
remain future-slice work.

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

#### 5.B.1 — Square-root UKF (full 15-state) — **shipping in slices**

**Slice structure.** The originally-planned single-commit
"replace classical Ukf with SquareRootUkf" landed as too large for
a single audit cycle (sigma-point math + Cholesky primitives +
15-state error-state filter + 4 sensor measurement updates +
6-state attitude variant + classical-Ukf retirement is ~2000 LoC
of new code). Sub-divided into:

- **5.B.1.A — SR-UKF math primitives** (shipped):
  sigma-point generator, `cholupdate` rank-1 update / downdate,
  Householder-QR predict-side Cholesky combiner. Pure
  mathematical foundation in `crates/openbmp-fc/src/sr_ukf.rs`.
- **5.B.1.B/C — `SquareRootUkf` 15-state filter +
  `SquareRootUkfAttitude` 6-state variant + classical `Ukf`
  retirement** (shipped): the full error-state filter built on
  the 5.B.1.A primitives. Predict step (linearised square-root
  covariance propagation via QR of `F · S || √Q`, fully nonlinear
  sigma-point predict tracked as an internal follow-on note) +
  sigma-point GNSS / baro / mag measurement updates with
  cholupdate-based covariance reduction + the full
  `Estimator` trait impl. The 6-state attitude variant wraps the
  15-state filter with position / velocity / accel-bias slots
  re-pinned at `1e-12` covariance floors after covariance-changing
  operations; GNSS / baro updates are no-ops on the attitude variant.
  The Phase-4.C classical
  6-state `Ukf` and `UkfParams` are **deleted** along with
  their internal helpers (`unscented_square_moments`,
  `accumulate_ukf_covariance`, `accumulate_ukf_measurement_covariance`,
  `sigma_from_vector`, `quaternion_error_vector`, `mod
  ukf_tests`); ~520 lines retired from `estimator.rs`.

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

**Exit criterion.** SR-UKF passes the Phase-5.B.1 pre-push property
surface: deterministic predict replay, sigma-point round-trip
invariants, `cholupdate` update / downdate covariance recovery,
QR covariance recovery, sensor-update smoke tests, GNSS innovation
whitening through lag 10, direct sixty-second covariance boundedness,
textbook scalar Kalman algebra on the linear GNSS update, attitude-only
predict boundedness, and an EKF-vs-SR-UKF synthetic sensor-trajectory
agreement check whose tolerances are documented in
`crates/openbmp-fc/tests/expected/ukf-vs-ekf-tolerance.toml`.

**Validation evidence.** Unit + property tests; tolerance-table
case; SR-UKF within-platform predict determinism test. No scenario
selects SR-UKF yet, so existing scenario determinism gates remain
byte-identical coverage for the default code path rather than a
public scenario selector for this filter.

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

#### 5.B.3 — IMM (Bar-Shalom) maneuver-aware estimator **— shipped**

**Honest scope.** The shipped surface is a 2-mode-default
Bar-Shalom IMM hardcoded over a `Vec<Ekf>` bank with a compile-time
cap of `MAX_IMM_MODES = 4`. The original plan called for a 3-mode
boost / coast / descent canonical bank with regime-tuning evidence
and integration as a lane under 5.B.2's multi-instance routing;
both are deferred to a new **§ 5.B.6** follow-on entry. The
shipped 2-mode bank is the textbook minimum demonstration and
ships at full algorithmic quality (mixing, per-mode predict /
update, log-sum-exp mode-probability normalisation, fused output
state).

**What shipped.**

- `openbmp_fc::imm::ImmEstimator` — `Vec<Ekf>` bank of `2 ≤ N ≤ 4`
  mode-conditioned sub-filters. Implements the
  `openbmp_fc::estimator::Estimator` trait so it slots into
  `EstimatorJob` unchanged.
  - `mix()`: computes predicted mode probabilities
    `c̄_j = Σ_i Π_ij μ_i` and mixing weights
    `μ_ij = Π_ij μ_i / c̄_j`, blends per-mode priors via the
    spread-term covariance formula, and re-initialises each
    sub-filter with the mixed prior.
  - `update_*`: each sub-filter independently runs the call;
    captures per-mode `chi2_j` and `log det S_j` to form a
    Gaussian log-likelihood `log Λ_j = −0.5 (chi2_j + d log 2π +
    log det S_j)`.
  - `update_mode_probabilities()`: log-sum-exp normaliser over
    the current prediction prior (`c̄_j` after `mix()`, or the
    initial prior before the first predict) and `log Λ_j`.
    Numerical hygiene re-normalises to
    exactly `Σ μ_j = 1`.
  - `fused_position()` / `fused_attitude()`: probability-weighted
    mean of per-mode outputs, with quaternion renormalisation.
- EKF surface extension supporting the IMM:
  - `Ekf::last_log_det_s_gnss / _baro / _mag` getters return
    `log det S = 2 · Σ log L_diag` from the same Cholesky
    factorisation already done for the chi-square statistic. Reset
    to `f64::NAN` in `begin_tick`.
  - `Ekf::internal_state()` / `Ekf::set_internal_state(...)`
    round-trip pair: snapshot all 5 sub-vectors (pos, vel, q,
    gyro_bias, accel_bias) plus the full 15×15 covariance, then
    write them back. Quaternion is renormalised on writeback.
- New `EstimatorMode` topic
  (`crates/openbmp-fc/src/topics.rs`): published every tick by the
  IMM, carrying `(active_mode: u8, mode_probabilities: [f64; 4],
  mode_count: u8)`. Idle when the selected estimator is
  `Ekf` / `Mekf`.
- Scenario plumbing:
  `FcEstimatorKind::{Ekf, Mekf, Imm}`. New v3-only `[fc.imm]`
  block carrying `transition_matrix`,
  `initial_mode_probabilities`, and `[[fc.imm.mode]]` per-mode
  EKF tuning overrides on top of the base `[fc.ekf]`. Validator
  rejects malformed transition matrices (rows that don't sum to
  1 within `1e-9`, square-shape violations, out-of-range entries),
  malformed initial probabilities, and mode-count mismatches.

**What was deferred to § 5.B.6.**

- 3-mode boost / coast / descent canonical bank with regime-tuning
  evidence and a flight-phase-aware scenario.
- `EstimatorMode`-driven autopilot gain-schedule selection.
- IMM as a lane under 5.B.2's multi-instance routing.
- IMM over UKF / MEKF (would require generic-over-`Estimator`
  refactor with `set_state_from_mixed` trait method).
- Variable-Structure IMM (VSIMM) and adaptive-transition variants.
- `b̂(τ̂)`-style signed bias-magnitude estimate on `EstimatorMode`.

**Exit criterion.** A closed-loop attitude-hold scenario with the
new estimator wired in completes 1000 RK4 steps deterministically;
two reruns produce byte-identical Parquet. Mode-mixing,
probability-simplex, and likelihood-driven probability evolution
are exercised at the math layer.

**Validation evidence.**

- Math: `crates/openbmp-fc/src/imm.rs` ships 13 unit tests:
  constructor validation (mode count, transition-matrix row sum,
  initial-probability sum); probability-simplex invariant after
  measurement updates; fused position is the weighted mean of
  per-mode positions; byte-stable determinism across two IMM
  instances fed identical streams; `EstimatorMode` topic
  zero-padding and estimator-job publication; prediction-only
  Markov-transition evolution; gate-rejected measurements recording
  current likelihoods; antipodal-quaternion fallback; log-sum-exp
  numerical-stability under `NEG_INFINITY` entries; and
  mode-probability evolution under synthetic likelihood separation.
- EKF invariants: 3 unit tests in
  `crates/openbmp-fc/src/estimator.rs` covering
  `log det S = 2 · Σ log L_diag` reconstruction, per-sensor
  reset in `begin_tick`, and bit-exact `internal_state` round-trip.
- Scenario validator: 5 unit tests in
  `crates/openbmp-scenario/src/scenario.rs` covering v3 happy-path
  acceptance, v2 schema rejection, transition-matrix-row-sum
  rejection, initial-probability-sum rejection, and mode-count
  mismatch.
- End-to-end:
  `crates/openbmp-cli/tests/closed_loop_imm_e2e.rs` — 1000 RK4
  steps with end-time stop; byte-identical Parquet across two
  reruns.

**References.** Bar-Shalom, Y., Kirubarajan, T., and Li, X. R.
(2001). *Estimation with Applications to Tracking and
Navigation*, Wiley §11.6 — primary mathematical source. Blom,
H. A. P. and Bar-Shalom, Y. (1988). *The interacting multiple
model algorithm for systems with Markovian switching
coefficients*, IEEE Transactions on Automatic Control 33(8),
780-783 — foundational paper.

**Scope guardrail.** IMM is a maneuvering-target-tracking
technique in the original literature; OpenBMP uses it strictly for
self-state estimation under regime change. No multi-target
extension lands in Phase 5. The shipped 2-mode default and the
deferred 3-mode boost / coast / descent canonical bank both target
self-state-estimation use cases only.

#### 5.B.6 — IMM extensions: 3-mode boost/coast/descent bank + lane integration (follow-on from 5.B.3)

**Scope.** Pick up the IMM surfaces that § 5.B.3 deferred:

- 3-mode canonical bank with regime-tuning evidence: per-mode
  `EkfParams` calibrated for boost (high accel-bias process noise
  from engine vibration), coast (low Q, ballistic), and descent
  (intermediate Q with reentry-style aero unmodelled-dynamics
  margin). Includes a flight-phase-aware demo scenario where the
  mode-probability profile tracks the actual mission-graph phase.
- IMM-as-lane integration with 5.B.2's `[fc.estimator_lanes]`
  voter — the IMM bank becomes a single lane that competes with
  EKF and MEKF lanes via the existing voter policies.
- IMM over UKF / MEKF: refactor to generic-over-`Estimator` so
  the bank can mix attitude-only filters or non-EKF state
  representations. Requires a `set_state_from_mixed` trait
  method.
- `EstimatorMode`-driven autopilot gain-schedule selection: the
  `gain_schedule` block is currently keyed on
  `mission.phases.<name>` paths. § 5.B.6 adds an alternative key
  on the IMM active-mode index for academic-study scenarios where
  the autopilot retunes by estimated regime rather than mission
  phase.
- Optional: signed bias-magnitude `b̂(τ̂)` on `EstimatorMode` (the
  running sum is already computed inside the mixing step but is
  not currently surfaced).

**Exit criterion.** A 3-mode boost / coast / descent demo
scenario shows the mode probability tracking the actual mission
phase within a documented lag bound, and the IMM lane integrates
cleanly with 5.B.2's voter when both ship.

**Validation evidence.** Per-regime tuning evidence (residual
distributions per mode under matched / mismatched conditions);
analytic case where mode probabilities track a known regime
schedule.

**References.** Bar-Shalom, Y., Kirubarajan, T., and Li, X. R.
(2001). *Estimation with Applications to Tracking and Navigation*,
Wiley §11. The textbook §11.6 worked example is also picked up by
§ 5.E.4 (Bar-Shalom textbook reproducibility).

**Scope guardrail.** Self-state-estimation under regime change
only. No multi-target tracking, no maneuvering-target-tracking
literature use cases.

#### 5.B.4 — Willsky windowed-mean-shift GLRT (vector-form) **— shipped**

**Honest scope.** The shipped surface is the **vector-form** Willsky
1976 windowed-mean-shift GLRT, running on the per-sensor whitened
innovations `ν̃ = L⁻¹ ν` that the EKF / UKF / MEKF now export on
[`EstimatorStatus`][estimator-status]. The Patton-Frank parity-space
residual generator originally bundled with this slice is deferred
to a separate **§ 5.B.5** sub-phase — parity-space is a
mathematically distinct algorithm (linear combinations of
measurements that are zero under H₀, with per-fault residual
signature decomposition), not a refinement of the windowed GLRT.

[estimator-status]: ../crates/openbmp-fc/src/topics.rs

**What shipped.**

- `openbmp_fc::glrt::WindowedMeanShiftGlrt<const D: usize>` — math
  module implementing the test for a single sensor lane with
  innovation dimension `D`. Maintains a circular buffer of the most
  recent `window_size` whitened innovation vectors; on each
  `step()`, computes `Λ(τ) = ‖Σ ν̃_i‖² / N_τ` for every candidate
  `τ` in the window via a running-sum trick, then trips when
  `max_τ Λ(τ) > χ²⁻¹(1 − α/W, D)` (Bonferroni-corrected over the
  `W` candidate jump times).
- New `DetectorKind::WindowedMeanShiftGlrt` variant in
  `openbmp_fc::fdir`. The `FdirJob` constructs three per-sensor
  detectors (`GnssDim = 6`, `BaroDim = 1`, `MagDim = 3`) when this
  kind is selected. On a trip, sets the matching `FDIR_BIT_*` and
  publishes a new `FdirGlrtDiagnostic` topic with the estimated
  jump step, statistic, and threshold.
- EKF / UKF / MEKF extended with whitened-innovation export.
  `update_gnss` / `update_baro` / `update_mag` Cholesky-factor the
  innovation covariance `S = LLᵀ` and store `ν̃ = L⁻¹ ν`. Five
  invariant unit tests pin `‖ν̃‖² = chi2` per sensor and
  bit-stability across two EKF instances fed the same measurement.
- Scenario plumbing: `[fc.fdir.detector].kind` promoted from a
  free-form `String` (parser-only since Phase 5.0) to a typed enum
  `FcFdirDetectorKindV5::WindowedMeanShiftGlrt`. New
  `false_alarm_rate` field with default `0.001`. The validator
  rejects missing `window_samples`, out-of-range `false_alarm_rate`,
  and any leftover `parity_threshold` (which belongs to § 5.B.5).
- FC bridge (`build_fdir_params` in
  `crates/openbmp-cli/src/runner/fc.rs`) consumes the new typed
  block and overrides the legacy `detector_kind` when
  `[fc.fdir.detector]` is present.

**What was deferred to § 5.B.5.**

- Patton-Frank parity-space residual generator (separate algorithm).
- Estimated bias signature `b̂(τ̂)` exposed on `FdirGlrtDiagnostic`
  (the running sum required is computed but not surfaced; the
  shipped diagnostic carries only `(estimated_jump_step, statistic,
  threshold)`).
- Multi-sensor cross-lane fault isolation (the shipped detector
  reports per-sensor trips independently).

**Exit criterion.** A closed-loop attitude-hold scenario with the
new detector wired in completes 1000 RK4 steps deterministically;
two reruns produce byte-identical Parquet.

A 4σ synthetic step-injection unit test in `glrt.rs` proves the
detector trips within ±2 samples of the true jump time τ — the
scenario layer cannot easily inject a sensor bias step (no
scenario-syntax fault-injection block; deferred), so the trip-time
accuracy and no-false-trip claims live at the math layer rather
than the e2e layer.

**Validation evidence.**

- Math: `crates/openbmp-fc/src/glrt.rs` ships 9 unit tests:
  constructor validation (window_size = 0, false_alarm_rate
  ∉ (0, 1)); empty-step / NotTripped after construction; pure-H₀
  no-trip over 200 i.i.d. unit-Gaussian samples; 4σ step-injection
  trips within ±2 samples; byte-stable determinism across two
  detector instances fed the same stream; reset clears state;
  scalar-D = 1 specialisation; single-sample window collapses to
  the single-sample chi-square test.
- EKF invariants: 5 unit tests in
  `crates/openbmp-fc/src/estimator.rs` covering `‖ν̃‖² = chi2`
  for GNSS / baro / mag, `begin_tick` clearing the slots, and
  bit-stable whitening across two instances.
- End-to-end:
  `crates/openbmp-cli/tests/closed_loop_fdir_glrt_e2e.rs` — 1000
  RK4 steps with end-time stop; byte-identical Parquet across two
  reruns.
- Scenario validator: 4 new tests in
  `crates/openbmp-scenario/src/scenario.rs` covering v3-only
  acceptance, v3 validation under valid params, rejection of
  `parity_threshold` mixed with `windowed_mean_shift_glrt`, missing
  `window_samples`, and out-of-range `false_alarm_rate`.

**References.** Willsky, A. S. and Jones, H. L. (1976). *A
generalized likelihood ratio approach to the detection and
estimation of jumps in linear systems*, IEEE Transactions on
Automatic Control 21(1), 108-112 — primary mathematical source.
Hamilton, J. D. (1994). *Time Series Analysis*, §9.4 — textbook
reformulation for Gaussian innovation sequences.

**Scope guardrail.** FDIR responds to faults via the standard
chi-square / GLRT family. No tuning is calibrated to a specific
fielded sensor's failure modes; the shipped 32-sample window and
α = 0.001 are textbook engineering defaults.

#### 5.B.5 — Patton-Frank parity-space residual generator (follow-on from 5.B.4)

**Scope.** Pick up the parity-space surface that § 5.B.4 deferred:

- Layered Patton-Frank parity-space residual generator over the
  IMU + GNSS + baro + mag measurement set. Each sensor's
  measurement matrix `H_s` projects the (joint) state into its
  measurement; parity vectors are the orthogonal-complement
  combinations that are zero under H₀.
- Per-fault residual signature decomposition: each fault hypothesis
  (sensor bias, sensor stuck, sensor dropout) produces a different
  parity-vector signature; signature matching isolates the faulty
  sensor without retraining the estimator on each hypothesis.
- New `DetectorKind::ParitySpace` variant; consumes the
  `parity_threshold` field already reserved on
  `[fc.fdir.detector]`.
- Optional: signed bias-magnitude `b̂(τ̂)` on
  `FdirGlrtDiagnostic` (the running sum required already exists
  inside `WindowedMeanShiftGlrt::step` but is not currently
  surfaced — § 5.B.5 can expose it for cross-validation against the
  parity-space estimate).

**Exit criterion.** A scenario injecting a synthetic step-bias on
one sensor lane (GNSS or baro) is correctly isolated by the
parity-space generator vs the alternative-sensor hypothesis. The
windowed GLRT detector continues to trip in the same scenario; the
parity-space generator adds **isolation**, not detection.

**Validation evidence.** Unit tests for the parity-vector
construction (orthogonality `H_s' P = 0`); analytic-toy two-sensor
isolation case where one of two redundant lanes carries the fault.

**References.** Patton, R. J. and Frank, P. M. (2000).
*Parity-space approach to model-based fault detection and
isolation*, in *Issues of Fault Diagnosis for Dynamic Systems*,
Springer.

**Scope guardrail.** Parity-space requires a sensor-fault
injection block in the scenario format (deferred from § 5.B.4 for
the same reason: needs scenario-syntax design work). § 5.B.5 should
ship that block as part of its own scope or coordinate with a
separate scenario-format slice.

---

### Group C — Environment

#### 5.C.1 — Piecewise-exponential atmosphere (0-1000 km) **— shipped**

**Honest scope.** The shipped surface is a **layered exponential**
atmosphere, **not** NRLMSISE-00. The full NRLMSISE-00 model
(Picone et al. 2002) requires solar-flux dependence (F10.7,
F10.7-avg, Ap), per-species number densities, and an absolute epoch
in TAI seconds — its parameter table runs into thousands of
coefficients that this slice cannot pin authoritatively. The
layered-exponential approximation captures the **altitude-dominant**
variation that determines orbital drag and is the standard
engineering atmosphere used in orbital-mechanics textbooks.

**What shipped.**

- `openbmp_physics::PiecewiseExponentialAtmosphere : AtmosphereModel`
  with a 14-layer table covering 0-1000 km. Within each layer
  `ρ(h) = ρ_base · exp(−(h − h_base) / H_layer)`; each layer is
  reported with a scale-height-effective temperature
  `T_layer = M_air · g_0 · H_layer / R`, which keeps the
  density / pressure / temperature / speed-of-sound triple
  self-consistent through the ideal-gas law. The source table pins
  density and scale height, not local thermodynamic temperature; above
  the lower atmosphere this `temperature_k` is an effective fit value,
  not a substitute for NRLMSISE-00 thermospheric temperature output.
- `PiecewiseExpExoatmosphericPolicy` (`FailClosed` /
  `ZeroDensityAboveCeiling`) mirrors the existing
  `ExoatmosphericPolicy` for `UsStandard1976`.
- 12 unit tests: non-finite / negative altitude rejection, ceiling
  policy, sea-level density match, monotonic decrease within layers
  and across boundaries, layer-base reproduction, finite-positive
  invariants throughout the envelope, byte-stability, and
  documented-engineering-range bracket at 400 km.
- Switches in via `environment.atmosphere = "piecewise_exponential"`
  and the matching `[atmosphere].kind = "piecewise_exponential"`
  block (no per-scenario layer overrides — the shipped table is
  fixed for byte-stability across machines).
- Wired through both `phase2_point_mass` and `phase2_rigid_body`
  runners via a new shared `RuntimeAtmosphere` enum
  (`crates/openbmp-cli/src/runner/atmosphere.rs`) that dispatches
  between `UsStandard1976` and `PiecewiseExponentialAtmosphere`. The
  enum implements `AtmosphereModel` so every existing
  drag-adapter / telemetry call site flows through unchanged. The
  USSA76 byte-stable path is preserved verbatim.

**What was deferred.**

- Solar-flux dependence (F10.7, F10.7-avg, Ap) — the layered fit
  uses a single mean profile, not a daily-driven density model.
  Picked up by **§ 5.C.4**.
- Per-species number densities (N2, O2, O, He, Ar, H, N) — § 5.C.4.
- Diurnal / latitude / longitude / season variation — § 5.C.4.
- An in-house NRLMSISE-00 Rust port. Sourcing the ≈ 10 000-coefficient
  table authoritatively and committing to a determinism-stable
  port belongs in **§ 5.C.4**.

**Validation evidence.**

- Math:
  `crates/openbmp-physics/src/atmosphere/piecewise_exponential.rs`
  ships 12 unit tests covering the items listed above.
- End-to-end:
  `crates/openbmp-cli/tests/sounding_piecewise_exp_atmosphere_e2e.rs`
  asserts the demo scenario completes 4100 RK4 steps with end-time
  stop, the per-step atmosphere telemetry channels are populated
  with finite-positive values throughout the flight, the sea-level
  density matches the layered model's tabulated base value
  (1.225 kg/m³), the minimum density across the 204 km apogee
  arc lands in the LEO engineering envelope, and two reruns
  produce byte-identical Parquet.

**References.**

- Vallado, D. A. (2013). *Fundamentals of Astrodynamics and
  Applications*, 4th ed., Table 8-4 ("Exponential Atmosphere
  Model"). Microcosm Press / Springer — primary tabulation source.
- Curtis, H. D. (2014). *Orbital Mechanics for Engineering
  Students*, 3rd ed., Appendix D — same layer table reproduced.
- Wertz, J. R. and Larson, W. J. (1999). *Space Mission Analysis
  and Design*, 3rd ed., §8.1.4. Microcosm Press.
- US Standard Atmosphere 1976 supplemental reference profile
  (NOAA-S/T 76-1562 Part 2) — underlying source.
- Picone et al. (2002), *NRLMSISE-00 empirical model of the
  atmosphere: Statistical comparison and scientific issues*,
  J. Geophys. Res. 107(A12) — full NRLMSISE-00 reference (deferred).

**Scope guardrail.** Public layered-exponential coefficients only,
sourced from openly-published orbital-mechanics references. No
solar-flux dependence, no per-species data, no operational
tunings. Any future scenario that needs solar-driven densities must
wait for the dedicated NRLMSISE-00 follow-on slice.

#### 5.C.2 — EGM2008 zonal-harmonic gravity (degrees 2-6) **— shipped**

**Honest scope.** The shipped surface is the **zonal-only**
truncation of EGM2008 at degrees 2 through 6. Tesseral and sectoral
terms, the Cunningham recursion that supports them, and the
degree-20 working envelope originally proposed for this slice are
deferred to a follow-on phase that pins higher-degree normalised
coefficients. For zonal-dominated low-Earth-orbit drift effects
(right-ascension drift, argument-of-perigee drift, nodal regression),
J_2 through J_6 captures the dominant secular perturbations.

**What shipped.**

- `openbmp_physics::Egm2008ZonalGravity : GravityModel`, parametrised
  by `(µ, R_e, J_n[5], degree)` with `degree ∈ [2, 6]`. Constructor
  validation rejects non-positive µ / R_e, non-finite J_n, or
  out-of-range degree.
- `wgs84_egm2008_zonal()` constructor pins WGS84 µ / R_e and the
  Pavlis et al. 2012 J_n table (`EGM2008_J3..J6` constants in
  `crates/openbmp-physics/src/gravity.rs`).
- Math implements the Cartesian gradient form
  `g_n = ∇V_n` with `V_n = −(µ/r) (R_e/r)^n J_n P_n(ξ)`, ξ = z/r,
  evaluated via the standard Legendre recurrence
  `P_{n+1} = ((2n+1) ξ P_n − n P_{n-1}) / (n+1)` (and its derivative).
  Locked operand order matches the existing `J2Gravity` path so
  `degree = 2` with `J_3..J_6 = 0` produces byte-identical output.
- Switches in via `environment.gravity = "egm2008"` (no per-scenario
  coefficients, µ, or R_e overrides — the zonal table is pinned and
  the contract is byte-stable across machines).
- Wired into both `phase2_point_mass` and `phase2_rigid_body`
  runners. Demo: `scenarios/leo-orbit-egm2008/scenario.toml` propagates
  a 1 kg point mass through one nominal LEO orbit period (5556 s,
  RK4 dt = 1.0 s, 30° inclination) under the full degree-6 zonal
  expansion.

**What was deferred.**

- Tesseral and sectoral terms (orders > 0). Sourcing the full
  EGM2008 normalised-coefficient table (≈ 5 million entries through
  degree 2190) and committing to a determinism-stable Cunningham
  implementation belongs in **§ 5.C.3** below.
- Per-scenario `degree` and `coefficients_path` overrides. The
  shipped surface is degree-pinned at 6 with the public J_n table
  inlined as constants; future overridable surfaces arrive
  alongside the tesseral implementation in § 5.C.3.
- Degree-20 GPS-orbit reproduction tolerance. Without tesseral terms
  this is not achievable; the math-side `J2Gravity`-degeneracy and
  Legendre-recurrence unit tests cover correctness within shipped
  scope. The full GPS-class tolerance lands in § 5.C.3.

**Validation evidence.**

- Math: `crates/openbmp-physics/src/gravity.rs` ships 8 unit tests:
  constructor validation (rejects non-positive µ / R_e, non-finite
  J_n, out-of-range degree), origin-singularity, degree-2 degeneracy
  (byte-identical to `J2Gravity` at multiple positions), LEO-altitude
  acceleration sanity, higher-degree differentiation at non-equatorial
  points, determinism across reruns, pole geometry, and Legendre
  recurrence vs closed-form `P_n(ξ)` for low degrees.
- End-to-end: `crates/openbmp-cli/tests/leo_orbit_egm2008_e2e.rs`
  asserts the demo scenario completes 5556 RK4 steps with end-time
  stop, the final ECI radius stays within 5 km of the initial
  6 778 km circular radius (no orbit blow-up), and two reruns
  produce byte-identical Parquet (deterministic gravity model
  propagates through the closed-loop pipeline).

**References.** Pavlis, N. K., Holmes, S. A., Kenyon, S. C., and
Factor, J. K., *The development and evaluation of the Earth
Gravitational Model 2008 (EGM2008)*, J. Geophys. Res. 117(B4),
2012 — public NGA-released zonal coefficients pinned in
`EGM2008_J3..J6`. Vallado, D. A., *Fundamentals of Astrodynamics
and Applications*, 4th ed., §8.6 — Cartesian J_n gradient form.
Montenbruck, O. and Gill, E., *Satellite Orbits — Models, Methods,
Applications*, §3.2 — Legendre recurrence derivation.

**Scope guardrail.** Public NGA-released zonal coefficients only.
No derived datasets, no operational tunings, no satellite-specific
calibrations. Tesseral terms deferred to § 5.C.3.

#### 5.C.3 — EGM2008 tesseral / sectoral expansion (Cunningham recursion)

**Scope.** Pick up the tesseral and sectoral terms of EGM2008 that
the shipped § 5.C.2 zonal truncation deferred. Specifically:

- Extend `Egm2008ZonalGravity` (or introduce a sibling type
  `Egm2008Gravity` if the zonal-only fast path is preserved) to
  evaluate the full `(C_nm, S_nm)` series via the standard
  Cunningham (1970) recursion for the associated Legendre functions
  in ECEF, rotated into ECI on demand.
- Add a `data/gravity/EGM2008/` directory carrying public
  `(C_nm, S_nm)` coefficients up to a documented degree / order
  cap, with sibling `provenance.md` and SHA-256 pins in the
  scenario.
- Add per-scenario `degree`, `order`, and `coefficients_path`
  overrides on the `gravity = "egm2008"` selector. The shipped
  zonal-pinned surface remains the default to preserve byte
  stability for existing scenarios.
- Documented degree-20 working envelope; scenarios that opt into
  degrees > 20 carry `validation = "experimental"` until their
  own tolerance evidence lands.

**Exit criterion.** EGM2008-degree-20 gravity reproduces a
published GPS-orbit-class trajectory within a tolerance recorded
in `tests/expected/egm2008-tesseral.toml`. The shipped § 5.C.2
zonal scenario (`scenarios/leo-orbit-egm2008/`) stays
byte-identical when the runtime selects the default zonal-pinned
configuration (regression gate on the existing e2e).

**Validation evidence.** Cunningham-recursion unit tests against
closed-form `P_n^m(ξ)` for low degrees / orders; analytic-toy
property check that `C_n0` reduces to `−J_n` (zonal degeneracy);
LEO-orbit tolerance test that adds tesseral perturbations on top
of the shipped zonal scenario and matches a published reference
trajectory.

**References.** Pavlis, N. K., Holmes, S. A., Kenyon, S. C., and
Factor, J. K. (2012). *The development and evaluation of the Earth
Gravitational Model 2008 (EGM2008)*, J. Geophys. Res. 117, B04406.
Cunningham, L. E. (1970). *On the computation of the spherical
harmonic terms needed during the numerical integration of the
orbital motion of an artificial satellite*, Cel. Mech. 2(2). 
Montenbruck, O. and Gill, E. (2000). *Satellite Orbits — Models,
Methods, Applications*, §3.2.

**Scope guardrail.** Public NGA-released coefficients only. No
derived coefficient sets, no operational tunings, no
satellite-specific calibrations. NRLMSIS-side time-varying
gravity (Earth tides, ocean loading) stays Phase 6.

#### 5.C.4 — NRLMSISE-00 full Rust port

**Scope.** Pick up the solar-flux-driven, per-species atmosphere
that § 5.C.1 deferred. Specifically:

- In-house Rust port of NRLMSISE-00 (Picone et al. 2002) with the
  full public coefficient set under `data/atmosphere/NRLMSISE-00/`
  and a sibling `provenance.md` plus SHA-256 pins in the scenario.
- Public mathematical formulation only; no derived coefficient
  sets, no operational tunings.
- New type `NrlMsise00 : AtmosphereModel`. Returns density,
  temperature, and per-species number densities (N2, O2, O, He,
  Ar, H, N) over the documented validity envelope (0-1000 km).
- Switches in via the existing `[atmosphere].kind = "nrlmsise00"`
  scenario selector (already registered, currently rejected at
  parse time). Adds the required scenario fields `f10_7`,
  `f10_7_avg`, `ap_index` (scalar Ap), and an absolute epoch in
  TAI seconds.
- Adds `NrlMsise00` to the runner's `RuntimeAtmosphere` enum so
  every existing drag adapter / telemetry call site picks it up
  unchanged.
- Fails closed outside the validity envelope (no extrapolation;
  no clamp / linear opt-in for NRLMSISE-00).

**Exit criterion.** NRLMSISE-00 reproduces the published
reference profile from the original Picone et al. 2002 paper at
the documented test points within the tolerance recorded in
`tests/expected/nrlmsise00.toml`. The scenario fails closed at
1001 km altitude. The shipped § 5.C.1 piecewise-exponential
scenario stays byte-identical when the runtime selects
`piecewise_exponential` (regression gate on the existing e2e).

**Validation evidence.** Unit tests against the public reference
profiles; analytic-toy scenario where a sounding rocket ascends
through a documented density profile and the kernel sees the
expected drag column.

**References.** Picone, J. M., Hedin, A. E., Drob, D. P., and
Aikin, A. C. (2002). *NRLMSISE-00 empirical model of the
atmosphere: Statistical comparison and scientific issues*,
J. Geophys. Res. 107(A12).

**Scope guardrail.** NRLMSIS 2.x and HWM14 stay Phase 6
follow-ons. JB-2008 stays Phase 6. § 5.C.4 ships only the 2002
NRLMSISE-00 baseline.

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

#### 5.D.3 — Dormand-Prince 5(4) fixed-step integrator **— shipped**

**Honest scope.** The shipped surface is the **5th-order solution
of the DOPRI5(4) tableau at a fixed step size**. The embedded
4th-order solution, the PI step controller, and the adaptive-stepping
scenario profile flag originally proposed for this slice are
explicitly deferred to a follow-on slice. The fixed-step shape is a
drop-in higher-order alternative to `Rk4FixedStep` for scenarios
where 4th-order RK4 truncation error is the limiting factor.

**What shipped.**

- `openbmp_sim::Dopri54FixedStep` integrator implementing
  `Integrator<S>` for both `PointMassState` and `RigidBodyState`.
- Standard Dormand-Prince 5(4) Butcher tableau pinned exactly per
  Dormand & Prince (1980); coefficients live in a `dopri54_tableau`
  module with explicit per-coefficient names.
- Six derivative evaluations per step. Only the 5th-order weights
  (`B1, B3, B4, B5, B6`; `B2 = 0`, `B7 = 0`) participate in the
  combined update. The FSAL 7th derivative is not evaluated in this
  fixed-step surface; it is reserved for the embedded-error /
  adaptive path.
- Locked operand order matching the existing `Rk4FixedStep`
  determinism contract: explicit parentheses prevent compiler
  re-association on every weighted sum, no FMA on the hot path.
  Tagged `IntegratorDeterminism::BitStable`.
- 10 unit tests covering: zero / negative dt rejection, zero-derivative
  preservation of state, exactness for constant acceleration (a
  polynomial well within the 5th-order exactness range), 5th-order
  error behaviour on `dy/dt = -y` over `[0, 1]`, direct accuracy
  comparison vs `Rk4FixedStep` (DOPRI5 must be ≥ 10× more accurate
  on the same exponential-decay test), non-finite-derivative
  propagation, invalid-state rejection, byte-stability across
  reruns, determinism class assertion.

**What was deferred.**

- The embedded 4th-order solution (`E1, E3, E4, E5, E6, E7` weights)
  and the per-step error norm. Picked up by **§ 5.D.4**.
- PI / I step-size controller and the adaptive step-size loop —
  § 5.D.4.
- An `IntegratorDeterminism::StateStable` profile registration and
  the `--profile=adaptive` / `[simulation] profile = "adaptive"`
  scenario plumbing — § 5.D.4.
- The DOPRI8(7) tableau and its adaptive variant — folded into
  § 5.D.4 as a stretch (separate ID reserved if it grows out of
  scope).
- A separate determinism CI gate for the adaptive profile —
  § 5.D.4.

**Validation evidence.**

- Math: the 10 unit tests in `crates/openbmp-sim/src/integrator.rs`
  enumerated above.
- The `dopri54_is_more_accurate_than_rk4_on_exponential_decay`
  test gives a quantitative anchor: at `dt = 0.1 s` over 10 steps,
  DOPRI5 lands within `1e-7` of `exp(-1)`, RK4 within `~3.4e-7`
  — DOPRI5 is at least 10× more accurate on the same step size,
  consistent with the order-5 vs order-4 expectation.

**References.** Dormand, J. R., and Prince, P. J. (1980). *A family
of embedded Runge-Kutta formulae*, J. Comp. Appl. Math. 6(1):19-26
— Butcher tableau pinned in `dopri54_tableau` constants. Hairer,
Nørsett, and Wanner (1993). *Solving Ordinary Differential
Equations I*, 2nd rev. ed., §II.5 Table 5.2 (Springer) — same
table reproduced for cross-check.

**Scope guardrail.** Fixed-step only on shipping; the existing
RK4 default profile stays the byte-stable reference for every
shipped scenario. No release artifact is benchmarked against the
new integrator until § 5.D.4 lands with its own state-stable CI
gate.

#### 5.D.4 — DOPRI5(4) adaptive integrator with PI step controller **— shipped**

**Status.** Shipped on the **point-mass runner only**. The `[solver]
profile = "adaptive-explicit"` + `trajectory_method = "dopri54"` +
`determinism = "state-stable"` triple now selects
`openbmp_sim::Dopri54Adaptive` via the new
`crates/openbmp-cli/src/runner/integrator.rs` `RuntimeIntegrator`
enum dispatch. The fixed-step default
(`profile = "fixed-step-explicit"` / `trajectory_method = "rk4"` /
`determinism = "bit-stable"`) is preserved bit-for-bit on every
existing scenario (the runner-side e2e suite re-runs unchanged).
The rigid-body runner adaptive path and the per-component error norm
moved to § 5.D.5 (shipped); DOPRI8(7) moves further to § 5.D.6;
see scope deferrals below.

**Scope (shipped).**

- Embedded 4th-order solution weights (`E1, E3, E4, E5, E6, E7`)
  added to the shipped `dopri54_tableau`. The 7th-stage derivative
  `k7` (the FSAL slot reserved by § 5.D.3) is evaluated here for
  the first time and feeds the embedded-solution input — § 5.D.3
  intentionally does NOT compute it.
- New type `openbmp_sim::Dopri54Adaptive : Integrator<S>`. Outer
  loop accumulates sub-steps to fit the kernel's outer `dt`,
  inner loop is `try_substep` + accept/reject + PI factor.
  Persistent state (`last_h_s`, `last_err_prev`) lives in
  `Cell<Option<f64>>` so the `Integrator::advance(&self, ...)`
  trait surface stays unchanged. If a rejected sub-step is already
  at `min_dt_s`, the integrator fails closed instead of accepting a
  step outside the requested tolerance.
- Scaled error norm: `err = h · ||e'|| / (atol + rtol · ||y||)`,
  where `||·||` is the new `SimStateDerivative::l2_norm()` /
  `Integratable::scalar_state_size()` trait extension. Scalar
  tolerance form (Hairer-Nørsett-Wanner Vol I §II.4 simplified
  shape); per-component refinement deferred to § 5.D.5.
- PI step controller with hardcoded Gustafsson 1991 exponents
  (`α = 0.7`, `β = 0.4`), safety = 0.9, factor clamp `[0.2, 5.0]`,
  embedded order = 4. Fallback to a non-PI factor on the very
  first step (when `last_err_prev` is `None`).
- New `IntegratorDeterminism::StateStable` enum variant. Tagged on
  `Dopri54Adaptive` only; `Rk4FixedStep` and `Dopri54FixedStep`
  remain `BitStable`. `StateStable` is documented as
  within-platform bit-stable: same target triple + toolchain +
  LLVM optimisation level + `Cell` seed produce byte-identical
  reruns; cross-platform behaviour may diverge through libm
  `pow()` / `ln()` differences.
- Scenario plumbing: `[solver]` block (already parser-only) is now
  consumed by the runner. `(profile, trajectory_method,
  determinism)` triples not wired in the runner are rejected at
  scenario-load with `CliError::UnsupportedScenario` — `dopri853`,
  `rkf78`, `implicit-source-term`, `partitioned-hypersonic`,
  `adaptive-explicit + non-dopri54` all fall through to the
  runner-side validator unit tests in
  `crates/openbmp-cli/src/runner/integrator.rs`.
- New demo scenario:
  `scenarios/leo-orbit-egm2008-adaptive/scenario.toml` (sibling of
  the § 5.C.2 fixed-RK4 LEO orbit). End-to-end test
  `crates/openbmp-cli/tests/leo_orbit_egm2008_adaptive_e2e.rs`
  asserts the orbit completes within the same 5 km radius
  envelope and produces byte-identical Parquet across two reruns
  on the same platform.

**Scope (deferred at § 5.D.4 ship time — landed locations).**

- Rigid-body runner adaptive path → **shipped in § 5.D.5**.
  `phase2_rigid_body` now dispatches through
  `build_runtime_integrator(document)?` exactly like the point-mass
  runner; the audit-follow-up reject gate is gone.
- Per-component (vector-form) tolerance / error norm →
  **shipped in § 5.D.5**. `Integratable::weighted_error_norm`
  implements the HNW Vol I §II.4 RMS form with
  `sc_i = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)` per component;
  `Dopri54Adaptive::try_substep` consumes it directly.
- DOP853 8(5,3) tableau and fixed/adaptive variants →
  **shipped in § 5.D.6**. The `dopri853` trajectory method is now
  wired for both `(fixed-step-explicit, dopri853, bit-stable)` and
  `(adaptive-explicit, dopri853, state-stable)`.
- Property test that the PI controller drives the error norm into
  its declared band over a long trajectory → **shipped in § 5.D.5**
  (`dopri54_adaptive_pi_controller_stays_in_band_over_long_run`
  drives `try_substep` directly and asserts no clamp pinning, tight
  consecutive-step ratio band, and < 10 % rejection rate).

**Exit criterion (achieved).** The adaptive integrator runs the
LEO-orbit demo end-to-end with the documented radius envelope and
within-platform byte-stability. The fixed-step default profile is
byte-identical on every existing scenario (verified via
`leo_orbit_egm2008_e2e`, `sounding_piecewise_exp_atmosphere_e2e`,
`end_to_end`, `sounding_rocket_e2e`, `calisto_e2e`,
`multi_body_e2e`, `parachute_recovery_e2e`).

**Validation evidence (current tree).** 15 unit tests on
`Dopri54Adaptive` covering embedded-error norm, accept/reject
monotonicity, FSAL k7 reuse, step-clamp boundaries,
floor-failure handling, the §5.D.5 per-component breach case, the
PI band-stability test, and 5th-order convergence on a polynomial
trajectory. 11 runner-validator unit tests on
`build_runtime_integrator_from_solver` cover the wired-combo
positives, every deferred-combo rejection path, and the full
supported cross-product. The §5.D.4 rigid-body non-RK4 negative CLI
snapshot was retired when §5.D.5 wired the positive rigid-body
adaptive path; unwired methods still fail closed at the shared
runner-dispatch layer. 2 e2e tests cover the LEO-orbit-adaptive
scenario.

**References.** Dormand, J. R., and Prince, P. J. (1980).
*A family of embedded Runge-Kutta formulae*, J. Comp. Appl.
Math. 6(1):19-26. Hairer, Nørsett, and Wanner (1993). *Solving
Ordinary Differential Equations I*, 2nd rev. ed., §II.4 ("Practical
Step-Size Control"). Gustafsson, K. (1991). *Control theoretic
techniques for stepsize selection in explicit Runge-Kutta
methods*. ACM TOMS 17(4):533-554.

**Scope guardrail (held).** Adaptive profile is opt-in via
`[solver]` and labelled `state-stable, not bit-stable`. The
bit-stable default (`Rk4FixedStep`) is the one any release
artifact is benchmarked against. The adaptive profile cannot
replace the fixed-step default in any existing scenario without an
explicit `[solver]` block in the scenario header. The default
codepath when `[solver]` is absent is preserved bit-for-bit.

#### 5.D.5 — Adaptive integrator: rigid-body runner + per-component error norm + PI band test **— shipped**

**Status.** Shipped on the rigid-body runner. `[solver] profile =
"adaptive-explicit"` now drives `Dopri54Adaptive` through the
`RuntimeIntegrator` enum dispatch on **both** runners (point-mass
and rigid-body); the §5.D.4 audit-follow-up reject gate that
refused non-RK4 rigid-body selections has been retired. The scaled
error norm is now the per-component Hairer-Nørsett-Wanner Vol I
§II.4 RMS form; the §5.D.4 scalar placeholder
`||e||₂ / (atol + rtol·||y||₂)` is gone. DOP853 / DOPRI8(7) moves
to § 5.D.6.

**Scope (shipped).**

- Per-component error norm trait extension on
  `openbmp_models::Integratable`:
  ```rust
  fn weighted_error_norm(
      &self,                          // y_{n+1}
      prev_state: &Self,              // y_n
      error_deriv: &Self::Derivative, // e' = Σ E_i k_i
      h: f64, atol: f64, rtol: f64,
  ) -> f64;
  ```
  HNW Vol I §II.4 RMS form `err = sqrt((1/N) · Σ_i ((h·e'_i / sc_i))²)`
  with `sc_i = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)`. The
  per-component walk is locked and follows the same state/derivative
  prefix order as the existing `l2_norm`; on `RigidBodyState` it
  intentionally extends the inertia traversal to all 9 matrix
  entries while `scalar_state_size` remains a 20-component
  diagnostic.
- `PointMassState` impl: 7 components (pos×3, vel×3, mass×1) with
  derivative pairing pos↔vel, vel↔accel, mass↔mass_rate.
- `RigidBodyState` impl: 26 components (pos×3, vel×3, quat×4,
  ang_vel×3, mass×1, cg×3, inertia×9) — the full 9-entry inertia
  rate is summed (matching `RigidBodyDerivative::dimension`); the
  symmetric off-diagonal pairs are double-counted but harmlessly,
  because each scaled term is normalised by its own per-component
  scale and the controller's setpoint-1 dynamics absorb a constant
  factor √2 silently. The legacy `scalar_state_size` retains its
  20-component diagonal-only shape as a state-magnitude diagnostic
  for telemetry consumers; the divergence between the two methods is
  documented inline.
- `Dopri54Adaptive::try_substep` now calls
  `weighted_error_norm` directly. The scalar form is gone — no
  feature flag, no opt-in, no backward-compat shim. The shipped
  `leo-orbit-egm2008-adaptive` Parquet baseline updates as a side
  effect (the byte-stable check is "two reruns produce the same
  bytes," which is preserved).
- Rigid-body runner adaptive wiring: `phase2_rigid_body::run`
  dispatches through `build_runtime_integrator(document)?` exactly
  like the point-mass runner. The §5.D.4 audit-follow-up reject
  gate is removed.
- New rigid-body adaptive demo scenario:
  `scenarios/calisto-adaptive/scenario.toml` — RocketPy Calisto
  with `[solver]` adaptive-explicit / dopri54 / state-stable,
  `rtol = 1e-7`, `atol = 1e-9`. The e2e test
  `crates/openbmp-cli/tests/calisto_adaptive_e2e.rs` asserts the
  apogee stays within ±2 % of the RocketPy 3349 m AGL baseline
  (same envelope as the Phase-3.11 fixed-RK4 e2e gate), the drogue
  + main parachute mission graph fires, and two reruns produce
  byte-identical Parquet within-platform.
- PI controller band-stability property test in
  `Dopri54Adaptive::tests`. Drives `try_substep` directly on an
  exp-decay trajectory at tight tolerances, observing the
  controller's accept/reject pattern. Asserts:
    * Post-warmup, no clamp pinning (no `min_h` / `max_h` saturation).
    * Consecutive accepted-h ratios stay in `[0.5, 2.0]`
      (much tighter than the factor clamp `[0.2, 5.0]`).
    * Rejection rate < 10 % over the full run (HNW Vol I §II.4
      cites < 5 % as typical for well-tuned controllers; the
      10 % threshold catches a broken controller without flaking
      on benign initial-step rejections).
    * Steady-state median accepted h sits in the
      tolerance-implied sanity band.

**Scope (deferred to § 5.D.6).**

- DOP853 / `Dopri87Adaptive`: 8th-order Dormand-Prince embedded
  pair with 5(3) error estimators (the SciPy `DOP853` /
  Hairer Vol I §II.5 Table 5.4 form, 12 primary stages + 4
  interpolation abscissas) → **shipped in § 5.D.6** for the
  12-primary-stage fixed/adaptive integrators. Dense-output
  interpolation remains deferred.
- Higher-order property tests on multi-step trajectories
  (Lorenz, Van der Pol) — useful for stress-testing DOP853 once
  it lands; not required for the current shipped surface.

**Exit criterion (achieved).** The rigid-body adaptive runner
produces apogee within the 2 % cross-tool envelope on the Calisto
scenario, byte-stable across two same-platform reruns. The
per-component error norm is verified via 10 trait-impl unit tests
plus the `dopri54_adaptive_per_component_norm_surfaces_multi_scale_breach`
case. The PI controller band test passes. All existing rigid-body
e2e tests (calisto, multi_body, parachute_recovery, sounding_rocket
rigid subset, diff-flatness suite, tank_slosh) remain
byte-stable on the default (no-`[solver]`) codepath.

**Validation evidence.** 10 `weighted_error_norm` unit tests
covering both PointMassState and RigidBodyState (uniform RMS
formula, dimension-26 invariant, multi-scale state, bit-stability).
15 `Dopri54Adaptive` unit tests including the new band-stability
test. Calisto-adaptive e2e + LEO-orbit-egm2008-adaptive e2e. Full
existing rigid-body and point-mass e2e suites pass.

**References.** Hairer, E., Nørsett, S. P., and Wanner, G. (1993).
*Solving Ordinary Differential Equations I: Nonstiff Problems*,
2nd ed., §II.4 ("Practical Step-Size Control") — per-component
RMS error norm. Gustafsson, K. (1991). *Control theoretic
techniques for stepsize selection in explicit Runge-Kutta
methods*. ACM TOMS 17(4):533-554.

**Scope guardrail (held).** Adaptive profile remains opt-in via
`[solver]` and labelled `state-stable, not bit-stable`. The
bit-stable default (`Rk4FixedStep`) continues to be the
release-artifact benchmark profile. The default codepath when
`[solver]` is absent is preserved bit-for-bit on every existing
scenario, including the analytic-toy determinism gate.

#### 5.D.6 — DOP853 8(5,3) integrator (fixed-step + adaptive variants) **— shipped**

**Status.** Shipped on both runners (the rigid-body runner inherits
the dispatch through `RuntimeIntegrator` exactly like § 5.D.5 wired
DOPRI5(4) for it). The `[solver]` triples
`(fixed-step-explicit, dopri853, bit-stable)` and
`(adaptive-explicit, dopri853, state-stable)` are now wired
positively in `build_runtime_integrator_from_solver`. The new
adaptive variant uses the SciPy / Hairer-Wanner combined
err5/err3 stabilised error norm — when err5 vanishes
coincidentally, the err3 keeps the denominator finite and the
controller continues to make a sensible step-size decision.

**Scope (shipped).**

- New `dopri853_tableau` mod with the 12-stage DOP853 Butcher
  tableau coefficients pinned against SciPy
  `scipy/integrate/_ivp/dop853_coefficients.py` (Hairer's reference
  Fortran `dop853.f`, Hairer-Nørsett-Wanner Vol I §II.5 Table 5.4).
  Constants written as the SciPy decimal literals verbatim;
  `B - B̂_3` style derivations in the E3 estimator are computed at
  const-eval time. The 4 extra abscissas / dense-output
  coefficients are NOT included — § 5.D.6 does not ship the order-7
  dense interpolator.
- `openbmp_sim::Dopri853FixedStep` — 8th-order fixed-step
  integrator. 12 stage evaluations per step; locked-order weighted
  sum with no FMA. Tagged `IntegratorDeterminism::BitStable`.
- `openbmp_sim::Dopri853Adaptive` — 8(5,3) adaptive variant with
  I-controller (no PI β term — SciPy convention). Combined error
  norm:
  ```
  sc_i      = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)        (per-component)
  err5_rms² = (1/N) · Σ_i ( h · e5'_i / sc_i )²              (HNW Vol I §II.4 RMS)
  err3_rms² = (1/N) · Σ_i ( h · e3'_i / sc_i )²
  err       = err5_rms² / sqrt(err5_rms² + 0.01 · err3_rms²)
  ```
  factored through the existing `Integratable::weighted_error_norm`
  trait surface (no new trait extension required). Controller
  constants: `safety = 0.9`, `min_factor = 0.2`, `max_factor = 10.0`,
  `error_exponent = -1/8` (SciPy DOP853's
  `error_estimator_order = 7`, so `-1 / (7 + 1)`). Tagged
  `IntegratorDeterminism::StateStable`. Same fail-closed contract
  as `Dopri54Adaptive` — at `min_h_s` with err > 1, the integrator
  returns `IntegratorError::InvalidStep` rather than silently
  accepting a step outside tolerance.
- `RuntimeIntegrator` enum extended with `Dopri853Fixed` and
  `Dopri853Adaptive` variants. Dispatch logic updated to wire both
  triples; the prior "deferred" reject paths for `dopri853` are
  gone. Updated the `solver_cross_product_accepts_only_wired_triples`
  unit test plus added 4 new tests for the dopri853 wiring path
  (positive selection × 2 + negative `rkf78` reject + missing
  adaptive block defensive error).
- Demo scenario `scenarios/leo-orbit-egm2008-dopri853-adaptive/`
  with `rtol = 1e-16`, `atol = 1e-19`. E2E tests assert the orbit
  completes within ±5 km of the initial radius, produces
  byte-identical Parquet across two reruns on the same platform,
  and does not collapse to the staged fixed-step DOP853 trajectory.

**Scope (deferred).**

- Order-7 dense-output interpolator. SciPy's DOP853 ships the
  full 16-stage tableau (12 primary + 4 extra) with `D` matrix
  coefficients for an order-7 dense-output spline. We have not
  wired this — only the 12 primary stages are encoded. Useful
  follow-on if a downstream consumer needs sub-step state
  interpolation (e.g., event-trigger time refinement).
- PI variant with β-term smoothing on top of the err5/err3
  stabilisation. SciPy doesn't ship one. Plausible refinement if
  the I-controller is observed to oscillate on a stiff problem;
  not motivated by the current shipped surface.

**Exit criterion (achieved).** `Dopri853FixedStep` and
`Dopri853Adaptive` reproduce a degree-8 polynomial trajectory to
within machine precision on the 8th-order primary solution. The
DOP853 LEO-orbit demo completes within the same ±5 km radius
envelope as the §5.D.4 DOPRI5(4) baseline, with within-platform
byte-stability across two reruns and a regression assertion that the
adaptive run is not byte-identical to a staged fixed-step DOP853 run.
All existing e2e tests (point-mass adaptive + fixed-step, rigid-body
adaptive + fixed-step, analytic-toy determinism gate) remain
byte-stable on the default (no-`[solver]`) codepath.

**Validation evidence.** 6 fixed-step unit tests (tableau row sums
match abscissas, B sums to 1, E5 / E3 sum to 0, polynomial
degree-8 trajectory exact integration, determinism marker is
BitStable, two-rerun bit-stability). 12 adaptive unit tests
(constructor validation × 3, determinism marker is StateStable,
degree-8 primary-solution exact integration, sub-step accumulation
lands exactly at `dt`, final fragment below `min_h_s`, min-floor
fail-closed behaviour, reset history clearing, I-controller
band-stability, two-rerun bit-stability, and DOP853-vs-DOPRI54
sanity at loose tolerance). 4 runner-validator
unit tests (dopri853 fixed-step + adaptive positive selection,
rkf78 + missing-adaptive-block negative rejections, plus the
existing cross-product test extended to recognise dopri853 as
wired). 3 e2e tests on `leo-orbit-egm2008-dopri853-adaptive`.

**References.** Prince, P. J., and Dormand, J. R. (1981). *High
order embedded Runge-Kutta formulae*. J. Comp. Appl. Math.
7(1):67-75 — original Prince-Dormand 8(7) paper. Hairer, Nørsett,
and Wanner (1993). *Solving Ordinary Differential Equations I*,
2nd rev. ed., §II.5 Table 5.4 — the DOP853 tableau Hairer
reformulated with a 5(3) embedded estimator (the form SciPy and
this slice ship). Hairer's reference Fortran `dop853.f`; SciPy
`scipy/integrate/_ivp/dop853_coefficients.py` and `rk.py` for the
combined err5/err3 norm formulation.

**Scope guardrail (held).** Adaptive profile remains opt-in via
`[solver]` and labelled `state-stable, not bit-stable`. The
fixed-step DOP853 variant is `bit-stable` — same default-codepath
contract as `Rk4FixedStep` and `Dopri54FixedStep`. The
release-artifact benchmark profile remains the no-`[solver]`
default (`Rk4FixedStep`).

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
  ├── 5.B.1 (SR-UKF) ──► 5.B.2 (multi-lane)
  │
  ├── 5.B.3 (IMM 2-mode bank) ──► 5.B.6 (IMM 3-mode + lanes + UKF/MEKF)
  │     (5.B.3 was the original "3-mode IMM as lane in 5.B.2" line
  │      item; shipped as the honest 2-mode-default standalone
  │      downscope. 5.B.6 picks up the deferred 3-mode boost/coast/
  │      descent canonical bank, the IMM-as-lane integration, and
  │      the IMM-over-UKF/MEKF generic refactor.)
  │
  ├── 5.B.4 (windowed-mean-shift GLRT) ──► 5.B.5 (parity-space residual generator)
  │     (5.B.4 was the original "GLRT + parity" line item; shipped as
  │      the honest GLRT-only downscope. 5.B.5 picks up the deferred
  │      Patton-Frank parity-space surface.)
  │
  ├── 5.C.1 (piecewise-exp atmosphere) ──► 5.C.4 (NRLMSISE-00 full port)
  │     (5.C.1 was the original NRLMSISE-00 line item; shipped as the
  │      honest engineering downscope. 5.C.4 picks up the deferred
  │      solar-flux-driven full port.)
  │
  ├── 5.C.2 (EGM2008 zonal) ──► 5.C.3 (EGM2008 tesseral / Cunningham)
  │     (5.C.2 was the original full-EGM2008 line item; shipped as the
  │      honest zonal-only downscope. 5.C.3 picks up the deferred
  │      tesseral / sectoral expansion.)
  │
  ├── 5.D.1 (multi-rate) ──► 5.D.2 (multi-body)
  │
  ├── 5.D.3 (DOPRI5 fixed-step) ──► 5.D.4 (DOPRI5(4) adaptive + PI controller, point-mass runner)
  │                                       │
  │                                       ▼
  │                                  5.D.5 (rigid-body adaptive runner +
  │                                         per-component error norm + PI band test)
  │                                       │
  │                                       ▼
  │                                  5.D.6 (DOP853 8(5,3) integrator —
  │                                         fixed-step + adaptive variants)
  │     (Group D adaptive-integrator chain: 5.D.3 was the original DOPRI5/8
  │      adaptive line item, shipped as the honest fixed-step DOPRI5
  │      downscope. 5.D.4 picks up the DOPRI5(4) adaptive surface on the
  │      point-mass runner; 5.D.5 wires the rigid-body runner and ships
  │      the per-component error norm + PI band-stability test; 5.D.6
  │      adds the DOP853 8(5,3) integrator at the top of the order
  │      hierarchy. All four sub-phases shipped.)
  │
  └── 5.E.1 (HIL bridge) ──► 5.E.2 (ULog/PX4) ──► 5.E.3 (dataflash/ArduPilot)
                                                       │
                                                       ▼
                                                   5.E.4 (Bar-Shalom) ──► 5.E.5 (long-duration soak)
```

A → B → C → D run in parallel; E sub-phases are gated by their
prerequisite sub-phases (5.E.5 is gated by 5.A.1 + 5.A.5 + 5.B.1).
The follow-on sub-phases (5.C.3, 5.C.4) are gated by their parent
shipped slices (5.C.2 and 5.C.1) and exist to track the deferred
surfaces from those slices' honest downscopes. Group D's adaptive-
integrator chain (5.D.3 → 5.D.4 → 5.D.5 → 5.D.6) is fully shipped.

## Risks and contingencies

- **Determinism regressions.** SR-UKF (§ 5.B.1), EGM2008 Cunningham
  recursion (§ 5.C.3), Clarabel SOCP (§ 5.A.4 — shipped), and the
  adaptive integrator family
  (§ 5.D.4 / § 5.D.5 / § 5.D.6 — all shipped) are floating-point-
  heavy; the determinism CI gate is the canary. The shipped fixed-
  step / zonal-only / engineering-atmosphere slices honour the
  bit-stable default profile; the tesseral / full-port follow-ons
  (§ 5.C.3, § 5.C.4) are the ones most likely to need a
  `state-stable` profile flag, and each names the gate in its own
  section. The shipped Group D adaptive integrators
  (`Dopri54Adaptive`, `Dopri853Adaptive`) are tagged
  `IntegratorDeterminism::StateStable` as opt-in labels and verify
  within-platform byte-stability on both runners
  (`leo_orbit_egm2008_adaptive_e2e`,
  `leo_orbit_egm2008_dopri853_adaptive_e2e`,
  `calisto_adaptive_e2e`); the bit-stable defaults (`Rk4FixedStep`,
  `Dopri54FixedStep`, `Dopri853FixedStep`) remain available as
  release-artifact benchmarks. If any future sub-phase produces
  non-bit-stable output without an explicit profile flag, gate the
  work behind one and document the diff before merge.
- **External-log availability.** If a chosen public log is removed
  from the upstream archive during Phase 5, the case is paused, the
  provenance entry retired, and a substitute public log is sourced.
  No backup-private-log path lands in the repo.
- **NRLMSISE-00 numerical envelope (§ 5.C.4).** The 2002 Fortran
  reference has known numerical quirks at boundaries. The Rust
  port matches the public reference profile within tolerance;
  quirks outside the documented validity envelope fail closed
  rather than degrade silently. The shipped § 5.C.1 layered
  exponential carries no solar-flux dependence, so this risk is
  scoped to § 5.C.4 specifically.
- **EGM2008 coefficient table sourcing (§ 5.C.3).** The full
  normalised `(C_nm, S_nm)` table runs to ≈ 5 million entries; the
  shipped § 5.C.2 sidesteps this by pinning only the public zonal
  J_n constants in source. § 5.C.3 must source the higher-degree
  table from a citable public NGA release with SHA-256 pin and
  sibling provenance.md, or document a degree cap that keeps the
  table inlined.
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
