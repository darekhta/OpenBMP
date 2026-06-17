# Parity Program — Implementation Progress Ledger

**Status:** `experimental` (tracking document; ships no code).
**Snapshot:** 2026-06-12, current branch state. Verified by code inspection
against each design doc's §9 acceptance criteria and `requirements.toml`
traceability (110 requirement ids).

**Marking criteria (the don't-overstate rule applies):**

- `implemented` — code + tests landed and the WP's acceptance bullets are
  satisfied on inspection. Final authority remains the `13` §2 gate set in
  CI; this ledger records evidence, it does not replace gates.
- `partial` — material elements landed; the entry names what is missing.
- `in progress` — uncommitted working-tree work.
- `not started` — no trace in the workspace.

A tier is only as done as its weakest required WP. Update this ledger in the
same PR as any WP merge.

---

## 1. Roll-up

Across the series: **207 work packages** (108 in `01`–`12`, 99 in `14`–`25`).
As of this snapshot: **22 implemented · 21 partial · 0 in progress ·
164 not started.**

| Doc | Dimension | Position on its tier ladder | WPs impl/partial/total | Next unblocked WP |
|---|---|---|---|---|
| 01 | Multibody dynamics | T0 plus WP-01.1 spatial-vector/CRBA/RNEA/dense-FD/ABA/state-integration substrate crate; simulator wiring and oracle validation still open | 0/1/6 | finish WP-01.1 |
| 02 | Structural, loads, slosh, POGO | T0 (+ feed-side POGO prerequisite now exists via 05) | 0/0/9 | WP-02.1-a |
| 03 | Aero database & CFD | T0 baseline | 0/0/9 | WP-03.1 |
| 04 | Aerothermal, real-gas, TPS | T0 (+ shared `openbmp-thermochem` scaffold) | 0/0/11 | WP-04.1-a |
| 05 | Propulsion high-fidelity | **T1+T2 implemented**; T3 partial; T4/T5 wired-substrate partial | 3/6/9 | finish WP-05.3 (CEARUN/Cantera tolerance tables) |
| 06 | Coupled MIMO GNC | T0 (+ lane-voting diagnostic improvement) | 0/0/10 | WP-06.1 (after 01) |
| 07 | Trajectory optimization | **T0-T1 implemented**; STM/multiple-shooting substrate closed with cross-tier tolerance evidence | 2/0/8 | WP-07.2 |
| 08 | Environment, gravity, frames | T0 plus WP-08.1 degree-2 tesseral substrate and WP-08.2 Battin third-body/SRP/Schwarzschild analytic pieces; high-degree Pines/Gottlieb and Orekit force-stack validation still open | 0/2/7 | finish WP-08.1 or WP-08.2 |
| 09 | Sensors, nav, actuators | T0 (specific force still finite-difference) | 0/0/12 | WP-09.1 |
| 10 | Flight SW in the loop (XIL) | **T1–T3 implemented**; T4 partial | 3/2/8 | finish WP-10.4, or WP-10.6 in parallel |
| 11 | Monte Carlo, UQ, validation | **T0–T1 implemented**; T2–T4 partial | 3/5/9 | WP-11.7 |
| 12 | Determinism, real-time, compute | **substantially implemented**; Linux aarch64 bit-stable CI lane present; GPU offload boundary pending | 9/0/10 | WP-12.4-b |
| 14 | Contact, touchdown, landing | WP-14.1 crate + runner force/diagnostic path present; WP-14.2 crate primitives landed; WP-14.3 implemented; WP-14.4 massless oleo/crush gear with per-pad ContactPair footpads implemented | 2/2/10 | WP-14.5 after WP-09.7, or WP-14.7 after terrain substrate |
| 15 | Plume & SRP | WP-15.1 L2 substrate plus point-mass solid-motor and rigid-body thermochemical liquid telemetry with documented MFR, point-mass nozzle-derived geometry, active mount-derived spacing, and off-byte-goldens landed | 0/1/12 | finish WP-15.1 liquid calibration/rigid full geometry |
| 16 | Parachute & recovery | T0 baseline (`recovery/` rack) | 0/0/11 | WP-16.1 |
| 17 | Cryogenic fluid management | not started | 0/0/8 | WP-17.1 |
| 18 | Ground segment & countdown | not started | 0/0/8 | WP-18.1 |
| 19 | Day-of-launch winds & commit | not started (WP-19.2 now has WP-07.0 prerequisite) | 0/0/6 | WP-19.1 |
| 20 | Telemetry, RF links, network | dictionary export exists; physics not started | 0/1/8 | WP-20.1 |
| 21 | Run-data, regression, visualization | not started | 0/0/8 | WP-21.1 |
| 22 | Acoustics & overpressure | not started | 0/0/7 | WP-22.1 |
| 23 | EPS & avionics emulation | not started | 0/0/6 | WP-23.1 |
| 24 | Postflight reconstruction & sys-ID | not started (`compare-telemetry` seed exists) | 0/0/7 | WP-24.1 |
| 25 | RPOD & docking | not started (vocabulary + multi-body + contact prerequisites exist) | 0/0/8 | WP-25.1 |

**Phase view (`00` §5).** Phase A is the critical path: of its five tracks,
`12` (MC/determinism substrate) and `11` T0–T2 are substantially done and
`05` T1 is done, but `01` (multibody tree) remains partial and `08`
(tesseral gravity) has only the first degree-2 substrate — they gate `02`,
`06`, and most of Phase B/C. Phase D started
early on its independent track (`10` T1–T3 done). The extension rounds are
design-complete, implementation-untouched except `14` (substrate) and the
`20` dictionary-export head start.

---

## 2. Per-dimension detail

### 01 — Flexible & articulated multibody dynamics

Baseline confirmed: single rigid body in `crates/openbmp-sim/src/kernel.rs`,
separation as mass-property partition, SDOF bending
(`crates/openbmp-vehicle/src/structural.rs`), equivalent-pendulum slosh
(`crates/openbmp-vehicle/src/tank/`). WP-01.1 is now **partial**:
`crates/openbmp-multibody` exists as the L2 spatial-vector substrate crate with
angular-over-linear `SpatialMotion`, moment-over-force `SpatialForce`,
force-dual cross products, Pluecker transforms, `SpatialInertia` construction
from validated `MassProperties`, the WP-01.1 `Joint` vocabulary, and a
topologically ordered `MultibodyTree`/`MultibodyState` shape with deterministic
q/qd offsets and fail-closed topology/state validation (`REQ-MULTIBODY-001`).
It also has a fixed-transform CRBA/RNEA self-consistency substrate:
`JointSpaceInertia`, `joint_space_inertia_crba_fixed_transforms()`, and
`inverse_dynamics_rnea_fixed_transforms()` prove CRBA columns against
zero-velocity RNEA on a small tree (`REQ-MULTIBODY-002`). The q-dependent
layer now adds `Joint::joint_transform()`, `PluckerTransform::then()`,
`body_transform_parent_to_child_at_state()`,
`joint_space_inertia_crba_at_state()`, and
`inverse_dynamics_rnea_at_state_no_bias()`, with transform-composition,
quaternion, body-index, and state-dependent CRBA/RNEA column checks
(`REQ-MULTIBODY-003`). The biased RNEA layer adds
`inverse_dynamics_rnea_at_state()` with generalized velocity-bias terms, a root
parent acceleration seed for gravity-style forcing, and per-body external
spatial forces; tests cover zero-velocity equivalence, single-free-flyer bias
force, root acceleration forcing, external-force subtraction, and fail-closed
force input validation (`REQ-MULTIBODY-004`). The dense forward-dynamics bridge
adds `forward_dynamics_dense_at_state()`, forming `H(q)` through CRBA and
`C(q, qd, a_root, f_ext)` through RNEA before solving `H qdd = tau - C`, with
round-trip, bad-force, and singular-system checks (`REQ-MULTIBODY-005`).
The ABA substrate adds `forward_dynamics_aba_at_state()`, performing the
three-pass articulated-body recursion over q-dependent transforms, velocity
bias terms, root parent acceleration, and per-body external forces with
fixed-order symmetric LDLT local joint solves; tests prove it matches the
dense bridge and RNEA round-trip on a moving nontrivial tree and rejects bad
forces or singular articulated blocks fail-closed (`REQ-MULTIBODY-007`). The
same ABA-vs-dense and RNEA round-trip sample chain is now also driven from a
checked TOML tolerance fixture with strict max-absolute-error thresholds
(`REQ-MULTIBODY-026`).
The energy/momentum substrate adds `generalized_momentum_at_state()` and
`kinetic_energy_at_state()`, with a checked planar point-mass
double-pendulum TOML fixture proving analytic kinetic energy and revolute
generalized momentum within strict tolerances (`REQ-MULTIBODY-028`).
The state-integration substrate adds `MultibodyDerivative`,
`advance_state_by()`, `project_state()`, `scalar_state_size()`, and
`weighted_error_norm()` with derivative arithmetic, quaternion projection,
scalar norm, and adaptive-error checks (`REQ-MULTIBODY-006`). The
unreleased welded-subtree handoff now exposes
`release_welded_subtree_as_free_flyer()`, remapping a selected welded subtree
into a deterministic free-flyer tree while preserving the released body's pose,
body-frame spatial velocity, and descendant joint state (`REQ-MULTIBODY-027`).
The adapter-facing kinematic lift adds `coordinate_derivative_from_velocity()` and
`derivative_from_state_and_acceleration()`, mapping scalar-joint velocities,
free-flyer and spherical quaternion rates, and free-flyer translation rates
into deterministic `q_dot`/`qd_dot` derivatives with bad-acceleration
rejection (`REQ-MULTIBODY-008`). The simulator-state adapter substrate now
relaxes the generic state/derivative trait bounds from `Copy` to `Clone`,
updates RK4/DOPRI and aggregate vehicle model forwarding ownership paths for
cloneable states/contexts, and adds `MultibodySimState` with tree-derived
quaternion offsets so vector-backed multibody states can satisfy
`openbmp-models::SimState` (`REQ-MULTIBODY-009`). The first kernel-facing
load-to-derivative bridge now runs ABA and lifts the result into
`MultibodyDerivative`, with a byte-identical identity-orientation,
zero-CG-offset, zero-translational-velocity single-free-flyer comparison
against the existing `RigidBodyDerivative` equations (`REQ-MULTIBODY-010`).
The root free-flyer load adapter maps body-frame moment plus parent-frame force
into locked moment-then-force generalized slots, rotates force through the root
quaternion, zero-fills non-root slots, and feeds the ABA derivative bridge
(`REQ-MULTIBODY-011`). The runner bridge now depends on `openbmp-multibody` and
constructs a one-body root free-flyer `MultibodyTree` plus `MultibodySimState`
from the existing rigid-body initial seed for primary `[multi_body]` scenarios,
including deterministic quaternion, position, angular-rate, and body-frame
linear-velocity q/qd mapping with fail-closed mass-property validation
(`REQ-MULTIBODY-012`). The runner derivative bridge now feeds root body-frame
moments and ECI-frame forces through the multibody load adapter, validates the
zero-load derivative path during primary `[multi_body]` setup, and proves
runner-level no-joint identity-orientation equivalence against the current
`RigidBodyDerivative` equations (`REQ-MULTIBODY-013`). A scenario-level
byte-equivalence guard now parses and runs a disabled primary `[multi_body]`
configuration through the real runner setup path, proving the current rigid
kernel telemetry stays byte-identical while the root free-flyer bridge is
exercised (`REQ-MULTIBODY-014`). The bridge now also seeds primary
`[multi_body]` roots from the mounted primary-body mass resources instead of
the pre-split composite mass, evaluates the live runner force/moment adapter
stacks with the initial environment and rack snapshots, and feeds those loads
through the multibody derivative bridge; tests prove primary mass selection and
nonzero gravity/direct-torque adapter load flow (`REQ-MULTIBODY-015`). The
prepared rigid-body session now also maintains a primary root free-flyer
multibody shadow, refreshes it from the current rigid state plus active
primary-body mass resources before each authoritative rigid-kernel step, and
evaluates the current runner force/moment stacks with held rack snapshots,
environment/wind, and mission-phase views; a session-stepped regression proves
two pre-step shadow refreshes carry the expected gravity/direct-torque loads
without changing rigid telemetry authority (`REQ-MULTIBODY-016`). A second
session-stepped synthetic scenario fires a real `scenario_script`
`engine_command`, publishes the resulting gimballed liquid-engine snapshot
through the runner/kernel adapter views, and proves the primary shadow
derivative sees signed engine-cluster force and moment contributions
(`REQ-MULTIBODY-017`). The session now also mirrors currently propagating
separated lanes as root free-flyer multibody shadows, covering both initial
lanes and lanes created by jettison events, sampling each separated lane state
and evaluating the same per-body runner force/moment adapter stacks;
booster-owned direct-torque regressions prove the separated shadow derivative
sees the signed pitch-torque load without changing rigid separated-lane
authority (`REQ-MULTIBODY-018`). The primary shadow now also has a first
constant-mass RK4 forecast path: the runner can project a root free-flyer
`MultibodySimState` back into `RigidBodyState`, evaluate force/moment adapters
at multibody RK stages, and prove a no-rotation gravity-only primary shadow
forecasts the same next rigid state as the current rigid kernel. The
multibody free-flyer quaternion convention is now explicit, body velocity lifts
into parent-frame position rate, parent forces rotate into body generalized
force slots, raw RK substep quaternions drive `q_dot` until projection, and a
constant-mass direct-torque/changing-attitude forecast matches the current
rigid kernel (`REQ-MULTIBODY-019`). The RK4 forecast now also evaluates the
same mass-rate adapter surface as the rigid kernel at each stage and proves a
no-rotation solid-motor burn forecasts the same variable-mass next rigid state
as the current kernel (`REQ-MULTIBODY-020`). Separated-lane shadows now record
the same one-step RK4 forecast, and an initial separated direct-torque lane
proves the forecast matches that lane's next rigid state without changing
separated-lane telemetry authority (`REQ-MULTIBODY-021`). A scenario-script
jettison regression now also copies the forecast recorded immediately after the
jettison-created booster lane is mirrored and proves it matches the next
separated-lane state over the first post-release tick (`REQ-MULTIBODY-024`).
The separated shadow also now proves a landing-controller-commanded,
booster-owned liquid-engine lane drains mass through the owner-filtered
engine-rate adapter and forecasts the same powered next separated-lane state
(`REQ-MULTIBODY-025`). The primary shadow now also seeds single-body active
stacks from the kernel's authoritative mass state, avoiding liquid-engine
snapshot double-drain, and proves a pitch-gimballed liquid-engine burn forecasts
the same next rigid state over one powered step (`REQ-MULTIBODY-022`). The same
synthetic gimballed ascent now validates two consecutive powered forecast ticks
against the current rigid kernel (`REQ-MULTIBODY-023`). A clipped Phalcon-9 TVC
probe now validates the primary-shadow forecast over 150 real runner ticks with
the nine-engine gimballed liquid-engine cluster, real FC guidance/autopilot
surfaces, and nonzero lateral gimballed thrust (`REQ-MULTIBODY-029`), and the
full 40 s Phalcon-9 TVC probe now runs to completion with the original pitch
schedule while every recorded primary-shadow RK4 forecast matches the
authoritative rigid state (`REQ-MULTIBODY-030`). The substrate now also proves
an offset inertial engine on a revolute gimbal carries nonzero root/gimbal
CRBA coupling, drives root reaction acceleration under gimbal torque, and
cross-checks ABA, dense forward dynamics, and biased RNEA
(`REQ-MULTIBODY-031`). Missing for full WP-01.1:
replacing the rigid-kernel propagation path, broader variable-mass multibody
propagation beyond the primary-root and powered separated-lane shadow forecasts,
full momentum-conserving welded-to-free joint-release propagation and full
separated-body multibody propagation beyond first-post-release separated shadow
forecasts and the substrate handoff, full orbital-ascent gimballed validation,
runner/scenario wiring for articulated gimbal joints, and Spatial_v2 oracle
checks.
WP-01.2 … WP-01.6: **not started**.

### 02 — Structural dynamics, loads, slosh & POGO

WP-02.1-a … WP-02.3-b: **not started** (still one SDOF mode per axis;
all-fluid pendulum slosh; no modal ingestion, no CLA).
WP-02.4-a/-b (POGO/CSI, structural half): **not started** — but the feed
half this capstone joins (doc 05 WP-05.5-a) landed its substrate today
(`crates/openbmp-feedsystem/src/pogo.rs`, `crates/openbmp-runner/src/pogo.rs`
with `PogoStabilityVerdict`, scenario `propulsion.pogo` block), so the
cross-doc prerequisite is no longer hypothetical. The structural
longitudinal modal model remains the missing half.

### 03 — Aerodynamic database & CFD coupling

Baseline confirmed: `crates/openbmp-aero` T0 (single-panel modified
Newtonian in `hypersonic.rs`, DATCOM-lite buildup in `buildup.rs`,
locked-order `AeroDeck` interpolation in `deck.rs`); no per-entry σ, no
run-matrix, no `openbmp-aerodb`. WP-03.1 … WP-03.9: **not started**.

### 04 — Aerothermal, real-gas & TPS

Baseline confirmed plus one shared scaffold: `openbmp-thermochem` landed
(deck + parser on a (log pc, MR) grid — combustion-deck shape for doc 05;
no equilibrium-air closure). Heating stack unchanged: Sutton-Graves
(`validated-toy`), cold-gas Fay-Riddell scaffold, Tauber-Sutton and
Tannehill typed-reserved (refusing), toy ablators, Park payload pinned.
WP-04.1-a … WP-04.4-b: **not started**.

### 05 — Propulsion high-fidelity

| WP | Status | Evidence / missing |
|---|---|---|
| WP-05.1 pressure-thrust + altitude | **implemented** | `NozzlePerformance` in `crates/openbmp-propulsion/src/motor.rs`; scenario `ambient_pressure_correction = "pressure_thrust"`; `crates/openbmp-runner/tests/pressure_thrust.rs` + fixture; REQ-PROP-001/V-PROP-001 |
| WP-05.2-a nozzle separation clipping | **implemented** | `NozzleSeparationCriterion::{Summerfield,Schmucker}`; scenario opt-in; test in `pressure_thrust.rs`; REQ-PROP-002 |
| WP-05.2-b transient solid ballistics | **implemented** | `TransientChamber` pc(t) ODE + erosive hook in `grain.rs`; `[propulsion.motor.grain] mode = "transient"`; test; REQ-PROP-003 |
| WP-05.3 thermochem deck ingestion | **partial** | crate + parser + scenario block + SHA pin + fixtures + inline grain, reduced feed-network, liquid-engine thermochemical runtime consumption, empirical `c_star_efficiency` band propagation, and a provenance-recorded Cantera 3.2.0 `gri30.yaml` LOX/LCH4 reference tolerance table landed; `LiquidEnginePerformance` derives thrust, nominal choked mass flow, `Isp`, and mass-flow/`Isp` envelopes from looked-up thermochemical state + nozzle geometry; missing: full CEARUN/Cantera tolerance-table matrix for LOX/RP-1, LOX/LH2, NTO/MMH, and broader LOX/LCH4 coverage |
| WP-05.4-a feed network + transient chamber | **partial** | `openbmp-feedsystem` (graph/network/line/chamber/control/transient) + `crates/openbmp-runner/src/feed_network.rs` + scenario `propulsion.feed_networks` validation; missing: dedicated acceptance tests + tolerance tables |
| WP-05.4-b turbopump map + NPSH | **partial** | `pump.rs` (`Turbopump`, normalized map, design point) + scenario pump blocks + runner pressure/cavitation coupling + synthetic provenance-backed tolerance table; missing: public real-pump calibration |
| WP-05.4-c MOC line transients | **partial** | `line.rs` (`MocLine`) + scenario line blocks + runner pressure perturbation coupling + synthetic provenance-backed Joukowsky tolerance table; missing: richer boundary library, standalone line topology, public benchmark tables |
| WP-05.5-a POGO feed half | **partial** | `pogo.rs` (feedsystem + runner) + `propulsion.pogo` schema + synthetic provenance-backed stability tolerance table; missing: structural modal-data consumption and public Saturn V/Titan case-history evidence |
| WP-05.5-b engine-out & fault library | **partial** | `crates/openbmp-sil/tests/propulsion_stimulus.rs`; `openbmp mc` propulsion-fault campaign path with UQ flags; full doc fault set unverified |

### 06 — Coupled MIMO GNC

Baseline confirmed: per-axis allocator/controllers, 15-state error-state
EKF, GLRT/IMM/voter present. Today's `a903c50` added covariance-trace lane
voting in `estimator_lanes.rs` (+ SR-UKF touch-up) — a diagnostics
improvement, not a backlog WP. WP-06.1 … WP-06.4-d: **not started**
(dense-B allocation, n×n DARE, gain scheduling, load-relief, tight nav,
engine-out reconfiguration all absent).

### 07 — Trajectory optimization & mission design

`corrector.rs` (Gauss-Newton/LM shooting) and the locked
`TerminalCondition` vocabulary exist and the lock **holds** (no
range/aimpoint variants; `RendezvousState` present). WP-07.0 is
**implemented**: `correct_two_body_apogee` gives `DifferentialCorrector::solve`
a non-test caller through `openbmp trajopt correct-apogee`, and
`openbmp trajopt correct-apogee-scenario` uses `openbmp-runner` as the
scenario forward map for `scenarios/trajopt-two-body-apogee/`. Both paths
produce postcard I-loads only on convergence and report non-convergence
without writing one (`REQ-TRAJOPT-001`).
WP-07.1: **implemented** — `src/stm.rs` integrates deterministic two-body
variational equations with a state-transition matrix and validates the STM
against finite-difference and complex-step columns; `src/shooting.rs` seeds
and evaluates M-segment continuity defects with a block-bidiagonal
`[STM_i, -I]` Jacobian, reports duration-sensitivity columns, appends soft
exterior-penalty rows for node state-component boxes, radius/speed path norms,
rotating-atmosphere qbar, and qbar-alpha, and `MultipleShootingCorrector`
performs damped fixed-endpoint interior-node correction with honest
non-convergence reporting plus fixed-initial terminal-condition correction,
free-duration terminal correction, controlled terminal correction with
piecewise-constant ECI acceleration controls, and a trybuild no-surface-
coordinate tripwire (`REQ-TRAJOPT-002`). The checked-in
`scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml` table now
gates a perturbed T1 multiple-shooting solve against the T0 single-shooting
apogee solution to `< 1e-6`, closing WP-07.1 acceptance.
WP-07.2 … WP-07.6: **not started**.

### 08 — Environment, gravity & frames

Baseline confirmed: `Egm2008ZonalGravity` is zonal-only, hard-capped at
degree 6 (`gravity.rs`), and WP-08.1 is now **partial** through
`TesseralGravity`, `DegreeTwoTesseralCoefficients`, and `TideSystem`: the first
degree-2/order-2 static harmonic surface supports C20, C21/S21, and C22/S22
terms, keeps the degree-2/order-0 WGS84 J2 path byte-identical to `J2Gravity`,
stays finite near the pole, and rejects unsupported degree/order requests
fail-closed (`REQ-ENV-001`). Missing for full WP-08.1: high-degree
Pines/Gottlieb synthesis, EGM2008 coefficient ingestion/provenance/tripwire,
and NGA HARMONIC_SYNTH benchmark tables. IAU 1976/1980 equinox frames remain
the frame path (no CIO); NRLMSISE-00/HWM14 means only (no perturbed-atmosphere
decorator); WMM2025 remains the magnetic path (no IGRF-14, no gradient); SPK
DAF parser present. WP-08.2 is now **partial**: `ThirdBodyGravity` keeps its
existing API but evaluates perturbations with Battin's cancellation-free
`f(q)` form, and tests document the |r|≪|r_b| case where the naive difference
loses the small component (`REQ-ENV-002`). `SolarRadiationPressure<E>` adds
opt-in cannonball SRP with conical Earth shadow, and `RelativisticCorrection`
adds the Schwarzschild first-post-Newtonian acceleration with named `c`
(`REQ-ENV-003`). Missing for full WP-08.2: the Orekit code-to-code
Sun/Moon/SRP force-stack fixture and its ≤1 m/day secular growth evidence.
No tides, SRP macro-models, Lense-Thirring, or de Sitter terms yet. WP-08.3 …
WP-08.7: **not started**.

### 09 — Sensors, navigation & actuators

Baseline confirmed: specific-force truth is still finite-difference
(`crates/openbmp-runner/src/fc_bridge.rs`, velocity delta minus gravity);
GNSS is a position oracle; actuators first-order.
WP-09.1 … WP-09.12: **not started**.

### 10 — Flight-software-in-the-loop (XIL)

| WP | Status | Evidence / missing |
|---|---|---|
| WP-10.1 SIL over transport | **implemented** | `Transport` + in-process/stream/child-stdio backends, `LockstepSimMaster`, SHA-256 determinism gate, ZOH tolerance table, 12 tests in `crates/openbmp-runner/tests/fc_transport.rs` |
| WP-10.2 fault library + XIL ports | **implemented** | `fault.rs` (13 scalar + 5 bus transforms), `MaPort`/`EesPort`, lifecycle FSM, `[fc.transport_faults]`, empty-schedule byte-identity, `crates/openbmp-sil/src/xil.rs` evidence adapters |
| WP-10.3 soft-real-time bench | **implemented** | `openbmp-rt` pacer (FreeRun/RealTime/Paced, jitter histogram p50/p99/p99.9, overrun count), runner `rt.rs`, `[realtime]` schema, byte-identity across modes (`tests/realtime.rs`) |
| WP-10.4 FMI 3.0 importer | **partial** | typed instance access, modelDescription parsing, single+multi masters, 5 pinned Reference FMUs + FMPy cross-check scripts; missing: clocked event scheduling, predictor/corrector co-sim loop |
| WP-10.5 FMI 3.0 exporter | **partial** | `export.rs` + `export_abi.rs` (C ABI, XSD validation, point-mass FMU + trace tolerance table); missing: full plant mapping to value references |
| WP-10.6 AFTS containment monitor | **not started** | no `openbmp-afts`, no IIP propagator, no rule table |
| WP-10.7 PIL on emulated ISA | **not started** | no factored `fc_step` entry, no Renode coupling |
| WP-10.8 cFS in the loop | **not started** | no cFS wiring |

### 11 — Monte Carlo, UQ & validation

| WP | Status | Evidence / missing |
|---|---|---|
| WP-11.0 MC substrate | **implemented** | `openbmp-mc` (Welford, Clopper-Pearson, samples table + convergence trace), `DeterministicRng::for_mc_sample` domain |
| WP-11.1 DoE + sizing | **implemented** | LHS, native Sobol (pinned Joe-Kuo direction numbers under `data/sobol/` + provenance), Owen scrambling, Iman-Conover, Wilks sizing, convergence gate + R-hat, `openbmp mc` subcommands + `tests/mc_cli.rs` |
| WP-11.2 credibility + error budget | **partial** | `openbmp-uq` complete (correlated error budget, aleatory/epistemic classes, 8-factor credibility record, min-is-binding, floor flags wired into `mc summarize`/footprint/fault campaigns); missing: auto-emission by future drivers |
| WP-11.3 nested aleatory/epistemic | **partial** | probability-box + variance split, nested footprint execution, `mc nested-summarize`; missing: generalized nested orchestration beyond landing-footprint |
| WP-11.4 rare events | **implemented** | subset simulation (Au-Beck) + cross-entropy IS on sealed synthetic limit states; compile-fail tripwire `crates/openbmp-testkit/tests/limit_state_no_aimpoint_compile_fail.rs` + UI test |
| WP-11.5 MMS + order verification | **partial** | manufactured ODE + observed-order/Richardson/GCI (`openbmp verify-order`, testkit verification module); campaign-scale integration pending |
| WP-11.6 BET reconstruction | **partial** | RTS smoother + batch Gauss-Newton + NEES/NIS in `crates/openbmp-testkit/src/reconstruction.rs`; `openbmp reconstruct` CLI; pseudo-flight campaign + LOCAL workflow documentation pending |
| WP-11.7 cross-discipline UQ wiring | **not started** | upstream per-entry margins (03/04/05 decks) not yet flowing |
| WP-11.8 campaign determinism/real-time | **partial** | delivered through doc 12's reducers/checkpointing; ensemble byte-diff gate extension pending |

### 12 — Determinism, real-time & compute

| WP | Status | Evidence / missing |
|---|---|---|
| WP-12.0-a check-provenance CI | **implemented** | CI job runs `openbmp check-provenance` over `data/` + `scenarios/` |
| WP-12.0-b FP guard portable | **implemented** | `crates/openbmp-core/src/fp.rs` (x86_64 MXCSR + aarch64 FPCR), FMA ban in `.cargo/config.toml` |
| WP-12.0-c `for_mc_sample` | **implemented** | domain-tagged campaign stream in `rng.rs` |
| WP-12.1 order-independent reducer | **implemented** | Welford merge tree in `openbmp-mc`; serial fold == merge tree tests |
| WP-12.2-a parallel fan-out | **implemented** | worker-count-invariant campaigns (tests at 1/2/4/8 workers) |
| WP-12.2-b checkpoint/resume | **implemented** | `FileCheckpointStore`, resume byte-identity, footprint `--checkpoint-json` |
| WP-12.3-a frame pacer | **implemented** | `openbmp-rt` (shared with doc 10) |
| WP-12.3-b aarch64 CI lane | **implemented** | native `ubuntu-24.04-arm` `determinism-gate (aarch64)` downloads the same-run x86_64 reference artifact and byte-diffs the fixed-step canonical scenario outputs; traced by REQ-DET-004/V-DET-004 |
| WP-12.4-a dense output | **implemented** | `advance_with_dense_output` for Dopri54/853, state-stable opt-in, off the bit-stable path |
| WP-12.4-b GPU offload boundary | **not started** | no ingested-deck GPU pathway |

### 14 — Contact dynamics, touchdown & landing

WP-14.1: **partial** — `openbmp-contact` crate landed (`e4a1a4d`:
half-space geometry, Kelvin-Voigt/Hertz/Hunt-Crossley normal laws,
regularized Coulomb, fixed sub-step stability bound, energy audit;
REQ-CONTACT-001). Scenario `[contact]` schema (`ContactConfig`) and runner
force-adapter wiring are present (`crates/openbmp-runner/src/contact.rs` plus
point-mass/rigid-body/vehicle edits), with contact diagnostics telemetry,
`RunOutcome.contact` endpoint classification, run-level energy audit, and
substep-driven kernel step sizing, plus point-mass golden-stability proof.
Missing for full acceptance: gear-leg assemblies.
WP-14.2: **partial** — `openbmp-contact` now exposes one-sided scalar
stops, two-sided backlash gaps, and monotone latch primitives with validation,
unilateral force clamping, backlash dead-zone width evidence, Kelvin-Voigt
closed-form stop restitution evidence, and engage-once latch property tests
(`REQ-CONTACT-003`). Scenario/joint fixtures remain future work with the
articulated mechanism wiring.
WP-14.3: **implemented** — `openbmp-contact` now exposes anchored stick/slip
friction with static cone breakaway, kinetic sliding, Karnopp restick window,
deterministic anchor state updates, tangential anchor elastic-energy reporting,
incline stiction/sliding closed-form helpers, a drive-spring stick-slip
oscillator fixture matched against an independent ideal Karnopp reference, a
consecutive-hold rest detector, and Housner rocking-block threshold and period
anchors (`REQ-CONTACT-004`). Schema-v3 `[contact]` can now select
`friction_law = "anchored_stiction"` and the runner contact adapters route it
through time-gated anchor state; contact diagnostics now carry tangential
speed/sticking state and runner rest classification uses a kinetic-energy floor
plus consecutive sticking hold (`REQ-CONTACT-005`).
WP-14.4: **implemented** — `openbmp-vehicle` now exposes validated
`LandingGearLeg`, `OleoStage`, and `CrushCore` primitives with oleo
polytropic-force/energy tests and irreversible crush-core plateau-stroke plus
monotonicity tests (`REQ-CONTACT-006`). Schema-v3 rigid-body scenarios can
now declare `[vehicle.landing_gear]`; the runner wires the rack as force and
moment adapters, emits `force.landing_gear.{x,y,z}_n` plus per-leg load,
stroke, gap, crush, and contact telemetry, records a pinned synthetic gear data
SHA, and verifies the checked-in 3-D four-leg drop fixture
(`REQ-CONTACT-007`). `RunOutcome.landing_gear` now classifies the synthetic
drop as `Rest` under a consecutive quiet-speed hold and carries a deterministic
energy audit that closes to <1% on the fixture; the runner can recover body-x
section loads from final per-leg samples and the fixture cross-checks mid-body
shear/bending against independently read final front-leg telemetry. Each
runtime leg now owns a per-pad `ContactPair` in external-normal mode, so the
contact substrate owns pad geometry, gap/rate evaluation, opt-in regularized
friction, tangential-speed evidence, and friction-force contribution while the
oleo/crush strut supplies the normal load.
WP-14.5 … WP-14.10: **not started**.

### 15 — Plume environments & SRP

WP-15.1: **partial** — `openbmp-plume` now exists as an L2 crate with
coordinate-free `PlumeState` assembly inputs and closed-form similarity
diagnostics: NPR, exit-pressure ratio, `C_T`, momentum-flux ratio,
Prandtl-Meyer initial turn angle, reduced cluster-merge distance/flag, and
PIFS onset. Crate tests cover the Prandtl-Meyer closed form, adapted/
overexpanded zero-turn limits, analytic cone-overlap merge distance, and
state scalar assembly (`REQ-PLUME-001`). Schema v3 now accepts opt-in
`[aero.plume]`, and the point-mass solid-motor runner emits live plume
telemetry from chamber/nozzle state plus the sampled atmosphere
(`REQ-PLUME-002`). Rigid-body thermochemical liquid-engine scenarios now emit
the same plume telemetry from live engine mass-flow snapshots, sampled
atmosphere, and ideal-nozzle reconstruction (`REQ-PLUME-003`), with active
cluster spacing derived from engine mount points for merge telemetry
(`REQ-PLUME-004`). Point-mass and rigid-body canonical telemetry bytes are now
covered for the no-`[aero.plume]` path, including an otherwise empty `[aero]`
block (`REQ-PLUME-005`). The point-mass path now derives its single-engine
count, nozzle exit area, and zero center spacing from the solid motor instead
of the static compatibility fields (`REQ-PLUME-006`). Missing for WP
completion: rigid full geometry beyond active spacing and liquid-engine
calibration beyond the ideal-nozzle aggregate. The substrate
`momentum_flux_ratio` diagnostic now follows the documented total-thrust
similarity equation, including pressure thrust and reference area
(`REQ-PLUME-007`).

### 16 — Parachute, decelerator & recovery

T0 baseline confirmed (`crates/openbmp-vehicle/src/recovery/` drag-swap
models; `scenarios/parachute-recovery`). WP-16.1 … WP-16.11: **not
started**.

### 17 — Cryogenic fluid management

**Not started.** No property layer, no tank energy balance. (The
`openbmp-feedsystem` crate now exists as the design's default home for the
cryo module.)

### 18 — Ground segment, countdown & launch release

**Not started.** No `openbmp-ground`, no LCC engine, no GSE/T-0 models.

### 19 — Day-of-launch winds & commit operations

**Not started.** No measured-wind ingestion or redesign driver. WP-19.2
now has its WP-07.0 wired-corrector prerequisite; WP-19.1 is unblocked.

### 20 — Telemetry, RF links & ground network

WP-20.8 dictionary export: **partial** — a channel-dictionary export with
JSON and XTCE-shaped formats already ships (`openbmp dict`,
`crates/openbmp-cli/src/commands/dict.rs`); the CCSDS-shaped frame stream
and round-trip tests from the design are absent.
WP-20.1 … WP-20.7: **not started** (no `openbmp-comm`, no link physics).

### 21 — Run-data, regression & visualization

**Not started.** `openbmp mc` emits samples tables/convergence traces (doc
11 surface), but no run-record index, verdict rules, trends, booklets, or
CZML/glTF export exist.

### 22 — Acoustics, vibroacoustics & overpressure

**Not started.** No spectral types or DSM/IOP code.

### 23 — Electrical power & avionics emulation

**Not started.** No `openbmp-eps`, no battery/bus-profile/string-kill code.
(The bridge fault library from doc 10 is the substrate WP-23.5 will
decorate.)

### 24 — Postflight reconstruction & model correlation

**Not started.** The pre-existing `openbmp compare-telemetry` LOCAL
comparison is a seed for WP-24.1, and doc 11's RTS/GN/NEES-NIS substrate
landed — but no parameter registry, output-error estimator, identifiability
analysis, or governance pipeline exists.

### 25 — Rendezvous, proximity operations & docking

**Not started.** Prerequisites in place: multi-body propagation,
`RendezvousState` in the locked vocabulary, and the `openbmp-contact`
substrate. No LVLH/CW/TH machinery, rel-nav, prox-ops guidance, safety
verifier, or capture logic.

---

## 3. Cross-cutting locks & gates status

- **Terminal-condition vocabulary lock holds:** `TerminalCondition` carries
  orbital/flight-condition variants only; no range or aimpoint field
  exists.
- **New tripwire added with capability** (per `00` invariant 7): sealed
  `LimitState` + `limit_state_no_aimpoint_compile_fail.rs` UI test landed
  alongside the rare-event machinery.
- **Determinism toolchain hardened:** FP-environment guard (x86_64 +
  aarch64 code paths), FMA contraction ban, provenance check in CI; the
  aarch64 CI determinism lane is the one outstanding piece (WP-12.3-b).
- **Traceability:** 110 requirement ids in `requirements.toml`, including
  the REQ-PROP, REQ-MC, REQ-CONTACT, REQ-PLUME, and REQ-TRAJOPT families.
- **Byte-stable-by-default pattern observed** in everything that landed:
  pressure-thrust, transient grain, transports, realtime, contact, and
  fault schedules are all scenario-gated opt-ins.

---

## 4. Recommended next moves (from this snapshot)

1. **Close the open partials before opening new fronts:** WP-05.3 runtime
   deck consumption; WP-14.1 gear-leg closure; WP-12.4-b GPU boundary.
2. **Advance the Phase-A gates:** finish WP-01.1 (spatial-vector tree) and
   finish WP-08.1 (high-degree Pines/Gottlieb tesseral gravity) — they block
   most of Phase B/C (02, 06, 07 closed-loop quality).
3. Start WP-07.2 (Hermite-Simpson/SQP) or use the implemented WP-07.0/WP-07.1
   corrector stack as the prerequisite for WP-19.2.
4. WP-15.1 (plume state) is newly unblocked by 05's chamber/nozzle state.
