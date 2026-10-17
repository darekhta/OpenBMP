# Parity Program — Implementation Progress Ledger

**Status:** `experimental` (tracking document; ships no code).
**Snapshot:** 2026-06-28, current branch state. Verified by code inspection
against each design doc's §9 acceptance criteria and `requirements.toml`
traceability (356 requirement ids).

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
As of this snapshot: **68 implemented · 38 partial · 2 in progress ·
99 not started.**

**Working-tree additions (2026-06-28, uncommitted, tests green).** Two WPs moved
from `not started` to `in progress` this session, both additive/opt-in (no
golden-hash or default-path change):

- **WP-01.4 (slosh as a body)** — `openbmp-multibody::slosh` adds
  `PendulumSloshBody` -> revolute-pendulum `TreeBodySpec` with the Abramson
  `omega_n^2 = a / L_n` relation, validated integration-free (exact joint
  inertia `m L^2`, frequency formula, geometry/mass, multi-mode determinism; 5
  tests). The Abramson coefficient mapping already exists in
  `openbmp-vehicle::tank`. Remaining: runner tree wiring and moving-base
  dynamic validation (the forward dynamics it relies on are verified correct;
  see the WP-01.1 note below for the one open energy-accessor question).
- **WP-03.7 (stage-separation proximity deck)** — `openbmp-aerodb::stagesep`
  adds `ProximityInterferenceDeck`: per-body six-coefficient increment grid over
  (separation distance x Mach) with bilinear interpolation, a fail-closed
  runoff invariant (increment -> 0 at the far edge), superposition, and a
  panel-solver `build_from_pair` hook; condition-only (no aimpoint/range). The
  **panel-mesh increment generator now exists**: `build_from_panel_meshes`
  differences the affected body's six-coefficients with the other body present
  (proximity, via a combined-geometry shadow BVH) vs absent (isolated), backed
  by a new additive `openbmp-aero` method `solve_about_cg_with_shadow_bvh`
  (integrates loads over this body's panels against a caller-supplied external
  shadow BVH). 10 stagesep tests (incl. demonstrated cross-body shadowing and
  lateral-separation runoff). Note: the inviscid ray-shadow does not decay with
  *pure axial* separation (infinite ray), so the runoff invariant applies to
  separation paths that move bodies out of the shadow. **Provenance-pinned TOML
  ingestion** also added (`load_toml_str`, schema `openbmp.stagesep_proximity.v1`,
  guarded by the condition-only target/aimpoint/range dual-use lint). 13 tests.
  Remaining: scenario-block + runner staging consumption and Ares-I-class
  published validation.

**WP-01.1 dynamics — VERIFIED CORRECT; one OPEN/UNRESOLVED energy observation
(retraction of an earlier in-session "bug" claim).** A deep investigation this
session reached this conclusion:

- **Forward dynamics are correct.** `forward_dynamics_aba_at_state` was checked
  against a fully independent **maximal-coordinate constrained-Newton-Euler**
  reference `qdd` (built from scratch: per-body world-frame Newton-Euler + a
  5-row revolute constraint solved as a KKT system). They agree to **~1e-16** at
  multiple configurations, including a non-identity base orientation and a
  non-zero joint angle, and the crate `qdd` is exactly rotation-invariant in the
  base orientation (the correct gravity-free property). The reference itself
  conserves kinetic energy by construction (`dKE/dt ~ -8e-16` from its
  accelerations). ABA also matches `forward_dynamics_dense_at_state` to ~7e-18.
  So the articulated forward dynamics — including the velocity-bias `c`-term —
  are **not** defective.

- **One unresolved inconsistency.** `kinetic_energy_at_state` (the CRBA
  `0.5 qd^T H qd` accessor) is **not conserved** over finite-rate integration of
  a free-flyer-root + revolute-child system (step-size independent drift, ~few %
  per second). Because the instantaneous `qdd` is verified correct, this points
  to the **energy-accessor / integration coupling**, not the forward dynamics.
  Root cause was **not** determined.

- **Caveat (don't-overstate):** the diagnostic harness used to chase this had
  multiple convention bugs of its own (two `Rz(theta)` sign errors were found
  and fixed mid-investigation), so the residual energy-drift observation should
  be **independently reproduced and re-derived before being treated as a crate
  defect.** An earlier in-session claim that the floating-base *dynamics* had a
  velocity-bias sign bug is **retracted**: a c-term sign flip "fixed" energy but
  broke momentum, and the dynamics were subsequently proven correct against the
  independent reference — that earlier signal was a harness/accessor artifact.

- **Status:** core dynamics were left **unchanged** (no fix shipped, none
  warranted on current evidence). The `#[ignore]`d test
  `floating_base_articulated_dynamics_conserve_energy_at_finite_rate` documents
  the open energy-accessor observation (energy + momentum conservation over a
  finite-rate integration); it is reworded to state the dynamics are verified
  correct and the failure is unconfirmed/unresolved.

Net effect on doc `01`: the dynamics substrate is in better shape than the
earlier note implied — WP-01.4/01.2/01.3 are **not** blocked by a confirmed
dynamics bug. The open item is to determine whether the energy-accessor
non-conservation is a real `kinetic_energy_at_state`/integration issue or a
measurement artifact, with an independent (non-harness) check.

| Doc | Dimension | Position on its tier ladder | WPs impl/partial/total | Next unblocked WP |
|---|---|---|---|---|
| 01 | Multibody dynamics | T0 plus WP-01.1 spatial-vector/CRBA/RNEA/dense-FD/ABA/state-integration substrate crate with independent Python RBDA oracle evidence; simulator wiring and true Spatial_v2 executable oracle remain open | 0/1/6 | finish WP-01.1 |
| 02 | Structural, loads, slosh, POGO | T1 multi-mode lateral modal substrate, opt-in mode-summed reaction feedback, cylindrical Abramson fixed/slosh mass split, CSV-v1 external modal asset ingestion, station line-load recovery substrate, FC-portable CSI margin/notch/modal-assembly/filter-insertion plus synthetic SIL boundedness evidence, and method-level structural POGO feedline-matrix/sampled-margin/reduced-eigen plus scenario-facing report paths implemented; CB/Newmark/gen-alpha native CLA substrate partial and beam/shell-FEM/event-force substrate partial remain open, now with closed CLA event-class fixture evidence for liftoff/max-Q/ignition/separation/MECO templates | 7/2/9 | finish WP-02.3-a or WP-02.3-b |
| 03 | Aero database & CFD | T0 baseline plus implemented WP-03.1 local-inclination panel/CG moment/line-load/STL/BVH substrate with analytic sphere/cone, wedge-cavity, and public HB-2 Figure 12b high-Mach forebody-CA evidence; implemented WP-03.2 reference-temperature skin friction with analytic Cf, synthetic sharp-cone trend, and public HB-2 Figure 12b high-Mach skin-friction total-CA evidence; implemented WP-03.4 `openbmp-aerodb` run-matrix/deck-builder and recorded-solver ingestion/orchestration skeleton; implemented WP-03.6 reduced/six-channel sigma, GCI, correlated deck draw, scenario/runner hook, generated campaign execution, and UQ credibility path; partial WP-03.3 DATCOM-style buildup and WP-03.5 attached-flow VLM/grid-fin substrate remain open; public DATCOM validation, AVL VLM code-to-code closure, and later WP-03.8 solver orchestration/field parsing remain open | 4/2/9 | finish WP-03.3 or WP-03.5 |
| 04 | Aerothermal, real-gas, TPS | T0 plus implemented WP-04.2-a shared `openbmp-thermochem` L2 scaffold with FC-tripwire isolation, partial WP-04.1-a off-default Tauber-Sutton Earth-air table/radiative/total-heating substrate with schema/runtime `q_rad` telemetry, and WP-04.1-b Sutton-Graves TR R-376 gas coefficient presets, DKR stagnation-heating cross-check, smooth-wall Re_θ/M_e onset/ramp evidence, PANT/Reda rough-wall transition selector with provenance-pinned checked thresholds, and analytic sharp-cone transition-location fixture implemented; Apollo/FIRE radiative validation, real-gas, and TPS solver work still open | 2/1/11 | finish WP-04.1-a validation |
| 05 | Propulsion high-fidelity | **T1+T2+T3+T4 implemented**; T5 deterministic POGO feed-half and fault-library paths implemented with calibrated vehicle-specific POGO margins retained as validation ceilings | 9/0/9 | keep propulsion gates green |
| 06 | Coupled MIMO GNC | T0 plus WP-06.1 dense-B WLS allocator, WP-06.2-a dense DARE/LQR oracle, WP-06.2-b scalar coupled-gain scheduling/LQG observer substrate, WP-06.2-c structural-filter prewarped-Tustin/table/margin/runtime-insertion substrate, WP-06.3-a load-relief alpha/beta/q-alpha/wind/schema/runtime path implemented with QCAT/no-target evidence, coupled-gain and bending-filter tolerance evidence, python-control/SciPy oracle evidence, off-default runtime insertion, focused Dryden/von-Karman controller-SIL q-alpha trend evidence, and local §2 merge-sweep evidence through the prior batch; WP-06.3-b dense-INCA/PCH plus v3/runner-wired optional LiquidEngine TVC/TWD plant substrates are partial with scalar INDI live-SIL reaction recovery, active FC dense-INCA allocation, live dense-INCA structural/TWD residual reduction, nominal physical TVC-gimbal dense allocation, and failed-engine TVC-gimbal column masking covered; WP-06.4-c allocation-infeasible FDIR flagging plus commander `health.degraded` mapping, confirmation counters, SAFE thrust-termination engine command substrate, no-ground-redirect SAFE compile-fail guard, propellant-safing intent topic, runner feed/budget safing consumption including simplified pressurization-disable report consumption, FC target-free SAFE attitude-hold substrate, focused FC FDIR→SAFE pipeline validation, focused FcRunner SAFE output validation, focused rigid-body mission SAFE shutdown/recovery validation, scalar engine-out throttle retrim, calibrated scalar fan-out engine-out trim-bias table, geometry-derived scalar engine-out trim auto table substrate, tank-rack SAFE drain-disable latching, pressurization-disable feed fail-closed behavior, configured constant-rate SAFE dump path, pressure-coupled SAFE dump orifice sizing substrate, choked ullage-gas SAFE vent substrate, opt-in adiabatic and wall-relaxed ullage-gas SAFE vent thermal substrates, deterministic SAFE dump backpressure-ramp substrate, ambient-atmosphere SAFE dump backpressure source, and finite-volume tank pressurization substrate are partial, while tight nav plus flight-calibrated integrator bias estimation, cryogenic phase-change, flight-calibrated regulator/bottle thermal coupling, plume-coupled vent backpressure, and production pressurant hardware validation remain open | 5/2/10 | finish WP-06.3-b or WP-06.4-c |
| 07 | Trajectory optimization | **T0-T1 and WP-07.4 LCvx implemented**; WP-07.2 Hermite-Simpson/SQP is partial with dense-QP/SQP wrappers, sparse Jacobian/KKT diagnostics, active constrained-KKT step and diagnostic handoff evidence, residual-only sparse-KKT SQP scalar/nonlinear hard-path feasibility, optional final active constrained-KKT report, final-multiplier evidence, optimized-reference/I-load output, public toy/benchmark evidence, and a public sparse-NLP backend contract/preflight substrate plus bounded dense-SQP adapter; WP-07.3 LGR/ph-refinement is partial with residual, objective, hard/nonlinear path, sparse-Jacobian, active constrained-KKT step and diagnostic handoff evidence, residual-only sparse-KKT SQP scalar/nonlinear hard-path feasibility, optional final active constrained-KKT report, and final-multiplier, dense-solver, optimized-reference, refinement, scalar-integrator, brachistochrone transcription and dense-SQP feasibility recovery, Bryson-Denham active-arc dense-QP plus sparse constrained-KKT handoff evidence, and COPS/YAPSS Goddard dynamics/singular-arc/fixed-mesh seed, residual-localized split-refinement seeding, plus fixed-mesh and split-refined terminal-objective evidence; WP-07.4-a shared conic plumbing is the satisfied enabler counted with WP-07.4; WP-07.5 is partial with a discrete-affine plus forward-Euler-linearized SCvx virtual-control/trust-region/state-triggered-constraint and always-active linear state/control and second-order-cone path-constraint substrate, FOH problem builders for generic continuous dynamics, flat 13-state/6-control rigid-body dynamics, 14-state mass-depleting body-thrust powered-landing references with provenance-pinned Szmuk-Acikmese constants, envelope reports plus linear, reference-linearized, and conic envelope solver rows, pinned Szmuk full-budget conic-row affine-solve terminal-closure and virtual-gap evidence, one-window conic-row free-final-time sweep virtual-gap diagnostics, pinned Szmuk affine-solve, free-final-time search, and published initial-time sweep gap diagnostics with per-candidate/window terminal/virtual-control/state/control-path/cone rows, exported quaternion-projection, continuous-FOH projection-hook relinearization handoff evidence, and continuous-FOH full-reference conic-row refresh and virtual-gap reduction handoff evidence, and validated-toy continuous-FOH powered-landing solve evidence, reduced schema-1-style aero-coupled rigid-body and powered-landing references, and optional `openbmp-aero::AeroMethod`-coupled rigid-body and powered-landing references, plus deterministic free-final-time search, continuous-FOH time search, and sweep wrappers; WP-07.6 is partial with a generic PMP Hamiltonian/costate/control-stationarity witness, scalar min-energy costate-shooting evidence, scalar dense-QP equality-dual/costate convention evidence, explicit HS/LGR raw-costate convention adapters, solver-produced HS scalar-PMP-matched and LGR quadrature-normalized dense-QP dual handoff into those adapters, sparse augmented-KKT and constrained-KKT PMP costate/Hamiltonian wiring including public Bryson-Denham active-set diagnostics, direct constrained-KKT certificate I-load handoff, and LGR quadrature normalization, residual-only sparse-KKT SQP scalar/nonlinear hard-path feasibility, optional final active constrained-KKT report, final-multiplier PMP/KKT evidence, sparse-SQP PMP/KKT certificate payload round-trips with validated costate-comparison and control-stationarity summaries, and inline plus scalar-PMP, dense-SQP-certified, and sparse-SQP-certified file-backed optimized-reference closed-loop runner scenarios with auditable I-load certificates, HS/LGR payload-certificate surfaces, and dense-SQP plus sparse-SQP final sparse-KKT certificate round-trips with optional validated costate-comparison and control-stationarity summaries. Production constrained large-sparse NLP backend adapters, general LGR nonlinear benchmark closure, full Goddard nonlinear solver/final-altitude closure, full published Szmuk SCvx trajectory closure, and production HS/LGR constrained large-sparse-NLP PMP/I-load optimality closure remain open. | 3/4/7 | finish WP-07.2, WP-07.3, WP-07.5, or WP-07.6 |
| 08 | Environment, gravity, frames | T0 plus WP-08.3 time-scale/EOP ingestion implemented; WP-08.1 degree-2 tesseral substrate, frame-coupled `EarthFixedGravity`, v3 tesseral selector, and degree-10 ICGEM-backed EGM2008 Pines transition runtime path partial; WP-08.2 Battin third-body/SRP/Schwarzschild analytic pieces and WP-08.4 CIO matrix primitives partial; full high-degree Pines/Gottlieb and Orekit force-stack validation still open | 1/3/7 | finish WP-08.1, WP-08.2, or WP-08.4 |
| 09 | Sensors, nav, actuators | T0 plus partial WP-09.1 opt-in force-accumulator IMU truth and stationary rotating-frame runner validation source; WP-09.2 selectable second-order servo, deterministic oscillatory actuator faults, direct-torque RCS MIB/PWPF runtime path with scalar, bank/blowdown, coupled-bank multi-axis schema/runner modes, and direct tank feed pressure/drain coupling; WP-09.4 strapdown increment and two-sample coning/sculling substrate; finite-difference remains default | 0/3/12 | finish WP-09.1, WP-09.2, or WP-09.4 |
| 10 | Flight SW in the loop (XIL) | **T1–T3 implemented**; T4 partial; T5 entry partial; A0 partial | 3/4/7 | finish WP-10.4, or WP-10.7 Renode coupling |
| 11 | Monte Carlo, UQ, validation | **T0–T1 implemented**; T2–T4 partial with WP-11.5 campaign-scale code verification closed | 4/4/9 | WP-11.7 |
| 12 | Determinism, real-time, compute | **implemented through T4**; Linux aarch64 bit-stable CI lane plus offline GPU-deck boundary present | 10/0/10 | keep determinism gates green |
| 14 | Contact, touchdown, landing | WP-14.1 half-space substrate + runner force/diagnostic path implemented; WP-14.2 crate primitives and current-path planar four-bar CFE row reachability plus opt-in prismatic gear-DOF coordinate telemetry/mechanism binding landed; WP-14.3 implemented; WP-14.4 massless oleo/crush gear with per-pad ContactPair footpads implemented | 3/1/10 | WP-14.5 after WP-09.7, or WP-14.7 after terrain substrate |
| 15 | Plume & SRP | WP-15.1 L2 substrate plus point-mass solid-motor and rigid-body thermochemical liquid telemetry with documented MFR, point-mass nozzle-derived geometry, active mount-derived spacing, and off-byte-goldens landed | 0/1/12 | finish WP-15.1 liquid calibration/rigid full geometry |
| 16 | Parachute & recovery | T0 baseline plus WP-16.1 partial single-stage and drogue/main staged inflation/ramped-area substrate with explicit/derived fill time, optional active-stage Pflanz finite-mass peak-load telemetry, and opt-in scalar apparent-mass mass/force/telemetry wiring; WP-16.2 partial adapter-level wind-relative recovery drag validation through the shared environment sample; WP-16.3 partial slack-taut riser line-element primitive with fixed-tick event locator, schema declaration, runner constant-velocity, per-riser force-integrated endpoint advance, and shared translated canopy-state integration, aggregate riser telemetry, aggregate line-force consumption, live `C_D A` drag handoff for riser-declared devices, and rigid moment consumption; WP-16.4 partial opt-in sequence snatch-load diagnostic from declared riser stiffness and effective mass plus one-tick mortar reaction impulse force path | 0/4/11 | finish WP-16.1, WP-16.2, WP-16.3, or WP-16.4 |
| 17 | Cryogenic fluid management | not started | 0/0/8 | WP-17.1 |
| 18 | Ground segment & countdown | WP-18.1 L7 `openbmp-ground` crate plus deterministic countdown DAG/count-clock/hold substrate, schema-v3 `[countdown]` front door, runner-side countdown report, and gated `ground.countdown.*` summary telemetry partial; event injection, LCC predicates, GSE/T-0, release, clearance, recycle, and rehearsal layers still open | 0/1/8 | finish WP-18.1 event injection |
| 19 | Day-of-launch winds & commit | WP-19.1 measured-wind ingestion/QC/splice/filter/synthetic-family front door implemented; WP-19.2 T2 spliced-wind I-load recentering artifact/CLI and terminal-vocabulary closure guard implemented; WP-19.3 degraded q-alpha/q-beta/gimbal load-indicator surrogate and drift guard implemented; WP-19.4 report-card/latch substrate plus runner/SIL telemetry indicator extraction, updated-I-load byte digest binding, pitch/yaw I-load-applied native-runner report generation, SIL package stimulation/evidence verifier, and DOL-level commit-readiness latch/CLI handoff partial; WP-19.5 synthetic persistence RSS knockdown substrate partial; WP-19.6 synthetic geometric launch-COLA screening substrate partial | 3/3/6 | finish operational WP-19.4/WP-19.5/WP-19.6 handoff |
| 20 | Telemetry, RF links, network | WP-20.1 comm geometry/pass substrate implemented; WP-20.2 antenna/body-mask deck substrate implemented with runner budget use, watertight panel-mesh BVH ray masks, and panel-mesh-derived mask regeneration; WP-20.3 link-budget/FER substrate implemented with LinkState telemetry and no-comm off-default hash evidence; WP-20.4 link-channel packet-effect substrate implemented with FC transport fail-closed binding, latency queues, best-margin bridge-link selection with hysteresis, data-loss timeout hold/terminate verdict evidence, and zero-effect SIL determinism hash evidence; WP-20.5 declared electron-density plasma blackout profiles plus live aerothermal proxy coupling partial; WP-20.6 declared bridge pass-plan handover, sampled/interpolated per-link pass reports, declared relay availability windows, and declared GEO relay two-hop budgets implemented; WP-20.7 vehicle-bound tracking observable reports with deterministic white-noise/Gauss-Markov instrument decks, CSV stream export, compile-fail API tripwire, and synthetic BET/CRLB validation implemented; WP-20.8 feature-gated telemetry frame/dictionary export implemented | 7/1/8 | finish WP-20.5 real edge-state/RAM-C validation |
| 21 | Run-data, regression, visualization | WP-21.1 file-index substrate implemented; WP-21.2 verdict-rule engine implemented; WP-21.3 baseline-band and out-of-family substrate implemented with deterministic median/MAD and percentile-envelope artifacts, JSONL-index generation, doc-18 predicate consumption, and synthetic rate evidence; WP-21.4 trend store, drift gates, scoreboard golden, and CI fail-on-drift hook implemented; WP-21.5 deterministic SVG plots, spike hunt, run-diff residuals, and booklet assembly implemented; WP-21.6 deterministic CZML/glTF export with runner-backed multi-body tracks and pinned upstream schema validation implemented; WP-21.7 Open MCT catalog/replay JSON substrate plus feature-gated host example implemented; WP-21.8 campaign manifest planner plus worker-backed overlay/trend/verdict executor implemented | 8/0/8 | keep run-data gates green |
| 22 | Acoustics & overpressure | WP-22.1 acoustics crate and deterministic spectral machinery implemented; WP-22.2 free-field single-engine DSM substrate plus schema-v3 runner report, gated summary telemetry, Schema-1 DSM deck parser, deck-backed runner path, synthetic per-band absorption deck path with named atmosphere-profile selection, coarse SP-8072 Figure 5/10/12/13/14 runtime curve paths, and coarse SP-8072 curve tolerance-table substrate partial, lacking production-quality SP-8072 spectrum/directivity/source/axial-spectrum/apparent-source decks, production/public atmospheric-attenuation validation, and public worked-example reference-value validation; WP-22.3 synthetic pad suppression and per-band cluster-interaction deck parsers/scenario/runner hooks, exact identical-engine acoustic-power scaling, geometric engine-offset summation, declared pad path-length propagation term, declared single-bend plume-axis geometry, and scalar cluster coherence/shielding correction partial, lacking calibrated duct/trench factors, production pad-deflector geometry, calibrated plume interaction/specific directivity, published suppression data, and liftoff signature anchoring; WP-22.4 core IOP pulse primitives plus schema-v3 runner report/telemetry, declared liquid-engine source coupling with thermochemistry-derived `mdot*c*` source-rate coupling, declared ignite-command timing evidence, point-mass/rigid-body runtime-observed ignite timing evidence, deterministic runtime source-history reconstruction, and runtime-reconstructed envelope evidence, synthetic duct/trench deck path, and synthetic suppression-deck path partial, lacking calibrated/public start-rate reconstruction, calibrated/public duct and suppression decks, and published trend anchoring; WP-22.5 synthetic ascent acoustic class deck parser/evaluator, deterministic Mach-gate class selection, multi-sample ascent trace enveloping, synthetic fairing-internal insertion-loss/fill-factor deck parser/evaluator, and schema-v3 runner report/telemetry partial, lacking full `03` geometry-driven regime/zone selection, published MPE/user-guide curves, payload-zone spatial maps, and flight/guide residual anchoring; WP-22.6 synthetic vibroacoustic response transfer deck parser/evaluator, schema-v3 runner report/telemetry, acceleration-PSD surface-mass scaling, modal-force PSD handoff, fairing/response-side per-sample P95/50 input rows and sample-trace evidence, deterministic response-side MC sample materialization and response-campaign scenario overlay, structural loads handoff CSV artifact, deterministic MC-designed ascent trace materialization, first checkpointed/worker-backed executable MC ascent-acoustics campaign runner with downstream fairing/response P95 summary reporting, optional evaluated artifact-directory materialization, and a response-focused acoustics-response campaign wrapper, first-bending structural acoustic consumer report with first-increment engine-station line-load recovery, normal P95/50 SPL statistics substrate, ascent/fairing/response trace P95/50 table artifacts, P95/50 Wilks sizing evidence, and off-by-default `21` run-data storage/indexing for nonempty P95/50, handoff, structural-acoustics response, and structural-acoustics line-load recovery CSV artifacts partial, lacking generated fairing design-variable CLI samplers, generalized checkpointed/parallel MC campaign orchestration beyond ascent-trace-driven acoustic commands, calibrated transfer evidence, and broader `02` multi-mode/FEM-derived LTM/OTM structural-stack integration beyond the first bending-mode out-of-band consumer | 1/4/7 | finish WP-22.2 |
| 23 | EPS & avionics emulation | not started | 0/0/6 | WP-23.1 |
| 24 | Postflight reconstruction & sys-ID | WP-24.1 file-fed `openbmp-sysid` residual harness implemented; parameter registry, output-error estimation, identifiability, and governance pipeline still open | 1/0/7 | WP-24.2 |
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
generalized momentum within strict tolerances plus 10,000 s RK4 free-root
double-pendulum relative energy-drift evidence below `1e-6`
(`REQ-MULTIBODY-028`).
The state-integration substrate adds `MultibodyDerivative`,
`advance_state_by()`, `project_state()`, `scalar_state_size()`, and
`weighted_error_norm()` with derivative arithmetic, quaternion projection,
scalar norm, and adaptive-error checks (`REQ-MULTIBODY-006`). The
unreleased welded-subtree handoff now exposes
`release_welded_subtree_as_free_flyer()`, remapping a selected welded subtree
into a deterministic free-flyer tree while preserving the released body's pose,
body-frame spatial velocity, and descendant joint state (`REQ-MULTIBODY-027`).
The multibody crate also exposes `system_spatial_momentum_at_state()` and
`subtree_spatial_momentum_at_state()`, accumulating body spatial momentum into
the virtual root-parent frame and proving the welded-subtree release preserves
the selected subtree momentum when re-seeded as a free-flyer root
(`REQ-MULTIBODY-052`).
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
(`REQ-MULTIBODY-031`). Schema-v3 `[[multi_body.gimbal_joint]]` declarations
now validate owned engine/body references plus declared engine inertial
properties, and the rigid runner materializes each declaration into a root
free-flyer plus revolute-engine multibody tree during session preparation
(`REQ-MULTIBODY-032`). The session-stepped rigid runner now also refreshes a
non-authoritative articulated shadow for each declared gimbal from live parent
mass properties and engine thrust snapshots, applying thrust at
`thrust_application_body_m` through ABA and proving powered root/gimbal
acceleration; a two-engine default-authority regression now also proves separate
shadows are materialized and stepped for multiple primary-body gimbal
declarations in one tick (`REQ-MULTIBODY-033`), and derives the one-axis
child-from-parent joint coordinate plus finite-difference rate from the live
thrust direction and `neutral_thrust_body` (`REQ-MULTIBODY-034`). The schema
and rigid runner now
also accept a right-handed `secondary_axis_body`, build a massless intermediate
gimbal frame, decompose live pitch/yaw thrust into secondary and primary
child-from-parent coordinates/rates, and prove the two-axis articulated shadow
derivative carries powered root and gimbal accelerations (`REQ-MULTIBODY-035`).
Articulated gimbal shadows now also store a non-authoritative one-step RK4
forecast over the articulated tree under held live thrust, with the two-axis
regression proving the forecast advances one session step and changes gimbal
coordinates/rates (`REQ-MULTIBODY-036`).
The same forecast path now projects articulated root slots back into
`RigidBodyState`, preserving parent mass properties and proving powered
translational and angular root response (`REQ-MULTIBODY-037`).
The multibody crate now also ingests a provenance-bearing
Spatial_v2-compatible ABA oracle fixture matrix for three deterministic
sample-tree cases and compares ABA accelerations against checked static vectors
(`REQ-MULTIBODY-038`); the matrix is explicitly labeled self-consistency, so it
does not claim an independent Featherstone Spatial_v2 run.
It also ingests a generator-backed oracle matrix from
`scripts/generate_spatial_v2_independent_aba_oracle.py`, a NumPy
RBDA/Spatial_v2-convention ABA implementation outside the Rust runtime. The
committed self-describing fixture covers a chain, a branched free-flyer tree, a
free-flyer/spherical/welded/revolute chain, and four seeded randomized
small-tree cases; the crate builds each Rust tree from fixture-declared body
metadata, compares ABA accelerations against that fixture, and the verification
path regenerates it byte-for-byte (`REQ-MULTIBODY-067`). This is
independent-generator evidence, not a MATLAB/Octave Featherstone Spatial_v2
executable run.
The scenario schema and rigid runner now also expose guarded
`propagation_authority = "primary_root_free_flyer"` mode, which rejects
separated/articulated combinations, replaces the primary rigid state with the
root-free-flyer multibody forecast after each fixed-step RK4 tick, and records
the applied handoff (`REQ-MULTIBODY-039`).
The same authority selector now also supports
`propagation_authority = "separated_root_free_flyer"` for active separated
root-free-flyer lanes, replacing both initially separated lanes and
jettison-created lanes after a pre-step forecast exists while keeping
articulated gimbal joints outside that authority mode (`REQ-MULTIBODY-040`).
It now also supports `propagation_authority = "articulated_gimbal_root"` for
primary-body one-axis or two-axis gimbals, replacing the primary rigid state
with the projected articulated root forecast and recording the handoff. Multiple
primary-body gimbals without CFE constraints now use one coupled free-flyer tree
with all declared gimbal children as the root handoff source
(`REQ-MULTIBODY-041`). After the first authoritative handoff, subsequent
pre-step mirrors carry the prior articulated forecast's joint coordinates and
rates into the next authoritative seed while refreshing root slots from the
rigid state when live-inferred gimbal deflection is unchanged; a newly delivered
live deflection refreshes the articulated joint slots from delivered TVC
gimbal angle/rate snapshots for standard axes, from live thrust direction for
powered nonstandard axes, or from a snapshot-derived unit thrust direction for
zero-thrust nonstandard axes before the next forecast (`REQ-MULTIBODY-042`). A repeated-step two-axis
authority regression
now advances four powered ticks, proving each projected root handoff matches
the post-step rigid state while carried two-axis coordinates/rates remain
finite and keep changing (`REQ-MULTIBODY-064`).
The single-gimbal authority path now also carries scenario-bound scalar contact
mechanism effects: a stop mechanism contributes its generalized force to the
authoritative forecast and an already-engaged monotone latch state remains
carried with the internal articulated state, while a latch centered on a newly
delivered live deflection engages after root-authority coordinate refresh
(`REQ-CONTACT-003`).
The simulation kernel now exposes `RigidBodySeparationStates` and
`jettison_rigid_body_with_states()` as the explicit-state installation seam for
externally computed welded-release handoffs, validating same-time continuing
and departing states before exact installation (`REQ-MULTIBODY-043`).
The rigid runner now also exposes
`propagation_authority = "welded_release_jettison"` for guarded configured
welded-pair jettison, computes each departing lower-body free-flyer state
through `release_welded_subtree_as_free_flyer()`, supports declared dry welded
descendants under the departing root, installs single and batch same-time
release states through `jettison_rigid_body_with_states()`, applies configured
welded translation/orientation, dry descendant mass aggregation, separation
delta-v, angular-rate tip-off, departing attitude offsets, and mass-property
COM offsets, and rejects initial lanes and gimbal joints (`REQ-MULTIBODY-044`).
That runner path now audits the pre-projection release handoff by comparing the
selected subtree spatial momentum against the released free-flyer tree's system
spatial momentum and fails closed on non-finite or out-of-tolerance residuals
(`REQ-MULTIBODY-053`).
It also aliases `direct_torque` effectors mounted to welded descendants onto the
departing root lane and transforms their declared torque axes through the full
welded-orientation chain into the root body frame (`REQ-MULTIBODY-054`).
The welded-release path now also aliases self-contained descendant-mounted
liquid engines onto the departing root lane, transforms child-frame thrust and
mount points through the welded translation/orientation chain, while same-root
tank-coupled bindings are handled separately by `REQ-MULTIBODY-060` and
transformed recovery-frame risers/sequence are handled separately by
`REQ-MULTIBODY-062` (`REQ-MULTIBODY-055`).
It also aliases standalone descendant-mounted tanks onto the departing root
lane, transforms child-frame tank reaction force/moment through the welded
translation/orientation chain, and transforms tank CG/inertia contributions into
the departing root body frame while keeping unsupported descendant resources
fail-closed (`REQ-MULTIBODY-056`).
It also aliases scalar descendant-mounted recovery devices onto the departing
root lane for recovery drag ownership and scalar apparent-mass routing, with
frame-dependent riser/sequence routing handled separately by
`REQ-MULTIBODY-062` (`REQ-MULTIBODY-057`).
It also aliases descendant-mounted aero decks and `AeroMethod` resources onto
the departing root lane, transforms root-body air-relative velocity into the
aero mounted-body frame, rotates aero force back into the root body frame, and
shifts aero-method moments by the welded descendant translation crossed with
transformed force (`REQ-MULTIBODY-058`).
It also aliases descendant-mounted solid motors onto the departing root lane for
thrust and scalar motor mass/mass-rate routing, and transforms child-body +Z
motor thrust into the root body frame before ECI rotation
(`REQ-MULTIBODY-059`).
It also permits same-root tank-coupled descendant engine/tank bindings, reusing
welded engine/tank owner aliases and transforms while treating tanks as the
absolute propellant-mass owner so engine `consumed_kg` is not subtracted twice
(`REQ-MULTIBODY-060`).
It also aliases massless descendant-mounted landing-gear legs onto the departing
root lane and transforms their hardpoint/strut-axis geometry into that root body
frame before landing-gear contact force and moment evaluation
(`REQ-MULTIBODY-061`).
It also aliases descendant recovery devices with declared risers/sequence to the
departing root lane and transforms riser anchor/canopy geometry, velocities,
shared-canopy state, and sequence mortar velocity into that root body frame
before recovery-riser and sequence force snapshots are evaluated
(`REQ-MULTIBODY-062`).
It also permits descendant-mounted `linear_actuator` and
`second_order_servo` scalar effectors under a welded-release alias and
preserves the existing schema-2 aero-deck axis snapshot binding so
already-routed descendant aero can consume those deflections
(`REQ-MULTIBODY-063`).
It also permits descendant direct-torque RCS bank effectors with same-root feed
tanks, proving the rotated direct-torque output reaches the departing root lane
while the payload-mounted tank state drains under RCS pulse demand and the
departed root mass decreases with the routed tank mass rate
(`REQ-MULTIBODY-065`).
It also permits descendant direct-torque RCS coupled-bank groups with roll,
pitch, and yaw members on the same welded descendant, proving all three
coupled-bank actuals remain observable, route through the welded-frame
direct-torque aliases into all three departed-root angular-rate axes, drain the
payload-mounted feed tank, and decrease departed-root mass with the routed tank
mass rate (`REQ-MULTIBODY-066`).
The multibody crate now also exposes `CfeLoopAssembly` plus
`assemble_cfe_loop_terms_at_state()`, which consumes the current ABA free
acceleration path and state-dependent CRBA matrix to assemble
`G*qdd_free + Gdot*qdot` and `G*H(q)^-1*G^T` row-space terms for the contact CFE
solver boundary (`REQ-MULTIBODY-045`). It also exposes system/subtree
spatial-momentum helpers and a welded-release subtree-momentum conservation
regression (`REQ-MULTIBODY-052`). Schema-v3
`[[multi_body.cfe_loop]]` declarations now make that row-space boundary
scenario-reachable as validated parser fixtures with finite residuals, unique
loop ids, and sparse non-zero Jacobian rows (`REQ-MULTIBODY-046`). The rigid
runner now consumes those rows for the current single articulated-gimbal shadow
path by solving a Baumgarte tree CFE system and injecting `G^T*lambda` into the
non-authoritative derivative/forecast (`REQ-MULTIBODY-047`). The same path now
emits per-loop CFE solve telemetry for the multiplier, stabilized acceleration,
and acceleration residual (`REQ-MULTIBODY-048`). When the guarded
`articulated_gimbal_root` authority mode is enabled, the CFE-constrained
forecast is the single-gimbal root handoff source (`REQ-MULTIBODY-049`), but
scenario-declared vehicle loop-closure wiring remains open. The L2 contact
crate now provides concrete planar four-bar/side-brace CFE row geometry
(`REQ-CONTACT-009`) for a later tree-facing adapter.
Missing for full WP-01.1:
replacing the rigid-kernel propagation path beyond the guarded primary-root
handoff, separated-lane root handoff, single-gimbal articulated
root/internal-state handoff, and no-CFE coupled multi-gimbal root handoff,
broader variable-mass multibody propagation beyond
the primary-root and powered separated-lane root-free-flyer paths,
broader dynamic/resource-owning subtree release beyond transformed direct-torque,
self-contained engine, standalone-tank, scalar-recovery, transformed-aero, and
transformed-solid-motor, same-root tank-coupled engine/tank, and transformed
landing-gear, recovery-riser/sequence, scalar-aero-effector aliasing, and
direct-torque RCS bank/coupled-bank feed-tank mass-rate routing, full
orbital-ascent gimballed validation, broader authoritative articulated thrust
propagation beyond the current single-gimbal multi-step two-axis and no-CFE
coupled multi-gimbal carry paths, including custom propulsion gimbal-frame
conventions and validated full tail-wags-dog benchmark/coupled-acceleration
closure,
and a true
MATLAB/Octave Featherstone Spatial_v2 executable oracle run across randomized
small trees. The in-repo independent Python
RBDA/Spatial_v2-convention oracle matrix now covers three deterministic
topologies plus four seeded randomized small-tree cases, but it is
intentionally not claimed as an executable Spatial_v2 run.
WP-01.2 … WP-01.6: **not started**.

### 02 — Structural dynamics, loads, slosh & POGO

WP-02.1-a: **implemented** (`REQ-STRUCT-001`/`V-STRUCT-001`). The vehicle crate
now exposes declaration-ordered `ModalBlock` / `ModeParams` lateral bending
with the same semi-implicit-Euler arithmetic as legacy `BendingMode`, N=1
legacy equivalence tests, per-mode frequency/damping equation checks, summed
gyro/accelerometer/reaction pickup, and zero accelerometer pickup for legacy
`[vehicle.bending]`. Schema-v3 `[vehicle.modes]` parses
`[[vehicle.modes.lateral]]` entries with fail-closed validation and cannot be
co-declared with `[vehicle.bending]`; the runner structural rack builds either
path and threads `accel_pickup_m_s2` through the rigid-body FC bridge alongside
`gyro_pickup_rad_s`.
WP-02.1-b: **implemented** (`REQ-STRUCT-002`/`V-STRUCT-002`). Schema-v3
`[vehicle.reaction] enabled = true` is valid only alongside legacy
`[vehicle.bending]` or v3 `[vehicle.modes]`; absent reaction config keeps the
structural reaction vector exactly zero while gyro/accelerometer pickup remains
active. When enabled, the mode-summed lever reaction flows through the rigid-body
kernel torque path and force-accumulator sensor-truth path. A full 40 s
synthetic Phalcon-9 TVC ascent-probe regression with `[vehicle.bending]` plus
reaction enabled reaches end time with bounded angular-rate magnitude and
unit-quaternion norm.
WP-02.1-c: **implemented** (`REQ-STRUCT-003`/`V-STRUCT-003`). The vehicle crate
adds `AbramsonMultiMass`, an opt-in cylindrical Dodge/Abramson fixed-mass plus
N slosh-mass moving-mass model with `mode_count`, checked `J_1'` root/mass/
length/height coefficients, fixed-mass static-CG balancing, and the same
semi-implicit per-axis integration used by the existing linear slosh path.
Scenario TOML accepts `moving_mass.kind = "abramson_multi_mass"` with linear
first-mode initial slosh fields, the runner tank rack builds it, and
`data/slosh/` carries the source-hash-pinned Dodge Table 3.2 coefficient deck.
WP-02.2-a: **implemented** (`REQ-STRUCT-004`/`V-STRUCT-004`). `openbmp-modelio`
is now the L1 ingestion boundary for external reduced-model assets and ships a
CSV-v1 `ModalAsset` parser with fail-closed schema, numeric, duplicate-id,
frequency-reference, and MAC checks. Schema-v3 `[vehicle.modes.asset]` loads a
SHA-pinned CSV under `vehicle.modes.asset.file`, rejects co-declared inline
mode rows, and the runner structural rack maps the asset into the existing
`ModalBlock` path. `data/modal/` carries the synthetic SOL103-style modal asset
and provenance. OP2/OP4 binary readers remain deferred.
WP-02.2-b: **implemented** (`REQ-STRUCT-005`/`V-STRUCT-005`).
`openbmp-structdyn` now owns a deterministic line-load recovery kernel with the
closed `OutputId`/`SectionLoad` vocabulary, locked-order dense GEMV over modal
coordinates and accelerations, and an explicit mode-acceleration residual
correction. Schema-v3 `[loads.recovery]` declares stations and structural output
rows, requires a structural mode block, and the rigid-body runner emits
`structural.load.<station>.<output>` telemetry from the live modal state.
WP-02.3-a: **partial** (`REQ-STRUCT-006`/`V-STRUCT-006`). `openbmp-structdyn`
now includes dense Craig-Bampton transform/reduction helpers, sparse
shift-invert fixed-interface mode extraction, frequency/MAC modal correlation,
full MAC and mass-weighted cross-orthogonality matrix evidence helpers, and
constant-step Newmark-β/generalized-α modal transient integrators with
precomputed effective Cholesky factors. Schema-v3 `[vehicle.transient]`
byte-locks legacy, Newmark-β, or generalized-α modal stepping, and the runner
uses the native integrators for explicit Newmark/generalized-α configs while
omitted configs keep legacy behavior. Tests cover CB static-boundary
stiffness/fixed-interface mode coupling, sparse extraction mass-orthonormality
and CB handoff, synthetic frequency/MAC benchmark correlation, full MAC and
cross-orthogonality matrix evidence, zero-weight generalized-α/Newmark
equivalence, quadratic manufactured Newmark plus generalized-α modal transients
below `1e-10`, parser gates, and runner native-transient stepping. Remaining:
provenance-pinned CalculiX `*FREQUENCY` / `*SUBSTRUCTURE` output for the real
MAC/frequency benchmark closure and public GARTEUR/AIRMOD modal decks.
WP-02.3-b: **partial** (`REQ-STRUCT-007`/`V-STRUCT-007`).
`openbmp-structdyn::fem` now assembles deterministic two-node
Euler-Bernoulli beam mass/stiffness matrices plus three-node constant-strain
plane-stress membrane triangle mass/stiffness matrices, four-node bilinear
quadrilateral membrane mass/stiffness matrices with 2x2 Gauss integration,
three-node Reissner-Mindlin plate-bending mass/stiffness matrices, and combined
six-DOF triangular plus four-node quadrilateral shell assemblies with
drilling-rotation regularization; the quadrilateral shell preserves the
bilinear membrane block and a deterministic two-triangle Reissner-Mindlin plate
split. It exposes clamped free-DOF matrix extraction and carries clamped-free
Blevins frequency references; a synthetic uniform cantilever regression matches
the first five Blevins frequencies within the 1% V7 gate through the existing
sparse fixed-interface extractor.
`openbmp-structdyn::events` now parses closed CLA event classes
(`liftoff_release`, `max_q_gust`, `engine_ignition`, `stage_separation`,
`meco`) and closed event profile kinds, validates a local CLA event-library
fixture for those templates plus checked ignition/separation/MECO modal
stiffness-scale schedules, evaluates step/ramp-release/rectangular-impulse/
half-sine-impulse physical forces plus deterministic max-Q Dryden exponential
and normalized von Karman gust envelopes, projects `Phi^T F`, and emits
max-absolute modal force envelopes over declared sample grids.
Schema-v3 `[loads.events]` now parses declared event force rows with optional
closed `flight_event_kind`, explicit physical-force vectors, mode-shape rows,
and optional positive `modal_stiffness_scale_1` multipliers, requires a
structural mode block, rejects incompatible event/profile pairs, and the
rigid-body runner exposes deterministic max-absolute modal force envelopes plus
typed flight-event-class CSV bytes on `RunOutcome` while also injecting
projected event forces into the live structural rack modal state at each rack
step. Legacy modal stepping
applies event-profiled stiffness multipliers, and explicit
Newmark/generalized-alpha transients advance through per-step
variable-stiffness modal systems. Remaining: calibrated shell formulations
beyond deterministic triangular/quadrilateral shells,
calibrated event-library distribution/coupling beyond the normalized
deterministic gust envelopes and local checked ignition/separation/MECO
stiffness schedules, public GARTEUR/AIRMOD modal benchmark decks, and
flight-load correlation.
WP-02.4-a (POGO structural half): **implemented** (`REQ-STRUCT-008` /
`V-STRUCT-008`). `openbmp-structdyn::pogo` now evaluates longitudinal modal
receptance and sampled Nyquist clearance against caller-provided feed transfer
samples, reports `|1 + L(j omega)|` minimum distance plus peak open-loop gain,
supports feed-transfer scaling for accumulator/dispersion sweeps, assembles
lossless 1-D feedline transfer matrices, evaluates declared min/nominal/max
pump Cb/Mb dispersion boxes with worst-corner reporting, and evaluates the
reduced first-order feed closed-loop cubic roots plus a finite-dimensional
lumped feedline-coupled characteristic with an RHP/marginal verdict. It also
exposes a uniform distributed-line analytic benchmark report for feedline
cascade versus equivalent single-line closed-form residuals, plus a reduced
first-order feed/structure MMS residual evaluator with a synthetic sinusoidal
manufactured transient recovered below `1e-10`. `openbmp-structdyn::pogo` also
parses and fail-closed validates the provenance-backed public NASA qualitative
POGO case-history deck under `data/feed_system`, covering Gemini-Titan II and
Saturn V S-IC/S-II observed bands, mitigation outcomes, source references, and
accumulator/suppressor trend evidence.
The scenario-facing `[propulsion.pogo]` runner path now preserves the feed-side
startup stability gate, applies declared `structural_nyquist_clearance_1`
with a default of `0.1`, can fail closed on below-clearance sampled structural
Nyquist reports when `require_structural_nyquist_clearance = true`, and emits a
normalized structural Nyquist plus closed-loop eigen and optional
pump-dispersion `PogoRunReport` with required `provenance.md` pointer
preservation, optional `[propulsion.pogo.feedline]` mapping from standalone
`[[propulsion.lines]]`, `RunOutcome.pogo.feedline_coupled`, and deterministic
structural-margin CSV bytes on `RunOutcome.pogo`. Remaining ceiling:
quantitative vehicle-specific Titan/Saturn margin closure is not claimed because
the public records do not provide calibrated pump/feed parameter sets.
WP-02.4-b (CSI margin tool): **implemented at method level** (`REQ-STRUCT-009` /
`V-STRUCT-009`). `openbmp-fc::csi` now assembles common-denominator modal CSI
plant coefficient arrays from caller-owned modal path definitions and scratch
buffers, consumes plain coefficient arrays and caller-owned frequency slices,
computes sampled gain margin, phase margin, Nyquist critical-point distance,
and peak gain for one frozen-time plant or a caller-buffered frozen-time
trajectory sweep, and designs a continuous bending notch that exports the
existing `filters::NotchConfig` shape plus unity-magnitude continuous lead/lag
sections. Schema-v3 `[csi]` now parses off-default frozen-time coefficient
sweeps with positive frequency grids, non-negative gain/phase criteria, unique
frozen-point ids, strictly increasing frozen times, and finite nonzero
numerator/denominator polynomials; it also accepts `[csi.modal_assembly]`
frozen points that derive gyro-rate or accelerometer CSI plants from the active
structural mode set, plus `[csi.filter_insertion]` to derive and insert a
three-axis FC gyro-notch table from the selected structural modal frequency.
The rigid-body runner consumes either plant source into `RunOutcome.csi`
summary and per-frozen-point CSV evidence and passes CSI-designed notches into
the FC bridge before autopilot registration. `crates/openbmp-runner/tests/csi_sil.rs`
adds the synthetic time-domain SIL check: a 40 s Phalcon-9 TVC FC run with
structural bending, CSI modal assembly, CSI filter insertion, SIL monitor
observation, actuator-stream evidence, bounded angular-rate magnitude, and
unit-quaternion norm. Focused tests keep the FC portability lock,
no-default-features build, margin math, LTV worst-case sweep, notch
attenuation, lead/lag phase synthesis, parser gates, explicit/modal runner
report emission, CSI filter insertion, bounded SIL evidence, and fail-closed
validation covered. Ceiling: not cFS/HIL qualified and not GVT/flight-test
correlated.

### 03 — Aerodynamic database & CFD coupling

Baseline confirmed: `crates/openbmp-aero` T0 (single-panel modified
Newtonian in `hypersonic.rs`, DATCOM-lite buildup in `buildup.rs`,
locked-order `AeroDeck` interpolation in `deck.rs`). `REQ-AERO-001` /
`V-AERO-001` adds the doc-03/WP-03.6 deck-side substrate for optional
row-major one-sigma `CN/CD/CM` tables plus exponential Mach covariance metadata:
`AeroDeck::with_sigma_tables`, `AeroDeck::lookup_sigma`,
`AeroUncertaintyCorrelation`, and strict schema-1/schema-2 `[uncertainty]`
parsing reject malformed or negative sigma tables and invalid
`[uncertainty.correlation]` lengths fail-closed while sigma-absent decks preserve
nominal behavior. `REQ-AERO-002` /
`V-AERO-002` implements WP-03.1: watertight/BVH infrastructure
exists, and `LocalInclinationPanels` now exposes `PanelSolution` with
caller-CG moments plus axial station-bin force/moment loads while preserving
the legacy `AeroMethod` output, plus synthetic sphere `CD = Cp_max/2`
tolerance evidence, tangent-cone sharp-cone tolerance evidence, and opt-in
BVH centroid-ray shadowing with nested-shell occlusion evidence, a named
wedge-cavity exact-set fixture, public-domain HB-2 Figure 12b high-Mach
forebody-CA digitized evidence, and strict ASCII STL string/file plus binary
STL byte/file ingest for `PanelMesh`. `REQ-AERO-003` / `V-AERO-003` implements WP-03.2:
off-default `PanelSkinFriction`, reference-temperature laminar/turbulent `Cf`,
tangential drag accumulation into panel force/CG moment/axial bins, and optional
`[skin_friction]` panel-deck parsing, plus a synthetic sharp-cone smooth-wall
trend fixture for positive viscous increment, lower high-Re increment, and
small high-Re correction behavior, plus public-domain HB-2 Figure 12b
diameter-Reynolds skin-friction total-CA evidence. `REQ-AERO-004` / `V-AERO-004` implements
WP-03.4: `openbmp-aerodb` exists as an offline L6 crate with
condition-only `RunNode`, deterministic `RunMatrix`, an explicit
target/aimpoint/range vocabulary lint for run-matrix TOML, strict run-matrix TOML
parsing, reduced `AeroDeck` baking from ordered in-Rust `AeroMethod` sweeps, a
no-ground-aimpoint compile-fail tripwire, an FC dependency tripwire ban on
`openbmp-aerodb`, and a schema-v3 `[aero.run_matrix]` scenario file handoff
resolved and pinned as `aero.run_matrix.file` before runner-side strict
`RunMatrix::load_toml_str` parsing, plus strict recorded solver-case manifest
parsing that requires one row-major artifact per run-matrix node, exact
condition echo, solver/case/provenance text, output kind/path, SHA-256 pin, and
caller-supplied artifact byte verification, plus strict normalized external-solver
coefficient CSV deck baking that validates row count, node order, exact
condition echo, finite `CN/CD/CM`, and per-row solver/case/provenance before
building a reduced `AeroDeck`, plus strict normalized surface-Cp CSV load
reporting and deck baking over `PanelMesh` triangles with node/panel order,
condition echo, finite `Cp`, provenance, body-frame CG, full body force/moment
coefficient, and optional axial-bin validation, plus a recorded SU2
`history.csv` coefficient adapter that consumes final-row `CFx/CFz/CMy` or
zero-sideslip `CD/CL/CMy` data, plus a recorded SU2 surface CSV adapter that
maps vertex coordinates and `Pressure_Coefficient` onto `PanelMesh` before
using the surface-Cp load/deck path, plus a recorded Cart3D ASCII `.triq`
surface-Cp adapter that validates TRI/Q geometry against `PanelMesh`, uses the
first per-vertex scalar as Cp, and reuses the surface-Cp load/deck path, plus a
recorded OpenFOAM sampled-surface Cp adapter that parses whitespace-delimited
`x y z Cp` rows, maps them to `PanelMesh` vertices, and reuses the surface-Cp
load/deck path, plus a recorded OpenFOAM `forceCoeffs` adapter that consumes
final-row zero-sideslip
`Cd/Cl/CmPitch` data.
`REQ-AERO-005` / `V-AERO-005` adds the WP-03.6 reduced uncertainty assembler:
`openbmp-uq::GridConvergence` for monotone three-grid observed-order/GCI, plus
`openbmp-aerodb` coefficient sigma budgets that RSS-combine WT/CFD/model-form/
extrapolation sources, can use fine-grid GCI as the CFD source, and attach
row-major reduced `AeroDeck` sigma tables, including strict row-major external
GCI sigma TOML/CSV ingestion. `REQ-AERO-006` / `V-AERO-006` adds
the WP-03.6 deterministic draw substrate: `CorrelatedDeckDrawSettings` plus
`draw_reduced_correlated_deck` return an exact clone for sigma-absent reduced
decks, otherwise use domain-separated `DeterministicRng::for_mc_sample` streams
and Cholesky factors of exponential Mach plus optional alpha/beta covariance to
perturb CN/CD/CM by node sigma with backward-compatible same-Mach alpha/beta
sharing when axis lengths are absent, optionally mixing reduced CN/CD/CM
residuals through a deck-declared coefficient-correlation matrix; settings can
resolve an omitted Mach correlation length from deck-declared metadata.
`REQ-AERO-007` /
`V-AERO-007` adds schema-v3 `[aero.deck_draw]`, validates deterministic draw
settings fail-closed, and has the runner apply the declared draw after external
deck load or `[aero.buildup]` bake before deck-method consumption, including
optional alpha/beta covariance lengths and deck-declared covariance defaulting
when the scenario omits the Mach length.
`REQ-AERO-008` / `V-AERO-008` adds CLI campaign-facing sample scheduling and
execution: `openbmp mc aero-deck-draw-overlays` materializes deterministic
per-sample `[aero.deck_draw]` merge overlays and a campaign-manifest fragment,
while `openbmp mc aero-deck-draw-campaign` emits a full generated campaign
manifest and runs it through the native campaign executor with stable overlay
ids/files and checked sample-index, Mach/alpha/beta-correlation,
dimension-base, and prefix validation, supports optional alpha/beta covariance
flags, supports omitting `mach_correlation_length` so deck-declared covariance
metadata supplies it at runtime, and preflights the resolved deck's required
RNG dimension count so six-sigma decks reserve nine streams before campaign
launch, plus optional
`--uq-toml` sidecar credibility gating, Markdown report output, and
base-scenario `[aero.uq]` upstream-budget merge through the shared
`CampaignCredibilityInputs` contract. The campaign evidence now includes a
two-sample synthetic grid-sigma deck run with generated alpha/beta covariance
overlays, deck-declared Mach covariance defaulting, materialized run scenarios,
and ordinary campaign summary artifacts. `REQ-AERO-009` /
`V-AERO-009` adds the nominal runtime six-coefficient deck substrate:
`AeroDeck` can carry optional row-major `CX/CY/CZ/Cl/Cm/Cn` tables and
matching optional six-channel sigma tables, `DeckLookup` maps explicit six
tables directly into body-frame force/moment while preserving the reduced
fallback, strict schema-1/schema-2 TOML can parse `[six_coefficients.*]` and
`[uncertainty.six_coefficients.*]`, and `openbmp-aerodb::DeckBuilder` can bake
ordered in-Rust force/moment sweeps into decks carrying both reduced and
explicit six-channel tables. `REQ-AERO-010` / `V-AERO-010` extends the
deck-draw substrate to those six-channel sigma tables: `draw_correlated_aero_deck`
preserves nominal six tables during reduced draws, uses separate deterministic
correlated fields for `CX/CY/CZ/Cl/Cm/Cn`, keeps reduced coefficients
nominal when only six sigmas are present, and is now the runner's deck-load
draw helper for `[aero.deck_draw]`. `REQ-AERO-011` / `V-AERO-011` adds
six-channel RSS/GCI sigma ingestion: `SixCoefficientSigmaBudget` and the strict
`openbmp.aerodb_six_gci_sigma = 1` TOML plus
`openbmp.aerodb_six_gci_sigma.csv.v1` CSV paths ingest row-major
fine/medium/coarse `CX/CY/CZ/Cl/Cm/Cn` studies, RSS-combine source terms, and
attach six-channel sigma tables to decks that already carry nominal six
coefficients. `REQ-AERO-012` / `V-AERO-012` adds strict normalized recorded
solver six-coefficient CSV ingestion:
`openbmp.aerodb_solver_six_coefficients.csv.v1` rows echo the run-matrix
condition, carry finite body-axis `CX/CY/CZ/Cl/Cm/Cn`, build explicit
six-channel deck tables, derive reduced `CN/CD/CM` compatibility tables, and
can be labeled in solver-case manifests as `normalized_six_coefficients_csv`.
`REQ-AERO-013` / `V-AERO-013` adds six-channel coefficient-correlation
metadata and draw mixing: strict `[uncertainty.correlation] six_coefficient`
6x6 matrices are validated and parsed by `openbmp-aero`, preserved through
deck reconstruction, and used by `draw_correlated_aero_deck` to mix correlated
`CX/CY/CZ/Cl/Cm/Cn` residual fields before applying per-node sigma.
`REQ-AERO-014` / `V-AERO-014` adds the first recorded external-solver
command-plan substrate: strict `openbmp.aerodb_solver_command_plan = 1` TOML
captures per-node direct program/argv/env/working-directory/timeout intent plus
expected output artifact pins and projects those outputs into the existing
SHA-checked `SolverCaseManifest` shape without launching processes.
`REQ-AERO-015` / `V-AERO-015` adds direct no-shell execution for those plans:
`execute_with_root` resolves case paths under a caller root, applies argv/env,
captures stdout/stderr SHA evidence, enforces timeouts, rejects nonzero exits,
and verifies declared output artifact SHA pins.
`REQ-AERO-016` / `V-AERO-016` adds deterministic SU2-Euler case config
generation: `Su2EulerCaseTemplate::render_cases` emits one row-major config per
run-matrix node with condition echoes, protected SU2 freestream/reference/output
keys, command metadata, and config SHA evidence, while
`write_configs_to_root` materializes only those configs under an existing root.
`REQ-AERO-017` / `V-AERO-017` adds bounded retry execution for direct
command plans: `execute_with_root_retrying` applies an explicit
`SolverCommandRetryPolicy`, records per-attempt failure fields, retries only
configured transient execution classes, and keeps unsafe path/static
validation failures non-retryable.
`REQ-AERO-018` / `V-AERO-018` adds deterministic Slurm array-script rendering:
`render_cluster_schedule` converts validated command plans into SHA-pinned
`sbatch` script artifacts with MPI ranks, CPU/task counts, quoted argv/env, and
one array task per run-matrix case without submitting jobs.
`REQ-AERO-019` / `V-AERO-019` adds Slurm submission/state-query evidence:
`submit_with_root` writes and submits rendered schedules through a direct
command vector, parses the scheduler job id, and `query_job_state_with_root`
classifies scheduler state output while retaining stdout/stderr digests.
`REQ-AERO-020` / `V-AERO-020` adds bounded completion polling and output
collection: `wait_for_completion_with_root` samples scheduler state until
successful completion or fail-closed terminal/exhausted states, and
`collect_expected_outputs_with_root` verifies each planned output artifact SHA
under the solver root.
`REQ-AERO-021` / `V-AERO-021` adds recorded OpenFOAM scalar case-field parsing:
`OpenFoamScalarField::parse_str` reads `FoamFile`, `dimensions`,
`internalField`, and `boundaryField` scalar dictionaries, and
`bake_openfoam_boundary_cp_field_deck` consumes named boundary patch Cp values
in panel order.
`REQ-AERO-022` / `V-AERO-022` adds recorded OpenFOAM vector-field parsing and
scalar/vector field-set validation: `OpenFoamVectorField::parse_str` reads
`volVectorField`-style `U` dictionaries, and `OpenFoamRecordedCaseFields`
validates expected `FoamFile.object` names for scalar and vector dictionaries.
`REQ-AERO-023` / `V-AERO-023` adds declared case-root loading for recorded
OpenFOAM scalar/vector field files through `OpenFoamCaseFieldFile` and
`OpenFoamRecordedCaseFields::from_case_root`, including unsafe-path and
unreadable-file rejection.
`REQ-AERO-024` / `V-AERO-024` adds deterministic selected time-directory
discovery for supported OpenFOAM scalar/vector field files through
`OpenFoamRecordedCaseFields::discover_time_directory`.
`REQ-AERO-025` / `V-AERO-025` adds recorded `polyMesh/boundary` metadata parsing
and validates that scalar/vector field boundary patches are declared by the mesh
boundary metadata.
`REQ-AERO-026` / `V-AERO-026` adds recorded `polyMesh`
points/faces/owner/neighbour connectivity parsing plus boundary-range checks
against the face list and internal-face prefix.
`REQ-AERO-027` / `V-AERO-027` adds recorded `turbulenceProperties` parsing and
common RAS/LES model field-set semantic validation.
`REQ-AERO-028` / `V-AERO-028` adds recorded optional `polyMesh`
point/face/cell zone topology parsing and validates zone labels against parsed
points/faces/owner/neighbour connectivity.
`REQ-AERO-029` / `V-AERO-029` adds recorded `polyMesh` face/cell geometry
reconstruction from parsed points/faces/owner/neighbour connectivity.
`REQ-AERO-030` / `V-AERO-030` adds recorded `turbulenceProperties`
coefficient-block parsing and calibrated scalar coefficient tolerance checks.
`REQ-AERO-031` / `V-AERO-031` adds calibrated six-coefficient AeroDeck
validation cases with per-channel tolerances and deterministic error reports.
`REQ-AERO-032` / `V-AERO-032` adds site-specific remote artifact staging
through caller-supplied direct transfer commands with stdout/stderr digest
evidence and SHA-verified expected-output collection.
Still missing:
WP-03.3 adds an opt-in DATCOM-style projected-planform and finite-length-corrected crossflow, equation-15-style incremental foredrag, fin/body carryover, area-rule wave-drag, protuberance-drag, full-matrix dynamic-derivative deck/runtime substrate, parser-gated synthetic validation-fixture harness, public NACA Report 1048 equation-level, body-I/body-II crossflow, equation-15 force/moment, and equation-15 incremental-foredrag fixture evidence, public NACA Report 1048 finite-length crossflow drag-ratio evidence, Sears-Haack/area-rule wave-drag branch fixture evidence, DATCOM fin/body carryover factor branch evidence, and equivalent-protuberance drag branch evidence (`REQ-AERO-033` / `V-AERO-033`).
WP-03.5 now has a first attached-flow fin lift/hinge substrate, analytic
rectangular finite-wing slope/CL/induced-drag/AC evidence, first grid-fin
authority substrate, deterministic horseshoe-vortex influence assembly,
fixed-pivot dense solve, operand-order bit-pattern evidence, bounded subsonic
coefficient scaling, deterministic horseshoe derivative channels plus a
parser-gated derivative comparison harness, a public NACA Report 921
method-convention fixture, public NASA SP-405 rectangular-wing lift-slope
evidence, and synthetic trend/residual evidence
(`REQ-AERO-034` / `V-AERO-034`). WP-03.1, WP-03.2, WP-03.4, and WP-03.6 are
**implemented**; WP-03.3 and WP-03.5 are **partial**;
WP-03.7 … WP-03.9 remain **not started**.

### 04 — Aerothermal, real-gas & TPS

Baseline confirmed. `REQ-AEROTHERMAL-005` / `V-AEROTHERMAL-005` implements
WP-04.2-a: `openbmp-thermochem` is the shared L2 thermochemistry scaffold
(deck + parser on a (log pc, MR) grid — combustion-deck shape for doc 05;
no equilibrium-air closure), documents the propulsion/aerothermal boundary, and
is included in the FC forbidden-edge tripwire. `REQ-AEROTHERMAL-001` /
`V-AEROTHERMAL-001`
adds the deck-independent WP-04.1-b Sutton-Graves gas coefficient substrate:
`SuttonGravesGas` exposes NASA TR R-376 base-gas and Earth-air coefficient
presets, default Earth-air output remains byte-identical, and non-default
gas presets scale heat flux by the public table coefficient ratio. Still
covers WP-04.1-b alongside `REQ-AEROTHERMAL-002`, which adds the smooth-wall
Re_theta/M_e onset/ramp substrate plus the PANT/Reda rough-wall transition
selector with provenance-pinned checked thresholds, endpoint heat-flux
continuity, analytic sharp-cone transition-location tolerance evidence, and
fail-closed input tests. `REQ-AEROTHERMAL-003` adds the DKR stagnation-heating
cross-check with explicit imperial-to-SI conversion and a hot-wall correction
fixture. `REQ-AEROTHERMAL-004` makes WP-04.1-a **partial**: the default
Tauber-Sutton surface remains off, while explicit `earth_air()` selection uses
the provenance-pinned table, fail-closed interpolation, W/cm²-to-W/m²
conversion, total-heating composition, schema-v3 `[aerothermal.radiative]`
selection, and runner `aerothermal.q_rad_w_m2` telemetry. Apollo-4/FIRE-II
code-to-code radiative validation is still missing. Other stack elements remain unchanged:
cold-gas Fay-Riddell scaffold, Tannehill typed-reserved (refusing), toy
ablators, Park payload pinned. WP-04.2-b … WP-04.4-b remain **not started**;
WP-04.1-b and WP-04.2-a are **implemented**.

### 05 — Propulsion high-fidelity

| WP | Status | Evidence / missing |
|---|---|---|
| WP-05.1 pressure-thrust + altitude | **implemented** | `NozzlePerformance` in `crates/openbmp-propulsion/src/motor.rs`; scenario `ambient_pressure_correction = "pressure_thrust"`; `crates/openbmp-runner/tests/pressure_thrust.rs` + fixture; REQ-PROP-001/V-PROP-001 |
| WP-05.2-a nozzle separation clipping | **implemented** | `NozzleSeparationCriterion::{Summerfield,Schmucker}`; scenario opt-in; test in `pressure_thrust.rs`; REQ-PROP-002 |
| WP-05.2-b transient solid ballistics | **implemented** | `TransientChamber` pc(t) ODE + erosive hook in `grain.rs`; `[propulsion.motor.grain] mode = "transient"`; test; REQ-PROP-003 |
| WP-05.3 thermochem deck ingestion | **implemented** | crate + parser + scenario block + SHA pin + fixtures + inline grain, reduced feed-network, liquid-engine thermochemical runtime consumption, empirical `c_star_efficiency` band propagation, provenance-recorded Cantera 3.2.0 LOX/LCH4, LOX/LH2, LOX/RP-1 n-dodecane-surrogate, and N2O4/N2H4 hydrazine-hypergolic reference tolerance tables, plus NASA CEA 3.2.1 true NTO/MMH and non-surrogate LOX/RP-1 rocket-equilibrium reference tolerance tables over 20-point 1-30 MPa grids with multiple in-deck interpolation cases landed; `LiquidEnginePerformance` derives thrust, nominal choked mass flow, `Isp`, and mass-flow/`Isp` envelopes from looked-up thermochemical state + nozzle geometry; real-engine efficiency/hot-fire calibration remains a validation ceiling rather than an open WP-05.3 implementation gap |
| WP-05.4-a feed network + transient chamber | **implemented** | `openbmp-feedsystem` (graph/network/line/chamber/control/transient/blowdown) + `crates/openbmp-runner/src/feed_network.rs` + scenario `propulsion.feed_networks` validation + synthetic steady feed-network tolerance table for reduced and graph solves + public GFSSP V6 water-distribution pipe-network worked-case tolerance table + public GFSSP V6 Example 8 choked tank-blowdown analytic tolerance table + public GFSSP V6 Example 10 digitized propellant-tank pressurization pressure-history fixture with residual gate, reduced calibrated pressure-history closure, and lumped finite-volume ideal-gas ullage solver closure + synthetic transient chamber/dual-valve tolerance table + synthetic throttle/MR controller tolerance table |
| WP-05.4-b turbopump map + NPSH | **implemented** | `pump.rs` (`Turbopump`, normalized map, design point) + scenario pump blocks + runner pressure/cavitation coupling + synthetic provenance-backed tolerance table + public TOP-FLO catalog duty-point/NPSHr/motor-sizing calibration fixture; ceiling: proprietary rocket turbopump map shape and cavitation-test calibration |
| WP-05.4-c MOC line transients | **implemented** | `line.rs` (`MocLine`) + upstream velocity/head/nonlinear gas-source and downstream velocity/head/linear-accumulator/nonlinear gas-accumulator endpoint boundary conditions + local named-line `MocLineTopology` substrate with common Courant-step validation and all-or-nothing stepping + zero-storage `MocLineGraphTopology` junction semantics + embedded scenario line blocks + runner pressure perturbation coupling + standalone `[[propulsion.lines]]` and `[[propulsion.line_junctions]]` parsing/runner stepping with per-line snapshots + `oxidizer_line_id` / `fuel_line_id` coupling into transient feed-network legs + synthetic and public Joukowsky valve-closure tolerance tables; ceiling: broader lossy GFSSP-style branch/minor-loss network semantics |
| WP-05.5-a POGO feed half | **implemented** | `pogo.rs` (feedsystem + runner + structdyn) + `propulsion.pogo` schema, `mode_source = "vehicle_bending"` consumption of `[vehicle.bending]`, optional `[propulsion.pogo.feedline]` references to standalone `[[propulsion.lines]]`, `RunOutcome.pogo.feedline_coupled` emission from the lumped feedline-coupled eigen analyzer, lossless 1-D feedline transfer-matrix assembly with distributed-line analytic benchmark, synthetic provenance-backed stability tolerance table, pump-dispersion and sampled-Nyquist reports, and public qualitative Titan/Gemini + Saturn case-history fixture; ceiling: calibrated pump reconstruction and quantitative vehicle-specific Titan/Saturn margin closure |
| WP-05.5-b engine-out & fault library | **implemented** | deterministic `[propulsion.faults]` scheduled payloads verified across scenario parser and runner mapping for `stuck`, `hard_off`, `over_thrust`, `hard_start_overpressure`, `cavitation_thrust_loss`, and `gimbal_locked`; pump-cavitation triggers, MR-runaway valve drift, SIL scheduled-engine stimulation/EKF observation, MC sampled fault-library overlays including `stuck`, and `openbmp mc propulsion-faults` campaign path with UQ summaries traced through `REQ-PROP-020`..`REQ-PROP-027`; ceiling: calibrated probabilities, physical leak-flow data, and vehicle-specific FMEA |

### 06 — Coupled MIMO GNC

Baseline confirmed: per-axis controllers, 15-state error-state EKF,
GLRT/IMM/voter present. Today's `a903c50` added covariance-trace lane voting
in `estimator_lanes.rs` (+ SR-UKF touch-up) — a diagnostics improvement, not
a backlog WP. WP-06.1 is now **implemented** through
`EffectivenessMatrix`, `EffectorBounds`, `WlsActiveSetAllocator`,
`WlsAllocationOutput`, and `DenseWlsControlAllocator` in
`crates/openbmp-fc/src/allocation.rs`: the path accepts coupled dense-`B`
rows/columns, asymmetric bounds, per-tick rate tightening, deterministic
active-set solving, active-at-bound flags, virtual residuals, phase-gated
fixed-zero columns, no-default-features coverage, schema-v3
`[fc.autopilot_allocation] kind = "dense_wls"` parsing, runner construction
from `dense_effectors`, and mixer `EffectorCommandSet` routing while preserving
the legacy diagonal allocator dispatch, QCAT `wls_alloc`-derived ADMIRE and
synthetic TVC tolerance-table checks, a dense B-matrix no-target vocabulary
schema tripwire, FC dependency and FC-lint tripwire coverage, runner telemetry
byte-preservation coverage, and allocator E2E byte-stability coverage
(`REQ-GNC-001`). The local §2 merge sweep is green for
`cargo build --workspace --all-features --locked`,
`cargo clippy --workspace --all-targets --all-features --locked -- -D warnings`,
`cargo test --workspace --all-features --locked`,
`openbmp check-provenance data/gnc`, `openbmp check-provenance scenarios`,
`scripts/check_requirements_traceability.py`, `scripts/check_parity_docs.py`,
`cargo deny check`, `cargo audit`, and `cargo machete`; local `cargo nextest`
is unavailable, so the full workspace cargo test is the recorded substitute.
WP-06.2-a is now **implemented** (`REQ-GNC-002`): `lqr.rs` exposes
`solve_dare_nxn_doubling`, `solve_dare_nxn_doubling_with_options`,
`dense_dare_residual_norm`, and `CoupledLqrSolution` for bounded dense
row-major DARE/LQR solves, and `tables.rs` exposes `CoupledGainNode` as the
validated table target for offline gains. The new
`data/gnc/coupled-lqr-dare-tolerance-v1.toml` fixture generated with
python-control `dlqr` / SciPy `solve_discrete_are` checks scalar and coupled toy
plants, residuals, iteration bounds, and bit-stable reruns, while a focused test
proves fail-closed behavior for an uncontrolled unstable mode. The local §2
merge sweep is green for workspace build, workspace clippy, full workspace cargo
test, GNC/scenario provenance, traceability, parity-doc validation, cargo deny,
cargo audit, cargo machete, and formatting; local `cargo nextest` is
unavailable, so the full workspace cargo test is the recorded substitute.
WP-06.2-b is now **implemented** (`REQ-GNC-003`):
`tables.rs` exposes
`GainScheduleTable`, `ScheduleVariable`, and `GainInterp` around scheduled
`CoupledGainNode` entries; `schedule.rs` exposes `ScheduledController`,
`interpolate_gain_schedule`, `reset_integral_states_for_bumpless_transfer`,
`sweep_closed_loop_row_sum_margin`, and `sweep_siso_discrete_loop_margins`;
and `lqr.rs` exposes `solve_lqg_observer_nxn_dual_dare` plus
`CoupledLqgObserverSolution`. The new
`data/gnc/gain-schedule-lqg-tolerance-v1.toml` fixture records a scalar Mach
schedule with node/interior interpolation, observer-gain interpolation, a
SISO `>=6 dB`/`>=30 deg` margin sweep, conservative row-sum stability,
bumpless integral-state reset, and SciPy `solve_discrete_are` dual-DARE LQG
observer covariance/gain references. This remains validated-toy method
evidence only, not a real vehicle gain schedule or MIMO robustness
certification. Focused V-GNC-003 gates, workspace build, workspace clippy, full
workspace cargo test, traceability, parity-doc validation, formatting,
GNC/scenario provenance, and supply-chain hygiene are green in the 2026-06-26
batch sweep.
WP-06.2-c is now **implemented** (`REQ-GNC-004`):
`filters.rs` exposes `BiquadSection` and `prewarped_tustin_notch` for
prewarped-Tustin second-order notch sections, and `tables.rs` exposes
`BendingFilterSectionSpec`, `BendingFilterNode`, `BendingFilterTable`, and
`interpolate_bending_filter_table` so schedules carry physical mode/zero/pole
parameters instead of raw IIR taps. The new
`data/gnc/bending-filter-tustin-tolerance-v1.toml` fixture records two scalar
Mach knots plus an interior query with closed-form coefficients, stable-pole
validation, `>=18 dB` toy mode attenuation, and filtered analytic
bending-plus-slosh modal margins of `>=6 dB`/`>=30 deg` at both nodes and an
interior point through `csi::analyze_filtered_frozen_plant`. `AutopilotParams`
now exposes an off-default `BendingFilterRuntimeConfig` insertion path that
evaluates TimeS/DynamicPressurePa schedules, updates stateful per-axis cascades,
attenuates enabled gyro axes before the rate loop, preserves disabled axes, and
fails closed for unsupported runtime schedule variables. Focused V-GNC-004
gates, workspace build, workspace clippy, full workspace cargo test,
traceability, parity-doc validation, formatting, GNC/scenario provenance, and
supply-chain hygiene are green in the 2026-06-26 batch sweep.
WP-06.3-a is now **implemented** (`REQ-GNC-005`):
`loadrelief.rs` exposes `LoadReliefParams`, `AlphaEstimate`,
`LoadReliefCommand`, `estimate_alpha_beta`, `evaluate_load_relief`, and
`GyroBlender`; `AutopilotParams::load_relief` installs an off-default
pre-attitude-loop correction that reads position, density, and optional modeled
wind; `openbmp-msgs` appends `estimator.wind`, and the runner FC bridge
publishes modeled ECI wind from the environment sample. The v3
`[fc.autopilot_params.load_relief]` schema is scenario-facing, validates
strictly ascending blend schedules, rejects target/aimpoint vocabulary, and the
runner maps it into `AutopilotParams::load_relief`. The new
`data/gnc/load-relief-alpha-beta-tolerance-v1.toml` fixture records alpha/beta
geometry and a deterministic q-alpha limiter series; focused FC/message/runner
gates are green. `load_relief_reduces_post_response_qalpha_under_gust_envelopes`
adds deterministic Dryden and von-Karman controller-SIL envelopes and proves the
enabled load-relief path reduces post-response peak `q·alpha` by at least 20%
versus drift-minimum while requiring nonzero pitch command. WP-06.3-b is now
**partial** (`REQ-GNC-006`): `openbmp-propulsion::tvc` exposes a deterministic
second-order TVC plant substrate with rate/position limits, external load-torque
feedback, and `-J * delta_ddot` TWD reaction reporting, `LiquidEngine` can opt
into that plant for delivered thrust-vector rotation and last-step reaction
evidence, v3 engine `tvc_actuator` I-loads parse and build through the runner,
`data/gnc/tvc-twd-inca-tolerance-v1.toml` records the TVC/TWD/INCA tolerance
evidence,
while `indi.rs` exposes `DenseIncaController`, `DenseIncaOutput`, `DenseIncaError`, and
`PseudoControlHedger`, reusing the dense WLS allocator to solve `B_eff`
increments, report pseudo-control deficit, and keep the hedged integral bounded
when a rate-limited actuator cannot deliver the requested pseudo-control. Dense
INCA now also matches fixture-recorded compensated/uncompensated RMS values and
shows >90% residual RMS reduction against a deterministic slowly varying
TWD-like reaction series relative to a no-compensation baseline. Focused
propulsion, runner, and `--features indi` FC gates are green, and the structural reaction
live-SIL regression proves an opt-in TWD-style reaction perturbs the synthetic
TVC probe attitude-rate history while remaining bounded, and a feature-gated
scalar INDI runner regression reduces live reaction-induced body-rate residual
RMS below 75% of the PID residual. Active FC dense-INCA allocation is now wired
through `autopilot.pseudo_control_cmd`, `kind = "dense_inca"` schema/runner
construction, and mixer pseudo-control consumption. Dense-INCA columns can now
target physical engine-gimbal aliases (`<engine_id>.gimbal_pitch` /
`<engine_id>.gimbal_yaw`), inherit engine gimbal bounds/rate limits, and merge
back into gated engine commands without dropping throttle or ignition state. The
dense-INCA live runner regression now uses those physical TVC-gimbal columns on
the powered TVC probe to reduce structural-reaction body-rate residual RMS below
75% of the PID residual. A narrow failed-engine availability path now publishes
failed physical TVC gimbal aliases through `actuator.effector_availability`,
the mixer drops those columns before dense allocation, and a shortened powered
TVC probe survives a scripted engine hard-off to EndTime. Tight nav plus full
engine-out retrim and abort-to-SAFE remain open. WP-06.4-c is now also
**partial** (`REQ-GNC-007`): when availability removes a dense column and
remaining authority cannot attain demand, the mixer marks `EffectorCommandSet`
saturated, FDIR latches `FDIR_BIT_ALLOCATION_INFEASIBLE` from that signal, and
commander maps allocation-infeasible-only trips into canonical `health.degraded`
while keeping `safe_state_requested` false. Commander-side confirmation counters
now delay both DEGRADED and SAFE transitions until configured consecutive-tick
counts are met, defaulting to one tick for existing behavior. The mixer also
turns `safe_state_requested` into zero-throttle, zero-gimbal, ignite-false,
shutdown-true `EngineDemand` and per-engine `EngineCommandSet` output without
waiting for a fresh autopilot engine demand (`REQ-GNC-008`). The
`ballistic_state_compile_fail` trybuild harness now rejects a public
`RequestSafeState` construction with `ground_aimpoint_eci_m`, locking the SAFE
action vocabulary against ground redirect fields (`REQ-GNC-009`). The shared
message dictionary, FC step output, mixer, and runner FC bus now carry a
`PropellantSafingCommand` intent topic for feed isolation, engine-valve closure,
pressurization disable, and drain disable during SAFE (`REQ-GNC-010`). The
runner now consumes the latest safing command in point-mass and rigid-body
propellant-budget paths, latches configured feed networks closed, clears tank
drain rates when drains are disabled, and forces propellant-budgeted engine
feed scales to zero with shutdown requests (`REQ-GNC-011`). The autopilot now
latches the attitude measured at SAFE entry as a target-free hold reference,
suppresses normal guidance/trajectory/load-relief corrections while SAFE is
active, exposes `AutopilotStatus.safe_attitude_hold_active`, and publishes
zero-throttle shutdown demand (`REQ-GNC-012`). A focused FC pipeline test now
proves a non-allocation FDIR trip drives commander SAFE request, autopilot SAFE
hold, mixer propellant safing, and per-engine shutdown output on one bus
(`REQ-GNC-013`). A focused `FcRunner` test proves the runner-facing accessors
surface SAFE status, SAFE hold, shutdown engine demand, propellant safing
intent, and per-engine shutdown commands after an injected previous-tick
non-allocation FDIR trip (`REQ-GNC-014`). A focused rigid-body `Session` test
now clips the Phalcon-9 TVC probe, injects a mission `request_safe_state`
event, proves pre-SAFE thrust, post-SAFE zero-thrust Shutdown across all nine
engines, bounded finite rigid-body state, and end-time completion
(`REQ-GNC-015`). The propellant budget report now also carries a simplified
`pressurization_enabled` flag that defaults true for normal evaluation and is
flipped false by active SAFE pressurization-disable intent (`REQ-GNC-016`).
Failed-engine snapshots now publish failed engine ids through
`EffectorAvailability`, and the mixer shuts those engines down while retrimming
scalar throttle over the remaining phase-authorized engines with a 1.0 clamp
(`REQ-GNC-017`). Focused FC/scenario/runner tests now cover a calibrated
`[[fc.engine_out_trim]]` table that adds per-survivor gimbal biases after scalar
engine-out throttle retrim (`REQ-GNC-022`). SAFE drain-disable intent now reaches `TankRack`, latching
declared, budget, and RCS feed drains closed during active SAFE (`REQ-GNC-018`).
SAFE pressurization-disable intent now fails reported engine feed pressure closed
with shutdown requests even when feed isolation and valve closure are not requested
(`REQ-GNC-019`). Focused scenario/tank-rack tests now cover distinct SAFE dump
intent plus configured constant-rate `safe_dump_rate_kg_per_s` application while
ordinary drains remain closed (`REQ-GNC-020`). Focused scenario/tank-rack tests
now also cover pressure-coupled `safe_dump_orifice` sizing from live ullage
pressure while ordinary drains remain closed (`REQ-GNC-021`). Focused
scenario/tank-rack tests now cover `safe_dump_vent` choked ullage-gas venting
that reduces the next-step pressure available to the liquid dump orifice
(`REQ-GNC-023`). Focused scenario/runner tests now cover optional
`[fc.engine_out_trim_auto]` derivation of scalar fan-out trim rows from declared
engine mount geometry and max thrust (`REQ-GNC-024`). Focused scenario/tank-rack
tests now cover opt-in `safe_dump_vent.thermal_model = "adiabatic_ullage"`,
which lowers next-step ullage pressure and liquid-orifice dump below the fixed
temperature path (`REQ-GNC-025`). Focused scenario/tank-rack tests now cover
deterministic `backpressure_ramp` schedules on SAFE dump orifices and vents,
reducing later liquid dump flow and gas venting as downstream pressure rises
(`REQ-GNC-026`). Focused scenario/tank-rack tests now cover
`safe_dump_vent.thermal_model = "adiabatic_wall_relaxation"`, including required
wall-temperature/tau validation and warm-wall post-vent pressure recovery over
the pure adiabatic path (`REQ-GNC-027`). Focused scenario/tank-rack tests now
cover `backpressure_source = "ambient_atmosphere"` on SAFE dump orifices and
vents, including point-mass/rigid runner ambient-pressure handoff and reduced
dump/vent flow under higher ambient pressure (`REQ-GNC-028`). Focused
scenario/tank-rack tests now cover opt-in finite-volume tank pressurization
hardware using the existing feedsystem model, including ullage-required parser
validation, heat-capacity-ratio rejection, higher later liquid-orifice dump
flow, and SAFE pressurization-disable latching (`REQ-GNC-029`).
WP-06.4-d is now **partial**
(`REQ-GNC-030`/`REQ-GNC-031`/`REQ-GNC-032`/`REQ-GNC-033`):
`openbmp-fc::tables`
exposes `RobustControllerNode` and `RobustControllerTable` for provenance-pinned
reduced discrete state-space controller imports with bounded dimensions,
strictly sorted scalar schedule nodes, finite row-major `A_c/B_c/C_c/D_c`
matrices, SHA-256 source-digest validation, target-vocabulary rejection, and
two-buffer table activation. `openbmp-fc::schedule` exposes
`ImportedRobustController`, `RobustControllerMatrices`,
`ImportedRobustControllerStep`, and `interpolate_robust_controller_table`,
clamps/interpolates scheduled matrices, validates runtime measurements, computes
`u[k] = C_c x_c[k] + D_c y[k]`, and advances
`x_c[k+1] = A_c x_c[k] + B_c y[k]` in locked row-major order. Schema-v3
`[fc.robust_controller]` now declares `iload_file` plus optional
`iload_file_sha256`, resolves the file through `Scenario::resolved_files`, and
fails closed on schema-v2 use or SHA mismatch. The runner bridge now passes
resolved scenario files into `FcRunner::new_with_resolved_files`; when the
robust-controller declaration is present, the runner rechecks the optional pin,
parses the `[robust_controller]` / `[[robust_controller.nodes]]` TOML envelope,
materializes a validated `RobustControllerTable`, and declares it as the active
FC table at boot. The FC schedule layer now also exposes
`SisoStateSpaceRef`, `siso_state_space_loop_margins`, and
`sweep_imported_robust_controller_siso_margins`, backed by the pinned
`data/gnc/robust-controller-siso-margin-tolerance-v1.toml` fixture, to sweep
public/toy SISO gain/phase margins over imported robust-controller schedule
nodes and interior points.
Flight-calibrated integrator bias estimation plus cryogenic phase-change,
flight-calibrated regulator/bottle thermal coupling, plume-coupled vent
backpressure, production pressurant hardware validation, closed-loop
robust-controller measurement/command mapping, external H-infinity/mu synthesis
reproduction, and MIMO mu-margin validation remain open.

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
WP-07.2: **partial** — `src/collocation.rs` exposes Hermite-Simpson segment and
fixed-mesh transcription evaluation with dynamics defects, Hermite midpoint
defects, Simpson Lagrange cost, stacked defect reports, finite/dimension
validation, cubic exactness, malformed-input rejection, Simpson cost
integration, and scalar sine MMS fourth-order midpoint residual evidence through
`scenarios/trajopt-two-body-apogee/hermite-simpson-mms-tolerance.toml`; it also
exposes a flat fixed-mesh decision layout, node-state boundary residuals, soft
node/midpoint state/control path exterior penalties, Simpson-weighted quadratic
control objective residuals, dense hard scalar path inequality rows with
violation reports, hard scalar path decision-bound merges consumed by the dense
SQP lane, nonlinear vector hard path inequality rows with deterministic
central-finite-difference CSC sparse path-Jacobian evidence, deterministic
columnwise and colored central-finite-difference CSC sparse residual Jacobian
reports plus caller-supplied analytic dynamics-Jacobian CSC sparse residual
Jacobian reports with scalar-integrator numeric evidence and forward-mode
autodiff dynamics-Jacobian CSC sparse residual Jacobian reports with nonlinear
numeric evidence, sparse Gauss-Newton/KKT normal-block assembly from CSC
residual Jacobians plus sparse normal-equation RHS/matvec/conjugate-gradient
solve evidence, sparse augmented-KKT assembly with deterministic LDL KKT
solve/step evidence plus raw KKT residual-multiplier row-family slicing for
later costate/PMP witnesses plus unscaled node-costate estimate and raw
Hamiltonian diagnostic evidence, active scalar hard-path constrained sparse-KKT
step evidence with active inequality multipliers and constrained sparse-KKT
diagnostic handoff into residual-multiplier/costate/Hamiltonian adapters through
`fixed_mesh_ocp_sparse_constrained_kkt_step_reports_active_hard_path_multipliers`,
fixed-mesh sparse step-report evidence, and a residual-only sparse-KKT SQP
outer loop with accepted-step diagnostics, scalar hard-bound enforcement,
nonlinear hard-path feasibility preservation, and final multiplier evidence through
`fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers`
and
`fixed_mesh_ocp_sparse_kkt_sqp_enforces_hard_path_decision_bounds` and
`fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility`, and
validated sparse-SQP PMP/KKT certificate reports that prefer final active
constrained-KKT multipliers when available and round-trip through optimized
reference I-load payloads via
`sparse_kkt_sqp_pmp_kkt_certificate_attaches_to_optimized_reference_iload`, including optional validated same-mesh costate-comparison summaries, a feature-gated fixed-mesh
quadratic-control dense-QP wrapper plus dense-SQP solve wrapper with final
residual and bound-violation evidence, a feature-gated constrained dense-SQP wrapper that consumes nonlinear hard path
rows through linearized QP inequalities, deterministic optimized reference
profile and I-load payload emission from node/midpoint fixed-mesh samples,
FC-native optimized-reference profile guidance that revalidates/interpolates the
same radius/speed/FPA shape and proves the existing autopilot trajectory loop
tracks it through an actuator command, plus a host testkit postcard I-load
round-trip bridge that converts decoded trajopt reference samples into that
FC-native guidance shape without an `openbmp-fc -> openbmp-trajopt` edge, and a
schema-v3 runner `guidance = "optimized_reference"` inline-sample or
file-backed postcard I-load surface that publishes an interpolated
`ReferenceState` for the active phase plus validated-toy inline and PMP-derived
file-backed optimized-reference scenarios that export valid
`guidance.reference.*` telemetry and nonempty FC actuator command-stream reports
(`REQ-TRAJOPT-004`), exact
scalar-integrator residual assembly, a provenance-pinned validated-toy scalar
minimum-energy integrator benchmark that recovers analytic `x(t)=t`, `u(t)=1`,
cost `1`, a public closed-form brachistochrone cycloid transcription/cost
benchmark through `scenarios/trajopt-brachistochrone/brachistochrone-cycloid-benchmark.toml`,
a fixed-control brachistochrone dense-SQP state/free-duration restoration test
through `solve_dense_sqp_with_free_final_time`, final-time objective wrapper
evidence on a scalar integrator with an active time lower bound, and a
free-control brachistochrone time-objective metric test through
`solve_dense_sqp_with_free_final_time_objective` that reaches the public
cycloid time and residual tolerances without claiming full stationarity,
a public Bryson-Denham state-constrained fixed-mesh dense-QP wrapper benchmark
with sparse constrained-KKT handoff and repeat-determinism evidence through
`scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml`,
a validated-toy T1/T2 shared-ascent Hermite-Simpson mesh cross-check through
`multiple-shooting-tolerance.toml`, fixed-mesh dense-SQP wrapper
repeat-determinism evidence, malformed
fixed-mesh/path/objective/Jacobian/autodiff/bound/wrapper rejection, and a feature-gated
path that feeds fixed-mesh residuals into the dense SQP solver.
`src/nlp.rs` now exposes the public sparse-NLP backend contract, preflight
assembler, and checked solve wrapper (`ProblemDims`, `DecisionLayout`,
`SparseVec`, `CscMatrix`, `NlpResidual`, `NlpAssemblyReport`,
`NlpSolveValidationSettings`, `NlpCheckedSolveReport`, `NlpSolution`,
`OptimalControlProblem`, `Transcription`, `NlpSolver`, `assemble_sparse_nlp`,
`solve_sparse_nlp_checked`, and `solve_sparse_nlp_checked_with_settings`), plus
feature-gated `DenseSqpNlpSettings` and `DenseSqpNlpSolver` for bounded
deterministic first-order SQP adapter cases through the same checked solve
wrapper, with
focused tests covering CSC/vector/residual validation, closed
`TerminalCondition` OCP assembly, objective-gradient dimensions, inequality
bounds, malformed backend shapes, backend output-shape validation, final
assembly validation, and backend-reported objective consistency
plus unconstrained quadratic objective reduction and equality/two-sided-bound
feasibility restoration (`REQ-TRAJOPT-010`).
Feature-gated `src/sqp.rs` exposes deterministic Clarabel dense QP solves,
box-bounded linearized least-squares, linear-objective least-squares, and
equality-constrained linear-objective and elastic residual-slack SQP step
conversion, bounded nonlinear least-squares-plus-linear-objective and
equality-constrained linear-objective SQP drivers with exact residual-penalty
merit globalization, elastic equality fallback, and residual-only restoration
fallback, and a bounded nonlinear least-squares SQP driver with
finite-difference Jacobians, merit-function backtracking,
equality/inequality/validation/repeated-solve evidence, bounded best-effort
reporting, and fail-closed residual validation
(`REQ-TRAJOPT-003`). Remaining WP-07.2 work: production constrained large-sparse NLP backend adapters,
production sparse KKT factorization, fully stationary free-control
brachistochrone NLP closure, general Bryson-Denham nonlinear solver closure
beyond the fixed-mesh dense-QP benchmark, and full PMP/costate
optimality pipeline closure.
WP-07.3: **partial** — `src/pseudospectral.rs` now exposes
`LegendreGaussRadauGrid` and `legendre_gauss_radau_grid` as a deterministic
left-LGR grid substrate with `-1` included as the first collocation node, `+1`
appended as a terminal support node, positive quadrature weights, an `N x N+1`
barycentric differentiation matrix, support-value differentiation, and
collocation-value integration helpers. It also exposes
`LgrPseudospectralSegment`, `LgrPseudospectralSegmentReport`,
`LgrPseudospectralTranscription`, `LgrPseudospectralTranscriptionReport`,
`evaluate_lgr_pseudospectral_segment`, and
`evaluate_lgr_pseudospectral_transcription` for physical intervals, reporting
collocation times, physical quadrature weights, differential residuals,
segment-boundary continuity residuals, residual counts, residual norms, and
quadrature cost through the existing collocation dynamics/Lagrange callbacks.
It also exposes `LgrPseudospectralDecisionLayout` and
`LgrPseudospectralDecisionReport` plus
`LgrPseudospectralDecisionBoundsReport`,
`LgrPseudospectralReferenceProfileMapping`, and
`LgrPseudospectralReferenceProfileReport` for fixed-time flat decision vectors,
validating mesh times, per-segment LGR degrees, state/control dimensions,
state/control block offsets, total variable counts, and finite decision values;
packing row-major support-state and collocation-control blocks; returning
segment support-state, collocation-control, and mesh-node state slices,
including zero-control slices; decoding decisions into validated multi-segment
transcriptions; and evaluating decoded decisions through the existing LGR
transcription evaluator. It also exposes
`LgrPseudospectralBoundaryConstraint`, `LgrPseudospectralScalarVariable`,
`LgrPseudospectralSoftPathConstraint`,
`LgrPseudospectralHardPathConstraint`,
`LgrPseudospectralNonlinearHardPathConstraint`,
`LgrPseudospectralPathInequalityReport`,
`LgrPseudospectralNonlinearPathInequalityReport`,
`LgrPseudospectralQuadraticControlCost`, `LgrPseudospectralMayerCost`,
`LgrPseudospectralObjectiveReport`,
`LgrPseudospectralKktCollocationMultiplierSample`,
`LgrPseudospectralKktResidualMultiplierReport`,
`LgrPseudospectralKktCollocationCostateEstimate`,
`LgrPseudospectralKktCostateEstimateReport`,
`LgrPseudospectralKktHamiltonianSample`,
`LgrPseudospectralKktHamiltonianReport`,
`LgrPseudospectralFixedMeshDenseConstrainedSqpReport`,
`LgrPseudospectralFixedMeshDenseQuadraticQpReport`,
`LgrPseudospectralFixedMeshDenseSqpReport`,
`LgrPseudospectralFixedMeshOcp`, and
`LgrPseudospectralSparsePathInequalityJacobianReport`,
`LgrPseudospectralColoredSparseResidualJacobianReport`,
`LgrPseudospectralSparseResidualJacobianReport`, plus
`LgrPseudospectralFixedMeshResidualReport` for fixed-time boundary, soft-path,
quadratic control objective residual, hard scalar path inequality, nonlinear
hard path inequality assembly, finite-difference sparse columnwise and colored
residual-Jacobian reporting, analytic/autodiff residual-Jacobian reporting,
active scalar hard-path constrained sparse-KKT step and diagnostic handoff evidence, Mayer/Lagrange
objective reporting, raw residual-multiplier/costate/Hamiltonian diagnostics,
residual-only sparse-KKT SQP scalar/nonlinear hard-path feasibility, optional
final active constrained-KKT report, final-multiplier evidence, and nonlinear
path-inequality Jacobian reporting, plus
feature-gated fixed-time dense QP/SQP wrapping, validating
mesh-node boundary targets/components, collocation-node
scalar soft/hard path bounds/components, nonlinear hard vector path bound
dimensions/samples, nonlinear path outputs, finite-difference sparse-Jacobian
settings, stable residual/nonlinear-inequality row dimensions, colored
residual-Jacobian report consistency, analytic/autodiff residual-Jacobian
dynamics sample and report consistency, finite Mayer objective values, objective
report totals, CSC sparse residual/path-Jacobian report shape, finite
decision-bound reports, optimized-reference mappings/reports/I-load payloads,
and control-cost components, appending boundary
residuals after differential and continuity rows,
then path and objective residuals, transcribing hard path bounds into dense
`Aineq * decision <= rhs` rows with lhs/violation helpers, merging hard scalar
path bounds into dense solver decision-bound reports, emitting strictly
increasing optimized `ReferenceProfileSample` sequences from LGR support-state
samples while skipping duplicate segment-boundary samples and packaging them
into validated postcard `ILoadPayload` artifacts, evaluating nonlinear
hard path bounds as bound-split `g(z) <= 0` rows with raw path values and
max-violation reporting, evaluating scalar objectives from LGR-quadrature
Lagrange costs, configured quadratic control costs, and arbitrary terminal
Mayer callbacks, splitting caller-supplied KKT residual multipliers by LGR row
family, estimating raw collocation costates from differential multipliers,
evaluating raw Hamiltonian diagnostics at LGR collocation samples, computing
deterministic central-finite-difference CSC sparse
columnwise and colored residual Jacobians, caller-supplied analytic
dynamics-Jacobian and forward-mode autodiff CSC sparse residual Jacobians,
sparse Gauss-Newton normal blocks, sparse augmented-KKT matrices, active scalar
hard-path constrained sparse-KKT step reports with active inequality multipliers
and constrained sparse-KKT diagnostic handoff adapters,
residual-only sparse-KKT SQP accepted-step iteration reports with scalar
hard-path decision-bound enforcement, nonlinear hard-path feasibility
preservation, final bound violations, scalar hard-path violations, final
nonlinear hard-path violations, final KKT multipliers, and optional final active
constrained-KKT step reports, plus nonlinear
path-inequality Jacobians with coefficient lookup and dense expansion
helpers, forming feature-gated dense Clarabel equality-constrained
quadratic-control QPs for affine LGR residual models, calling the feature-gated
`DenseSqpSolver` lanes for fixed-time residual restoration, terminal-state
linear objectives, and constrained nonlinear hard-path row consumption, and
reporting row counts, residual norms,
max residuals, quadrature cost, quadratic
control cost, scalar objective totals, final dense-SQP residuals, terminal
objective values, decision-bound violations, scalar hard-path violations, and
nonlinear hard-path violations.
It also exposes
`LgrPseudospectralMeshRefinementSettings`,
`LgrPseudospectralSegmentErrorEstimate`,
`LgrPseudospectralMeshRefinementAction`,
`LgrPseudospectralSegmentRefinementRecommendation`,
`LgrPseudospectralMeshRefinementReport`, and
`estimate_lgr_pseudospectral_mesh_refinement` for candidate-vs-reference
support-state error estimates and deterministic accept/raise-degree/split
recommendations, plus `LgrPseudospectralRefinedSegmentMapping`,
`LgrPseudospectralMeshRefinementApplication`, and
`apply_lgr_pseudospectral_mesh_refinement` for seeded raised-degree and split
refined transcriptions, and `LgrPseudospectralMeshRefinementIterationReport`,
`LgrPseudospectralMeshRefinementLoopReport`, and
`refine_lgr_pseudospectral_mesh_with_reference` for a caller-supplied
reference-driven estimate/apply loop. Focused tests prove three-point
closed-form node/weight values, exact Radau monomial quadrature through degree
`2N-2`, exact support-polynomial differentiation, exact single-segment and
two-segment transcription/quadrature for `x=t^2`, `x_dot=2t`, continuity
stacking, two-segment flat decision decode/evaluate exactness, zero-control
decision-layout support, malformed decision-layout rejection, fixed-time OCP
boundary residual appending, boundary mismatch reporting, soft path residual
and LGR-weighted quadratic control objective residual appending, hard path dense
inequality transcription and violation reporting, hard path decision-bound merge
evidence for dense solver consumption, feature-gated fixed-time dense SQP
scalar-integrator minimum-energy dense-QP benchmark evidence recovering
`x(t)=t`, `u(t)=1`, cost `1`, public closed-form LGR brachistochrone
transcription/time-cost evidence against the cycloid fixture, public
COPS/YAPSS Goddard fixture evidence pinning the dimensionless dynamics,
singular-arc relation, fixed-mesh LGR seed residual accounting,
residual-localized split-refinement seeding, and fixed-mesh plus split-refined
terminal-objective handoff plus published final-altitude gap audit,
feature-gated public COPS/YAPSS Goddard fixed-mesh and split-refined
terminal-altitude objective dense-SQP evidence improving
terminal altitude and reducing equality residual from the seed while reporting
the current fixed-time handoff outside the published final-time, altitude, and
residual bands, feature-gated
moderate-mesh fixed-time LGR brachistochrone dense-SQP free-control feasibility
recovery from a perturbed cycloid control decision, feature-gated
fixed-time dense QP Bryson-Denham state-constrained benchmark evidence
recovering the active arc, objective `4`, hard-path feasibility, and
repeat-determinism plus sparse constrained-KKT active-row/multiplier handoff
from the dense benchmark solution through
`scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml`,
feature-gated
fixed-time dense SQP scalar-integrator restoration plus repeat-determinism
evidence, feature-gated
fixed-time dense constrained SQP consumption of nonlinear hard-path rows,
optimized reference-profile and postcard I-load round-trip evidence,
nonlinear hard path inequality
evaluation and malformed nonlinear path rejection, sparse residual-Jacobian
numeric evidence against the LGR differentiation and boundary-row reference,
colored residual-Jacobian numeric evidence against the columnwise report,
analytic residual-Jacobian numeric evidence against the finite-difference
report, forward-mode autodiff residual-Jacobian numeric evidence against the
analytic report, sparse nonlinear path-inequality Jacobian numeric evidence
against a linear path-function derivative reference, Mayer/Lagrange objective
numeric evidence with terminal state reporting, raw residual-multiplier
row-family slicing plus raw collocation-costate and Hamiltonian diagnostic
evidence, sparse Gauss-Newton normal-step evidence against dense normal
references, sparse augmented-KKT assembly/LDL/step, active scalar hard-path
constrained sparse-KKT multiplier and diagnostic handoff evidence through
`lgr_fixed_mesh_ocp_sparse_constrained_kkt_step_reports_active_hard_path_multipliers`,
and sparse-KKT-derived multiplier diagnostic evidence, residual-only sparse-KKT SQP
convergence/final-multiplier, hard-bound enforcement, and nonlinear hard-path feasibility evidence through
`lgr_fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers`
and
`lgr_fixed_mesh_ocp_sparse_kkt_sqp_enforces_hard_path_decision_bounds` and
`lgr_fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility`,
and validated quadrature-normalized sparse-SQP PMP/KKT certificate reports that
prefer final active constrained-KKT multipliers when available and round-trip
through optimized-reference I-load payloads via
`lgr_sparse_kkt_sqp_pmp_kkt_certificate_attaches_to_optimized_reference_iload`,
including optional validated same-mesh costate-comparison summaries,
malformed
sparse/colored/analytic/autodiff residual, objective, and path-Jacobian
input/report rejection, malformed boundary/path/objective/decision rejection,
exact candidate-vs-reference ph convergence, degree-raise and split
recommendation behavior, seeded refined-transcription state/control
interpolation, reference-driven loop
convergence/cap reporting, checked
`lgr-spectral-convergence-tolerance.toml` smooth-arc interpolation error
collapse for `x(t)=exp(t)`, the `lgr-scalar-integrator-benchmark.toml` solved
validated-toy scalar-integrator benchmark fixture, the
`brachistochrone-cycloid-benchmark.toml` public closed-form LGR
transcription/time-cost fixture plus moderate-mesh dense-SQP feasibility
recovery evidence, the
`bryson-denham-state-constrained-benchmark.toml` public active-arc LGR
dense-QP plus sparse constrained-KKT handoff fixture, the
`goddard-cops-benchmark.toml` public COPS/YAPSS
Goddard dynamics, singular-arc, fixed-mesh seed residual-accounting,
residual-localized split-refinement seeding, fixed-mesh terminal-objective
accounting, split-refined terminal-objective handoff, and published
final-altitude gap fixture, and scalar-PMP LGR dense-QP
quadrature-normalized covector bridge evidence via
`lgr_dense_qp_duals_feed_pmp_costate_convention_bridge`,
and malformed input rejection
(`REQ-TRAJOPT-005`). Remaining WP-07.3 work: general LGR nonlinear benchmark
solver closure, full Goddard nonlinear solver/final-altitude closure, full
constrained external sparse NLP integration, and production nonlinear covector
scaling/sign mapping beyond the scalar dense-QP bridge.
WP-07.4-a: **satisfied** through the new portable `openbmp-convex` crate, which
houses `ClarabelSettings`, `deterministic_clarabel_settings`,
`ConvexMatrixError`, and `ClarabelCscMatrixBuilder` behind a `conic` feature.
`openbmp-fc` and `openbmp-trajopt` now delegate their existing deterministic
settings helpers to that shared profile without creating an FC/trajopt edge, and
`landing.rs` uses the shared CSC builder for its SOCP epigraph matrix
(`REQ-TRAJOPT-007`).
WP-07.4: **satisfied** through a feature-gated fixed-time LCvx
powered-descent-to-state SOCP substrate in `openbmp-trajopt::lcvx`: local
Cartesian position/velocity/log-mass nodes, ZOH interval-control or FOH
node-control thrust-acceleration and `sigma` variables, constant-gravity
dynamics, log-mass depletion, thrust epigraph SOC cones, optional glideslope
and pointing SOC cones, deterministic Clarabel settings, lossless
`sigma = ||u||` diagnostics, repeat-deterministic toy vertical-descent evidence,
known-solution FOH node-control vertical-descent evidence, optional mass-log
Taylor physical thrust-force bounds with linear upper rows and SOC lower rows,
deterministic golden-section time-of-flight search with bracket-shrink and
midpoint-improvement evidence, malformed-input and malformed-search-settings
rejection, and a compile-fail target-vocabulary tripwire with no
lat/lon/range/aimpoint fields. The first provenance-pinned Acikmese-Ploen Mars
glideslope benchmark now checks final mass within `0.5%` of the published
fuel-use result with strict `< 1e-6` feasibility/lossless diagnostics and
repeated-solve bit determinism (`REQ-TRAJOPT-006`). For roll-up accounting,
WP-07.4-a is the satisfied conic-plumbing enabler inside the WP-07.4 bucket;
WP-07.4 is the implemented work-package count.
WP-07.5 … WP-07.6:
WP-07.5: **partial** through a feature-gated discrete-affine SCvx substrate in
`openbmp-trajopt::scvx`: FOH affine dynamics, interval virtual controls, box
trust regions, active-on-reference state-triggered linear constraints,
always-active per-node linear state/control path constraints, second-order-cone state/control path constraints,
accepted/rejected iteration diagnostics with `rho`, merit, trust-region radius,
weighted terminal error, virtual-control norm, state-triggered violation,
linear-state violation, linear-control violation, and second-order-cone violation, plus a
forward-Euler FOH linearization helper that central-differences
caller-provided continuous dynamics into row-major affine segments, a continuous
FOH outer-loop solver that repeatedly rebuilds around the accepted trajectory
until reference state/control updates settle, FOH problem
builders that assemble generic continuous-dynamics, flat 13-state/6-control
rigid-body, 14-state mass-depleting body-thrust powered-landing,
reduced-aero-coupled rigid-body and powered-landing, and feature-gated
`openbmp-aero::AeroMethod`-coupled rigid-body and powered-landing references
into discrete affine SCvx problems, and a flat
13-state/6-control rigid-body dynamics helper with ECI position/velocity,
body-to-ECI `wxyz` quaternion, body-frame angular velocity, ECI force,
body-frame moment, mass, row-major body inertia, inertia-rate coupling, and
external acceleration, plus Szmuk-style powered-landing dynamics with mass
depletion, body-thrust rotation, thrust-arm torque, and provenance-pinned
Szmuk-Acikmese constants/reference handoff plus a powered-landing envelope
report for mass, thrust, gimbal, tilt, angular-rate, and glideslope
violations, and a linear-envelope row builder for per-node mass,
glideslope-halfspace, axial-thrust, and gimbal-halfspace constraints, plus a
reference-linearized envelope builder for finite-facet upper-thrust,
angular-rate, and body-tilt rows, plus a conic-envelope builder for exact
glideslope, upper-thrust-norm, gimbal, and angular-rate SOC rows with body
tilt still reference-linearized, plus reduced `CN/CD/CM` aero
force/moment composition that mirrors `openbmp-aero` schema 1 and adds aero
loads into the rigid-body control vector, plus a feature-gated bridge from any
`openbmp-aero::AeroMethod` into the same SCvx rigid-body and powered-landing
load paths and FOH builders, plus
deterministic golden-section time-of-flight search over caller-built fixed-time
SCvx problems with a merit-plus-time-penalty objective and per-candidate
terminal/virtual-control diagnostics, plus fixed-time candidate diagnostics
that retain terminal/virtual-control/linear/cone rows for non-converged
attempts, plus a deterministic continuous-FOH time-of-flight search wrapper
over caller-built fixed-time continuous relinearization solves with
per-candidate outer-loop diagnostics and a matching continuous-FOH sweep
wrapper, plus a deterministic multi-window free-final-time sweep wrapper that records each window's search report or
stable failure reason plus optional failed-candidate diagnostics and ranks the
best converged window. Evidence covers scalar affine-integrator convergence,
repeated-solve bit determinism, inactive/active trigger behavior, affine
linear state/control path and second-order-cone constraint enforcement,
continuous-dynamics linearization numerics, continuous FOH outer-loop reference
refresh to convergence, continuous FOH full-reference update handoff for
reference-dependent path-row refresh, deterministic free-final-time bracket
shrinkage and repeatability on a scalar integrator, deterministic
continuous-FOH free-final-time bracket shrinkage plus sweep partial-failure
diagnostics with per-candidate outer-loop diagnostics on a scalar integrator, provenance-pinned Szmuk
in-plane powered-landing free-final-time bracket shrinkage to the published
agreement tolerance with candidate diagnostic rows, deterministic scalar
time-sweep partial-failure diagnostics, provenance-pinned Szmuk initial-time
guess sweep diagnostics over every declared initial-time window, weighted terminal targets
leaving zero-weight components untargeted, rigid-body quaternion/Euler
dynamics numerics, rigid-body linearization handoff evidence, rigid-body FOH
problem assembly, powered-landing dynamics numerics, powered-landing FOH
problem assembly, continuous-FOH solve convergence on a 14-state
vertical-translation powered-landing toy with terminal/virtual-control residual
and solved-envelope tolerance evidence, provenance-pinned Szmuk-Acikmese constants and powered-landing
reference handoff with final mass untargeted, powered-landing linear, reference-linearized, and conic envelope
row generation, solver-row handoff, and full-budget conic-envelope affine-solve
evidence that closes the terminal residual, reports the remaining
virtual-control gap, and enforces envelope rows on the pinned Szmuk reference,
one-window conic-envelope free-final-time sweep diagnostics that report
candidate non-convergence on the virtual-control gap and retain fixed-time
terminal/virtual-control/linear/cone rows directly on the failed window, a pinned in-plane
single-linearization affine FOH solve that reduces virtual-control/dynamics-defect
norm, proves raw quaternion drift blocks strict powered-landing relinearization
until the exported reference-state quaternion projection helper normalizes the
handoff, proves the continuous FOH reference-projection hook reaches the next
Szmuk relinearization instead of erroring on that drift, proves the continuous
FOH full-reference update hook refreshes conic envelope rows across Szmuk
powered-landing relinearizations, reduces the conic continuous-FOH
virtual-control gap under a relaxed affine tolerance, reports the remaining
reference-update gap honestly, proves the continuous-FOH free-final-time
fixed-candidate diagnostic reports that same conic reference-update gap at the
pinned Szmuk center time, proves failed continuous-FOH free-final-time sweep
windows retain center fixed-candidate diagnostics across every declared Szmuk
initial-time window, and continues reporting normalized-envelope
violations honestly, direct quaternion projection helper
normalization and malformed-input rejection, powered-landing envelope satisfaction and violation-report
evidence, reduced-aero rigid-body and powered-landing FOH problem assembly with
load propagation,
`openbmp-aero::DeckLookup` rigid-body and powered-landing FOH problem assembly
with load propagation,
reduced-aero force/moment mapping and ECI rotation, aero-coupled rigid-body
and powered-landing load addition, `openbmp-aero::DeckLookup` bridge and
rigid-body plus powered-landing coupled-dynamics evidence,
malformed solver/linearization/rigid-body/aero/builder/second-order-cone/powered-landing-conic-envelope input rejection, and a compile-fail
target-vocabulary tripwire
(`REQ-TRAJOPT-008`). Remaining WP-07.5 work: nonlinear 6-DoF/aero model
generation beyond the current flat, powered-landing, reduced-aero
rigid-body/powered-landing, and aero-method rigid-body/powered-landing builder
substrate, full published Szmuk trajectory closure, and flight-real-time
validation.
WP-07.6: **partial** through `src/pmp.rs`, feature-gated `src/sqp.rs`,
`src/collocation.rs`, and `src/pseudospectral.rs`:
generic local state/control/costate
Hamiltonian-profile evaluation reports `H = L + lambda^T f` and maximum drift
from the first sample; a generic finite-difference Hamiltonian
control-stationarity witness reports per-sample `dH/du` gradients, maximum
gradient norms, and a tolerance verdict for free-control PMP checks, and
`evaluate_pmp_hamiltonian_control_stationarity_from_report` reuses stored
Hamiltonian report samples to attach those summaries to certificate payloads;
same-mesh
PMP-vs-direct-method/KKT costate comparison
reports component and vector-norm differences with finite time/costate
tolerances; and the scalar fixed-time minimum-energy integrator witness solves
the PMP two-point condition by deterministic costate shooting, reports
per-iteration terminal residuals, emits a Hamiltonian constancy report, compares
the solved PMP costate against a same-mesh KKT estimate, proves zero `dH/du`
on the optimum and nonzero stationarity residuals for off-optimum controls, and
proves repeated solve bit determinism plus malformed input rejection; the common
`PmpDirectCostateConvention` path requires an explicit raw-to-PMP scale, and the
Hermite-Simpson node-costate and LGR collocation-costate reports expose
`pmp_costate_convention_report` adapters with focused generic/HS/LGR scaling
tests; the dense-QP lane exposes equality/inequality dual vectors and checks the
scalar fixed-time minimum-energy equality multiplier against the PMP costate on
the same mesh, while the Hermite-Simpson dense-QP wrapper matches
solver-produced equality duals to the scalar PMP costate through an explicit
convention report and exposes `dense_qp_pmp_hamiltonian_report`, and the LGR
dense-QP wrapper matches quadrature-normalized solver-produced equality duals to
the scalar PMP costate at collocation nodes and exposes
`dense_qp_pmp_quadrature_weight_normalized_hamiltonian_report`, with both mapped
profiles yielding scalar PMP Hamiltonian constancy and round-tripping
Hamiltonian/control-stationarity/costate certificates through validated I-load payloads;
Hermite-Simpson and LGR
sparse augmented-KKT reports also expose `sparse_kkt_pmp_costate_convention_report`
and `sparse_kkt_pmp_hamiltonian_report`; constrained sparse-KKT step reports
feed equivalent residual-multiplier/costate/Hamiltonian diagnostic adapters,
including public Bryson-Denham active-set PMP diagnostic handoff evidence and
PMP Hamiltonian adapters, while LGR sparse-KKT adds
`sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report` and
`sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report`, with
identity-convention, public Bryson-Denham active constrained-KKT diagnostic and
direct certificate/I-load handoff, and LGR weight-normalized tests proving the
common PMP evaluator wiring against the
existing raw diagnostics and without inferring production covector scaling; residual-only sparse-KKT SQP reports provide
Hermite-Simpson and LGR scalar/nonlinear hard-path feasibility evidence,
optional final active constrained-KKT reports, and accepted-iterate final multipliers for the same PMP/KKT
report path through
`fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers`
and
`fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility`,
`lgr_fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers`, and
`lgr_fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility`;
the accepted sparse-KKT SQP report surfaces now build validated HS/LGR PMP/KKT
certificates, prefer the final constrained active-row report when present, and
round-trip those certificates through optimized-reference I-load payloads with optional validated same-mesh costate-comparison summaries;
direct active constrained-KKT report surfaces now build validated HS/LGR
PMP/KKT certificates and round-trip public Bryson-Denham active-set certificates
through optimized-reference I-load payloads;
validated-toy inline, PMP-derived file-backed, and HS dense-SQP/sparse-SQP
PMP/KKT-certified file-backed optimized-reference runner scenarios export valid
reference telemetry and FC actuator command-stream evidence;
`ILoadHeader` now carries `ILoadObjectiveClass` audit metadata
with schema `1.3.0`, explicit
optimized-reference and day-of-launch recentering labels, default
terminal-residual-correction labeling for existing apogee-correction paths, and
optional fail-closed `ILoadOptimalityCertificate`
Hamiltonian/control-stationarity/costate tolerance evidence for PMP-derived
file-backed optimized-reference payloads; HS/LGR
optimized-reference emitters expose
`optimized_reference_iload_payload_with_optimality_certificate` to attach that
evidence to validated postcard payloads, accepted dense-SQP HS/LGR solutions
expose final sparse-KKT PMP certificate reports that round-trip through
optimized-reference postcard payloads, accepted sparse-SQP HS/LGR report
surfaces expose equivalent PMP/KKT certificate payload round-trips, all
certificate reports can attach validated same-mesh PMP-vs-KKT costate summaries
through `with_pmp_costate_comparison` and validated control-stationarity
summaries through `with_pmp_control_stationarity` with a stored
`control_stationarity_report`, and the runner consumes dense-SQP and sparse-SQP
certified file-backed I-loads through the normal optimized-reference bridge
(`REQ-TRAJOPT-009`). The constrained sparse-KKT diagnostic adapters close the
step-report-to-PMP handoff for active scalar hard-path rows, without claiming
production constrained large-sparse NLP optimality. Remaining WP-07.6 work: production Hermite-Simpson/LGR
constrained large-sparse NLP residual-multiplier closure and a production indirect
optimality witness beyond the validated-toy scalar arc.

### 08 — Environment, gravity & frames

Baseline confirmed: `Egm2008ZonalGravity` is zonal-only, hard-capped at
degree 6 (`gravity.rs`), and WP-08.1 is now **partial** through
`TesseralGravity`, `FiniteDifferencePinesGravity`,
`DegreeTwoTesseralCoefficients`,
`NormalizedDegreeTwoTesseralCoefficients`, `NormalizedHarmonicCoefficient`,
`NormalizedHarmonicField`, `NormalizedHarmonicFieldIter`,
`IcgemGfcNormalizedField`, `HarmonicLongitudeTrigonometry`, `HarmonicTruncation`,
`HarmonicSynthesisPlan`, `HarmonicSynthesisTier`, `PinesLegendreTable`,
`PinesLongitudePolynomials`, `PinesSynthesisPoint`, `PinesPotentialSum`,
`GottliebPotentialSum`, and
`TideSystem`: the first degree-2/order-2 static harmonic surface supports C20,
C21/S21, and C22/S22 terms, converts fully-normalized degree-2 blocks, stores
normalized `Cbar/Sbar` fields in deterministic packed `(n, m)` order, exposes
explicit `without_central_term` stripping for full coefficient files whose
`Cbar00` central term must not be double-counted by
point-mass-plus-correction evaluators, checked fully-normalized scale factors,
a std-gated fail-closed
`from_normalized_toml_str` parser for the OpenBMP normalized harmonic TOML
schemas, a std-gated fail-closed `from_icgem_gfc_str` parser for static
ICGEM/NGA-style fully-normalized `gfc` coefficient lines plus
`from_icgem_gfc_str_with_metadata` source `µ`/radius/source-degree validation
and direct `FiniteDifferencePinesGravity::new_from_icgem_gfc_str`
construction, checked runtime
high-degree EGM2008 tier requests for the planned 70/120/360 truncations that
must resolve against a concrete field envelope and the bounded Pines scratch
tables, a bounded `cos(mλ)`/`sin(mλ)` recurrence table, a singularity-free
Pines direction-cosine longitude-polynomial table, a reusable
truncation-envelope validator, a bounded Holmes-Featherstone/Pines `A_nm(u)`
recurrence table, a body-fixed synthesis-point validator, a normalized Pines
scalar-potential correction sum,
a normalized Gottlieb-style scalar-potential recomposition oracle cross-checked
against the Pines sum, Pines and Gottlieb-style symmetric finite-difference
acceleration and acceleration-gradient oracles plus a public static
`GravityModel` wrapper with `new_from_full_normalized_field` central-term
stripping, `acceleration_gradient_eci_s2`, and checks against the existing
analytic degree-2 tesseral terms and point-mass tensor, and default-zero
coefficient-slot iteration, rebuilds the current
`Egm2008ZonalGravity` J2-J6 truncation from the
`data/gravity/egm2008-zonal-degree6-normalized-v1.toml` normalized zonal
fixture through the public TOML parser, parses the
`data/gravity/wgs84-degree2-normalized-v1.toml` provenance pin through the same
path, parses the synthetic non-zonal
`data/gravity/synthetic-degree4-normalized-field-v1.toml` fixture through the
general normalized-field schema, parses
`data/gravity/synthetic-degree4-normalized-icgem-v1.gfc` through the
ICGEM-style static coefficient-and-metadata schema, parses the provenance-pinned
clipped `data/gravity/egm2008-degree10-normalized-icgem-v1.gfc` real EGM2008
degree-10/order-10 block through the same source-metadata path, builds
`FiniteDifferencePinesGravity` directly from that GFC text, and proves the
degree-10 real non-zonal block changes acceleration relative to the degree-2
truncation. Schema-v3 `gravity = "egm2008"` scenarios can now optionally
resolve that ICGEM GFC text through `environment.egm2008_coefficients_file`,
custom `egm2008_degree`/`egm2008_order`, or the named `egm2008_tier`
selectors (`degree70`, `degree120`, `degree360`); the runner builds the
source-constant finite-difference Pines transition model, wraps it in
`EarthFixedGravity`, and point-mass coverage proves the file-backed path differs
from the legacy zonal-only EGM2008 path. Synthetic degree-70 tier coverage also
proves named tiers resolve through the same GFC+Pines path and reject undersized
source headers fail-closed. The degree-2/order-0 WGS84 J2 path remains
byte-identical to `J2Gravity`, stays finite near the pole, and rejects unsupported
degree/order/duplicate/out-of-envelope coefficient requests fail-closed
(`REQ-ENV-001`). Missing for full WP-08.1:
full runtime degree-70/120/360 Pines synthesis over real EGM2008 coefficient blocks,
full analytic normalized Gottlieb acceleration-gradient oracle, full high-degree
EGM2008 coefficient-block provenance/tripwire, and NGA HARMONIC_SYNTH benchmark
tables. IAU 1976/1980 equinox frames remain
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
WP-08.3 is now **implemented**: `EarthOrientationSample` carries
`cip_offset_x_rad`/`cip_offset_y_rad`, legacy constructors default both fields
to exact zero, `with_cip_offsets` rejects non-finite offsets fail-closed, and
`EarthOrientationTable::sample` interpolates dX/dY alongside UT1-UTC, polar
motion, and optional LOD. The runner `openbmp-eop-v1` parser accepts optional
`cip_offset_x_arcsec`/`cip_offset_y_arcsec` pairs and rejects rows that declare
only one offset. `TimeScaleBridge` now exposes pinned UTC↔UT1/TAI/TT
conversions plus the compact TT↔TDB approximation, and an ERFA `eraDtdb`-
compatible TT↔TDB path backed by the transcribed Fairhead-Bretagnon coefficient
table with upstream C reference pins. Runner celestial UTC/TT/TDB epoch
conversion now uses the ERFA-compatible helper, supplies topocentric dtdb
observer geometry from `[frames.local_origin]` when present, and samples
`epoch.eop` UT1-UTC for UTC-to-TDB phase when available. Four-day provenance-pinned
finals2000A-derived raw and equivalent `openbmp-eop-v1` fixtures exercise real
polar motion, UT1-UTC, LOD, dX/dY parser reachability, and fixed-width row MJD
to scenario-relative time conversion. A four-day provenance-pinned EOP 14 C04
fixture exercises raw C04 header/data ingestion in the same path (`REQ-ENV-004`).
No tides, SRP macro-models, Lense-Thirring, or
de Sitter terms yet. WP-08.4 is now **partial**: the HAL-portable physics frame
layer exposes ERFA-pinned CIO primitive helpers for ERA (`eraEra00`), TIO
locator `s'` (`eraSp00`), CIO locator `s` from caller-provided X/Y (`eraS06`),
CIO-adjacent IAU 2006 Fukushima-Williams pieces (`eraObl06`, `eraPfw06`,
`eraFw2m`, `eraBpn2xy`), IAU 2000A/2006 nutation (`eraNut00a`,
`eraNut06a`), matrix-backed X/Y/s generation (`eraPnm06a`, `eraXys06a`),
C2I from caller-provided X/Y/s (`eraC2ixys`), polar motion (`eraPom00`), and
`RPOM * R3(ERA) * RC2I` assembly (`eraC2tcio`, `REQ-ENV-005`). `CioXysTable` and
`CioFrameModel` now compose caller-provided or generated X/Y/s with pinned EOP
dX/dY, UT1/TT dates, ERA, TIO `s'`, and polar motion into GCRS↔ITRS matrices.
`FrameProfile::IersCio` and runner `frame_profile = "iers-cio"` wiring consume
SHA-pinned `epoch.eop` and leap-second inputs, default to generated IAU
2006/2000A X/Y/s, and still accept SHA-pinned `epoch.cio_xys` tables while
leaving `IersTabulated` unchanged. Full generated GCRS→ITRS matrices are pinned
against a three-case ERFA UTC/EOP grid, and generated X/Y/s is checked against
IERS TN36 Ch.5 electronic Tables 5.2a, 5.2b, and 5.2d reference values.
`EarthFixedGravity` now rotates the degree-2 tesseral substrate through
ToyFixedEarth, WGS84 uniform rotation, and generated-X/Y/s `IersCio` frame
contexts, and schema-v3 runner scenarios can consume the same bounded
degree-2 model through `environment.gravity = "tesseral"` or
`gravity_base = "tesseral"` under third-body gravity. CIO primitive/profile
substrate is now **partial** because the
tesseral-gravity-through-`IersCio` Orekit force-stack acceptance remains open.
WP-08.5 … WP-08.7: **not started**.

### 09 — Sensors, navigation & actuators

Baseline confirmed: GNSS is still a position oracle and actuators remain
first-order. WP-09.1 is now **partial**: `openbmp-sensors::SpecificForceTruth`
exists, `[sensors.<imu>] specific_force_source = "force_accumulator"` is
accepted only for IMUs, and the runner FC bridge can fill IMU truth from
non-gravity force/mass plus rigid-body moment-derived angular acceleration.
The `stationary_rotating_frame` validation source covers the rotating-Earth
stationary body-frame acceptance case. The finite-difference source remains
default. WP-09.2 is now **partial**: `openbmp-vehicle::SecondOrderServo` exists
as a deterministic second-order bandwidth/rate/acceleration/backlash substrate,
and `[[vehicle.assembly.effectors]] kind = "second_order_servo"` is parsed and
built by the runner rack; `openbmp-vehicle::RcsMinimumImpulseBit` and
`PwpfModulator` provide the deterministic RCS MIB/PWPF substrate, and schema v3
`direct_torque` effectors can opt into scalar RCS pulse quantization or a
body-frame bank with deterministic geometry allocation and per-thruster
blowdown scaling built by the runner rack. Physical RCS thrusters can also bind
to declared ullage tanks, use live tank-pressure scale to derate thrust, and
emit additive tank drain from actual impulse. `openbmp-vehicle::RcsCoupledAllocator`
adds a vehicle-level vector torque-impulse allocation substrate over physical
thruster geometry, and schema v3 `mode = "coupled_bank"` groups one roll, pitch,
and yaw `direct_torque` effector through that allocator in the runner rack.
Optional coupled-bank PWPF gates each member axis before shared vector
allocation. Load-time oscillatory effector faults now inject deterministic
sinusoidal command offsets before the normal actuator path. A deterministic
rate-limit/backlash describing-function helper now anchors the WP-09.2
first-harmonic limit-cycle evidence. Resonant actuator extensions remain open.
WP-09.3 is now **partial**: `openbmp-sensors::SyntheticAirData` exposes
compressible subsonic/Rayleigh Pitot impact pressure, Mach inversion, calibrated
airspeed, ISA pressure-altitude inversion, alpha/beta vane geometry, and
fixed-latency `capture_time` evidence on a non-GNSS sensor. Schema validation
and the runner bridge can build `kind = "airdata"` from
`data/sensors/airdata-textbook.toml`; `AirDataSample`, `FcStepInput::with_airdata`,
`AirDataIngest`, `VotedAirDataIngest`, and `openbmp-bridge` protocol v4 now
carry the Pitot/static/vane channels across direct FC publication, redundant
FC ingestion, and bridge packets. Estimator fusion of air-data remains open.
WP-09.4 is now **partial**: `openbmp-sensors::strapdown` exposes
`InertialIncrement`, deterministic delta-theta/delta-v quantization,
constant-truth and RK4 time-varying high-rate window generation with locked
sequence numbers, and a two-sample coning/sculling reference that reports
coning, velocity-rotation, and sculling correction terms before applying the
quaternion update.
`SyntheticImu::with_high_rate` can now stash configured high-rate increment
windows without changing the legacy `SensorMeasurement::Imu` output: the first
or non-advancing timestamp window uses constant truth, and later advancing
timestamps use RK4 over linearly interpolated successive runner truth samples.
Schema/runner wiring accepts IMU-only `[sensors.<imu>.high_rate]` blocks with
sub-sample count plus delta-theta/delta-v LSBs. `openbmp-msgs` exposes the
`sensor.imu_increments` topic, the runner publishes `ImuIncrementWindow` on the
direct FC bus path, and bridge protocol v4 carries `ImuIncrementPacket` windows
through the runner and PIL packet adapters into `FcStepInput` before dispatch;
transport-fault scalar rules can target per-increment delta-theta, delta-v, and
dt fields by sample index. Kernel RK-stage truth export, three/four-sample
algorithms, and Schuler-period free-INS validation remain open. WP-09.5 …
WP-09.12 are **not started**.

### 10 — Flight-software-in-the-loop (XIL)

| WP | Status | Evidence / missing |
|---|---|---|
| WP-10.1 SIL over transport | **implemented** | `Transport` + in-process/stream/child-stdio backends, `LockstepSimMaster`, SHA-256 determinism gate, ZOH tolerance table, 12 tests in `crates/openbmp-runner/tests/fc_transport.rs` |
| WP-10.2 fault library + XIL ports | **implemented** | `fault.rs` (13 scalar + 5 bus transforms), `MaPort`/`EesPort`, lifecycle FSM, `[fc.transport_faults]`, empty-schedule byte-identity, `crates/openbmp-sil/src/xil.rs` evidence adapters |
| WP-10.3 soft-real-time bench | **implemented** | `openbmp-rt` pacer (FreeRun/RealTime/Paced, jitter histogram p50/p99/p99.9, overrun count), runner `rt.rs`, `[realtime]` schema, byte-identity across modes (`tests/realtime.rs`) |
| WP-10.4 FMI 3.0 importer | **partial** | typed instance access, modelDescription parsing, single+multi masters, periodic input-clock activation, checkpointed multi-FMU predictor/corrector macro step, 5 pinned Reference FMUs + FMPy cross-check scripts; missing: complete clocked event scheduling and complete Reference-FMU matrix |
| WP-10.5 FMI 3.0 exporter | **partial** | `export.rs` + `export_abi.rs` (C ABI, XSD validation, point-mass FMU + trace tolerance table); missing: full plant mapping to value references |
| WP-10.6 AFTS containment monitor | **partial** | `openbmp-afts` L2 crate now exposes WGS84-spherical and WGS84-ellipsoid central-gravity `IipPropagator` options, optional WGS84 uniform-rotation longitude correction, optional deterministic `ExponentialAtmosphericDrag`, `ContainmentPolygon`, `ContainmentRule` keep-inside/keep-out modes, scalar `CorridorRule` checks for altitude/speed/flight-path-angle bounds, geospatial `GateRule` checks over successive IIP samples, red/green `ZoneRule` checks with red-zone priority and green-zone union semantics, and latched `AftsMonitor`; schema v3 parses and validates `[afts.propagator]` plus `[afts]` keep-inside, keep-out, corridor, gate, and zone rule tables with evidence strings and optional `evidence_file` SHA pins; the runner samples telemetry-aligned point-mass/rigid-body truth states into a sidecar `AftsRunReport` without changing kernel stop reasons; SIL evidence bundles and the CLI surface include that report; native `SilStimulation` can append AFTS zone rules in memory without mutating package scenario files. Tests cover radial closed-form IIP, a non-radial rotating-Earth closed-form IIP tolerance-table fixture under `data/afts/`, WGS84 Earth-rotation correction, WGS84 ellipsoid geodetic latitude, exponential-drag IIP perturbation, hand-computed point-in-polygon cases, keep-out firing, corridor firing, gate crossing, green/red zone firing, latch behavior, no-impact horizon, no FC dependency, trybuild no-inverse `solve_burn_to_reach` rejection, AFTS parser validation/resolved-file hashing, runner-side latching including ellipsoid corridor altitude, and SIL AFTS zone stimulation. Remaining ceiling: simulation evidence only, not certified range-safety qualification. |
| WP-10.7 PIL on emulated ISA | **partial** | `openbmp-fc::fc_step` now exposes a hardware-portable topic-level step facade (`FcStepInput` -> `FcStepOutput`) that publishes sensor topics, dispatches one tick, captures actuator/health/status outputs, fails closed on missing topic registration, and keeps bridge/runner dependencies out of `openbmp-fc`; `openbmp-bridge` now checks packet/codec/lockstep-validator modules on `thumbv7em-none-eabihf` without default features while host transports/faults/XIL ports remain `std`-gated, and exposes `encode_into`/`frame_into` for caller-owned target buffers; `openbmp-pil` adds a no-std-capable byte/mailbox/framed-endpoint/frame-pump/byte-poll shim that decodes bridge `SensorPacket` bytes or length-prefixed frames, calls `fc_step`, encodes `ActuatorCommandPacket` bytes into caller storage, exposes a `repr(C)` mailbox with status/error codes and a `PilMailboxLayout` offset report for downstream GDB-symbol wrappers, exposes a `PilFrameIo` complete-frame read-step-write loop for downstream UART/semihosting wrappers, exposes a fixed-buffer `PilFramePump` for UART-like byte streams that waits for complete frames and preserves queued/pending frames, exposes `poll_byte_stream_endpoint_once` over `PilByteIo` so a target poll loop can read one chunk, drive the pump, write a completed command frame, and retry pending output after write failure, exposes a safe `PilFirmware` wrapper plus `PilFirmwareSymbolContract` advisory mailbox/step symbol names for downstream target storage, and exposes `PilWcetBudget` / `PilWcetReport` integer instruction-count × CPI deadline evidence helpers; `openbmp-runner::FcRunner::step_bridge_packet` adapts `SensorPacket`/`ActuatorCommandPacket` frames through that facade for host bridge transport while preserving timing instrumentation, `pil_framed_endpoint_matches_runner_bridge_packet_adapter` proves framed PIL command bytes match the runner adapter under the host oracle, `openbmp-runner::pil::RenodePilPlan` renders a validated socket-UART Renode `.resc` plan with telnet negotiation disabled, optional GDB server, and deterministic microsecond global quantum, `GdbPilSymbolBinding` resolves mailbox and step-entry addresses from target ELF32/ELF64 symtab/dynsym bytes or files, `GdbPilMailboxPlan` maps the resolved target mailbox address plus `PilMailboxLayout` into deterministic GDB Remote Serial Protocol memory read/write/breakpoint packets and target-read decoders, `GdbRspSession` drives checked RSP packet exchange over caller-owned streams with ack/checksum handling, target-error handling, memory reads/writes, breakpoint insert/remove, continue, mailbox sensor loading, and mailbox status/command reads, and `PilFrameStream`/`RenodeSocketPilClient` provide the host raw-frame socket client for that UART stream with payload limits and command step/time validation; missing: downstream-exported linked target image, live Renode target execution/equivalence, real UART driver, and measured WCET evidence |
| WP-10.8 cFS in the loop | **not started** | no cFS wiring |

### 11 — Monte Carlo, UQ & validation

| WP | Status | Evidence / missing |
|---|---|---|
| WP-11.0 MC substrate | **implemented** | `openbmp-mc` (Welford, Clopper-Pearson, samples table + convergence trace), `DeterministicRng::for_mc_sample` domain |
| WP-11.1 DoE + sizing | **implemented** | LHS, native Sobol (pinned Joe-Kuo direction numbers under `data/sobol/` + provenance), Owen scrambling, Iman-Conover, Wilks sizing, convergence gate + R-hat, `openbmp mc` subcommands + `tests/mc_cli.rs` |
| WP-11.2 credibility + error budget | **partial** | `openbmp-uq` complete (correlated error budget, aleatory/epistemic classes, 8-factor credibility record, min-is-binding, floor flags wired through the shared campaign credibility helper into `mc summarize`/footprint/fault campaigns); missing: auto-emission by future drivers |
| WP-11.3 nested aleatory/epistemic | **partial** | probability-box + variance split, nested footprint execution, `mc nested-summarize`; missing: generalized nested orchestration beyond landing-footprint |
| WP-11.4 rare events | **implemented** | subset simulation (Au-Beck) + cross-entropy IS on sealed synthetic limit states; compile-fail tripwire `crates/openbmp-testkit/tests/limit_state_no_aimpoint_compile_fail.rs` + UI test |
| WP-11.5 MMS + order verification | **implemented** | manufactured ODE + observed-order/Richardson/GCI (`openbmp verify-order`, testkit verification module) plus bounded deterministic multi-case MMS lambda campaign evidence through `--campaign-cases` |
| WP-11.6 BET reconstruction | **partial** | RTS smoother + batch Gauss-Newton + NEES/NIS in `crates/openbmp-testkit/src/reconstruction.rs`; `openbmp reconstruct synthetic-linear`, `observe-fc`, `export-trajectory`, `export-trajectory-mapping`, and `local-workflow` CLI surfaces; deterministic LOCAL-only workflow manifests record export/mapping/compare commands and guardrails that external references are not read, copied into the repo, or used as flight/targeting input. Remaining gap: full covariance/transition histories for real-FC RTS smoothing and actual external-tool evidence around the export/compare flow |
| WP-11.7 cross-discipline UQ wiring | **partial** | `openbmp-uq::UpstreamMargin`, `CorrelatedErrorBudget::from_upstream_margins`, and doc-02/03/04/05 helper adapters can convert structural bending, aerodynamic coefficient, aerothermal model, and thermochemistry `c_star_efficiency` bands into ordered source-tagged epistemic budget inputs with credibility evidence; `[vehicle.bending.uq]`, `[aero.uq]`, `[aerothermal.uq]`, `StructuralRack`, aero/aerothermal runner helpers, and `EngineRack` gather structural frequency, aerodynamic coefficient, aerothermal `q_conv_w_m2`, and resolved thermochemical liquid-engine `c_star_efficiency` bands into `RunOutcome.upstream_uq`; footprint MC reports also carry runner-gathered upstream UQ for campaign credibility; `CampaignCredibilityInputs` plus `evaluate_campaign_credibility_contract` give MC campaign drivers a typed sidecar-plus-upstream-UQ credibility path, preserving upstream-only correlation matrices and merging sidecar/upstream blocks as independent sources; scalar `mc summarize`, sampled propulsion-fault MC, and `openbmp footprint-mc` use that contract before final evidence is written; missing: future campaign additions still need review/traceability to keep that typed contract |
| WP-11.8 campaign determinism/real-time | **partial** | delivered through doc 12's reducers/checkpointing plus `openbmp-mc` scalar-report byte identity across worker counts and pinned native aarch64 digest coverage; full campaign cross-arch byte gate pending |

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
| WP-12.3-b aarch64 CI lane | **implemented** | native `ubuntu-24.04-arm` `determinism-gate (aarch64)` downloads the same-run x86_64 reference artifact, checks the pinned `openbmp-mc` scalar-report digest, and byte-diffs the fixed-step canonical scenario outputs; traced by REQ-DET-004/V-DET-004 |
| WP-12.4-a dense output | **implemented** | `advance_with_dense_output` for Dopri54/853, state-stable opt-in, off the bit-stable path |
| WP-12.4-b GPU offload boundary | **implemented** | synthetic offline atmosphere-density deck under `data/atmosphere/gpu-offline-density-subset-v1.toml` carries no-live-GPU metadata, UQ band, sibling provenance record, and SHA-256 pin; `crates/openbmp-testkit/tests/gpu_offload_boundary.rs` reproduces the sampled subset through `AtmosphereModel` + `PiecewiseExponentialAtmosphere`, verifies the provenance hash pin, and scans workspace manifests for known GPU framework dependencies; traced by `REQ-DET-005`/`V-DET-005` |

### 14 — Contact dynamics, touchdown & landing

WP-14.1: **implemented** — `openbmp-contact` crate landed (`e4a1a4d`:
half-space geometry, Kelvin-Voigt/Hertz/Hunt-Crossley normal laws,
regularized Coulomb, fixed sub-step stability bound, energy audit;
REQ-CONTACT-001). Scenario `[contact]` schema (`ContactConfig`) and runner
force-adapter wiring are present (`crates/openbmp-runner/src/contact.rs` plus
point-mass/rigid-body/vehicle edits), with contact diagnostics telemetry,
`RunOutcome.contact` endpoint classification, run-level energy audit,
substep-driven kernel step sizing, and point-mass golden-stability proof.
The former gear-leg assembly acceptance gap is closed by WP-14.4's validated
landing-gear rack, per-pad `ContactPair` footpads, pinned four-leg drop
fixture, rest classification, force/moment adapters, section-load recovery,
and <1% energy-audit closure (`REQ-CONTACT-006`/`REQ-CONTACT-007`).
WP-14.2: **partial** — `openbmp-contact` now exposes one-sided scalar
stops, two-sided backlash gaps, monotone latch primitives, and an allocation-free
scalar mechanism evaluator with validation, unilateral force clamping, backlash
dead-zone width evidence, Kelvin-Voigt closed-form stop restitution evidence,
declaration-order latch state updates, and engage-once latch property tests
(`REQ-CONTACT-003`). Schema-v3 `[contact]` can now declare optional
`[[contact.mechanism]]` angular stop/backlash/latch fixtures with kind-specific
fail-closed validation and duplicate-id rejection; schema-v3
`[[multi_body.gimbal_joint]]` can bind one contact mechanism by `mechanism_id`
when the mechanism coordinate matches the engine id, select primary or secondary
gimbal-axis routing with `mechanism_axis`, and the rigid runner feeds that
mechanism response into the selected articulated gimbal generalized force during
shadow derivative/forecast evaluation. Focused root-authority regressions now
prove the same scalar stop force participates in the single-gimbal
`articulated_gimbal_root` handoff and that already-engaged monotone latch state
is carried with the root-authoritative internal state; a latch centered on a
newly delivered live deflection now engages after the root-authority coordinate
refresh. Kernel-facing TVC load-torque snapshots now project onto articulated
gimbal axes as generalized joint force in the derivative/forecast path, and TVC
inertial-reaction snapshots now apply as a root-body external moment; validated
full tail-wags-dog benchmark/coupled-acceleration closure remains open. Schema-v3
`[[multi_body.cfe_loop]]` rows
now feed the single articulated-gimbal shadow CFE solve path and emit solve
telemetry; the constrained forecast also participates in the guarded
single-gimbal root-authority handoff. Schema-v3 `[[multi_body.cfe_gimbal_lock]]`
now provides the first concrete CFE model on that path by deriving residuals,
velocity residuals, and a unit Jacobian from the live articulated gimbal state
(`REQ-MULTIBODY-050`). `openbmp-contact` also now exposes
`PlanarFourBarLoop` as the first concrete four-bar/side-brace geometry helper,
materializing distance residuals, velocity residuals, and two-column Jacobian
rows from live input/output angles (`REQ-CONTACT-009`). Schema-v3
`[[multi_body.cfe_planar_four_bar]]` now makes that concrete row model
scenario-reachable on the current two-axis articulated-gimbal shadow path and
feeds the same CFE solve/telemetry path (`REQ-MULTIBODY-051`). Remaining gap:
authoritative prismatic landing-gear tree propagation, broader vehicle
tree-joint authority, and public CFE/ADAMS benchmark closure.
WP-14.6: **partial** — `openbmp-contact::cfe` now exposes scalar, dense, and
tree-facing Baumgarte CFE solves (`REQ-CONTACT-008`) that validate
caller-assembled effective inverse mass/residual terms, row-major constraint
Jacobians, and caller-owned scratch/output buffers; solve with deterministic
no-pivot LDLᵀ; return Lagrange multipliers and stabilized accelerations; and
project `G^T*lambda` into generalized constraint-force outputs for higher-level
tree integrators. `openbmp-multibody` now assembles those row-space terms from a
live tree's ABA free acceleration and CRBA matrix (`REQ-MULTIBODY-045`).
Schema-v3 `[[multi_body.cfe_loop]]` rows are parsed, validated, and consumed by
the single articulated-gimbal shadow path, where the rigid runner solves the CFE
system and injects projected generalized forces into non-authoritative
derivative/forecast evaluation, with per-loop solve telemetry for multipliers,
stabilized accelerations, and residuals; in guarded single-gimbal
`articulated_gimbal_root` mode, that same constrained forecast is the root
handoff source (`REQ-MULTIBODY-046`, `REQ-MULTIBODY-047`,
`REQ-MULTIBODY-048`, `REQ-MULTIBODY-049`). Concrete
`[[multi_body.cfe_gimbal_lock]]` declarations derive the CFE row from live
articulated gimbal q/qd and share the same solve, projection, telemetry, and
guarded root-authority path (`REQ-MULTIBODY-050`).
The contact substrate now also exposes `PlanarFourBarLoop`,
`PlanarFourBarCfeRow`, and `PlanarFourBarOutputAngleSolutions`, validating
fixed-pivot/link geometry and materializing a scalar distance CFE row plus
closed-form output-angle branches for planar four-bar/side-brace mechanisms
(`REQ-CONTACT-009`). Schema-v3 `[[multi_body.cfe_planar_four_bar]]`
declarations bind that geometry to distinct primary/secondary axes on the
current two-axis articulated-gimbal shadow, materialize live distance and
velocity residuals plus a two-column Jacobian, and emit the standard CFE solve
telemetry (`REQ-MULTIBODY-051`).
The vehicle crate now exposes `LandingGearPrismaticDof` and
`LandingGearPrismaticState` as the first prismatic landing-gear DOF coordinate
contract, validating bounded stroke geometry, deriving from `LandingGearLeg`,
and mapping compression stroke/rate to tree-facing displacement/rate plus
footpad center coordinates without adding a multibody dependency
(`REQ-CONTACT-010`). Schema-v3 `[vehicle.landing_gear]` legs can now opt into
`prismatic_dof`, and the runner publishes per-leg prismatic displacement,
displacement-rate, and body-frame footpad-center telemetry only for declared
legs while preserving the current massless force path (`REQ-CONTACT-011`).
Those prismatic declarations can also bind `mechanism_id` to a declared scalar
mechanism whose coordinate matches the leg id; the parser rejects missing or
mismatched mechanisms, and the runner evaluates the mechanism on live
prismatic displacement/rate, owns latch state, records a
`LandingGearPrismaticMechanismSample`, and adds only the positive compressive
component to the external-normal gear load (`REQ-CONTACT-012`).
Focused tests prove scalar closed form, coupled two-row closed form,
scalar/dense equivalence, tree-loop generalized-force projection, multibody
CFE term assembly, bad tree-loop dimensions, indefinite-matrix rejection,
scenario parser reachability, runner-side gimbal-shadow force injection, and
CFE solve telemetry plus guarded root-authority handoff. They now also prove
parser and runner coverage for a concrete articulated-gimbal CFE lock and
scenario-reachable current-path planar four-bar CFE geometry, plus the
vehicle-side and opt-in scenario/runner prismatic gear-DOF coordinate and
mechanism-force contract. Remaining gap: authoritative prismatic landing-gear
tree propagation, broader tree authority, and public CFE/ADAMS benchmark
closure.
WP-14.3: **implemented** — `openbmp-contact` now exposes anchored stick/slip
friction with static cone breakaway, kinetic sliding, Karnopp restick window,
deterministic anchor state updates, tangential anchor elastic-energy reporting,
incline stiction/sliding closed-form helpers, a drive-spring stick-slip
oscillator fixture matched against an independent ideal Karnopp reference, a
consecutive-hold rest detector, and Housner rocking-block threshold and period
anchors (`REQ-CONTACT-004`). Schema-v3 `[contact]` can now select
`friction_law = "anchored_stiction"` and the runner contact adapters route it
through shared accepted-substep anchor state with failed-step rollback;
contact diagnostics now carry tangential
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
models; `scenarios/parachute-recovery`). WP-16.1 is now **partial**:
`openbmp-vehicle` exposes deterministic polynomial/exponential canopy
inflation primitives and closed-form infinite-mass opening-load diagnostics,
single-stage `ParachuteDrag` and two-stage `DrogueMainRecovery` stages can opt
into fill-time area growth,
`canopy_fill_time_s` derives `t_f = n * D0 / v_s^k` from line-stretch speed
inputs, schema-v3 `[[vehicle.assembly.recovery]]` accepts `inflation = { ... }`
for `parachute_drag` and `drogue_inflation`/`main_inflation` for `drogue_main`
with explicit or derived fill time, and the runner rack starts the fill clock
on `deploy`, `deploy_drogue`, or `deploy_main` so `recovery.<id>.drag_area_m2`
ramps through the existing snapshot/telemetry surface. Inflating parachutes also
publish opt-in infinite-mass opening-load diagnostics (`inflation_progress`,
`opening_dynamic_pressure_pa`, `opening_load_n`, and `opening_peak_load_n`)
from the same pre-step dynamic pressure used by recovery drag, plus optional
Pflanz finite-mass peak-load telemetry from declared inline active-stage
`*.pflanz` tables or SHA-pinned file-backed decks
(`opening_ballistic_parameter_a`, `opening_pflanz_reduction_factor_x1`, and
`opening_finite_mass_peak_load_n`). The repository carries
`data/recovery/pflanz-x1-synthetic-v1.toml` plus `data/recovery/provenance.md`
as synthetic parser/runtime evidence.
`PflanzReductionTable` adds the validated finite-mass
reduction-table/interpolation primitive plus eq. 3.2.4/3.2.5 helpers and an
exact `X1(A -> infinity) = 1` limit check (`REQ-RECOVERY-001`).
The point-mass runner now also carries synthetic integrated descent-EOM
peak-load regressions, including an `A`-sweep tolerance fixture, that match
independently computed fixed-step `X1` reduction ratios through the deploy,
inflation, recovery snapshot, force, and telemetry path.
Optional `apparent_mass_coeff_k_a` on a single-stage or staged profile now
enables scalar T1 apparent mass: recovery snapshots publish `apparent_mass_kg` and
`apparent_mass_rate_kg_s`, point-mass and rigid mass paths add those terms,
the recovery force adapter applies `-(dm_a/dt) * v_air`, and telemetry exposes
the two apparent-mass channels when configured.
Wind-relative recovery drag is now validated at adapter level through
`EnvironmentSample::air_relative_velocity_eci_m_s`: a focused constant-wind
test proves zero vehicle velocity with no wind has zero force, while nonzero
`wind_eci_m_s` produces the closed-form recovery drag vector
(`REQ-RECOVERY-002`). A point-mass runner regression now also drives declared
`[wind]` through the real wind rack, environment sample, recovery deploy, and
recovery force stack, matching the closed-form one-dimensional constant-wind
drift position and velocity. A paired vertical descent regression reaches
`GroundImpact` and proves declared wind shifts the synthetic splash point
downwind while the no-wind twin remains horizontally fixed.
Production/digitized real `X1` table ingestion/calibration, real-table
integrated Pflanz validation, tensor/higher-order apparent-mass dynamics, and
reefing/disreef staging remain open. WP-16.2 still needs public drift-campaign
calibration/dispersed splash-point UQ and tensor/high-order apparent mass
before completion.
WP-16.3 now also has a partial slack-taut riser substrate: `openbmp-vehicle`
exposes a tension-only Kelvin-Voigt line element with exact-zero slack force,
damping-clamped non-compressive tension, elastic-energy reporting, equal and
opposite endpoint forces, and an attach-point `r x F` moment helper; schema-v3
recovery rows can declare validated `risers = [{ ... }]` for parachute recovery
kinds, including optional prescribed endpoint velocity, optional positive
per-riser endpoint mass, and an optional shared translated `riser_canopy`
state; the runner publishes aggregate riser tension, body-frame anchor force,
body-frame attach moment, and elastic-energy diagnostics through recovery
snapshots and telemetry, advances declared endpoints by prescribed velocity,
per-riser line-force integration, or summed-force shared-canopy integration
after slack-taut engagement, the force stack consumes aggregate riser force as a
`recovery_riser_force` component, live snapshots transmit each riser-declared
device's aerodynamic `C_D A` drag load through the riser force/moment totals
with tension-weighted per-riser moment distribution while `recovery_drag` skips
that same scalar term, and the rigid moment stack consumes the aggregate
force/moment diagnostics as a `recovery_riser` moment component about the live
CG. WP-16.4 is now **partial**: schema-v3 parachute recovery rows with declared
risers can opt into `sequence = { snatch_delta_v_m_s, snatch_effective_mass_kg,
pack_drag_load_n, mortar_pack_mass_kg, mortar_pack_velocity_body_m_s }`; the
runner derives equivalent riser stiffness as `sum(EA/L0)`, publishes
default-zero `RecoverySnapshot` snatch terms after deployment, and
point-mass/rigid telemetry expose `recovery.<id>.snatch_*` channels, with tests
proving the §3.3.1-style peak estimate
`delta_v * sqrt(k_line * m_e) + F_drag,pack`. When both mortar fields are
declared, the rack latches the equal-and-opposite reaction impulse
`-m_pack * v_pack_body` on accepted deploy commands, applies it as a constant
force for exactly the next kernel tick through `recovery_sequence_force`, and
publishes the body-frame impulse/force telemetry. Full multibody two-body
coupling, bag-strip/line-stretch kinematics, force-applied snatch transients,
reefing/disreef, clusters, water impact, sea state, and retrieval remain open.
WP-16.5-WP-16.11: **not started**.

### 17 — Cryogenic fluid management

**Not started.** No property layer, no tank energy balance. (The
`openbmp-feedsystem` crate now exists as the design's default home for the
cryo module.)

### 18 — Ground segment, countdown & launch release

WP-18.1 countdown substrate: **partial** —
`openbmp-ground` is now registered in the workspace as the L7 host-side ground
crate with no FC, runner, simulator, telemetry, CLI, scenario, or hardware
dependency edge (`REQ-GROUND-001`). The crate exposes a deterministic
countdown plan model with step anchors, windows, step/external prerequisites,
hold-or-abort failure actions, criterion timelines, count-clock evaluation,
ordered event emission, and byte-stable CSV rendering. Validation rejects
unsupported schemas, empty ids/plans, non-finite anchors, terminal counts
before initial counts, negative windows, duplicate/nonmonotonic steps, bad hold
ids, same-or-later step dependencies, malformed criterion timelines, and
unknown external criteria at evaluation. Tests prove deterministic event order,
hold re-entry with count-clock frozen while simulation time advances, abort
latching, fail-closed dependency/criterion handling, and canonical CSV bytes.
Schema-v3 `[countdown]` now adds a parser front door (`REQ-GROUND-002`) for
T-clock initial/terminal anchors, optional countdown id, `max_hold_s`, ordered
steps, prerequisites, hold-or-abort failure actions, and emitted event ids,
with fail-closed tests for v2 reservation, nonmonotonic anchors, same-or-later
step dependencies, invalid T-clock strings, and unsupported failure actions.
`openbmp-runner` now materializes that block into an `openbmp-ground`
`CountdownPlan` and emits a `RunOutcome.countdown` report plus byte-stable CSV
(`REQ-GROUND-003`), with a runner regression proving ordered step events and
no report for the same v3 scenario without `[countdown]`. The point-mass and
rigid telemetry recorders now allocate gated `ground.countdown.*` summary
channels only for `[countdown]` scenarios and populate verdict, verdict code,
final sim/count time, event counts, and binding step from the precomputed
report (`REQ-GROUND-004`). Missing for full WP-18.1 closure: countdown event
injection into mission/ground models, per-event telemetry/timelines, and
broader canonical byte-equivalence gates. WP-18.2-WP-18.8: **not started**.

### 19 — Day-of-launch winds & commit operations

WP-19.1 measured-wind front door: **implemented** —
`openbmp-physics::wind::measured` now exposes measured profile construction,
fail-closed QC screens, deterministic two-source splicing with byte-stable
per-sample attribution CSV, Martin-Graham-style first-order response/filtering,
and a seeded `DeterministicRng` synthetic balloon-family generator. Scenario
v3 now accepts `[day_of_launch.wind]` declarations for measured, spliced, and
synthetic-family inputs, rejects the block under v2, validates profile rows,
QC/filter thresholds, splice references, and synthetic-family grids, and
`data/wind/synthetic-balloon-family-v1.toml` carries the pinned synthetic
deck/provenance (`REQ-DOL-001`). This is the T1 ingestion substrate only.

WP-19.2 DOL I-load redesign: **implemented** —
`openbmp-trajopt::dol` consumes the spliced wind profile and closed
`TerminalCondition` vocabulary, validates finite/increasing load-window inputs
and insertion residual bounds, recenters pitch/yaw bias gain tables to the
load-window mean induced alpha/beta, and emits a deterministic postcard
`ILoadPayload` plus a JSON-serializable DOL update artifact with stale versus
redesigned load margins, not-worse fraction, profile digest, solver provenance,
terminal kind, and residual limits. `openbmp trajopt dol-redesign` materializes
the scenario v3 `[day_of_launch.wind] kind = "spliced"` declaration, writes the
I-load, optionally writes the report JSON, and accepts signed angular
references/biases. The trybuild guard
`dol_no_terminal_extension_compile_fail` freezes the boundary by proving the
DOL surface cannot add `TerminalCondition::DayOfLaunchLoadRatio`
(`REQ-DOL-002`). This is still only the T2 3-DOF redesign substrate:
automatic runner wind dispatch, 6-DOF verification/report-card generation,
persistence knockdown application, and launch COLA remain outside the redesign
command.

WP-19.3 load-indicator surrogate: **implemented** —
`openbmp-trajopt::dol` now exposes q-alpha/q-beta/gimbal indicator samples,
deterministic least-squares station-moment surrogate fitting, held-out
validation reports, and a deck-hash drift gate. The fit path rejects malformed
or underdetermined matrices, non-finite samples, negative dynamic pressure,
deck-hash drift, singular normal equations, held-out correlation below the
declared threshold, and unconservative absolute-moment error above the
declared bound. `data/wind/dol-load-indicator-surrogate-v1.toml` plus
`data/wind/provenance.md` pin the synthetic feature vocabulary, upstream hash,
and validation thresholds (`REQ-DOL-003`). This is a degraded synthetic
q-alpha/q-beta/gimbal screening substrate only: certified station-load fits,
runner launch-commit screening, operational 6-DOF report-card integration,
site-fitted persistence knockdowns, and operational launch COLA remain open.

WP-19.4 go/no-go report card: **partial** —
`openbmp-trajopt::dol` now exposes a deterministic report-card substrate with
typed verification indicators, `DolReportVerdict` monotone latching,
`build_dol_report_card`, and `dol_report_card_csv`. The builder validates
finite artifact fields and thresholds, rejects empty indicator sets, latches
`no_go` on margin, not-worse-fraction, residual, or design-vs-verify
indicator breaches, records the first binding check, and emits a byte-stable
ordered CSV report-card artifact checked against
`crates/openbmp-trajopt/tests/expected/dol-report-card-v1.csv`
(`REQ-DOL-004`). `openbmp trajopt dol-materialize-indicators` now provides a
runner/SIL telemetry extraction seam: a spec CSV maps named DOL indicators to
telemetry columns, deterministic reducers, allowed error bands, and optional
time windows, then writes the indicator CSV consumed by
`openbmp trajopt dol-report-card`. The report-card command reads a
`dol-redesign` JSON update artifact plus that runner/SIL-style verification
indicator CSV, writes deterministic CSV and optional JSON report-card
artifacts, and exits fail-closed on `no_go` verdicts.
The DOL update artifact and report card carry `iload_sha256_hex` for the
encoded updated I-load bytes. `openbmp trajopt dol-verify-run` now requires both
the verifier design-profile digest and the updated I-load SHA-256 digest to
match the DOL update artifact before invoking the runner, decodes and validates
the authenticated updated I-load bytes, applies `dol_pitch_bias_rad` and
`dol_yaw_bias_rad` time-axis gain tables to `pitch_program` FC
ascent-reference schedules when the verifier scenario declares
`guidance = "ascent_reference"`, revalidates the in-memory scenario, runs that
verification scenario through the same telemetry override path as
`openbmp run`, materializes the declared indicators from the produced telemetry,
writes the indicator CSV, builds the report card, and exits fail-closed on
digest mismatch, malformed I-load application, or `no_go`. `openbmp trajopt
dol-verify-sil` runs the same authenticated I-load checks through the native
SIL package/testbench seam: it derives adjusted pitch/yaw schedules, injects
them as `SilStimulation` `ParameterOverride` records, writes SIL telemetry and
an evidence manifest carrying those stimuli, materializes indicators, and
builds the report card fail-closed. The pitch-program ascent-reference surface
now carries an optional `yaw_rad` schedule so updated yaw-bias tables are
applied rather than ignored. `build_dol_commit_readiness` and `openbmp trajopt
dol-commit-readiness` now consume the report card, persistence knockdown
report, and COLA report into a byte-stable DOL-level readiness latch and
file-fed CLI handoff (`REQ-DOL-007`). Missing for full closure: operational
DOLILU/LCC sequencing.

WP-19.5 persistence knockdowns: **partial** —
`openbmp-trajopt::dol` now exposes a deterministic synthetic persistence
substrate with `DolPersistenceDeck`, altitude-banded square-root lead-time
`DolPersistenceBand` sigma values, `DolPersistenceIndicatorInput`,
`evaluate_persistence_knockdowns`, and
`validate_persistence_truth_samples` (`REQ-DOL-005`). The evaluator validates
finite non-overlapping bands, finite non-negative lead times, named indicator
inputs, deck-covered indicator altitudes, non-negative gust/system sigmas, and
finite wind sensitivities; it computes persistence, gust, and system margin
knockdowns, combines them by RSS, subtracts the total from nominal margins,
tracks the minimum knocked-down margin, and latches `no_go` on the first
indicator below the declared threshold. The checked-in
`data/wind/dol-persistence-knockdown-v1.toml` fixture pins the synthetic deck
and truth samples used by the validation test. The readiness latch consumes the
persistence report as one required gate (`REQ-DOL-007`). Missing for full
closure: site-fitted persistence statistics, Wilks-sized range-data closure,
and operational LCC integration.

WP-19.6 launch COLA: **partial** —
`openbmp-trajopt::dol` now exposes a deterministic synthetic geometric
launch-COLA substrate with `DolColaTrajectorySample`, `DolColaWindowSlot`,
`DolColaCatalogObject`, `DolColaScreeningInput`, and `screen_launch_cola`
(`REQ-DOL-006`). The screener validates named finite launch-window slots with
strictly increasing piecewise-linear trajectory samples, named finite
constant-velocity ECI catalog states, non-negative object radii, and a
non-negative vehicle keepout radius; it propagates catalog objects, evaluates
closest approach per segment, emits deterministic per-slot/object miss
distance and clearance rows, counts blocked slots, and latches `no_go` on the
first blocked slot/object. The checked-in
`data/cola/launch-cola-screening-v1.toml` fixture pins one clear and one
blocked synthetic slot. The readiness latch consumes the COLA report as one
required gate (`REQ-DOL-007`). Missing for full closure: SGP4/covariance
realism, probability-of-collision integration, operational catalog handling,
CLI, and operational LCC integration.

DOL commit-readiness latch: **partial** —
`openbmp-trajopt::dol` now exposes `DOL_COMMIT_READINESS_SCHEMA_VERSION`,
`DolCommitReadinessInput`, `DolCommitGateReport`,
`DolCommitReadinessReport`, `build_dol_commit_readiness`, and
`dol_commit_readiness_csv` (`REQ-DOL-007`). The builder requires the DOL
report card, persistence knockdown report, and launch-COLA screening report
together; recomputes each gate verdict from stored row evidence; rejects
malformed schema versions, inconsistent pass/blocked flags, inconsistent
minimum margins, inconsistent blocked-slot counts, and stale verdict/binding
fields; emits ordered `report_card`, `persistence`, and `cola` gate rows; and
latches `no_go` monotonically in that order with the first binding gate/detail
preserved. `crates/openbmp-trajopt/tests/expected/dol-commit-readiness-v1.csv`
locks the canonical readiness CSV bytes. `openbmp trajopt
dol-commit-readiness` reads the three JSON reports, writes deterministic CSV
plus optional JSON readiness artifacts, prints the latched gate summary, and
exits fail-closed after evidence is written when the combined verdict is
`no_go`. This is not an operational LCC sequencer handoff.

### 20 — Telemetry, RF links & ground network

WP-20.1 comm geometry: **implemented** — `openbmp-comm` now exposes WGS84
ground-site geometry, ENU visibility samples, azimuth-binned terrain masks,
rise/set event extraction, deterministic pass intervals, and byte-stable CSV
pass-table serialization; schema-v3 `[comm]` / `[[comm.sites]]` parser
validation accepts site geometry and mask bins, and `openbmp-runner` emits
deterministic pass-table bytes through `RunOutcome.comm` plus `comm.<site>.*`
telemetry channels for sampled point-mass and rigid-body primary states.
`data/comm/` now records a provenance-backed fixed-Earth line-scan tolerance
table for geometric visibility and rise/set extraction plus an SGP4/TLE
external-library LEO visibility tolerance table for rise/set extraction.
WP-20.2 antenna/body masking: **implemented** — `openbmp-comm` now exposes
axisymmetric antenna gain decks, triangle-ray-cast body-mask deck precompute,
nearest-direction runtime lookup, and deterministic mesh-hash-bound deck
hashing, backed by a provenance-recorded synthetic fixture under `data/comm/`.
`openbmp-aero` now provides watertight two-manifold validation and deterministic
`PanelMeshBvh` ray queries. Schema-v3 `[comm]` now binds
`[[comm.antennas]]` gain/body-mask deck paths with SHA-256 pins through the
standard resolved-file mechanism. Runner point-mass and rigid-body setup now
parses declared decks from resolved bytes, validates them through
`openbmp-comm`, precomputes body masks from either comm-native triangle tables
or an `openbmp.panel_mesh_aero = 1` watertight panel-mesh deck source, rejects
open/non-manifold panel meshes fail-closed, and reports resolved/derived deck
hashes plus sample counts in `RunOutcome.comm`. Runner link budgets now rotate
vehicle-to-site line of sight into the body frame, add sampled antenna gain to
EIRP, and gate link visibility when the body-mask lookup is blocked.
WP-20.8 frame/dictionary export: **implemented** — a channel-dictionary
export with JSON and XTCE-shaped formats already ships through `openbmp dict`,
and `openbmp-telemetry` now adds the WP-owned off-by-default
`interop-export` feature: deterministic CCSDS-shaped, explicitly
non-conformant binary frame streams from `TelemetryTable`, deterministic
channel-dictionary JSON and XTCE-shaped XML from `TelemetrySchema`, a
lossless frame-stream importer, byte-stability tests, non-conformance label
checks, and a round-trip regression that compares decoded Parquet bytes
against the native table's Parquet bytes.
WP-20.3 link budget/FER: **implemented** — `openbmp-comm` now exposes
`LinkBudgetInput`, `LinkState`, `free_space_path_loss_db`,
`evaluate_link_budget`, `FerCurveDeck`, `FerCurveSample`, `ElevationLossDeck`,
and `ElevationLossSample` for deterministic free-space budget arithmetic, dB
operand ordering, log-linear FER interpolation, elevation-indexed additive
atmospheric/rain losses, no-link FER behavior, and fail-closed out-of-envelope
rejection, backed by `data/comm/link-budget-fer-v1.toml` and
`data/comm/attenuation-loss-v1.toml` provenance. Schema-v3 `[comm]` now binds
`[[comm.links]]` budget constants, FER curve decks, atmospheric loss decks,
and rain loss decks with SHA-256 pins, and the point-mass/rigid-body runner
paths publish `comm.link.<link>.*` `LinkState` telemetry channels, including
`antenna_gain_dbi` and `body_blocked` evidence from the linked antenna decks.
A no-`[comm]` point-mass guard proves the off-default path emits no comm
report or comm channels and keeps the canonical telemetry hash stable across
repeated runs. Declared electron-density blackout profiles are covered by
WP-20.5.
WP-20.4 link-driven telemetry effects: **implemented** — `openbmp-comm` now
exposes `LinkChannelModel`, `LinkPacketContext`, `LinkPacketEffect`,
`LinkPacketDisposition`, `LinkErrorAction`, `LinkPacketDirection`, and
`link_stream_id_from_id` for deterministic packet-effect evaluation from
`LinkState`. The model validates context/state, computes one-way light-time
plus declared processing delay, maps no-visibility/blackout to packet drop,
and maps FER Bernoulli draws from a domain-keyed deterministic RNG to drop or
bit-flip dispositions, backed by `data/comm/link-channel-model-v1.toml`
provenance. It also exposes an exact-binomial loss-rate gate over deterministic
packet streams, with no-link/blackout treated as expected 100% errored packets.
Schema-v3 `[[comm.links]]` now accepts `packet_processing_delay_s` and
`packet_error_action` plus optional `packet_loss_rate_gate` and
`data_loss_timeout_s`, validates them fail-closed, and the runner reports the
validated model, downlink/uplink loss-rate gate evidence, and monotone downlink
data-loss timeout latch evidence with a stable monitor-side `hold`/`terminate`
verdict in `RunOutcome.comm`. The runner also
evaluates the selected comm link for FC bridge sensor and command frames,
supports default first-declared selection plus opt-in greedy `best_margin`
selection across visible non-blackout declared links with configurable margin
hysteresis plus a declared pass-plan selector for bridge handover windows,
queues delivered frames by step-quantized downlink/uplink latency, preserves
zero-step delivered comm-link determinism, and fails closed on drop, bit-flip,
blackout, or scheduled pass-plan gap packet effects at the `[fc.transport]`
seam. The zero-effect comm/SIL guard proves an installed read-only
`SilMonitor` preserves the canonical telemetry hash and actuator stream
evidence under a delivered zero-step link. Declared electron-density blackout
profiles are covered by WP-20.5.
WP-20.6 station network handover: **implemented** — schema-v3 `[comm]` accepts
`bridge_link_selection = "declared_plan"` with non-overlapping
`[[comm.bridge_pass_plan]]` windows keyed to declared link ids, validates
references and times fail-closed, and the runner selects the active declared
link by latest observation time while dropping bridge packets fail-closed during
schedule gaps. `RunOutcome.comm.bridge_selections` records contiguous
link/gap-selection intervals with step/time spans, sample counts, and
sensor/command drop or bit-flip counts, and `RunOutcome.comm.bridge_handovers`
records explicit link/gap transition events. `RunOutcome.comm.link_passes`
records each sampled visible/non-blackout link pass with sampled step/time
span, sampled and interpolated duration, sample count, min/max/mean margin,
sampled margin profile, interpolated start/end boundary times, interpolated
boundary-plus-sample margin profile, and bridge packet drop/bit-flip counts
attributed to the pass. Declared `[[comm.relays]]` GEO nodes accept optional
non-overlapping `availability_windows` that gate relay-to-ground visibility by
time and are reported in `RunOutcome.comm.links[*].relay`; per-link `relay_id`
composes vehicle-to-relay and relay-to-ground budgets into a two-hop
`LinkState` using bottleneck margin and combined independent-hop FER. This
closes WP-20.6 at the declared scheduling/no DSN-negotiation ceiling.
WP-20.5 entry plasma blackout: **partial** — `openbmp-comm` exposes
`PlasmaBlackoutModel`, `PlasmaBlackoutInput`, `PlasmaBlackoutState`, and
`plasma_frequency_hz`; schema-v3 `[[comm.links]]` accepts
`blackout_electron_density_source = "deck" | "live_aerothermal"`, optional
deck/SHA pins, and attenuation ramp parameters; and the runner loads
`[plasma_blackout]` electron-density profiles or a declared
`[aerothermal.electron_density_proxy]` live sample, applies
attenuation/blackout to the vehicle-originating hop, reports active source
evidence, and derives `RunOutcome.comm.blackout_windows` artifacts. Missing:
true `04` equilibrium-air edge-state electron density and RAM-C II validation.
WP-20.7 tracking observables: **implemented** — `openbmp-comm` exposes
`TrackingInstrumentModel`, `TrackingObservableInput`, and
`TrackingObservable`; schema-v3 `[comm.tracking]` opts into deterministic
latency, bias, quantization, and optional pinned white-noise/Gauss-Markov
instrument decks; and the runner emits `RunOutcome.comm.tracking` deck evidence
plus visible-site vehicle-bound `RunOutcome.comm.tracking_observables` records
with truth time, measurement time, site id, range, range-rate, azimuth,
elevation, applied white-noise values, and applied Gauss-Markov bias states.
It also exports a byte-stable `RunOutcome.comm.tracking_observables_csv`
measurement stream, and a trybuild tripwire proves downstream crates cannot
literal-construct `TrackingObservableInput` outside the vehicle-bound
visibility-sample path. A synthetic `openbmp-testkit` batch Gauss-Newton
regression recovers injected tracking biases and verifies the CRLB covariance
against the inverse normal matrix.

### 21 — Run-data, regression & visualization

WP-21.1 run records and campaign index: **implemented** —
`openbmp-rundata` now exists as the L7 run-data crate with deterministic
`RunRecord`, `ArtifactRef`, `GateResult`, `RunLabel`, `ChannelDigest`,
`NumericChannelDigest`, `channel_digests()`, `RunIndex`,
`append_record_jsonl()`, and deterministic Parquet index mirrors. `openbmp run`
exposes off-by-default `--run-index-jsonl`, `--run-index-parquet`, and
`--run-code-id` emission flags that build records from runner telemetry and
written telemetry artifacts. Tests prove identical records produce identical
JSONL bytes and record hashes, channel digests match direct row-order
statistics, JSONL appends preserve the existing prefix, unterminated existing
indexes fail closed, duplicate run ids are rejected, malformed SHA-256 pins
are rejected, Parquet mirrors are byte-stable, and a real CLI run emits
parseable JSONL plus a matching Parquet mirror.

WP-21.2 verdict rule engine: **implemented** — `openbmp-rundata::verdict` now
exposes closed TOML rule documents (`VerdictRuleDocument`) and typed
limit, tolerance-vs-golden, monotonicity, event-window, statistical-band, and
cross-run-delta rule forms. It emits stable `VerdictReport` / `VerdictRecord`
JSON, evaluates over telemetry tables plus optional baseline records, rejects
malformed/duplicate/unknown-field rule documents, and fails closed for unknown
channels, non-numeric metrics, absent events, and missing baselines. `openbmp
run` exposes off-by-default `--run-verdict-rules`, `--run-verdict-json`, and
optional `--run-verdict-baseline-jsonl` flags that emit deterministic verdict
report JSON after a real run; missing rule/output flag pairs fail closed before
scenario loading. Tests, a checked TOML fixture, and CLI end-to-end coverage
cover the current path, including generated property checks for inclusive
bounds, monotonic samples, and unknown-channel fail-closed behavior plus a
committed report golden. `openbmp-sil` remains tap-specific; the shared
contract is the stable pass/fail/skip verdict vocabulary rather than a new
dependency edge.
WP-21.3 baseline ensemble bands and out-of-family detection: **implemented** —
`openbmp-rundata::bands` now exposes `BaselineBandSpec`,
`BaselineBandArtifact`, `BaselineBand`, `OutOfFamilyFinding`,
`generate_baseline_band_artifact()`, and `evaluate_out_of_family()`, plus
JSONL-index generation and a doc-18-style launch-commit out-of-family
predicate that consumes the same artifact schema without translation. It
generates deterministic median/MAD and percentile-envelope JSON artifacts from
`RunRecord` families, rejects empty inputs, bad percentiles, and missing or
non-numeric metrics fail-closed, serializes committed crate and `data/rundata/`
golden artifacts, evaluates candidate records against the shared schema, and
checks synthetic false-alarm/detection rates against a TOML tolerance fixture.
WP-21.4 trend store, drift gates, and scoreboard: **implemented** —
`openbmp-rundata::trend` now exposes deterministic `TrendMetricSpec`,
`TrendStore`, `TrendSeries`, `TrendPoint`, `DriftDetectorConfig`,
`DriftReport`, `generate_trend_store()`, `evaluate_trend_store()`,
`generate_scoreboard_markdown()`, and `ci_gate_failures()`. It aggregates
matching `RunRecord` metrics by family and `code_ordinal`, sorts source run
ids, validates duplicate specs, missing ordinals, conflicting code ids, and
missing/non-numeric metrics fail-closed, runs fixed-threshold Page-Hinkley
drift checks, serializes stable JSON, emits a committed Markdown scoreboard
golden, and has a CI-visible constructed drift fixture proving the
fail-on-drift helper trips only for opted-in metrics.
WP-21.5 deterministic plots and report booklets: **implemented** —
`openbmp-rundata::render` now exposes fixed-layout `SvgPlotSpec`,
`TimeSeries`, `PercentileFanPoint`, `RunDiffReport`, `SpikeFinding`,
`ReportBooklet`, `extract_numeric_time_series()`, `render_time_series_svg()`,
`percentile_fan()`, `render_percentile_fan_svg()`, `run_diff_report()`,
`render_run_diff_svg()`, and `render_report_booklet()`. It renders static SVG
time-series, percentile-fan, and run-diff residual artifacts with stable
coordinates/colors/fonts, computes top-k spike windows by descending
`abs(rhs-lhs)` with deterministic time/residual tie-breaking, serializes
run-diff and booklet schemas as stable JSON, emits a deterministic Markdown
booklet, and compares committed SVG/Markdown goldens in tests. Bad plot
dimensions, unknown channels, missing samples, non-numeric samples, mismatched
time grids, empty series, and invalid `top_k` fail closed.
WP-21.6 CZML and glTF export: **implemented** — `openbmp-rundata::export3d` now
exposes `SceneExport`, `BodyTrack`, `KinematicSample`, `EventInterval`,
`TelemetryBodyTrackSpec`, `GroundSite`, `generate_czml()`,
`generate_gltf_json()`, `scene_export_from_telemetry_table()`,
`validate_czml_export()`, and `validate_gltf_export()`. It emits deterministic
CZML-shaped trajectory/attitude/event/site JSON and glTF 2.0 JSON with embedded
base64 animation buffers, bufferViews, accessors, animation samplers/channels,
and OpenBMP per-body track extras, sorts tracks/events/sites deterministically,
validates empty tracks, empty samples, non-finite values, duplicate/decreasing
sample times, invalid intervals, unknown telemetry channels, missing samples,
non-numeric samples, and malformed generated documents fail-closed, checks
committed JSON goldens, and has runner-backed multi-body coverage that exports
`scenarios/multi-body/initial-formation.toml` telemetry into primary-plus-
observer CZML/glTF tracks. The verification command also runs
`scripts/check_export3d_schemas.py`, which validates the committed goldens
against pinned upstream Khronos glTF 2.0 and CZML writer JSON schema trees.
WP-21.7 Open MCT shim: **implemented** — `openbmp-rundata::openmct` now exposes
`OpenMctCatalog`, `OpenMctCatalogObject`, `OpenMctReplayFrame`,
`generate_openmct_catalog()`, `generate_openmct_replay_frames()`, and
`openmct_replay_frames_to_json_bytes()`. It emits deterministic static catalog
JSON for numeric telemetry channels, deterministic replay-frame JSON for
declared numeric channels at a finite positive replay speed, round-trips the
catalog schema, compares committed JSON goldens, and rejects empty channels,
bad replay speed, unknown channels, and non-numeric samples fail-closed. The
feature-gated `openmct-host` module and `openmct_replay_server` example binary
serve generated catalog JSON at `/catalog.json` and replay frames over
WebSocket `/replay` without affecting default builds. The documented example
command binds loopback, points Open MCT at the catalog URL and replay WebSocket,
and exits after a declared bounded connection count for smoke use. Tests compare
committed catalog/replay JSON goldens, check the RFC WebSocket accept key,
serve catalog HTTP bytes, run a loopback WebSocket replay smoke, and
compile-check the example behind `--features openmct-host`. This closes
WP-21.7 as an example shim, not a supported OpenBMP service.
WP-21.8 campaign manifests: **implemented** — `openbmp-rundata::campaign` now
exposes `CampaignManifest`, `CampaignScenario`, `CampaignPlan`,
`CampaignRunSpec`, `CampaignShard`, `CampaignVerdictSummary`,
`expand_campaign_plan()`, `partition_campaign_runs()`, and
`aggregate_campaign_verdicts()`. It parses closed TOML manifests, validates
optional `CampaignOverlayFile` rows including default append mode and explicit
merge mode, expands
scenario × seed × overlay matrices into deterministic run specs and labels,
round-trips a committed plan JSON golden, proves flattened run sets are stable
across 1/4/16 worker shards, and fails the campaign verdict when any run
verdict report fails. `openbmp campaign plan` now writes deterministic plan JSON
and optional worker-shard JSON from those manifests, with binary smoke coverage
and locked help text. `openbmp campaign run` now executes scenario x seed runs
through a bounded worker pool with optional overlay TOML files declared by
manifest-relative `[[overlay_files]]`; default `mode = "append"` preserves
fragment concatenation, while `mode = "merge"` parses base and overlay TOML
with `toml_edit`, recursively merges tables, appends matching arrays of tables,
and rejects malformed merge overlays fail-closed. Overlay runs materialize the
combined scenario under `runs/<run_key>/scenario.toml` and use that digest in
the run record. The executor applies manifest seeds to parsed scenarios, writes per-run
telemetry under `runs/<run_key>/`, writes one local run index per run directory,
merges records into the shared `run-index.jsonl` in expanded-plan order,
optionally writes `run-index.parquet`, writes per-run `verdict.json` files when
rules are declared, writes `summary.json`, aggregates verdicts, and rejects
selected overlays without files fail-closed. Manifest-relative `trend_specs`
documents now feed `[[metrics]]` rows into the existing trend-store pipeline
after the run index is complete, producing `trend-store.json`,
`drift-report.json`, and `trend-scoreboard.md` plus stable `ci_failures` in the
campaign summary. Combined verdict/trend `ci_failures` are now serialized at
the campaign-summary level, and `openbmp campaign run --fail-on-ci-gates`
returns non-zero after writing artifacts when those gates are nonempty; CI keeps
that hook visible with a failing-verdict smoke. A worker-count regression proves
the campaign plan, aggregate JSONL/Parquet indexes, summary, trend artifacts,
per-run verdicts, local run indexes, and telemetry artifacts are byte-identical
across 1/4/16 workers. This closes the WP-21.8 acceptance scope without
claiming keyed or semantic patch operations beyond recursive table merge and
matching array-of-tables append.

### 22 — Acoustics, vibroacoustics & overpressure

WP-22.1 acoustics skeleton and spectral machinery: **implemented** —
`openbmp-acoustics` now exists as the L2 acoustics crate with no runner,
simulator, or flight-controller dependency edge. It exposes `AcousticBand`,
`Spl`, `Psd`, `SplSpectrum`, `PsdSpectrum`, the standard one-third-octave
band grid, deterministic band-grid CSV bytes, OASPL/mean-square-pressure
reductions, SPL energy addition, per-band transfer application, spectral
enveloping, and flat-in-band PSD/SPL conversion. Tests prove the band grid is
byte-stable, OASPL matches locked energy summation, transfer/envelope ordering
is deterministic, PSD↔SPL conversion closes pressure power on synthetic
spectra, bad inputs fail closed, and property tests preserve band power and
energy addition within the WP tolerance.

WP-22.2 DSM liftoff acoustics: **partial** —
`openbmp-acoustics::dsm` now contains a deterministic free-field,
single-engine DSM substrate with acoustic power from exit conditions,
normalized source-line allocation, Strouhal-shaped spectral fractions,
angle-interpolated directivity decks, spherical spreading, scalar or
per-band-deck air absorption, station spectrum synthesis, height-stepped
liftoff OASPL traces, committed byte-stable synthetic station/trace CSV fixtures, a
Schema-1 provenance-pinned DSM source-allocation/directivity deck parser, and a
Schema-1 provenance-pinned synthetic air-absorption deck parser with data
fixtures, including a named atmosphere-profile fixture selectable through
`absorption_profile_id`. The acoustics data directory also includes a
Schema-1 public-source manifest for NASA SP-8072 / NTRS `19710023719`, pinning
the retrieved PDF SHA-256 and DSM-relevant figure inventory for future
digitized decks, and implements the SP-8072 Figure 11 core-length fit
(`x_t / d_e`) as a validated helper. Coarse, provenance-pinned SP-8072
Figure 5 spectrum, Figure 10 directivity-index, Figure 12 axial source-power,
and Figure 13 axial spectrum curve decks now parse and derive runtime DSM
spectrum/directivity/source/source-by-band weighting inputs; Figure 5 spectrum
uses Strouhal interpolation and band-width normalization, Figure 12 source
allocations use trapezoidal tributary widths, Figure 13 axial spectrum uses
modified-axial-Strouhal interpolation over source locations, and Figure 10
directivity uses Strouhal/angle interpolation and declared exit
diameter/velocity. A coarse Figure 14 apparent-source curve deck now parses and
can drive one apparent source location per band for the earlier SP-8072
source-location approach. All five shipped SP-8072 curve decks validate their
manifest id, source PDF hash, and figure id against the pinned public source
manifest, and runner curve loaders reject manifest mismatches after scenario
SHA-pin verification before treating them as public-source-bound evidence.
The DSM module also includes a coarse SP-8072 curve tolerance-table substrate
that compares evaluated DSM spectra against reference spectra, records OASPL
and per-band residual pass/fail rows, and emits deterministic CSV evidence.
Schema-v3 `[acoustics.dsm]` now validates declared
DSM inputs, resolves optional SHA-pinned `deck_file`, `source_curve_file`,
`axial_spectrum_curve_file`, `apparent_source_curve_file`,
`directivity_curve_file`, `spectrum_curve_file`, and `absorption_deck_file`
inputs, preserves optional `absorption_profile_id` selection for named
atmospheric absorption profiles, derives source-curve core length from
`exit_mach`, and the point-mass and rigid runner tails emit an off-default
`RunOutcome.acoustics` report with station spectrum and OASPL-vs-height CSV
bytes plus gated `acoustics.dsm.*` summary telemetry. Tests cover
acoustic-power and propagation closed forms,
normalization, directivity
interpolation, synthetic station byte stability, height-decay trace byte
stability, parser fail-closed checks, SP-8072 core-length formula values,
coarse Figure 5 spectrum parsing/interpolation/shape derivation, coarse Figure
10 directivity parsing/interpolation/derivation, coarse Figure 12 curve
parsing/interpolation/allocation, coarse Figure 13 axial-spectrum
parsing/interpolation/runtime wiring, coarse Figure 14 apparent-source
parsing/interpolation/runtime wiring, SP-8072 curve-deck source-manifest
binding validation, runner SP-8072 curve-loader manifest-mismatch rejection,
coarse SP-8072 curve tolerance-table residual decisions,
absorption-deck parsing/runtime wiring, deck parser
fail-closed checks, SP-8072 source-manifest fail-closed checks,
deck-file/source-curve/axial-spectrum-curve/apparent-source/directivity-curve/
spectrum-curve/absorption-deck resolution and bad-pin checks, runner
fixture-byte matching, deck-backed, curve-backed, axial-spectrum,
apparent-source, and absorption-deck runner
provenance reporting, named atmospheric absorption profile selection, summary
telemetry values, and no-report/no-channel
behavior when `[acoustics]` is absent. Missing for full closure:
production-quality SP-8072 allocation/directivity/spectrum/axial-spectrum/apparent-source
curve decks under runtime formats, production/public atmospheric-attenuation
validation, plus public worked-example reference-value reproduction.

WP-22.3 pad geometry effects, clusters, water suppression: **partial** —
`openbmp-acoustics::pad` now contains a Schema-1 synthetic pad suppression deck
parser with deterministic SHA-256 reporting, provenance metadata checks,
one-third-octave band validation, unique suppression-class validation, and
fail-closed malformed-deck checks. `data/acoustics/pad-suppression-v1.toml`
ships dry and `water_nominal` synthetic knockdown classes with sibling
provenance. Schema-v3 `[acoustics.dsm]` now resolves optional
`suppression_deck_file`, `suppression_deck_file_sha256`, and
`suppression_class`, rejects orphan SHA/class declarations, requires a class
when a deck is present, and fails closed on bad pins. The shared runner
acoustics evaluator applies the selected suppression class as a per-band
negative transfer on the DSM directivity deck, verifies the suppression deck
band grid, and reports the deck SHA-256, dataset id, source class, selected
class id, and class uncertainty. Schema-v3 `[acoustics.dsm]` also accepts
`cluster_engine_count` for exact incoherent identical-engine scaling from the
declared per-engine exit condition; the runner reports `cluster_engine_count`
and `cluster_power_gain_db` and proves the station OASPL increases by
10 log10(N). Explicit `[[acoustics.dsm.cluster_engines]]` offsets plus
`receiver_azimuth_deg` now drive geometric incoherent summation: each engine is
evaluated at its lateral receiver range/directivity and spectra are
pressure-energy-added. Tests cover parser pass/fail cases, scenario
resolution/bad-pin checks, point-mass provenance reporting plus suppressed
OASPL below the unsuppressed synthetic DSM fixture, exact four-engine OASPL
scaling, and a two-engine offset cluster matched against two independent
single-engine effective-radius runs. `pad_path_length_extra_m` now adds a
declared duct/trench/deflector propagation-distance term before spreading and
absorption; a one-source zero-absorption regression proves the exact
`20 log10((d + extra) / d)` station-level delta. `pad_deflection_origin_axial_m`
plus `pad_deflection_angle_deg` now map distributed source coordinates through
a declared single-bend plume axis before range/directivity evaluation; a
one-source zero-absorption regression proves the resulting range change
against a closed-form spreading oracle. `cluster_correlation_coefficient` and
`cluster_shielding_loss_db` now add a declared scalar cross-pressure coherence
term and broadband loss after cluster summation; a same-location two-engine
regression proves full coherence plus shielding against a closed-form OASPL
delta. `openbmp-acoustics::pad` also contains a Schema-1 synthetic
pad-cluster-interaction deck parser with deterministic SHA-256 reporting,
provenance metadata checks, one-third-octave band validation, unique class
validation, per-band correlation vectors in [0, 1], non-negative per-band
shielding-loss vectors, and fail-closed malformed-deck checks.
`data/acoustics/pad-cluster-interaction-v1.toml` ships `incoherent_open` and
`coherent_shielded` synthetic classes with sibling provenance. Schema-v3
`[acoustics.dsm]` resolves optional `cluster_interaction_deck_file`,
`cluster_interaction_deck_file_sha256`, and `cluster_interaction_class`,
requires a class when a deck is present, requires a cluster declaration,
rejects scalar interaction conflicts, and fails closed on bad pins. The shared
runner applies the selected per-band correlation/shielding class to identical
or geometric clusters, verifies the deck band grid, and reports deck SHA-256,
dataset id, source class, selected class id, and class uncertainty; a
same-location two-engine point-mass regression proves the deck-backed
`coherent_shielded` class against the same closed-form OASPL delta and checks
runner provenance. Missing for full closure: calibrated duct/trench path
factors, production pad-deflector geometry, calibrated plume interaction,
cluster-specific directivity, published suppression-curve digitization, and a
liftoff signature residual table.

WP-22.4 ignition overpressure model: **partial** —
`openbmp-acoustics::iop` now contains deterministic core primitives for
start-transient source-rate scaling, duct/trench residence-time coupling,
pressure-amplitude suppression knockdown, inverse-square receiver decay, and
declared lower/upper dB uncertainty bounds. `evaluate_iop_pulse()` returns a
nominal peak overpressure, lower/upper peak bounds, and ramp-plus-residence
duration. Schema-v3 `[acoustics.iop]` now validates station id, explicit source
rate/ramp declarations or a mutually exclusive liquid-engine `source_engine_id`
that derives the source rate from effective max thrust divided by
`ignition_transient_s`, or from thermochemistry-derived nominal `mdot*c*`
divided by `ignition_transient_s` when the source engine declares
`thermochemical_performance`, inline duct/trench volume, exit area, discharge
velocity, geometry gain, receiver distance, suppression loss, and uncertainty
fields.
It can also replace the inline duct terms with a SHA-pinned synthetic
`duct_deck_file` plus `duct_class`; the runner resolves and verifies the deck,
uses the selected open-pad or long-trench class, and reports deck provenance.
It can add a SHA-pinned synthetic `suppression_deck_file` plus
`suppression_class` to the scalar pressure knockdown; the runner resolves and
verifies the deck, applies the selected dry or nominal-water class additively,
and reports suppression deck provenance.
The shared point-mass/rigid-body acoustics evaluator maps the declaration into
`RunOutcome.acoustics.iop` and allocates gated telemetry channels for nominal
peak, lower peak, upper peak, and duration; when source_engine_id is used it
also reports the first declared scenario-script at-time ignite command for
that source engine and the first runtime-observed point-mass or rigid-body
source-engine ignite firing, when present, plus a deterministic runtime
source-history CSV, peak finite-difference source rate reconstructed from
engine snapshots, and a runtime-reconstructed IOP peak/lower/upper/duration
envelope from the observed positive source-rate peak.
Tests cover closed-form peak and duration math, monotone source-rate scaling,
inverse-square distance scaling, suppression ordering, uncertainty bounds,
fail-closed input validation, schema-v3 acceptance/rejection,
duct/suppression deck parser and provenance behavior, engine source
derivation, thermochemistry-backed `mdot*c*` source rate, declared
ignite-command timing evidence, point-mass and rigid-body runtime-observed
ignite timing evidence, runtime source-history reconstruction, and
runtime-reconstructed envelope evidence, and point-mass/rigid-body runner
report/telemetry values. Missing for full closure: calibrated/public
`d(mdot*c*)/dt` reconstruction and command-response pressure-history
validation, calibrated/public duct/trench and suppression decks, and public
IOP trend residuals.

WP-22.5 ascent fluctuating pressure + fairing internal: **partial** —
`openbmp-acoustics::ascent` now contains a Schema-1 synthetic ascent acoustic
environment deck parser/evaluator and a Schema-1 synthetic fairing-internal
acoustic transfer deck parser/evaluator. The checked ascent fixture
`data/acoustics/ascent-acoustic-env-v1.toml` declares attached-flow and
transonic toy classes with Mach gates, dynamic-pressure reference OASPL,
q-scaling exponents, OASPL-normalized one-third-octave relative band shapes,
and uncertainty labels. The checked fairing fixture
`data/acoustics/fairing-internal-v1.toml` declares light/heavy synthetic
fairing insertion-loss classes. Schema-v3 `[acoustics.ascent]` resolves and
pin-verifies the deck, validates either one explicitly selected class/trajectory
sample or `[[acoustics.ascent.samples]]` trace rows, supports
`class_selection = "mach_gate"` to select the unique deck class whose Mach gate
contains each sample, and the shared runner reports deck provenance, selected
class ids, peak sample metadata, envelope OASPL, uncertainty, deterministic
envelope spectrum CSV, deterministic sample trace CSV, and gated telemetry.
Schema-v3
`[acoustics.fairing_internal]` requires `[acoustics.ascent]`, resolves and
pin-verifies its fairing deck, applies fill-factor gain plus per-band
insertion loss to the external ascent envelope spectrum, and reports fairing
provenance, internal OASPL, uncertainty, deterministic spectrum CSV, and
gated telemetry. Tests cover parser/provenance fields, SHA-256 digest syntax,
exact q-scaling, exact OASPL normalization, fairing transfer math,
ascent trace envelope math, unknown-class rejection, explicit-class Mach-gate
rejection, automatic Mach-gate no-match and ambiguity rejection,
fill-factor bounds, band-grid mismatch rejection, malformed-deck rejection,
schema validation, deck pinning, runner provenance reporting, and telemetry
values. Missing for full closure: full `03` geometry-driven regime/zone
selection, published MPE/user-guide curves, payload-zone spatial maps, and
flight or payload-guide residual anchoring.

WP-22.6 vibroacoustic response + P95/50 environments + `02` handoff:
**partial** — `openbmp-acoustics::response` now contains a Schema-1 synthetic
vibroacoustic response transfer deck parser/evaluator. The checked fixture
`data/acoustics/vibroacoustic-transfer-v1.toml` declares light/heavy synthetic
panel classes with per-band acceleration-PSD gains, reference surface mass,
reference area, and uncertainty labels. The evaluator converts external
`SplSpectrum` input into pressure PSD, applies square-law surface-mass scaling
to acceleration PSD, computes modal-force PSD from pressure PSD times
`(area_m2 * mode_shape_factor)^2`, and exposes deterministic integrated
mean-square acceleration and force reductions. Schema-v3 `[acoustics.response]`
now resolves/pin-verifies the response deck, consumes either the ascent envelope
or fairing-internal spectrum, reports response provenance, acceleration and
modal-force PSD CSVs, integrated mean-square acceleration and force, and gated
telemetry. Tests cover parser/provenance fields, SHA-256 digest syntax,
SPL-to-pressure-PSD conversion, surface-mass scaling, modal-force
participation math, integrated reductions, malformed-deck rejection,
duplicate-class rejection, negative-gain rejection, invalid input rejection,
unknown-class rejection, band-grid mismatch rejection, schema validation, deck
pinning, runner provenance reporting, PSD CSV emission, and telemetry values.
`openbmp-acoustics::stats` now also computes deterministic normal P95/50 SPL
environment spectra from multiple common-grid spectra using the shared
standard-normal inverse, per-band dB mean/sigma, and deterministic CSV table
bytes. Multi-sample schema-v3 `[acoustics.ascent]` runs now retain per-sample
spectra and record P95/50 OASPL, tolerance factor, and environment-table CSV
bytes on `RunOutcome.acoustics.ascent`; fairing-internal reports apply the same
method to each transferred ascent sample spectrum, can pair retained ascent
spectra with `[[acoustics.fairing_internal.samples]]` fairing-side fill-factor
and class overrides, and record the corresponding internal P95/50 table plus
deterministic fairing sample trace CSV. Response reports also run the selected transfer over
the retained source sample spectra, optionally pair those spectra with
`[[acoustics.response.samples]]` response-side inputs, and record P95/50
acceleration/modal-force PSD tables, deterministic response sample trace CSV,
plus integrated P95/50 mean-square acceleration and force artifacts.
The runner records one-sided Wilks P95/50 sample-count sizing, achieved
confidence, adequacy, and deterministic sizing CSV evidence beside ascent,
fairing-internal, and response P95/50 artifacts. When CLI run-data output is
enabled, `openbmp run` writes nonempty ascent/fairing/response P95/50 CSV
payloads plus fairing/response P95/50 sample traces under the run-data artifact root
and indexes their roles, paths, and SHA-256 pins in the `RunRecord`. Response
reports also expose a deterministic
structural loads handoff CSV with modal-force PSD, per-band mean-square force,
signed modal participation, and total mean-square force; the CLI indexes that
handoff alongside the acoustics artifacts. The typed first-bending structural
acoustic consumer also derives a first-increment engine-station equivalent
reaction-moment line-load recovery CSV from the declared `slope_at_engine` and
modal acceleration PSD, and `openbmp run` indexes it as
`structural_acoustics_first_bending_line_load_recovery_csv`.
`openbmp mc acoustics-ascent-trace`
now materializes deterministic LHS/Sobol/Owen-Sobol
`[[acoustics.ascent.samples]]` TOML snippets over declared dynamic-pressure and
Mach ranges, so the existing runner P95/50 path can consume MC-designed ascent
traces. `openbmp mc acoustics-response-samples` now materializes deterministic
`[[acoustics.response.samples]]` TOML snippets over surface-mass, area, and
mode-shape ranges for response-side P95/50 input variation. `openbmp mc
acoustics-ascent-campaign` now injects generated ascent samples into a base
`[acoustics.ascent]` scenario, executes the normal runner, and writes the
evaluated sample trace CSV plus P95/50/Wilks evidence. When the base scenario
also declares `[acoustics.fairing_internal]` and `[acoustics.response]`, the
campaign report now surfaces the downstream fairing-internal P95/50 OASPL and
response P95/50 acceleration/modal-force mean-square values plus Wilks
sufficiency flags. Completed campaigns can now write nonempty evaluated
campaign CSV artifacts to `--output-artifact-dir`, including ascent/fairing
P95/50 environment tables, fairing P95/50 sample traces, response loads
handoff, response P95/50 PSD tables, response P95/50 sample traces,
structural-acoustic artifacts when present, and a SHA-pinned `manifest.json`
for the written artifacts; with
`--checkpoint-json`, it resumes per-sample OASPL evaluations through the
shared scalar checkpoint reducer and bounded workers before final artifact
emission. `openbmp mc acoustics-response-campaign` now wraps that same
generated ascent-trace path with a response-specific base-scenario check,
response P95/50 acceleration, modal-force, and Wilks summary reporting, and
optional generated response-side sample overlays in final campaign scenarios.
Missing for full closure: generated fairing design-variable CLI samplers,
generalized checkpointed/parallel MC campaign orchestration beyond these
ascent-trace-driven acoustic commands, calibrated SEA-lite/Franken transfer
evidence, and full `02` multi-mode/FEM-derived LTM/OTM structural consumer
wiring.

WP-22.7: **not started**.

### 23 — Electrical power & avionics emulation

**Not started.** No `openbmp-eps`, no battery/bus-profile/string-kill code.
(The bridge fault library from doc 10 is the substrate WP-23.5 will
decorate.)

### 24 — Postflight reconstruction & model correlation

WP-24.1: **implemented** — `openbmp-sysid` now provides the file-fed L6
residual harness with validated scalar time series, strict timestamp ordering,
unit/frame reconciliation, actual-to-reference interpolation, total and
phase-window residual statistics, out-of-family flags, local CSV ingestion,
and deterministic JSON reports. The CLI exposes `openbmp sysid residuals` for
LOCAL reference/actual CSV comparison (`REQ-SYSID-001`). Remaining doc-24
gaps: no parameter registry, output-error estimator, identifiability analysis,
or governance/deck-ingestion pipeline.

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
- **Traceability:** 360 requirement ids and 362 verification items in
  `requirements.toml`, including the REQ-PROP, REQ-GNC, REQ-MC, REQ-CONTACT,
  REQ-PLUME, and REQ-TRAJOPT families.
- **Byte-stable-by-default pattern observed** in everything that landed:
  pressure-thrust, transient grain, transports, realtime, contact, and
  fault schedules are all scenario-gated opt-ins.

---

## 4. Recommended next moves (from this snapshot)

1. **Close the open partials before opening new fronts:** WP-14.6 CFE
   scenario/model/public-benchmark closure, or continue doc 05 T4 public
   calibration/benchmark closure for WP-05.4-b/WP-05.4-c.
2. **Advance the Phase-A gates:** finish WP-01.1 (spatial-vector tree) and
   finish WP-08.1 (high-degree Pines/Gottlieb tesseral gravity) — they block
   most of Phase B/C (02, 06, 07 closed-loop quality).
3. Start WP-07.2 (Hermite-Simpson/SQP) or use the implemented WP-07.0/WP-07.1
   corrector stack as the prerequisite for WP-19.2.
4. WP-15.1 (plume state) is newly unblocked by 05's chamber/nozzle state.
