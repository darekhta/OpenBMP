# Rendezvous, Proximity Operations & Docking

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`07-trajectory-optimization-and-mission-design.md` (whose terminal-condition
vocabulary this document extends under lock),
`14-contact-dynamics-touchdown-and-landing.md` (whose contact substrate this
document is the promised consumer of).

> One-line scope: cooperative orbital rendezvous as a simulated, GNC-closed,
> contact-completed capability — relative-motion frames and linear models
> (CW and Tschauner-Hempel), multi-burn targeting and approach guidance
> (glideslope, hold points, corridors), mandatory passive-abort safety
> verification, relative-navigation sensors and a relative estimator,
> collision-avoidance/retreat FDIR, docking-envelope verification with
> soft-capture contact through `openbmp-contact`, and a two-vehicle SIL
> configuration — under a cooperative-target vocabulary with mechanical
> locks.

Doc `14` reserved this application explicitly ("the forthcoming
RPOD/docking doc … consume `openbmp-contact` directly"); doc `07`'s
vocabulary already contains `RendezvousState` as an inertial cooperative
state; doc `09` builds the vision/measurement substrate. This document
assembles the dimension and adds the locks the relative-motion surface
needs.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **NASA/Orion** — 6-DOF RPOD GN&C simulations for handling qualities and
  sequencing; the Vision Navigation System (flash lidar to docking range)
  was flight-tested on STS-134 (STORRM) before use; Lockheed validates
  docking sensors on robotic gantries; JSC's Systems Engineering Simulator
  trains visiting-vehicle capture (public NTRS/feature articles).
- **SpaceX/Dragon** — DragonEye lidar flight-tested against ISS on Shuttle
  missions before Dragon flew RPOD; crew-in-the-loop integrated sims with
  NASA control centers; the public `iss-sim` docking simulator is built
  from the actual Crew Dragon display designs (separate code, same team) —
  evidence of an end-to-end RPOD sim culture.
- **Rocket Lab** — MAX flight software markets RPOD heritage ("OTV's +
  RPOD"); the VICTUS HAZE award has Rocket Lab building and operating a
  rendezvous/proximity-operations demonstration.
- **Method canon** — Clohessy-Wiltshire/Hill linearized relative motion;
  Tschauner-Hempel/Yamanaka-Ankersen for eccentric chiefs; glideslope
  approaches (Hablani); V-bar/R-bar station-keeping and hops; passive
  abort safety (free-drift clearance) as a design requirement; IDSS — the
  *public* International Docking System Standard interface document —
  defines capture-envelope conditions.

### 1.2 Parity target (capability, within posture)

1. **Relative frames & dynamics** — chief-centered LVLH/RIC frame
   machinery; CW state-transition matrix for circular chiefs;
   Tschauner-Hempel/Yamanaka-Ankersen STM for eccentric chiefs; full
   nonlinear truth via the existing two-vehicle propagation
   (`[multi_body]`).
2. **Targeting & approach guidance** — CW two-impulse transfers and
   multi-burn sequences to hold points; glideslope approach law; V-bar/
   R-bar profiles with station-keeping; approach corridors and keep-out
   sphere as *constraints*; terminal conditions strictly through the `07`
   vocabulary plus one new locked type (`RelativeBox`, whose closing-speed
   bound is capped at construction — WP-25.2).
3. **Passive-abort safety** — mandatory verification that a free-drift
   (and declared single-fault burn) trajectory from every approach segment
   stays outside the keep-out volume for N orbits — fail-closed: an
   approach plan without a passing safety certificate does not run
   closed-loop.
4. **Relative navigation** — rendezvous sensor models (range/range-rate
   and bearing with FOV/range gates, reflector-cooperative modes),
   angles-only far-field tier, PnP pose handoff from `09`'s vision tier for
   terminal approach; an FC-side relative estimator (CW-propagated
   error-state EKF) behind the portability lock.
5. **Prox-ops FDIR** — corridor/rate violation detection, retreat and
   collision-avoidance-maneuver (CAM) library (declared, verified against
   the same passive-safety machinery), abort-to-hold semantics.
6. **Docking & capture** — IDSS-shaped capture-envelope verification
   (closing rate, lateral rate, misalignment angle boxes), soft-capture
   compliance + latch FSM through `openbmp-contact` (doc `14` WP-14.2
   primitives), capture-probability Monte Carlo over dispersions; berthing
   variant (grapple-envelope hold) as configuration.
7. **Two-vehicle SIL** — chaser FC in the loop (doc `10` boundary), target
   as declared trajectory or second propagated body; full mission case
   (far-field → hold points → corridor → capture) as the capstone
   scenario.

### 1.3 Parity ceiling (honest boundary)

- **No real sensor characterization.** Flash-lidar point clouds, real
  camera radiometry, and flight-qualified rendezvous-sensor error maps are
  proprietary; models are parametric measurement-level (per `09`'s
  ceiling), with synthetic target geometry.
- **No ISS/visiting-vehicle program data.** Approach profiles and rules are
  reconstructed from public literature; the *public* IDSS interface
  definition bounds the docking-envelope numbers used.
- **No crewed handling-qualities validation.** Piloted-loop evaluation
  (SES-class) is out of scope; everything here is autonomous GNC.
- **No non-cooperative operations.** Inspection/servicing of non-declared,
  non-cooperative objects — relative nav against uncooperative targets,
  grappling, debris capture — is **out of scope by posture**, not deferred:
  the cooperative-target validator and the closing-speed-capped
  `RelativeBox` constructor (WP-25.2) make the formulation unrepresentable
  rather than merely avoided.
- **Contact fidelity ceiling** — inherited from doc `14` (compliant
  penalty tier; no flight-calibrated docking-mechanism parameters).

---

## 2. Current state in source

- `crates/openbmp-runner/src/{separated_attitude.rs,separated_landing.rs}`
  + `scenarios/multi-body` — simultaneous multi-body propagation exists
  (v3): the truth-side substrate for chaser+target.
- `crates/openbmp-trajopt/src/target.rs` — the locked terminal-condition
  vocabulary including `RendezvousState` (inertial cooperative state);
  `07` designs the solver ladder above it.
- `crates/openbmp-physics/src/frames.rs` + `08` machinery — inertial/ECEF
  frames; no LVLH/relative-frame machinery yet.
- `crates/openbmp-fc/src/{guidance.rs,estimator.rs,fdir.rs}` — the FC
  surfaces the relative guidance/estimator/FDIR additions extend
  (portability-locked).
- `crates/openbmp-sensors/src/` — measurement-model patterns; `09`
  (designed) adds vision/PnP and the error vocabulary.
- `14` (designed) — `openbmp-contact` substrate + latch primitives +
  reserved docking consumer; `15` (designed) — plume impingement machinery
  for approach-plume effects on the target.
- No relative dynamics, no rel-nav measurements, no prox-ops guidance, no
  capture mechanism exists.

---

## 3. Target architecture

### 3.1 Crate boundary

No new crate by default: relative frames/dynamics land in
`openbmp-physics` (kinematics layer), guidance/estimator/FDIR in
`openbmp-fc` (portability lock holds — they are flight algorithms),
measurement models in `openbmp-sensors`, capture mechanics consume
`openbmp-contact`, orchestration in the runner. If the relative-dynamics
surface grows past its host, `openbmp-relmotion` (L2) is the reviewed
fallback. This mirrors doc `16`'s no-new-crate pattern.

### 3.2 Relative frames & dynamics (T1)

- **Frames:** chief-centered LVLH/RIC (radial, in-track, cross-track) with
  exact rotating-frame kinematics from the chief's inertial state;
  transforms both ways with locked operand order.
- **CW (circular chief):**

```
ẍ − 3n²x − 2nẏ = aₓ
ÿ + 2nẋ        = a_y
z̈ + n²z        = a_z
```

  with the closed-form STM `Φ_cw(t)` for coast arcs.
- **Tschauner-Hempel / Yamanaka-Ankersen (eccentric chief):** TH equations
  in true anomaly with the YA state-transition matrix (the standard
  singularity-free formulation) for coast propagation.
- **Truth:** full nonlinear two-vehicle propagation through the existing
  kernel (`[multi_body]`); linear models are FC-side approximations whose
  error vs truth is itself a V&V quantity.

### 3.3 Targeting & approach guidance (T2)

- **CW targeting:** two-impulse transfer between relative states over time
  `τ` by STM block inversion (`Δv₁ = Φ_rv⁻¹(τ)·(r_f − Φ_rr(τ)·r₀) − v₀`,
  fail-closed near singular `Φ_rv`); multi-burn sequences as chained
  segments between **hold points** (declared relative states with
  station-keeping deadbands).
- **Glideslope law (Hablani):** commanded approach along a declared
  glideslope with decaying closing rate (`ρ̇ = −a·ρ + b`), discretized into
  deterministic burn schedules.
- **Profiles:** V-bar approach (in-track), R-bar approach (radial,
  natural-braking), fly-around (declared-radius circumnavigation at
  standoff). All profiles are sequences over the same segment vocabulary.
- **Constraints:** keep-out sphere (KOS) radius, approach-corridor cone
  (axis, half-angle), range-dependent closing-rate ceiling — evaluated
  continuously; violations latch FDIR events (§3.6).
- **Far-field phasing** stays in doc `07`'s machinery (orbit-level
  targeting to `RendezvousState`); this document begins at relative-frame
  acquisition range.

### 3.4 Passive-abort safety verification (T3)

For every approach segment: propagate free-drift (no further burns) from a
declared sample set along the segment (entry, mid, end states + dispersion
draws via `11`) for N orbits; require minimum KOS clearance; repeat for the
declared single-fault cases (one burn missed, one burn at declared
over/under-performance). Output: a **safety certificate** artifact
(per-segment clearance statistics, provenance-pinned) — the closed-loop
executive refuses to fly an approach whose certificate is missing, stale
(plan hash mismatch), or failing. This mechanizes the passive-safety design
rule of the RPOD canon.

### 3.5 Relative navigation (T3–T5)

- **Sensors (plant side, `openbmp-sensors`):** rendezvous range/range-rate
  + bearing instrument (FOV gate, range gates, cooperative-reflector mode
  with declared acquisition envelope, `09` error vocabulary); far-field
  angles-only bearing tier; terminal vision pose via `09`'s PnP (marker/
  feature-based, behind the `vision` feature).
- **Estimator (FC side):** error-state EKF on relative state with
  CW/TH-propagated covariance, measurement gating consistent with the
  existing estimator patterns; angles-only far-field initialization
  (batch over a declared arc via the `11` machinery offline, or
  declared-prior handoff). Estimator divergence latches FDIR.
- **Truth-referenced scoring:** estimate-vs-truth monitors reuse
  `openbmp-sil` machinery (the live-SIL observation pattern).

### 3.6 Prox-ops FDIR: corridors, retreat, CAM (T4)

Detection: corridor-cone violation, KOS penetration prediction (CW
short-horizon propagation of the *estimated* state), closing-rate-ceiling
violation, estimator-health flags. Responses (declared, monotone):
hold-at-current-range → retreat along the approach axis → CAM (a declared
separation burn whose post-burn free-drift is itself verified by the §3.4
machinery at plan time). All responses are vehicle-relative and
pre-verified; FDIR can only move the chaser *away* from the target or hold.

### 3.7 Docking & capture (T5)

- **Envelope verification:** at declared capture-attempt range, check
  closing rate, lateral rate, misalignment angles, and lateral offset
  against the declared (IDSS-shaped) capture box; out-of-envelope ⇒
  mandatory retreat (never "try harder").
- **Soft capture:** compliance + latch FSM through `openbmp-contact`
  (WP-14.2 latch + WP-14.1 compliant contact between docking-interface
  frames); post-capture relative motion damps through declared compliance;
  hard-capture latch completes the FSM.
- **Capture MC:** dispersion campaigns (nav errors, actuator errors, link
  dropouts from `20` if configured) → capture probability + envelope-margin
  statistics with `11` sizing.
- **Berthing variant:** grapple-envelope hold (station-keep inside a
  declared box for a declared duration) without contact dynamics.

### 3.8 Two-vehicle SIL & capstone (T6)

Chaser FC behind the `10` SIL boundary (all guidance/estimation/FDIR of
§3.3–§3.7 run as flight software); target as declared ephemeris or second
propagated body (passive attitude or scripted); plume-impingement loads on
the target during terminal approach via `15`'s impingement machinery
(self-consistent approach-plume inhibition zones as corridor constraints).
Capstone scenario: LEO target, far-field handoff from `07`, hold points,
V-bar approach, capture — dispersed, judged via `21` verdicts.

### 3.9 Fidelity tiers

- **T0 (current):** multi-body truth propagation only.
- **T1:** relative frames + CW/TH/YA models + error-vs-truth
  quantification — `validated-toy` → `research` (YA vs literature).
- **T2:** targeting + glideslope + hold points + profiles (open-loop) —
  `validated-toy`.
- **T3:** passive-safety verifier + rel-nav sensors + relative EKF
  (closed-loop) — `validated-toy`.
- **T4:** corridors/retreat/CAM FDIR — `validated-toy`.
- **T5:** capture envelope + soft-capture contact + capture MC —
  `validated-toy` (mechanism tier inherited from `14`).
- **T6:** two-vehicle SIL capstone + plume-inhibit corridors —
  `validated-toy`.

---

## 4. Invariant preservation

- **Determinism:** STMs and guidance laws are closed-form/fixed-iteration;
  dispersion draws on `DeterministicRng` domains; two-vehicle propagation
  already obeys the kernel contract; capture sub-stepping inherits `14`'s
  fixed sub-step discipline.
- **Byte-stable default:** behind `[rpod]` scenario blocks; goldens
  untouched; the relative-frame machinery is pure library code until
  configured.
- **FC portability:** guidance/estimator/FDIR land in `openbmp-fc` with no
  new dependency edges (tripwire stays green); sensors stay plant-side;
  the safety verifier is an *offline* artifact producer whose certificate
  the FC consumes as an I-load-class input across the `10` boundary.
- **Provenance:** capture-envelope boxes, sensor decks, and safety
  certificates are pinned; certificates embed the plan hash they certify.
- **Fail-closed:** singular targeting geometry, missing/stale safety
  certificates, out-of-envelope capture attempts, unknown profile/segment
  types — all structured errors or mandatory retreats.
- **Labels:** per §3.9; nothing exceeds `research`.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| LVLH transforms round-trip + rotating-frame kinematics | property/analytic | T1 | < 1e-12 rel |
| CW STM vs closed form; period/secular terms | analytic | T1 | < 1e-12 rel |
| YA STM vs numerical integration of TH equations | internal code-to-code | T1 | < 1e-9 rel over declared e-range |
| Linear-model error vs nonlinear truth vs range | characterization | T1 | documented error envelope (table) — the honesty product |
| Relative ephemeris vs external astrodynamics oracle (Orekit/GMAT-class, external process) | code-to-code | T1 | < declared tolerance on the reference case (table) |
| Two-impulse CW targeting reaches target state | analytic | T2 | terminal residual < 1e-9 rel (linear); documented vs truth |
| Glideslope law tracks declared profile | scenario | T2 | rate/range profile within declared band |
| Hold-point station-keeping deadband behavior | scenario | T2 | bounded excursion; deterministic burn schedule |
| Passive-safety verifier: constructed unsafe segment is refused | property/scenario | T3 | fail-closed (no closed-loop run without passing certificate) |
| Relative EKF: NEES/NIS consistency on synthetic truth | statistical (per `11` convention) | T3 | in-bounds fraction per declared band |
| Angles-only observability: range unobservable without maneuver, recovered after declared maneuver | analytic property | T3 | matches known observability result |
| Corridor violation → retreat → CAM sequence | scenario | T4 | declared monotone response; CAM post-drift clears KOS |
| Capture envelope gating (out-of-box ⇒ retreat) | property | T5 | exact box semantics |
| Soft-capture contact: latch engages once; post-capture energy damps | scenario (via `14` machinery) | T5 | latch monotone; energy audit closes |
| Capture probability MC convergence | statistical | T5 | Clopper-Pearson reported; deterministic across workers |
| Capstone mission end-to-end (SIL) | golden/scenario | T6 | byte-stable; verdicts green via `21` |

---

## 7. Dependencies on other parity docs

- `07` — far-field phasing to `RendezvousState`; the terminal-condition
  vocabulary this document extends under lock; STM machinery patterns.
- `14` — contact substrate (compliant contact, latch FSM) for soft
  capture; this is the consumer doc `14` reserved.
- `09` — measurement-error vocabulary, vision/PnP tier for terminal pose,
  actuator models for prox-ops thrusters (RCS MIB matters at these Δv
  scales).
- `08` — frames/epochs, gravity for truth propagation, J₂-class effects on
  long-drift safety verification.
- `10` — SIL boundary for the chaser FC; safety certificate as an
  I-load-class artifact; two-vehicle SIL wiring.
- `15` — plume impingement on the target; approach-plume inhibition
  corridors.
- `11`/`12` — dispersion campaigns (capture MC, safety sampling),
  deterministic parallel execution, NEES/NIS conventions.
- `06` — estimator/FDIR patterns the relative additions follow; engine-out
  analog (thruster-out) reconfiguration context.
- `20` — link effects during prox-ops (dropout-tolerant FDIR cases).
- `21` — capstone verdicts, certificates and reports as stored artifacts.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| Fehse, *Automated Rendezvous and Docking of Spacecraft* | method canon (profiles, safety, sensors) | published textbook |
| Yamanaka-Ankersen STM paper | TH propagation formulation | published literature |
| Hablani glideslope guidance paper | approach-law formulation | published literature |
| IDSS Interface Definition Document | public docking-envelope parameter source | public standard document |
| Orekit / GMAT | external relative-ephemeris oracles (code-to-code, external process) | Apache-2.0 |
| Basilisk | named MC/formation reference implementation | ISC (external) |
| NASA STORRM / Orion VNS public papers | sensor-tier reference practice | public NTRS |
| 42 (GSFC) | attitude/prox-ops cross-check oracle where applicable | NASA open source |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-25.1 — Relative frames + CW/TH/YA models + error characterization

- **title:** LVLH/RIC machinery, CW closed-form STM, Yamanaka-Ankersen STM, linear-vs-nonlinear error envelopes, external-oracle cross-check.
- **goal:** The mathematical substrate of the whole dimension, with the
  honesty product built in: documented envelopes for where the linear
  models are trustworthy.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (per §3.1; boundary review recorded in the PR).
- **touched:** `crates/openbmp-physics/src/frames.rs` (+ new relative
  module), unit/property tests, characterization fixtures.
- **approach:** §3.2; closed forms with locked operand order; truth via
  `[multi_body]` propagation.
- **acceptance:**
  - pure library addition; canonical goldens byte-identical
  - CW/YA analytic and internal code-to-code cases per §5 (< 1e-12 /
    < 1e-9 rel)
  - linear-vs-truth error envelope documented (tolerance table)
  - external-oracle relative-ephemeris case within declared tolerance
  - all `13` §2 gates green
- **validation_label:** `validated-toy` → `research` (YA literature anchor)
- **dual_use_note:** mathematics only; locks arrive with the first
  command surface (WP-25.2).
- **est_effort:** 2–3 weeks
- **parity_ceiling:** linear models to documented envelopes; truth is the
  existing kernel.

### WP-25.2 — Targeting + `RelativeBox` + cooperative-target locks

- **title:** CW two-impulse/multi-burn targeting between hold points; the `RelativeBox` terminal condition with construction-capped closing speed; cooperative-target validator; ground-frame tripwire.
- **goal:** The first command surface, shipped *with* its locks: matched-
  velocity-by-type terminal conditions, a target validator that rejects
  surface-intersecting "targets", and the schema-level absence of ground
  coordinates — all proven in the same PR.
- **fidelity_tier:** T2
- **depends_on:** [WP-25.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-trajopt/src/target.rs` (locked vocabulary
  addition), `crates/openbmp-scenario` (`[rpod]` schema), tripwires in
  `crates/openbmp-testkit`.
- **approach:** §3.3 targeting; locks: closing-speed-capped `RelativeBox`
  constructor, perigee-floor target validator, ground-frame schema tripwire.
- **acceptance:**
  - off by default; goldens byte-identical
  - two-impulse residual < 1e-9 rel (linear); documented vs truth
  - **tripwires (same PR):** high-closing-rate `RelativeBox`
    unconstructible (compile-fail/test); surface-intersecting target
    rejected at load; no ground-frame field in `[rpod]` schema (closure
    test)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** active guardrail WP — the cooperative-target locks
  land here, before any closed-loop capability.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** linear targeting tier; far-field phasing remains
  `07`'s.

### WP-25.3 — Approach profiles: glideslope, V-bar/R-bar, hold points, corridors

- **title:** Glideslope law, V-bar/R-bar/fly-around profiles, station-keeping deadbands, corridor-cone/KOS/rate-ceiling constraint evaluation (open-loop).
- **goal:** The prox-ops profile vocabulary, evaluated open-loop against
  truth: deterministic burn schedules, constraint monitors, and the
  segment structure everything downstream (safety, FDIR, SIL) reuses.
- **fidelity_tier:** T2
- **depends_on:** [WP-25.2]
- **new_crates:** none.
- **touched:** `crates/openbmp-fc/src/guidance.rs` (profile/segment
  vocabulary — portability lock), `crates/openbmp-runner` wiring, scenario
  schema, gated telemetry.
- **approach:** §3.3; declared profiles as segment sequences; constraints
  evaluated continuously with latched violations.
- **acceptance:**
  - off by default; goldens byte-identical
  - glideslope tracks declared profile within band; R-bar natural-braking
    behavior matches CW prediction
  - constraint monitors exact on constructed violations (property)
  - `fc_dependency_tripwire` green
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** approach-only vocabulary; constraints are protective.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** canonical profile set; no optimized approach
  trajectories (that remains `07`-tier work under the same locks).

### WP-25.4 — Passive-abort safety verifier + certificate artifact

- **title:** Free-drift + single-fault clearance verification per approach segment over dispersions; plan-hash-bound safety certificates; closed-loop refusal without one.
- **goal:** The safety rule of the RPOD canon, mechanized and fail-closed:
  no certified plan, no closed-loop approach — and the certificate is a
  pinned, reviewable artifact.
- **fidelity_tier:** T3
- **depends_on:** [WP-25.3; cross-doc: WP-11.0 (MC substrate)]
- **new_crates:** none.
- **touched:** offline verifier (host-side module per §3.1), certificate
  schema + provenance integration, runner gate, fixtures.
- **approach:** §3.4; declared N orbits, sample sets, fault cases;
  Wilks/CP-sized clearance statistics.
- **acceptance:**
  - off by default; goldens byte-identical
  - constructed unsafe segment fails certification; closed-loop run
    without/with-stale certificate is refused (fail-closed tests)
  - clearance statistics deterministic across worker counts
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** the protective core of the posture — every approach
  must be proven to miss under fault.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** declared fault set; no program-certified safety case.

### WP-25.5 — Relative-navigation sensors + relative EKF (closed loop)

- **title:** Range/range-rate/bearing rendezvous sensor + angles-only tier (plant side); CW/TH-propagated error-state relative EKF with gating (FC side); estimate-vs-truth scoring.
- **goal:** Closing the loop honestly: the chaser flies on estimated
  relative state from modeled sensors, with consistency statistics and the
  known angles-only observability behavior demonstrated.
- **fidelity_tier:** T3
- **depends_on:** [WP-25.3; cross-doc: `09` error vocabulary; WP-09.10 for
  the vision tier later]
- **new_crates:** none.
- **touched:** `crates/openbmp-sensors` (rendezvous instrument),
  `crates/openbmp-fc/src/estimator.rs` (relative error-state EKF —
  portability lock), `crates/openbmp-sil` scoring reuse, scenario schema.
- **approach:** §3.5; FOV/range gates; NEES/NIS per `11` convention;
  divergence latches FDIR flag.
- **acceptance:**
  - off by default; goldens byte-identical
  - NEES/NIS in-bounds fraction within declared band on synthetic truth
  - angles-only property: range unobservable pre-maneuver, recovered
    post-maneuver (test)
  - closed-loop glideslope on estimated state stays within declared
    corridor on the reference case
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** sensors are cooperative-mode measurement models;
  vocabulary locks from WP-25.2 unchanged.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** parametric measurement-level sensors; no point-cloud/
  radiometric fidelity.

### WP-25.6 — Prox-ops FDIR: corridor enforcement, retreat, CAM

- **title:** Violation detection on estimated state, monotone hold→retreat→CAM response ladder, plan-time CAM verification through the safety machinery.
- **goal:** The protective autonomy of real prox-ops: any anomaly moves the
  chaser away or holds it, with every escape trajectory pre-verified to
  clear the keep-out volume.
- **fidelity_tier:** T4
- **depends_on:** [WP-25.4, WP-25.5]
- **new_crates:** none.
- **touched:** `crates/openbmp-fc/src/fdir.rs` (prox-ops responses —
  portability lock), CAM library schema, scenario fixtures.
- **approach:** §3.6; short-horizon CW prediction on estimated state;
  declared monotone ladder; CAM drift verified at plan time (WP-25.4
  machinery).
- **acceptance:**
  - off by default; goldens byte-identical
  - constructed corridor violation triggers the declared ladder
    deterministically; CAM post-drift clears KOS (scenario + certificate)
  - responses are provably hold-or-separate (property over the response
    set — no response reduces range)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** responses can only hold or increase separation —
  enforced by test.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** declared response ladder; no certified abort
  envelopes.

### WP-25.7 — Docking capture: envelope gate, soft capture via contact, capture MC

- **title:** IDSS-shaped capture-box gating (out-of-envelope ⇒ retreat), soft-capture compliance + latch through `openbmp-contact`, berthing hold variant, capture-probability campaigns.
- **goal:** The contact-completed ending doc `14` promised: capture
  succeeds only inside a declared envelope, engages through the compliant
  latch substrate, and earns a dispersed capture-probability statement.
- **fidelity_tier:** T5
- **depends_on:** [WP-25.5; cross-doc: WP-14.1/WP-14.2 (contact + latch)]
- **new_crates:** none.
- **touched:** capture module (runner wiring + interface frames),
  `data/` envelope decks (IDSS-shaped, public-source provenance), capture
  MC campaign config, telemetry.
- **approach:** §3.7; box gate exact; compliance/latch per `14`; MC via
  `11`/`12`.
- **acceptance:**
  - off by default; goldens byte-identical
  - envelope gate exact (property); out-of-envelope attempts retreat,
    never contact
  - latch engages exactly once; post-capture energy audit closes (with
    `14` machinery)
  - capture-probability campaign deterministic across workers with CP
    intervals reported
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** capture only inside matched-velocity envelope —
  the WP-25.2 type cap makes the alternative unrepresentable.
- **est_effort:** 3 weeks
- **parity_ceiling:** `14`'s compliant-contact tier; no flight-calibrated
  mechanism parameters.

### WP-25.8 — Two-vehicle SIL capstone + plume-inhibit corridors

- **title:** Chaser-FC-in-SIL full mission (far-field handoff → holds → corridor → capture) with `15` plume-impingement inhibition zones and `21`-judged verdicts.
- **goal:** The dimension's definition-of-done: one dispersed, judged,
  byte-stable mission scenario proving every piece composes behind the
  flight-software boundary.
- **fidelity_tier:** T6
- **depends_on:** [WP-25.6, WP-25.7; cross-doc: WP-10.1 (SIL transport),
  WP-15.8 (impingement) optional]
- **new_crates:** none.
- **touched:** capstone scenario under `scenarios/`, SIL wiring, verdict
  rules, documentation.
- **approach:** §3.8; target as declared ephemeris first, propagated body
  second; plume-inhibit zones as corridor constraints where `15` is
  available.
- **acceptance:**
  - off by default; goldens byte-identical (new scenario adds its own
    goldens)
  - capstone byte-stable; verdicts green; dispersed variant reports
    capture probability with CP intervals
  - SIL determinism hash stable across transports (per `10` gate)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** the full stack runs under every lock introduced
  above; the capstone is itself the regression proof that they hold.
- **est_effort:** 3 weeks
- **parity_ceiling:** simulated cooperative mission; no program V&V, no
  crewed handling qualities.

---

## 10. References

- Fehse, W., *Automated Rendezvous and Docking of Spacecraft* (Cambridge) —
  profiles, passive safety, sensor architecture.
- Clohessy, W. H., & Wiltshire, R. S., "Terminal Guidance System for
  Satellite Rendezvous" (JAS, 1960).
- Yamanaka, K., & Ankersen, F., "New State Transition Matrix for Relative
  Motion on an Arbitrary Elliptical Orbit" (JGCD, 2002).
- Hablani, H. B., et al., "Guidance and Relative Navigation for Autonomous
  Rendezvous in a Circular Orbit" (JGCD, 2002) — glideslope law.
- International Docking System Standard (IDSS) Interface Definition
  Document (public).
- Orion RPOD GN&C simulation papers (NTRS 20070025134, NTRS 20200001393);
  STORRM flight-test reports (NTRS).
- Goodman, J. L., "History of Space Shuttle Rendezvous and Proximity
  Operations" (JSC, NTRS) — operational canon.
- Public reporting on DragonEye flight tests and integrated mission
  simulations (NASA Commercial Crew articles).
