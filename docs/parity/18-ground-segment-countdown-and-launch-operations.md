# Ground Segment, Countdown & Launch-Release Operations

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`docs/mission-graph-architecture.md` (the mission state machine this document
builds beside, not inside).

> One-line scope: bring the ground half of a launch inside the simulation
> boundary — a declarative countdown autosequencer, a launch-commit-criteria
> engine, GSE/umbilical/T-0 interface models, hold/recycle/scrub logic with
> consumables bookkeeping, pad geometry with hold-down release and
> tower-clearance screening, and a scripted rehearsal harness over the SIL
> boundary.

Every reference organization simulates its ground segment: the audit
dimension exists because a launch is a *joint* vehicle/ground system, and the
current workspace simulates only the vehicle. The mission state machine
(`crates/openbmp-mission`) has pad-side vocabulary (standby, countdown hold,
liftoff) but nothing evaluates commit criteria, models the T-0 interface, or
sequences ground events against vehicle state.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **NASA KSC — SCCS** (Spaceport Command and Control System) is the Artemis
  launch-control software hub; NASA documents that SCCS development and
  launch-team training require **GSE and launch-vehicle
  simulations/emulators** across the system's life cycle — countdown
  rehearsals run the firing room against an emulated vehicle at the pad.
- **SpaceX** runs a terminal-count autosequence (~T-10 min on Falcon 9) with
  automated abort rules through T-0: public statements describe a "standard
  auto-abort triggered due to out of family data during engine power check";
  Starship simulation job postings put "vehicle **and ground systems**"
  explicitly inside the simulation models the team builds.
- **Rocket Lab** runs an onboard auto-sequence from T-2:00, formal
  launch-commit criteria, and wet/mission dress rehearsals as standard
  milestones (Electron Payload User Guide).
- **Firefly** advertises simulation infrastructure spanning "launch sites,
  launch vehicles, and spacecraft".
- **NASA MSFC — CLVTOPS-class practice**: liftoff tower-clearance drift and
  separation proximity screening by Monte Carlo, validated against Ares I-X
  flight data — a standing analysis product tied to pad geometry.
- **MAESTRO** (SLS SIL) acts as automated test conductor — configuration,
  control, archiving — for avionics-in-the-loop campaigns: rehearsal
  automation is itself a capability.

### 1.2 Parity target (capability, within posture)

1. **Countdown autosequencer** — a declarative, deterministic countdown
   timeline (steps, prerequisites, windows, built-in holds, recycle edges)
   evaluated against simulated vehicle + ground state, with auto-abort rules
   active through a modeled T-0.
2. **Launch-commit-criteria (LCC) engine** — declarative predicates over
   telemetry channels (limits, rates, persistence windows, dependency logic,
   out-of-family detection against an ensemble baseline), latched verdicts,
   fail-closed semantics.
3. **GSE/umbilical/T-0 models** — ground power/data paths that hand over at
   declared events (couples docs `23` and `20`), umbilical disconnect
   transients, swing-arm clearance volumes, hold-down latch release with a
   dispersed release window, thrust-at-release gating.
4. **Hold/recycle/scrub graph** — countdown re-entry points with consumables
   bookkeeping (cryo topping from `17`, pressurant budget, battery state from
   `23`) deciding whether a recycle is *feasible*.
5. **Pad geometry & liftoff clearance screening** — tower/umbilical keep-out
   volumes, drift Monte Carlo over winds/thrust dispersions (CLVTOPS-class
   deliverable on the existing rigid-body kernel).
6. **Rehearsal harness** — scripted wet-dress / mission-dress rehearsals with
   fault injection over the `10` SIL boundary, producing deterministic
   verdict reports (consumed by `21`).

### 1.3 Parity ceiling (honest boundary)

- **No real GSE control software or hardware interfaces.** SCCS-class systems
  command real plumbing and power; OpenBMP models declared ground plant
  behavior, never drives hardware (`docs/safety-boundaries.md`). The HAL/
  bridge boundary stays abstract.
- **No site-certified procedures or operator-certification claims.** Real
  countdown procedures, console certification, and range coordination are
  operational artifacts; the open substitute is *publicly documented
  countdown structure* (published timelines) expressed in a synthetic,
  clearly labeled procedure schema.
- **No facility hydraulics design.** Ground cryo plant is a declared
  boundary-condition model (flow/temperature capability envelopes), not a
  facility simulation; `17` owns the tank-side physics.
- **No real weather feeds at runtime.** Weather enters as scenario data or
  the `19` measured-profile workflow; live met integration is operational
  practice, documented as a LOCAL workflow only.

---

## 2. Current state in source

- `crates/openbmp-mission/src/hsm.rs`, `regions.rs`, `events.rs` — the
  hardware-portable hierarchical mission state machine with pad-side states
  and event bindings; runs FC-side. It is the *vehicle's* view of the
  countdown; there is no ground-side counterpart.
- `crates/openbmp-scenario-script/` — simulator-side scripted actions (engine
  commands, jettison): the stimulus pattern the rehearsal harness generalizes.
- `crates/openbmp-runner/src/mission.rs` — runner wiring of mission events.
- `crates/openbmp-sil/src/checks.rs`, `monitor.rs` — verdict machinery the
  LCC engine's report card reuses.
- `crates/openbmp-bridge/src/fault.rs`, `ports.rs` — fault-injection seams
  (doc `10`) the rehearsal harness drives.
- Liftoff today is an initial condition: no hold-down, no release window, no
  T-0 electrical/data handover, no tower geometry. `14` (contact) gives the
  mechanical substrate a hold-down release can use.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-ground` (L7): ground-segment models are simulator-side orchestration
(they read truth and inject boundary conditions), so they sit beside the
runner, never below the FC boundary. Depends on `openbmp-core`/`-state`/
`-models`/`-scenario` types and the runner's wiring seams; **never** a
dependency of `openbmp-fc` (the FC keeps seeing only bus channels). The
clearance-screening WP reuses `openbmp-mc`.

### 3.2 Countdown autosequencer

A declarative timeline document (TOML, `deny_unknown_fields`):

```toml
[[countdown.step]]
id          = "strongback_retract"
at          = "T-00:04:30"
window_s    = 30.0
requires    = ["lcc.range_green", "tanks.lox_replenish_stable"]
on_fail     = "hold:built_in_t4"   # hold point or "abort"
emits       = ["gse.strongback_retract"]
```

Semantics: a DAG of steps with absolute count anchors, prerequisite predicate
references (LCC ids or step completions), failure edges to declared hold
points or abort, and emitted events consumed by ground models (`17` loading
FSM, `23` power handover, `20` telemetry routing) and by the vehicle through
existing scenario-script/bridge seams. Evaluation is deterministic:
registration-order iteration, fixed evaluation cadence on the sim clock, no
wall-clock. Built-in holds advance a *count clock* distinct from sim time
(the count can hold while physics continues), mirroring real practice.

### 3.3 Launch-commit-criteria engine

Declarative predicate library over named telemetry channels:

- **Forms:** static limits, rate limits, persistence windows (`violated for
  > N s`), hysteresis bands, cross-channel dependencies (`A green requires B
  green`), and **out-of-family detection** — robust z-score (median/MAD)
  against a *baseline ensemble band* generated from prior runs of the same
  scenario family (the band ships as a provenance-pinned artifact; `21`'s
  baseline machinery produces it). Out-of-family is how reference practice
  catches "legal but wrong" readings (the documented SpaceX T-0 auto-abort
  class).
- **Verdict semantics:** each criterion evaluates green/red/violated-latched
  on the fixed cadence; latching is monotone within a count segment and
  resets only at declared recycle points. Unknown channels, missing
  baselines, or schema drift **fail closed** (criterion red, load error at
  parse time where detectable) — consistent with the scenario-consumer
  agreement rule.
- **Auto-abort through T-0:** criteria remain armed through the release
  window; a violation after engine start commands shutdown + safing through
  the normal FC command path (the vehicle's own abort-to-SAFE from `06` —
  the ground engine only *requests*, the FC state machine executes).

### 3.4 GSE, umbilical & T-0 interface

- **Umbilical model:** declared ground power/data paths with disconnect
  events: power handover (vehicle on internal power at T-X, couples `23`'s
  EPS source switch), data-path drop (telemetry routing switches from
  umbilical transport to RF link, couples `20`), purge/pressurant supply
  termination (couples `17`).
- **Disconnect transients:** deterministic event timing with declared
  dispersion (drawn via `DeterministicRng` in MC); a disconnect-failure fault
  type for rehearsals.
- **Hold-down & release:** latch elements (from `14`'s primitive vocabulary)
  holding the vehicle against thrust buildup; release gated on a
  thrust-at-release criterion (per-engine chamber pressure from `05` T2
  transients ≥ declared threshold within a window, else shutdown — the
  classic pad abort); release-timing dispersion across latches produces
  realistic tip-off, feeding clearance screening.
- **Swing-arm/strongback clearance volumes:** declared keep-out geometry with
  retract kinematics on sequencer events.

### 3.5 Pad geometry & liftoff clearance screening

A CLVTOPS-class deliverable on existing machinery: tower/umbilical keep-out
volumes + the rigid-body kernel + ground-wind profiles (`08`) + thrust
misalignment/mistiming dispersions (`05`, `11`) → Monte Carlo drift envelopes
and minimum-clearance statistics during the first vehicle-lengths of flight.
Pure forward screening; the deliverable is a clearance report (probability of
infringement vs declared volumes, with Wilks/Clopper-Pearson sizing from
`11`).

### 3.6 Hold/recycle/scrub graph & consumables

Recycle edges declare the re-entry step and the *consumable cost* of taking
them (cryo drainback/re-chill from `17`, pressurant budget, battery
state-of-charge from `23`, crew/duration limits as declared scalars). The
engine answers: is a recycle to T-X feasible within remaining consumables and
window? Scrub executes the drainback/safing sequence. This is bookkeeping +
graph evaluation — deterministic and testable.

### 3.7 Rehearsal harness

Scripted rehearsal documents (wet-dress: count to T-N and safe; mission-dress:
full count + flight on the SIL boundary) binding: the sequencer, the LCC
engine, fault injections (`10`'s ports — sensor faults, link dropouts from
`20`, GSE failures from §3.4), and expected-outcome assertions. Output: a
deterministic rehearsal report (steps, verdicts, aborts, timeline deltas)
through the `openbmp-sil` checks/`21` run-record machinery. This is the
capability-parity translation of firing-room training and integrated
sims — exercised by CI, not by an ops team.

### 3.8 Fidelity tiers

- **T0 (current):** mission FSM pad states; liftoff as initial condition.
- **T1:** sequencer + LCC engine, synthetic countdown, auto-abort to safing —
  `validated-toy`.
- **T2:** umbilical/T-0 handover + hold-down release window + thrust-at-
  release gate — `validated-toy`.
- **T3:** clearance screening MC + recycle/consumables graph — `validated-toy`
  (analytic anchors) / `checked`.
- **T4:** rehearsal harness + report card + fault campaigns — `validated-toy`.

---

## 4. Invariant preservation

- **Determinism:** sequencer/LCC evaluation on fixed cadences in registration
  order; count clock derived from sim clock; dispersions only via
  `DeterministicRng` domains; no wall-clock, no unordered maps in evaluation
  paths.
- **Byte-stable default:** all of it behind `[ground]`/`[countdown]` scenario
  blocks; canonical goldens untouched.
- **FC portability:** the FC never imports ground models; ground requests
  reach it as bus commands/channels through existing seams; the mission HSM
  remains the *vehicle's* authority (ground proposes, FC disposes).
- **Provenance:** countdown structure anchored to published timelines is
  digitized into synthetic procedure files with `provenance.md`; LCC baseline
  bands are pinned artifacts; no operational procedure documents are
  committed.
- **Fail-closed:** unknown steps/criteria/channels, missing baselines, and
  infeasible schedules are load-time errors.
- **Labels:** `validated-toy` ceiling for the engines; clearance statistics
  inherit `11`'s convergence reporting.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| Sequencer DAG: topological determinism, hold/recycle re-entry | property | T1 | identical event streams across runs/thread counts |
| LCC predicate semantics (limits, persistence, hysteresis, latching) | unit/property | T1 | exact (boolean semantics) |
| Out-of-family detector on synthetic ensembles | analytic | T1 | detection at declared z within declared false-alarm rate (table) |
| Auto-abort through T-0: engine-start underperformance → shutdown + safing | scenario (with `05` T2) | T2 | abort latency ≤ declared bound; deterministic |
| Power/data handover continuity | scenario (with `23`/`20` stubs) | T2 | no channel gap > declared; energy bookkeeping closes |
| Hold-down release: zero-dispersion release reproduces unconstrained liftoff | regression | T2 | byte-identical to gold after release |
| Tip-off from dispersed latch timing | analytic bound | T2/T3 | small-dispersion rate matches closed-form impulse asymmetry < 5% |
| Tower-clearance MC vs analytic drift bound (constant wind, no control) | analytic | T3 | mean drift < 1%; envelope contains analytic case |
| Recycle feasibility: consumables conservation across hold→recycle→resume | scenario (with `17`) | T3 | closure audit < 1e-9 rel |
| Rehearsal report determinism + golden fixture | golden | T4 | byte-identical |


---

## 7. Dependencies on other parity docs

- `17` — loading/conditioning timeline as sequencer-driven states; recycle
  consumables (drainback, re-chill, pressurant).
- `05` — engine-start transient (T2) for the thrust-at-release gate and
  start-abort class.
- `14` — latch primitives for hold-down; contact substrate if pad dynamics
  are modeled beyond release impulse.
- `23` — internal-power handover, battery state in recycle feasibility.
- `20` — umbilical→RF telemetry routing switch; link-continuity LCC
  predicate.
- `19` — day-of-launch wind/loads go-no-go enters the LCC as one criterion
  family; `19` owns its computation.
- `10` — fault-injection ports and SIL boundary for rehearsals.
- `11`/`12` — MC machinery for clearance screening and dispersion campaigns;
  determinism substrate.
- `21` — rehearsal/clearance reports as run records; LCC baseline bands.
- `06` — abort-to-SAFE execution on the FC side.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| Published countdown timelines (Falcon 9, Electron PUG, Shuttle press kits) | structural anchors, digitized + pinned | public documents |
| NASA SCCS public articles + fact sheets | reference architecture description | public |
| CLVTOPS/Ares I-X clearance NTRS papers | method + validation anchor for §3.5 | public NTRS |
| NASA cFS sample ground tooling (NOS3 ground segment patterns) | architecture reference for ground/vehicle split | NASA open source |
| Yamcs / OpenC3 COSMOS | named host-side ground-software ecosystem (adapters live in `21`) | AGPL-3.0 / tri-license |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-18.1 — `openbmp-ground` skeleton + countdown autosequencer

- **title:** L7 ground crate with the declarative countdown DAG: steps, anchors, windows, prerequisites, built-in holds, deterministic count clock.
- **goal:** A countdown becomes a first-class simulated artifact: declared
  steps evaluated against sim state on a deterministic cadence, with hold
  semantics that separate count time from physics time.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-ground` (L7)** — first commit is the skeleton +
  placement justification (§3.1); no `openbmp-fc` edge ever.
- **touched:** `crates/openbmp-ground/**(new)`, `crates/openbmp-runner`
  (wiring seam), `crates/openbmp-scenario` (`[countdown]` schema), telemetry
  channels (gated).
- **approach:** §3.2; TOML DAG, registration-order evaluation, fixed cadence,
  fail-closed parse.
- **acceptance:**
  - `[countdown]` off by default; canonical goldens byte-identical
  - DAG property tests: deterministic event stream, hold re-entry exactness,
    cycle detection fails closed at load
  - count-clock vs sim-clock separation verified in a hold scenario
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — scheduling + veto logic only.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** synthetic procedures; no site-certified countdown.

### WP-18.2 — Launch-commit-criteria engine

- **title:** Declarative LCC predicates (limits, rates, persistence, hysteresis, dependencies) with latched verdicts and fail-closed semantics.
- **goal:** Go/no-go stops being implicit: every criterion is a named,
  testable predicate with deterministic latching, and the sequencer consumes
  criterion ids as prerequisites.
- **fidelity_tier:** T1
- **depends_on:** [WP-18.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-ground` (lcc module), scenario schema
  (`[lcc]`), `crates/openbmp-sil/src/checks.rs` reuse for verdict records.
- **approach:** §3.3 predicate forms minus out-of-family (WP-18.3); monotone
  latching; unknown channels red + load-time error where detectable.
- **acceptance:**
  - off by default; goldens byte-identical
  - predicate semantics exact under unit/property tests (persistence windows,
    hysteresis, dependency closure)
  - unknown channel/criterion fails closed (load error)
  - verdict report golden fixture byte-stable
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — veto-only surface.
- **est_effort:** 2 weeks
- **parity_ceiling:** criteria values are synthetic; no certified LCC set.

### WP-18.3 — Out-of-family detection + baseline bands

- **title:** Robust z-score (median/MAD) channel screening against provenance-pinned ensemble baseline bands, as an LCC predicate family.
- **goal:** The auto-abort class that catches "legal but anomalous" readings:
  channels are screened against what *this scenario family normally does*,
  not just static limits — the documented reference-practice pattern.
- **fidelity_tier:** T1
- **depends_on:** [WP-18.2]
- **new_crates:** none.
- **touched:** `crates/openbmp-ground` (oof module), baseline-band artifact
  schema (shared with doc `21`), `data/` fixture bands + `provenance.md`.
- **approach:** §3.3; bands generated offline from N seeded runs (doc `21`
  machinery or a self-contained generator at this WP), pinned, versioned;
  missing band ⇒ criterion red.
- **acceptance:**
  - off by default; goldens byte-identical
  - detection/false-alarm rates on synthetic ensembles match analytic
    expectations (tolerance table)
  - band artifact is SHA-pinned and fails closed on mismatch
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** bands from simulation ensembles, never fleet data.

### WP-18.4 — Umbilical/T-0 handover + GSE event models

- **title:** Ground power/data/fluid path models with declared disconnect events, handover coupling to EPS/telemetry/cryo seams, disconnect-failure faults.
- **goal:** T-0 becomes a modeled interface: internal-power transfer, data
  routing switch, purge/supply termination, and strongback/swing-arm retract
  events — each observable, dispersible, and fault-injectable.
- **fidelity_tier:** T2
- **depends_on:** [WP-18.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-ground` (gse module), interfaces to `23`
  (power source switch), `20` (routing), `17` (supply termination) with
  stubs where those docs haven't landed, fault types in
  `crates/openbmp-bridge/src/fault.rs` pattern.
- **approach:** §3.4; event-driven boundary models; declared dispersions via
  `DeterministicRng`.
- **acceptance:**
  - off by default; goldens byte-identical
  - handover continuity case: no unaccounted channel/power gap (audit)
  - disconnect-failure fault produces the declared abort path
    deterministically
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** declared boundary behavior; no facility hardware
  modeling.

### WP-18.5 — Hold-down release + thrust-at-release gate

- **title:** Latch-based hold-down with dispersed release window and per-engine thrust-proof gating (pad-abort on underperformance).
- **goal:** Liftoff becomes a dynamic event: engines start against latches,
  prove chamber pressure within the window, and release with realistic
  timing dispersion — or shut down on the pad. Tip-off emerges physically.
- **fidelity_tier:** T2
- **depends_on:** [WP-18.4; cross-doc: WP-14.2 (latch primitive), WP-05.2-b
  (transient start) or a declared thrust-ramp stub]
- **new_crates:** none.
- **touched:** `crates/openbmp-ground` (release module), `crates/openbmp-runner`
  force wiring, scenario schema, telemetry.
- **approach:** §3.4; latched constraint until release predicate; per-latch
  release times drawn deterministically; gate = all engines ≥ threshold
  within window else shutdown command via FC path.
- **acceptance:**
  - off by default; goldens byte-identical
  - zero-dispersion release regression: byte-identical post-release
    trajectory vs unconstrained liftoff
  - underperforming-engine case shuts down on the pad (no liftoff;
    deterministic report)
  - small-dispersion tip-off rate matches closed-form impulse-asymmetry
    estimate < 5%
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — fail-closed launch gating.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** latch mechanics at primitive tier; no real release
  hardware data.

### WP-18.6 — Pad geometry + tower-clearance Monte Carlo

- **title:** Keep-out volumes (tower, arms, strongback) and CLVTOPS-class drift/clearance screening over wind/thrust/release dispersions.
- **goal:** The standing liftoff-clearance analysis product: minimum-clearance
  statistics and infringement probability with convergence reporting, on the
  existing rigid-body kernel.
- **fidelity_tier:** T3
- **depends_on:** [WP-18.5; cross-doc: WP-11.0/WP-11.1 (MC + sizing)]
- **new_crates:** none (consumes `openbmp-mc`).
- **touched:** `crates/openbmp-ground` (clearance module), scenario schema
  (`[pad.geometry]`), report artifact.
- **approach:** §3.5; signed-distance checks against declared volumes per
  step; per-sample seeds via `DeterministicRng::for_mc_sample`;
  Wilks/Clopper-Pearson sizing.
- **acceptance:**
  - off by default; goldens byte-identical
  - analytic constant-wind drift case: mean < 1% error; envelope contains
    closed form
  - clearance report deterministic across worker counts (order-independent
    reduction from `12`)
  - all `13` §2 gates green
- **validation_label:** `checked` → `validated-toy`
- **dual_use_note:** far from line — forward screening of own liftoff.
- **est_effort:** 2 weeks
- **parity_ceiling:** method-replica of CLVTOPS practice; no flight-validated
  drift database.

### WP-18.7 — Hold/recycle/scrub graph + consumables feasibility

- **title:** Recycle edges with consumable costs (cryo, pressurant, power) and feasibility evaluation; scrub/drainback sequence.
- **goal:** "Can we recycle to T-4 and still fly today?" becomes a computed,
  deterministic answer with a conservation audit — the operational logic
  that couples ground and vehicle resource states.
- **fidelity_tier:** T3
- **depends_on:** [WP-18.1; cross-doc: WP-17.7 (loading timeline), WP-23.1
  (battery state) or declared stubs]
- **new_crates:** none.
- **touched:** `crates/openbmp-ground` (recycle module), scenario schema,
  report.
- **approach:** §3.6; graph evaluation over declared costs + live resource
  states; scrub path drives `17` drainback.
- **acceptance:**
  - off by default; goldens byte-identical
  - hold→recycle→resume conserves consumables (audit < 1e-9 rel)
  - infeasible recycle (insufficient consumables) fails closed with the
    declared verdict
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** declared cost models; no facility scheduling.

### WP-18.8 — Rehearsal harness + report card

- **title:** Scripted WDR/MDR rehearsal documents binding sequencer, LCC, fault injections, and expected outcomes into deterministic reports.
- **goal:** The firing-room-training analog an open repo can have: repeatable
  fault-seeded countdown rehearsals run in CI, with verdicts that catch
  regressions in the whole ground/vehicle stack.
- **fidelity_tier:** T4
- **depends_on:** [WP-18.2, WP-18.4; cross-doc: WP-10.2 (fault ports)]
- **new_crates:** none.
- **touched:** `crates/openbmp-ground` (rehearsal module), rehearsal schema,
  `crates/openbmp-sil` report reuse, scenario fixtures under
  `tests/fixtures/`.
- **approach:** §3.7; rehearsal document = scripted faults + expected
  verdicts; report through sil/`21` machinery.
- **acceptance:**
  - off by default; goldens byte-identical
  - WDR fixture (count to T-N, safe) and MDR fixture (count + SIL flight)
    produce byte-stable reports
  - injected GSE/sensor/link faults produce the declared abort/hold verdicts
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — exercises protective logic.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** rehearses the sim, not an ops team; no training-
  effectiveness claims.

---

## 10. References

- NASA SCCS certification and fact-sheet articles (KSC, public): GSE +
  vehicle emulation for launch-team training and software test.
- Shuttle/Falcon 9/Electron published countdown timelines (press kits,
  Electron Payload User Guide 8.0).
- SpaceX public statements on T-0 auto-abort ("out of family data during
  engine power check", March 2020) and Starship vehicle+ground simulation
  scope (job-posting text).
- CLVTOPS liftoff/separation proximity analyses and Ares I-X validation
  (NTRS 20205010276, NTRS 20110014623).
- SLS SIL / ARTEMIS / MAESTRO public articles (MSFC) — automated test
  conductor pattern.
- NASA NOS3 (ground/vehicle split reference architecture; open source).
- Wilks tolerance-limit sizing and Clopper-Pearson intervals — per doc `11`
  references.
