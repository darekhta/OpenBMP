# Day-of-Launch Winds, I-Load Update & Commit Operations

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`07-trajectory-optimization-and-mission-design.md` (whose terminal-condition
vocabulary lock this document inherits verbatim),
`08-environment-gravity-and-frames.md` (whose statistical atmosphere this
document complements with *measured* profiles).

> One-line scope: the DOLILU-class day-of-launch loop — measured wind-profile
> ingestion and splicing, wind-persistence dispersion, first-stage steering
> (I-load) redesign through the `07` solver stack against flight/orbital
> conditions and structural-load constraints, reduced load-indicator
> screening, a deterministic go/no-go report card, and a launch
> collision-avoidance screening tier.

Doc `11` explicitly placed "operational / predicted wind-aloft / I-load
update workflow (DOLILU-class)" out of its scope; doc `08` owns
climatological atmospheres and statistical dispersions, not measured-profile
operations. This document closes that gap. The boundary: **`08` answers
"what might the atmosphere be"; `19` answers "the balloon just measured
this — redesign, screen, and decide."**

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **Shuttle DOLILU II** flew on 66 consecutive missions: Jimsphere and AMPS
  balloons plus 50 MHz Doppler radar wind profilers feed an iterative
  **3-DOF design simulation** producing pitch/yaw steering I-loads and
  throttle tables (angle-of-attack and sideslip recentering, Martin-Graham
  low-pass wind filtering for persistence); a **6-DOF verification
  simulation** confirms; **42 Structural Load Indicators** — simplified
  surrogates of higher-fidelity structural limits over Mach 0.6–2.2 — are
  screened with RSS-combined wind-persistence/system/gust dispersions; an
  independent team re-verifies; I-loads are uplinked about L-90 min with the
  design balloon at L-4:50 and a final loads go/no-go at L-0:30
  (NTRS 20110003654).
- **SLS/Artemis** continues the practice: spliced low-altitude balloon +
  Doppler radar wind profiler + lidar + Earth-GRAM-above profiles feed
  trajectory assessment verifying the I-loads (roll/pitch/yaw/throttle vs
  altitude tables) are safe to fly (Artemis I DOLILU summary,
  NTRS 20230004429); **CHANGO** computes day-of-launch boost-stage I-loads
  and targets (NTRS 20190000719).
- **SpaceX** releases balloons through the count and computes a quantitative
  loads ratio against vehicle capability — the 2015 DSCOVR scrub was called
  on a measured 1.5× loads-limit exceedance. Public evidence shows
  assess-and-commit; a steering rebias is not publicly documented — the
  parity target is therefore the NASA-documented full loop, with
  assess-and-commit as a strict subset.
- **Rocket Lab / ULA** document balloon-driven upper-wind commit processes
  and day-of-launch flight design respectively.
- **Launch collision avoidance (COLA):** NASA CARA-class screening of the
  powered trajectory against the catalog closes launch-window slots — a
  standing day-of-launch product (NTRS 20240007836).

### 1.2 Parity target (capability, within posture)

1. **Measured-wind ingestion** — balloon/profiler/lidar profile schemas with
   quality control (gap/spike/shear checks, fail-closed), deterministic
   splicing rules between sources and altitude bands, and a synthetic
   balloon-family generator for in-repo testing.
2. **Persistence & knockdowns** — wind-change-over-lead-time dispersion
   (filtered-profile + persistence-statistics model), RSS combination of
   persistence/system/gust contributions into screening margins.
3. **I-load redesign** — first-stage pitch/yaw steering-table regeneration
   through the existing `07` machinery (3-DOF design pass on the point-mass
   path; objectives and boundary conditions drawn *only* from the `07`
   terminal-condition vocabulary, plus structural-margin objectives), and
   throttle-bucket placement.
4. **Verification pass** — full 6-DOF FC-in-the-loop run on the updated
   I-loads through the normal runner stack.
5. **Load-indicator screening** — a reduced, fast-to-evaluate indicator set
   (q·α/q·β envelopes, station bending-moment surrogates from `02`/`03`)
   evaluated over the Mach window with dispersion knockdowns; a
   deterministic go/no-go report card with margins.
6. **Launch COLA tier** — forward screening of the planned trajectory
   against declared catalog states for window-slot closure.

### 1.3 Parity ceiling (honest boundary)

- **No operational meteorological infrastructure.** Real DOLILU consumes
  range-operated balloon/profiler networks in real time. Open substitute:
  synthetic profile families (provenance-pinned generators) in the repo;
  *real* sounding data (e.g. public radiosonde archives) only through the
  LOCAL workflow of `docs/external-telemetry-validation.md` — never
  committed.
- **No certified loads database.** The 42-indicator Shuttle set was derived
  from the vehicle's certified loads cycle. Open substitute: indicator
  surrogates derived from this repo's own `02`/`03` models, with the
  derivation documented and the surrogate-vs-full correlation reported.
- **No range/airspace interfaces.** Balloon-release coordination, range
  displays, and console procedures are operational practice, named not
  designed.
- **Persistence statistics are method-replicas.** Real persistence models are
  fit to thousands of site soundings; the in-repo model is the documented
  *method* (filtered profiles + lead-time variance growth) with synthetic or
  user-LOCAL site statistics, labeled accordingly.

---

## 2. Current state in source

- `crates/openbmp-physics/src/wind/` + `data/wind/` — climatological wind
  (HWM14 quiet-time + disturbance) and constant/layered/gust profiles;
  scenario `[wind]` blocks. No measured-profile ingestion, no QC, no
  splicing.
- `crates/openbmp-trajopt/src/{corrector.rs,iload.rs,target.rs}` — the
  forward-only differential corrector, I-load emission surface, and the
  locked terminal-condition vocabulary (doc `07` extends to the full solver
  ladder). The I-load schema is the natural output container for redesigned
  steering tables.
- `crates/openbmp-fc/src/{tables.rs,guidance.rs,trajectory.rs}` — FC-side
  consumption of I-load tables across the SIL boundary (doc `10`).
- `crates/openbmp-runner/src/{point_mass.rs,rigid_body.rs,wind.rs}` — the
  3-DOF-capable and 6-DOF paths the design/verify passes run on.
- `02`/`03` (designed) — modal loads and aero line-load machinery the
  indicator surrogates reduce.
- Nothing evaluates measured winds, regenerates I-loads, or produces a
  go/no-go artifact today.

---

## 3. Target architecture

### 3.1 Crate boundary

Default placement: a `dol` module in `openbmp-trajopt` (the redesign loop is
an offline driver around existing solver machinery) plus ingestion types in
`openbmp-physics::wind`. If the module outgrows the host, `openbmp-dol`
(L6, offline pipeline) is the fallback — reviewed per `13` §5. Nothing here
is FC-reachable except the emitted I-load artifact, which crosses the
existing `10` boundary like any other I-load.

### 3.2 Measured-wind ingestion & splicing (T1)

```rust
/// One measured wind profile from one source instrument.
pub struct MeasuredWindProfile {
    pub source: WindSource,            // Balloon | DopplerProfiler | Lidar | Synthetic
    pub released_at: Epoch,            // measurement validity anchor
    pub samples: Vec<WindSample>,      // altitude-ordered (z, u, v, quality)
    pub provenance: ProvenanceRef,     // pinned; LOCAL sources marked
}
```

- **Quality control (fail-closed):** monotone-altitude check, gap limits per
  altitude band, spike/shear-rate screens (max |Δwind|/Δz per band),
  staleness limits vs declared lead time. A profile failing QC is rejected
  with a structured error — never silently smoothed.
- **Splicing:** declared band assignments per source (e.g. balloon to its
  burst altitude, profiler over its gate range, climatology above), overlap
  blending with fixed weights, output = one `SplicedWindProfile` with
  per-band source attribution. Deterministic; splice rules live in the
  scenario document.
- **Synthetic balloon families:** a provenance-pinned generator (seeded,
  `DeterministicRng`) producing physically plausible profile ensembles
  (layered shears + gusts consistent with `08`'s dispersion shapes) for
  tests, CI, and the V&V cases.
- **Filtering:** Martin-Graham-style low-pass on the spliced profile (the
  documented DOLILU conditioning step) with declared cutoff; both raw and
  filtered profiles are retained in the run record.

### 3.3 Persistence & knockdown model (T4)

Wind-change-over-lead-time dispersion: `σ_Δw(z, τ)` grows with lead time τ
between the design profile and flight. In-repo model: band-wise variance
growth curves (declared/synthetic site statistics deck, provenance-pinned;
user-LOCAL replacements documented) + gust allowance + system dispersions
(thrust/mass/aero from `11` budgets), combined RSS per indicator:

```
margin_i = limit_i − [ response_i(spliced wind)
           + RSS(persistence_i(τ), gust_i, system_i) ]
```

The knockdown structure mirrors the documented DOLILU combination; its
statistics earn `method-replica` labeling per `08`'s convention.

### 3.4 I-load redesign loop (T2)

A 3-DOF design pass on the point-mass path: regenerate the pitch/yaw
steering tables (and optional throttle-bucket placement) flying *through the
spliced measured wind*, subject to:

- **Boundary conditions:** drawn exclusively from the `07`
  `TerminalCondition` vocabulary (staging-state or insertion-condition
  targets). **This document adds no new terminal-condition types** — that
  closure is enforced by a tripwire (WP-19.2).
- **Objectives:** recenter α/β histories about declared references in the
  load-critical Mach window (the DOLILU objective), maximize the minimum
  screened load margin, and hold insertion dispersion within the declared
  box. Objectives are vehicle-intrinsic and load-structural only.
- **Solver:** the `07` ladder as available — T0 corrector for table
  perturbation, T2/T3 collocation for the full redesign; deterministic
  settings per `12`.

Output: an `ILoadUpdate` artifact (the existing `iload.rs` schema +
update metadata: design profile hash, solver provenance, margins) — the
uplink analog.

### 3.5 Verification pass & report card (T3)

The redesigned I-loads run through the full 6-DOF FC-in-the-loop stack
(normal runner + `10` SIL when configured). The **report card** assembles:

- per-indicator margins (design profile + knockdowns),
- insertion-condition residuals from the verification run,
- QC/splice attribution and staleness,
- a latched go/no-go verdict with the binding indicator named.

The report is a deterministic artifact (consumed by `18`'s LCC as one
criterion family and by `21` as a run record).

### 3.6 Load-indicator screening set (T3)

A reduced indicator vector evaluated per Mach gate without full transient
loads recovery: q·α/q·β envelope points, station bending-moment surrogates
(linear combinations of q·α, q·β, gimbal deflection, and modal amplitudes
fitted offline against `02`'s full recovery on a dispersion matrix), and
gimbal-authority margin. The fit (indicator vs full-model) ships with its
correlation/tolerance table; indicators are re-fit whenever `02`/`03` decks
change (drift gate in CI).

### 3.7 Launch COLA tier (T5)

Forward screening of the planned powered + early-orbit trajectory against
declared catalog object states: closest-approach distance/probability per
launch-window slot (the catalog is synthetic in-repo; real catalogs are
LOCAL-only). Output: per-slot clear/blocked table. Screening-only — no
maneuver design surface exists here.

### 3.8 Fidelity tiers

- **T0 (current):** climatological/declared winds only.
- **T1:** ingestion + QC + splicing + synthetic families — `validated-toy`.
- **T2:** 3-DOF redesign + I-load artifact — `validated-toy`.
- **T3:** 6-DOF verification + indicator screening + report card —
  `validated-toy` → `research` (method anchored to the published DOLILU
  papers).
- **T4:** persistence/knockdown statistics — `method-replica` labeling.
- **T5:** COLA screening — `checked`.

---

## 4. Invariant preservation

- **Determinism:** ingestion/splice/filter are pure functions of pinned
  inputs; redesign uses `07`'s deterministic solver settings; report
  generation is canonical-ordered.
- **Byte-stable default:** everything behind `[day_of_launch]`; goldens
  untouched.
- **FC portability:** only the I-load artifact crosses to the FC, through the
  existing `10` path; the FC never sees wind profiles or the design loop.
- **Provenance:** profile decks, persistence statistics, indicator fits, and
  catalogs are pinned with `provenance.md`; real soundings/catalogs are
  LOCAL-only per `docs/external-telemetry-validation.md`.
- **Fail-closed:** QC rejection, stale profiles, missing bands, indicator-fit
  drift beyond tolerance, and unknown schema fields are errors.
- **Labels:** per §3.8; statistics claims carry `method-replica` honesty.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| QC screens (gaps/spikes/shear/staleness) | property | T1 | exact semantics; rejected profiles never reach splice |
| Splice determinism + band attribution | unit/golden | T1 | byte-stable artifact |
| Martin-Graham filter response | analytic | T1 | magnitude response matches closed form < 1e-6 |
| Redesign effectiveness: synthetic wind family, redesigned vs stale I-loads | statistical (MC via `11`) | T2 | redesigned min-margin ≥ stale min-margin in ≥ declared fraction of samples (table) |
| α/β recentering in the load window | scenario | T2 | mean α within declared band of reference (table) |
| Insertion residuals after redesign | scenario | T2/T3 | within declared insertion box |
| Indicator surrogate vs full `02` recovery on dispersion matrix | code-to-code (internal) | T3 | correlation ≥ 0.95; max unconservative error < declared bound (table); drift gate |
| Report card golden + go/no-go latching | golden/property | T3 | byte-stable; monotone latching |
| Persistence variance growth reproduces deck statistics | statistical | T4 | within Wilks-sized bounds |
| COLA vs closed-form conjunction geometry (two-body synthetic) | analytic | T5 | miss distance < 1e-6 rel |

The published DOLILU papers serve as *method* anchors (workflow-shape
conformance is documented, not numerically claimed).

---

## 7. Dependencies on other parity docs

- `07` — the solver ladder, I-load schema, and the terminal-condition
  vocabulary lock this document inherits; redesign quality scales with the
  available tier (corrector-only at minimum).
- `08` — climatology for splice-above bands; dispersion shapes for synthetic
  families; frames/epochs.
- `02`/`03` — full loads/aero machinery the indicator surrogates are fitted
  against; q·α infrastructure.
- `06` — the FC autopilot consuming updated I-loads in the verification
  pass; load-relief interaction documented (redesign assumes the flown
  load-relief configuration).
- `10` — I-load transfer across the SIL boundary; verification-in-SIL mode.
- `11`/`12` — dispersion campaigns, Wilks sizing, deterministic parallel
  evaluation of the screening matrix.
- `18` — consumes the go/no-go card as an LCC criterion family; supplies the
  countdown timeline (design/verify/uplink anchors).
- `21` — report-card storage, baseline bands, trend tracking of margins
  across runs.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| DOLILU II operations paper (NTRS 20110003654) | method anchor (workflow, filtering, SLI structure) | public NTRS |
| Artemis I DOLILU summary (NTRS 20230004429) | splice-source structure anchor | public NTRS |
| CHANGO (NTRS 20190000719) | day-of-launch I-load tool reference | public NTRS |
| Public radiosonde archives (IGRA-class) | LOCAL-only realism checks, never committed | public data, LOCAL workflow |
| CARA / launch-COLA assessment (NTRS 20240007836) | screening-method reference | public NTRS |
| sgp4/skyfield-class propagation | code-to-code oracle for COLA geometry (external process) | MIT |
| Earth-GRAM | named reference for splice-above climatology (per `08`, LOCAL ingest) | NASA release |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-19.1 — Measured-wind ingestion, QC, splicing + synthetic families

- **title:** `MeasuredWindProfile`/`SplicedWindProfile` schemas, fail-closed QC screens, declared splice rules, Martin-Graham filtering, seeded synthetic balloon-family generator.
- **goal:** Measured winds become a first-class, quality-controlled,
  deterministic input distinct from climatology — the front door of the
  whole day-of-launch loop, testable without any real data.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none (module per §3.1; boundary review recorded in the PR).
- **touched:** `crates/openbmp-physics/src/wind/` (ingestion types),
  `crates/openbmp-scenario` (`[day_of_launch.wind]` schema), `data/wind/`
  synthetic-family deck + `provenance.md`.
- **approach:** §3.2; QC fail-closed; fixed-weight overlap blending; filter
  with declared cutoff; generator on `DeterministicRng`.
- **acceptance:**
  - off by default; canonical goldens byte-identical
  - QC property tests exact (each screen individually and composed)
  - splice artifact byte-stable with per-band attribution
  - filter magnitude response matches closed form < 1e-6
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** measured winds feed steering-through-weather only; no
  impact-surface coupling (the ballistic-state provenance lock is untouched).
- **est_effort:** 2–3 weeks
- **parity_ceiling:** synthetic/LOCAL profiles; no operational met network.

### WP-19.2 — 3-DOF I-load redesign driver + vocabulary-closure tripwire

- **title:** Redesign loop on the point-mass path through spliced wind: α/β recentering + load-margin objective, `07`-vocabulary boundary conditions, `ILoadUpdate` artifact, closure tripwire.
- **goal:** The heart of DOLILU: steering tables regenerated for the measured
  atmosphere, emitted through the existing I-load schema with full
  provenance — and a proof that this new surface cannot express anything the
  `07` lock forbids.
- **fidelity_tier:** T2
- **depends_on:** [WP-19.1; cross-doc: WP-07.0 (wired corrector) minimum,
  WP-07.2+ preferred]
- **new_crates:** none.
- **touched:** `crates/openbmp-trajopt/src/{iload.rs,target.rs}` (+ new
  `dol` module), `crates/openbmp-cli` (subcommand), tripwire test in
  `crates/openbmp-testkit`.
- **approach:** §3.4; objective = recentering + min-margin maximization;
  solver settings per `12`; artifact carries design-profile hash + solver
  provenance.
- **acceptance:**
  - off by default; goldens byte-identical
  - synthetic-family statistical case: redesigned min-margin ≥ stale
    min-margin in ≥ declared fraction (tolerance table)
  - insertion residuals within the declared box on the reference scenario
  - **tripwire:** DOL surface cannot construct/extend `TerminalCondition`
    (compile-fail or type-closure test, in the same PR)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** active guardrail WP — inherits and re-proves the `07`
  lock at the new surface in the same PR.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** 3-DOF design tier; redesign quality bounded by the
  available `07` tier and open aero/loads decks.

### WP-19.3 — Load-indicator surrogate set + fit pipeline

- **title:** Reduced screening indicators (q·α/q·β gates, station-moment surrogates, gimbal margin) fitted offline against full `02`/`03` recovery, with correlation tables and a drift gate.
- **goal:** Fast, honest screening: the Shuttle-SLI pattern reproduced from
  this repo's own models, with the surrogate quality measured and
  re-checked whenever upstream decks change.
- **fidelity_tier:** T3
- **depends_on:** [WP-19.2; cross-doc: WP-02.1-a/WP-02.2-a (modal loads),
  WP-03.6 (aero UQ) where available — degraded q·α-only mode otherwise]
- **new_crates:** none.
- **touched:** `dol` module (indicators + fit pipeline), `data/` fit decks +
  `provenance.md`, CI drift gate.
- **approach:** §3.6; least-squares fit over a seeded dispersion matrix;
  unconservative-error bound reported; drift gate compares fit hash to deck
  hashes.
- **acceptance:**
  - off by default; goldens byte-identical
  - correlation ≥ 0.95 and max unconservative error < declared bound on the
    held-out matrix (tolerance table)
  - drift gate fails when upstream decks change without re-fit
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** structural-margin quantities only (per `02`'s
  load-recovery scoping).
- **est_effort:** 2–3 weeks
- **parity_ceiling:** surrogates of in-repo models, not of a certified loads
  cycle.

### WP-19.4 — 6-DOF verification pass + go/no-go report card

- **title:** Full FC-in-the-loop verification run on updated I-loads; deterministic report card (margins, residuals, attribution, latched verdict) consumed by `18`'s LCC.
- **goal:** The design/verify split that makes DOLILU trustworthy: a cheap
  designer, an honest full-stack verifier, and one artifact that says go or
  no-go with the binding indicator named.
- **fidelity_tier:** T3
- **depends_on:** [WP-19.2, WP-19.3]
- **new_crates:** none.
- **touched:** `dol` module (verify driver + report), `crates/openbmp-runner`
  invocation seam, report schema shared with `18`/`21`.
- **approach:** §3.5; verification = normal runner (optionally SIL mode per
  `10`); report canonical-ordered; verdict latching monotone.
- **acceptance:**
  - off by default; goldens byte-identical
  - report golden fixture byte-stable; verdict latching property test
  - design-vs-verify consistency: indicator values from the 6-DOF run within
    the surrogate's declared error band (else report flags surrogate breach
    and goes no-go — fail-closed)
  - all `13` §2 gates green
- **validation_label:** `validated-toy` → `research` (method anchored to
  published DOLILU workflow)
- **dual_use_note:** veto-only artifact — the report card can only block a
  launch.
- **est_effort:** 2 weeks
- **parity_ceiling:** method-shape conformance to DOLILU, no operational
  certification.

### WP-19.5 — Persistence statistics + dispersion knockdowns

- **title:** Band-wise wind-change-over-lead-time variance deck, gust and system dispersion RSS combination into screening margins.
- **goal:** Margins account for the wind changing between the design balloon
  and flight — the statistical heart of the commit decision, as a
  documented method-replica.
- **fidelity_tier:** T4
- **depends_on:** [WP-19.3; cross-doc: WP-11.1 (DoE/Wilks)]
- **new_crates:** none.
- **touched:** `dol` module (knockdowns), `data/` persistence deck +
  `provenance.md`, report card fields.
- **approach:** §3.3; declared/synthetic site-statistics deck; RSS
  combination; Wilks-sized statistical verification.
- **acceptance:**
  - off by default; goldens byte-identical
  - synthetic-truth case: realized wind-change variance within Wilks-sized
    bounds of the deck (table)
  - margins monotonically decrease with lead time (property)
  - all `13` §2 gates green
- **validation_label:** `validated-toy` (`method-replica` statistics
  labeling per `08` convention)
- **dual_use_note:** far from line — statistics about own-ascent margins.
- **est_effort:** 2 weeks
- **parity_ceiling:** synthetic site statistics; real site fits are
  user-LOCAL.

### WP-19.6 — Launch COLA screening

- **title:** Forward closest-approach screening of the planned trajectory against declared catalog states; per-slot clear/blocked window table.
- **goal:** The launch-window product every range process includes, as pure
  forward screening over declared inputs.
- **fidelity_tier:** T5
- **depends_on:** [WP-19.1; cross-doc: `08` propagation]
- **new_crates:** none.
- **touched:** `dol` module (cola), synthetic catalog fixture under
  `tests/fixtures/`, report fields, `crates/openbmp-cli` subcommand.
- **approach:** §3.7; declared screening volumes/probability threshold;
  catalog synthetic in-repo, real catalogs LOCAL-only.
- **acceptance:**
  - off by default; goldens byte-identical
  - two-body synthetic conjunction matches closed-form miss distance
    < 1e-6 rel
  - slot table deterministic; real-catalog path documented as LOCAL with no
    committed data (checked by provenance gates)
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** screening/veto only; no maneuver-synthesis surface
  exists here.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** geometric screening; no operational catalog quality or
  covariance realism.

---

## 10. References

- "Day-of-Launch I-Load Update Operations" (Shuttle DOLILU II), NTRS
  20110003654 — workflow, Martin-Graham filtering, SLI screening, IV&V
  structure.
- Artemis I DOLILU Summary (NTRS 20230004429) — splice sources
  (balloon/DRWP/lidar/GRAM), I-load verification practice.
- CHANGO day-of-launch guidance tool (NTRS 20190000719).
- SLS launch windows and day-of-launch processes (NTRS 20205004470).
- Launch collision avoidance efficacy assessment (NTRS 20240007836); CARA
  program documentation (GSFC).
- Public reporting of upper-level-wind commit decisions (DSCOVR 2015 scrub
  loads-ratio statement; Falcon 9 launch weather criteria fact sheet).
- IGRA radiosonde archive (public sounding data; LOCAL-only use).
- Doc `07` references for the solver ladder; doc `02`/`03` references for
  loads/aero machinery.
