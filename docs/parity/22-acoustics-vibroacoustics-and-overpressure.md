# Acoustics, Vibroacoustic Environments & Ignition Overpressure

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`02-structural-dynamics-loads-slosh-pogo.md` (the loads consumer),
`15-plume-environments-and-supersonic-retropropulsion.md` (the source
supplier).

> One-line scope: the dynamic-environments discipline the series has so far
> deferred from three directions — SP-8072 distributed-source liftoff
> acoustics, ignition-overpressure engineering models, ascent fluctuating-
> pressure environments, fairing internal acoustics, empirical
> vibroacoustic response transfer, and P95/50 maximum-expected-environment
> derivation feeding the `02` loads stack — all engineering-correlation
> tier, solver-consuming for anything higher.

Three existing docs explicitly point here: `02` lists "acoustic environment
from plume/separation transient" as adjacent-not-covered; `05` lists
"acoustic environment (plume/separation-driven liftoff acoustics)" as out of
scope; `03` lists "combustion-driven acoustic environment" out of scope.
Reference orgs staff this as a named team, and the canonical open method
(NASA SP-8072 distributed source method) has **no maintained open
implementation** — a verified first-open-tool opportunity.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **NASA** — SP-8072 Distributed Source Method (DSM-1/DSM-2) remains the
  backbone of liftoff acoustic prediction, updated for Ares/SLS with
  RSRM-derived core-length and directivity modifications
  (NTRS 20090023640); ignition overpressure is treated as a paired design
  environment (engine-start transient + duct geometry); vibroacoustic
  response prediction ran on VAPEPS (SEA + flight/ground database, JPL-
  managed, legacy); environment statistics follow NASA-STD-7001 (P95/50
  maximum expected flight level).
- **SpaceX** — a dedicated environments team "predicting and measuring
  vibroacoustic loads due to aeroacoustic environments (acoustics, buffet,
  over pressure, unsteady flow) as well as engine dynamics" (official
  posting text).
- **Blue Origin / Relativity** — staffed loads-and-dynamic-environments
  roles producing liftoff/aero/engine forcing functions; flight-data
  anchoring of vibe/shock/acoustic models is an explicit duty.
- **Rocket Lab** — published payload acoustic maximum predicted environment
  (122.9 dB OASPL) and offers mission-specific environment refinement from
  flight data (Payload User Guide).

### 1.2 Parity target (capability, within posture)

1. **Liftoff acoustics (DSM)** — SP-8072 DSM-2-class prediction: total
   acoustic power from plume mechanical power with efficiency, sources
   distributed along the (deflected) plume with per-source spectra
   (Strouhal-scaled allocation), directivity indices, pad geometry path
   effects (deflector/duct/free-field), water-suppression knockdown decks,
   and receiver spectra (1/3-octave SPL) at declared vehicle stations.
2. **Ignition overpressure (IOP)** — engineering correlation for the
   ignition pulse (source strength from the engine-start mass-flow/chamber
   ramp of `05` T2, duct/trench geometry terms, suppression knockdown),
   producing pulse amplitude/duration envelopes at vehicle stations.
3. **Ascent fluctuating pressure** — empirical OASPL and spectra vs flight
   condition (attached turbulent boundary layer, separated flow, and
   shock-oscillation zone classes keyed to `03` geometry/regimes),
   protuberance class factors; the in-flight aeroacoustic environment.
4. **Fairing internal acoustics** — noise-reduction (insertion-loss) curve
   decks with fill-factor correction, producing internal payload-zone
   spectra from external envelopes.
5. **Vibroacoustic response transfer** — empirical SPL→acceleration-PSD
   transfer (Franken-class scaling plus declared single-junction SEA-lite
   with surface mass/area parameters), zone-level random-vibe environments.
6. **Environment statistics** — P95/50-class maximum-expected-environment
   derivation over MC dispersions (`11` machinery), spectra enveloping,
   and handoff of (a) modal forcing PSDs to `02` and (b) environment tables
   to scenario consumers.

### 1.3 Parity ceiling (honest boundary)

- **No CAA/LES.** Computational aeroacoustics, jet-noise LES, and coupled
  FEM/BEM vibroacoustics are solver-consumer territory: ingestion path +
  provenance + UQ, never an in-repo solver (`13` §6).
- **No facility-correlated absolute levels.** Reference levels are anchored
  to *published* curves (Saturn/Shuttle/Ares static-fire and liftoff data in
  NTRS); proprietary pad-measurement campaigns cannot be matched. Honest
  labels: `validated-toy` for method mechanics, `research` only against
  published curves with stated residuals.
- **No certified component environments.** Derived environments are
  simulation products with documented dispersion bases, never qualification
  levels; NASA-STD-7001 is followed as a *method* reference.
- **IOP ceiling.** IOP is notoriously facility-specific; the model is an
  engineering envelope (correlation + knockdown decks) with wide declared
  uncertainty, not a transient CFD reconstruction.
- **Response ceiling.** SEA-lite/Franken transfer predicts zone-level PSDs;
  component-level response and force-limited vibration practice are named,
  not designed.

---

## 2. Current state in source

- Nothing acoustic exists in the workspace: no source model, no spectra
  machinery, no SPL/PSD types.
- `15` (designed) supplies the inputs DSM needs: per-engine `PlumeState`
  (exit velocity/diameter, NPR, thrust, cluster geometry) and the
  deflected-plume description; its WP ladder explicitly leaves acoustics
  out (deferral noted in `15`'s adjacent-gaps list).
- `05` T2 (designed) supplies the engine-start transient that drives IOP
  source strength.
- `02` (designed) consumes random-vibe/modal forcing; its CLA event library
  is the structural side of the same coin.
- `03` (designed) supplies flow-regime classification (attached/separated/
  shock zones) for ascent fluctuating pressure.
- `crates/openbmp-physics/src/statistics.rs` and `openbmp-mc` provide the
  statistical substrate for P95/50 derivation.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-acoustics` (L2): consumes `15` plume state, `05` start transients,
`03` regime data, `08` ambient properties; produces station spectra and
environment tables; feeds `02` forcing and scenario consumers. No
`openbmp-fc` edge ever (environments are analysis products, not flight
inputs).

### 3.2 Spectral machinery (T1 substrate)

One-third-octave band grid (declared band set), `Spl` and `Psd` types with
unit-checked constructors, band integration/summation with locked operand
order, OASPL reduction, and spectra algebra (level addition, transfer
application, enveloping). All downstream models speak these types;
property-tested exactness (Parseval-style closure on synthetic spectra).

### 3.3 Liftoff acoustics — distributed source method (T1–T2)

Following SP-8072 DSM-2 with the published Ares-era modifications:

```
W_acoustic = η(ξ) · ½ ṁ V_e²            // acoustic efficiency vs plume class
L_W = 10·log10(W_acoustic / W_ref)
```

- **Source allocation:** the laterally-distributed source line follows the
  plume centerline (free or deflected per pad geometry from `18`'s
  `[pad.geometry]`); per-source acoustic-power allocation and
  Strouhal-scaled spectra (`St = f·D_e/V_e`) from the published DSM
  allocation curves, digitized as provenance-pinned decks; core-length
  parameterization with the published modification factors.
- **Directivity:** per-band directivity-index decks (angle from plume axis),
  applied per source-receiver pair.
- **Propagation:** spherical spreading + declared air-absorption deck;
  path-length per source-receiver from pad/vehicle geometry as the vehicle
  rises (height-stepped evaluation produces the classic liftoff SPL-vs-time
  signature).
- **Water suppression:** knockdown decks (ΔdB vs band vs water-flow class)
  from published suppression studies; declared, uncertainty-tagged.
- **Receivers:** declared vehicle stations (external skin zones); output =
  1/3-octave SPL spectra + OASPL vs time.

### 3.4 Ignition overpressure (T2)

Engineering pulse model: source impulse proportional to the rate of
combustion-gas generation during start (`d(ṁ c*)/dt` class scaling from the
`05` T2 transient), duct/trench volume and exit-area terms, distance decay,
and suppression knockdown — yielding per-station peak overpressure and
duration envelopes with wide declared uncertainty. Anchored qualitatively to
the published Shuttle IOP history (pre/post STS-1 suppression change) as a
trend case.

### 3.5 Ascent fluctuating-pressure environments (T3)

Zone-classified empirical model: for each declared vehicle zone and flight
condition (q, M from the trajectory), select regime — attached TBL,
separated/reattaching, shock-oscillation — via `03` geometry/regime flags;
compute OASPL from the published empirical levels for that class (q-scaled),
with normalized spectra per class (declared decks); protuberance factors as
multipliers. Output: zone external fluctuating-pressure spectra vs time
along any trajectory.

### 3.6 Fairing internal acoustics (T3)

Insertion-loss decks (noise reduction vs band for declared fairing classes,
digitized from published payload-environment literature) + fill-factor
correction (payload volume ratio adjustment) applied to external envelopes;
output = payload-zone internal spectra for comparison against published
user-guide MPE curves (Electron 122.9 dB OASPL class anchors the sanity
check).

### 3.7 Vibroacoustic response + environment statistics (T4)

- **Transfer:** Franken-class empirical SPL→acceleration-PSD scaling
  (surface-mass-normalized), or a declared single-junction SEA-lite (panel
  + cavity with declared loss factors) where parameters exist — both as
  uncertainty-tagged decks; output zone random-vibe PSDs.
- **Statistics:** dispersed runs (`11`) over source/transfer/trajectory
  uncertainty → per-band level distributions → P95/50-class maximum
  expected environment per NASA-STD-7001 method (normal-tolerance-limit on
  levels in dB), plus enveloping rules; deliverables are environment tables
  + spectra artifacts.
- **Handoffs:** modal forcing PSDs shaped for `02`'s transient/CLA machinery
  (declared mapping from zone pressure spectra to modal force PSDs via area/
  mode-shape weighting); scenario-visible environment tables for recovery/
  payload consumers (`16` parachute snatch zones are *not* acoustic — no
  coupling).

### 3.8 Ingestion path (T5)

The `13` §6 pattern for higher fidelity: external CAA/test-derived source or
transfer decks (e.g. user-supplied LES-derived source spectra) enter via
parser + provenance + UQ wrapper + coupling adapter + code-to-code case
against the in-repo tier.

### 3.9 Fidelity tiers

- **T0 (current):** nothing.
- **T1:** spectral machinery + free-field DSM (single engine, undeflected)
  — `validated-toy` (worked-example reproduction).
- **T2:** deflected/duct geometry + clusters + water knockdown + IOP —
  `validated-toy` → `research` (published-curve anchors, honest residuals).
- **T3:** ascent fluctuating pressure + fairing internal — `validated-toy`
  → `research` (published MPE comparisons).
- **T4:** response transfer + P95/50 statistics + `02` handoff —
  `validated-toy` (method) with `method-replica` statistics labeling.
- **T5:** ingestion path — `research`.

---

## 4. Invariant preservation

- **Determinism:** all decks pinned; band math locked-order; dispersions via
  `DeterministicRng`; no FFTs of nondeterministic signals (spectra are
  band-domain by construction).
- **Byte-stable default:** behind `[acoustics]`; goldens untouched.
- **FC portability:** no FC edge; environments never feed guidance.
- **Provenance:** every allocation/directivity/absorption/suppression/
  insertion-loss/transfer deck is digitized from a cited public source with
  `provenance.md` + SHA; the inline-data tripwire covers acoustic decks.
- **Fail-closed:** band-grid mismatches, deck-envelope violations, unknown
  zone/regime classes are errors.
- **Labels:** per §3.9; absolute-level claims never exceed `research`
  against published curves, with residuals stated.

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| Band algebra (sum/integrate/OASPL/envelope) | property/analytic | T1 | < 1e-12 rel (pure arithmetic) |
| SP-8072 worked example reproduction (free-field) | code-to-code (published worked example) | T1 | OASPL < 1 dB; band levels < 2 dB (table) |
| Spreading/absorption vs closed form | analytic | T1 | < 1e-9 rel |
| Liftoff SPL-vs-height signature shape vs published Saturn/Ares curves | public benchmark (digitized) | T2 | peak location + level within declared band (honest residuals, table) |
| Water-suppression knockdown application | unit | T2 | exact deck application |
| IOP trend case (suppression on/off ordering, scaling with start rate) | trend/qualitative | T2 | correct ordering + declared envelope (documented) |
| Ascent OASPL vs q in attached zone | analytic scaling | T3 | follows declared q-scaling < 1% |
| Fairing internal vs published MPE class curve | public benchmark | T3 | within declared band of the published curve (table) |
| P95/50 derivation on synthetic dispersions | statistical | T4 | matches closed-form normal tolerance limit (Wilks-sized, table) |
| `02` forcing handoff energy consistency | internal | T4 | band-power bookkeeping closes < 1e-9 rel |
| Ingested deck round trip + code-to-code vs in-repo tier | per `13` §6 | T5 | documented case with tolerance table |

---

## 7. Dependencies on other parity docs

- `15` — plume state (exit conditions, cluster geometry, deflection) is the
  DSM source input; `15` explicitly deferred acoustics here.
- `05` — engine-start transient drives IOP source strength; engine dynamics
  bands inform zone environments.
- `03` — regime classification (attached/separated/shock) and geometry for
  ascent fluctuating pressure; buffet remains `03`-ceiling territory
  (forcing handoff only).
- `02` — consumer of modal forcing PSDs and environment tables; CLA events
  pair with acoustic-derived random environments.
- `18` — pad geometry (deflector/trench/water class) for liftoff path
  effects.
- `08` — ambient properties for propagation/absorption.
- `11`/`12` — dispersion campaigns for P95/50; deterministic parallel
  evaluation.
- `21` — environment tables/spectra as stored, trended artifacts.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| NASA SP-8072 (DSM-1/2 curves, worked example) | digitized decks + reproduction case | public NASA SP |
| Ares-era DSM modification papers (NTRS 20090023640) | core-length/directivity updates | public NTRS |
| JASA 50-year launch-acoustics review | method survey + anchor curves | published literature |
| NASA-STD-7001 | P95/50 method reference | public standard |
| VAPEPS documentation (NTRS 19900011397) | SEA-lite method reference | public NTRS (legacy) |
| Published Shuttle IOP history (STS-1 era reports) | IOP trend anchor | public NTRS |
| Payload user guides (Electron MPE class curves) | fairing-internal sanity anchors | public documents |
| pyNastran / CalculiX | named structural-side interop (via `02`), not used here | BSD-3 / GPL-2 |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-22.1 — `openbmp-acoustics` skeleton + spectral machinery

- **title:** L2 acoustics crate with 1/3-octave band grid, SPL/PSD types, locked-order band algebra, enveloping ops.
- **goal:** The typed spectral substrate every model in this document and
  every consumer in `02` speaks — exact, unit-checked, and property-tested.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-acoustics` (L2)** — skeleton + placement
  justification first (§3.1); no `openbmp-fc` edge.
- **touched:** `crates/openbmp-acoustics/**(new)`, unit/property tests.
- **approach:** §3.2; declared band set; deterministic reductions.
- **acceptance:**
  - no scenario surface yet; goldens trivially byte-identical
  - band algebra exact (< 1e-12 rel) under property tests
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** far from line — math types.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** band-domain only; no waveform synthesis.

### WP-22.2 — DSM liftoff acoustics (free-field single engine)

- **title:** SP-8072 DSM-2 source allocation, Strouhal-scaled spectra, directivity decks, spreading/absorption, station receivers; worked-example reproduction.
- **goal:** The first open implementation of the canonical liftoff-acoustics
  method, proven against the document's own worked example.
- **fidelity_tier:** T1
- **depends_on:** [WP-22.1; cross-doc: WP-15.1 (PlumeState) or a declared
  exit-condition stub]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (dsm module), `data/` DSM decks +
  `provenance.md`, scenario schema (`[acoustics]`), gated telemetry.
- **approach:** §3.3 free-field path; digitized allocation/directivity
  decks; height-stepped evaluation.
- **acceptance:**
  - off by default; canonical goldens byte-identical
  - SP-8072 worked example: OASPL < 1 dB, band levels < 2 dB (tolerance
    table)
  - spreading/absorption closed forms < 1e-9 rel
  - all `13` §2 gates green
- **validation_label:** `validated-toy` → `research` (worked-example anchor)
- **dual_use_note:** far from line.
- **est_effort:** 3 weeks
- **parity_ceiling:** free-field, single engine; absolute levels carry
  published-curve uncertainty.

### WP-22.3 — Pad geometry effects, clusters, water suppression

- **title:** Deflected-plume source paths, duct/trench terms, multi-engine cluster summation, water-suppression knockdown decks; liftoff signature case.
- **goal:** Realistic pads: the SPL-vs-time liftoff signature with deflector
  and suppression effects, summed correctly over engine clusters.
- **fidelity_tier:** T2
- **depends_on:** [WP-22.2; cross-doc: `18` `[pad.geometry]` or declared
  stub]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (pad module), `data/` suppression
  decks + `provenance.md`, scenario schema.
- **approach:** §3.3 deflected path + published modification factors;
  incoherent cluster summation; knockdown application.
- **acceptance:**
  - off by default; goldens byte-identical
  - cluster summation exact for identical engines (10·log10 N check)
  - liftoff signature peak location/level within declared band of the
    digitized published curve (honest residuals, table)
  - all `13` §2 gates green
- **validation_label:** `research` (published-curve anchor, residuals
  stated)
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** engineering geometry terms; no facility calibration.

### WP-22.4 — Ignition overpressure model

- **title:** Engineering IOP pulse (start-transient scaling + duct terms + suppression knockdown) with per-station envelopes and trend anchoring.
- **goal:** The paired liftoff environment: a wide-banded, honestly
  uncertain IOP envelope that scales correctly with start rate and
  suppression — the design-input shape reference practice uses.
- **fidelity_tier:** T2
- **depends_on:** [WP-22.3; cross-doc: WP-05.2-b (start transient) or
  declared ramp stub]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (iop module), scenario schema,
  telemetry.
- **approach:** §3.4; declared correlation with uncertainty band; trend
  validation.
- **acceptance:**
  - off by default; goldens byte-identical
  - scaling property: pulse amplitude monotone in start rate; suppression
    ordering correct (pre/post-STS-1 pattern)
  - envelope uncertainty band declared in the tolerance table (no point
    claim)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** facility-specific IOP physics cannot be matched;
  envelope-with-uncertainty only.

### WP-22.5 — Ascent fluctuating pressure + fairing internal

- **title:** Zone/regime-classified empirical OASPL + spectra vs (q, M); insertion-loss + fill-factor fairing internal environments; MPE sanity anchor.
- **goal:** The in-flight acoustic environment over any trajectory, and the
  payload-zone internal levels users actually compare against launch-vehicle
  user guides.
- **fidelity_tier:** T3
- **depends_on:** [WP-22.1; cross-doc: `03` regime flags or declared zone
  classes]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (ascent + fairing modules),
  `data/` class decks + `provenance.md`, scenario schema.
- **approach:** §3.5/§3.6; published class levels/spectra digitized;
  q-scaling; insertion-loss decks.
- **acceptance:**
  - off by default; goldens byte-identical
  - attached-zone OASPL follows declared q-scaling < 1%
  - fairing internal case within declared band of the published MPE class
    curve (table)
  - regime misclassification fails closed (unknown class = error)
  - all `13` §2 gates green
- **validation_label:** `validated-toy` → `research`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** empirical class curves; no CFD/flight-correlated zone
  maps.

### WP-22.6 — Vibroacoustic response + P95/50 environments + `02` handoff

- **title:** Franken-class/SEA-lite SPL→PSD transfer decks, dispersed maximum-expected-environment statistics, modal forcing handoff.
- **goal:** Close the loop to structures: zone random-vibe PSDs and
  P95/50-class environment tables derived with `11`'s machinery, and
  forcing the `02` stack can consume.
- **fidelity_tier:** T4
- **depends_on:** [WP-22.3, WP-22.5; cross-doc: WP-11.1 (DoE/sizing),
  `02` forcing interface]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (response + stats modules),
  `data/` transfer decks + `provenance.md`, `02` interface types, artifacts
  for `21`.
- **approach:** §3.7; uncertainty-tagged transfer decks; normal tolerance
  limits on dB levels; area/mode-shape weighted forcing map.
- **acceptance:**
  - off by default; goldens byte-identical
  - P95/50 derivation matches closed-form tolerance limit on synthetic
    dispersions (Wilks-sized, table)
  - forcing handoff band-power bookkeeping closes < 1e-9 rel
  - environment-table artifact byte-stable, stored via `21`
  - all `13` §2 gates green
- **validation_label:** `validated-toy` (`method-replica` statistics
  labeling)
- **dual_use_note:** far from line — environments feed structural margins.
- **est_effort:** 3 weeks
- **parity_ceiling:** zone-level empirical transfer; no FEM/BEM coupled
  response, no qualification levels.

### WP-22.7 — High-fidelity ingestion path

- **title:** Parser + provenance + UQ wrapper + coupling adapter for externally produced source/transfer decks (CAA/test), with a code-to-code case vs the in-repo tier.
- **goal:** The solver-consumer escape hatch: users with LES/test data plug
  it in without the repo shipping a solver.
- **fidelity_tier:** T5
- **depends_on:** [WP-22.6]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (ingest module), deck schema docs,
  example fixture under `tests/fixtures/`.
- **approach:** `13` §6 pattern end-to-end.
- **acceptance:**
  - off by default; goldens byte-identical
  - SHA-pinned round trip; fails closed on mismatch
  - code-to-code case (synthetic "external" deck vs in-repo tier)
    documented with tolerance table
  - all `13` §2 gates green
- **validation_label:** `research`
- **dual_use_note:** far from line; published-curve provenance discipline
  only.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** ingestion only; external data quality is the user's
  provenance burden.

---

## 10. References

- NASA SP-8072, "Acoustic Loads Generated by the Propulsion System" —
  DSM-1/DSM-2, allocation/directivity curves, worked example.
- Ares I liftoff-acoustics DSM modifications (NTRS 20090023640).
- "Supersonic jet noise from launch vehicles: 50 years since NASA SP-8072"
  (JASA review, 2022).
- NASA-STD-7001 (vibroacoustic test criteria; P95/50 method).
- VAPEPS program documentation (NTRS 19900011397).
- Shuttle ignition-overpressure reports (STS-1 era, NTRS) — suppression
  history trend anchor.
- Published payload user guides (acoustic MPE classes) — fairing-internal
  anchors.
- Franken empirical vibroacoustic transfer method (published literature).
