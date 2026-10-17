# Acoustics, Vibroacoustic Environments & Ignition Overpressure

**Status:** `experimental` (design intent; WP-22.1 implemented; WP-22.2 partial; WP-22.3 partial; WP-22.4 partial; WP-22.5 partial; WP-22.6 partial).
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

- `crates/openbmp-acoustics` now provides the WP-22.1 L2 spectral substrate:
  standard one-third-octave band construction, validated SPL and PSD types,
  locked-order OASPL/mean-square-pressure reductions, SPL energy addition,
  transfer application, spectral enveloping, flat-in-band PSD/SPL conversion,
  and a committed byte-stable band-grid fixture.
- `openbmp-acoustics::dsm` now provides the first WP-22.2 free-field,
  single-engine DSM substrate: acoustic power from declared exit conditions,
  normalized source-line allocation, deterministic Strouhal-shaped band
  fractions, angle-interpolated directivity decks, spherical spreading,
  scalar or per-band-deck air absorption, station spectra, height-stepped
  liftoff OASPL traces, byte-stable synthetic CSV fixtures, a Schema-1
  provenance-pinned DSM source-allocation/directivity deck parser, and a
  Schema-1 provenance-pinned synthetic air-absorption deck parser, and a
  sibling synthetic named-atmosphere absorption profile deck with representative
  temperature/pressure/humidity metadata and scenario-level
  `absorption_profile_id` selection. It also carries a Schema-1 NASA
  SP-8072 public-source manifest for NTRS citation `19710023719`, including
  the PDF SHA-256 pin and DSM-relevant figure inventory, plus the SP-8072
  Figure 11 core-length fit helper for `x_t / d_e`, a coarse
  provenance-pinned Figure 5 sound-power spectrum curve deck that can be
  converted into normalized one-third-octave band fractions, a coarse
  provenance-pinned Figure 12 axial source-power curve deck that can be
  converted into normalized point-source allocations, a coarse
  provenance-pinned Figure 13 axial spectrum curve deck that can derive
  source-position-dependent source-by-band weights, and a coarse
  provenance-pinned Figure 10 directivity-index curve deck that can be
  converted into per-band DSM directivity rows, plus a coarse
  provenance-pinned Figure 14 apparent-source curve deck that can place one
  source location per band, and a coarse SP-8072 curve tolerance-table
  substrate that compares evaluated DSM spectra against reference spectra and
  renders deterministic pass/fail CSV rows. The shipped Figure 5/10/12/13/14
  curve decks and runner DSM curve loaders also validate their manifest id,
  source PDF hash, and figure id against the pinned public NASA SP-8072 source
  manifest after scenario SHA-pin verification, before being treated as
  public-source evidence. Schema-v3 `[acoustics.dsm]`
  now maps declared inline synthetic DSM inputs, SHA-pinned `deck_file`
  inputs, SHA-pinned SP-8072 `source_curve_file` inputs, and SHA-pinned
  SP-8072 `apparent_source_curve_file`, `axial_spectrum_curve_file`,
  `directivity_curve_file`, and `spectrum_curve_file` inputs into the
  runner's out-of-band `RunOutcome.acoustics` report and can override scalar
  absorption with a SHA-pinned `absorption_deck_file`, optionally selecting a
  named atmosphere profile from that deck via `absorption_profile_id`, all
  behind gated `acoustics.dsm.*` summary telemetry without changing default
  telemetry or force paths. Missing for full WP-22.2 closure:
  production-quality digitized
  SP-8072 spectrum/directivity/source/axial-spectrum/apparent-source curves,
  production/public atmosphere-specific absorption validation, and published
  SP-8072 worked-example reference-value validation.
  Missing downstream: full ascent/fairing acoustic closure, vibroacoustic
  response, P95/50 environments, and high-fidelity ingestion.
- `openbmp-acoustics::pad` now provides the first WP-22.3 substrate: a
  Schema-1 provenance-pinned synthetic pad suppression deck parser with dry
  and nominal-water per-band knockdown classes. Schema-v3 `[acoustics.dsm]`
  can consume `suppression_deck_file`, `suppression_deck_file_sha256`, and
  `suppression_class`; the runner applies the selected class as a per-band
  transfer on the DSM directivity deck and records suppression provenance in
  `RunOutcome.acoustics`. Schema-v3 `[acoustics.dsm]` can also declare
  `cluster_engine_count` for exact incoherent identical-engine power scaling
  from the declared per-engine exit condition, or explicit
  `cluster_engines` lateral offsets for incoherent geometric summation against
  the receiver bearing, and `pad_path_length_extra_m` for declared
  duct/trench/deflector propagation path-length effects. It can also declare
  `pad_deflection_origin_axial_m` plus `pad_deflection_angle_deg` to map
  source-line coordinates through a deterministic single-bend plume axis
  before source-receiver range and local directivity-angle evaluation.
  Declared `cluster_correlation_coefficient` and
  `cluster_shielding_loss_db` fields add a scalar cross-pressure coherence
  term and broadband shielding loss after cluster summation, and a
  provenance-pinned synthetic `cluster_interaction_deck_file` plus
  `cluster_interaction_class` can apply per-band cluster correlation and
  shielding vectors while reporting deck/class provenance. Missing for full
  WP-22.3 closure: calibrated duct/trench effects, production pad-deflector
  geometry, calibrated plume interaction, cluster-specific directivity,
  published suppression-curve digitization, and a liftoff signature residual
  table.
- `openbmp-acoustics::iop` now provides the first WP-22.4 substrate: a
  deterministic ignition-overpressure pulse model with source strength from
  `d(mdot*c*)/dt`, duct/trench residence-time coupling, inverse-square
  receiver decay, pressure-amplitude suppression knockdown, and declared dB
  uncertainty bounds. It also exposes a Schema-1 provenance-pinned synthetic
  IOP duct/trench deck parser and a Schema-1 provenance-pinned synthetic IOP
  suppression deck parser. Schema-v3 `[acoustics.iop]` accepts either an
  explicit source rate/ramp or a declared liquid-engine `source_engine_id`
  whose start-rate proxy is derived from `max_thrust_n /
  ignition_transient_s`, or from thermochemistry-derived nominal `mdot*c* /
  ignition_transient_s` when the source engine declares
  `thermochemical_performance`; it maps inline duct terms or a SHA-pinned
  `duct_deck_file` plus `duct_class` into the shared runner acoustics report,
  can add a SHA-pinned `suppression_deck_file` plus `suppression_class` to the
  scalar pressure-amplitude suppression loss, and gates `acoustics.iop.*`
  summary telemetry. It reports deck SHA-256, dataset id, source class, and
  selected class id when decks are used, and records the first declared
  `at_time` scenario-script ignite command for the source engine when one
  exists plus the first runtime-observed point-mass or rigid-body source-engine
  ignite firing when it occurs. Point-mass and rigid-body source-engine IOP
  reports also carry a deterministic runtime source-history CSV, peak
  finite-difference source rate reconstructed from source-engine snapshots, and
  a runtime-reconstructed IOP peak/lower/upper/duration envelope from the
  observed positive source-rate peak.
  Missing for WP-22.4
  closure: calibrated/public `d(mdot*c*)/dt` reconstruction and validation of
  runtime command-response pressure history,
  calibrated/public duct/trench and suppression decks, and published IOP trend
  anchoring.
- `openbmp-acoustics::ascent` now provides the first WP-22.5 substrate: a
  Schema-1 provenance-pinned synthetic ascent acoustic environment class deck
  parser/evaluator and a Schema-1 synthetic fairing-internal acoustic transfer
  deck parser/evaluator. Schema-v3 `[acoustics.ascent]` consumes a SHA-pinned
  `deck_file` plus either a single `class_id`/`dynamic_pressure_pa`/`mach`
  sample, a `class_selection = "mach_gate"` sample that selects the unique deck
  class whose Mach gate contains the sample, or `[[acoustics.ascent.samples]]`
  trace rows; schema-v3
  `[acoustics.fairing_internal]` requires `[acoustics.ascent]`, consumes a
  SHA-pinned fairing insertion-loss `deck_file` plus `class_id` and
  `fill_factor`, and applies that transfer to the external ascent envelope spectrum.
  The shared runner emits `RunOutcome.acoustics.ascent` and
  `RunOutcome.acoustics.fairing_internal`, deterministic one-third-octave
  envelope spectrum CSV and trace CSV bytes, and gated `acoustics.ascent.*` plus
  `acoustics.fairing_internal.*` summary telemetry. The fixtures carry
  attached-flow/transonic toy classes, light/heavy fairing insertion-loss toy
  classes, exact SHA-256 reporting, OASPL-normalized energy and trace envelope checks, and
  fail-closed malformed-deck/band-grid checks, including no-match and
  ambiguous Mach-gate class selection. Missing for WP-22.5 closure:
  full geometry-driven `03` regime/zone selection, published payload-user-guide
  or MPE class curves, payload-zone spatial maps, and flight or user-guide
  residual anchoring.
- `openbmp-acoustics::response` now provides the first WP-22.6 substrate: a
  Schema-1 provenance-pinned synthetic vibroacoustic response transfer deck
  parser/evaluator, SPL-to-pressure-PSD conversion, acceleration-PSD response
  transfer with square-law surface-mass scaling, and pressure-PSD-to-modal-force
  PSD handoff using declared zone area and mode-shape participation.
  Schema-v3 `[acoustics.response]` consumes either the ascent envelope or
  fairing-internal spectrum, resolves and pin-verifies the response deck, and
  emits `RunOutcome.acoustics.response`, deterministic acceleration/modal-force
  PSD CSV bytes, and gated summary telemetry. The checked fixture carries
  light/heavy synthetic panel classes, exact SHA-256 reporting, deterministic
  integrated mean-square acceleration and force reductions, and fail-closed
  malformed-deck/class/input/band-grid checks.
  `openbmp-acoustics::stats` also provides the first P95/50
  maximum-expected-environment substrate: deterministic per-band normal
  P95/50 SPL statistics over common-grid spectra, OASPL reduction, and CSV
  table bytes. Multi-sample schema-v3 `[acoustics.ascent]` reports now retain
  per-sample spectra and emit P95/50 OASPL, tolerance factor, and table CSV
  bytes as out-of-band run artifacts; `[acoustics.fairing_internal]` applies the
  same method to transferred trace sample spectra, can pair those spectra with
  optional `[[acoustics.fairing_internal.samples]]` fairing-side inputs, and
  emits an internal P95/50 table plus deterministic fairing sample traces.
  `[acoustics.response]` now runs the selected response transfer over
  the retained source sample spectra, can pair those spectra with optional
  `[[acoustics.response.samples]]` response-side inputs, and emits P95/50
  acceleration/modal-force PSD tables, deterministic response sample traces,
  plus integrated P95/50 mean-square acceleration and force artifacts.
  The runner records one-sided Wilks P95/50 sample-count sizing and achieved
  confidence evidence next to ascent, fairing-internal, and response P95/50
  artifacts. Off-by-default CLI run-data output now writes nonempty
  ascent/fairing/response P95/50 CSV payloads and fairing/response P95/50 sample traces
  under the run-data artifact root and indexes their roles, paths, and SHA-256
  pins in the `RunRecord`. Response
  reports also emit a deterministic structural loads handoff CSV, and the CLI
  indexes that artifact for `21` access. `openbmp-runner::structural` now
  consumes the typed acoustic modal-force PSD for rigid-body scenarios with
  `[vehicle.bending]`, derives a deterministic first-mode SDOF random-response
  summary with per-band displacement/acceleration response CSV bytes, and
  records it on `RunOutcome.structural_acoustics` without changing the
  time-domain force path. Off-by-default CLI run-data output also writes and
  indexes that structural acoustic response CSV with a SHA-256-pinned
  `structural_acoustics_first_bending_response_csv` role. That report now also
  recovers a first-increment engine-station equivalent reaction-moment
  line-load table from the same first bending mode slope and modal acceleration
  PSD, and `openbmp run` writes/indexes it as
  `structural_acoustics_first_bending_line_load_recovery_csv`.
  `openbmp mc acoustics-ascent-trace` now materializes
  deterministic LHS/Sobol/Owen-Sobol
  `[[acoustics.ascent.samples]]` TOML snippets over declared dynamic-pressure
  and Mach ranges for the existing runner P95/50 path to consume, and
  `openbmp mc acoustics-response-samples` now materializes deterministic
  `[[acoustics.response.samples]]` snippets over declared surface-mass, area,
  and mode-shape ranges for response-side P95/50 input variation. `openbmp mc
  acoustics-ascent-campaign` now injects generated ascent samples into a base
  `[acoustics.ascent]` scenario, runs the normal runner, writes the evaluated
  per-sample ascent trace CSV plus P95/50/Wilks summary evidence, surfaces
  downstream fairing-internal and vibroacoustic-response P95/50/Wilks summary
  values when the base scenario declares those blocks, can optionally
  materialize nonempty evaluated campaign CSV artifacts under a caller-provided
  artifact directory, and can optionally checkpoint/resume per-sample OASPL
  evaluations through the shared `openbmp-mc` scalar checkpoint reducer with
  bounded worker fan-out before writing final artifacts on completion.
  `openbmp mc acoustics-response-campaign` now exposes the same generated
  ascent-trace campaign path as a response-focused wrapper that requires
  `[acoustics.response]` in the base scenario and reports response P95/50
  acceleration, modal-force, and Wilks evidence on completion; it can also
  append generated `[[acoustics.response.samples]]` rows to completed campaign
  scenarios from response-side range flags. Missing for WP-22.6 closure:
  generated fairing design-variable CLI samplers,
  generalized checkpointed/parallel MC campaign orchestration beyond these
  ascent-trace-driven acoustic commands, calibrated SEA-lite/Franken transfer
  evidence, and broader `02` structural-stack integration beyond the first bending-mode
  out-of-band consumer, including multi-mode/FEM-derived LTM/OTM recovery.
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
| SP-8072 Figure 5 spectrum curve deck | public curve parser/integration fixture | T1 | exact deck parse + normalized band-power derivation; coarse digitization label |
| SP-8072 Figure 11 core-length fit | source formula regression | T1 | exact formula values < 1e-12 abs |
| SP-8072 Figure 10 directivity curve deck | public curve parser/integration fixture | T1 | exact deck parse + band directivity derivation; coarse digitization label |
| SP-8072 Figure 12 axial source-power curve deck | public curve parser/integration fixture | T1 | exact deck parse + normalized source allocation; coarse digitization label |
| SP-8072 Figure 14 apparent-source curve deck | public curve parser/integration fixture | T1 | exact deck parse + apparent source location derivation; coarse digitization label |
| DSM air-absorption deck | synthetic parser/integration fixture | T1 | exact deck parse + per-band propagation application; toy-data label |
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
- **implementation_status:** partial — the deterministic free-field DSM
  mechanics are in `openbmp-acoustics::dsm` with synthetic closed-form and
  byte-stability tests, and schema-v3 `[acoustics.dsm]` emits a runner report,
  plus gated summary telemetry. A provenance-pinned Schema-1 DSM source /
  directivity deck parser, synthetic fixture, and scenario/runner `deck_file`
  path exist. A Schema-1 SP-8072 source manifest pins the public NASA NTRS
  source PDF and inventories the method figures that future digitized decks
  must cite, and the Figure 11 `x_t / d_e` core-length fit is implemented as a
  validated helper. Coarse Figure 5 spectrum, Figure 10 directivity-index,
  Figure 12 axial source-power, and Figure 13 axial spectrum curve decks now
  parse and derive runtime DSM spectrum/directivity/source/source-by-band
  weighting inputs. A coarse Figure 14 apparent-source curve deck also parses
  and can place one apparent source per band for the earlier SP-8072
  source-location approach. All five coarse SP-8072 curve decks validate their
  manifest id, source PDF hash, and figure id against the pinned public source
  manifest, and runner curve loaders reject manifest mismatches after scenario
  SHA-pin verification. Schema-v3 scenarios can consume these through
  `spectrum_curve_file`, `directivity_curve_file`, `source_curve_file` with
  `exit_mach`, `axial_spectrum_curve_file`, and `apparent_source_curve_file`,
  and can select named atmospheric profiles from a SHA-pinned absorption deck
  with `absorption_profile_id`. A coarse SP-8072 curve tolerance-table
  substrate can compare evaluated DSM spectra to reference spectra and emit
  deterministic pass/fail CSV evidence, but production-quality SP-8072
  spectrum/directivity/source/axial-spectrum/apparent-source curve decks,
  production/public atmospheric-attenuation validation, and public
  worked-example reference-value reproduction remain open.
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
- **implementation_status:** partial — `openbmp-acoustics::pad` now exposes a
  Schema-1 synthetic pad suppression deck parser and
  `data/acoustics/pad-suppression-v1.toml` provides dry and nominal-water
  one-third-octave knockdown classes. Schema-v3 `[acoustics.dsm]` resolves a
  SHA-pinned `suppression_deck_file`, requires `suppression_class`, rejects
  bad pins and orphan fields, and the shared point-mass/rigid-body acoustics
  evaluator applies the selected class as a per-band DSM transfer while
  reporting the deck SHA-256, dataset id, source class, selected class, and
  uncertainty. The same schema now accepts optional `cluster_engine_count`,
  treating the declared exit condition as per engine and scaling total
  acoustic power by N with exact 10 log10(N) station-OASPL evidence for
  identical engines. It also accepts explicit
  `[[acoustics.dsm.cluster_engines]]` offsets plus optional
  `receiver_azimuth_deg`; the runner evaluates each engine as a separate
  laterally shifted receiver geometry and pressure-energy-adds the spectra,
  with a regression proving equality against two independent single-engine
  runs at the corresponding effective radii. This is deterministic
  parser/runtime wiring only. It also accepts `pad_path_length_extra_m`,
  which adds an explicit non-negative duct/trench/deflector path-length term
  to propagation before spreading and absorption, with an exact one-source
  zero-absorption `20 log10((d + extra) / d)` regression. It also accepts
  `pad_deflection_origin_axial_m` plus `pad_deflection_angle_deg` for a
  deterministic single-bend plume-axis approximation and proves the resulting
  source-range change with a one-source zero-absorption spreading oracle.
  It now accepts declared `cluster_correlation_coefficient` and
  `cluster_shielding_loss_db`, combining geometric-cluster spectra with a
  scalar cross-pressure term plus broadband loss while preserving the default
  incoherent energy-add path. It also accepts a SHA-pinned synthetic
  `cluster_interaction_deck_file` plus `cluster_interaction_class`; the
  Schema-1 parser validates per-band correlation and shielding vectors, the
  scenario schema resolves and pin-verifies the deck while rejecting scalar
  interaction conflicts, and the runner applies the selected class to
  identical or geometric clusters while reporting deck SHA-256, dataset id,
  source class, selected class id, and uncertainty. Calibrated duct/trench
  factors, production pad-deflector geometry, calibrated plume interaction,
  cluster-specific directivity, published suppression data, and liftoff
  signature anchoring remain open.
- **fidelity_tier:** T2
- **depends_on:** [WP-22.2; cross-doc: `18` `[pad.geometry]` or declared
  stub]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (pad module), `data/` suppression
  and cluster-interaction decks + `provenance.md`, scenario schema.
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
- **implementation_status:** partial — `openbmp-acoustics::iop` now exposes
  core deterministic primitives for `IopStartTransient`,
  `IopDuctCoupling`, `IopUncertainty`, `IopPulseInput`, and
  `IopPulseEnvelope`. `evaluate_iop_pulse()` computes a nominal peak
  overpressure from start-rate scaling, duct residence time, geometry gain,
  inverse-square distance decay, and suppression pressure knockdown, then
  applies declared lower/upper dB uncertainty bounds and reports a
  ramp-plus-residence duration. Schema-v3 `[acoustics.iop]` validates explicit
  source-rate/ramp declarations, optional `source_engine_id` derivation from a
  declared liquid-engine ignition transient, thermochemistry-derived nominal
  `mdot*c*` source strength when the source engine declares
  `thermochemical_performance`, duct, receiver, suppression, and uncertainty
  fields; the shared point-mass/rigid-body acoustics evaluator reports the IOP
  envelope through `RunOutcome.acoustics.iop` and emits gated summary telemetry
  for peak, lower/upper peak bounds, and duration. Focused
  tests prove closed-form peak and duration math, monotone source-rate
  scaling, inverse-square distance scaling, suppression ordering, uncertainty
  bounds, fail-closed input validation, schema acceptance/rejection, engine
  source derivation, thermochemistry-backed `mdot*c*` source-rate coupling,
  declared source-engine ignite-command timing evidence, point-mass and
  rigid-body runtime-observed source-engine ignite timing evidence,
  deterministic runtime source-history reconstruction from engine snapshots,
  runtime-reconstructed IOP peak/lower/upper/duration envelope evidence, runner
  report
  generation, and telemetry values. Missing: calibrated/public
  `d(mdot*c*)/dt` reconstruction and validation of runtime command-response
  pressure history, calibrated/public duct/trench and suppression data decks,
  and published Shuttle IOP trend residuals.
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
- **implementation_status:** partial — `openbmp-acoustics::ascent` exposes a
  Schema-1 synthetic ascent acoustic environment deck parser/evaluator and a
  Schema-1 synthetic fairing-internal acoustic transfer deck parser/evaluator.
  `data/acoustics/ascent-acoustic-env-v1.toml` declares attached-flow and
  transonic toy classes with Mach gates, dynamic-pressure reference OASPL,
  q-scaling exponents, OASPL-normalized one-third-octave relative band shapes,
  and uncertainty labels. `data/acoustics/fairing-internal-v1.toml` declares
  light/heavy synthetic fairing insertion-loss classes and uncertainty labels.
  Schema-v3 `[acoustics.ascent]` resolves and pin-verifies `deck_file`,
  validates either a single `class_id`/`dynamic_pressure_pa`/`mach` sample,
  a `class_selection = "mach_gate"` sample/trace that selects the unique class
  whose deck Mach gate contains each sample, or `[[acoustics.ascent.samples]]`
  trace rows, and the shared runner reports deck
  provenance, peak sample metadata, envelope OASPL, uncertainty, deterministic
  envelope spectrum CSV bytes, deterministic sample trace CSV bytes, and gated
  telemetry.
  Schema-v3 `[acoustics.fairing_internal]` requires `[acoustics.ascent]`,
  resolves and pin-verifies its fairing deck, validates `station_id`,
  `class_id`, and `fill_factor`, applies fill-factor gain plus per-band
  insertion-loss to the external ascent envelope spectrum, and reports deck provenance,
  selected class, internal OASPL, uncertainty, deterministic spectrum CSV
  bytes, and gated telemetry. Tests prove exact parser/provenance fields,
  deterministic SHA-256 digest syntax, q-scaling, OASPL normalization, ascent
  trace envelope math, fairing transfer math, unknown class rejection,
  explicit-class Mach-gate rejection, automatic Mach-gate no-match/ambiguity
  rejection, fill-factor bounds, fairing/ascent band-grid mismatch rejection,
  malformed-deck rejection, schema validation, deck pinning, runner provenance
  reporting, and telemetry values. This makes no claim to published
  payload-user-guide/MPE curves, payload-zone spatial maps, full
  geometry-driven `03` regime/zone selection, or
  flight-correlated zone maps.
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
- **implementation_status:** partial — `openbmp-acoustics::response` exposes
  the Schema-1 synthetic vibroacoustic transfer-deck boundary, light/heavy panel
  fixture, SPL-to-pressure-PSD conversion, acceleration-PSD transfer with
  surface-mass scaling, and modal-force PSD handoff from pressure PSD, zone
  area, and mode-shape participation. Schema-v3 `[acoustics.response]` consumes
  either the ascent envelope or fairing-internal spectrum and emits response
  provenance, acceleration/modal-force PSD CSV bytes, integrated mean-square
  acceleration/force values, and gated summary telemetry. The crate, scenario,
  and runner tests prove parser provenance, fail-closed deck/input handling,
  response scaling, dependency validation, deck pinning, deterministic
  integrated mean-square reductions, and modal-force handoff reporting.
  `openbmp-acoustics::stats` now adds deterministic normal P95/50 SPL
  statistics over common-grid spectra, including per-band dB mean/sigma,
  P95/50 spectrum/OASPL, and CSV table bytes. The runner derives those table
  bytes for multi-sample ascent traces and records them on
  `RunOutcome.acoustics.ascent`, and derives the corresponding internal table
  for fairing-internal transfers sourced from those traces. Fairing-internal
  reports can pair retained ascent spectra with
  `[[acoustics.fairing_internal.samples]]` per-sample fill factors and class
  overrides and record deterministic fairing sample trace CSV evidence.
  Response reports also derive P95/50 acceleration/modal-force PSD tables from the retained
  source sample spectra, optionally pair those spectra with
  `[[acoustics.response.samples]]` per-sample response inputs for surface mass,
  area, mode-shape factor, and class overrides, and record integrated P95/50
  mean-square acceleration/force artifacts plus deterministic response sample
  trace CSV evidence. The runner now also records one-sided Wilks P95/50
  sample-count sizing, achieved confidence, adequacy, and deterministic sizing
  CSV evidence beside ascent, fairing-internal, and response P95/50 artifacts.
  Off-by-default CLI run-data output writes and indexes nonempty
  ascent/fairing/response P95/50 CSV payloads plus fairing/response P95/50 sample traces
  with SHA-256 pins. Response
  reports also emit a deterministic structural loads handoff CSV with
  modal-force PSD, per-band mean-square force, signed modal participation, and
  total mean-square force, and the CLI indexes that artifact for `21` access.
  Rigid-body runs with `[vehicle.bending]` now consume the typed modal-force
  PSD through `openbmp-runner::structural` and emit a
  `RunOutcome.structural_acoustics` first-mode random-response report with
  modal displacement/acceleration mean-square totals and per-band CSV evidence,
  still out-of-band from the time-domain force authority, and `openbmp run`
  indexes that response CSV in run-data output under the
  `structural_acoustics_first_bending_response_csv` role. The same report now
  emits a first-increment engine-station equivalent reaction-moment line-load
  recovery CSV and run-data indexes it under the
  `structural_acoustics_first_bending_line_load_recovery_csv` role.
  `openbmp mc acoustics-ascent-trace` now materializes deterministic
  LHS/Sobol/Owen-Sobol `[[acoustics.ascent.samples]]` TOML snippets over
  declared dynamic-pressure and Mach ranges, giving the existing runner P95/50
  path an MC-designed trace input. `openbmp mc acoustics-response-samples` now
  materializes deterministic `[[acoustics.response.samples]]` TOML snippets
  over surface-mass, area, and mode-shape ranges for response-side P95/50
  input variation. `openbmp mc acoustics-ascent-campaign` now injects generated
  ascent samples into a base `[acoustics.ascent]` scenario, executes the normal
  runner, writes the evaluated trace CSV, can optionally write the full
  generated scenario, reports P95/50 plus Wilks sufficiency evidence, and can
  surface downstream fairing-internal and vibroacoustic-response P95/50/Wilks
  summary values when the generated scenario evaluates those reports. It can
  also write nonempty evaluated campaign CSV artifacts to
  `--output-artifact-dir` on completion, including ascent/fairing P95/50
  environments, fairing P95/50 sample traces, response P95/50 PSD tables,
  response P95/50 sample traces,
  response loads handoff, and structural-acoustic CSVs when present, with a
  `manifest.json` containing roles, relative paths, and SHA-256 pins for the
  written artifacts. It can use
  an opt-in checkpoint JSON with bounded workers to resume per-sample OASPL
  evaluation before final artifact emission. `openbmp mc
  acoustics-response-campaign` now wraps that same generated ascent-trace path
  with a response-specific base-scenario check and response P95/50 summary
  reporting, and can append generated response-side sample rows to the final
  campaign scenario while rejecting bases that already declare response samples.
  Generated fairing design-variable CLI samplers, generalized
  checkpointed/parallel MC campaign orchestration beyond these
  ascent-trace-driven acoustic commands, calibrated transfer evidence, and
  broader `02` structural-stack integration beyond the first bending-mode
  out-of-band consumer, including multi-mode/FEM-derived LTM/OTM recovery, are
  still open.
- **fidelity_tier:** T4
- **depends_on:** [WP-22.3, WP-22.5; cross-doc: WP-11.1 (DoE/sizing),
  `02` forcing interface]
- **new_crates:** none.
- **touched:** `crates/openbmp-acoustics` (response + stats modules),
  `crates/openbmp-runner` response report, `crates/openbmp-cli` run-data
  artifact writes and MC trace materializer, `data/` transfer decks +
  `provenance.md`, `02` interface types, artifacts for `21`.
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
