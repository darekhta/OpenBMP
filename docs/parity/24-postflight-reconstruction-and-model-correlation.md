# Postflight Reconstruction, System Identification & Model Correlation

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`11-monte-carlo-uq-and-validation.md` (whose BET machinery this document
builds on), `docs/external-telemetry-validation.md` (the LOCAL-only rule
that governs any real data).

> One-line scope: the "every flight improves the models" discipline — a
> sim-vs-flight differencing harness, output-error/filter-error parameter
> estimation (thrust/Isp multipliers, aero-coefficient increments, IMU
> errors, mass, wind co-estimation) with identifiability and Cramér-Rao
> reporting, per-discipline reconciliation recipes, anomaly-timeline
> tooling, and a provenance-gated governance pipeline through which
> estimated corrections re-enter the decks as reviewed, labeled artifacts —
> never automatically.

Doc `11` designs the BET half (RTS smoother, batch Gauss-Newton, NEES/NIS)
and stops there: its scope list defers parameter estimation and model
correlation. This document is the second half. The defining constraint:
in-repo work runs on **sim-generated pseudo-flight data with known truth**;
real telemetry enters only through the LOCAL workflow and never lands in the
repository.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **NASA/SLS — Artemis I** ran a coordinated, pre-planned reconstruction
  campaign (approach paper NTRS 20205004519): Best Estimated Trajectory
  fusing telemetry + radar + environment observations, then per-discipline
  model correlation — ascent loads (NTRS 20230008005), base aerodynamics
  (NTRS 20240000053), base aerothermal (NTRS 20230018483), and as-flown
  propulsion parameter estimation (booster Isp, thrust multipliers, dry
  mass).
- **Shuttle** reconstructed propulsion performance from flight data for
  decades (EKF over SRB head pressure, IMU acceleration, radar —
  NTRS 19890061688); day-of-launch and loads models were continuously
  re-anchored.
- **NASA flight-mechanics sys-ID lineage** — Maine & Iliff's maximum-
  likelihood output-error method (MMLE3; NASA RP-1168) is the canonical
  formulation; modern launch-vehicle aero parameter estimation continues it
  (published Pegasus first-flight aero-database update case).
- **SpaceX** instrumented Falcon 9 boosters *specifically* to reconstruct
  supersonic-retropropulsion flight regimes with NASA ("flight
  reconstruction, CFD analysis, imagery campaign"); Starship updates frame
  "data and flight learnings as our primary payload"; GNC postings list
  "post-flight data review and correlation with analyses" as a duty.
- **Rocket Lab** reviewed 25,000+ telemetry channels in anomaly
  investigations and offers coupled-loads analysis "incorporating data from
  previous flights" — model correlation as a customer-facing product.

### 1.2 Parity target (capability, within posture)

1. **Differencing harness** — channel-aligned sim-vs-"flight" comparison
   (resampling, time-registration, unit/frame reconciliation through the
   existing schema), residual statistics per channel/phase, automated
   out-of-family flagging via `21`'s rule machinery.
2. **Estimable-parameter registry** — a closed, declared vocabulary of
   vehicle-intrinsic parameters θ: thrust/Isp multipliers, aero-coefficient
   increments per Mach bin, IMU bias/scale-factor sets, dry-mass and CG
   corrections, wind-profile node values; bounds, priors, units, and the
   models they touch.
3. **Output-error estimation** — iterated maximum-likelihood: propagate the
   full sim with θ, form measurement residuals, compute sensitivities
   (finite-difference or STM-based via the `07` machinery), Gauss-Newton /
   Levenberg-Marquardt update, convergence and covariance reporting;
   filter-error variant (process noise) as the upper tier.
4. **Identifiability** — Fisher information, Cramér-Rao lower bounds,
   parameter correlation matrices; fail-closed refusal to report
   unidentifiable parameter sets (rank/conditioning gates); instrumentation
   planning loop (which channels make θ observable — the DFI-planning
   analog).
5. **Per-discipline recipes** — documented, tested reconciliation
   workflows: propulsion (multipliers from acceleration + chamber-pressure
   channels), aerodynamics (force-coefficient increments from
   accelerometer-derived axial/normal coefficients vs the deck), mass
   properties, environment (wind co-estimation consistent with `19`'s
   measured profiles), navigation (IMU error recovery cross-checked against
   `09`'s injected truth).
6. **Anomaly-timeline tooling** — deterministic change-point extraction and
   event-sequence assembly over channel sets (the 25,000-channel triage
   pattern), feeding fault-tree bookkeeping documents.
7. **Governance pipeline** — estimated corrections exported as *candidate
   deck increments*: provenance-pinned artifacts carrying estimate,
   uncertainty, validity envelope, and validation label, entering the repo
   only through human-reviewed PRs; tripwires keep real-vehicle parameters
   out.

### 1.3 Parity ceiling (honest boundary)

- **No real BET inputs in-repo.** Genuine reconstructions need proprietary
  telemetry/radar/IMU streams. In-repo: pseudo-flight campaigns
  (sim-generated truth + `09`/`20` measurement models + withheld parameter
  perturbations). Real data: LOCAL-only comparison per
  `docs/external-telemetry-validation.md`, results never committed, never
  used as tuning targets for shipped decks.
- **No flight-quality claim transfer.** A correlation performed locally
  against real data cannot raise any in-repo label; labels rise only
  through the public-benchmark ladder of `docs/verification.md`.
- **Estimation-method ceiling.** Output-error/filter-error with GN/LM;
  no adaptive/online estimation, no neural surrogates; frequency-domain
  sys-ID named, not designed.
- **Anomaly tooling ceiling.** Change-point extraction and timeline
  assembly; root-cause reasoning, fault-tree solving, and physical failure
  replication stay human work (the AMOS-6-class campaign is practice to
  *support*, not automate).

---

## 2. Current state in source

- `11` (designed) — RTS smoother, batch Gauss-Newton over initial state,
  NEES/NIS consistency machinery (WP-11.6): the state-side substrate this
  document extends to *parameters*.
- `crates/openbmp-sil/src/monitor.rs` — estimate-vs-truth observation
  (recent live-SIL work): the truth-referenced comparison pattern
  generalized here.
- `crates/openbmp-fc/src/replay.rs` — deterministic replay surface usable
  for re-propagation under perturbed parameters.
- `crates/openbmp-physics/src/external_reference.rs` +
  `docs/external-telemetry-validation.md` — the quarantined LOCAL
  comparison workflow (gross-error sanity checking) this document upgrades
  into a disciplined reconciliation pipeline (still LOCAL).
- `crates/openbmp-telemetry` schema + `21` (designed) run records — the
  data layer the differencing harness reads.
- `crates/openbmp-trajopt` STM machinery (designed in `07`) — sensitivity
  computation path.
- No parameter registry, no output-error estimator, no identifiability
  reporting, no governance pipeline exists.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-sysid` (L6, offline analysis): consumes telemetry artifacts,
measurement streams (`20` tracking observables when configured), and
scenario/provenance metadata; drives forward runs through the existing
runner programmatically; produces reconciliation reports and candidate deck
increments. Host-side, never FC-reachable. Alternative (module inside
`openbmp-mc`) is rejected in design: estimation is not sampling; the
boundary is reviewed in the skeleton WP regardless.

### 3.2 Differencing harness (T1)

- **Registration:** time-base alignment (declared epoch mapping + optional
  estimated time-bias as a θ entry), resampling to a common grid
  (deterministic interpolation, declared method), unit/frame reconciliation
  through the channel schema (mismatches fail closed).
- **Residual products:** per-channel residual series, phase-segmented
  statistics (mean/RMS/max, by mission phase from the event timeline),
  out-of-family flags via `21` baseline bands, and a residual report
  artifact.
- **Inputs:** another sim run (regression diffing), a pseudo-flight bundle
  (truth + sensed channels), or — LOCAL only — a user-supplied real bundle
  conforming to the external-telemetry contract.

### 3.3 Parameter registry (T2)

```rust
/// Closed, declared vocabulary of estimable vehicle-intrinsic parameters.
pub enum EstimableParameter {
    ThrustMultiplier { engine: EngineId },
    IspMultiplier { engine: EngineId },
    AeroCoeffIncrement { coeff: AeroCoeff, mach_bin: MachBin },
    ImuBias { axis: Axis3, kind: BiasKind },     // per 09 error vocabulary
    ImuScaleFactor { axis: Axis3 },
    DryMassDelta { body: BodyId },
    CgOffset { body: BodyId, axis: Axis3 },
    WindNode { altitude_bin: AltBin, component: WindComponent },
    TimeBias,
}
```

A **closed enum** (the vocabulary-lock pattern): every entry is
vehicle-intrinsic or environmental; there is no surface for
position-of-anything-else, range, or impact-adjacent quantities, and adding
a variant is a reviewed API change with a tripwire (WP-24.2). Each entry declares
bounds, prior (mean/σ), units, and the model seam it perturbs (implemented
as typed hooks in the affected crates, off by default).

### 3.4 Output-error estimator (T2–T5)

Maximum-likelihood output error (Maine-Iliff/MMLE lineage):

```
J(θ) = Σ_k  ν_k(θ)ᵀ R⁻¹ ν_k(θ) + (θ−θ₀)ᵀ P₀⁻¹ (θ−θ₀)
ν_k(θ) = y_k − h(x_k(θ))            // measured minus model output
θ_{i+1} = θ_i + (Sᵀ R⁻¹ S + P₀⁻¹ + λI)⁻¹ Sᵀ R⁻¹ ν      // GN / LM
```

- **Sensitivities S = ∂y/∂θ:** central finite differences over full forward
  runs (deterministic, embarrassingly parallel via `12`) at T2; STM/
  variational propagation through the `07` machinery where the parameter
  enters smoothly (T5 optimization).
- **Fixed iteration policy:** declared max iterations + convergence
  tolerance; divergence is a reported failure (never a silent best-effort).
- **Filter-error tier (T5):** process-noise-aware variant (Kalman-filter
  inner loop with θ in the model, ML over innovations) for wind
  co-estimation and flexible-mode-contaminated channels.
- **Outputs:** θ̂, covariance, residual whiteness diagnostics (reusing
  NEES/NIS from `11`), convergence trace — one reconciliation report.

### 3.5 Identifiability & instrumentation planning (T3)

Fisher information `F = Sᵀ R⁻¹ S` at θ₀; CRLB = diag(F⁻¹)^½; parameter
correlation matrix from normalized F⁻¹. Gates: condition-number and
per-parameter CRLB-vs-bound checks **fail closed** — an unidentifiable set
is refused with the offending correlations named (no quietly-pinned
parameters). The same machinery inverts into planning: given candidate
channel sets, report which θ become identifiable — the
developmental-flight-instrumentation planning loop (Artemis DFI analog) as
an offline analysis.

### 3.6 Per-discipline recipes (T3–T4)

Documented, fixtured workflows (each a scenario family + registry subset +
expected-recovery tolerance):

- **Propulsion:** thrust/Isp multipliers from axial acceleration +
  chamber-pressure channels through ascent; cross-checked against `05`'s
  efficiency-band vocabulary (estimates must land inside declared bands or
  the report flags model-form error).
- **Aerodynamics:** accelerometer-derived force coefficients vs the `03`
  deck → per-Mach-bin increments with the deck's UQ as prior; output shaped
  exactly like a `03` UQ overlay so the governance pipeline can carry it.
- **Mass properties:** dry-mass/CG deltas from coast-phase dynamics.
- **Environment:** wind-node co-estimation (filter-error tier) consistent
  with `19`'s profile schema — reconstructed winds are a *diagnostic*
  product, comparable against the measured profile that flew.
- **Navigation:** IMU bias/SF recovery cross-validated against `09`'s
  injected truth (closed-loop self-test of the whole pipeline).

### 3.7 Anomaly-timeline tooling (T4)

Deterministic change-point extraction (CUSUM-class, shared detectors with
`21`) over declared channel sets; event-sequence assembly (ordered,
cross-channel, with lead/lag annotations); timeline artifact + fault-tree
bookkeeping template (structured document linking events to hypotheses to
evidence runs). Supports the triage pattern; concludes nothing by itself.

### 3.8 Governance pipeline (T4)

Estimated corrections leave as **candidate deck increments**: artifacts
carrying (parameter, estimate, covariance, validity envelope, source-run
provenance chain, label ≤ `research`). Entry into shipped decks happens only
by human-reviewed PR that treats the increment like any external data
(provenance + tolerance tables + label justification). Tripwires: (a) the
existing inline-data tripwire extends to increment files; (b) a new check
rejects any increment artifact whose provenance chain marks a LOCAL/real
source (real-data corrections are structurally non-committable); (c) the
parameter vocabulary is closed (§3.3).

### 3.9 Fidelity tiers

- **T0 (current):** golden diffing + LOCAL gross-error comparison.
- **T1:** differencing harness + residual reports — `validated-toy`.
- **T2:** registry + output-error GN/LM on pseudo-flight — `validated-toy`.
- **T3:** identifiability/CRLB + propulsion/aero/nav recipes —
  `validated-toy` → `research` (method anchored to RP-1168/published
  cases).
- **T4:** anomaly timelines + governance pipeline — `checked`/
  `validated-toy`.
- **T5:** filter-error + wind co-estimation + STM sensitivities —
  `research`.

---

## 4. Invariant preservation

- **Determinism:** estimation is iterated forward simulation — every inner
  run obeys the existing determinism contract; FD perturbation schedules and
  LM damping sequences are declared; parallel sensitivity runs reduce
  order-independently (`12`).
- **Byte-stable default:** parameter hooks in model crates are off-by-
  default config seams; no scenario changes behavior unless
  `[sysid]`-driven runs set them; goldens untouched.
- **FC portability:** untouched; the estimator perturbs plant/measurement
  models, never FC internals (IMU errors enter through the `09` seams).
- **Provenance:** pseudo-flight bundles, priors, and increments pinned;
  the real-data path is LOCAL by construction (§3.8 tripwire makes the
  committed variant impossible).
- **Fail-closed:** unidentifiable sets, non-convergence, registration
  mismatches, and vocabulary violations are structured failures.
- **Labels:** reconciliation reports carry the *lowest* label in their
  evidence chain; nothing here raises labels (only the public ladder does).

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| Registration/resampling exactness on constructed signals | unit/property | T1 | exact (declared interpolation) |
| Residual statistics vs hand-computed on fixtures | analytic | T1 | < 1e-12 rel |
| Linear-system estimation: GN recovers closed-form least-squares | analytic | T2 | < 1e-9 rel; covariance matches closed form |
| Pseudo-flight recovery: withheld θ* recovered | self-consistency (MC over seeds, `11` sizing) | T2/T3 | θ̂ within 2·CRLB of θ* in ≥ declared fraction (table) |
| CRLB on a linear case vs analytic Fisher inverse | analytic | T3 | < 1e-9 rel |
| Unidentifiable-pair refusal (constructed collinear parameters) | property | T3 | fail-closed with named correlation |
| Propulsion recipe: multiplier recovery inside `05` band semantics | scenario | T3 | per recipe tolerance table |
| Aero recipe: increment recovery vs injected deck perturbation | scenario | T3 | per-bin recovery within CRLB bounds |
| Nav recipe vs `09` injected truth | closed-loop self-test | T3 | bias/SF within declared bounds |
| Whiteness diagnostics (NEES/NIS reuse) | statistical | T2+ | in-bounds fraction per `11` convention |
| Method anchor: published Pegasus-class aero-ID case *structure* reproduced on synthetic analog | code-to-code (method) | T3 | workflow-shape conformance documented (no proprietary numbers) |
| Governance: LOCAL-marked increment rejected by the tripwire | tripwire | T4 | structurally non-committable |
| Change-point detectors on synthetic step/drift | analytic | T4 | shared-detector tolerances (with `21`) |

---

## 7. Dependencies on other parity docs

- `11` — BET substrate (RTS/GN/NEES-NIS), Wilks/CP sizing for recovery
  campaigns, credibility records the reports feed.
- `21` — run records and baseline bands the harness reads; shared
  change-point detectors; report storage and trends.
- `09` — measurement models and error vocabulary for pseudo-flight
  generation and the nav recipe.
- `20` — tracking observables as additional measurement streams; link
  metadata for dropout-aware registration.
- `05`/`03` — efficiency-band and deck-UQ vocabularies the propulsion/aero
  recipes must respect; increment schemas shaped to their UQ overlays.
- `07` — STM/variational machinery for T5 sensitivities.
- `19` — wind-profile schema for co-estimation diagnostics.
- `12` — parallel sensitivity execution, order-independent reductions,
  checkpointing for long campaigns.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| NASA RP-1168 (Maine & Iliff, output-error theory) | method source | public NASA RP |
| Klein & Morelli, *Aircraft System Identification* | method reference | published textbook |
| Artemis I reconstruction paper set (NTRS 20205004519 + discipline papers) | workflow anchors | public NTRS |
| Shuttle propulsion reconstruction (NTRS 19890061688) | method anchor | public NTRS |
| Published Pegasus-class LV aero-ID case studies | method-structure anchor | published literature |
| SciPy/Dakota-class optimizers | code-to-code oracle on saved J(θ) evaluations (external process) | OSS/external |
| Orekit/GMAT estimation | code-to-code for state-side cross-checks (per `11`) | Apache-2.0 |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-24.1 — `openbmp-sysid` skeleton + differencing harness

- **title:** L6 analysis crate with time registration, resampling, unit/frame reconciliation, phase-segmented residual reports, out-of-family flags.
- **goal:** Sim-vs-anything comparison becomes a disciplined artifact
  instead of an eyeball: the front door for regression diffing,
  pseudo-flight work, and the LOCAL workflow.
- **fidelity_tier:** T1
- **depends_on:** [cross-doc: WP-21.1 (run records) preferred; standalone
  file mode otherwise]
- **new_crates:** **`openbmp-sysid` (L6)** — skeleton + placement
  justification first (§3.1).
- **touched:** `crates/openbmp-sysid/**(new)`, `crates/openbmp-cli`
  (subcommand), report schema, fixtures under `tests/fixtures/`.
- **approach:** §3.2; declared interpolation; fail-closed reconciliation;
  residual report canonical-ordered.
- **acceptance:**
  - no default-path effect; canonical goldens byte-identical
  - registration/resampling exact on constructed signals (property)
  - residual statistics match hand-computed fixtures < 1e-12 rel
  - mismatched units/frames fail closed
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — comparison tooling.
- **est_effort:** 2 weeks
- **parity_ceiling:** file-fed harness; real data LOCAL-only.

### WP-24.2 — Parameter registry + model hooks + closure tripwire

- **title:** Closed `EstimableParameter` vocabulary with bounds/priors/units, typed perturbation hooks in propulsion/aero/mass/sensor seams (off by default), vocabulary-closure tripwire.
- **goal:** "What may be estimated" becomes a reviewed, locked surface —
  vehicle-intrinsic by construction — and the model crates gain the
  perturbation seams estimation needs without behavior change.
- **fidelity_tier:** T2
- **depends_on:** [WP-24.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-sysid` (registry),
  `crates/openbmp-propulsion`/`-aero`/`-vehicle`/`-sensors` (gated hooks),
  tripwire in `crates/openbmp-testkit`.
- **approach:** §3.3; hooks as config-gated multipliers/increments with
  locked application order.
- **acceptance:**
  - hooks off by default; canonical goldens byte-identical
  - identity perturbation (θ = θ₀) is byte-identical to no-hook run
    (golden)
  - **tripwire:** vocabulary closure proven (new-variant attempt fails the
    declared test), in the same PR
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** active guardrail WP — closed vehicle-intrinsic
  vocabulary; the closure tripwire ships in this PR.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** declared parameter set; no free-form model morphing.

### WP-24.3 — Output-error estimator (GN/LM) on pseudo-flight

- **title:** Maximum-likelihood output error with FD sensitivities over parallel forward runs, declared iteration policy, covariance + whiteness reporting.
- **goal:** The core capability: withheld-truth recovery campaigns prove the
  estimator end to end on the full nonlinear stack.
- **fidelity_tier:** T2
- **depends_on:** [WP-24.2; cross-doc: WP-12.2-a (parallel fan-out),
  WP-11.0 (MC substrate)]
- **new_crates:** none.
- **touched:** `crates/openbmp-sysid` (estimator), pseudo-flight bundle
  generator, reconciliation report schema.
- **approach:** §3.4; central FD; GN with LM fallback; divergence is a
  reported failure; NEES/NIS reuse from `11`.
- **acceptance:**
  - no default-path effect; goldens byte-identical
  - linear analytic case matches closed-form LS + covariance < 1e-9 rel
  - pseudo-flight campaign: θ* recovered within 2·CRLB in ≥ declared
    fraction of seeds (tolerance table, `11` sizing)
  - estimator runs deterministic across worker counts
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** estimates only closed-registry, vehicle-intrinsic
  entries.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** offline batch estimation; no online/adaptive ID.

### WP-24.4 — Identifiability, CRLB & instrumentation planning

- **title:** Fisher information, Cramér-Rao bounds, correlation matrices, fail-closed unidentifiability gates, channel-set planning reports.
- **goal:** Honest estimation: every estimate ships with its theoretical
  floor, collinear parameter sets are refused with names, and the
  DFI-planning question — "which channels would make this estimable?" —
  becomes an offline analysis.
- **fidelity_tier:** T3
- **depends_on:** [WP-24.3]
- **new_crates:** none.
- **touched:** `crates/openbmp-sysid` (identifiability module), report
  fields, planning subcommand.
- **approach:** §3.5; condition gates with declared thresholds;
  planning = F recomputation over candidate channel sets.
- **acceptance:**
  - no default-path effect; goldens byte-identical
  - linear-case CRLB matches analytic Fisher inverse < 1e-9 rel
  - constructed collinear pair refused with named correlation (property)
  - planning report identifies the constructed discriminating channel
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** local (linearized) identifiability; no global
  structural-identifiability theory.

### WP-24.5 — Discipline recipes: propulsion, aero, mass, navigation

- **title:** Fixtured reconciliation workflows recovering thrust/Isp multipliers, per-Mach aero increments, dry-mass/CG deltas, and IMU errors, each shaped to its home doc's UQ vocabulary.
- **goal:** The Artemis-pattern per-discipline campaign, runnable in CI on
  pseudo-flights: estimates that land as `05`-band checks and `03`-overlay
  increments, plus the `09` closed-loop nav self-test.
- **fidelity_tier:** T3
- **depends_on:** [WP-24.4]
- **new_crates:** none.
- **touched:** `crates/openbmp-sysid` (recipes), scenario fixtures
  (`scenarios/phalcon9` family + `scenarios/sounding-rocket` for cheap
  cases), per-recipe tolerance tables.
- **approach:** §3.6; each recipe = registry subset + channel set +
  expected-recovery tolerances + report template.
- **acceptance:**
  - no default-path effect; goldens byte-identical
  - each recipe recovers its injected perturbation within its table
  - propulsion estimates outside `05` declared bands are flagged as
    model-form error (not silently accepted)
  - aero increments serialize in the `03` UQ-overlay shape
  - nav recipe matches `09` injected truth within declared bounds
  - all `13` §2 gates green
- **validation_label:** `validated-toy` → `research` (method anchors)
- **dual_use_note:** vehicle-intrinsic recipes only.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** pseudo-flight proof; real-data execution stays LOCAL.

### WP-24.6 — Anomaly timelines + fault-tree bookkeeping

- **title:** Deterministic change-point extraction over channel sets, cross-channel event-sequence assembly, timeline artifacts, fault-tree bookkeeping template.
- **goal:** The triage half of anomaly investigation: from thousands of
  channels to an ordered, evidence-linked timeline — supporting human
  root-cause work without pretending to do it.
- **fidelity_tier:** T4
- **depends_on:** [WP-24.1; shares detectors with WP-21.4]
- **new_crates:** none.
- **touched:** `crates/openbmp-sysid` (timeline module), artifact schema,
  bookkeeping template doc.
- **approach:** §3.7; CUSUM-class shared detectors; deterministic ordering
  and tie-breaking.
- **acceptance:**
  - no default-path effect; goldens byte-identical
  - constructed multi-channel fault scenario yields the known event order
    (fixture)
  - timeline artifact byte-stable; template renders via `21` booklets
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** triage tooling; no automated root cause.

### WP-24.7 — Governance pipeline + LOCAL-quarantine tripwire

- **title:** Candidate deck-increment artifacts (estimate + covariance + envelope + provenance chain + label), human-review entry path, tripwires rejecting real-data-derived increments and label inflation.
- **goal:** The loop closes safely: corrections re-enter decks as reviewed,
  pinned, labeled artifacts — and fitted-to-real-flight parameters are
  structurally non-committable.
- **fidelity_tier:** T4
- **depends_on:** [WP-24.5]
- **new_crates:** none.
- **touched:** `crates/openbmp-sysid` (export), increment schema +
  `provenance.md` integration, tripwires in `crates/openbmp-testkit`,
  `docs/external-telemetry-validation.md` cross-reference update.
- **approach:** §3.8; provenance-chain source classes; label flooring;
  `openbmp check-provenance` coverage.
- **acceptance:**
  - no default-path effect; goldens byte-identical
  - increment round trip: export → review-style ingest → deck overlay
    matches estimate exactly
  - **tripwire:** LOCAL/real-source increment rejected (structural test);
    label above evidence floor rejected
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** active guardrail WP — mechanizes the LOCAL quarantine
  and label honesty.
- **est_effort:** 2 weeks
- **parity_ceiling:** governance for this repo's decks; no claim about
  downstream forks.

---

## 10. References

- Maine, R. E., & Iliff, K. W., NASA RP-1168 — *Application of Parameter
  Estimation to Aircraft Stability and Control* (output-error/MMLE
  formulation).
- Klein, V., & Morelli, E. A., *Aircraft System Identification: Theory and
  Practice*.
- Artemis I reconstruction set: approach (NTRS 20205004519), ascent loads
  (NTRS 20230008005), base aerodynamics (NTRS 20240000053), base
  aerothermal (NTRS 20230018483).
- Shuttle propulsion performance reconstruction (NTRS 19890061688).
- Published launch-vehicle aerodynamic parameter-estimation case studies
  (Pegasus first-flight database update; AIAA JSR aero-ID literature).
- NASA-SpaceX propulsive-descent flight-reconstruction partnership overview
  (NTRS 20170008535).
- Docs `11`, `21`, `09` references for the substrate machinery.
