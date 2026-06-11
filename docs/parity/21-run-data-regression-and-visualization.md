# Run-Data Management, Regression Infrastructure & Visualization

**Status:** `experimental` (design intent; no code shipped by this document).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**Prerequisite reading:** `00-overview.md`, `13-agent-execution-playbook.md`,
`11-monte-carlo-uq-and-validation.md` and
`12-determinism-realtime-and-compute.md` (whose campaign outputs this
document stores, judges, trends, and renders).

> One-line scope: the connective tissue of a production sim org — a
> provenance-pinned run-record store and campaign index, a declarative
> post-run verdict engine with ensemble baseline bands and out-of-family
> detection, regression trend gates and a scoreboard, deterministic plot and
> report generation (Koviz-class comparison operations), 3D/timeline export
> (CZML/glTF) for external viewers, host-side shims for open ground tools
> (OpenMCT/Yamcs-class), and declarative campaign manifests for CI.

The audit pattern at every reference org is the same: simulation value comes
from the *data system around the runs*, not just the physics. SpaceX routes
all HOOTL/HITL/vehicle data into one telemetry store where automated checks
auto-pass >99.99% of channels and engineers see only flagged residue, with
per-change regression "verification events"; NASA's Trick ships TrickView,
Koviz (MC analysis, run-vs-run comparison, report booklets), TrickOps (run
orchestration), and DON (multi-user 3D review); JSC flies EDGE/DOUG graphics;
Ames maintains Open MCT. Docs `11`/`12` deliberately stopped at CSV/Parquet
emission — this document owns everything after the bytes land.

---

## 1. Parity target & ceiling

### 1.1 Reference practice

- **SpaceX** — unified telemetry store across simulation and flight;
  automated data review with historical-run overlay; regression verification
  events per software change (documented by practitioners); GNC-specific
  data visualization called out in simulation job postings.
- **NASA Trick ecosystem** — Koviz (Monte-Carlo data analysis, spike
  hunting, run-diff, plot booklets, video sync), TrickView (live variable
  browsing), TrickOps (CI/run orchestration), MonteCarloGeneration with full
  repeatability records, checkpoint/restart; DON v3 for distributed 3D
  review; EDGE/DOUG for real-time 3D graphics; Open MCT for telemetry
  display.
- **Relativity** — a staffed data-analysis-platform team turning raw
  telemetry into interactive time-series tooling *for* GNC engineers.
- **Rocket Lab** — 25,000-channel post-flight reviews as routine practice
  (the tooling implication is the point).

### 1.2 Parity target (capability, within posture)

1. **Run records & campaign index** — every run (single or MC sample) gets a
   canonical record: scenario content hash, seed, code version, gate
   results, channel-statistics digest, artifact paths — appended to a
   deterministic JSONL + Parquet index.
2. **Verdict engine** — declarative post-run checks (limits,
   tolerance-vs-golden, statistical bands, monotonicity, event-window
   assertions) extending the existing `openbmp-sil` checks; ensemble
   baseline bands with robust out-of-family detection (shared artifact
   schema with `18`'s LCC).
3. **Trend store & regression gates** — per-metric history across commits;
   deterministic drift detectors; a generated scoreboard document; CI gate
   hooks.
4. **Plots & report booklets** — deterministic SVG plot generation; MC
   percentile fans and overlays; run-diff and spike-hunt operations
   (Koviz-class); golden-stable report booklets.
5. **3D/timeline export** — CZML and glTF animation tracks (trajectory,
   attitude, events, ground sites) for external viewers (CesiumJS-class,
   DON/EDGE-class consumption); no in-repo renderer.
6. **Host-tool shims** — feature-gated, host-only adapters streaming runs to
   Open MCT (JSON/WebSocket shape) and Yamcs-class consumers (via `20`'s
   frame/dictionary exports); examples, not services.
7. **Campaign manifests** — declarative run matrices (TrickOps-shape) for
   CI and batch execution, binding `11`'s MC and `12`'s orchestration.

### 1.3 Parity ceiling (honest boundary)

- **No hosted service or GUI product.** Reference orgs run databases,
  web dashboards, and display walls; OpenBMP ships deterministic artifacts
  + standard-format exports + host-side shims. The interactive layer is
  external open software by design (posture: deterministic core, consumable
  edges).
- **No fleet/flight data.** The store holds simulation runs; real telemetry
  remains LOCAL-only per `docs/external-telemetry-validation.md` (doc `24`
  owns that workflow's analysis side).
- **No database engine.** The index is append-only canonical files (JSONL +
  Parquet), not a DBMS; "query" means reading Parquet. This keeps
  byte-discipline and provenance simple; a user can load the index into any
  external store.
- **Rendering ceiling.** Plot generation is publication-static (SVG);
  interactive exploration belongs to the external tools the exports feed.

---

## 2. Current state in source

- `crates/openbmp-telemetry/src/` — channelized Parquet/CSV/JSON writers
  with schema v1; the byte-diff gate runs on these outputs. No index above
  individual runs.
- `crates/openbmp-sil/src/{checks.rs,monitor.rs}` — verdict machinery
  (estimate-vs-truth monitors, honest pass/fail records) — the seed of the
  verdict engine.
- `crates/openbmp-cli` — `diff`, `check-provenance`, MC subcommands; golden
  scenario archive workflow in CI (`docs/verification.md`).
- `crates/openbmp-mc/src/lib.rs` — Welford/Clopper-Pearson reducers whose
  outputs need a home and a trend line.
- Criterion benchmarks produce HTML locally (dev-only); no deterministic
  plotting exists for results.
- No run index, no baseline bands, no trends, no scoreboard, no 3D export,
  no external-tool shims, no campaign manifests.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-rundata` (L7): consumes telemetry artifacts and scenario/provenance
metadata; produces records, verdicts, trends, reports, and exports. Host-side
by nature; never an `openbmp-fc` dependency; the runner gains only a thin
emission hook. Plotting uses a pinned pure-Rust SVG path (the `plotters` SVG
backend or equivalent reviewed in the skeleton PR) to keep byte-stable
artifacts.

### 3.2 Run records & campaign index (T1)

```rust
pub struct RunRecord {
    pub run_id: RunId,                 // hash(scenario_hash, seed, code_id)
    pub scenario_hash: Sha256,         // content pin (existing provenance machinery)
    pub seed: u64,
    pub code_id: CodeId,               // version + feature set; no wall-clock
    pub gates: GateResults,            // determinism/provenance/test outcomes as recorded
    pub channel_digest: ChannelDigest, // per-channel min/max/mean/rms/final + sample counts
    pub artifacts: Vec<ArtifactRef>,   // relative paths + SHA pins
    pub labels: Vec<RunLabel>,         // scenario family, campaign id, MC sample index
}
```

Canonical serialization (sorted keys, fixed float formatting) to
`runs.jsonl` + a Parquet mirror for bulk analysis. **No timestamps** in the
record itself (wall-clock breaks byte-discipline); wall metadata may live in
a sidecar explicitly excluded from determinism comparisons. The index is
append-only; campaign records reference member run ids. The channel digest
is computed in one pass with locked operand order (Welford from
`openbmp-mc`).

### 3.3 Verdict engine (T1–T2)

Declarative rule documents (TOML, `deny_unknown_fields`):

- **Rule forms:** channel limits (abs/rel), tolerance-vs-golden (reusing the
  tolerance-table convention of `docs/verification.md`), event-window
  assertions (event A within [t1,t2] of event B), monotonicity/sign
  invariants, statistical bands (mean/percentile within Clopper-Pearson
  bounds), and cross-run deltas (vs a named baseline run).
- **Baseline ensemble bands (T2):** per-channel robust bands
  (median ± k·MAD, percentile envelopes) generated from N seeded runs of a
  scenario family by a deterministic generator; pinned as artifacts with
  `provenance.md`; shared schema with `18`'s LCC out-of-family predicate
  (one band format, two consumers).
- **Semantics:** verdicts are typed (`pass`/`fail`/`skip` + measured value +
  bound), latched per run, fail-closed on unknown channels/rules; the run
  record stores the verdict set. The honest-verdict rule from
  `openbmp-sil` applies: a rule that cannot be evaluated is a failure, not a
  pass.

### 3.4 Trend store & regression gates (T2)

Per-metric history: `(code_id ordinal, scenario family, metric) → value`
appended at CI time from run records. Drift detection: deterministic
CUSUM/Page-Hinkley-class detectors with declared thresholds (no randomness),
plus simple step-change rules (k consecutive points beyond band). Outputs:
- a **scoreboard document** (generated Markdown table: per-family metric
  status, trend arrows, last-change code id) — committed artifact pattern
  like a golden;
- a **CI gate hook**: configurable fail-on-drift for declared metrics
  (e.g. orbit-insertion dispersion p95, clearance probability, runtime per
  sim-second).

### 3.5 Plots & report booklets (T3)

Deterministic SVG generation: time-series with event annotations, MC
percentile fans (p5/p50/p95 envelopes from `11` outputs), run-diff overlays
(A vs B with residual subplot), spike-hunt tables (top-k |Δ| windows between
runs — the Koviz operation), and booklet assembly (one document per
campaign: cover metadata, verdict summary, plot pages). Fixed fonts/layout,
canonical float formatting, no timestamps → golden-stable artifacts.

### 3.6 3D/timeline export (T4)

- **CZML:** trajectory packets (position/attitude over time in `08` frames),
  event intervals (staging, holds, blackout from `20`), ground sites and
  pass links — consumable by CesiumJS-class viewers out of the box.
- **glTF:** animation tracks (node TRS over time) for the vehicle assembly
  tree (per-body for `multi-body` scenarios), with a declared
  geometry-placeholder convention (real CAD is a user concern).
- Both deterministic, schema-validated in tests, and explicitly
  *export-only* (no viewer in-repo) — the DON/EDGE-class capability via the
  open ecosystem.

### 3.7 Host-tool shims (T4)

Feature-gated, host-only examples (documented as examples, excluded from the
determinism story): an Open MCT adapter (static JSON catalog + WebSocket
replay of a run record at declared speed) and a Yamcs-direction note (Yamcs
consumes `20`'s frame/dictionary exports directly). These prove the exports
against real open tools without making OpenBMP a service.

### 3.8 Campaign manifests (T5)

Declarative run matrices: scenario × seed-range × config overlays × expected
verdict rules, executed by the `12` orchestration substrate, recorded into
the index, judged by the verdict engine, trended by §3.4 — the TrickOps
shape. CI consumes manifests for the canonical regression set.

### 3.9 Fidelity tiers

- **T0 (current):** per-run telemetry files + golden diff + sil checks.
- **T1:** run records + index + verdict engine core — `validated-toy`.
- **T2:** baseline bands + out-of-family + trends + scoreboard + CI gate —
  `validated-toy`.
- **T3:** plots/booklets + run-diff/spike-hunt — `checked` (golden-stable).
- **T4:** CZML/glTF export + Open MCT shim — `checked`.
- **T5:** campaign manifests end-to-end — `validated-toy`.

---

## 4. Invariant preservation

- **Determinism:** canonical serialization everywhere; digests via locked
  single-pass reducers; detectors deterministic; plots byte-stable; the
  only wall-clock data lives in an excluded sidecar.
- **Byte-stable default:** emission hooks off by default
  (`[rundata]` block / CLI flags); existing outputs and goldens unchanged.
- **FC portability:** untouched — everything here is at or above L7.
- **Provenance:** baseline bands, manifests, and report goldens are pinned;
  run records embed the existing scenario content-pin machinery; artifact
  references carry SHAs.
- **Fail-closed:** unknown rules/channels, missing baselines, schema drift,
  and unverifiable rules are failures.
- **Labels:** infrastructure earns `checked`/`validated-toy`; it makes no
  physics claims of its own and must not launder upstream labels (verdict
  records carry the upstream artifact's label verbatim).

---

## 5. V&V plan

| Case | Type | Tier | Tolerance/criterion |
|---|---|---|---|
| Record canonicalization: same run → identical bytes; index append-only | golden/property | T1 | byte-identical |
| Channel digest vs direct statistics on the Parquet | internal code-to-code | T1 | exact (same reducer) |
| Verdict semantics per rule form | unit/property | T1 | exact; unverifiable ⇒ fail |
| Baseline-band generation determinism + detection/false-alarm rates on synthetic ensembles | statistical | T2 | rates within declared bounds (table) |
| Drift detectors on synthetic step/ramp/noise series | analytic | T2 | detection delay/false-alarm match closed-form expectations (table) |
| Scoreboard + booklet goldens | golden | T2/T3 | byte-identical |
| Run-diff/spike-hunt on constructed pairs | unit | T3 | exact top-k windows |
| CZML/glTF schema validation + round-trip spot checks | schema/golden | T4 | validates; byte-stable |
| Open MCT shim smoke (host-only, non-CI or CI-lite lane) | integration | T4 | catalog loads; replay streams declared channels |
| Manifest execution: N×M matrix recorded, judged, trended | scenario | T5 | deterministic across worker counts (with `12`) |


---

## 7. Dependencies on other parity docs

- `11` — MC outputs (samples tables, convergence traces, Wilks/CP
  statistics) are primary stored/trended/plotted content; statistical rule
  forms reuse its machinery.
- `12` — orchestration substrate executes manifests; order-independent
  reductions guarantee index determinism under parallel campaigns;
  checkpoint/resume interacts with record finalization.
- `18` — shares the baseline-band artifact schema (LCC out-of-family);
  rehearsal reports are run records.
- `19` — go/no-go report cards stored and trended (margin history).
- `20` — frame/dictionary exports feed Yamcs-class consumers; pass/link
  reports stored; CZML includes sites/passes/blackout intervals.
- `24` — consumes the index for sim-vs-flight diff workflows; its
  reconciliation reports are stored artifacts with their own schema.
- `10` — SIL determinism hashes recorded per run; fault-campaign results
  judged here.

---

## 8. Open-source leverage

| Tool/data | Use mode | License/status |
|---|---|---|
| Koviz / TrickView / TrickOps | method anchors for comparison ops, booklets, manifests | NASA open source |
| NASA Open MCT | external display target for the shim | Apache-2.0 |
| Yamcs | external consumer via `20` exports | AGPL-3.0 (external process) |
| OpenC3 COSMOS | named alternative consumer | AGPL-3.0 core (external) |
| CZML specification (CesiumJS) | export format | open spec; viewers external |
| glTF 2.0 | export format | Khronos open spec |
| plotters (SVG backend) | deterministic plot rendering (pinned) | MIT/Apache-2.0 |
| DON v3 / EDGE-DOUG public pages | reference architecture descriptions | public |
| Practitioner documentation of unified-store practice | method anchor for §3.3-§3.4 | public web |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-21.1 — `openbmp-rundata` skeleton: run records + campaign index

- **title:** Canonical `RunRecord`, channel digests, append-only JSONL+Parquet index, runner emission hook.
- **goal:** Every run becomes a durable, hashable, queryable fact — the
  substrate for verdicts, trends, and everything downstream.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-rundata` (L7)** — skeleton + placement
  justification first (§3.1).
- **touched:** `crates/openbmp-rundata/**(new)`, `crates/openbmp-runner`
  (emission hook), `crates/openbmp-cli` (subcommand), schema docs.
- **approach:** §3.2; canonical serialization; single-pass digests via
  `openbmp-mc` reducers; wall-clock only in the excluded sidecar.
- **acceptance:**
  - emission off by default; canonical goldens byte-identical
  - identical run → identical record bytes (golden); index append-only
    property test
  - digest matches direct statistics exactly
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — bookkeeping.
- **est_effort:** 2 weeks
- **parity_ceiling:** file-based index; no DBMS, no service.

### WP-21.2 — Verdict rule engine

- **title:** Declarative post-run checks (limits, tolerance-vs-golden, event windows, monotonicity, statistical bands, cross-run deltas) with fail-closed honest verdicts.
- **goal:** The >99.99%-auto-checked pattern: rules judge every run, humans
  read only failures, and "could not evaluate" is never a pass.
- **fidelity_tier:** T1
- **depends_on:** [WP-21.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (verdict module),
  `crates/openbmp-sil/src/checks.rs` (shared verdict types), rule schema
  docs, fixtures under `tests/fixtures/`.
- **approach:** §3.3 forms; typed verdicts with measured values; label
  passthrough verbatim.
- **acceptance:**
  - off by default; goldens byte-identical
  - per-form semantics exact under unit/property tests
  - unknown channel/rule and unevaluable rule ⇒ fail (property)
  - verdict report golden byte-stable
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; upstream labels pass through verbatim.
- **est_effort:** 2 weeks
- **parity_ceiling:** rule library tier; no anomaly ML.

### WP-21.3 — Baseline ensemble bands + out-of-family detection

- **title:** Deterministic band generation (median/MAD, percentile envelopes) from seeded run families; robust out-of-family rule form; shared artifact schema with doc 18.
- **goal:** "Is this run normal for its family?" becomes a computed verdict —
  the band artifact serves both post-run review here and `18`'s LCC
  out-of-family predicate.
- **fidelity_tier:** T2
- **depends_on:** [WP-21.2; cross-doc: WP-11.0 (MC substrate)]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (bands module), band artifact schema
  (+ `provenance.md` pattern), `data/` fixture bands.
- **approach:** §3.3; generator over N seeded runs; pinned artifacts;
  missing band ⇒ fail.
- **acceptance:**
  - off by default; goldens byte-identical
  - band generation deterministic (same family → same artifact hash)
  - detection/false-alarm rates on synthetic ensembles within declared
    bounds (tolerance table)
  - schema consumed by the `18` predicate stub without translation
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** simulation ensembles only; real flight data stays
  LOCAL.

### WP-21.4 — Trend store, drift gates & scoreboard

- **title:** Per-metric history across code ids, deterministic drift detectors, generated scoreboard document, CI fail-on-drift hook.
- **goal:** Regressions surface as trends, not surprises: the sim scoreboard
  every reference org maintains, in committed-artifact form.
- **fidelity_tier:** T2
- **depends_on:** [WP-21.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (trend module), scoreboard
  generation, `.github/workflows/ci.yml` hook (declared metrics), docs.
- **approach:** §3.4; CUSUM/Page-Hinkley-class with fixed thresholds;
  append-only history keyed by code-id ordinal.
- **acceptance:**
  - off by default; goldens byte-identical
  - detectors match closed-form delay/false-alarm expectations on synthetic
    series (tolerance table)
  - scoreboard golden byte-stable; CI hook fails on a constructed drift
    fixture
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** trend gates on declared metrics; no automatic root
  cause.

### WP-21.5 — Deterministic plots + report booklets (Koviz-class ops)

- **title:** SVG time-series/percentile-fan/run-diff/spike-hunt rendering and campaign booklet assembly, golden-stable.
- **goal:** The analysis artifacts engineers actually read — MC fans,
  A-vs-B residuals, top-k change windows — generated deterministically from
  the index.
- **fidelity_tier:** T3
- **depends_on:** [WP-21.1; WP-21.3 for fan/band overlays]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (render module; pinned plotters SVG
  backend), booklet schema, goldens under `tests/expected/`.
- **approach:** §3.5; fixed layout/fonts/format; spike hunt = top-k |Δ|
  windows with deterministic tie-breaking.
- **acceptance:**
  - off by default; goldens byte-identical
  - booklet and plot goldens byte-stable across runs and platforms on the
    declared profiles (with `12`)
  - spike-hunt exactness on constructed pairs
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** far from line.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** static rendering; interactivity belongs to external
  tools.

### WP-21.6 — CZML + glTF export

- **title:** Trajectory/attitude/event CZML packets and glTF animation tracks for the assembly tree, schema-validated, export-only.
- **goal:** Any run becomes viewable in the open 3D ecosystem
  (CesiumJS-class globes, glTF viewers) — the DON/EDGE-class capability
  without an in-repo renderer.
- **fidelity_tier:** T4
- **depends_on:** [WP-21.1; cross-doc: `08` frames; `20` for site/pass/
  blackout intervals where configured]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (export module), schema validation
  tests, example docs.
- **approach:** §3.6; deterministic packet/track generation; placeholder
  geometry convention documented.
- **acceptance:**
  - off by default; goldens byte-identical
  - exports validate against CZML/glTF schemas in tests; artifacts
    byte-stable
  - multi-body scenario exports per-body tracks (fixture:
    `scenarios/multi-body`)
  - all `13` §2 gates green
- **validation_label:** `checked`
- **dual_use_note:** far from line — rendering data for runs already
  governed by repo locks.
- **est_effort:** 2 weeks
- **parity_ceiling:** export-only; user supplies viewers and CAD.

### WP-21.7 — Open MCT shim (host example)

- **title:** Feature-gated host adapter: static catalog + WebSocket replay of a run record into NASA Open MCT.
- **goal:** Prove the exports against a real open mission-control display —
  the ops-display dimension at example tier, keeping OpenBMP service-free.
- **fidelity_tier:** T4
- **depends_on:** [WP-21.1; WP-20.8 helpful but not required]
- **new_crates:** none (example binary behind a feature, host-only).
- **touched:** `crates/openbmp-rundata` (example), docs (run instructions),
  CI-lite smoke lane (optional, non-blocking).
- **approach:** §3.7; replay at declared speed; explicitly outside the
  determinism story and excluded from goldens.
- **acceptance:**
  - feature off by default; no effect on any gate when disabled
  - smoke: catalog loads in Open MCT and declared channels stream (manual or
    CI-lite, documented)
  - all `13` §2 gates green
- **validation_label:** `experimental`
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** example shim; no supported service.

### WP-21.8 — Campaign manifests (TrickOps-shape)

- **title:** Declarative run matrices (scenario × seeds × overlays × rules) executed on the `12` substrate, recorded, judged, and trended end-to-end.
- **goal:** One document runs the canonical regression campaign: the CI
  entry point that ties the whole data system together.
- **fidelity_tier:** T5
- **depends_on:** [WP-21.2, WP-21.4; cross-doc: WP-12.2-a (parallel
  fan-out)]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (manifest module),
  `crates/openbmp-cli` (campaign subcommand), CI integration, manifest
  fixtures.
- **approach:** §3.8; manifest → run set → records → verdicts → trends;
  deterministic across worker counts.
- **acceptance:**
  - off by default; goldens byte-identical
  - N×M fixture campaign produces identical index/verdict/trend artifacts
    across 1/4/16 workers (with `12` reductions)
  - failing rule in the matrix fails the campaign verdict (and the CI hook)
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 2 weeks
- **parity_ceiling:** batch manifests; no scheduler/cluster manager.

---

## 10. References

- NASA Trick documentation: Koviz, TrickView, TrickOps,
  MonteCarloGeneration (github.com/nasa — open source).
- DON v3 (KSC, NASA Software Catalog KSC-13775); DOUG/EDGE public
  documentation (JSC).
- NASA Open MCT (github.com/nasa/openmct).
- Practitioner accounts of unified telemetry stores and automated data
  review in launch-vehicle test programs (public engineering posts).
- CZML specification (CesiumJS project); glTF 2.0 specification (Khronos).
- CUSUM / Page-Hinkley change-detection literature (standard references).
- Docs `11`/`12` references for the statistical and orchestration
  machinery this document consumes.
