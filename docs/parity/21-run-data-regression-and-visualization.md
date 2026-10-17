# Run-Data Management, Regression Infrastructure & Visualization

**Status:** `experimental` (design intent; WP-21.1 through WP-21.8 implemented).
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
  with schema v1; the byte-diff gate runs on these outputs. The run-data
  layer now indexes these artifacts when explicitly requested.
- `crates/openbmp-sil/src/{checks.rs,monitor.rs}` — verdict machinery
  (estimate-vs-truth monitors, honest pass/fail records) — the seed of the
  verdict engine.
- `crates/openbmp-cli` — `diff`, `check-provenance`, MC subcommands, and
  off-by-default `openbmp run --run-index-jsonl/--run-index-parquet`
  emission; golden scenario archive workflow in CI (`docs/verification.md`).
- `crates/openbmp-mc/src/lib.rs` — Welford/Clopper-Pearson reducers whose
  outputs need a home and a trend line.
- Criterion benchmarks produce HTML locally (dev-only), separately from the
  deterministic SVG and Markdown artifacts produced by `openbmp-rundata`.
- `crates/openbmp-rundata` now provides the WP-21.1 file-index substrate:
  deterministic `RunRecord`, artifact/gate/label records, per-channel
  telemetry digests, canonical JSONL bytes, append-only JSONL index helpers
  with duplicate-run-id rejection, and deterministic Parquet index mirrors.
  Index persistence lives in the private `src/index.rs` module; `RunIndex`
  and `append_record_jsonl` remain exported from the crate root.
  It also has the WP-21.2 verdict-rule core: closed TOML rule documents for
  limits, tolerance-vs-golden, monotonicity, event windows, statistical bands,
  and cross-run deltas, evaluated fail-closed over telemetry tables and
  optional baseline records with byte-stable JSON reports and off-by-default
  CLI emission. The WP-21.3 baseline-band substrate can generate
  deterministic median/MAD and percentile-envelope JSON artifacts from
  `RunRecord` families and evaluate candidate run records against the shared
  band schema. The WP-21.4 trend substrate aggregates per-metric histories by
  scenario family and `code_ordinal`, runs deterministic Page-Hinkley drift
  checks, emits a byte-stable Markdown scoreboard, and exposes a CI
  fail-on-drift helper covered by a constructed drift fixture. The WP-21.5
  render substrate produces fixed-layout SVG time-series plots, percentile
  fans, run-diff residual plots, deterministic top-k spike-hunt reports, and
  Markdown report booklets with committed goldens. The first WP-21.6 export
  substrate emits deterministic CZML-shaped JSON and glTF JSON artifacts with
  embedded animation buffers/accessors from declared body tracks, event
  intervals, and ground sites with committed goldens and local structural
  validators; full external schema conformance and runner/multi-body fixture
  wiring remain open. The WP-21.7 Open MCT substrate
  emits deterministic static catalog JSON and replay-frame JSON for declared
  numeric telemetry channels, and a feature-gated host example serves the
  generated catalog plus replay frames through loopback HTTP/WebSocket requests
  with a CI-lite WebSocket smoke. The WP-21.8 campaign substrate
  parses deterministic TrickOps-shaped manifests, expands scenario/seed/overlay
  matrices, shards run specs across worker counts, aggregates verdict reports
  fail-closed, exposes `openbmp campaign plan` to write deterministic plan and
  optional worker-shard JSON, and exposes `openbmp campaign run` for bounded
  worker execution of scenario x seed cases with optional overlay TOML files in
  append or parsed merge mode.
  The executor applies manifest seeds to parsed scenarios, materializes combined
  overlay scenarios per run, writes per-run telemetry, merges per-run local
  indexes into a shared run index in expanded-plan order, optionally writes a
  Parquet mirror, writes per-run verdict JSON, trend artifacts, combined
  verdict/trend CI-failure summaries, and an aggregate summary. Overlay files
  default to byte-compatible append mode; `mode = "merge"` parses base and
  overlay TOML with `toml_edit`, recursively merges tables, appends matching
  arrays of tables, and rejects invalid merge overlays fail-closed.

---

## 3. Target architecture

### 3.1 Crate boundary

`openbmp-rundata` (L7): consumes telemetry artifacts and scenario/provenance
metadata; produces records, verdicts, trends, reports, and exports. Host-side
by nature; never an `openbmp-fc` dependency; the runner gains only a thin
emission hook. The current `render` module writes SVG directly using
`std::fmt::Write` with fixed layout and numeric formatting. A replacement
rendering backend would need to preserve the declared byte-stability contract;
`plotters` is not a current dependency of this crate.

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
- **Byte-stable default:** run-data emission hooks are off by default
  (`openbmp run` CLI flags); existing outputs and goldens unchanged.
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
| plotters (SVG backend) | considered alternative; current renderer emits SVG directly | MIT/Apache-2.0 |
| DON v3 / EDGE-DOUG public pages | reference architecture descriptions | public |
| Practitioner documentation of unified-store practice | method anchor for §3.3-§3.4 | public web |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the `13` §2 gate set.

### WP-21.1 — `openbmp-rundata` skeleton: run records + campaign index

- **title:** Canonical `RunRecord`, channel digests, append-only JSONL+Parquet index, runner emission hook.
- **implementation_status:** implemented. `crates/openbmp-rundata` exists as
  the L7 run-data crate and exposes deterministic `RunRecord`, `ArtifactRef`,
  `GateResult`, `RunLabel`, `ChannelDigest`, `NumericChannelDigest`,
  `channel_digests()`, `RunIndex`, `append_record_jsonl()`, and deterministic
  Parquet index mirrors. `openbmp run` now has off-by-default
  `--run-index-jsonl`, `--run-index-parquet`, and `--run-code-id` emission
  hooks that build records from the runner telemetry table and written
  telemetry artifacts. Tests prove identical records produce identical JSONL
  bytes and record hashes, channel digests match direct row-order statistics,
  JSONL appends preserve the existing prefix, unterminated existing indexes
  fail closed, duplicate run ids are rejected, malformed SHA-256 pins are
  rejected, Parquet mirrors are byte-stable, and a real CLI run emits
  parseable JSONL plus a matching Parquet mirror.
- **goal:** Every run becomes a durable, hashable, queryable fact — the
  substrate for verdicts, trends, and everything downstream.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** **`openbmp-rundata` (L7)** — skeleton + placement
  justification first (§3.1).
- **touched:** `crates/openbmp-rundata/**(new)`, `crates/openbmp-cli`
  (`openbmp run` emission flags and report surface), schema docs. The runner
  core remains unchanged because `runner::run()` already returns the telemetry
  table required for record construction.
- **approach:** §3.2; canonical serialization; single-pass digests via
  `openbmp-mc` reducers; wall-clock only in the excluded sidecar.
- **acceptance:**
  - emission off by default; canonical goldens byte-identical
  - identical run → identical record bytes (golden); index append-only
    property test
  - digest matches direct statistics exactly
  - JSONL and Parquet run-index outputs are emitted only when requested and
    are parseable/byte-stable
  - all `13` §2 gates green
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line — bookkeeping.
- **est_effort:** 2 weeks
- **parity_ceiling:** file-based index; no DBMS, no service.

### WP-21.2 — Verdict rule engine

- **title:** Declarative post-run checks (limits, tolerance-vs-golden, event windows, monotonicity, statistical bands, cross-run deltas) with fail-closed honest verdicts.
- **implementation_status:** implemented. `openbmp-rundata::verdict` exposes
  closed TOML rule documents (`VerdictRuleDocument`) and typed rule variants
  for limits, tolerance-vs-golden, monotonicity, event windows, statistical
  bands, and cross-run deltas. It emits stable `VerdictReport` /
  `VerdictRecord` JSON, evaluates over `TelemetryTable` plus optional
  baseline `RunRecord`, rejects malformed/duplicate/unknown-field rule
  documents, and fails closed for unknown channels, non-numeric metrics, absent
  events, and missing baselines. `openbmp run` now has off-by-default
  `--run-verdict-rules`, `--run-verdict-json`, and optional
  `--run-verdict-baseline-jsonl` flags that write deterministic verdict-report
  JSON after a real run; missing rule/output flag pairs fail closed before
  scenario loading. Unit tests, a checked TOML fixture, and CLI end-to-end
  coverage exercise the current path, including generated property checks for
  inclusive bounds, monotonic samples, and unknown-channel fail-closed behavior
  plus a committed verdict-report JSON golden. `openbmp-sil` remains
  tap-specific; the shared contract is the stable pass/fail/skip verdict
  vocabulary rather than a new dependency edge.
- **goal:** The >99.99%-auto-checked pattern: rules judge every run, humans
  read only failures, and "could not evaluate" is never a pass.
- **fidelity_tier:** T1
- **depends_on:** [WP-21.1]
- **new_crates:** none.
- **touched:** `crates/openbmp-rundata` (verdict module),
  `crates/openbmp-cli` (off-by-default report emission flags), rule schema
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
- **implementation_status:** implemented. `openbmp-rundata::bands` exposes
  `BaselineBandSpec`, `BaselineBandArtifact`, `BaselineBand`,
  `OutOfFamilyFinding`, `generate_baseline_band_artifact()`, and
  `evaluate_out_of_family()`, plus JSONL-index generation and a doc-18-style
  launch-commit out-of-family predicate that consumes the same artifact schema
  without translation. The artifact schema carries stable family id, sample
  count, channel/metric bands, median, MAD, percentile envelope, and MAD
  envelope fields; generation sorts specs deterministically, rejects empty
  inputs, bad percentiles, and missing/non-numeric metrics, and serializes to
  stable JSON with committed crate and `data/rundata/` golden fixtures. Tests
  prove deterministic artifact bytes, JSON round-trip, linear percentile
  interpolation, direct digest consumption without telemetry tables,
  append-only JSONL run-index generation, shared-schema out-of-family
  evaluation, doc-18 predicate fail-closed behavior, and synthetic
  false-alarm/detection-rate tolerance evidence.
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
- **touched:** `crates/openbmp-rundata/src/render.rs` (direct SVG emission),
  booklet schema, goldens under `crates/openbmp-rundata/tests/fixtures/`.
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
- **implementation_status:** implemented and traced by `REQ-RUNDATA-006` /
  `V-RUNDATA-006`: `openbmp-rundata::export3d` exposes `SceneExport`,
  `BodyTrack`, `KinematicSample`, `TelemetryBodyTrackSpec`, `EventInterval`,
  `GroundSite`, `generate_czml()`, `generate_gltf_json()`,
  `scene_export_from_telemetry_table()`, `validate_czml_export()`, and
  `validate_gltf_export()`. It emits deterministic CZML-shaped trajectory,
  attitude, event, and site packets, plus deterministic glTF 2.0 JSON with
  embedded base64 animation buffers, bufferViews, accessors, animation
  samplers/channels, and OpenBMP per-body track extras. The export path sorts
  tracks/events/sites deterministically, validates empty tracks, empty samples,
  non-finite values, duplicate/decreasing sample times, invalid intervals,
  unknown telemetry channels, missing samples, non-numeric samples, and
  malformed generated documents fail-closed, checks committed JSON goldens,
  proves per-body track ordering on a constructed multi-body fixture, and
  exports the real runner output from
  `scenarios/multi-body/initial-formation.toml` into validated CZML/glTF
  primary-plus-observer tracks. `scripts/check_export3d_schemas.py` downloads
  pinned upstream Khronos glTF 2.0 and CZML writer schema trees and validates
  the committed CZML/glTF goldens through AJV.
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
- **implementation_status:** implemented and traced by `REQ-RUNDATA-007` /
  `V-RUNDATA-007`: `openbmp-rundata::openmct` emits deterministic static
  catalog JSON for numeric telemetry channels and deterministic replay-frame
  JSON for declared numeric channels at a finite positive replay speed.
  `OpenMctCatalog::from_json_bytes()` round-trips catalog artifacts, and
  helpers reject empty namespaces, empty replay channel lists, invalid replay
  speed, unknown channels, missing samples, and non-numeric samples fail-closed.
  The opt-in `openmct-host` feature exposes `openmct_host` helpers for serving
  `GET /catalog.json` and WebSocket `GET /replay` requests from already
  generated catalog/replay JSON artifacts. The example binary is built only
  with `--features openmct-host`:
  `cargo run -p openbmp-rundata --features openmct-host --example
  openmct_replay_server --
  crates/openbmp-rundata/tests/fixtures/openmct-catalog.json
  crates/openbmp-rundata/tests/fixtures/openmct-replay.json
  127.0.0.1:8765 1`, then configure Open MCT to load
  `http://127.0.0.1:8765/catalog.json` and subscribe to
  `ws://127.0.0.1:8765/replay`. Tests compare committed catalog/replay JSON
  goldens, check the RFC WebSocket accept key, prove catalog HTTP serving, run a
  loopback WebSocket replay smoke, and compile-check the feature-gated example.
  This remains an example shim rather than a supported OpenBMP service.
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
- **implementation_status:** implemented and traced by `REQ-RUNDATA-008` /
  `V-RUNDATA-008`: `openbmp-rundata::campaign` parses closed TOML campaign
  manifests, expands scenario x seed x overlay matrices into stable run specs,
  serializes deterministic plan JSON, partitions run specs into deterministic
  round-robin shards, and aggregates per-run verdict reports so any failed run
  fails the campaign summary. The CLI exposes `openbmp campaign plan`, which
  reads a manifest, writes the deterministic plan JSON, optionally writes worker
  shard JSON, rejects zero-worker shard requests fail-closed, and is covered by a
  binary smoke test plus the top-level help snapshot. The CLI also exposes
  `openbmp campaign run`: it writes `plan.json`, runs each scenario x seed case
  through a bounded worker pool with per-run telemetry under `runs/<run_key>/`,
  applies selected manifest-relative `[[overlay_files]]` rows, materializes a
  combined per-run `scenario.toml` artifact, appends a shared `run-index.jsonl`,
  optionally writes `run-index.parquet`, writes per-run `verdict.json` files
  when rules are declared, writes `summary.json`, aggregates verdicts, and
  rejects selected overlays without files fail-closed. Overlay files default to
  `mode = "append"` for legacy fragment behavior; `mode = "merge"` parses base
  and overlay TOML with `toml_edit`, recursively merges tables, appends matching
  arrays of tables, materializes the merged scenario, and rejects malformed
  merge overlays fail-closed.
  When the manifest declares a manifest-relative `trend_specs` TOML document,
  the executor parses its `[[metrics]]` rows as `TrendMetricSpec`, evaluates the
  completed `run-index.jsonl`, writes `trend-store.json`, `drift-report.json`,
  and `trend-scoreboard.md`, and records stable `ci_failures` strings in
  `summary.json`. The campaign summary also carries combined verdict/trend
  `ci_failures`; `openbmp campaign run --fail-on-ci-gates` writes all artifacts
  first and then exits non-zero when those gates are nonempty, with a CI job
  exercising the failing-verdict path. `openbmp campaign run --workers N`
  executes run specs through a bounded worker pool, writes one local
  `run-index.jsonl` per run directory, merges records into the campaign
  `run-index.jsonl` in expanded-plan order, and has regression coverage proving
  plan, index, Parquet, summary, verdict, trend, and per-run telemetry artifact
  bytes are identical across 1/4/16 workers. The implementation intentionally
  stops short of keyed or semantic patch operations beyond recursive table
  merge and matching array-of-tables append.
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
