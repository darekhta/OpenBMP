# Monte Carlo, UQ & Validation

**Status:** `experimental` / partial implementation trace.
**Audience:** the engineer or LLM agent implementing the parity work packages
for the certification-scale Monte Carlo / UQ / V&V dimension.
**One-line summary:** Build a reusable, byte-deterministic Monte Carlo +
uncertainty-quantification + verification-and-validation substrate — DoE
(LHS / Sobol+Owen / Iman-Conover / Wilks sizing), NASA-STD-7009B credibility
accounting, rare-event *methods* on synthetic limit-states, MMS / order-of-
accuracy / Richardson-GCI code verification, and BET reconstruction against
*open* telemetry — that wires the existing per-discipline uncertainty into
honest, academic-scope probabilistic statements without ever asserting a real
vehicle's reliability.

> Prerequisite reading: `00-overview.md` (parity definition, the
> solver-consumer posture §1.1, the ten invariants §3, the crate map §4, the
> phased DAG §5, the forward-only escalation §6) and
> `13-agent-execution-playbook.md` (the §4 template this doc follows, the §5
> work-package schema, the §2 gate set). This is the dimension that turns every
> *other* dimension's per-entry uncertainty into a campaign result; read it
> after the physics docs it consumes.

---

## 1. Parity target & ceiling

### 1.1 Capability target

Bring OpenBMP's dispersion / UQ / V&V machinery to **method-and-architecture
parity** with the dispersion decks used in production launch-vehicle and GNC
practice — the POST2 / Dakota / SALib / Basilisk-MC class of tooling — *within
the OpenBMP posture*: open, simulation-only, byte-deterministic, and
academic-scope-labelled.

Concretely, "approaching parity" for this dimension means:

- **Design of experiments.** Replace naive i.i.d. draws with space-filling and
  low-discrepancy designs — Latin Hypercube Sampling (LHS), Sobol nets with
  Joe–Kuo direction numbers and Owen scrambling for randomized QMC (RQMC),
  Iman–Conover rank-correlation induction — and a Wilks sample-size calculator
  so a scenario can *request* a tolerance-interval requirement
  ("99.865 % coverage at 90 % confidence") and be *told* the required N.
- **Statistical rigour.** Every campaign reports a confidence interval on the
  binary mission outcome (Clopper–Pearson), running mean/variance with a
  convergence diagnostic (Welford + MC standard error), cross-chain stability
  (Gelman–Rubin R̂), and distribution-free tolerance intervals from order
  statistics (Wilks) — not a bare pass/fail count.
- **Credibility accounting.** Promote the home-grown four-level
  `ValidationStatus` proxy to the NASA-STD-7009B eight-factor 0–4
  Capability/Results credibility matrix, keep the minimum-is-binding reducer,
  and *wire* the rewired error budget into the MC driver so every campaign emits
  a 7009B-vocabulary credibility record tied to evidence, fail-closed below a
  configured floor.
- **Uncertainty propagation.** Rewire the error budget from flat
  `RSS + |bias|` to a source-tagged correlation-matrix aggregate
  `√(sᵀ Σ s)` with explicit aleatory/epistemic tags, and a nested
  (double-loop) driver producing a probability box and an aleatory/epistemic
  variance split — the structure the current monolithic `correlated_bias`
  cannot express.
- **Rare-event *methods*.** Subset Simulation (Au–Beck) and cross-entropy
  importance sampling as a *methodology demonstration* on synthetic limit-states
  with analytically-known failure probabilities — never a real P(LOC)/P(LOM).
- **Code verification.** A Method-of-Manufactured-Solutions + observed-order-of-
  accuracy + Richardson/GCI harness over the integrator family and physics
  RHS, plus a code-to-code export adapter (Orekit/GMAT/Basilisk/Dakota).
- **Validation.** A Best-Estimated-Trajectory reconstruction half (batch
  Gauss–Newton + RTS smoother + NEES/NIS consistency) validated on synthetic-
  truth models, and *locally* cross-validated against open Falcon-9-class
  webcast telemetry per the existing external-telemetry policy.

### 1.2 The honest parity ceiling

Five things cannot be matched in an open academic repo, and the design states
each plus its open substitute (this is a hard requirement, `00` §1):

1. **Calibrated absolute P(LOC)/P(LOM).** Real flight-reliability probabilities
   require proprietary component failure-rate databases, qualification-test
   data, and flight heritage that no open dataset provides. OpenBMP demonstrates
   the *methodology* (Subset Simulation, CE-IS) on **synthetic limit-states with
   analytically-known answers** and reports convergence/CoV honestly; it must
   **never** present the resulting probability as a real vehicle's reliability.
   The crate names every such output `synthetic-limit-state` and refuses to bind
   it to a named vehicle.
2. **Validated aerodynamic / aerothermal uncertainty bands.** Those come from
   wind-tunnel and CFD-vs-test correlation that is proprietary / export-
   controlled. Open substitute: published benchmark cases plus model-form bands
   declared honestly as *epistemic* (the bands flow in from `03-…`, `04-…`,
   `05-…` as ingested per-entry UQ, never invented here).
3. **A real Best-Estimated-Trajectory.** Genuine BETs use telemetry, radar, and
   onboard IMU/GPS data that are not public. Open substitute: reconstruction
   from scraped webcast telemetry treated as a *LOCAL-only sanity comparison*,
   never ground truth, with NEES/NIS consistency on synthetic-truth models
   providing the rigorous filter check.
4. **Certification / accreditation.** AIAA G-077 is a *guide*, not a cert
   standard; NASA-STD-7009B credibility scoring is achievable as a
   *self-assessment* but a real flight-cert authority's sign-off is permanently
   out of scope by design (`docs/standards-posture.md`).
5. **GPU / exascale parity.** Production decks run 10⁶ runs on lab clusters; an
   open repo reaches 10⁵–10⁶ *deterministic CPU* runs but should **not** chase
   bit-reproducible f64 GPU parity (a known determinism hazard). The credible
   substitute is deterministic CPU parallelism + checkpointing + QMC variance
   reduction cutting the required N (the heavy lifting lives in
   `12-determinism-realtime-and-compute.md`).

The honest framing throughout: OpenBMP validates **methods** against analytic,
code-to-code, and MMS references and labels every aggregate academic-scope —
exactly as `crates/openbmp-physics/src/uq.rs` already does, generalised and
wired in.

---

## 2. Current state in source

Verified against the working tree (paths are load-bearing; the agent must
regress against them).

### 2.1 The deterministic RNG substrate — exists, is the foundation

`crates/openbmp-core/src/rng.rs` provides `DeterministicRng` wrapping
`rand_chacha::ChaCha8Rng`. It implements the counter-based, domain-separated
keying philosophy the research pack calls for: constructors derive a 32-byte
seed from `(scenario_seed, step_index, channel/sensor/effector/wind/stimulus id,
component_id)` with a four-byte domain tag (`b"SENS"`, `b"EFFC"`, `b"WIND"`,
`b"STIM"`, and MC `b"MCRN"`) in bytes `[28..32]` to guarantee non-collision
across stream families. There are *pinned reference-stream* tests
(`for_*_reference_stream_is_locked`) that fail on any drift.
`for_mc_sample(campaign_seed, sample_index, dimension_id)` now gives each MC
sample a stream that is independent of worker count and completion order.

### 2.2 The Phalcon-9 orbit Monte Carlo — ported to the reusable MC substrate

`crates/openbmp-cli/tests/phalcon9_orbit_monte_carlo.rs` is the N=16 demo. It:

- declares `const SAMPLES: u64 = 16;`
- uses `openbmp_mc::{SampleRng, ScalarOutcome, run_scalar_campaign}` rather than
  an inline PRNG;
- disperses Isp/thrust (per stage), dry mass (per body), propellant fill (per
  tank, underfill-only), initial ECI state, and a constant NED wind, all with
  hard-coded 1-σ figures;
- keeps the existing envelope assertions, but the campaign report now carries
  ordered samples, Welford perigee statistics, a convergence trace, and a
  Clopper-Pearson interval for the sustainable-insertion fraction;
- has a separate determinism test that re-runs sample 3 and compares
  `to_bits()`.

This is now the canonical caller for `WP-11.0`: the *physics* dispersions stay
unchanged while the *statistics* surface uses `openbmp-mc`.

### 2.3 Propulsion fault-library campaigns — exists for scheduled engine faults

`crates/openbmp-mc/src/lib.rs` exposes `PropulsionFaultLibrary`,
`PropulsionFaultTemplate`, and `PropulsionFaultOverlay` for deterministic
scheduled propulsion-fault materialization. Each template has a validated
activation probability and engine-fault payload; `materialize()` keys draws by
`SampleRng(campaign_seed, sample_index, dimension_id)` and emits an appendable
`[propulsion.faults]` TOML fragment. The regression parses the fragment through
`openbmp-scenario`, so the normal scenario validator remains the authority for
engine ids and payload schema.

`openbmp mc propulsion-faults` now provides the end-to-end campaign path for
this library shape: it reads a base scenario and a `[[faults]]` manifest,
materializes one overlay per sample, parses each mutated scenario through the
normal validator, executes the standard runner, reads a terminal telemetry
channel, writes `sample_index,fault_ids,activated_fault_count,value,success`
CSV rows, and returns Welford plus Clopper-Pearson summaries. The same library
manifest can include `[uq]` with `budget_toml`, `credibility_floor`, and
`report_md`; paths resolve relative to the library file and the campaign
fail-closes below the configured floor before sample CSV evidence is accepted.
This is a scoped propulsion-fault campaign runner; checkpointed multi-worker
orchestration remains separate MC work.

### 2.4 The footprint Monte Carlo — the correctly-shaped precedent

`crates/openbmp-runner/src/footprint.rs` and
`crates/openbmp-cli/src/commands/footprint_mc.rs` already do it the right way:
they use `openbmp_core::DeterministicRng`, are scenario-gated by
`[landing_footprint.monte_carlo]`, emit a *samples table* and a summary through
the telemetry/CSV interface, and the module header is explicit that it "never
produces an actuator command" — range-safety post-processing only. `openbmp-mc`
should generalise *this* interface (sample table + summary writer + scenario
gate), not the orbit test's bespoke shape. The footprint MC is the proof that
the project already has the right boundary; the new crate makes it reusable.

### 2.5 The UQ error budget — L1 substrate plus MC campaign wiring

`crates/openbmp-uq/` now owns the first-class uncertainty substrate called for
by WP-11.2 and is traced by `REQ-UQ-001` / `V-UQ-001`. It exposes:

- legacy-compatible `ErrorBudget` /
  `UncertaintyContribution { model_id, one_sigma, status, justification }`,
  preserving the historical `√(Σ σᵢ²) + |correlated_bias|` aggregate for callers
  that still consume the flat RSS shape;
- `CorrelatedErrorBudget`, `UncertaintySource`, `UncertaintyClass`, and
  `CorrelationMatrix` for source-tagged `√(sᵀΣs)` aggregation with explicit
  aleatory/epistemic sub-aggregates;
- deterministic correlation-matrix validation: finite entries, unit diagonal,
  symmetry, bounds, and positive-semidefinite rejection;
- `CredibilityRecord` over the NASA-STD-7009B-shaped eight factors, with absent
  factors binding to L0, factor-wise minimum reduction across sources, and a
  `legacy_label()` mapping back to OpenBMP's public validation labels.

`crates/openbmp-physics/src/uq.rs` is now a compatibility re-export shim, so the
duplicate `ValidationStatus` enum is gone and the UQ path uses the canonical
`openbmp_core::ValidationStatus`.

The first MC-facing consumption paths are also wired under `REQ-UQ-002` /
`V-UQ-002`: `openbmp mc summarize`, `openbmp footprint-mc`, and
`openbmp mc propulsion-faults` can read a UQ budget with `--uq-toml`, enforce a
configured `--credibility-floor`, and optionally write a deterministic Markdown
evidence report with `--credibility-report-md`. `openbmp footprint-mc` also
consumes the scenario-owned `[landing_footprint.monte_carlo.uq]` declaration
with `budget_toml`, `credibility_floor`, and `report_md`; `openbmp mc
propulsion-faults` consumes the same fields from the fault-library `[uq]`
manifest table. Those paths use
`openbmp-mc::evaluate_campaign_credibility` over the same
`CorrelatedErrorBudget`, preserve ordinary reports when the UQ TOML is omitted,
and fail-close if the binding 7009B level is below the floor. The remaining
WP-11.2 gap is requiring future MC campaign drivers to expose the same manifest
UQ contract automatically.

### 2.7 The testkit — the V&V plumbing already exists

`crates/openbmp-testkit/`:

- `src/analytic.rs` — closed-form solutions (constant-acceleration drop,
  harmonic oscillator, two-body Keplerian) — the seed material for the MMS
  manufactured states and the order-of-accuracy reference solutions.
- `ManufacturedScalarOde` — smooth MMS reference
  `y(t)=0.75+sin(0.3t)+0.05t^3` with `dy/dt=-lambda*y+source(t)` and
  `source(t)=y_exact'(t)+lambda*y_exact(t)` for data-free integrator code
  verification.
- `src/verification.rs` — `StepError`, `OrderVerificationReport`, and
  `order_verification_report` for observed order, Richardson extrapolation,
  and Roache GCI summaries over `h, h/2, h/4` studies.
- `src/reconstruction.rs` — pure linear-Gaussian NEES/NIS, RTS smoother, and
  batch Gauss-Newton helpers for synthetic reconstruction evidence without an
  `openbmp-fc` dependency edge.
- `src/tolerance.rs` — the `expected.toml` parser (`ToleranceTable { case,
  source, validation, metrics }`, `MetricTolerance { name, expected,
  absolute_tolerance, relative_tolerance }`) per `docs/verification.md §
  Tolerance Tables`. Every new V&V case in this dimension ships one.
- `src/determinism.rs` — byte-stable diff + replay utilities (the determinism
  oracle the campaign-level byte-diff gate extends to ensemble statistics).
- `tests/inline_data_tripwire.rs`, `tests/fc_dependency_tripwire.rs`,
  `tests/ballistic_state_compile_fail.rs` — the four-pillar provenance, FC
  portability, and forward-only locks every WP keeps green.

### 2.7 Requirements & verification ladder

`requirements.toml` carries the machine-readable requirement set
(`REQ-…`/`V-…`); `docs/verification.md` defines the five-layer ladder (MMS →
model-verification vs CEA/correlations → code-to-code → public benchmark → UQ
reporting), the four validation labels, the tolerance-table contract, the
determinism gate, and the NASA-STD-7009B / AIAA G-077 vocabulary posture. The
`monte-carlo-nightly` CI lane already exists for the footprint and Phalcon-9
seeded dispersion checks — this is where the campaign-scale V&V cases land.

**Net assessment.** The substrate (DeterministicRng), the correctly-shaped
precedent (footprint MC), the `openbmp-uq` correlated error-budget substrate,
the V&V plumbing (testkit), and the reporting shape all exist. What is missing
is (a) broad scenario-level campaign orchestration and checkpointing beyond the
current reusable scalar and propulsion-fault paths, (b) automatic 7009B sidecar
adoption for future campaign drivers, (c) reconstruction / NEES-NIS /
code-to-code export, and (d) campaign-scale cross-architecture determinism
(deferred to `12-…`). This is a *partial → approaching* program.

---

## 3. Target architecture

Two new crates, plus extensions to `DeterministicRng`, the runner, and the CLI.

```text
NEW:
  openbmp-uq    L1  promote uq.rs: source-tagged correlation-matrix error budget,
                    aleatory/epistemic tags, p-box, NASA-STD-7009B 8-factor matrix
  openbmp-mc    L7  campaign orchestrator: DoE (LHS/Sobol+Owen/Iman-Conover/Wilks),
                    Welford/Clopper-Pearson reducers, nested driver, rare-event methods,
                    convergence gate, sample-table + credibility-report writers

EXTENDED:
  openbmp-core      + DeterministicRng::for_mc_sample (b"MCRN" domain tag)
  openbmp-testkit   + MMS manufactured states, order-of-accuracy harness, NEES/NIS helper
  openbmp-runner    + MC campaign hook (one forward 6-DOF run = one sample evaluation)
  openbmp-cli       + `openbmp mc <scenario>` / `openbmp verify-order` / `openbmp reconstruct`
```

**Layer justification.** `openbmp-uq` is L1 (alongside `openbmp-state`): it is a
pure data-and-math crate (error budget, credibility matrix, p-box arithmetic)
with no simulator dependency, consumable by physics crates that *produce*
per-entry uncertainty and by `openbmp-mc` that *aggregates* it. `openbmp-mc` is
L7 (alongside `openbmp-runner`): it orchestrates forward runs, so it sits above
the runner and depends on it; it must **never** be depended on by `openbmp-fc`
or any controller/physics crate (FC portability lock, `00` §3.3). Neither crate
introduces an up-layer edge or a cycle.

### 3.1 `openbmp-uq` — error budget, credibility, p-box (L1)

#### 3.1.1 Source-tagged correlation-matrix error budget

Replace the flat `√(Σ σᵢ²) + |bias|` with the quadratic form
`σ_agg = √(sᵀ Σ s)` where `s` is the vector of per-source 1-σ contributions and
`Σ` is the correlation matrix (`Σᵢᵢ = 1`, `Σᵢⱼ = ρᵢⱼ`). Each contribution
carries an `UncertaintyClass` tag so the nested driver can separate the
irreducible (aleatory) part from the lack-of-knowledge (epistemic) part.

```rust
/// Aleatory (irreducible run-to-run variability) vs epistemic
/// (lack-of-knowledge) classification of an uncertainty source.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum UncertaintyClass {
    /// Irreducible variability: winds, sensor noise, turbulence.
    Aleatory,
    /// Reducible lack-of-knowledge: a poorly-known bias mean, a
    /// model-form coefficient within a band, a low-heritage component.
    Epistemic,
}

/// One uncertainty source's contribution. Generalises the current
/// `UncertaintyContribution` with a class tag and a 7009B credibility
/// record in place of the bare `ValidationStatus`.
pub struct UncertaintySource {
    pub source_id: String,
    pub one_sigma: f64,
    pub class: UncertaintyClass,
    pub credibility: CredibilityRecord, // §3.1.3
    pub justification: String,          // evidence pointer
}

/// Source-tagged error budget with an explicit correlation structure.
pub struct CorrelatedErrorBudget {
    pub sources: Vec<UncertaintySource>,
    /// Symmetric PSD correlation matrix, row/col order = `sources`.
    /// `None` ⇒ identity (independent) ⇒ reduces to the legacy RSS.
    pub correlation: Option<CorrelationMatrix>,
}

impl CorrelatedErrorBudget {
    /// σ_agg = √(sᵀ Σ s). Validates Σ symmetric + positive-semidefinite
    /// (eigenvalue floor) before use; rejects a non-PSD matrix fail-closed.
    pub fn aggregate_one_sigma(&self) -> Result<f64, UqError>;
    /// Aleatory-only and epistemic-only sub-aggregates (mask `s` by class).
    pub fn aleatory_one_sigma(&self) -> Result<f64, UqError>;
    pub fn epistemic_one_sigma(&self) -> Result<f64, UqError>;
    /// Binding credibility = min over the 8×{0..4} factor matrix (§3.1.3).
    pub fn binding_credibility(&self) -> Option<CredibilityRecord>;
}
```

The legacy `correlated_bias` collapses into the off-diagonal of `Σ` (a fully
correlated source pair has `ρ = 1`). With `Σ = I` the new aggregate is exactly
the old RSS, so the rewire is a strict generalisation — the WP that lands it
keeps every existing `uq.rs` numeric test passing by setting `correlation:
None`.

#### 3.1.2 Variance decomposition & the probability box

The nested (double-loop) MC (§3.2.4) produces a family of conditional CDFs
`F(y|θ)`, one per epistemic sample θ. The p-box is the envelope:

- `F_lo(y) = min_θ F(y|θ)`, `F_hi(y) = max_θ F(y|θ)`.
- A requirement `P(y ∈ spec) ≥ p*` must hold on the **worst-case (lower)**
  bound `F_lo` — the design refuses to report only the mean CDF.

Total-variance split (the law of total variance), computed directly from the
nested ensemble:

```
Var(y) = E_θ[ Var_a(y|θ) ]   +   Var_θ[ E_a(y|θ) ]
         └─ aleatory part ─┘       └─ epistemic part ─┘
```

```rust
pub struct ProbabilityBox {
    pub support: Vec<f64>,          // y grid
    pub lower_cdf: Vec<f64>,        // F_lo(y)
    pub upper_cdf: Vec<f64>,        // F_hi(y)
}
pub struct VarianceSplit {
    pub aleatory: f64,
    pub epistemic: f64,
    pub total: f64,                 // == aleatory + epistemic (checked)
}
```

Where no probability is justified for an epistemic source, the design supports
interval / Dempster–Shafer arithmetic (Ferson SAND2002-4015) rather than forcing
a distribution — an honest epistemic band, not a fabricated one.

`REQ-UQ-003` / `V-UQ-003` now implement the reusable reducer substrate for this
section: `openbmp-uq::ProbabilityBox` builds the conditional-CDF envelope from
nested scalar samples, `openbmp-uq::VarianceSplit` computes
`E_θ[Var_a] + Var_θ[E_a]`, and `openbmp-mc::analyze_nested_scalar_samples`
attaches a campaign-facing lower-tail requirement verdict that explicitly uses
the lower p-box bound. `openbmp mc nested-summarize` exposes that reducer for
materialized nested scalar sample CSVs grouped by epistemic condition and can
write deterministic p-box CSV evidence. The landing-footprint runner now also
accepts `[landing_footprint.monte_carlo.nested]` plus per-source
`uncertainty_class = "aleatory" | "epistemic"` tags, maps the nested
`(epistemic, aleatory)` grid onto deterministic flat sample indices, emits
`[nested_uq]` summary evidence, and lets `openbmp footprint-mc` write the
declared p-box CSV.

#### 3.1.3 NASA-STD-7009B credibility matrix

Generalise `minimum_status()` from one `ValidationStatus` to an eight-factor
0–4 matrix. The eight factors (two assessment groups):

```rust
#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum CredibilityLevel { L0, L1, L2, L3, L4 } // 0 = insufficient … 4 = full

#[derive(Copy, Clone, Debug, PartialEq, Eq, Hash)]
pub enum CredibilityFactor {
    // M&S Capability assessment
    Verification,        // (1) code/solution verification evidence
    Validation,          // (2) comparison vs real-world / reference data
    InputPedigree,       // (3) provenance/quality of inputs
    ResultsUncertainty,  // (4) UQ completeness
    ResultsRobustness,   // (5) sensitivity / robustness
    // Process assessment
    UseHistory,          // (6)
    MsManagement,        // (7)
    PeopleQualification, // (8)
}

pub struct CredibilityRecord {
    /// Level per factor; absent factor ⇒ L0 (fail-closed).
    pub scores: BTreeMap<CredibilityFactor, CredibilityLevel>,
    /// Evidence pointer per scored factor (the V&V case that justifies it).
    pub evidence: BTreeMap<CredibilityFactor, String>,
}

impl CredibilityRecord {
    /// Binding credibility = the MINIMUM level across contributing factors
    /// (a chain is as strong as its weakest link) — generalises
    /// `ErrorBudget::minimum_status`.
    pub fn binding_level(&self) -> CredibilityLevel;
    /// Map onto the four legacy labels for the public summary line.
    pub fn legacy_label(&self) -> ValidationStatus; // from openbmp-core
    pub fn render_markdown(&self) -> String;        // generalises render_markdown
}
```

A `CredibilityFloor` is configurable per scenario; a campaign whose binding
level falls below the floor is **labelled academic-only and the requirement
verdict is refused** (fail-closed, reusing the existing scenario-consumer-
agreement lint path). This is itself a dual-use safeguard (§6): it prevents any
aggregate from being mistaken for operational/certified/targeting-grade.

### 3.2 `openbmp-mc` — campaign orchestrator (L7)

#### 3.2.1 Design of experiments

```rust
/// A named dispersion input: a marginal CDF over a scalar parameter.
pub trait Dispersion {
    /// Inverse CDF (quantile) x = F⁻¹(u), u ∈ (0,1). Deterministic, total.
    fn inverse_cdf(&self, u: f64) -> f64;
    fn class(&self) -> UncertaintyClass; // aleatory | epistemic
    fn id(&self) -> DispersionId;
}

/// A design-of-experiments generator: returns an N×d matrix of points in
/// the unit cube [0,1)^d, mapped to inputs by per-column inverse_cdf.
pub trait DoeGenerator {
    fn generate(&self, n: usize, d: usize, rng: &mut DeterministicRng) -> DesignMatrix;
}

pub struct LatinHypercube;              // McKay-Beckman-Conover 1979
pub struct SobolJoeKuo { scramble: OwenScramble }   // Joe-Kuo 2008 + Owen 1997
pub struct ImanConover { target: CorrelationMatrix } // post-hoc rank reorder
```

**LHS** (McKay–Beckman–Conover 1979): for `d` inputs, `N` samples, draw a random
Latin permutation `πₖⱼ` per column and set
`uₖⱼ = (πₖⱼ − 1 + Uₖⱼ)/N`, `Uₖⱼ ~ Uniform(0,1)`; then `xₖⱼ = Fⱼ⁻¹(uₖⱼ)`. One
sample per `1/N` marginal stratum guaranteed. The permutation and the `U` jitter
both draw from `DeterministicRng::for_mc_sample`, so the design is byte-stable.

**Sobol** (Joe–Kuo 2008): generate coordinate `j` by XOR-ing the gray-code-
indexed direction numbers `v_{j,k}` built from the primitive-polynomial /
initial-direction-number table (ingested as a provenance-pinned `data/sobol/`
asset — see §8). The current `SobolSequence` implementation is the native
unscrambled base generator backed by `data/sobol/new-joe-kuo-6.21201`, with the
source URL and SHA-256 pin recorded in `data/sobol/provenance.md`.
**Owen scrambling** (Owen 1997) applies a per-digit, per-dimension
nested-uniform permutation keyed by a deterministic hash of
`(campaign_seed, dim, digit, node)`. The implemented `OwenScramble` path uses a
nested bit scramble keyed by `(scramble_seed, dimension, bit, prior_prefix)` and
is exposed as `openbmp mc sobol --scramble-seed`. This yields an *unbiased* RQMC
estimator whose variance is `O(N⁻³ (log N)^{d−1})` for smooth integrands versus
`O(N⁻¹)` for plain MC. `R` independent Owen-scrambled replicates give an honest
CI even though each net is deterministic: the sample mean of the replicate
means, with the replicate variance as the standard error.

QMC quality is reported via the star discrepancy `D*_N` and the Koksma–Hlawka
bound `|I − I_N| ≤ V(f) · D*_N`.

**Iman–Conover** (1982): impose a target Spearman/Pearson correlation `R` on an
existing column-independent design by Cholesky `R = L Lᵀ`, generating van-der-
Waerden scores, and reordering each input column by the rank of the
corresponding `L·scores` column — distribution-free, marginal-preserving. The
implemented `ImanConover` transform preserves each `DesignMatrix` column's
sorted values exactly and is available through `openbmp mc iman-conover`. Uses
`nalgebra`'s Cholesky (already in the dep tree).

#### 3.2.2 Welford running statistics & Clopper–Pearson

```rust
/// Order-independent running mean/variance (Chan-Golub-LeVeque 1979).
pub struct Welford { n: u64, mean: f64, m2: f64 }
impl Welford {
    pub fn push(&mut self, x: f64);            // M_n = M_{n-1}+(x−M_{n-1})/n
    pub fn merge(&self, other: &Welford) -> Welford; // parallel merge (§4.1)
    pub fn variance(&self) -> f64;             // m2/(n−1)
    pub fn standard_error(&self) -> f64;       // √(variance/n)
}
```

Welford recursion: `Mₙ = Mₙ₋₁ + (xₙ − Mₙ₋₁)/n`,
`Sₙ = Sₙ₋₁ + (xₙ − Mₙ₋₁)(xₙ − Mₙ)`; MC standard error `= √(Sₙ / (n(n−1)))`.
The **parallel merge** (combine two `(n, mean, M2)` triples) is what makes
rayon fan-out bit-identical regardless of thread count (§4.1) — the single most
important determinism requirement of this dimension.

For a binary mission outcome with `k` successes in `N`, the **exact two-sided
Clopper–Pearson interval** (1934) is
`[ B⁻¹(α/2; k, N−k+1), B⁻¹(1−α/2; k+1, N−k) ]`
with `B⁻¹` the inverse regularized incomplete beta (from `statrs`, MIT, or a
Newton iteration on the regularized beta). For zero observed failures use the
one-sided rule-of-three `p ≤ 3/N` at 95 %.

#### 3.2.3 Wilks sizing & convergence gate

The distribution-free two-sided tolerance interval covering proportion `P` with
confidence `C` from order statistics needs `N` satisfying Wilks' formula
`1 − Pᴺ − N(1−P)Pᴺ⁻¹ ≥ C`. For the certification-class
`P = 0.99865` (3-σ), `C = 0.90`, this gives `N = 2880`. The calculator is a
closed-form monotone root-find:

```rust
pub fn wilks_two_sided_n(coverage: f64, confidence: f64) -> usize;
pub fn wilks_one_sided_n(coverage: f64, confidence: f64) -> usize; // N ≥ log(1−C)/log(P)
```

The **convergence gate** stops a campaign early when the metric CI half-width
relative to `|mean|` falls below a tolerance over a window:

```rust
pub struct ConvergenceGate { rel_half_width_tol: f64, window: usize }
pub enum CampaignStop { Converged { at: u64 }, ReachedN, WilksFloor }
```

It also reports the **Gelman–Rubin R̂** across independent seed-chains for
ensemble stability (R̂ → 1 ⇒ converged). The campaign reports the *honest*
finite-N answer; it never tunes N to make a claim land.

#### 3.2.4 Nested (double-loop) aleatory/epistemic driver

```rust
pub struct NestedCampaign {
    pub epistemic: Vec<Box<dyn Dispersion>>, // outer loop (class = Epistemic)
    pub aleatory: Vec<Box<dyn Dispersion>>,  // inner loop (class = Aleatory)
    pub n_epistemic: usize,
    pub n_aleatory: usize,
    pub doe: Box<dyn DoeGenerator>,
}
```

Outer loop samples epistemic θ `N_e` times; for each θ the inner loop runs `N_a`
aleatory samples to produce a conditional CDF `F(y|θ)`. The ensemble of inner
CDFs forms the p-box (§3.1.2); the variance split falls out directly. A flat
(single-loop) campaign is the degenerate `N_e = 1` case.

#### 3.2.5 Rare-event *methods* — synthetic limit-states only

```rust
/// A scalar performance / limit-state. FAILURE is g(state) < 0.
/// The vocabulary is CLOSED and forward-only (§6): apogee shortfall,
/// insertion-box miss, max-q exceedance, structural-load violation.
/// It is a COMPILE / lint error to reference a ground aimpoint.
pub trait LimitState {
    fn g(&self, qoi: &QoiVector) -> f64;
    fn label(&self) -> &str;             // names the synthetic / flight condition
}

pub struct SubsetSimulation { p0: f64, levels_max: usize } // Au-Beck 2001
pub struct CrossEntropyIs   { family: BiasFamily }         // Rubinstein-Kroese
```

**Subset Simulation** (Au–Beck 2001): express the inputs in standard-normal
coordinates (inverse-CDF wrappers around the dispersions). Define failure
`F = {g(x) < 0}`. Choose intermediate thresholds `b₁ > b₂ > … > b_m = 0` so each
conditional `P(Fᵢ | Fᵢ₋₁) ≈ p₀` (typically 0.1). Then
`P(F) = P(F₁) · ∏ᵢ₌₂ᵐ P(Fᵢ | Fᵢ₋₁)`. Level 0 is plain MC; each level uses a
component-wise Modified Metropolis (MMA) sampler in standard-normal space,
accepting per the ratio of standard-normal marginals while keeping samples in
`Fᵢ₋₁`; `bᵢ` is adaptively the `p₀`-quantile of the level's `g`-values. The
estimator CoV grows only `~ (log P)²` — vastly better than MC's `1/√(N·P)`.

**Cross-Entropy IS** (Rubinstein–Kroese): pick a parametric biasing family
`h(x; v)` (e.g. shifted/scaled Gaussian); iterate
`v_{t+1} = argmax_v (1/N) Σ_k 1{g(x_k) < γ_t} (f(x_k)/h(x_k; v_t)) log h(x_k; v)`
using the elite-quantile `γ_t`; the final unbiased IS estimator is
`P̂ = (1/N) Σ 1{g<0} f/h`.

**Hard constraint (§6):** the `LimitState` vocabulary is closed and forward-
only; a `g` referencing any ground-target coordinate is rejected by the
scenario-consumer-agreement lint — this is what keeps a rare-event reliability
tool from becoming a CEP/fire-control optimizer. The output is labelled
`synthetic-limit-state` and never bound to a named vehicle.

`REQ-MC-010` / `V-MC-010` implement the first WP-11.4 rare-event slice:
`openbmp-mc` exposes `StandardNormalPoint`, the closed `LimitState` interface,
`SyntheticLinearLimitState` with exact `Φ(-β)` truth, deterministic
`SubsetSimulation`, deterministic `CrossEntropyIs`, replay-identical
`RareEventEstimateReport` evidence, and a fail-closed runtime guard that
rejects non-`synthetic-limit-state` labels. The `LimitState` trait is sealed,
and `openbmp-testkit` carries a compile-fail tripwire proving external crates
cannot implement a ground-aimpoint limit state. The scenario parser now accepts
schema-v3 synthetic-only `[monte_carlo.limit_state]`,
`[monte_carlo.subset_simulation]`, and `[monte_carlo.cross_entropy]` manifests,
and the pre-serde consumer-agreement lint rejects ground-aimpoint / target
vocabulary under `monte_carlo.limit_state` before typed validation. `openbmp mc
rare-event` consumes those synthetic scenario manifests and can emit
deterministic TOML evidence for the estimator reports and adaptation traces.

#### 3.2.6 Code verification — MMS, order-of-accuracy, Richardson/GCI

(In `openbmp-testkit`, driven by `openbmp mc`/`openbmp verify-order`.)

**Method of Manufactured Solutions** (Salari–Knupp SAND2000-1444; Roache 2002):
pick a smooth manufactured state `x_manuf(t)`, substitute into the ODE
`ẋ = f(x, t)` to derive the source term `S(t) = ẋ_manuf − f(x_manuf, t)`, add `S`
to the RHS, integrate, and the numerical solution must reproduce `x_manuf` to
truncation order. **Observed order of accuracy:** run on `h, h/2, h/4`, form
`e(h) = ‖x_num − x_manuf‖`, and `p = log(e(h)/e(h/2)) / log 2` must approach the
scheme's formal order (RK4 → 4, DOP853 → 8). **Richardson / GCI:**
`GCI = Fs · |e_fine| / (rᵖ − 1)` with safety factor `Fs = 1.25`, refinement
ratio `r` — a numerical-error bar, not just a convergence claim.

#### 3.2.7 Reconstruction & validation — BET half

```rust
pub struct RtsSmoother;     // Rauch-Tung-Striebel backward pass
pub struct BatchLeastSquares { max_iter: usize } // Gauss-Newton
pub struct NeesNisCheck;    // chi-square consistency (Bar-Shalom, Gelb)
```

The forward EKF already exists in `openbmp-fc`. The missing half:

- **RTS backward smoother** (Rauch–Tung–Striebel 1965): reusing stored `Φ_k` and
  covariances, `C_k = P_{k|k} Φ_kᵀ P_{k+1|k}⁻¹`;
  `x_{k|N} = x_{k|k} + C_k(x_{k+1|N} − x_{k+1|k})`;
  `P_{k|N} = P_{k|k} + C_k(P_{k+1|N} − P_{k+1|k})C_kᵀ`.
- **Batch weighted Gauss–Newton** (Tapley–Schutz–Born 2004):
  minimise `J(x₀) = Σ_k (y_k − h(x_k))ᵀ W_k (y_k − h(x_k))`, `W_k = R_k⁻¹`;
  normal equations `(Σ H_kᵀ W_k H_k) δx = Σ H_kᵀ W_k (y_k − h(x_k))`, with
  `H_k = (∂h/∂x) Φ(t_k, t₀)`; iterate (argmin via `argmin` crate).
- **NEES/NIS consistency** (Gelb 1974; Bar-Shalom 2001): on a linear-Gaussian
  truth model with known `Q, R`, the normalized estimation/innovation error
  squared must lie within the chi-square 95 % bounds — the rigorous,
  data-free filter check that makes the reconstruction credible even though a
  real BET is out of reach (ceiling §1.2.3).

The **open-telemetry cross-validation** (BET vs scraped Falcon-9-class webcast
readouts) is LOCAL-only per the existing external-telemetry policy: never
committed, never vendored, never a flight/targeting input — a comparison
reference only under `docs/external-telemetry-validation.md`.

### 3.3 Fidelity tiers

| Tier | Scope | Earns (typical) |
|---|---|---|
| **T0** Foundation | `openbmp-mc` on `DeterministicRng`; replace the N=16 SplitMix64 orbit MC; Welford running mean/variance + Clopper–Pearson CI on the binary outcome; samples table + convergence trace. No new physics. | `validated-toy` (Clopper–Pearson calibration case) |
| **T1** DoE + sizing | LHS + Sobol(Joe–Kuo)+Owen RQMC + Iman–Conover; Wilks sizing; convergence gate. Defensible auto-sized N=2k–5k campaigns. | `research` (Wilks coverage + QMC-vs-MC variance benchmark) |
| **T2** Credibility + error-budget rewire | `openbmp-uq`: 7009B 8-factor matrix; `√(sᵀΣs)` aleatory/epistemic budget; **wire** it into the MC driver → per-campaign credibility report, fail-closed below floor. | `checked` → `validated-toy` (RSS-equivalence + min-reducer) |
| **T3** Rare-event + nested UQ | Subset Simulation + CE-IS on synthetic limit-states; nested double-loop p-box + variance split. | `research` (analytic `Φ(−β)` recovery) |
| **T4** Verification + reconstruction | MMS + observed-order-of-accuracy + Richardson/GCI; code-to-code export; RTS smoother + batch GN + NEES/NIS; LOCAL BET cross-validation. | `research` (observed p matches formal order; NEES/NIS) |
| **T5** Campaign scale + real-time | rayon order-independent (Welford-merge-tree) reductions, bit-identical across thread count; checkpoint/resume; aarch64 FPCR guard; deny FMA contraction; faster-than-real-time scheduler. (**Owned by `12-determinism-realtime-and-compute.md`; this doc consumes it.**) | `validated-toy` (thread-count byte-identity) |

---

## 4. Invariant preservation

For each of the ten `00` §3 invariants, concretely for this dimension:

1. **Byte-determinism.** Per-sample seed = `DeterministicRng::for_mc_sample(
   campaign_seed, sample_index, dimension_id)` with a `b"MCRN"` domain tag —
   independent of worker count and completion order (counter-based / Philox
   philosophy). All reductions are **order-independent**: the Welford parallel
   merge (`delta = meanB − meanA; mean = meanA + delta·nB/n; M2 = M2A + M2B +
   delta²·nA·nB/n`) combined as a *tree*, or sort-then-fold, so the float sum is
   bit-identical regardless of thread schedule. No `mul_add`, no wall-clock, no
   system RNG, no unordered iteration (BTreeMap not HashMap for any scored-factor
   map). A pinned reference-stream test (mirroring the existing
   `for_*_reference_stream_is_locked`) locks `for_mc_sample`. The campaign-level
   byte-diff gate extends the determinism oracle to **ensemble statistics** (the
   samples table and the summary), per `00` §8 risk register.

2. **Byte-stable-by-default.** `openbmp-mc` is off until a scenario declares
   `[monte_carlo]` (and `[monte_carlo.subset_simulation]`, `[uq]`,
   `[monte_carlo.nested]` for the higher tiers). No existing golden changes
   until a scenario opts in; the footprint MC and the orbit test keep their
   current numbers (the orbit test's *physics* dispersions are preserved
   verbatim when its *statistics* surface is swapped). This follows the
   established `[vehicle.bending]` pattern exactly.

3. **FC hardware-portability lock.** `openbmp-mc` is L7 and `openbmp-uq` is L1;
   **neither is ever a dependency of `openbmp-fc`**. The MC orchestrator
   consumes forward 6-DOF runs through the runner — it never feeds a controller.
   The `fc_dependency_tripwire.rs` stays green; no new FC edge is introduced.

4. **Lockstep-clock & no-hot-path-allocation.** The MC driver runs *offline*
   (campaign orchestration), not inside the FC tick, so the lockstep-clock
   contract is untouched. Within a sample, the existing runner per-tick
   allocation discipline is unchanged. The campaign loop itself reuses scratch
   reducers (Welford, the design matrix) rather than allocating per sample.

5. **Four-pillar provenance.** The Sobol direction-number table lands in
   `data/sobol/new-joe-kuo-6.21201` with a sibling `provenance.md`, SHA-256
   pin, and license note (Joe–Kuo terms, §8). No inline Sobol table or TOML
   fixtures in `*.rs`. **No literal WGS84 GM_earth / J2 / R_earth numerals**
   appear in `openbmp-mc`/`openbmp-uq` or this doc — orbital QoIs read
   `WGS84_MU_M3_S2` etc. from `openbmp-physics::frames` symbolically (as the
   orbit test already does via `MU_EARTH = WGS84_MU_M3_S2`).

6. **Synthetic / public-data-only.** Every dispersion deck checked in is
   synthetic or textbook-derived. The Earth-GRAM perturbed-atmosphere decks and
   the scraped webcast BET telemetry are **ingest-only, LOCAL, never committed**
   (§8). Rare-event outputs are labelled `synthetic-limit-state`.

7. **Forward-only mechanical locks.** The `LimitState` vocabulary is closed and
   forward-only (apogee/insertion-box/max-q/structural-load); a new compile-fail
   /lint test proves a `g` cannot reference a ground aimpoint (§6). The footprint
   MC boundary ("never produces an actuator command") is preserved when the new
   convergence/rare-event statistics are layered on — statistics *about* a
   forward footprint, never a solve *for* an aimpoint.

8. **Validation labels.** Every new model/case declares exactly one of
   `experimental` → `checked` → `validated-toy` → `research` with a tolerance
   table; the 7009B matrix is the *evidence underneath* the label, and
   `CredibilityRecord::legacy_label()` keeps the public summary in the four-label
   vocabulary. No artifact claims `flight-qualified`/`certified`/`operational`/
   `mission-ready`; rare-event probabilities are explicitly `academic-scope`.

9. **Workspace lints & docs.** `missing_docs`, `unsafe_code`, `unused_must_use`,
   `float_cmp` stay `deny`; `unwrap`/`expect`/`panic` stay `warn` outside tests.
   The non-PSD-correlation and zero-N-campaign paths are fail-closed (`Result`,
   not `panic`).

10. **Requirements traceability.** Each tier adds `REQ-MC-…`/`REQ-UQ-…` entries
    with `V-…` verification evidence; `check_requirements_traceability.py` stays
    green.

---

## 5. V&V plan

Verification (solving the equations right) and validation (solving the right
equations) are kept distinct per AIAA G-077. Each tier earns the label its
evidence justifies; tolerance tables are mandatory for every numeric claim
(`docs/verification.md § Tolerance Tables`).

### 5.1 Verification cases

| Case | Method | Tolerance | Tier / Label |
|---|---|---|---|
| Clopper–Pearson calibration | For known Bernoulli `p`, `N`, the exact interval attains ≥ nominal coverage across `p ∈ [1e-4, 0.5]` | coverage ≥ nominal (conservative) for all tested `p` | T0 / `validated-toy` |
| Welford vs two-pass variance | Running variance equals the two-pass formula | `rel ≤ 1e-12` | T0 / `validated-toy` |
| Welford parallel-merge byte-identity | Tree-merge across K partitions == sequential fold, bit-for-bit | `to_bits()` equal | T0/T5 / `validated-toy` |
| Wilks coverage self-check | Draw from a known dist, build the two-sided 99.865/90 interval at Wilks-N, MC-verify empirical coverage ≥ 90 % | empirical coverage ≥ 0.90 | T1 / `research` |
| QMC vs MC variance | Owen-scrambled Sobol RQMC error decays faster than `1/√N` on a smooth test integrand | observed slope steeper than MC; `R`-replicate CI brackets truth | T1 / `research` |
| Iman–Conover correlation induction | Induced Spearman ρ matches target | `|ρ_induced − ρ_target| ≤ 0.02` | T1 / `validated-toy` |
| RSS-equivalence of the new budget | `√(sᵀΣs)` with `Σ=I` equals legacy `√(Σσ²)` | `to_bits()` equal on the uq.rs test vectors | T2 / `checked` |
| 7009B min-reducer | Binding level == min over scored factors | exact | T2 / `validated-toy` |
| Subset Simulation analytic | Linear limit-state `g(x)=β − (1/√d)Σxᵢ` in std-normal: exact `P=Φ(−β)`; recover `P ∈ [1e-3, 1e-6]` | `|log₁₀ P̂ − log₁₀ Φ(−β)| ≤ 0.3`, CoV within Au–Beck bound | T3 / `research` |
| CE-IS analytic | Same series-system benchmark, IS estimator unbiased | as above | T3 / `research` |
| MMS order-of-accuracy | Manufactured smooth state; observed `p = log(e(h)/e(h/2))/log 2` | RK4: `p ∈ [3.8, 4.2]`; DOP853: `p ∈ [7.5, 8.5]` | T4 / `research` |
| Richardson/GCI bar | `GCI` brackets the extrapolated error | reported, monotone under refinement | T4 / `validated-toy` |
| EKF/RTS NEES/NIS | Linear-Gaussian truth, known `Q,R`: normalized errors within chi-square 95 % bounds over many runs | fraction in-bounds ∈ [0.93, 0.97] | T4 / `research` |

### 5.2 Validation cases

| Case | Reference | Disposition |
|---|---|---|
| Two-body / J2 propagation code-to-code | Orekit / GMAT (Apache-2.0) canonical LEO test orbit (Vallado) | position/velocity within each tool's stated tolerance over multiple orbits; export adapter only — `research` |
| 6-DOF attitude / dispersion cross-check | Basilisk (ISC) MC harness | OpenBMP dispersion statistics consistent with Basilisk same-physics run — `research` |
| DoE / sensitivity cross-check | Dakota (LGPL, COUPLE only) / SALib (MIT) on saved sample/response CSVs | OpenBMP native LHS/Sobol designs + Sobol indices statistically equivalent — `research` |
| Earth-GRAM dispersion cross-check | Earth-GRAM 2016 perturbed profiles (ingest-only, LOCAL) | OpenBMP apogee / max-q spread consistent with a GRAM-driven reference deck — `validated-toy` (LOCAL) |
| BET vs open Falcon-9-class telemetry | scraped webcast readouts (LOCAL, never committed) | reconstructed BET brackets OpenBMP's predicted ascent dispersion envelope — sanity comparison only, **not** ground truth |

**Promotion rule (per `docs/verification.md`):** a rare-event or reconstruction
case cannot reach `research` without (a) at least one analytic/MMS code-
verification case, (b) one public/code-to-code comparison, (c) a declared
validity envelope, and (d) a documented uncertainty story. Code-to-code
agreement alone is *not* validation.

---

## 7. Dependencies on other parity docs

- **`12-determinism-realtime-and-compute.md`** — owns T5: rayon order-
  independent reductions, checkpoint/resume, aarch64 FPCR guard, deny-FMA build
  flags, faster-than-real-time pacing. `openbmp-mc` *consumes* this substrate;
  the Welford parallel-merge contract (§4.1) is the shared interface. **Tightest
  coupling in the program.**
- **`08-environment-gravity-and-frames.md`** — supplies the perturbed-atmosphere
  / GRAM-style dispersion deck and the tesseral-gravity propagation the orbital
  QoIs depend on; the Earth-GRAM ingest (§8) is shared.
- **`09-sensors-navigation-and-actuators.md`** — supplies the IMU/GNSS error
  models whose per-entry UQ feeds the aleatory inner loop; the EKF whose forward
  pass the BET reconstruction reuses lives at this boundary (in `openbmp-fc`).
- **`03/04/05` (aero / aerothermal / propulsion)** — the **producers** of the
  per-entry bias/random margins that flow into `CorrelatedErrorBudget` as
  epistemic model-form bands. This doc *consumes* their UQ database objects; it
  never invents aero/thermal uncertainty (ceiling §1.2.2).
- **`02-structural-dynamics-loads-slosh-pogo.md`** — supplies the structural-load
  limit-state QoIs (max-q·α, gimbal/load exceedance) the rare-event methods act
  on.
- **`06-gnc-coupled-mimo-and-control.md`** / **`07-trajectory-optimization-and-
  mission-design.md`** — the closed-loop GNC and the optimizer are the *fixed
  controller* dispersed against in the MC philosophy; the forward-only terminal-
  condition vocabulary lock is shared with the `LimitState` lock here.
- **`10-flight-software-in-the-loop-xil.md`** — the SIL boundary and fault-
  injection ports; large-N campaigns may run flight SW in the loop, and the AFTS
  containment-monitor forward-only posture aligns with the `LimitState` lock.
- **`13-agent-execution-playbook.md`** — the gate set, the §5 WP schema, the
  per-PR checklist this backlog obeys.

---

## 8. Open-source leverage

Use-mode is one of **port** (reimplement the algorithm in-repo), **couple** (run
externally as an oracle, compare outputs), or **ingest** (consume data with
provenance). License compatibility is checked against the MIT/Apache-2.0
workspace by `cargo deny`.

| Tool / asset | License | Mode | Use |
|---|---|---|---|
| **statrs** | MIT | port-as-dep | Inverse regularized incomplete beta (Clopper–Pearson), gamma / chi-square CDFs (NEES/NIS). Add as workspace dep rather than reimplementing special functions. |
| **argmin** | MIT/Apache-2.0 | port-as-dep | Gauss–Newton / L-BFGS for batch BET reconstruction and the CE-IS inner optimization. |
| **rayon** | MIT/Apache-2.0 | port-as-dep | Data-parallel sample fan-out with order-independent (Welford-merge-tree) reductions (T5, via `12-…`). |
| **Joe–Kuo direction numbers** | free academic+commercial (authors' terms) | ingest + port | Reimplement the Sobol generator *natively* (keeps byte-determinism under project control); ingest the direction-number table to `data/sobol/` with provenance + SHA pin. Do **not** depend on a less-audited Sobol crate. |
| **Dakota** (Sandia) | LGPL v2.1 | couple only | Code-to-code oracle for LHS / Sobol / sensitivity indices / PCE. LGPL ⇒ link/couple fine; **reimplement from algorithms, never copy source** into the MIT/Apache tree. |
| **SALib** (Python) | MIT | couple / ingest | Reference Sobol first/total indices, Morris, FAST, PAWN on saved OpenBMP sample/response CSVs; bootstrap which dispersions dominate. MIT ⇒ safe to port index formulas. |
| **Orekit** (Java) / **GMAT** (NASA) | Apache-2.0 | couple | Code-to-code oracle for orbit propagation, J2/zonal gravity, and batch-LS/EKF OD (BET cross-check). Export adapter only; do not port. |
| **Basilisk** (AVS Lab) | ISC | couple | Code-to-code oracle for 6-DOF attitude dynamics, RCS/TVC effectors, and its MC dispersion harness. |
| **NASA Earth-GRAM 2016** | NASA OSS / U.S.-Gov (export-screened) | ingest only, LOCAL | De-facto launch-vehicle dispersion atmosphere; generate perturbed density/temperature/wind profiles offline as the ascent-MC atmosphere deck. **Never commit GRAM binaries or generated decks** (mirror the external-telemetry policy). |
| Open Falcon-9-class webcast telemetry | scraped, third-party | ingest only, LOCAL | BET cross-validation reference; LOCAL/casual only, never committed/vendored/CI'd, never a flight/targeting input. |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, every `13` §2 gate green. Each WP
proposing a new crate ships the skeleton + placement justification as the first
reviewable commit (`13` §5).

---

### WP-11.0 — `openbmp-mc` skeleton + deterministic substrate; replace N=16 orbit MC

- **title:** Reusable deterministic MC crate on `DeterministicRng` (Welford + Clopper–Pearson)
- **goal:** Stand up `openbmp-mc` (L7) and `DeterministicRng::for_mc_sample` (`b"MCRN"` tag); replace the ad-hoc inline-`SplitMix64` N=16 orbit Monte Carlo with the reusable orchestrator producing a samples table + a Clopper–Pearson CI + a Welford convergence trace, regressing the *physics* dispersions unchanged. The substrate every Phase-A/D MC depends on.
- **fidelity_tier:** T0
- **depends_on:** []
- **new_crates:** `openbmp-mc` (L7, alongside `openbmp-runner`) — placement justified: orchestrates forward runs, never depended on by `openbmp-fc`.
- **touched:** `crates/openbmp-core/src/rng.rs` (+`for_mc_sample`, pinned-stream test); new `crates/openbmp-mc/`; `crates/openbmp-cli/tests/phalcon9_orbit_monte_carlo.rs` (port off SplitMix64); `crates/openbmp-cli/src/commands/` (+`mc` subcommand).
- **approach:** §3.2.1 (sample seeding), §3.2.2 (Welford + Clopper–Pearson). Reuse the footprint-MC samples-table/writer shape (§2.4). `for_mc_sample(campaign_seed, sample_index, dimension_id)` with `b"MCRN"` in bytes `[28..32]`.
- **acceptance:**
  - new MC path off by default; canonical goldens + footprint-MC numbers byte-identical.
  - the orbit MC's per-sample physics dispersions reproduce bit-for-bit after the SplitMix64→`for_mc_sample` port (a pinned-stream regression).
  - Clopper–Pearson calibration case in a tolerance table (coverage ≥ nominal across `p ∈ [1e-4, 0.5]`).
  - Welford-vs-two-pass variance `rel ≤ 1e-12`; pinned `for_mc_sample` reference-stream test.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; statistics about forward runs only.
- **est_effort:** ~1 week
- **parity_ceiling:** no DoE/variance-reduction yet; no rare events; N still hand-set.

---

### WP-11.1 — DoE (LHS / Sobol+Owen / Iman–Conover) + Wilks sizing + convergence gate

- **title:** Add space-filling / low-discrepancy designs and tolerance-interval sizing
- **goal:** Replace naive draws with LHS and Owen-scrambled Sobol RQMC, Iman–Conover correlation induction, and a Wilks sample-size calculator so a scenario requests "99.865 % @ 90 % CL" and is told N; a convergence gate stops the campaign when the metric CI half-width falls below tolerance.
- **fidelity_tier:** T1
- **depends_on:** [WP-11.0]
- **new_crates:** []
- **touched:** `crates/openbmp-mc/` (DoE module); `data/sobol/new-joe-kuo-6.21201` + `provenance.md`; `crates/openbmp-mc/` (Wilks + convergence gate).
- **approach:** §3.2.1 (LHS, Sobol Joe–Kuo + Owen, Iman–Conover), §3.2.3 (Wilks root-find + convergence gate + Gelman–Rubin R̂).
- **acceptance:**
  - DoE off by default behind `[monte_carlo.doe]`; goldens byte-identical.
  - Wilks two-sided 99.865/90 coverage self-check: empirical coverage ≥ 0.90 (tolerance table, `research`).
  - QMC-vs-MC variance benchmark: Owen-scrambled Sobol error decays faster than `1/√N` on a smooth integrand, `R`-replicate CI brackets truth.
  - Iman–Conover: `|ρ_induced − ρ_target| ≤ 0.02`.
  - Sobol table provenance + SHA pin; native generator (no external Sobol crate).
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line.
- **est_effort:** ~1.5–2 weeks
- **parity_ceiling:** designs verified statistically vs SALib/Dakota offline; no calibrated absolute probabilities.

---

### WP-11.2 — Promote `uq.rs` → `openbmp-uq`: 7009B matrix + correlation-matrix error budget, WIRED

- **title:** First-class credibility + source-tagged error budget, wired into the MC driver
- **goal:** Promote `uq.rs` to `openbmp-uq` (L1); replace `√(Σσ²)+|bias|` with `√(sᵀΣs)` + aleatory/epistemic tags; replace the four-level proxy with the NASA-STD-7009B 8-factor 0–4 matrix (keep min-is-binding); delete the duplicate `ValidationStatus` and depend on the core one; **wire** the budget into the MC driver so every campaign emits a credibility report tied to evidence, fail-closed below a configured floor.
- **implementation_status:** partially implemented. `openbmp-uq` exists as an L1
  pure data/math crate with `CorrelatedErrorBudget`, `CorrelationMatrix`,
  `UncertaintyClass`, and `CredibilityRecord`; the old
  `openbmp-physics::uq` module is a re-export shim; RSS-equivalence,
  non-PSD rejection, aleatory/epistemic masking, and 7009B min-reduction are
  tested under `REQ-UQ-001` / `V-UQ-001`. The generic scalar
  `openbmp mc summarize --uq-toml ... --credibility-floor ...
  --credibility-report-md ...`, `openbmp footprint-mc`, and
  `openbmp mc propulsion-faults` paths now consume that substrate, emit
  deterministic credibility reports, and fail-close below the requested floor
  under `REQ-UQ-002` / `V-UQ-002`. Footprint MC also supports the scenario-owned
  `[landing_footprint.monte_carlo.uq]` manifest table, and propulsion-fault
  campaigns support the fault-library `[uq]` manifest table. Remaining work:
  require future MC drivers to emit the same report automatically.
- **fidelity_tier:** T2
- **depends_on:** [WP-11.0]
- **new_crates:** `openbmp-uq` (L1, alongside `openbmp-state`) — pure data/math, no simulator dep.
- **touched:** `crates/openbmp-physics/src/uq.rs` → new `crates/openbmp-uq/`; `crates/openbmp-mc/` (consume the budget); `crates/openbmp-physics/` (re-export shim for back-compat); CI `check-provenance` lane.
- **approach:** §3.1.1 (`CorrelatedErrorBudget`), §3.1.3 (`CredibilityRecord`, 8 factors, `binding_level`, `legacy_label`). `Σ=I` reproduces legacy RSS exactly.
- **acceptance:**
  - RSS-equivalence: `√(sᵀΣs)` with `Σ=I` byte-identical to legacy `aggregate_one_sigma` on the existing `uq.rs` test vectors.
  - non-PSD `Σ` rejected fail-closed; min-reducer binding-level case exact.
  - `legacy_label()` round-trips the four labels; `check-provenance` linter in CI.
  - a scenario below the credibility floor is labelled academic-only and its requirement verdict refused.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** the credibility floor is itself a safeguard (§6); far from line.
- **est_effort:** ~1.5–2 weeks
- **parity_ceiling:** self-assessment only; no external cert-authority sign-off (§1.2.4).

---

### WP-11.3 — Nested aleatory/epistemic driver + probability box + variance split

- **title:** Double-loop MC with p-box and aleatory/epistemic variance decomposition
- **goal:** Stack two DoE calls into a nested driver; emit the conditional-CDF family, the worst-case (lower) p-box bound for requirement verification, and the `E_θ[Var_a] + Var_θ[E_a]` variance split.
- **implementation_status:** partially implemented. `openbmp-uq` now exposes the
  pure nested-sample reducers (`ProbabilityBox`, `VarianceSplit`) and
  `openbmp-mc` exposes `analyze_nested_scalar_samples` with a
  `NestedLowerTailRequirement` verdict evaluated on the lower p-box bound under
  `REQ-UQ-003` / `V-UQ-003`; `openbmp mc nested-summarize` analyzes
  materialized nested scalar sample CSVs and optionally emits a deterministic
  p-box CSV. `landing_footprint.monte_carlo.nested` is wired through the
  scenario parser, runner, summary TOML, and `openbmp footprint-mc` p-box CSV
  writer using per-source `uncertainty_class` tags. Remaining work: generalize
  the same nested execution pattern beyond landing-footprint campaigns.
- **fidelity_tier:** T3
- **depends_on:** [WP-11.1, WP-11.2]
- **new_crates:** []
- **touched:** `crates/openbmp-uq/` (p-box / Dempster–Shafer arithmetic), `crates/openbmp-mc/` (nested scalar reducer), `crates/openbmp-scenario/` (nested footprint schema), `crates/openbmp-runner/` (nested footprint execution), `crates/openbmp-cli/` (nested summaries and p-box writers).
- **approach:** §3.2.4 (nested loop), §3.1.2 (p-box + total-variance split).
- **acceptance:**
  - nested mode off by default behind `[monte_carlo.nested]`; flat campaigns unchanged (N_e=1 degenerate case byte-identical to WP-11.1).
  - variance-split identity `aleatory + epistemic = total` holds to `rel ≤ 1e-10` on a synthetic separable model.
  - requirement verified on the lower p-box bound, not the mean CDF (asserted).
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** ~1.5 weeks
- **parity_ceiling:** epistemic bands consumed from `03/04/05`, not invented here.

---

### WP-11.4 — Rare-event methods (Subset Simulation + CE-IS) on synthetic limit-states

- **title:** Au–Beck Subset Simulation + cross-entropy IS, forward-only limit-state lock
- **goal:** Estimate P(failure)-class probabilities from 1e3–1e4 runs on **synthetic** limit-states with analytically-known answers; demonstrate the methodology and CoV behaviour honestly; lock the limit-state vocabulary forward-only.
- **implementation_status:** implemented for synthetic scenario manifests.
  `openbmp-mc` exposes the
  synthetic standard-normal limit-state substrate, analytic linear
  `SyntheticLinearLimitState`, a sealed `LimitState` trait, deterministic `SubsetSimulation`,
  deterministic `CrossEntropyIs`, replay-identical rare-event reports with CoV
  and analytic log10-error evidence, a runtime guard against
  non-`synthetic-limit-state` labels, and a compile-fail ground-aimpoint
  tripwire under `REQ-MC-010` / `V-MC-010`. `openbmp-scenario` parses v3
  synthetic rare-event manifests and fail-closes target-like vocabulary under
  `[monte_carlo.limit_state]`. `openbmp mc rare-event` runs the declared
  synthetic estimators and writes deterministic TOML evidence when requested.
- **fidelity_tier:** T3
- **depends_on:** [WP-11.1, WP-11.2]
- **new_crates:** []
- **touched:** `crates/openbmp-mc/` (`LimitState`, `SubsetSimulation`, `CrossEntropyIs`, std-normal input wrappers, MMA sampler); `crates/openbmp-scenario/` (v3 synthetic rare-event manifest and consumer-agreement lint); `crates/openbmp-cli/` (`openbmp mc rare-event` TOML evidence runner); `crates/openbmp-testkit/tests/limit_state_no_aimpoint_compile_fail.rs`.
- **approach:** §3.2.5 (Subset Simulation MMA + adaptive quantiles; CE-IS elite-quantile iteration). Inputs in standard-normal coords; deterministic MCMC per seed.
- **acceptance:**
  - rare-event mode off by default behind `[monte_carlo.subset_simulation]` / `[monte_carlo.cross_entropy]`.
  - analytic linear limit-state `g=β−(1/√d)Σxᵢ`: recovers `P ∈ [1e-3, 1e-6]` with `|log₁₀ P̂ − log₁₀ Φ(−β)| ≤ 0.3` and CoV within the Au–Beck bound (tolerance table, `research`).
  - **new compile-fail/tripwire test** proves a `LimitState` referencing a ground aimpoint is rejected; consumer-agreement lint refuses such a `g`.
  - every rare-event output labelled `synthetic-limit-state`, never bound to a named vehicle.
  - MCMC chains deterministic per seed (replay byte-identical).
  - all §2 gates green.
- **validation_label:** `research` (methodology, on synthetic limit-states)
- **dual_use_note:** **active guardrail (§6a):** the limit-state vocabulary is closed and forward-only; the proving test is mandatory. Never a real P(LOC)/P(LOM) (§1.2.1).
- **est_effort:** ~3–4 weeks
- **parity_ceiling:** synthetic limit-states only; absolute reliability of any real vehicle is structurally out of scope.

---

### WP-11.5 — Code verification: MMS + observed order-of-accuracy + Richardson/GCI

- **title:** Manufactured-solution + order-of-accuracy + GCI harness over integrators and physics RHS
- **goal:** Prove the integrators and ODE RHS solve the equations correctly: observed order `p` matches the scheme's formal order; report a Richardson/GCI numerical-error bar. The most rigorous, data-free credibility return available.
- **fidelity_tier:** T4
- **depends_on:** [WP-11.0]
- **new_crates:** []
- **touched:** `crates/openbmp-testkit/src/analytic.rs` (+ manufactured states, source terms); new `crates/openbmp-testkit/src/verification.rs` (order-of-accuracy + GCI); `crates/openbmp-cli/src/commands/` (+`verify-order`).
- **implementation_status:** implemented for a smooth manufactured scalar ODE
  and the real fixed-step RK4 / DOP853 integrator implementations under
  `REQ-MC-011` / `V-MC-011`. `openbmp verify-order --method all` reports
  RK4 `p≈4.05/4.03` and DOP853 `p≈8.17/8.13` on the current step schedule,
  with deterministic TOML evidence available via `--output-toml`.
- **approach:** §3.2.6 (MMS source-term derivation; `h, h/2, h/4` log-ratio; GCI with `Fs=1.25`).
- **acceptance:**
  - RK4 observed `p ∈ [3.8, 4.2]`; DOP853 observed `p ∈ [7.5, 8.5]` (tolerance tables, `research`).
  - GCI monotone under refinement; brackets the extrapolated error.
  - manufactured states + source terms documented; no external data needed.
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line.
- **est_effort:** ~1–1.5 weeks
- **parity_ceiling:** verification (right-equations-solved-right), not validation against reality.

---

### WP-11.6 — Reconstruction (RTS smoother + batch GN) + NEES/NIS + code-to-code export

- **title:** BET reconstruction half + filter-consistency check + Orekit/GMAT/Basilisk export
- **goal:** Add the missing reconstruction half (RTS backward smoother + batch Gauss–Newton) reusing the existing forward EKF; prove filter consistency (NEES/NIS) on synthetic-truth models; add a code-to-code export adapter; LOCAL-only cross-validate against open Falcon-9-class telemetry.
- **fidelity_tier:** T4
- **depends_on:** [WP-11.0, WP-11.5]
- **new_crates:** []
- **touched:** new reconstruction module (reuses `openbmp-fc` EKF `Φ`/covariances via the bridge, **no new FC edge**); `crates/openbmp-testkit/` (NEES/NIS chi-square helper); `crates/openbmp-cli/src/commands/` (+`reconstruct`, +export adapter).
- **implementation_status:** first synthetic substrate slice implemented under
  `REQ-MC-012` / `V-MC-012`. `crates/openbmp-testkit/src/reconstruction.rs`
  now provides chi-square consistency bands, normalized-error-squared,
  NEES/NIS in-bounds reports, RTS smoothing, and linear batch
  Gauss-Newton. `openbmp reconstruct synthetic-linear` runs the deterministic
  fixture and reports NEES/NIS `0.95` with near-zero batch update error. The
  next FC-observation slice is also implemented under `REQ-MC-013` /
  `V-MC-013`: `openbmp reconstruct observe-fc <scenario.toml>` runs the
  existing read-only SIL monitor, consumes real `estimator.status` innovation
  histories from the in-loop FC, and reports per-sensor NIS consistency
  fractions plus rejected-innovation and covariance-condition diagnostics.
  The follow-on real-FC NEES slice is implemented under `REQ-MC-014` /
  `V-MC-014`: `estimator.status` now publishes position, velocity, and
  attitude covariance diagonals, and `observe-fc` reduces covariance-backed
  position/velocity/attitude estimate-vs-truth NEES samples.
  The code-to-code export-adapter slice is implemented under `REQ-MC-015` /
  `V-MC-015`: `openbmp reconstruct export-trajectory <scenario.toml>
  --output-csv <path>` writes deterministic primary-body ECI trajectory CSV
  suitable for local Orekit/GMAT-style exchange and later comparison through
  the existing external telemetry harness.
  Tool-specific tolerance-pack scaffolding is implemented under `REQ-MC-016` /
  `V-MC-016`: `openbmp reconstruct export-trajectory-mapping --output-toml
  <path> --pack strict|leo-research` writes compare-telemetry mappings for the
  exported CSV columns, and the strict pack round-trips through
  `compare-telemetry` against an OpenBMP-generated reference.
  The current closed-loop attitude-hold scenario is useful diagnostic input but
  not a validation claim; its mag NIS and some NEES channels are visibly
  out-of-family. Remaining WP-11.6 work: expose/full-consume full covariance and
  transition histories for real-FC RTS smoothing, add actual external
  tool-specific code-to-code evidence around the export/compare flow, and keep
  open-telemetry BET comparisons LOCAL-only.
- **approach:** §3.2.7 (RTS recursion; batch GN normal equations via `argmin`; NEES/NIS chi-square bounds). Open-telemetry ingest LOCAL-only (§8, §6b).
- **acceptance:**
  - NEES/NIS on a linear-Gaussian truth (known `Q,R`): in-bounds fraction ∈ [0.93, 0.97] (tolerance table, `research`).
  - two-body/J2 ephemeris reconciles with Orekit/GMAT within stated tolerance (export adapter; `research`).
  - BET-vs-open-telemetry case documented as LOCAL, never committed; no real telemetry in the repo or CI.
  - FC portability lock intact (`fc_dependency_tripwire.rs` green).
  - all §2 gates green.
- **validation_label:** `research` (NEES/NIS + code-to-code); BET-vs-telemetry is LOCAL sanity only.
- **dual_use_note:** **active guardrail (§6b):** telemetry is LOCAL, never a guidance/targeting input. No real BET (§1.2.3).
- **est_effort:** ~2–3 weeks
- **parity_ceiling:** a genuine BET needs non-public telemetry/radar/IMU; NEES/NIS on synthetic truth is the rigorous substitute.

---

### WP-11.7 — Wire per-entry aero/struct/prop UQ into the MC error budget

- **title:** Flow ingested per-discipline uncertainty into `CorrelatedErrorBudget` as epistemic bands
- **status:** first ingest adapter landed under `REQ-UQ-004` / `V-UQ-004`:
  `openbmp-uq::UpstreamMargin` validates bounded upstream deck bands and turns
  them into source-tagged `UncertaintySource` values, and
  `propulsion_c_star_efficiency_margin_source` maps doc-05 thermochemistry
  `c_star_efficiency` bands into epistemic budget sources with credibility
  evidence. Runner gathering, MC campaign consumption, and the remaining
  structural/aero/aerothermal deck adapters are still future work.
- **goal:** Close the loop: consume the per-entry bias/random margins the aero (`03`), aerothermal (`04`), propulsion (`05`), and structural (`02`) database objects produce, mapping them into the source-tagged error budget as epistemic model-form bands, so a campaign's credibility report reflects the *actual* upstream uncertainty rather than hand-set figures.
- **fidelity_tier:** T4
- **depends_on:** [WP-11.2, WP-11.3]
- **new_crates:** []
- **touched:** `crates/openbmp-uq/` (ingest adapter); `crates/openbmp-runner/` (gather per-discipline UQ into the budget); `crates/openbmp-mc/` (campaign consumes it).
- **approach:** §3.1.1–§3.1.2; tag each ingested band epistemic, place it in `Σ` with its declared correlation, surface it in the p-box.
- **acceptance:**
  - per-discipline UQ off by default; campaigns without UQ decks byte-identical.
  - an ascent campaign's epistemic variance share reflects the ingested aero/prop bands (asserted against a synthetic deck with known bands).
  - credibility report cites the upstream evidence pointers (`InputPedigree`/`ResultsUncertainty` factors).
  - all §2 gates green.
- **validation_label:** `validated-toy` (synthetic decks); upstream bands carry their own labels.
- **dual_use_note:** far from line.
- **est_effort:** ~1.5 weeks
- **parity_ceiling:** bands are as good as the upstream decks; validated aero/thermal bands need proprietary data (§1.2.2).

---

### WP-11.8 — Campaign-scale determinism + real-time (consumes `12-…`)

- **title:** rayon order-independent reductions, checkpoint/resume, cross-arch byte-identity
- **goal:** Make 1e5–1e6-run campaigns feasible while preserving byte-determinism: rayon fan-out with Welford-merge-tree reductions bit-identical regardless of thread count; checkpoint/resume; aarch64 FPCR guard; deny-FMA build flags; faster-than-real-time scheduler with a reported real-time factor.
- **implementation_status:** partially implemented for scalar campaign fan-out
  through `run_scalar_campaign_parallel` / REQ-MC-008 and scalar JSON
  checkpoint/resume through `run_scalar_campaign_resumable` plus
  `openbmp mc resume-scalar` / REQ-MC-009. Offline footprint MC also supports
  checkpoint/resume via `openbmp footprint-mc --checkpoint-json`; campaign-scale
  cross-arch byte identity remains open.
- **fidelity_tier:** T5
- **depends_on:** [WP-11.0, WP-11.1]
- **new_crates:** [] (the real-time/compute substrate is owned by `12-determinism-realtime-and-compute.md`)
- **touched:** `crates/openbmp-mc/` (rayon reduction tree, checkpoint store); shared with `12-…` (FPCR guard, build flags).
- **approach:** §4.1 (Welford parallel merge as a tree), §3.3-T5. Per-sample seed independent of worker count; persist completed-index set + reducer state for resume.
- **acceptance:**
  - thread-count byte-identity: K=1 vs K=8 vs K=64 produce bit-identical ensemble statistics (`to_bits()`).
  - checkpoint/resume: an interrupted campaign resumes to the same final statistics as an uninterrupted one.
  - x86_64 and aarch64 produce byte-identical campaign output (FPCR guard + deny-FMA).
  - real-time factor reported; pacing optional and off by default.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** ~2–3 weeks (shared with `12-…`)
- **parity_ceiling:** CPU determinism only; no bit-reproducible f64 GPU parity (§1.2.5).

---

## 10. References

**Design of experiments & quasi-Monte Carlo**

- McKay, Beckman, Conover, "A comparison of three methods for selecting values of input variables," *Technometrics* 21(2), 1979 (LHS).
- Joe & Kuo, "Constructing Sobol sequences with better two-dimensional projections," *SIAM J. Sci. Comput.* 30(5), 2008; direction numbers at `https://web.maths.unsw.edu.au/~fkuo/sobol/`.
- Owen, "Scrambled net variance for integrals of smooth functions," *Ann. Stat.* 25(4), 1997.
- Iman & Conover, "A distribution-free approach to inducing rank correlation among input variables," *Commun. Stat.* 1982.
- Saltelli et al., *Global Sensitivity Analysis: The Primer*, Wiley 2008.

**Tolerance intervals, confidence, sizing**

- Wilks, "Determination of sample sizes for setting tolerance limits," *Ann. Math. Stat.* 12(1), 1941.
- Clopper & Pearson, "The use of confidence or fiducial limits illustrated in the case of the binomial," *Biometrika* 26, 1934.
- Hahn & Meeker, *Statistical Intervals: A Guide for Practitioners*, Wiley 1991.
- Chan, Golub, LeVeque, "Updating formulae and a pairwise algorithm for computing sample variances," 1979 (Welford parallel merge).
- NASA/SP-2010-580 (Hanson & Beard), "Applying Monte Carlo Simulation to Launch Vehicle Design and Requirements Verification," NASA TP-2010-216447.

**Rare-event probability**

- Au & Beck, "Estimation of small failure probabilities in high dimensions by subset simulation," *Probabilistic Engineering Mechanics* 16(4), 2001.
- Zuev, "Subset Simulation Method for Rare Event Estimation: An Introduction," arXiv:1505.03506.
- Rubinstein & Kroese, *The Cross-Entropy Method*, Springer 2004; Kroese et al., *Handbook of Monte Carlo Methods*, Wiley 2011.
- Papaioannou, Geyer, Straub, "Improved cross entropy-based importance sampling," *Reliability Eng. & System Safety*, 2019.

**Aleatory/epistemic separation & UQ frameworks**

- Helton & Johnson, "Quantification of margins and uncertainties: alternative representations of epistemic uncertainty," *Reliability Eng. & System Safety* 96, 2011.
- Roy & Oberkampf, "A comprehensive framework for verification, validation, and uncertainty quantification in scientific computing," *CMAME* 200, 2011.
- Ferson et al., "Constructing Probability Boxes and Dempster-Shafer Structures," Sandia SAND2002-4015.
- NASA Langley Multidisciplinary UQ Challenge (Crespo, Kenny, Giesy), AIAA 2014-1347.

**Credibility & V&V standards**

- NASA-STD-7009B, Office of the NASA Chief Engineer, 2024-03-05.
- NASA-HDBK-7009A companion handbook.
- AIAA G-077-1998(R2002), "Guide for the Verification and Validation of CFD Simulations."
- ASME V&V 10 / V&V 20.

**Code verification**

- Roache, *Verification and Validation in Computational Science and Engineering*, Hermosa 1998; "Code Verification by the Method of Manufactured Solutions," *J. Fluids Eng.* 124(1), 2002.
- Salari & Knupp, "Code Verification by the Method of Manufactured Solutions," Sandia SAND2000-1444.
- Oberkampf & Roy, *Verification and Validation in Scientific Computing*, Cambridge 2010.

**Estimation & reconstruction**

- Tapley, Schutz, Born, *Statistical Orbit Determination*, Academic Press 2004.
- Rauch, Tung, Striebel, "Maximum likelihood estimates of linear dynamic systems," *AIAA J.* 3(8), 1965.
- Crassidis & Junkins, *Optimal Estimation of Dynamic Systems*, 2nd ed., CRC 2011.
- Gelb (ed.), *Applied Optimal Estimation*, MIT Press 1974 (NEES/NIS).
- Bar-Shalom, Li, Kirubarajan, *Estimation with Applications to Tracking and Navigation*, Wiley 2001.

**Determinism & compute**

- Salmon, Moraes, Dror, Shaw, "Parallel random numbers: as easy as 1,2,3" (Random123/Philox), SC11, 2011.
- IEEE 754-2019; Goldberg, "What Every Computer Scientist Should Know About Floating-Point Arithmetic," *ACM Computing Surveys*, 1991.

**Tools** (licenses in §8): Dakota (Sandia), SALib, NASA Earth-GRAM 2016, Orekit, GMAT, Basilisk, `statrs`, `argmin`, `rayon`.

*Upstream OpenBMP anchors:* `docs/verification.md` (five-layer ladder, labels,
tolerance tables, 7009B/G-077 posture), `docs/external-telemetry-validation.md`
(LOCAL-only telemetry policy), `docs/data-provenance.md`,
`docs/launch-vehicle-fidelity-frontier.md` (slosh-closure convergence risk).

*Companion parity documents:* `00-overview.md`, `02-structural-dynamics-loads-
slosh-pogo.md`, `03-aerodynamics-database-and-cfd-coupling.md`,
`04-aerothermal-realgas-and-tps.md`, `05-propulsion-high-fidelity.md`,
`06-gnc-coupled-mimo-and-control.md`, `07-trajectory-optimization-and-mission-
design.md`, `08-environment-gravity-and-frames.md`, `09-sensors-navigation-and-
actuators.md`, `10-flight-software-in-the-loop-xil.md`,
`12-determinism-realtime-and-compute.md`, `13-agent-execution-playbook.md`.
