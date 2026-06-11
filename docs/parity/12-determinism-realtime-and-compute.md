# Determinism, Real-Time & Compute

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent implementing the parity work packages.
**One-line summary:** Preserve OpenBMP's byte-determinism under heavy physics and
massively-parallel Monte Carlo (order-independent Welford-merge-tree reductions,
per-run keyed streams), add a shared soft-/faster-than-real-time frame pacer
(`openbmp-rt`) for the XIL bench, add checkpoint/resume for preemptible HPC, and
close the audit's four "approaching" minor gaps — the aarch64 FP-environment
guard, `check-provenance` in CI, optional dense output for adaptive integrators,
and a provenance-pinned GPU offload *boundary* for ingested-deck generation
(never the live f64 ODE loop).

> Read `00-overview.md` (constitution, invariants, DAG, solver-consumer posture)
> and `13-agent-execution-playbook.md` (§4 template, §5 backlog schema, gate set)
> first. This is the only audited dimension already rated *approaching*; the
> target is **at-parity** (deterministic parallel MC + soft-real-time pacing +
> checkpointing), with four narrow gaps to close. Determinism is not a feature of
> this dimension — it is the **substrate every other doc stands on**, so this doc
> is deliberately conservative: it adds compute machinery *around* the locked
> kernel without ever touching the locked operand order, the locked tableaux, or
> the canonical-time recurrence.

---

## 1. Parity target & ceiling

### 1.1 Target (capability, within posture)

Production launch-vehicle simulators (POST2, Trick-based 6-DOF decks, Dakota
campaign drivers, OPAL-RT/Concurrent-RT HIL benches) deliver three compute
properties OpenBMP must match in *method and architecture*:

1. **Deterministic, reproducible large-N Monte Carlo at cluster scale** — `1e5`–
   `1e6` independent 6-DOF runs whose aggregate statistics are *exactly*
   reproducible regardless of how many workers ran them, in what order they
   completed, or whether the campaign was preempted and resumed. This is what
   makes a dispersion result auditable and a regression bisectable.
2. **Soft-real-time and faster-than-real-time frame pacing** — a fixed-step loop
   that can run at, above, or (paced) exactly at wall-clock, with an honest
   real-time-factor and a jitter/overrun budget. This is the substrate the XIL
   doc (`10-flight-software-in-the-loop-xil.md`) Tier-3 HIL-software bench needs;
   it is co-owned here because the pacing primitive is a pure compute concern.
3. **Cross-architecture bit-reproducibility** — the same byte-identical output on
   the reference profile *and* on the other architectures the project actually
   runs CI on, so reproducibility is not silently x86_64-only.

OpenBMP already has the hard part — a locked, FMA-free, canonical-time kernel
with a domain-separated keyed RNG and a CI byte-diff gate. The target is to wrap
that locked core in:

- an **order-independent parallel reduction** layer (Welford merge tree) so
  `rayon`-parallel sample fan-out is bit-identical to a serial fold;
- a **per-run keyed-stream** discipline (counter-based RNG philosophy) so a
  sample's result depends only on `(campaign_seed, sample_index)`, never on
  thread count or completion order;
- a **checkpoint/resume store** that persists the completed-index set plus
  reducer state, so a preempted campaign resumes without recomputation and
  without changing a single output bit;
- a **frame pacer** (`openbmp-rt`) giving real-time / faster-than-real-time
  scheduling with a measured jitter histogram and overrun detector;
- and the **four gap closures**: aarch64 FP-environment guard, `check-provenance`
  wired into CI, optional dense output for the adaptive integrators (label-gated,
  off the bit-stable path), and a GPU offload *boundary* restricted to offline
  ingested-deck generation with provenance + UQ on the ingested product.

### 1.2 Parity ceiling (honest)

What an open, simulation-only repo **cannot** match here, and the credible open
substitute:

- **Bit-reproducible f64 GPU parity.** Production decks may run `1e6` cases on lab
  clusters, some GPU-accelerated. Reproducible f64 on GPU is a known determinism
  hazard (non-associative warp reductions, atomics, vendor `libm`, driver and
  arch-dependent fused-multiply-add). OpenBMP **will not chase a bit-reproducible
  f64 GPU ODE loop**. *Open substitute:* deterministic CPU parallelism +
  checkpointing, with QMC variance reduction (doc 11) cutting the required N, and
  a strictly-bounded GPU role limited to **offline ingested-deck generation**
  whose product is provenance-pinned, UQ-tagged, and reproduced through the
  deterministic CPU path (T4 below). GPU output never enters the live loop.
- **Certified hard-real-time on flight-grade hardware.** True bounded-latency on a
  rad-hard processor with a real RTOS, isolated cores, and qualified bus
  electricals is out of scope (and is the XIL doc's HIL ceiling). *Open
  substitute:* a soft-real-time bench on a commodity host (isolated CPU +
  `SCHED_FIFO`/`clock_nanosleep`) with a `cyclictest`-style jitter histogram and
  an overrun detector — surfacing deadline behavior the host SIL hides, with
  measured-not-certified numbers.
- **Cross-platform byte equality outside the declared reference profile.**
  Bit-identical replay is contractually guaranteed only on the reference profile
  (target triple + toolchain + LLVM opt level + clean FP environment). Other
  platforms are `state-stable`, not `bit-stable`, because platform `libm`
  `pow()`/`ln()` differ. The aarch64 work (T3) *narrows* this to a second declared
  bit-stable profile for the fixed-step integrators (no `libm`-transcendental in
  their hot path) but does **not** promise universal cross-platform byte equality.
- **Verifying determinism is a property of the methodology, not a guarantee of
  correctness.** A byte-identical wrong answer is still wrong; determinism is
  reproducibility, and the V&V plan (doc 11) is what earns the credibility labels.

This doc labels its tiers honestly (`checked` → `validated-toy`) and never claims
flight readiness, certification, or operational status.

---

## 2. Current state in source

Verified against the working tree (paths are load-bearing; regress against them).

### 2.1 What already exists and is strong

- **Domain-separated keyed RNG.** `crates/openbmp-core/src/rng.rs` —
  `DeterministicRng` wraps `rand_chacha::ChaCha8Rng` and reads **no** system
  entropy. Constructors `from_raw_seed`, `from_seed`, `for_channel`,
  `for_sensor_component`, `for_effector_component`, `for_stimulus_component`,
  `for_wind_component` each pack a `(scenario_seed, step, id, component)` tuple
  into the 32-byte seed with a 4-byte domain tag (`b"SENS"`, `b"EFFC"`, `b"WIND"`,
  `b"STIM"`) so families never collide. Pinned reference-stream tests
  (`*_reference_stream_is_locked`) fail on any byte drift. This **already follows
  the counter-based / keyed-stream philosophy** the research pack names
  (Random123/Philox-style): each draw reconstructs its stream from a tuple with no
  shared cursor — the exact property a thread-count-independent parallel MC needs.
  The campaign layer just adds a *new* domain (`(campaign_seed, sample_index)`).

- **Locked, FMA-free integrators with label-gated dense output.**
  `crates/openbmp-sim/src/integrator.rs` —
  - `rk4_weighted_sum` (lines ~63–72) computes `(k1 + 2·k2 + 2·k3 + k4)/6` in
    **explicit-parenthesized locked order**, single final scalar multiply, with a
    module-doc contract banning `f64::mul_add` ("FMA hardware rounds once;
    software emulation rounds twice").
  - `Rk4FixedStep`, `Dopri54FixedStep`, `Dopri853FixedStep` are tagged
    `IntegratorDeterminism::BitStable`; every stage increment is a
    locked-order left-fold (`((k1*A) + (k2*B)) + ...`).
  - `Dopri54Adaptive` / `Dopri853Adaptive` are tagged
    `IntegratorDeterminism::StateStable` (cross-platform `pow()`/`ln()` in the PI
    controller). PI controller per Gustafsson 1991 (`PI_ALPHA=0.7`,
    `PI_BETA=0.4`, `SAFETY=0.9`).
  - **Adaptive dense output is opt-in and state-stable.**
    `Dopri54Adaptive::advance_with_dense_output` records accepted substep stages
    and exposes `Dopri54DenseOutput`, a quartic continuous extension;
    `Dopri853Adaptive::advance_with_dense_output` evaluates the SciPy/Hairer
    order-7 interpolant using the endpoint derivative plus three extra
    dense-output stages.

- **Canonical time.** `crates/openbmp-sim/src/kernel.rs` — the step loop
  overwrites the integrated time with `start + step * dt` via a single
  multiplication: `let canonical_time_s = self.initial_time_s + (next_step.value()
  as f64) * self.dt_s;` (line ~635). The module doc (lines 8–25) states the full
  determinism contract: canonical multiplication (no `t += dt` accumulation),
  clean FP environment, locked weighted-sum, deterministic sub-step times, and
  tracing-bytes-excluded-from-output.

- **FP-environment guard (x86_64 + aarch64).** `openbmp-core::FpEnvironment`
  decodes x86_64 MXCSR via inline `stmxcsr` and aarch64 FPCR via inline
  `mrs {x}, fpcr`. It rejects MXCSR FTZ/DAZ/non-nearest rounding and FPCR
  FZ/FZ16/non-nearest RMode. `openbmp-sim::SimulationKernel` calls
  `FpEnvironment::assert_current_clean()` at point-mass and rigid-body
  construction, preserving the public `SimulationError::FpEnvironmentDirty`
  surface.

- **FMA contraction is disabled for declared bit-stable targets.**
  `.cargo/config.toml` sets `-C target-feature=-fma` for
  `[target.x86_64-unknown-linux-gnu]` and `-C llvm-args=--fp-contract=off` for
  `[target.aarch64-unknown-linux-gnu]` and `[target.aarch64-apple-darwin]`.
  `incremental = false` is global.

- **CI determinism gate.** `.github/workflows/ci.yml` `determinism` job runs five
  canonical scenarios twice (plus once with `RUST_LOG=trace` redirected) on
  `x86_64-unknown-linux-gnu` and byte-diffs the Parquet/CSV via `openbmp diff` /
  `diff -q`. `cross-platform` job runs macOS/Windows as `state-stable` smoke only.
  A weekly `monte-carlo-nightly` lane (cron `17 3 * * 0`) runs the footprint and
  Phalcon-9 MC determinism/robustness tests.

### 2.2 What is missing or single-threaded (the work)

- **Parallel MC substrate is partially closed.** `openbmp-mc` now exposes
  `run_scalar_campaign_parallel`, which evaluates keyed scalar samples over a
  bounded `rayon` worker pool, checks the worker-side FP environment before
  sample evaluation, sorts by `sample_index`, and reduces with the fixed Welford
  merge tree. The unit evidence proves bit-identical reports for
  `worker_count ∈ {1,2,4,8}`.
- **Checkpoint/resume substrate is partially closed.** `openbmp-mc` now exposes
  `ScalarCampaignCheckpoint`, `FileCheckpointStore`, and
  `run_scalar_campaign_resumable`; the JSON checkpoint stores completed-index
  reducer leaves plus scenario/toolchain metadata, and tests prove
  one-shot == chunked-resume report bits. `openbmp mc resume-scalar` exposes the
  same checkpoint/resume path for precomputed scalar sample CSVs. The offline
  runner footprint path now supports opt-in `--checkpoint-json` /
  `--max-new-samples` checkpointing and proves resumed CSV/TOML outputs are
  byte-identical to one-shot outputs.
- **`openbmp-rt` crate present as the shared timing substrate.** The crate now
  exposes `PaceMode`, `FramePacer`, signed `JitterHistogram` percentiles,
  overrun counting, `real_time_factor()`, and the Liu-Layland schedulability
  advisory under [REQ-RT-002]. It remains host-side L7, depends only on
  `openbmp-core` plus `std`, and is tested to leave functional state identical
  across FreeRun / RealTime / Paced modes. The runner consumes it through an
  opt-in v3 `[realtime]` block, accepts optional `[[realtime.task]]` periodic
  budgets, and reports jitter, frame execution timing, frame-budget overruns,
  and the schedulability advisory outside telemetry.
- **No distributed checkpoint/resume.** Checkpointing is single-process /
  single-store. Distributed or multi-node sample ownership is not implemented.
- **`check-provenance` is wired into CI.** The CLI command is implemented
  (`crates/openbmp-cli/src/main.rs` `Command::CheckProvenance`,
  `crates/openbmp-cli/src/cli.rs` `CheckProvenance { root }`,
  `crates/openbmp-cli/src/commands/provenance.rs`) and prints "N files seen, M
  without provenance.md". The `.github/workflows/ci.yml` `provenance` job builds
  `openbmp-cli`, then runs `openbmp check-provenance data` and
  `openbmp check-provenance scenarios`; generated scenario `out/` telemetry
  directories are skipped by the CLI walker.
- **aarch64 FP guard + FMA ban present.** `openbmp-core::FpEnvironment` now
  decodes aarch64 FPCR and `.cargo/config.toml` carries aarch64
  `--fp-contract=off` stanzas.
- **Adaptive dense output is wired for DOPRI5 and DOP853.** Both are opt-in,
  state-stable, and rejected outside `adaptive-explicit` profiles.

The takeaway: the **determinism foundation is solid and the keyed-stream design
is already correct for parallelism**; the work is additive compute machinery plus
four narrow gap closures, none of which touch the locked operand order.

---

## 3. Target architecture

### 3.1 New / changed crates

```text
NEW:
  openbmp-rt          L7   soft-/faster-than-real-time frame pacer + jitter
                           accounting + overrun detection (shared with doc 10)

CHANGED:
  openbmp-core        L0   + DeterministicRng::for_mc_sample (campaign domain);
                           + portable FpEnvironment guard (x86_64 MXCSR moves
                             here from openbmp-sim; aarch64 FPCR added; others
                             no-op) so the guard is reusable by openbmp-rt and
                             by any worker thread, not only the sim kernel.
  openbmp-sim         L2   integrator.rs: optional dense-output trait
                             (StateStable-only, label-gated); kernel calls the
                             relocated openbmp-core FP guard.
  openbmp-mc          L7   (doc 11 owns the crate) — this doc contributes the
                             OrderIndependentReducer + WelfordMerge + CheckpointStore
                             + ParallelDriver primitives it is built on.
  openbmp-cli         —    + `mc resume` / checkpoint flags; `check-provenance`
                             wired into a CI job (no code change, a workflow change).
  .cargo/config.toml  —    + aarch64 target stanza (target-feature=-fma path or
                             equivalent contraction ban) once a second bit-stable
                             profile is declared.
  .github/workflows/ci.yml — + provenance job; + aarch64 determinism lane (T3).
```

`openbmp-rt` placement justification (review before building): it sits at **L7**
(runner/tooling tier) next to `openbmp-runner`/`openbmp-bridge`. It depends only
on `openbmp-core` (for the relocated `FpEnvironment` guard and `SimTime`) and
`std`. It must **never** be depended on by `openbmp-fc` (would break the FC
hardware-portability lock — pacing is a host concern; the FC reads only the
injected `Clock` trait). The bridge/XIL master *consumes* `openbmp-rt`, the FC
does not. The MXCSR→`openbmp-core` relocation must keep the `unsafe_code` deny
intact by confining the single `asm!` block behind an `#[allow(unsafe_code)]`
with the existing SAFETY comment, exactly as today.

### 3.2 Determinism math — the load-bearing equations

#### 3.2.1 Per-run keyed streams (thread-count independence)

A campaign of `N` samples must satisfy: the result of sample `i` is a pure
function of `(campaign_seed, i)` and nothing else — not the worker that ran it,
not the number of workers, not completion order. The existing
`DeterministicRng` keying already guarantees this *within* a run (each draw
reconstructs its stream from a tuple). The campaign adds one domain:

```
seed_bytes[ 0.. 8] = campaign_seed.to_le_bytes()
seed_bytes[ 8..16] = (sample_index as u64).to_le_bytes()
seed_bytes[16..24] = 0           (reserved: future per-input-block id)
seed_bytes[24..28] = component.to_le_bytes()
seed_bytes[28..32] = b"MCSM"     (new domain tag, distinct from SENS/EFFC/WIND/STIM)
DeterministicRng::for_mc_sample(campaign_seed, sample_index, component)
```

This is the **counter-based RNG** philosophy (Salmon et al., *Parallel random
numbers: as easy as 1,2,3*, SC11 — Philox keyed streams): the i-th stream is
addressable in O(1) with no sequential dependence, so `rayon` can hand sample `i`
to any thread and the bytes are identical. A new `b"MCSM"` domain tag plus a
pinned reference-stream test (mirroring the existing `*_reference_stream_is_locked`
tests) keeps the campaign RNG byte-locked.

#### 3.2.2 Order-independent reduction (Welford merge tree)

A naive parallel sum `Σ x_i` is **not** bit-identical to a serial fold because
floating-point addition is non-associative. The fix is a reducer whose *merge*
operation produces the same result regardless of association — achievable two
ways, both shipped:

**(a) Welford parallel-variance merge** (Chan–Golub–LeVeque 1979; Chan et al.
parallel variance). Each partial accumulates a triple `(n, mean, M2)`:

```
update(x):   n' = n + 1
             δ  = x − mean
             mean' = mean + δ / n'
             M2'   = M2 + δ · (x − mean')

merge(A, B): n  = n_A + n_B
             δ  = mean_B − mean_A
             mean = mean_A + δ · (n_B / n)
             M2   = M2_A + M2_B + δ² · (n_A · n_B / n)
sample_variance = M2 / (n − 1)
MC standard error = sqrt( M2 / (n · (n−1)) )
```

The merge is **commutative and associative to the bit** *only if the partition is
deterministic*. Therefore the parallel driver does **not** reduce in completion
order; it reduces over a **fixed binary merge tree keyed by `sample_index`** — a
balanced fold whose shape is determined by the index set, not by which worker
finished first:

```
reduce_tree(indices: sorted [0..N)):
  leaves[i] = welford_singleton(result[i])     # one sample → (1, x_i, 0)
  while len > 1:
    next[j] = merge(level[2j], level[2j+1])      # adjacent pairs, fixed shape
  return level[0]
```

Because the tree shape is a pure function of `N` (and of the *set* of completed
indices on resume — see §3.2.4), the reduced `(n, mean, M2)` triple is
bit-identical across any thread count and any completion order. This is the
project's hard requirement from `00-overview.md` §8 risk register ("the byte-diff
gate extends to ensemble statistics").

**(b) Sort-then-fold fallback for non-Welford aggregates.** For statistics with
no commutative-merge form (e.g. a strict ordered sum used in a quantile, or a
Kahan-compensated total), the driver collects per-sample scalars into a vector
**indexed by `sample_index`**, sorts by index (a no-op if already dense), and
folds in index order. Identical to the serial fold by construction. Used for the
Clopper-Pearson success-count and the order-statistics a Wilks tolerance interval
needs (doc 11).

> **Invariant:** the reducer never iterates a `HashMap` or a completion-order
> queue. All folds are over `sample_index`-sorted sequences or over a
> fixed-shape merge tree. This is the §3 (unordered-iteration ban) invariant made
> concrete for the parallel path.

#### 3.2.3 Floating-point environment normalization (cross-arch)

Bit-stable replay requires every worker thread, on every architecture, to run in
the **same** FP environment: round-to-nearest-ties-to-even, FTZ off, DAZ off, no
FMA contraction. The guard becomes a portable `FpEnvironment` checker in
`openbmp-core`:

```
x86_64:  read MXCSR (stmxcsr); reject if FTZ(bit15) | DAZ(bit6) | RC(bits13–14)≠0
aarch64: read FPCR;            reject if FZ(bit24) | FZ16(bit19) | RMode(bits22–23)≠0
                               (FPCR.FZ = flush-to-zero, the aarch64 analogue of
                                MXCSR FTZ+DAZ; RMode 00 = round-to-nearest-even)
other:   no-op (state-stable only; documented, not silently trusted)
```

The aarch64 path reads FPCR via `mrs {x}, fpcr` behind the same single
`#[allow(unsafe_code)]` SAFETY-commented block pattern as the x86_64 `stmxcsr`.
The contraction ban is enforced in the build (`.cargo/config.toml` aarch64 stanza
disabling FMA contraction) so the fixed-step integrators — which contain **no
`libm` transcendental in their hot path** — become a *second declared bit-stable
profile*. The adaptive integrators stay `StateStable` (their PI controller calls
`pow()`/`ln()`).

The worker pool calls `FpEnvironment::assert_clean()` **once per worker thread at
spawn**, not per sample, so the guard cost is amortized and a thread that
inherited a dirty environment (e.g. a transitive dependency that set FTZ) fails
closed before producing any sample.

#### 3.2.4 Checkpoint / resume (bit-invariant)

A preemptible campaign persists, at a configurable cadence (every `k` completed
samples or `t` wall-seconds):

```
Checkpoint {
  campaign_seed:       u64,
  scenario_sha256:     [u8; 32],          # pins the exact scenario bytes
  reducer_state:       Welford triples per QoI,   # NOT the raw samples
  completed_indices:   RoaringBitmap / sorted Vec<u32>,
  schema_version:      u32,
  toolchain_profile:   String,            # target triple + rustc + opt level
}
```

Resume **skips** completed indices and recomputes only the remainder. The
**bit-invariance theorem** the implementation must preserve: *the final reduced
triple is identical whether the campaign ran in one shot or was checkpointed and
resumed K times*, because (a) each sample's value depends only on
`(campaign_seed, i)`, and (b) the merge tree is rebuilt from the **full set** of
completed indices at finalization, not incrementally folded in
completion/resume order. (Incremental folding into a running accumulator would be
association-order-dependent and is therefore **banned**; the checkpoint stores
the per-leaf or per-subtree Welford state keyed by index range, and finalization
re-merges over the fixed tree.) A resume that observes a `scenario_sha256` or
`toolchain_profile` mismatch fails closed — a different scenario or profile is a
different campaign.

#### 3.2.5 Frame pacing & jitter accounting (`openbmp-rt`)

Soft-real-time loop, absolute-time scheduling (no drift accumulation), per
Liu–Layland schedulability:

```
for k in 0..:
    t_k       = t_0 + k · h_c              # minor-frame boundary (canonical, no +=)
    clock_nanosleep(TIMER_ABSTIME, t_k)    # sleep TO an absolute instant
    t_release = now()
    jitter_k  = t_release − t_k            # signed; histogram p50/p99/p99.9/max
    if jitter_k > jitter_budget: overrun_count += 1
    run_frame()                            # fixed-step plant; constant compute budget
    t_done    = now()
    rtf       = sim_seconds_advanced / (t_done − t_0)   # real-time factor (reported)
```

`mode ∈ { FreeRun (as fast as possible, rtf reported), Paced(target_rtf),
RealTime(rtf=1) }`. The pacer **must not** be on the determinism path: pacing
changes *when* a frame runs, never *what* it computes (the plant integrator is
the same fixed-step locked kernel). Jitter is *measured and reported*, never fed
back into state. Schedulability self-check against the rate-monotonic bound
`U ≤ n(2^(1/n) − 1)` (Liu & Layland 1973) is emitted as an advisory, not a gate.

### 3.3 Rust trait surfaces

```rust
// ---- openbmp-core: portable FP environment guard (relocated + extended) ----

/// Result of inspecting the host floating-point control register.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct FpEnvironment {
    pub flush_to_zero: bool,   // MXCSR FTZ | DAZ  (x86_64);  FPCR.FZ (aarch64)
    pub rounding_mode: u32,    // 0 == round-to-nearest-ties-to-even
    pub arch_checked: bool,    // false on architectures with no portable read
}

impl FpEnvironment {
    /// Read the host control register. No-op (`arch_checked=false`) on
    /// architectures without a portable read.
    #[must_use]
    pub fn read() -> Self;

    /// Returns `Ok(())` iff round-to-nearest-ties-to-even and FTZ/DAZ off,
    /// or the architecture is unchecked. Fails closed otherwise so a dirty
    /// environment cannot silently corrupt bit-stable replay.
    pub fn assert_clean(&self) -> Result<(), FpEnvironmentDirty>;
}

/// Error returned when the FP environment is not in the determinism-contract
/// state. Mirrors the existing `SimulationError::FpEnvironmentDirty` payload
/// so the relocation is a move, not a behavior change.
#[derive(Copy, Clone, Debug, Eq, PartialEq, thiserror::Error)]
#[error("floating-point environment is dirty (ftz/daz={flush_to_zero}, \
         rounding_mode={rounding_mode}); cannot guarantee bit-stable replay")]
pub struct FpEnvironmentDirty {
    pub flush_to_zero: bool,
    pub rounding_mode: u32,
}

// DeterministicRng gains one constructor; existing constructors untouched.
impl DeterministicRng {
    /// Per-Monte-Carlo-sample keyed stream. Domain tag `b"MCSM"`.
    /// Result of sample `sample_index` depends only on
    /// `(campaign_seed, sample_index, component)` — never on thread
    /// count or completion order (counter-based / Philox philosophy).
    #[must_use]
    pub fn for_mc_sample(campaign_seed: u64, sample_index: u32, component: u32) -> Self;
}

// ---- openbmp-mc primitives this doc contributes (crate owned by doc 11) ----

/// A bit-stable, order-independent accumulator. The *merge* must be exact
/// (associative-to-the-bit when the partition is deterministic).
pub trait OrderIndependentReducer: Clone + Send {
    type Sample;
    type Output;
    fn singleton(sample: &Self::Sample) -> Self;
    /// Exact merge of two partials. MUST be invoked only over a fixed,
    /// `sample_index`-keyed merge-tree shape — never in completion order.
    fn merge(self, other: Self) -> Self;
    fn finalize(&self) -> Self::Output;
}

/// Streaming Welford triple (n, mean, M2). Implements OrderIndependentReducer.
#[derive(Copy, Clone, Debug)]
pub struct Welford { pub n: u64, pub mean: f64, pub m2: f64 }
impl Welford {
    #[must_use] pub fn variance(&self) -> f64;       // M2 / (n-1)
    #[must_use] pub fn std_error(&self) -> f64;      // sqrt(M2 / (n(n-1)))
}

/// Deterministic parallel campaign driver. `run` and `resume` produce a
/// bit-identical reduced output regardless of `worker_count`.
pub trait ParallelCampaign {
    type Reducer: OrderIndependentReducer;
    /// Pure function of (campaign_seed, sample_index): the per-sample work.
    fn evaluate(&self, campaign_seed: u64, sample_index: u32)
        -> Result<<Self::Reducer as OrderIndependentReducer>::Sample, CampaignError>;
    /// Fan out [0..n) over `worker_count` workers; reduce over the fixed
    /// index-keyed merge tree; identical bytes for any worker_count.
    fn run(&self, campaign_seed: u64, n: u32, worker_count: usize,
           ckpt: Option<&mut dyn CheckpointStore>) -> Result<CampaignResult<Self>, CampaignError>;
    fn resume(&self, ckpt: &mut dyn CheckpointStore) -> Result<CampaignResult<Self>, CampaignError>;
}

/// Persists completed-index set + per-subtree reducer state, fail-closed on
/// scenario/profile mismatch. Stores Welford state, NOT raw samples.
pub trait CheckpointStore {
    fn save(&mut self, ck: &Checkpoint) -> Result<(), CheckpointError>;
    fn load(&mut self) -> Result<Option<Checkpoint>, CheckpointError>;
}

// ---- openbmp-rt: frame pacer (shared with 10-flight-software-in-the-loop-xil.md) ----

#[derive(Copy, Clone, Debug)]
pub enum PaceMode { FreeRun, Paced { target_rtf: f64 }, RealTime }

/// Drives a fixed-step loop, measuring jitter and overruns. Never touches the
/// plant's computed state — pacing changes *when*, never *what*.
pub trait FramePacer {
    /// Block until the next minor-frame boundary `t_0 + k·h_c` (absolute-time).
    fn wait_next_frame(&mut self) -> FrameTiming;
    fn jitter_histogram(&self) -> &JitterHistogram;   // p50/p99/p99.9/max
    fn real_time_factor(&self) -> f64;                // sim_s / wall_s
    fn overrun_count(&self) -> u64;
}

#[derive(Copy, Clone, Debug)]
pub struct FrameTiming {
    pub frame_index: u64,
    pub scheduled: SimTime,        // canonical t_0 + k·h_c
    pub jitter_ns: i64,            // signed (release − scheduled)
    pub overran: bool,
}
```

### 3.4 Fidelity tiers

The tiers are independently shippable; nothing above T0 is required for the one
below it to be useful (the playbook's incremental-value rule).

- **T0 — Determinism gap closures (the cheap, high-value first PRs).**
  (a) Wire `check-provenance` into CI as its own job over `scenarios/` + `data/`.
  (b) Relocate the x86_64 MXCSR guard into `openbmp-core::FpEnvironment` (move,
  no behavior change) and add the **aarch64 FPCR** read + a build-flag contraction
  ban for aarch64, declaring it a second bit-stable profile for the fixed-step
  integrators. (c) Add `DeterministicRng::for_mc_sample` + pinned reference
  stream. *No new physics, no parallelism yet.* Earns `checked`.

- **T1 — Order-independent reduction primitives.** `OrderIndependentReducer`,
  `Welford`, and the fixed index-keyed merge tree, with the **serial** driver
  first (single-thread fold == merge-tree result, asserted bit-identical).
  Replaces the ad-hoc footprint reduction with a reusable reducer. Earns
  `validated-toy` (associativity/commutativity proved by property test +
  analytic-variance MMS-style case).

- **T2 — Deterministic parallel fan-out + checkpoint/resume.** `rayon`
  (MIT/Apache-2.0) scalar fan-out is implemented in `openbmp-mc` with
  bit-identical reduced output for `worker_count ∈ {1,2,4,8}`. File-backed scalar
  checkpoint/resume is implemented in `openbmp-mc` and exposed as
  `openbmp mc resume-scalar` for precomputed scalar sample CSVs, with one-shot
  == K-resume bit identity. Offline footprint MC checkpoint/resume is exposed via
  `openbmp footprint-mc --checkpoint-json ... --max-new-samples ...`, with
  resumed final CSV/TOML byte-identical to one-shot output.

- **T3 — Soft-/faster-than-real-time pacing (`openbmp-rt`).** Fixed-step loop
  with absolute-time `clock_nanosleep`, `PaceMode` selection, jitter histogram,
  overrun detector, real-time-factor reporting; schedulability advisory.
  Add an **aarch64 determinism CI lane** that byte-diffs the fixed-step canonical
  scenarios on an aarch64 runner against the locked reference bytes (closing the
  cross-arch reproducibility loop opened in T0). Earns `validated-toy` for the
  pacer determinism-independence; the jitter numbers are `checked` (measured, not
  certified).

- **T4 — Dense output (label-gated) + GPU offload boundary for ingested decks.**
  (a) An optional `DenseOutput` trait for the **adaptive** integrators only
  (`StateStable`), implementing the DOP853 order-7 / DOPRI5 4th-order
  interpolants, **off the bit-stable path and explicitly `state-stable`-labeled**;
  used for event localization / output resampling, never for the canonical
  fixed-step gate. (b) A strictly-bounded **GPU offload boundary**: GPU may
  generate an *offline* ingested deck (e.g. a large aero/atmosphere perturbation
  table) whose product lands in `data/` with `provenance.md`, a SHA-256 pin, a
  UQ tag, and a **deterministic-CPU reproduction case** for a sampled subset
  (solver-consumer posture: the GPU is an external producer, not the live loop).
  GPU output **never** enters the f64 ODE kernel. Earns `checked` (dense output:
  `state-stable` interpolation error vs analytic; GPU boundary: provenance +
  subset reproduction, not a determinism claim on the GPU itself).

---

## 4. Invariant preservation

How each `00-overview.md` §3 invariant is concretely upheld for *this* dimension.

- **Byte-determinism (Invariant 1).** This is the dimension's whole point. New
  randomness flows through `DeterministicRng::for_mc_sample` (no system RNG, no
  wall-clock). The parallel reducer is **order-independent by construction** —
  fixed index-keyed merge tree (§3.2.2), never a `HashMap`/completion-order fold —
  so the byte-diff gate extends to ensemble statistics (a new CI assertion:
  reduced triple identical for `worker_count ∈ {1,2,4,8}`). No `f64::mul_add`
  anywhere; the locked tableaux and `rk4_weighted_sum` are untouched. The frame
  pacer is provably off the determinism path (it changes *when*, never *what*).

- **Byte-stable-by-default (Invariant 2).** Parallel MC, checkpointing, dense
  output, and pacing are **all opt-in**: a scenario without a `[campaign]` /
  `[realtime]` block runs exactly as today (serial, bit-identical). The existing
  five determinism-gate scenarios stay byte-identical because none of them opt in.
  Dense output is gated behind an explicit `dense_output = true` flag and only on
  the already-`StateStable` adaptive DOPRI5 / DOP853 paths; unsupported methods
  fail closed, so the bit-stable fixed-step goldens are unaffected.

- **FC hardware-portability lock (Invariant 3).** `openbmp-rt` is **L7** and is
  forbidden from the `openbmp-fc` dependency graph (it would inject a host pacing
  concern into the controller). The FC continues to read time only through the
  injected `Clock` trait (Invariant 4); the pacer lives entirely on the
  simulator/bridge side. The `fc_dependency_tripwire.rs` allow-list is **not**
  extended to `openbmp-rt`; if a future edge appears, the tripwire fails closed.

- **Lockstep-clock & no-hot-path-allocation (Invariant 4).** The pacer uses
  absolute-time scheduling (`t_0 + k·h_c`, canonical, no `+=` drift) and pre-
  allocates its jitter histogram and frame buffers — no per-frame allocation. The
  parallel driver pre-sizes its per-worker reducer vector by `worker_count` and
  reduces over a pre-shaped tree; no per-sample allocation in the fold.

- **Four-pillar provenance (Invariant 5).** T0's first deliverable **closes the
  exact gap the audit named**: `check-provenance` becomes a CI job, so every
  `data/`/`scenarios/` TOML's sibling `provenance.md` + SHA-256 pin is enforced
  on-disk, not only by the inline-data unit test. Any ingested deck from the T4
  GPU boundary lands under `data/` with `provenance.md` + SHA pin + UQ tag or the
  job fails. **No WGS84 numerals are introduced** by any compute primitive (the
  reducers and pacer are pure numerics over passed-in state).

- **Validation labels (Invariant 8).** Each tier declares its label (§3.4): T0
  `checked`; T1/T2/T3 `validated-toy`; T4 `checked`. The dense-output
  interpolation is explicitly `state-stable`, never promoted to `bit-stable`. No
  artifact claims `flight-qualified`/`certified`. A tolerance-table TOML
  accompanies any numeric claim (interpolation error, jitter percentiles).

- **Fail-closed (Invariant 9).** `FpEnvironment::assert_clean` returns a
  `Result`, never panics; a dirty worker fails before producing a sample. A
  resume with mismatched `scenario_sha256`/`toolchain_profile` returns a typed
  `CheckpointError`. An overrun is recorded, not panicked-on.

- **Requirements traceability (Invariant 10).** Each WP adds a `requirements.toml`
  entry with the CI job / test as verification evidence (e.g. the
  worker-count-invariance test, the resume-invariance test, the aarch64
  determinism lane).

---

## 5. V&V plan

Determinism/compute is unusually verifiable: most claims are *exact* (bit
equality) or have a closed-form/analytic reference. The plan maps each tier to
verification cases, a tolerance table, and the label it earns.

| # | Case | Type | Tolerance | Tier | Label earned |
|---|---|---|---|---|---|
| V1 | Reduced `(n,mean,M2)` identical for `worker_count ∈ {1,2,4,8}` | self-referential bit-diff | **byte-exact** | T2 | validated-toy |
| V2 | Serial fold == merge-tree result, same inputs | self-referential bit-diff | **byte-exact** | T1 | validated-toy |
| V3 | One-shot run == checkpoint+K-resume run | self-referential bit-diff | **byte-exact** | T2 | validated-toy |
| V4 | Welford merge recovers analytic mean/variance of a known distribution (e.g. Gaussian, uniform) | analytic / MMS-style | mean ≤ 1e-12 rel, var ≤ 1e-10 rel vs closed form on fixed sample set | T1 | validated-toy |
| V5 | aarch64 fixed-step canonical scenarios byte-diff vs the same-run x86_64 reference artifact | code-to-code (arch-to-arch) | **byte-exact** on fixed-step; state-stable tol on adaptive | T3 | validated-toy |
| V6 | `FpEnvironment` guard rejects a deliberately FTZ/DAZ/round-down-set environment; accepts clean | unit + property | exact reject/accept | T0 | checked |
| V7 | `for_mc_sample` pinned reference stream | unit (locked bytes) | **byte-exact** | T0 | checked |
| V8 | Dense-output interpolant vs analytic solution of a linear ODE between steps | analytic | interpolation error ≤ scheme order (7 for DOP853, 4 for DOPRI5) in a grid-halving log-ratio | T4 | checked (state-stable) |
| V9 | Pacer real-time-factor independence: identical sim output in FreeRun vs RealTime vs Paced | self-referential bit-diff | **byte-exact** output (only timing differs) | T3 | validated-toy |
| V10 | Jitter histogram on a documented host: p50/p99/p99.9/max vs an advisory budget | measured benchmark | reported, not gated; advisory p99.9 ≪ h_c | T3 | checked |
| V11 | Schedulability advisory vs Liu–Layland bound `U ≤ n(2^(1/n)−1)` | analytic | advisory correctness on hand-checked frame sets | T3 | checked |
| V12 | `check-provenance` CI job fails a planted un-provenanced TOML, passes the clean tree | CI gate self-test | exact fail/pass | T0 | checked |
| V13 | GPU-ingested deck: deterministic-CPU reproduction of a sampled subset within UQ band | code-to-code (consumer) | subset rel error within declared deck UQ; provenance + SHA pin present | T4 | checked |

**Code-to-code anchors (open tools).** V1/V2/V3/V9 are *self-referential* (the
serial path is the oracle). V4's analytic-variance and V8's interpolation-order
cases are MMS-flavored (doc 11's MMS/Richardson harness reuse). V5's aarch64 lane
is the cross-architecture analogue of the existing x86_64 determinism gate. For
the campaign *statistics* themselves (means, CIs, Sobol indices) the validation
oracle is **Dakota / SALib** as specified in doc 11 — this doc only proves the
*reduction is bit-stable*, not that the statistics are correct (that is doc 11).

**What the labels do not claim.** `validated-toy` here means "bit-reproducibility
and order-independence are proved on the reference profile(s)"; it is **not** a
statement that any physical result is flight-trustworthy, nor that the pacing
numbers hold on an uncharacterized host. The GPU boundary earns `checked` for
provenance + subset reproduction only — never a determinism claim about the GPU.

---

## 7. Dependencies on other parity docs

- **`11-monte-carlo-uq-and-validation.md`** — tightest coupling. Doc 11 owns the
  `openbmp-mc` campaign orchestrator (LHS/Sobol/Wilks/subset-sim, Clopper-Pearson,
  7009B credibility); **this doc owns the determinism-under-parallelism
  primitives** (`OrderIndependentReducer`, `Welford` merge, `CheckpointStore`,
  `ParallelCampaign`) that orchestrator is built on. The byte-diff gate's
  extension to ensemble statistics is a joint deliverable. Phase A in the DAG runs
  `11 T0–T2` and `12 T0–T2` together.
- **`10-flight-software-in-the-loop-xil.md`** — **shares `openbmp-rt`.** Doc 10's
  Tier-3 soft-real-time HIL-software bench *is* this doc's T3 pacer applied to the
  lockstep SIL loop; the `FramePacer` trait and jitter/overrun accounting are
  defined here and consumed there. The fixed-step-plant requirement (adaptive
  steppers break bit-true replay) is a shared constraint.
- **`00-overview.md`** — the invariants (esp. 1, 2, 3, 4, 5, 8) and the
  `openbmp-rt L7` placement this doc finalizes; the §8 risk-register entry
  "Determinism under parallel MC and real-time pacing" is *this* doc's mandate.
- **`13-agent-execution-playbook.md`** — the gate set (which `check-provenance`
  joins), the §5 backlog schema, the recommended-first-PRs note that pairs
  `WP-12.0`/`WP-11.0`.
- **Downstream consumers (loose).** `08-environment-gravity-and-frames.md`,
  `01-flexible-multibody-dynamics.md`, `05-propulsion-high-fidelity.md`,
  `06-gnc-coupled-mimo-and-control.md` — every heavy model added by these docs
  runs *inside* the locked kernel this doc protects; they depend on the
  determinism substrate but this doc does not depend on them. The dense-output T4
  work benefits event localization used across `01`/`05`/`08`.
- **`03`/`04` ingestion** — the T4 GPU offload boundary follows the same
  ingestion+provenance+UQ pattern that `03-aerodynamics-database-and-cfd-coupling.md`
  and `04-aerothermal-realgas-and-tps.md` use for external solver decks.

---

## 8. Open-source leverage

| Tool | License | Mode | Use |
|---|---|---|---|
| **rayon** | MIT / Apache-2.0 | port-of-dependency | Data-parallel fan-out of independent MC samples. The keyed-stream RNG already makes thread-count-independence achievable; rayon supplies the work-stealing pool. Reductions are made order-independent *by us* (Welford merge tree), never relying on rayon's reduce ordering. |
| **Random123 / Philox (concept)** | BSD | reference (algorithm) | The counter-based / keyed-stream philosophy `for_mc_sample` already follows. We do **not** vendor Random123 — ChaCha8 via `rand_chacha` is the workspace-canonical keyed RNG and stays so; we cite the philosophy. |
| **Linux PREEMPT_RT / SCHED_FIFO / `clock_nanosleep` / `cyclictest`** | GPL-2.0 (host platform) | couple (host) | Run the T3 fixed-step pacer under an RT kernel on isolated cores; use `cyclictest` to characterize the host's jitter floor and size the per-frame budget. Host platform only — no source linkage with the Rust workspace. |
| **NASA Trick (job-ordering / checkpoint-restart patterns)** | NASA-1.3 / permissive | ingest (design reference) | Mirror Trick's deterministic data-recording and checkpoint-restart *patterns* in Rust (informs the `CheckpointStore` design and the bit-invariant resume theorem). Learn-from, not a runtime dependency. |
| **Dakota / SALib** | LGPL / MIT | couple / oracle (via doc 11) | Cross-check that the *reduced statistics* match a reference implementation (doc 11 owns this). This doc uses them only transitively to confirm the bit-stable reducer's outputs are statistically equivalent to an independent tool. |
| **IEEE 754-2019 + Goldberg (1991)** | standard / article | reference | Authoritative basis for the rounding-mode/FMA-contraction discipline the FP guard and build flags enforce. |
| **`roaring` (Rust) — optional** | Apache-2.0 | port-of-dependency (optional) | Compact `completed_indices` bitmap for the checkpoint store at `1e6`-scale; a sorted `Vec<u32>` is the dependency-free fallback. Confirm transitive deps stay permissive (`cargo deny`). |

No GPU framework is adopted into the workspace. The T4 GPU boundary treats any
GPU toolchain (vendor-specific) as an **external offline producer** whose only
in-repo footprint is a provenance-pinned `data/` deck — exactly the
solver-consumer posture (`00-overview.md` §1.1) applied to compute.

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full gate set (`13`
§2). Tier letters follow §3.4.

---

**WP-12.0-a — Wire `check-provenance` into CI.**
- **goal:** Close the audit's named provenance gap: the existing
  `openbmp check-provenance` CLI command is not invoked by any CI job, so the
  on-disk sibling-`provenance.md` + SHA-pin check is unenforced. Add a CI job that
  runs it over `scenarios/` and `data/`. This is a dual-use hardening (research
  pack: "Put the check-provenance linter into CI … so the forward-only and
  academic-scope labels cannot be silently stripped").
- **implementation_status:** implemented in `.github/workflows/ci.yml` as the
  `provenance` job; traceable as `REQ-PROV-001` / `V-PROV-001`.
- **fidelity_tier:** T0
- **depends_on:** []
- **new_crates:** none
- **touched:** `.github/workflows/ci.yml` (new `provenance` job);
  `requirements.toml`.
- **approach:** Add a job mirroring the other lint jobs that builds `openbmp-cli`
  and runs `openbmp check-provenance scenarios/` and `openbmp check-provenance
  data/`, failing if any file lacks a sibling `provenance.md` or fails its SHA
  pin. No code change to the command itself (verified present at
  `crates/openbmp-cli/src/commands/provenance.rs`).
- **acceptance:**
  - New CI job present and green on the current clean tree.
  - A planted un-provenanced TOML (in a throwaway test branch / fixture) makes the
    job fail (V12).
  - All §2 gates green; goldens byte-identical (no scenario change).
- **validation_label:** checked
- **dual_use_note:** Hardens the forward-only/academic-scope provenance labels;
  far from line.
- **est_effort:** 0.5 day
- **parity_ceiling:** Does not validate the *content* of any deck, only that
  provenance metadata exists and pins.

---

**WP-12.0-b — Relocate FP guard to `openbmp-core` + add aarch64 FPCR path.**
- **goal:** Make the FP-environment guard portable and reusable. Move the
  x86_64 MXCSR check out of `openbmp-sim::kernel` into a portable
  `openbmp-core::FpEnvironment` (move, not rewrite) and add the **aarch64 FPCR**
  read (FZ/FZ16/RMode), closing the previously documented cross-arch
  determinism gap.
- **implementation_status:** implemented as `openbmp-core::FpEnvironment` with
  x86_64 MXCSR and aarch64 FPCR decoders; traceable as `REQ-DET-002` /
  `V-DET-002`.
- **fidelity_tier:** T0
- **depends_on:** []
- **new_crates:** none
- **touched:** `crates/openbmp-core/src/` (new `fp.rs` / `FpEnvironment`);
  `crates/openbmp-sim/src/kernel.rs` (call relocated guard at lines ~474, ~1459);
  `crates/openbmp-sim/src/error.rs` (re-export / map `FpEnvironmentDirty`);
  `.cargo/config.toml` (aarch64 contraction-ban stanza);
  `crates/openbmp-core/src/lib.rs`; `requirements.toml`.
- **approach:** §3.2.3 and §3.3. Keep the single `asm!` block behind
  `#[allow(unsafe_code)]` with the existing SAFETY comment (x86_64 `stmxcsr`,
  aarch64 `mrs {x}, fpcr`). Preserve the exact reject semantics (FTZ/DAZ/RC for
  x86_64; FZ/FZ16/RMode for aarch64). The kernel keeps its current
  construction-time call; the only change is the function's home.
- **acceptance:**
  - x86_64 behavior byte-identical to today; the relocated
    `fp_environment_is_clean_under_default_test_runner` test still passes.
  - New aarch64 unit/property test: guard rejects a deliberately FZ/round-down
    environment, accepts a clean one (V6).
  - `openbmp-core` still builds `no_std`-clean for the FC HAL target (the guard is
    `std`/arch-gated and must not regress the `hal-portability` job).
  - All §2 gates green; goldens byte-identical.
- **validation_label:** checked
- **dual_use_note:** Far from line; pure determinism hygiene.
- **est_effort:** 2–3 days
- **parity_ceiling:** Does not by itself prove aarch64 *output* byte-equality
  (that is V5/WP-12.3-b); proves the guard is correct on both arches.

---

**WP-12.0-c — `DeterministicRng::for_mc_sample` campaign domain.**
- **goal:** Add the per-Monte-Carlo-sample keyed stream (domain tag `b"MCSM"`) so
  a sample's randomness depends only on `(campaign_seed, sample_index,
  component)` — the substrate for thread-count-independent MC.
- **fidelity_tier:** T0
- **depends_on:** []
- **new_crates:** none
- **touched:** `crates/openbmp-core/src/rng.rs`; `requirements.toml`.
- **approach:** §3.2.1; mirror the existing constructor pattern and add a pinned
  `for_mc_sample_reference_stream_is_locked` test and a never-collides-with-other-
  families test (as the existing `for_stimulus_component` tests do).
- **acceptance:**
  - Pinned reference-stream test (V7) and cross-family non-collision test green.
  - Determinism property test (`same tuple → same stream`).
  - All §2 gates green; goldens byte-identical (no existing call site changes).
- **validation_label:** checked
- **dual_use_note:** Far from line.
- **est_effort:** 0.5 day
- **parity_ceiling:** Provides the keyed stream; does not yet build the campaign.

---

**WP-12.1 — Order-independent reducer + Welford merge tree (serial).**
- **goal:** Replace ad-hoc per-sample reduction with a reusable
  `OrderIndependentReducer` + `Welford` and a **fixed index-keyed merge tree**,
  proving serial-fold == merge-tree result bit-for-bit. The bit-stable
  foundation every parallel statistic stands on.
- **fidelity_tier:** T1
- **depends_on:** [WP-12.0-c]
- **new_crates:** proposes `openbmp-mc` (L7) — crate boundary owned jointly with
  doc 11; first commit is the skeleton + placement justification (no up-layer
  edge, no FC edge), reviewed before the impl lands.
- **touched:** `crates/openbmp-mc/` (new); `crates/openbmp-runner/src/footprint.rs`
  (adopt the reducer); `requirements.toml`.
- **approach:** §3.2.2. Implement `Welford` update + exact `merge`; the merge tree
  is a pure function of the sorted index set. Property-test merge
  commutativity/associativity *under the fixed partition*; analytic-variance case
  (V4) on a fixed sample set.
- **acceptance:**
  - V2 (serial fold == merge tree) byte-exact.
  - V4 (analytic mean/variance) within the tolerance table.
  - Footprint MC reduction now goes through the reducer with goldens unchanged.
  - All §2 gates green; new model off by default.
- **validation_label:** validated-toy
- **dual_use_note:** Statistics ABOUT forward runs; far from line.
- **est_effort:** 1 week
- **parity_ceiling:** Single-thread only; no parallelism, no resume yet.

---

**WP-12.2-a — Deterministic parallel fan-out (`rayon`).**
- **goal:** Parallelize `ParallelCampaign::run` over `worker_count` workers and
  prove the reduced output is **bit-identical for any worker count** — the hard
  requirement from `00-overview.md` §8.
- **implementation_status:** implemented for scalar campaign fan-out as
  `openbmp_mc::run_scalar_campaign_parallel`; checkpoint/resume remains
  WP-12.2-b.
- **requirements:** [REQ-MC-008]
- **fidelity_tier:** T2
- **depends_on:** [WP-12.1, WP-12.0-b]
- **new_crates:** none (extends `openbmp-mc`); adds `rayon` workspace dep.
- **touched:** `crates/openbmp-mc/`; workspace `Cargo.toml` (rayon);
  `deny.toml` if needed; `requirements.toml`.
- **approach:** §3.2.2/§3.3. Fan out `[0..n)`; **never** reduce in completion
  order — collect per-`sample_index` and reduce over the fixed merge tree. Call
  `FpEnvironment::assert_current_clean()` before worker-side sample evaluation
  (fail-closed).
- **acceptance:**
  - V1: reduced `(n,mean,M2)` byte-identical for `worker_count ∈ {1,2,4,8}`.
  - A `cargo deny` / `cargo machete` pass with rayon added (no policy/advisory hit;
    unused-dep clean).
  - Per-worker FP guard rejects a dirty thread (injected test).
  - All §2 gates green; parallel path off by default (serial remains default).
- **validation_label:** validated-toy
- **dual_use_note:** Far from line.
- **est_effort:** 1 week
- **parity_ceiling:** Reduction bit-stable; statistical *correctness* of the
  campaign is doc 11's V&V, not this WP's.

---

**WP-12.2-b — Checkpoint / resume for preemptible campaigns.**
- **goal:** Persist completed-index set + per-subtree reducer state so a preempted
  `1e5`–`1e6` campaign resumes without recomputation and **without changing a
  single output bit** (the bit-invariant resume theorem, §3.2.4).
- **implementation_status:** implemented for scalar campaign JSON checkpoints
  and chunked resume via `run_scalar_campaign_resumable`, CLI scalar resume via
  `openbmp mc resume-scalar`, and offline footprint checkpoint/resume via
  `openbmp footprint-mc --checkpoint-json`. Distributed/multi-node checkpoint
  ownership remains outside this work package.
- **requirements:** [REQ-MC-009]
- **fidelity_tier:** T2
- **depends_on:** [WP-12.2-a]
- **new_crates:** none (extends `openbmp-mc`); optional `roaring` dep (or
  `Vec<u32>` fallback).
- **touched:** `crates/openbmp-mc/`; `crates/openbmp-cli/` (`mc resume` /
  checkpoint flags); `requirements.toml`.
- **approach:** §3.2.4. Store Welford subtree state keyed by index range (NOT raw
  samples); rebuild the full merge tree at finalization. Fail closed on
  `scenario_sha256` / `toolchain_profile` mismatch.
- **acceptance:**
  - V3: one-shot run == checkpoint+K-resume run, byte-exact reduced output.
  - Resume with a tampered `scenario_sha256` returns a typed error (fail-closed).
  - All §2 gates green; checkpointing opt-in.
- **validation_label:** validated-toy
- **dual_use_note:** Audit aid (reproducible, bisectable); far from line.
- **est_effort:** 1 week
- **parity_ceiling:** Does not address distributed/multi-node coordination beyond
  a shared checkpoint store; single-store assumption documented.

---

**WP-12.3-a — `openbmp-rt` soft-/faster-than-real-time pacer.**
- **goal:** A shared frame pacer (with doc 10) giving FreeRun / Paced / RealTime
  modes, absolute-time scheduling, a jitter histogram, an overrun detector, and a
  reported real-time factor — turning the host SIL into a soft-real-time bench
  without hardware.
- **requirements:** [REQ-RT-002]
- **fidelity_tier:** T3
- **depends_on:** [WP-12.0-b]
- **new_crates:** `openbmp-rt` (L7) — host-side pacer crate with placement
  justification (depends only on `openbmp-core` + `std`; **forbidden** from the
  `openbmp-fc` graph).
- **touched:** `crates/openbmp-rt/`; `crates/openbmp-runner/src/rt.rs`;
  `crates/openbmp-scenario/src/document.rs`; `Cargo.toml`; `requirements.toml`;
  (doc 10 applies it to the broader SIL transport bench in its own WP).
- **approach:** §3.2.5/§3.3. Absolute-time `clock_nanosleep(TIMER_ABSTIME, t_0 +
  k·h_c)`; signed-jitter histogram (p50/p99/p99.9/max); overrun counter;
  `real_time_factor()`; Liu–Layland schedulability advisory. Pacer **off** the
  state path.
- **acceptance:**
  - V9: byte-identical sim output across FreeRun/RealTime/Paced (only timing
    differs) — proves pacing is off the determinism path.
  - V10: jitter histogram emitted on a documented host (reported, not gated).
  - V11: schedulability advisory matches hand-checked frame sets.
  - `fc_dependency_tripwire.rs` confirms no `openbmp-fc → openbmp-rt` edge.
  - All §2 gates green; pacing opt-in.
- **validation_label:** validated-toy (pacing-independence); checked (measured
  jitter)
- **dual_use_note:** Changes *when*, never *what*; carries no objective; far from
  line.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** Soft-real-time on a commodity host; **not** certified
  hard-real-time on flight hardware (that is the XIL HIL ceiling).

---

**WP-12.3-b — aarch64 determinism CI lane.**
- **implementation_status:** implemented in `.github/workflows/ci.yml` as
  `determinism-gate (aarch64)`: the x86_64 determinism job uploads
  `x86_64-determinism-reference-${{ github.sha }}`, and the native
  `ubuntu-24.04-arm` job verifies the aarch64 host profile, runs the fixed-step
  canonical scenario set, and byte-diffs those outputs against the same-run
  x86_64 reference artifact. Traceable as `REQ-DET-004` / `V-DET-004`.
- **goal:** Close the cross-architecture reproducibility loop: byte-diff the
  fixed-step canonical scenarios on an aarch64 runner against the same-run
  x86_64 reference bytes, declaring a **second bit-stable profile** for the fixed-step
  integrators.
- **fidelity_tier:** T3
- **depends_on:** [WP-12.0-b]
- **new_crates:** none
- **touched:** `.github/workflows/ci.yml` (aarch64 determinism lane);
  `docs/software-architecture.md` (declare the second profile);
  `requirements.toml`.
- **approach:** Run the existing fixed-step determinism scenarios on a native
  `ubuntu-24.04-arm` runner with the contraction-ban build flags from
  WP-12.0-b; assert byte-exact outputs against the x86_64 reference artifact
  produced by the same workflow run (V5).
- **acceptance:**
  - V5: aarch64 fixed-step Parquet byte-identical to the same-run x86_64
    reference artifact.
  - The second bit-stable profile is documented (target triple + toolchain + opt
    level + FP-guard pass).
  - All §2 gates green.
- **validation_label:** validated-toy
- **dual_use_note:** Far from line.
- **est_effort:** 1 week (mostly CI plumbing; depends on aarch64 runner
  availability — flagged as an open question).
- **parity_ceiling:** Bit-equality only on the two declared profiles; does not
  promise universal cross-platform byte equality.

---

**WP-12.4-a — Optional dense output for adaptive integrators (state-stable).**
- **goal:** Add a label-gated `DenseOutput` interpolant (DOP853 order-7 /
  DOPRI5 4th-order) for the **adaptive** integrators only, for event localization
  and output resampling — explicitly `state-stable`, off the bit-stable path. This
  is the audit's "optional dense output" gap, candidly recorded in the source.
- **fidelity_tier:** T4
- **depends_on:** [WP-12.0-b]
- **new_crates:** none
- **touched:** `crates/openbmp-sim/src/integrator.rs` (the 4 extra DOP853
  dense-output abscissas the tableau comment notes are missing; a `DenseOutput`
  trait); `requirements.toml`.
- **approach:** §3.4 T4(a). Implement the SciPy/Hairer dense-output interpolant
  using the already-evaluated stages plus the extra abscissas; gate behind an
  explicit `dense_output = true` flag; **never** invoked by the fixed-step gate.
- **implementation_status:** implemented traceable as `REQ-DET-003` /
  `V-DET-003`: `openbmp-sim::DenseOutput`,
  `Dopri54DenseOutput`, `Dopri853DenseOutput`,
  `Dopri54Adaptive::advance_with_dense_output`, and
  `Dopri853Adaptive::advance_with_dense_output` implement the DOPRI5 quartic and
  DOP853 order-7 continuous extensions from accepted adaptive substeps; runner
  dispatch accepts `solver.adaptive.dense_output = true` for
  `adaptive-explicit` + `dopri54` or `dopri853` and rejects fixed-step dense
  output.
- **acceptance:**
  - V8: interpolation error matches the DOPRI5 4th-order and DOP853 order-7
    dense-output behavior in grid-halving log-ratios on a linear-ODE analytic
    case.
  - Fixed-step bit-stable goldens **unchanged** (dense output is adaptive-only and
    off by default).
  - Dense output explicitly labeled `state-stable` in docs and tolerance table.
  - All §2 gates green.
- **validation_label:** checked (state-stable interpolation)
- **dual_use_note:** Far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** State-stable, not bit-stable; an output convenience, not a
  determinism guarantee.

---

**WP-12.4-b — GPU offload boundary for offline ingested-deck generation.**
- **goal:** Define a strictly-bounded GPU role: GPU may generate an *offline*
  ingested deck (large perturbation/aero/atmosphere table) whose product is
  provenance-pinned, UQ-tagged, and reproduced on the deterministic CPU path for a
  sampled subset — never entering the live f64 ODE loop.
- **fidelity_tier:** T4
- **depends_on:** [WP-12.2-a, WP-12.0-a]
- **new_crates:** none (no GPU framework enters the workspace)
- **touched:** `data/<deck>/` + `provenance.md` (ingested deck);
  `crates/openbmp-runner/` (consume the deck through the existing model trait
  surface); `requirements.toml`; a code-to-code subset-reproduction test.
- **approach:** §3.4 T4(b); solver-consumer posture (`00-overview.md` §1.1)
  applied to compute. GPU is an external producer; the in-repo footprint is the
  provenance-pinned deck + a deterministic-CPU reproduction of a sampled subset
  within the declared UQ band.
- **acceptance:**
  - V13: deterministic-CPU reproduction of a sampled subset within the deck's UQ
    band; `provenance.md` + SHA pin present (enforced by the WP-12.0-a job).
  - No GPU dependency added to the workspace; no GPU code on any live path.
  - All §2 gates green.
- **validation_label:** checked (provenance + subset reproduction)
- **dual_use_note:** Offline producer only; never a live actuator command; far
  from line. (Offline-only is also why GPU non-determinism is not a hazard.)
- **est_effort:** 1–2 weeks (boundary + ingestion + reproduction case; excludes
  the external GPU tooling itself, which is out of repo)
- **parity_ceiling:** No bit-reproducible f64 GPU parity is attempted or claimed;
  the GPU is an external, provenance-pinned data producer only.

---

## 10. References

**Determinism, parallel reduction, RNG**
- Salmon, Moraes, Dror, Shaw, *Parallel random numbers: as easy as 1,2,3*
  (Random123 / Philox), SC11, 2011 — counter-based keyed-stream philosophy.
- Chan, Golub, LeVeque, *Updating formulae and a pairwise algorithm for computing
  sample variances*, 1979 — the Welford parallel-variance merge.
- B. P. Welford, *Note on a method for calculating corrected sums of squares and
  products*, Technometrics 4(3), 1962.
- IEEE 754-2019 — rounding modes, FMA contraction.
- D. Goldberg, *What Every Computer Scientist Should Know About Floating-Point
  Arithmetic*, ACM Computing Surveys, 1991.
- rayon documentation — deterministic data-parallel iterators,
  https://docs.rs/rayon.

**Integrators / dense output**
- Dormand & Prince, *A family of embedded Runge-Kutta formulae*, J. Comp. Appl.
  Math. 6(1), 1980.
- Hairer, Nørsett, Wanner, *Solving Ordinary Differential Equations I*, 2nd rev.
  ed., Springer, 1993 — DOP853 / DOPRI5 tableaux, dense output (§II.5/§II.6),
  PI step control (§II.4).
- Gustafsson, *Control theoretic techniques for stepsize selection in explicit
  Runge-Kutta methods*, ACM TOMS, 1991.
- SciPy `_ivp/dop853_coefficients.py` — the pinned-against reference for the
  DOP853 tableau and order-7 dense-output abscissas.

**Real-time / scheduling**
- Liu & Layland, *Scheduling Algorithms for Multiprogramming in a Hard-Real-Time
  Environment*, JACM 20(1), 1973 — rate-monotonic bound.
- Linux PREEMPT_RT wiki; SCHED_DEADLINE (CBS/EDF) kernel docs; `cyclictest`.
- OPAL-RT / Concurrent-RT deterministic-HIL criteria (p99.9 jitter ≪ minor frame).

**Verification & validation (shared with doc 11)**
- Roache, *Verification and Validation in Computational Science and Engineering*,
  Hermosa, 1998; *Code Verification by the Method of Manufactured Solutions*,
  J. Fluids Eng. 124(1), 2002.
- Oberkampf & Roy, *Verification and Validation in Scientific Computing*,
  Cambridge, 2010.
- NASA Trick Simulation Environment — checkpoint/restart and deterministic
  data-recording patterns, https://github.com/nasa/trick.

**Upstream OpenBMP anchors**
- `docs/software-architecture.md` § Determinism Profile (locked weighted-sum
  order, no FMA, MXCSR guard).
- `docs/verification.md` (five-layer V&V ladder, validation labels).
- `crates/openbmp-core/src/rng.rs`, `crates/openbmp-sim/src/integrator.rs`,
  `crates/openbmp-sim/src/kernel.rs`, `.cargo/config.toml`,
  `.github/workflows/ci.yml`,
  `crates/openbmp-cli/src/commands/provenance.rs` (verified current state, §2).

---

*Companion documents:* `00-overview.md` (constitution) ·
`11-monte-carlo-uq-and-validation.md` (campaign orchestrator, statistics V&V) ·
`10-flight-software-in-the-loop-xil.md` (shares `openbmp-rt`) ·
`13-agent-execution-playbook.md` (gates, backlog schema).
