# Coupled MIMO GNC, Allocation, Load-Relief & FDIR

**Status:** `experimental` (design intent; this document ships no code).
**Audience:** the engineer or LLM agent implementing the parity work packages for the GNC dimension, working entirely behind the `openbmp-fc` portability lock.
**One-line summary:** upgrade OpenBMP's per-axis (diagonal) controllers and per-axis allocators to a coupled-plant MIMO GNC stack — dense effectiveness-matrix allocation (Härkegård active-set WLS), `n×n` LQR/LQG with non-diagonal inertia and gain scheduling, drift-/load-minimum ascent load-relief with a `q·α` limiter, structural-filter co-design, tightly-coupled GNSS/INS error-state EKF, MEKF attitude with covariance reset, TVC/tail-wags-dog modeling, and redundancy/FDIR reconfiguration with abort-to-SAFE-state — all forward-only and offline-synthesized where the synthesis is heavyweight.

> Read `00-overview.md` (parity definition, invariants, crate map, DAG) and
> `13-agent-execution-playbook.md` (template, gate set, backlog schema) before this
> document. This doc is Phase-C work in the program DAG and depends on the coupled
> plant produced by Phase-A/B (`01`, `02`, `05`, `09`).

---

## 1. Parity target & ceiling

### 1.1 Target capability (within posture)

Bring OpenBMP's flight-control and navigation stack to **method/architecture parity**
with the launch-vehicle GNC practice documented at NASA/MSFC (SLS Ascent Flight
Control), ESA/Ariane reusable-launcher GNC, and the open aircraft-allocation literature.
Concretely, at the end of this dimension OpenBMP can:

- allocate a **virtual moment/force command** `v ∈ ℝ^m` (`m ∈ {3,…,6}`) onto `N > m`
  physical effectors through a **dense control-effectiveness matrix `B`** with box and
  rate bounds, prioritization, and a minimum-effort secondary objective — capturing
  genuinely coupled effector clusters (multi-engine TVC sharing pitch/yaw/roll, an RCS
  quad with redundant roll) that the current per-axis allocator can only treat as a
  selector;
- fly a vehicle with **products of inertia and gyroscopic cross-coupling** using an
  `n×n` coupled-plant LQR/LQG computed offline from a DARE/CARE solver, **gain-scheduled**
  along the ascent (Mach / `q̄` / mass / time) with bumpless transfer;
- run a **blended drift-minimum / load-minimum** ascent control law through max-Q with an
  **angle-of-attack estimator** (relative-velocity + modeled wind), a **`q·α` (or `q·β`)
  limiter**, and a **gyro blender** across multiple rate-gyro stations, with an array of
  **pole/zero-parameterized structural bending/slosh filters** co-designed with the law;
- estimate state with a **tightly-coupled GNSS/INS error-state EKF** on raw
  pseudorange/Doppler observables (works with `<4` satellites under powered flight,
  per-satellite chi-square gating), and attitude with a **MEKF** including the correct
  multiplicative covariance reset;
- model **TVC actuator dynamics + the tail-wags-dog (TWD) reaction** and compensate with
  **INDI/INCA** plus pseudo-control hedging; and
- **reconfigure** on effector/sensor faults — drop a `B` column on engine-out and re-solve
  the WLS, vote/FDI on redundant lanes, and **abort to a SAFE flight state** (thrust
  termination, attitude-hold, propellant-safe) through a guarded transition table.

The matching reference architecture is the SLS ascent control stack (Orr et al., AAS
14-038): gyro blender → bending-filter array → PD/PID rigid-body law with scheduled gains
→ load-relief term + `q·α` limiter → control allocation → actuator. This document's tiers
reconstruct that architecture on **open benchmark vehicles** (ADMIRE allocation matrix,
generic free-free-beam + pendulum-slosh plants, synthetic multi-engine TVC clusters).

### 1.2 Parity ceiling (honest boundary)

OpenBMP can reach method parity and code-to-code / MMS verification; it **cannot** reach
validation parity. The specific things that require proprietary data or flight heritage,
and their open substitutes:

| Cannot match (proprietary / ITAR / flight-heritage) | Open substitute |
|---|---|
| Flight-validated control-effectiveness matrices, TVC/gimbal Jacobians, slosh masses, bending mode shapes for real vehicles (Falcon, SLS, Ariane) | Published academic benchmark vehicles (ADMIRE `B` matrix), synthetic TVC clusters, generic free-free-beam + pendulum-slosh models, MMS |
| Wind-tunnel / proprietary-CFD aero databases anchoring the load-relief `q·α` gains | Code-to-code vs open CFD (`03`/`04`) + standard gust/atmosphere models (Dryden/von-Kármán MIL-HDBK-1797, GRAM-class via `08`) |
| Certified flight-SW V&V, man-rating, the actual adaptive-augmenting-control (AAC) tuning flown on SLS | The NESC published stability-analysis **method** (AVSAT margin sweeps, Monte-Carlo dispersion) reproduced on open models — methodology, not certified result |
| Hardware actuator transfer functions (real EHA/EMA dynamics, backlash, load-torque feedback) | Published 2nd-order + rate-limit actuator models + the TWD analytic reaction term |
| Real GNSS receiver tracking-loop behavior under launch dynamics / jamming | RTKLIB-class observable models + synthetic high-dynamics observables (`09`) |

**Uncertain-convergence item (flagged, not promised):** the slosh-coupled upper-stage
orbit closure (`00` §8 risk register) is treated here as a *control
co-design* problem with explicitly uncertain convergence — Tier-3/4 ships the load-relief,
structural-filter, and reconfiguration machinery regardless of whether this specific
closure lands, and the doc never claims it does.

This document does **not** imply flight readiness or certification; `docs/standards-posture.md`
and `docs/safety-boundaries.md` permanently disclaim it.

---

## 2. Current state in source

All paths verified against the working tree. `openbmp-fc` (L4) depends only on
hardware-portable crates (`openbmp-core`, `openbmp-msgs`, `openbmp-physics`,
`openbmp-mission`, `openbmp-sensors`, `nalgebra`, `num-traits`, `thiserror`, `indexmap`,
optional `clarabel`) — verified in `crates/openbmp-fc/Cargo.toml`; the FC dependency
tripwire (`crates/openbmp-testkit/tests/fc_dependency_tripwire.rs`) is green. `clarabel` is
already an optional dependency behind the `mpc` feature.

### 2.1 Control allocation — `crates/openbmp-fc/src/allocation.rs`

Two legacy allocators sit behind the `ControlAllocator` enum:

- `PrioritisedRedistributedAllocator` — body-axis priority permutation +
  per-effector axis assignment; proportional split `uᵢ = τ_a · Lᵢ / ΣL` with
  clamp-at-capacity saturation.
- `WeightedPseudoInverseAllocator` — per-axis `uᵢ = τ_axis · Lᵢ² / Σ Lⱼ²` (the
  weighted-pseudo-inverse special case with `W = diag(1/Lᵢ²)`), with a deterministic
  active-set redistribution pass (`allocate_axis_weighted_pseudo_inverse`).
- `EffectivenessMatrix`, `EffectorBounds`, `WlsActiveSetAllocator`, and
  `WlsAllocationOutput` now expose the first dense-`B` bounded-WLS substrate;
  `DenseWlsControlAllocator` makes it mixer-facing for fixed `[roll, pitch, yaw]`
  virtual rows. The path supports arbitrary coupled effector columns,
  asymmetric position bounds, per-tick rate tightening from `u_prev` and `dt_s`,
  deterministic active-set iteration, active-at-bound flags, virtual residuals,
  attainability, phase-gated fixed-zero columns, and fail-closed
  finite/dimension/bounds validation (`REQ-GNC-001` / `V-GNC-001`).

**Critical limitation (the Tier-0 baseline to lift):** the module docstring states it
explicitly that the legacy path is still the single-axis `direct_torque` surface, and
the diagonal special case is still preserved for existing scenarios. Schema-v3
`[fc.autopilot_allocation] kind = "dense_wls"` can now declare
`dense_effectors` in fixed `[roll, pitch, yaw]` row order, the runner constructs
the stateful dense allocator from declared effector limits, and `mixer.rs` routes
its `EffectorCommandSet` output. The dense-`B` path is cross-checked against
QCAT `wls_alloc`-derived ADMIRE and synthetic TVC vectors, carries the
no-target B-matrix schema tripwire, and is treated as implemented for WP-06.1.
The remaining GNC gap is above the allocator: the controllers, estimator, and
reconfiguration layers still assume diagonal/per-axis behavior until WP-06.2+.

### 2.2 Controllers

- `crates/openbmp-fc/src/lqr.rs` — `solve_lqr_rate_loop` solves a **scalar 2-state**
  per-axis DARE (state `z = [ω−ω_ref, ∫(ω−ω_ref)]`, `A=[[1,0],[dt,1]]`, `B=[dt/J,0]`) by a
  **structure-preserving doubling algorithm** (`solve_dare_2x2_doubling`, Anderson 1978 / Chu
  et al. 2004), fully unrolled scalar arithmetic. The module now also exposes
  the first WP-06.2-a dense coupled-plant oracle:
  `solve_dare_nxn_doubling`, `solve_dare_nxn_doubling_with_options`,
  `dense_dare_residual_norm`, and `CoupledLqrSolution`. That path validates
  bounded row-major dimensions, finite matrix entries, SPD `Q`/`R`,
  deterministic Cholesky/LU solves, residual/iteration bounds, bit-stability,
  a scalar plus coupled three-state/two-input tolerance table, and fail-closed
  behavior for an uncontrolled unstable mode (`REQ-GNC-002` / `V-GNC-002`).
  The module also exposes the WP-06.2-b LQG dual-DARE observer helper
  `solve_lqg_observer_nxn_dual_dare` and `CoupledLqgObserverSolution`, checked
  against SciPy `solve_discrete_are` toy references (`REQ-GNC-003` /
  `V-GNC-003`). The active runtime `RateLoopKind::Lqr` still consumes scalar
  per-axis gains unless the off-default scheduled-gain / bending-filter paths
  are explicitly installed.
- `crates/openbmp-fc/src/indi.rs` — INDI rate loop (Smeur 2016) with **diagonal `G_eff`**
  and the per-axis SISO assumption stated in its anti-scope; composition with L1 is rejected
  at load; multi-body assemblies are rejected at load.
- `crates/openbmp-fc/src/l1_adaptive_full.rs` — Cao-Hovakimyan 2010 four-piece scalar L1
  per axis (reference model, state predictor, piecewise-constant adaptation, first-order LPF)
  with the `ω_c·L < 1` projection check.
- `crates/openbmp-fc/src/mpc.rs` — Clarabel-backed (feature `mpc`) single-step box QP
  `solve_attitude_box_qp` (`min ½‖u−u_ref‖² s.t. −limit ≤ uᵢ ≤ limit`) and a
  `RecedingHorizonAttitudeMpc` over a **small-angle** attitude-error state with deterministic
  Clarabel settings.
- `crates/openbmp-fc/src/autopilot.rs` — Stevens & Lewis three-loop PID (rate / attitude /
  trajectory) with back-calculation anti-windup (`crates/openbmp-fc/src/anti_windup.rs`),
  phase-scheduled gains via the parameter registry, and a TVC path that writes a **single**
  `gimbal_pitch_rad`/`gimbal_yaw_rad` pair into `EngineDemand`.

### 2.3 Estimation

- `crates/openbmp-fc/src/estimator.rs` — 15-state error-state EKF (`x =
  [δp, δv, attitude_error(3), b_g, b_a]`) with multiplicative attitude error and
  per-measurement innovation gates; consumes `sensor.imu/gnss/magnetometer/barometer/star_tracker`.
  Also a 6-state `Mekf` (multiplicative quaternion, attitude-only). **GNSS is fused at the
  position/velocity level**, not raw observables; the 6-D block innovation gate is the known
  failure mode under thrust.
- `crates/openbmp-fc/src/sr_ukf.rs` — square-root UKF (15-state + 6-state attitude), Householder
  QR predict + `cholupdate` rank-1 measurement updates, no hot-path refactorization.
- `crates/openbmp-fc/src/ud.rs` — UD-factorized Kalman update.
- `crates/openbmp-fc/src/imm.rs` — Bar-Shalom IMM (`MAX_IMM_MODES = 4`) with log-sum-exp
  mode-probability update.

### 2.4 FDIR, voting, scheduling, commander, tables

- `crates/openbmp-fc/src/glrt.rs` — Willsky 1976 windowed-mean-shift GLRT (Bonferroni
  family-wise threshold), CUSUM (`StepOutcome`), feeding `crates/openbmp-fc/src/fdir.rs`
  (15+ fault-tree bits incl. `FDIR_BIT_AUTOPILOT_SATURATION`, `FDIR_BIT_BODY_RATE_REDLINE`,
  `FDIR_BIT_ACTUATOR_REJECT`).
- `crates/openbmp-fc/src/voter.rs` — `MidValueSelectScalar`, `WeightedMeanScalar`,
  `PassThroughVoter` over the `Voter<T>` trait; covariance-weighted variant referenced.
- `crates/openbmp-fc/src/health.rs` — arming gate; `crates/openbmp-fc/src/commander.rs` —
  single state-machine owner built on `openbmp-mission`'s `MissionPhaseGraph` /
  `MissionStateMachine` (`crates/openbmp-mission/src/hsm.rs`), event-bound transitions.
- `crates/openbmp-fc/src/scheduler.rs` — cyclic scheduler, `(priority, registration_index)`
  dispatch order, declared budgets, no post-registration allocation (lockstep-clock + no-
  hot-path-allocation contract, enforced by `crates/openbmp-testkit/src/fc_lints.rs`).
- `crates/openbmp-fc/src/tables.rs` — cFE-TBL-style validated-then-activated typed table
  registry (two-buffer pending/active swap) — **the natural home for the offline-synthesized
  gain schedules and `B`-matrix I-loads.**
- `crates/openbmp-fc/src/mixer.rs` — phase-gated authority + actuator-channel map; routes
  per-engine `gimbal_pitch_rad`/`gimbal_yaw_rad` (`EngineDemand`, `EffectorCommandSet` in
  `crates/openbmp-msgs/src/lib.rs`, `MAX_EFFECTOR_COMMANDS = 128`) and consumes
  `actuator.effector_availability` to suppress unavailable effector columns before dense
  allocation.

**Net:** every controller and allocator currently assumes diagonal inertia / per-axis
decoupling; the coupled-MIMO, dense-`B`, load-relief, and TVC-gimbal availability mask
substrates now exist, while tight-nav and full FDIR reconfiguration layers do not. The
infrastructure (tables registry, scheduler, voter/FDIR primitives,
`EngineDemand` gimbal fields, Clarabel hook, SR-UKF/UD Cholesky machinery) is in place to
host them without breaking the portability lock.

---

## 3. Target architecture

### 3.1 Crate placement

**No new crate is required, and none may be added that would break the FC lock.** All work
lands inside `openbmp-fc` (L4) using its existing hardware-portable dependencies. The data
that the new code needs from the simulator — vehicle inertia, gimbal geometry → `B` rows,
wind profile, atmospheric density, multi-station gyro mounting, per-engine state — arrives
**only through the FC-bridge / sensor-actuator boundary** (`openbmp-runner/src/fc_bridge.rs`
populating bus topics in `openbmp-msgs`), never by importing `openbmp-sim`,
`openbmp-scenario`, `openbmp-runner`, `openbmp-telemetry`, `openbmp-bridge`, or
`openbmp-aerothermal`. `BodyAxis` in `allocation.rs` already mirrors
`openbmp_scenario::TorqueAxis` precisely to avoid the scenario dependency — the same pattern
applies to every new I-load type below.

Offline synthesis (DARE/CARE for `n×n` LQR/LQG, H∞/μ, structural-filter co-design) is **not**
in the flight crate at runtime: the heavyweight synthesizers run in Python/MATLAB offline,
and only the resulting linear gains / filter coefficients / `B` rows are imported as
provenance-pinned scenario I-loads through the `tables.rs` registry. The **`n×n` DARE
doubling solver is the one synthesis path that may also live in-repo** (Rust, deterministic)
because it is small and is itself a verification oracle.

```text
openbmp-fc  L4  (portability lock — touched, no new forbidden edge)
  allocation.rs   + dense-B Härkegård active-set WLS (box+rate bounds, warm-start)
  lqr.rs          + n×n coupled-plant DARE/CARE (doubling, Cholesky), LQG observer
  schedule.rs     + (new module) gain-schedule interpolation + bumpless transfer
  loadrelief.rs   + (new module) drift/load-min blend, alpha estimator, q·alpha limiter
  filters.rs      + scheduled biquad bending/slosh sections (pole/zero parameterization)
  indi.rs         + INCA coupling to WLS + pseudo-control hedging + TWD-aware B_eff
  estimator.rs    + tightly-coupled GNSS/INS error-state EKF (raw pseudorange/Doppler)
  mekf.rs         + (promote) MEKF with Markley 2023 covariance reset
  reconfig.rs     + (new module) engine-out B-column drop + re-solve + degraded-mode flag
  commander.rs    + abort-to-SAFE guarded transition table (NOMINAL→DEGRADED→SAFE)
  tables.rs       + new Table impls: EffectivenessMatrixTable, GainScheduleTable,
                    BendingFilterTable, RobustControllerTable  (cFE-TBL semantics)
```

### 3.2 Full effectiveness-matrix allocation (Tier 1)

**Formulation — bounded WLS (Härkegård `wls_alloc`).** Map virtual command `v ∈ ℝ^m`
onto `u ∈ ℝ^N` through the dense effectiveness matrix `B ∈ ℝ^{m×N}`:

```
minimize over u   ‖W_u (u − u_d)‖²   (secondary: minimum effort about preferred u_d)
subject to        u = argmin ‖W_v (B u − v)‖²   (primary: attainability)
                  u_min ≤ u ≤ u_max              (box bounds)
```

solved via the single-objective γ-weighted surrogate (γ ≫ 1, typically `1e6`):

```
u* = argmin_{u_min ≤ u ≤ u_max}  ‖ A u − b ‖²,
A  = [ √γ · W_v B ; W_u ],   b = [ √γ · W_v v ; W_u u_d ].
```

The **interior (no active bound) closed form** is the weighted pseudo-inverse —
the generalization of the current `Lᵢ²` split:

```
u = u_d + B_W^#(v − B u_d),   B_W^# = W^{-1} Bᵀ (B W^{-1} Bᵀ)^{-1},   W = W_uᵀ W_u.
```

**Active-set recursion (finite-step, KKT-optimal).** Warm-start from the previous tick's
`u` (essential for real-time); partition variables into free set `F` and active (bounded)
set `A`; solve the equality-constrained LS on `F` via QR / normal equations of `A_F`; if a
free variable would violate a bound, add it to `A` at the bound and re-solve; if the
Lagrange-multiplier sign on an active variable indicates the cost can decrease by freeing
it, free it; iterate to KKT optimality. Terminates in `≤ N` steps. **Rate limits** enter by
tightening the per-tick box:
`u_min,k = max(u_pos_min, u_prev,k − rate_k·dt)`, `u_max,k = min(u_pos_max, u_prev,k + rate_k·dt)`.

**Rust trait surface** (lands in `allocation.rs`, extends the existing `ControlAllocator`
enum):

```rust
/// Dense control-effectiveness map. `B` has `m` virtual-command rows
/// (roll/pitch/yaw torque, optionally axial/lateral force) and `N`
/// effector columns. Built once from gimbal/thruster geometry supplied
/// through the FC-bridge; the FC never imports the scenario type.
#[derive(Clone, Debug)]
pub struct EffectivenessMatrix {
    /// Row-major `m×N`. Row order is the fixed `BodyAxis`/`BodyForce`
    /// ordering; column order is the effector registration order.
    b: nalgebra::DMatrix<f64>,
    effectors: Vec<EffectorId>,
}

/// Box + rate bounds for one allocator step, evaluated per tick.
#[derive(Clone, Debug)]
pub struct EffectorBounds {
    pub u_min_pos: Vec<f64>,   // hard position floor
    pub u_max_pos: Vec<f64>,   // hard position ceiling
    pub rate_limit: Vec<f64>,  // |Δu|/dt ceiling, per effector
}

/// Härkegård active-set bounded-WLS allocator over a dense B.
#[derive(Clone, Debug)]
pub struct WlsActiveSetAllocator {
    matrix: EffectivenessMatrix,
    w_v: nalgebra::DVector<f64>, // virtual-error weights (diag)
    w_u: nalgebra::DVector<f64>, // effort weights (diag)
    gamma: f64,                  // attainability priority weight
    u_pref: nalgebra::DVector<f64>,
}

impl WlsActiveSetAllocator {
    /// Solve one allocation step with warm start from `u_prev`,
    /// returning the per-effector commands, the active set, and a
    /// per-axis attainability/saturation report. Deterministic:
    /// fixed iteration order, index tie-break, no BLAS threading.
    pub fn allocate(
        &self,
        v: &[f64],                 // virtual command, length m
        u_prev: &[f64],            // warm start, length N
        bounds: &EffectorBounds,
        dt_s: f64,
    ) -> WlsAllocationOutput;
}

pub struct WlsAllocationOutput {
    pub u: Vec<(EffectorId, f64)>,
    pub active_at_bound: Vec<bool>,
    pub virtual_residual: Vec<f64>, // v − B u (per virtual axis)
    pub attainable: bool,           // residual within tolerance
    pub iterations: u16,
}
```

The existing per-axis allocators stay as the **diagonal special case** (`B` block-diagonal,
one effector per axis) so all current scenarios remain byte-identical until one opts into a
dense `B` block. Determinism notes: no `f64::mul_add`; QR via a fixed Householder order;
ties broken by the lowest effector index; no nalgebra multithreaded BLAS path.

**Solver-consumer note.** When soft constraints, multiple secondary objectives, or a SOCP
form (minimum-time-to-saturation) are needed beyond what active-set WLS handles, the QP
backend is **Clarabel** (already a pinned optional dep, pure-Rust, no FFI — preferred for
deterministic builds) or OSQP via `osqp.rs` (Apache-2.0, C core vendored/pinned). The
active-set WLS is the default in-repo deterministic tier; the QP solver is the escalation.

### 3.3 Coupled `n×n` LQR/LQG (Tier 2)

**Plant.** Rigid-body attitude with non-diagonal inertia and gyroscopic coupling:

```
J ω̇ = M − ω × (J ω),   J = full symmetric inertia tensor (products of inertia ≠ 0).
```

Linearize about a trajectory node `i`: `ẋ = A_i x + B_i u`, with
`x = [δθ(3), δω(3), ∫δθ(3), (optional flex q, q̇)]`. The cross-products in `J` and the
`−ω×(Jω)` gyroscopic term make `A_i` non-block-diagonal — pitch/yaw/roll no longer decouple,
so the scalar 2-state DARE is structurally insufficient.

**Discrete LQR.** `K = (R + B_dᵀ P B_d)^{-1} B_dᵀ P A_d` with `P` the stabilizing solution of
the DARE

```
P = A_dᵀ P A_d − A_dᵀ P B_d (R + B_dᵀ P B_d)^{-1} B_dᵀ P A_d + Q.
```

**Solver — `n×n` structure-preserving doubling (generalize the existing scalar SDA).** Propagate
the triple `(A_k, G_k, H_k)` with `G_0 = B R^{-1} Bᵀ`, `H_0 = Q`:

```
M_k    = (I + G_k H_k)^{-1}
A_{k+1} = A_k M_k A_k
G_{k+1} = G_k + A_k M_k G_k A_kᵀ
H_{k+1} = H_k + A_kᵀ H_k M_k A_k        →   H_∞ = P.
```

The matrix inversion `(I + G_k H_k)^{-1}` uses an LU/Cholesky from `nalgebra` (the scalar
version inverts a 2×2 by hand; the `n×n` version uses `nalgebra::LU`, operand order pinned).
The doubling route is preferred over the Laub Schur method for embedded determinism (no
ordered-eigenvalue tie-break), but a Schur fallback (`faer` real Schur) is available as a
cross-check oracle.

**LQG / LTR.** Add a Kalman observer `x̂̇ = A x̂ + B u + L(y − C x̂)` with `L` from the dual
DARE (filter ARE), controller `u = −K x̂`. Loop-Transfer-Recovery tunes the filter (Doyle-Stein)
to recover the LQR robustness margins lost by the estimator. **Synthesis is offline**
(python-control `dlqr`/`dlqe`, or the in-repo `n×n` DARE solver as oracle); only `K·x̂` and
the observer update run in the loop.

```rust
/// Offline-synthesized coupled-plant gain set, imported as a cFE-style
/// table I-load. Carries no target/range field — only flight-condition
/// node tag and the linear gains. Scenario-consumer-agreement and
/// inline-data tripwires extend to this type.
#[derive(Clone, Debug)]
pub struct CoupledGainNode {
    pub schedule_value: f64,            // Mach / qbar / mass-fraction / time
    pub k_feedback: nalgebra::DMatrix<f64>, // m×n  (−K)
    pub l_observer: Option<nalgebra::DMatrix<f64>>, // n×p (LQG)
    pub a_d: nalgebra::DMatrix<f64>,    // discrete plant for the observer
    pub b_d: nalgebra::DMatrix<f64>,
    pub c: nalgebra::DMatrix<f64>,
}

/// In-repo n×n DARE solver (doubling) — verification oracle and the
/// allowed in-flight synthesis path for modest n.
pub fn solve_dare_nxn_doubling(
    a: &DMatrix<f64>, b: &DMatrix<f64>,
    q: &DMatrix<f64>, r: &DMatrix<f64>,
    max_iter: usize, tol: f64,
) -> Result<DMatrix<f64>, LqrError>;
```

### 3.4 Gain scheduling (Tier 2)

Pre-computed gain sets `K_i` at scheduling nodes `s_i ∈ {nondimensional time, Mach, q̄,
mass-fraction}`. In-loop: pick interval, blend `K(s) = (1−λ)K_i + λ K_{i+1}`,
`λ = (s−s_i)/(s_{i+1}−s_i)` (linear or smoothstep). **Filter coefficients are scheduled in
pole/zero frequency-and-damping parameterization, never raw IIR taps** (interpolating taps
can transiently destabilize a section). **Bumpless transfer / anti-windup** resets integral
states across schedule boundaries. For LPV rigor, a polytopic/affine-parameter controller
(Apkarian-Gahinet, single LMI feasibility over parameter vertices) holds stability for
arbitrarily fast `ṡ` — that LMI synthesis is **offline**; the in-loop part is interpolation +
bumpless transfer. The `tables.rs` two-buffer registry hosts the `GainScheduleTable`;
discipline: **every schedule interior point is V&V'd, not only the nodes.**

```rust
pub struct GainScheduleTable {
    pub variable: ScheduleVariable,    // Mach | DynamicPressure | MassFraction | Time
    pub nodes: Vec<CoupledGainNode>,   // sorted ascending by schedule_value
    pub interp: GainInterp,            // Linear | Smoothstep
}
impl Table for GainScheduleTable { /* validate monotone nodes, finite gains */ }

pub struct ScheduledController {
    table: GainScheduleTable,
    integ_state: nalgebra::DVector<f64>, // reset on bumpless transitions
}
impl ScheduledController {
    pub fn gains_at(&self, s: f64) -> nalgebra::DMatrix<f64>; // blended K(s)
    pub fn on_schedule_boundary(&mut self); // bumpless reset
}
```

### 3.5 Load-relief: drift-minimum / load-minimum blend + `q·α` limiter (Tier 3)

**Classic blended law (Greensite / SLS form, Orr AAS 14-038).** The gimbal command is

```
δ = a0·θ_err + a1·θ̇_err + g_α(t)·α̂ + g_acc·N_z
```

where `θ_err` is attitude error, `α̂` is estimated angle of attack, `N_z` is the
body-lateral accelerometer. The blend coefficient `k(t) ∈ [0,1]` schedules between
**drift-minimum** (`k=0`, no α feedback — holds attitude, spikes `q·α`) and **load-minimum**
(`k=1`, full α feedback — weathervanes into the wind, reduces `q·α`):
`g_α(t) = k(t)·g_α,max`, ramped up entering max-Q and back down after.

**Angle-of-attack estimator (no air-data probe).** From INS velocity, modeled wind, and
attitude:

```
V_rel = V_inertial − V_wind          (ECI/ECEF, supplied via FC-bridge wind topic)
V_rel^b = R(q)·V_rel                  (rotate to body using the MEKF attitude)
α̂ = atan2(w_rel^b, u_rel^b),  β̂ = atan2(v_rel^b, u_rel^b).
```

`q̄ = ½ ρ ‖V_rel‖²` reuses the existing `nav_metrics::dynamic_pressure_air_relative_with_density`
(ρ arrives via `EnvironmentEstimate`). The **`q·α` (or `q·β`) limiter** clamps the product
to a structural-load ceiling — implemented as a path-constraint on the commanded attitude in
the guidance loop (Orr-Russell 2022 modern variant) and/or as a hard feedback saturation,
declared as a structural-load limit, **never parameterized by a ground aimpoint.**

**Gyro blender.** Optimally weights `n` rate-gyro stations `w_i` (`Σ w_i = 1`) chosen to
minimize observed bending contamination at the station slopes — a fixed offline-computed
weight vector applied in the loop. This pairs with the bending-filter array (§3.6).

```rust
pub struct LoadReliefParams {
    pub a0: f64, pub a1: f64,        // rigid-body attitude PD
    pub g_alpha_max: f64,            // load-min alpha gain
    pub g_acc: f64,                  // lateral-accelerometer gain
    pub blend_schedule: Vec<(f64, f64)>, // (schedule_value, k) ramp, k∈[0,1]
    pub q_alpha_limit_pa_rad: f64,   // structural-load ceiling (Pa·rad)
}
pub struct AlphaEstimate { pub alpha_rad: f64, pub beta_rad: f64, pub qbar_pa: f64 }

/// Estimate alpha/beta from relative velocity + modeled wind + attitude.
/// Wind and density arrive via FC-bridge topics; no scenario import.
pub fn estimate_alpha_beta(
    v_inertial: Vector3<f64>, v_wind: Vector3<f64>,
    q_body_from_inertial: UnitQuaternion<f64>, density_kg_m3: f64,
) -> AlphaEstimate;

pub struct GyroBlender { weights: Vec<f64> } // Σ w_i = 1, offline-computed
```

### 3.6 Structural-filter co-design (Tier 2/3)

**Flex plant (sim side, doc `02`).** Modal coordinates
`η̈_j + 2ζ_jω_j η̇_j + ω_j² η_j = φ_j(x_sensor)·(generalized force)`; the sensor sees
`θ_meas = θ_rigid + Σ_j slope_j(x_gyro)·η_j`. **Filter (FC side).** A cascade of 2nd-order
IIR sections

```
H(s) = Π_n  (s² + 2ζ_n ω_n s + ω_n²) / (s² + 2ζ_d ω_d s + ω_d²)
```

(notch when `ω_n ≈ ω_d`, `ζ_n < ζ_d`), discretized by **Tustin with prewarping** at the mode
frequency `ω_j`: `s → (2/T)(z−1)/(z+1)`, prewarp constant `K = ω_j / tan(ω_j T/2)`. Sections
either **gain-stabilize** (attenuate the mode below 0 dB crossing) or **phase-stabilize**
(shape phase to encircle the mode stably). The existing `filters.rs` `Biquad` hosts each
section; the new work is the prewarped-Tustin coefficient generator and the **pole/zero
frequency scheduling** (with mode-frequency dispersion) — never raw-tap scheduling.

Co-design (offline) is a single structured-H∞ / μ problem: parameterize filter coefficients
as tunable blocks, minimize `‖W_perf·S‖_∞` subject to `‖flex-channel‖ < 1` for robustness to
`±` frequency dispersion (Hanson, arXiv:1802.01875). The FC imports the resulting sections as
a `BendingFilterTable`.

```rust
pub struct BiquadSection { pub b: [f64;3], pub a: [f64;3] } // a[0]=1
/// Prewarped-Tustin discretization of one continuous 2nd-order section
/// at mode frequency omega_mode (rad/s), step dt_s.
pub fn prewarped_tustin_notch(
    omega_n: f64, zeta_n: f64, omega_d: f64, zeta_d: f64,
    omega_mode: f64, dt_s: f64,
) -> Result<BiquadSection, &'static str>;

pub struct BendingFilterSectionSpec {
    pub mode_frequency_rad_s: f64,
    pub zero_frequency_rad_s: f64,
    pub zero_damping_ratio: f64,
    pub pole_frequency_rad_s: f64,
    pub pole_damping_ratio: f64,
}

pub struct BendingFilterNode { pub schedule_value: f64, pub sections: Vec<BendingFilterSectionSpec> }
pub struct BendingFilterTable { pub nodes: Vec<BendingFilterNode> }
impl Table for BendingFilterTable { /* validate stable poles |z|<1 */ }
```

### 3.7 TVC actuator + tail-wags-dog + INDI/INCA (Tier 3)

**TWD term (sim plant, doc `01`/`05`).** Engine of mass `m_e`, inertia `I_e` about the
gimbal pivot at distance `l` from CG, gimbal angle `δ`. Vehicle pitch torque:

```
M = T · l_arm · sin(δ) − I_e·δ̈ − m_e·l·(coupled accel),
```

the `−I_e·δ̈` being the TWD reaction (gimbaling the engine accelerates its own inertia,
producing a reaction torque on the vehicle). **Actuator:** 2nd-order
`δ̈ + 2ζ_a ω_a δ̇ + ω_a² δ = ω_a² δ_cmd` with rate/position limits and a load-torque feedback
that lowers effective `ω_a`.

**INDI (robust to the unmodeled TWD).** Instead of inverting the full model, use measured
angular acceleration and increment:

```
δ_cmd = δ_prev + B_eff^{-1} (ν − ω̇_meas),
```

`ν` = pseudo-control (desired angular accel from the outer loop), `B_eff` = gimbal control
effectiveness — this cancels model-dependent terms (including TWD) using the measurement.
**INCA** couples INDI to the WLS allocator (incremental control allocation: the WLS solves
for `Δu` against the dense `B_eff` over the over-actuated cluster). **Pseudo-Control Hedging
(PCH)** feeds the actuator saturation/dynamics deficit back to the reference model so the
integrator does not wind up against actuator lag:
`ν_ref,hedged = ν_ref − (ν_commanded − ν_achievable_est)`. The existing `indi.rs` provides the
scalar increment; the new work is the dense-`B_eff` INCA bridge to the WLS allocator and the
PCH term.

### 3.8 Tightly-coupled GNSS/INS error-state EKF + MEKF (Tier 4)

**Tightly-coupled GNSS/INS (Groves ch. 14).** 17-state error vector
`x = [δp(3), δv(3), ψ(3 attitude error), b_a(3), b_g(3), δt_clk, δt_clk_dot]`. Nominal
strapdown propagation (ECEF or local-nav); error dynamics `ẋ = F x + G w` where `δv̇` couples
the attitude error `ψ` via the specific-force skew `−[f^b ×]`, plus Coriolis/transport and
gravity-gradient terms. **Measurement (raw observables):**

```
ρ̂_s = ‖r_sat − r_rx‖ + c·δt_clk        (predicted pseudorange)
z_s  = ρ_meas,s − ρ̂_s                    (innovation, per satellite)
H row = [ −e_s · (∂r_rx/∂δp) ,  0 … , 1(clock-bias) , 0 ]   (e_s = LOS unit vector)
Doppler row uses  e_s · (v_sat − v_rx)  and the clock-drift state.
```

Update (Joseph form for symmetry): `K = P Hᵀ (H P Hᵀ + R)^{-1}`, `x += K z`,
`P = (I−KH)P(I−KH)ᵀ + K R Kᵀ`. **Critical (project-memory fix):** apply **per-satellite
chi-square innovation gating**, *not* a 6-D block gate — the 6-D gate collapses under thrust;
raise velocity/accel process noise during powered flight, and prefer a **scalar-sequential**
update for determinism. Carrier-phase integer ambiguity (LAMBDA) is out of launch scope.

The synthetic raw observables themselves (ephemeris → sat pos/vel, ionosphere/troposphere,
pseudorange/Doppler) are produced in `openbmp-sensors` (doc `09`) and delivered through the
FC-bridge as a new `GnssObservableSet` topic; the FC consumes them, it does not generate them.

**MEKF with covariance reset (Markley 2023).** Global quaternion `q` carries full attitude;
local 3-parameter error `a = 2·δq_vec` keeps the covariance non-singular. State `x = [a(3),
b_g(3)]`. Propagation with `q̇ = ½ q ⊗ ω`, `b_g` random walk, `F` from `−[ω×]` and the bias
coupling. After each vector-measurement update, the **multiplicative reset**:
`q⁺ = q ⊗ δq(â)` with `δq = [1; ½â]` normalized, then `â ← 0`; the covariance reset maps `P`
through the first-order reset Jacobian `G(â) ≈ I − ½[â×]` (Markley et al. 2023 — distinct
from the measurement-update Jacobian). The existing `Mekf` (6-state) is promoted to a
`mekf.rs` module with the correct reset Jacobian and validated against the Markley test.

### 3.9 Redundancy / FDIR reconfiguration + abort-to-SAFE (Tier 4)

- **TMR voting:** `y_out = median(y1,y2,y3)` (already `MidValueSelectScalar`); vector
  fault-tolerant weighted mean with covariance weights, gating an outlier by Mahalanobis
  residual `r_i² = (y_i − y_cons)ᵀ S^{-1}(y_i − y_cons)`.
- **FDI by parity/residual:** analytical-redundancy residual `e = y_meas − C x̂` (innovation);
  declare fault when the GLRT/CUSUM statistic exceeds threshold with a confirmation count
  (existing `glrt.rs`/`fdir.rs`).
- **Engine-out reconfiguration (the structural reason for the dense `B`):** on confirmed loss
  of effector `k`, set `u_min,k = u_max,k = 0` (or drop column `B[:,k]`); the WLS allocator
  automatically redistributes across the remaining cluster. Re-trim absorbs the lost axial
  thrust + the off-CG moment of remaining engines as a new bias in the allocator/integrator.
  If the **attainable-moment set** no longer contains the demand (`v` outside `B·[u_min,u_max]`),
  raise a **degraded-mode flag** (`FDIR_BIT_ALLOCATION_INFEASIBLE`). The per-axis selector
  *cannot* do this; the dense `B` is the enabler. The current substrate already marks saturated
  dense allocation output as allocation-infeasible in FDIR after availability masking and maps that
  signal into the commander's `health.degraded` region with configurable confirmation counters
  (`REQ-GNC-007` / `V-GNC-007`). The mixer also treats `VehicleStatus.safe_state_requested`
  as an engine-demand override and publishes zero-throttle shutdown commands even without a fresh
  autopilot engine demand (`REQ-GNC-008` / `V-GNC-008`) plus a
  `propulsion.propellant_safing_cmd` intent topic for feed isolation, valve closure,
  pressurization disable, and drain disable (`REQ-GNC-010` / `V-GNC-010`). The runner now
  consumes that intent in configured feed-network valve commands and propellant-budget
  feed/drain outputs (`REQ-GNC-011` / `V-GNC-011`) and propagates SAFE pressurization-disable
  intent into the simplified propellant-budget report (`REQ-GNC-016` / `V-GNC-016`). The autopilot
  now turns SAFE into a target-free latched entry-attitude hold, suppressing normal
  guidance/trajectory/load-relief references while damping rates and emitting zero-throttle shutdown
  demand (`REQ-GNC-012` /
  `V-GNC-012`). A focused FC pipeline test now validates non-allocation FDIR → commander SAFE →
  autopilot SAFE hold → mixer shutdown/safing handoff on one bus (`REQ-GNC-013` /
  `V-GNC-013`). A focused `FcRunner` test now validates that the runner-facing accessors surface
  SAFE status, SAFE hold, shutdown engine demand, propellant safing intent, and per-engine shutdown
  commands after an injected non-allocation FDIR trip (`REQ-GNC-014` / `V-GNC-014`). A clipped
  rigid-body TVC session now validates that a HAL-portable mission `request_safe_state` event
  drives all nine engines to zero-thrust Shutdown, keeps the rigid-body state finite/bounded, and
  reaches end time (`REQ-GNC-015` / `V-GNC-015`). Failed-engine snapshots now also publish failed
  engine ids through `EffectorAvailability`, and the mixer shuts those engines down while
  redistributing scalar throttle over the remaining phase-authorized engines with a 1.0 clamp
  (`REQ-GNC-017` / `V-GNC-017`). Scalar fan-out can now also apply calibrated per-failed-engine
  survivor gimbal trim biases from `[[fc.engine_out_trim]]` after throttle retrim
  (`REQ-GNC-022` / `V-GNC-022`). SAFE drain-disable intent now also reaches the runner tank rack:
  declared tank drains, propellant-budget drains, and RCS feed drains are ignored once active SAFE
  latches drains disabled (`REQ-GNC-018` / `V-GNC-018`). SAFE pressurization-disable intent now
  also fails feed pressure closed in the propellant-budget report even without feed isolation or
  valve-close requests (`REQ-GNC-019` / `V-GNC-019`). SAFE now also carries a distinct dump intent
  and the runner opens scenario-declared constant-rate tank dump paths while ordinary drains stay
  disabled (`REQ-GNC-020` / `V-GNC-020`). Tanks can also declare a pressure-coupled
  `safe_dump_orifice`, and `TankRack` computes live ullage-pressure orifice flow when SAFE dump is
  active (`REQ-GNC-021` / `V-GNC-021`). Tanks can now also declare a choked ullage-gas
  `safe_dump_vent` that reduces the next-step pressure available to that liquid orifice
  (`REQ-GNC-023` / `V-GNC-023`) and can opt into an adiabatic ullage-gas thermal model
  for that vent (`REQ-GNC-025` / `V-GNC-025`). SAFE dump orifices and vents can now also
  declare deterministic backpressure ramps for time-varying downstream pressure
  (`REQ-GNC-026` / `V-GNC-026`), and SAFE dump vents can opt into a wall-relaxed
  adiabatic thermal model with required wall-temperature/tau validation
  (`REQ-GNC-027` / `V-GNC-027`). SAFE dump orifices and vents can also source
  runtime backpressure from the current ambient atmosphere sample
  (`REQ-GNC-028` / `V-GNC-028`). Tanks can also declare an opt-in finite-volume
  pressurant hardware substrate that feeds later pressure-coupled SAFE dump
  orifice flow unless SAFE disables pressurization (`REQ-GNC-029` / `V-GNC-029`).
  Scalar fan-out can now also derive an optional
  `EngineOutTrimTable` from declared engine mount geometry and max thrust via
  `[fc.engine_out_trim_auto]` (`REQ-GNC-024` / `V-GNC-024`). Flight-calibrated integrator
  bias estimation plus cryogenic phase-change, flight-calibrated regulator/bottle
  thermal coupling, plume-coupled vent backpressure, and production pressurant
  hardware validation remain open.
- **Abort-to-SAFE:** a monotonic guarded state machine `NOMINAL → DEGRADED → SAFE` (new bits +
  the commander's existing `MissionStateMachine`), entered on rate divergence, structural-load
  exceedance, double-sensor fault, or attitude-unrecoverable, commanding **thrust termination
  / attitude-hold / propellant-safe**. Confirmation timers and the engine thrust-termination
  command substrate and propellant-safing intent topic are implemented, the runner consumes that
  safing intent in feed/budget paths, the autopilot has a target-free SAFE attitude-hold substrate,
  focused FC and `FcRunner` pipeline tests validate FDIR-to-hold/shutdown/safing handoff, a clipped
  rigid-body TVC run validates scenario-driven SAFE shutdown/recovery (`REQ-GNC-015` /
  `V-GNC-015`), the runner budget report consumes SAFE pressurization-disable intent
  (`REQ-GNC-016` / `V-GNC-016`), and the testkit compile-fail tripwire proves the public
  `RequestSafeState` action cannot carry a ground aimpoint (`REQ-GNC-009` / `V-GNC-009`).
  The propellant-safe path now has configured, pressure-coupled, thermally coupled, and ambient-
  backpressure SAFE dump substrates plus an opt-in finite-volume pressurization substrate; plume-
  coupled vent backpressure, cryogenic phase-change, and production regulator/bottle validation
  remain open. **It never commands a ground aimpoint, an alternate target, or a redirect-to-impact**
  (§6).

```rust
/// Allocation feasibility w.r.t. the attainable moment set after a
/// reconfiguration. Used to raise the degraded-mode flag.
pub fn moment_demand_attainable(
    b: &EffectivenessMatrix, v: &[f64],
    u_min: &[f64], u_max: &[f64],
) -> bool;

/// Forward-only abort ladder. No transition carries a target; every
/// SAFE action is thrust-off / attitude-hold / propellant-safe.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AbortMode { Nominal, Degraded, Safe }
pub struct AbortLadder { /* guarded transitions + confirmation timers */ }
```

### 3.10 Fidelity tiers

| Tier | Capability | In-repo? | Label target |
|---|---|---|---|
| **T0** | *Baseline to preserve.* Per-axis diagonal LQR(2-state)/INDI/L1/box-MPC; per-axis WLS/redistributed allocators (degenerate to selector); voter/GLRT/IMM/UD/SR-UKF/FDIR/scheduler present; 2121+ tests green; coupled-inertia / gimbaled-cluster cases rejected at load. | yes | (existing) |
| **T1** | Dense-`B` Härkegård active-set WLS, box+rate bounds, warm-start; per-axis allocator kept as diagonal special case. | yes | `validated-toy` (code-to-code vs QCAT) |
| **T2** | `n×n` coupled-plant LQR/LQG (non-diagonal inertia, integral states) offline via doubling DARE; gain scheduling + bumpless transfer; pole/zero-parameterized bending filters. | partial (synthesis offline; in-loop in-repo) | `validated-toy` (linear margin checks) |
| **T3** | Blended drift-/load-min law + α/β estimator + `q·α` limiter + gyro blender; TWD + actuator plant; INDI/INCA + PCH. | yes (law in-repo; flex/TWD plant from `01`/`05`) | `validated-toy`→`research` |
| **T4** | Tightly-coupled GNSS/INS error-state EKF (raw observables, per-sat gating); MEKF + covariance reset; offline H∞/μ scheduled controller I-load; engine-out `B`-column drop + re-solve; abort-to-SAFE ladder. | in-loop yes; H∞/μ synthesis offline; observables from `09` | `research` (RTKLIB/Markley benchmarks) |

---

## 4. Invariant preservation

- **Byte-determinism (`00` §3.1).** All new arithmetic uses locked operand order and
  explicit parentheses; **no `f64::mul_add`**, no wall-clock, no system RNG, no unordered
  iteration. The WLS active-set uses a fixed Householder QR order with **lowest-index
  tie-break** and **no multithreaded BLAS** (nalgebra single-threaded path pinned). The `n×n`
  DARE doubling fixes the LU pivot order; iteration count is bounded by `DARE_MAX_ITERATIONS`.
  The tight-EKF uses **scalar-sequential** updates (deterministic, gate-then-apply per
  satellite in fixed PRN order). The CI byte-diff gate on the canonical scenario set is the
  enforcement.
- **Byte-stable-by-default (`00` §3.2).** Every new capability is **off by default**, gated by
  an explicit scenario block (`[vehicle.allocation.dense_b]`, `[fc.gain_schedule]`,
  `[fc.load_relief]`, `[fc.bending_filters]`, `[fc.tight_nav]`, `[fc.reconfiguration]`). Until
  a scenario opts in, the diagonal allocators and scalar LQR run unchanged and all golden
  archives stay byte-identical. The per-axis allocator is retained as the literal diagonal
  special case of the dense `B`.
- **FC hardware-portability lock (`00` §3.3).** **No new forbidden edge.** All code lands in
  `openbmp-fc` over its existing portable deps + `nalgebra` (already present) and optional
  `clarabel` (already present). Simulator data — inertia tensor, gimbal geometry → `B` rows,
  wind, density, multi-station gyro slopes, raw GNSS observables, per-engine state — arrives
  **only via FC-bridge bus topics**. New scenario-mirrored FC-local types (e.g.
  `EffectivenessMatrix`, `ScheduleVariable`) follow the existing `BodyAxis`-mirrors-`TorqueAxis`
  pattern so the FC never imports `openbmp-scenario`. The
  `crates/openbmp-testkit/tests/fc_dependency_tripwire.rs` stays green.
- **Lockstep-clock + no-hot-path-allocation (`00` §3.4).** Time only through the injected
  `Clock`. The WLS allocator and scheduled controller **pre-allocate** their work buffers at
  registration (matrix dimensions are known once `B` is loaded) and reuse them per tick; the
  per-tick path does not allocate or clone binding/topic tables. Offline-synthesized gains
  enter through the `tables.rs` two-buffer swap (validated-then-activated), never reallocated
  in the loop. Enforced by `crates/openbmp-testkit/src/fc_lints.rs`.
- **Four-pillar provenance (`00` §3.5).** The `B` matrices, gain schedules, filter sections,
  and robust-controller I-loads are scenario/data TOML with sibling `provenance.md` + SHA-256
  pin; benchmark constants (ADMIRE `B`, RTKLIB ephemeris) live only in their declared
  source-of-truth files (allow-listed in the inline-data tripwire **in the same PR**). **No
  WGS84 `GM_earth` / `J2` / `R_earth` literals** appear in any new doc or non-allow-listed
  source — referenced symbolically (the tight-EKF and gravity-gradient terms use the
  `openbmp-physics` constants, never inline numerals).
- **Validation labels (`00` §3.8).** Each tier declares exactly one of
  `experimental → checked → validated-toy → research` with the required evidence and a
  tolerance-table TOML for every numeric claim (§5). No artifact claims `flight-qualified` /
  `certified` / `operational` / `mission-ready`.
- **Forward-only locks (`00` §3.7, §6).** The allocator, controllers, load-relief law, and
  abort ladder accept **no target/range/aimpoint field**; locks tighten with capability — each
  new I-load type adds a tripwire that proves it cannot express a ground-aimpoint solution
  (§6).

---

## 5. V&V plan

Verification follows the five-layer ladder (`docs/verification.md`): MMS / analytic →
model-verification → code-to-code → public benchmark → UQ reporting. Tolerance tables ship as
TOML under `tests/expected/` with each WP.

| Case | Method | Tier | Tolerance | Label earned |
|---|---|---|---|---|
| Dense-`B` WLS vs QCAT `wls_alloc`/`dir_alloc` on published matrices (ADMIRE `B`, synthetic 3-engine TVC cluster) | code-to-code | T1 | match to `≤1e-9` on the attainable region; **identical active set** at the saturation boundary | `validated-toy` |
| `n×n` DARE/CARE vs python-control `dlqr`/SLICOT on a random stabilizable `(A,B,Q,R)` bank | code-to-code + residual | T2 | DARE residual `‖A'PA−P−A'PB(R+B'PB)⁻¹B'PA+Q‖ < 1e-10`; gains match to machine tol | `validated-toy` |
| Bending/slosh closed-loop margins vs analytic free-free beam + pendulum slosh (known `ω_j`, `ζ_j`) | MMS + linear Nichols/Nyquist | T2/T3 | gain margin `≥ 6 dB`, phase margin `≥ 30°` **at every schedule node AND interior point** | `validated-toy` |
| α/β estimator vs analytic relative-velocity geometry under modeled wind | MMS | T3 | `α̂` error `< 1e-9 rad` on the noise-free manufactured case | `validated-toy` |
| Load-relief `q·α` reduction: Dryden/von-Kármán gust through max-Q, drift-min (`k=0`) vs blended (`k=1`) in the live SIL | qualitative trend | T3 | blended **reduces peak `q·α`** with the expected pitch-command trade, matching the SLS AAS 14-038 blended-vs-nonblended trend | `research` (qualitative) |
| MEKF covariance reset vs Markley 2023 (JGCD 46(10)) + brute-force MC of attitude-error distribution + analytic TRIAD/QUEST static two-vector | analytic + MC | T4 | reproduce published reset-Jacobian behavior; TRIAD/QUEST match to `1e-9` | `research` |
| Tight GNSS/INS vs RTKLIB observables from published broadcast ephemeris (IGS/CDDIS RINEX nav) on a synthetic ascent | code-to-code | T4 | position/velocity error **bounded with `<4` sats**; per-satellite gating rejects an injected outlier/spoof **while the 6-D block gate (the known OpenBMP failure mode) does not** | `research` |
| Engine-out reconfiguration: scripted single-engine-loss injection in the live SIL | scenario integration | T4 | WLS redistributes across remaining engines; attitude held within margin; if attainable-moment set no longer contains demand → FDIR raises DEGRADED and (if uncontainable) transitions to SAFE | `validated-toy` |
| 6-DOF rigid-body integration vs an open trajectory tool (dymos/GMAT orbital arc; an open re-derivation of the POST2-class EOM formulation) under identical forcing | code-to-code | T2 | translational/rotational state agreement within the documented integrator tolerance | `validated-toy` |

Each numeric WP adds a tolerance-table TOML and exercises the model **through a scenario**
(not only in isolation), per the `13` §3 checklist. The `q·α` and SLS-trend cases earn only a
**qualitative** `research` framing — they match a published *trend*, not a vehicle's
telemetry.

---

## 7. Dependencies on other parity docs

- **`01-flexible-multibody-dynamics.md`** — the coupled rigid-body plant (non-diagonal `J`,
  gyroscopic `−ω×(Jω)`) and the FFRF flex substrate that produces the bending generalized
  coordinates the LQG observer and bending filters act on. **Gates T2 (coupled LQR needs the
  non-diagonal inertia plant) and T3 (TWD/flex plant).**
- **`02-structural-dynamics-loads-slosh-pogo.md`** — modal model (`ω_j`, `ζ_j`, mode shapes /
  sensor slopes) for the bending-filter co-design and the slosh pendulum for the load-relief /
  filter V&V; the `q·α` structural-load ceiling traces to the loads model.
- **`03-aerodynamics-database-and-cfd-coupling.md`** — aero coefficients that set the
  load-relief `q·α` gains and the open-CFD code-to-code substitute for the ceiling's
  provenance.
- **`05-propulsion-high-fidelity.md`** — engine mass/inertia, thrust, and the actuator/TWD
  plant parameters for §3.7; per-engine state for engine-out reconfiguration.
- **`07-trajectory-optimization-and-mission-design.md`** — the flight/orbital-condition
  optimizer scope used by the `q·α`-limiting guidance variant; shares the terminal-condition
  vocabulary lock.
- **`08-environment-gravity-and-frames.md`** — frames (ECI/ECEF/local-nav), gravity (for the
  tight-EKF gravity-gradient term), and the wind/gust models (Dryden/von-Kármán, GRAM) the α
  estimator and load-relief consume.
- **`09-sensors-navigation-and-actuators.md`** — force-based IMU, **raw pseudorange/Doppler
  GNSS observable generation** (the tight-EKF measurement source), actuator dynamics, and
  multi-station gyro models for the gyro blender. **Gates T4 tight-nav.**
- **`10-flight-software-in-the-loop-xil.md`** — the live SIL where load-relief, engine-out,
  and abort-to-SAFE are exercised end-to-end; the fault-injection ports for the scripted
  failures.
- **`11-monte-carlo-uq-and-validation.md`** — the dispersion campaigns (bending/slosh
  frequency, slosh mass, aero, actuator, delay) for the AVSAT-style margin sweeps and the
  Markley reset MC.
- **`12-determinism-realtime-and-compute.md`** — the determinism gate that the dense linear
  algebra and scalar-sequential EKF must satisfy.

DAG position: Phase C (`00` §5), depends on the coupled plant from Phase A/B. Recommended
first WP is the dense-`B` WLS allocator (T1) — self-contained, code-to-code-verifiable, and
the structural enabler for engine-out reconfiguration.

---

## 8. Open-source leverage

| Tool | License | Use mode |
|---|---|---|
| Härkegård **QCAT** toolbox (`wls_alloc`/`dir_alloc`/`sls_alloc`) | academic, freely distributed | **ingest as reference/port:** port the active-set WLS to Rust; use MATLAB reference outputs as golden code-to-code test vectors. Reimplement clean-room (algorithm reference, not vendored code). |
| **Clarabel.rs** | Apache-2.0, pure Rust | **couple:** QP/SOCP backend for constrained allocation + small-angle MPC when active-set WLS is insufficient (soft constraints, SOCP min-time-to-saturation). **Already a pinned optional dep** behind the `mpc` feature — no FFI, deterministic. |
| **OSQP** + `osqp.rs` | Apache-2.0 | **couple/port:** alternative QP backend (ADMM). C core must be vendored/pinned and built reproducibly for lockstep determinism — prefer Clarabel for the no-FFI path. |
| **python-control / Slycot**, **dkpy** | python-control BSD-3; **Slycot GPL-3.0 (arm's length — offline only, never linked)**; dkpy MIT/permissive | **ingest/couple-offline:** compute LQR/LQG/H∞/μ gains and DARE/CARE OFFLINE, export reduced `(A,B,C,D)` controllers as scenario I-loads; the validation oracle for the in-repo `n×n` DARE doubling. GPL stays out of the Rust workspace. |
| **RTKLIB** | BSD-2-Clause | **ingest:** reference GNSS observable models (ephemeris → sat pos/vel, iono/tropo, pseudorange/Doppler) to validate the tight-EKF measurement model and to generate synthetic-but-physical observables in `openbmp-sensors` (doc `09`). |
| **faer-rs / nalgebra** (lapack feature) | faer MIT/Apache-2.0; nalgebra Apache-2.0 | **couple:** dense LA backbone — `n×n` DARE (doubling, optional Schur cross-check via faer), QR for active-set WLS, Cholesky for Joseph-form/UD updates. nalgebra already in the FC dep graph. |
| **NASA NESC / GVSC** SLS ascent-control reports + AVSAT method (AAS 14-038/14-051, NTRS 20110015701) | US Gov, public domain | **ingest:** the architecture + stability-analysis blueprint (gyro blender, bending-filter array, load relief, AAC, AVSAT margin sweeps) — design + V&V methodology source, not code. |
| **NASA cFS / cFE** (Table/Time/Event services) | Apache-2.0 | **couple (future, doc `10`):** the `tables.rs` registry already mirrors cFE-TBL semantics; bridge a cFS app over the FMI/bridge crate for the SIL North Star. |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each, green on the full `13` §2 gate set. No WP
introduces a new crate; all land in `openbmp-fc` behind off-by-default scenario blocks.

---

### WP-06.1 — Dense-`B` Härkegård active-set WLS allocator

- **goal:** Replace the per-axis selector with a true dense-`B` bounded-WLS allocator so a
  multi-engine TVC cluster sharing pitch/yaw/roll and an RCS quad with redundant roll allocate
  correctly. Structural enabler for engine-out reconfiguration (WP-06.4-b). Keep the per-axis
  allocator as the diagonal special case.
- **status:** implemented — `EffectivenessMatrix`, `EffectorBounds`,
  `WlsActiveSetAllocator`, `WlsAllocationOutput`, and `DenseWlsControlAllocator`
  now provide an FC-portable dense-`B` WLS path with coupled rows/columns,
  rate-tightened asymmetric bounds, phase-gated fixed-zero columns, deterministic
  active-set tests, no-default-features coverage, schema-v3
  `[fc.autopilot_allocation] kind = "dense_wls"` parsing, runner construction
  from `dense_effectors`, mixer `EffectorCommandSet` routing, and unchanged legacy
  diagonal allocator dispatch (`REQ-GNC-001` / `V-GNC-001`). `data/gnc/
  qcat-wls-allocation-tolerance-v1.toml` now covers QCAT `wls_alloc`-derived
  ADMIRE and synthetic 3-engine TVC tolerance cases, including active-set signs
  and iteration counts, and the dense B-matrix schema rejects target/range/
  aimpoint vocabulary. The WP-specific FC dependency tripwire, FC lint tripwires,
  runner telemetry byte-preservation check, and allocator end-to-end byte-stability
  check are now part of `V-GNC-001`. The local §2 merge sweep is green for
  workspace build, workspace clippy, full workspace cargo test, GNC provenance,
  scenario provenance, parity docs, requirements traceability, and supply-chain
  tools; `cargo nextest` was not installed locally, so
  `cargo test --workspace --all-features --locked` is the local test substitute.
- **fidelity_tier:** T1
- **depends_on:** []
- **new_crates:** none
- **touched:** `crates/openbmp-fc/src/allocation.rs`, `crates/openbmp-msgs/src/lib.rs`
  (B-matrix I-load topic), `crates/openbmp-fc/src/mixer.rs` (route per-effector commands),
  `crates/openbmp-testkit/` (QCAT golden vectors + tripwire allow-list).
- **approach:** Implement §3.2 — `EffectivenessMatrix`, `WlsActiveSetAllocator`, γ-weighted
  surrogate, finite-step active-set with warm-start, box+rate bounds. Pre-allocate work
  buffers at construction. Add as a third `ControlAllocator` variant; diagonal allocators
  unchanged.
- **acceptance:**
  - new dense-`B` allocator off by default (`[vehicle.allocation.dense_b]`); all canonical
    goldens byte-identical.
  - code-to-code vs QCAT `wls_alloc` on the ADMIRE `B` and a synthetic 3-engine TVC cluster
    matches to `≤1e-9` on the attainable region **and reports the identical active set** at the
    saturation boundary (tolerance table). **Implemented:** the tolerance table lives at
    `data/gnc/qcat-wls-allocation-tolerance-v1.toml` and is enforced by
    `dense_wls_allocator_matches_qcat_admire_and_synthetic_tolerance_tables`.
  - deterministic across reruns (bit-identical, `to_bits()` test); warm-start convergence
    bounded (`iterations ≤ N`).
  - no new forbidden FC edge; `fc_dependency_tripwire` + `fc_lints` green; the
    runner telemetry byte-preservation and allocator E2E byte-stability checks are
    green in `V-GNC-001`. Workspace build/clippy/test, provenance, traceability,
    parity-doc, and supply-chain gates are green in the local merge sweep, with
    `cargo test` substituted for unavailable local `cargo nextest`.
- **validation_label:** `validated-toy`
- **dual_use_note:** `B` carries only gimbal/thruster geometry; the parser regression
  `fc_autopilot_allocation_dense_wls_rejects_targeting_vocabulary` asserts the
  B-matrix I-load schema has no range/target/aimpoint field.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** does NOT validate against a real vehicle's flight `B` matrix (proprietary);
  ADMIRE/synthetic clusters only.

---

### WP-06.2-a — `n×n` coupled-plant DARE solver (doubling) + LQR

- **goal:** Generalize the scalar 2-state DARE to an `n×n` coupled-plant LQR so a vehicle with
  products of inertia / gyroscopic cross-coupling flies stably; provide the in-repo
  verification oracle for the offline gain pipeline.
- **status:** implemented — `solve_dare_nxn_doubling`,
  `solve_dare_nxn_doubling_with_options`, `dense_dare_residual_norm`, and
  `CoupledLqrSolution` now provide the FC-portable dense DARE/LQR oracle, and
  `CoupledGainNode` provides the validated table target for offline gains
  (`REQ-GNC-002` / `V-GNC-002`). The solver is deterministic, bounded, validates
  SPD `Q`/`R`, matches `data/gnc/coupled-lqr-dare-tolerance-v1.toml` generated
  with python-control `dlqr`/SciPy `solve_discrete_are`, and fails closed on an
  uncontrolled unstable mode. The local §2 merge sweep is green for this slice:
  workspace build, workspace clippy, full workspace cargo test, GNC/scenario
  provenance, traceability, parity-doc validation, cargo deny, cargo audit,
  cargo machete, and formatting all pass; local `cargo nextest` is unavailable,
  so the full workspace cargo test is the recorded substitute.
- **fidelity_tier:** T2
- **depends_on:** [WP-06.1]
- **new_crates:** none
- **touched:** `crates/openbmp-fc/src/lqr.rs`, `crates/openbmp-fc/src/tables.rs`
  (`CoupledGainNode` I-load).
- **approach:** Implement §3.3 — `solve_dare_nxn_doubling` (Cholesky/LU-based, pinned operand
  order), discrete LQR gain assembly, `CoupledGainNode` table type. Synthesis offline or
  in-repo; only `K·x` runs in the loop.
- **acceptance:**
  - off by default; goldens byte-identical.
  - DARE residual `< 1e-10` and gains match python-control `dlqr`/SLICOT to machine tol on a
    random stabilizable `(A,B,Q,R)` bank (tolerance table). **Implemented:** the
    current table is a scalar plus coupled toy bank generated with
    python-control `dlqr` / SciPy `solve_discrete_are`.
  - bit-stable across reruns; converges within bounded iterations; fails closed on
    non-stabilizable input. **Implemented:** covered by
    `coupled_lqr_dense_solver_matches_tolerance_table` and
    `coupled_lqr_dense_solver_fails_closed_on_uncontrolled_unstable_mode`.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line (gains carry flight-condition node tag only); tripwire on the
  `CoupledGainNode` schema.
- **est_effort:** 2 weeks
- **parity_ceiling:** does NOT validate the *vehicle* gains; only the solver vs an open oracle.

---

### WP-06.2-b — Gain scheduling + bumpless transfer + LQG observer

- **goal:** Schedule the coupled gains along the ascent (Mach/`q̄`/mass/time) with bumpless
  transfer, and add the LQG observer for unmeasured (flex) states with LTR.
- **status:** implemented (2026-06-26) — `tables.rs` exposes `GainScheduleTable`,
  `ScheduleVariable`, and `GainInterp` for strictly sorted scalar scheduled
  `CoupledGainNode` entries with optional all-or-none observer gains;
  `schedule.rs` exposes `ScheduledController`, gain interpolation,
  conservative closed-loop row-sum and SISO gain/phase margin sweeps, and
  least-squares bumpless integral-state reset; `lqr.rs` exposes the dual-DARE
  LQG observer gain helper (`REQ-GNC-003` / `V-GNC-003`). The
  `data/gnc/gain-schedule-lqg-tolerance-v1.toml` fixture checks node/interior
  interpolation, `>=6 dB`/`>=30 deg` SISO margins, conservative row-sum
  stability, bumpless command continuity, and SciPy dual-DARE observer
  covariance/gain agreement. This is validated-toy method evidence only, not a
  real vehicle schedule or MIMO robustness certification. `V-GNC-003` focused
  gates and the batch §2 merge sweep are green.
- **fidelity_tier:** T2
- **depends_on:** [WP-06.2-a]
- **new_crates:** none
- **touched:** new `crates/openbmp-fc/src/schedule.rs`, `crates/openbmp-fc/src/lqr.rs`
  (observer/dual DARE), `crates/openbmp-fc/src/tables.rs` (`GainScheduleTable`).
- **approach:** §3.4 — `GainScheduleTable`, `ScheduledController` with pole/zero-safe
  parameterization, integral-state reset on boundaries; dual DARE for `L`, Doyle-Stein LTR
  offline.
- **acceptance:**
  - off by default; goldens byte-identical.
  - linear eigenvalue/margin check passes (`≥6 dB`/`≥30°`) **at every schedule node and a swept
    grid of interior points** (tolerance table); no bump at boundary transitions (integral
    reset verified). **Implemented:** the toy fixture covers nodes plus an interior point
    with SISO gain/phase margins, conservative row-sum stability, and command-continuity
    checks.
  - all §2 gates green. **Implemented:** focused `V-GNC-003` gates, workspace
    build/clippy/test, formatting, traceability, parity-doc, provenance, and
    supply-chain hygiene gates are green in the 2026-06-26 batch sweep.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line; schedule columns carry no target — tripwire on the schedule
  table schema.
- **est_effort:** 2 weeks
- **parity_ceiling:** does NOT reproduce a flown schedule; method-only on open plants.

---

### WP-06.2-c — Structural bending/slosh filter array (prewarped Tustin, scheduled)

- **goal:** Co-designed (offline) bending/slosh notch + phase/gain-stabilization filter array,
  imported as a table, with pole/zero scheduling against mode-frequency dispersion.
- **status:** implemented (2026-06-26) — discrete prewarped-Tustin section
  generation, physical-parameter table scheduling, stable-pole fail-closed validation, and
  analytic bending/slosh margin tolerance evidence are implemented; off-default FC runtime
  gyro/sensor-chain insertion is implemented; `V-GNC-004` focused gates and the batch §2
  merge sweep are green.
- **fidelity_tier:** T2/T3
- **depends_on:** [WP-06.2-b]
- **new_crates:** none
- **touched:** `crates/openbmp-fc/src/filters.rs`, `crates/openbmp-fc/src/tables.rs`
  (`BendingFilterTable`), `crates/openbmp-fc/src/csi.rs`, `crates/openbmp-fc/src/autopilot.rs`,
  `data/gnc/bending-filter-tustin-tolerance-v1.toml`.
- **approach:** §3.6 — `prewarped_tustin_notch`, `BendingFilterTable`, gyro/sensor-chain
  insertion; schedule pole/zero frequencies, never raw taps.
- **acceptance:**
  - off by default; goldens byte-identical. **Implemented:** no runtime insertion path is
    active by default unless `AutopilotParams::bending_filter` is installed; the workspace
    batch test sweep is green.
  - closed-loop margins vs analytic free-free beam + pendulum slosh (MMS, known `ω_j`/`ζ_j`)
    meet `≥6 dB`/`≥30°` at nodes and interior points (tolerance table). **Implemented:** the
    current toy table covers node/interior physical-parameter
    interpolation, mode-frequency attenuation (`≥18 dB`), and filtered analytic
    bending-plus-slosh modal margins (`≥6 dB`/`≥30°`) at both nodes and an interior point.
  - filter sections validated stable (`|z|<1`) at load; fail-closed on unstable poles.
    **Implemented:** `BiquadSection::has_stable_poles`, `prewarped_tustin_notch`, and
    `BendingFilterTable` validation reject invalid/unstable sections before activation; the
    autopilot runtime rejects unsupported live schedule variables and invalid regenerated
    sections.
  - all §2 gates green. **Implemented:** focused `V-GNC-004` gates, workspace
    build/clippy/test, formatting, traceability, parity-doc, provenance, and
    supply-chain hygiene gates are green in the 2026-06-26 batch sweep.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** depends on doc `02` for real mode shapes; uses generic modal models.

---

### WP-06.3-a — Angle-of-attack estimator + `q·α` limiter + drift/load-min blend

- **goal:** Blended drift-minimum / load-minimum ascent control through max-Q with an α/β
  estimator and a `q·α` (structural-load) limiter, plus a gyro blender.
- **status:** implemented (2026-06-26) — `loadrelief.rs` now exposes the FC-portable
  alpha/beta estimator, validated load-relief blend schedule, structural `q·α` limiter, and
  offline-weighted gyro blender; the autopilot has an off-default
  `AutopilotParams::load_relief` insertion path that consumes position, environment, and
  optional modeled-wind topics before the attitude loop; `openbmp-msgs` adds an append-only
  `estimator.wind` topic and the runner FC bridge publishes modeled ECI wind. The scenario
  v3 `[fc.autopilot_params.load_relief]` block validates the I-load and rejects
  target/aimpoint vocabulary; focused controller-SIL Dryden/von-Kármán envelopes prove
  post-response peak `q·α` reduction versus drift-minimum with nonzero pitch-command trade.
- **fidelity_tier:** T3
- **depends_on:** [WP-06.2-c]
- **new_crates:** none
- **touched:** new `crates/openbmp-fc/src/loadrelief.rs`, `crates/openbmp-fc/src/autopilot.rs`
  (insert load-relief term), `crates/openbmp-msgs/src/lib.rs` (wind topic from FC-bridge),
  `crates/openbmp-scenario/src/document.rs`, `crates/openbmp-scenario/src/scenario.rs`,
  `crates/openbmp-runner/src/fc.rs`, `crates/openbmp-runner/src/fc_bridge.rs`,
  `crates/openbmp-runner/src/point_mass.rs`, `crates/openbmp-runner/src/rigid_body.rs`,
  `docs/scenario-format.md`, `data/gnc/load-relief-alpha-beta-tolerance-v1.toml`.
- **approach:** §3.5 — `estimate_alpha_beta`, `LoadReliefParams`, blend schedule `k(t)`,
  `q·α` ceiling, `GyroBlender`. Wind/density via FC-bridge topics.
- **acceptance:**
  - off by default; goldens byte-identical. **Implemented:** default
    `AutopilotParams::load_relief = None` leaves the actuator command unchanged; the runtime
    pitch correction appears only when the option is installed, and the scenario block is
    v3-only/off-default.
  - α/β estimator matches analytic relative-velocity geometry to `<1e-9 rad` on the noise-free
    MMS case (tolerance table). **Implemented:** the checked toy fixture covers alpha, beta,
    and qbar geometry with `1e-12 rad` angle tolerances.
  - live-SIL Dryden/von-Kármán gust through max-Q: blended (`k=1`) reduces peak `q·α` vs
    drift-min (`k=0`) with the expected pitch-command trade (qualitative trend vs AAS
    14-038). **Implemented:** the focused controller-SIL test runs deterministic Dryden and
    von-Kármán envelopes through the live autopilot job and a bounded response model; enabling
    load relief reduces post-response peak `q·α` by at least 20% against drift-minimum and
    requires nonzero pitch command. This is trend evidence, not vehicle-calibrated telemetry.
  - all §2 gates green. **Focused status:** `V-GNC-005` gates are green for FC, scenario,
    message, and runner surfaces; the workspace-wide §2 merge sweep remains the CI authority at
    the next batch boundary.
- **validation_label:** `validated-toy` (estimator) / `research` (qualitative trend)
- **dual_use_note:** **load-relief stays structural-load/drift-minimizing — refuse any variant
  parameterized by a ground aimpoint;** the `q·α` ceiling is a structural limit, not a target.
  The parser tripwire rejects target/aimpoint vocabulary in the scenario I-load block.
- **est_effort:** 2 weeks
- **parity_ceiling:** matches a published *trend*, not a vehicle's `q·α` telemetry; gains rest on
  generic aero (doc `03`).

---

### WP-06.3-b — TVC actuator + tail-wags-dog plant + INDI/INCA + PCH

- **goal:** Model the TVC actuator (2nd-order + rate/position limits + load-torque) and the TWD
  reaction, and compensate with INDI/INCA + pseudo-control hedging coupled to the dense-`B`
  allocator.
- **status:** partial (2026-06-26) — controller-side dense-INCA and pseudo-control hedging
  primitives are present behind the existing `indi` feature, and `openbmp-propulsion::tvc`
  now exposes the second-order TVC plant substrate with rate/position limits, external
  load-torque feedback, and `-J * delta_ddot` TWD inertial reaction reporting. `LiquidEngine`
  can opt into that plant and use the plant-delivered pitch/yaw angles for thrust-vector
  rotation while exposing the current actuator state and last TVC reaction step, and v3
  `[[vehicle.assembly.engines]]` blocks may opt into that plant through `tvc_actuator`.
  `data/gnc/tvc-twd-inca-tolerance-v1.toml` records the closed-form TVC step,
  load-torque reaction, and dense-INCA residual RMS tolerances used by the focused tests.
  `DenseIncaController` reuses the FC-local dense WLS allocator to solve the `B_eff`
  increment equation, reports requested/achieved increments and pseudo-control deficit, and
  `PseudoControlHedger` integrates only available pseudo-control. The FC runtime path now carries
  scalar INDI desired/measured angular-acceleration pseudo-control on
  `autopilot.pseudo_control_cmd`, the mixer can route that topic into a feature-gated
  `DenseIncaControlAllocator`, and v3 `kind = "dense_inca"` allocation blocks build through the
  runner under the `indi` feature. Dense INCA now shows effective-authority recovery against a
  deterministic TWD-like reaction series relative to a no-compensation baseline, structural
  reaction live-SIL proves the opt-in TWD-style reaction perturbs a synthetic TVC probe
  trajectory, and a feature-gated scalar INDI runner regression reduces live reaction-induced
  body-rate residual RMS below 75% of the PID residual on the shortened TVC probe. A
  feature-gated dense-INCA runner regression now allocates physical engine-gimbal dense aliases
  on that powered TVC probe and proves live structural-reaction body-rate residual RMS drops
  below 75% of the PID residual. The FC dictionary now also carries
  `actuator.effector_availability`, the mixer removes unavailable effector ids from phase
  authority before dense allocation, and the direct runner publishes failed physical engine
  pitch/yaw gimbal aliases into that topic; a shortened dense-INCA TVC probe proves a scripted
  engine hard-off reaches the failed state while the run survives to end time. Full engine-out
  retrim, degraded-mode flagging, abort-to-SAFE transitions, and flight-calibrated TVC authority
  remain open.
- **fidelity_tier:** T3
- **depends_on:** [WP-06.1, WP-06.3-a]
- **new_crates:** none
- **touched:** `crates/openbmp-propulsion/src/tvc.rs` (second-order TVC plant + load torque +
  TWD reaction substrate), `crates/openbmp-propulsion/src/engine.rs` (optional `LiquidEngine`
  TVC plant runtime path), `crates/openbmp-scenario/src/document.rs` / `scenario.rs` (v3
  `tvc_actuator` I-load parsing and validation), `crates/openbmp-runner/src/engines.rs`
  (runner construction), `crates/openbmp-runner/src/fc.rs` / `fc_bridge.rs` (topic registration
  and dense allocator construction), `crates/openbmp-msgs/src/lib.rs` (pseudo-control and
  effector-availability topics), `crates/openbmp-fc/src/autopilot.rs` / `mixer.rs` /
  `step.rs` / `indi.rs` (scalar INDI diagnostics, pseudo-control publication,
  effector-availability publication/consumption, dense-INCA allocation bridge, and PCH), reusing
  `crates/openbmp-fc/src/allocation.rs` dense WLS primitives,
  `crates/openbmp-runner/tests/structural_reaction.rs` (scalar INDI and dense-INCA live
  residual regressions),
  `data/gnc/tvc-twd-inca-tolerance-v1.toml` and `data/gnc/provenance.md`.
- **approach:** §3.7 — INCA against `B_eff`, PCH term, filtered angular-accel estimate;
  lift the current INDI/L1 composition ban only where characterized.
- **acceptance:**
  - off by default; goldens byte-identical. **Partial:** the new helpers are feature-gated under
    `indi`, the propulsion TVC plant remains opt-in, and existing scenarios do not declare
    `tvc_actuator`, so default simulations remain unchanged.
  - TVC actuator plant includes second-order bandwidth, rate/position limits, load torque, and
    TWD inertial reaction. **Partial:** `openbmp-propulsion::tvc` has focused tests for the
    closed-form unloaded step response, load-torque deflection reduction plus reaction torque
    reporting, rate/position limiting, and fail-closed invalid inputs. `LiquidEngine` has focused
    tests proving the optional plant shapes delivered thrust and that load torque changes the
    thrust vector while reporting reaction torque. Scenario tests prove the v3 engine
    `tvc_actuator` block parses, v2 rejects it as reserved, and invalid actuator parameters fail
    closed; a runner construction test proves the block enables the optional TVC plant. The TVC
    step and load-torque tests now compare against the provenance-recorded tolerance table.
  - INDI/INCA cancels an injected TWD reaction in the live SIL (effective-authority recovery vs
    no-compensation baseline), tolerance table on the residual angular-accel tracking error.
    **Partial:** a focused unit test cancels a synthetic matched TWD-like angular-acceleration
    reaction through the dense allocator; another focused test shows >90% residual RMS reduction
    on a deterministic slowly varying TWD-like reaction series versus a no-compensation baseline
    and matches the provenance-recorded compensated/uncompensated RMS values. `V-STRUCT-002`
    proves the structural reaction opt-in creates a nonzero attitude-rate history delta in a live
    synthetic TVC probe, and the feature-gated
    `scalar_indi_rate_loop_reduces_live_structural_reaction_residual` runner regression proves
    the scalar INDI runtime loop reduces live reaction-induced body-rate residual RMS below 75%
    of the PID residual. Feature-gated FC/schema/runner construction tests prove
    `autopilot.pseudo_control_cmd` is published/registered, `dense_inca` schema blocks validate,
    the runner builds `DenseIncaControlAllocator`, and the mixer consumes pseudo-control instead
    of semantic actuator demand for that allocator. The runner now accepts physical engine-gimbal
    dense aliases (`<engine_id>.gimbal_pitch` / `<engine_id>.gimbal_yaw`), inherits engine
    gimbal bounds/rate limits for those columns, and merges dense alias commands back into the
    gated `EngineCommandSet` without dropping throttle or ignition state. `MAX_EFFECTOR_COMMANDS`
    is sized for a 9-engine dual-axis TVC cluster, and the mixer now fails closed on effector
    command overflow. The feature-gated
    `dense_inca_runtime_reduces_live_structural_reaction_residual` runner regression allocates
    dense-INCA columns across the powered TVC probe's physical engine gimbals and reduces live
    structural-reaction body-rate residual RMS below 75% of the PID residual. The adjacent
    WP-06.4-c degraded-mode substrate now maps saturated post-availability dense allocation to
    `FDIR_BIT_ALLOCATION_INFEASIBLE` and commander's `health.degraded` region (`REQ-GNC-007`).
    Commander-side confirmation counters now gate degraded and SAFE health-region transitions.
    Full engine-out retrim, SAFE transition validation, and flight-calibrated TVC authority remain
    open.
  - PCH prevents integrator wind-up against a rate-limited actuator (bounded integral state
    under saturation). **Partial:** focused tests prove pseudo-control deficit reporting under a
    rate-limited actuator and bounded hedged integration when requested pseudo-control is fully
    unavailable.
  - all §2 gates green. **Focused status:** `V-GNC-006` propulsion + runner + FC substrate gates
    are green for the feature slice; workspace-wide §2 remains deferred to the next batch
    boundary.
- **validation_label:** `validated-toy`
- **dual_use_note:** far from line (vehicle-intrinsic authority).
- **est_effort:** 2–3 weeks
- **parity_ceiling:** uses published 2nd-order + rate-limit actuator models, not hardware EHA/EMA
  transfer functions.

---

### WP-06.4-a — MEKF attitude with Markley 2023 covariance reset

- **goal:** Promote the 6-state MEKF to a first-class module with the correct multiplicative
  covariance reset (Markley 2023), singularity-free attitude with non-singular 3-parameter
  covariance.
- **fidelity_tier:** T4
- **depends_on:** [WP-06.2-c]
- **new_crates:** none
- **touched:** new `crates/openbmp-fc/src/mekf.rs` (promote from `estimator.rs`),
  `crates/openbmp-fc/src/estimator.rs`.
- **approach:** §3.8 MEKF — state `[a(3), b_g(3)]`, multiplicative reset
  `q⁺ = q ⊗ δq(â)`, reset Jacobian `G(â) ≈ I − ½[â×]`.
- **acceptance:**
  - off by default; goldens byte-identical.
  - reproduces the Markley 2023 reset-Jacobian behavior; cross-checks a brute-force MC of the
    attitude-error distribution; matches analytic TRIAD/QUEST on the static two-vector case to
    `1e-9` (tolerance table).
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** validated on analytic/benchmark cases, not flight star-tracker data.

---

### WP-06.4-b — Tightly-coupled GNSS/INS error-state EKF (raw observables, per-sat gating)

- **goal:** Fuse raw pseudorange/Doppler directly with strapdown INS (17-state, clock states),
  working with `<4` satellites under powered flight; fix the known 6-D-block-gate collapse with
  per-satellite chi-square gating.
- **fidelity_tier:** T4
- **depends_on:** [WP-06.4-a, doc `09` raw-observable generation]
- **new_crates:** none
- **touched:** `crates/openbmp-fc/src/estimator.rs`, `crates/openbmp-fc/src/glrt.rs`
  (per-sat gate), `crates/openbmp-msgs/src/lib.rs` (`GnssObservableSet` topic).
- **approach:** §3.8 tight EKF — Groves error dynamics, raw-observable measurement model,
  scalar-sequential Joseph-form update, per-satellite chi-square gate, elevated powered-flight
  process noise.
- **acceptance:**
  - off by default; goldens byte-identical.
  - vs RTKLIB observables from published broadcast ephemeris (IGS/CDDIS RINEX nav) on a
    synthetic ascent: position/velocity error bounded with `<4` sats; per-satellite gating
    **rejects an injected outlier/spoof while the 6-D block gate does not** (tolerance table +
    the failure-mode regression).
  - deterministic (fixed PRN order, scalar-sequential); fails closed on degenerate geometry.
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line (navigation, not targeting); ephemeris/observables carry no
  aimpoint.
- **est_effort:** 3–4 weeks
- **parity_ceiling:** synthetic observables + RTKLIB models; not a real receiver's tracking-loop
  behavior under jamming.

---

### WP-06.4-c — Engine-out reconfiguration + abort-to-SAFE ladder

- **goal:** On confirmed effector loss, drop the `B` column and re-solve the WLS; raise a
  degraded-mode flag when the attainable-moment set no longer contains the demand; and provide
  a forward-only abort ladder `NOMINAL → DEGRADED → SAFE` commanding thrust-off / attitude-hold
  / propellant-safe.
- **status:** partial (2026-06-26) — the failed-engine TVC-gimbal availability path from
  WP-06.3-b now feeds a degraded-mode substrate: the mixer reports saturated
  `EffectorCommandSet` output when availability removes a dense column and the remaining
  authority cannot attain demand, FDIR latches `FDIR_BIT_ALLOCATION_INFEASIBLE` from that topic,
  and commander maps allocation-infeasible-only trips into `health.degraded` while keeping
  `safe_state_requested` false. Commander-side `fdir_degraded_confirm_ticks` and
  `fdir_safe_confirm_ticks` counters gate degraded and SAFE transitions while defaulting to one
  tick for current behavior (`REQ-GNC-007` / `V-GNC-007`). The mixer now also converts
  `safe_state_requested` into zero-throttle, zero-gimbal, ignite-false, shutdown-true
  `EngineDemand` and per-engine `EngineCommandSet` output even when no fresh engine demand
  arrived in that tick (`REQ-GNC-008` / `V-GNC-008`). The
  `ballistic_state_compile_fail` testkit harness now also rejects a `RequestSafeState`
  construction carrying `ground_aimpoint_eci_m`, keeping the SAFE action vocabulary
  non-targeting (`REQ-GNC-009` / `V-GNC-009`). The shared message dictionary and mixer now
  publish `PropellantSafingCommand` on `propulsion.propellant_safing_cmd`, carrying
  feed-isolation, valve-closure, pressurization-disable, and drain-disable intent through
  FC step output and runner registration (`REQ-GNC-010` / `V-GNC-010`). The runner now consumes
  that latest safing command in point-mass and rigid-body propellant-budget paths, latches
  configured feed networks closed, clears tank drain rates when drains are disabled, and forces
  propellant-budgeted engine feed scales to zero with shutdown requests (`REQ-GNC-011` /
  `V-GNC-011`). The vehicle-side budget report now carries `pressurization_enabled`, defaults it
  true for normal budget evaluation, and the runner SAFE clamp flips it false when active SAFE
  intent disables pressurization (`REQ-GNC-016` / `V-GNC-016`). The autopilot now latches the
  attitude measured at SAFE entry as an internal target-free hold reference, suppresses normal
  guidance/trajectory/load-relief corrections while SAFE is active, exposes
  `AutopilotStatus.safe_attitude_hold_active`, and publishes zero-throttle shutdown demand
  (`REQ-GNC-012` / `V-GNC-012`). A focused commander/autopilot/mixer pipeline test
  proves non-allocation FDIR trips reach SAFE hold, propellant safing, and per-engine shutdown
  commands on one FC bus (`REQ-GNC-013` / `V-GNC-013`). A focused `FcRunner` integration test proves
  those outputs are visible through the runner accessors used by `FcBridge` after a previous-tick
  non-allocation FDIR trip (`REQ-GNC-014` / `V-GNC-014`). A focused rigid-body `Session`
  integration test clips the Phalcon-9 TVC probe, injects a mission `request_safe_state` event,
  verifies pre-SAFE thrust, post-SAFE zero-thrust Shutdown across all nine engines, bounded finite
  rigid-body state, and clean end-time completion (`REQ-GNC-015` / `V-GNC-015`). Failed-engine
  snapshots now also publish failed engine ids through `EffectorAvailability`; the mixer consumes
  that FC-portable engine availability list, commands unavailable engines shutdown/zero, retrims
  scalar throttle over the remaining phase-authorized engines by total-authorized/available, and
  clamps the redistributed throttle to 1.0 (`REQ-GNC-017` / `V-GNC-017`). Scalar fan-out can now
  also apply calibrated per-failed-engine survivor gimbal trim biases from `[[fc.engine_out_trim]]`
  after throttle retrim (`REQ-GNC-022` / `V-GNC-022`). SAFE drain-disable intent
  now also reaches `TankRack`, which latches drain paths disabled and ignores declared, budget, and
  RCS feed drains while inactive safing leaves them enabled (`REQ-GNC-018` / `V-GNC-018`).
  SAFE pressurization-disable intent now also fails all reported engine feed-pressure scales closed
  with shutdown requests even when feed isolation and valve closure are not requested
  (`REQ-GNC-019` / `V-GNC-019`). SAFE now also publishes a distinct `dump_enabled` intent,
  schema-v3 tanks can declare `safe_dump_rate_kg_per_s`, and `TankRack` applies only that
  configured dump rate when SAFE dump is active and ordinary drains are disabled (`REQ-GNC-020` /
  `V-GNC-020`). Tanks can also declare a pressure-coupled `safe_dump_orifice`, and `TankRack`
  adds live ullage-pressure orifice flow to the configured fixed dump rate while SAFE dump is
  active (`REQ-GNC-021` / `V-GNC-021`). Tanks can now also declare a choked
  ullage-gas `safe_dump_vent` that reduces the next-step pressure available to the liquid
  dump orifice when SAFE dump is active (`REQ-GNC-023` / `V-GNC-023`) and can opt into an
  adiabatic ullage-gas thermal model for that vent (`REQ-GNC-025` / `V-GNC-025`). SAFE dump
  orifices and vents can now also declare deterministic backpressure ramps for time-varying
  downstream pressure (`REQ-GNC-026` / `V-GNC-026`), and SAFE dump vents can opt into a
  wall-relaxed adiabatic thermal model with required wall-temperature/tau validation
  (`REQ-GNC-027` / `V-GNC-027`). SAFE dump orifices and vents can also source
  runtime backpressure from the current ambient atmosphere sample
  (`REQ-GNC-028` / `V-GNC-028`). Tanks can also declare an opt-in finite-volume
  pressurant hardware substrate that feeds later pressure-coupled SAFE dump
  orifice flow unless SAFE disables pressurization (`REQ-GNC-029` / `V-GNC-029`).
  Scalar fan-out can now also derive an
  optional `EngineOutTrimTable` from declared engine mount geometry and max thrust via
  `[fc.engine_out_trim_auto]` (`REQ-GNC-024` / `V-GNC-024`). Flight-calibrated integrator bias
  estimation plus cryogenic phase-change, flight-calibrated regulator/bottle
  thermal coupling, plume-coupled vent backpressure, and production pressurant
  hardware validation remain open.
- **fidelity_tier:** T4
- **depends_on:** [WP-06.1, WP-06.3-b]
- **new_crates:** none
- **touched:** `crates/openbmp-fc/src/mixer.rs` (post-availability dense saturation evidence),
  `crates/openbmp-fc/src/fdir.rs` (`FDIR_BIT_ALLOCATION_INFEASIBLE`),
  `crates/openbmp-fc/src/commander.rs` (`health.degraded` mapping), and
  `crates/openbmp-runner/src/fc.rs` (test helper canonical health-region states). The mixer
  also owns the current SAFE thrust-termination command override, and
  `crates/openbmp-testkit/tests/ballistic_state_compile_fail.rs` owns the no-ground-redirect
  compile-fail tripwire. `crates/openbmp-msgs/src/lib.rs`, `crates/openbmp-fc/src/step.rs`,
  and `crates/openbmp-runner/src/fc.rs` carry the current propellant-safing intent topic, while
  `crates/openbmp-runner/src/fc_bridge.rs`, `crates/openbmp-runner/src/feed_network.rs`,
  `crates/openbmp-runner/src/point_mass.rs`, `crates/openbmp-runner/src/propulsion.rs`, and
  `crates/openbmp-runner/src/rigid_body.rs` consume it in runner feed/budget safing paths.
  `crates/openbmp-vehicle/src/propellant_budget.rs` carries the simplified budget-report
  pressurization-enable diagnostic consumed by the runner SAFE clamp.
  `crates/openbmp-msgs/src/lib.rs` and `crates/openbmp-fc/src/autopilot.rs` carry the SAFE
  attitude-hold diagnostic and latched-entry-reference substrate; `crates/openbmp-fc/src/commander.rs`
  now carries the focused FC SAFE pipeline validation; `crates/openbmp-runner/src/fc.rs` carries the
  focused runner-facing SAFE output validation; `crates/openbmp-runner/tests/session_equivalence.rs`
  carries the focused rigid-body mission SAFE shutdown/recovery validation. `crates/openbmp-msgs/src/lib.rs`,
  `crates/openbmp-fc/src/mixer.rs`, and `crates/openbmp-runner/src/fc_bridge.rs` carry the
  FC-portable failed-engine availability and scalar engine-out retrim substrate. `crates/openbmp-runner/src/tanks.rs`,
  `crates/openbmp-runner/src/point_mass.rs`, and `crates/openbmp-runner/src/rigid_body.rs` carry
  the tank-rack SAFE drain-disable latch. `crates/openbmp-runner/src/propulsion.rs` carries the
  simplified pressurization-disable feed fail-closed path. `crates/openbmp-fc/src/mixer.rs`,
  `crates/openbmp-scenario/src/document.rs`, and `crates/openbmp-runner/src/fc.rs` carry the
  calibrated `[[fc.engine_out_trim]]` scalar fan-out trim-bias table and
  `[fc.engine_out_trim_auto]` geometry-derived trim table substrate, while
  `crates/openbmp-runner/src/tanks.rs` and `crates/openbmp-scenario/src/document.rs` carry the
  SAFE dump rate, pressure-coupled liquid orifice, choked ullage-gas vent, opt-in adiabatic and
  wall-relaxed ullage-gas vent thermal substrates, deterministic SAFE dump backpressure ramps,
  ambient-atmosphere backpressure handoff, and finite-volume tank pressurization substrate.
  `crates/openbmp-runner/src/point_mass.rs` and `crates/openbmp-runner/src/rigid_body.rs` now hand
  current ambient atmosphere pressure to `TankRack` only for SAFE dump paths that request ambient
  backpressure. Future completion still needs flight-calibrated integrator bias estimation plus
  cryogenic phase-change, flight-calibrated regulator/bottle thermal coupling, plume-coupled vent
  backpressure, and production pressurant hardware validation.
- **approach:** §3.9 — `moment_demand_attainable`, `B`-column drop + WLS re-solve, re-trim
  bias absorption, `AbortLadder` guarded transition table with confirmation timers.
- **acceptance:**
  - off by default; goldens byte-identical.
  - scripted single-engine-loss injection in the live SIL: WLS redistributes; attitude held
    within margin; if demand becomes unattainable → DEGRADED, and uncontainable → SAFE
    (thrust-off/attitude-hold), with confirmation timers verified. **Partial:** focused FC tests
    prove post-availability dense-allocation saturation is produced when remaining authority is
    insufficient, FDIR latches `FDIR_BIT_ALLOCATION_INFEASIBLE` from that saturation signal, and
    commander publishes `health.degraded` without requesting SAFE for allocation-infeasible-only
    trips; additional commander tests prove two-tick confirmation delays both DEGRADED and SAFE
    transitions until the configured count is met. A focused mixer test proves SAFE status
    publishes thrust-termination engine demand and per-engine shutdown commands without waiting
    for a fresh autopilot engine demand, and now also proves propellant-safing intent is
    published on the FC bus. Focused FC/scenario/runner tests prove `[[fc.engine_out_trim]]`
    applies calibrated survivor gimbal biases after scalar throttle retrim, is v3-gated, rejects
    invalid engine references/self-targeting, and translates scenario ids into the FC table.
    Focused runner tests prove active safing commands close configured
    feed valves, clear budget tank drains, zero propellant-budget feed scales, and request
    engine shutdown while inactive commands leave budget outputs unchanged; the same budget clamp
    now also proves SAFE pressurization-disable intent flips the simplified budget report's
    `pressurization_enabled` flag false while normal budget evaluation defaults it true. Focused
    tank-rack tests prove SAFE dump intent opens configured constant-rate dump paths,
    pressure-coupled ullage liquid-orifice paths, and choked ullage-gas vent paths while ordinary
    drains remain disabled, and focused scenario tests prove `safe_dump_rate_kg_per_s`,
    `safe_dump_orifice`, and `safe_dump_vent` validation. A focused
    autopilot test proves SAFE entry ignores a conflicting normal guidance reference, advertises
    `safe_attitude_hold_active`, emits zero-throttle shutdown demand, and commands correction when
    the vehicle drifts away from the latched entry attitude. A focused FC pipeline test proves a
    non-allocation FDIR trip drives commander SAFE request, autopilot SAFE hold, mixer propellant
    safing, and per-engine shutdown output on a shared bus. A focused `FcRunner` test proves the
    same SAFE status/hold/shutdown/safing outputs are visible through runner accessors after an
    injected previous-tick non-allocation FDIR trip. A focused rigid-body `Session` test proves a
    mission `request_safe_state` event shuts down the live nine-engine TVC cluster to exactly zero
    thrust while the rigid-body state remains valid/bounded and the run reaches end time.
  - abort ladder is monotonic and guarded; **no transition carries a target**; the
    ballistic-state compile-fail tripwire is extended to reject SAFE ground-aimpoint vocabulary
    and green.
  - all §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** **SAFE state only — thrust termination / attitude-hold / propellant-safe;
  never an alternate target or redirect-to-impact.** This is the highest-sensitivity WP — the
  abort ladder has no optimize-to-ground-condition path and the compile-fail test proves it.
- **est_effort:** 2–3 weeks
- **parity_ceiling:** the *method* (guarded FDIR + SAFE abort); not a certified AFTS/range-safety
  result.

---

### WP-06.4-d — Offline H∞/μ-synthesized scheduled controller import

- **status:** partial and traced by `REQ-GNC-030` / `V-GNC-030`,
  `REQ-GNC-031` / `V-GNC-031`, `REQ-GNC-032` / `V-GNC-032`, and
  `REQ-GNC-033` / `V-GNC-033`.
  `openbmp-fc::tables` now exposes `RobustControllerNode` and
  `RobustControllerTable` for provenance-pinned reduced discrete state-space
  controller imports with bounded dimensions, strictly sorted scalar schedule
  nodes, finite row-major `A_c/B_c/C_c/D_c` matrices, SHA-256 source-digest
  validation, target-vocabulary rejection, and two-buffer table activation.
  `openbmp-fc::schedule` now exposes `ImportedRobustController`,
  `RobustControllerMatrices`, `ImportedRobustControllerStep`, and
  `interpolate_robust_controller_table` to clamp/interpolate scheduled matrices,
  validate runtime measurements, compute `u[k] = C_c x_c[k] + D_c y[k]`, and
  advance `x_c[k+1] = A_c x_c[k] + B_c y[k]` in locked row-major order.
  Schema-v3 `[fc.robust_controller]` now declares `iload_file` plus optional
  `iload_file_sha256`, resolves the file through `Scenario::resolved_files`, and
  fails closed on schema-v2 use or SHA mismatch. The runner-side FC bridge now
  passes resolved scenario files into `FcRunner::new_with_resolved_files`; when a
  robust-controller I-load is present, `FcRunner` rechecks the optional SHA pin,
  parses the `[robust_controller]` / `[[robust_controller.nodes]]` TOML envelope,
  converts it into `RobustControllerTable`, validates it, and declares it in the
  FC table registry at boot. `openbmp-fc::schedule` now also exposes
  `SisoStateSpaceRef`, `siso_state_space_loop_margins`, and
  `sweep_imported_robust_controller_siso_margins` for deterministic public/toy
  SISO frequency-response margin sweeps over imported robust-controller schedules,
  backed by `data/gnc/robust-controller-siso-margin-tolerance-v1.toml`. This does
  not yet claim closed-loop robust-controller measurement mapping, command mixing,
  external H∞/μ synthesis reproduction, MIMO μ-margin validation, or real vehicle
  robust-control tuning.
- **goal:** Import an offline (Python/MATLAB) H∞/μ-synthesized, balanced-reduced
  `(A,B,C,D)` controller as a provenance-pinned table I-load and run only the linear controller
  in the loop, for robust stability against bending/slosh/aero/actuator uncertainty.
- **fidelity_tier:** T4
- **depends_on:** [WP-06.2-b, WP-06.2-c]
- **new_crates:** none
- **touched:** `crates/openbmp-fc/src/tables.rs` (`RobustControllerTable`),
  `crates/openbmp-fc/src/schedule.rs` (run the imported state-space).
- **approach:** §1.1 / research §"H∞/μ" — do **not** build the synthesizer in the flight crate;
  import the reduced controller, run `x_c[k+1] = A_c x_c + B_c y`, `u = C_c x_c + D_c y`.
- **acceptance:**
  - off by default; goldens byte-identical.
  - imported controller reproduces the offline tool's closed-loop frequency response within
    tolerance on the open benchmark plant (tolerance table); margins `≥6 dB`/`≥30°` and `μ < 1`
    at nodes.
  - I-load has provenance.md + SHA pin; deterministic state-space iteration.
  - all §2 gates green.
- **validation_label:** `research`
- **dual_use_note:** far from line; controller I-load carries no target — tripwire on the schema.
- **est_effort:** 1–2 weeks
- **parity_ceiling:** reproduces the offline tool's controller on open plants, not a flown AAC
  tuning.

---

## 10. References

**Control allocation**
- O. Härkegård, "Efficient Active Set Algorithms for Solving Constrained Least Squares Problems
  in Aircraft Control Allocation," IEEE CDC 2002, doi:10.1109/CDC.2002.1184694; QCAT toolbox,
  research.harkegard.se/qcat/ (`wls_alloc.m`).
- M. Bodson, "Evaluation of Optimization Methods for Control Allocation," J. Guidance, Control,
  and Dynamics 25(4), 2002.
- W. Durham, K. Bordignon, R. Beck, *Aircraft Control Allocation*, Wiley 2017.

**Coupled LQR/LQG, robust control, scheduling**
- B.D.O. Anderson, J.B. Moore, *Optimal Control: Linear Quadratic Methods*, Dover 2007.
- F.L. Lewis, D. Vrabie, V. Syrmos, *Optimal Control*, 3rd ed., Wiley 2012 (DARE doubling, LTR).
- A. Laub, "A Schur Method for Solving Algebraic Riccati Equations," IEEE TAC 24(6), 1979.
- J. Doyle, G. Stein, "Multivariable Feedback Design," IEEE TAC 26(1), 1981 (LQG/LTR).
- K. Zhou, J. Doyle, K. Glover, *Robust and Optimal Control*, Prentice Hall 1996 (DGKF, D-K).
- J. Doyle, K. Glover, P. Khargonekar, B. Francis, "State-Space Solutions to Standard H2 and
  H-inf Control Problems," IEEE TAC 34(8), 1989.
- P. Apkarian, P. Gahinet, "A Convex Characterization of Gain-Scheduled H-inf Controllers,"
  IEEE TAC 40(5), 1995 (LPV/LMI).
- W. Rugh, J. Shamma, "Research on Gain Scheduling," Automatica 36(10), 2000.
- *dkpy: Robust Control with Structured Uncertainty in Python*, arXiv:2511.13927.

**Launch-vehicle ascent control / load-relief / structural filters**
- J. Orr et al., "Space Launch System Ascent Flight Control Design," AAS 14-038,
  ntrs.nasa.gov/citations/20140008731.
- J. Orr, J. Russell, "A Modern Load Relief Guidance Scheme for Space Launch Vehicles," AAS
  22-111, ntrs.nasa.gov/citations/20220000587.
- C.E. Hall et al., "Design of Launch Vehicle Flight Control Systems Using Ascent Vehicle
  Stability Analysis Tool," ntrs.nasa.gov/citations/20110015701.
- A.L. Greensite, *Analysis and Design of Space Vehicle Flight Control Systems*, Spartan 1970.
- NASA, "Integrated design optimization of structural bending filter and …," arXiv:1802.01875.
- B. Wie, *Space Vehicle Dynamics and Control*, 2nd ed., AIAA 2008.

**TVC / TWD / INDI**
- S. Sieberling, Q.P. Chu, J.A. Mulder, "Robust Flight Control Using Incremental NDI and
  Angular Acceleration Prediction," JGCD 33(6), 2010.
- E. Smeur, Q. Chu, G. de Croon, "Adaptive Incremental NDI for Quadrotors," JGCD 39(3), 2016.
- "Nonlinear Dynamic Inversion with Actuator Dynamics: An Incremental Control Perspective,"
  JGCD, doi:10.2514/1.G007079.

**Navigation / attitude estimation**
- P.D. Groves, *Principles of GNSS, Inertial, and Multisensor Integrated Navigation Systems*,
  2nd ed., Artech House 2013 (ch. 14, tight coupling).
- J. Farrell, *Aided Navigation: GPS with High Rate Sensors*, McGraw-Hill 2008.
- J. Solà, "Quaternion kinematics for the error-state Kalman filter," arXiv:1711.02508.
- F.L. Markley, J.L. Crassidis, *Fundamentals of Spacecraft Attitude Determination and Control*,
  Springer 2014.
- F.L. Markley, J.L. Crassidis, N. Reynolds, "Error-Covariance Reset in the MEKF for Attitude
  Estimation," JGCD 46(10), 2023, doi:10.2514/1.G007653.
- E.J. Lefferts, F.L. Markley, M.D. Shuster, "Kalman Filtering for Spacecraft Attitude
  Estimation," JGCD 5(5), 1982.
- RTKLIB, github.com/tomojitakasu/RTKLIB (BSD-2).

**Redundancy / FDIR**
- R. Isermann, *Fault-Diagnosis Systems*, Springer 2006.
- ESA FTC-CRE, "Fault Tolerant Control for a Cluster of Rocket Engines," ResearchGate 371878930.
- A.S. Willsky, H.L. Jones, "A generalized likelihood ratio approach to the detection and
  estimation of jumps in linear systems," IEEE TAC 21(1), 1976.

**Solvers / numerics**
- Clarabel.rs (Apache-2.0); OSQP + osqp.rs (Apache-2.0); faer-rs / nalgebra; python-control
  (BSD-3) / Slycot (GPL-3.0, offline only); NASA cFS/cFE (Apache-2.0).

*Companion documents:* `00-overview.md` (constitution), `13-agent-execution-playbook.md`
(playbook); upstream anchors `docs/verification.md`, `docs/standards-posture.md`,
`docs/safety-boundaries.md`.
