# Trajectory Optimization & Mission Design

**Status:** `experimental` (WP-07.0 T0 offline two-body and scenario-backed
corrector driver/CLI paths landed; WP-07.1 STM/multiple-shooting substrate is
implemented; WP-07.2 Hermite-Simpson defect/cost transcription, fixed-mesh
residual/objective/path/inequality/Jacobian assembly, MMS order evidence, and
feature-gated dense Clarabel SQP substrate plus public benchmark fixtures, a
public sparse-NLP backend contract/preflight substrate with a bounded dense-SQP
adapter, and an FC
optimized-reference tracking hook are partial; WP-07.3 has a first LGR grid,
multi-segment residual/quadrature/continuity substrate, fixed-time decision
layout decode/evaluate plus boundary/soft-path/quadratic-objective residual and
hard/nonlinear path inequality substrate, finite-difference sparse columnwise
and colored residual-Jacobian, analytic/autodiff residual-Jacobian,
Mayer/Lagrange objective reporting, sparse Gauss-Newton normal-step, sparse
augmented-KKT/LDL step evidence, active hard-path constrained-KKT step evidence,
raw residual-multiplier/costate/Hamiltonian
diagnostics including sparse-KKT step-derived diagnostics, residual-only
sparse-KKT SQP outer-loop scalar/nonlinear hard-path feasibility, optional final
active constrained-KKT report, final-multiplier evidence, and nonlinear
path-inequality Jacobian reporting,
optimized-reference/I-load output, fixed-time dense
QP/SQP/constrained-SQP wrapper evidence, and reference-driven ph-refinement
estimate/apply/loop substrate plus scalar-integrator, brachistochrone transcription and dense-SQP feasibility
recovery, and Bryson-Denham active-arc dense-QP plus sparse constrained-KKT handoff benchmark evidence; WP-07.4 LCvx is satisfied; WP-07.5 has a first
discrete-affine SCvx substrate with virtual-control, trust-region diagnostics,
and state-triggered linear-constraint evidence, and WP-07.6 has a first PMP
Hamiltonian/costate/control-stationarity witness substrate plus scalar dense-QP dual/costate
convention evidence, explicit HS/LGR raw-costate convention adapters,
solver-produced HS scalar-PMP-matched and LGR quadrature-normalized dense-QP
dual handoff into those adapters, sparse augmented-KKT and constrained-KKT PMP
costate/Hamiltonian wiring including public Bryson-Denham active-set diagnostics,
direct constrained-KKT certificate I-load handoff, and LGR quadrature
normalization, residual-only
sparse-KKT SQP scalar/nonlinear hard-path feasibility, optional final active
constrained-KKT report, final-multiplier PMP/KKT evidence, sparse-SQP PMP/KKT
certificate payload round-trips with validated costate-comparison and
control-stationarity summaries, and inline plus scalar-PMP, dense-SQP-certified,
and sparse-SQP-certified file-backed optimized-reference closed-loop runner
scenarios with auditable I-load certificates plus HS/LGR dense-SQP and
sparse-SQP payload-certificate surfaces;
nonlinear 6-DoF/aero closure and the production
closed-loop optimality pipeline remain design intent).
**Audience:** the engineer or LLM agent implementing the `openbmp-trajopt` work
packages, and reviewers checking the forward-only locks.
**One-line summary:** wire the dead differential corrector to the real
propagator, then climb the ladder — multiple shooting (STM Jacobian) →
Hermite-Simpson collocation + SQP on `clarabel` → LGR pseudospectral + ph-mesh →
lossless-convex (LCvx) and successive-convex (SCvx) powered-descent-**to-state**
→ closed-loop wiring with a PMP indirect optimality witness — while the
terminal-condition vocabulary lock and offline/L4 placement keep every solver
**provably unable to express a ground-aimpoint fire-control solution.**

> Prerequisite reading: `00-overview.md` (parity definition, invariants, §6
> escalation) and `13-agent-execution-playbook.md` (template, gate set, backlog
> schema). This is the dimension `00` §6 names as *closest to the dual-use line*;
> the dual-use note (§6 here) is not boilerplate — it is a build constraint.

---

## 1. Parity target & ceiling

### 1.1 Target (capability parity within posture)

Production launch-vehicle/mission-design houses solve the continuous optimal
control problem (OCP)

```
min  J = φ(x(t_f), t_f)            (Mayer)   + ∫ L(x,u,t) dt   (Lagrange)
s.t. ẋ = f(x, u, t, p)            (dynamics)
     g(x, u, t) ≤ 0               (path: q̄, q·α, axial/lateral load, heat rate)
     ψ(x(t_0), x(t_f), t_f) = 0   (boundary: orbital / inertial / Δv conditions)
```

with a small family of mature method classes: **direct transcription**
(collocation: POST2, OTIS, SOS, Copernicus), **pseudospectral collocation**
(GPOPS-II/PSOPT class), **multiple shooting** (the robust variant of shooting
that keeps a high-order integrator in the loop), **indirect / Pontryagin**
methods as an optimality cross-check, and **convex powered-descent guidance**
(LCvx / G-FOLD and SCvx, the Falcon-9-class result). The target for OpenBMP is
*method and architecture* parity: implement these formulations, on the workspace
propagator and the already-vendored `clarabel` conic solver, **scoped to the
closed forward-only terminal-condition vocabulary** (`TerminalCondition` in
`crates/openbmp-physics/src/profile.rs`), with the same V&V methodology (known-
optimal benchmark OCPs, Method of Manufactured Solutions for transcription order
of accuracy, code-to-code against PSOPT/GMAT). The audit verdict for `07` is
**at-parity (method)** in `00` §2.

### 1.2 The ceiling (what an open repo cannot match)

State this in every WP that touches a numeric trajectory claim:

1. **Flight-validated vehicle models.** SpaceX/Rocket Lab tune optimizers against
   proprietary CFD-correlated aero decks, measured engine throttle/Isp maps,
   thrust tail-off, and slosh/structural models, then close the loop against real
   landing-burn telemetry. OpenBMP has no flight heritage; **absolute trajectory
   fidelity is unknowable**, regardless of solver quality. We demonstrate the
   *methods* at research fidelity.
2. **Certified real-time onboard solver.** Falcon-9 runs CVXGEN-generated,
   worst-case-execution-time-bounded, flight-qualified SOCP code on
   radiation-tolerant avionics with formal V&V. An open Rust SOCP (`clarabel`)
   solves the *same problem class* but **cannot claim WCET / flight-cert parity.**
3. **SNOPT.** The production sparse SQP behind OTIS/POST2/Copernicus is
   commercial; we substitute an in-house SQP-on-`clarabel` loop and optionally
   FFI to IPOPT (EPL-1.0, feature-gated) — never a claim of SNOPT parity.
4. **Proprietary reference trajectories.** Actual Falcon/Electron ascent/descent
   I-loads are not public. We validate against *published* benchmark OCPs with
   known optima, not against a fielded vehicle.

**Credible open substitutes (the V&V backbone of §5):** Goddard, brachistochrone,
Bryson-Denham, Acikmese-Ploen G-FOLD, Szmuk 6-DoF as truth cases; MMS for
transcription order; code-to-code against PSOPT/GMAT/Copernicus *published*
geometries; IPOPT/OpEn as the open NLP substitute for SNOPT; `clarabel` as the
open substitute for CVXGEN; and the LOCAL-only scraped-webcast Falcon-9 telemetry
cross-check (per `docs/external-telemetry-validation.md`) for external-validity
gross-error detection **only** — never a flight or targeting input. Every artifact
caps at `validated-toy`, rising to `research` exactly where a published benchmark
with a known optimum anchors it. Nothing here is `flight-qualified`.

### 1.3 Honestly uncertain-convergence items

- **SCvx (T5) convergence on aero-coupled 6-DoF entry** is genuinely uncertain;
  ship it with the diagnostics (‖ν‖ virtual-control magnitude, trust-region ρ,
  iteration trace) exposed, not hidden behind a boolean, and report non-
  convergence honestly (the corrector already models this: `converged: bool` is
  inspected, not asserted).
- **The slosh-coupled upper-stage closure** (`00` §8, risk row 1) is a *control
  co-design* problem; the optimizer ships regardless of whether that specific
  closure lands.

---

## 2. Current state in source

Verified by reading the files below (paths are load-bearing baselines to regress
against):

**`crates/openbmp-trajopt/` — the offline / L4 scaffold.**

- `src/lib.rs` — crate is `#![cfg_attr(not(feature = "std"), no_std)]`,
  `#![forbid(unsafe_code)]`, denies `unwrap/expect/panic`. Depends **only** on
  `openbmp-core`, `openbmp-physics`, `postcard`, `serde`, `thiserror`
  (`Cargo.toml`). It re-exports `TerminalCondition` from `openbmp-physics`. It is
  **not** linked by `openbmp-fc` (FC portability lock holds today).
- `src/corrector.rs` — `DifferentialCorrector` is a complete Gauss-Newton /
  Levenberg-Marquardt shooting solver: `solve<F>(condition, mu, x0, forward_map)`
  where `F: FnMut(&[f64]) -> Result<BallisticState, TrajoptError>`. It builds a
  dense finite-difference Jacobian (`finite_difference_jacobian`, relative step
  `h·(1+|xⱼ|)`), forms normal equations `(JᵀJ + λI) Δx = −Jᵀr`
  (`gauss_newton_step`), solves them by Gaussian elimination with partial
  pivoting (`solve_linear_system`), limits the step norm (`apply_step`), and
  reports `DifferentialCorrection { free_variables, residual, iterations,
  converged }`. **`converged: bool` is honest** — a `false` is "best effort
  within budget", not a solution. **The only callers are in `#[cfg(test)]`**: the
  `forward_map` is an analytic two-body `tangential_state(radius, speed)` closure.
  This is the "unwired" state — a correct solver with no production driver.
- `src/target.rs` — `terminal_residual(condition, state, mu)` reduces a
  `BallisticState` to classical elements (energy → `a`, eccentricity vector → `e`,
  `acos(h_z/|h|)` → `i`, apogee radius, flight-path angle) and returns the
  residual vector + RSS norm **relative to orbital / inertial conditions only**.
  `MaximizePayloadMass` returns a zero residual (it is an objective, not a
  constraint), and `solve()` rejects it explicitly.
- `src/iload.rs` — versioned postcard I-load producer:
  `ILoadPayload { header, metadata, event_bindings_postcard, gain_tables,
  reference_profile, optimality_certificate }`, `ILoadSchemaVersion {1,3,0}`,
  `TerminalConditionKind`, `ILoadObjectiveClass`, `ILoadOptimalityEvidenceKind`,
  and `ILoadOptimalityCertificate` (the recorded terminal/objective classes plus
  optional PMP/oracle optimality evidence for audit),
  `encode_iload_payload` validates then `postcard::to_allocvec`. `TrajoptError`
  covers `InvalidPayload`, `Serialize`, `SingularSystem`.
- `src/driver.rs` — a T0 offline driver,
  `correct_two_body_apogee`, now gives `DifferentialCorrector::solve` a
  non-test caller. It varies a single tangential cutoff-speed variable, uses a
  deterministic two-body RK4 forward map, targets
  `TerminalCondition::ApogeeRadius`, wraps the terminal state with
  `BallisticState::from_forward_simulation` provenance, and emits a postcard
  I-load only when `converged` is true. The CLI path is
  `openbmp trajopt correct-apogee --target-apogee-radius-m ... --output-iload ...`.
  `openbmp trajopt correct-apogee-scenario` now runs a real scenario through
  `openbmp-runner`, extracts terminal ECI state from runner telemetry, and
  scales the scenario's initial velocity direction as the single free variable.
  The reproducible fixture lives under `scenarios/trajopt-two-body-apogee/`
  with provenance and a tolerance table. This is traced by `REQ-TRAJOPT-001`;
  it is still a toy single-shooting slice, not multiple shooting, collocation,
  or powered-descent optimization.
- `src/stm.rs` — the first T1 substrate: a deterministic two-body Cartesian
  variational propagator that integrates state plus 6x6 STM with fixed-step RK4
  and validates the STM against central-difference and complex-step columns. It
  is a sensitivity path only; it adds no target vocabulary.
- `src/shooting.rs` — an M-segment multiple-shooting continuity evaluator. It
  seeds dynamically consistent two-body nodes, reports stacked defects
  `phi_i(x_i) - x_{i+1}`, builds the row-major block-bidiagonal Jacobian
  `[STM_i, -I]`, and now also reports duration-sensitivity columns
  `dc_i/dt_i = f(phi_i)`. `MultipleShootingCorrector` adds the first solve loop
  by holding endpoints fixed and applying damped Gauss-Newton corrections to
  interior nodes through that STM Jacobian; it also supports fixed-initial-state
  terminal-condition correction using the closed `TerminalCondition` equality
  vocabulary, including a free-duration variant that solves downstream node
  states and segment times together, plus a controlled variant that solves
  downstream nodes and piecewise-constant ECI acceleration controls together.
  `MultipleShootingSoftConstraint` appends signed exterior-penalty rows for
  Cartesian state-component boxes, radius/speed norm path limits, rotating-
  atmosphere dynamic pressure, and qbar-alpha using a declared body-forward axis.
  This is traced by `REQ-TRAJOPT-002`, including a downstream compile-fail
  tripwire that refuses surface-coordinate fields on the multiple-shooting node
  surface, plus
  `scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml`, which
  gates the T1 multiple-shooting result against the T0 apogee solution.

**`crates/openbmp-physics/src/profile.rs` — the vocabulary + propagator pieces.**

- `enum TerminalCondition` (line 1442) — the **closed forward-only vocabulary**:
  `OrbitalElements{a,e,i}`, `ApogeeRadius{radius_m}`,
  `FlightPathAngleAtBurnout{angle_rad}`, `RendezvousState{position_eci_m,
  velocity_eci_m_s}` (an orbital/spacecraft state, *not* a surface coordinate —
  the doc comment says so), `MaximizePayloadMass`. **There is no range, aimpoint,
  lat/lon, or target-coordinate variant. This is the lock.**
- `BallisticState::from_forward_simulation(...)` + `ForwardSimulationProvenance`
  (lines 1242–1359) — footprint/ballistic states are constructible only with a
  provenance token; bare literals are refused downstream
  (`ballistic_state_compile_fail.rs`).
- `trait AscentReferenceGenerator` (line 152) — `ascent_reference(state, time) ->
  AscentReference`, with `PitchProgramAscentReference`, `GravityTurnAscentReference`,
  `ClosedLoopInsertionAscentReference` (a PEG-style closed-loop insertion to a
  `target_radius_m`/orbital plane, with `time_to_go_s()`), `PegAscentReference`,
  `SequencedAscentReference`. `AscentState` carries position/velocity/altitude/
  q̄/mass-fraction/sensed-thrust-accel. `rk4_gravity_step` (line 3140) +
  `rk4_drag_wind_step` (line 3561) are the in-crate fixed-step integrators.
- `crates/openbmp-runner/src/lib.rs::run(scenario) -> RunOutcome` (line 270) is
  the workspace forward-propagation entry point the offline driver wires as the
  shooting `forward_map`.

**`crates/openbmp-fc/` — the convex-solver surface (the LCvx/SCvx backend).**

- `src/landing.rs` — `solve_accel_norm_epigraph` builds a Clarabel SOCP
  (`ZeroConeT(3)` for `a = desired`, `SecondOrderConeT(4)` for `‖a‖ ≤ t`,
  `minimize t`) and returns `LandingSocpCommand`. The module docstring already
  says "Full `LCvxLD` / `SCvx` trajectory generation is built from this primitive
  by constraining thrust vectors and glideslope cones over a finite horizon" —
  i.e. the cone surface for T4/T5 exists, the discretized SOCP does not.
- `src/mpc.rs::deterministic_settings()` — pinned Clarabel settings
  (`max_iter=100`, all tolerances `1e-9`, `verbose=false`) — the determinism
  anchor reused by every conic solve.
- `src/guidance.rs`, `src/autopilot.rs` — the three-loop autopilot tracks a
  `ReferenceState`; this is the consumer the optimizer output feeds in T6.

**`clarabel`** is already a workspace dependency (`openbmp-fc` `mpc.rs`/
`landing.rs`), Apache-2.0, with deterministic settings wired.

**Summary maturity:** WP-07.0 is implemented: the differential corrector now has
T0 offline two-body and scenario-backed apogee CLI I-load paths; WP-07.1 is
implemented with STM/complex-step verification, multiple-shooting continuity,
duration and control sensitivity checks, fixed-endpoint and fixed-initial
terminal-condition correction, soft node/path penalties including
qbar/q-alpha, free-duration and controlled terminal correction, a T0/T1
cross-tier tolerance table, and a no-surface-coordinate compile-fail tripwire.
WP-07.2 is partial through `collocation.rs` and feature-gated `sqp.rs`: it
exposes Hermite-Simpson segment and fixed-mesh transcription defect/cost
evaluation, a fixed-mesh decision layout with boundary residual assembly, soft
exterior-penalty path residuals, dense hard scalar path inequality rows,
hard scalar path decision-bound merges for the dense SQP lane,
nonlinear vector hard path inequality rows with sparse CSC finite-difference
Jacobian reports, Simpson-weighted quadratic control objective residuals,
deterministic columnwise and colored central-finite-difference CSC sparse
residual Jacobian reports, caller-supplied analytic and forward-mode autodiff
CSC sparse residual Jacobian reports, sparse Gauss-Newton/KKT normal-block
assembly plus sparse normal-equation RHS/matvec/conjugate-gradient solve
evidence, sparse augmented-KKT assembly with deterministic LDL solve evidence,
active scalar hard-path constrained sparse-KKT step evidence with active
inequality multipliers and constrained sparse-KKT diagnostic handoff evidence,
KKT residual-multiplier row-family slicing plus raw node-costate estimates for
later PMP witnesses, raw Hamiltonian diagnostics, and fixed-mesh sparse
Gauss-Newton/KKT step reports plus residual-only sparse-KKT SQP
scalar/nonlinear hard-path feasibility enforcement and final-multiplier reports, a scalar sine MMS
order tolerance table, a validated-toy scalar integrator minimum-energy
benchmark, a public closed-form brachistochrone
transcription/cost benchmark, a public Bryson-Denham state-constrained
fixed-mesh dense-QP benchmark, a validated-toy T1/T2 shared-ascent
Hermite-Simpson mesh cross-check, and a deterministic Clarabel dense-QP lane
plus a bounded nonlinear least-squares SQP driver with a fixed-mesh solve
wrapper, constrained dense-SQP nonlinear hard-path consumption, optimized
reference-profile/I-load emission, finite-difference Jacobians, fixed-mesh
repeat-determinism evidence, merit backtracking, and an FC-native optimized
reference-profile guidance hook that revalidates/interpolates the same
radius/speed/FPA shape and drives the existing autopilot trajectory loop through
a focused tracking test. The closed
terminal-condition vocabulary also feeds a public sparse-NLP backend contract in
`nlp.rs` (`ProblemDims`, `DecisionLayout`, `SparseVec`, `CscMatrix`,
`NlpResidual`, `NlpAssemblyReport`, `NlpSolveValidationSettings`,
`NlpCheckedSolveReport`, `NlpSolution`, `OptimalControlProblem`,
`Transcription`, `NlpSolver`, `assemble_sparse_nlp`, and checked solve wrappers),
plus a feature-gated `DenseSqpNlpSolver` / `DenseSqpNlpSettings` adapter that
uses the deterministic Clarabel dense-QP lane for bounded first-order SQP steps
through the same checked solve wrapper, with fail-closed CSC, objective-gradient,
residual/Jacobian, inequality-bound, closed `TerminalCondition`, positive-`mu`,
backend output-shape, final assembly, and reported-objective consistency
validation traced by `REQ-TRAJOPT-010`.
The closed
terminal-condition vocabulary still holds; a Clarabel SOCP epigraph smoke test,
a forward propagator, and a PEG-style closed-loop ascent reference exist. Still
missing: a production constrained large-sparse NLP backend adapter,
production sparse KKT factorization,
full public benchmark nonlinear-solver closure, general LGR nonlinear/Goddard
benchmark closure,
LCvx/SCvx horizon problem, production PMP/I-load optimality closure, and
indirect cross-check.

---

## 3. Target architecture

### 3.1 Crate strategy

Keep everything **offline / host-side / L4 in `openbmp-trajopt`.** This is a
hard dual-use placement lock (§6). `openbmp-fc` must never gain an edge to
`openbmp-trajopt` (FC portability lock, `00` §3.3; enforced by
`fc_dependency_tripwire.rs`). The convex cone *primitives* live in `openbmp-fc`
(`landing.rs`/`mpc.rs`) because they are FC-portable and already there; the
*horizon assembly* (building the discretized SOCP/SQP matrices) lives in
`openbmp-trajopt`, calling the small portable cone-solve surface implemented in
WP-07.4-a:

- **Implemented:** `openbmp-convex` is the L2 portable home for shared
  deterministic Clarabel settings and CSC row-building helpers. `openbmp-fc`
  reaches it only under `mpc`; `openbmp-trajopt` reaches it only under `sqp`.
  Both crates still depend on `clarabel` directly for their solver calls.
- **Avoid:** making `openbmp-trajopt` depend on `openbmp-fc`. Do not; lift the
  shared `deterministic_settings()` into a portable home (e.g.
  `openbmp-models` or a tiny `openbmp-convex` portable crate) if reuse is needed,
  reviewed in the WP that first needs it.

Shared portable crate:

| Crate | Layer | Purpose | Doc gate |
|---|---|---|---|
| `openbmp-convex` | L2 (portable) | shared deterministic conic-solve settings + CSC builders reused by `openbmp-fc` (FC-portable) and `openbmp-trajopt` (host) | WP-07.4-a |

`Cargo.toml` additions: `openbmp-convex` as the shared portable helper crate,
`clarabel` (already vendored), `nalgebra` (already a workspace dep — STMs,
quaternions, dense LA), optionally `argmin` (MIT/Apache — line-search, trust-
region, BFGS building blocks), `num-dual` (MIT/Apache — forward-mode AD for
sparse Jacobians), and a **feature-gated, off-by-default** `ipopt-sys` FFI for
the large-sparse-NLP backend (EPL-1.0 license boundary documented; never folded
into the Apache/MIT crates statically).

### 3.2 Core trait surfaces

The corrector's `forward_map` closure pattern generalizes cleanly. Define an OCP
abstraction that all methods consume, and a transcription that lowers it to a
sparse NLP.

```rust
/// A forward-only optimal-control problem. Boundary conditions and the
/// objective draw ONLY from the closed `TerminalCondition` vocabulary
/// and vehicle-intrinsic objectives; there is no surface-coordinate
/// field, by construction (see §6). (`openbmp-trajopt`.)
pub trait OptimalControlProblem {
    /// State dimension n_x and control dimension n_u.
    fn dims(&self) -> ProblemDims;

    /// Right-hand side ẋ = f(x, u, t, p). Reuses the workspace
    /// propagator's force/moment models so optimizer and simulator
    /// share one dynamics definition.
    fn dynamics(&self, x: &[f64], u: &[f64], t: f64) -> Result<Vec<f64>, TrajoptError>;

    /// Lagrange integrand L(x, u, t) (vehicle-intrinsic: fuel, time,
    /// energy, Δv). Returns 0.0 for pure-feasibility problems.
    fn running_cost(&self, x: &[f64], u: &[f64], t: f64) -> f64;

    /// Mayer term φ(x_f, t_f) — e.g. −m_f for min-fuel, t_f for min-time.
    fn terminal_cost(&self, x_f: &[f64], t_f: f64) -> f64;

    /// Path inequality g(x,u,t) ≤ 0 (q̄, q·α, axial/lateral load,
    /// heat-rate). Empty for unconstrained arcs.
    fn path_constraints(&self, x: &[f64], u: &[f64], t: f64) -> Vec<f64>;

    /// The closed forward-only boundary condition. Residual is to an
    /// orbital / inertial / vehicle-intrinsic / site-RADIUS condition,
    /// never a surface aimpoint.
    fn terminal_condition(&self) -> &TerminalCondition;

    /// Central-body μ used to reduce terminal states to elements.
    fn mu_m3_s2(&self) -> f64;
}

/// Lowers an OCP onto a mesh into a sparse NLP. Implemented by the
/// transcription tiers: MultipleShooting (T1), HermiteSimpson (T2),
/// LegendreGaussRadau (T3).
pub trait Transcription {
    /// Decision-vector layout (states, controls, midpoints, t_f, p).
    fn layout(&self) -> DecisionLayout;

    /// Assemble equality constraints c(z) = 0 (defects + boundary) and
    /// inequalities d_L ≤ d(z) ≤ d_U (path) with their sparse Jacobian.
    fn assemble(
        &self,
        problem: &dyn OptimalControlProblem,
        z: &[f64],
    ) -> Result<NlpResidual, TrajoptError>;

    /// Objective F(z) and its (sparse) gradient.
    fn objective(&self, problem: &dyn OptimalControlProblem, z: &[f64])
        -> Result<(f64, SparseVec), TrajoptError>;
}

/// Sparse NLP backend. The in-house SQP-on-clarabel implements this;
/// the feature-gated IpoptBackend also implements it.
pub trait NlpSolver {
    fn solve(
        &self,
        transcription: &dyn Transcription,
        problem: &dyn OptimalControlProblem,
        z0: &[f64],
    ) -> Result<NlpSolution, TrajoptError>;
}
```

Data structures:

```rust
pub struct ProblemDims { pub n_x: usize, pub n_u: usize, pub n_path: usize }

/// Block-banded sparse Jacobian in CSC, the shape every transcription
/// produces and every NLP backend consumes.
pub struct NlpResidual {
    pub equalities: SparseVec,        // c(z): defects + Hermite + boundary
    pub inequalities: SparseVec,      // d(z): path constraints
    pub jacobian: CscMatrix<f64>,     // block-banded (HS) / bidiagonal (shooting)
}

pub struct NlpSolution {
    pub z: Vec<f64>,
    pub multipliers: Vec<f64>,        // KKT λ — costate estimate for the indirect cross-check
    pub objective: f64,
    pub kkt_residual: f64,
    pub converged: bool,              // honest, like DifferentialCorrection::converged
    pub iterations: usize,
}
```

The multiple-shooting tier is the **minimal generalization of the existing
corrector**: the free vector becomes `[s_0 .. s_{M-1}, controls, t_f]`, the
residual stacks continuity defects, and the dense FD Jacobian becomes a block-
bidiagonal sparse one. The corrector's `solve_linear_system` is replaced by the
sparse KKT factorization but the Gauss-Newton/LM logic carries over verbatim.

### 3.3 The math, by tier

The fidelity ladder mirrors the research pack; each tier is independently
shippable (`00` §9, `13` §1).

**T0 — Wire the corrector (no new math).** Give `DifferentialCorrector::solve`
a production caller: an offline driver/CLI subcommand whose `forward_map`
integrates the workspace propagator (`openbmp-runner::run` over a parameterized
scenario, or a thin `rk4_gravity_step`-driven shooting loop) and reduces the
terminal `BallisticState` to elements. Targets, e.g., circular LEO insertion
(`TerminalCondition::OrbitalElements`), emits a postcard I-load via `iload.rs`,
and reports `converged`/`residual` honestly. **Deliverable:** `solve()` has a
non-test caller; one reproducible scenario produces a reference trajectory.

**T1 — Multiple shooting with STM / variational Jacobian.** Segments
`[t_i, t_{i+1}]`, decision vars `s_i` (segment-initial state) + controls
(piecewise-const/linear/spline) + `t_f`. Let `φ(t_{i+1}; t_i, s_i, u_i)` be the
flow of `ẋ = f` integrated from `s_i`. **Continuity defects**

```
c_i = φ(t_{i+1}; t_i, s_i, u_i) − s_{i+1} = 0,   i = 0..M−1
```

Boundary: `terminal_residual(condition, φ(t_f), μ) = 0`. Sensitivities
`∂c_i/∂s_i` via the **variational equation** for the state-transition matrix:

```
Φ̇(t) = (∂f/∂x)(t) · Φ(t),   Φ(t_i) = I,
```

integrated alongside the state (augmented RK4), giving `∂φ(t_{i+1})/∂s_i =
Φ(t_{i+1})`. If `∂f/∂x` is awkward to derive, use **complex-step**
differentiation (`f(x + ih eⱼ)` → `Im/h`, machine-precision, no subtractive
cancellation) per column. The NLP Jacobian is **block-bidiagonal**; the same
sparse SQP/Gauss-Newton backend handles it. Add box bounds and simple path
constraints (q̄, q·α) as penalty or active-set. *References:* Bock & Plitt 1984;
Betts 2010 §4.5; Stoer & Bulirsch Ch. 7.

**T2 — Hermite-Simpson direct collocation + SQP on `clarabel`.** Mesh
`t_0<…<t_N`, `h_k = t_{k+1}−t_k`, `f_k = f(x_k, u_k)`. Separated HS adds midpoint
state/control as variables. **Hermite midpoint constraint**:

```
x_{k+1/2} = ½(x_k + x_{k+1}) + (h_k/8)(f_k − f_{k+1})
```

**HS defect (Simpson collocation):**

```
ζ_k = x_{k+1} − x_k − (h_k/6)(f_k + 4 f_{k+1/2} + f_{k+1}) = 0
```

with `f_{k+1/2} = f(x_{k+1/2}, u_{k+1/2}, t_{k+1/2})`. **Lagrange cost** by
Simpson: `J_L = Σ (h_k/6)(L_k + 4 L_{k+1/2} + L_{k+1})`. Decision vector
`z = [x_0..x_N, x_{1/2}.., u_0..u_N, u_{1/2}.., t_f, p]`. NLP:
`min F(z) s.t. c(z)=0 (defects+Hermite+boundary), d_L ≤ d(z) ≤ d_U (path)`. The
Jacobian is **block-banded** (each defect touches only nodes `k, k+1/2, k+1`) —
exploit with sparse linear algebra; build it by forward-mode AD (`num-dual`) or
sparsity-colored finite differences. **Solve via an SQP loop:** at iterate `z`,
form the QP

```
min_Δz  ½ Δzᵀ H Δz + ∇Fᵀ Δz   s.t.  A Δz + c = 0,   d_L ≤ D Δz + d ≤ d_U,
```

with `H` a BFGS or Gauss-Newton Hessian, solved by `clarabel` (the QP is a conic
problem); line-search on a merit function; KKT system
`[H Aᵀ; A 0][Δz; Δλ] = −[∇L; c]`. **Scale variables to O(1)** (canonical DU/TU)
— Betts emphasizes scaling dominates convergence. HS is **O(h⁴)** accurate;
verify that order by MMS (§5). Optional feature-gated IPOPT backend for large
sparse problems. *References:* Hargraves & Paris 1987 (original HS); Betts 2010
Ch. 4 (transcription) + Ch. 1–2 (sparse SQP/IP); Kelly 2017 (HS derivation +
pseudocode); Rao 2009 (survey).

**T3 — LGR pseudospectral + ph-mesh refinement.** On each interval map
`t → τ ∈ [−1,+1)`. Collocate at `N` Legendre-Gauss-Radau points (roots of
`P_{N−1}+P_N`; include −1, exclude +1) via Golub-Welsch (eigenvalues of the
Jacobi matrix) or Newton on `P_N`. Approximate the state by a Lagrange basis on
`N+1` points; the **integral (better-conditioned) form** is

```
x_{N+1} − x_0 = Σ_i w_i (h/2) f_i,
```

with LGR quadrature weights `w_i`; equivalently the differential form
`Σ_j D_{ij} x_j = (h/2) f(x_i, u_i)` with the `N×(N+1)` LGR **differentiation
matrix** `D` from barycentric Lagrange weights. Same NLP shape, same backend as
T2. **ph-mesh refinement (Patterson-Hager-Rao):** per interval `k` compute a
relative error `e_k = max |x_LGR-reconstructed − x_poly| / (1 + max|x|)`. If
`e_k > tol`: estimate the polynomial degree needed; if `≤ N_max`, **raise degree
p** (spectral, exponential payoff on smooth arcs); else **subdivide (h)** (for
nonsmooth / bang-bang arcs, e.g. staging). Iterate transcribe → solve → estimate
→ refine until all `e_k ≤ tol`. **Costate estimate** via the KKT multipliers
(covector mapping) — the optimality witness feeding T6. *References:* Patterson &
Rao 2014 (GPOPS-II, ACM TOMS); Garg et al. 2010 (unified framework); Patterson,
Hager, Rao 2015 (ph-refinement); Darby, Hager, Rao 2011.

**T4 — LCvx min-fuel powered descent/ascent to a STATE (Acikmese-Ploen).** The
Falcon-9-class result: a min-fuel problem with a **non-convex** lower thrust
bound `T_min ≤ ‖T‖ ≤ T_max` (engine cannot throttle to zero) plus a thrust-
pointing cone is **provably equivalent to a convex SOCP** after a lossless
slack relaxation. State `[r; v]`, mass `m`, log-mass `z = ln m`. Dynamics
`ṙ = v`, `v̇ = g + T/m`, `ṁ = −‖T‖/(Isp·g0)`. **Change of variables**:
`u = T/m`, `σ = ‖T‖/m` (slack), `α = 1/(Isp·g0)`, so `v̇ = g + u`, `ż = −α σ`.
**Relaxed constraints:**

```
‖u‖ ≤ σ                              (SOC — lossless: optimal makes it tight)
μ0(1 − (z−z0) + ½(z−z0)²) ≤ σ ≤ μ1(1 − (z−z0))   (2nd-order Taylor of thrust bounds about z0)
n̂ᵀ u ≥ cos(θ_p) · σ                 (pointing cone, SOC)
‖(r − r_f)_horiz‖ ≤ tan(γ) · (r − r_f)_vert   (glideslope cone, SOC)
```

with `z0 = ln(m_wet − α T_max t)`, `μ0 = T_min e^{−z0}`, `μ1 = T_max e^{−z0}`.
Discretize (FOH/ZOH over `N` nodes) → a **single SOCP**: `min −z_N` (max final
mass) s.t. linear dynamics + the SOC cones + boundary `(r_f, v_f)` at the **site
RADIUS / altitude / state** — *never a ground aimpoint* (§6). Solve once with
`clarabel`, reusing `deterministic_settings()`. Free final time `t_f` is an
**outer golden-section line search** (the SOCP is solved per `t_f`). The G-FOLD
two-phase (min-error feasibility, then min-fuel) targets a **state/altitude/
divert-feasible region**, gated so the "target" is a radius/altitude, not a
fire-control coordinate. *References:* Acikmese & Ploen 2007 (JGCD); Acikmese,
Carson, Blackmore 2013 (IEEE TCST, lossless convexification); Blackmore 2016.
Build directly on `landing.rs`'s existing `SecondOrderConeT` surface.

**T5 — SCvx nonconvex (Mao/Szmuk/Acikmese).** When dynamics are genuinely
nonlinear (6-DoF attitude, mass-varying inertia, aerodynamics) so LCvx does not
apply, solve the nonconvex OCP as a **sequence of convex SOCP subproblems** by
successive linearization about the previous iterate, with a trust region and
virtual control. At iterate `k`, FOH-linearize:

```
x_{i+1} = A_i x_i + B_i⁻ u_i + B_i⁺ u_{i+1} + S_i σ + R_i + ν_i,
```

`A_i = ∂f/∂x`, `B = ∂f/∂u` from the reference (discretized FOH via an augmented
STM / matrix-exponential sub-interval integration), `σ` = time-dilation (free
final time), `ν_i` = **virtual control** (penalized with weight `w_ν` to keep
subproblems feasible). **Trust region:** `‖x − x_ref‖ ≤ η`, `‖u − u_ref‖ ≤ η`
(or soft quadratic-penalty TR). Convex subproblem (SOCP):

```
min  [original convex cost + w_ν ‖ν‖₁ + w_tr ‖δ‖]
s.t. linearized dynamics + convex constraints + linearized nonconvex constraints + TR,
```

solved with `clarabel`. **Update:** compute the actual-vs-predicted reduction
ratio `ρ = (J(x_k) − J(x_{k+1})) / (J(x_k) − L(x_{k+1}))`; if `ρ > ρ1` accept and
**grow** the trust region, else shrink and reject (classic TR management).
Iterate until `‖ν‖ → 0` (dynamic feasibility recovered) and the step `< tol`.
**State-triggered constraints** (e.g. velocity-trigger AoA limits for entry):
`g(x) ≤ 0` enforced only when trigger `h(x) < 0`, encoded continuously as
`min(−h, 0)·g ≤ 0`, linearized. Provable superlinear local rate, global to a
KKT/stationary point. 6-DoF uses quaternion kinematics (`nalgebra`) and
inertia(mass) coupling. *References:* Mao, Szmuk, Acikmese 2016 (CDC,
arXiv:1608.05133) + 2018 (arXiv:1804.06539, superlinear); Szmuk & Acikmese 2018
(AIAA 2018-0617, 6-DoF free-final-time); Szmuk, Reynolds, Acikmese (JGCD 2020,
state-triggered, arXiv:1811.10803).

**T6 — Closed-loop wiring + PMP indirect cross-check.** (a) Feed optimizer output
(reference trajectory + feedforward steering) into the FC autopilot/MPC as a
**tracked `ReferenceState`** — crossing the FC boundary as a **pre-validated
postcard I-load** the FC re-validates, *never as a live solver computing a
target inside the flight loop* (§6, placement lock). (b) Add the **Pontryagin
Minimum Principle** verifier as a CI optimality witness, not as a production
solver. Hamiltonian `H = L + λᵀ f`; costate `λ̇ = −∂H/∂x`; optimal control
`u* = argmin_u H`. For control-affine `f = a(x) + B(x)u`, `‖u‖ ≤ u_max`, this
gives the **primer-vector** bang-bang `u* = −u_max · Bᵀλ/‖Bᵀλ‖`; transversality
`λ(t_f) = (∂φ/∂x_f) ν`, and `H(t_f) = 0` for free `t_f` (autonomous). For vacuum
ascent the primer satisfies the **bilinear-tangent steering law**
`tan χ = (c1 t + c2)/(c3 t + c4)`. The costate two-point BVP is solved by
**shooting on `λ(t_0)`** — the *same Gauss-Newton machinery as the corrector*,
applied to the costate residual at `t_f`. The witness: `H` constant on autonomous
arcs, costates matching the KKT-multiplier estimate from T2/T3 within mesh
tolerance → the team claims **"verified optimal", not just "NLP converged".**
*References:* Bryson & Ho 1975; Lawden 1963 (primer vector); Longuski et al.
2014; Betts 2010 §4.11 (costate from collocation multipliers).

### 3.4 Fidelity tiers (the ladder)

| Tier | Capability | Earns (max) | Effort |
|---|---|---|---|
| **T0** | Wire the corrector: offline driver/CLI, real propagator `forward_map`, LEO insertion, I-load emit | `validated-toy` | ~1 wk |
| **T1** | Multiple shooting; STM/variational (or complex-step) Jacobian; block-bidiagonal defects; box/path penalties | `validated-toy` | ~2–3 wk |
| **T2** | Hermite-Simpson collocation; sparse SQP on `clarabel` (opt. IPOPT FFI); Mayer+Lagrange; full path constraints | `research` (MMS O(h⁴) + benchmark) | ~4–6 wk |
| **T3** | LGR pseudospectral + ph-mesh; costate estimate from KKT | `research` (Goddard/benchmarks) | ~4–6 wk on T2 |
| **T4** | LCvx min-fuel powered-descent-**to-state** SOCP; glideslope+pointing cones; FOH; golden-section t_f | `research` (Acikmese-Ploen G-FOLD) | ~2–3 wk (parallel to T2/T3) |
| **T5** | SCvx nonconvex 6-DoF/aero descent+entry; virtual control + TR; state-triggered constraints | `research` (Szmuk 6-DoF) where it converges, else `experimental` | ~4–8 wk on T4 |
| **T6** | Closed-loop wiring (optimizer → FC tracked reference via I-load); PMP costate-shooting CI witness | `validated-toy` (pipeline), `research` (optimality regression) | continuous |

---

## 4. Invariant preservation (concretely for this dimension)

- **Byte-determinism (`00` §3.1).** All solver internals advance by locked-
  operand-order weighted sums; never `f64::mul_add`, never wall-clock, never
  system RNG, never unordered iteration over a `HashMap`. `clarabel` uses
  `deterministic_settings()` (pinned `max_iter`, all tolerances `1e-9`,
  `verbose=false`) — the same anchor `mpc.rs` uses. SQP/LM damping, golden-section
  brackets, and trust-region schedules are deterministic functions of the inputs.
  Any stochastic synthesis substep (e.g. multi-start initial guesses) draws from
  `openbmp-core::DeterministicRng` with a domain-separated `(seed, step, id,
  component)` tuple, recorded in `ILoadHeader::synthesis_seed`. The emitted
  postcard I-load is byte-stable (`postcard` is already the stable wire format in
  `iload.rs`; the `iload_payload_encodes_as_postcard` round-trip test guards it).
- **Byte-stable-by-default (`00` §3.2).** Everything here is **offline / L4** and
  emits artifacts (I-loads, reference trajectories); it does not alter any live
  scenario kernel path, so all existing scenario goldens stay byte-identical with
  zero opt-in. When T6 feeds a reference into the FC, the consuming scenario gates
  it behind an explicit config block (the established `[vehicle.bending]`-style
  pattern); existing scenarios that do not opt in remain byte-identical.
- **FC hardware-portability lock (`00` §3.3).** `openbmp-fc` must never depend on
  `openbmp-trajopt`. The optimizer is host-side; its output crosses to the FC
  only as a **validated postcard payload** the FC re-validates — the same one-way
  boundary the I-load schema already encodes. `fc_dependency_tripwire.rs` stays
  green; do not add a `trajopt` edge to `openbmp-fc` for the T4/T5 cone reuse —
  share the cone *primitive direction* (FC → portable home), never the reverse.
- **Provenance (`00` §3.5).** I-loads carry `ILoadHeader { schema_version,
  terminal_condition_kind, objective_class, synthesis_seed, producer }` +
  `SynthesisMetadata { scenario_digest, source_revision, method }`, with an
  optional fail-closed `ILoadOptimalityCertificate` for file-backed optimality
  evidence. The terminal-condition/objective-class audit fields and optional
  certificate let reviewers confirm no aimpoint leaked in (§6.5) and that any
  attached PMP/oracle evidence satisfies its declared tolerances; broader
  closed-loop consumption remains WP-07.6 work. Any benchmark constants (Goddard parameters,
  Acikmese-Ploen case parameters) live only in their declared source-of-truth
  data file with a sibling `provenance.md` + SHA-256 pin; no inline data in
  `*.rs` (`inline_data_tripwire.rs`). **No literal WGS84 `GM_earth`, `J2`,
  `R_earth` numerals anywhere in this doc or new code** — symbolic references
  only (`WGS84_MU_M3_S2`, `WGS84_A_M` are the allow-listed constants).
- **Validation labels (`00` §3.8).** Each tier declares exactly one of
  `experimental → checked → validated-toy → research`, justified by §5 evidence,
  with a tolerance-table TOML for every numeric claim. No artifact ever claims
  `flight-qualified`/`certified`/`operational`/`mission-ready` — the ceiling
  (§1.2) forbids it.
- **Fail-closed (`00` §3.9).** Every solver path returns `Result`, never panics;
  non-convergence is a reported `converged: false` (the corrector already does
  this), not an assertion. `unwrap/expect/panic` stay out of non-test code.

---

## 5. V&V plan

The five-layer ladder (`docs/verification.md`): MMS code-verification →
model-verification → code-to-code → public benchmark → UQ. For trajectory
optimization the "public benchmark" rung is *published OCPs with known optima*,
since no flight data exists (§1.2).

### 5.1 Verification cases and which label each tier earns

| Case | Type | Tier(s) | What it proves | Label earned |
|---|---|---|---|---|
| **Two-body apsis correction** (existing `corrects_tangential_speed_to_target_apogee`) | analytic | T0/T1 | corrector + shooting closes a known orbital residual | `validated-toy` |
| **T0/T1 apogee cross-tier regression** (`multiple-shooting-tolerance.toml`) | synthetic cross-tier | T0/T1 | perturbed T1 multiple-shooting solve matches the T0 single-shooting apogee terminal state to `< 1e-6` | `validated-toy` |
| **Brachistochrone** | known-optimal OCP | T2/T3 | HS & LGR transcription accuracy; costate/multiplier estimate | `research` |
| **Bryson-Denham** (state-constrained) | known-optimal OCP | T2/T3 | path-constraint handling; active-set correctness | `research` |
| **Goddard max-altitude (singular arc)** | bang-singular-bang OCP | T3 | ph-mesh refinement on a nonsmooth arc (Betts; PHR ph paper) | `research` |
| **MMS for transcription** | manufactured | T2 (O(h⁴)), T3 (spectral) | defect residual → 0 at the *expected order* as `h→0` | `research` (order-of-accuracy) |
| **Acikmese-Ploen / G-FOLD Mars soft-landing min-fuel** | published numerical solution | T4 | LCvx SOCP optimal mass + feasibility vs JGCD 2007 / TCST 2013 | `research` |
| **Szmuk 6-DoF free-final-time landing** | published trajectory | T5 | SCvx convergence behavior vs AIAA 2018-0617 / arXiv:1811.10803 | `research` where it converges; else `experimental` |
| **Orbit-raising / low-thrust transfer** (Hull/Betts) | published | T1–T3 | `OrbitalElements` terminal targeting | `research` |
| **PMP costate cross-check** | indirect witness | T6 | `H` constant on autonomous arcs; costates match KKT multipliers | `research` (optimality regression) |
| **Code-to-code: PSOPT / GMAT / Copernicus** | external oracle | T2–T4 | cost, state/control histories, costates within mesh tolerance | external-validity anchor |

### 5.2 Tolerance tables (representative; each WP ships its own TOML)

| Quantity | Tier | Tolerance | Notes |
|---|---|---|---|
| Orbital residual norm (apsis/elements) | T0/T1 | `< 1e-2` (mixed m / m·s⁻¹) | matches existing `meters_tolerance` test scale |
| T0/T1 terminal state cross-tier delta | T1 | `< 1e-6` on terminal position, velocity, continuity, and terminal residual norms | `scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml` |
| HS defect convergence order | T2 | slope `4.0 ± 0.3` on log‖ζ‖ vs log h | MMS order-of-accuracy gate |
| LGR spectral convergence | T3 | error decays faster than any algebraic order on smooth arcs | exponential-decay check |
| Brachistochrone optimal cost | T2/T3 | `< 1e-4` relative to closed form | |
| Goddard final altitude | T3 | within published-solution band (cite Betts table) | singular-arc tolerance |
| G-FOLD final mass (Acikmese-Ploen case) | T4 | `< 0.5%` of published optimal | LCvx optimality |
| G-FOLD constraint satisfaction | T4 | glideslope/pointing/thrust violation `< 1e-6` | feasibility |
| SCvx virtual-control at convergence ‖ν‖ | T5 | `< 1e-6` | dynamic-feasibility recovery |
| PMP Hamiltonian constancy | T6 | `|H(t) − H(t0)| < 1e-4` on autonomous arcs | optimality witness |

Every tolerance table is a checked-in TOML with a validation label and validity
envelope (`13` §3). No benchmark constant is inlined in `*.rs`.

### 5.3 What is explicitly **not** validated (state in each WP PR)

Absolute fidelity against any real vehicle (no flight heritage); WCET / real-time
flight-cert of the conic solver; SNOPT-grade sparse-SQP robustness; any
trajectory's operational suitability. These are the §1.2 ceiling items.

---

## 7. Dependencies on other parity docs

- **`08-environment-gravity-and-frames.md`** — tesseral gravity, third-body/SRP,
  CIO frames. The OCP `dynamics()` should consume the same gravity/frame models
  the simulator uses; higher-fidelity `f` improves every optimized trajectory.
  (T0–T3 can start on the existing central-gravity propagator.)
- **`06-gnc-coupled-mimo-and-control.md`** — the FC autopilot/MPC that **tracks**
  the optimized reference in T6; the closed-loop wiring is the join point. The
  `ReferenceState` topic and three-loop autopilot are the consumer.
- **`05-propulsion-high-fidelity.md`** — pressure-thrust / transient ballistics /
  throttle-Isp maps feed the `dynamics()` thrust model and the LCvx mass-log /
  thrust-bound constants; min-fuel realism depends on it.
- **`01-flexible-multibody-dynamics.md`** / **`02-structural-dynamics-loads-slosh-pogo.md`**
  — the load path constraints (axial/lateral load, q·α) and the slosh-coupled
  closure risk (`00` §8) the optimizer's path constraints must respect.
- **`10-flight-software-in-the-loop-xil.md`** — the I-load crosses the
  SIL/transport boundary the XIL doc defines; the AFTS *containment monitor*
  there is **never** a trajopt objective (`00` §6).
- **`11-monte-carlo-uq-and-validation.md`** — dispersed initial conditions /
  parameter uncertainty for robust reference generation; per-entry margins flow
  into MC; the `DispersionSource` enum (already closed against launch-azimuth /
  impact-location perturbations) is the boundary.
- **`12-determinism-realtime-and-compute.md`** — deterministic conic-solve
  settings and byte-stable artifact emission; the WCET ceiling (§1.2) is the
  real-time-cert boundary.

---

## 8. Open-source leverage

| Tool | License | Mode | Use |
|---|---|---|---|
| **Clarabel.rs** | Apache-2.0 (already vendored) | **couple** | Native-Rust IP conic solver. The convex subproblem solver for LCvx (T4), SCvx subproblems (T5), and the SQP QP (T2). No FFI; `deterministic_settings()` already wired in `mpc.rs`/`landing.rs`. |
| **IPOPT** (coin-or) | EPL-1.0 | **couple via FFI, feature-gated OFF by default** | Large-sparse-NLP backend for T2/T3 (industry-standard primal-dual IP, exact Hessian, MUMPS/HSL). EPL is weak file-level copyleft: FFI-link is fine, keep OPTIONAL, document the license boundary, **never** statically fold EPL source into the Apache/MIT crates. Default to MUMPS (LGPL) or bundled solver; HSL (MA57/MA97) is separately academic-licensed. |
| **OpEn** (optimization-engine) | MIT OR Apache-2.0 | **port-pattern / couple** | Pure-Rust embedded nonconvex optimizer (PANOC + penalty/ALM). Reference for an EPL-free in-Rust NLP path; candidate for the SCvx penalty/TR inner solve or a real-time-grade fallback. Fully compatible; can be a direct dep. |
| **argmin** + **nalgebra** + **num-dual** | MIT OR Apache-2.0 / Apache-BSD / MIT-Apache | **couple** | `argmin`: line-search, trust-region, BFGS/L-BFGS, Gauss-Newton/LM for the SQP outer loop, multiple-shooting Newton, PMP costate shooting. `num-dual`: forward-mode dual numbers for sparse AD Jacobians. `nalgebra` (already a dep): STMs, quaternions, dense/sparse LA. |
| **GPOPS-II** (papers) | commercial (closed) | **ingest methodology only** | ACM TOMS + ph-refinement papers are the *authoritative spec* for the LGR transcription, differentiation matrices, and mesh-refinement error estimator (T3). **Do NOT couple/port code** — clean-room from the open papers. |
| **PSOPT** | LGPL v2.1+ | **ingest / external oracle** | Run identical OCPs in PSOPT offline; compare cost/state/control/costates. LGPL would force relinking obligations if linked — use **only as an external oracle process**, never a linked dep. |
| **NASA GMAT** | Apache-2.0 | **couple as external V&V oracle** | The reference semantics for differential-corrector targeting; run reference orbit-raise/insertion in GMAT, export ephemerides, compare optimizer output at insertion. External tool / data oracle; no linking needed. |
| **EmbersArc/SCvx, Natsoulas/lcvx-pdg, Ceaser626/Lossless-convexification** | per-repo (mixed/often MIT or unspecified) | **ingest / study only** | Algorithm-validation references for TR management, virtual-control weighting, mass-log linearization constants, numerical cross-checks. **Clean-room reimplement from the papers; do not vendor; verify each repo's license before reusing any snippet.** |

---

## 9. Work-package backlog

Executed in `depends_on` order, one PR each (`13` §5). Every WP keeps all §2
gates green and answers the §3 per-PR checklist.

---

### WP-07.0 — Wire the differential corrector to the real propagator

- **title:** Give `DifferentialCorrector` a production offline driver/CLI caller.
- **implementation_status:** implemented. `correct_two_body_apogee` wires
  `DifferentialCorrector::solve` to a deterministic two-body RK4 forward map,
  `openbmp trajopt correct-apogee` writes a postcard I-load only on convergence,
  and `openbmp trajopt correct-apogee-scenario` uses `openbmp-runner` as the
  scenario forward map for the checked-in
  `scenarios/trajopt-two-body-apogee/` fixture. `REQ-TRAJOPT-001` traces the
  convergence, non-convergence, provenance, and tolerance evidence.
- **goal:** Turn correct-but-dead solver code into a tested capability with **no
  new math**: an offline driver whose `forward_map` integrates the workspace
  propagator to hit an `OrbitalElements`/`ApogeeRadius` terminal condition,
  emitting a postcard I-load. Proves the gate-green PR loop on the cheapest
  trajopt increment (`13` §7 recommended first PR).
- **fidelity_tier:** T0
- **depends_on:** []
- **new_crates:** none
- **touched:** `crates/openbmp-trajopt/src/{lib.rs,corrector.rs}` (add driver
  module); `crates/openbmp-cli/src/cli.rs` (+ `Command::Trajopt` subcommand with
  a `Synthesize` variant) and the CLI dispatch; new `tests/` + a fixture
  scenario.
- **approach:** §3.3 T0. Driver builds a `forward_map: FnMut(&[f64]) ->
  Result<BallisticState, TrajoptError>` that parameterizes a scenario (e.g.
  burnout speed / pitch breakpoints), runs the propagator (`openbmp-runner::run`
  or a `rk4_gravity_step` shooting loop), and wraps the terminal state with
  `BallisticState::from_forward_simulation` + a `ForwardSimulationProvenance`
  token. Drive `solve()`; on `converged`, emit an `ILoadPayload` via
  `encode_iload_payload`; on non-convergence, report `residual`/`iterations`
  honestly and exit non-zero.
- **acceptance:**
  - Off by default; no existing scenario golden changes (offline driver only).
  - One reproducible scenario produces a reference trajectory + I-load; the
    two-body apsis case closes the residual to `< 1e-2` in a tolerance table.
  - Driver reports `converged=false` honestly on an infeasible target (test).
  - `DifferentialCorrector::solve` has a non-test caller (grep gate).
  - All §2 gates green; provenance for the fixture scenario present.
- **validation_label:** `validated-toy`
- **dual_use_note:** Forward-only by construction — consumes only
  `TerminalCondition`; residuals are orbital/inertial. Add a grep/compile assert
  that the driver exposes no surface-coordinate input.
- **est_effort:** ~1 wk
- **parity_ceiling:** Single-shooting only; no path constraints; no flight
  fidelity. Does not validate any real trajectory.

---

### WP-07.1 — Multiple shooting with STM / variational Jacobian

- **title:** Generalize the corrector from M=1 to M-segment multiple shooting.
- **implementation_status:** implemented. `src/stm.rs` integrates deterministic
  two-body variational equations with a state-transition matrix and verifies
  the STM against complex-step sensitivities, and
  `src/shooting.rs` evaluates M-segment continuity defects plus a block-
  bidiagonal `[STM_i, -I]` Jacobian and duration-sensitivity columns.
  `MultipleShootingCorrector` now performs damped fixed-endpoint interior-node
  correction and reports honest non-convergence for inconsistent endpoints; it
  also performs fixed-initial terminal-condition correction against closed
  `TerminalCondition` equality residuals, including a free-duration variant
  that solves downstream nodes and segment times together and a controlled
  variant that solves downstream nodes plus piecewise-constant ECI acceleration
  controls. Soft path rows now include qbar and qbar-alpha mappings from node
  state, density, and a declared body-forward axis. A trybuild UI test proves
  the public multiple-shooting node surface cannot carry target latitude/
  longitude fields. The checked-in
  `scenarios/trajopt-two-body-apogee/multiple-shooting-tolerance.toml` table
  now gates a perturbed T1 multiple-shooting solve against the T0
  single-shooting apogee solution to `< 1e-6`.
- **goal:** Robust ascent-to-orbit reference generation that reuses the existing
  physics propagator, replacing the single-shooting limitation with block-
  bidiagonal continuity defects and an STM-based Jacobian.
- **fidelity_tier:** T1
- **depends_on:** [WP-07.0]
- **new_crates:** none
- **touched:** `crates/openbmp-trajopt/src/{corrector.rs,lib.rs}` (+ new
  `shooting.rs`, `stm.rs`); the `OptimalControlProblem` trait (§3.2).
- **approach:** §3.3 T1. Free vector `[s_0..s_{M-1}, controls, t_f]`; stacked
  continuity defects `c_i = φ(t_{i+1}; s_i, u_i) − s_{i+1}`; Jacobian via the
  variational STM `Φ̇ = (∂f/∂x)Φ` integrated with augmented RK4, **or**
  complex-step per column. Block-bidiagonal sparse normal equations (extend the
  corrector's `solve_linear_system` to exploit the structure, or use a sparse
  factorization). Box bounds + q̄/q·α penalties.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - M-segment result matches the T0 single-shooting solution on a shared case to
    `< 1e-6` (cross-tier regression) in a tolerance table.
  - STM Jacobian agrees with complex-step Jacobian column-wise to `< 1e-8` (MMS-
    style verification case).
  - **New compile-fail tripwire** (§6.2) proving the M-segment free-variable /
    `OptimalControlProblem` surface has no surface-coordinate field.
  - All §2 gates green.
- **validation_label:** `validated-toy`
- **dual_use_note:** Adds the §6.2 proving tripwire. Boundary stays
  `TerminalCondition`-only; the STM is a sensitivity, not a target.
- **est_effort:** ~2–3 wk
- **parity_ceiling:** No NLP-grade path constraints (penalty only); no objective
  optimization beyond feasibility. Not flight-validated.

---

### WP-07.2 — Hermite-Simpson direct collocation + SQP on clarabel

- **title:** Fixed-mesh HS transcription with sparse SQP (clarabel QP subproblem).
- **implementation_status:** partial (2026-06-26) — `collocation.rs` exposes
  Hermite-Simpson segment and fixed-mesh transcription evaluation with dynamics
  defects, Hermite midpoint defects, Simpson Lagrange cost, fixed-mesh decision
  layout, boundary residual assembly, soft path residuals, dense hard scalar
  path inequality rows, hard scalar path decision-bound merges for the dense SQP
  lane, nonlinear vector hard path inequality rows with sparse CSC
  finite-difference Jacobian reports, Simpson-weighted quadratic control
  objective residuals, columnwise and colored central-finite-difference CSC
  sparse residual Jacobian reports, caller-supplied analytic dynamics-Jacobian
  CSC sparse residual Jacobian reports, forward-mode autodiff dynamics-Jacobian
  CSC sparse residual Jacobian reports, sparse Gauss-Newton/KKT normal-block
  assembly plus sparse normal-equation RHS/matvec/conjugate-gradient solve
  evidence, sparse augmented-KKT assembly with deterministic LDL solve evidence,
active scalar hard-path constrained sparse-KKT step evidence with active
  inequality multipliers and constrained sparse-KKT diagnostic handoff evidence,
  KKT residual-multiplier row-family slicing plus raw node-costate estimates for
  later PMP witnesses, raw Hamiltonian diagnostics, and fixed-mesh sparse
  Gauss-Newton/KKT step reports plus a residual-only sparse-KKT SQP outer loop
  that reports accepted step scales, scalar hard-path decision-bound enforcement, nonlinear hard-path feasibility preservation, convergence status, final bound violations, scalar hard-path violations, final nonlinear hard-path violations, and final multipliers,
  sparse-SQP PMP/KKT certificate payload reports that prefer the final active
  constrained-KKT report when present, can attach validated same-mesh
  PMP-vs-KKT costate summaries, and round-trip through optimized-reference
  I-load payloads,
  validation, MMS
  order evidence, and a validated-toy scalar integrator minimum-energy
  benchmark that recovers analytic `x(t)=t`, `u(t)=1`, cost `1`; it checks a
  public closed-form brachistochrone cycloid
  transcription/cost fixture against the known optimal time, restores a
  perturbed fixed-control brachistochrone state grid and free duration through
  the dense SQP wrapper, checks a free-control brachistochrone
  final-time-objective run against the public cycloid time/residual tolerances,
  and checks a public Bryson-Denham state-constrained fixed-mesh
  dense-QP wrapper benchmark with sparse constrained-KKT handoff and
  repeat-deterministic Clarabel output; it
  reuses the T0/T1
  synthetic apogee tolerance table as a T1/T2 shared-ascent Hermite-Simpson mesh
  cross-check; it emits optimized node/midpoint reference-profile samples and
  deterministic I-load payloads from solved fixed-mesh decisions;
  `openbmp-fc::guidance` exposes an FC-native
  `OptimizedReferenceProfileGuidance` consumer that revalidates the same
  radius/speed/FPA sample shape, phase-gates and interpolates it, publishes
  `ReferenceState`, and proves the existing autopilot PID trajectory loop reacts
  through a focused actuator-command test; `openbmp-testkit` also proves a
  host-side postcard I-load round trip can convert decoded trajopt reference
  samples into that FC-native guidance shape and publish an interpolated
  `ReferenceState` without adding an `openbmp-fc -> openbmp-trajopt` edge, and
  schema-v3 runner configs can select `guidance = "optimized_reference"` with
  inline `[fc.optimized_reference]` samples or a resolved postcard I-load file
  that instantiate the same FC-native guidance job and publish an interpolated
  active-phase `ReferenceState` and validated-toy inline plus PMP-derived
  file-backed optimized-reference runner scenarios export valid
  `guidance.reference.*` telemetry plus nonempty FC actuator command-stream
  evidence under `REQ-TRAJOPT-004`;
  feature-gated `sqp.rs` exposes deterministic Clarabel dense QP solves,
  box-bounded least-squares, linear-objective least-squares,
  equality-constrained linear-objective, and linearized-inequality
  least-squares SQP step conversion, plus bounded nonlinear
  least-squares-plus-linear-objective, equality-constrained linear-objective,
  and nonlinear least-squares SQP drivers with exact residual-penalty merit
  globalization plus elastic residual-slack and residual-only restoration
  fallback for equality-objective steps and with/without nonlinear inequality
  rows; `collocation.rs` exposes
  feature-gated fixed-mesh
  quadratic-control dense-QP and dense-SQP solve wrappers with final residual,
  bound-violation, and repeat-determinism reporting under
  `REQ-TRAJOPT-003`/`V-TRAJOPT-003`, including a public
  `solve_dense_sqp_with_free_final_time` wrapper for uniform-mesh final-time
  restoration and a `solve_dense_sqp_with_free_final_time_objective`
  equality-constrained linear-objective wrapper with scalar-integrator active
  final-time-bound evidence and brachistochrone free-control time/residual
  metric evidence; public sparse-NLP backend contracts plus preflight assembly
  and checked backend-solve validation, plus a feature-gated deterministic
  dense-SQP sparse-NLP adapter for bounded contract cases, are now exposed by
  `nlp.rs` under `REQ-TRAJOPT-010`; full
  production constrained large-sparse NLP backend adapters, production sparse KKT
  factorization, fully stationary free-control public benchmark NLP closure,
  and full PMP/costate optimality pipeline closure remain open.
- **goal:** A general fixed-mesh OCP solver: min-fuel / min-time ascent with full
  path constraints, cross-checked against T1. The first `research`-label tier
  (MMS order-of-accuracy + known-optimal benchmark).
- **fidelity_tier:** T2
- **depends_on:** [WP-07.1]
- **new_crates:** `openbmp-convex` exists for WP-07.4-a shared conic settings and
  CSC helpers; WP-07.2's full constrained large-sparse NLP backend remains open
  separately.
- **touched:** `crates/openbmp-trajopt/src/` (`collocation.rs` implemented;
  feature-gated dense QP/SQP `sqp.rs` implemented; `nlp.rs` implements the
  public sparse-NLP backend contract, assembly preflight, checked solve wrapper,
  and bounded feature-gated dense-SQP adapter; production large-sparse/KKT NLP
  backend adapters still open);
  `crates/openbmp-fc/src/guidance.rs` (FC-native optimized-reference profile
  consumer hook implemented; full postcard payload ingestion inside FC remains
  intentionally out of scope);
  `crates/openbmp-scenario/src/{document.rs,lib.rs,scenario.rs}` and
  `crates/openbmp-runner/src/{fc.rs,fc_bridge.rs}` (v3 inline or file-backed
  optimized-reference scenario config and runner guidance construction
  implemented; full PMP/costate optimality closure still open);
  `crates/openbmp-testkit/tests/optimized_reference_iload_to_fc.rs` (host
  bridge evidence from decoded trajopt I-load samples into FC-native guidance);
  `scenarios/trajopt-two-body-apogee/hermite-simpson-mms-tolerance.toml`;
  `scenarios/trajopt-two-body-apogee/hermite-simpson-scalar-integrator-benchmark.toml`;
  `scenarios/trajopt-brachistochrone/brachistochrone-cycloid-benchmark.toml`;
  `scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml`;
  future optional `Cargo.toml` additions such as feature-gated `ipopt-sys`.
- **approach:** §3.3 T2. Separated HS defects + Hermite midpoint constraint;
  Simpson Lagrange cost; block-banded sparse Jacobian via the internal
  forward-mode AD scalar or sparsity-colored FD; SQP outer loop with `clarabel` solving the QP
  subproblem and a merit-function line search (**partial satisfied:** bounded
  nonlinear least-squares driver plus colored finite-difference and
  forward-mode autodiff residual Jacobian reports, sparse Gauss-Newton
  normal-block assembly plus sparse normal-equation CG step mechanics,
  augmented-KKT assembly plus deterministic LDL solve/step mechanics, active
  scalar hard-path constrained sparse-KKT step/multiplier mechanics plus
  constrained sparse-KKT residual-multiplier/costate/Hamiltonian diagnostic
  handoff, and raw
  residual-multiplier row-family slicing with unscaled node-costate estimates,
  raw Hamiltonian diagnostics, residual-only sparse-KKT SQP scalar/nonlinear
  hard-path feasibility and final-multiplier evidence, plus constrained
  dense-SQP nonlinear inequality consumption and the public `Transcription` /
  `NlpSolver` sparse-NLP contract plus validated `assemble_sparse_nlp`
  preflight; full constrained large-sparse NLP backend adapters, production sparse KKT
  factorization, and the PMP costate mapping/witness still open);
  canonical-unit (DU/TU) scaling. Optional `ipopt` feature flag for the
  large-sparse backend (EPL boundary documented).
- **acceptance:**
  - Off by default; goldens byte-identical.
  - **MMS:** manufactured solution → HS defect residual decays at slope
    `4.0 ± 0.3` (O(h⁴)) on a log-log mesh-refinement sweep, in a tolerance table
    (**partial satisfied:** scalar sine midpoint residual table and focused
    `hermite_simpson_midpoint_mms_residual_is_fourth_order` test).
  - **Validated-toy benchmark:** scalar minimum-energy integrator QP recovers
    analytic `x(t)=t`, `u(t)=1`, cost `1` from a provenance-pinned tolerance
    table (**satisfied:** `fixed_mesh_ocp_scalar_integrator_min_energy_benchmark_matches_analytic_solution`
    consumes `hermite-simpson-scalar-integrator-benchmark.toml`; it is a
    validated-toy analytic OCP, not a public research benchmark).
  - Brachistochrone optimal cost within `< 1e-4` relative to closed form
    (**partial satisfied:** `hermite_simpson_brachistochrone_closed_form_benchmark_matches_cycloid`
    evaluates the public cycloid solution from a provenance-pinned fixture and
    checks the HS transcription residual plus time cost;
    `fixed_mesh_ocp_brachistochrone_dense_sqp_restores_perturbed_cycloid`
    restores a perturbed fixed-control public-cycloid state grid,
    `linearized_least_squares_objective_step_adds_linear_term` proves the
    dense-QP step can consume an explicit linear objective term needed by
    free-time/free-control SQP work,
    `linearized_equality_objective_step_moves_along_nullspace` proves the
    dense-QP step can preserve linearized equality residuals while descending a
    linear objective,
    `dense_sqp_linear_objective_continues_from_feasible_residual` proves the
    outer dense-SQP loop keeps optimizing that linear term even from a
    residual-feasible starting point, and
    `dense_sqp_equality_objective_preserves_linear_residual` proves the
    equality-constrained objective SQP lane preserves a scalar residual while
    optimizing through its nullspace, and
    `dense_sqp_equality_objective_accepts_nonlinear_tangent_step` proves the
    exact residual-penalty merit accepts a nonlinear equality tangent descent
    step and then improves residual through restoration, and
    `linearized_elastic_equality_objective_step_uses_slack_when_bounds_block_residual`
    plus
    `dense_sqp_equality_objective_elastic_fallback_handles_blocked_linearized_residual`
    prove the equality-objective lane has a bounded residual-slack fallback for
    blocked hard equality QPs, and
    `fixed_mesh_ocp_free_final_time_objective_drives_scalar_integrator_to_bound`
    proves the fixed-mesh free-final-time objective wrapper drives a scalar
    integrator to an active final-time bound while maintaining the expected
    least-squares residual tolerance, and
    `fixed_mesh_ocp_brachistochrone_dense_sqp_restores_free_duration` recovers a
    perturbed final time through `solve_dense_sqp_with_free_final_time`, and
    `fixed_mesh_ocp_brachistochrone_free_control_time_objective_recovers_cycloid`
    reaches the public cycloid time and residual tolerances with free controls
    through `solve_dense_sqp_with_free_final_time_objective`; fully stationary
    free-control NLP closure remains open);
    Bryson-Denham state-constrained case satisfies the constraint and matches the
    published optimum (**partial satisfied:**
    `fixed_mesh_ocp_bryson_denham_state_constrained_benchmark_matches_active_arc`
    and `fixed_mesh_ocp_bryson_denham_dense_quadratic_qp_wrapper_matches_active_arc`
    and
    `fixed_mesh_ocp_bryson_denham_dense_solution_feeds_sparse_constrained_kkt_report`
    solve the normalized state-constrained fixed-mesh dense QP from a
    provenance-pinned fixture through the exposed wrapper, feed its dense
    solution into active sparse constrained-KKT row and residual-multiplier
    handoff evidence, enforce the active hard state bound, and match objective
    `4`; general nonlinear benchmark solve closure remains open).
  - Cross-check vs T1 on a shared ascent case within mesh tolerance
    (**partial satisfied:** `hermite_simpson_cross_checks_t1_shared_ascent_case`
    reuses `multiple-shooting-tolerance.toml` and gates terminal deltas plus the
    40 s HS collocation mesh-defect norm).
  - Optimized fixed-mesh reference profile emits deterministic I-load samples
    with strict time ordering and can be replayed by the FC-side
    optimized-reference guidance hook (**partial satisfied:** output artifact
    plus focused `OptimizedReferenceProfileGuidance` validation/interpolation,
    host testkit postcard I-load bridge, v3 inline/file-backed
    optimized-reference runner wiring, and autopilot trajectory-loop tracking
    evidence; full PMP/costate optimality remains WP-07.6 work).
  - clarabel uses the pinned deterministic settings; result byte-stable across
    two runs (**partial satisfied:** dense QP repeated-solve test plus
    `fixed_mesh_ocp_dense_sqp_wrapper_is_repeat_deterministic` and
    `fixed_mesh_ocp_bryson_denham_benchmark_qp_is_repeat_deterministic`; broader
    nonlinear public-benchmark OCP determinism remains open).
  - All §2 gates green; benchmark constants in a provenance-pinned data file.
- **validation_label:** `research` (MMS + known-optimal benchmark)
- **dual_use_note:** Objective is vehicle-intrinsic (fuel/time/energy); boundary
  is `TerminalCondition`-only; §6.2 tripwire extended to the `Transcription`
  decision-vector layout.
- **est_effort:** ~4–6 wk
- **parity_ceiling:** No SNOPT-grade robustness; fixed mesh (no auto-refinement
  until T3); not WCET-bounded; not flight-validated.

---

### WP-07.3 — LGR pseudospectral + ph-mesh refinement

- **title:** Legendre-Gauss-Radau collocation with Patterson-Hager-Rao ph-mesh.
- **implementation_status:** partial (2026-06-26) — `pseudospectral.rs` exposes
  `LegendreGaussRadauGrid` and `legendre_gauss_radau_grid` as a deterministic
  left-LGR grid substrate on `[-1, 1]`; the grid includes `-1` as the first
  collocation node, excludes `+1` from collocation nodes, appends `+1` as the
  terminal interpolation support node, reports positive quadrature weights,
  exposes an `N x N+1` barycentric differentiation matrix, and provides helpers
  for differentiating support-node values and integrating collocation-node
  values. It also exposes `LgrPseudospectralSegment`,
  `LgrPseudospectralSegmentReport`, `LgrPseudospectralTranscription`,
  `LgrPseudospectralTranscriptionReport`,
  `evaluate_lgr_pseudospectral_segment`, and
  `evaluate_lgr_pseudospectral_transcription`, validating support states,
  collocation controls, nonempty adjacent segments, and matching segment
  dimensions, evaluating existing `CollocationDynamics`/`CollocationLagrange`
  callbacks at LGR nodes, and reporting physical collocation times, physical
  quadrature weights, differential residuals, segment-boundary continuity
  residuals, residual counts, residual norms, and quadrature cost. It also
  exposes `LgrPseudospectralDecisionLayout` and
  `LgrPseudospectralDecisionReport` plus
  `LgrPseudospectralDecisionBoundsReport`,
  `LgrPseudospectralReferenceProfileMapping`, and
  `LgrPseudospectralReferenceProfileReport` for fixed-time flat decision vectors,
  validating per-segment LGR degrees, mesh times, state/control dimensions,
  state/control block offsets, total variable counts, and finite decisions;
  packing row-major support-state and collocation-control blocks; returning
  segment support-state, collocation-control, and mesh-node state slices,
  including zero-control slices; decoding decisions into validated multi-segment
  transcriptions; and evaluating decoded decisions through the existing LGR
  transcription evaluator. It also exposes
  `LgrPseudospectralBoundaryConstraint`,
  `LgrPseudospectralScalarVariable`,
  `LgrPseudospectralSoftPathConstraint`,
  `LgrPseudospectralHardPathConstraint`,
  `LgrPseudospectralKktCollocationMultiplierSample`,
  `LgrPseudospectralKktResidualMultiplierReport`,
  `LgrPseudospectralKktCollocationCostateEstimate`,
  `LgrPseudospectralKktCostateEstimateReport`,
  `LgrPseudospectralKktHamiltonianSample`,
  `LgrPseudospectralKktHamiltonianReport`,
  `LgrPseudospectralNonlinearHardPathConstraint`,
  `LgrPseudospectralPathInequalityReport`,
  `LgrPseudospectralNonlinearPathInequalityReport`,
  `LgrPseudospectralQuadraticControlCost`,
  `LgrPseudospectralMayerCost`,
  `LgrPseudospectralObjectiveReport`,
  `LgrPseudospectralSparsePathInequalityJacobianReport`,
  `LgrPseudospectralColoredSparseResidualJacobianReport`,
  `LgrPseudospectralSparseResidualJacobianReport`,
  `LgrPseudospectralSparseGaussNewtonHessianReport`,
  `LgrPseudospectralSparseAugmentedKktReport`,
  `LgrPseudospectralSparseConstrainedKktReport`,
  `LgrPseudospectralSparseNormalSolveReport`,
  `LgrPseudospectralSparseKktSolveReport`,
  `LgrPseudospectralFixedMeshSparseGaussNewtonStepReport`,
  `LgrPseudospectralFixedMeshSparseKktStepReport`,
  `LgrPseudospectralFixedMeshSparseConstrainedKktStepReport`,
  `LgrPseudospectralFixedMeshDenseConstrainedSqpReport`,
  `LgrPseudospectralFixedMeshDenseQuadraticQpReport`,
  `LgrPseudospectralFixedMeshDenseSqpReport`,
  `LgrPseudospectralFixedMeshOcp`, and
  `LgrPseudospectralFixedMeshResidualReport` for fixed-time boundary,
  soft-path, quadratic control objective residual, hard scalar path inequality,
  nonlinear hard path inequality assembly, finite-difference sparse columnwise
  and colored residual-Jacobian reporting, analytic/autodiff residual-Jacobian
  reporting, sparse Gauss-Newton normal-block/CG, sparse augmented-KKT/LDL
  step reporting, active scalar hard-path constrained sparse-KKT step reporting
  with constrained sparse-KKT diagnostic handoff,
  Mayer/Lagrange objective reporting, raw residual-multiplier/
  costate/Hamiltonian diagnostics including sparse-KKT step-derived diagnostics,
  residual-only sparse-KKT SQP scalar/nonlinear hard-path feasibility, optional
  final active constrained-KKT report, final-multiplier evidence,
  quadrature-normalized sparse-SQP PMP/KKT certificate payload reports that
  prefer the final active constrained-KKT report when present, can attach
  validated same-mesh PMP-vs-KKT costate summaries, and round-trip
  through optimized-reference I-load payloads, and nonlinear
  path-inequality Jacobian reporting, plus
  feature-gated fixed-time dense QP/SQP wrapping,
  validating mesh-node boundary targets/components,
  collocation-node scalar soft/hard path bounds/components, nonlinear hard vector
  path bound dimensions/samples, nonlinear path outputs, finite-difference
  sparse-Jacobian settings, stable residual/nonlinear-inequality row dimensions,
  colored residual-Jacobian report consistency, analytic/autodiff
  residual-Jacobian dynamics sample and report consistency, finite Mayer
  objective values, objective report totals, CSC sparse residual/path-Jacobian
  report shape, finite decision-bound reports, optimized-reference
  mappings/reports/I-load payloads, and control-cost components,
  appending boundary residuals after
  differential and continuity rows, then path and objective residuals,
  transcribing hard path bounds into dense `Aineq * decision <= rhs` rows with
  lhs/violation helpers, merging hard scalar path bounds into dense solver
  decision-bound reports, emitting strictly increasing optimized
  `ReferenceProfileSample` sequences from LGR support-state samples while
  skipping duplicate segment-boundary samples and packaging them into validated
  postcard `ILoadPayload` artifacts, evaluating nonlinear hard path bounds as bound-split
  `g(z) <= 0` rows with raw path values and max-violation reporting, evaluating
  scalar objectives from LGR-quadrature Lagrange costs, configured quadratic
  control costs, and arbitrary terminal Mayer callbacks, splitting
  caller-supplied KKT residual multipliers by LGR row family, estimating raw
  collocation costates from differential multipliers, evaluating raw Hamiltonian
  diagnostics at LGR collocation samples, computing deterministic
  central-finite-difference CSC sparse columnwise and colored residual Jacobians,
  caller-supplied analytic dynamics-Jacobian and forward-mode autodiff CSC sparse
  residual Jacobians, sparse Gauss-Newton Hessian/KKT normal blocks, sparse
  `-J'r` right-hand sides, sparse normal-block matrix-vector products,
  deterministic conjugate-gradient sparse normal solves, sparse augmented-KKT
  matrices, sparse constrained-KKT systems for active scalar hard-path rows,
  constrained sparse-KKT residual-multiplier/costate/Hamiltonian diagnostic
  adapters, deterministic LDL KKT solves, fixed-mesh sparse Gauss-Newton/KKT step
  reports with linearized residuals and residual multipliers, residual-only
  sparse-KKT SQP accepted-step iteration reports with scalar hard-path decision-bound enforcement, final bound violations, scalar hard-path violations, final KKT multipliers, and optional final active constrained-KKT step reports, plus
  nonlinear
  path-inequality Jacobians with coefficient lookup and dense expansion helpers,
  forming feature-gated dense Clarabel
  equality-constrained quadratic-control QPs for affine LGR residual models,
  calling the feature-gated `DenseSqpSolver` lanes for fixed-time residual
  restoration, terminal-state linear objectives, and constrained nonlinear
  hard-path row consumption, and reporting
  row counts, residual norms, max residuals,
  quadrature cost, quadratic control cost, scalar objective totals, final
  dense-SQP residuals, terminal objective values, decision-bound violations,
  scalar hard-path violations, and nonlinear hard-path violations. It also
  exposes `LgrPseudospectralMeshRefinementSettings`,
  `LgrPseudospectralSegmentErrorEstimate`,
  `LgrPseudospectralMeshRefinementAction`,
  `LgrPseudospectralSegmentRefinementRecommendation`,
  `LgrPseudospectralMeshRefinementReport`, and
  `estimate_lgr_pseudospectral_mesh_refinement` for candidate-vs-reference
  support-state error estimates and deterministic accept/raise-degree/split
  recommendations, plus `LgrPseudospectralRefinedSegmentMapping`,
  `LgrPseudospectralMeshRefinementApplication`, and
  `apply_lgr_pseudospectral_mesh_refinement` for seeded raised-degree and split
  refined transcriptions, and `LgrPseudospectralMeshRefinementIterationReport`,
  `LgrPseudospectralMeshRefinementLoopReport`, and
  `refine_lgr_pseudospectral_mesh_with_reference` for a caller-supplied
  reference-driven estimate/apply loop. Focused tests prove the three-point
  nodes and weights against a closed-form reference, exact Radau quadrature for
  monomials through degree `2N-2`, exact differentiation of a support
  polynomial, exact single-segment and two-segment transcription/quadrature for
  `x=t^2`, `x_dot=2t`, continuity stacking, two-segment flat decision
  decode/evaluate exactness, zero-control decision-layout support, malformed
  decision-layout rejection, fixed-time OCP boundary residual appending,
  boundary mismatch reporting, soft path residual and LGR-weighted quadratic
  control objective residual appending, hard path dense inequality
  transcription and violation reporting, hard path decision-bound merge evidence
  for dense solver consumption, feature-gated fixed-time dense SQP
  scalar-integrator minimum-energy dense-QP benchmark evidence recovering
  `x(t)=t`, `u(t)=1`, cost `1`, public closed-form LGR brachistochrone
  transcription/time-cost evidence against the cycloid fixture,
  public COPS/YAPSS Goddard fixture evidence pinning the dimensionless dynamics,
  singular-arc relation, fixed-mesh LGR seed residual accounting,
  residual-localized split-refinement seeding, and split-refined
  terminal-objective handoff plus published final-altitude gap audit,
  feature-gated public COPS/YAPSS Goddard fixed-mesh and split-refined
  terminal-altitude objective dense-SQP evidence improving terminal altitude
  and reducing equality residual from the seed while reporting the current
  fixed-time handoff outside the published final-time, altitude, and residual
  bands,
  feature-gated moderate-mesh fixed-time LGR brachistochrone dense-SQP
  free-control feasibility recovery from a perturbed cycloid control decision,
  feature-gated fixed-time dense QP Bryson-Denham state-constrained benchmark
  evidence recovering the active arc, objective `4`, hard-path feasibility, and
  repeat determinism plus sparse constrained-KKT active-row/multiplier handoff
  from the dense benchmark solution through
  `scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml`,
  feature-gated
  fixed-time dense SQP scalar-integrator restoration plus repeat-determinism
  evidence, feature-gated
  fixed-time dense constrained SQP consumption of nonlinear hard-path rows,
  optimized reference-profile and postcard I-load round-trip evidence,
  nonlinear hard path inequality
  evaluation and malformed nonlinear path rejection, sparse residual-Jacobian
  numeric evidence against the LGR differentiation and boundary-row reference,
  colored residual-Jacobian numeric evidence against the columnwise report,
  analytic residual-Jacobian numeric evidence against the finite-difference
  report, forward-mode autodiff residual-Jacobian numeric evidence against the
  analytic report, sparse nonlinear path-inequality Jacobian numeric evidence
  against a linear path-function derivative reference, Mayer/Lagrange objective
  numeric evidence with terminal state reporting, raw residual-multiplier
  row-family slicing plus raw collocation-costate and Hamiltonian diagnostic
  evidence, sparse Gauss-Newton normal-step evidence against dense normal
  references via
  `lgr_fixed_mesh_ocp_sparse_gauss_newton_step_matches_dense_normal_system`,
  sparse augmented-KKT assembly/LDL/step via
  `lgr_fixed_mesh_ocp_sparse_augmented_kkt_step_reports_multipliers`,
  active scalar hard-path constrained sparse-KKT multiplier and diagnostic
  handoff evidence via
  `lgr_fixed_mesh_ocp_sparse_constrained_kkt_step_reports_active_hard_path_multipliers`,
  and sparse-KKT-derived multiplier diagnostic evidence,
  residual-only sparse-KKT SQP convergence/final-multiplier plus optional final active constrained-KKT report, hard-bound
  enforcement, and nonlinear hard-path feasibility evidence via
  `lgr_fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers`,
  `lgr_fixed_mesh_ocp_sparse_kkt_sqp_enforces_hard_path_decision_bounds`, and
  `lgr_fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility`, malformed
  sparse/colored/analytic/autodiff residual, objective, and path-Jacobian
  input/report rejection, malformed boundary/path/objective/decision rejection,
  exact candidate-vs-reference ph convergence, degree-raise and split
  recommendation behavior, seeded refined-transcription state/control
  interpolation, reference-driven loop convergence/cap reporting, checked
  `lgr-spectral-convergence-tolerance.toml` smooth-arc interpolation error
  collapse for `x(t)=exp(t)`, the
  `lgr-scalar-integrator-benchmark.toml` solved validated-toy scalar-integrator
  benchmark fixture, the `brachistochrone-cycloid-benchmark.toml` public
  closed-form LGR transcription/time-cost fixture, the
  `bryson-denham-state-constrained-benchmark.toml` public active-arc LGR
  dense-QP fixture, the `goddard-cops-benchmark.toml` public COPS/YAPSS
  Goddard dynamics, singular-arc, fixed-mesh seed residual-accounting,
  residual-localized split-refinement seeding, fixed-mesh terminal-objective
  accounting, split-refined terminal-objective handoff, and published
  final-altitude gap fixture, scalar-PMP LGR dense-QP quadrature-normalized
  covector bridge evidence via
  `lgr_dense_qp_duals_feed_pmp_costate_convention_bridge`,
  and malformed
  dimension/non-finite/transcription/refinement/loop input rejection under
  `REQ-TRAJOPT-005`. General LGR nonlinear benchmark solver closure, full
  Goddard nonlinear solver/final-altitude closure, full constrained external
  sparse NLP integration, production nonlinear covector scaling/sign mapping
  beyond the scalar dense-QP bridge, and production PMP witness integration
  remain open.
- **goal:** GPOPS-II-class accuracy with automatic mesh refinement and costate
  estimates from KKT multipliers (the optimality witness feeding T6), reusing the
  T2 NLP backend.
- **fidelity_tier:** T3
- **depends_on:** [WP-07.2]
- **new_crates:** none
- **touched:** `crates/openbmp-trajopt/src/` (+ `pseudospectral.rs`).
- **approach:** §3.3 T3. LGR nodes via Golub-Welsch (Jacobi-matrix eigenvalues)
  or Newton on `P_N`; barycentric differentiation/integration matrices; integral-
  form collocation; ph relative-error estimate `e_k` with degree-raise vs
  subdivide logic; costate from KKT multipliers (covector mapping). Reuse the T2
  SQP/IPOPT backend.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - LGR nodes/weights match a Gauss-quadrature reference to `< 1e-12`
    (**partial satisfied:** `lgr_grid_matches_three_point_closed_form_reference`
    checks the three-point left-LGR closed-form nodes/weights, while
    `lgr_quadrature_is_exact_through_radau_degree` checks exact monomial
    quadrature through degree `2N-2`).
  - LGR differentiation matrix differentiates support polynomials at
    collocation nodes (**partial satisfied:**
    `lgr_differentiation_is_exact_for_support_polynomial`; not yet a full OCP
    transcription).
  - Single- and multi-interval LGR differential transcription and quadrature evaluate
    against the same dynamics/Lagrange callback vocabulary as WP-07.2
    (**partial satisfied:**
    `lgr_segment_transcription_is_exact_for_polynomial_integrator` and
    `lgr_transcription_stacks_segments_and_continuity_for_polynomial_integrator`,
    plus `lgr_decision_layout_decodes_and_evaluates_polynomial_transcription`
    for flat decision decode/evaluate and
    `lgr_fixed_mesh_ocp_appends_boundary_residuals_to_polynomial_transcription`
    for boundary rows, plus
    `lgr_fixed_mesh_ocp_appends_path_and_quadratic_control_objective_residuals`
    for soft scalar path residuals and LGR-weighted quadratic control objective
    rows, and
    `lgr_fixed_mesh_ocp_transcribes_hard_path_bounds_to_dense_inequalities`
    for hard scalar path inequalities, plus
    `lgr_fixed_mesh_ocp_evaluates_nonlinear_hard_path_inequalities` for
    nonlinear hard path rows, and
    `lgr_fixed_mesh_ocp_sparse_residual_jacobian_matches_linear_reference` for
    finite-difference sparse residual derivatives,
    `lgr_fixed_mesh_ocp_colored_sparse_residual_jacobian_matches_columnwise_report`
    for colored residual derivatives,
    `lgr_fixed_mesh_ocp_analytic_sparse_residual_jacobian_matches_finite_difference_report`
    for analytic residual derivatives,
    `lgr_fixed_mesh_ocp_autodiff_sparse_residual_jacobian_matches_analytic_report`
    for autodiff residual derivatives, plus
    `lgr_fixed_mesh_ocp_objective_report_transcribes_lagrange_mayer_and_quadratic_costs`
    for Mayer/Lagrange objective reporting, plus
    `lgr_fixed_mesh_ocp_slices_kkt_multipliers_and_reports_costate_hamiltonian_diagnostics`
    for raw LGR residual-multiplier row-family slicing and raw
    collocation-costate/Hamiltonian diagnostics, plus
    `lgr_fixed_mesh_ocp_merges_hard_path_bounds_for_dense_solver` for dense
    solver bound handoff,
    `lgr_fixed_mesh_ocp_scalar_integrator_min_energy_benchmark_matches_analytic_solution`
    for a feature-gated solved validated-toy LGR dense-QP benchmark, plus
    `lgr_fixed_mesh_ocp_bryson_denham_dense_quadratic_qp_wrapper_matches_active_arc`
    and
    `lgr_fixed_mesh_ocp_bryson_denham_dense_solution_feeds_sparse_constrained_kkt_report`
    and
    `lgr_fixed_mesh_ocp_bryson_denham_benchmark_qp_is_repeat_deterministic`
    for public Bryson-Denham state-constrained LGR dense-QP active-arc,
    sparse constrained-KKT handoff, hard-path feasibility, objective, and
    repeat-determinism evidence, plus
    `lgr_brachistochrone_closed_form_benchmark_matches_cycloid`
    for public closed-form LGR brachistochrone transcription/time-cost evidence,
    plus
    `lgr_brachistochrone_dense_sqp_restores_perturbed_cycloid`
    for feature-gated moderate-mesh fixed-time LGR brachistochrone dense-SQP
    free-control feasibility recovery from a perturbed cycloid control
    decision, plus
    `lgr_fixed_mesh_ocp_dense_sqp_wrapper_restores_scalar_integrator_and_is_repeat_deterministic`
    for feature-gated fixed-time dense SQP restoration, plus
    `lgr_fixed_mesh_ocp_dense_constrained_sqp_wrapper_consumes_nonlinear_hard_path_rows`
    for feature-gated constrained dense SQP nonlinear hard-path row consumption,
    plus
    `lgr_fixed_mesh_ocp_emits_optimized_reference_profile_and_iload_payload`
    for optimized reference-profile/I-load output, plus
    `lgr_fixed_mesh_ocp_sparse_nonlinear_path_jacobian_matches_linear_reference`
    for sparse nonlinear path derivatives; general nonlinear benchmark solver closure
    remains open).
  - ph relative-error estimate `e_k` with degree-raise vs subdivide logic
    (**partial satisfied:**
    `lgr_mesh_refinement_accepts_exact_candidate_against_reference` and
    `lgr_mesh_refinement_recommends_degree_raise_then_split`;
    `lgr_mesh_refinement_application_raises_degree_and_preserves_seed` and
    `lgr_mesh_refinement_application_splits_segment_and_preserves_seed` seed
    refined transcriptions from those recommendations; and
    `lgr_mesh_refinement_loop_refines_until_reference_converges` plus
    `lgr_mesh_refinement_loop_reports_iteration_cap_without_convergence` cover
    the reference-driven estimate/apply loop. Solved-reference generation from a
    full LGR OCP backend remains open).
  - Smooth-arc error decays faster than any algebraic order (spectral check) vs
    the HS O(h⁴) baseline, in a tolerance table (**partial satisfied:**
    `lgr_interpolation_error_is_spectral_on_smooth_arc` reads
    `scenarios/trajopt-two-body-apogee/lgr-spectral-convergence-tolerance.toml`
    and checks LGR interpolation error collapse for `x(t)=exp(t)` with declared
    ratio and finest-error tolerances; solved-OCP spectral convergence remains
    open).
  - **Goddard max-altitude (singular arc):** ph-mesh resolves the bang-singular-
    bang structure; final altitude within the published-solution band (Betts /
    PHR ph paper) with provenance (**partial satisfied:**
    `lgr_goddard_cops_benchmark_pins_dynamics_and_singular_arc` pins the
    COPS/YAPSS dimensionless dynamics and singular-arc relation,
    `lgr_goddard_cops_fixed_mesh_seed_reports_dynamics_defect` and
    `lgr_goddard_seed_residual_localizes_split_refinement` account for the seed
    residual and localize a split-refinement recommendation, and
    `lgr_goddard_terminal_altitude_objective_dense_sqp_improves_seed` plus
    `lgr_goddard_split_refined_seed_feeds_terminal_altitude_objective_dense_sqp`
    prove fixed-mesh and split-refined terminal-altitude dense-SQP handoff
    improves altitude and equality residual from the seed, while
    `lgr_goddard_split_refined_terminal_objective_reports_published_altitude_gap`
    pins the COPS final-time/final-altitude band and reports the current
    fixed-time handoff as not closed; published final-altitude band closure
    remains open).
  - Costate estimate matches the PMP shooting costate (forward-link to T6) within
    mesh tolerance on a case with a known analytic costate (**partial
    satisfied:** raw LGR differential-residual multipliers are split by
    collocation sample and exposed as raw collocation-costate/Hamiltonian
    diagnostics by
    `lgr_fixed_mesh_ocp_slices_kkt_multipliers_and_reports_costate_hamiltonian_diagnostics`;
    analytic PMP shooting match and final covector sign/scaling convention remain
    open).
  - All §2 gates green.
- **validation_label:** `research` (Goddard + spectral-convergence benchmark)
- **dual_use_note:** Same boundary lock; the costate is an optimality diagnostic,
  not a target. §6.2 tripwire covers any new target/condition types.
- **est_effort:** ~4–6 wk on top of T2
- **parity_ceiling:** Smooth-problem spectral advantage only; nonsmooth arcs rely
  on h-refinement robustness; not flight-validated.

---

### WP-07.4-a — Shared deterministic conic-solve surface (boundary review)

- **title:** Factor `deterministic_settings()` + CSC builders into a portable home.
- **goal:** Let `openbmp-trajopt` (host) and `openbmp-fc` (portable) share the
  pinned Clarabel settings and cone-matrix builders **without** creating a
  `trajopt → fc` or `fc → trajopt` edge — preserving the FC portability lock.
- **fidelity_tier:** T4 (enabler)
- **depends_on:** [WP-07.2]
- **new_crates:** `openbmp-convex` L2 (portable) — implemented as the shared
  deterministic Clarabel settings and CSC row-builder home.
- **touched:** `Cargo.toml`, `crates/openbmp-convex/`,
  `crates/openbmp-fc/{Cargo.toml,src/mpc.rs,src/landing.rs}`,
  `crates/openbmp-trajopt/{Cargo.toml,src/sqp.rs}`.
- **status:** **satisfied.** `openbmp-convex` exposes `ClarabelSettings`,
  `deterministic_clarabel_settings`, `ConvexMatrixError`, and
  `ClarabelCscMatrixBuilder` behind its `conic` feature. `openbmp-fc` preserves
  `mpc::ClarabelSettings` and `mpc::deterministic_settings` by delegating to the
  shared helper, and `landing.rs` assembles its SOCP epigraph matrix through the
  shared CSC builder. `openbmp-trajopt` preserves
  `TrajoptClarabelSettings` and `deterministic_sqp_settings` by delegating to
  the same helper. This is traced by `REQ-TRAJOPT-007`.
- **approach:** §3.1. Either a tiny portable `openbmp-convex` crate or a re-export
  from `openbmp-models`. The settings constant and CSC helpers move down; both
  consumers depend on the portable home; `openbmp-trajopt` depends on `clarabel`
  directly (host-side, no FC concern).
- **acceptance:**
  - `fc_dependency_tripwire.rs` green; **no** new `openbmp-fc → openbmp-trajopt`
    or reverse edge.
  - `mpc.rs`/`landing.rs` behavior-preserving solver tests remain green.
  - The new crate's layer/edges are justified here and in `REQ-TRAJOPT-007`.
  - All §2 gates green.
- **validation_label:** `checked` (refactor, behavior-preserving)
- **dual_use_note:** Placement lock — the shared surface is the *cone primitive*,
  flowing FC → portable, never `trajopt → fc`.
- **est_effort:** ~3–5 days
- **parity_ceiling:** Plumbing only; validates no trajectory.

---

### WP-07.4 — LCvx min-fuel powered-descent-to-STATE SOCP

- **title:** Acikmese-Ploen lossless-convex min-fuel descent on the existing cone.
- **goal:** Deterministic, real-time-grade powered-descent / ascent-**to-state**
  guidance — the single highest-ROI capability — built on `landing.rs`'s
  `SecondOrderConeT` surface. G-FOLD-style two-phase to a **STATE/altitude**.
- **fidelity_tier:** T4
- **depends_on:** [WP-07.2, WP-07.4-a]
- **new_crates:** none (or `openbmp-convex` from WP-07.4-a)
- **touched:** `crates/openbmp-trajopt/src/` (+ `lcvx.rs`); after WP-07.4-a,
  reuse the shared portable cone primitive instead of duplicating settings.
- **status:** **satisfied.** `crates/openbmp-trajopt/src/lcvx.rs` now exposes the
  feature-gated `LcvxTerminalStateTarget`, `LcvxControlDiscretization`,
  `LcvxPoweredDescentProblem`, `LcvxPoweredDescentSolution`,
  `LcvxTaylorThrustBounds`, and
  `solve_lcvx_powered_descent_fixed_time` fixed-time SOCP substrate, plus
  `LcvxTimeOfFlightSearchSettings`, `LcvxTimeOfFlightSearchReport`, and
  `solve_lcvx_powered_descent_time_of_flight_search` as a deterministic
  golden-section outer loop over time of flight. It consumes the shared
  deterministic Clarabel settings from WP-07.4-a and proves a toy vertical
  powered-descent-to-state solve with tight `sigma = ||u||`, cone-violation
  diagnostics, fixed-time and time-search repeated-solve determinism,
  optional ZOH interval-control or FOH node-control dynamics, optional mass-log
  Taylor thrust-force bounds with linear upper rows and SOC lower rows,
  time-search bracket shrinkage with midpoint objective improvement,
  malformed-input/settings rejection, a known-solution FOH node-control
  vertical-descent regression, a provenance-pinned Acikmese-Ploen Mars
  glideslope benchmark whose final mass is within `0.5%` of the published
  fuel-use result with strict `< 1e-6` feasibility/lossless diagnostics and
  repeated-solve bit determinism, and a compile-fail target-vocabulary
  tripwire. This slice is traced by `REQ-TRAJOPT-006`.
- **approach:** §3.3 T4. Assemble the discretized SOCP: FOH dynamics matrices,
  mass-log change of variables `(u, σ, z)`, the second-order Taylor thrust-bound
  linearization about `z0`, glideslope + pointing SOC cones, boundary at the site
  **radius/altitude/state**. `min −z_N`. Golden-section outer loop over free
  `t_f`. Solve with `clarabel` + `deterministic_settings()`.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - **Acikmese-Ploen / G-FOLD Mars case:** final mass within `< 0.5%` of the
    published optimal (JGCD 2007 / TCST 2013), in a tolerance table with the case
    parameters in a provenance-pinned data file.
  - Glideslope/pointing/thrust-bound violations `< 1e-6` (feasibility).
  - Lossless check: optimal `σ = ‖u‖` (relaxation tight) to `< 1e-6`.
  - Byte-stable across two runs (deterministic settings).
  - **New compile-fail tripwire** (§6.5) proving the LCvx target type carries a
    **radius/altitude/state**, with **no** lat/lon/range/aimpoint field.
  - All §2 gates green.
- **validation_label:** `research` (published G-FOLD case)
- **dual_use_note:** **The most dual-use-sensitive WP.** Target is a
  radius/altitude/state, refused as a ground aimpoint at compile time (§6.1,
  §6.2, §6.5). G-FOLD phase-1 feasibility targets a state/divert region, not a
  designated impact coordinate. I-load records the objective/terminal class.
- **est_effort:** ~2–3 wk
- **parity_ceiling:** Convex (point-mass + log-mass) only; no 6-DoF/aero (that is
  T5); no WCET / flight-cert (§1.2 item 2); not flight-validated.

---

### WP-07.5 — SCvx nonconvex 6-DoF / aero-coupled descent & entry

- **title:** Successive convexification with virtual control + trust region.
- **goal:** In-house nonconvex guidance for nonlinear 6-DoF / aero-coupled
  descent and entry with state-triggered constraints — the genuine "production
  nonconvex guidance" deliverable, with honest convergence diagnostics.
- **fidelity_tier:** T5
- **depends_on:** [WP-07.4]
- **new_crates:** none
- **touched:** `crates/openbmp-trajopt/src/` (+ `scvx.rs`, `discretize.rs`).
- **status:** **partial (2026-06-26).** `src/scvx.rs` now exposes the
  feature-gated `ScvxTerminalStateTarget`, `ScvxAffineDynamicsSegment`,
  `ScvxLinearStateFunction`, `ScvxLinearControlFunction`, `ScvxStateTriggeredConstraint`,
  `ScvxLinearStateConstraint`,
  `ScvxLinearControlConstraint`,
  `ScvxSecondOrderConeStateConstraint`,
  `ScvxSecondOrderConeControlConstraint`,
  `ScvxDiscreteAffineProblem`, `ScvxForwardEulerFohProblemDefinition`,
  `ScvxSettings`,
  `ScvxForwardEulerLinearizationSettings`, `ScvxContinuousFohSettings`,
  `ScvxContinuousFohIterationReport`, `ScvxContinuousFohSolution`,
  `ScvxContinuousFohTimeOfFlightSearchSettings`,
  `ScvxContinuousFohTimeOfFlightCandidateReport`,
  `ScvxContinuousFohTimeOfFlightSearchReport`,
  `ScvxContinuousFohTimeOfFlightSweepSettings`,
  `ScvxContinuousFohTimeOfFlightSweepWindowReport`,
  `ScvxContinuousFohTimeOfFlightSweepReport`,
  `ScvxRigidBodyStateLayout`,
  `ScvxRigidBodyControlLayout`, `ScvxRigidBodyDynamicsParameters`,
  `ScvxRigidBodyDynamicsSettings`, `ScvxPoweredLandingStateLayout`,
  `ScvxPoweredLandingControlLayout`,
  `ScvxPoweredLandingDynamicsParameters`,
  `ScvxPoweredLandingEnvelopeLimits`, `ScvxPoweredLandingEnvelopeSample`,
  `ScvxPoweredLandingEnvelopeReport`,
  `ScvxPoweredLandingLinearEnvelopeConstraints`,
  `ScvxPoweredLandingLinearizedEnvelopeSettings`,
  `ScvxPoweredLandingConicEnvelopeConstraints`,
  `ScvxPoweredLandingConicEnvelopeSettings`, `ScvxReducedAeroCoefficients`,
  `ScvxReducedAeroParameters`, `ScvxReducedAeroForceMoment`,
  `ScvxReducedAeroReferenceProfile`, `ScvxTimeOfFlightSearchSettings`,
  `ScvxTimeOfFlightCandidateReport`, `ScvxTimeOfFlightSearchReport`,
  `ScvxTimeOfFlightSweepSettings`, `ScvxTimeOfFlightSweepWindowReport`,
  `ScvxTimeOfFlightSweepReport`, `ScvxTerminationStatus`,
  `ScvxIterationReport`, `ScvxSolution`,
  `build_scvx_forward_euler_foh_problem`,
  `solve_scvx_forward_euler_foh`,
  `solve_scvx_continuous_foh_time_of_flight_search`,
  `solve_scvx_continuous_foh_time_of_flight_sweep`,
  `solve_scvx_forward_euler_foh_with_reference_projection`,
  `solve_scvx_forward_euler_foh_with_reference_update`,
  `build_scvx_rigid_body_foh_problem`,
  `build_scvx_powered_landing_foh_problem`,
  `build_scvx_reduced_aero_powered_landing_foh_problem`,
  `build_scvx_reduced_aero_rigid_body_foh_problem`,
  `linearize_scvx_forward_euler_foh`, `evaluate_scvx_rigid_body_dynamics`,
  `evaluate_scvx_powered_landing_dynamics`,
  `evaluate_scvx_powered_landing_envelope`,
  `build_scvx_powered_landing_linear_envelope_constraints`,
  `build_scvx_powered_landing_linearized_envelope_constraints`,
  `build_scvx_powered_landing_conic_envelope_constraints`,
  `evaluate_scvx_reduced_aero_force_moment`,
  `evaluate_scvx_reduced_aero_powered_landing_dynamics`,
  `evaluate_scvx_aero_coupled_rigid_body_dynamics`,
  `evaluate_scvx_continuous_foh_time_of_flight_candidate`, and
  `evaluate_scvx_time_of_flight_candidate`,
  `solve_scvx_time_of_flight_search`, `solve_scvx_discrete_affine`
  substrate, plus feature-gated
  `ScvxAeroMethodReferenceProfile`,
  `build_scvx_aero_method_powered_landing_foh_problem`,
  `build_scvx_aero_method_rigid_body_foh_problem`,
  `evaluate_scvx_aero_method_force_moment`,
  `evaluate_scvx_aero_method_coupled_rigid_body_dynamics`, and
  `evaluate_scvx_aero_method_powered_landing_dynamics` when the
  `openbmp-trajopt/aero` feature is enabled. `ScvxTerminalStateTarget` also
  carries optional finite non-negative per-component weights so published
  cases can leave components such as final mass free while still using a local
  state-vector target. The current solver
  assembles deterministic dense QP subproblems, upgraded to conic QP/SOCP subproblems when second-order-cone rows are present over node states, FOH node
  controls, and interval virtual controls; applies initial-state equality,
  weighted terminal objective/convergence norms, control bounds, box trust
  regions, always-active per-node linear state/control path constraints,
  always-active per-node second-order-cone state/control path constraints, and
  active-on-reference state-triggered linear constraints; and emits
  accepted/rejected iteration diagnostics
  including `rho`, trust-region radius, merit, terminal error,
  virtual-control norm, state-triggered violation, linear-state violation,
  linear-control violation, and second-order-cone violation. The fixed-time
  candidate diagnostic helper reports objective, terminal-error,
  virtual-control, linear-state, linear-control, second-order-cone, iteration,
  and termination diagnostics for one candidate even when it does not converge.
  The time-of-flight search wrapper applies deterministic golden-section search
  over caller-built fixed-time SCvx problems, ranks converged candidates by merit plus a
  non-negative time penalty, and reports bracket/candidate diagnostics including
  terminal-error and virtual-control norms for every accepted candidate. The
  continuous-FOH time-of-flight wrapper applies the same deterministic search
  over caller-built fixed-time continuous relinearization solves, ranks only
  converged continuous candidates, and reports candidate outer-loop,
  terminal-error, virtual-control, linear-path, and cone diagnostics; its sweep
  wrapper records each continuous window's search report or stable failure
  reason plus optional failed fixed-time continuous-candidate diagnostics. The
  time-of-flight sweep wrapper evaluates an ordered set of center-time windows,
  records each window's search report or stable failure reason plus optional
  failed-candidate diagnostics, and ranks the best converged window without
  hiding partial non-convergence. The
  continuous FOH solver repeatedly rebuilds a forward-Euler/FOH affine problem
  around the accepted trajectory, solves the existing discrete-affine SCvx
  subproblem, reports outer-loop reference-update norms, and stops once the
  affine solve and reference updates satisfy their declared tolerances. Its
  projection and full-reference update variants let callers normalize state
  references and refresh reference-dependent path rows before each affine
  rebuild. The
  FOH problem builders validate finite state/control/time references, local
  state-vector targets with optional non-negative weights, ordered controls,
  state-triggered rows, linear state/control path rows, second-order-cone
  state/control path rows, rigid-body and powered-landing layouts, per-node
  reduced-aero profiles, and
  feature-gated per-node `openbmp-aero` method contexts, then assemble generic
  continuous-dynamics, flat rigid-body, 14-state mass-depleting body-thrust
  powered-landing, reduced-aero-coupled rigid-body, reduced-aero-coupled
  powered-landing, and aero-method-coupled rigid-body and powered-landing
  reference trajectories into discrete affine SCvx problems. The
  linearization helper
  central-differences caller-provided continuous dynamics at each reference
  state and midpoint control, then emits forward-Euler row-major `A`, FOH
  `B^-`/`B^+`, and affine-offset segments. The flat rigid-body helper evaluates
  a 13-state/6-control 6-DoF model with ECI position/velocity, body-to-ECI
  `wxyz` quaternion, body-frame angular velocity, ECI force, body-frame moment,
  mass, body inertia, inertia-rate coupling, and external acceleration,
  matching the simulator's quaternion kinematics and Euler angular-acceleration
  convention. The powered-landing helper evaluates the Szmuk-style
  mass-depleting 6-DoF body-thrust model with thrust rotation, thrust-arm
  torque, gravity, mass-flow coefficient, quaternion kinematics, and Euler
  angular acceleration. The powered-landing envelope evaluator checks finite
  trajectory samples against mass, thrust, body-thrust gimbal, body tilt,
  angular-rate, and glideslope bounds using the gravity-defined local-up axis
  and reports per-sample plus maximum violations. The powered-landing
  linear-envelope builder emits per-node mass-bound, glideslope-halfspace,
  axial-thrust, and gimbal-halfspace rows that feed the existing linear
  state/control constraint QP path. The reference-linearized envelope builder
  extends those rows with finite-facet upper-thrust-norm rows, finite-facet
  angular-rate-norm rows, and one body-tilt row linearized around each reference
  state. The conic-envelope builder emits exact SOC rows for glideslope,
  upper thrust norm, body-thrust gimbal, and angular-rate norm, keeps mass
  bounds and minimum axial thrust on linear rows, and keeps body tilt
  reference-linearized. Full published trajectory closure remains open. The
  reduced-aero helper mirrors the `openbmp-aero` schema-1
  `CN/CD/CM` convention, rotates body force into ECI with the SCvx quaternion,
  and adds aero force/moment into the rigid-body control vector for
  aero-coupled dynamics; it also feeds the powered-landing mass-depleting
  body-thrust dynamics by adding aero ECI force to translational acceleration
  and aero body moment to angular acceleration while keeping mass depletion
  thrust-only. The feature-gated aero-method bridge consumes any
  `openbmp-aero::AeroMethod` result, including deck, hypersonic, or hybrid
  providers, and feeds it into the same SCvx rigid-body and powered-landing
  load paths without a simulator/runner/scenario dependency. Focused tests
  cover scalar
  affine-integrator convergence with negligible virtual control,
  repeated-solve bit determinism, inactive/active state-triggered behavior,
  always-active linear state/control path enforcement, second-order-cone
  state/control path enforcement, and
  malformed-input rejection, weighted terminal targets leaving zero-weight
  components untargeted, deterministic free-final-time bracket shrinkage and
  repeatability on a scalar integrator, deterministic continuous-FOH
  free-final-time bracket shrinkage with per-candidate outer-loop diagnostics
  plus sweep partial-failure diagnostics on a scalar integrator, provenance-pinned Szmuk in-plane
  powered-landing free-final-time bracket shrinkage to the published agreement
  tolerance with per-candidate terminal/virtual-control diagnostics,
  deterministic scalar time-sweep partial-failure diagnostics, provenance-pinned
  Szmuk initial-time guess sweep diagnostics over every declared initial-time
  window, continuous
  FOH outer-loop reference refresh to convergence, affine continuous-dynamics
  linearization numerics, rigid-body quaternion/Euler dynamics numerics,
  rigid-body linearization handoff evidence, powered-landing dynamics numerics,
  powered-landing FOH problem assembly, continuous-FOH solve convergence on a
  14-state vertical-translation powered-landing toy with terminal/virtual-control
  residual and solved-envelope tolerance evidence, provenance-pinned
  Szmuk-Acikmese 6-DoF free-final-time constants and powered-landing reference
  handoff with final mass untargeted plus envelope-bound satisfaction, powered-landing
  linear, reference-linearized, and conic envelope row generation, solver-row
  handoff, and full-budget conic-envelope affine-solve evidence that closes
  the terminal residual, reports the remaining virtual-control gap, and
  enforces envelope rows on the pinned Szmuk reference, one-window
  conic-envelope free-final-time sweep diagnostics that report candidate
  non-convergence on the virtual-control gap and retain fixed-time
  terminal/virtual-control/linear/cone rows directly on the failed window, a pinned
  in-plane single-linearization affine FOH solve that reduces
  virtual-control/dynamics-defect norm, proves raw quaternion drift blocks
  strict powered-landing relinearization until the exported reference-state
  quaternion projection helper normalizes the handoff, proves the continuous
  FOH reference-projection hook reaches the next Szmuk relinearization instead
  of erroring on that drift, proves the continuous FOH full-reference update
  hook refreshes conic envelope rows across Szmuk powered-landing
  relinearizations, reduces the conic continuous-FOH virtual-control gap under
  a relaxed affine tolerance, reports the remaining reference-update gap
  honestly, proves the continuous-FOH free-final-time fixed-candidate diagnostic
  reports that same conic reference-update gap at the pinned Szmuk center time,
  proves failed continuous-FOH free-final-time sweep windows retain center
  fixed-candidate diagnostics across every declared Szmuk initial-time window,
  and continues reporting normalized-envelope violations honestly,
  direct quaternion projection helper normalization and
  malformed-input rejection, separate
  powered-landing envelope violation reporting, rigid-body FOH problem assembly,
  reduced-aero rigid-body and powered-landing FOH problem assembly with load
  propagation, reduced-aero force/moment mapping and ECI rotation,
  aero-coupled rigid-body and powered-landing load addition,
  `openbmp-aero::DeckLookup` bridge, rigid-body and powered-landing FOH problem
  assembly, and rigid-body plus powered-landing coupled-dynamics evidence,
  malformed linearization/aero/builder rejection, plus a
  compile-fail target-vocabulary tripwire for the SCvx terminal target type.
  This slice is traced by `REQ-TRAJOPT-008` and does not yet claim full
  published Szmuk trajectory closure or flight-real-time validation.
- **approach:** §3.3 T5. FOH discretization of linearized dynamics (augmented STM
  / matrix-exponential sub-interval integration for `A_i, B_i^±, S_i, R_i`);
  virtual control `ν_i` with weight `w_ν`; trust region with the
  actual-vs-predicted ratio `ρ` acceptance loop; quaternion attitude kinematics
  (`nalgebra`) + inertia(mass) coupling; state-triggered constraints via the
  continuous `min(−h,0)·g ≤ 0` encoding. Reuse `clarabel` + the T4 discretization
  utilities. Expose `‖ν‖`, `ρ`, and the iteration trace as outputs.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - **Szmuk 6-DoF free-final-time case:** trajectory matches the published
    solution (AIAA 2018-0617 / arXiv:1811.10803) within stated tolerance **where
    it converges**; `‖ν‖ < 1e-6` at convergence; report non-convergence honestly
    (no boolean hiding) per §1.3.
  - State-triggered constraint correctly inactive above / active below the
    trigger (test case).
  - Byte-stable across two runs; convergence diagnostics emitted.
  - **§6.2 tripwire** extended to any new SCvx target/constraint types.
  - All §2 gates green.
- **validation_label:** `research` where it converges; `experimental` on
  aero-coupled entry cases that do not (state it explicitly).
- **dual_use_note:** Same locks; "landing-error" / divert phases target a
  state/region, never a ground coordinate. Adds the proving tripwire.
- **est_effort:** ~4–8 wk
- **parity_ceiling:** Convergence on aero-coupled 6-DoF entry is uncertain (§1.3);
  no WCET / flight-cert; not flight-validated.

---

### WP-07.6 — Closed-loop wiring + PMP indirect optimality witness

- **title:** Feed optimizer output to the FC as a tracked reference; add the PMP
  costate-shooting CI verifier.
- **goal:** Complete the pipeline — optimizer → validated I-load → FC autopilot
  tracked reference — and add a rigorous "verified optimal" regression gate so
  the team can claim optimality, not merely "NLP converged".
- **fidelity_tier:** T6
- **depends_on:** [WP-07.2 (KKT multipliers), WP-07.4 (or any optimized
  reference), WP-07.3 (costate cross-check, optional)]
- **new_crates:** none
- **touched:** `crates/openbmp-trajopt/src/` (`pmp.rs` partial witness landed;
  `iload.rs` objective-class metadata extension landed; feature-gated `sqp.rs`
  dense-QP dual exposure landed; `collocation.rs`/`pseudospectral.rs` raw
  costate-to-PMP convention adapters landed);
  `crates/openbmp-fc/src/guidance.rs` /
  `topics.rs` **only on the consuming side** (the FC re-validates the I-load — no
  `trajopt` edge);
  `crates/openbmp-runner/tests/fixtures/fc-reference-telemetry-optimized-reference.toml`,
  `crates/openbmp-runner/tests/fixtures/fc-reference-telemetry-optimized-reference-iload.toml`,
  and `crates/openbmp-runner/tests/fc_reference_telemetry.rs` (inline plus
  PMP-derived file-backed optimized-reference closed-loop runner evidence);
- **status:** **partial.** `pmp.rs` now exposes `PmpHamiltonianSample`,
  `PmpHamiltonianReport`, `PmpHamiltonianProfile`,
  `PmpHamiltonianControlStationaritySettings`,
  `PmpHamiltonianControlStationaritySample`,
  `PmpHamiltonianControlStationarityReport`,
  `PmpCostateComparisonSettings`, `PmpCostateComparisonSample`,
  `PmpCostateComparisonReport`,
  `PmpScalarMinEnergyProblem`, `PmpScalarMinEnergyShootingSettings`,
  `PmpScalarMinEnergyIteration`, `PmpScalarMinEnergySolution`,
  `evaluate_pmp_hamiltonian_profile`,
  `evaluate_pmp_hamiltonian_control_stationarity`,
  `evaluate_pmp_hamiltonian_control_stationarity_from_report`,
  `compare_pmp_costate_profiles`, and `solve_pmp_scalar_min_energy_shooting`.
  The generic witness evaluates `H = L + lambda^T f` along finite local
  state/control/costate profiles and reports maximum Hamiltonian drift from the
  first sample. The control-stationarity witness central-differences `H` with
  respect to each control component, reports per-sample `dH/du` gradients,
  maximum gradient norms, and a tolerance verdict, with focused tests proving
  zero stationarity residuals on the scalar min-energy optimum and nonzero
  residuals for off-optimum controls. The costate-comparison
  helper compares same-mesh PMP and direct-method/KKT costate profiles with
  finite time/costate tolerances. The scalar validated-toy shooting witness solves
  the fixed-time minimum-energy integrator by deterministic costate shooting,
  reports per-iteration terminal residuals, returns a Hamiltonian constancy
  report, proves PMP-vs-KKT same-mesh costate agreement on the analytic arc, and
  proves repeated-solve bit determinism. `PmpDirectCostateConvention` and
  `apply_pmp_direct_costate_convention` require an explicit finite nonzero
  raw-to-PMP scale before comparing raw direct-method costate estimates, and the
  Hermite-Simpson node-costate and LGR collocation-costate reports expose
  `pmp_costate_convention_report` adapters into that shared PMP comparison
  shape; focused tests cover generic convention scaling, an HS raw node-costate
  bridge against the scalar PMP arc, and LGR raw collocation-costate scaling. The
  feature-gated dense-QP lane now
  exposes equality/inequality dual vectors, and a scalar fixed-time
  minimum-energy dense-QP equality multiplier is checked against the PMP costate
  on the same mesh with the expected sign/scaling convention. The
  Hermite-Simpson dense-QP wrapper now feeds solver-produced equality duals
  through raw KKT multiplier splits and an explicit PMP convention report that
  matches the scalar PMP shooting costate on the same mesh; it also exposes
  `dense_qp_pmp_hamiltonian_report` for constant scalar PMP Hamiltonian samples
  from the solved state/control profile and round-trips that Hamiltonian/costate
  certificate in a validated I-load payload. The LGR dense-QP wrapper now also
  exposes a quadrature-weight-normalized PMP convention report, matches the same
  scalar PMP costate at collocation nodes, and exposes
  `dense_qp_pmp_quadrature_weight_normalized_hamiltonian_report` for the same
  scalar PMP Hamiltonian constancy, with the same certificate round-trip through
  a validated I-load payload. The Hermite-Simpson and LGR sparse
  augmented-KKT reports also expose `sparse_kkt_pmp_costate_convention_report`
  and `sparse_kkt_pmp_hamiltonian_report`; constrained sparse-KKT step reports
  feed equivalent residual-multiplier/costate/Hamiltonian diagnostic adapters,
  including public Bryson-Denham active-set PMP diagnostic handoff evidence and
  PMP Hamiltonian adapters, while LGR sparse-KKT additionally exposes
  `sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report` and
  `sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report`; current
  tests run identity convention reports against the existing raw Hamiltonian
  diagnostics, exercise HS/LGR public Bryson-Denham active constrained-KKT
  diagnostic and direct certificate/I-load handoffs, and check the LGR
  weight-normalized route to prove state/control
  sampling and common PMP evaluator wiring without asserting final production
  scaling. The residual-only sparse-KKT SQP reports now give Hermite-Simpson and
  LGR scalar/nonlinear hard-path feasibility evidence plus accepted-iterate
  final multipliers that feed the same PMP/KKT report path
  for validated-toy residual-multiplier closure evidence. Accepted sparse-KKT
  SQP report surfaces now build HS/LGR PMP/KKT certificates, prefer final
  constrained active-row multipliers when available, and round-trip those
  certificates through optimized-reference I-load payloads with optional
  validated same-mesh costate-comparison summaries and report-derived
  control-stationarity summaries attached to the I-load optimality certificate.
  Direct active constrained-KKT report surfaces also build HS/LGR PMP/KKT
  certificates and round-trip public Bryson-Denham active-set certificates
  through optimized-reference I-load payloads.
  These tests prove the
  handoff path without inferring production covector scaling, and the constrained
  sparse-KKT diagnostic adapters close the active scalar hard-path
  step-report-to-PMP handoff without claiming production constrained
  large-sparse NLP optimality. Validated-toy
  runner scenarios now select inline `guidance = "optimized_reference"` and a
  resolved postcard I-load file generated from the scalar PMP minimum-energy
  witness, plus Hermite-Simpson dense-SQP and sparse-SQP PMP/KKT certified
  I-load files, export valid reference telemetry, and emit FC actuator
  command-stream reports, proving inline and certified file-backed
  closed-loop optimized-reference paths through the runner.
  `ILoadHeader` now
  records `objective_class` via `ILoadObjectiveClass`, `new_with_objective` lets
  optimized-reference and day-of-launch recentering paths set explicit audit
  labels, `ILoadOptimalityCertificate` lets PMP-derived I-loads carry
  fail-closed Hamiltonian/control-stationarity/costate tolerance evidence, and
  `optimized_reference_iload_payload_with_optimality_certificate` lets
  Hermite-Simpson and LGR optimized-reference emitters attach that evidence to
  validated postcard payloads. Dense-SQP final sparse-KKT certificate reports
  now let accepted Hermite-Simpson and quadrature-normalized LGR dense-SQP
  solutions attach validated PMP/KKT Hamiltonian certificates to
  optimized-reference postcard payloads and can attach validated same-mesh
  PMP-vs-KKT costate summaries plus validated control-stationarity summaries
  through `with_pmp_control_stationarity`. Sparse-SQP final report surfaces expose
  the same validated certificate payload path, preferring constrained final
  active-row reports when available and the same optional costate-comparison
  and control-stationarity attachment paths, and the runner file-backed scenario
  consumes Hermite-Simpson dense-SQP and sparse-SQP final sparse-KKT certified
  I-loads through the normal closed-loop optimized-reference bridge. The schema version is
  `1.3.0`. This is traced by `REQ-TRAJOPT-009`. Production Hermite-Simpson/LGR
  constrained large-sparse NLP residual-multiplier closure and a production
  indirect solver remain open beyond the explicit raw-costate convention
  handoff, validated-toy runner evidence, scalar PMP I-load witness, scalar
  dense-QP equality-dual witness, and dense-SQP/sparse-SQP final sparse-KKT
  certificate surfaces.
- **approach:** §3.3 T6. (a) Emit the optimized reference trajectory + feedforward
  as a postcard I-load (extend `ILoadHeader` with an objective-class field); the
  FC consumes and **re-validates** it, tracking the `ReferenceState`. (b) PMP
  verifier: costate ODE `λ̇ = −∂H/∂x`, primer-vector control law, transversality /
  `H(t_f)=0`; solve the costate TPBVP by Gauss-Newton shooting on `λ(t_0)` (reuse
  the corrector machinery); assert `H` constant on autonomous arcs and costates
  matching the T2/T3 KKT-multiplier estimate.
- **acceptance:**
  - Closed-loop scenario **off by default** behind an explicit config block;
    existing goldens byte-identical.
  - The FC consumes the I-load through the existing validation boundary; **no new
    `openbmp-fc → openbmp-trajopt` edge** (`fc_dependency_tripwire.rs` green).
  - PMP witness: `|H(t) − H(t0)| < 1e-4` on autonomous arcs (tolerance table);
    costate matches KKT-multiplier estimate within mesh tolerance — wired as a CI
    optimality regression test.
  - I-load records the objective + terminal-condition class (audit field).
  - **§6.2/§6.5 tripwire** confirms the closed-loop reference path carries no
    surface-coordinate field end-to-end.
  - All §2 gates green.
- **validation_label:** `validated-toy` (pipeline), `research` (PMP optimality
  regression).
- **dual_use_note:** **The boundary-crossing WP.** Reinforces the placement lock:
  the optimizer output crosses as a pre-validated I-load the FC re-validates,
  never as a live solver computing a target inside the flight loop (§6.3). The
  audit metadata (§6.6) records objective/terminal class so no aimpoint can leak.
- **est_effort:** continuous (PMP verifier ~1–2 wk; wiring ~1 wk)
- **parity_ceiling:** Demonstrates the method pipeline at research fidelity, not
  flight-qualified parity; no WCET on the in-loop tracker (the optimizer stays
  offline by design).

---

## 10. References

**Direct transcription / collocation / sparse NLP**
- J. T. Betts, *Practical Methods for Optimal Control and Estimation Using
  Nonlinear Programming*, 2nd ed., SIAM, 2010 (Ch. 4 transcription; Ch. 1–2
  sparse SQP/IP; §4.5 multiple shooting; §4.11 costate-from-multipliers).
- C. R. Hargraves & S. W. Paris, "Direct Trajectory Optimization Using Nonlinear
  Programming and Collocation," *JGCD* 10(4), 1987 (original Hermite-Simpson).
- M. Kelly, "An Introduction to Trajectory Optimization: How to Do Your Own
  Direct Collocation," *SIAM Review* 59(4), 2017 (HS derivation + pseudocode).
- A. V. Rao, "A Survey of Numerical Methods for Optimal Control,"
  *Adv. Astronaut. Sci.* 135(1), 2009.

**Pseudospectral / ph-mesh**
- M. A. Patterson & A. V. Rao, "GPOPS-II ...," *ACM TOMS* 41(1), Art. 1, 2014,
  https://doi.org/10.1145/2558904.
- Garg, Patterson, Hager, Rao, Benson, Huntington, "A unified framework for the
  numerical solution of optimal control problems using pseudospectral methods,"
  *Automatica* 46(11), 2010.
- Patterson, Hager, Rao, "A ph mesh refinement method for optimal control,"
  *Optimal Control Appl. Methods* 36(4):398–421, 2015,
  https://people.clas.ufl.edu/hager/files/ph.pdf.
- Darby, Hager, Rao, "An hp-adaptive pseudospectral method for solving optimal
  control problems," *OCAM* 32(4):476–502, 2011.

**Multiple shooting**
- H. G. Bock & K. J. Plitt, "A Multiple Shooting Algorithm for Direct Solution of
  Optimal Control Problems," *IFAC Proc.* 1984.
- Stoer & Bulirsch, *Introduction to Numerical Analysis*, Ch. 7.

**Indirect / Pontryagin (cross-check)**
- A. E. Bryson & Y.-C. Ho, *Applied Optimal Control*, 1975.
- D. F. Lawden, *Optimal Trajectories for Space Navigation*, 1963 (primer vector).
- Longuski, Guzman, Prussing, *Optimal Control with Aerospace Applications*,
  Springer, 2014.

**Convex powered descent (LCvx / SCvx)**
- B. Acikmese & S. R. Ploen, "Convex Programming Approach to Powered Descent
  Guidance for Mars Landing," *JGCD* 30(5):1353–1366, 2007.
- B. Acikmese, J. M. Carson, L. Blackmore, "Lossless Convexification of Nonconvex
  Control Bound and Pointing Constraints of the Soft Landing Optimal Control
  Problem," *IEEE TCST* 21(6), 2013.
- L. Blackmore, "Autonomous Precision Landing of Space Rockets," *NAE Frontiers
  of Engineering*, 2016.
- Y. Mao, M. Szmuk, B. Acikmese, "Successive Convexification of Non-Convex
  Optimal Control Problems and Its Convergence Properties," *CDC* 2016,
  arXiv:1608.05133; and "Successive Convexification: A Superlinearly Convergent
  Algorithm ...," arXiv:1804.06539, 2018.
- M. Szmuk & B. Acikmese, "Successive Convexification for 6-DoF Mars Rocket
  Powered Landing with Free-Final-Time," *AIAA SciTech* 2018, 10.2514/6.2018-0617.
- M. Szmuk, T. Reynolds, B. Acikmese, "Successive Convexification for Real-Time
  6-DoF Powered Descent Guidance with State-Triggered Constraints," *JGCD* 2020,
  arXiv:1811.10803.

**Software / oracles**
- Clarabel.rs (oxfordcontrol/Clarabel, Apache-2.0); IPOPT (coin-or, EPL-1.0);
  OpEn / optimization-engine (MIT/Apache); argmin (MIT/Apache); nalgebra; num-dual;
  PSOPT (LGPL, external oracle); NASA GMAT (Apache-2.0, external oracle);
  GPOPS-II (commercial — papers only).
- Open reference impls (study only, do not vendor): EmbersArc/SCvx,
  Natsoulas/lcvx-pdg, Ceaser626/Lossless-convexification.

**Cross-references (this series):** `00-overview.md`,
`13-agent-execution-playbook.md`, `05-propulsion-high-fidelity.md`,
`06-gnc-coupled-mimo-and-control.md`, `08-environment-gravity-and-frames.md`,
`10-flight-software-in-the-loop-xil.md`, `11-monte-carlo-uq-and-validation.md`,
`12-determinism-realtime-and-compute.md`. **In-repo anchors:**
`crates/openbmp-trajopt/src/{corrector.rs,target.rs,iload.rs}`,
`crates/openbmp-fc/src/{landing.rs,mpc.rs}`,
`crates/openbmp-physics/src/profile.rs` (`TerminalCondition`),
`docs/ascent-guidance.md`, `docs/safety-boundaries.md`,
`docs/external-telemetry-validation.md`.
