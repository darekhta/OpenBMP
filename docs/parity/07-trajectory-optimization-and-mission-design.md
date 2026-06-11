# Trajectory Optimization & Mission Design

**Status:** `experimental` (WP-07.0 T0 offline two-body and scenario-backed
corrector driver/CLI paths landed; WP-07.1 STM/multiple-shooting substrate is
partial; higher-tier methods remain design intent).
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
  reference_profile }`, `ILoadSchemaVersion {1,0,0}`, `TerminalConditionKind`
  (the recorded objective/terminal class for audit), `encode_iload_payload`
  validates then `postcard::to_allocvec`. `TrajoptError` covers
  `InvalidPayload`, `Serialize`, `SingularSystem`.
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
- `src/shooting.rs` — a fixed-duration M-segment multiple-shooting continuity
  evaluator. It seeds dynamically consistent two-body nodes, reports stacked
  defects `phi_i(x_i) - x_{i+1}`, and builds the row-major block-bidiagonal
  Jacobian `[STM_i, -I]`. `MultipleShootingCorrector` adds the first solve
  loop by holding endpoints fixed and applying damped Gauss-Newton corrections
  to interior nodes through that STM Jacobian, and now also supports a
  fixed-initial-state terminal-condition correction mode using the closed
  `TerminalCondition` equality vocabulary. This is traced by `REQ-TRAJOPT-002`,
  including a downstream compile-fail tripwire that refuses surface-coordinate
  fields on the multiple-shooting node surface; it is not yet full WP-07.1
  acceptance closure.

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
T0 offline two-body and scenario-backed apogee CLI I-load paths; WP-07.1 has a
partial STM, multiple-shooting continuity, and fixed-endpoint interior-node
correction substrate, fixed-initial terminal-condition correction, and a
no-surface-coordinate compile-fail tripwire; the closed terminal-condition
vocabulary still holds; a Clarabel SOCP epigraph smoke test, a forward
propagator, and a PEG-style closed-loop ascent reference exist. No
multiple-shooting control/free-time solve, NLP transcription, pseudospectral
method, LCvx/SCvx horizon problem, closed-loop wiring of an optimized reference,
or indirect cross-check exists.

---

## 3. Target architecture

### 3.1 Crate strategy

Keep everything **offline / host-side / L4 in `openbmp-trajopt`.** This is a
hard dual-use placement lock (§6). `openbmp-fc` must never gain an edge to
`openbmp-trajopt` (FC portability lock, `00` §3.3; enforced by
`fc_dependency_tripwire.rs`). The convex cone *primitives* live in `openbmp-fc`
(`landing.rs`/`mpc.rs`) because they are FC-portable and already there; the
*horizon assembly* (building the discretized SOCP/SQP matrices) lives in
`openbmp-trajopt`, calling a small portable cone-solve surface. Two routes,
decided in WP-07.4-a:

- **Preferred:** factor the Clarabel call behind a thin
  `openbmp-trajopt`-internal `ConicProblem`/`ConicSolve` wrapper that takes the
  same CSC/cone description `landing.rs` uses, so `openbmp-trajopt` depends on
  `clarabel` directly (it is host-side, no FC lock concern) and shares the
  *deterministic settings constant* by re-exporting it from a portable location.
- **Avoid:** making `openbmp-trajopt` depend on `openbmp-fc`. Do not; lift the
  shared `deterministic_settings()` into a portable home (e.g.
  `openbmp-models` or a tiny `openbmp-convex` portable crate) if reuse is needed,
  reviewed in the WP that first needs it.

New crate, proposed and reviewed before building:

| Crate | Layer | Purpose | Doc gate |
|---|---|---|---|
| `openbmp-convex` (optional) | L2 (portable) | shared deterministic conic-solve settings + CSC builders reused by `openbmp-fc` (FC-portable) and `openbmp-trajopt` (host) | WP-07.4-a |

No new crate is *required* if the shared-settings refactor is small; the WP
proposes the boundary and the reviewer rules before code lands (`13` §5).

`Cargo.toml` additions (host-side only; never reachable from `openbmp-fc`):
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
  terminal_condition_kind, synthesis_seed, producer }` + `SynthesisMetadata
  { scenario_digest, source_revision, method }`. **Extend the header to record
  the objective class and terminal-condition class** so an audit can confirm no
  aimpoint leaked in (§6.5). Any benchmark constants (Goddard parameters,
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
- **implementation_status:** partial. `src/stm.rs` integrates deterministic
  two-body variational equations with a state-transition matrix and verifies
  the STM against complex-step sensitivities, and
  `src/shooting.rs` evaluates fixed-duration M-segment continuity defects plus
  a block-bidiagonal `[STM_i, -I]` Jacobian. `MultipleShootingCorrector` now
  performs damped fixed-endpoint interior-node correction and reports honest
  non-convergence for inconsistent endpoints; it also performs fixed-initial
  terminal-condition correction against closed `TerminalCondition` equality
  residuals. A trybuild UI test proves the public multiple-shooting node surface
  cannot carry target latitude/longitude fields. Remaining acceptance work: the
  control/free-time part of the full free vector, T0 cross-tier regression
  tolerance table, and box/path penalty handling.
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
- **goal:** A general fixed-mesh OCP solver: min-fuel / min-time ascent with full
  path constraints, cross-checked against T1. The first `research`-label tier
  (MMS order-of-accuracy + known-optimal benchmark).
- **fidelity_tier:** T2
- **depends_on:** [WP-07.1]
- **new_crates:** `openbmp-convex` *(optional, proposed in WP-07.4-a; only if the
  shared-settings refactor is needed first — review the boundary before building)*
- **touched:** `crates/openbmp-trajopt/src/` (+ `collocation.rs`, `sqp.rs`,
  `sparse.rs`); the `Transcription` + `NlpSolver` traits (§3.2);
  `Cargo.toml` (+ `clarabel`, `num-dual`, optional feature-gated `ipopt-sys`).
- **approach:** §3.3 T2. Separated HS defects + Hermite midpoint constraint;
  Simpson Lagrange cost; block-banded sparse Jacobian via `num-dual` forward-mode
  AD or sparsity-colored FD; SQP outer loop with `clarabel` solving the QP
  subproblem and a merit-function line search; canonical-unit (DU/TU) scaling.
  Optional `ipopt` feature flag for the large-sparse backend (EPL boundary
  documented).
- **acceptance:**
  - Off by default; goldens byte-identical.
  - **MMS:** manufactured solution → HS defect residual decays at slope
    `4.0 ± 0.3` (O(h⁴)) on a log-log mesh-refinement sweep, in a tolerance table.
  - Brachistochrone optimal cost within `< 1e-4` relative to closed form;
    Bryson-Denham state-constrained case satisfies the constraint and matches the
    published optimum.
  - Cross-check vs T1 on a shared ascent case within mesh tolerance.
  - clarabel uses the pinned deterministic settings; result byte-stable across
    two runs (determinism lane).
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
- **goal:** GPOPS-II-class accuracy with automatic mesh refinement and costate
  estimates from KKT multipliers (the optimality witness feeding T6), reusing the
  T2 NLP backend.
- **fidelity_tier:** T3
- **depends_on:** [WP-07.2]
- **new_crates:** none
- **touched:** `crates/openbmp-trajopt/src/` (+ `pseudospectral.rs`, `mesh.rs`).
- **approach:** §3.3 T3. LGR nodes via Golub-Welsch (Jacobi-matrix eigenvalues)
  or Newton on `P_N`; barycentric differentiation/integration matrices; integral-
  form collocation; ph relative-error estimate `e_k` with degree-raise vs
  subdivide logic; costate from KKT multipliers (covector mapping). Reuse the T2
  SQP/IPOPT backend.
- **acceptance:**
  - Off by default; goldens byte-identical.
  - LGR nodes/weights match a Gauss-quadrature reference to `< 1e-12`.
  - Smooth-arc error decays faster than any algebraic order (spectral check) vs
    the HS O(h⁴) baseline, in a tolerance table.
  - **Goddard max-altitude (singular arc):** ph-mesh resolves the bang-singular-
    bang structure; final altitude within the published-solution band (Betts /
    PHR ph paper) with provenance.
  - Costate estimate matches the PMP shooting costate (forward-link to T6) within
    mesh tolerance on a case with a known analytic costate.
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
- **new_crates:** `openbmp-convex` L2 (portable) — *proposed; review the layer and
  dependency edges before building (`13` §5). Skip if the refactor is trivial
  enough to keep `deterministic_settings()` re-exported from an existing portable
  crate.*
- **touched:** `crates/openbmp-fc/src/mpc.rs` (move/re-export settings);
  `crates/openbmp-trajopt/Cargo.toml` (+ `clarabel`).
- **approach:** §3.1. Either a tiny portable `openbmp-convex` crate or a re-export
  from `openbmp-models`. The settings constant and CSC helpers move down; both
  consumers depend on the portable home; `openbmp-trajopt` depends on `clarabel`
  directly (host-side, no FC concern).
- **acceptance:**
  - `fc_dependency_tripwire.rs` green; **no** new `openbmp-fc → openbmp-trajopt`
    or reverse edge.
  - `mpc.rs`/`landing.rs` byte-identical solver behavior (their tests unchanged).
  - The new crate's layer/edges justified in the PR (`13` §5) and reviewed.
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
- **touched:** `crates/openbmp-trajopt/src/` (+ `lcvx.rs`); reuse the
  `openbmp-fc` cone primitive via the WP-07.4-a portable home.
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
- **touched:** `crates/openbmp-trajopt/src/` (+ `indirect.rs`, `iload.rs`
  extension for objective-class metadata); `crates/openbmp-fc/src/guidance.rs` /
  `topics.rs` **only on the consuming side** (the FC re-validates the I-load — no
  `trajopt` edge); a closed-loop scenario gated behind an explicit config block.
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
