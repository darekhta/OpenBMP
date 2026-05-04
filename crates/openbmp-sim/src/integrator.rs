//! Numerical integration: [`SimState`] trait, [`Integrator`] trait,
//! and the canonical fixed-step Runge-Kutta 4 implementation
//! [`Rk4FixedStep`].
//!
//! The integrator is generic over [`SimState`]. OpenBMP implements it
//! for [`openbmp_state::PointMassState`] and
//! [`openbmp_state::RigidBodyState`].
//!
//! # Determinism contract
//!
//! `Rk4FixedStep` is tagged [`IntegratorDeterminism::BitStable`]. The
//! contract requires:
//!
//! 1. The locked weighted-sum order of [`rk4_weighted_sum`] —
//!    Phase-3.15.E moved the combiner off the derivative trait
//!    (where it baked in RK4 specifics) into this integrator-side
//!    helper that uses the derivative's primitive `Add` + `Mul<f64>`
//!    ops.
//! 2. No `f64::mul_add` in the integrator hot path.
//! 3. Sub-step times computed by addition from the current step time
//!    (not by accumulating across steps — the kernel handles
//!    cross-step time via canonical multiplication).
//! 4. Post-step `project()` call to apply manifold constraints (no-op
//!    for `PointMassState`; quaternion renormalisation for
//!    `RigidBodyState`).
//!
//! See `docs/software-architecture.md § Determinism Profile` for the
//! rationale (locked weighted-sum order, no FMA, MXCSR guard).

use openbmp_core::{Duration, SimTime};
use openbmp_models::SimStateDerivative;

use crate::error::{IntegratorError, ModelEvalError};

use openbmp_models::SimState;

/// Canonical RK4 weighted-sum helper.
///
/// Computes `(k1 + 2·k2 + 2·k3 + k4) / 6` using the derivative's
/// primitive `Add` and `Mul<f64>` ops, with **locked** evaluation
/// order:
///
/// 1. `k2 * 2` is computed first.
/// 2. `k1 + (k2 * 2)` is the first addend.
/// 3. `k3 * 2` is computed next.
/// 4. The two scaled stages are added: `((k1 + 2·k2) + (k3 * 2))`.
/// 5. `k4` is added last.
/// 6. The whole sum is multiplied by `1/6` once at the end (not
///    pre-distributed across stages — that introduces an extra
///    rounding before the sum).
///
/// Phase-3.15.E moved this off [`SimStateDerivative`] so the
/// derivative trait surface only requires the generic `Add` +
/// `Mul<f64>` primitives. Future integrators (DOPRI5/8, RKF78)
/// implement their own weighted-sum helpers using the same
/// primitives and the same locked-order discipline, without the
/// derivative trait advertising one specific stage scheme.
///
/// Never use `f64::mul_add` here: FMA hardware rounds once;
/// software emulation rounds twice — cross-platform bit-stable
/// replay requires two explicit roundings.
#[must_use]
pub fn rk4_weighted_sum<D>(k1: D, k2: D, k3: D, k4: D) -> D
where
    D: SimStateDerivative,
{
    const TWO: f64 = 2.0;
    const SIXTH: f64 = 1.0 / 6.0;
    // DETERMINISM CONTRACT: explicit parentheses prevent compiler
    // re-association; single final scalar multiplication; no FMA.
    ((k1 + (k2 * TWO)) + (k3 * TWO) + k4) * SIXTH
}

/// Determinism class of an integrator.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum IntegratorDeterminism {
    /// Same input → bit-identical output across reruns on the same
    /// platform profile.
    BitStable,
    /// Same input → bit-identical output across reruns on one
    /// platform profile, but cross-platform byte equality is not
    /// guaranteed. Adaptive-step integrators use this class because
    /// deterministic `pow()` / `ln()` call order still depends on the
    /// platform libm implementation.
    StateStable,
}

/// Numerical integrator trait.
///
/// `S` is the state type the integrator advances. The derivative
/// closure `derive_fn` is called at each Runge-Kutta sub-step with the
/// intermediate state and the corresponding sub-step time. It returns
/// either the derivative or a typed model-evaluation error; the
/// integrator short-circuits before any state mutation on the first
/// failing stage.
pub trait Integrator<S: SimState> {
    /// Determinism class declared by this integrator.
    #[must_use]
    fn determinism(&self) -> IntegratorDeterminism;

    /// Advance `state` by `dt`.
    ///
    /// `derive_fn` is the user-supplied closure that computes
    /// `dy/dt` at the given (state, time) pair. The integrator may
    /// call it multiple times per step at intermediate sub-step states.
    /// A `ModelEvalError` returned by the closure short-circuits the
    /// step (no state mutation, no later RK stages run) and surfaces
    /// as [`IntegratorError::ModelEval`].
    ///
    /// # Errors
    ///
    /// Returns [`IntegratorError::InvalidStep`] if `dt` is not strictly
    /// positive and finite, [`IntegratorError::NonFiniteDerivative`] if
    /// any RK stage produces a `NaN`/`Inf` derivative,
    /// [`IntegratorError::NonFiniteState`] if any start, intermediate,
    /// or final state is not valid for integration, or
    /// [`IntegratorError::ModelEval`] if any stage's derivative
    /// closure reports a typed model failure.
    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>;
}

/// Canonical fixed-step Runge-Kutta 4 integrator.
///
/// 4th-order accurate, single-stage, four derivative evaluations per
/// step. Bit-stable across reruns on the same platform profile.
#[derive(Copy, Clone, Debug, Default)]
pub struct Rk4FixedStep;

impl<S: SimState> Integrator<S> for Rk4FixedStep {
    fn determinism(&self) -> IntegratorDeterminism {
        IntegratorDeterminism::BitStable
    }

    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        let h = dt.as_seconds();
        if !h.is_finite() || h <= 0.0 {
            return Err(IntegratorError::InvalidStep { dt_seconds: h });
        }
        if !state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let half_h = h * 0.5;

        let t0 = state.time();
        let t0_s = t0.as_seconds();
        let t_mid = SimTime::from_seconds(t0_s + half_h);
        let t_end = SimTime::from_seconds(t0_s + h);

        // Stage 1: derivative at the start of the step.
        let k1 = derive_fn(state, t0)?;
        if !k1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 2: midpoint state predicted by k1.
        let mid_k2 = state.advance_by(half_h, &k1);
        if !mid_k2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k2 = derive_fn(&mid_k2, t_mid)?;
        if !k2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 3: midpoint state predicted by k2.
        let mid_k3 = state.advance_by(half_h, &k2);
        if !mid_k3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k3 = derive_fn(&mid_k3, t_mid)?;
        if !k3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 4: end-state predicted by k3.
        let end_k4 = state.advance_by(h, &k3);
        if !end_k4.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k4 = derive_fn(&end_k4, t_end)?;
        if !k4.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Combine: state + h * (k1 + 2 k2 + 2 k3 + k4) / 6.
        // Locked-order weighted sum is now an integrator-local
        // helper (Phase-3.15.E moved it off the derivative trait so
        // future integrators can implement their own combination
        // without the trait surface advertising one stage scheme).
        let weighted = rk4_weighted_sum(k1, k2, k3, k4);
        let mut new_state = state.advance_by(h, &weighted);

        // Apply manifold constraints (no-op for PointMassState;
        // quaternion renormalisation for RigidBodyState).
        new_state.project();

        if !new_state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        Ok(new_state)
    }
}

// ---------------------------------------------------------------------
// Phase-5.D.3 — Dormand-Prince 5(4) fixed-step integrator
// ---------------------------------------------------------------------

/// Dormand-Prince 5(4) Butcher-tableau coefficients used by
/// [`Dopri54FixedStep`].
///
/// Pinned exactly per Dormand, J. R., and Prince, P. J. (1980),
/// *A family of embedded Runge-Kutta formulae*, J. Comp. Appl.
/// Math. 6(1):19-26 — the "DOPRI5" entry, also tabulated in Hairer,
/// Nørsett, and Wanner, *Solving Ordinary Differential Equations I*,
/// 2nd rev. ed., §II.5 Table 5.2 (Springer, 1993). The constants
/// below reproduce that table verbatim.
///
/// The shipped variant is the **5th-order solution only** — the
/// embedded 4th-order solution and the adaptive PI step controller
/// are explicitly deferred (see `docs/phase-5-plan.md § 5.D.3`).
mod dopri54_tableau {
    // c-vector (sub-step times relative to h):
    pub const C2: f64 = 1.0 / 5.0;
    pub const C3: f64 = 3.0 / 10.0;
    pub const C4: f64 = 4.0 / 5.0;
    pub const C5: f64 = 8.0 / 9.0;
    // c6 = 1, c7 = 1 (FSAL-eligible; not exploited by the fixed-step
    // implementation).

    // a-matrix rows.
    pub const A21: f64 = 1.0 / 5.0;

    pub const A31: f64 = 3.0 / 40.0;
    pub const A32: f64 = 9.0 / 40.0;

    pub const A41: f64 = 44.0 / 45.0;
    pub const A42: f64 = -56.0 / 15.0;
    pub const A43: f64 = 32.0 / 9.0;

    pub const A51: f64 = 19_372.0 / 6_561.0;
    pub const A52: f64 = -25_360.0 / 2_187.0;
    pub const A53: f64 = 64_448.0 / 6_561.0;
    pub const A54: f64 = -212.0 / 729.0;

    pub const A61: f64 = 9_017.0 / 3_168.0;
    pub const A62: f64 = -355.0 / 33.0;
    pub const A63: f64 = 46_732.0 / 5_247.0;
    pub const A64: f64 = 49.0 / 176.0;
    pub const A65: f64 = -5_103.0 / 18_656.0;

    // 5th-order solution weights (b-vector). `B2 = 0` and `B7 = 0`
    // for the DOPRI5 5th-order solution. The fixed-step shipped path
    // does not evaluate `k7`; the embedded-error / adaptive path in
    // [`super::Dopri54Adaptive`] does evaluate `k7` to form the
    // 4th-order companion solution.
    pub const B1: f64 = 35.0 / 384.0;
    // B2 = 0 — Dormand-Prince has a zero second-stage weight.
    pub const B3: f64 = 500.0 / 1_113.0;
    pub const B4: f64 = 125.0 / 192.0;
    pub const B5: f64 = -2_187.0 / 6_784.0;
    pub const B6: f64 = 11.0 / 84.0;

    // Phase-5.D.4 — Dormand-Prince 5(4) embedded 4th-order weights
    // and the FSAL stage's a-row coefficients used to evaluate `k7`.
    // Pinned per Dormand & Prince (1980) Table II / Hairer-Nørsett-
    // Wanner Vol I §II.5 Table 5.2. The error vector is
    // `e = h · Σ E_i k_i` where `E_i = B_i − B̂_i`; both `E2 = 0` and
    // `E7 = -B̂7` per the tableau.
    pub const A71: f64 = 35.0 / 384.0;
    // A72 = 0 — same zero-weight pattern as B2.
    pub const A73: f64 = 500.0 / 1_113.0;
    pub const A74: f64 = 125.0 / 192.0;
    pub const A75: f64 = -2_187.0 / 6_784.0;
    pub const A76: f64 = 11.0 / 84.0;

    pub const E1: f64 = 71.0 / 57_600.0;
    // E2 = 0.
    pub const E3: f64 = -71.0 / 16_695.0;
    pub const E4: f64 = 71.0 / 1_920.0;
    pub const E5: f64 = -17_253.0 / 339_200.0;
    pub const E6: f64 = 22.0 / 525.0;
    pub const E7: f64 = -1.0 / 40.0;
}

/// Dormand-Prince 5(4) fixed-step integrator (5th-order accurate).
///
/// **Honest scope.** The shipped variant uses the DOPRI5 5th-order
/// solution at a **fixed step size** — the embedded 4th-order solution
/// and the adaptive PI step controller are explicitly deferred to a
/// follow-on slice (see `docs/phase-5-plan.md § 5.D.3`). The
/// fixed-step shape is a drop-in higher-order alternative to
/// [`Rk4FixedStep`] for scenarios where 4th-order RK4 truncation
/// error is the limiting factor.
///
/// 5th-order accurate, fixed-step, six derivative evaluations per
/// step. Bit-stable across reruns on the same platform profile.
#[derive(Copy, Clone, Debug, Default)]
pub struct Dopri54FixedStep;

impl<S: SimState> Integrator<S> for Dopri54FixedStep {
    fn determinism(&self) -> IntegratorDeterminism {
        IntegratorDeterminism::BitStable
    }

    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        use dopri54_tableau::{
            A21, A31, A32, A41, A42, A43, A51, A52, A53, A54, A61, A62, A63, A64, A65, B1, B3, B4,
            B5, B6, C2, C3, C4, C5,
        };

        let h = dt.as_seconds();
        if !h.is_finite() || h <= 0.0 {
            return Err(IntegratorError::InvalidStep { dt_seconds: h });
        }
        if !state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        let t0 = state.time();
        let t0_s = t0.as_seconds();
        let t2 = SimTime::from_seconds(t0_s + C2 * h);
        let t3 = SimTime::from_seconds(t0_s + C3 * h);
        let t4 = SimTime::from_seconds(t0_s + C4 * h);
        let t5 = SimTime::from_seconds(t0_s + C5 * h);
        let t6 = SimTime::from_seconds(t0_s + h);

        // Stage 1.
        let k1 = derive_fn(state, t0)?;
        if !k1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 2: y2 = y0 + h * (a21 * k1)
        let s2 = state.advance_by(h, &(k1 * A21));
        if !s2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k2 = derive_fn(&s2, t2)?;
        if !k2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 3: y3 = y0 + h * (a31 * k1 + a32 * k2)
        // DETERMINISM CONTRACT: locked order (k1 first, then k2).
        let inc3 = (k1 * A31) + (k2 * A32);
        let s3 = state.advance_by(h, &inc3);
        if !s3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k3 = derive_fn(&s3, t3)?;
        if !k3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 4: y4 = y0 + h * (a41 * k1 + a42 * k2 + a43 * k3)
        let inc4 = ((k1 * A41) + (k2 * A42)) + (k3 * A43);
        let s4 = state.advance_by(h, &inc4);
        if !s4.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k4 = derive_fn(&s4, t4)?;
        if !k4.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 5: y5 = y0 + h * (a51 * k1 + a52 * k2 + a53 * k3 + a54 * k4)
        let inc5 = (((k1 * A51) + (k2 * A52)) + (k3 * A53)) + (k4 * A54);
        let s5 = state.advance_by(h, &inc5);
        if !s5.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k5 = derive_fn(&s5, t5)?;
        if !k5.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 6: y6 = y0 + h * (a61 k1 + a62 k2 + a63 k3 + a64 k4 + a65 k5)
        let inc6 = ((((k1 * A61) + (k2 * A62)) + (k3 * A63)) + (k4 * A64)) + (k5 * A65);
        let s6 = state.advance_by(h, &inc6);
        if !s6.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k6 = derive_fn(&s6, t6)?;
        if !k6.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // 5th-order solution: y = y0 + h * (b1 k1 + b3 k3 + b4 k4 + b5 k5 + b6 k6)
        // (b2 is identically zero in the DOPRI5 tableau, so k2 is not
        // weighted into the solution.)
        // DETERMINISM CONTRACT: explicit parentheses prevent compiler
        // re-association; locked addend order; no FMA.
        let weighted = ((((k1 * B1) + (k3 * B3)) + (k4 * B4)) + (k5 * B5)) + (k6 * B6);
        let mut new_state = state.advance_by(h, &weighted);

        new_state.project();

        if !new_state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        Ok(new_state)
    }
}

// ---------------------------------------------------------------------
// Phase-5.D.4 — Dormand-Prince 5(4) adaptive integrator with PI step
// controller
// ---------------------------------------------------------------------

use std::cell::Cell;

/// Errors returned by [`Dopri54Adaptive::new`].
#[derive(Copy, Clone, Debug, PartialEq)]
pub enum AdaptiveIntegratorError {
    /// `atol` was non-positive or non-finite.
    InvalidAtol,
    /// `rtol` was non-positive or non-finite.
    InvalidRtol,
    /// `min_h_s` or `max_h_s` was non-positive, non-finite, or
    /// `max_h_s < min_h_s`.
    InvalidStepBounds,
    /// One of the controller gains (`safety_factor`, `pi_alpha`,
    /// `pi_beta`, `min_factor`, `max_factor`) was non-finite or out
    /// of its documented range.
    InvalidControllerGains,
}

/// Dormand-Prince 5(4) adaptive integrator with PI step controller.
///
/// The 5th-order solution from [`Dopri54FixedStep`] is augmented with
/// the embedded 4th-order companion `y_4` computed from the same
/// stages plus an FSAL `k7` evaluation. The per-step error derivative
/// `e' = Σ E_i k_i` drives a Gustafsson PI step
/// controller:
///
/// ```text
///   sc_i  = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)             (per-component)
///   err   = sqrt( (1/N) · Σ_i ( h · e'_i / sc_i )^2 )           (HNW Vol I §II.4 RMS)
///   factor = safety · err^(−α/p) · err_prev^(β/p)
///   p = 4, α = 0.7, β = 0.4, safety = 0.9
///   factor ∈ [min_factor, max_factor]
/// ```
///
/// Steps with `err > 1` are rejected and retried with `h` shrunk by
/// `factor`; accepted steps update `last_h` and `last_err_prev`. The
/// outer loop accumulates sub-steps until the cumulative time equals
/// the kernel's `dt` exactly (the last sub-step is clamped to fit). If
/// a rejected step is already at the configured floor, the integrator
/// returns [`IntegratorError::InvalidStep`] instead of silently
/// accepting a step outside tolerance.
///
/// # Error norm (Phase-5.D.5)
///
/// The scaled error norm uses the **per-component** Hairer-Nørsett-
/// Wanner Vol I §II.4 RMS form via
/// [`Integratable::weighted_error_norm`]: each component-wise scaled
/// error term is divided by its own per-component scale `sc_i`,
/// then averaged in RMS. This is the formulation §5.D.4 deferred —
/// the original scalar form `err = h · ||e'||₂ / (atol + rtol ·
/// scalar_state_size)` masked component-i breaches when other
/// components had large magnitudes (a 1 m position drift hidden by
/// `||y||₂ ≈ 1e6` m radius). The per-component form is required for
/// the rigid-body adaptive runner where state spans nine orders of
/// magnitude (position 1e6 m, quaternion 1, inertia 1e-3 kg·m²).
///
/// # Determinism
///
/// Tagged [`IntegratorDeterminism::StateStable`]. Within a single
/// platform profile (target triple + toolchain + LLVM optimisation
/// level) the integrator is **bit-stable across reruns**: the
/// step-size search is purely deterministic given identical inputs.
/// The `state-stable` label is for cross-platform behaviour where
/// platform-libm differences in `pow()` / `ln()` may lead to
/// slightly different step-size sequences. Internal persistent
/// state (`last_h`, `last_err_prev`) lives in [`Cell`] so the
/// `&self` trait surface stays unchanged. Each
/// [`crate::SimulationKernel`] owns one integrator instance; callers
/// must not share one adaptive integrator across interleaved kernels,
/// because doing so would intentionally share PI-controller history.
#[derive(Debug)]
pub struct Dopri54Adaptive {
    safety_factor: f64,
    min_factor: f64,
    max_factor: f64,
    pi_alpha: f64,
    pi_beta: f64,
    atol: f64,
    rtol: f64,
    min_h_s: f64,
    max_h_s: f64,
    /// PI controller persistent state. `Cell` keeps the
    /// `Integrator::advance(&self, ...)` trait surface unchanged.
    last_h_s: Cell<Option<f64>>,
    last_err_prev: Cell<Option<f64>>,
}

/// PI-controller exponents (Gustafsson 1991, recommended for 5th-order
/// embedded RK pairs).
const PI_ALPHA_DEFAULT: f64 = 0.7;
const PI_BETA_DEFAULT: f64 = 0.4;
const SAFETY_DEFAULT: f64 = 0.9;
const MIN_FACTOR_DEFAULT: f64 = 0.2;
const MAX_FACTOR_DEFAULT: f64 = 5.0;
/// Order of the lower-order embedded solution (DOPRI5(4) = 4).
const EMBEDDED_ORDER: f64 = 4.0;

impl Dopri54Adaptive {
    /// Construct an adaptive integrator with the given tolerances and
    /// step bounds. PI controller gains use Gustafsson 1991's
    /// recommended values for 5th-order embedded RK pairs.
    ///
    /// # Errors
    ///
    /// Returns [`AdaptiveIntegratorError`] when any tolerance or bound
    /// is non-positive / non-finite, or when `max_h_s < min_h_s`.
    pub fn new(
        atol: f64,
        rtol: f64,
        min_h_s: f64,
        max_h_s: f64,
    ) -> Result<Self, AdaptiveIntegratorError> {
        if !atol.is_finite() || atol <= 0.0 {
            return Err(AdaptiveIntegratorError::InvalidAtol);
        }
        if !rtol.is_finite() || rtol <= 0.0 {
            return Err(AdaptiveIntegratorError::InvalidRtol);
        }
        if !min_h_s.is_finite() || min_h_s <= 0.0 || !max_h_s.is_finite() || max_h_s < min_h_s {
            return Err(AdaptiveIntegratorError::InvalidStepBounds);
        }
        Ok(Self {
            safety_factor: SAFETY_DEFAULT,
            min_factor: MIN_FACTOR_DEFAULT,
            max_factor: MAX_FACTOR_DEFAULT,
            pi_alpha: PI_ALPHA_DEFAULT,
            pi_beta: PI_BETA_DEFAULT,
            atol,
            rtol,
            min_h_s,
            max_h_s,
            last_h_s: Cell::new(None),
            last_err_prev: Cell::new(None),
        })
    }

    /// Reset the controller's persistent state. Useful for unit tests
    /// that want a fresh PI history between scenarios.
    pub fn reset(&self) {
        self.last_h_s.set(None);
        self.last_err_prev.set(None);
    }

    fn pi_step_factor(&self, err: f64) -> f64 {
        // Standard Gustafsson PI: factor = safety · err^(−α/p) · prev^(β/p).
        // The `prev` term is omitted on the first accepted step (no
        // history) by treating `prev` as `1.0` so the formula degenerates
        // to an I-controller.
        let alpha_over_p = self.pi_alpha / EMBEDDED_ORDER;
        let beta_over_p = self.pi_beta / EMBEDDED_ORDER;
        let prev = self.last_err_prev.get().unwrap_or(1.0);
        let mut factor = self.safety_factor * err.powf(-alpha_over_p) * prev.powf(beta_over_p);
        if !factor.is_finite() || factor <= 0.0 {
            factor = self.min_factor;
        }
        factor.max(self.min_factor).min(self.max_factor)
    }

    /// Single DOPRI5(4) sub-step from `state` of size `h`. Returns
    /// `(new_state_5th_order, scaled_error_norm)`. Does NOT mutate
    /// the controller's persistent state — that's the caller's job.
    fn try_substep<S, F>(
        &self,
        state: &S,
        derive_fn: &F,
        h: f64,
    ) -> Result<(S, f64), IntegratorError>
    where
        S: SimState,
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        use dopri54_tableau::{
            A21, A31, A32, A41, A42, A43, A51, A52, A53, A54, A61, A62, A63, A64, A65, A71, A73,
            A74, A75, A76, B1, B3, B4, B5, B6, C2, C3, C4, C5, E1, E3, E4, E5, E6, E7,
        };

        let t0 = state.time();
        let t0_s = t0.as_seconds();
        let t2 = SimTime::from_seconds(t0_s + C2 * h);
        let t3 = SimTime::from_seconds(t0_s + C3 * h);
        let t4 = SimTime::from_seconds(t0_s + C4 * h);
        let t5 = SimTime::from_seconds(t0_s + C5 * h);
        let t6 = SimTime::from_seconds(t0_s + h);
        let t7 = t6;

        // Stages 1..6 — same as Dopri54FixedStep.
        let k1 = derive_fn(state, t0)?;
        if !k1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let s2 = state.advance_by(h, &(k1 * A21));
        if !s2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k2 = derive_fn(&s2, t2)?;
        if !k2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc3 = (k1 * A31) + (k2 * A32);
        let s3 = state.advance_by(h, &inc3);
        if !s3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k3 = derive_fn(&s3, t3)?;
        if !k3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc4 = ((k1 * A41) + (k2 * A42)) + (k3 * A43);
        let s4 = state.advance_by(h, &inc4);
        if !s4.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k4 = derive_fn(&s4, t4)?;
        if !k4.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc5 = (((k1 * A51) + (k2 * A52)) + (k3 * A53)) + (k4 * A54);
        let s5 = state.advance_by(h, &inc5);
        if !s5.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k5 = derive_fn(&s5, t5)?;
        if !k5.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc6 = ((((k1 * A61) + (k2 * A62)) + (k3 * A63)) + (k4 * A64)) + (k5 * A65);
        let s6 = state.advance_by(h, &inc6);
        if !s6.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k6 = derive_fn(&s6, t6)?;
        if !k6.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // 5th-order solution (same as Dopri54FixedStep).
        let weighted5 = ((((k1 * B1) + (k3 * B3)) + (k4 * B4)) + (k5 * B5)) + (k6 * B6);
        let mut new_state = state.advance_by(h, &weighted5);
        new_state.project();
        if !new_state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        // Stage 7 — FSAL evaluation at the 5th-order endpoint, used to
        // form the embedded 4th-order companion via the E_i weights.
        // The A7_i row reproduces the B_i weights so y_7 = y_5 (FSAL
        // property); k7 = f(y_5, t_end).
        let inc7 = (((((k1 * A71) + (k3 * A73)) + (k4 * A74)) + (k5 * A75)) + (k6 * A76)) * 1.0;
        let s7 = state.advance_by(h, &inc7);
        if !s7.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k7 = derive_fn(&s7, t7)?;
        if !k7.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Error derivative: e' = Σ E_i k_i (E2 = 0 implied). The
        // multiplication by `h` to obtain `e = y_5 − y_4` happens
        // inside `weighted_error_norm`.
        let error_deriv =
            (((((k1 * E1) + (k3 * E3)) + (k4 * E4)) + (k5 * E5)) + (k6 * E6)) + (k7 * E7);
        // Per-component scaled error RMS norm (Phase-5.D.5):
        //   sc_i = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)
        //   err = sqrt( (1/N) · Σ_i ( h · e'_i / sc_i )^2 )
        let scaled_err =
            new_state.weighted_error_norm(state, &error_deriv, h, self.atol, self.rtol);

        Ok((new_state, scaled_err))
    }
}

impl<S: SimState> Integrator<S> for Dopri54Adaptive {
    fn determinism(&self) -> IntegratorDeterminism {
        IntegratorDeterminism::StateStable
    }

    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        let dt_total = dt.as_seconds();
        if !dt_total.is_finite() || dt_total <= 0.0 {
            return Err(IntegratorError::InvalidStep {
                dt_seconds: dt_total,
            });
        }
        if !state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        // Initial sub-step size: prefer last accepted h, fall back to
        // dt_total clamped to [min_h_s, max_h_s].
        let mut h = self
            .last_h_s
            .get()
            .unwrap_or(dt_total)
            .max(self.min_h_s)
            .min(self.max_h_s)
            .min(dt_total);

        let mut current = *state;
        let mut elapsed = 0.0_f64;
        let mut last_accepted_err: Option<f64> = self.last_err_prev.get();
        // Hard cap on number of sub-steps so a misbehaving derivative
        // can't loop forever.
        let max_substeps: usize = 1_000_000;
        let mut substeps_taken: usize = 0;

        while elapsed < dt_total {
            substeps_taken += 1;
            if substeps_taken > max_substeps {
                return Err(IntegratorError::InvalidStep {
                    dt_seconds: dt_total,
                });
            }

            // Clamp h so the sub-step lands within dt_total.
            let remaining = dt_total - elapsed;
            let h_try = if remaining <= self.min_h_s {
                remaining
            } else {
                h.min(remaining).max(self.min_h_s)
            };

            // Per-iteration controller history (so the rejection retry
            // path uses the most recent observation).
            self.last_err_prev.set(last_accepted_err);

            let (proposed_state, err) = self.try_substep(&current, &derive_fn, h_try)?;

            if err <= 1.0 {
                // Accept.
                current = proposed_state;
                elapsed += h_try;
                last_accepted_err = Some(err.max(1.0e-10));
                let factor = self.pi_step_factor(err.max(1.0e-10));
                h = (h_try * factor).max(self.min_h_s).min(self.max_h_s);
            } else {
                // Reject: shrink h. Use the I-controller form (no PI
                // β-term) on rejection per Hairer-Nørsett-Wanner §II.4
                // recommendation.
                let alpha_over_p = self.pi_alpha / EMBEDDED_ORDER;
                let mut factor = self.safety_factor * err.powf(-alpha_over_p);
                if !factor.is_finite() || factor <= 0.0 {
                    factor = self.min_factor;
                }
                factor = factor.max(self.min_factor).min(1.0);
                if h_try <= self.min_h_s + f64::EPSILON {
                    // Already at floor and still outside tolerance:
                    // fail closed instead of silently violating the
                    // configured error bound.
                    return Err(IntegratorError::InvalidStep { dt_seconds: h_try });
                }
                h = (h_try * factor).max(self.min_h_s).min(self.max_h_s);
            }
        }

        // Persist the controller's state for the next `advance` call.
        self.last_h_s.set(Some(h));
        self.last_err_prev.set(last_accepted_err);
        Ok(current)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::derivative::PointMassDerivative;
    use approx::assert_abs_diff_eq;
    use nalgebra::Vector3;
    use openbmp_core::{Position3, SimTime, Velocity3};
    use openbmp_models::VehicleState;
    use openbmp_state::PointMassState;
    use uom::si::f64::Mass;
    use uom::si::mass::kilogram;

    fn unit_kg() -> Mass {
        Mass::new::<kilogram>(1.0)
    }

    fn one_kg_at_origin() -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            unit_kg(),
        )
    }

    #[test]
    fn rk4_rejects_zero_dt() {
        let state = one_kg_at_origin();
        let result = Rk4FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.0),
        );
        assert!(matches!(result, Err(IntegratorError::InvalidStep { .. })));
    }

    #[test]
    fn rk4_rejects_negative_dt() {
        let state = one_kg_at_origin();
        let result = Rk4FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(-0.01),
        );
        assert!(matches!(result, Err(IntegratorError::InvalidStep { .. })));
    }

    #[test]
    fn rk4_zero_derivative_returns_state_with_advanced_time() {
        let state = PointMassState::new(
            SimTime::from_seconds(1.0),
            Position3::new(7.0, 0.0, 0.0),
            Velocity3::new(0.0, 8.0, 0.0),
            unit_kg(),
        );
        let dt = Duration::from_seconds(0.5);
        let next = Rk4FixedStep
            .advance(&state, |_s, _t| Ok(PointMassDerivative::zero()), dt)
            .expect("zero derivative must succeed");
        assert_abs_diff_eq!(next.time.as_seconds(), 1.5);
        // Position and velocity unchanged because derivative is zero.
        assert_abs_diff_eq!(next.position.vector.x, 7.0);
        assert_abs_diff_eq!(next.velocity.vector.y, 8.0);
    }

    /// RK4 is exact for polynomials of degree ≤ 4. Constant
    /// acceleration produces a quadratic `x(t)` and linear `v(t)`,
    /// both well within RK4's exactness range. The integrated state
    /// should match the analytic closed form to floating-point
    /// rounding only.
    #[test]
    fn rk4_constant_acceleration_matches_analytic_for_one_step() {
        // 1 kg point mass at rest; constant acceleration -9.81 m/s² in z.
        let state = one_kg_at_origin();
        let g_eci = Vector3::new(0.0, 0.0, -9.81);

        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: g_eci, // mass=1, so force/mass = g.
                    mass_rate_kg_s: 0.0,
                })
            };

        let dt = Duration::from_seconds(0.01);
        let next = Rk4FixedStep
            .advance(&state, derive, dt)
            .expect("step must succeed");

        // Analytic: x(0.01) = 0 + 0·0.01 + ½·(-9.81)·0.0001 = -4.905e-4.
        let expected_z = 0.5 * -9.81 * 0.01_f64 * 0.01;
        assert_abs_diff_eq!(next.position.vector.z, expected_z, epsilon = 1.0e-15);
        // Analytic: v(0.01) = 0 + (-9.81)·0.01 = -0.0981.
        assert_abs_diff_eq!(next.velocity.vector.z, -0.0981, epsilon = 1.0e-15);
        // Time advanced.
        assert_abs_diff_eq!(next.time.as_seconds(), 0.01);
    }

    /// RK4 on `dy/dt = -y` over 10 steps of `dt = 0.1` should match
    /// `exp(-1) ≈ 0.36787944...` to 4th-order accuracy.
    ///
    /// Per-step truncation error scales as `(h⁵ / 120) · y^(5)`. For
    /// `y(t) = exp(-t)`, `|y^(5)|` is bounded by 1 over `[0, 1]`, so
    /// the per-step error is `~8.3e-7`. After 10 steps the
    /// accumulated error stays bounded by a small multiple of that
    /// because the equation is dissipative; observed value is
    /// `~3.4e-7`.
    #[test]
    fn rk4_exponential_decay_step_within_4th_order_error() {
        // We piggy-back on PointMassState by treating mass as the scalar
        // y. `dy/dt = -y` ↔ `mass_rate = -mass`.
        let mut state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::zeros(),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: -s.mass.get::<kilogram>(),
                })
            };

        let dt = Duration::from_seconds(0.1);
        for _ in 0..10 {
            state = Rk4FixedStep.advance(&state, derive, dt).expect("step");
        }
        let exp_neg_one = (-1.0_f64).exp();
        assert_abs_diff_eq!(state.mass.get::<kilogram>(), exp_neg_one, epsilon = 1.0e-6);
    }

    #[test]
    fn rk4_propagates_non_finite_derivative_as_error() {
        let state = one_kg_at_origin();
        let derive =
            |_s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::new(f64::NAN, 0.0, 0.0),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: 0.0,
                })
            };
        let dt = Duration::from_seconds(0.01);
        let result = Rk4FixedStep.advance(&state, derive, dt);
        assert!(matches!(result, Err(IntegratorError::NonFiniteDerivative)));
    }

    #[test]
    fn rk4_rejects_invalid_initial_state() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(0.0),
        );
        let result = Rk4FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.01),
        );
        assert!(matches!(result, Err(IntegratorError::NonFiniteState)));
    }

    #[test]
    fn rk4_rejects_invalid_intermediate_state() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let derive =
            |_s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::zeros(),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: -10.0,
                })
            };
        let result = Rk4FixedStep.advance(&state, derive, Duration::from_seconds(1.0));
        assert!(matches!(result, Err(IntegratorError::NonFiniteState)));
    }

    #[test]
    fn rk4_is_bit_stable_across_two_runs() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0, 2.0, 3.0),
            Velocity3::new(0.4, 0.5, 0.6),
            Mass::new::<kilogram>(1.5),
        );
        let g = Vector3::new(0.1, -0.2, -9.81);
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: g,
                    mass_rate_kg_s: 0.0,
                })
            };

        let dt = Duration::from_seconds(0.01);
        let a = Rk4FixedStep.advance(&state, derive, dt).expect("step a");
        let b = Rk4FixedStep.advance(&state, derive, dt).expect("step b");

        // Bit equality.
        assert_eq!(a.position.vector.x.to_bits(), b.position.vector.x.to_bits());
        assert_eq!(a.position.vector.y.to_bits(), b.position.vector.y.to_bits());
        assert_eq!(a.position.vector.z.to_bits(), b.position.vector.z.to_bits());
        assert_eq!(a.velocity.vector.z.to_bits(), b.velocity.vector.z.to_bits());
        assert_eq!(
            a.mass.get::<kilogram>().to_bits(),
            b.mass.get::<kilogram>().to_bits()
        );
    }

    #[test]
    fn rk4_determinism_class_is_bit_stable() {
        let i = Rk4FixedStep;
        assert_eq!(
            <Rk4FixedStep as Integrator<PointMassState>>::determinism(&i),
            IntegratorDeterminism::BitStable
        );
    }

    // -----------------------------------------------------------------
    // Phase-5.D.3 — Dormand-Prince 5(4) fixed-step tests
    // -----------------------------------------------------------------

    #[test]
    fn dopri54_rejects_zero_dt() {
        let state = one_kg_at_origin();
        let result = Dopri54FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.0),
        );
        assert!(matches!(result, Err(IntegratorError::InvalidStep { .. })));
    }

    #[test]
    fn dopri54_rejects_negative_dt() {
        let state = one_kg_at_origin();
        let result = Dopri54FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(-0.5),
        );
        assert!(matches!(result, Err(IntegratorError::InvalidStep { .. })));
    }

    #[test]
    fn dopri54_zero_derivative_returns_state_with_advanced_time() {
        let state = PointMassState::new(
            SimTime::from_seconds(2.0),
            Position3::new(7.0, 0.0, 0.0),
            Velocity3::new(0.0, 8.0, 0.0),
            unit_kg(),
        );
        let dt = Duration::from_seconds(0.5);
        let next = Dopri54FixedStep
            .advance(&state, |_s, _t| Ok(PointMassDerivative::zero()), dt)
            .expect("zero derivative must succeed");
        assert_abs_diff_eq!(next.time.as_seconds(), 2.5);
        assert_abs_diff_eq!(next.position.vector.x, 7.0);
        assert_abs_diff_eq!(next.velocity.vector.y, 8.0);
    }

    /// Dormand-Prince 5(4) is exact for polynomials of degree ≤ 5;
    /// constant acceleration produces a quadratic position and linear
    /// velocity, well within DOPRI5's exactness range. The integrated
    /// state must match the analytic closed form to floating-point
    /// rounding only.
    #[test]
    fn dopri54_constant_acceleration_matches_analytic_for_one_step() {
        let state = one_kg_at_origin();
        let g_eci = Vector3::new(0.0, 0.0, -9.81);

        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: g_eci,
                    mass_rate_kg_s: 0.0,
                })
            };

        let dt = Duration::from_seconds(0.01);
        let next = Dopri54FixedStep
            .advance(&state, derive, dt)
            .expect("step must succeed");

        let expected_z = 0.5 * -9.81 * 0.01_f64 * 0.01;
        assert_abs_diff_eq!(next.position.vector.z, expected_z, epsilon = 1.0e-15);
        assert_abs_diff_eq!(next.velocity.vector.z, -0.0981, epsilon = 1.0e-15);
        assert_abs_diff_eq!(next.time.as_seconds(), 0.01);
    }

    /// DOPRI5 on `dy/dt = -y` over 10 steps of `dt = 0.1` matches
    /// `exp(-1) ≈ 0.36787944...` to 5th-order accuracy.
    ///
    /// Per-step truncation error scales as `~ h^6 / 720 · y^(6)`; for
    /// `y = exp(-t)` this is `~1.4e-9` per step. Accumulated error
    /// after 10 steps is bounded by roughly `1e-8`, an order of
    /// magnitude tighter than the same `dt = 0.1` RK4 test
    /// (`~ 3.4e-7`).
    #[test]
    fn dopri54_exponential_decay_step_within_5th_order_error() {
        let mut state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::zeros(),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: -s.mass.get::<kilogram>(),
                })
            };

        let dt = Duration::from_seconds(0.1);
        for _ in 0..10 {
            state = Dopri54FixedStep.advance(&state, derive, dt).expect("step");
        }
        let exp_neg_one = (-1.0_f64).exp();
        // 5th-order tolerance is much tighter than RK4's 1e-6.
        assert_abs_diff_eq!(state.mass.get::<kilogram>(), exp_neg_one, epsilon = 1.0e-7);
    }

    /// Direct accuracy comparison: DOPRI5 must do strictly better than
    /// RK4 on the same exponential-decay trajectory at the same step
    /// size — that's the entire reason to ship a higher-order
    /// integrator. The error ratio captures the order improvement
    /// directly.
    #[test]
    fn dopri54_is_more_accurate_than_rk4_on_exponential_decay() {
        fn run_with<I: Integrator<PointMassState>>(integrator: &I, dt_s: f64, steps: usize) -> f64 {
            let mut state = PointMassState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Mass::new::<kilogram>(1.0),
            );
            let derive =
                |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                    Ok(PointMassDerivative {
                        velocity_m_s: Vector3::zeros(),
                        acceleration_m_s2: Vector3::zeros(),
                        mass_rate_kg_s: -s.mass.get::<kilogram>(),
                    })
                };
            let dt = Duration::from_seconds(dt_s);
            for _ in 0..steps {
                state = integrator.advance(&state, derive, dt).expect("step");
            }
            state.mass.get::<kilogram>()
        }
        let exp_neg_one = (-1.0_f64).exp();
        let rk4_final = run_with(&Rk4FixedStep, 0.1, 10);
        let dopri_final = run_with(&Dopri54FixedStep, 0.1, 10);
        let rk4_err = (rk4_final - exp_neg_one).abs();
        let dopri_err = (dopri_final - exp_neg_one).abs();
        assert!(
            dopri_err < rk4_err,
            "DOPRI5 must beat RK4 on exp-decay test: RK4 err {rk4_err:.3e}, \
             DOPRI5 err {dopri_err:.3e}",
        );
        // Lower bound on the order improvement: DOPRI5 should be at
        // least 10x more accurate at this step size.
        assert!(
            dopri_err * 10.0 < rk4_err,
            "DOPRI5 should be ≥10× more accurate than RK4 here: \
             RK4 err {rk4_err:.3e}, DOPRI5 err {dopri_err:.3e}",
        );
    }

    #[test]
    fn dopri54_propagates_non_finite_derivative_as_error() {
        let state = one_kg_at_origin();
        let derive =
            |_s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: Vector3::new(f64::NAN, 0.0, 0.0),
                    acceleration_m_s2: Vector3::zeros(),
                    mass_rate_kg_s: 0.0,
                })
            };
        let dt = Duration::from_seconds(0.01);
        let result = Dopri54FixedStep.advance(&state, derive, dt);
        assert!(matches!(result, Err(IntegratorError::NonFiniteDerivative)));
    }

    #[test]
    fn dopri54_rejects_invalid_initial_state() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(0.0),
        );
        let result = Dopri54FixedStep.advance(
            &state,
            |_s, _t| Ok(PointMassDerivative::zero()),
            Duration::from_seconds(0.01),
        );
        assert!(matches!(result, Err(IntegratorError::NonFiniteState)));
    }

    #[test]
    fn dopri54_is_bit_stable_across_two_runs() {
        let state = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0, 2.0, 3.0),
            Velocity3::new(0.4, 0.5, 0.6),
            Mass::new::<kilogram>(1.5),
        );
        let g = Vector3::new(0.1, -0.2, -9.81);
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: g,
                    mass_rate_kg_s: 0.0,
                })
            };

        let dt = Duration::from_seconds(0.01);
        let a = Dopri54FixedStep
            .advance(&state, derive, dt)
            .expect("step a");
        let b = Dopri54FixedStep
            .advance(&state, derive, dt)
            .expect("step b");

        assert_eq!(a.position.vector.x.to_bits(), b.position.vector.x.to_bits());
        assert_eq!(a.position.vector.y.to_bits(), b.position.vector.y.to_bits());
        assert_eq!(a.position.vector.z.to_bits(), b.position.vector.z.to_bits());
        assert_eq!(a.velocity.vector.z.to_bits(), b.velocity.vector.z.to_bits());
        assert_eq!(
            a.mass.get::<kilogram>().to_bits(),
            b.mass.get::<kilogram>().to_bits()
        );
    }

    #[test]
    fn dopri54_determinism_class_is_bit_stable() {
        let i = Dopri54FixedStep;
        assert_eq!(
            <Dopri54FixedStep as Integrator<PointMassState>>::determinism(&i),
            IntegratorDeterminism::BitStable
        );
    }

    // -----------------------------------------------------------------
    // Phase-5.D.4 — Dormand-Prince 5(4) adaptive integrator tests.
    // -----------------------------------------------------------------

    // Returns Result to satisfy the `Integrator::advance` derive_fn trait
    // bound; never errors in this helper.
    #[allow(clippy::unnecessary_wraps)]
    fn exp_decay_derive(
        s: &PointMassState,
        _t: SimTime,
    ) -> Result<PointMassDerivative, ModelEvalError> {
        // dy/dt = -y, with `y` carried in the mass field.
        Ok(PointMassDerivative {
            velocity_m_s: Vector3::zeros(),
            acceleration_m_s2: Vector3::zeros(),
            mass_rate_kg_s: -s.mass.get::<kilogram>(),
        })
    }

    fn exp_decay_initial_state() -> PointMassState {
        PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        )
    }

    #[test]
    fn dopri54_adaptive_constructor_rejects_invalid_atol() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                Dopri54Adaptive::new(bad, 1.0e-6, 1.0e-9, 1.0).unwrap_err(),
                AdaptiveIntegratorError::InvalidAtol,
                "atol = {bad} should be rejected",
            );
        }
    }

    #[test]
    fn dopri54_adaptive_constructor_rejects_invalid_rtol() {
        for bad in [0.0, -0.1, f64::NAN, f64::INFINITY] {
            assert_eq!(
                Dopri54Adaptive::new(1.0e-9, bad, 1.0e-9, 1.0).unwrap_err(),
                AdaptiveIntegratorError::InvalidRtol,
                "rtol = {bad} should be rejected",
            );
        }
    }

    #[test]
    fn dopri54_adaptive_constructor_rejects_invalid_step_bounds() {
        // max < min
        assert_eq!(
            Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0, 0.5).unwrap_err(),
            AdaptiveIntegratorError::InvalidStepBounds,
        );
        // min ≤ 0
        assert_eq!(
            Dopri54Adaptive::new(1.0e-9, 1.0e-6, 0.0, 1.0).unwrap_err(),
            AdaptiveIntegratorError::InvalidStepBounds,
        );
        // non-finite
        assert_eq!(
            Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, f64::INFINITY).unwrap_err(),
            AdaptiveIntegratorError::InvalidStepBounds,
        );
    }

    #[test]
    fn dopri54_adaptive_determinism_class_is_state_stable() {
        let i = Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, 1.0).unwrap();
        assert_eq!(
            <Dopri54Adaptive as Integrator<PointMassState>>::determinism(&i),
            IntegratorDeterminism::StateStable
        );
    }

    /// Integrating dy/dt = -y over [0, 1] with adaptive control should
    /// land within tolerance of exp(-1).
    #[test]
    fn dopri54_adaptive_exp_decay_meets_declared_tolerance() {
        let integrator = Dopri54Adaptive::new(1.0e-12, 1.0e-9, 1.0e-9, 0.1).unwrap();
        let state = exp_decay_initial_state();
        let final_state = integrator
            .advance(&state, exp_decay_derive, Duration::from_seconds(1.0))
            .expect("adaptive advance must succeed");
        let exp_neg_one = (-1.0_f64).exp();
        let err = (final_state.mass.get::<kilogram>() - exp_neg_one).abs();
        // With rtol=1e-9, atol=1e-12, the achieved error should beat
        // the rtol·|y| bound by a comfortable margin (5th-order).
        assert!(
            err < 1.0e-9,
            "adaptive ||err|| = {err} exceeds declared tolerance",
        );
    }

    /// Two independently-constructed integrators fed an identical
    /// stream produce bit-identical final states — within-platform
    /// bit-stability proves the adaptive step-size search is purely
    /// deterministic.
    #[test]
    fn dopri54_adaptive_within_platform_bit_stable_across_two_runs() {
        let make_integrator = || Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, 0.1).unwrap();
        let state = exp_decay_initial_state();
        let a = make_integrator()
            .advance(&state, exp_decay_derive, Duration::from_seconds(1.0))
            .expect("a");
        let b = make_integrator()
            .advance(&state, exp_decay_derive, Duration::from_seconds(1.0))
            .expect("b");
        assert_eq!(
            a.mass.get::<kilogram>().to_bits(),
            b.mass.get::<kilogram>().to_bits(),
            "adaptive integrator not bit-stable across reruns",
        );
    }

    /// Sub-step accumulation must land at exactly the requested dt
    /// (the last sub-step is clamped to fit). Time field is the proof.
    #[test]
    fn dopri54_adaptive_substep_accumulation_lands_exactly_at_dt() {
        let integrator = Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, 1.0).unwrap();
        let state = exp_decay_initial_state();
        let dt = 0.7_f64;
        let final_state = integrator
            .advance(&state, exp_decay_derive, Duration::from_seconds(dt))
            .expect("advance");
        // Accumulator should land at exactly dt; floating-point sums
        // of multiple sub-steps may have rounding, but the
        // final-clamp logic ensures elapsed = dt exactly via the
        // remaining = dt − elapsed clamp on the last sub-step.
        let final_t = final_state.time.as_seconds();
        assert!(
            (final_t - dt).abs() < 1.0e-12,
            "final time {final_t} not within 1e-12 of dt = {dt}",
        );
    }

    /// The final sub-step is allowed to be below `min_h_s`; otherwise
    /// `advance()` would overshoot `dt` whenever the outer kernel step
    /// or final remainder is smaller than the adaptive floor.
    #[test]
    fn dopri54_adaptive_final_substep_can_be_below_min_h_to_fit_dt() {
        let integrator = Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-3, 1.0).unwrap();
        let state = exp_decay_initial_state();
        let dt = 1.0e-4_f64;

        let final_state = integrator
            .advance(&state, exp_decay_derive, Duration::from_seconds(dt))
            .expect("advance");

        assert_eq!(
            final_state.time.as_seconds().to_bits(),
            dt.to_bits(),
            "adaptive integrator must not overshoot an outer dt below min_h_s"
        );
    }

    /// A floor is a hard integration limit, not permission to accept a
    /// step that still violates the configured tolerance.
    #[test]
    fn dopri54_adaptive_rejects_when_tolerance_cannot_be_met_at_min_h() {
        let integrator = Dopri54Adaptive::new(1.0e-300, 1.0e-300, 1.0e-3, 1.0e-3).unwrap();
        let state = exp_decay_initial_state();
        let err = integrator
            .advance(&state, exp_decay_derive, Duration::from_seconds(1.0e-3))
            .expect_err("unachievable tolerance at h_min must fail closed");

        assert!(
            matches!(err, IntegratorError::InvalidStep { dt_seconds } if dt_seconds.to_bits() == 1.0e-3_f64.to_bits()),
            "expected InvalidStep at h_min, got {err:?}"
        );
    }

    /// At very tight tolerance the adaptive integrator should match a
    /// fixed-step DOPRI5 reference (or be tighter) on the same step
    /// budget. Sanity check on the embedded-error correctness.
    #[test]
    fn dopri54_adaptive_at_tight_tolerance_beats_fixed_step_dopri5() {
        // Fixed-step at dt = 0.1 over [0, 1] (10 steps).
        let mut state = exp_decay_initial_state();
        for _ in 0..10 {
            state = Dopri54FixedStep
                .advance(&state, exp_decay_derive, Duration::from_seconds(0.1))
                .expect("fixed step");
        }
        let exp_neg_one = (-1.0_f64).exp();
        let fixed_err = (state.mass.get::<kilogram>() - exp_neg_one).abs();
        // Adaptive at very tight tolerance.
        let integrator = Dopri54Adaptive::new(1.0e-15, 1.0e-12, 1.0e-9, 0.1).unwrap();
        let state0 = exp_decay_initial_state();
        let adaptive = integrator
            .advance(&state0, exp_decay_derive, Duration::from_seconds(1.0))
            .expect("adaptive");
        let adaptive_err = (adaptive.mass.get::<kilogram>() - exp_neg_one).abs();
        assert!(
            adaptive_err <= fixed_err,
            "adaptive (rtol=1e-12) err {adaptive_err} should beat fixed-step \
             DOPRI5 (dt=0.1) err {fixed_err}",
        );
    }

    /// Reset clears persistent controller state.
    #[test]
    fn dopri54_adaptive_reset_clears_persistent_state() {
        let integrator = Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, 1.0).unwrap();
        let state = exp_decay_initial_state();
        let _ = integrator.advance(&state, exp_decay_derive, Duration::from_seconds(0.5));
        // last_h should be Some after a successful advance.
        assert!(integrator.last_h_s.get().is_some());
        integrator.reset();
        assert!(integrator.last_h_s.get().is_none());
        assert!(integrator.last_err_prev.get().is_none());
    }

    /// Embedded-error magnitude on a constant-acceleration trajectory
    /// (which DOPRI5 integrates exactly) should be at the
    /// floating-point-rounding floor — the 5th-order solution and the
    /// 4th-order companion both reproduce the closed form, so the
    /// difference is essentially round-off.
    #[test]
    fn dopri54_adaptive_embedded_error_vanishes_on_polynomial_trajectory() {
        // Constant acceleration: dx/dt = v, dv/dt = a (constant).
        let g = -9.81;
        let derive =
            |s: &PointMassState, _t: SimTime| -> Result<PointMassDerivative, ModelEvalError> {
                Ok(PointMassDerivative {
                    velocity_m_s: s.velocity.vector,
                    acceleration_m_s2: Vector3::new(0.0, 0.0, g),
                    mass_rate_kg_s: 0.0,
                })
            };
        let state = exp_decay_initial_state();
        // Use a "relaxed" tolerance pair so the *scaled* error norm
        // can drop to machine epsilon. With rtol = 1e-12 the
        // denominator `(atol + rtol · ||y||)` is so small that even
        // machine-epsilon raw error in the embedded weights amplifies
        // to ~1e-8 in scaled units; that's expected, not a bug. We
        // verify the scaled error is small *relative to the
        // tolerance* — the actual unscaled error is (err · scale),
        // which should be near machine epsilon.
        let integrator = Dopri54Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, 1.0).unwrap();
        // Use try_substep directly to inspect the per-step error norm.
        let (_new, err) = integrator
            .try_substep(&state, &derive, 0.01)
            .expect("substep");
        // For a constant-accel trajectory DOPRI5 is exact (degree-2 in
        // v, degree-3 in x, all within the 5th-order exactness range)
        // AND the embedded 4th-order companion is exact for the same
        // polynomial degrees. The scaled error should be well below
        // the unit threshold (PI controller would expand h aggressively).
        assert!(
            err < 1.0e-3,
            "embedded scaled error {err} should be far below 1.0 for an exact polynomial trajectory at rtol=1e-6",
        );
    }

    /// Tight tolerance forces step rejection; loose tolerance lets h
    /// expand toward `max_h`. Verify by comparing the final `last_h`.
    #[test]
    fn dopri54_adaptive_step_size_tracks_tolerance_band() {
        let state = exp_decay_initial_state();
        // Loose tolerance: h should expand.
        let loose = Dopri54Adaptive::new(1.0e-3, 1.0e-2, 1.0e-9, 1.0).unwrap();
        let _ = loose.advance(&state, exp_decay_derive, Duration::from_seconds(1.0));
        let h_loose = loose.last_h_s.get().expect("h after advance");
        // Tight tolerance: h should shrink.
        let tight = Dopri54Adaptive::new(1.0e-15, 1.0e-12, 1.0e-9, 1.0).unwrap();
        let _ = tight.advance(&state, exp_decay_derive, Duration::from_seconds(1.0));
        let h_tight = tight.last_h_s.get().expect("h after advance");
        assert!(
            h_loose > h_tight,
            "loose tolerance h ({h_loose}) should exceed tight tolerance h ({h_tight})",
        );
    }

    /// Phase-5.D.5 — PI controller band-stability over a long
    /// sequence of single sub-steps.
    ///
    /// The public `Integrator::advance(...)` API rolls multiple
    /// sub-steps into one call and lands exactly at `dt`, so the
    /// integrator's `last_h_s` after each call reflects whatever
    /// remaining time fragment the loop had to clear — not the
    /// controller's preferred steady-state h. To observe the
    /// controller's behaviour we drive the private `try_substep`
    /// helper directly in a tight loop, mirroring the inner-loop
    /// accept/reject pattern but without the outer-loop fragment
    /// clamping.
    ///
    /// Trajectory: `dy/dt = -y` from `y(0) = 1`. At
    /// `(rtol, atol) = (1e-12, 1e-12)` the controller's optimal
    /// `h ≈ (atol·5! / y)^(1/(p+1)) ≈ 6.5e-3` for the 5th-order
    /// method, sitting comfortably inside `[1e-9, 5e-2]`.
    ///
    /// Asserts:
    /// * After warm-up, the accepted-h sequence stays inside a
    ///   per-step factor band of `[0.5, 2.0]` (much tighter than
    ///   the factor clamp `[0.2, 5.0]` — a healthy PI controller
    ///   converges fast).
    /// * Rejection rate over the full run stays below 10 % (HNW Vol I
    ///   §II.4 cites < 5 % as typical for well-tuned controllers; the
    ///   10 % threshold is loose enough to catch a broken controller
    ///   without flaking on benign initial-step rejections).
    /// * Steady-state median accepted h lies inside the sanity band
    ///   for the configured tolerance.
    #[test]
    fn dopri54_adaptive_pi_controller_stays_in_band_over_long_run() {
        let initial = exp_decay_initial_state();
        let atol = 1.0e-12;
        let rtol = 1.0e-12;
        let min_h = 1.0e-9;
        let max_h = 5.0e-2;
        let integrator = Dopri54Adaptive::new(atol, rtol, min_h, max_h).unwrap();

        let n_steps = 200;
        let mut state = initial;
        // Initial trial step: same logic the public `advance` uses
        // (fall back to the largest allowed h when there is no
        // history).
        let mut h = max_h;
        let mut accepted_h_history: Vec<f64> = Vec::with_capacity(n_steps);
        let mut last_err_prev: Option<f64> = None;
        let mut total_accepts: usize = 0;
        let mut total_rejects: usize = 0;

        // Drive single sub-steps with an outer accept/reject pattern
        // parallel to the integrator's internal one; this lets us
        // observe the per-sub-step h directly.
        while total_accepts < n_steps {
            integrator.last_err_prev.set(last_err_prev);
            let h_try = h.max(min_h).min(max_h);
            let (proposed, err) = integrator
                .try_substep(&state, &exp_decay_derive, h_try)
                .expect("try_substep must succeed for a benign integrand");

            if err <= 1.0 {
                // Accept: use the integrator's PI hot path directly
                // while `last_err_prev` still holds the previous
                // accepted error.
                let err_clamped = err.max(1.0e-10);
                let factor = integrator.pi_step_factor(err_clamped);
                state = proposed.with_time(SimTime::from_seconds(state.time.as_seconds() + h_try));
                accepted_h_history.push(h_try);
                last_err_prev = Some(err_clamped);
                h = (h_try * factor).max(min_h).min(max_h);
                total_accepts += 1;
            } else {
                // Reject: I-controller shrink (no β term).
                let alpha_over_p = integrator.pi_alpha / EMBEDDED_ORDER;
                let mut factor = integrator.safety_factor * err.powf(-alpha_over_p);
                if !factor.is_finite() || factor <= 0.0 {
                    factor = integrator.min_factor;
                }
                factor = factor.max(integrator.min_factor).min(1.0);
                h = (h_try * factor).max(integrator.min_h_s);
                total_rejects += 1;
            }
        }

        // 1) No clamp pinning post-warmup. The first accept usually
        //    follows a rejection from the max_h initial guess and
        //    can land near the asymptotic optimum cleanly; skip the
        //    first 5 entries to be safe.
        for (i, &h_acc) in accepted_h_history.iter().enumerate().skip(5) {
            assert!(
                h_acc > min_h * 10.0,
                "accepted h at step {i} = {h_acc} pinned near min_h_s = {min_h}",
            );
            assert!(
                h_acc < max_h * 0.999,
                "accepted h at step {i} = {h_acc} pinned at max_h_s = {max_h}",
            );
        }

        // 2) Consecutive-step ratio in a tight steady-state band.
        for (i, window) in accepted_h_history.windows(2).enumerate().skip(5) {
            let ratio = window[1] / window[0];
            assert!(
                (0.5..=2.0).contains(&ratio),
                "consecutive accepted-h ratio at step {i} = {ratio} \
                 outside the steady-state band [0.5, 2.0]; \
                 controller is oscillating ({} → {})",
                window[0],
                window[1],
            );
        }

        // 3) Rejection rate < 10 % over the full run (HNW typical
        //    benchmark: well-tuned controllers reject < 5 %).
        let total_attempts = total_accepts + total_rejects;
        // Both counters fit in usize and stay well under 2^53 for any
        // realistic test horizon, so the f64 cast is exact in practice.
        #[allow(clippy::cast_precision_loss)]
        let reject_frac = (total_rejects as f64) / (total_attempts as f64);
        assert!(
            reject_frac < 0.10,
            "rejection rate {reject_frac:.3} too high — controller wasted work \
             ({total_rejects} rejected / {total_attempts} attempts)",
        );

        // 4) Steady-state median accepted h is in a sensible band.
        let mut steady: Vec<f64> = accepted_h_history[20..].to_vec();
        steady.sort_by(f64::total_cmp);
        let median_h = steady[steady.len() / 2];
        assert!(
            (1.0e-4..=1.0e-1).contains(&median_h),
            "steady-state median accepted h = {median_h} outside the \
             1e-4..1e-1 sanity band for exp-decay at \
             rtol={rtol}, atol={atol}"
        );
    }

    /// Phase-5.D.5 — verifies the per-component norm refinement is in
    /// effect. On a multi-scale point-mass state where one component
    /// is 10⁶ and another is 1, the per-component RMS form correctly
    /// surfaces a position-component breach that the older scalar
    /// form `||e||₂ / (atol + rtol·||y||₂)` would mask. The test
    /// constructs a synthetic error derivative that produces a
    /// "drift" in the position component while the rest are zero,
    /// and confirms `weighted_error_norm` returns a value far above
    /// the controller setpoint of 1.0 — i.e., it would correctly
    /// trigger a step rejection.
    #[test]
    fn dopri54_adaptive_per_component_norm_surfaces_multi_scale_breach() {
        use openbmp_models::Integratable;

        let prev = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0e6, 0.0, 0.0),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        // Position drifts by 1 m, mass drifts by 1e-12 kg — both are
        // at scale 1.0 in scaled units (rtol=1e-9 · |y|).
        let new = PointMassState::new(
            SimTime::ZERO,
            Position3::new(1.0e6 + 1.0, 0.0, 0.0),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0 + 1.0e-12),
        );
        let err_deriv = PointMassDerivative {
            velocity_m_s: Vector3::new(1.0, 0.0, 0.0),
            acceleration_m_s2: Vector3::zeros(),
            mass_rate_kg_s: 1.0e-12,
        };
        let h = 1.0;
        let atol = 1.0e-12;
        let rtol = 1.0e-9;
        let err = new.weighted_error_norm(&prev, &err_deriv, h, atol, rtol);
        // The 1 m position breach at rtol·1e6 ≈ 1e-3 scale gives
        // scaled error ≈ 1e3 in just the position-x slot. RMS over
        // 7 components ≈ 1e3/sqrt(7) ≈ 378.
        assert!(
            err > 100.0,
            "per-component RMS form must surface multi-scale breach \
             (got {err}); a scalar `||e||₂ / ||y||₂` form would mask \
             it because ||y||₂ ≈ 1e6 swallows the 1 m drift",
        );
    }

    // -----------------------------------------------------------------
    // RigidBodyState SimState impl tests (Phase 2.1.B)
    // -----------------------------------------------------------------

    mod rigid_body {
        use super::ModelEvalError;
        use super::*;
        use crate::derivative::RigidBodyDerivative;
        use approx::assert_abs_diff_eq;
        use nalgebra::{
            Matrix3, Quaternion as NalgebraQuaternion, UnitQuaternion as NalgebraUnitQuaternion,
            Vector3,
        };
        use openbmp_core::{
            AngularVelocity3, Body, Eci, Position3, Quaternion, SimTime, UnitQuaternion, Velocity3,
        };
        use openbmp_models::Integratable;
        use openbmp_state::{MassProperties, RigidBodyState};
        use uom::si::f64::Mass;
        use uom::si::mass::kilogram;

        fn unit_state() -> RigidBodyState {
            RigidBodyState::new(
                SimTime::ZERO,
                Position3::origin(),
                Velocity3::zero(),
                Quaternion::<Body, Eci>::from_unit_quaternion(UnitQuaternion::identity()),
                AngularVelocity3::zero(),
                MassProperties::with_uniform_inertia(
                    Mass::new::<kilogram>(1.0),
                    Position3::origin(),
                    1.0,
                ),
            )
        }

        #[test]
        fn advance_by_zero_derivative_only_advances_time() {
            let s = unit_state();
            let d = RigidBodyDerivative::zero();
            let s2 = s.advance_by(0.5, &d);
            assert_abs_diff_eq!(s2.time.as_seconds(), 0.5);
            assert_abs_diff_eq!(s2.position.vector.x, 0.0);
            assert_abs_diff_eq!(s2.velocity.vector.x, 0.0);
            assert_abs_diff_eq!(s2.angular_velocity.vector.x, 0.0);
            assert_abs_diff_eq!(s2.mass_props.mass.get::<kilogram>(), 1.0);
        }

        #[test]
        fn advance_by_integrates_center_of_mass_rate() {
            let s = unit_state();
            let mut d = RigidBodyDerivative::zero();
            d.center_of_mass_rate_body_m_s = Vector3::new(0.2, -0.4, 0.6);
            let s2 = s.advance_by(0.5, &d);
            assert_abs_diff_eq!(s2.mass_props.center_of_mass_body.vector.x, 0.1);
            assert_abs_diff_eq!(s2.mass_props.center_of_mass_body.vector.y, -0.2);
            assert_abs_diff_eq!(s2.mass_props.center_of_mass_body.vector.z, 0.3);
        }

        #[test]
        fn project_renormalises_unit_quaternion() {
            let mut s = unit_state();
            // Inflate the quaternion to magnitude 2 to test the
            // renormalisation path.
            let inflated = NalgebraQuaternion::new(2.0, 0.0, 0.0, 0.0);
            s.orientation = Quaternion::<Body, Eci>::from_unit_quaternion(
                NalgebraUnitQuaternion::new_unchecked(inflated),
            );
            // Sanity: non-unit before project.
            let before = s.orientation.q.coords;
            assert_abs_diff_eq!((before.x.powi(2) + before.w.powi(2)).sqrt(), 2.0);
            <RigidBodyState as Integratable>::project(&mut s);
            let after = s.orientation.q.coords;
            let n = (after.x.powi(2) + after.y.powi(2) + after.z.powi(2) + after.w.powi(2)).sqrt();
            assert_abs_diff_eq!(n, 1.0, epsilon = 1.0e-15);
        }

        #[test]
        fn rk4_zero_derivative_runs_to_completion_under_rk4() {
            let state = unit_state();
            let dt = Duration::from_seconds(0.01);
            let derive =
                |_s: &RigidBodyState, _t: SimTime| -> Result<RigidBodyDerivative, ModelEvalError> {
                    Ok(RigidBodyDerivative::zero())
                };
            let next = Rk4FixedStep
                .advance(&state, derive, dt)
                .expect("rk4 zero step must succeed");
            assert_abs_diff_eq!(next.time.as_seconds(), 0.01);
            // Quaternion still unit after project().
            let n2 = next.orientation.q.coords.x.powi(2)
                + next.orientation.q.coords.y.powi(2)
                + next.orientation.q.coords.z.powi(2)
                + next.orientation.q.coords.w.powi(2);
            assert_abs_diff_eq!(n2, 1.0, epsilon = 1.0e-15);
        }

        /// Sign / convention check: integrate a constant body-axis
        /// rate `ω = (1, 0, 0)` rad/s (1 rad/s about body +x) for one
        /// small step and compare against the expected finite rotation.
        ///
        /// With locked convention `q_dot = 0.5 · q ⊗ [0, ω_body]`,
        /// starting from identity quaternion and ω = (1, 0, 0), the
        /// quaternion-rate at t=0 is `0.5 · 1 ⊗ (0+1i) = 0.5i`. After
        /// `dt = 0.01 s` of integration, the quaternion's i-component
        /// is approximately `0.5 * 0.01 = 0.005` (and w drops slightly
        /// after renorm). The exact closed-form rotation is
        /// `q(dt) = (cos(dt/2), sin(dt/2), 0, 0)`.
        ///
        /// Note that this *does* require the user-supplied derivative
        /// closure to compute `q_dot` correctly. Phase-2.1.B ships
        /// only the integration shape; the closure is supplied by the
        /// kernel in 2.1.D's torque-free scenario. The test here builds
        /// the closure inline, exercising only `advance_by` /
        /// `project()` from this sub-phase.
        #[test]
        fn quaternion_kinematics_sign_test_for_single_step() {
            let state = unit_state();
            let omega_body = Vector3::new(1.0, 0.0, 0.0); // 1 rad/s about +x
            let dt_s = 0.01;

            let derive =
                |s: &RigidBodyState, _t: SimTime| -> Result<RigidBodyDerivative, ModelEvalError> {
                    // q_dot = 0.5 · q ⊗ [0, ω]
                    let q = s.orientation.q.into_inner();
                    let omega_quat =
                        NalgebraQuaternion::new(0.0, omega_body[0], omega_body[1], omega_body[2]);
                    let q_dot = q * omega_quat * 0.5;
                    Ok(RigidBodyDerivative {
                        velocity_m_s_eci: Vector3::zeros(),
                        acceleration_m_s2_eci: Vector3::zeros(),
                        quaternion_rate: q_dot,
                        angular_acceleration_rad_s2_body: Vector3::zeros(),
                        mass_rate_kg_s: 0.0,
                        center_of_mass_rate_body_m_s: Vector3::zeros(),
                        inertia_rate_body: Matrix3::zeros(),
                    })
                };

            let next = Rk4FixedStep
                .advance(&state, derive, Duration::from_seconds(dt_s))
                .expect("step must succeed");

            // Closed form: (cos(dt/2), sin(dt/2), 0, 0). Note nalgebra's
            // coords order is (i, j, k, w).
            let expected_w = (dt_s / 2.0).cos();
            let expected_i = (dt_s / 2.0).sin();
            let coords = next.orientation.q.coords;
            assert_abs_diff_eq!(coords.w, expected_w, epsilon = 1.0e-12);
            assert_abs_diff_eq!(coords.x, expected_i, epsilon = 1.0e-12);
            assert_abs_diff_eq!(coords.y, 0.0, epsilon = 1.0e-15);
            assert_abs_diff_eq!(coords.z, 0.0, epsilon = 1.0e-15);
        }
    }
}
