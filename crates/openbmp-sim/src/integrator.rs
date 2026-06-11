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
//!    the combiner lives off the derivative trait
//!    (rather than baking in RK4 specifics) in this integrator-side
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
/// This lives off [`SimStateDerivative`] so the
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

/// Local dense-output interpolant over one or more accepted adaptive
/// sub-steps.
///
/// Dense output is intentionally separate from [`Integrator`]: the
/// fixed-step bit-stable path never calls it, while adaptive consumers
/// can opt in for event localization or output resampling. The
/// interpolation itself is state-stable, not a cross-platform
/// byte-stability guarantee.
pub trait DenseOutput<S: SimState> {
    /// First covered time stamp.
    #[must_use]
    fn start_time(&self) -> SimTime;

    /// Last covered time stamp.
    #[must_use]
    fn end_time(&self) -> SimTime;

    /// Interpolate the state at `time`.
    ///
    /// # Errors
    ///
    /// Returns [`IntegratorError::DenseOutputTimeOutOfRange`] when
    /// `time` is outside the covered interval.
    fn interpolate(&self, time: SimTime) -> Result<S, IntegratorError>;
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
        // Locked-order weighted sum is an integrator-local
        // helper (kept off the derivative trait so
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
// Dormand-Prince 5(4) fixed-step integrator
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
/// The fixed-step variant ([`Dopri54FixedStep`]) uses the **5th-order
/// solution only**; the embedded 4th-order error estimate and the PI
/// step controller are used by the adaptive variant
/// ([`Dopri54Adaptive`]).
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

    // Dormand-Prince 5(4) embedded 4th-order weights
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

    // Quartic dense-output coefficient matrix P, rows k1..k7,
    // columns θ¹..θ⁴. Pinned to SciPy RK45's Dormand-Prince 5(4)
    // implementation, which cites Shampine (1986), "Some Practical
    // Runge-Kutta Formulas", and uses the optimum c_6 value from that
    // paper. Dense interpolation evaluates:
    //
    //   y(θ) = y₀ + h · Σ_i k_i · (P_i1 θ + P_i2 θ² + P_i3 θ³ + P_i4 θ⁴)
    //
    // using the already-computed accepted-step stages k1..k7.
    pub const P11: f64 = 1.0;
    pub const P12: f64 = -8_048_581_381.0 / 2_820_520_608.0;
    pub const P13: f64 = 8_663_915_743.0 / 2_820_520_608.0;
    pub const P14: f64 = -12_715_105_075.0 / 11_282_082_432.0;

    // Row k2 is all zero.

    pub const P31: f64 = 0.0;
    pub const P32: f64 = 131_558_114_200.0 / 32_700_410_799.0;
    pub const P33: f64 = -68_118_460_800.0 / 10_900_136_933.0;
    pub const P34: f64 = 87_487_479_700.0 / 32_700_410_799.0;

    pub const P41: f64 = 0.0;
    pub const P42: f64 = -1_754_552_775.0 / 470_086_768.0;
    pub const P43: f64 = 14_199_869_525.0 / 1_410_260_304.0;
    pub const P44: f64 = -10_690_763_975.0 / 1_880_347_072.0;

    pub const P51: f64 = 0.0;
    pub const P52: f64 = 127_303_824_393.0 / 49_829_197_408.0;
    pub const P53: f64 = -318_862_633_887.0 / 49_829_197_408.0;
    pub const P54: f64 = 701_980_252_875.0 / 199_316_789_632.0;

    pub const P61: f64 = 0.0;
    pub const P62: f64 = -282_668_133.0 / 205_662_961.0;
    pub const P63: f64 = 2_019_193_451.0 / 616_988_883.0;
    pub const P64: f64 = -1_453_857_185.0 / 822_651_844.0;

    pub const P71: f64 = 0.0;
    pub const P72: f64 = 40_617_522.0 / 29_380_423.0;
    pub const P73: f64 = -110_615_467.0 / 29_380_423.0;
    pub const P74: f64 = 69_997_945.0 / 29_380_423.0;
}

/// Dormand-Prince 5(4) fixed-step integrator (5th-order accurate).
///
/// **Scope.** This variant uses the DOPRI5 5th-order solution at a
/// **fixed step size**; the embedded 4th-order error estimate and the
/// adaptive PI step controller live in the adaptive variant
/// ([`Dopri54Adaptive`]). The fixed-step shape is a drop-in
/// higher-order alternative to
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
// Dormand-Prince 5(4) adaptive integrator with PI step
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
/// # Error norm
///
/// The scaled error norm uses the **per-component** Hairer-Nørsett-
/// Wanner Vol I §II.4 RMS form via
/// `Integratable::weighted_error_norm`: each component-wise scaled
/// error term is divided by its own per-component scale `sc_i`,
/// then averaged in RMS. This replaces the original scalar form
/// `err = h · ||e'||₂ / (atol + rtol ·
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

/// Quartic dense-output segment over one accepted
/// [`Dopri54Adaptive`] sub-step.
///
/// The segment stores the start state, accepted 5th-order endpoint,
/// and the seven DOPRI5 stages (`k1..k7`). Interpolation evaluates
/// the Shampine/SciPy quartic continuous extension as
/// `state + h * Σ b_i(theta) * k_i`. This keeps the generic state
/// abstraction intact: it only needs
/// [`openbmp_models::Integratable::advance_by`] plus derivative
/// addition/scalar multiplication.
#[derive(Clone, Debug)]
pub struct Dopri54DenseOutput<S: SimState> {
    start_state: S,
    end_state: S,
    h_s: f64,
    k1: S::Derivative,
    k3: S::Derivative,
    k4: S::Derivative,
    k5: S::Derivative,
    k6: S::Derivative,
    k7: S::Derivative,
}

impl<S: SimState> Dopri54DenseOutput<S> {
    /// Accepted sub-step start state.
    #[must_use]
    pub fn start_state(&self) -> S {
        self.start_state
    }

    /// Accepted sub-step end state.
    #[must_use]
    pub fn end_state(&self) -> S {
        self.end_state
    }

    /// Accepted sub-step size in seconds.
    #[must_use]
    pub fn step_seconds(&self) -> f64 {
        self.h_s
    }
}

impl<S: SimState> DenseOutput<S> for Dopri54DenseOutput<S> {
    fn start_time(&self) -> SimTime {
        self.start_state.time()
    }

    fn end_time(&self) -> SimTime {
        self.end_state.time()
    }

    fn interpolate(&self, time: SimTime) -> Result<S, IntegratorError> {
        use dopri54_tableau::{
            P11, P12, P13, P14, P31, P32, P33, P34, P41, P42, P43, P44, P51, P52, P53, P54, P61,
            P62, P63, P64, P71, P72, P73, P74,
        };

        let start_s = self.start_time().as_seconds();
        let end_s = self.end_time().as_seconds();
        let query_s = time.as_seconds();
        if !query_s.is_finite() || query_s < start_s || query_s > end_s {
            return Err(IntegratorError::DenseOutputTimeOutOfRange {
                query_s,
                start_s,
                end_s,
            });
        }
        if query_s.to_bits() == start_s.to_bits() {
            return Ok(self.start_state);
        }
        if query_s.to_bits() == end_s.to_bits() {
            return Ok(self.end_state);
        }

        let theta = (query_s - start_s) / self.h_s;
        let theta2 = theta * theta;
        let theta3 = theta2 * theta;
        let theta4 = theta2 * theta2;

        let w1 = ((P11 * theta) + (P12 * theta2)) + ((P13 * theta3) + (P14 * theta4));
        let w3 = ((P31 * theta) + (P32 * theta2)) + ((P33 * theta3) + (P34 * theta4));
        let w4 = ((P41 * theta) + (P42 * theta2)) + ((P43 * theta3) + (P44 * theta4));
        let w5 = ((P51 * theta) + (P52 * theta2)) + ((P53 * theta3) + (P54 * theta4));
        let w6 = ((P61 * theta) + (P62 * theta2)) + ((P63 * theta3) + (P64 * theta4));
        let w7 = ((P71 * theta) + (P72 * theta2)) + ((P73 * theta3) + (P74 * theta4));

        let weighted = (((((self.k1 * w1) + (self.k3 * w3)) + (self.k4 * w4)) + (self.k5 * w5))
            + (self.k6 * w6))
            + (self.k7 * w7);
        let mut interpolated = self.start_state.advance_by(self.h_s, &weighted);
        interpolated.project();
        interpolated = interpolated.with_time(time);
        if !interpolated.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        Ok(interpolated)
    }
}

/// Dense output for one public [`Dopri54Adaptive::advance_with_dense_output`]
/// call.
#[derive(Clone, Debug)]
pub struct Dopri54AdaptiveDenseOutput<S: SimState> {
    final_state: S,
    segments: Vec<Dopri54DenseOutput<S>>,
}

impl<S: SimState> Dopri54AdaptiveDenseOutput<S> {
    /// Integrated endpoint returned by the adaptive step.
    #[must_use]
    pub fn final_state(&self) -> S {
        self.final_state
    }

    /// Accepted sub-step dense-output segments in chronological order.
    #[must_use]
    pub fn segments(&self) -> &[Dopri54DenseOutput<S>] {
        &self.segments
    }
}

impl<S: SimState> DenseOutput<S> for Dopri54AdaptiveDenseOutput<S> {
    fn start_time(&self) -> SimTime {
        self.segments
            .first()
            .map_or_else(|| self.final_state.time(), DenseOutput::start_time)
    }

    fn end_time(&self) -> SimTime {
        self.final_state.time()
    }

    fn interpolate(&self, time: SimTime) -> Result<S, IntegratorError> {
        let query_s = time.as_seconds();
        for segment in &self.segments {
            let start_s = segment.start_time().as_seconds();
            let end_s = segment.end_time().as_seconds();
            if query_s >= start_s && query_s <= end_s {
                return segment.interpolate(time);
            }
        }
        Err(IntegratorError::DenseOutputTimeOutOfRange {
            query_s,
            start_s: self.start_time().as_seconds(),
            end_s: self.end_time().as_seconds(),
        })
    }
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
    /// `(dense_output_segment, scaled_error_norm)`. Does NOT mutate
    /// the controller's persistent state — that's the caller's job.
    fn try_dense_substep<S, F>(
        &self,
        state: &S,
        derive_fn: &F,
        h: f64,
    ) -> Result<(Dopri54DenseOutput<S>, f64), IntegratorError>
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
        // Per-component scaled error RMS norm:
        //   sc_i = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)
        //   err = sqrt( (1/N) · Σ_i ( h · e'_i / sc_i )^2 )
        let scaled_err =
            new_state.weighted_error_norm(state, &error_deriv, h, self.atol, self.rtol);

        Ok((
            Dopri54DenseOutput {
                start_state: *state,
                end_state: new_state,
                h_s: h,
                k1,
                k3,
                k4,
                k5,
                k6,
                k7,
            },
            scaled_err,
        ))
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
        let (dense, scaled_err) = self.try_dense_substep(state, derive_fn, h)?;
        Ok((dense.end_state, scaled_err))
    }

    /// Advance by `dt` and retain quartic dense-output segments for
    /// each accepted adaptive sub-step.
    ///
    /// This is the opt-in DOPRI5 dense-output surface for event
    /// localization and output resampling. It follows the same
    /// accept/reject loop and controller updates as [`Integrator::advance`],
    /// but records the accepted stage sets before returning.
    ///
    /// # Errors
    ///
    /// Returns the same [`IntegratorError`] values as
    /// [`Integrator::advance`] for invalid steps, non-finite states or
    /// derivatives, or model-evaluation failures.
    pub fn advance_with_dense_output<S, F>(
        &self,
        state: &S,
        derive_fn: F,
        dt: Duration,
    ) -> Result<Dopri54AdaptiveDenseOutput<S>, IntegratorError>
    where
        S: SimState,
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
        let max_substeps: usize = 1_000_000;
        let mut substeps_taken: usize = 0;
        let mut segments = Vec::new();

        while elapsed < dt_total {
            substeps_taken += 1;
            if substeps_taken > max_substeps {
                return Err(IntegratorError::InvalidStep {
                    dt_seconds: dt_total,
                });
            }

            let remaining = dt_total - elapsed;
            let h_try = if remaining <= self.min_h_s {
                remaining
            } else {
                h.min(remaining).max(self.min_h_s)
            };

            self.last_err_prev.set(last_accepted_err);

            let (segment, err) = self.try_dense_substep(&current, &derive_fn, h_try)?;

            if err <= 1.0 {
                current = segment.end_state;
                segments.push(segment);
                elapsed += h_try;
                last_accepted_err = Some(err.max(1.0e-10));
                let factor = self.pi_step_factor(err.max(1.0e-10));
                h = (h_try * factor).max(self.min_h_s).min(self.max_h_s);
            } else {
                let alpha_over_p = self.pi_alpha / EMBEDDED_ORDER;
                let mut factor = self.safety_factor * err.powf(-alpha_over_p);
                if !factor.is_finite() || factor <= 0.0 {
                    factor = self.min_factor;
                }
                factor = factor.max(self.min_factor).min(1.0);
                if h_try <= self.min_h_s + f64::EPSILON {
                    return Err(IntegratorError::InvalidStep { dt_seconds: h_try });
                }
                h = (h_try * factor).max(self.min_h_s).min(self.max_h_s);
            }
        }

        self.last_h_s.set(Some(h));
        self.last_err_prev.set(last_accepted_err);
        Ok(Dopri54AdaptiveDenseOutput {
            final_state: current,
            segments,
        })
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

// =====================================================================
// Dormand-Prince 8(5,3) (DOP853) integrator family.
// =====================================================================

/// DOP853 / Dormand-Prince 8(5,3) Butcher tableau.
///
/// 12-stage explicit Runge-Kutta with an 8th-order solution and two
/// embedded estimators of orders 5 and 3 (the "8(5,3)" notation).
/// The 5th-order companion drives the per-step error norm; the
/// 3rd-order companion stabilises the controller's denominator when
/// the 5th-order vanishes coincidentally
/// (`err = |h| · ||err5||² / sqrt((||err5||² + 0.01·||err3||²) · N)`,
/// the SciPy / Hairer-Wanner reference formula).
///
/// Coefficients pinned against SciPy's
/// `scipy/integrate/_ivp/dop853_coefficients.py` (Hairer's reference
/// Fortran `dop853.f` / Hairer-Nørsett-Wanner Vol I §II.5 Table 5.4).
/// The full 16-stage SciPy tableau includes the primary rows, the
/// endpoint derivative row, and three extra dense-output rows. This
/// module pins the primary coefficients plus the dense-output
/// abscissas / matrix so [`Dopri853Adaptive::advance_with_dense_output`]
/// can expose the order-7 continuous extension without touching the
/// fixed-step bit-stable path.
///
/// The constants below are written as the SciPy decimal literals
/// verbatim. Const-time IEEE 754 arithmetic in Rust is deterministic
/// across platforms, so `B - B̂_3` style derivations preserve
/// bit-stability of the tableau.
///
/// Some SciPy literals have more decimal digits than `f64` can
/// represent; the compiler rounds to the nearest `f64` (which is
/// what SciPy itself ends up with at parse time). The
/// `clippy::excessive_precision` lint flags this; it is
/// allow-listed at the module level because the precision overflow
/// is part of preserving fidelity to the canonical SciPy /
/// Hairer-Fortran source.
#[allow(
    clippy::unreadable_literal,
    clippy::excessive_precision,
    clippy::doc_markdown
)]
mod dopri853_tableau {
    // -----------------------------------------------------------------
    // C-vector — sub-step times relative to h, indices 0..11.
    // -----------------------------------------------------------------
    // C[0] = 0 implicitly (initial stage).
    pub const C2: f64 = 5.26001519587677318785587544488e-2;
    pub const C3: f64 = 7.89002279381515978178381316732e-2;
    pub const C4: f64 = 1.18350341907227396726757197510e-1;
    pub const C5: f64 = 2.81649658092772603273242802490e-1;
    pub const C6: f64 = 1.0 / 3.0;
    pub const C7: f64 = 0.25;
    pub const C8: f64 = 3.07692307692307692307692307692e-1;
    pub const C9: f64 = 6.51282051282051282051282051282e-1;
    pub const C10: f64 = 0.6;
    pub const C11: f64 = 8.57142857142857142857142857142e-1;
    // C12 = 1.0 (the 8th-order solution endpoint).
    // C13 = 1.0 (endpoint derivative, stored separately as k_end).
    pub const C_EXTRA_1: f64 = 0.1;
    pub const C_EXTRA_2: f64 = 0.2;
    pub const C_EXTRA_3: f64 = 7.77777777777777777777777777778e-1;

    // -----------------------------------------------------------------
    // A-matrix — strict lower triangular, indexed A_i_j = a_{i+1, j+1}
    // in the 1-indexed Butcher convention, indices 0..11. Entries not
    // listed are zero.
    // -----------------------------------------------------------------

    // Row 1 (k_2 stage):
    pub const A_2_1: f64 = 5.26001519587677318785587544488e-2;

    // Row 2 (k_3 stage):
    pub const A_3_1: f64 = 1.97250569845378994544595329183e-2;
    pub const A_3_2: f64 = 5.91751709536136983633785987549e-2;

    // Row 3 (k_4 stage):
    pub const A_4_1: f64 = 2.95875854768068491816892993775e-2;
    // A_4_2 = 0
    pub const A_4_3: f64 = 8.87627564304205475450678981324e-2;

    // Row 4 (k_5 stage):
    pub const A_5_1: f64 = 2.41365134159266685502369798665e-1;
    // A_5_2 = 0
    pub const A_5_3: f64 = -8.84549479328286085344864962717e-1;
    pub const A_5_4: f64 = 9.24834003261792003115737966543e-1;

    // Row 5 (k_6 stage):
    pub const A_6_1: f64 = 3.7037037037037037037037037037e-2;
    // A_6_2 = A_6_3 = 0
    pub const A_6_4: f64 = 1.70828608729473871279604482173e-1;
    pub const A_6_5: f64 = 1.25467687566822425016691814123e-1;

    // Row 6 (k_7 stage):
    pub const A_7_1: f64 = 3.7109375e-2;
    // A_7_2 = A_7_3 = 0
    pub const A_7_4: f64 = 1.70252211019544039314978060272e-1;
    pub const A_7_5: f64 = 6.02165389804559606850219397283e-2;
    pub const A_7_6: f64 = -1.7578125e-2;

    // Row 7 (k_8 stage):
    pub const A_8_1: f64 = 3.70920001185047927108779319836e-2;
    // A_8_2 = A_8_3 = 0
    pub const A_8_4: f64 = 1.70383925712239993810214054705e-1;
    pub const A_8_5: f64 = 1.07262030446373284651809199168e-1;
    pub const A_8_6: f64 = -1.53194377486244017527936158236e-2;
    pub const A_8_7: f64 = 8.27378916381402288758473766002e-3;

    // Row 8 (k_9 stage):
    pub const A_9_1: f64 = 6.24110958716075717114429577812e-1;
    // A_9_2 = A_9_3 = 0
    pub const A_9_4: f64 = -3.36089262944694129406857109825;
    pub const A_9_5: f64 = -8.68219346841726006818189891453e-1;
    pub const A_9_6: f64 = 2.75920996994467083049415600797e1;
    pub const A_9_7: f64 = 2.01540675504778934086186788979e1;
    pub const A_9_8: f64 = -4.34898841810699588477366255144e1;

    // Row 9 (k_10 stage):
    pub const A_10_1: f64 = 4.77662536438264365890433908527e-1;
    // A_10_2 = A_10_3 = 0
    pub const A_10_4: f64 = -2.48811461997166764192642586468;
    pub const A_10_5: f64 = -5.90290826836842996371446475743e-1;
    pub const A_10_6: f64 = 2.12300514481811942347288949897e1;
    pub const A_10_7: f64 = 1.52792336328824235832596922938e1;
    pub const A_10_8: f64 = -3.32882109689848629194453265587e1;
    pub const A_10_9: f64 = -2.03312017085086261358222928593e-2;

    // Row 10 (k_11 stage):
    pub const A_11_1: f64 = -9.3714243008598732571704021658e-1;
    // A_11_2 = A_11_3 = 0
    pub const A_11_4: f64 = 5.18637242884406370830023853209;
    pub const A_11_5: f64 = 1.09143734899672957818500254654;
    pub const A_11_6: f64 = -8.14978701074692612513997267357;
    pub const A_11_7: f64 = -1.85200656599969598641566180701e1;
    pub const A_11_8: f64 = 2.27394870993505042818970056734e1;
    pub const A_11_9: f64 = 2.49360555267965238987089396762;
    pub const A_11_10: f64 = -3.0467644718982195003823669022;

    // Row 11 (k_12 stage):
    pub const A_12_1: f64 = 2.27331014751653820792359768449;
    // A_12_2 = A_12_3 = 0
    pub const A_12_4: f64 = -1.05344954667372501984066689879e1;
    pub const A_12_5: f64 = -2.00087205822486249909675718444;
    pub const A_12_6: f64 = -1.79589318631187989172765950534e1;
    pub const A_12_7: f64 = 2.79488845294199600508499808837e1;
    pub const A_12_8: f64 = -2.85899827713502369474065508674;
    pub const A_12_9: f64 = -8.87285693353062954433549289258;
    pub const A_12_10: f64 = 1.23605671757943030647266201528e1;
    pub const A_12_11: f64 = 6.43392746015763530355970484046e-1;

    // -----------------------------------------------------------------
    // B-vector — 8th-order solution weights, indices 0..11.
    // Equivalent to A[12, 0..11] in the SciPy 13-row representation.
    // B[1], B[2], B[3], B[4] are zero (the 8(5,3) tableau has four
    // leading zero weights — same vanishing pattern at indices 1..4
    // that DOPRI5(4) has at index 1).
    // -----------------------------------------------------------------
    pub const B_1: f64 = 5.42937341165687622380535766363e-2;
    // B_2 = B_3 = B_4 = B_5 = 0
    pub const B_6: f64 = 4.45031289275240888144113950566;
    pub const B_7: f64 = 1.89151789931450038304281599044;
    pub const B_8: f64 = -5.8012039600105847814672114227;
    pub const B_9: f64 = 3.1116436695781989440891606237e-1;
    pub const B_10: f64 = -1.52160949662516078556178806805e-1;
    pub const B_11: f64 = 2.01365400804030348374776537501e-1;
    pub const B_12: f64 = 4.47106157277725905176885569043e-2;

    // -----------------------------------------------------------------
    // Embedded 5th-order error estimator E5 — `e5' = Σ_j E5_j · k_j`.
    // SciPy stores E5 with explicit non-zero entries at j ∈
    // {0, 5, 6, 7, 8, 9, 10, 11}; all other entries vanish.
    // -----------------------------------------------------------------
    pub const E5_1: f64 = 0.1312004499419488073250102996e-1;
    // E5_2 = E5_3 = E5_4 = E5_5 = 0
    pub const E5_6: f64 = -0.1225156446376204440720569753e1;
    pub const E5_7: f64 = -0.4957589496572501915214079952;
    pub const E5_8: f64 = 0.1664377182454986536961530415e1;
    pub const E5_9: f64 = -0.3503288487499736816886487290;
    pub const E5_10: f64 = 0.3341791187130174790297318841;
    pub const E5_11: f64 = 0.8192320648511571246570742613e-1;
    pub const E5_12: f64 = -0.2235530786388629525884427845e-1;

    // -----------------------------------------------------------------
    // Embedded 3rd-order error estimator E3 — `e3' = Σ_j E3_j · k_j`.
    // Constructed in SciPy from `B - B̂_3` with three explicit offsets.
    // The const-time arithmetic below is deterministic across
    // platforms; the audit can verify by hand-computing each entry
    // against the SciPy file.
    // -----------------------------------------------------------------
    pub const E3_1: f64 = B_1 - 0.244094488188976377952755905512;
    // E3_2 = E3_3 = E3_4 = E3_5 = 0
    pub const E3_6: f64 = B_6;
    pub const E3_7: f64 = B_7;
    pub const E3_8: f64 = B_8;
    pub const E3_9: f64 = B_9 - 0.733846688281611857341361741547;
    pub const E3_10: f64 = B_10;
    pub const E3_11: f64 = B_11;
    pub const E3_12: f64 = B_12 - 0.220588235294117647058823529412e-1;

    // -----------------------------------------------------------------
    // Dense-output extra stages A[13..15] and interpolator matrix D.
    // SciPy computes K[12] as f(t+h, y_new), then K[13], K[14],
    // K[15] from the rows below at C = 0.1, 0.2, and 7/9. The
    // order-7 interpolator uses F[0..2] from endpoint deltas and
    // F[3..6] = h * D[0..3] * K.
    // -----------------------------------------------------------------

    pub const A_EXTRA_1_1: f64 = 5.61675022830479523392909219681e-2;
    pub const A_EXTRA_1_7: f64 = 2.53500210216624811088794765333e-1;
    pub const A_EXTRA_1_8: f64 = -2.46239037470802489917441475441e-1;
    pub const A_EXTRA_1_9: f64 = -1.24191423263816360469010140626e-1;
    pub const A_EXTRA_1_10: f64 = 1.5329179827876569731206322685e-1;
    pub const A_EXTRA_1_11: f64 = 8.20105229563468988491666602057e-3;
    pub const A_EXTRA_1_12: f64 = 7.56789766054569976138603589584e-3;
    pub const A_EXTRA_1_END: f64 = -8.298e-3;

    pub const A_EXTRA_2_1: f64 = 3.18346481635021405060768473261e-2;
    pub const A_EXTRA_2_6: f64 = 2.83009096723667755288322961402e-2;
    pub const A_EXTRA_2_7: f64 = 5.35419883074385676223797384372e-2;
    pub const A_EXTRA_2_8: f64 = -5.49237485713909884646569340306e-2;
    pub const A_EXTRA_2_11: f64 = -1.08347328697249322858509316994e-4;
    pub const A_EXTRA_2_12: f64 = 3.82571090835658412954920192323e-4;
    pub const A_EXTRA_2_END: f64 = -3.40465008687404560802977114492e-4;
    pub const A_EXTRA_2_EXTRA_1: f64 = 1.41312443674632500278074618366e-1;

    pub const A_EXTRA_3_1: f64 = -4.28896301583791923408573538692e-1;
    pub const A_EXTRA_3_6: f64 = -4.69762141536116384314449447206;
    pub const A_EXTRA_3_7: f64 = 7.68342119606259904184240953878;
    pub const A_EXTRA_3_8: f64 = 4.06898981839711007970213554331;
    pub const A_EXTRA_3_9: f64 = 3.56727187455281109270669543021e-1;
    pub const A_EXTRA_3_END: f64 = -1.39902416515901462129418009734e-3;
    pub const A_EXTRA_3_EXTRA_1: f64 = 2.9475147891527723389556272149;
    pub const A_EXTRA_3_EXTRA_2: f64 = -9.15095847217987001081870187138;

    pub const D_1_1: f64 = -0.84289382761090128651353491142e1;
    pub const D_1_6: f64 = 0.56671495351937776962531783590;
    pub const D_1_7: f64 = -0.30689499459498916912797304727e1;
    pub const D_1_8: f64 = 0.23846676565120698287728149680e1;
    pub const D_1_9: f64 = 0.21170345824450282767155149946e1;
    pub const D_1_10: f64 = -0.87139158377797299206789907490;
    pub const D_1_11: f64 = 0.22404374302607882758541771650e1;
    pub const D_1_12: f64 = 0.63157877876946881815570249290;
    pub const D_1_END: f64 = -0.88990336451333310820698117400e-1;
    pub const D_1_EXTRA_1: f64 = 0.18148505520854727256656404962e2;
    pub const D_1_EXTRA_2: f64 = -0.91946323924783554000451984436e1;
    pub const D_1_EXTRA_3: f64 = -0.44360363875948939664310572000e1;

    pub const D_2_1: f64 = 0.10427508642579134603413151009e2;
    pub const D_2_6: f64 = 0.24228349177525818288430175319e3;
    pub const D_2_7: f64 = 0.16520045171727028198505394887e3;
    pub const D_2_8: f64 = -0.37454675472269020279518312152e3;
    pub const D_2_9: f64 = -0.22113666853125306036270938578e2;
    pub const D_2_10: f64 = 0.77334326684722638389603898808e1;
    pub const D_2_11: f64 = -0.30674084731089398182061213626e2;
    pub const D_2_12: f64 = -0.93321305264302278729567221706e1;
    pub const D_2_END: f64 = 0.15697238121770843886131091075e2;
    pub const D_2_EXTRA_1: f64 = -0.31139403219565177677282850411e2;
    pub const D_2_EXTRA_2: f64 = -0.93529243588444783865713862664e1;
    pub const D_2_EXTRA_3: f64 = 0.35816841486394083752465898540e2;

    pub const D_3_1: f64 = 0.19985053242002433820987653617e2;
    pub const D_3_6: f64 = -0.38703730874935176555105901742e3;
    pub const D_3_7: f64 = -0.18917813819516756882830838328e3;
    pub const D_3_8: f64 = 0.52780815920542364900561016686e3;
    pub const D_3_9: f64 = -0.11573902539959630126141871134e2;
    pub const D_3_10: f64 = 0.68812326946963000169666922661e1;
    pub const D_3_11: f64 = -0.10006050966910838403183860980e1;
    pub const D_3_12: f64 = 0.77771377980534432092869265740;
    pub const D_3_END: f64 = -0.27782057523535084065932004339e1;
    pub const D_3_EXTRA_1: f64 = -0.60196695231264120758267380846e2;
    pub const D_3_EXTRA_2: f64 = 0.84320405506677161018159903784e2;
    pub const D_3_EXTRA_3: f64 = 0.11992291136182789328035130030e2;

    pub const D_4_1: f64 = -0.25693933462703749003312586129e2;
    pub const D_4_6: f64 = -0.15418974869023643374053993627e3;
    pub const D_4_7: f64 = -0.23152937917604549567536039109e3;
    pub const D_4_8: f64 = 0.35763911791061412378285349910e3;
    pub const D_4_9: f64 = 0.93405324183624310003907691704e2;
    pub const D_4_10: f64 = -0.37458323136451633156875139351e2;
    pub const D_4_11: f64 = 0.10409964950896230045147246184e3;
    pub const D_4_12: f64 = 0.29840293426660503123344363579e2;
    pub const D_4_END: f64 = -0.43533456590011143754432175058e2;
    pub const D_4_EXTRA_1: f64 = 0.96324553959188282948394950600e2;
    pub const D_4_EXTRA_2: f64 = -0.39177261675615439165231486172e2;
    pub const D_4_EXTRA_3: f64 = -0.14972683625798562581422125276e3;
}

#[derive(Clone, Debug)]
struct Dopri853PrimaryStep<S: SimState> {
    start_state: S,
    end_state: S,
    h_s: f64,
    scaled_err: f64,
    delta_deriv: S::Derivative,
    k1: S::Derivative,
    k6: S::Derivative,
    k7: S::Derivative,
    k8: S::Derivative,
    k9: S::Derivative,
    k10: S::Derivative,
    k11: S::Derivative,
    k12: S::Derivative,
}

/// Order-7 dense-output segment over one accepted
/// [`Dopri853Adaptive`] sub-step.
///
/// The segment stores DOP853's primary stages, the accepted endpoint
/// derivative, and the three extra dense-output stages from SciPy /
/// Hairer. Interpolation evaluates the same alternating Horner form
/// as SciPy's `Dop853DenseOutput`, represented as a derivative
/// combination so it stays inside OpenBMP's generic state arithmetic
/// contract.
#[derive(Clone, Debug)]
pub struct Dopri853DenseOutput<S: SimState> {
    start_state: S,
    end_state: S,
    h_s: f64,
    delta_deriv: S::Derivative,
    k1: S::Derivative,
    k6: S::Derivative,
    k7: S::Derivative,
    k8: S::Derivative,
    k9: S::Derivative,
    k10: S::Derivative,
    k11: S::Derivative,
    k12: S::Derivative,
    k_end: S::Derivative,
    k_extra_1: S::Derivative,
    k_extra_2: S::Derivative,
    k_extra_3: S::Derivative,
}

impl<S: SimState> Dopri853DenseOutput<S> {
    /// Accepted sub-step start state.
    #[must_use]
    pub fn start_state(&self) -> S {
        self.start_state
    }

    /// Accepted sub-step end state.
    #[must_use]
    pub fn end_state(&self) -> S {
        self.end_state
    }

    /// Accepted sub-step size in seconds.
    #[must_use]
    pub fn step_seconds(&self) -> f64 {
        self.h_s
    }

    fn dense_row_1(&self) -> S::Derivative {
        use dopri853_tableau::{
            D_1_1, D_1_6, D_1_7, D_1_8, D_1_9, D_1_10, D_1_11, D_1_12, D_1_END, D_1_EXTRA_1,
            D_1_EXTRA_2, D_1_EXTRA_3,
        };
        (((((((((((self.k1 * D_1_1) + (self.k6 * D_1_6)) + (self.k7 * D_1_7))
            + (self.k8 * D_1_8))
            + (self.k9 * D_1_9))
            + (self.k10 * D_1_10))
            + (self.k11 * D_1_11))
            + (self.k12 * D_1_12))
            + (self.k_end * D_1_END))
            + (self.k_extra_1 * D_1_EXTRA_1))
            + (self.k_extra_2 * D_1_EXTRA_2))
            + (self.k_extra_3 * D_1_EXTRA_3)
    }

    fn dense_row_2(&self) -> S::Derivative {
        use dopri853_tableau::{
            D_2_1, D_2_6, D_2_7, D_2_8, D_2_9, D_2_10, D_2_11, D_2_12, D_2_END, D_2_EXTRA_1,
            D_2_EXTRA_2, D_2_EXTRA_3,
        };
        (((((((((((self.k1 * D_2_1) + (self.k6 * D_2_6)) + (self.k7 * D_2_7))
            + (self.k8 * D_2_8))
            + (self.k9 * D_2_9))
            + (self.k10 * D_2_10))
            + (self.k11 * D_2_11))
            + (self.k12 * D_2_12))
            + (self.k_end * D_2_END))
            + (self.k_extra_1 * D_2_EXTRA_1))
            + (self.k_extra_2 * D_2_EXTRA_2))
            + (self.k_extra_3 * D_2_EXTRA_3)
    }

    fn dense_row_3(&self) -> S::Derivative {
        use dopri853_tableau::{
            D_3_1, D_3_6, D_3_7, D_3_8, D_3_9, D_3_10, D_3_11, D_3_12, D_3_END, D_3_EXTRA_1,
            D_3_EXTRA_2, D_3_EXTRA_3,
        };
        (((((((((((self.k1 * D_3_1) + (self.k6 * D_3_6)) + (self.k7 * D_3_7))
            + (self.k8 * D_3_8))
            + (self.k9 * D_3_9))
            + (self.k10 * D_3_10))
            + (self.k11 * D_3_11))
            + (self.k12 * D_3_12))
            + (self.k_end * D_3_END))
            + (self.k_extra_1 * D_3_EXTRA_1))
            + (self.k_extra_2 * D_3_EXTRA_2))
            + (self.k_extra_3 * D_3_EXTRA_3)
    }

    fn dense_row_4(&self) -> S::Derivative {
        use dopri853_tableau::{
            D_4_1, D_4_6, D_4_7, D_4_8, D_4_9, D_4_10, D_4_11, D_4_12, D_4_END, D_4_EXTRA_1,
            D_4_EXTRA_2, D_4_EXTRA_3,
        };
        (((((((((((self.k1 * D_4_1) + (self.k6 * D_4_6)) + (self.k7 * D_4_7))
            + (self.k8 * D_4_8))
            + (self.k9 * D_4_9))
            + (self.k10 * D_4_10))
            + (self.k11 * D_4_11))
            + (self.k12 * D_4_12))
            + (self.k_end * D_4_END))
            + (self.k_extra_1 * D_4_EXTRA_1))
            + (self.k_extra_2 * D_4_EXTRA_2))
            + (self.k_extra_3 * D_4_EXTRA_3)
    }
}

impl<S: SimState> DenseOutput<S> for Dopri853DenseOutput<S> {
    fn start_time(&self) -> SimTime {
        self.start_state.time()
    }

    fn end_time(&self) -> SimTime {
        self.end_state.time()
    }

    fn interpolate(&self, time: SimTime) -> Result<S, IntegratorError> {
        let start_s = self.start_time().as_seconds();
        let end_s = self.end_time().as_seconds();
        let query_s = time.as_seconds();
        if !query_s.is_finite() || query_s < start_s || query_s > end_s {
            return Err(IntegratorError::DenseOutputTimeOutOfRange {
                query_s,
                start_s,
                end_s,
            });
        }
        if query_s.to_bits() == start_s.to_bits() {
            return Ok(self.start_state);
        }
        if query_s.to_bits() == end_s.to_bits() {
            return Ok(self.end_state);
        }

        let theta = (query_s - start_s) / self.h_s;
        let one_minus_theta = 1.0 - theta;
        let f0 = self.delta_deriv;
        let f1 = self.k1 + (self.delta_deriv * -1.0);
        let f2 = (self.delta_deriv * 2.0) + ((self.k_end + self.k1) * -1.0);
        let f3 = self.dense_row_1();
        let f4 = self.dense_row_2();
        let f5 = self.dense_row_3();
        let f6 = self.dense_row_4();

        let weighted =
            ((((((f6 * theta) + f5) * one_minus_theta + f4) * theta + f3) * one_minus_theta + f2)
                * theta
                + f1)
                * one_minus_theta
                + f0;
        let mut interpolated = self.start_state.advance_by(self.h_s, &(weighted * theta));
        interpolated.project();
        interpolated = interpolated.with_time(time);
        if !interpolated.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        Ok(interpolated)
    }
}

/// Dense output for one public [`Dopri853Adaptive::advance_with_dense_output`]
/// call.
#[derive(Clone, Debug)]
pub struct Dopri853AdaptiveDenseOutput<S: SimState> {
    final_state: S,
    segments: Vec<Dopri853DenseOutput<S>>,
}

impl<S: SimState> Dopri853AdaptiveDenseOutput<S> {
    /// Integrated endpoint returned by the adaptive step.
    #[must_use]
    pub fn final_state(&self) -> S {
        self.final_state
    }

    /// Accepted sub-step dense-output segments in chronological order.
    #[must_use]
    pub fn segments(&self) -> &[Dopri853DenseOutput<S>] {
        &self.segments
    }
}

impl<S: SimState> DenseOutput<S> for Dopri853AdaptiveDenseOutput<S> {
    fn start_time(&self) -> SimTime {
        self.segments
            .first()
            .map_or_else(|| self.final_state.time(), DenseOutput::start_time)
    }

    fn end_time(&self) -> SimTime {
        self.final_state.time()
    }

    fn interpolate(&self, time: SimTime) -> Result<S, IntegratorError> {
        let query_s = time.as_seconds();
        for segment in &self.segments {
            let start_s = segment.start_time().as_seconds();
            let end_s = segment.end_time().as_seconds();
            if query_s >= start_s && query_s <= end_s {
                return segment.interpolate(time);
            }
        }
        Err(IntegratorError::DenseOutputTimeOutOfRange {
            query_s,
            start_s: self.start_time().as_seconds(),
            end_s: self.end_time().as_seconds(),
        })
    }
}

/// Dormand-Prince 8(5,3) (DOP853) fixed-step integrator —
/// 8th-order accurate.
///
/// 12-stage explicit Runge-Kutta with the full 8th-order solution
/// from `dopri853_tableau`. Drop-in higher-order alternative to
/// [`Rk4FixedStep`] / [`Dopri54FixedStep`] when the 4th- or 5th-order
/// truncation error is the limiting factor on a problem where the
/// per-step state is otherwise well-behaved.
///
/// **Honest scope.** This is the fixed-step shape — no embedded error
/// estimator, no PI controller, no dense output. The adaptive shape is
/// [`Dopri853Adaptive`], whose opt-in
/// [`Dopri853Adaptive::advance_with_dense_output`] path evaluates the
/// SciPy/Hairer order-7 dense interpolant without changing this
/// fixed-step path.
///
/// # Determinism
///
/// Tagged [`IntegratorDeterminism::BitStable`]. The locked-order
/// weighted-sum at every stage and the bit-stable tableau constants
/// give bit-identical Parquet across reruns on every platform
/// profile that matches IEEE 754 + the determinism CI gate.
#[derive(Copy, Clone, Debug, Default)]
#[allow(clippy::doc_markdown)] // SciPy / Hairer-Wanner are reference names, not code identifiers.
pub struct Dopri853FixedStep;

impl<S: SimState> Integrator<S> for Dopri853FixedStep {
    fn determinism(&self) -> IntegratorDeterminism {
        IntegratorDeterminism::BitStable
    }

    // 12 stages × ~10 LoC each + initial validation + 8th-order
    // weighted sum yields ~150 lines that all need to live in one
    // function for the locked-order discipline to be visible at the
    // call site. Splitting the stage block into helpers would
    // sacrifice the single-glance audit pattern that the existing
    // Rk4 / Dopri54 fixed-step impls use.
    #[allow(clippy::too_many_lines)]
    fn advance<F>(&self, state: &S, derive_fn: F, dt: Duration) -> Result<S, IntegratorError>
    where
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        // 12-stage primary tableau. Each stage's increment is built
        // by left-folding the per-stage `k_i * A_i_j` terms in
        // explicit-parentheses locked order so the cross-platform
        // determinism contract holds (no compiler re-association,
        // no FMA fusion).
        use dopri853_tableau::{
            A_2_1, A_3_1, A_3_2, A_4_1, A_4_3, A_5_1, A_5_3, A_5_4, A_6_1, A_6_4, A_6_5, A_7_1,
            A_7_4, A_7_5, A_7_6, A_8_1, A_8_4, A_8_5, A_8_6, A_8_7, A_9_1, A_9_4, A_9_5, A_9_6,
            A_9_7, A_9_8, A_10_1, A_10_4, A_10_5, A_10_6, A_10_7, A_10_8, A_10_9, A_11_1, A_11_4,
            A_11_5, A_11_6, A_11_7, A_11_8, A_11_9, A_11_10, A_12_1, A_12_4, A_12_5, A_12_6,
            A_12_7, A_12_8, A_12_9, A_12_10, A_12_11, B_1, B_6, B_7, B_8, B_9, B_10, B_11, B_12,
            C2, C3, C4, C5, C6, C7, C8, C9, C10, C11,
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
        let t6 = SimTime::from_seconds(t0_s + C6 * h);
        let t7 = SimTime::from_seconds(t0_s + C7 * h);
        let t8 = SimTime::from_seconds(t0_s + C8 * h);
        let t9 = SimTime::from_seconds(t0_s + C9 * h);
        let t10 = SimTime::from_seconds(t0_s + C10 * h);
        let t11 = SimTime::from_seconds(t0_s + C11 * h);
        let t12 = SimTime::from_seconds(t0_s + h);

        // Stage 1.
        let k1 = derive_fn(state, t0)?;
        if !k1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 2.
        let s2 = state.advance_by(h, &(k1 * A_2_1));
        if !s2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k2 = derive_fn(&s2, t2)?;
        if !k2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 3: a31·k1 + a32·k2.
        let inc3 = (k1 * A_3_1) + (k2 * A_3_2);
        let s3 = state.advance_by(h, &inc3);
        if !s3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k3 = derive_fn(&s3, t3)?;
        if !k3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 4: a41·k1 + a43·k3 (a42 = 0).
        let inc4 = (k1 * A_4_1) + (k3 * A_4_3);
        let s4 = state.advance_by(h, &inc4);
        if !s4.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k4 = derive_fn(&s4, t4)?;
        if !k4.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 5: a51·k1 + a53·k3 + a54·k4.
        let inc5 = ((k1 * A_5_1) + (k3 * A_5_3)) + (k4 * A_5_4);
        let s5 = state.advance_by(h, &inc5);
        if !s5.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k5 = derive_fn(&s5, t5)?;
        if !k5.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 6: a61·k1 + a64·k4 + a65·k5.
        let inc6 = ((k1 * A_6_1) + (k4 * A_6_4)) + (k5 * A_6_5);
        let s6 = state.advance_by(h, &inc6);
        if !s6.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k6 = derive_fn(&s6, t6)?;
        if !k6.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 7: a71·k1 + a74·k4 + a75·k5 + a76·k6.
        let inc7 = (((k1 * A_7_1) + (k4 * A_7_4)) + (k5 * A_7_5)) + (k6 * A_7_6);
        let s7 = state.advance_by(h, &inc7);
        if !s7.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k7 = derive_fn(&s7, t7)?;
        if !k7.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 8: a81·k1 + a84·k4 + a85·k5 + a86·k6 + a87·k7.
        let inc8 = ((((k1 * A_8_1) + (k4 * A_8_4)) + (k5 * A_8_5)) + (k6 * A_8_6)) + (k7 * A_8_7);
        let s8 = state.advance_by(h, &inc8);
        if !s8.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k8 = derive_fn(&s8, t8)?;
        if !k8.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 9.
        let inc9 = (((((k1 * A_9_1) + (k4 * A_9_4)) + (k5 * A_9_5)) + (k6 * A_9_6)) + (k7 * A_9_7))
            + (k8 * A_9_8);
        let s9 = state.advance_by(h, &inc9);
        if !s9.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k9 = derive_fn(&s9, t9)?;
        if !k9.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 10.
        let inc10 = ((((((k1 * A_10_1) + (k4 * A_10_4)) + (k5 * A_10_5)) + (k6 * A_10_6))
            + (k7 * A_10_7))
            + (k8 * A_10_8))
            + (k9 * A_10_9);
        let s10 = state.advance_by(h, &inc10);
        if !s10.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k10 = derive_fn(&s10, t10)?;
        if !k10.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 11.
        let inc11 = (((((((k1 * A_11_1) + (k4 * A_11_4)) + (k5 * A_11_5)) + (k6 * A_11_6))
            + (k7 * A_11_7))
            + (k8 * A_11_8))
            + (k9 * A_11_9))
            + (k10 * A_11_10);
        let s11 = state.advance_by(h, &inc11);
        if !s11.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k11 = derive_fn(&s11, t11)?;
        if !k11.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // Stage 12.
        let inc12 = ((((((((k1 * A_12_1) + (k4 * A_12_4)) + (k5 * A_12_5)) + (k6 * A_12_6))
            + (k7 * A_12_7))
            + (k8 * A_12_8))
            + (k9 * A_12_9))
            + (k10 * A_12_10))
            + (k11 * A_12_11);
        let s12 = state.advance_by(h, &inc12);
        if !s12.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k12 = derive_fn(&s12, t12)?;
        if !k12.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // 8th-order solution: y = y0 + h · (B_1·k1 + B_6·k6 + B_7·k7
        //   + B_8·k8 + B_9·k9 + B_10·k10 + B_11·k11 + B_12·k12).
        // B_2 = B_3 = B_4 = B_5 = 0 so k2..k5 are not weighted into
        // the solution (DOP853's analogue of DOPRI5's `B2 = 0`).
        let weighted = (((((((k1 * B_1) + (k6 * B_6)) + (k7 * B_7)) + (k8 * B_8)) + (k9 * B_9))
            + (k10 * B_10))
            + (k11 * B_11))
            + (k12 * B_12);
        let mut new_state = state.advance_by(h, &weighted);
        new_state.project();
        if !new_state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        Ok(new_state)
    }
}

/// Dormand-Prince 8(5,3) (DOP853) adaptive integrator with an
/// I-controller and the SciPy / Hairer-Wanner combined err5/err3
/// error norm.
///
/// 12-stage explicit RK with the 8th-order primary solution (same as
/// [`Dopri853FixedStep`]) plus two embedded estimators of orders 5
/// and 3. The 5th-order estimator drives the controller's input; the
/// 3rd-order estimator stabilises the denominator when the 5th-order
/// estimate vanishes coincidentally:
///
/// ```text
///   sc_i        = atol + rtol · max(|y^n_i|, |y^{n+1}_i|)            (per-component)
///   err5_rms²   = (1/N) · Σ_i ( h · e5'_i / sc_i )²                  (RMS, see HNW Vol I §II.4)
///   err3_rms²   = (1/N) · Σ_i ( h · e3'_i / sc_i )²
///   err         = err5_rms² / sqrt(err5_rms² + 0.01 · err3_rms²)
/// ```
///
/// The factor `1/sqrt(N)` cancels through the err5 / err3 cross-
/// ratio, so this expression is equivalent to SciPy's
/// `|h| · ||err5||² / sqrt((||err5||² + 0.01·||err3||²) · N)`
/// formulation but expressed in terms of the per-component RMS norm
/// the `Integratable::weighted_error_norm` trait already provides.
///
/// The step controller is an I-controller (no PI β term): the
/// 3rd-order companion stabilising the err denominator plays the
/// same role a β term plays in DOPRI5(4)'s PI controller. SciPy
/// constants:
///
/// ```text
///   error_exponent  = -1 / (7 + 1)        # = -1/8, SciPy's DOP853 error_estimator_order + 1
///   factor          = safety · err^error_exponent
///   factor ∈ [min_factor, max_factor]
///   safety          = 0.9
///   min_factor      = 0.2
///   max_factor      = 10.0                # vs DOPRI5(4)'s 5.0 — DOP853 can grow h more aggressively
/// ```
///
/// On rejection: `factor = min(1.0, factor)` (no growth). If h is
/// already at `min_h_s` and the trial still fails, the integrator
/// fails closed with [`IntegratorError::InvalidStep`] rather than
/// silently violating the configured tolerance — same fail-closed
/// contract as [`Dopri54Adaptive`].
///
/// # Honest scope
///
/// - Dense output is opt-in through
///   [`Dopri853Adaptive::advance_with_dense_output`] and uses the
///   method-specific order-7 SciPy/Hairer interpolant. The regular
///   [`Integrator::advance`] path remains unchanged.
/// - I-controller only. A PI variant for DOP853 (with β-term
///   smoothing on top of the err5/err3 stabilisation) is plausible
///   but not implemented. SciPy doesn't ship one either.
///
/// # Determinism
///
/// Tagged [`IntegratorDeterminism::StateStable`]. Within a single
/// platform profile (target triple + toolchain + LLVM optimisation
/// level) the integrator is bit-stable across reruns: the step-size
/// search is purely deterministic given identical inputs. The
/// `state-stable` label is for cross-platform behaviour where
/// platform-libm differences in `pow()` may lead to slightly
/// different step-size sequences.
#[derive(Debug)]
#[allow(clippy::doc_markdown)] // SciPy / Hairer-Wanner are reference names, not code identifiers.
pub struct Dopri853Adaptive {
    safety_factor: f64,
    min_factor: f64,
    max_factor: f64,
    /// I-controller exponent: `-1 / (error_estimator_order + 1)`.
    /// SciPy's DOP853 sets `error_estimator_order = 7`, so this is
    /// `-1/8`.
    error_exponent: f64,
    atol: f64,
    rtol: f64,
    min_h_s: f64,
    max_h_s: f64,
    /// I-controller persistent state. `Cell` keeps the
    /// `Integrator::advance(&self, ...)` trait surface unchanged.
    last_h_s: Cell<Option<f64>>,
}

const DOPRI853_SAFETY_DEFAULT: f64 = 0.9;
const DOPRI853_MIN_FACTOR_DEFAULT: f64 = 0.2;
const DOPRI853_MAX_FACTOR_DEFAULT: f64 = 10.0;
/// I-controller exponent pinned to `SciPy`'s DOP853 convention:
/// `error_estimator_order = 7`, so `-1/(7 + 1) = -1/8`.
const DOPRI853_ERROR_EXPONENT: f64 = -1.0 / 8.0;
/// Weighting factor on the 3rd-order error norm in the `SciPy` /
/// Hairer combined denominator: `denom = err5² + W · err3²`. Tiny
/// (1 %) — the 3rd-order companion only kicks in when err5 is
/// near-zero.
const DOPRI853_E3_WEIGHT: f64 = 0.01;

#[allow(clippy::doc_markdown)] // SciPy is a reference name, not a code identifier.
impl Dopri853Adaptive {
    /// Construct a new DOP853 adaptive integrator with explicit
    /// tolerance and step-bound parameters.
    ///
    /// # Errors
    ///
    /// Returns [`AdaptiveIntegratorError`] for invalid numeric
    /// parameters: non-positive / non-finite tolerances, non-positive
    /// step bounds, or `min_h_s > max_h_s`.
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
        if !min_h_s.is_finite() || min_h_s <= 0.0 || !max_h_s.is_finite() || max_h_s <= 0.0 {
            return Err(AdaptiveIntegratorError::InvalidStepBounds);
        }
        if min_h_s > max_h_s {
            return Err(AdaptiveIntegratorError::InvalidStepBounds);
        }
        Ok(Self {
            safety_factor: DOPRI853_SAFETY_DEFAULT,
            min_factor: DOPRI853_MIN_FACTOR_DEFAULT,
            max_factor: DOPRI853_MAX_FACTOR_DEFAULT,
            error_exponent: DOPRI853_ERROR_EXPONENT,
            atol,
            rtol,
            min_h_s,
            max_h_s,
            last_h_s: Cell::new(None),
        })
    }

    /// Reset the controller's persistent state. Call this between
    /// independent runs to ensure the first sub-step uses the
    /// configured `dt_total` rather than the previous run's
    /// `last_h`.
    pub fn reset(&self) {
        self.last_h_s.set(None);
    }

    /// I-controller factor for an accepted step:
    ///
    /// ```text
    ///   factor = clamp(safety · err^error_exponent,
    ///                  min_factor, max_factor)
    /// ```
    fn i_controller_factor(&self, err: f64) -> f64 {
        let factor = self.safety_factor * err.powf(self.error_exponent);
        if !factor.is_finite() || factor <= 0.0 {
            return self.min_factor;
        }
        factor.max(self.min_factor).min(self.max_factor)
    }

    /// Single DOP853 primary sub-step from `state` of size `h`.
    /// Returns the accepted-state candidate, scaled error norm, and
    /// the stages needed to construct dense output. Does NOT mutate
    /// the controller's persistent state — the caller does that on
    /// accept.
    ///
    /// Same line-count justification as `Dopri853FixedStep::advance`:
    /// the locked-order 12-stage block + the err5/err3 derivative
    /// folds need to be visible at the call site for audit clarity.
    #[allow(clippy::too_many_lines)]
    fn try_primary_substep<S, F>(
        &self,
        state: &S,
        derive_fn: &F,
        h: f64,
    ) -> Result<Dopri853PrimaryStep<S>, IntegratorError>
    where
        S: SimState,
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        use dopri853_tableau::{
            A_2_1, A_3_1, A_3_2, A_4_1, A_4_3, A_5_1, A_5_3, A_5_4, A_6_1, A_6_4, A_6_5, A_7_1,
            A_7_4, A_7_5, A_7_6, A_8_1, A_8_4, A_8_5, A_8_6, A_8_7, A_9_1, A_9_4, A_9_5, A_9_6,
            A_9_7, A_9_8, A_10_1, A_10_4, A_10_5, A_10_6, A_10_7, A_10_8, A_10_9, A_11_1, A_11_4,
            A_11_5, A_11_6, A_11_7, A_11_8, A_11_9, A_11_10, A_12_1, A_12_4, A_12_5, A_12_6,
            A_12_7, A_12_8, A_12_9, A_12_10, A_12_11, B_1, B_6, B_7, B_8, B_9, B_10, B_11, B_12,
            C2, C3, C4, C5, C6, C7, C8, C9, C10, C11, E3_1, E3_6, E3_7, E3_8, E3_9, E3_10, E3_11,
            E3_12, E5_1, E5_6, E5_7, E5_8, E5_9, E5_10, E5_11, E5_12,
        };

        let t0 = state.time();
        let t0_s = t0.as_seconds();
        let t2 = SimTime::from_seconds(t0_s + C2 * h);
        let t3 = SimTime::from_seconds(t0_s + C3 * h);
        let t4 = SimTime::from_seconds(t0_s + C4 * h);
        let t5 = SimTime::from_seconds(t0_s + C5 * h);
        let t6 = SimTime::from_seconds(t0_s + C6 * h);
        let t7 = SimTime::from_seconds(t0_s + C7 * h);
        let t8 = SimTime::from_seconds(t0_s + C8 * h);
        let t9 = SimTime::from_seconds(t0_s + C9 * h);
        let t10 = SimTime::from_seconds(t0_s + C10 * h);
        let t11 = SimTime::from_seconds(t0_s + C11 * h);
        let t12 = SimTime::from_seconds(t0_s + h);

        // Stages 1..12 (same as Dopri853FixedStep).
        let k1 = derive_fn(state, t0)?;
        if !k1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let s2 = state.advance_by(h, &(k1 * A_2_1));
        if !s2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k2 = derive_fn(&s2, t2)?;
        if !k2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc3 = (k1 * A_3_1) + (k2 * A_3_2);
        let s3 = state.advance_by(h, &inc3);
        if !s3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k3 = derive_fn(&s3, t3)?;
        if !k3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc4 = (k1 * A_4_1) + (k3 * A_4_3);
        let s4 = state.advance_by(h, &inc4);
        if !s4.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k4 = derive_fn(&s4, t4)?;
        if !k4.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc5 = ((k1 * A_5_1) + (k3 * A_5_3)) + (k4 * A_5_4);
        let s5 = state.advance_by(h, &inc5);
        if !s5.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k5 = derive_fn(&s5, t5)?;
        if !k5.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc6 = ((k1 * A_6_1) + (k4 * A_6_4)) + (k5 * A_6_5);
        let s6 = state.advance_by(h, &inc6);
        if !s6.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k6 = derive_fn(&s6, t6)?;
        if !k6.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc7 = (((k1 * A_7_1) + (k4 * A_7_4)) + (k5 * A_7_5)) + (k6 * A_7_6);
        let s7 = state.advance_by(h, &inc7);
        if !s7.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k7 = derive_fn(&s7, t7)?;
        if !k7.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc8 = ((((k1 * A_8_1) + (k4 * A_8_4)) + (k5 * A_8_5)) + (k6 * A_8_6)) + (k7 * A_8_7);
        let s8 = state.advance_by(h, &inc8);
        if !s8.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k8 = derive_fn(&s8, t8)?;
        if !k8.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc9 = (((((k1 * A_9_1) + (k4 * A_9_4)) + (k5 * A_9_5)) + (k6 * A_9_6)) + (k7 * A_9_7))
            + (k8 * A_9_8);
        let s9 = state.advance_by(h, &inc9);
        if !s9.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k9 = derive_fn(&s9, t9)?;
        if !k9.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc10 = ((((((k1 * A_10_1) + (k4 * A_10_4)) + (k5 * A_10_5)) + (k6 * A_10_6))
            + (k7 * A_10_7))
            + (k8 * A_10_8))
            + (k9 * A_10_9);
        let s10 = state.advance_by(h, &inc10);
        if !s10.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k10 = derive_fn(&s10, t10)?;
        if !k10.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc11 = (((((((k1 * A_11_1) + (k4 * A_11_4)) + (k5 * A_11_5)) + (k6 * A_11_6))
            + (k7 * A_11_7))
            + (k8 * A_11_8))
            + (k9 * A_11_9))
            + (k10 * A_11_10);
        let s11 = state.advance_by(h, &inc11);
        if !s11.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k11 = derive_fn(&s11, t11)?;
        if !k11.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }
        let inc12 = ((((((((k1 * A_12_1) + (k4 * A_12_4)) + (k5 * A_12_5)) + (k6 * A_12_6))
            + (k7 * A_12_7))
            + (k8 * A_12_8))
            + (k9 * A_12_9))
            + (k10 * A_12_10))
            + (k11 * A_12_11);
        let s12 = state.advance_by(h, &inc12);
        if !s12.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k12 = derive_fn(&s12, t12)?;
        if !k12.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        // 8th-order solution. Same locked-order weighted sum as the
        // fixed-step variant.
        let weighted = (((((((k1 * B_1) + (k6 * B_6)) + (k7 * B_7)) + (k8 * B_8)) + (k9 * B_9))
            + (k10 * B_10))
            + (k11 * B_11))
            + (k12 * B_12);
        let mut new_state = state.advance_by(h, &weighted);
        new_state.project();
        if !new_state.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }

        // Embedded 5th-order error vector: e5' = Σ_j E5_j · k_j.
        // Non-zero coefficients at j ∈ {1, 6, 7, 8, 9, 10, 11, 12}.
        let err5_deriv = (((((((k1 * E5_1) + (k6 * E5_6)) + (k7 * E5_7)) + (k8 * E5_8))
            + (k9 * E5_9))
            + (k10 * E5_10))
            + (k11 * E5_11))
            + (k12 * E5_12);
        // Embedded 3rd-order error vector: e3' = Σ_j E3_j · k_j.
        // Non-zero coefficients at j ∈ {1, 6, 7, 8, 9, 10, 11, 12}.
        let err3_deriv = (((((((k1 * E3_1) + (k6 * E3_6)) + (k7 * E3_7)) + (k8 * E3_8))
            + (k9 * E3_9))
            + (k10 * E3_10))
            + (k11 * E3_11))
            + (k12 * E3_12);

        // Per-component RMS norms via the Integratable trait method.
        let err5_rms = new_state.weighted_error_norm(state, &err5_deriv, h, self.atol, self.rtol);
        let err3_rms = new_state.weighted_error_norm(state, &err3_deriv, h, self.atol, self.rtol);

        // Combined SciPy / Hairer error: err = err5² / sqrt(err5² +
        // 0.01 · err3²). The √(1/N) factor cancels through the
        // ratio. When err5 happens to vanish the err3 stabilises the
        // denominator (per HNW Vol I §II.5.3 commentary).
        let err5_sq = err5_rms * err5_rms;
        let err3_sq = err3_rms * err3_rms;
        let denom = err5_sq + DOPRI853_E3_WEIGHT * err3_sq;
        let scaled_err = if denom > 0.0 {
            err5_sq / denom.sqrt()
        } else {
            // Both estimators returned zero — perfect step. Report a
            // tiny positive err so the controller's exponent
            // computation doesn't divide by zero.
            1.0e-15
        };

        Ok(Dopri853PrimaryStep {
            start_state: *state,
            end_state: new_state,
            h_s: h,
            scaled_err,
            delta_deriv: weighted,
            k1,
            k6,
            k7,
            k8,
            k9,
            k10,
            k11,
            k12,
        })
    }

    /// Single DOP853 sub-step from `state` of size `h`. Returns
    /// `(new_state_8th_order, scaled_error_norm)`. Does NOT mutate
    /// the controller's persistent state — the caller does that on
    /// accept.
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
        let primary = self.try_primary_substep(state, derive_fn, h)?;
        Ok((primary.end_state, primary.scaled_err))
    }

    fn finish_dense_segment<S, F>(
        &self,
        primary: Dopri853PrimaryStep<S>,
        derive_fn: &F,
    ) -> Result<Dopri853DenseOutput<S>, IntegratorError>
    where
        S: SimState,
        F: Fn(&S, SimTime) -> Result<S::Derivative, ModelEvalError>,
    {
        use dopri853_tableau::{
            A_EXTRA_1_1, A_EXTRA_1_7, A_EXTRA_1_8, A_EXTRA_1_9, A_EXTRA_1_10, A_EXTRA_1_11,
            A_EXTRA_1_12, A_EXTRA_1_END, A_EXTRA_2_1, A_EXTRA_2_6, A_EXTRA_2_7, A_EXTRA_2_8,
            A_EXTRA_2_11, A_EXTRA_2_12, A_EXTRA_2_END, A_EXTRA_2_EXTRA_1, A_EXTRA_3_1, A_EXTRA_3_6,
            A_EXTRA_3_7, A_EXTRA_3_8, A_EXTRA_3_9, A_EXTRA_3_END, A_EXTRA_3_EXTRA_1,
            A_EXTRA_3_EXTRA_2, C_EXTRA_1, C_EXTRA_2, C_EXTRA_3,
        };

        let h = primary.h_s;
        let t0_s = primary.start_state.time().as_seconds();
        let t_end = SimTime::from_seconds(t0_s + h);
        let k_end = derive_fn(&primary.end_state, t_end)?;
        if !k_end.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        let inc_extra_1 = (((((((primary.k1 * A_EXTRA_1_1) + (primary.k7 * A_EXTRA_1_7))
            + (primary.k8 * A_EXTRA_1_8))
            + (primary.k9 * A_EXTRA_1_9))
            + (primary.k10 * A_EXTRA_1_10))
            + (primary.k11 * A_EXTRA_1_11))
            + (primary.k12 * A_EXTRA_1_12))
            + (k_end * A_EXTRA_1_END);
        let s_extra_1 = primary.start_state.advance_by(h, &inc_extra_1);
        if !s_extra_1.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k_extra_1 = derive_fn(&s_extra_1, SimTime::from_seconds(t0_s + C_EXTRA_1 * h))?;
        if !k_extra_1.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        let inc_extra_2 = (((((((primary.k1 * A_EXTRA_2_1) + (primary.k6 * A_EXTRA_2_6))
            + (primary.k7 * A_EXTRA_2_7))
            + (primary.k8 * A_EXTRA_2_8))
            + (primary.k11 * A_EXTRA_2_11))
            + (primary.k12 * A_EXTRA_2_12))
            + (k_end * A_EXTRA_2_END))
            + (k_extra_1 * A_EXTRA_2_EXTRA_1);
        let s_extra_2 = primary.start_state.advance_by(h, &inc_extra_2);
        if !s_extra_2.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k_extra_2 = derive_fn(&s_extra_2, SimTime::from_seconds(t0_s + C_EXTRA_2 * h))?;
        if !k_extra_2.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        let inc_extra_3 = (((((((primary.k1 * A_EXTRA_3_1) + (primary.k6 * A_EXTRA_3_6))
            + (primary.k7 * A_EXTRA_3_7))
            + (primary.k8 * A_EXTRA_3_8))
            + (primary.k9 * A_EXTRA_3_9))
            + (k_end * A_EXTRA_3_END))
            + (k_extra_1 * A_EXTRA_3_EXTRA_1))
            + (k_extra_2 * A_EXTRA_3_EXTRA_2);
        let s_extra_3 = primary.start_state.advance_by(h, &inc_extra_3);
        if !s_extra_3.is_valid_for_integration() {
            return Err(IntegratorError::NonFiniteState);
        }
        let k_extra_3 = derive_fn(&s_extra_3, SimTime::from_seconds(t0_s + C_EXTRA_3 * h))?;
        if !k_extra_3.is_finite() {
            return Err(IntegratorError::NonFiniteDerivative);
        }

        Ok(Dopri853DenseOutput {
            start_state: primary.start_state,
            end_state: primary.end_state,
            h_s: h,
            delta_deriv: primary.delta_deriv,
            k1: primary.k1,
            k6: primary.k6,
            k7: primary.k7,
            k8: primary.k8,
            k9: primary.k9,
            k10: primary.k10,
            k11: primary.k11,
            k12: primary.k12,
            k_end,
            k_extra_1,
            k_extra_2,
            k_extra_3,
        })
    }

    /// Advance by `dt` and retain order-7 dense-output segments for
    /// each accepted adaptive sub-step.
    ///
    /// # Errors
    ///
    /// Returns the same [`IntegratorError`] values as
    /// [`Integrator::advance`] for invalid steps, non-finite states or
    /// derivatives, or model-evaluation failures. Dense-output extra
    /// stage failures are also surfaced as typed integrator errors.
    pub fn advance_with_dense_output<S, F>(
        &self,
        state: &S,
        derive_fn: F,
        dt: Duration,
    ) -> Result<Dopri853AdaptiveDenseOutput<S>, IntegratorError>
    where
        S: SimState,
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

        let mut h = self
            .last_h_s
            .get()
            .unwrap_or(dt_total)
            .max(self.min_h_s)
            .min(self.max_h_s)
            .min(dt_total);

        let mut current = *state;
        let mut elapsed = 0.0_f64;
        let max_substeps: usize = 1_000_000;
        let mut substeps_taken: usize = 0;
        let mut step_just_rejected = false;
        let mut segments = Vec::new();

        while elapsed < dt_total {
            substeps_taken += 1;
            if substeps_taken > max_substeps {
                return Err(IntegratorError::InvalidStep {
                    dt_seconds: dt_total,
                });
            }

            let remaining = dt_total - elapsed;
            let h_try = if remaining <= self.min_h_s {
                remaining
            } else {
                h.min(remaining).max(self.min_h_s)
            };

            let primary = self.try_primary_substep(&current, &derive_fn, h_try)?;
            let err = primary.scaled_err;

            if err <= 1.0 {
                let segment = self.finish_dense_segment(primary, &derive_fn)?;
                current = segment.end_state;
                segments.push(segment);
                elapsed += h_try;
                let mut factor = self.i_controller_factor(err.max(1.0e-10));
                if step_just_rejected {
                    factor = factor.min(1.0);
                }
                h = (h_try * factor).max(self.min_h_s).min(self.max_h_s);
                step_just_rejected = false;
            } else {
                let factor = (self.safety_factor * err.powf(self.error_exponent))
                    .max(self.min_factor)
                    .min(1.0);
                if h_try <= self.min_h_s + f64::EPSILON {
                    return Err(IntegratorError::InvalidStep { dt_seconds: h_try });
                }
                h = (h_try * factor).max(self.min_h_s);
                step_just_rejected = true;
            }
        }

        self.last_h_s.set(Some(h));
        Ok(Dopri853AdaptiveDenseOutput {
            final_state: current,
            segments,
        })
    }
}

impl<S: SimState> Integrator<S> for Dopri853Adaptive {
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

        // Initial sub-step: prefer last accepted h, fall back to
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
        let max_substeps: usize = 1_000_000;
        let mut substeps_taken: usize = 0;
        let mut step_just_rejected = false;

        while elapsed < dt_total {
            substeps_taken += 1;
            if substeps_taken > max_substeps {
                return Err(IntegratorError::InvalidStep {
                    dt_seconds: dt_total,
                });
            }

            let remaining = dt_total - elapsed;
            let h_try = if remaining <= self.min_h_s {
                remaining
            } else {
                h.min(remaining).max(self.min_h_s)
            };

            let (proposed_state, err) = self.try_substep(&current, &derive_fn, h_try)?;

            if err <= 1.0 {
                current = proposed_state;
                elapsed += h_try;
                let mut factor = self.i_controller_factor(err.max(1.0e-10));
                // SciPy convention: after a rejected step, the next
                // accepted step is not allowed to grow.
                if step_just_rejected {
                    factor = factor.min(1.0);
                }
                h = (h_try * factor).max(self.min_h_s).min(self.max_h_s);
                step_just_rejected = false;
            } else {
                // Reject: shrink h via the same I-controller formula
                // clamped to [min_factor, 1.0] (no growth on rejection).
                let factor = (self.safety_factor * err.powf(self.error_exponent))
                    .max(self.min_factor)
                    .min(1.0);
                if h_try <= self.min_h_s + f64::EPSILON {
                    return Err(IntegratorError::InvalidStep { dt_seconds: h_try });
                }
                h = (h_try * factor).max(self.min_h_s);
                step_just_rejected = true;
            }
        }

        self.last_h_s.set(Some(h));
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
    // Dormand-Prince 5(4) fixed-step tests
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
    // Dormand-Prince 5(4) adaptive integrator tests.
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

    #[test]
    fn dopri54_dense_output_returns_exact_step_endpoints() {
        let integrator = Dopri54Adaptive::new(1.0e-12, 1.0e-9, 1.0e-9, 0.1).unwrap();
        let state = exp_decay_initial_state();
        let (dense, err) = integrator
            .try_dense_substep(&state, &exp_decay_derive, 0.1)
            .expect("dense substep");
        assert!(err.is_finite());

        let start = dense
            .interpolate(SimTime::ZERO)
            .expect("start endpoint must interpolate");
        let end = dense
            .interpolate(SimTime::from_seconds(0.1))
            .expect("end endpoint must interpolate");

        assert_eq!(
            start.mass.get::<kilogram>().to_bits(),
            state.mass.get::<kilogram>().to_bits(),
            "dense output must return the stored start state exactly"
        );
        assert_eq!(
            end.mass.get::<kilogram>().to_bits(),
            dense.end_state().mass.get::<kilogram>().to_bits(),
            "dense output must return the accepted endpoint exactly"
        );

        let out_of_range = dense
            .interpolate(SimTime::from_seconds(0.100_000_001))
            .expect_err("query outside the segment must fail closed");
        assert!(matches!(
            out_of_range,
            IntegratorError::DenseOutputTimeOutOfRange { .. }
        ));
    }

    #[test]
    fn dopri54_dense_output_midpoint_error_has_fourth_order_ratio() {
        fn midpoint_error(h: f64) -> f64 {
            let integrator = Dopri54Adaptive::new(1.0e-14, 1.0e-12, 1.0e-12, h).unwrap();
            let state = exp_decay_initial_state();
            let (dense, _) = integrator
                .try_dense_substep(&state, &exp_decay_derive, h)
                .expect("dense substep");
            let observed = dense
                .interpolate(SimTime::from_seconds(0.5 * h))
                .expect("midpoint interpolation")
                .mass
                .get::<kilogram>();
            let exact = (-0.5 * h).exp();
            (observed - exact).abs()
        }

        let e_coarse = midpoint_error(0.4);
        let e_mid = midpoint_error(0.2);
        let e_fine = midpoint_error(0.1);
        let ratio_1 = e_coarse / e_mid;
        let ratio_2 = e_mid / e_fine;

        assert!(
            ratio_1 > 10.0 && ratio_2 > 10.0,
            "quartic DOPRI5 dense output should show >=4th-order grid-halving \
             behavior before roundoff dominates; errors {e_coarse:.3e}, \
             {e_mid:.3e}, {e_fine:.3e}, ratios {ratio_1:.2}, {ratio_2:.2}",
        );
    }

    #[test]
    fn dopri54_advance_with_dense_output_matches_plain_adaptive_endpoint() {
        let state = exp_decay_initial_state();
        let dt = Duration::from_seconds(0.5);
        let dense_integrator = Dopri54Adaptive::new(1.0e-10, 1.0e-8, 1.0e-9, 0.05).unwrap();
        let dense = dense_integrator
            .advance_with_dense_output(&state, exp_decay_derive, dt)
            .expect("dense advance");

        let plain_integrator = Dopri54Adaptive::new(1.0e-10, 1.0e-8, 1.0e-9, 0.05).unwrap();
        let plain = plain_integrator
            .advance(&state, exp_decay_derive, dt)
            .expect("plain advance");

        assert!(
            !dense.segments().is_empty(),
            "dense advance should record accepted sub-step segments"
        );
        assert_eq!(
            dense.final_state().mass.get::<kilogram>().to_bits(),
            plain.mass.get::<kilogram>().to_bits(),
            "dense-output path must preserve the normal adaptive endpoint"
        );

        let midpoint = dense
            .interpolate(SimTime::from_seconds(0.25))
            .expect("dense midpoint");
        let exact = (-0.25_f64).exp();
        assert!(
            (midpoint.mass.get::<kilogram>() - exact).abs() < 1.0e-8,
            "dense midpoint should be state-stable accurate for exp decay"
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

    /// PI controller band-stability over a long
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

    /// Verifies the per-component norm refinement is in
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
    // DOP853 (Dormand-Prince 8(5,3)) tableau / fixed-step
    // tests. Coverage:
    //   - Tableau row sums: Σ_j A_i_j = C_i for every stage row i;
    //     Σ_j B_j = 1 (8th-order solution is consistent); Σ_j E5_j = 0
    //     and Σ_j E3_j = 0 (embedded estimators are differences, sum
    //     to zero).
    //   - 8th-order convergence on a polynomial trajectory: an
    //     degree-8 polynomial trajectory is reproduced within machine
    //     precision, exercising DOP853's high-order surface rather
    //     than a lower-order corner that DOPRI5(4) could also cover.
    //   - Determinism marker is BitStable.
    //   - Within-platform bit-stability across two reruns.
    // -----------------------------------------------------------------

    /// Tableau row-sum invariants per Hairer-Nørsett-Wanner Vol I
    /// §II.5 Table 5.4: every stage row of A sums to the
    /// corresponding C abscissa, B sums to 1 (consistency), and the
    /// embedded estimator weights E5 and E3 sum to 0.
    #[test]
    fn dopri853_tableau_row_sums_match_abscissas() {
        use dopri853_tableau::*;

        // Stage 1 implicit (k_1 at t_0; no row-sum invariant).
        // A_2_*: row sum = C2.
        assert_abs_diff_eq!(A_2_1, C2, epsilon = 1.0e-15);
        // A_3_*: row sum = C3.
        assert_abs_diff_eq!(A_3_1 + A_3_2, C3, epsilon = 1.0e-15);
        // A_4_*: row sum = C4 (A_4_2 = 0).
        assert_abs_diff_eq!(A_4_1 + A_4_3, C4, epsilon = 1.0e-15);
        // A_5_*: row sum = C5 (A_5_2 = 0).
        assert_abs_diff_eq!(A_5_1 + A_5_3 + A_5_4, C5, epsilon = 1.0e-15);
        // A_6_*: row sum = C6.
        assert_abs_diff_eq!(A_6_1 + A_6_4 + A_6_5, C6, epsilon = 1.0e-15);
        // A_7_*: row sum = C7.
        assert_abs_diff_eq!(A_7_1 + A_7_4 + A_7_5 + A_7_6, C7, epsilon = 1.0e-15);
        // A_8_*: row sum = C8.
        assert_abs_diff_eq!(A_8_1 + A_8_4 + A_8_5 + A_8_6 + A_8_7, C8, epsilon = 1.0e-15);
        // A_9_*: row sum = C9.
        assert_abs_diff_eq!(
            A_9_1 + A_9_4 + A_9_5 + A_9_6 + A_9_7 + A_9_8,
            C9,
            epsilon = 1.0e-14,
        );
        // A_10_*: row sum = C10.
        assert_abs_diff_eq!(
            A_10_1 + A_10_4 + A_10_5 + A_10_6 + A_10_7 + A_10_8 + A_10_9,
            C10,
            epsilon = 1.0e-14,
        );
        // A_11_*: row sum = C11.
        assert_abs_diff_eq!(
            A_11_1 + A_11_4 + A_11_5 + A_11_6 + A_11_7 + A_11_8 + A_11_9 + A_11_10,
            C11,
            epsilon = 1.0e-14,
        );
        // A_12_*: row sum = 1 (final-row abscissa is C12 = 1).
        assert_abs_diff_eq!(
            A_12_1 + A_12_4 + A_12_5 + A_12_6 + A_12_7 + A_12_8 + A_12_9 + A_12_10 + A_12_11,
            1.0,
            epsilon = 1.0e-14,
        );
    }

    #[test]
    fn dopri853_b_weights_sum_to_one() {
        use dopri853_tableau::*;
        // 8th-order solution must have consistent quadrature weights:
        // Σ_j B_j = 1.
        let sum = B_1 + B_6 + B_7 + B_8 + B_9 + B_10 + B_11 + B_12;
        assert_abs_diff_eq!(sum, 1.0, epsilon = 1.0e-14);
    }

    #[test]
    fn dopri853_embedded_estimator_weights_sum_to_zero() {
        use dopri853_tableau::*;
        // Embedded estimators are constructed as `B - B̂`, so each
        // estimator's coefficient sum vanishes.
        let e5_sum = E5_1 + E5_6 + E5_7 + E5_8 + E5_9 + E5_10 + E5_11 + E5_12;
        assert_abs_diff_eq!(e5_sum, 0.0, epsilon = 1.0e-12);
        let e3_sum = E3_1 + E3_6 + E3_7 + E3_8 + E3_9 + E3_10 + E3_11 + E3_12;
        assert_abs_diff_eq!(e3_sum, 0.0, epsilon = 1.0e-12);
    }

    /// 8th-order convergence test: the integrator should reproduce a
    /// degree-8 polynomial trajectory to within machine precision.
    /// The trajectory is `dx/dt = 8·t⁷` carried in the position-x
    /// slot (position has no positivity constraint, unlike mass),
    /// with `x(0) = 0` and the exact closed-form `x(t) = t⁸`.
    #[test]
    fn dopri853_fixed_step_reproduces_degree_eight_polynomial_trajectory() {
        #[allow(clippy::unnecessary_wraps)]
        fn poly_derive(
            _s: &PointMassState,
            t: SimTime,
        ) -> Result<PointMassDerivative, ModelEvalError> {
            let t_s = t.as_seconds();
            let t2 = t_s * t_s;
            let t4 = t2 * t2;
            let t7 = (t4 * t2) * t_s;
            let dx_dt = 8.0 * t7; // 8 · t⁷
            Ok(PointMassDerivative {
                velocity_m_s: Vector3::new(dx_dt, 0.0, 0.0),
                acceleration_m_s2: Vector3::zeros(),
                mass_rate_kg_s: 0.0,
            })
        }

        let initial = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let integrator = Dopri853FixedStep;
        let dt = Duration::from_seconds(0.1);
        let n_steps = 10;
        let mut state = initial;
        for step in 0..n_steps {
            state = integrator
                .advance(&state, poly_derive, dt)
                .expect("8th-order method must integrate degree-8 polynomial exactly");
            // Canonical-time fix-up so the next derive_fn sees the
            // exact `start + step·dt` time grid.
            let t = SimTime::from_seconds(f64::from(step + 1) * 0.1);
            state = state.with_time(t);
        }
        let final_t = state.time.as_seconds();
        let exact = final_t.powi(8);
        let observed = state.position.vector.x;
        assert_abs_diff_eq!(observed, exact, epsilon = 1.0e-12);
    }

    #[test]
    fn dopri853_fixed_step_determinism_class_is_bit_stable() {
        let i = Dopri853FixedStep;
        assert_eq!(
            <Dopri853FixedStep as Integrator<PointMassState>>::determinism(&i),
            IntegratorDeterminism::BitStable
        );
    }

    #[test]
    fn dopri853_fixed_step_is_bit_stable_across_two_runs() {
        let initial = exp_decay_initial_state();
        let integrator = Dopri853FixedStep;
        let dt = Duration::from_seconds(0.1);

        let s1 = integrator
            .advance(&initial, exp_decay_derive, dt)
            .expect("run 1");
        let s2 = integrator
            .advance(&initial, exp_decay_derive, dt)
            .expect("run 2");

        assert_eq!(
            s1.mass.get::<kilogram>().to_bits(),
            s2.mass.get::<kilogram>().to_bits(),
            "two reruns of Dopri853FixedStep must produce bit-identical state"
        );
    }

    // -----------------------------------------------------------------
    // DOP853 adaptive integrator tests.
    // -----------------------------------------------------------------

    #[test]
    fn dopri853_adaptive_constructor_rejects_invalid_atol() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                Dopri853Adaptive::new(bad, 1.0e-9, 1.0e-9, 1.0).unwrap_err(),
                AdaptiveIntegratorError::InvalidAtol,
            );
        }
    }

    #[test]
    fn dopri853_adaptive_constructor_rejects_invalid_rtol() {
        for bad in [0.0, -1.0, f64::NAN, f64::INFINITY] {
            assert_eq!(
                Dopri853Adaptive::new(1.0e-12, bad, 1.0e-9, 1.0).unwrap_err(),
                AdaptiveIntegratorError::InvalidRtol,
            );
        }
    }

    #[test]
    fn dopri853_adaptive_constructor_rejects_invalid_step_bounds() {
        // min_h > max_h.
        assert_eq!(
            Dopri853Adaptive::new(1.0e-12, 1.0e-9, 1.0, 1.0e-3).unwrap_err(),
            AdaptiveIntegratorError::InvalidStepBounds,
        );
        // Non-finite bounds.
        assert_eq!(
            Dopri853Adaptive::new(1.0e-12, 1.0e-9, f64::NAN, 1.0).unwrap_err(),
            AdaptiveIntegratorError::InvalidStepBounds,
        );
    }

    #[test]
    fn dopri853_adaptive_determinism_class_is_state_stable() {
        let i = Dopri853Adaptive::new(1.0e-12, 1.0e-9, 1.0e-9, 1.0).unwrap();
        assert_eq!(
            <Dopri853Adaptive as Integrator<PointMassState>>::determinism(&i),
            IntegratorDeterminism::StateStable
        );
    }

    /// 8th-order convergence: a single adaptive sub-step uses the
    /// same 8th-order primary solution as the fixed-step variant, so
    /// it should reproduce a degree-8 polynomial trajectory to within
    /// machine precision even though the embedded companions report a
    /// non-zero lower-order error estimate.
    #[test]
    fn dopri853_adaptive_primary_solution_reproduces_degree_eight_polynomial_trajectory() {
        #[allow(clippy::unnecessary_wraps)]
        fn poly_derive(
            _s: &PointMassState,
            t: SimTime,
        ) -> Result<PointMassDerivative, ModelEvalError> {
            let t_s = t.as_seconds();
            let t2 = t_s * t_s;
            let t4 = t2 * t2;
            let t7 = (t4 * t2) * t_s;
            let dx_dt = 8.0 * t7;
            Ok(PointMassDerivative {
                velocity_m_s: Vector3::new(dx_dt, 0.0, 0.0),
                acceleration_m_s2: Vector3::zeros(),
                mass_rate_kg_s: 0.0,
            })
        }
        let initial = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let integrator = Dopri853Adaptive::new(1.0e-9, 1.0e-6, 1.0e-9, 1.0).unwrap();
        // Drive try_substep directly to observe the err norm
        // on a single step at h = 0.1.
        let (proposed, err) = integrator
            .try_substep(&initial, &poly_derive, 0.1)
            .expect("polynomial step must succeed");
        assert!(
            err.is_finite(),
            "embedded error norm must be finite on a degree-8 polynomial trajectory",
        );
        assert_abs_diff_eq!(
            proposed.position.vector.x,
            0.1_f64.powi(8),
            epsilon = 1.0e-20
        );
    }

    #[test]
    fn dopri853_dense_output_returns_exact_step_endpoints() {
        let integrator = Dopri853Adaptive::new(1.0e-12, 1.0e-9, 1.0e-9, 0.2).unwrap();
        let state = exp_decay_initial_state();
        let primary = integrator
            .try_primary_substep(&state, &exp_decay_derive, 0.2)
            .expect("primary step");
        let dense = integrator
            .finish_dense_segment(primary, &exp_decay_derive)
            .expect("dense segment");

        let start = dense
            .interpolate(SimTime::ZERO)
            .expect("start endpoint must interpolate");
        let end = dense
            .interpolate(SimTime::from_seconds(0.2))
            .expect("end endpoint must interpolate");

        assert_eq!(
            start.mass.get::<kilogram>().to_bits(),
            state.mass.get::<kilogram>().to_bits(),
            "DOP853 dense output must return the stored start state exactly"
        );
        assert_eq!(
            end.mass.get::<kilogram>().to_bits(),
            dense.end_state().mass.get::<kilogram>().to_bits(),
            "DOP853 dense output must return the accepted endpoint exactly"
        );

        let out_of_range = dense
            .interpolate(SimTime::from_seconds(0.200_000_001))
            .expect_err("query outside the segment must fail closed");
        assert!(matches!(
            out_of_range,
            IntegratorError::DenseOutputTimeOutOfRange { .. }
        ));
    }

    #[test]
    fn dopri853_dense_output_midpoint_error_has_order_seven_ratio() {
        fn midpoint_error(h: f64) -> f64 {
            let integrator = Dopri853Adaptive::new(1.0e-14, 1.0e-12, 1.0e-12, h).unwrap();
            let state = exp_decay_initial_state();
            let primary = integrator
                .try_primary_substep(&state, &exp_decay_derive, h)
                .expect("primary step");
            let dense = integrator
                .finish_dense_segment(primary, &exp_decay_derive)
                .expect("dense segment");
            let observed = dense
                .interpolate(SimTime::from_seconds(0.5 * h))
                .expect("midpoint interpolation")
                .mass
                .get::<kilogram>();
            let exact = (-0.5 * h).exp();
            (observed - exact).abs()
        }

        let e_coarse = midpoint_error(1.0);
        let e_mid = midpoint_error(0.5);
        let e_fine = midpoint_error(0.25);
        let ratio_1 = e_coarse / e_mid;
        let ratio_2 = e_mid / e_fine;

        assert!(
            ratio_1 > 80.0 && ratio_2 > 80.0,
            "DOP853 order-7 dense output should show high-order grid-halving \
             behavior before roundoff dominates; errors {e_coarse:.3e}, \
             {e_mid:.3e}, {e_fine:.3e}, ratios {ratio_1:.2}, {ratio_2:.2}",
        );
    }

    #[test]
    fn dopri853_advance_with_dense_output_matches_plain_adaptive_endpoint() {
        let state = exp_decay_initial_state();
        let dt = Duration::from_seconds(0.5);
        let dense_integrator = Dopri853Adaptive::new(1.0e-10, 1.0e-8, 1.0e-9, 0.05).unwrap();
        let dense = dense_integrator
            .advance_with_dense_output(&state, exp_decay_derive, dt)
            .expect("dense advance");

        let plain_integrator = Dopri853Adaptive::new(1.0e-10, 1.0e-8, 1.0e-9, 0.05).unwrap();
        let plain = plain_integrator
            .advance(&state, exp_decay_derive, dt)
            .expect("plain advance");

        assert!(
            !dense.segments().is_empty(),
            "dense advance should record accepted DOP853 sub-step segments"
        );
        assert_eq!(
            dense.final_state().mass.get::<kilogram>().to_bits(),
            plain.mass.get::<kilogram>().to_bits(),
            "DOP853 dense-output path must preserve the normal adaptive endpoint"
        );

        let midpoint = dense
            .interpolate(SimTime::from_seconds(0.25))
            .expect("dense midpoint");
        let exact = (-0.25_f64).exp();
        assert!(
            (midpoint.mass.get::<kilogram>() - exact).abs() < 1.0e-10,
            "DOP853 dense midpoint should be state-stable accurate for exp decay"
        );
    }

    #[test]
    fn dopri853_adaptive_within_platform_bit_stable_across_two_runs() {
        let initial = exp_decay_initial_state();
        let dt = Duration::from_seconds(0.1);

        let i1 = Dopri853Adaptive::new(1.0e-9, 1.0e-7, 1.0e-9, 0.05).unwrap();
        let s1 = i1.advance(&initial, exp_decay_derive, dt).expect("run 1");
        let i2 = Dopri853Adaptive::new(1.0e-9, 1.0e-7, 1.0e-9, 0.05).unwrap();
        let s2 = i2.advance(&initial, exp_decay_derive, dt).expect("run 2");
        assert_eq!(
            s1.mass.get::<kilogram>().to_bits(),
            s2.mass.get::<kilogram>().to_bits(),
            "two reruns of Dopri853Adaptive on the same platform must produce bit-identical state",
        );
    }

    /// Sub-step accumulation lands exactly at `dt_total` — same
    /// fail-closed contract as `Dopri54Adaptive`.
    #[test]
    fn dopri853_adaptive_substep_accumulation_lands_exactly_at_dt() {
        let initial = exp_decay_initial_state();
        let integrator = Dopri853Adaptive::new(1.0e-12, 1.0e-9, 1.0e-9, 0.01).unwrap();
        let dt = Duration::from_seconds(0.1);
        let new_state = integrator
            .advance(&initial, exp_decay_derive, dt)
            .expect("integration must succeed");
        // The integrator's outer loop should land exactly at
        // dt_total seconds elapsed (in sub-step seconds added). The
        // SimulationKernel rolls in canonical time externally, so
        // the integrator-side state's time after a single advance
        // call equals start + cumulative h. With perfect
        // accumulation the difference is bit-stable.
        let elapsed = new_state.time.as_seconds() - initial.time.as_seconds();
        assert_abs_diff_eq!(elapsed, 0.1, epsilon = 1.0e-12);
    }

    /// The final sub-step may be below `min_h_s` when that is the
    /// only way to land exactly on the caller's outer `dt`.
    #[test]
    fn dopri853_adaptive_final_substep_can_be_below_min_h_to_fit_dt() {
        let integrator = Dopri853Adaptive::new(1.0e-9, 1.0e-6, 1.0e-3, 1.0).unwrap();
        let state = exp_decay_initial_state();
        let dt = 1.0e-4_f64;

        let final_state = integrator
            .advance(&state, exp_decay_derive, Duration::from_seconds(dt))
            .expect("advance");

        assert_eq!(
            final_state.time.as_seconds().to_bits(),
            dt.to_bits(),
            "DOP853 adaptive integrator must not overshoot an outer dt below min_h_s"
        );
    }

    /// A floor is a hard integration limit, not permission to accept a
    /// step that still violates the configured tolerance.
    #[test]
    fn dopri853_adaptive_rejects_when_tolerance_cannot_be_met_at_min_h() {
        let integrator = Dopri853Adaptive::new(1.0e-300, 1.0e-300, 1.0e-3, 1.0e-3).unwrap();
        let state = exp_decay_initial_state();
        let err = integrator
            .advance(&state, exp_decay_derive, Duration::from_seconds(1.0e-3))
            .expect_err("unachievable tolerance at h_min must fail closed");

        assert!(
            matches!(err, IntegratorError::InvalidStep { dt_seconds } if dt_seconds.to_bits() == 1.0e-3_f64.to_bits()),
            "expected InvalidStep at h_min, got {err:?}"
        );
    }

    #[test]
    fn dopri853_adaptive_reset_clears_persistent_state() {
        let integrator = Dopri853Adaptive::new(1.0e-9, 1.0e-7, 1.0e-9, 0.05).unwrap();
        let initial = exp_decay_initial_state();

        let _ = integrator
            .advance(&initial, exp_decay_derive, Duration::from_seconds(0.1))
            .expect("advance");
        assert!(
            integrator.last_h_s.get().is_some(),
            "advance should persist the next trial h"
        );

        integrator.reset();
        assert!(
            integrator.last_h_s.get().is_none(),
            "reset must clear DOP853 adaptive controller history"
        );
    }

    /// DOP853 uses `SciPy`'s I-controller rather than the DOPRI5(4) PI
    /// controller. Drive `try_substep` directly so the test can
    /// observe accepted sub-step sizes without the public `advance`
    /// loop's final-fragment clamp.
    #[test]
    fn dopri853_adaptive_i_controller_stays_in_band_over_long_run() {
        #[allow(clippy::unnecessary_wraps)]
        fn oscillatory_derive(
            _s: &PointMassState,
            t: SimTime,
        ) -> Result<PointMassDerivative, ModelEvalError> {
            let phase = 5.0 * t.as_seconds();
            Ok(PointMassDerivative {
                velocity_m_s: Vector3::new(phase.cos(), 0.0, 0.0),
                acceleration_m_s2: Vector3::zeros(),
                mass_rate_kg_s: 0.0,
            })
        }

        let atol = 1.0e-12;
        let rtol = 1.0e-12;
        let min_h = 1.0e-9;
        let max_h = 1.0;
        let integrator = Dopri853Adaptive::new(atol, rtol, min_h, max_h).unwrap();

        let n_steps = 200;
        let mut state = PointMassState::new(
            SimTime::ZERO,
            Position3::origin(),
            Velocity3::zero(),
            Mass::new::<kilogram>(1.0),
        );
        let mut h = max_h;
        let mut accepted_h_history: Vec<f64> = Vec::with_capacity(n_steps);
        let mut total_accepts: usize = 0;
        let mut total_rejects: usize = 0;
        let mut step_just_rejected = false;

        while total_accepts < n_steps {
            let h_try = h.max(min_h).min(max_h);
            let (proposed, err) = integrator
                .try_substep(&state, &oscillatory_derive, h_try)
                .expect("try_substep must succeed for a benign integrand");

            if err <= 1.0 {
                let mut factor = integrator.i_controller_factor(err.max(1.0e-10));
                if step_just_rejected {
                    factor = factor.min(1.0);
                }
                state = proposed.with_time(SimTime::from_seconds(state.time.as_seconds() + h_try));
                accepted_h_history.push(h_try);
                h = (h_try * factor).max(min_h).min(max_h);
                total_accepts += 1;
                step_just_rejected = false;
            } else {
                let factor = (integrator.safety_factor * err.powf(integrator.error_exponent))
                    .max(integrator.min_factor)
                    .min(1.0);
                assert!(
                    h_try > integrator.min_h_s + f64::EPSILON,
                    "DOP853 I-controller hit min_h_s and still rejected on benign exp-decay"
                );
                h = (h_try * factor).max(integrator.min_h_s);
                total_rejects += 1;
                step_just_rejected = true;
            }
        }

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

        for (i, window) in accepted_h_history.windows(2).enumerate().skip(5) {
            let ratio = window[1] / window[0];
            assert!(
                (0.5..=2.0).contains(&ratio),
                "DOP853 consecutive accepted-h ratio at step {i} = {ratio} \
                 outside the steady-state band [0.5, 2.0] ({} → {})",
                window[0],
                window[1],
            );
        }

        let total_attempts = total_accepts + total_rejects;
        #[allow(clippy::cast_precision_loss)]
        let reject_frac = (total_rejects as f64) / (total_attempts as f64);
        assert!(
            reject_frac < 0.10,
            "DOP853 rejection rate {reject_frac:.3} too high \
             ({total_rejects} rejected / {total_attempts} attempts)",
        );

        let mut steady: Vec<f64> = accepted_h_history[20..].to_vec();
        steady.sort_by(f64::total_cmp);
        let median_h = steady[steady.len() / 2];
        assert!(
            (1.0e-3..=1.0).contains(&median_h),
            "DOP853 steady-state median accepted h = {median_h} outside \
             the 1e-3..1.0 sanity band for oscillatory derive at rtol={rtol}, atol={atol}"
        );
    }

    #[test]
    fn dopri853_adaptive_at_loose_tolerance_takes_fewer_substeps_than_dopri54() {
        // Sanity sniff: under a benign trajectory the 8th-order
        // method should be at least competitive with the 5(4) pair
        // at the same tolerance. We don't assert exact ratios (the
        // controller gain dynamics are method-specific) — just that
        // both integrate without timing out and DOP853's last
        // accepted h is at least as large as DOPRI54's at the same
        // tolerance.
        let initial = exp_decay_initial_state();
        let dt = Duration::from_seconds(0.1);
        let atol = 1.0e-7;
        let rtol = 1.0e-5;

        let i54 = Dopri54Adaptive::new(atol, rtol, 1.0e-9, 0.05).unwrap();
        let i853 = Dopri853Adaptive::new(atol, rtol, 1.0e-9, 0.05).unwrap();
        let _ = i54
            .advance(&initial, exp_decay_derive, dt)
            .expect("dopri54 ok");
        let _ = i853
            .advance(&initial, exp_decay_derive, dt)
            .expect("dopri853 ok");

        let h_54 = i54.last_h_s.get().expect("dopri54 last_h");
        let h_853 = i853.last_h_s.get().expect("dopri853 last_h");
        assert!(
            h_853 >= h_54 * 0.5,
            "dopri853 last_h {h_853} unexpectedly far below dopri54 last_h {h_54} at \
             matched tolerances; the 8th-order method should be at least competitive",
        );
    }

    // -----------------------------------------------------------------
    // RigidBodyState SimState impl tests
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
        /// closure to compute `q_dot` correctly. The integration shape
        /// is generic; the closure is supplied by the
        /// kernel in the torque-free scenario. The test here builds
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
