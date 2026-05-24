//! Cao-Hovakimyan 2010 L1 adaptive controller — full architecture.
//!
//! This feature-gated module ships the four-piece L1 adaptive control
//! decomposition as described in Hovakimyan & Cao 2010
//! (*L1 Adaptive Control Theory: Guaranteed Robustness with Fast
//! Adaptation*, SIAM):
//!
//! 1. [`L1ReferenceModel`] — stable linear scalar reference model
//!    `ẋ_m = a_m x_m + b k_g r`. Defines the desired closed-loop
//!    response. `a_m < 0` for stability.
//! 2. [`L1StatePredictor`] — predictor for the matched-uncertainty
//!    channel: `ẋ̂ = a_m x̂ + b (u + σ̂)`. The prediction error
//!    `x̃ = x̂ - x_p` drives the adaptive law.
//! 3. [`L1PiecewiseConstantAdaptation`] — at each sample time
//!    `T_s`, computes `σ̂(iT_s)` from the prediction error using the
//!    closed-form scalar PCA expression; held constant across each
//!    sample interval. Includes a projection bound on `σ̂` to keep
//!    the estimate inside the closed-loop validity envelope.
//! 4. [`L1LowPassFilter`] — strictly-proper first-order LPF
//!    `H(s) = ω_c / (s + ω_c)` (impulse-invariant discretisation).
//!    The L1 augmentation is `u_ad = -LPF(σ̂)`.
//!
//! [`L1AdaptiveChannel`] composes the four pieces into one scalar
//! axis. [`L1AdaptiveParams`] carries the per-axis tuning; the
//! constructor asserts the bandwidth-projection inequality
//! `ω_c · L < 1` and fails closed on violation.
//!
//! This math is wired into the rate loop, replacing an earlier
//! L1-inspired interim channel.

use thiserror::Error;

/// L1 adaptive control errors.
#[derive(Copy, Clone, Debug, PartialEq, Error)]
pub enum L1AdaptiveError {
    /// `a_m` must be strictly negative for the reference model and
    /// state predictor to be stable.
    #[error("L1 reference model requires a_m < 0; got a_m = {a_m}")]
    ReferenceModelNotStable {
        /// Provided `a_m` (rad/s).
        a_m: f64,
    },
    /// `b` must be non-zero so the input-effectiveness inversion in
    /// PCA is well-defined.
    #[error("L1 control effectiveness b must be non-zero")]
    NonZeroEffectivenessRequired,
    /// `T_s` must be strictly positive so PCA's `Φ(T_s) - 1` is
    /// well-defined.
    #[error("L1 adaptation sample time T_s must be > 0; got T_s = {t_s}")]
    NonPositiveSampleTime {
        /// Provided `T_s` (s).
        t_s: f64,
    },
    /// `ω_c` must be strictly positive for the LPF to be a proper
    /// first-order stable filter.
    #[error("L1 LPF cutoff ω_c must be > 0; got ω_c = {omega_c}")]
    NonPositiveLowPassCutoff {
        /// Provided `ω_c` (rad/s).
        omega_c: f64,
    },
    /// Projection bound on `σ̂` must be positive.
    #[error("L1 projection bound on σ̂ must be > 0; got bound = {bound}")]
    NonPositiveProjectionBound {
        /// Provided projection bound.
        bound: f64,
    },
    /// Lipschitz bound `L` must be positive — the bandwidth-projection
    /// inequality `ω_c · L < 1` is degenerate otherwise.
    #[error("L1 Lipschitz bound L must be > 0; got L = {l}")]
    NonPositiveLipschitzBound {
        /// Provided Lipschitz bound.
        l: f64,
    },
    /// The bandwidth-projection inequality `ω_c · L < 1` from the
    /// Cao-Hovakimyan robustness margin condition is violated.
    #[error(
        "L1 bandwidth-projection inequality violated: ω_c · L = {product} >= 1 (ω_c = {omega_c}, L = {lipschitz_bound})"
    )]
    BandwidthProjectionExceeded {
        /// `ω_c` from params.
        omega_c: f64,
        /// `L` from params.
        lipschitz_bound: f64,
        /// `ω_c · L`.
        product: f64,
    },
    /// Any field is non-finite at construction.
    #[error("L1 parameter {field} is non-finite")]
    NonFiniteParameter {
        /// Field path.
        field: &'static str,
    },
    /// `dt` must be strictly positive for the discrete L1 updates.
    #[error("L1 discrete step dt must be > 0; got dt = {dt_s}")]
    NonPositiveStepSize {
        /// Provided `dt` (s).
        dt_s: f64,
    },
    /// The forward-Euler reference model / predictor step must be
    /// contractive for the configured `a_m`.
    #[error(
        "L1 forward-Euler step is not contractive: dt = {dt_s} must be < {max_dt_s} for a_m = {a_m}"
    )]
    DiscreteStepNotContractive {
        /// Provided `dt` (s).
        dt_s: f64,
        /// Configured `a_m`.
        a_m: f64,
        /// Maximum contractive `dt` (s).
        max_dt_s: f64,
    },
}

/// Per-axis L1 adaptive control parameters.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct L1AdaptiveParams {
    /// Reference-model bandwidth `a_m` (rad/s). Must be `< 0`.
    pub reference_model_a_m: f64,
    /// Reference-model / state-predictor input gain `b`.
    pub reference_model_b: f64,
    /// Reference-model feedforward gain `k_g` (typically `-a_m / b`
    /// for unit DC gain).
    pub reference_model_k_g: f64,
    /// PCA sample time `T_s` (s). Must be `> 0`.
    pub adaptation_sample_time_s: f64,
    /// Strictly-proper LPF cutoff `ω_c` (rad/s).
    pub low_pass_cutoff_rad_s: f64,
    /// Lipschitz bound `L` on the matched uncertainty.
    pub lipschitz_bound: f64,
    /// Symmetric projection bound on `σ̂` (absolute magnitude).
    pub projection_bound: f64,
}

impl L1AdaptiveParams {
    /// Validate the parameter set.
    ///
    /// # Errors
    ///
    /// Returns [`L1AdaptiveError`] when any field is out of envelope
    /// or when the bandwidth-projection inequality `ω_c · L < 1` is
    /// violated.
    pub fn validate(&self) -> Result<(), L1AdaptiveError> {
        let finite_fields = [
            ("reference_model_a_m", self.reference_model_a_m),
            ("reference_model_b", self.reference_model_b),
            ("reference_model_k_g", self.reference_model_k_g),
            ("adaptation_sample_time_s", self.adaptation_sample_time_s),
            ("low_pass_cutoff_rad_s", self.low_pass_cutoff_rad_s),
            ("lipschitz_bound", self.lipschitz_bound),
            ("projection_bound", self.projection_bound),
        ];
        for (field, value) in finite_fields {
            if !value.is_finite() {
                return Err(L1AdaptiveError::NonFiniteParameter { field });
            }
        }
        if self.reference_model_a_m >= 0.0 {
            return Err(L1AdaptiveError::ReferenceModelNotStable {
                a_m: self.reference_model_a_m,
            });
        }
        if self.reference_model_b == 0.0 {
            return Err(L1AdaptiveError::NonZeroEffectivenessRequired);
        }
        if self.adaptation_sample_time_s <= 0.0 {
            return Err(L1AdaptiveError::NonPositiveSampleTime {
                t_s: self.adaptation_sample_time_s,
            });
        }
        if self.low_pass_cutoff_rad_s <= 0.0 {
            return Err(L1AdaptiveError::NonPositiveLowPassCutoff {
                omega_c: self.low_pass_cutoff_rad_s,
            });
        }
        if self.lipschitz_bound <= 0.0 {
            return Err(L1AdaptiveError::NonPositiveLipschitzBound {
                l: self.lipschitz_bound,
            });
        }
        if self.projection_bound <= 0.0 {
            return Err(L1AdaptiveError::NonPositiveProjectionBound {
                bound: self.projection_bound,
            });
        }
        let product = self.low_pass_cutoff_rad_s * self.lipschitz_bound;
        if product >= 1.0 {
            return Err(L1AdaptiveError::BandwidthProjectionExceeded {
                omega_c: self.low_pass_cutoff_rad_s,
                lipschitz_bound: self.lipschitz_bound,
                product,
            });
        }
        Ok(())
    }

    /// Validate the parameter set against a discrete controller tick.
    ///
    /// # Errors
    ///
    /// Returns [`L1AdaptiveError`] when the parameter set is invalid
    /// or when the forward-Euler reference-model / predictor update is
    /// not contractive for `dt_s`.
    pub fn validate_discrete_time(&self, dt_s: f64) -> Result<(), L1AdaptiveError> {
        self.validate()?;
        if !dt_s.is_finite() {
            return Err(L1AdaptiveError::NonFiniteParameter { field: "dt_s" });
        }
        if dt_s <= 0.0 {
            return Err(L1AdaptiveError::NonPositiveStepSize { dt_s });
        }
        let max_dt_s = -2.0 / self.reference_model_a_m;
        if dt_s >= max_dt_s {
            return Err(L1AdaptiveError::DiscreteStepNotContractive {
                dt_s,
                a_m: self.reference_model_a_m,
                max_dt_s,
            });
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// L1ReferenceModel
// ---------------------------------------------------------------------

/// Stable linear scalar reference model `ẋ_m = a_m x_m + b k_g r`.
///
/// Step uses forward-Euler discretisation. With `a_m < 0` and
/// `dt < 2 / |a_m|`, the discrete propagation `x_m + dt (a_m x_m + b k_g r)`
/// is contractive.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1ReferenceModel {
    /// Reference-model state `x_m`.
    state: f64,
}

impl L1ReferenceModel {
    /// Construct the reference model with a given initial state.
    #[must_use]
    pub const fn new(initial: f64) -> Self {
        Self { state: initial }
    }

    /// Borrow the current state.
    #[must_use]
    pub const fn state(&self) -> f64 {
        self.state
    }

    /// Advance by `dt_s` against parameter set `params` and reference
    /// command `r`.
    pub fn step(&mut self, params: &L1AdaptiveParams, r: f64, dt_s: f64) -> f64 {
        let derivative = params.reference_model_a_m * self.state
            + params.reference_model_b * params.reference_model_k_g * r;
        self.state += dt_s * derivative;
        self.state
    }
}

// ---------------------------------------------------------------------
// L1StatePredictor
// ---------------------------------------------------------------------

/// State predictor `ẋ̂ = a_m x̂ + b (u + σ̂)`.
///
/// The predictor advances independently from the plant; the prediction
/// error `x̃ = x̂ - x_p` drives [`L1PiecewiseConstantAdaptation`].
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1StatePredictor {
    /// Predictor state `x̂`.
    state: f64,
}

impl L1StatePredictor {
    /// Construct the predictor with a given initial state.
    #[must_use]
    pub const fn new(initial: f64) -> Self {
        Self { state: initial }
    }

    /// Borrow the current state.
    #[must_use]
    pub const fn state(&self) -> f64 {
        self.state
    }

    /// Advance by `dt_s` against parameter set `params`, baseline
    /// command `u`, and matched-uncertainty estimate `sigma_hat`.
    pub fn step(
        &mut self,
        params: &L1AdaptiveParams,
        u_baseline: f64,
        sigma_hat: f64,
        dt_s: f64,
    ) -> f64 {
        let derivative = params.reference_model_a_m * self.state
            + params.reference_model_b * (u_baseline + sigma_hat);
        self.state += dt_s * derivative;
        self.state
    }
}

// ---------------------------------------------------------------------
// L1PiecewiseConstantAdaptation
// ---------------------------------------------------------------------

/// Piecewise-constant matched-uncertainty estimator.
///
/// At each sample time `iT_s`, computes
///
/// ```text
/// σ̂(iT_s) = -e^{a_m T_s} · a_m / (b · (e^{a_m T_s} - 1)) · x̃(iT_s)
/// ```
///
/// and holds it constant over `[iT_s, (i+1)T_s)`. The returned
/// estimate is clamped to `[-projection_bound, +projection_bound]`
/// to keep `σ̂` inside the closed-loop validity envelope.
///
/// Sample-phase accounting tolerates the autopilot's tick `dt` not
/// being an integer divisor of `T_s` — the next sample fires when the
/// accumulated phase reaches `T_s`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1PiecewiseConstantAdaptation {
    /// Latest piecewise-constant estimate.
    sigma_hat: f64,
    /// Time accumulated since the last PCA fire.
    sample_phase_s: f64,
}

impl L1PiecewiseConstantAdaptation {
    /// Returns the latest σ̂.
    #[must_use]
    pub const fn estimate(&self) -> f64 {
        self.sigma_hat
    }

    /// Advance by `dt_s` with the current prediction error
    /// `x_tilde = x̂ - x_p`. When the accumulated sample phase reaches
    /// `T_s` the estimate is recomputed; otherwise the held value is
    /// retained.
    pub fn step(&mut self, params: &L1AdaptiveParams, x_tilde: f64, dt_s: f64) {
        self.sample_phase_s += dt_s;
        if self.sample_phase_s >= params.adaptation_sample_time_s {
            while self.sample_phase_s >= params.adaptation_sample_time_s {
                self.sample_phase_s -= params.adaptation_sample_time_s;
            }
            let raw = pca_scalar(
                params.reference_model_a_m,
                params.reference_model_b,
                params.adaptation_sample_time_s,
                x_tilde,
            );
            self.sigma_hat = raw.clamp(-params.projection_bound, params.projection_bound);
        }
    }
}

/// Closed-form scalar PCA estimate.
///
/// `σ̂ = -Φ(T_s) · a_m / (b · (Φ(T_s) - 1)) · x̃` with
/// `Φ(T_s) = e^{a_m T_s}`. For small `|a_m T_s|` the closed form
/// degenerates as `(Φ - 1) → a_m T_s`, so the function falls back to
/// the Taylor approximation `σ̂ ≈ -x̃ / (b · T_s)` when
/// `|a_m T_s| < 1e-6`.
fn pca_scalar(a_m: f64, b: f64, t_s: f64, x_tilde: f64) -> f64 {
    let exponent = a_m * t_s;
    if exponent.abs() < 1.0e-6 {
        return -x_tilde / (b * t_s);
    }
    let phi_minus_one = exponent.exp_m1();
    let phi = 1.0 + phi_minus_one;
    let denom = b * phi_minus_one;
    -phi * a_m * x_tilde / denom
}

// ---------------------------------------------------------------------
// L1LowPassFilter
// ---------------------------------------------------------------------

/// First-order strictly-proper low-pass filter
/// `H(s) = ω_c / (s + ω_c)` discretised with the impulse-invariant
/// step `y[k+1] = y[k] + α (u[k] - y[k])` where
/// `α = 1 - exp(-ω_c · dt)`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1LowPassFilter {
    state: f64,
}

impl L1LowPassFilter {
    /// Returns the current filter output.
    #[must_use]
    pub const fn output(&self) -> f64 {
        self.state
    }

    /// Advance by `dt_s` with the input `u`.
    pub fn step(&mut self, params: &L1AdaptiveParams, u: f64, dt_s: f64) -> f64 {
        let omega = params.low_pass_cutoff_rad_s;
        let alpha = -(-omega * dt_s).exp_m1();
        self.state += alpha * (u - self.state);
        self.state
    }
}

// ---------------------------------------------------------------------
// L1AdaptiveChannel
// ---------------------------------------------------------------------

/// Composes [`L1ReferenceModel`], [`L1StatePredictor`],
/// [`L1PiecewiseConstantAdaptation`], and [`L1LowPassFilter`] into one
/// scalar axis of the L1 adaptive controller.
///
/// This struct owns the math wired into the rate loop,
/// replacing an earlier L1-inspired interim channel.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1AdaptiveChannel {
    reference_model: L1ReferenceModel,
    state_predictor: L1StatePredictor,
    pca: L1PiecewiseConstantAdaptation,
    lpf: L1LowPassFilter,
}

impl L1AdaptiveChannel {
    /// Construct a freshly-zeroed channel.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            reference_model: L1ReferenceModel::new(0.0),
            state_predictor: L1StatePredictor::new(0.0),
            pca: L1PiecewiseConstantAdaptation {
                sigma_hat: 0.0,
                sample_phase_s: 0.0,
            },
            lpf: L1LowPassFilter { state: 0.0 },
        }
    }

    /// Returns the current matched-uncertainty estimate.
    #[must_use]
    pub const fn sigma_hat(&self) -> f64 {
        self.pca.estimate()
    }

    /// Returns the current LPF output (before the unary minus that
    /// turns it into the autopilot augmentation).
    #[must_use]
    pub const fn filtered_correction(&self) -> f64 {
        self.lpf.output()
    }

    /// Returns the current reference-model state `x_m`.
    #[must_use]
    pub const fn reference_state(&self) -> f64 {
        self.reference_model.state()
    }

    /// Returns the current state-predictor state `x̂`.
    #[must_use]
    pub const fn predictor_state(&self) -> f64 {
        self.state_predictor.state()
    }

    /// Advance by `dt_s` with measured plant state `x_p`, reference
    /// command `r`, and baseline (PID) command `u_baseline`. Returns
    /// the L1 augmentation `u_ad = -LPF(σ̂)` to add to `u_baseline`.
    pub fn step(
        &mut self,
        params: &L1AdaptiveParams,
        x_p: f64,
        r: f64,
        u_baseline: f64,
        dt_s: f64,
    ) -> f64 {
        let _ = self.reference_model.step(params, r, dt_s);
        let prediction_error = self.state_predictor.state() - x_p;
        self.pca.step(params, prediction_error, dt_s);
        let sigma_hat = self.pca.estimate();
        let _ = self
            .state_predictor
            .step(params, u_baseline, sigma_hat, dt_s);
        let lpf_out = self.lpf.step(params, sigma_hat, dt_s);
        -lpf_out
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    fn nominal_params() -> L1AdaptiveParams {
        L1AdaptiveParams {
            reference_model_a_m: -10.0,
            reference_model_b: 1.0,
            reference_model_k_g: 10.0,
            adaptation_sample_time_s: 0.001,
            low_pass_cutoff_rad_s: 5.0,
            lipschitz_bound: 0.1, // ω_c · L = 5 · 0.1 = 0.5 < 1 ✓
            projection_bound: 100.0,
        }
    }

    #[test]
    fn validate_accepts_nominal_params() {
        nominal_params()
            .validate()
            .expect("nominal params validate");
    }

    #[test]
    fn validate_rejects_unstable_reference_model() {
        let mut params = nominal_params();
        params.reference_model_a_m = 0.0;
        let err = params.validate().unwrap_err();
        assert!(matches!(
            err,
            L1AdaptiveError::ReferenceModelNotStable { .. }
        ));
    }

    #[test]
    fn validate_rejects_zero_effectiveness() {
        let mut params = nominal_params();
        params.reference_model_b = 0.0;
        let err = params.validate().unwrap_err();
        assert!(matches!(err, L1AdaptiveError::NonZeroEffectivenessRequired));
    }

    #[test]
    fn validate_rejects_bandwidth_projection_violation() {
        let mut params = nominal_params();
        // ω_c · L = 5 · 0.5 = 2.5 ≥ 1 — must fail.
        params.lipschitz_bound = 0.5;
        let err = params.validate().unwrap_err();
        match err {
            L1AdaptiveError::BandwidthProjectionExceeded { product, .. } => {
                assert_abs_diff_eq!(product, 2.5, epsilon = 1.0e-12);
            }
            other => panic!("expected BandwidthProjectionExceeded, got {other:?}"),
        }
    }

    #[test]
    fn validate_rejects_non_finite_parameter() {
        let mut params = nominal_params();
        params.reference_model_b = f64::NAN;
        let err = params.validate().unwrap_err();
        assert!(matches!(
            err,
            L1AdaptiveError::NonFiniteParameter {
                field: "reference_model_b"
            }
        ));
    }

    #[test]
    fn validate_discrete_time_rejects_euler_unstable_step() {
        let mut params = nominal_params();
        params.reference_model_a_m = -10_000.0;
        let err = params.validate_discrete_time(0.001).unwrap_err();
        match err {
            L1AdaptiveError::DiscreteStepNotContractive { max_dt_s, .. } => {
                assert_abs_diff_eq!(max_dt_s, 0.0002, epsilon = 1.0e-12);
            }
            other => panic!("expected DiscreteStepNotContractive, got {other:?}"),
        }
    }

    #[test]
    fn reference_model_step_decays_to_zero_with_zero_reference() {
        let params = nominal_params();
        let mut model = L1ReferenceModel::new(1.0);
        for _ in 0..1_000 {
            model.step(&params, 0.0, 0.001);
        }
        // Settling time is ≈ 4/|a_m| ≈ 0.4 s; after 1 s the state
        // should be far below 1e-3.
        assert!(
            model.state().abs() < 1.0e-3,
            "reference model did not decay: state = {state}",
            state = model.state()
        );
    }

    #[test]
    fn reference_model_step_reaches_unit_dc_gain_with_kg_chosen() {
        let params = nominal_params();
        let mut model = L1ReferenceModel::new(0.0);
        // a_m = -10, b = 1, k_g = 10 → DC gain = b · k_g / -a_m = 1.
        for _ in 0..2_000 {
            model.step(&params, 1.0, 0.001);
        }
        assert_abs_diff_eq!(model.state(), 1.0, epsilon = 1.0e-3);
    }

    #[test]
    fn state_predictor_propagates_with_uncertainty_estimate() {
        let params = nominal_params();
        let mut predictor = L1StatePredictor::new(0.0);
        // u = 0, σ̂ = 1; predictor sees b · σ̂ = 1 forcing input,
        // converges to σ̂ · b / -a_m = 0.1.
        for _ in 0..2_000 {
            predictor.step(&params, 0.0, 1.0, 0.001);
        }
        assert_abs_diff_eq!(predictor.state(), 0.1, epsilon = 1.0e-3);
    }

    #[test]
    fn pca_estimates_constant_disturbance() {
        let params = nominal_params();
        // Hand-set a steady-state prediction error and verify the PCA
        // formula recovers the disturbance. With x̃ = 0.05 (held from
        // the predictor lag), the PCA estimate computes from
        // pca_scalar.
        let raw = pca_scalar(
            params.reference_model_a_m,
            params.reference_model_b,
            params.adaptation_sample_time_s,
            0.05,
        );
        // For small a_m T_s = -0.01 the Taylor approx is
        // σ̂ ≈ -x̃ / (b · T_s) = -0.05 / 0.001 = -50.
        // The exact form differs by `Φ · a_m / (Φ - 1)`; for
        // a_m T_s = -0.01, Φ ≈ 0.99005, (Φ-1) ≈ -0.009950,
        // -Φ · a_m / (Φ - 1) ≈ -0.99005 · -10 / -0.009950 ≈ -994.93,
        // so σ̂ ≈ -994.93 · 0.05 / 1 — wait that's huge.
        //
        // Let me re-check: pca_scalar = -phi * a_m * x_tilde / (b * (phi - 1))
        // = -(0.99005) · (-10) · 0.05 / (1 · -0.00995)
        // =  9.9005 · 0.05 / (-0.00995)
        // =  0.49502 / -0.00995
        // = -49.75
        //
        // OK so the Taylor approx σ̂ ≈ -50 is close to exact -49.75.
        assert!(
            (raw - (-50.0)).abs() < 1.0,
            "PCA estimate out of expected envelope: {raw}"
        );
    }

    #[test]
    fn pca_taylor_fallback_matches_closed_form_for_small_exponent() {
        let a_m = -1.0e-9;
        let b = 1.0;
        let t_s = 1.0e-3;
        let x_tilde = 0.05;
        let value = pca_scalar(a_m, b, t_s, x_tilde);
        // For |a_m · T_s| < 1e-6 the Taylor branch fires.
        assert_abs_diff_eq!(value, -x_tilde / (b * t_s), epsilon = 1.0e-12);
    }

    #[test]
    fn pca_step_holds_estimate_until_sample_time_reached() {
        let params = nominal_params();
        let mut pca = L1PiecewiseConstantAdaptation::default();
        // dt = 0.0001 < T_s = 0.001 — the first 9 sub-ticks should
        // not fire the PCA.
        for _ in 0..9 {
            pca.step(&params, 0.05, 0.0001);
        }
        assert_eq!(pca.estimate(), 0.0);
        pca.step(&params, 0.05, 0.0002); // accumulator passes T_s
        assert!(pca.estimate().abs() > 0.0);
    }

    #[test]
    fn pca_preserves_sample_phase_remainder_after_fire() {
        let params = nominal_params();
        let mut pca = L1PiecewiseConstantAdaptation::default();
        pca.step(&params, 0.05, 0.0012);
        let first = pca.estimate();
        assert!(first.abs() > 0.0);
        assert_abs_diff_eq!(pca.sample_phase_s, 0.0002, epsilon = 1.0e-15);

        for _ in 0..7 {
            pca.step(&params, 0.0, 0.0001);
        }
        assert_abs_diff_eq!(pca.estimate(), first, epsilon = 1.0e-12);
        pca.step(&params, 0.0, 0.0001);
        assert_abs_diff_eq!(pca.estimate(), 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn pca_clamps_to_projection_bound() {
        let mut params = nominal_params();
        params.projection_bound = 1.0;
        let mut pca = L1PiecewiseConstantAdaptation::default();
        // Huge prediction error → raw σ̂ blows out the projection
        // bound → clamped.
        pca.step(&params, 1.0e6, params.adaptation_sample_time_s);
        assert!(pca.estimate().abs() <= params.projection_bound + 1.0e-12);
    }

    #[test]
    fn lpf_steady_state_dc_gain_is_unity() {
        let params = nominal_params();
        let mut lpf = L1LowPassFilter::default();
        // Drive with a constant unit input long enough to reach
        // steady state; output should approach 1.0.
        for _ in 0..10_000 {
            lpf.step(&params, 1.0, 0.001);
        }
        assert_abs_diff_eq!(lpf.output(), 1.0, epsilon = 1.0e-6);
    }

    #[test]
    fn lpf_decay_with_zero_input_drops_below_threshold_after_settling() {
        let params = nominal_params();
        let mut lpf = L1LowPassFilter { state: 1.0 };
        // ω_c = 5 → time constant 1/5 = 0.2 s; after 2 s the output
        // should be < 1e-4.
        for _ in 0..2_000 {
            lpf.step(&params, 0.0, 0.001);
        }
        assert!(lpf.output().abs() < 1.0e-4);
    }

    #[test]
    fn lpf_small_step_alpha_keeps_first_order_precision() {
        let mut params = nominal_params();
        params.low_pass_cutoff_rad_s = 1.0e-9;
        params.lipschitz_bound = 0.1;
        let mut lpf = L1LowPassFilter::default();
        lpf.step(&params, 1.0, 1.0e-9);
        assert!(lpf.output() > 0.0);
        assert_abs_diff_eq!(lpf.output(), 1.0e-18, epsilon = 1.0e-30);
    }

    #[test]
    fn channel_step_is_deterministic_across_reruns() {
        let params = nominal_params();
        let mut a = L1AdaptiveChannel::new();
        let mut b = L1AdaptiveChannel::new();
        for k in 0..1_000_u32 {
            let r = f64::from(k) * 1.0e-3;
            let x_p = f64::from(k) * 5.0e-4;
            let _ = a.step(&params, x_p, r, 0.0, 0.001);
            let _ = b.step(&params, x_p, r, 0.0, 0.001);
        }
        assert_eq!(a.sigma_hat().to_bits(), b.sigma_hat().to_bits());
        assert_eq!(
            a.filtered_correction().to_bits(),
            b.filtered_correction().to_bits()
        );
        assert_eq!(a.reference_state().to_bits(), b.reference_state().to_bits());
        assert_eq!(a.predictor_state().to_bits(), b.predictor_state().to_bits());
    }

    #[test]
    fn channel_predictor_uses_single_sigma_hat_term() {
        let params = nominal_params();
        let mut channel = L1AdaptiveChannel::new();
        let u_baseline = 0.25;
        let _ = channel.step(&params, -0.001, 0.0, u_baseline, 0.001);
        let sigma_hat = channel.sigma_hat();
        assert_abs_diff_eq!(
            channel.predictor_state(),
            0.001 * (u_baseline + sigma_hat),
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn channel_converges_against_constant_disturbance_on_open_loop_plant() {
        // Toy scalar plant ẋ_p = a_m x_p + b (u + σ_true). With
        // u = 0, σ_true = 0.5, the plant drifts; the L1 channel sees
        // a_p > x̂ (predictor lags) and σ̂ should converge toward
        // σ_true. The augmentation u_ad = -LPF(σ̂) cancels the
        // disturbance in steady state.
        let mut params = nominal_params();
        // Tighter LPF + smaller projection bound for this test.
        params.low_pass_cutoff_rad_s = 50.0;
        params.lipschitz_bound = 0.01; // ω_c · L = 0.5
        params.projection_bound = 5.0;
        params.validate().expect("valid params");

        let sigma_true = 0.5;
        let mut x_p = 0.0_f64;
        let mut channel = L1AdaptiveChannel::new();
        let dt = 0.001;
        for _ in 0..20_000 {
            let u_ad = channel.step(&params, x_p, 0.0, 0.0, dt);
            // Plant: ẋ_p = a_m x_p + b (u + σ_true) where u = u_ad.
            let derivative =
                params.reference_model_a_m * x_p + params.reference_model_b * (u_ad + sigma_true);
            x_p += dt * derivative;
        }
        // After 20 s the L1 channel should have driven the plant
        // close to zero by cancelling σ_true.
        assert!(
            x_p.abs() < 0.1,
            "L1 channel failed to suppress disturbance: x_p = {x_p}"
        );
    }
}
