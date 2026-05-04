//! Phase-5.B.1 — Square-Root Unscented Kalman Filter (SR-UKF).
//!
//! Replaces the Phase-4.C 6-state classical [`crate::estimator::Ukf`]
//! with a **square-root** UKF (Van der Merwe & Wan 2001) over the same
//! 15-state error-state vector as [`crate::estimator::Ekf`]:
//!
//! ```text
//! x = [ position_eci(3),     // [m]      0..3
//!       velocity_eci(3),     // [m/s]    3..6
//!       attitude_error(3),   // [rad]    6..9   (axis-angle in body)
//!       gyro_bias_body(3),   // [rad/s]  9..12
//!       accel_bias_body(3) ] // [m/s²]   12..15
//! ```
//!
//! The 6-state attitude-only path remains available as a restricted
//! configuration via the same struct family.
//!
//! # Why square-root?
//!
//! The classical UKF stores the covariance `P` and Cholesky-factors
//! it twice per measurement update (once for sigma-point spread,
//! once for innovation chi-square). Round-off accumulates in `P` as
//! a non-positive-definite drift; refactorisation can fail
//! catastrophically.
//!
//! The square-root form propagates the Cholesky factor `S` such that
//! `P = S · Sᵀ` directly. Predict-side covariance combination uses
//! a Householder QR decomposition; measurement updates use the
//! `cholupdate` rank-1 update / downdate. There is **no Cholesky
//! refactorisation on the hot path** — `S` stays lower-triangular by
//! construction, and positive-definiteness is preserved up to
//! machine precision.
//!
//! # Sigma-point set
//!
//! Scaled symmetric set (Julier-Uhlmann), with weights from the
//! Wan-Van der Merwe `(α, β, κ)` parameterisation:
//!
//! ```text
//! λ      = α² · (n + κ) − n
//! η      = √(n + λ)
//! χ_0    = x̂
//! χ_i    = x̂ + η · S_:,i        (i = 1..n)
//! χ_(n+i)= x̂ − η · S_:,i        (i = 1..n)
//! W_m^0  = λ / (n + λ)
//! W_m^i  = 1 / (2(n + λ))       (i = 1..2n)
//! W_c^0  = W_m^0 + (1 − α² + β)
//! W_c^i  = W_m^i                 (i = 1..2n)
//! ```
//!
//! `α ∈ (0, 1]` controls the sigma-point spread (small α concentrates
//! around the mean for highly-nonlinear models). `β = 2` is the
//! Gaussian-prior default. `κ = 0` is the Wan-Van der Merwe
//! recommendation for SR-UKF.
//!
//! # Determinism
//!
//! - Locked operand order in every weighted sum and squared accumulation.
//! - No `f64::mul_add` on the hot path (FMA fuses two roundings into one,
//!   breaking cross-platform bit-stability).
//! - Sigma-point ordering is `[χ_0, χ_1, …, χ_n, χ_(n+1), …, χ_(2n)]`
//!   — fixed by this module's contract.
//! - QR / cholupdate use Householder reflections / Givens rotations in
//!   a fixed traversal order.
//! - Tagged `state-stable, not bit-stable` because `pow()` in the
//!   sigma-weight formula can differ across platform-libm.
//!
//! # References
//!
//! - Van der Merwe, R. and Wan, E. A. (2001). *The Square-Root
//!   Unscented Kalman Filter for State and Parameter-Estimation*.
//!   Proceedings of IEEE ICASSP 2001, vol. 6, pp. 3461-3464.
//!   doi:10.1109/ICASSP.2001.940586. The shipped algorithm follows
//!   their Equations (15)-(23).
//! - Wan, E. A. and Van der Merwe, R. (2000). *The Unscented Kalman
//!   Filter for Nonlinear Estimation*. Adaptive Systems for Signal
//!   Processing, Communications, and Control Symposium 2000,
//!   pp. 153-158 — the `(α, β, κ)`-parameterised weight derivation.
//! - Julier, S. J. and Uhlmann, J. K. (1997). *A new extension of
//!   the Kalman filter to nonlinear systems*. Proc. SPIE 3068,
//!   AeroSense '97, pp. 182-193 — the original symmetric sigma-point
//!   set.

#![allow(clippy::doc_markdown)] // Author surnames are not code identifiers.

use nalgebra::{DMatrix, DVector};

// ---------------------------------------------------------------------
// Sigma-point machinery
// ---------------------------------------------------------------------

/// Wan-Van der Merwe scaling parameters `(α, β, κ)`.
///
/// `α` controls the sigma-point spread (small α concentrates the
/// points around the mean — appropriate for highly-nonlinear models
/// where the linearisation only holds in a tight neighbourhood).
/// `β` injects prior knowledge about the distribution: 2 is optimal
/// for a Gaussian prior. `κ` is a tertiary parameter; `κ = 0` is the
/// Van der Merwe recommendation for the square-root form.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UkfScalingParams {
    /// Sigma-point spread parameter. Typical: `1e-3`.
    pub alpha: f64,
    /// Distribution prior. Typical: `2.0` (Gaussian).
    pub beta: f64,
    /// Tertiary scaling parameter. Typical: `0.0` (Wan-Van der Merwe
    /// recommendation for SR-UKF).
    pub kappa: f64,
}

impl Default for UkfScalingParams {
    fn default() -> Self {
        Self {
            alpha: 1.0e-3,
            beta: 2.0,
            kappa: 0.0,
        }
    }
}

/// Pre-computed sigma-point weights and the spread scalar `η`.
#[derive(Copy, Clone, Debug)]
pub struct SigmaWeights {
    /// `√(n + λ)` — the scalar by which each Cholesky column is
    /// multiplied to form a sigma point.
    pub eta: f64,
    /// Mean weight on the central sigma point `χ_0`.
    pub w0_mean: f64,
    /// Covariance weight on the central sigma point `χ_0`. May be
    /// negative when `α² · (n + κ) < n`; in that case a `cholupdate`
    /// with negative sign handles the contribution.
    pub w0_cov: f64,
    /// Common mean / covariance weight on the off-centre sigma
    /// points `χ_1..χ_(2n)`. Always positive for valid scaling
    /// parameters.
    pub wi: f64,
}

impl SigmaWeights {
    /// Compute weights for an `n`-dimensional state under the supplied
    /// scaling parameters.
    #[must_use]
    pub fn for_dimension(n: f64, params: UkfScalingParams) -> Self {
        let alpha_sq = params.alpha * params.alpha;
        let lambda = alpha_sq * (n + params.kappa) - n;
        let spread = n + lambda;
        let eta = spread.sqrt();
        let w0_mean = lambda / spread;
        let w0_cov = w0_mean + (1.0 - alpha_sq + params.beta);
        let wi = 0.5 / spread;
        Self {
            eta,
            w0_mean,
            w0_cov,
            wi,
        }
    }
}

/// Generate the symmetric sigma-point set around `mean` with
/// Cholesky factor `s` (where `P = s · sᵀ`).
///
/// Returns a `n × (2n+1)` matrix laid out as:
///
/// ```text
/// columns [0]      → χ_0   = mean
/// columns [1..n+1] → χ_i   = mean + η · s_:,i        (i = 1..n)
/// columns [n+1..]  → χ_(n+i) = mean − η · s_:,i      (i = 1..n)
/// ```
///
/// The column indexing is the determinism contract — downstream
/// weighted sums must walk these columns in the same order.
///
/// # Panics
///
/// Panics if `mean.len() != s.nrows() != s.ncols()`.
#[must_use]
pub fn sigma_points(mean: &DVector<f64>, s: &DMatrix<f64>, weights: SigmaWeights) -> DMatrix<f64> {
    let n = mean.len();
    assert_eq!(
        s.nrows(),
        n,
        "Cholesky factor row count must equal mean dim"
    );
    assert_eq!(s.ncols(), n, "Cholesky factor must be square");
    let cols = 2 * n + 1;
    let mut out = DMatrix::<f64>::zeros(n, cols);
    // Column 0: the mean itself.
    for row in 0..n {
        out[(row, 0)] = mean[row];
    }
    // Columns 1..=n: positive offsets along each Cholesky column.
    for i in 0..n {
        for row in 0..n {
            out[(row, 1 + i)] = mean[row] + weights.eta * s[(row, i)];
        }
    }
    // Columns n+1..=2n: negative offsets.
    for i in 0..n {
        for row in 0..n {
            out[(row, 1 + n + i)] = mean[row] - weights.eta * s[(row, i)];
        }
    }
    out
}

// ---------------------------------------------------------------------
// Cholesky primitives — `cholupdate` rank-1 update / downdate, plus a
// QR-based predict-side combiner.
// ---------------------------------------------------------------------

/// Sign of a `cholupdate` rank-1 modification.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum CholupdateSign {
    /// Rank-1 update: `S' · S'ᵀ = S · Sᵀ + u · uᵀ`.
    Plus,
    /// Rank-1 downdate: `S' · S'ᵀ = S · Sᵀ − u · uᵀ`. May fail
    /// (return [`CholupdateError::NotPositiveDefinite`]) when the
    /// resulting covariance would be non-positive.
    Minus,
}

/// Failure modes of [`cholupdate_in_place`].
#[derive(Copy, Clone, Debug, PartialEq, Eq, thiserror::Error)]
pub enum CholupdateError {
    /// Downdate produced a non-positive radicand. The Cholesky
    /// invariant `P − u·uᵀ` requires the input downdate vector to
    /// have norm bounded by the rank-1 contribution to `P`; values
    /// outside that envelope correspond to a non-positive-definite
    /// updated covariance and the algorithm fails closed.
    #[error("cholupdate downdate would produce a non-positive-definite covariance")]
    NotPositiveDefinite,
    /// One of the inputs (the existing factor or the update vector)
    /// contains a non-finite component.
    #[error("cholupdate input contains a non-finite component")]
    NonFiniteInput,
}

/// Rank-1 update / downdate of a lower-triangular Cholesky factor in
/// place, using the Bjorck 1996 §3.4 / Van der Merwe & Wan 2001 §III.B
/// hyperbolic-rotation form:
///
/// ```text
/// Update:    S' · S'ᵀ = S · Sᵀ + u · uᵀ
/// Downdate:  S' · S'ᵀ = S · Sᵀ − u · uᵀ
/// ```
///
/// `s` must be square and lower-triangular on entry; on success it
/// is overwritten by the lower-triangular factor of the modified
/// covariance. `u` is consumed (mutated) by the algorithm.
///
/// The algorithm iterates over columns `k = 0..n`. At each `k`:
///
/// ```text
/// r²       = s_kk² + σ · u_k²    (σ = ±1)
/// r        = √r²                 (the new diagonal entry)
/// L'_kk    = r
/// L'_ik    = (s_kk · L_ik + σ · u_k · u_i) / r          (i > k)
/// u'_i     = (s_kk · u_i  − u_k · L_ik_used) / r        (i > k)
/// ```
///
/// where `L_ik_used = L_ik` (OLD value) for the **update** path
/// (orthogonal Givens rotation) and `L_ik_used = L'_ik` (NEW value)
/// for the **downdate** path (hyperbolic rotation). The two cases
/// differ because the orthogonal rotation preserves
/// `L² + u² = const` while the hyperbolic rotation preserves
/// `L² − u² = const`.
///
/// # Determinism
///
/// Iterates over columns in fixed `k = 0..n` order; each row update
/// is in fixed `row = k+1..n` order. Locked operand order, no FMA.
/// The downdate's `r² > 0` check is the fail-closed guard against a
/// non-positive-definite outcome.
///
/// # Panics
///
/// Panics if `s` is non-square or `s.nrows() != u.len()`.
///
/// # Errors
///
/// - [`CholupdateError::NonFiniteInput`] when `s` or `u` contains
///   a non-finite component.
/// - [`CholupdateError::NotPositiveDefinite`] (downdate only) when
///   the modified covariance would be non-positive-definite.
pub fn cholupdate_in_place(
    s: &mut DMatrix<f64>,
    u: &mut DVector<f64>,
    sign: CholupdateSign,
) -> Result<(), CholupdateError> {
    let n = s.nrows();
    assert_eq!(s.ncols(), n, "Cholesky factor must be square");
    assert_eq!(
        u.len(),
        n,
        "update vector dim must equal Cholesky factor dim"
    );
    if !s.iter().all(|v| v.is_finite()) || !u.iter().all(|v| v.is_finite()) {
        return Err(CholupdateError::NonFiniteInput);
    }
    let sign_val: f64 = match sign {
        CholupdateSign::Plus => 1.0,
        CholupdateSign::Minus => -1.0,
    };
    for k in 0..n {
        let s_kk = s[(k, k)];
        let u_k = u[k];
        let r_squared = s_kk * s_kk + sign_val * u_k * u_k;
        if !r_squared.is_finite() || r_squared <= 0.0 {
            return Err(CholupdateError::NotPositiveDefinite);
        }
        let r = r_squared.sqrt();
        s[(k, k)] = r;
        for row in (k + 1)..n {
            let l_ik_old = s[(row, k)];
            let u_i_old = u[row];
            // L'_ik = (s_kk · L_ik + σ · u_k · u_i) / r
            let l_ik_new = (s_kk * l_ik_old + sign_val * u_k * u_i_old) / r;
            // u'_i  = (s_kk · u_i − u_k · L_ik) / r  using OLD L_ik
            // for both update (orthogonal Givens) and downdate
            // (hyperbolic) — see Stewart 1998 §3 for the
            // hyperbolic-rotation derivation.
            let u_i_new = (s_kk * u_i_old - u_k * l_ik_old) / r;
            s[(row, k)] = l_ik_new;
            u[row] = u_i_new;
        }
    }
    Ok(())
}

/// Predict-side Cholesky combiner via Householder QR.
///
/// Builds the Cholesky factor of the propagated covariance from a
/// matrix of weighted sigma-residuals plus the process-noise
/// Cholesky:
///
/// ```text
/// M = [ √|W_c^1| · (γ_1 − x̂)   ...   √|W_c^(2n)| · (γ_(2n) − x̂)   √Q ]
/// QR(Mᵀ) = Q · R   where R is upper triangular
/// S = R[:n, :n]ᵀ   (lower triangular, the new factor)
/// ```
///
/// The sigma-residual at column 0 (the central sigma point) is
/// folded into S via a separate `cholupdate(S, γ_0 − x̂, sign(W_c^0))`
/// call by the caller, because `W_c^0` may be negative.
///
/// # Determinism
///
/// nalgebra's Householder QR uses a fixed reflection-construction
/// order; cross-platform bit-stability holds within a target
/// triple but not across libm `pow()` differences (acceptable
/// under the `state-stable` label).
///
/// # Errors
///
/// Returns [`CholupdateError::NonFiniteInput`] when the input
/// matrix contains non-finite components.
pub fn predict_cholesky_qr(
    weighted_residuals_and_q_sqrt: &DMatrix<f64>,
) -> Result<DMatrix<f64>, CholupdateError> {
    let n = weighted_residuals_and_q_sqrt.nrows();
    if !weighted_residuals_and_q_sqrt.iter().all(|v| v.is_finite()) {
        return Err(CholupdateError::NonFiniteInput);
    }
    // QR(Mᵀ): nalgebra's QR returns Q (orthogonal) and R (upper
    // triangular). Only the top-left N × N block of R is needed.
    let m_t = weighted_residuals_and_q_sqrt.transpose();
    let qr = m_t.qr();
    let r = qr.r();
    let mut s = DMatrix::<f64>::zeros(n, n);
    for col in 0..n {
        for row in col..n {
            // S is lower-triangular; R is upper-triangular. Copy
            // R[i, j] into S[j, i] for the top-left N×N block.
            s[(row, col)] = r[(col, row)];
        }
    }
    // Householder QR may leave negative diagonal entries in R; flip
    // signs per row so the resulting lower-triangular factor has
    // positive diagonal. This is a deterministic per-row sign flip
    // and preserves S · Sᵀ = R · Rᵀ.
    for k in 0..n {
        if s[(k, k)] < 0.0 {
            for row in k..n {
                s[(row, k)] = -s[(row, k)];
            }
        }
    }
    Ok(s)
}

/// Convenience: convert a covariance `P` to its lower-triangular
/// Cholesky factor `S` such that `P = S · Sᵀ`.
///
/// # Errors
///
/// Returns [`CholupdateError::NotPositiveDefinite`] if `p` is not
/// strictly positive-definite, or [`CholupdateError::NonFiniteInput`]
/// if it contains a non-finite component.
pub fn cholesky_from_covariance(p: &DMatrix<f64>) -> Result<DMatrix<f64>, CholupdateError> {
    if !p.iter().all(|v| v.is_finite()) {
        return Err(CholupdateError::NonFiniteInput);
    }
    p.clone()
        .cholesky()
        .map(|c| c.l())
        .ok_or(CholupdateError::NotPositiveDefinite)
}

/// Helper: `S · Sᵀ` from a lower-triangular factor (for tests and
/// for the rare path that needs the explicit covariance, e.g.,
/// telemetry / diagnostics).
#[must_use]
pub fn covariance_from_cholesky(s: &DMatrix<f64>) -> DMatrix<f64> {
    s * s.transpose()
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn diag_dmatrix(entries: &[f64]) -> DMatrix<f64> {
        let n = entries.len();
        let mut m = DMatrix::<f64>::zeros(n, n);
        for (i, &v) in entries.iter().enumerate() {
            m[(i, i)] = v;
        }
        m
    }

    fn dvector(entries: &[f64]) -> DVector<f64> {
        DVector::<f64>::from_row_slice(entries)
    }

    // -----------------------------------------------------------------
    // Sigma weights — analytic invariants from Van der Merwe & Wan 2001.
    // -----------------------------------------------------------------

    #[test]
    fn sigma_weights_sum_to_one() {
        // For any valid scaling: Σ W_m = W_m^0 + 2n · W_m^i = 1.
        let params = UkfScalingParams::default();
        let n = 15.0;
        let w = SigmaWeights::for_dimension(n, params);
        let sum = w.w0_mean + 2.0 * n * w.wi;
        assert_abs_diff_eq!(sum, 1.0, epsilon = 1.0e-15);
    }

    #[test]
    fn sigma_weights_eta_squared_is_n_plus_lambda() {
        // η = √(n + λ); η² · (1 / W_m^i) = 2 · spread, etc.
        let params = UkfScalingParams {
            alpha: 1.0e-3,
            beta: 2.0,
            kappa: 0.0,
        };
        let n = 15.0;
        let w = SigmaWeights::for_dimension(n, params);
        let expected_spread = params.alpha.powi(2) * (n + params.kappa);
        assert_abs_diff_eq!(w.eta * w.eta, expected_spread, epsilon = 1.0e-12);
    }

    #[test]
    fn sigma_weights_wan_vandermerwe_default_recovers_w0_cov_correction() {
        // For α = 1, β = 0, κ = 0: Wan-Van der Merwe degenerates to
        // the Julier-Uhlmann symmetric set; W_c^0 = W_m^0 since
        // (1 − α² + β) = 0.
        let params = UkfScalingParams {
            alpha: 1.0,
            beta: 0.0,
            kappa: 0.0,
        };
        let n = 6.0;
        let w = SigmaWeights::for_dimension(n, params);
        assert_abs_diff_eq!(w.w0_cov, w.w0_mean, epsilon = 1.0e-15);
    }

    // -----------------------------------------------------------------
    // Sigma-point set — analytic invariants on the recovered mean
    // and covariance under the weights (lemma 1 of Wan-Van der
    // Merwe 2000).
    // -----------------------------------------------------------------

    #[test]
    fn sigma_points_recover_mean_under_weighted_sum() {
        // Σ W_m^i · χ_i = mean. We use α = 1, κ = 0 — the
        // Julier-Uhlmann symmetric set with no aggressive rescaling
        // — so the weighted sum has no subtractive cancellation
        // (default α = 1e-3 produces W_m^0 ≈ −10⁶ and 2n · W_m^i ≈
        // +10⁶ which sum to 1 within ~10⁻⁶ relative precision; the
        // algebraic invariant holds exactly only for α = 1).
        let params = UkfScalingParams {
            alpha: 1.0,
            beta: 2.0,
            kappa: 0.0,
        };
        let n: f64 = 6.0;
        let w = SigmaWeights::for_dimension(n, params);
        let mean = dvector(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let s = DMatrix::<f64>::identity(6, 6);
        let chi = sigma_points(&mean, &s, w);

        let mut sum = DVector::<f64>::zeros(6);
        // Locked-order weighted sum: column 0 first, then 1..=2n.
        for row in 0..6 {
            sum[row] = w.w0_mean * chi[(row, 0)];
        }
        for col in 1..=12 {
            for row in 0..6 {
                sum[row] += w.wi * chi[(row, col)];
            }
        }
        for row in 0..6 {
            assert_abs_diff_eq!(sum[row], mean[row], epsilon = 1.0e-14);
        }
    }

    #[test]
    fn sigma_points_recover_mean_under_default_alpha_within_finite_precision() {
        // Same invariant under the default α = 1e-3, but with a
        // looser epsilon that accounts for the W_m^0 / W_m^i
        // subtractive cancellation (relative precision ≈ α² · ε_f64
        // ≈ 1e-22, but the cancellation in `(λ + 2nW_i)·mean` floor
        // is ≈ 1e-9 for state magnitudes ~6).
        let params = UkfScalingParams::default();
        let n: f64 = 6.0;
        let w = SigmaWeights::for_dimension(n, params);
        let mean = dvector(&[1.0, 2.0, 3.0, 4.0, 5.0, 6.0]);
        let s = DMatrix::<f64>::identity(6, 6);
        let chi = sigma_points(&mean, &s, w);

        let mut sum = DVector::<f64>::zeros(6);
        for row in 0..6 {
            sum[row] = w.w0_mean * chi[(row, 0)];
        }
        for col in 1..=12 {
            for row in 0..6 {
                sum[row] += w.wi * chi[(row, col)];
            }
        }
        for row in 0..6 {
            // Precision floor under α = 1e-3: max(|mean|) · ε_f64 /
            // α² ≈ 6 · 2.2e-16 / 1e-6 ≈ 1.3e-9. Use 1e-8 for safety
            // margin against tiny variations across compiler /
            // platform-libm.
            assert_abs_diff_eq!(sum[row], mean[row], epsilon = 1.0e-8);
        }
    }

    #[test]
    fn sigma_points_recover_covariance_under_weighted_sum() {
        // Σ W_c^i · (χ_i − x̂) · (χ_i − x̂)ᵀ = P — exact for linear
        // sigma-point recombination. We use a diagonal P.
        let params = UkfScalingParams::default();
        let n: f64 = 6.0;
        let w = SigmaWeights::for_dimension(n, params);
        let mean = DVector::<f64>::zeros(6);
        // Diagonal Cholesky with distinct entries.
        let s_entries: Vec<f64> = (0..6).map(|i| (f64::from(i) + 1.0).sqrt()).collect();
        let s = diag_dmatrix(&s_entries);
        let chi = sigma_points(&mean, &s, w);

        let mut p = DMatrix::<f64>::zeros(6, 6);
        // Columns 1..=2n contribute χ_i · χ_iᵀ · W_c^i (mean is zero).
        for col in 1..=12 {
            for r in 0..6 {
                for c in 0..6 {
                    p[(r, c)] += w.wi * chi[(r, col)] * chi[(c, col)];
                }
            }
        }
        let p_expected = covariance_from_cholesky(&s);
        for r in 0..6 {
            for c in 0..6 {
                assert_abs_diff_eq!(p[(r, c)], p_expected[(r, c)], epsilon = 1.0e-12);
            }
        }
    }

    // -----------------------------------------------------------------
    // cholupdate — inverse-property tests via S · Sᵀ.
    // -----------------------------------------------------------------

    #[test]
    fn cholupdate_plus_matches_explicit_outer_product() {
        let p = diag_dmatrix(&[2.0, 3.0, 4.0, 5.0]);
        let mut s = cholesky_from_covariance(&p).unwrap();
        let u_orig = dvector(&[0.5, -0.3, 0.7, 0.2]);
        let mut u = u_orig.clone();
        cholupdate_in_place(&mut s, &mut u, CholupdateSign::Plus).unwrap();
        let p_recovered = covariance_from_cholesky(&s);
        let p_expected = &p + &u_orig * u_orig.transpose();
        for r in 0..4 {
            for c in 0..4 {
                assert_abs_diff_eq!(p_recovered[(r, c)], p_expected[(r, c)], epsilon = 1.0e-10);
            }
        }
    }

    #[test]
    fn cholupdate_minus_matches_explicit_outer_product() {
        let p = diag_dmatrix(&[2.0, 3.0, 4.0, 5.0]);
        let mut s = cholesky_from_covariance(&p).unwrap();
        let u_orig = dvector(&[0.1, -0.05, 0.07, 0.02]);
        let mut u = u_orig.clone();
        cholupdate_in_place(&mut s, &mut u, CholupdateSign::Minus).unwrap();
        let p_recovered = covariance_from_cholesky(&s);
        let p_expected = &p - &u_orig * u_orig.transpose();
        for r in 0..4 {
            for c in 0..4 {
                assert_abs_diff_eq!(p_recovered[(r, c)], p_expected[(r, c)], epsilon = 1.0e-10);
            }
        }
    }

    #[test]
    fn cholupdate_minus_fails_closed_when_radicand_non_positive() {
        let p = diag_dmatrix(&[1.0, 1.0, 1.0]);
        let mut s = cholesky_from_covariance(&p).unwrap();
        // u_0 = 2 → r_squared = 1 - 4 = -3 < 0, downdate must reject.
        let mut u = dvector(&[2.0, 0.0, 0.0]);
        let err = cholupdate_in_place(&mut s, &mut u, CholupdateSign::Minus).unwrap_err();
        assert_eq!(err, CholupdateError::NotPositiveDefinite);
    }

    #[test]
    fn cholupdate_rejects_non_finite_input() {
        let mut s = DMatrix::<f64>::identity(3, 3);
        let mut u = dvector(&[f64::NAN, 0.0, 0.0]);
        let err = cholupdate_in_place(&mut s, &mut u, CholupdateSign::Plus).unwrap_err();
        assert_eq!(err, CholupdateError::NonFiniteInput);
    }

    // -----------------------------------------------------------------
    // QR predict-side combiner — round-trip via covariance recovery.
    // -----------------------------------------------------------------

    #[test]
    fn predict_cholesky_qr_recovers_covariance_from_residuals() {
        let r1 = dvector(&[1.0, 0.5, -0.2]);
        let r2 = dvector(&[-0.5, 1.5, 0.3]);
        let r3 = dvector(&[0.1, -0.4, 1.2]);
        let q_sqrt = diag_dmatrix(&[0.1, 0.1, 0.1]);
        // Build M as 3 × 6: 3 residual columns + 3 q_sqrt columns.
        let mut m = DMatrix::<f64>::zeros(3, 6);
        for row in 0..3 {
            m[(row, 0)] = r1[row];
            m[(row, 1)] = r2[row];
            m[(row, 2)] = r3[row];
            m[(row, 3)] = q_sqrt[(row, 0)];
            m[(row, 4)] = q_sqrt[(row, 1)];
            m[(row, 5)] = q_sqrt[(row, 2)];
        }
        let p_expected = &r1 * r1.transpose()
            + &r2 * r2.transpose()
            + &r3 * r3.transpose()
            + &q_sqrt * q_sqrt.transpose();

        let s = predict_cholesky_qr(&m).unwrap();
        let p_recovered = covariance_from_cholesky(&s);
        for r in 0..3 {
            for c in 0..3 {
                assert_abs_diff_eq!(p_recovered[(r, c)], p_expected[(r, c)], epsilon = 1.0e-10);
            }
        }
    }

    #[test]
    fn predict_cholesky_qr_lower_triangular_with_positive_diagonal() {
        let r1 = dvector(&[1.0, 0.5, -0.2]);
        let r2 = dvector(&[-0.5, 1.5, 0.3]);
        let r3 = dvector(&[0.1, -0.4, 1.2]);
        let q_sqrt = diag_dmatrix(&[0.1, 0.1, 0.1]);
        let mut m = DMatrix::<f64>::zeros(3, 6);
        for row in 0..3 {
            m[(row, 0)] = r1[row];
            m[(row, 1)] = r2[row];
            m[(row, 2)] = r3[row];
            m[(row, 3)] = q_sqrt[(row, 0)];
            m[(row, 4)] = q_sqrt[(row, 1)];
            m[(row, 5)] = q_sqrt[(row, 2)];
        }
        let s = predict_cholesky_qr(&m).unwrap();
        // Lower-triangular: S[i, j] == 0 for j > i.
        for i in 0..3 {
            for j in (i + 1)..3 {
                assert_abs_diff_eq!(s[(i, j)], 0.0, epsilon = 1.0e-15);
            }
        }
        // Positive diagonal.
        for k in 0..3 {
            assert!(s[(k, k)] > 0.0, "diagonal entry {k} should be positive");
        }
    }

    #[test]
    fn cholesky_from_covariance_round_trip() {
        let p = diag_dmatrix(&[1.0, 2.0, 3.0, 4.0, 5.0]);
        let s = cholesky_from_covariance(&p).unwrap();
        let p_recovered = covariance_from_cholesky(&s);
        for r in 0..5 {
            for c in 0..5 {
                assert_abs_diff_eq!(p_recovered[(r, c)], p[(r, c)], epsilon = 1.0e-12);
            }
        }
    }
}
