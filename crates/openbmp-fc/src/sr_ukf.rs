//! Square-Root Unscented Kalman Filter (SR-UKF).
//!
//! A **square-root** UKF-family estimator over the same 15-state error-state
//! vector as [`crate::estimator::Ekf`]:
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
//! machine precision. The restricted `SquareRootUkfAttitude` wrapper is
//! the one exception: after covariance-changing operations it projects
//! the unused 9-state subspace back to fixed floors and rebuilds the
//! Cholesky factor so the exposed 6-state covariance cannot drift.
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
//!   doi:10.1109/ICASSP.2001.940586. The shipped measurement update
//!   follows their Equations (19)-(23); the predict step intentionally
//!   uses the linearized inertial error-state propagation documented
//!   below rather than their full nonlinear sigma-point predict.
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
#[cfg(not(feature = "std"))]
use num_traits::Float;
use std::boxed::Box;
use std::format;

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
/// where `L_ik_used = L_ik` (the OLD value captured before writing
/// `L'_ik`) for both the **update** path (orthogonal Givens rotation)
/// and the **downdate** path (Stewart-style hyperbolic rotation). With
/// `cos(θ) = s_kk / r` and `sin(θ) = u_k / r`, the update is the
/// ordinary rotation
/// `L'_ik = cos θ · L_ik + sin θ · u_i`,
/// `u'_i = cos θ · u_i − sin θ · L_ik`. The downdate uses the same
/// locked OLD-value capture with `σ = -1`, preserving
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
    // signs per column of S so the resulting lower-triangular factor
    // has a positive diagonal. This deterministic column sign flip
    // preserves S · Sᵀ.
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

fn covariance_diag_condition(p: &DMatrix<f64>) -> f64 {
    let dim = p.nrows().min(p.ncols());
    let mut max_diag = 0.0_f64;
    let mut min_diag = f64::INFINITY;
    for i in 0..dim {
        let diag = p[(i, i)];
        if !diag.is_finite() {
            return f64::INFINITY;
        }
        let non_negative = diag.max(0.0);
        max_diag = max_diag.max(non_negative);
        if non_negative > 0.0 {
            min_diag = min_diag.min(non_negative);
        }
    }
    if !min_diag.is_finite() || min_diag == 0.0 {
        f64::INFINITY
    } else {
        max_diag / min_diag
    }
}

fn covariance_diag3(p: &DMatrix<f64>, start: usize) -> [f64; 3] {
    [
        p[(start, start)],
        p[(start + 1, start + 1)],
        p[(start + 2, start + 2)],
    ]
}

// =====================================================================
// `SquareRootUkf` (15-state error-state filter)
// =====================================================================

use nalgebra::{SVector, UnitQuaternion, Vector3};
use openbmp_core::{Eci, Position3, SimTime};
use openbmp_physics::gravity::{self, ConstantGravity, GravityModel};
use openbmp_physics::kinematics::{
    quaternion_error_small_angle, quaternion_from_axis_angle, quaternion_from_omega,
    renormalize_quaternion,
};
use openbmp_physics::magnetic::{EarthDipoleField, MagneticFieldEci};

use crate::error::EstimatorError;
use crate::params::ParamSection;
use crate::topics::{
    AttitudeEstimate, BarometerSample, EstimatorMode, EstimatorStatus, GnssSample, ImuSample,
    MagnetometerSample, PositionEstimate, StarTrackerSample,
};

/// 15-state error-state vector dimension.
const SRUKF_STATE_DIM: usize = 15;
const ATTITUDE_UNDER_OBSERVABLE_VARIANCE_RAD2: f64 = 1.0;
const ATTITUDE_ONLY_UNUSED_VARIANCE: f64 = 1.0e-12;

/// Configuration parameters for the SR-UKF — same physical interpretation
/// as [`crate::estimator::EkfParams`] (process / measurement noise,
/// Gauss-Markov bias time constants, innovation gating). The
/// sigma-point-specific scaling parameters live in
/// [`UkfScalingParams`] and are stored separately on the filter so they
/// can be tuned without touching the noise budget.
#[derive(Clone, Debug, PartialEq)]
pub struct SquareRootUkfParams {
    /// Process-noise standard deviation on attitude rate (rad/s).
    pub sigma_w_gyro: f64,
    /// Process-noise standard deviation on accelerometer bias random
    /// walk (m/s²/√s).
    pub sigma_w_accel_bias: f64,
    /// Process-noise standard deviation on gyro bias random walk
    /// (rad/s/√s).
    pub sigma_w_gyro_bias: f64,
    /// Process-noise standard deviation on the position state (m/√s).
    pub sigma_w_position_m: f64,
    /// Process-noise standard deviation on the velocity state (m/s/√s).
    /// Without it the position/velocity covariance collapses below the
    /// GNSS measurement-noise floor; once ascent dynamics arrive the
    /// (otherwise nominal) GNSS innovation then blows the overconfident
    /// gate, the filter rejects every fix, dead-reckons, and diverges.
    /// Mirrors [`crate::estimator::EkfParams::sigma_w_velocity_m_s`].
    pub sigma_w_velocity_m_s: f64,
    /// First-order Gauss-Markov gyro-bias time constant. `∞` preserves
    /// the random-walk limit.
    pub tau_gyro_bias_s: f64,
    /// First-order Gauss-Markov accel-bias time constant.
    pub tau_accel_bias_s: f64,
    /// Measurement-noise standard deviation on each GNSS position
    /// component (m).
    pub sigma_gnss_pos_m: f64,
    /// Measurement-noise standard deviation on each GNSS velocity
    /// component (m/s).
    pub sigma_gnss_vel_m_s: f64,
    /// Measurement-noise standard deviation on barometric altitude (m).
    pub sigma_baro_alt_m: f64,
    /// Measurement-noise standard deviation on each magnetometer
    /// component (nT).
    pub sigma_mag_nt: f64,
    /// Measurement-noise standard deviation on the star-tracker attitude
    /// fix (rad, per axis). The star tracker is a full 3-DOF attitude
    /// observation — without fusing it the attitude estimate drifts on
    /// gyro + (2-DOF) magnetometer alone and diverges during ascent.
    /// Mirrors [`crate::estimator::EkfParams::sigma_star_tracker_rad`].
    pub sigma_star_tracker_rad: f64,
    /// Optional explicit innovation-gate threshold (`NaN` derives from
    /// `innovation_false_alarm_rate`).
    pub innovation_gate: f64,
    /// False-alarm probability for derived chi-square gates.
    pub innovation_false_alarm_rate: f64,
    /// Dead-reckoning timeout in seconds.
    pub dead_reckon_timeout_s: f64,
    /// Multiplier on position/velocity process-noise covariance while the
    /// high-dynamics regime is active (see
    /// [`crate::estimator::Estimator::set_high_dynamics_process_noise`]).
    /// `1.0` is a no-op. Mirrors
    /// [`crate::estimator::EkfParams::high_dynamics_q_scale`].
    pub high_dynamics_q_scale: f64,
}

impl Default for SquareRootUkfParams {
    fn default() -> Self {
        Self {
            sigma_w_gyro: 0.01,
            sigma_w_accel_bias: 1.0e-4,
            sigma_w_gyro_bias: 1.0e-5,
            sigma_w_position_m: 0.0,
            sigma_w_velocity_m_s: 0.0,
            tau_gyro_bias_s: f64::INFINITY,
            tau_accel_bias_s: f64::INFINITY,
            sigma_gnss_pos_m: 5.0,
            sigma_gnss_vel_m_s: 0.5,
            sigma_baro_alt_m: 2.0,
            sigma_mag_nt: 100.0,
            sigma_star_tracker_rad: 1.0e-4,
            innovation_gate: f64::NAN,
            innovation_false_alarm_rate: 0.01,
            dead_reckon_timeout_s: 1.5,
            high_dynamics_q_scale: 1.0,
        }
    }
}

impl ParamSection for SquareRootUkfParams {
    const NAME: &'static str = "estimator.sr_ukf";
}

impl SquareRootUkfParams {
    fn gate_for_dof(&self, dof: f64) -> f64 {
        if self.innovation_gate.is_finite() && self.innovation_gate > 0.0 {
            return self.innovation_gate;
        }
        let probability = (1.0 - self.innovation_false_alarm_rate).clamp(0.5, 0.999_999_999);
        openbmp_physics::statistics::chi_square_inverse_cdf_wilson_hilferty(probability, dof)
    }
}

/// Gravity-model adapter — identical contract to the one in
/// `crate::estimator::Ekf`, owned by the SR-UKF here so the two
/// estimators don't share private types.
struct GravityAdapter {
    inner: Box<dyn GravityModel + Send + Sync>,
}

impl std::fmt::Debug for GravityAdapter {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("GravityAdapter").finish_non_exhaustive()
    }
}

impl GravityAdapter {
    fn new<G: GravityModel + Send + Sync + 'static>(model: G) -> Self {
        Self {
            inner: Box::new(model),
        }
    }

    fn at(
        &self,
        position_eci_m: Vector3<f64>,
        time: SimTime,
    ) -> Result<Vector3<f64>, EstimatorError> {
        let p = Position3::<Eci>::new(position_eci_m.x, position_eci_m.y, position_eci_m.z);
        self.inner
            .gravity_eci_m_s2(p, time)
            .map_err(|err| EstimatorError::InvalidConfig {
                reason: format!("gravity model rejected query: {err}"),
            })
    }
}

#[allow(clippy::expect_used)]
fn default_constant_gravity_down_z() -> ConstantGravity {
    ConstantGravity::down_z(gravity::STANDARD_GRAVITY_M_S2)
        .expect("STANDARD_GRAVITY_M_S2 is positive and finite")
}

/// 15-state error-state square-root Unscented Kalman Filter.
///
/// State layout (matching the [`crate::estimator::Ekf`]):
///
/// ```text
/// x_err = [ δposition_eci(3),     // 0..3
///           δvelocity_eci(3),     // 3..6
///           δattitude_axis_ang(3),// 6..9   (multiplicative δq via exp)
///           δgyro_bias_body(3),   // 9..12
///           δaccel_bias_body(3) ] // 12..15
/// ```
///
/// The full attitude is `q_full = q_nominal ⊗ exp(δθ/2)`. After every
/// measurement update the error mean is reset and folded into the
/// nominal state.
///
/// # Algorithm shape
///
/// **Predict.** A single nominal-state propagation through the
/// IMU-driven inertial model (matching the EKF), then a sigma-point
/// covariance propagation in the error-state subspace. We use the
/// linearized error dynamics (`F · S` where `F` is the standard
/// error-state Jacobian) for the predict-side Cholesky combiner; a
/// fully-nonlinear sigma-point propagation of the FULL state through
/// the IMU model converges to this in the small-error limit and is
/// deferred follow-on work (see `docs/roadmap.md`).
///
/// **Measurement update.** Sigma-point form (Van der Merwe & Wan 2001
/// Eq. 19-23): generate sigma points around the current error mean,
/// project each through the measurement function `h(·)`, recombine
/// into the innovation Cholesky factor `S_z` via QR + `cholupdate`,
/// compute the cross-covariance `P_xz`, form the Kalman gain `K`, and
/// apply the rank-`m` `cholupdate(S, K · S_z, Minus)` for each column
/// of `K · S_z`.
///
/// # Determinism
///
/// Same contract as [`crate::estimator::Ekf`] plus the SR-UKF-specific
/// guarantees from this module's primitives: locked sigma-point
/// column ordering, locked Givens-rotation row order, no `mul_add` on
/// the hot path. Tagged `state-stable, not bit-stable` because the
/// `pow()` in the sigma-weight formula and the `exp()` in the
/// quaternion log/exp can drift across platform-libm.
#[allow(clippy::struct_excessive_bools)] // per-sensor `last_*_updated_this_tick` flags mirror `Ekf`.
pub struct SquareRootUkf {
    params: SquareRootUkfParams,
    scaling: UkfScalingParams,
    weights: SigmaWeights,
    /// Whether the high-dynamics process-noise regime is active (scales
    /// position/velocity process noise by `params.high_dynamics_q_scale`).
    high_dynamics_q_active: bool,
    /// Nominal ECI position (m).
    pos_eci: Vector3<f64>,
    /// Nominal ECI velocity (m/s).
    vel_eci: Vector3<f64>,
    /// Nominal body-to-ECI rotation.
    q_body_to_eci: UnitQuaternion<f64>,
    /// Estimated gyro bias in the body frame (rad/s).
    gyro_bias: Vector3<f64>,
    /// Estimated accel bias in the body frame (m/s²).
    accel_bias: Vector3<f64>,
    /// Body-frame angular velocity, debiased, last predict.
    omega_body: Vector3<f64>,
    /// Square-root error-state covariance (lower-triangular Cholesky
    /// factor of `P`, where `P = S · Sᵀ`). 15×15.
    s: DMatrix<f64>,
    last_imu: Option<ImuSample>,
    time_since_corrective_s: f64,
    last_chi2_imu: f64,
    last_chi2_gnss: f64,
    last_chi2_baro: f64,
    last_chi2_mag: f64,
    last_chi2_star_tracker: f64,
    last_innovation_rejected: bool,
    last_gnss_innovation_whitened: [f64; 6],
    last_gnss_updated_this_tick: bool,
    last_baro_innovation_whitened: f64,
    last_baro_updated_this_tick: bool,
    last_mag_innovation_whitened: [f64; 3],
    last_mag_updated_this_tick: bool,
    last_log_det_s_gnss: f64,
    last_log_det_s_baro: f64,
    last_log_det_s_mag: f64,
    initialized: bool,
    gravity: GravityAdapter,
    mag_field: Box<dyn MagneticFieldEci>,
}

impl std::fmt::Debug for SquareRootUkf {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SquareRootUkf")
            .field("params", &self.params)
            .field("scaling", &self.scaling)
            .field("pos_eci", &self.pos_eci)
            .field("vel_eci", &self.vel_eci)
            .field("q_body_to_eci", &self.q_body_to_eci)
            .field("gyro_bias", &self.gyro_bias)
            .field("accel_bias", &self.accel_bias)
            .field("initialized", &self.initialized)
            .field("time_since_corrective_s", &self.time_since_corrective_s)
            .finish_non_exhaustive()
    }
}

impl SquareRootUkf {
    /// Construct a freshly-initialised SR-UKF anchored at the origin
    /// with identity attitude, zero biases, the default flat-Earth
    /// gravity model, and the academic-tier dipole magnetic-field
    /// model.
    #[must_use]
    pub fn new(params: SquareRootUkfParams) -> Self {
        Self::new_with_scaling(params, UkfScalingParams::default())
    }

    /// Construct with explicit Wan-Van der Merwe scaling parameters.
    ///
    /// # Panics
    ///
    /// The unreachable `expect` on the diagonal-positive Cholesky
    /// factorisation guards against a hypothetical `nalgebra` bug;
    /// it cannot fire on the constructor's hard-coded positive
    /// diagonal.
    #[must_use]
    #[allow(clippy::expect_used, clippy::cast_precision_loss)] // small known constant SRUKF_STATE_DIM = 15 is exact in f64.
    pub fn new_with_scaling(params: SquareRootUkfParams, scaling: UkfScalingParams) -> Self {
        let n = SRUKF_STATE_DIM as f64;
        let weights = SigmaWeights::for_dimension(n, scaling);
        // Initial covariance — generous, matches Ekf::new defaults.
        // Stored as Cholesky factor: S_init = √diag(...) since P_init
        // is diagonal.
        let mut p = DMatrix::<f64>::zeros(SRUKF_STATE_DIM, SRUKF_STATE_DIM);
        for i in 0..3 {
            p[(i, i)] = 100.0; // position
            p[(i + 3, i + 3)] = 10.0; // velocity
            p[(i + 6, i + 6)] = 0.1; // attitude error
            p[(i + 9, i + 9)] = 0.001; // gyro bias
            p[(i + 12, i + 12)] = 0.01; // accel bias
        }
        let s =
            cholesky_from_covariance(&p).expect("diagonal positive covariance must factor cleanly");
        Self {
            params,
            scaling,
            weights,
            high_dynamics_q_active: false,
            pos_eci: Vector3::zeros(),
            vel_eci: Vector3::zeros(),
            q_body_to_eci: UnitQuaternion::identity(),
            gyro_bias: Vector3::zeros(),
            accel_bias: Vector3::zeros(),
            omega_body: Vector3::zeros(),
            s,
            last_imu: None,
            time_since_corrective_s: 0.0,
            last_chi2_imu: 0.0,
            last_chi2_gnss: 0.0,
            last_chi2_baro: 0.0,
            last_chi2_mag: 0.0,
            last_chi2_star_tracker: 0.0,
            last_innovation_rejected: false,
            last_gnss_innovation_whitened: [0.0; 6],
            last_gnss_updated_this_tick: false,
            last_baro_innovation_whitened: 0.0,
            last_baro_updated_this_tick: false,
            last_mag_innovation_whitened: [0.0; 3],
            last_mag_updated_this_tick: false,
            last_log_det_s_gnss: f64::NAN,
            last_log_det_s_baro: f64::NAN,
            last_log_det_s_mag: f64::NAN,
            initialized: false,
            gravity: GravityAdapter::new(default_constant_gravity_down_z()),
            mag_field: Box::new(EarthDipoleField::default()),
        }
    }

    /// Replace the gravity model.
    #[must_use]
    pub fn with_gravity_model<G: GravityModel + Send + Sync + 'static>(
        mut self,
        gravity: G,
    ) -> Self {
        self.gravity = GravityAdapter::new(gravity);
        self
    }

    /// Replace the magnetic-field model.
    #[must_use]
    pub fn with_mag_field_model<M: MagneticFieldEci + 'static>(mut self, mag: M) -> Self {
        self.mag_field = Box::new(mag);
        self
    }

    /// Seed the filter with a known initial pose and velocity.
    pub fn seed(
        &mut self,
        pos_eci: Vector3<f64>,
        vel_eci: Vector3<f64>,
        q_body_to_eci: UnitQuaternion<f64>,
    ) {
        self.pos_eci = pos_eci;
        self.vel_eci = vel_eci;
        self.q_body_to_eci = q_body_to_eci;
        renormalize_quaternion(&mut self.q_body_to_eci);
        self.initialized = true;
    }

    /// Maximum diagonal entry of the recovered covariance — useful for
    /// telemetry / health checks.
    #[must_use]
    pub fn covariance_max_diag(&self) -> f64 {
        let p = covariance_from_cholesky(&self.s);
        (0..SRUKF_STATE_DIM).map(|i| p[(i, i)]).fold(0.0, f64::max)
    }

    fn covariance_diagnostics(&self) -> ([f64; 3], [f64; 3], [f64; 3], f64, f64, bool) {
        let p = covariance_from_cholesky(&self.s);
        let position_variance_eci_m2 = covariance_diag3(&p, 0);
        let velocity_variance_eci_m2_s2 = covariance_diag3(&p, 3);
        let attitude_variance_rad2 = covariance_diag3(&p, 6);
        let attitude_variance_max_rad2 = attitude_variance_rad2.iter().copied().fold(0.0, f64::max);
        let condition_proxy = covariance_diag_condition(&p);
        (
            position_variance_eci_m2,
            velocity_variance_eci_m2_s2,
            attitude_variance_rad2,
            attitude_variance_max_rad2,
            condition_proxy,
            !attitude_variance_max_rad2.is_finite()
                || attitude_variance_max_rad2 > ATTITUDE_UNDER_OBSERVABLE_VARIANCE_RAD2,
        )
    }

    /// Apply an error-state delta `δx` to the nominal state. Same
    /// semantics as `crate::estimator::Ekf::apply_state_update`.
    fn apply_state_update(&mut self, dx: &SVector<f64, SRUKF_STATE_DIM>) {
        self.pos_eci += Vector3::new(dx[0], dx[1], dx[2]);
        self.vel_eci += Vector3::new(dx[3], dx[4], dx[5]);
        let attitude_error = Vector3::new(dx[6], dx[7], dx[8]);
        if attitude_error.norm() > 0.0 {
            let dq = quaternion_from_axis_angle(attitude_error);
            self.q_body_to_eci *= dq;
            renormalize_quaternion(&mut self.q_body_to_eci);
        }
        self.gyro_bias += Vector3::new(dx[9], dx[10], dx[11]);
        self.accel_bias += Vector3::new(dx[12], dx[13], dx[14]);
    }

    /// Predict-side covariance transition `F` (15×15) for a step of
    /// length `dt`.
    ///
    /// This MIRRORS the validated, flight-proven EKF covariance
    /// propagation (`crate::estimator::Ekf::predict`), which is
    /// deliberately DECOUPLED: the transition is the identity except for
    /// first-order Gauss-Markov decay on the two bias blocks. The
    /// inter-state error couplings (δṗ←δv, δv̇←δθ and δv̇←δaccel_bias,
    /// δθ̇←δgyro_bias) are NOT propagated into the covariance. Attitude is
    /// therefore carried by the full, nonlinear NOMINAL gyro integration
    /// (done in `predict`) and corrected by the magnetometer /
    /// star-tracker — it is never destabilised by GNSS cross-covariance
    /// feedback.
    ///
    /// The fully-coupled `F·S` propagation an "ideal" UKF would use is
    /// numerically fragile under the high specific force of powered ascent
    /// (2-3 g): it inflates the attitude↔gyro-bias cross-covariance, and
    /// the resulting GNSS-velocity-driven attitude corrections diverge
    /// (the vehicle loses attitude and never reaches orbit). Matching the
    /// EKF's decoupled structure is the same design choice the EKF already
    /// makes; the SR-UKF stays distinct in its square-root numerics and
    /// sigma-point measurement updates. Hardening the fully-coupled
    /// propagation is deferred (see crate docs).
    fn error_state_jacobian(&self, dt: f64) -> DMatrix<f64> {
        let mut f = DMatrix::<f64>::identity(SRUKF_STATE_DIM, SRUKF_STATE_DIM);
        // Gauss-Markov decay on the bias blocks — the only non-identity
        // term, matching `Ekf::predict`'s `scale_covariance_state` on
        // states 9..12 (gyro bias) and 12..15 (accel bias).
        let gyro_decay = gauss_markov_decay_local(dt, self.params.tau_gyro_bias_s);
        let accel_decay = gauss_markov_decay_local(dt, self.params.tau_accel_bias_s);
        for i in 0..3 {
            f[(i + 9, i + 9)] = gyro_decay;
            f[(i + 12, i + 12)] = accel_decay;
        }
        f
    }

    /// Process-noise Cholesky factor (`√Q`) for a step of length `dt`.
    /// Diagonal: nonzero on the attitude and bias blocks. Position and
    /// velocity inherit noise through the F·S coupling rather than a
    /// direct Q contribution (matching the EKF Q-diagonal layout).
    fn process_noise_cholesky(&self, dt: f64) -> DMatrix<f64> {
        let mut q_sqrt = DMatrix::<f64>::zeros(SRUKF_STATE_DIM, SRUKF_STATE_DIM);
        // Position / velocity process noise. q_sqrt is the Cholesky factor
        // of the (diagonal) process-noise covariance Q, so each entry is
        // √(σ² · scale · dt) = σ · √(scale · dt) — matching the EKF, which
        // adds σ² · scale · dt to its covariance diagonal. Without these
        // terms the position/velocity covariance collapses and the filter
        // diverges under ascent dynamics (see `sigma_w_velocity_m_s`).
        let position_velocity_q_scale = if self.high_dynamics_q_active
            && self.params.high_dynamics_q_scale.is_finite()
            && self.params.high_dynamics_q_scale > 0.0
        {
            self.params.high_dynamics_q_scale
        } else {
            1.0
        };
        let pv_root = (position_velocity_q_scale * dt).sqrt();
        let s_pos = self.params.sigma_w_position_m * pv_root;
        let s_vel = self.params.sigma_w_velocity_m_s * pv_root;
        for i in 0..3 {
            q_sqrt[(i, i)] = s_pos;
            q_sqrt[(i + 3, i + 3)] = s_vel;
        }
        // δθ block: σ_w_gyro · √dt
        let s_att = self.params.sigma_w_gyro * dt.sqrt();
        for i in 0..3 {
            q_sqrt[(i + 6, i + 6)] = s_att;
        }
        // gyro bias block.
        let var_gyro_bias = gauss_markov_process_variance_local(
            self.params.sigma_w_gyro_bias,
            dt,
            self.params.tau_gyro_bias_s,
        );
        for i in 0..3 {
            q_sqrt[(i + 9, i + 9)] = var_gyro_bias.sqrt();
        }
        // accel bias block.
        let var_accel_bias = gauss_markov_process_variance_local(
            self.params.sigma_w_accel_bias,
            dt,
            self.params.tau_accel_bias_s,
        );
        for i in 0..3 {
            q_sqrt[(i + 12, i + 12)] = var_accel_bias.sqrt();
        }
        q_sqrt
    }

    /// Generic measurement-update step: sigma-point form with
    /// `cholupdate`-based covariance reduction.
    ///
    /// `predict_measurement` is called on each sigma point's state
    /// to produce the predicted measurement vector. `r_sqrt` is the
    /// Cholesky factor of the measurement-noise covariance.
    /// Returns the chi-square statistic for innovation gating.
    #[allow(
        clippy::too_many_arguments,
        clippy::too_many_lines,
        clippy::many_single_char_names, // standard Kalman naming: r, m, n, k, s, z
    )]
    fn sigma_point_update<F>(
        &mut self,
        z: &DVector<f64>,
        r_sqrt: &DMatrix<f64>,
        predict_measurement: F,
        sensor_label: &'static str,
        gate_dof: f64,
    ) -> Result<(f64, DVector<f64>, f64), EstimatorError>
    where
        F: Fn(&SquareRootUkf, &DVector<f64>) -> DVector<f64>,
    {
        let m = z.len();
        let n = SRUKF_STATE_DIM;
        // 1. Sigma points around current error mean = 0.
        let zero_mean = DVector::<f64>::zeros(n);
        let chi = sigma_points(&zero_mean, &self.s, self.weights);
        // 2. Project each sigma point through h(): build perturbed
        //    nominal state per column, evaluate the measurement.
        let total_cols = 2 * n + 1;
        let mut zeta = DMatrix::<f64>::zeros(m, total_cols);
        for col in 0..total_cols {
            let perturbation: DVector<f64> = chi.column(col).into_owned();
            let z_pred = predict_measurement(self, &perturbation);
            for r in 0..m {
                zeta[(r, col)] = z_pred[r];
            }
        }
        // 3. Weighted mean of zeta.
        let mut z_hat = DVector::<f64>::zeros(m);
        for r in 0..m {
            z_hat[r] = self.weights.w0_mean * zeta[(r, 0)];
        }
        for col in 1..total_cols {
            for r in 0..m {
                z_hat[r] += self.weights.wi * zeta[(r, col)];
            }
        }
        // 4. Innovation Cholesky S_z via QR(weighted residuals + R_sqrt)
        //    + cholupdate(W_c^0). Off-centre weights wi are positive,
        //    so √wi is real.
        let mut residual_stack = DMatrix::<f64>::zeros(m, 2 * n + m);
        let sqrt_wi = self.weights.wi.sqrt();
        for col in 1..total_cols {
            for r in 0..m {
                residual_stack[(r, col - 1)] = sqrt_wi * (zeta[(r, col)] - z_hat[r]);
            }
        }
        // Append R_sqrt columns.
        for c in 0..m {
            for r in 0..m {
                residual_stack[(r, 2 * n + c)] = r_sqrt[(r, c)];
            }
        }
        let mut s_z =
            predict_cholesky_qr(&residual_stack).map_err(|e| EstimatorError::InvalidConfig {
                reason: format!(
                    "{sensor_label}: predict-side QR rejected non-finite residuals: {e:?}"
                ),
            })?;
        // Fold central residual via cholupdate(W_c^0).
        let mut central_residual: DVector<f64> = DVector::<f64>::zeros(m);
        for r in 0..m {
            central_residual[r] = (zeta[(r, 0)] - z_hat[r]) * self.weights.w0_cov.abs().sqrt();
        }
        let central_sign = if self.weights.w0_cov >= 0.0 {
            CholupdateSign::Plus
        } else {
            CholupdateSign::Minus
        };
        cholupdate_in_place(&mut s_z, &mut central_residual, central_sign).map_err(|e| {
            EstimatorError::InvalidConfig {
                reason: format!("{sensor_label}: central-residual cholupdate rejected: {e:?}"),
            }
        })?;
        // 5. Cross-covariance P_xz = Σ W_c^i · (χ_i) · (ζ_i − ẑ)ᵀ (the
        //    χ_i are already centred since the error mean is zero).
        let mut p_xz = DMatrix::<f64>::zeros(n, m);
        // Central sigma point contribution (uses W_c^0; χ_0 is zero so
        // contributes nothing — kept for symmetry / clarity).
        for r in 0..n {
            for c in 0..m {
                p_xz[(r, c)] += self.weights.w0_cov * chi[(r, 0)] * (zeta[(c, 0)] - z_hat[c]);
            }
        }
        for col in 1..total_cols {
            for r in 0..n {
                for c in 0..m {
                    p_xz[(r, c)] += self.weights.wi * chi[(r, col)] * (zeta[(c, col)] - z_hat[c]);
                }
            }
        }
        // 6. Kalman gain K = P_xz · (S_z · S_zᵀ)⁻¹ via two triangular
        //    right-solves. Equivalently: K = (P_xz / S_zᵀ) / S_z.
        let mut y = DMatrix::<f64>::zeros(n, m);
        for row in 0..n {
            let rhs = row_as_dvector(&p_xz, row);
            let solved =
                solve_lower_triangular(&s_z, &rhs).map_err(|e| EstimatorError::InvalidConfig {
                    reason: format!("{sensor_label}: lower gain solve failed: {e:?}"),
                })?;
            for c in 0..m {
                y[(row, c)] = solved[c];
            }
        }
        let mut k = DMatrix::<f64>::zeros(n, m);
        for row in 0..n {
            let rhs = row_as_dvector(&y, row);
            let solved = solve_lower_transpose_triangular(&s_z, &rhs).map_err(|e| {
                EstimatorError::InvalidConfig {
                    reason: format!("{sensor_label}: upper gain solve failed: {e:?}"),
                }
            })?;
            for c in 0..m {
                k[(row, c)] = solved[c];
            }
        }
        // 7. Innovation, chi² gate, whitened innovation, log det S.
        let innovation = z - &z_hat;
        let mut log_det_s = 0.0_f64;
        for i in 0..m {
            log_det_s += s_z[(i, i)].ln();
        }
        log_det_s *= 2.0;
        // Whitened innovation: ν̃ = S_z⁻¹ · ν via lower-triangular solve.
        let whitened = solve_lower_triangular(&s_z, &innovation).map_err(|e| {
            EstimatorError::InvalidConfig {
                reason: format!("{sensor_label}: lower-triangular solve failed: {e:?}"),
            }
        })?;
        let chi2 = whitened.dot(&whitened);
        let gate = self.params.gate_for_dof(gate_dof);
        if chi2 > gate {
            self.last_innovation_rejected = true;
            return Err(EstimatorError::InnovationGateRejected {
                measurement: sensor_label,
                chi2,
                gate,
            });
        }
        // 8. State update + covariance downdate.
        let dx = &k * &innovation;
        let mut dx_static = SVector::<f64, SRUKF_STATE_DIM>::zeros();
        for i in 0..SRUKF_STATE_DIM {
            dx_static[i] = dx[i];
        }
        self.apply_state_update(&dx_static);
        // S update: U = K · S_z;  for each column u_i of U:
        //   cholupdate(S, u_i, Minus)
        let u = &k * &s_z;
        for col in 0..m {
            let mut u_col: DVector<f64> = u.column(col).into_owned();
            cholupdate_in_place(&mut self.s, &mut u_col, CholupdateSign::Minus).map_err(|e| {
                EstimatorError::InvalidConfig {
                    reason: format!(
                        "{sensor_label}: covariance downdate rejected (col {col}): {e:?}"
                    ),
                }
            })?;
        }
        self.last_innovation_rejected = false;
        Ok((chi2, whitened, log_det_s))
    }
}

// --- standalone helpers (private to this module) ---

fn gauss_markov_decay_local(dt_s: f64, tau_s: f64) -> f64 {
    if tau_s.is_finite() && tau_s > 0.0 {
        (-dt_s / tau_s).exp()
    } else {
        1.0
    }
}

fn gauss_markov_process_variance_local(sigma: f64, dt_s: f64, tau_s: f64) -> f64 {
    if tau_s.is_finite() && tau_s > 0.0 {
        0.5 * sigma * sigma * tau_s * (1.0 - (-2.0 * dt_s / tau_s).exp())
    } else {
        sigma * sigma * dt_s
    }
}

/// Solve `L · x = b` for a lower-triangular `L`. Returns `Err` if the
/// diagonal contains a zero (singular system).
fn solve_lower_triangular(
    l: &DMatrix<f64>,
    b: &DVector<f64>,
) -> Result<DVector<f64>, &'static str> {
    let n = l.nrows();
    if n != b.len() || l.ncols() != n {
        return Err("dimension mismatch");
    }
    let mut x = DVector::<f64>::zeros(n);
    for i in 0..n {
        let mut sum = 0.0_f64;
        for j in 0..i {
            sum += l[(i, j)] * x[j];
        }
        let diag = l[(i, i)];
        if !diag.is_finite() || diag == 0.0 {
            return Err("singular triangular factor");
        }
        x[i] = (b[i] - sum) / diag;
    }
    Ok(x)
}

fn solve_lower_transpose_triangular(
    lower: &DMatrix<f64>,
    rhs: &DVector<f64>,
) -> Result<DVector<f64>, &'static str> {
    let dimension = lower.nrows();
    if dimension != rhs.len() || lower.ncols() != dimension {
        return Err("dimension mismatch");
    }
    let mut solution = DVector::<f64>::zeros(dimension);
    for reverse_index in 0..dimension {
        let row = dimension - 1 - reverse_index;
        let mut sum = 0.0_f64;
        for col in (row + 1)..dimension {
            sum += lower[(col, row)] * solution[col];
        }
        let diag = lower[(row, row)];
        if !diag.is_finite() || diag == 0.0 {
            return Err("singular triangular factor");
        }
        solution[row] = (rhs[row] - sum) / diag;
    }
    Ok(solution)
}

fn row_as_dvector(m: &DMatrix<f64>, row: usize) -> DVector<f64> {
    let mut out = DVector::<f64>::zeros(m.ncols());
    for c in 0..m.ncols() {
        out[c] = m[(row, c)];
    }
    out
}

impl crate::estimator::Estimator for SquareRootUkf {
    fn name(&self) -> &'static str {
        "estimator.sr_ukf"
    }

    fn set_high_dynamics_process_noise(&mut self, active: bool) {
        self.high_dynamics_q_active = active;
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        if !dt.is_finite() || dt <= 0.0 {
            return Ok(());
        }
        let Some(imu) = self.last_imu else {
            return Ok(());
        };

        // 1. Apply Gauss-Markov decay to the nominal biases.
        let gyro_decay = gauss_markov_decay_local(dt, self.params.tau_gyro_bias_s);
        let accel_decay = gauss_markov_decay_local(dt, self.params.tau_accel_bias_s);
        self.gyro_bias *= gyro_decay;
        self.accel_bias *= accel_decay;

        // 2. Nominal-state propagation (mirrors EKF).
        let omega_meas = imu.gyro_rad_s - self.gyro_bias;
        let accel_meas = imu.accel_m_s2 - self.accel_bias;
        self.omega_body = omega_meas;
        let dq = quaternion_from_omega(omega_meas, dt);
        self.q_body_to_eci *= dq;
        renormalize_quaternion(&mut self.q_body_to_eci);
        let r_body_to_eci = self.q_body_to_eci.to_rotation_matrix();
        let f_eci = r_body_to_eci * accel_meas;
        let g_eci = self.gravity.at(self.pos_eci, SimTime::ZERO)?;
        let a_eci = f_eci + g_eci;
        self.vel_eci += a_eci * dt;
        self.pos_eci += self.vel_eci * dt;

        // 3. Square-root error-state covariance propagation.
        //    S_new = QR(F · S || √Q).R[:n, :n]ᵀ  (linearized predict).
        //    The fully-nonlinear sigma-point predict converges to this
        //    in the small-error limit and is tracked as a follow-on
        //    refinement (see crate-level docs).
        let f = self.error_state_jacobian(dt);
        let f_s = &f * &self.s;
        let q_sqrt = self.process_noise_cholesky(dt);
        let n = SRUKF_STATE_DIM;
        let mut stack = DMatrix::<f64>::zeros(n, 2 * n);
        for col in 0..n {
            for row in 0..n {
                stack[(row, col)] = f_s[(row, col)];
            }
        }
        for col in 0..n {
            for row in 0..n {
                stack[(row, n + col)] = q_sqrt[(row, col)];
            }
        }
        self.s = predict_cholesky_qr(&stack).map_err(|e| EstimatorError::InvalidConfig {
            reason: format!("predict QR rejected non-finite stack: {e:?}"),
        })?;

        self.time_since_corrective_s += dt;
        if !self.pos_eci.iter().all(|v| v.is_finite())
            || !self.vel_eci.iter().all(|v| v.is_finite())
        {
            return Err(EstimatorError::NonFiniteState { stage: "predict" });
        }
        Ok(())
    }

    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError> {
        self.last_imu = Some(*sample);
        self.last_chi2_imu = 0.0;
        Ok(())
    }

    fn update_gnss(&mut self, sample: &GnssSample) -> Result<(), EstimatorError> {
        // GNSS is fused as two INDEPENDENT, independently-gated 3-D blocks —
        // position (states 0..3) then velocity (states 3..6) — processed
        // sequentially, mirroring `crate::estimator::Ekf::update_gnss`. The
        // GNSS measurement-noise R is block-diagonal and the measurements are
        // linear in the state, so sequential processing is mathematically
        // identical to the joint 6-D update when BOTH blocks pass their gate;
        // the difference is that each block is gated on its own 3-DOF
        // chi-square. A velocity-innovation spike under high thrust therefore
        // rejects ONLY the velocity block — the good position fix is still
        // applied — instead of the previous single combined 6-D gate dropping
        // the whole fix and dead-reckoning the filter into divergence. (This
        // was the SR-UKF's full-ascent divergence: a 6-D gate that collapsed
        // under 2-3 g thrust.)
        self.last_gnss_updated_this_tick = true;

        // --- position block (states 0..3) ---
        let z_pos = DVector::<f64>::from_iterator(3, (0..3).map(|i| sample.position_eci_m[i]));
        let mut r_sqrt_pos = DMatrix::<f64>::zeros(3, 3);
        for i in 0..3 {
            r_sqrt_pos[(i, i)] = self.params.sigma_gnss_pos_m;
        }
        let predict_pos = |filter: &SquareRootUkf, perturbation: &DVector<f64>| -> DVector<f64> {
            let pos =
                filter.pos_eci + Vector3::new(perturbation[0], perturbation[1], perturbation[2]);
            DVector::<f64>::from_iterator(3, (0..3).map(|i| pos[i]))
        };
        let pos_result = self.sigma_point_update(&z_pos, &r_sqrt_pos, predict_pos, "gnss", 3.0);

        // --- velocity block (states 3..6), evaluated AFTER the position
        //     update so the two form a proper sequential (joint-equivalent)
        //     pass ---
        let z_vel = DVector::<f64>::from_iterator(3, (0..3).map(|i| sample.velocity_eci_m_s[i]));
        let mut r_sqrt_vel = DMatrix::<f64>::zeros(3, 3);
        for i in 0..3 {
            r_sqrt_vel[(i, i)] = self.params.sigma_gnss_vel_m_s;
        }
        let predict_vel = |filter: &SquareRootUkf, perturbation: &DVector<f64>| -> DVector<f64> {
            let vel =
                filter.vel_eci + Vector3::new(perturbation[3], perturbation[4], perturbation[5]);
            DVector::<f64>::from_iterator(3, (0..3).map(|i| vel[i]))
        };
        let vel_result = self.sigma_point_update(&z_vel, &r_sqrt_vel, predict_vel, "gnss", 3.0);

        // Combine the two blocks' diagnostics. A gate rejection is recoverable
        // (the block simply isn't applied); any other error is a hard numeric
        // failure and propagates. The whitened innovation telemetry is the
        // concatenation of the two blocks and the chi-square / log-det are
        // their sums, so the `‖ν̃‖² = chi2` invariant still holds.
        let mut chi2_total = 0.0_f64;
        let mut log_det_total = 0.0_f64;
        let applied_pos = match pos_result {
            Ok((chi2, whitened, log_det)) => {
                self.last_gnss_innovation_whitened[..3].copy_from_slice(whitened.as_slice());
                chi2_total += chi2;
                log_det_total += log_det;
                true
            }
            Err(EstimatorError::InnovationGateRejected { chi2, .. }) => {
                self.last_gnss_innovation_whitened[..3].fill(0.0);
                chi2_total += chi2;
                false
            }
            Err(other) => return Err(other),
        };
        let applied_vel = match vel_result {
            Ok((chi2, whitened, log_det)) => {
                self.last_gnss_innovation_whitened[3..].copy_from_slice(whitened.as_slice());
                chi2_total += chi2;
                log_det_total += log_det;
                true
            }
            Err(EstimatorError::InnovationGateRejected { chi2, .. }) => {
                self.last_gnss_innovation_whitened[3..].fill(0.0);
                chi2_total += chi2;
                false
            }
            Err(other) => return Err(other),
        };

        self.last_chi2_gnss = chi2_total;
        self.last_log_det_s_gnss = log_det_total;

        if applied_pos || applied_vel {
            self.time_since_corrective_s = 0.0;
            self.initialized = true;
        }
        if !applied_pos || !applied_vel {
            self.last_innovation_rejected = true;
            // Report the combined statistic against the per-block 3-DOF gate
            // (a rejection means at least one block breached its own gate).
            return Err(EstimatorError::InnovationGateRejected {
                measurement: "gnss",
                chi2: self.last_chi2_gnss,
                gate: self.params.gate_for_dof(3.0),
            });
        }
        self.last_innovation_rejected = false;
        Ok(())
    }

    fn update_baro(&mut self, sample: &BarometerSample) -> Result<(), EstimatorError> {
        let measured_alt_m = openbmp_physics::atmosphere::pressure_altitude_troposphere_m(
            sample.pressure_pa - sample.bias_pa,
        );
        let z = DVector::<f64>::from_row_slice(&[measured_alt_m]);
        let mut r_sqrt = DMatrix::<f64>::zeros(1, 1);
        r_sqrt[(0, 0)] = self.params.sigma_baro_alt_m;
        let predict_z = |filter: &SquareRootUkf, perturbation: &DVector<f64>| -> DVector<f64> {
            DVector::<f64>::from_row_slice(&[filter.pos_eci.z + perturbation[2]])
        };
        let (chi2, whitened, log_det_s) =
            self.sigma_point_update(&z, &r_sqrt, predict_z, "baro", 1.0)?;
        self.last_chi2_baro = chi2;
        self.last_baro_innovation_whitened = whitened[0];
        self.last_baro_updated_this_tick = true;
        self.last_log_det_s_baro = log_det_s;
        Ok(())
    }

    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError> {
        if !sample.healthy {
            return Ok(());
        }
        let measured = sample.field_body_nt - sample.hard_iron_body_nt;
        let sample_time = sample.time;
        let z = DVector::<f64>::from_iterator(3, (0..3).map(|i| measured[i]));
        let mut r_sqrt = DMatrix::<f64>::zeros(3, 3);
        for i in 0..3 {
            r_sqrt[(i, i)] = self.params.sigma_mag_nt;
        }
        let mag_field_eci = self.mag_field.field_eci_nt(self.pos_eci, sample_time);
        let predict_z = |filter: &SquareRootUkf, perturbation: &DVector<f64>| -> DVector<f64> {
            // The field model is intentionally evaluated once at the
            // nominal position. Over the sigma spread used for attitude
            // updates, field variation with position is negligible next
            // to the attitude projection being estimated here.
            let attitude_error = Vector3::new(perturbation[6], perturbation[7], perturbation[8]);
            let dq = if attitude_error.norm() > 0.0 {
                quaternion_from_axis_angle(attitude_error)
            } else {
                UnitQuaternion::identity()
            };
            let q_perturbed = filter.q_body_to_eci * dq;
            let r_eci_to_body = q_perturbed.to_rotation_matrix().transpose();
            let predicted = r_eci_to_body.matrix() * mag_field_eci;
            DVector::<f64>::from_iterator(3, (0..3).map(|i| predicted[i]))
        };
        let (chi2, whitened, log_det_s) =
            self.sigma_point_update(&z, &r_sqrt, predict_z, "mag", 3.0)?;
        self.last_chi2_mag = chi2;
        for i in 0..3 {
            self.last_mag_innovation_whitened[i] = whitened[i];
        }
        self.last_mag_updated_this_tick = true;
        self.last_log_det_s_mag = log_det_s;
        Ok(())
    }

    fn update_star_tracker(&mut self, sample: &StarTrackerSample) -> Result<(), EstimatorError> {
        if !sample.healthy {
            return Ok(());
        }
        // The star tracker delivers the full attitude as q_eci_to_body. Form
        // the body-frame small-angle error between the measured and nominal
        // attitude; this is a direct, LINEAR observation of the attitude-error
        // state (indices 6..9), exactly as in `Ekf::update_star_tracker`. For
        // a measurement that is linear in the error state, the sigma-point
        // update reduces to the EKF's H = I-on-attitude update: the predicted
        // measurement for a sigma point is simply its attitude-error
        // perturbation, and the innovation is the observed error (the error
        // mean is zero, so ẑ = 0). Being a full 3-DOF fix, this constrains
        // EVERY attitude axis — what a single magnetometer vector cannot —
        // and is what keeps the attitude estimate from drifting during ascent.
        let [mx, my, mz, mw] = sample.q_eci_to_body_xyzw;
        // q_body_to_eci(measured) = inverse(q_eci_to_body) = conjugate for a
        // unit quaternion: (x, y, z, w) -> (-x, -y, -z, w).
        let q_be_meas_xyzw = [-mx, -my, -mz, mw];
        let q_nom = self.q_body_to_eci;
        let innovation =
            quaternion_error_small_angle([q_nom.i, q_nom.j, q_nom.k, q_nom.w], q_be_meas_xyzw);
        let z = DVector::<f64>::from_iterator(3, (0..3).map(|i| innovation[i]));
        let mut r_sqrt = DMatrix::<f64>::zeros(3, 3);
        for i in 0..3 {
            r_sqrt[(i, i)] = self.params.sigma_star_tracker_rad;
        }
        let predict_z = |_filter: &SquareRootUkf, perturbation: &DVector<f64>| -> DVector<f64> {
            DVector::<f64>::from_iterator(3, (0..3).map(|i| perturbation[i + 6]))
        };
        let (chi2, _whitened, _log_det_s) =
            self.sigma_point_update(&z, &r_sqrt, predict_z, "star_tracker", 3.0)?;
        self.last_chi2_star_tracker = chi2;
        self.time_since_corrective_s = 0.0;
        self.initialized = true;
        Ok(())
    }

    fn attitude(&self) -> AttitudeEstimate {
        let q = self.q_body_to_eci.into_inner();
        AttitudeEstimate {
            time: SimTime::ZERO,
            q_body_to_eci_xyzw: [q.i, q.j, q.k, q.w],
            omega_body_rad_s: self.omega_body,
            gyro_bias_body_rad_s: self.gyro_bias,
        }
    }

    fn position(&self) -> PositionEstimate {
        PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: self.pos_eci,
            velocity_eci_m_s: self.vel_eci,
            accel_bias_body_m_s2: self.accel_bias,
        }
    }

    fn status(&self) -> EstimatorStatus {
        let (
            position_variance_eci_m2,
            velocity_variance_eci_m2_s2,
            attitude_variance_rad2,
            attitude_variance_max_rad2,
            covariance_condition_proxy,
            attitude_under_observable,
        ) = self.covariance_diagnostics();
        EstimatorStatus {
            time: SimTime::ZERO,
            initialized: self.initialized,
            dead_reckoning: self.time_since_corrective_s > self.params.dead_reckon_timeout_s,
            imu_chi2: self.last_chi2_imu,
            gnss_chi2: self.last_chi2_gnss,
            baro_chi2: self.last_chi2_baro,
            mag_chi2: self.last_chi2_mag,
            star_tracker_chi2: self.last_chi2_star_tracker,
            innovation_rejected: self.last_innovation_rejected,
            gnss_innovation_whitened: self.last_gnss_innovation_whitened,
            gnss_updated_this_tick: self.last_gnss_updated_this_tick,
            baro_innovation_whitened: self.last_baro_innovation_whitened,
            baro_updated_this_tick: self.last_baro_updated_this_tick,
            mag_innovation_whitened: self.last_mag_innovation_whitened,
            mag_updated_this_tick: self.last_mag_updated_this_tick,
            attitude_variance_max_rad2,
            position_variance_eci_m2,
            velocity_variance_eci_m2_s2,
            attitude_variance_rad2,
            covariance_condition_proxy,
            attitude_under_observable,
        }
    }

    fn estimator_mode(&self) -> Option<EstimatorMode> {
        None
    }

    fn begin_tick(&mut self) {
        self.last_chi2_imu = 0.0;
        self.last_chi2_gnss = 0.0;
        self.last_chi2_baro = 0.0;
        self.last_chi2_mag = 0.0;
        self.last_chi2_star_tracker = 0.0;
        self.last_innovation_rejected = false;
        self.last_gnss_innovation_whitened = [0.0; 6];
        self.last_gnss_updated_this_tick = false;
        self.last_baro_innovation_whitened = 0.0;
        self.last_baro_updated_this_tick = false;
        self.last_mag_innovation_whitened = [0.0; 3];
        self.last_mag_updated_this_tick = false;
        self.last_log_det_s_gnss = f64::NAN;
        self.last_log_det_s_baro = f64::NAN;
        self.last_log_det_s_mag = f64::NAN;
    }
}

// =====================================================================
// `SquareRootUkfAttitude` (6-state attitude variant)
// =====================================================================

/// 6-state attitude-only square-root UKF for consumers that only need
/// attitude + gyro-bias estimation. Supersedes an earlier
/// classical 6-state `Ukf`.
///
/// State layout:
///
/// ```text
/// x_err = [ δattitude_axis_ang(3),// 0..3
///           δgyro_bias_body(3) ]  // 3..6
/// ```
///
/// Internally implemented as a wrapper around [`SquareRootUkf`] that
/// projects the 15-state error-state down to the 6-state attitude
/// subspace for the magnetometer-only measurement update path. The
/// position / velocity / accel-bias state slots are held at zero,
/// re-pinned to tiny covariance floors after covariance-changing
/// operations by rebuilding the projected Cholesky factor, and not
/// reported; only the attitude + gyro-bias slots are exposed.
///
/// This implementation prioritises code reuse over maximum efficiency
/// — the 15-state internals carry trivial overhead for the unused
/// slots.
pub struct SquareRootUkfAttitude {
    inner: SquareRootUkf,
}

impl std::fmt::Debug for SquareRootUkfAttitude {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SquareRootUkfAttitude")
            .field("inner", &self.inner)
            .finish_non_exhaustive()
    }
}

impl SquareRootUkfAttitude {
    /// Construct a freshly-initialised attitude-only SR-UKF. Sets the
    /// position / velocity / accel-bias variances to a tiny floor so
    /// the underlying 15-state filter remains numerically stable
    /// without taking actual position / velocity / accel-bias
    /// updates.
    ///
    /// # Panics
    ///
    /// Same defensive `expect` as [`SquareRootUkf::new_with_scaling`]
    /// — the hard-coded diagonal-positive covariance always factors.
    #[must_use]
    #[allow(clippy::expect_used)]
    pub fn new(params: SquareRootUkfParams) -> Self {
        let mut inner = SquareRootUkf::new(params);
        pin_attitude_only_subspace(&mut inner)
            .expect("attitude-only covariance projection must factor cleanly");
        Self { inner }
    }

    /// Replace the magnetic-field model.
    #[must_use]
    pub fn with_mag_field_model<M: MagneticFieldEci + 'static>(mut self, mag: M) -> Self {
        self.inner = self.inner.with_mag_field_model(mag);
        self
    }

    /// Seed with a known initial attitude.
    pub fn seed(&mut self, q_body_to_eci: UnitQuaternion<f64>) {
        self.inner
            .seed(self.inner.pos_eci, self.inner.vel_eci, q_body_to_eci);
    }

    fn pin_unused_subspace(&mut self) -> Result<(), EstimatorError> {
        pin_attitude_only_subspace(&mut self.inner).map_err(|e| EstimatorError::InvalidConfig {
            reason: format!("attitude-only covariance projection failed: {e:?}"),
        })
    }
}

impl crate::estimator::Estimator for SquareRootUkfAttitude {
    fn name(&self) -> &'static str {
        "estimator.sr_ukf_attitude"
    }

    fn predict(&mut self, dt: f64) -> Result<(), EstimatorError> {
        self.inner.predict(dt)?;
        self.pin_unused_subspace()
    }

    fn update_imu(&mut self, sample: &ImuSample) -> Result<(), EstimatorError> {
        self.inner.update_imu(sample)
    }

    fn update_gnss(&mut self, _sample: &GnssSample) -> Result<(), EstimatorError> {
        // Attitude-only filter: GNSS is not consumed.
        Ok(())
    }

    fn update_baro(&mut self, _sample: &BarometerSample) -> Result<(), EstimatorError> {
        // Attitude-only filter: barometer is not consumed.
        Ok(())
    }

    fn update_mag(&mut self, sample: &MagnetometerSample) -> Result<(), EstimatorError> {
        self.inner.update_mag(sample)?;
        self.pin_unused_subspace()
    }

    fn attitude(&self) -> AttitudeEstimate {
        self.inner.attitude()
    }

    fn position(&self) -> PositionEstimate {
        // Attitude-only filter has no meaningful position estimate.
        PositionEstimate {
            time: SimTime::ZERO,
            position_eci_m: Vector3::zeros(),
            velocity_eci_m_s: Vector3::zeros(),
            accel_bias_body_m_s2: Vector3::zeros(),
        }
    }

    fn status(&self) -> EstimatorStatus {
        let mut s = self.inner.status();
        // Override dead_reckoning — meaningless for attitude-only.
        s.dead_reckoning = false;
        s.position_variance_eci_m2 = [0.0; 3];
        s.velocity_variance_eci_m2_s2 = [0.0; 3];
        s
    }

    fn begin_tick(&mut self) {
        self.inner.begin_tick();
    }
}

fn pin_attitude_only_subspace(inner: &mut SquareRootUkf) -> Result<(), CholupdateError> {
    let current = covariance_from_cholesky(&inner.s);
    let mut p = DMatrix::<f64>::zeros(SRUKF_STATE_DIM, SRUKF_STATE_DIM);
    for i in 0..3 {
        p[(i, i)] = ATTITUDE_ONLY_UNUSED_VARIANCE;
        p[(i + 3, i + 3)] = ATTITUDE_ONLY_UNUSED_VARIANCE;
        p[(i + 12, i + 12)] = ATTITUDE_ONLY_UNUSED_VARIANCE;
    }
    for r in 6..12 {
        for c in 6..12 {
            p[(r, c)] = current[(r, c)];
        }
    }
    inner.s = cholesky_from_covariance(&p)?;
    Ok(())
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss
)]
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

    fn deterministic_lower_factor(n: usize) -> DMatrix<f64> {
        let mut s = DMatrix::<f64>::zeros(n, n);
        for row in 0..n {
            for col in 0..=row {
                if row == col {
                    s[(row, col)] = 1.5 + 0.125 * (row as f64 + 1.0);
                } else {
                    let phase = (row as f64 + 1.0) * 1.7 + (col as f64 + 1.0) * 0.9;
                    s[(row, col)] = 0.04 * phase.sin();
                }
            }
        }
        s
    }

    fn deterministic_update_vector(n: usize, scale: f64) -> DVector<f64> {
        DVector::<f64>::from_iterator(
            n,
            (0..n).map(|i| {
                let phase = (i as f64 + 1.0) * 2.3;
                scale * phase.cos()
            }),
        )
    }

    fn assert_matrix_abs_diff(a: &DMatrix<f64>, b: &DMatrix<f64>, epsilon: f64) {
        assert_eq!(a.nrows(), b.nrows());
        assert_eq!(a.ncols(), b.ncols());
        for r in 0..a.nrows() {
            for c in 0..a.ncols() {
                assert_abs_diff_eq!(a[(r, c)], b[(r, c)], epsilon = epsilon);
            }
        }
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
    fn cholupdate_minus_matches_explicit_outer_product_non_diagonal_2x2() {
        let s_orig = DMatrix::<f64>::from_row_slice(2, 2, &[2.0, 0.0, 0.4, 1.5]);
        let p = covariance_from_cholesky(&s_orig);
        let u_orig = dvector(&[0.25, -0.15]);
        let mut s = s_orig;
        let mut u = u_orig.clone();
        cholupdate_in_place(&mut s, &mut u, CholupdateSign::Minus).unwrap();
        let p_recovered = covariance_from_cholesky(&s);
        let p_expected = &p - &u_orig * u_orig.transpose();
        assert_matrix_abs_diff(&p_recovered, &p_expected, 1.0e-12);
    }

    #[test]
    fn cholupdate_matches_explicit_outer_product_across_dimensions() {
        for n in [4_usize, 6, 15] {
            let s_orig = deterministic_lower_factor(n);
            let p = covariance_from_cholesky(&s_orig);
            for sign in [CholupdateSign::Plus, CholupdateSign::Minus] {
                let scale = if sign == CholupdateSign::Plus {
                    0.08
                } else {
                    0.02
                };
                let u_orig = deterministic_update_vector(n, scale);
                let mut s = s_orig.clone();
                let mut u = u_orig.clone();
                cholupdate_in_place(&mut s, &mut u, sign).unwrap();
                let p_recovered = covariance_from_cholesky(&s);
                let outer = &u_orig * u_orig.transpose();
                let p_expected = if sign == CholupdateSign::Plus {
                    &p + outer
                } else {
                    &p - outer
                };
                assert_matrix_abs_diff(&p_recovered, &p_expected, 1.0e-10);
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
    fn predict_cholesky_qr_recovers_covariance_with_offdiagonal_q() {
        let residuals = [
            dvector(&[0.8, -0.1, 0.3]),
            dvector(&[-0.2, 0.7, 0.5]),
            dvector(&[0.4, 0.2, -0.6]),
            dvector(&[0.1, -0.3, 0.9]),
        ];
        let q_sqrt = DMatrix::<f64>::from_row_slice(
            3,
            3,
            &[
                0.20, 0.0, 0.0, //
                0.05, 0.30, 0.0, //
                -0.02, 0.04, 0.25,
            ],
        );
        let mut m = DMatrix::<f64>::zeros(3, residuals.len() + 3);
        for (col, residual) in residuals.iter().enumerate() {
            for row in 0..3 {
                m[(row, col)] = residual[row];
            }
        }
        for col in 0..3 {
            for row in 0..3 {
                m[(row, residuals.len() + col)] = q_sqrt[(row, col)];
            }
        }
        let mut p_expected = &q_sqrt * q_sqrt.transpose();
        for residual in residuals {
            p_expected += &residual * residual.transpose();
        }

        let s = predict_cholesky_qr(&m).unwrap();
        let p_recovered = covariance_from_cholesky(&s);
        assert_matrix_abs_diff(&p_recovered, &p_expected, 1.0e-10);
        for i in 0..3 {
            for j in (i + 1)..3 {
                assert_eq!(s[(i, j)].to_bits(), 0.0_f64.to_bits());
            }
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

    // -----------------------------------------------------------------
    // SquareRootUkf integration tests
    // -----------------------------------------------------------------

    use crate::estimator::{Ekf, EkfParams, Estimator};
    use crate::topics::{
        BarometerSample as BSample, GnssSample as GSample, ImuSample as ISample,
        MagnetometerSample as MSample,
    };
    use openbmp_core::SimTime;

    fn fresh_filter() -> SquareRootUkf {
        let params = SquareRootUkfParams::default();
        let mut f = SquareRootUkf::new(params);
        f.seed(
            Vector3::new(0.0, 0.0, 100.0),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        f
    }

    fn imu_sample(gyro: Vector3<f64>, accel: Vector3<f64>) -> ISample {
        ISample {
            time: SimTime::ZERO,
            gyro_rad_s: gyro,
            accel_m_s2: accel,
            healthy: true,
        }
    }

    #[test]
    fn sr_ukf_constructor_seeds_initialised_state() {
        let f = fresh_filter();
        let pos = f.position();
        assert_abs_diff_eq!(pos.position_eci_m.z, 100.0, epsilon = 1.0e-12);
        // Default S has the documented diagonal layout.
        let p_max = f.covariance_max_diag();
        assert!(p_max >= 100.0); // position diag
        assert!(p_max.is_finite());
    }

    #[test]
    fn sr_ukf_predict_without_imu_sample_is_noop() {
        let mut f = fresh_filter();
        let pos_before = f.pos_eci;
        f.predict(0.01).expect("predict ok");
        assert_abs_diff_eq!(f.pos_eci.x, pos_before.x, epsilon = 1.0e-15);
        assert_abs_diff_eq!(f.pos_eci.y, pos_before.y, epsilon = 1.0e-15);
        assert_abs_diff_eq!(f.pos_eci.z, pos_before.z, epsilon = 1.0e-15);
    }

    #[test]
    fn sr_ukf_predict_integrates_imu_specific_force() {
        let mut f = fresh_filter();
        // Static IMU sample: gravity-cancelling specific force, zero rate.
        // Specific force in body = -g_eci in body frame. With identity
        // attitude, that's (0, 0, +9.81) m/s² (pointing up to cancel
        // gravity-down).
        let imu = imu_sample(Vector3::zeros(), Vector3::new(0.0, 0.0, 9.80665));
        f.update_imu(&imu).expect("imu sample ingest");
        f.predict(0.01).expect("predict ok");
        // Vertical velocity / position remain near zero (specific force
        // exactly cancels gravity).
        assert!(f.vel_eci.norm() < 1.0e-6);
    }

    #[test]
    fn sr_ukf_nominal_predict_matches_ekf_byte_for_byte() {
        let mut ekf = Ekf::new(EkfParams::default());
        let mut sr = SquareRootUkf::new(SquareRootUkfParams::default());
        let pos0 = Vector3::new(1.0, -2.0, 100.0);
        let vel0 = Vector3::new(0.5, -0.25, 0.1);
        let q0 = UnitQuaternion::identity();
        ekf.seed(pos0, vel0, q0);
        sr.seed(pos0, vel0, q0);
        let imu = imu_sample(
            Vector3::new(0.01, -0.02, 0.03),
            Vector3::new(0.1, -0.2, 9.80665),
        );
        ekf.update_imu(&imu).unwrap();
        sr.update_imu(&imu).unwrap();
        for _ in 0..2 {
            ekf.predict(0.01).unwrap();
            sr.predict(0.01).unwrap();
        }

        let ekf_pos = ekf.position();
        let sr_pos = sr.position();
        for i in 0..3 {
            assert_eq!(
                ekf_pos.position_eci_m[i].to_bits(),
                sr_pos.position_eci_m[i].to_bits()
            );
            assert_eq!(
                ekf_pos.velocity_eci_m_s[i].to_bits(),
                sr_pos.velocity_eci_m_s[i].to_bits()
            );
            assert_eq!(
                ekf_pos.accel_bias_body_m_s2[i].to_bits(),
                sr_pos.accel_bias_body_m_s2[i].to_bits()
            );
        }
        let ekf_att = ekf.attitude();
        let sr_att = sr.attitude();
        for i in 0..4 {
            assert_eq!(
                ekf_att.q_body_to_eci_xyzw[i].to_bits(),
                sr_att.q_body_to_eci_xyzw[i].to_bits()
            );
        }
        for i in 0..3 {
            assert_eq!(
                ekf_att.omega_body_rad_s[i].to_bits(),
                sr_att.omega_body_rad_s[i].to_bits()
            );
            assert_eq!(
                ekf_att.gyro_bias_body_rad_s[i].to_bits(),
                sr_att.gyro_bias_body_rad_s[i].to_bits()
            );
        }
    }

    #[test]
    fn sr_ukf_gnss_update_reduces_position_covariance() {
        let mut f = fresh_filter();
        let p_before = f.covariance_max_diag();
        let gnss = GSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(0.0, 0.0, 100.0),
            velocity_eci_m_s: Vector3::zeros(),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        };
        f.update_gnss(&gnss).expect("gnss update ok");
        let p_after = f.covariance_max_diag();
        assert!(
            p_after < p_before,
            "covariance should shrink after GNSS update: before {p_before}, after {p_after}"
        );
    }

    #[test]
    fn sr_ukf_linear_gnss_update_matches_scalar_kalman_algebra() {
        let params = SquareRootUkfParams {
            innovation_gate: 1.0e12,
            ..SquareRootUkfParams::default()
        };
        let mut f = SquareRootUkf::new(params);
        let prior_pos = Vector3::new(10.0, -20.0, 30.0);
        let prior_vel = Vector3::new(1.0, -2.0, 3.0);
        f.seed(prior_pos, prior_vel, UnitQuaternion::identity());
        let measurement_pos = Vector3::new(12.0, -23.0, 35.0);
        let measurement_vel = Vector3::new(0.8, -1.5, 2.5);
        let gnss = GSample {
            time: SimTime::ZERO,
            position_eci_m: measurement_pos,
            velocity_eci_m_s: measurement_vel,
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        };

        f.update_gnss(&gnss).unwrap();

        let updated_pos = f.position();
        let p = covariance_from_cholesky(&f.s);
        for i in 0..3 {
            let p_prior = 100.0;
            let r = f.params.sigma_gnss_pos_m * f.params.sigma_gnss_pos_m;
            let k = p_prior / (p_prior + r);
            let expected_state = prior_pos[i] + k * (measurement_pos[i] - prior_pos[i]);
            let expected_cov = (1.0 - k) * p_prior;
            assert_abs_diff_eq!(
                updated_pos.position_eci_m[i],
                expected_state,
                epsilon = 1.0e-8
            );
            assert_abs_diff_eq!(p[(i, i)], expected_cov, epsilon = 1.0e-8);
        }
        for i in 0..3 {
            let p_prior = 10.0;
            let r = f.params.sigma_gnss_vel_m_s * f.params.sigma_gnss_vel_m_s;
            let k = p_prior / (p_prior + r);
            let expected_state = prior_vel[i] + k * (measurement_vel[i] - prior_vel[i]);
            let expected_cov = (1.0 - k) * p_prior;
            assert_abs_diff_eq!(
                updated_pos.velocity_eci_m_s[i],
                expected_state,
                epsilon = 1.0e-8
            );
            assert_abs_diff_eq!(p[(i + 3, i + 3)], expected_cov, epsilon = 1.0e-8);
        }
    }

    #[test]
    fn sr_ukf_baro_update_shrinks_pos_z_uncertainty() {
        let mut f = fresh_filter();
        // P_zz from initial diag = 100.
        // 100 m altitude pressure ≈ 100129 Pa (USSA76 troposphere). The
        // exact value isn't important for this test — the assertion
        // is that the cov downdate is finite and shrinks the pos_z
        // diagonal entry.
        let baro = BSample {
            time: SimTime::ZERO,
            pressure_pa: 100_129.0,
            bias_pa: 0.0,
            healthy: true,
        };
        f.update_baro(&baro).expect("baro update ok");
        // Recover P from S; check (2, 2) (pos_z) shrunk.
        let p = covariance_from_cholesky(&f.s);
        assert!(p[(2, 2)] < 100.0);
        assert!(p[(2, 2)] > 0.0);
    }

    #[test]
    fn sr_ukf_mag_update_runs_without_panicking() {
        let mut f = fresh_filter();
        // Use the model-predicted field so the innovation is zero —
        // the assertion is that the cov downdate runs cleanly, not
        // that the filter accepts an arbitrary measurement.
        let predicted_eci = f.mag_field.field_eci_nt(f.pos_eci, SimTime::ZERO);
        let predicted_body = f.q_body_to_eci.to_rotation_matrix().transpose() * predicted_eci;
        let mag = MSample {
            time: SimTime::ZERO,
            field_body_nt: predicted_body,
            hard_iron_body_nt: Vector3::zeros(),
            healthy: true,
        };
        f.update_mag(&mag).expect("mag update ok");
        assert!(f.s.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn sr_ukf_central_negative_covariance_weight_path_runs() {
        let params = SquareRootUkfParams {
            sigma_gnss_pos_m: 50.0,
            sigma_gnss_vel_m_s: 50.0,
            ..SquareRootUkfParams::default()
        };
        let scaling = UkfScalingParams {
            alpha: 1.0,
            beta: 0.0,
            kappa: 3.0 - SRUKF_STATE_DIM as f64,
        };
        let mut f = SquareRootUkf::new_with_scaling(params, scaling);
        f.seed(
            Vector3::zeros(),
            Vector3::zeros(),
            UnitQuaternion::identity(),
        );
        assert!(f.weights.w0_cov < 0.0);

        let z = DVector::<f64>::from_row_slice(&[100.0]);
        let r_sqrt = DMatrix::<f64>::from_element(1, 1, 50.0);
        let predict_z = |_filter: &SquareRootUkf, perturbation: &DVector<f64>| -> DVector<f64> {
            DVector::<f64>::from_row_slice(&[perturbation[0] * perturbation[0]])
        };

        let (chi2, whitened, _log_det_s) = f
            .sigma_point_update(&z, &r_sqrt, predict_z, "negative-w0-test", 1.0)
            .unwrap();
        assert_abs_diff_eq!(chi2, 0.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(whitened[0], 0.0, epsilon = 1.0e-12);
    }

    #[test]
    fn sr_ukf_two_predicts_are_within_platform_bit_stable() {
        let imu = imu_sample(
            Vector3::new(0.01, 0.02, 0.03),
            Vector3::new(0.0, 0.0, 9.80665),
        );

        let mut f1 = fresh_filter();
        let mut f2 = fresh_filter();
        f1.update_imu(&imu).unwrap();
        f2.update_imu(&imu).unwrap();
        for _ in 0..10 {
            f1.predict(0.01).unwrap();
            f2.predict(0.01).unwrap();
        }
        for r in 0..15 {
            for c in 0..15 {
                assert_eq!(
                    f1.s[(r, c)].to_bits(),
                    f2.s[(r, c)].to_bits(),
                    "S[{r},{c}] diverged on identical inputs (within-platform bit-stability \
                     contract)",
                );
            }
        }
    }

    #[test]
    fn sr_ukf_attitude_filter_runs_predict_and_mag_update() {
        // 6-state SquareRootUkfAttitude smoke test. Mirrors the
        // classical Ukf use case.
        let mut f = SquareRootUkfAttitude::new(SquareRootUkfParams::default());
        f.seed(UnitQuaternion::identity());
        let imu = imu_sample(
            Vector3::new(0.0, 0.0, 0.01),
            Vector3::new(0.0, 0.0, 9.80665),
        );
        f.update_imu(&imu).unwrap();
        f.predict(0.01).unwrap();
        // Use the model-predicted field so the innovation passes the
        // gate; this is the smoke-test analogue of the retired
        // classical attitude UKF predict-and-mag-update test.
        let predicted_eci = f
            .inner
            .mag_field
            .field_eci_nt(f.inner.pos_eci, SimTime::ZERO);
        let predicted_body = f.inner.q_body_to_eci.to_rotation_matrix().transpose() * predicted_eci;
        let mag = MSample {
            time: SimTime::ZERO,
            field_body_nt: predicted_body,
            hard_iron_body_nt: Vector3::zeros(),
            healthy: true,
        };
        f.update_mag(&mag).unwrap();
        // Position is meaningless for the attitude-only variant.
        let pos = f.position();
        assert_eq!(pos.position_eci_m, Vector3::zeros());
        // GNSS / baro updates are no-ops.
        let gnss = GSample {
            time: SimTime::ZERO,
            position_eci_m: Vector3::new(1.0, 2.0, 3.0),
            velocity_eci_m_s: Vector3::zeros(),
            position_bias_eci_m: Vector3::zeros(),
            healthy: true,
        };
        f.update_gnss(&gnss).unwrap();
        // Attitude is still finite.
        let att = f.attitude();
        assert!(att.q_body_to_eci_xyzw.iter().all(|v| v.is_finite()));
    }

    #[test]
    fn sr_ukf_attitude_predict_only_keeps_unused_covariance_pinned() {
        let mut f = SquareRootUkfAttitude::new(SquareRootUkfParams::default());
        f.seed(UnitQuaternion::identity());
        let imu = imu_sample(Vector3::zeros(), Vector3::new(0.0, 0.0, 9.80665));
        f.update_imu(&imu).unwrap();
        for _ in 0..2_000 {
            f.predict(0.01).unwrap();
        }
        let p = covariance_from_cholesky(&f.inner.s);
        for i in 0..3 {
            assert_abs_diff_eq!(p[(i, i)], ATTITUDE_ONLY_UNUSED_VARIANCE, epsilon = 1.0e-24);
            assert_abs_diff_eq!(
                p[(i + 3, i + 3)],
                ATTITUDE_ONLY_UNUSED_VARIANCE,
                epsilon = 1.0e-24
            );
            assert_abs_diff_eq!(
                p[(i + 12, i + 12)],
                ATTITUDE_ONLY_UNUSED_VARIANCE,
                epsilon = 1.0e-24
            );
        }
        assert!(p.iter().all(|v| v.is_finite()));
    }
}
