//! Per-axis and coupled-plant LQR helpers for the rate loop.
//!
//! Supplies the optimal full-state feedback gains for a single
//! body axis under the standing project assumption that the
//! moment-of-inertia matrix is diagonal (rocket-class vehicles with
//! body-axis-aligned inertia ellipsoids). Each axis is then an
//! independent SISO plant
//!
//! ```text
//! ω̇ = τ / J
//! ```
//!
//! discretised by Forward-Euler at the loop step `dt`. Augmenting
//! the state with the integral of the rate-tracking error (in
//! reference-shifted coordinates) gives the 2×2 system
//!
//! ```text
//! z[k+1] = A z[k] + B τ[k]
//! z = [ω - ω_ref, ∫(ω - ω_ref) dt]
//! A = [[1, 0],
//!      [dt, 1]]
//! B = [dt/J, 0]
//! ```
//!
//! whose Discrete Algebraic Riccati Equation (DARE)
//!
//! ```text
//! P = A' P A − A' P B (R + B' P B)^{-1} B' P A + Q
//! ```
//!
//! is solved by a structure-preserving doubling algorithm. With
//! `Q = diag(q_omega, q_int)` and scalar `R = r`, the solver returns
//! the LQR feedback gains
//!
//! ```text
//! K = (R + B' P B)^{-1} B' P A = [k_omega, k_int]
//! u = − K z = − k_omega (ω − ω_ref) − k_int ∫(ω − ω_ref) dt.
//! ```
//!
//! All arithmetic is scalar — the 2×2 case unrolls cleanly without
//! pulling `nalgebra` into the hot path, which keeps the iteration
//! bit-stable across platforms.
//!
//! # References
//!
//! - Anderson B.D.O. (1978). Second-order convergent algorithms for
//!   the steady-state Riccati equation.
//! - Anderson B.D.O., Moore J.B. (1990). *Optimal Control: Linear
//!   Quadratic Methods*. Prentice-Hall.
//! - Chu E.K.W., Fan H.Y., Lin W.W., Wang C.S. (2004).
//!   Structure-preserving algorithms for periodic discrete-time
//!   algebraic Riccati equations.
//! - Lewis F.L., Vrabie D., Syrmos V.L. (2012). *Optimal Control*,
//!   3rd ed. Wiley.

#[cfg(not(feature = "std"))]
use alloc::vec;
#[cfg(not(feature = "std"))]
use num_traits::Float;
use std::vec::Vec;
use thiserror::Error;

/// Maximum DARE iterations before the doubling algorithm gives up.
/// The structure-preserving doubling algorithm (SDA) used here
/// converges quadratically: `‖H_k − P‖ ~ ρ^{2^k}` where ρ < 1 is the
/// spectral radius of the closed-loop system. 200 iterations is
/// already vastly beyond what any well-posed instance needs.
pub const DARE_MAX_ITERATIONS: usize = 200;

/// Convergence tolerance on the Frobenius-norm difference between
/// successive iterates of the DARE solution `P`.
pub const DARE_CONVERGENCE_TOL: f64 = 1.0e-12;

/// Maximum dense coupled-plant state dimension accepted by the
/// in-repo DARE oracle. Runtime control still uses precomputed gains;
/// this guard keeps validation buffers bounded.
pub const DENSE_DARE_MAX_STATES: usize = 16;

/// Maximum dense coupled-plant control dimension accepted by the
/// in-repo DARE oracle.
pub const DENSE_DARE_MAX_CONTROLS: usize = 16;

const DENSE_SYMMETRY_TOL: f64 = 1.0e-10;
const DENSE_PIVOT_TOL: f64 = 1.0e-14;

/// Errors raised by [`solve_lqr_rate_loop`].
#[derive(Copy, Clone, Debug, Error, PartialEq)]
pub enum LqrError {
    /// `dt_s <= 0`. The discretisation needs a positive step.
    #[error("LQR rate-loop dt_s must be > 0; got {dt_s}")]
    NonPositiveDt {
        /// Offending value.
        dt_s: f64,
    },
    /// Diagonal inertia element is non-positive.
    #[error("LQR rate-loop inertia must be > 0; got {inertia_kg_m2}")]
    NonPositiveInertia {
        /// Offending value.
        inertia_kg_m2: f64,
    },
    /// One of the cost weights is non-positive.
    #[error("LQR cost weight must be > 0; got {value} for {label}")]
    NonPositiveCostWeight {
        /// Cost-weight name (e.g. `"q_omega"`, `"r"`).
        label: &'static str,
        /// Offending value.
        value: f64,
    },
    /// A parameter is NaN or infinite.
    #[error("LQR parameter must be finite; got {value} for {label}")]
    NonFiniteParameter {
        /// Parameter name.
        label: &'static str,
        /// Offending value.
        value: f64,
    },
    /// DARE iteration did not converge within
    /// [`DARE_MAX_ITERATIONS`].
    #[error(
        "LQR DARE did not converge within {iterations} iterations \
         (residual {residual:e}; tolerance {tolerance:e})"
    )]
    DareDidNotConverge {
        /// Iterations consumed.
        iterations: usize,
        /// Final residual `‖P_{k+1} − P_k‖_F`.
        residual: f64,
        /// Required tolerance.
        tolerance: f64,
    },
    /// Dense solver dimension is zero or exceeds the fixed bound.
    #[error("LQR dense dimension {label} must be in 1..={max}; got {value}")]
    InvalidDimension {
        /// Dimension label.
        label: &'static str,
        /// Offending dimension.
        value: usize,
        /// Maximum accepted dimension.
        max: usize,
    },
    /// Row-major matrix length does not match declared dimensions.
    #[error("LQR matrix {label} length mismatch: expected {expected}, got {actual}")]
    MatrixLengthMismatch {
        /// Matrix label.
        label: &'static str,
        /// Expected row-major entry count.
        expected: usize,
        /// Actual entry count.
        actual: usize,
    },
    /// Cost matrix is expected to be symmetric but is not.
    #[error(
        "LQR cost matrix {label} must be symmetric; entries ({row},{col})={value:e} \
         and ({col},{row})={transpose:e} differ"
    )]
    NonSymmetricCost {
        /// Matrix label.
        label: &'static str,
        /// Row index.
        row: usize,
        /// Column index.
        col: usize,
        /// Forward entry.
        value: f64,
        /// Transposed entry.
        transpose: f64,
    },
    /// Matrix expected to be positive definite was not factorable.
    #[error("LQR matrix {label} must be symmetric positive definite")]
    MatrixNotPositiveDefinite {
        /// Matrix label.
        label: &'static str,
    },
    /// Dense linear solve failed because the matrix was singular or
    /// numerically ill-conditioned for this deterministic oracle.
    #[error("LQR dense linear solve failed for {label}")]
    LinearSolveFailed {
        /// Matrix label.
        label: &'static str,
    },
}

/// Optimal feedback gains for one body axis. Used as
/// `u = − k_omega · (ω − ω_ref) − k_int · ∫(ω − ω_ref) dt`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct LqrGains {
    /// Proportional-on-rate gain (`N·m / (rad/s)`).
    pub k_omega: f64,
    /// Integral-on-rate-error gain (`N·m / (rad·s)`).
    pub k_int: f64,
}

/// Dense coupled-plant DARE/LQR solution.
///
/// `p_row_major` is the stabilising Riccati solution `P` (`states × states`).
/// `k_row_major` is the full-state feedback matrix `K`
/// (`controls × states`) used as `u = -K x`.
#[derive(Clone, Debug, PartialEq)]
pub struct CoupledLqrSolution {
    /// State dimension.
    pub states: usize,
    /// Control dimension.
    pub controls: usize,
    /// Stabilising DARE solution `P`, row-major.
    pub p_row_major: Vec<f64>,
    /// LQR gain `K`, row-major (`controls × states`).
    pub k_row_major: Vec<f64>,
    /// Number of structure-preserving doubling iterations consumed.
    pub iterations: usize,
    /// Algebraic DARE residual `‖P - dare_rhs(P)‖_F`.
    pub residual_norm: f64,
}

/// Dense steady-state LQG observer solution from the dual DARE.
///
/// `p_row_major` is the estimator covariance-like stabilising DARE
/// solution (`states × states`). `l_row_major` is the predictor-form
/// observer gain `L` (`states × measurements`) for
/// `x_hat[k+1] = A x_hat[k] + B u[k] + L(y[k] - C x_hat[k])`.
#[derive(Clone, Debug, PartialEq)]
pub struct CoupledLqgObserverSolution {
    /// State dimension.
    pub states: usize,
    /// Measurement dimension.
    pub measurements: usize,
    /// Stabilising dual DARE solution `P`, row-major.
    pub p_row_major: Vec<f64>,
    /// Observer gain `L`, row-major (`states × measurements`).
    pub l_row_major: Vec<f64>,
    /// Number of structure-preserving doubling iterations consumed.
    pub iterations: usize,
    /// Algebraic dual DARE residual.
    pub residual_norm: f64,
}

/// Solve the per-axis rate-loop DARE and return the LQR feedback
/// gains.
///
/// # Errors
///
/// Returns [`LqrError::NonPositiveDt`] / [`LqrError::NonPositiveInertia`]
/// / [`LqrError::NonPositiveCostWeight`] when an input parameter is
/// non-positive, [`LqrError::NonFiniteParameter`] when an input is
/// NaN or infinite, and [`LqrError::DareDidNotConverge`] when the
/// fixed-point iteration fails to settle within
/// [`DARE_MAX_ITERATIONS`].
pub fn solve_lqr_rate_loop(
    dt_s: f64,
    inertia_kg_m2: f64,
    q_omega: f64,
    q_int: f64,
    r: f64,
) -> Result<LqrGains, LqrError> {
    require_finite("dt_s", dt_s)?;
    require_finite("inertia_kg_m2", inertia_kg_m2)?;
    require_finite("q_omega", q_omega)?;
    require_finite("q_int", q_int)?;
    require_finite("r", r)?;
    if dt_s <= 0.0 {
        return Err(LqrError::NonPositiveDt { dt_s });
    }
    if inertia_kg_m2 <= 0.0 {
        return Err(LqrError::NonPositiveInertia { inertia_kg_m2 });
    }
    require_positive("q_omega", q_omega)?;
    require_positive("q_int", q_int)?;
    require_positive("r", r)?;

    let b = dt_s / inertia_kg_m2;
    let p = solve_dare_2x2_doubling(dt_s, b, q_omega, q_int, r)?;
    let denom = r + b * b * p.p11;
    // K = (R + B' P B)^{-1} B' P A = (1/denom) · [b(p11 + dt·p12), b·p12].
    let k_omega = b * (p.p11 + dt_s * p.p12) / denom;
    let k_int = b * p.p12 / denom;
    Ok(LqrGains { k_omega, k_int })
}

/// Solve the dense coupled-plant DARE and return the discrete LQR
/// feedback matrix.
///
/// Matrices are row-major with shapes:
///
/// - `A`: `states × states`
/// - `B`: `states × controls`
/// - `Q`: `states × states`, symmetric positive definite
/// - `R`: `controls × controls`, symmetric positive definite
///
/// The returned gain `K` has shape `controls × states` and is used as
/// `u = -K x`. This helper is intended as the deterministic in-repo
/// oracle for offline coupled-gain generation; the hot flight loop
/// consumes precomputed gains.
///
/// # Errors
///
/// Returns [`LqrError`] when dimensions are invalid, matrices contain
/// non-finite entries, `Q`/`R` are not symmetric positive definite, a
/// deterministic dense solve fails, or the structure-preserving
/// doubling iteration does not converge.
#[allow(clippy::too_many_arguments)]
pub fn solve_dare_nxn_doubling(
    states: usize,
    controls: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    q_row_major: &[f64],
    r_row_major: &[f64],
) -> Result<CoupledLqrSolution, LqrError> {
    solve_dare_nxn_doubling_with_options(
        states,
        controls,
        a_row_major,
        b_row_major,
        q_row_major,
        r_row_major,
        DARE_MAX_ITERATIONS,
        DARE_CONVERGENCE_TOL,
    )
}

/// Variant of [`solve_dare_nxn_doubling`] with explicit iteration
/// controls for tests and offline tools.
#[allow(clippy::too_many_arguments)]
pub fn solve_dare_nxn_doubling_with_options(
    states: usize,
    controls: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    q_row_major: &[f64],
    r_row_major: &[f64],
    max_iterations: usize,
    tolerance: f64,
) -> Result<CoupledLqrSolution, LqrError> {
    validate_dimension("states", states, DENSE_DARE_MAX_STATES)?;
    validate_dimension("controls", controls, DENSE_DARE_MAX_CONTROLS)?;
    require_finite("tolerance", tolerance)?;
    if tolerance <= 0.0 {
        return Err(LqrError::NonPositiveCostWeight {
            label: "tolerance",
            value: tolerance,
        });
    }
    if max_iterations == 0 {
        return Err(LqrError::InvalidDimension {
            label: "max_iterations",
            value: 0,
            max: DARE_MAX_ITERATIONS,
        });
    }

    let mut a = DenseMatrix::from_row_major("A", states, states, a_row_major)?;
    let b = DenseMatrix::from_row_major("B", states, controls, b_row_major)?;
    let q = DenseMatrix::from_row_major("Q", states, states, q_row_major)?;
    let r = DenseMatrix::from_row_major("R", controls, controls, r_row_major)?;
    validate_symmetric("Q", &q)?;
    validate_symmetric("R", &r)?;
    let _ = cholesky_lower("Q", &q)?;
    let r_inv = invert_spd("R", &r)?;

    let b_t = b.transpose();
    let mut g = b.mul(&r_inv).mul(&b_t);
    let mut h = q.clone();
    let mut residual = f64::INFINITY;
    let mut iterations = 0usize;
    let mut converged = false;

    for iteration in 1..=max_iterations {
        let gh = g.mul(&h);
        let m = DenseMatrix::identity(states).add(&gh);
        let m_inv = invert_lu("I + G H", &m)?;

        let new_a = a.mul(&m_inv).mul(&a);
        let a_t = a.transpose();
        let new_g = g.add(&a.mul(&m_inv).mul(&g).mul(&a_t));
        let new_h = h.add(&a_t.mul(&h).mul(&m_inv).mul(&a));
        let new_g = new_g.symmetrized();
        let new_h = new_h.symmetrized();

        residual = new_h.sub(&h).frobenius_norm();
        if !residual.is_finite() {
            return Err(LqrError::DareDidNotConverge {
                iterations: iteration,
                residual,
                tolerance,
            });
        }

        a = new_a;
        g = new_g;
        h = new_h;
        iterations = iteration;
        if residual < tolerance {
            converged = true;
            break;
        }
    }

    if !converged {
        return Err(LqrError::DareDidNotConverge {
            iterations,
            residual,
            tolerance,
        });
    }

    let a0 = DenseMatrix::from_row_major("A", states, states, a_row_major)?;
    let b0 = DenseMatrix::from_row_major("B", states, controls, b_row_major)?;
    let b0_t = b0.transpose();
    let bt_p = b0_t.mul(&h);
    let s = r.add(&bt_p.mul(&b0)).symmetrized();
    let rhs = bt_p.mul(&a0);
    let k = solve_spd("R + B' P B", &s, &rhs)?;
    let residual_norm = dense_dare_residual_norm_from_matrices(&a0, &b0, &q, &r, &h)?;
    if !residual_norm.is_finite() || residual_norm > tolerance.sqrt() {
        return Err(LqrError::DareDidNotConverge {
            iterations,
            residual: residual_norm,
            tolerance: tolerance.sqrt(),
        });
    }

    Ok(CoupledLqrSolution {
        states,
        controls,
        p_row_major: h.data,
        k_row_major: k.data,
        iterations,
        residual_norm,
    })
}

/// Solves the steady-state LQG observer gain through the LQR dual.
///
/// Matrices are row-major with shapes:
///
/// - `A`: `states × states`
/// - `C`: `measurements × states`
/// - `W`: `states × states`, symmetric positive definite process-noise weight
/// - `V`: `measurements × measurements`, symmetric positive definite measurement-noise weight
///
/// The returned gain `L` has shape `states × measurements`.
///
/// # Errors
///
/// Returns [`LqrError`] for the same bounded-dimension, finite-value,
/// positive-definite, and convergence failures as
/// [`solve_dare_nxn_doubling`].
#[allow(clippy::too_many_arguments)]
pub fn solve_lqg_observer_nxn_dual_dare(
    states: usize,
    measurements: usize,
    a_row_major: &[f64],
    c_row_major: &[f64],
    w_row_major: &[f64],
    v_row_major: &[f64],
) -> Result<CoupledLqgObserverSolution, LqrError> {
    solve_lqg_observer_nxn_dual_dare_with_options(
        states,
        measurements,
        a_row_major,
        c_row_major,
        w_row_major,
        v_row_major,
        DARE_MAX_ITERATIONS,
        DARE_CONVERGENCE_TOL,
    )
}

/// Variant of [`solve_lqg_observer_nxn_dual_dare`] with explicit
/// iteration controls for tests and offline tools.
#[allow(clippy::too_many_arguments)]
pub fn solve_lqg_observer_nxn_dual_dare_with_options(
    states: usize,
    measurements: usize,
    a_row_major: &[f64],
    c_row_major: &[f64],
    w_row_major: &[f64],
    v_row_major: &[f64],
    max_iterations: usize,
    tolerance: f64,
) -> Result<CoupledLqgObserverSolution, LqrError> {
    validate_dimension("states", states, DENSE_DARE_MAX_STATES)?;
    validate_dimension("measurements", measurements, DENSE_DARE_MAX_CONTROLS)?;
    let a = DenseMatrix::from_row_major("A", states, states, a_row_major)?;
    let c = DenseMatrix::from_row_major("C", measurements, states, c_row_major)?;
    let solution = solve_dare_nxn_doubling_with_options(
        states,
        measurements,
        &a.transpose().data,
        &c.transpose().data,
        w_row_major,
        v_row_major,
        max_iterations,
        tolerance,
    )?;
    let l = DenseMatrix::from_row_major("K_dual", measurements, states, &solution.k_row_major)?
        .transpose();
    Ok(CoupledLqgObserverSolution {
        states,
        measurements,
        p_row_major: solution.p_row_major,
        l_row_major: l.data,
        iterations: solution.iterations,
        residual_norm: solution.residual_norm,
    })
}

/// Compute the dense DARE algebraic residual for an externally
/// supplied candidate `P`.
///
/// This is exposed for tolerance-table tests and offline verification
/// harnesses.
#[allow(clippy::too_many_arguments)]
pub fn dense_dare_residual_norm(
    states: usize,
    controls: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    q_row_major: &[f64],
    r_row_major: &[f64],
    p_row_major: &[f64],
) -> Result<f64, LqrError> {
    validate_dimension("states", states, DENSE_DARE_MAX_STATES)?;
    validate_dimension("controls", controls, DENSE_DARE_MAX_CONTROLS)?;
    let a = DenseMatrix::from_row_major("A", states, states, a_row_major)?;
    let b = DenseMatrix::from_row_major("B", states, controls, b_row_major)?;
    let q = DenseMatrix::from_row_major("Q", states, states, q_row_major)?;
    let r = DenseMatrix::from_row_major("R", controls, controls, r_row_major)?;
    let p = DenseMatrix::from_row_major("P", states, states, p_row_major)?;
    validate_symmetric("Q", &q)?;
    validate_symmetric("R", &r)?;
    validate_symmetric("P", &p)?;
    dense_dare_residual_norm_from_matrices(&a, &b, &q, &r, &p)
}

/// Symmetric 2×2 matrix `[[p11, p12], [p12, p22]]` used internally
/// by the structure-preserving doubling algorithm.
#[derive(Copy, Clone, Debug, Default)]
struct Sym2 {
    p11: f64,
    p12: f64,
    p22: f64,
}

/// Solve the DARE for the per-axis rate-loop system
/// `A = [[1, 0], [dt, 1]]`, `B = [b, 0]'`, `Q = diag(q1, q2)`,
/// `R = r` using the structure-preserving doubling algorithm
/// (Anderson 1978; Chu, Fan, Lin, Wang 2004).
///
/// SDA propagates the triple `(A_k, G_k, H_k)` whose limit
/// `H_∞ = P` is the stabilising DARE solution. Each iteration
/// approximately *squares* the residual, converging in
/// `O(log log(1/ε))` steps even for systems whose closed-loop
/// poles approach the unit circle.
#[allow(clippy::too_many_lines)]
fn solve_dare_2x2_doubling(
    dt_s: f64,
    b: f64,
    q_omega: f64,
    q_int: f64,
    r: f64,
) -> Result<Sym2, LqrError> {
    // Initial triple:
    //   A_0 = A
    //   G_0 = B R^{-1} B' = (b² / r) · e_1 e_1'
    //   H_0 = Q
    // Stored as scalars where the 2×2 is rank-deficient.
    let mut a11 = 1.0_f64;
    let mut a12 = 0.0_f64;
    let mut a21 = dt_s;
    let mut a22 = 1.0_f64;
    let mut g = Sym2 {
        p11: b * b / r,
        p12: 0.0,
        p22: 0.0,
    };
    let mut h = Sym2 {
        p11: q_omega,
        p12: 0.0,
        p22: q_int,
    };

    let mut residual = f64::INFINITY;
    let mut converged = false;
    for _ in 0..DARE_MAX_ITERATIONS {
        // M = (I + G H)^{-1}.
        // G H is a 2×2 matrix; for symmetric G and H it is generally
        // non-symmetric. Compute (I + G H) explicitly then invert.
        let gh11 = g.p11 * h.p11 + g.p12 * h.p12;
        let gh12 = g.p11 * h.p12 + g.p12 * h.p22;
        let gh21 = g.p12 * h.p11 + g.p22 * h.p12;
        let gh22 = g.p12 * h.p12 + g.p22 * h.p22;
        let m11 = 1.0 + gh11;
        let m12 = gh12;
        let m21 = gh21;
        let m22 = 1.0 + gh22;
        let det_m = m11 * m22 - m12 * m21;
        if !det_m.is_finite() || det_m.abs() < f64::MIN_POSITIVE {
            return Err(LqrError::DareDidNotConverge {
                iterations: DARE_MAX_ITERATIONS,
                residual,
                tolerance: DARE_CONVERGENCE_TOL,
            });
        }
        // Inverse of (I + G H).
        let inv11 = m22 / det_m;
        let inv12 = -m12 / det_m;
        let inv21 = -m21 / det_m;
        let inv22 = m11 / det_m;

        // A_next = A · inv · A.
        // First compute T = A · inv (2×2 dense).
        let t11 = a11 * inv11 + a12 * inv21;
        let t12 = a11 * inv12 + a12 * inv22;
        let t21 = a21 * inv11 + a22 * inv21;
        let t22 = a21 * inv12 + a22 * inv22;
        let new_a11 = t11 * a11 + t12 * a21;
        let new_a12 = t11 * a12 + t12 * a22;
        let new_a21 = t21 * a11 + t22 * a21;
        let new_a22 = t21 * a12 + t22 * a22;

        // G_next = G + A · inv · G · A'.
        // First compute U = inv · G (dense 2×2 since inv is non-symmetric).
        let u11 = inv11 * g.p11 + inv12 * g.p12;
        let u12 = inv11 * g.p12 + inv12 * g.p22;
        let u21 = inv21 * g.p11 + inv22 * g.p12;
        let u22 = inv21 * g.p12 + inv22 * g.p22;
        // V = A · U (dense 2×2).
        let v11 = a11 * u11 + a12 * u21;
        let v12 = a11 * u12 + a12 * u22;
        let v21 = a21 * u11 + a22 * u21;
        let v22 = a21 * u12 + a22 * u22;
        // V · A' (symmetrise via the upper triangle).
        let g_add11 = v11 * a11 + v12 * a12;
        let g_add12 = v11 * a21 + v12 * a22;
        let g_add22 = v21 * a21 + v22 * a22;
        let new_g = Sym2 {
            p11: g.p11 + g_add11,
            p12: g.p12 + g_add12,
            p22: g.p22 + g_add22,
        };

        // H_next = H + A' · H · inv · A.
        // Compute W = H · inv.
        let w11 = h.p11 * inv11 + h.p12 * inv21;
        let w12 = h.p11 * inv12 + h.p12 * inv22;
        let w21 = h.p12 * inv11 + h.p22 * inv21;
        let w22 = h.p12 * inv12 + h.p22 * inv22;
        // X = W · A.
        let x11 = w11 * a11 + w12 * a21;
        let x12 = w11 * a12 + w12 * a22;
        let x21 = w21 * a11 + w22 * a21;
        let x22 = w21 * a12 + w22 * a22;
        // A' · X.
        let h_add11 = a11 * x11 + a21 * x21;
        let h_add12 = a11 * x12 + a21 * x22;
        let h_add22 = a12 * x12 + a22 * x22;
        let new_h = Sym2 {
            p11: h.p11 + h_add11,
            p12: h.p12 + h_add12,
            p22: h.p22 + h_add22,
        };

        let dp11 = new_h.p11 - h.p11;
        let dp12 = new_h.p12 - h.p12;
        let dp22 = new_h.p22 - h.p22;
        residual = (dp11 * dp11 + 2.0 * dp12 * dp12 + dp22 * dp22).sqrt();

        a11 = new_a11;
        a12 = new_a12;
        a21 = new_a21;
        a22 = new_a22;
        g = new_g;
        h = new_h;

        if residual < DARE_CONVERGENCE_TOL {
            converged = true;
            break;
        }
    }
    let _ = (a11, a12, a21, a22);
    if !converged {
        return Err(LqrError::DareDidNotConverge {
            iterations: DARE_MAX_ITERATIONS,
            residual,
            tolerance: DARE_CONVERGENCE_TOL,
        });
    }
    Ok(h)
}

#[derive(Clone, Debug, PartialEq)]
struct DenseMatrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl DenseMatrix {
    fn from_row_major(
        label: &'static str,
        rows: usize,
        cols: usize,
        data: &[f64],
    ) -> Result<Self, LqrError> {
        let expected = rows * cols;
        if data.len() != expected {
            return Err(LqrError::MatrixLengthMismatch {
                label,
                expected,
                actual: data.len(),
            });
        }
        for value in data {
            require_finite(label, *value)?;
        }
        Ok(Self {
            rows,
            cols,
            data: data.to_vec(),
        })
    }

    fn zeros(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            data: vec![0.0; rows * cols],
        }
    }

    fn identity(size: usize) -> Self {
        let mut out = Self::zeros(size, size);
        for i in 0..size {
            out.set(i, i, 1.0);
        }
        out
    }

    fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.cols + col]
    }

    fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row * self.cols + col] = value;
    }

    fn transpose(&self) -> Self {
        let mut out = Self::zeros(self.cols, self.rows);
        for row in 0..self.rows {
            for col in 0..self.cols {
                out.set(col, row, self.get(row, col));
            }
        }
        out
    }

    fn add(&self, other: &Self) -> Self {
        let mut out = Self::zeros(self.rows, self.cols);
        for i in 0..self.data.len() {
            out.data[i] = self.data[i] + other.data[i];
        }
        out
    }

    fn sub(&self, other: &Self) -> Self {
        let mut out = Self::zeros(self.rows, self.cols);
        for i in 0..self.data.len() {
            out.data[i] = self.data[i] - other.data[i];
        }
        out
    }

    fn mul(&self, other: &Self) -> Self {
        let mut out = Self::zeros(self.rows, other.cols);
        for row in 0..self.rows {
            for inner in 0..self.cols {
                let lhs = self.get(row, inner);
                for col in 0..other.cols {
                    let idx = row * out.cols + col;
                    out.data[idx] += lhs * other.get(inner, col);
                }
            }
        }
        out
    }

    fn symmetrized(&self) -> Self {
        let mut out = self.clone();
        for row in 0..self.rows {
            for col in (row + 1)..self.cols {
                let avg = 0.5 * (self.get(row, col) + self.get(col, row));
                out.set(row, col, avg);
                out.set(col, row, avg);
            }
        }
        out
    }

    fn frobenius_norm(&self) -> f64 {
        let mut sum = 0.0_f64;
        for value in &self.data {
            sum += *value * *value;
        }
        sum.sqrt()
    }
}

fn validate_dimension(label: &'static str, value: usize, max: usize) -> Result<(), LqrError> {
    if (1..=max).contains(&value) {
        Ok(())
    } else {
        Err(LqrError::InvalidDimension { label, value, max })
    }
}

fn validate_symmetric(label: &'static str, matrix: &DenseMatrix) -> Result<(), LqrError> {
    for row in 0..matrix.rows {
        for col in (row + 1)..matrix.cols {
            let value = matrix.get(row, col);
            let transpose = matrix.get(col, row);
            if (value - transpose).abs() > DENSE_SYMMETRY_TOL {
                return Err(LqrError::NonSymmetricCost {
                    label,
                    row,
                    col,
                    value,
                    transpose,
                });
            }
        }
    }
    Ok(())
}

fn cholesky_lower(label: &'static str, matrix: &DenseMatrix) -> Result<DenseMatrix, LqrError> {
    let n = matrix.rows;
    let mut lower = DenseMatrix::zeros(n, n);
    for row in 0..n {
        for col in 0..=row {
            let mut sum = matrix.get(row, col);
            for k in 0..col {
                sum -= lower.get(row, k) * lower.get(col, k);
            }
            if row == col {
                if !sum.is_finite() || sum <= DENSE_PIVOT_TOL {
                    return Err(LqrError::MatrixNotPositiveDefinite { label });
                }
                lower.set(row, col, sum.sqrt());
            } else {
                let diag = lower.get(col, col);
                if diag.abs() <= DENSE_PIVOT_TOL {
                    return Err(LqrError::MatrixNotPositiveDefinite { label });
                }
                lower.set(row, col, sum / diag);
            }
        }
    }
    Ok(lower)
}

fn solve_spd(
    label: &'static str,
    matrix: &DenseMatrix,
    rhs: &DenseMatrix,
) -> Result<DenseMatrix, LqrError> {
    let lower = cholesky_lower(label, matrix)?;
    let n = matrix.rows;
    let mut y = DenseMatrix::zeros(n, rhs.cols);
    for col in 0..rhs.cols {
        for row in 0..n {
            let mut sum = rhs.get(row, col);
            for k in 0..row {
                sum -= lower.get(row, k) * y.get(k, col);
            }
            y.set(row, col, sum / lower.get(row, row));
        }
    }

    let mut x = DenseMatrix::zeros(n, rhs.cols);
    for col in 0..rhs.cols {
        for row_rev in 0..n {
            let row = n - 1 - row_rev;
            let mut sum = y.get(row, col);
            for k in (row + 1)..n {
                sum -= lower.get(k, row) * x.get(k, col);
            }
            x.set(row, col, sum / lower.get(row, row));
        }
    }
    Ok(x)
}

fn invert_spd(label: &'static str, matrix: &DenseMatrix) -> Result<DenseMatrix, LqrError> {
    solve_spd(label, matrix, &DenseMatrix::identity(matrix.rows))
}

fn invert_lu(label: &'static str, matrix: &DenseMatrix) -> Result<DenseMatrix, LqrError> {
    let n = matrix.rows;
    let mut lu = matrix.data.clone();
    let mut pivots: Vec<usize> = (0..n).collect();

    for k in 0..n {
        let mut pivot_row = k;
        let mut pivot_abs = lu[k * n + k].abs();
        for row in (k + 1)..n {
            let candidate_abs = lu[row * n + k].abs();
            if candidate_abs > pivot_abs {
                pivot_abs = candidate_abs;
                pivot_row = row;
            }
        }
        if !pivot_abs.is_finite() || pivot_abs <= DENSE_PIVOT_TOL {
            return Err(LqrError::LinearSolveFailed { label });
        }
        if pivot_row != k {
            for col in 0..n {
                lu.swap(k * n + col, pivot_row * n + col);
            }
            pivots.swap(k, pivot_row);
        }
        for row in (k + 1)..n {
            let factor = lu[row * n + k] / lu[k * n + k];
            lu[row * n + k] = factor;
            for col in (k + 1)..n {
                let idx = row * n + col;
                lu[idx] -= factor * lu[k * n + col];
            }
        }
    }

    let mut inverse = DenseMatrix::zeros(n, n);
    for rhs_col in 0..n {
        let mut y = vec![0.0_f64; n];
        for row in 0..n {
            let mut sum = if pivots[row] == rhs_col { 1.0 } else { 0.0 };
            for k in 0..row {
                sum -= lu[row * n + k] * y[k];
            }
            y[row] = sum;
        }

        let mut x = vec![0.0_f64; n];
        for row_rev in 0..n {
            let row = n - 1 - row_rev;
            let mut sum = y[row];
            for k in (row + 1)..n {
                sum -= lu[row * n + k] * x[k];
            }
            let diag = lu[row * n + row];
            if diag.abs() <= DENSE_PIVOT_TOL {
                return Err(LqrError::LinearSolveFailed { label });
            }
            x[row] = sum / diag;
        }

        for (row, value) in x.iter().enumerate().take(n) {
            inverse.set(row, rhs_col, *value);
        }
    }

    Ok(inverse)
}

fn dense_dare_residual_norm_from_matrices(
    a: &DenseMatrix,
    b: &DenseMatrix,
    q: &DenseMatrix,
    r: &DenseMatrix,
    p: &DenseMatrix,
) -> Result<f64, LqrError> {
    let a_t = a.transpose();
    let b_t = b.transpose();
    let pb = p.mul(b);
    let s = r.add(&b_t.mul(&pb)).symmetrized();
    let btpa = b_t.mul(p).mul(a);
    let k = solve_spd("R + B' P B", &s, &btpa)?;
    let rhs = a_t.mul(p).mul(a).sub(&a_t.mul(&pb).mul(&k)).add(q);
    Ok(p.sub(&rhs).frobenius_norm())
}

fn require_finite(label: &'static str, value: f64) -> Result<(), LqrError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(LqrError::NonFiniteParameter { label, value })
    }
}

fn require_positive(label: &'static str, value: f64) -> Result<(), LqrError> {
    if value > 0.0 {
        Ok(())
    } else {
        Err(LqrError::NonPositiveCostWeight { label, value })
    }
}

/// Closed-loop pole magnitudes of the discrete system
/// `A_cl = A − B K` for diagnostic / test use.
#[must_use]
pub fn closed_loop_pole_magnitudes(dt_s: f64, inertia_kg_m2: f64, gains: LqrGains) -> [f64; 2] {
    let b = dt_s / inertia_kg_m2;
    // A − B K, with B = [b, 0]', K = [k_omega, k_int]
    //   = [[1 − b k_omega, − b k_int], [dt, 1]]
    let a11 = 1.0 - b * gains.k_omega;
    let a12 = -b * gains.k_int;
    let a21 = dt_s;
    let a22 = 1.0;
    let trace = a11 + a22;
    let det = a11 * a22 - a12 * a21;
    let disc = trace * trace - 4.0 * det;
    if disc >= 0.0 {
        let s = disc.sqrt();
        let l1 = 0.5 * (trace + s);
        let l2 = 0.5 * (trace - s);
        [l1.abs(), l2.abs()]
    } else {
        // Complex conjugate pair: |λ|² = det.
        let mag = det.abs().sqrt();
        [mag, mag]
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
    use toml::Value;

    use super::*;

    const COUPLED_LQR_DARE_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/coupled-lqr-dare-tolerance-v1.toml");
    const GAIN_SCHEDULE_LQG_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/gain-schedule-lqg-tolerance-v1.toml");

    fn nominal() -> LqrGains {
        solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.1, 0.1).expect("nominal LQR converges")
    }

    fn toml_table<'a>(value: &'a Value, key: &str) -> &'a toml::map::Map<String, Value> {
        value
            .get(key)
            .and_then(Value::as_table)
            .unwrap_or_else(|| panic!("missing TOML table {key}"))
    }

    fn toml_array<'a>(value: &'a Value, key: &str) -> &'a [Value] {
        value
            .get(key)
            .and_then(Value::as_array)
            .map(Vec::as_slice)
            .unwrap_or_else(|| panic!("missing TOML array {key}"))
    }

    fn toml_f64(value: &toml::map::Map<String, Value>, key: &str) -> f64 {
        value
            .get(key)
            .and_then(Value::as_float)
            .unwrap_or_else(|| panic!("missing TOML float {key}"))
    }

    fn toml_usize(value: &toml::map::Map<String, Value>, key: &str) -> usize {
        value
            .get(key)
            .and_then(Value::as_integer)
            .and_then(|raw| usize::try_from(raw).ok())
            .unwrap_or_else(|| panic!("missing TOML usize {key}"))
    }

    fn toml_f64_array(value: &toml::map::Map<String, Value>, key: &str) -> Vec<f64> {
        value
            .get(key)
            .and_then(Value::as_array)
            .unwrap_or_else(|| panic!("missing TOML array {key}"))
            .iter()
            .map(|entry| {
                entry
                    .as_float()
                    .unwrap_or_else(|| panic!("TOML array {key} contains non-float"))
            })
            .collect()
    }

    fn dare_residual_norm(p: Sym2, dt_s: f64, b: f64, q_omega: f64, q_int: f64, r: f64) -> f64 {
        let a11 = 1.0_f64;
        let a12 = 0.0_f64;
        let a21 = dt_s;
        let a22 = 1.0_f64;

        let denom = r + b * b * p.p11;
        let k1 = b * (p.p11 + dt_s * p.p12) / denom;
        let k2 = b * p.p12 / denom;

        // A' P A.
        let pa11 = p.p11 * a11 + p.p12 * a21;
        let pa12 = p.p11 * a12 + p.p12 * a22;
        let pa21 = p.p12 * a11 + p.p22 * a21;
        let pa22 = p.p12 * a12 + p.p22 * a22;
        let apa11 = a11 * pa11 + a21 * pa21;
        let apa12 = a11 * pa12 + a21 * pa22;
        let apa22 = a12 * pa12 + a22 * pa22;

        // A' P B K, where B = [b, 0]'.
        let apb1 = b * (a11 * p.p11 + a21 * p.p12);
        let apb2 = b * (a12 * p.p11 + a22 * p.p12);
        let rhs = Sym2 {
            p11: apa11 - apb1 * k1 + q_omega,
            p12: apa12 - apb1 * k2,
            p22: apa22 - apb2 * k2 + q_int,
        };

        let d11 = p.p11 - rhs.p11;
        let d12 = p.p12 - rhs.p12;
        let d22 = p.p22 - rhs.p22;
        (d11 * d11 + 2.0 * d12 * d12 + d22 * d22).sqrt()
    }

    #[test]
    fn dare_converges_for_nominal_params() {
        let gains = nominal();
        assert!(gains.k_omega > 0.0);
        assert!(gains.k_int > 0.0);
    }

    #[test]
    fn dare_solution_has_small_algebraic_residual() {
        let dt = 0.001;
        let inertia = 1.0;
        let b = dt / inertia;
        let p = solve_dare_2x2_doubling(dt, b, 1.0, 0.05, 1.0)
            .expect("scenario-like LQR DARE converges");
        let residual = dare_residual_norm(p, dt, b, 1.0, 0.05, 1.0);
        assert!(
            residual < 1.0e-10,
            "DARE residual {residual:e} exceeds tolerance"
        );
    }

    #[test]
    fn coupled_lqr_dense_solver_matches_tolerance_table() {
        let data: Value =
            toml::from_str(COUPLED_LQR_DARE_TOLERANCE_TABLE).expect("coupled LQR table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_coupled_lqr_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let p_epsilon = toml_f64(tolerance, "absolute_p");
        let k_epsilon = toml_f64(tolerance, "absolute_k");
        let residual_epsilon = toml_f64(tolerance, "residual_norm");

        for problem in toml_array(&data, "problem") {
            let problem = problem.as_table().expect("problem table");
            let problem_name = problem
                .get("name")
                .and_then(Value::as_str)
                .expect("problem name");
            let states = toml_usize(problem, "states");
            let controls = toml_usize(problem, "controls");
            let solution = solve_dare_nxn_doubling(
                states,
                controls,
                &toml_f64_array(problem, "a_row_major"),
                &toml_f64_array(problem, "b_row_major"),
                &toml_f64_array(problem, "q_row_major"),
                &toml_f64_array(problem, "r_row_major"),
            )
            .unwrap_or_else(|err| panic!("{problem_name} dense DARE failed: {err}"));
            let rerun = solve_dare_nxn_doubling(
                states,
                controls,
                &toml_f64_array(problem, "a_row_major"),
                &toml_f64_array(problem, "b_row_major"),
                &toml_f64_array(problem, "q_row_major"),
                &toml_f64_array(problem, "r_row_major"),
            )
            .unwrap_or_else(|err| panic!("{problem_name} dense DARE rerun failed: {err}"));
            assert_eq!(solution, rerun, "{problem_name} bit-stable rerun");

            for (actual, expected) in solution
                .p_row_major
                .iter()
                .zip(toml_f64_array(problem, "expected_p"))
            {
                assert!(
                    (*actual - expected).abs() <= p_epsilon,
                    "{problem_name} P actual {actual:e}, expected {expected:e}, epsilon \
                     {p_epsilon:e}"
                );
            }
            for (actual, expected) in solution
                .k_row_major
                .iter()
                .zip(toml_f64_array(problem, "expected_k"))
            {
                assert!(
                    (*actual - expected).abs() <= k_epsilon,
                    "{problem_name} K actual {actual:e}, expected {expected:e}, epsilon \
                     {k_epsilon:e}"
                );
            }
            assert!(
                solution.residual_norm <= residual_epsilon,
                "{problem_name} residual {:e} exceeds {:e}",
                solution.residual_norm,
                residual_epsilon
            );
            assert!(
                solution.iterations <= toml_usize(problem, "expected_iterations_max"),
                "{problem_name} iterations {} exceed bound",
                solution.iterations
            );
        }
    }

    #[test]
    fn coupled_lqr_dense_solver_rejects_bad_inputs() {
        assert!(matches!(
            solve_dare_nxn_doubling(0, 1, &[], &[], &[], &[1.0]),
            Err(LqrError::InvalidDimension {
                label: "states",
                ..
            })
        ));
        assert!(matches!(
            solve_dare_nxn_doubling(1, 1, &[1.0], &[1.0], &[1.0], &[1.0, 2.0]),
            Err(LqrError::MatrixLengthMismatch { label: "R", .. })
        ));
        assert!(matches!(
            solve_dare_nxn_doubling(
                2,
                1,
                &[1.0, 0.0, 0.0, 1.0],
                &[1.0, 0.0],
                &[1.0, 0.2, 0.0, 1.0],
                &[1.0],
            ),
            Err(LqrError::NonSymmetricCost { label: "Q", .. })
        ));
        assert!(matches!(
            solve_dare_nxn_doubling(1, 1, &[1.0], &[1.0], &[1.0], &[0.0]),
            Err(LqrError::MatrixNotPositiveDefinite { label: "R" })
        ));
    }

    #[test]
    fn coupled_lqr_dense_solver_fails_closed_on_uncontrolled_unstable_mode() {
        let err = solve_dare_nxn_doubling_with_options(
            2,
            1,
            &[1.1, 0.0, 0.0, 0.9],
            &[0.0, 1.0],
            &[1.0, 0.0, 0.0, 1.0],
            &[1.0],
            32,
            1.0e-12,
        )
        .unwrap_err();
        assert!(matches!(
            err,
            LqrError::DareDidNotConverge { .. } | LqrError::LinearSolveFailed { .. }
        ));
    }

    #[test]
    fn lqg_observer_dual_dare_returns_transposed_dual_gain() {
        let a = [1.0, 0.1, 0.0, 0.98];
        let c = [1.0, 0.0];
        let w = [0.01, 0.0, 0.0, 0.02];
        let v = [0.05];
        let observer =
            solve_lqg_observer_nxn_dual_dare(2, 1, &a, &c, &w, &v).expect("observer DARE");
        let dual = solve_dare_nxn_doubling(2, 1, &[1.0, 0.0, 0.1, 0.98], &[1.0, 0.0], &w, &v)
            .expect("dual LQR DARE");
        assert_eq!(observer.p_row_major, dual.p_row_major);
        assert_eq!(
            observer.l_row_major,
            vec![dual.k_row_major[0], dual.k_row_major[1]]
        );
        assert!(observer.residual_norm < 1.0e-10);
    }

    #[test]
    fn lqg_observer_dual_dare_matches_tolerance_table() {
        let data: Value =
            toml::from_str(GAIN_SCHEDULE_LQG_TOLERANCE_TABLE).expect("LQG table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_gain_schedule_lqg_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let p_epsilon = toml_f64(tolerance, "absolute_observer_covariance");
        let l_epsilon = toml_f64(tolerance, "absolute_observer_gain");
        let residual_epsilon = toml_f64(tolerance, "residual_norm");
        let observer = toml_table(&data, "lqg_observer");
        let solution = solve_lqg_observer_nxn_dual_dare(
            toml_usize(observer, "states"),
            toml_usize(observer, "measurements"),
            &toml_f64_array(observer, "a_row_major"),
            &toml_f64_array(observer, "c_row_major"),
            &toml_f64_array(observer, "w_row_major"),
            &toml_f64_array(observer, "v_row_major"),
        )
        .expect("LQG observer DARE converges");

        for (actual, expected) in solution
            .p_row_major
            .iter()
            .zip(toml_f64_array(observer, "expected_p"))
        {
            assert!(
                (*actual - expected).abs() <= p_epsilon,
                "observer P actual {actual:e}, expected {expected:e}, epsilon {p_epsilon:e}"
            );
        }
        for (actual, expected) in solution
            .l_row_major
            .iter()
            .zip(toml_f64_array(observer, "expected_l"))
        {
            assert!(
                (*actual - expected).abs() <= l_epsilon,
                "observer L actual {actual:e}, expected {expected:e}, epsilon {l_epsilon:e}"
            );
        }
        assert!(solution.residual_norm <= residual_epsilon);
    }

    #[test]
    fn closed_loop_poles_are_strictly_inside_unit_circle() {
        let gains = nominal();
        let poles = closed_loop_pole_magnitudes(0.001, 1.0, gains);
        for (i, mag) in poles.iter().enumerate() {
            assert!(
                *mag < 1.0,
                "closed-loop pole {i} magnitude {mag} must be < 1 (stability)"
            );
        }
    }

    #[test]
    fn solver_rejects_non_positive_dt() {
        for dt in [0.0, -1.0e-3] {
            assert!(matches!(
                solve_lqr_rate_loop(dt, 1.0, 1.0, 0.1, 0.1),
                Err(LqrError::NonPositiveDt { .. })
            ));
        }
    }

    #[test]
    fn solver_rejects_non_positive_inertia() {
        for j in [0.0, -1.0] {
            assert!(matches!(
                solve_lqr_rate_loop(0.001, j, 1.0, 0.1, 0.1),
                Err(LqrError::NonPositiveInertia { .. })
            ));
        }
    }

    #[test]
    fn solver_rejects_non_positive_cost_weights() {
        // q_omega = 0.
        assert!(matches!(
            solve_lqr_rate_loop(0.001, 1.0, 0.0, 0.1, 0.1),
            Err(LqrError::NonPositiveCostWeight {
                label: "q_omega",
                ..
            })
        ));
        // q_int negative.
        assert!(matches!(
            solve_lqr_rate_loop(0.001, 1.0, 1.0, -0.1, 0.1),
            Err(LqrError::NonPositiveCostWeight { label: "q_int", .. })
        ));
        // r = 0.
        assert!(matches!(
            solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.1, 0.0),
            Err(LqrError::NonPositiveCostWeight { label: "r", .. })
        ));
    }

    #[test]
    fn solver_rejects_non_finite_parameters() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            assert!(matches!(
                solve_lqr_rate_loop(bad, 1.0, 1.0, 0.1, 0.1),
                Err(LqrError::NonFiniteParameter { label: "dt_s", .. })
            ));
            assert!(matches!(
                solve_lqr_rate_loop(0.001, bad, 1.0, 0.1, 0.1),
                Err(LqrError::NonFiniteParameter {
                    label: "inertia_kg_m2",
                    ..
                })
            ));
        }
    }

    #[test]
    fn larger_q_omega_yields_larger_proportional_gain() {
        let g_low = solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.1, 0.1).unwrap();
        let g_high = solve_lqr_rate_loop(0.001, 1.0, 100.0, 0.1, 0.1).unwrap();
        assert!(g_high.k_omega > g_low.k_omega);
    }

    #[test]
    fn smaller_r_yields_more_aggressive_gain() {
        let g_relaxed = solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.1, 1.0).unwrap();
        let g_aggressive = solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.1, 0.001).unwrap();
        assert!(g_aggressive.k_omega > g_relaxed.k_omega);
    }

    #[test]
    fn larger_q_int_yields_larger_integral_gain() {
        let g_low = solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.01, 0.1).unwrap();
        let g_high = solve_lqr_rate_loop(0.001, 1.0, 1.0, 1.0, 0.1).unwrap();
        assert!(g_high.k_int > g_low.k_int);
    }

    #[test]
    fn solver_is_bit_stable_across_reruns() {
        // The DARE iteration is deterministic; two solves with
        // identical inputs must produce identical bits. Asserts the
        // solver respects the byte-stability gate.
        let a = solve_lqr_rate_loop(0.001, 2.5, 0.4, 0.07, 0.13).unwrap();
        let b = solve_lqr_rate_loop(0.001, 2.5, 0.4, 0.07, 0.13).unwrap();
        assert_eq!(a, b);
    }

    /// Closed-loop simulation: with the LQR gain installed, a
    /// step rate command should be tracked with bounded error and
    /// no integrator runaway. Driven open-loop on the augmented
    /// system the test approximates an idealised plant — the e2e
    /// test exercises the same gain inside the full FC loop.
    #[test]
    fn closed_loop_step_response_settles_with_zero_steady_state_error() {
        let dt = 0.001;
        let inertia = 1.0;
        let g = solve_lqr_rate_loop(dt, inertia, 100.0, 1.0, 0.1).unwrap();
        let omega_ref = 1.0_f64;
        let mut omega = 0.0_f64;
        let mut e_int = 0.0_f64;
        for _ in 0..20_000 {
            let omega_err = omega - omega_ref;
            let u = -g.k_omega * omega_err - g.k_int * e_int;
            omega += dt * u / inertia;
            e_int += dt * omega_err;
        }
        // After 20 s of LQR closed-loop on a unit-inertia integrator
        // the rate has settled to within 0.1 % of the step command.
        assert_abs_diff_eq!(omega, omega_ref, epsilon = 1.0e-3);
    }

    /// Closed-loop simulation: under a constant matched torque
    /// disturbance, integral action drives steady-state rate error
    /// to zero (within the discretisation tolerance).
    #[test]
    fn closed_loop_rejects_constant_torque_disturbance() {
        let dt = 0.001;
        let inertia = 1.0;
        let g = solve_lqr_rate_loop(dt, inertia, 100.0, 1.0, 0.1).unwrap();
        let omega_ref = 0.0_f64;
        let disturbance_torque = 0.05_f64;
        let mut omega = 0.0_f64;
        let mut e_int = 0.0_f64;
        for _ in 0..5_000 {
            let omega_err = omega - omega_ref;
            let u = -g.k_omega * omega_err - g.k_int * e_int;
            omega += dt * (u + disturbance_torque) / inertia;
            e_int += dt * omega_err;
        }
        assert_abs_diff_eq!(omega, 0.0, epsilon = 1.0e-3);
    }

    #[test]
    fn pole_magnitudes_handle_complex_conjugate_pair() {
        // Construct a case whose closed-loop poles are complex.
        let gains = solve_lqr_rate_loop(0.001, 1.0, 1.0, 1.0, 1.0e-3).unwrap();
        let poles = closed_loop_pole_magnitudes(0.001, 1.0, gains);
        for mag in poles {
            assert!(mag.is_finite());
            assert!(mag < 1.0);
        }
    }
}
