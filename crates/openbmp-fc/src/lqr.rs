//! Per-axis 2-state LQR for the rate loop (Phase 5.A.3.B).
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
//! is solved by fixed-point iteration. With `Q = diag(q_omega, q_int)`
//! and scalar `R = r`, the solver returns the LQR feedback gains
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
//! - Anderson B.D.O., Moore J.B. (1990). *Optimal Control: Linear
//!   Quadratic Methods*. Prentice-Hall.
//! - Lewis F.L., Vrabie D., Syrmos V.L. (2012). *Optimal Control*,
//!   3rd ed. Wiley.

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

    use super::*;

    fn nominal() -> LqrGains {
        solve_lqr_rate_loop(0.001, 1.0, 1.0, 0.1, 0.1).expect("nominal LQR converges")
    }

    #[test]
    fn dare_converges_for_nominal_params() {
        let gains = nominal();
        assert!(gains.k_omega > 0.0);
        assert!(gains.k_int > 0.0);
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
