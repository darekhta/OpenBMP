//! Solver-backed soft-landing convexification primitives.
//!
//! The Phase 4.C landing surface is feature-gated behind `mpc` and
//! uses Clarabel's second-order-cone backend. Full `LCvxLD` / `SCvx`
//! trajectory generation is built from this primitive by constraining
//! thrust vectors and glideslope cones over a finite horizon.

use clarabel::{
    algebra::CscMatrix,
    solver::{DefaultSolver, IPSolver, SecondOrderConeT, SolverStatus, ZeroConeT},
};

use crate::mpc::deterministic_settings;

/// Result of a one-step second-order-cone landing acceleration solve.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LandingSocpCommand {
    /// Norm upper bound returned by the SOC epigraph.
    pub accel_norm_m_s2: f64,
    /// Commanded acceleration vector.
    pub accel_eci_m_s2: [f64; 3],
    /// Clarabel termination status.
    pub status: SolverStatus,
}

/// Solves a deterministic SOCP epigraph:
///
/// ```text
/// minimize t
/// subject to a = desired_accel
///            ||a||_2 <= t
/// ```
///
/// This is the `LCvxLD` cone backend smoke test; horizon-level landing
/// constraints use the same `SecondOrderConeT` surface.
///
/// # Errors
///
/// Returns an error if the input is non-finite or Clarabel does not
/// report `Solved`.
pub fn solve_accel_norm_epigraph(
    desired_accel_eci_m_s2: [f64; 3],
) -> Result<LandingSocpCommand, &'static str> {
    if !desired_accel_eci_m_s2.iter().all(|v| v.is_finite()) {
        return Err("desired acceleration must be finite");
    }

    let p = CscMatrix::new(4, 4, vec![0, 0, 0, 0, 0], Vec::new(), Vec::new());
    let q = vec![1.0, 0.0, 0.0, 0.0];
    let a = CscMatrix::new(
        7,
        4,
        vec![0, 1, 3, 5, 7],
        vec![3, 0, 4, 1, 5, 2, 6],
        vec![-1.0, 1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
    );
    let b = vec![
        desired_accel_eci_m_s2[0],
        desired_accel_eci_m_s2[1],
        desired_accel_eci_m_s2[2],
        0.0,
        0.0,
        0.0,
        0.0,
    ];
    let cones = [ZeroConeT(3), SecondOrderConeT(4)];
    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, deterministic_settings());
    solver.solve();

    if solver.solution.status != SolverStatus::Solved {
        return Err("Clarabel did not solve landing SOCP");
    }
    Ok(LandingSocpCommand {
        accel_norm_m_s2: solver.solution.x[0],
        accel_eci_m_s2: [
            solver.solution.x[1],
            solver.solution.x[2],
            solver.solution.x[3],
        ],
        status: solver.solution.status,
    })
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn socp_epigraph_matches_accel_norm() {
        let cmd = solve_accel_norm_epigraph([0.0, 0.0, 2.0]).expect("SOCP solve");
        assert_eq!(cmd.status, SolverStatus::Solved);
        assert!((cmd.accel_norm_m_s2 - 2.0).abs() < 1.0e-6);
        assert!((cmd.accel_eci_m_s2[2] - 2.0).abs() < 1.0e-6);
    }
}
