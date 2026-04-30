//! Solver-backed model-predictive-control primitives.
//!
//! Phase 4.C reintroduces the MPC surface behind the `mpc` feature.
//! The implementation uses Clarabel with deterministic settings
//! (`verbose = false`, pinned tolerances, no time limit in the solve
//! contract) and is intended to run only inside scheduled FC ticks.

use clarabel::{
    algebra::CscMatrix,
    solver::{DefaultSettings, DefaultSolver, IPSolver, NonnegativeConeT, SolverStatus},
};

/// Clarabel settings type used by the FC MPC path.
pub type ClarabelSettings = DefaultSettings<f64>;

/// Small attitude-control QP used by the receding-horizon controller.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeMpcProblem {
    /// Desired body torque before actuator limits.
    pub desired_torque_body: [f64; 3],
    /// Symmetric torque limit applied to each axis.
    pub torque_limit: f64,
}

/// MPC torque command returned by Clarabel.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeMpcCommand {
    /// Limited body torque.
    pub torque_body: [f64; 3],
    /// Solver primal objective.
    pub objective: f64,
    /// Clarabel termination status.
    pub status: SolverStatus,
}

/// Returns the deterministic Clarabel settings profile used by FC
/// solver-backed jobs.
#[must_use]
pub fn deterministic_settings() -> ClarabelSettings {
    DefaultSettings::<f64> {
        verbose: false,
        max_iter: 100,
        time_limit: f64::INFINITY,
        tol_gap_abs: 1.0e-9,
        tol_gap_rel: 1.0e-9,
        tol_feas: 1.0e-9,
        tol_infeas_abs: 1.0e-9,
        tol_infeas_rel: 1.0e-9,
        tol_ktratio: 1.0e-9,
        ..DefaultSettings::<f64>::default()
    }
}

/// Solves the single-step attitude box-constrained QP:
///
/// ```text
/// minimize 0.5 * ||u - u_ref||^2
/// subject to -limit <= u_i <= limit
/// ```
///
/// # Errors
///
/// Returns an error if the problem is non-finite, has a non-positive
/// limit, or Clarabel does not report `Solved`.
pub fn solve_attitude_box_qp(
    problem: AttitudeMpcProblem,
) -> Result<AttitudeMpcCommand, &'static str> {
    if !problem.torque_limit.is_finite() || problem.torque_limit <= 0.0 {
        return Err("torque_limit must be positive and finite");
    }
    if !problem.desired_torque_body.iter().all(|v| v.is_finite()) {
        return Err("desired torque must be finite");
    }

    let p = CscMatrix::identity(3);
    let q = problem
        .desired_torque_body
        .iter()
        .map(|v| -*v)
        .collect::<Vec<_>>();

    // A = [I; -I], so A*u + s = b with s >= 0 encodes u <= limit
    // and -u <= limit.
    let a = CscMatrix::new(
        6,
        3,
        vec![0, 2, 4, 6],
        vec![0, 3, 1, 4, 2, 5],
        vec![1.0, -1.0, 1.0, -1.0, 1.0, -1.0],
    );
    let b = vec![problem.torque_limit; 6];
    let cones = [NonnegativeConeT(6)];
    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, deterministic_settings());
    solver.solve();

    if solver.solution.status != SolverStatus::Solved {
        return Err("Clarabel did not solve attitude MPC QP");
    }
    Ok(AttitudeMpcCommand {
        torque_body: [
            solver.solution.x[0],
            solver.solution.x[1],
            solver.solution.x[2],
        ],
        objective: solver.solution.obj_val,
        status: solver.solution.status,
    })
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;

    #[test]
    fn attitude_box_qp_is_deterministic() {
        let problem = AttitudeMpcProblem {
            desired_torque_body: [2.0, -0.5, 0.25],
            torque_limit: 1.0,
        };
        let first = solve_attitude_box_qp(problem).expect("first solve");
        let second = solve_attitude_box_qp(problem).expect("second solve");
        assert_eq!(first.status, SolverStatus::Solved);
        for (a, b) in first.torque_body.iter().zip(second.torque_body) {
            assert!((*a - b).abs() < f64::EPSILON);
        }
        assert!((first.torque_body[0] - 1.0).abs() < 1.0e-6);
        assert!((first.torque_body[1] + 0.5).abs() < 1.0e-6);
    }
}
