#![allow(
    clippy::cast_precision_loss,
    clippy::missing_fields_in_debug,
    clippy::needless_range_loop
)]

//! Solver-backed model-predictive-control primitives.
//!
//! The MPC surface lives behind the `mpc` feature
//! with the single-step box QP `solve_attitude_box_qp`. On top of
//! that, a finite-horizon receding-horizon
//! attitude controller, [`RecedingHorizonAttitudeMpc`],
//! replaces the cascaded autopilot's attitude-loop P-controller with
//! an N-step quadratic program over the small-angle attitude-error
//! state. The implementation uses Clarabel with deterministic
//! settings (`verbose = false`, pinned tolerances, no time limit in
//! the solve contract) and is intended to run only inside scheduled
//! FC ticks.

use thiserror::Error;

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

/// Errors raised by [`RecedingHorizonAttitudeMpc::new`] and
/// [`RecedingHorizonAttitudeMpc::solve`].
#[derive(Copy, Clone, Debug, Error, PartialEq)]
pub enum AttitudeMpcError {
    /// Horizon length must be `>= 1`. A horizon of zero gives the
    /// trivial QP with no decision variables.
    #[error("attitude MPC horizon must be >= 1; got {horizon_n}")]
    NonPositiveHorizon {
        /// Offending horizon length.
        horizon_n: usize,
    },
    /// Horizon length exceeds the fixed-size per-solve template
    /// supported by this implementation.
    #[error("attitude MPC horizon must be <= {max_horizon}; got {horizon_n}")]
    HorizonTooLong {
        /// Offending horizon length.
        horizon_n: usize,
        /// Maximum supported horizon.
        max_horizon: usize,
    },
    /// Loop step must be `> 0`.
    #[error("attitude MPC dt_s must be > 0; got {dt_s}")]
    NonPositiveDt {
        /// Offending value.
        dt_s: f64,
    },
    /// One of the per-axis stage costs `q_x`, control costs `r_u`, or
    /// terminal costs `terminal_p` is non-positive (the diagonal
    /// Hessian must be strictly positive definite for a unique
    /// minimiser).
    #[error("attitude MPC cost weight must be > 0; got {value} for {label}")]
    NonPositiveCostWeight {
        /// Cost-weight name.
        label: &'static str,
        /// Offending value.
        value: f64,
    },
    /// One of the per-axis rate-command limits is non-positive.
    #[error("attitude MPC rate limit must be > 0; got {value} on axis {axis}")]
    NonPositiveRateLimit {
        /// Offending axis (0 = roll, 1 = pitch, 2 = yaw).
        axis: usize,
        /// Offending value.
        value: f64,
    },
    /// A parameter is NaN or infinite.
    #[error("attitude MPC parameter must be finite; got {value} for {label}")]
    NonFiniteParameter {
        /// Parameter name.
        label: &'static str,
        /// Offending value.
        value: f64,
    },
    /// Initial state passed to `solve` is non-finite.
    #[error("attitude MPC initial state must be finite on every axis")]
    NonFiniteInitialState,
    /// Clarabel failed to reach the `Solved` status.
    #[error("Clarabel did not solve the attitude MPC QP (status: {status:?})")]
    ClarabelFailed {
        /// Solver status reported by Clarabel.
        status: SolverStatus,
    },
}

/// Per-axis cost weights and box rate-command limits for the
/// receding-horizon attitude MPC.
///
/// The MPC's plant model is the small-angle attitude-error
/// integrator `x[k+1] = x[k] − dt · u[k]` per body axis, where `x`
/// is the attitude error (rad) and `u` is the commanded body rate
/// (rad/s). This is a command-level MPC: it does not model downstream
/// rate-loop lag, actuator saturation, or reference-attitude motion
/// across the horizon. Stage cost on axis `i` at horizon step `k`:
/// `q_x[i] · x[k]² + r_u[i] · u[k]²`. Terminal cost:
/// `terminal_p[i] · x[N]²`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeMpcParams {
    /// Number of horizon steps `N`. Decision variables are
    /// `[u[0], u[1], …, u[N − 1]]` per axis. Must be `>= 1`.
    pub horizon_n: usize,
    /// Per-axis stage cost on the attitude error. Must be `> 0`.
    pub q_x: [f64; 3],
    /// Per-axis stage cost on the commanded rate. Must be `> 0` so
    /// the QP is strongly convex.
    pub r_u: [f64; 3],
    /// Per-axis terminal cost on the final attitude error. Must be
    /// `> 0`.
    pub terminal_p: [f64; 3],
    /// Per-axis symmetric rate-command bound (rad/s). The rate
    /// command is constrained to `|u[k]| ≤ rate_limit_rad_s[axis]`
    /// for every horizon step.
    pub rate_limit_rad_s: [f64; 3],
}

impl AttitudeMpcParams {
    /// Validate parameters against the loop step `dt_s`.
    ///
    /// # Errors
    ///
    /// Returns the matching [`AttitudeMpcError`] variant when the
    /// horizon, `dt_s`, any cost weight, or any rate limit is
    /// non-positive, or when any parameter is non-finite.
    pub fn validate(&self, dt_s: f64) -> Result<(), AttitudeMpcError> {
        require_finite("dt_s", dt_s)?;
        if dt_s <= 0.0 {
            return Err(AttitudeMpcError::NonPositiveDt { dt_s });
        }
        if self.horizon_n == 0 {
            return Err(AttitudeMpcError::NonPositiveHorizon {
                horizon_n: self.horizon_n,
            });
        }
        for axis in 0..3 {
            require_finite_axis("q_x", self.q_x[axis])?;
            if self.q_x[axis] <= 0.0 {
                return Err(AttitudeMpcError::NonPositiveCostWeight {
                    label: "q_x",
                    value: self.q_x[axis],
                });
            }
            require_finite_axis("r_u", self.r_u[axis])?;
            if self.r_u[axis] <= 0.0 {
                return Err(AttitudeMpcError::NonPositiveCostWeight {
                    label: "r_u",
                    value: self.r_u[axis],
                });
            }
            require_finite_axis("terminal_p", self.terminal_p[axis])?;
            if self.terminal_p[axis] <= 0.0 {
                return Err(AttitudeMpcError::NonPositiveCostWeight {
                    label: "terminal_p",
                    value: self.terminal_p[axis],
                });
            }
            require_finite_axis("rate_limit_rad_s", self.rate_limit_rad_s[axis])?;
            if self.rate_limit_rad_s[axis] <= 0.0 {
                return Err(AttitudeMpcError::NonPositiveRateLimit {
                    axis,
                    value: self.rate_limit_rad_s[axis],
                });
            }
        }
        Ok(())
    }
}

fn require_finite(label: &'static str, value: f64) -> Result<(), AttitudeMpcError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(AttitudeMpcError::NonFiniteParameter { label, value })
    }
}

fn require_finite_axis(label: &'static str, value: f64) -> Result<(), AttitudeMpcError> {
    require_finite(label, value)
}

/// Receding-horizon attitude MPC.
///
/// Replaces the cascaded autopilot's attitude-loop P-controller
/// with an N-step quadratic program. The controller minimises a
/// quadratic stage + terminal cost over commanded body rates,
/// subject to per-axis box constraints, then publishes the first
/// step of the optimal sequence as the rate command consumed by the
/// downstream rate loop (PID / LQR / INDI / etc).
///
/// Decision variables (3·N): `[u_x[0], u_y[0], u_z[0], u_x[1], …,
/// u_y[N − 1], u_z[N − 1]]` — step-major and axis-interleaved. The
/// QP is built once at construction time using the deterministic
/// Clarabel settings shared with the rest of the MPC surface; only
/// the linear cost vector `q` (which depends on the current
/// attitude-error measurement `x[0]`) is rebuilt each `solve`.
///
/// The QP is block-diagonal in the three axes — the Forward-Euler
/// small-angle attitude-error dynamics decouple per axis under the
/// project's standing diagonal-inertia assumption. Solving the
/// joint QP rather than three separate ones keeps the call site
/// simple and lets future slices add inter-axis coupling without
/// changing the API.
///
/// # Determinism
///
/// Clarabel is configured with the pinned `deterministic_settings`
/// shared with `solve_attitude_box_qp`; two `solve`
/// calls with identical inputs produce bit-identical outputs on the
/// reference platform.
pub struct RecedingHorizonAttitudeMpc {
    params: AttitudeMpcParams,
    dt_s: f64,
    /// Stacked Hessian `P` (3N × 3N, block-diagonal in axes,
    /// symmetric, positive definite). Built once at construction.
    p_matrix: CscMatrix<f64>,
    /// Inequality constraint matrix `A` (6N × 3N): two box rows per
    /// decision variable for `u ≤ limit` and `−u ≤ limit`.
    a_matrix: CscMatrix<f64>,
    /// Inequality bounds `b` (6N elements). Constant.
    b_vec: Vec<f64>,
    /// Per-step linear-coefficient template, one row per axis. Used
    /// to build the per-solve `q` vector quickly.
    /// `linear_coefficient_template[axis][step] = q_x_axis · (N − 1
    /// − step) + terminal_p_axis`. Multiplied by `−2 · dt · x_0` at
    /// solve time.
    linear_coefficient_template: [[f64; 64]; 3],
}

impl std::fmt::Debug for RecedingHorizonAttitudeMpc {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("RecedingHorizonAttitudeMpc")
            .field("params", &self.params)
            .field("dt_s", &self.dt_s)
            .finish()
    }
}

/// Maximum supported MPC horizon for this first fixed-template
/// implementation. At the 1 kHz demo loop this is 64 ms of
/// command-level lookahead; longer real-time horizons should move the
/// template to heap storage and revisit solver reuse / sparsity.
pub const ATTITUDE_MPC_MAX_HORIZON: usize = 64;

impl RecedingHorizonAttitudeMpc {
    /// Build the MPC, pre-computing the static QP matrices.
    ///
    /// # Errors
    ///
    /// Returns the matching [`AttitudeMpcError`] when `params` fails
    /// validation or `params.horizon_n` exceeds
    /// [`ATTITUDE_MPC_MAX_HORIZON`].
    pub fn new(params: AttitudeMpcParams, dt_s: f64) -> Result<Self, AttitudeMpcError> {
        params.validate(dt_s)?;
        if params.horizon_n > ATTITUDE_MPC_MAX_HORIZON {
            return Err(AttitudeMpcError::HorizonTooLong {
                horizon_n: params.horizon_n,
                max_horizon: ATTITUDE_MPC_MAX_HORIZON,
            });
        }
        let n = params.horizon_n;
        let dim = 3 * n;

        // Build the block-diagonal Hessian P. For each axis the
        // condensed cost is
        //   J_axis = (1/2) u' P_axis u + q_axis' u + const
        // with
        //   P_axis = 2 · ( dt²·(Σ_{k=1..N-1} q_x · e_k e_k')
        //               + dt²·p · e_N e_N'
        //               + r_u · I )
        // where e_k ∈ R^N has its first k entries set to 1.
        // Equivalently:
        //   P_axis[i, j] = 2·( dt²·q_x · (N − 1 − max(i, j))
        //                   + dt²·p
        //                   + r_u · [i == j] )
        //
        // Stack the three blocks into a 3N × 3N matrix using the
        // step-major, axis-interleaved layout
        // `z = [u_x[0], u_y[0], u_z[0], u_x[1], …]`.
        let mut p_dense = vec![vec![0.0_f64; dim]; dim];
        for axis in 0..3 {
            let q_x = params.q_x[axis];
            let p_terminal = params.terminal_p[axis];
            let r_u = params.r_u[axis];
            for i in 0..n {
                for j in 0..n {
                    let m = i.max(j);
                    let stage_count = n - 1 - m; // q_x weight count
                    let mut entry = (q_x * stage_count as f64 + p_terminal) * dt_s * dt_s;
                    if i == j {
                        entry += r_u;
                    }
                    let (row, col) = (3 * i + axis, 3 * j + axis);
                    p_dense[row][col] = 2.0 * entry;
                }
            }
        }
        let p_matrix = csc_from_upper_triangle(&p_dense, dim);

        // Build the box-constraint matrix A and bounds b.
        // Each decision variable has two rows: `u ≤ limit` and
        // `−u ≤ limit`. Constraint rows are packed as
        //   row 2k:     z[k] ≤ limit_axis        (positive bound)
        //   row 2k + 1: −z[k] ≤ limit_axis       (negative bound)
        // for each variable index k = 3·step + axis.
        let mut a_rows = Vec::with_capacity(2 * dim);
        let mut a_cols = Vec::with_capacity(2 * dim);
        let mut a_vals = Vec::with_capacity(2 * dim);
        let mut b_vec = Vec::with_capacity(2 * dim);
        for var in 0..dim {
            let axis = var % 3;
            a_rows.push(2 * var);
            a_cols.push(var);
            a_vals.push(1.0);
            b_vec.push(params.rate_limit_rad_s[axis]);
            a_rows.push(2 * var + 1);
            a_cols.push(var);
            a_vals.push(-1.0);
            b_vec.push(params.rate_limit_rad_s[axis]);
        }
        let a_matrix = csc_from_triplets(2 * dim, dim, &a_rows, &a_cols, &a_vals);

        // Pre-compute the linear-coefficient template per axis. At
        // solve time, q[3·step + axis] = −2·dt·x_0[axis] · template.
        let mut linear_coefficient_template = [[0.0_f64; ATTITUDE_MPC_MAX_HORIZON]; 3];
        for axis in 0..3 {
            for step in 0..n {
                let stage_count = (n - 1 - step) as f64;
                linear_coefficient_template[axis][step] =
                    params.q_x[axis] * stage_count + params.terminal_p[axis];
            }
        }

        Ok(Self {
            params,
            dt_s,
            p_matrix,
            a_matrix,
            b_vec,
            linear_coefficient_template,
        })
    }

    /// Solve the QP for the given current attitude error and return
    /// the optimal first-step rate command per axis.
    ///
    /// # Errors
    ///
    /// Returns [`AttitudeMpcError::NonFiniteInitialState`] when any
    /// component of `x0_attitude_error_rad` is non-finite, or
    /// [`AttitudeMpcError::ClarabelFailed`] when the underlying QP
    /// solver does not reach `Solved`.
    pub fn solve(&self, x0_attitude_error_rad: [f64; 3]) -> Result<[f64; 3], AttitudeMpcError> {
        if !x0_attitude_error_rad.iter().all(|v| v.is_finite()) {
            return Err(AttitudeMpcError::NonFiniteInitialState);
        }
        let n = self.params.horizon_n;
        let dim = 3 * n;
        let mut q_vec = vec![0.0_f64; dim];
        for step in 0..n {
            for axis in 0..3 {
                q_vec[3 * step + axis] = -2.0
                    * self.dt_s
                    * x0_attitude_error_rad[axis]
                    * self.linear_coefficient_template[axis][step];
            }
        }
        let cones = [NonnegativeConeT(2 * dim)];
        let mut solver = DefaultSolver::new(
            &self.p_matrix,
            &q_vec,
            &self.a_matrix,
            &self.b_vec,
            &cones,
            deterministic_settings(),
        );
        solver.solve();
        if solver.solution.status != SolverStatus::Solved {
            return Err(AttitudeMpcError::ClarabelFailed {
                status: solver.solution.status,
            });
        }
        Ok([
            solver.solution.x[0],
            solver.solution.x[1],
            solver.solution.x[2],
        ])
    }

    /// Read-only access to the configured parameters.
    #[must_use]
    pub fn params(&self) -> &AttitudeMpcParams {
        &self.params
    }
}

/// Build a CSC matrix from a dense symmetric upper triangle. The
/// Clarabel API expects the upper triangle of `P` only.
fn csc_from_upper_triangle(p_dense: &[Vec<f64>], dim: usize) -> CscMatrix<f64> {
    let mut col_ptr = Vec::with_capacity(dim + 1);
    let mut row_idx = Vec::new();
    let mut vals = Vec::new();
    col_ptr.push(0);
    for col in 0..dim {
        for row in 0..=col {
            let v = p_dense[row][col];
            if v != 0.0 {
                row_idx.push(row);
                vals.push(v);
            }
        }
        col_ptr.push(row_idx.len());
    }
    CscMatrix::new(dim, dim, col_ptr, row_idx, vals)
}

/// Build a CSC matrix from triplet lists. Validates index ordering
/// by sorting per column.
fn csc_from_triplets(
    rows: usize,
    cols: usize,
    triplet_rows: &[usize],
    triplet_cols: &[usize],
    triplet_vals: &[f64],
) -> CscMatrix<f64> {
    assert_eq!(triplet_rows.len(), triplet_cols.len());
    assert_eq!(triplet_rows.len(), triplet_vals.len());
    let mut by_col: Vec<Vec<(usize, f64)>> = vec![Vec::new(); cols];
    for ((&r, &c), &v) in triplet_rows.iter().zip(triplet_cols).zip(triplet_vals) {
        by_col[c].push((r, v));
    }
    for entries in &mut by_col {
        entries.sort_by_key(|&(r, _)| r);
    }
    let mut col_ptr = Vec::with_capacity(cols + 1);
    let mut row_idx = Vec::new();
    let mut vals = Vec::new();
    col_ptr.push(0);
    for entries in &by_col {
        for &(r, v) in entries {
            row_idx.push(r);
            vals.push(v);
        }
        col_ptr.push(row_idx.len());
    }
    CscMatrix::new(rows, cols, col_ptr, row_idx, vals)
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

    /// Nominal-but-tuned-for-tests params. Cost balance picked so
    /// MPC saturates when the attitude error is large, drives `u_0`
    /// with the sign that reduces error under `x[k+1] = x[k] − dt*u[k]`,
    /// and converges the small-angle plant in closed loop within ≈ 100 ms.
    /// Production scenarios should re-tune; these weights are
    /// chosen for self-consistent unit-test assertions, not for
    /// typical real-world tracking.
    fn nominal_attitude_mpc_params() -> AttitudeMpcParams {
        AttitudeMpcParams {
            horizon_n: 20,
            q_x: [100.0, 100.0, 50.0],
            r_u: [0.001, 0.001, 0.001],
            terminal_p: [1_000.0, 1_000.0, 500.0],
            rate_limit_rad_s: [3.0, 3.0, 3.0],
        }
    }

    #[test]
    fn rh_attitude_mpc_constructs_with_nominal_params() {
        let mpc = RecedingHorizonAttitudeMpc::new(nominal_attitude_mpc_params(), 0.001);
        assert!(mpc.is_ok());
    }

    #[test]
    fn rh_attitude_mpc_validates_rejects_zero_horizon() {
        let mut p = nominal_attitude_mpc_params();
        p.horizon_n = 0;
        assert!(matches!(
            RecedingHorizonAttitudeMpc::new(p, 0.001),
            Err(AttitudeMpcError::NonPositiveHorizon { .. })
        ));
    }

    #[test]
    fn rh_attitude_mpc_validates_rejects_non_positive_dt() {
        for bad in [0.0, -1.0e-3] {
            assert!(matches!(
                RecedingHorizonAttitudeMpc::new(nominal_attitude_mpc_params(), bad),
                Err(AttitudeMpcError::NonPositiveDt { .. })
            ));
        }
    }

    #[test]
    fn rh_attitude_mpc_validates_rejects_non_positive_costs() {
        let mut p = nominal_attitude_mpc_params();
        p.q_x[1] = 0.0;
        assert!(matches!(
            RecedingHorizonAttitudeMpc::new(p, 0.001),
            Err(AttitudeMpcError::NonPositiveCostWeight { label: "q_x", .. })
        ));
        let mut p = nominal_attitude_mpc_params();
        p.r_u[2] = -1.0;
        assert!(matches!(
            RecedingHorizonAttitudeMpc::new(p, 0.001),
            Err(AttitudeMpcError::NonPositiveCostWeight { label: "r_u", .. })
        ));
        let mut p = nominal_attitude_mpc_params();
        p.terminal_p[0] = 0.0;
        assert!(matches!(
            RecedingHorizonAttitudeMpc::new(p, 0.001),
            Err(AttitudeMpcError::NonPositiveCostWeight {
                label: "terminal_p",
                ..
            })
        ));
    }

    #[test]
    fn rh_attitude_mpc_validates_rejects_non_positive_rate_limits() {
        let mut p = nominal_attitude_mpc_params();
        p.rate_limit_rad_s[1] = 0.0;
        assert!(matches!(
            RecedingHorizonAttitudeMpc::new(p, 0.001),
            Err(AttitudeMpcError::NonPositiveRateLimit { axis: 1, .. })
        ));
    }

    #[test]
    fn rh_attitude_mpc_validates_rejects_non_finite_parameters() {
        for bad in [f64::NAN, f64::INFINITY, f64::NEG_INFINITY] {
            let mut p = nominal_attitude_mpc_params();
            p.q_x[0] = bad;
            assert!(matches!(
                RecedingHorizonAttitudeMpc::new(p, 0.001),
                Err(AttitudeMpcError::NonFiniteParameter { .. })
            ));
        }
    }

    #[test]
    fn rh_attitude_mpc_solve_rejects_non_finite_initial_state() {
        let mpc = RecedingHorizonAttitudeMpc::new(nominal_attitude_mpc_params(), 0.001)
            .expect("nominal MPC builds");
        assert!(matches!(
            mpc.solve([f64::NAN, 0.0, 0.0]),
            Err(AttitudeMpcError::NonFiniteInitialState)
        ));
    }

    #[test]
    fn rh_attitude_mpc_solve_is_deterministic_across_reruns() {
        let mpc = RecedingHorizonAttitudeMpc::new(nominal_attitude_mpc_params(), 0.001)
            .expect("nominal MPC builds");
        let x0 = [0.05_f64, -0.02, 0.01];
        let a = mpc.solve(x0).expect("first solve");
        let b = mpc.solve(x0).expect("second solve");
        for (lhs, rhs) in a.iter().zip(b) {
            assert_eq!(lhs.to_bits(), rhs.to_bits());
        }
    }

    #[test]
    fn rh_attitude_mpc_zero_initial_state_returns_zero_command() {
        let mpc = RecedingHorizonAttitudeMpc::new(nominal_attitude_mpc_params(), 0.001)
            .expect("nominal MPC builds");
        let u0 = mpc.solve([0.0, 0.0, 0.0]).expect("solve");
        for v in u0 {
            assert!(v.abs() < 1.0e-9, "expected zero command, got {v}");
        }
    }

    #[test]
    fn rh_attitude_mpc_drives_rate_command_to_reduce_attitude_error() {
        // Positive attitude error → MPC should command positive rate
        // to drive x toward zero through the dynamics
        // x[k+1] = x[k] − dt · u[k].
        let mpc = RecedingHorizonAttitudeMpc::new(nominal_attitude_mpc_params(), 0.001)
            .expect("nominal MPC builds");
        let u0 = mpc.solve([0.1, -0.05, 0.0]).expect("solve");
        assert!(u0[0] > 0.0, "x_0 > 0 should drive u_0 > 0; got {}", u0[0]);
        assert!(u0[1] < 0.0, "x_1 < 0 should drive u_1 < 0; got {}", u0[1]);
        assert!(u0[2].abs() < 1.0e-6, "x_2 = 0 should drive u_2 ≈ 0");
    }

    #[test]
    fn rh_attitude_mpc_respects_rate_command_box() {
        // A large attitude error should drive the rate command to
        // its symmetric box bound on the affected axis.
        let mut p = nominal_attitude_mpc_params();
        p.rate_limit_rad_s = [0.2, 0.2, 0.2];
        let mpc = RecedingHorizonAttitudeMpc::new(p, 0.001).expect("MPC builds");
        let u0 = mpc.solve([1.0, 0.0, 0.0]).expect("solve");
        assert!(
            u0[0] >= 0.2 - 1.0e-6,
            "u_0 should saturate at +rate_limit; got {}",
            u0[0]
        );
        assert!(
            u0[0] <= 0.2 + 1.0e-6,
            "u_0 must not exceed rate_limit; got {}",
            u0[0]
        );
    }

    #[test]
    fn rh_attitude_mpc_step_response_settles_in_closed_loop() {
        // Run the MPC in closed loop on the small-angle plant
        // x[k+1] = x[k] − dt · u[k] (rate loop assumed perfect) and
        // confirm a non-trivial initial error converges.
        let dt = 0.001_f64;
        let params = nominal_attitude_mpc_params();
        let mpc = RecedingHorizonAttitudeMpc::new(params, dt).expect("MPC builds");
        let mut x = [0.1_f64, 0.05, -0.02];
        for _ in 0..2_000 {
            let u = mpc.solve(x).expect("solve");
            for axis in 0..3 {
                x[axis] -= dt * u[axis];
            }
        }
        for (axis, v) in x.iter().enumerate() {
            assert!(v.abs() < 1.0e-3, "axis {axis} did not converge: x = {v}");
        }
    }

    #[test]
    fn rh_attitude_mpc_horizon_too_long_rejected() {
        let mut p = nominal_attitude_mpc_params();
        p.horizon_n = ATTITUDE_MPC_MAX_HORIZON + 1;
        assert!(matches!(
            RecedingHorizonAttitudeMpc::new(p, 0.001),
            Err(AttitudeMpcError::HorizonTooLong {
                max_horizon: ATTITUDE_MPC_MAX_HORIZON,
                ..
            })
        ));
    }
}
