//! Lossless-convex powered-descent SOCP substrate.
//!
//! This module is feature-gated behind `sqp` because it uses Clarabel's
//! second-order-cone backend. The first surface is a fixed-time, local-frame
//! point-mass powered-descent-to-state problem with log-mass depletion,
//! thrust epigraph cones, and optional glideslope / pointing cones.

use alloc::vec;
use alloc::vec::Vec;
use core::f64::consts::FRAC_PI_2;

use clarabel::{
    algebra::CscMatrix,
    solver::{
        DefaultSolver, IPSolver, NonnegativeConeT, SecondOrderConeT, SolverStatus, SupportedConeT,
        ZeroConeT,
    },
};

use crate::iload::TrajoptError;
use crate::sqp::deterministic_sqp_settings;

const STATE_STRIDE: usize = 7;
const CONTROL_STRIDE: usize = 4;
// Positive objective scaling preserves the optimizer but improves numerical
// pressure on sigma in large published-case SOCPs.
const LCVX_OBJECTIVE_SCALE: f64 = 3.0e3;
const LCVX_TAYLOR_UPPER_BOUND_ROW_SCALE: f64 = 1.0e3;
const STANDARD_GRAVITY_M_S2: f64 = 9.806_65;

/// Terminal state for an LCvx powered-descent solve.
///
/// The target is expressed in the caller's local Cartesian state frame. It
/// intentionally carries no latitude, longitude, range, aimpoint, or other
/// surface-coordinate targeting vocabulary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LcvxTerminalStateTarget {
    /// Terminal local-frame position `[x, y, up]` in meters.
    pub position_m: [f64; 3],
    /// Terminal local-frame velocity in meters per second.
    pub velocity_m_s: [f64; 3],
}

/// Thrust-control discretization used by the fixed-time LCvx transcription.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LcvxControlDiscretization {
    /// One thrust vector and sigma value per interval.
    ZeroOrderHold,
    /// One thrust vector and sigma value per node with FOH interval dynamics.
    FirstOrderHold,
}

/// Fixed-time LCvx powered-descent-to-state problem.
#[derive(Clone, Debug, PartialEq)]
pub struct LcvxPoweredDescentProblem {
    /// Initial local-frame position `[x, y, up]` in meters.
    pub initial_position_m: [f64; 3],
    /// Initial local-frame velocity in meters per second.
    pub initial_velocity_m_s: [f64; 3],
    /// Initial vehicle mass in kilograms.
    pub initial_mass_kg: f64,
    /// Terminal state target.
    pub target: LcvxTerminalStateTarget,
    /// Constant local-frame gravity acceleration in meters per second squared.
    pub gravity_m_s2: [f64; 3],
    /// Number of fixed integration intervals.
    pub horizon_steps: usize,
    /// Fixed time of flight in seconds.
    pub time_of_flight_s: f64,
    /// Control discretization used by thrust and sigma variables.
    pub control_discretization: LcvxControlDiscretization,
    /// Specific impulse used by `z_dot = -sigma / (Isp * g0)`.
    pub specific_impulse_s: f64,
    /// Lower bound on the relaxed thrust-acceleration norm `sigma`.
    pub min_thrust_accel_m_s2: f64,
    /// Upper bound on the relaxed thrust-acceleration norm `sigma`.
    pub max_thrust_accel_m_s2: f64,
    /// Optional mass-log Taylor approximation of physical thrust-force bounds.
    ///
    /// When present, the fixed acceleration bounds are replaced by the
    /// Acikmese-Ploen-style Taylor force-bound schedule in sigma/log-mass
    /// coordinates.
    pub taylor_thrust_bounds: Option<LcvxTaylorThrustBounds>,
    /// Optional glideslope half-angle from the local vertical, in radians.
    pub glideslope_angle_rad: Option<f64>,
    /// Optional thrust pointing half-angle from the local vertical, in radians.
    pub max_pointing_angle_rad: Option<f64>,
}

/// Physical thrust-force bounds used for mass-log Taylor LCvx constraints.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LcvxTaylorThrustBounds {
    /// Minimum physical thrust force in newtons.
    pub min_thrust_n: f64,
    /// Maximum physical thrust force in newtons.
    pub max_thrust_n: f64,
}

/// Deterministic LCvx powered-descent solution.
#[derive(Clone, Debug, PartialEq)]
pub struct LcvxPoweredDescentSolution {
    /// Clarabel solver status.
    pub status: SolverStatus,
    /// Clarabel primal objective, minimizing `-z_final`.
    pub objective: f64,
    /// Fixed step size in seconds.
    pub time_step_s: f64,
    /// Raw Clarabel primal vector for deterministic repeat evidence.
    pub primal: Vec<f64>,
    /// Local-frame node positions.
    pub positions_m: Vec<[f64; 3]>,
    /// Local-frame node velocities.
    pub velocities_m_s: Vec<[f64; 3]>,
    /// Node log-masses.
    pub log_masses: Vec<f64>,
    /// Node masses in kilograms.
    pub masses_kg: Vec<f64>,
    /// Thrust accelerations at the active control samples.
    pub thrust_accel_m_s2: Vec<[f64; 3]>,
    /// Relaxed thrust-acceleration norms at the active control samples.
    pub sigma_m_s2: Vec<f64>,
    /// Final mass in kilograms.
    pub final_mass_kg: f64,
    /// Maximum `|sigma - ||u||||` lossless-relaxation gap.
    pub max_lossless_gap_m_s2: f64,
    /// Maximum glideslope inequality violation, in meters.
    pub max_glideslope_violation_m: f64,
    /// Maximum pointing-cone violation, in meters per second squared.
    pub max_pointing_violation_m_s2: f64,
    /// Maximum thrust-bound violation, in meters per second squared.
    pub max_thrust_bound_violation_m_s2: f64,
    /// Terminal position error relative to the target.
    pub terminal_position_error_m: [f64; 3],
    /// Terminal velocity error relative to the target.
    pub terminal_velocity_error_m_s: [f64; 3],
}

/// Deterministic golden-section time-of-flight search settings for an LCvx
/// powered-descent solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LcvxTimeOfFlightSearchSettings {
    /// Inclusive lower time-of-flight bound in seconds.
    pub lower_time_of_flight_s: f64,
    /// Inclusive upper time-of-flight bound in seconds.
    pub upper_time_of_flight_s: f64,
    /// Stop once the bracket width is no larger than this tolerance.
    pub tolerance_s: f64,
    /// Maximum number of inner fixed-time SOCP iterations.
    pub max_iterations: usize,
}

/// Result of deterministic LCvx free-final-time search.
#[derive(Clone, Debug, PartialEq)]
pub struct LcvxTimeOfFlightSearchReport {
    /// Best fixed-time LCvx solution found by the search.
    pub solution: LcvxPoweredDescentSolution,
    /// Time of flight associated with `solution`.
    pub best_time_of_flight_s: f64,
    /// Final lower bracket bound in seconds.
    pub final_lower_time_of_flight_s: f64,
    /// Final upper bracket bound in seconds.
    pub final_upper_time_of_flight_s: f64,
    /// Number of golden-section iterations completed.
    pub iterations: usize,
    /// Number of fixed-time SOCP candidate solves performed.
    pub candidate_count: usize,
}

/// Solve a fixed-time LCvx powered-descent-to-state SOCP.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the problem is malformed or Clarabel does not
/// report [`SolverStatus::Solved`].
pub fn solve_lcvx_powered_descent_fixed_time(
    problem: &LcvxPoweredDescentProblem,
) -> Result<LcvxPoweredDescentSolution, TrajoptError> {
    validate_lcvx_problem(problem)?;

    let layout = LcvxDecisionLayout::new(problem.horizon_steps, problem.control_discretization);
    let dt = problem.time_of_flight_s / problem.horizon_steps as f64;
    let (a, b, cones) = assemble_lcvx_constraints(problem, &layout, dt);
    let p = CscMatrix::new(
        layout.variable_count,
        layout.variable_count,
        vec![0; layout.variable_count + 1],
        Vec::new(),
        Vec::new(),
    );
    let mut q = vec![0.0; layout.variable_count];
    q[layout.log_mass(problem.horizon_steps)] = -LCVX_OBJECTIVE_SCALE;

    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, deterministic_sqp_settings());
    solver.solve();
    if solver.solution.status != SolverStatus::Solved {
        return Err(TrajoptError::InvalidPayload {
            reason: "Clarabel did not solve LCvx powered-descent SOCP",
        });
    }

    solution_from_primal(
        problem,
        &layout,
        dt,
        solver.solution.obj_val / LCVX_OBJECTIVE_SCALE,
        solver.solution.x,
    )
}

/// Solve a free-final-time LCvx powered-descent-to-state problem by applying a
/// deterministic golden-section search over fixed-time SOCP solves.
///
/// The inner problem is identical to [`solve_lcvx_powered_descent_fixed_time`]
/// except for `time_of_flight_s`. The outer objective is the inner Clarabel
/// objective, `-z_final`, so lower values retain more final mass.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the base problem or search settings are
/// malformed, or when any inner fixed-time SOCP solve fails.
pub fn solve_lcvx_powered_descent_time_of_flight_search(
    base_problem: &LcvxPoweredDescentProblem,
    settings: LcvxTimeOfFlightSearchSettings,
) -> Result<LcvxTimeOfFlightSearchReport, TrajoptError> {
    validate_lcvx_problem(base_problem)?;
    validate_lcvx_time_search_settings(settings)?;

    let inv_phi = (5.0_f64.sqrt() - 1.0) * 0.5;
    let inv_phi_sq = 1.0 - inv_phi;
    let mut lower = settings.lower_time_of_flight_s;
    let mut upper = settings.upper_time_of_flight_s;
    let mut left_time = lower + inv_phi_sq * (upper - lower);
    let mut right_time = lower + inv_phi * (upper - lower);
    let mut left = solve_lcvx_with_time_of_flight(base_problem, left_time)?;
    let mut right = solve_lcvx_with_time_of_flight(base_problem, right_time)?;
    let mut candidate_count = 2;
    let mut iterations = 0;
    let mut best_time = if left.objective <= right.objective {
        left_time
    } else {
        right_time
    };
    let mut best = if left.objective <= right.objective {
        left.clone()
    } else {
        right.clone()
    };

    while iterations < settings.max_iterations && (upper - lower) > settings.tolerance_s {
        if left.objective <= right.objective {
            upper = right_time;
            right_time = left_time;
            right = left;
            left_time = lower + inv_phi_sq * (upper - lower);
            left = solve_lcvx_with_time_of_flight(base_problem, left_time)?;
            candidate_count += 1;
        } else {
            lower = left_time;
            left_time = right_time;
            left = right;
            right_time = lower + inv_phi * (upper - lower);
            right = solve_lcvx_with_time_of_flight(base_problem, right_time)?;
            candidate_count += 1;
        }
        iterations += 1;

        let (candidate_time, candidate) = if left.objective <= right.objective {
            (left_time, &left)
        } else {
            (right_time, &right)
        };
        if candidate.objective < best.objective
            || (candidate.objective.to_bits() == best.objective.to_bits()
                && candidate_time < best_time)
        {
            best_time = candidate_time;
            best = candidate.clone();
        }
    }

    Ok(LcvxTimeOfFlightSearchReport {
        solution: best,
        best_time_of_flight_s: best_time,
        final_lower_time_of_flight_s: lower,
        final_upper_time_of_flight_s: upper,
        iterations,
        candidate_count,
    })
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct LcvxDecisionLayout {
    horizon_steps: usize,
    node_count: usize,
    control_discretization: LcvxControlDiscretization,
    control_sample_count: usize,
    control_offset: usize,
    variable_count: usize,
}

impl LcvxDecisionLayout {
    fn new(horizon_steps: usize, control_discretization: LcvxControlDiscretization) -> Self {
        let node_count = horizon_steps + 1;
        let control_sample_count = match control_discretization {
            LcvxControlDiscretization::ZeroOrderHold => horizon_steps,
            LcvxControlDiscretization::FirstOrderHold => node_count,
        };
        let control_offset = node_count * STATE_STRIDE;
        let variable_count = control_offset + control_sample_count * CONTROL_STRIDE;
        Self {
            horizon_steps,
            node_count,
            control_discretization,
            control_sample_count,
            control_offset,
            variable_count,
        }
    }

    fn position(&self, node: usize, axis: usize) -> usize {
        node * STATE_STRIDE + axis
    }

    fn velocity(&self, node: usize, axis: usize) -> usize {
        node * STATE_STRIDE + 3 + axis
    }

    fn log_mass(&self, node: usize) -> usize {
        node * STATE_STRIDE + 6
    }

    fn thrust(&self, interval: usize, axis: usize) -> usize {
        self.control_offset + interval * CONTROL_STRIDE + axis
    }

    fn sigma(&self, interval: usize) -> usize {
        self.control_offset + interval * CONTROL_STRIDE + 3
    }

    fn right_control_sample(&self, interval: usize) -> usize {
        match self.control_discretization {
            LcvxControlDiscretization::ZeroOrderHold => interval,
            LcvxControlDiscretization::FirstOrderHold => interval + 1,
        }
    }
}

fn validate_lcvx_problem(problem: &LcvxPoweredDescentProblem) -> Result<(), TrajoptError> {
    if problem.horizon_steps == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx horizon_steps must be >= 1",
        });
    }
    validate_finite(
        "LCvx initial position must be finite",
        &problem.initial_position_m,
    )?;
    validate_finite(
        "LCvx initial velocity must be finite",
        &problem.initial_velocity_m_s,
    )?;
    validate_finite(
        "LCvx target position must be finite",
        &problem.target.position_m,
    )?;
    validate_finite(
        "LCvx target velocity must be finite",
        &problem.target.velocity_m_s,
    )?;
    validate_finite("LCvx gravity must be finite", &problem.gravity_m_s2)?;
    validate_scalar(
        "LCvx initial_mass_kg must be positive and finite",
        problem.initial_mass_kg,
        |value| value > 0.0,
    )?;
    validate_scalar(
        "LCvx time_of_flight_s must be positive and finite",
        problem.time_of_flight_s,
        |value| value > 0.0,
    )?;
    validate_scalar(
        "LCvx specific_impulse_s must be positive and finite",
        problem.specific_impulse_s,
        |value| value > 0.0,
    )?;
    validate_scalar(
        "LCvx min_thrust_accel_m_s2 must be non-negative and finite",
        problem.min_thrust_accel_m_s2,
        |value| value >= 0.0,
    )?;
    validate_scalar(
        "LCvx max_thrust_accel_m_s2 must be positive and finite",
        problem.max_thrust_accel_m_s2,
        |value| value > 0.0,
    )?;
    if problem.min_thrust_accel_m_s2 > problem.max_thrust_accel_m_s2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx thrust acceleration bounds are inverted",
        });
    }
    if let Some(bounds) = problem.taylor_thrust_bounds {
        validate_lcvx_taylor_thrust_bounds(problem, bounds)?;
    }
    validate_optional_angle(
        "LCvx glideslope_angle_rad must be in (0, pi/2)",
        problem.glideslope_angle_rad,
    )?;
    validate_optional_angle(
        "LCvx max_pointing_angle_rad must be in (0, pi/2)",
        problem.max_pointing_angle_rad,
    )?;
    Ok(())
}

fn validate_lcvx_taylor_thrust_bounds(
    problem: &LcvxPoweredDescentProblem,
    bounds: LcvxTaylorThrustBounds,
) -> Result<(), TrajoptError> {
    validate_scalar(
        "LCvx min_thrust_n must be non-negative and finite",
        bounds.min_thrust_n,
        |value| value >= 0.0,
    )?;
    validate_scalar(
        "LCvx max_thrust_n must be positive and finite",
        bounds.max_thrust_n,
        |value| value > 0.0,
    )?;
    if bounds.min_thrust_n > bounds.max_thrust_n {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx Taylor thrust force bounds are inverted",
        });
    }
    let alpha = 1.0 / (problem.specific_impulse_s * STANDARD_GRAVITY_M_S2);
    let final_reference_mass =
        problem.initial_mass_kg - alpha * bounds.max_thrust_n * problem.time_of_flight_s;
    if !final_reference_mass.is_finite() || final_reference_mass <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx Taylor thrust reference mass must stay positive",
        });
    }
    Ok(())
}

fn validate_lcvx_time_search_settings(
    settings: LcvxTimeOfFlightSearchSettings,
) -> Result<(), TrajoptError> {
    validate_scalar(
        "LCvx lower_time_of_flight_s must be positive and finite",
        settings.lower_time_of_flight_s,
        |value| value > 0.0,
    )?;
    validate_scalar(
        "LCvx upper_time_of_flight_s must be positive and finite",
        settings.upper_time_of_flight_s,
        |value| value > 0.0,
    )?;
    if settings.lower_time_of_flight_s >= settings.upper_time_of_flight_s {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx time-of-flight search bounds must be increasing",
        });
    }
    validate_scalar(
        "LCvx time-of-flight search tolerance_s must be positive and finite",
        settings.tolerance_s,
        |value| value > 0.0,
    )?;
    if settings.max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx time-of-flight search max_iterations must be >= 1",
        });
    }
    Ok(())
}

fn solve_lcvx_with_time_of_flight(
    base_problem: &LcvxPoweredDescentProblem,
    time_of_flight_s: f64,
) -> Result<LcvxPoweredDescentSolution, TrajoptError> {
    let mut problem = base_problem.clone();
    problem.time_of_flight_s = time_of_flight_s;
    solve_lcvx_powered_descent_fixed_time(&problem)
}

fn assemble_lcvx_constraints(
    problem: &LcvxPoweredDescentProblem,
    layout: &LcvxDecisionLayout,
    dt: f64,
) -> (CscMatrix<f64>, Vec<f64>, Vec<SupportedConeT<f64>>) {
    let mut builder = CscBuilder::new(layout.variable_count);
    let mut rhs = Vec::new();
    let mut cones = Vec::new();

    let equality_rows = append_lcvx_equalities(problem, layout, dt, &mut builder, &mut rhs);
    cones.push(ZeroConeT(equality_rows));

    let inequality_rows = append_lcvx_thrust_bounds(problem, layout, dt, &mut builder, &mut rhs);
    if inequality_rows > 0 {
        cones.push(NonnegativeConeT(inequality_rows));
    }

    if let Some(bounds) = problem.taylor_thrust_bounds {
        append_lcvx_taylor_lower_thrust_cones(
            bounds,
            problem,
            layout,
            dt,
            &mut builder,
            &mut rhs,
            &mut cones,
        );
    }
    append_lcvx_thrust_cones(layout, &mut builder, &mut rhs, &mut cones);
    if let Some(angle_rad) = problem.max_pointing_angle_rad {
        append_lcvx_pointing_cones(angle_rad, layout, &mut builder, &mut rhs, &mut cones);
    }
    if let Some(angle_rad) = problem.glideslope_angle_rad {
        append_lcvx_glideslope_cones(
            problem,
            angle_rad,
            layout,
            &mut builder,
            &mut rhs,
            &mut cones,
        );
    }

    (builder.finish(), rhs, cones)
}

fn append_lcvx_equalities(
    problem: &LcvxPoweredDescentProblem,
    layout: &LcvxDecisionLayout,
    dt: f64,
    builder: &mut CscBuilder,
    rhs: &mut Vec<f64>,
) -> usize {
    let start_row = builder.row_count();
    for axis in 0..3 {
        builder.push_row(&[(layout.position(0, axis), 1.0)]);
        rhs.push(problem.initial_position_m[axis]);
    }
    for axis in 0..3 {
        builder.push_row(&[(layout.velocity(0, axis), 1.0)]);
        rhs.push(problem.initial_velocity_m_s[axis]);
    }
    builder.push_row(&[(layout.log_mass(0), 1.0)]);
    rhs.push(problem.initial_mass_kg.ln());

    let terminal_node = layout.horizon_steps;
    for axis in 0..3 {
        builder.push_row(&[(layout.position(terminal_node, axis), 1.0)]);
        rhs.push(problem.target.position_m[axis]);
    }
    for axis in 0..3 {
        builder.push_row(&[(layout.velocity(terminal_node, axis), 1.0)]);
        rhs.push(problem.target.velocity_m_s[axis]);
    }

    let half_dt_sq = 0.5 * dt * dt;
    let mass_depletion_scale = dt / (problem.specific_impulse_s * STANDARD_GRAVITY_M_S2);
    for interval in 0..layout.horizon_steps {
        let next = interval + 1;
        match layout.control_discretization {
            LcvxControlDiscretization::ZeroOrderHold => {
                for axis in 0..3 {
                    builder.push_row(&[
                        (layout.position(next, axis), 1.0),
                        (layout.position(interval, axis), -1.0),
                        (layout.velocity(interval, axis), -dt),
                        (layout.thrust(interval, axis), -half_dt_sq),
                    ]);
                    rhs.push(half_dt_sq * problem.gravity_m_s2[axis]);
                }
                for axis in 0..3 {
                    builder.push_row(&[
                        (layout.velocity(next, axis), 1.0),
                        (layout.velocity(interval, axis), -1.0),
                        (layout.thrust(interval, axis), -dt),
                    ]);
                    rhs.push(dt * problem.gravity_m_s2[axis]);
                }
                builder.push_row(&[
                    (layout.log_mass(next), 1.0),
                    (layout.log_mass(interval), -1.0),
                    (layout.sigma(interval), mass_depletion_scale),
                ]);
                rhs.push(0.0);
            }
            LcvxControlDiscretization::FirstOrderHold => {
                let left = interval;
                let right = layout.right_control_sample(interval);
                for axis in 0..3 {
                    builder.push_row(&[
                        (layout.position(next, axis), 1.0),
                        (layout.position(interval, axis), -1.0),
                        (layout.velocity(interval, axis), -dt),
                        (layout.thrust(left, axis), -(dt * dt / 3.0)),
                        (layout.thrust(right, axis), -(dt * dt / 6.0)),
                    ]);
                    rhs.push(half_dt_sq * problem.gravity_m_s2[axis]);
                }
                for axis in 0..3 {
                    builder.push_row(&[
                        (layout.velocity(next, axis), 1.0),
                        (layout.velocity(interval, axis), -1.0),
                        (layout.thrust(left, axis), -0.5 * dt),
                        (layout.thrust(right, axis), -0.5 * dt),
                    ]);
                    rhs.push(dt * problem.gravity_m_s2[axis]);
                }
                builder.push_row(&[
                    (layout.log_mass(next), 1.0),
                    (layout.log_mass(interval), -1.0),
                    (layout.sigma(left), 0.5 * mass_depletion_scale),
                    (layout.sigma(right), 0.5 * mass_depletion_scale),
                ]);
                rhs.push(0.0);
            }
        }
    }
    builder.row_count() - start_row
}

fn append_lcvx_thrust_bounds(
    problem: &LcvxPoweredDescentProblem,
    layout: &LcvxDecisionLayout,
    dt: f64,
    builder: &mut CscBuilder,
    rhs: &mut Vec<f64>,
) -> usize {
    let start_row = builder.row_count();
    for sample in 0..layout.control_sample_count {
        if let Some(bounds) = problem.taylor_thrust_bounds {
            let (_, upper) = lcvx_taylor_thrust_bounds_for_sample(bounds, problem, sample, dt);
            builder.push_row(&[
                (layout.sigma(sample), LCVX_TAYLOR_UPPER_BOUND_ROW_SCALE),
                (
                    layout.log_mass(sample),
                    upper * LCVX_TAYLOR_UPPER_BOUND_ROW_SCALE,
                ),
            ]);
            rhs.push(
                upper
                    * (1.0 + lcvx_taylor_reference_log_mass(bounds, problem, sample, dt))
                    * LCVX_TAYLOR_UPPER_BOUND_ROW_SCALE,
            );
        } else {
            builder.push_row(&[(layout.sigma(sample), 1.0)]);
            rhs.push(problem.max_thrust_accel_m_s2);
            builder.push_row(&[(layout.sigma(sample), -1.0)]);
            rhs.push(-problem.min_thrust_accel_m_s2);
        }
    }
    builder.row_count() - start_row
}

fn append_lcvx_taylor_lower_thrust_cones(
    bounds: LcvxTaylorThrustBounds,
    problem: &LcvxPoweredDescentProblem,
    layout: &LcvxDecisionLayout,
    dt: f64,
    builder: &mut CscBuilder,
    rhs: &mut Vec<f64>,
    cones: &mut Vec<SupportedConeT<f64>>,
) {
    if bounds.min_thrust_n == 0.0 {
        return;
    }
    for sample in 0..layout.control_sample_count {
        let (lower, _) = lcvx_taylor_thrust_bounds_for_sample(bounds, problem, sample, dt);
        let z0 = lcvx_taylor_reference_log_mass(bounds, problem, sample, dt);
        let lower_sqrt = (2.0 * lower).sqrt();
        let y_constant = -lower * (1.0 + z0);
        builder.push_row(&[
            (layout.sigma(sample), -1.0),
            (layout.log_mass(sample), -lower),
        ]);
        rhs.push(1.0 + y_constant);
        builder.push_row(&[(layout.log_mass(sample), -lower_sqrt)]);
        rhs.push(-lower_sqrt * z0);
        builder.push_row(&[
            (layout.sigma(sample), -1.0),
            (layout.log_mass(sample), -lower),
        ]);
        rhs.push(y_constant - 1.0);
        cones.push(SecondOrderConeT(3));
    }
}

fn lcvx_taylor_thrust_bounds_for_sample(
    bounds: LcvxTaylorThrustBounds,
    problem: &LcvxPoweredDescentProblem,
    sample: usize,
    dt: f64,
) -> (f64, f64) {
    let z0 = lcvx_taylor_reference_log_mass(bounds, problem, sample, dt);
    (
        (bounds.min_thrust_n * (-z0).exp()),
        (bounds.max_thrust_n * (-z0).exp()),
    )
}

fn lcvx_taylor_reference_log_mass(
    bounds: LcvxTaylorThrustBounds,
    problem: &LcvxPoweredDescentProblem,
    sample: usize,
    dt: f64,
) -> f64 {
    let alpha = 1.0 / (problem.specific_impulse_s * STANDARD_GRAVITY_M_S2);
    let reference_mass = problem.initial_mass_kg - alpha * bounds.max_thrust_n * sample as f64 * dt;
    reference_mass.ln()
}

fn append_lcvx_thrust_cones(
    layout: &LcvxDecisionLayout,
    builder: &mut CscBuilder,
    rhs: &mut Vec<f64>,
    cones: &mut Vec<SupportedConeT<f64>>,
) {
    for sample in 0..layout.control_sample_count {
        builder.push_row(&[(layout.sigma(sample), -1.0)]);
        rhs.push(0.0);
        for axis in 0..3 {
            builder.push_row(&[(layout.thrust(sample, axis), -1.0)]);
            rhs.push(0.0);
        }
        cones.push(SecondOrderConeT(4));
    }
}

fn append_lcvx_pointing_cones(
    angle_rad: f64,
    layout: &LcvxDecisionLayout,
    builder: &mut CscBuilder,
    rhs: &mut Vec<f64>,
    cones: &mut Vec<SupportedConeT<f64>>,
) {
    let tangent = angle_rad.tan();
    for sample in 0..layout.control_sample_count {
        builder.push_row(&[(layout.thrust(sample, 2), -tangent)]);
        rhs.push(0.0);
        builder.push_row(&[(layout.thrust(sample, 0), -1.0)]);
        rhs.push(0.0);
        builder.push_row(&[(layout.thrust(sample, 1), -1.0)]);
        rhs.push(0.0);
        cones.push(SecondOrderConeT(3));
    }
}

fn append_lcvx_glideslope_cones(
    problem: &LcvxPoweredDescentProblem,
    angle_rad: f64,
    layout: &LcvxDecisionLayout,
    builder: &mut CscBuilder,
    rhs: &mut Vec<f64>,
    cones: &mut Vec<SupportedConeT<f64>>,
) {
    let tangent = angle_rad.tan();
    for node in 0..layout.node_count {
        builder.push_row(&[(layout.position(node, 2), -tangent)]);
        rhs.push(-tangent * problem.target.position_m[2]);
        builder.push_row(&[(layout.position(node, 0), -1.0)]);
        rhs.push(-problem.target.position_m[0]);
        builder.push_row(&[(layout.position(node, 1), -1.0)]);
        rhs.push(-problem.target.position_m[1]);
        cones.push(SecondOrderConeT(3));
    }
}

fn solution_from_primal(
    problem: &LcvxPoweredDescentProblem,
    layout: &LcvxDecisionLayout,
    dt: f64,
    objective: f64,
    primal: Vec<f64>,
) -> Result<LcvxPoweredDescentSolution, TrajoptError> {
    if primal.len() != layout.variable_count || !primal.iter().all(|value| value.is_finite()) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LCvx solver returned malformed primal vector",
        });
    }

    let mut positions_m = Vec::with_capacity(layout.node_count);
    let mut velocities_m_s = Vec::with_capacity(layout.node_count);
    let mut log_masses = Vec::with_capacity(layout.node_count);
    let mut masses_kg = Vec::with_capacity(layout.node_count);
    for node in 0..layout.node_count {
        let position = [
            primal[layout.position(node, 0)],
            primal[layout.position(node, 1)],
            primal[layout.position(node, 2)],
        ];
        let velocity = [
            primal[layout.velocity(node, 0)],
            primal[layout.velocity(node, 1)],
            primal[layout.velocity(node, 2)],
        ];
        let log_mass = primal[layout.log_mass(node)];
        positions_m.push(position);
        velocities_m_s.push(velocity);
        log_masses.push(log_mass);
        masses_kg.push(log_mass.exp());
    }

    let mut thrust_accel_m_s2 = Vec::with_capacity(layout.control_sample_count);
    let mut sigma_m_s2 = Vec::with_capacity(layout.control_sample_count);
    let mut max_lossless_gap_m_s2: f64 = 0.0;
    let mut max_pointing_violation_m_s2: f64 = 0.0;
    let mut max_thrust_bound_violation_m_s2: f64 = 0.0;
    for sample in 0..layout.control_sample_count {
        let thrust = [
            primal[layout.thrust(sample, 0)],
            primal[layout.thrust(sample, 1)],
            primal[layout.thrust(sample, 2)],
        ];
        let sigma = primal[layout.sigma(sample)];
        let thrust_norm = norm3(thrust);
        max_lossless_gap_m_s2 = max_lossless_gap_m_s2.max((sigma - thrust_norm).abs());
        if let Some(bounds) = problem.taylor_thrust_bounds {
            let (lower, upper) = lcvx_taylor_thrust_bounds_for_sample(bounds, problem, sample, dt);
            let z0 = lcvx_taylor_reference_log_mass(bounds, problem, sample, dt);
            let delta_z = primal[layout.log_mass(sample)] - z0;
            let lower_sigma = lower * (1.0 - delta_z + 0.5 * delta_z * delta_z);
            let upper_sigma = upper * (1.0 - delta_z);
            max_thrust_bound_violation_m_s2 = max_thrust_bound_violation_m_s2
                .max((sigma - upper_sigma).max(0.0))
                .max((lower_sigma - sigma).max(0.0));
        } else {
            max_thrust_bound_violation_m_s2 = max_thrust_bound_violation_m_s2
                .max((sigma - problem.max_thrust_accel_m_s2).max(0.0))
                .max((problem.min_thrust_accel_m_s2 - sigma).max(0.0));
        }
        if let Some(angle_rad) = problem.max_pointing_angle_rad {
            let horizontal_norm = (thrust[0] * thrust[0] + thrust[1] * thrust[1]).sqrt();
            max_pointing_violation_m_s2 =
                max_pointing_violation_m_s2.max(horizontal_norm - angle_rad.tan() * thrust[2]);
        }
        thrust_accel_m_s2.push(thrust);
        sigma_m_s2.push(sigma);
    }
    max_pointing_violation_m_s2 = max_pointing_violation_m_s2.max(0.0);

    let mut max_glideslope_violation_m: f64 = 0.0;
    if let Some(angle_rad) = problem.glideslope_angle_rad {
        let tangent = angle_rad.tan();
        for position in &positions_m {
            let horizontal = [
                position[0] - problem.target.position_m[0],
                position[1] - problem.target.position_m[1],
            ];
            let horizontal_norm =
                (horizontal[0] * horizontal[0] + horizontal[1] * horizontal[1]).sqrt();
            let vertical = position[2] - problem.target.position_m[2];
            max_glideslope_violation_m =
                max_glideslope_violation_m.max(horizontal_norm - tangent * vertical);
        }
    }
    max_glideslope_violation_m = max_glideslope_violation_m.max(0.0);

    let terminal_node = layout.horizon_steps;
    let terminal_position_error_m = [
        positions_m[terminal_node][0] - problem.target.position_m[0],
        positions_m[terminal_node][1] - problem.target.position_m[1],
        positions_m[terminal_node][2] - problem.target.position_m[2],
    ];
    let terminal_velocity_error_m_s = [
        velocities_m_s[terminal_node][0] - problem.target.velocity_m_s[0],
        velocities_m_s[terminal_node][1] - problem.target.velocity_m_s[1],
        velocities_m_s[terminal_node][2] - problem.target.velocity_m_s[2],
    ];
    let final_mass_kg = masses_kg[terminal_node];

    Ok(LcvxPoweredDescentSolution {
        status: SolverStatus::Solved,
        objective,
        time_step_s: dt,
        primal,
        positions_m,
        velocities_m_s,
        log_masses,
        masses_kg,
        thrust_accel_m_s2,
        sigma_m_s2,
        final_mass_kg,
        max_lossless_gap_m_s2,
        max_glideslope_violation_m,
        max_pointing_violation_m_s2,
        max_thrust_bound_violation_m_s2,
        terminal_position_error_m,
        terminal_velocity_error_m_s,
    })
}

fn validate_finite(reason: &'static str, values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        return Ok(());
    }
    Err(TrajoptError::InvalidPayload { reason })
}

fn validate_scalar(
    reason: &'static str,
    value: f64,
    predicate: impl FnOnce(f64) -> bool,
) -> Result<(), TrajoptError> {
    if value.is_finite() && predicate(value) {
        return Ok(());
    }
    Err(TrajoptError::InvalidPayload { reason })
}

fn validate_optional_angle(reason: &'static str, angle: Option<f64>) -> Result<(), TrajoptError> {
    if let Some(value) = angle {
        validate_scalar(reason, value, |angle_rad| {
            angle_rad > 0.0 && angle_rad < FRAC_PI_2
        })?;
    }
    Ok(())
}

fn norm3(value: [f64; 3]) -> f64 {
    (value[0] * value[0] + value[1] * value[1] + value[2] * value[2]).sqrt()
}

#[derive(Clone, Debug, PartialEq)]
struct CscBuilder {
    cols: usize,
    rows: usize,
    triplets: Vec<(usize, usize, f64)>,
}

impl CscBuilder {
    fn new(cols: usize) -> Self {
        Self {
            cols,
            rows: 0,
            triplets: Vec::new(),
        }
    }

    fn row_count(&self) -> usize {
        self.rows
    }

    fn push_row(&mut self, entries: &[(usize, f64)]) {
        for &(col, value) in entries {
            if value != 0.0 {
                self.triplets.push((self.rows, col, value));
            }
        }
        self.rows += 1;
    }

    fn finish(self) -> CscMatrix<f64> {
        let mut columns = vec![Vec::new(); self.cols];
        for (row, col, value) in self.triplets {
            columns[col].push((row, value));
        }
        let mut col_ptr = Vec::with_capacity(self.cols + 1);
        let mut row_idx = Vec::new();
        let mut values = Vec::new();
        col_ptr.push(0);
        for column in columns {
            for (row, value) in column {
                row_idx.push(row);
                values.push(value);
            }
            col_ptr.push(values.len());
        }
        CscMatrix::new(self.rows, self.cols, col_ptr, row_idx, values)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use serde::Deserialize;

    const ACIKMESE_PLOEN_MARS_GLIDESLOPE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/trajopt/acikmese-ploen-mars-glideslope-v1.toml"
    ));

    #[derive(Debug, Deserialize)]
    struct MarsLcvxFixture {
        dataset_id: String,
        vehicle: MarsLcvxVehicle,
        environment: MarsLcvxEnvironment,
        case: MarsLcvxCase,
        expected: MarsLcvxExpected,
    }

    #[derive(Debug, Deserialize)]
    struct MarsLcvxVehicle {
        initial_mass_kg: f64,
        dry_mass_kg: f64,
        specific_impulse_s: f64,
        engine_count: usize,
        max_thrust_per_engine_n: f64,
        min_throttle_fraction: f64,
        max_throttle_fraction: f64,
        engine_cant_angle_deg: f64,
    }

    #[derive(Debug, Deserialize)]
    struct MarsLcvxEnvironment {
        mars_gravity_m_s2: f64,
    }

    #[derive(Debug, Deserialize)]
    struct MarsLcvxCase {
        time_of_flight_s: f64,
        horizon_steps: usize,
        paper_initial_position_km: [f64; 3],
        paper_initial_velocity_m_s: [f64; 3],
        paper_target_position_km: [f64; 3],
        paper_target_velocity_m_s: [f64; 3],
        glideslope_altitude_angle_deg: f64,
    }

    #[derive(Debug, Deserialize)]
    struct MarsLcvxExpected {
        published_fuel_usage_kg: f64,
        published_final_mass_kg: f64,
        relative_final_mass_tolerance: f64,
        max_terminal_error_m: f64,
        max_terminal_error_m_s: f64,
        max_cone_violation: f64,
    }

    #[test]
    fn lcvx_fixed_time_powered_descent_solves_lossless_vertical_case() -> Result<(), TrajoptError> {
        let solution = solve_lcvx_powered_descent_fixed_time(&vertical_descent_problem())?;

        assert_eq!(solution.status, SolverStatus::Solved);
        assert!((solution.time_step_s - 1.0).abs() < 1.0e-12);
        assert!(
            solution
                .terminal_position_error_m
                .iter()
                .all(|v| v.abs() < 1.0e-7)
        );
        assert!(
            solution
                .terminal_velocity_error_m_s
                .iter()
                .all(|v| v.abs() < 1.0e-7)
        );
        assert!(solution.max_lossless_gap_m_s2 < 1.0e-6);
        assert!(solution.max_glideslope_violation_m < 1.0e-8);
        assert!(solution.max_pointing_violation_m_s2 < 1.0e-8);
        assert!(solution.max_thrust_bound_violation_m_s2 < 1.0e-8);
        assert!(solution.final_mass_kg < vertical_descent_problem().initial_mass_kg);
        for thrust in &solution.thrust_accel_m_s2 {
            assert!(thrust[0].abs() < 1.0e-7);
            assert!(thrust[1].abs() < 1.0e-7);
            assert!((thrust[2] - 1.0).abs() < 1.0e-6);
        }
        for sigma in &solution.sigma_m_s2 {
            assert!((*sigma - 1.0).abs() < 1.0e-6);
        }

        Ok(())
    }

    #[test]
    fn lcvx_fixed_time_powered_descent_solves_foh_node_control_case() -> Result<(), TrajoptError> {
        let problem = foh_vertical_descent_problem();
        let solution = solve_lcvx_powered_descent_fixed_time(&problem)?;

        assert_eq!(solution.status, SolverStatus::Solved);
        assert!((solution.time_step_s - 2.0).abs() < 1.0e-12);
        assert_eq!(solution.thrust_accel_m_s2.len(), problem.horizon_steps + 1);
        assert_eq!(solution.sigma_m_s2.len(), problem.horizon_steps + 1);
        assert!(
            solution
                .terminal_position_error_m
                .iter()
                .all(|v| v.abs() < 1.0e-7)
        );
        assert!(
            solution
                .terminal_velocity_error_m_s
                .iter()
                .all(|v| v.abs() < 1.0e-7)
        );
        assert!(solution.max_lossless_gap_m_s2 < 1.0e-6);
        assert!(solution.max_glideslope_violation_m < 1.0e-8);
        assert!(solution.max_pointing_violation_m_s2 < 1.0e-8);
        assert!(solution.max_thrust_bound_violation_m_s2 < 1.0e-8);
        for thrust in &solution.thrust_accel_m_s2 {
            assert!(thrust[0].abs() < 1.0e-7);
            assert!(thrust[1].abs() < 1.0e-7);
            assert!((thrust[2] - 1.0).abs() < 1.0e-6);
        }
        for sigma in &solution.sigma_m_s2 {
            assert!((*sigma - 1.0).abs() < 1.0e-6);
        }

        Ok(())
    }

    #[test]
    fn lcvx_fixed_time_powered_descent_is_repeat_deterministic() -> Result<(), TrajoptError> {
        let problem = vertical_descent_problem();
        let first = solve_lcvx_powered_descent_fixed_time(&problem)?;
        let second = solve_lcvx_powered_descent_fixed_time(&problem)?;

        assert_eq!(first.primal.len(), second.primal.len());
        for (left, right) in first.primal.iter().zip(&second.primal) {
            assert_eq!(left.to_bits(), right.to_bits());
        }
        assert_eq!(first.objective.to_bits(), second.objective.to_bits());

        Ok(())
    }

    #[test]
    fn lcvx_fixed_time_powered_descent_enforces_taylor_thrust_bounds() -> Result<(), TrajoptError> {
        let mut problem = vertical_descent_problem();
        problem.min_thrust_accel_m_s2 = 0.0;
        problem.max_thrust_accel_m_s2 = 10.0;
        problem.taylor_thrust_bounds = Some(LcvxTaylorThrustBounds {
            min_thrust_n: 500.0,
            max_thrust_n: 2_000.0,
        });

        let solution = solve_lcvx_powered_descent_fixed_time(&problem)?;

        assert!(solution.max_thrust_bound_violation_m_s2 < 1.0e-8);
        for sample in 0..solution.sigma_m_s2.len() {
            let force_n = solution.sigma_m_s2[sample] * solution.masses_kg[sample];
            assert!(force_n >= 500.0 - 1.0e-5);
            assert!(force_n <= 2_000.0 + 1.0e-5);
        }

        Ok(())
    }

    #[test]
    fn lcvx_fixed_time_powered_descent_matches_acikmese_ploen_mars_glideslope_case()
    -> Result<(), TrajoptError> {
        let fixture = acikmese_ploen_mars_glideslope_fixture();
        let problem = acikmese_ploen_mars_glideslope_problem(&fixture);
        let solution = solve_lcvx_powered_descent_fixed_time(&problem)?;
        let repeat = solve_lcvx_powered_descent_fixed_time(&problem)?;

        assert_eq!(solution.objective.to_bits(), repeat.objective.to_bits());
        assert_eq!(
            solution.final_mass_kg.to_bits(),
            repeat.final_mass_kg.to_bits()
        );
        assert_eq!(
            solution.max_lossless_gap_m_s2.to_bits(),
            repeat.max_lossless_gap_m_s2.to_bits()
        );
        assert_eq!(solution.primal.len(), repeat.primal.len());
        for (left, right) in solution.primal.iter().zip(&repeat.primal) {
            assert_eq!(left.to_bits(), right.to_bits());
        }

        assert_eq!(fixture.dataset_id, "acikmese-ploen-mars-glideslope-v1");
        assert!(
            (fixture.expected.published_final_mass_kg
                - (fixture.vehicle.initial_mass_kg - fixture.expected.published_fuel_usage_kg))
                .abs()
                < 1.0e-12
        );
        let relative_final_mass_error = ((solution.final_mass_kg
            - fixture.expected.published_final_mass_kg)
            / fixture.expected.published_final_mass_kg)
            .abs();
        assert!(
            relative_final_mass_error <= fixture.expected.relative_final_mass_tolerance,
            "final mass {} differs from published {} by {}",
            solution.final_mass_kg,
            fixture.expected.published_final_mass_kg,
            relative_final_mass_error
        );
        assert!(
            solution
                .terminal_position_error_m
                .iter()
                .all(|value| value.abs() <= fixture.expected.max_terminal_error_m)
        );
        assert!(
            solution
                .terminal_velocity_error_m_s
                .iter()
                .all(|value| value.abs() <= fixture.expected.max_terminal_error_m_s)
        );
        assert!(
            solution.max_lossless_gap_m_s2 <= fixture.expected.max_cone_violation,
            "lossless gap {} exceeds {}",
            solution.max_lossless_gap_m_s2,
            fixture.expected.max_cone_violation
        );
        assert!(
            solution.max_glideslope_violation_m <= fixture.expected.max_cone_violation,
            "glideslope violation {} exceeds {}",
            solution.max_glideslope_violation_m,
            fixture.expected.max_cone_violation
        );
        assert!(
            solution.max_pointing_violation_m_s2 <= fixture.expected.max_cone_violation,
            "pointing violation {} exceeds {}",
            solution.max_pointing_violation_m_s2,
            fixture.expected.max_cone_violation
        );
        assert!(
            solution.max_thrust_bound_violation_m_s2 <= fixture.expected.max_cone_violation,
            "thrust-bound violation {} exceeds {}",
            solution.max_thrust_bound_violation_m_s2,
            fixture.expected.max_cone_violation
        );

        Ok(())
    }

    #[test]
    fn lcvx_time_of_flight_search_shrinks_bracket_and_improves_midpoint() -> Result<(), TrajoptError>
    {
        let problem = vertical_descent_with_gravity_problem();
        let settings = LcvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s: 1.6,
            upper_time_of_flight_s: 4.0,
            tolerance_s: 0.05,
            max_iterations: 20,
        };

        let report = solve_lcvx_powered_descent_time_of_flight_search(&problem, settings)?;
        let midpoint_solution = solve_lcvx_with_time_of_flight(
            &problem,
            0.5 * (settings.lower_time_of_flight_s + settings.upper_time_of_flight_s),
        )?;

        assert!(report.iterations > 0);
        assert!(report.iterations <= settings.max_iterations);
        assert_eq!(report.candidate_count, report.iterations + 2);
        assert!(
            report.final_upper_time_of_flight_s - report.final_lower_time_of_flight_s
                <= settings.tolerance_s
        );
        assert!(report.best_time_of_flight_s > settings.lower_time_of_flight_s);
        assert!(report.best_time_of_flight_s < settings.upper_time_of_flight_s);
        assert!(report.solution.objective <= midpoint_solution.objective);
        assert!(report.solution.max_lossless_gap_m_s2.is_finite());
        assert!(report.solution.max_thrust_bound_violation_m_s2 < 1.0e-8);
        assert!(
            report
                .solution
                .terminal_position_error_m
                .iter()
                .all(|v| v.abs() < 1.0e-7)
        );
        assert!(
            report
                .solution
                .terminal_velocity_error_m_s
                .iter()
                .all(|v| v.abs() < 1.0e-7)
        );

        Ok(())
    }

    #[test]
    fn lcvx_time_of_flight_search_is_repeat_deterministic() -> Result<(), TrajoptError> {
        let problem = vertical_descent_with_gravity_problem();
        let settings = LcvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s: 1.6,
            upper_time_of_flight_s: 4.0,
            tolerance_s: 0.05,
            max_iterations: 20,
        };

        let first = solve_lcvx_powered_descent_time_of_flight_search(&problem, settings)?;
        let second = solve_lcvx_powered_descent_time_of_flight_search(&problem, settings)?;

        assert_eq!(
            first.best_time_of_flight_s.to_bits(),
            second.best_time_of_flight_s.to_bits()
        );
        assert_eq!(
            first.solution.objective.to_bits(),
            second.solution.objective.to_bits()
        );
        assert_eq!(first.solution.primal.len(), second.solution.primal.len());
        for (left, right) in first.solution.primal.iter().zip(&second.solution.primal) {
            assert_eq!(left.to_bits(), right.to_bits());
        }

        Ok(())
    }

    #[test]
    fn lcvx_fixed_time_powered_descent_rejects_bad_problem() {
        let mut problem = vertical_descent_problem();
        problem.target.position_m[0] = f64::NAN;
        assert!(matches!(
            solve_lcvx_powered_descent_fixed_time(&problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = vertical_descent_problem();
        problem.horizon_steps = 0;
        assert!(matches!(
            solve_lcvx_powered_descent_fixed_time(&problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = vertical_descent_problem();
        problem.max_pointing_angle_rad = Some(FRAC_PI_2);
        assert!(matches!(
            solve_lcvx_powered_descent_fixed_time(&problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = vertical_descent_problem();
        problem.taylor_thrust_bounds = Some(LcvxTaylorThrustBounds {
            min_thrust_n: 2_000.0,
            max_thrust_n: 1_000.0,
        });
        assert!(matches!(
            solve_lcvx_powered_descent_fixed_time(&problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = vertical_descent_problem();
        problem.taylor_thrust_bounds = Some(LcvxTaylorThrustBounds {
            min_thrust_n: 0.0,
            max_thrust_n: 2_000_000.0,
        });
        assert!(matches!(
            solve_lcvx_powered_descent_fixed_time(&problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn lcvx_time_of_flight_search_rejects_bad_settings() {
        let problem = vertical_descent_problem();
        assert!(matches!(
            solve_lcvx_powered_descent_time_of_flight_search(
                &problem,
                LcvxTimeOfFlightSearchSettings {
                    lower_time_of_flight_s: 2.0,
                    upper_time_of_flight_s: 2.0,
                    tolerance_s: 0.1,
                    max_iterations: 10,
                }
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            solve_lcvx_powered_descent_time_of_flight_search(
                &problem,
                LcvxTimeOfFlightSearchSettings {
                    lower_time_of_flight_s: 1.0,
                    upper_time_of_flight_s: 3.0,
                    tolerance_s: 0.0,
                    max_iterations: 10,
                }
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            solve_lcvx_powered_descent_time_of_flight_search(
                &problem,
                LcvxTimeOfFlightSearchSettings {
                    lower_time_of_flight_s: 1.0,
                    upper_time_of_flight_s: 3.0,
                    tolerance_s: 0.1,
                    max_iterations: 0,
                }
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    fn vertical_descent_problem() -> LcvxPoweredDescentProblem {
        LcvxPoweredDescentProblem {
            initial_position_m: [0.0, 0.0, 2.0],
            initial_velocity_m_s: [0.0, 0.0, -2.0],
            initial_mass_kg: 1_000.0,
            target: LcvxTerminalStateTarget {
                position_m: [0.0, 0.0, 0.0],
                velocity_m_s: [0.0, 0.0, 0.0],
            },
            gravity_m_s2: [0.0, 0.0, 0.0],
            horizon_steps: 2,
            time_of_flight_s: 2.0,
            control_discretization: LcvxControlDiscretization::ZeroOrderHold,
            specific_impulse_s: 300.0,
            min_thrust_accel_m_s2: 0.0,
            max_thrust_accel_m_s2: 3.0,
            taylor_thrust_bounds: None,
            glideslope_angle_rad: Some(45.0_f64.to_radians()),
            max_pointing_angle_rad: Some(30.0_f64.to_radians()),
        }
    }

    fn foh_vertical_descent_problem() -> LcvxPoweredDescentProblem {
        LcvxPoweredDescentProblem {
            horizon_steps: 1,
            control_discretization: LcvxControlDiscretization::FirstOrderHold,
            ..vertical_descent_problem()
        }
    }

    fn vertical_descent_with_gravity_problem() -> LcvxPoweredDescentProblem {
        LcvxPoweredDescentProblem {
            gravity_m_s2: [0.0, 0.0, -1.0],
            max_thrust_accel_m_s2: 4.0,
            ..vertical_descent_problem()
        }
    }

    fn acikmese_ploen_mars_glideslope_fixture() -> MarsLcvxFixture {
        toml::from_str(ACIKMESE_PLOEN_MARS_GLIDESLOPE).expect("Mars LCvx fixture parses")
    }

    fn acikmese_ploen_mars_glideslope_problem(
        fixture: &MarsLcvxFixture,
    ) -> LcvxPoweredDescentProblem {
        let cant_rad = fixture.vehicle.engine_cant_angle_deg.to_radians();
        let net_min_thrust_n = fixture.vehicle.engine_count as f64
            * fixture.vehicle.max_thrust_per_engine_n
            * fixture.vehicle.min_throttle_fraction
            * cant_rad.cos();
        let net_max_thrust_n = fixture.vehicle.engine_count as f64
            * fixture.vehicle.max_thrust_per_engine_n
            * fixture.vehicle.max_throttle_fraction
            * cant_rad.cos();

        LcvxPoweredDescentProblem {
            initial_position_m: paper_mars_position_to_local_m(
                fixture.case.paper_initial_position_km,
            ),
            initial_velocity_m_s: paper_mars_vector_to_local(
                fixture.case.paper_initial_velocity_m_s,
            ),
            initial_mass_kg: fixture.vehicle.initial_mass_kg,
            target: LcvxTerminalStateTarget {
                position_m: paper_mars_position_to_local_m(fixture.case.paper_target_position_km),
                velocity_m_s: paper_mars_vector_to_local(fixture.case.paper_target_velocity_m_s),
            },
            gravity_m_s2: [0.0, 0.0, -fixture.environment.mars_gravity_m_s2],
            horizon_steps: fixture.case.horizon_steps,
            time_of_flight_s: fixture.case.time_of_flight_s,
            control_discretization: LcvxControlDiscretization::ZeroOrderHold,
            specific_impulse_s: fixture.vehicle.specific_impulse_s * cant_rad.cos(),
            min_thrust_accel_m_s2: 0.0,
            max_thrust_accel_m_s2: net_max_thrust_n / fixture.vehicle.dry_mass_kg,
            taylor_thrust_bounds: Some(LcvxTaylorThrustBounds {
                min_thrust_n: net_min_thrust_n,
                max_thrust_n: net_max_thrust_n,
            }),
            glideslope_angle_rad: Some(fixture.case.glideslope_altitude_angle_deg.to_radians()),
            max_pointing_angle_rad: None,
        }
    }

    fn paper_mars_position_to_local_m(value: [f64; 3]) -> [f64; 3] {
        [1_000.0 * value[2], 1_000.0 * value[1], 1_000.0 * value[0]]
    }

    fn paper_mars_vector_to_local(value: [f64; 3]) -> [f64; 3] {
        [value[2], value[1], value[0]]
    }
}
