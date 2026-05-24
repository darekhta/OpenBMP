//! Differential-flatness trajectory tracker.
//!
//! Implements the Mellinger & Kumar 2011 minimum-snap polynomial
//! trajectory generator and the analytical attitude / body-rate
//! reference derivation for thrust-along-body-z vehicles.
//!
//! Reference:
//! Mellinger, D. and Kumar, V., *Minimum snap trajectory generation
//! and control for quadrotors*, IEEE ICRA 2011, pp. 2520–2525.
//!
//! # Polynomial generator
//!
//! [`MinimumSnapTrajectory::new`] takes an ordered sequence of
//! `(position, time)` waypoints in ECI and solves a per-axis
//! saddle-point KKT system that minimizes the integrated squared
//! snap subject to:
//!
//! - position pinning at every waypoint,
//! - velocity / acceleration / jerk continuity at every internal
//!   junction,
//! - zero velocity / acceleration / jerk at the start and end.
//!
//! Each segment is a 7th-order polynomial in scaled segment time
//! `τ = (t − t_i) / T_i ∈ [0, 1]`. The KKT system is built with
//! shared cost matrix Q and constraint matrix A across the three
//! ECI axes and is solved once per construction.
//!
//! # Attitude / body-rate / angular-acceleration references
//!
//! [`flat_output_attitude_reference`] consumes a [`FlatOutputs`]
//! sample and a [`YawProfile`] and returns the analytical
//! body-z thrust direction, body-frame angular velocity, and
//! angular acceleration per Mellinger-Kumar §III. These reference
//! signals are bit-deterministic given the same flat outputs and
//! yaw profile.
//!
//! # Determinism
//!
//! The solver path uses a single deterministic partial-pivot LU
//! decomposition (`nalgebra::DMatrix::lu`). Polynomial coefficients
//! are bit-stable across reruns on the same platform profile.

use nalgebra::{DMatrix, DVector, Matrix3, Rotation3, UnitQuaternion, Vector3};

use crate::error::{AutopilotError, ControllerError};

/// One position waypoint in the trajectory.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MinimumSnapWaypoint {
    /// ECI position in metres.
    pub position_eci_m: Vector3<f64>,
    /// Scenario-time of this waypoint in seconds.
    pub time_s: f64,
}

/// Flat outputs (position + derivatives up to snap) at a sample time.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct FlatOutputs {
    /// ECI position (m).
    pub position_eci_m: Vector3<f64>,
    /// ECI velocity (m/s).
    pub velocity_eci_m_s: Vector3<f64>,
    /// ECI acceleration (m/s²).
    pub acceleration_eci_m_s2: Vector3<f64>,
    /// ECI jerk (m/s³).
    pub jerk_eci_m_s3: Vector3<f64>,
    /// ECI snap (m/s⁴).
    pub snap_eci_m_s4: Vector3<f64>,
}

/// Yaw profile sample at a given time.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct YawProfile {
    /// Yaw angle (rad), measured CCW about ECI +z.
    pub yaw_rad: f64,
    /// Yaw rate (rad/s).
    pub yaw_rate_rad_s: f64,
    /// Yaw angular acceleration (rad/s²).
    pub yaw_accel_rad_s2: f64,
}

/// Differential-flatness attitude / body-rate / angular-acceleration
/// reference triple.
#[derive(Clone, Debug, PartialEq)]
pub struct DifferentialFlatnessReference {
    /// Body→ECI quaternion.
    pub q_body_to_eci: UnitQuaternion<f64>,
    /// Body-frame angular velocity (rad/s).
    pub omega_body_rad_s: Vector3<f64>,
    /// Body-frame angular acceleration (rad/s²).
    pub alpha_body_rad_s2: Vector3<f64>,
    /// Required specific force in ECI (m/s²). The body-z axis is
    /// aligned with this vector.
    pub thrust_specific_force_eci_m_s2: Vector3<f64>,
}

/// Trajectory-generation error.
#[derive(Debug, thiserror::Error)]
pub enum TrajectoryError {
    /// Fewer than two waypoints supplied.
    #[error("at least two waypoints are required (got {count})")]
    TooFewWaypoints {
        /// Number of waypoints supplied.
        count: usize,
    },
    /// Waypoint times are not strictly increasing.
    #[error(
        "waypoint times must be strictly increasing; t[{index}]={t_prev} >= t[{next}]={t_next}"
    )]
    NonMonotonicTime {
        /// Index of the offending earlier waypoint.
        index: usize,
        /// Time at the earlier waypoint.
        t_prev: f64,
        /// Index of the offending later waypoint.
        next: usize,
        /// Time at the later waypoint.
        t_next: f64,
    },
    /// A waypoint position has a non-finite component.
    #[error("waypoint {index} position has a non-finite component: {position:?}")]
    NonFinitePosition {
        /// Index of the offending waypoint.
        index: usize,
        /// Offending position vector.
        position: [f64; 3],
    },
    /// The KKT solver could not produce a finite solution.
    #[error("minimum-snap KKT system is singular or produced non-finite coefficients")]
    SingularKkt,
    /// Segment duration is outside the deterministic conditioning envelope.
    #[error("segment {index} duration {duration_s} s is outside [{min_s}, {max_s}] s")]
    SegmentDurationOutOfRange {
        /// Segment index.
        index: usize,
        /// Segment duration in seconds.
        duration_s: f64,
        /// Minimum supported duration.
        min_s: f64,
        /// Maximum supported duration.
        max_s: f64,
    },
}

impl From<TrajectoryError> for AutopilotError {
    fn from(err: TrajectoryError) -> Self {
        AutopilotError::Trajectory {
            reason: err.to_string(),
        }
    }
}

impl From<TrajectoryError> for ControllerError {
    fn from(err: TrajectoryError) -> Self {
        ControllerError::from(AutopilotError::from(err))
    }
}

/// Piecewise 7th-order polynomial trajectory minimizing integrated
/// squared snap.
#[derive(Clone, Debug)]
pub struct MinimumSnapTrajectory {
    /// Ordered waypoints (M+1 entries for M segments).
    waypoints: Vec<MinimumSnapWaypoint>,
    /// Per-segment durations (length M).
    durations: Vec<f64>,
    /// Polynomial coefficients per axis. Each `Vec` has 8M entries
    /// arranged segment-major: `[c_{0,0}..c_{0,7}, c_{1,0}..c_{1,7}, …]`.
    coefficients: [Vec<f64>; 3],
}

const SEGMENT_DEGREE: usize = 7;
const SEGMENT_COEFFS: usize = SEGMENT_DEGREE + 1;
/// Shortest segment duration accepted by the deterministic KKT solve.
///
/// The minimum-snap cost scales as `1 / T^7`; below this envelope the
/// Hessian terms become large enough that the saddle-point solve is
/// poorly conditioned for scenario-authored waypoints.
pub const MINIMUM_SNAP_MIN_SEGMENT_DURATION_S: f64 = 1.0e-3;
/// Longest segment duration accepted by the deterministic KKT solve.
///
/// This is intentionally broad for academic scenarios while avoiding
/// near-zero snap-cost blocks that make the KKT system numerically
/// fragile.
pub const MINIMUM_SNAP_MAX_SEGMENT_DURATION_S: f64 = 600.0;

impl MinimumSnapTrajectory {
    /// Build a trajectory from the given waypoint sequence.
    ///
    /// # Errors
    ///
    /// Returns [`TrajectoryError`] when the sequence has fewer than
    /// two waypoints, contains non-monotonic times, contains non-finite
    /// positions, or produces a singular KKT system.
    pub fn new(waypoints: Vec<MinimumSnapWaypoint>) -> Result<Self, TrajectoryError> {
        if waypoints.len() < 2 {
            return Err(TrajectoryError::TooFewWaypoints {
                count: waypoints.len(),
            });
        }
        for (i, w) in waypoints.iter().enumerate() {
            if !w.position_eci_m.iter().all(|v| v.is_finite()) {
                return Err(TrajectoryError::NonFinitePosition {
                    index: i,
                    position: [w.position_eci_m.x, w.position_eci_m.y, w.position_eci_m.z],
                });
            }
            if !w.time_s.is_finite() {
                return Err(TrajectoryError::NonMonotonicTime {
                    index: i,
                    t_prev: w.time_s,
                    next: i,
                    t_next: w.time_s,
                });
            }
        }
        for i in 0..waypoints.len() - 1 {
            let duration_s = waypoints[i + 1].time_s - waypoints[i].time_s;
            if duration_s <= 0.0 {
                return Err(TrajectoryError::NonMonotonicTime {
                    index: i,
                    t_prev: waypoints[i].time_s,
                    next: i + 1,
                    t_next: waypoints[i + 1].time_s,
                });
            }
            if !(MINIMUM_SNAP_MIN_SEGMENT_DURATION_S..=MINIMUM_SNAP_MAX_SEGMENT_DURATION_S)
                .contains(&duration_s)
            {
                return Err(TrajectoryError::SegmentDurationOutOfRange {
                    index: i,
                    duration_s,
                    min_s: MINIMUM_SNAP_MIN_SEGMENT_DURATION_S,
                    max_s: MINIMUM_SNAP_MAX_SEGMENT_DURATION_S,
                });
            }
        }

        let segments = waypoints.len() - 1;
        let durations: Vec<f64> = (0..segments)
            .map(|i| waypoints[i + 1].time_s - waypoints[i].time_s)
            .collect();

        let coefficients = solve_minimum_snap(&waypoints, &durations)?;

        Ok(Self {
            waypoints,
            durations,
            coefficients,
        })
    }

    /// Trajectory start time (seconds).
    #[must_use]
    pub fn start_time_s(&self) -> f64 {
        self.waypoints[0].time_s
    }

    /// Trajectory end time (seconds).
    #[must_use]
    pub fn end_time_s(&self) -> f64 {
        self.waypoints[self.waypoints.len() - 1].time_s
    }

    /// Number of segments.
    #[must_use]
    pub fn segments(&self) -> usize {
        self.durations.len()
    }

    /// Borrow the per-axis coefficient vectors. Segment-major:
    /// each axis `Vec` is `8 * segments()` long.
    #[must_use]
    pub fn coefficients_per_axis(&self) -> &[Vec<f64>; 3] {
        &self.coefficients
    }

    /// Evaluate the trajectory at scenario time `t_s`. Times outside
    /// `[start_time_s, end_time_s]` are clamped to the endpoints; the
    /// polynomial value at clamped endpoints is the corresponding
    /// waypoint position.
    #[must_use]
    pub fn evaluate(&self, t_s: f64) -> FlatOutputs {
        let (segment, tau) = self.locate(t_s);
        let duration = self.durations[segment];
        let mut outputs = FlatOutputs::default();
        for (axis, axis_coefficients) in self.coefficients.iter().enumerate() {
            let coeffs = &axis_coefficients
                [segment * SEGMENT_COEFFS..segment * SEGMENT_COEFFS + SEGMENT_COEFFS];
            let derivatives = polynomial_evaluate(coeffs, tau, duration);
            outputs.position_eci_m[axis] = derivatives.position;
            outputs.velocity_eci_m_s[axis] = derivatives.velocity;
            outputs.acceleration_eci_m_s2[axis] = derivatives.acceleration;
            outputs.jerk_eci_m_s3[axis] = derivatives.jerk;
            outputs.snap_eci_m_s4[axis] = derivatives.snap;
        }
        outputs
    }

    fn locate(&self, t_s: f64) -> (usize, f64) {
        let start = self.start_time_s();
        let end = self.end_time_s();
        if t_s <= start {
            return (0, 0.0);
        }
        if t_s >= end {
            let last = self.durations.len() - 1;
            return (last, 1.0);
        }
        // Linear search; M is typically small. Binary search would only
        // matter past O(10) segments and is not warranted today.
        let mut acc = start;
        for (i, dt) in self.durations.iter().enumerate() {
            if t_s < acc + dt {
                let tau = (t_s - acc) / dt;
                return (i, tau);
            }
            acc += dt;
        }
        // Fallback: should not be reached given the bracket above; emit
        // the last segment's endpoint instead of panicking.
        let last = self.durations.len() - 1;
        (last, 1.0)
    }
}

/// Per-segment polynomial value plus its time derivatives up to snap.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
struct PolynomialDerivatives {
    position: f64,
    velocity: f64,
    acceleration: f64,
    jerk: f64,
    snap: f64,
}

/// Evaluate the segment polynomial and its time derivatives up to snap
/// at scaled segment time `tau ∈ [0, 1]` and segment duration `T`.
///
/// Time derivatives carry an explicit `1/T^k` factor so the polynomial
/// is parameterised in `τ` while the returned derivatives are in real
/// time.
#[allow(clippy::cast_precision_loss)]
// `j ∈ [0, 7]` and the falling-factorial product fits in `usize` and
// has zero rounding when cast to `f64`. The lint warns about generic
// `usize → f64` casts; the bound is explicitly small here.
fn polynomial_evaluate(coeffs: &[f64], tau: f64, duration: f64) -> PolynomialDerivatives {
    debug_assert_eq!(coeffs.len(), SEGMENT_COEFFS);
    let mut position = 0.0;
    let mut tau_pow = 1.0;
    for &c in coeffs {
        position += c * tau_pow;
        tau_pow *= tau;
    }

    let mut velocity = 0.0;
    let mut tau_pow_v = 1.0;
    for (j, c) in coeffs.iter().enumerate().skip(1) {
        velocity += (j as f64) * c * tau_pow_v;
        tau_pow_v *= tau;
    }
    velocity /= duration;

    let mut acceleration = 0.0;
    let mut tau_pow_a = 1.0;
    for (j, c) in coeffs.iter().enumerate().skip(2) {
        acceleration += ((j * (j - 1)) as f64) * c * tau_pow_a;
        tau_pow_a *= tau;
    }
    acceleration /= duration * duration;

    let mut jerk = 0.0;
    let mut tau_pow_j = 1.0;
    for (j, c) in coeffs.iter().enumerate().skip(3) {
        jerk += ((j * (j - 1) * (j - 2)) as f64) * c * tau_pow_j;
        tau_pow_j *= tau;
    }
    jerk /= duration * duration * duration;

    let mut snap = 0.0;
    let mut tau_pow_s = 1.0;
    for (j, c) in coeffs.iter().enumerate().skip(4) {
        snap += ((j * (j - 1) * (j - 2) * (j - 3)) as f64) * c * tau_pow_s;
        tau_pow_s *= tau;
    }
    let t4 = duration * duration * duration * duration;
    snap /= t4;

    PolynomialDerivatives {
        position,
        velocity,
        acceleration,
        jerk,
        snap,
    }
}

/// Return the row of the polynomial-coefficient evaluation operator for
/// the `derivative_order`-th derivative of `p` at `tau` for a segment
/// of duration `T`. The returned 8-element row dotted with the
/// segment's coefficient vector yields `p^{(k)}(t)` in real-time units.
#[allow(clippy::cast_precision_loss)]
// Same justification as `polynomial_evaluate`: bounded small `usize`
// values cast to `f64` exactly.
fn derivative_row(derivative_order: usize, tau: f64, duration: f64) -> [f64; SEGMENT_COEFFS] {
    let mut row = [0.0; SEGMENT_COEFFS];
    for (j, slot) in row.iter_mut().enumerate().skip(derivative_order) {
        let mut factor = 1.0_f64;
        for k in 0..derivative_order {
            factor *= (j - k) as f64;
        }
        let mut tau_pow = 1.0;
        for _ in 0..(j - derivative_order) {
            tau_pow *= tau;
        }
        *slot = factor * tau_pow;
    }
    // `derivative_order ≤ 4` always; truncating to `i32` is a no-op
    // here. `try_from` keeps the code defensive without an `unwrap`.
    let t_pow = duration.powi(i32::try_from(derivative_order).unwrap_or(0));
    for entry in &mut row {
        *entry /= t_pow;
    }
    row
}

/// Build the per-segment 8×8 snap-cost matrix block at a given
/// duration. Only the lower-right 4×4 sub-block is non-zero.
#[allow(clippy::cast_precision_loss)]
// `j, k ∈ [4, 7]`, factorial ratios fit in `u32` and convert to `f64`
// exactly.
fn segment_cost_block(duration: f64) -> [[f64; SEGMENT_COEFFS]; SEGMENT_COEFFS] {
    let mut q = [[0.0; SEGMENT_COEFFS]; SEGMENT_COEFFS];
    let t_pow = duration.powi(7);
    for (j, row) in q.iter_mut().enumerate().skip(4) {
        for (k, entry) in row.iter_mut().enumerate().skip(4) {
            let coeff_j = factorial_ratio(j, j - 4) as f64;
            let coeff_k = factorial_ratio(k, k - 4) as f64;
            let denom = (j + k - 7) as f64;
            *entry = coeff_j * coeff_k / (denom * t_pow);
        }
    }
    q
}

/// `n! / m!` for `n >= m`. Used at the polynomial-derivative
/// constants `j!/(j-k)!`. Restricted to small `n`/`m` so the
/// `usize` math never overflows in the supported degree range.
const fn factorial_ratio(n: usize, m: usize) -> usize {
    let mut acc = 1;
    let mut i = m + 1;
    while i <= n {
        acc *= i;
        i += 1;
    }
    acc
}

fn build_cost_matrix(durations: &[f64]) -> DMatrix<f64> {
    let segments = durations.len();
    let n_coeffs = segments * SEGMENT_COEFFS;
    let mut q = DMatrix::<f64>::zeros(n_coeffs, n_coeffs);
    for (segment, &duration) in durations.iter().enumerate() {
        let block = segment_cost_block(duration);
        let base = segment * SEGMENT_COEFFS;
        for (j, row) in block.iter().enumerate() {
            for (k, entry) in row.iter().enumerate() {
                q[(base + j, base + k)] = *entry;
            }
        }
    }
    q
}

fn write_segment_row(
    a: &mut DMatrix<f64>,
    row_index: usize,
    base: usize,
    row: &[f64; SEGMENT_COEFFS],
    sign: f64,
) {
    for (j, value) in row.iter().enumerate() {
        a[(row_index, base + j)] = sign * *value;
    }
}

fn build_constraints(
    waypoints: &[MinimumSnapWaypoint],
    durations: &[f64],
) -> (DMatrix<f64>, [Vec<f64>; 3]) {
    let segments = durations.len();
    let n_coeffs = segments * SEGMENT_COEFFS;
    let n_constraints = 5 * segments + 3;

    let mut a = DMatrix::<f64>::zeros(n_constraints, n_coeffs);
    let mut b_per_axis: [Vec<f64>; 3] = [
        vec![0.0; n_constraints],
        vec![0.0; n_constraints],
        vec![0.0; n_constraints],
    ];
    let mut row_index = 0;

    // Position pinning: p_i(0) = w_i, p_i(1) = w_{i+1} for each segment.
    for segment in 0..segments {
        let base = segment * SEGMENT_COEFFS;
        a[(row_index, base)] = 1.0;
        for (axis, b) in b_per_axis.iter_mut().enumerate() {
            b[row_index] = waypoints[segment].position_eci_m[axis];
        }
        row_index += 1;
        let row = derivative_row(0, 1.0, durations[segment]);
        write_segment_row(&mut a, row_index, base, &row, 1.0);
        for (axis, b) in b_per_axis.iter_mut().enumerate() {
            b[row_index] = waypoints[segment + 1].position_eci_m[axis];
        }
        row_index += 1;
    }

    // Internal continuity: velocity, acceleration, jerk match across
    // junctions. RHS is zero (already initialized).
    for junction in 0..segments.saturating_sub(1) {
        for order in 1..=3 {
            let left = derivative_row(order, 1.0, durations[junction]);
            let right = derivative_row(order, 0.0, durations[junction + 1]);
            write_segment_row(&mut a, row_index, junction * SEGMENT_COEFFS, &left, 1.0);
            write_segment_row(
                &mut a,
                row_index,
                (junction + 1) * SEGMENT_COEFFS,
                &right,
                -1.0,
            );
            row_index += 1;
        }
    }

    // Endpoint zero-derivative constraints at start and end.
    for order in 1..=3 {
        let row = derivative_row(order, 0.0, durations[0]);
        write_segment_row(&mut a, row_index, 0, &row, 1.0);
        row_index += 1;
    }
    let last = segments - 1;
    let last_base = last * SEGMENT_COEFFS;
    for order in 1..=3 {
        let row = derivative_row(order, 1.0, durations[last]);
        write_segment_row(&mut a, row_index, last_base, &row, 1.0);
        row_index += 1;
    }
    debug_assert_eq!(row_index, n_constraints);

    (a, b_per_axis)
}

fn assemble_kkt(q: &DMatrix<f64>, a: &DMatrix<f64>) -> DMatrix<f64> {
    let n_coeffs = q.nrows();
    let n_constraints = a.nrows();
    let n_total = n_coeffs + n_constraints;
    let mut kkt = DMatrix::<f64>::zeros(n_total, n_total);
    for i in 0..n_coeffs {
        for j in 0..n_coeffs {
            kkt[(i, j)] = q[(i, j)];
        }
    }
    for i in 0..n_constraints {
        for j in 0..n_coeffs {
            kkt[(n_coeffs + i, j)] = a[(i, j)];
            kkt[(j, n_coeffs + i)] = a[(i, j)];
        }
    }
    kkt
}

fn solve_minimum_snap(
    waypoints: &[MinimumSnapWaypoint],
    durations: &[f64],
) -> Result<[Vec<f64>; 3], TrajectoryError> {
    let segments = durations.len();
    let n_coeffs = segments * SEGMENT_COEFFS;
    let q = build_cost_matrix(durations);
    let (a, b_per_axis) = build_constraints(waypoints, durations);
    let n_constraints = a.nrows();
    let n_total = n_coeffs + n_constraints;

    let kkt = assemble_kkt(&q, &a);
    let lu = kkt.lu();

    let mut coefficients: [Vec<f64>; 3] = [
        vec![0.0; n_coeffs],
        vec![0.0; n_coeffs],
        vec![0.0; n_coeffs],
    ];
    for (axis, b) in b_per_axis.iter().enumerate() {
        let mut rhs = DVector::<f64>::zeros(n_total);
        for (i, value) in b.iter().enumerate() {
            rhs[n_coeffs + i] = *value;
        }
        let solution = lu.solve(&rhs).ok_or(TrajectoryError::SingularKkt)?;
        if !solution.iter().all(|v| v.is_finite()) {
            return Err(TrajectoryError::SingularKkt);
        }
        for (i, slot) in coefficients[axis].iter_mut().enumerate() {
            *slot = solution[i];
        }
    }
    Ok(coefficients)
}

/// Build the analytical body→ECI quaternion, body angular velocity, and
/// body angular acceleration that a thrust-along-body-z vehicle must
/// follow to track the given flat outputs.
///
/// The derivation follows Mellinger & Kumar 2011 §III. The required
/// specific force in ECI is `f = a_d − g_eci` where `g_eci` is the
/// gravity vector returned by
/// [`openbmp_physics::gravity::standard_down_z_eci_m_s2`]; the body-z
/// axis aligns with this vector. Body x and y axes are constructed in
/// the yaw-aligned plane. Body angular velocity components come from
/// the projection of `dz_b/dt` onto the body x/y axes (which is itself
/// a function of jerk); body angular acceleration components come from
/// the time-derivative of those projections (a function of snap and
/// the yaw acceleration).
///
/// Returns `None` when the required specific force is too small for a
/// well-defined thrust direction (free-fall regime); callers must
/// handle that case explicitly rather than propagating zero references.
#[must_use]
pub fn flat_output_attitude_reference(
    flat: &FlatOutputs,
    yaw: YawProfile,
) -> Option<DifferentialFlatnessReference> {
    let g_eci = openbmp_physics::gravity::standard_down_z_eci_m_s2();
    let f = flat.acceleration_eci_m_s2 - g_eci;
    let f_norm = f.norm();
    if f_norm < 1.0e-6 {
        return None;
    }

    // Body z-axis aligns with the desired specific force.
    let z_b = f / f_norm;

    // Yaw-aligned heading vector and its cross-product with body z give
    // body y; body x = body y × body z.
    let cos_yaw = yaw.yaw_rad.cos();
    let sin_yaw = yaw.yaw_rad.sin();
    let x_c = Vector3::new(cos_yaw, sin_yaw, 0.0);
    let y_b_unnormalized = z_b.cross(&x_c);
    let y_b_norm = y_b_unnormalized.norm();
    if y_b_norm < 1.0e-6 {
        // Body-z aligned with x_c — singular yaw-frame. Fall back to a
        // yaw-axis direction perpendicular to z_b, derived from the
        // global +y to keep a deterministic right-handed frame.
        let fallback = if z_b.y.abs() > 0.9 {
            Vector3::new(1.0, 0.0, 0.0)
        } else {
            Vector3::new(0.0, 1.0, 0.0)
        };
        let y_b = z_b.cross(&fallback).normalize();
        let x_b = y_b.cross(&z_b).normalize();
        return Some(build_reference_from_axes(
            x_b, y_b, z_b, flat, yaw, f, f_norm,
        ));
    }
    let y_b = y_b_unnormalized / y_b_norm;
    let x_b = y_b.cross(&z_b);

    Some(build_reference_from_axes(
        x_b, y_b, z_b, flat, yaw, f, f_norm,
    ))
}

fn build_reference_from_axes(
    x_b: Vector3<f64>,
    y_b: Vector3<f64>,
    z_b: Vector3<f64>,
    flat: &FlatOutputs,
    yaw: YawProfile,
    f: Vector3<f64>,
    f_norm: f64,
) -> DifferentialFlatnessReference {
    let rot = Rotation3::from_matrix_unchecked(Matrix3::from_columns(&[x_b, y_b, z_b]));
    let q_body_to_eci = UnitQuaternion::from_rotation_matrix(&rot);

    // h_w = dz_b/dt = j_d/||f|| − z_b (z_b · j_d) / ||f|| is the
    // lateral component of the desired specific-force rate in the plane
    // perpendicular to z_b. With ω = ω_x x_b + ω_y y_b + ω_z z_b and
    // dz_b/dt = ω_y x_b − ω_x y_b, projecting h_w onto x_b and −y_b
    // recovers ω_y and ω_x respectively.
    let force_rate_along_body_z = z_b.dot(&flat.jerk_eci_m_s3);
    let h_w = (flat.jerk_eci_m_s3 - z_b * force_rate_along_body_z) / f_norm;
    let omega_y = h_w.dot(&x_b);
    let omega_x = -h_w.dot(&y_b);

    // The yaw-rate component projected onto the body axes. The world
    // yaw axis is +z_w; Mellinger-Kumar §III gives
    // ω_z = ψ̇ · (z_w · z_b).
    let z_w = Vector3::new(0.0, 0.0, 1.0);
    let omega_z = yaw.yaw_rate_rad_s * z_w.dot(&z_b);
    let omega_body = Vector3::new(omega_x, omega_y, omega_z);

    // Angular acceleration α follows from the second derivative of the
    // body-z relation. Differentiate z_b = f / ||f|| twice, then use
    // z̈_b = (ω̇_y + ω_x ω_z) x_b + (ω_y ω_z - ω̇_x) y_b
    //          - (ω_x² + ω_y²) z_b
    // to recover the body-frame x/y components. This keeps the
    // Mellinger-Kumar snap contribution and the body-rate cross terms.
    let force_accel_along_body_z = h_w.dot(&flat.jerk_eci_m_s3) + z_b.dot(&flat.snap_eci_m_s4);
    let z_b_ddot = (flat.snap_eci_m_s4
        - z_b * force_accel_along_body_z
        - h_w * (2.0 * force_rate_along_body_z))
        / f_norm;
    let alpha_x = omega_y * omega_z - z_b_ddot.dot(&y_b);
    let alpha_y = z_b_ddot.dot(&x_b) - omega_x * omega_z;
    let alpha_z = yaw.yaw_accel_rad_s2 * z_w.dot(&z_b);
    let alpha_body = Vector3::new(alpha_x, alpha_y, alpha_z);

    DifferentialFlatnessReference {
        q_body_to_eci,
        omega_body_rad_s: omega_body,
        alpha_body_rad_s2: alpha_body,
        thrust_specific_force_eci_m_s2: f,
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

    fn waypoints_simple_two_point() -> Vec<MinimumSnapWaypoint> {
        vec![
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(0.0, 0.0, 0.0),
                time_s: 0.0,
            },
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(10.0, 0.0, 0.0),
                time_s: 4.0,
            },
        ]
    }

    fn waypoints_three_point_zigzag() -> Vec<MinimumSnapWaypoint> {
        vec![
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(0.0, 0.0, 0.0),
                time_s: 0.0,
            },
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(5.0, 2.0, 0.0),
                time_s: 2.0,
            },
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(10.0, 0.0, 0.0),
                time_s: 4.0,
            },
        ]
    }

    #[test]
    fn rejects_too_few_waypoints() {
        let err = MinimumSnapTrajectory::new(vec![]).unwrap_err();
        assert!(matches!(err, TrajectoryError::TooFewWaypoints { .. }));
    }

    #[test]
    fn rejects_non_monotonic_time() {
        let waypoints = vec![
            MinimumSnapWaypoint {
                position_eci_m: Vector3::zeros(),
                time_s: 0.0,
            },
            MinimumSnapWaypoint {
                position_eci_m: Vector3::zeros(),
                time_s: 0.0,
            },
        ];
        let err = MinimumSnapTrajectory::new(waypoints).unwrap_err();
        assert!(matches!(err, TrajectoryError::NonMonotonicTime { .. }));
    }

    #[test]
    fn rejects_out_of_range_segment_duration() {
        let waypoints = vec![
            MinimumSnapWaypoint {
                position_eci_m: Vector3::zeros(),
                time_s: 0.0,
            },
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(1.0, 0.0, 0.0),
                time_s: MINIMUM_SNAP_MIN_SEGMENT_DURATION_S / 2.0,
            },
        ];
        let err = MinimumSnapTrajectory::new(waypoints).unwrap_err();
        assert!(matches!(
            err,
            TrajectoryError::SegmentDurationOutOfRange { .. }
        ));
    }

    #[test]
    fn rejects_non_finite_position() {
        let waypoints = vec![
            MinimumSnapWaypoint {
                position_eci_m: Vector3::new(f64::NAN, 0.0, 0.0),
                time_s: 0.0,
            },
            MinimumSnapWaypoint {
                position_eci_m: Vector3::zeros(),
                time_s: 1.0,
            },
        ];
        let err = MinimumSnapTrajectory::new(waypoints).unwrap_err();
        assert!(matches!(err, TrajectoryError::NonFinitePosition { .. }));
    }

    #[test]
    fn single_segment_pins_endpoints_and_zero_derivatives() {
        let trajectory = MinimumSnapTrajectory::new(waypoints_simple_two_point()).unwrap();
        let start = trajectory.evaluate(0.0);
        let end = trajectory.evaluate(4.0);
        assert_abs_diff_eq!(start.position_eci_m.x, 0.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(end.position_eci_m.x, 10.0, epsilon = 1.0e-9);
        // Zero-derivative endpoint conditions.
        for v in [
            start.velocity_eci_m_s.x,
            start.acceleration_eci_m_s2.x,
            start.jerk_eci_m_s3.x,
            end.velocity_eci_m_s.x,
            end.acceleration_eci_m_s2.x,
            end.jerk_eci_m_s3.x,
        ] {
            assert!(v.abs() < 1.0e-8, "expected zero, got {v}");
        }
    }

    #[test]
    fn position_continuous_through_internal_junction() {
        let trajectory = MinimumSnapTrajectory::new(waypoints_three_point_zigzag()).unwrap();
        let mid = trajectory.evaluate(2.0);
        assert_abs_diff_eq!(mid.position_eci_m.x, 5.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(mid.position_eci_m.y, 2.0, epsilon = 1.0e-9);
    }

    #[test]
    fn velocity_acceleration_jerk_continuous_through_internal_junction() {
        let trajectory = MinimumSnapTrajectory::new(waypoints_three_point_zigzag()).unwrap();
        // Sample just before and just after the internal junction; the
        // continuity constraints make these match within polynomial-
        // evaluation precision.
        let before = trajectory.evaluate(2.0 - 1.0e-7);
        let after = trajectory.evaluate(2.0 + 1.0e-7);
        for axis in 0..3 {
            assert!(
                (before.velocity_eci_m_s[axis] - after.velocity_eci_m_s[axis]).abs() < 1.0e-5,
                "velocity discontinuity on axis {axis}"
            );
            assert!(
                (before.acceleration_eci_m_s2[axis] - after.acceleration_eci_m_s2[axis]).abs()
                    < 1.0e-4,
                "acceleration discontinuity on axis {axis}"
            );
            assert!(
                (before.jerk_eci_m_s3[axis] - after.jerk_eci_m_s3[axis]).abs() < 1.0e-3,
                "jerk discontinuity on axis {axis}"
            );
        }
    }

    #[test]
    fn coefficients_are_bit_stable_across_reruns() {
        let a = MinimumSnapTrajectory::new(waypoints_three_point_zigzag()).unwrap();
        let b = MinimumSnapTrajectory::new(waypoints_three_point_zigzag()).unwrap();
        for axis in 0..3 {
            assert_eq!(a.coefficients[axis].len(), b.coefficients[axis].len());
            for (lhs, rhs) in a.coefficients[axis].iter().zip(b.coefficients[axis].iter()) {
                assert_eq!(lhs.to_bits(), rhs.to_bits(), "coefficient bit mismatch");
            }
        }
    }

    #[test]
    fn evaluate_clamps_outside_trajectory() {
        let trajectory = MinimumSnapTrajectory::new(waypoints_simple_two_point()).unwrap();
        let before = trajectory.evaluate(-1.0);
        let after = trajectory.evaluate(100.0);
        assert_abs_diff_eq!(before.position_eci_m.x, 0.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(after.position_eci_m.x, 10.0, epsilon = 1.0e-9);
    }

    #[test]
    fn flat_output_reference_at_hover_trim_is_identity_quaternion() {
        // Hover-trim: a_d = 0, j_d = 0, s_d = 0 → desired specific force
        // is +z (counteracts gravity) and the body should be aligned
        // with ECI.
        let flat = FlatOutputs::default();
        let yaw = YawProfile::default();
        let reference = flat_output_attitude_reference(&flat, yaw).expect("hover-trim reference");
        // Body z aligned with ECI +z gives identity quaternion (modulo
        // double-cover sign).
        let q = reference.q_body_to_eci.into_inner();
        assert!((q.w.abs() - 1.0).abs() < 1.0e-9, "q.w = {q}", q = q.w);
        assert!(q.i.abs() < 1.0e-9);
        assert!(q.j.abs() < 1.0e-9);
        assert!(q.k.abs() < 1.0e-9);
        // Zero rates and zero accelerations at hover.
        assert!(reference.omega_body_rad_s.norm() < 1.0e-12);
        assert!(reference.alpha_body_rad_s2.norm() < 1.0e-12);
    }

    #[test]
    fn flat_output_reference_returns_none_in_free_fall() {
        // Free-fall: a_d = g_eci so the specific force is zero — the
        // thrust direction is undefined.
        let flat = FlatOutputs {
            acceleration_eci_m_s2: openbmp_physics::gravity::standard_down_z_eci_m_s2(),
            ..FlatOutputs::default()
        };
        assert!(flat_output_attitude_reference(&flat, YawProfile::default()).is_none());
    }

    #[test]
    fn flat_output_reference_yaw_rotates_body_about_z() {
        let flat = FlatOutputs::default();
        let yaw = YawProfile {
            yaw_rad: std::f64::consts::FRAC_PI_2,
            yaw_rate_rad_s: 0.5,
            yaw_accel_rad_s2: 0.0,
        };
        let reference = flat_output_attitude_reference(&flat, yaw).expect("reference");
        // 90° yaw → body x-axis points along ECI +y. Apply the
        // quaternion to body x and inspect the result.
        let body_x_in_eci = reference.q_body_to_eci * Vector3::x();
        assert_abs_diff_eq!(body_x_in_eci.x, 0.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(body_x_in_eci.y, 1.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(body_x_in_eci.z, 0.0, epsilon = 1.0e-9);
        // ω_z should be ψ̇ since z_b · z_w = 1 at hover.
        assert_abs_diff_eq!(reference.omega_body_rad_s.z, 0.5, epsilon = 1.0e-9);
    }

    #[test]
    fn flat_output_reference_angular_acceleration_includes_cross_coupling() {
        let g = -openbmp_physics::gravity::standard_down_z_eci_m_s2().z;
        let jerk_y = 2.0;
        let yaw_rate = 0.4;
        let flat = FlatOutputs {
            acceleration_eci_m_s2: Vector3::new(1.0, 0.0, 0.0),
            jerk_eci_m_s3: Vector3::new(0.0, jerk_y, 0.0),
            snap_eci_m_s4: Vector3::zeros(),
            ..FlatOutputs::default()
        };
        let yaw = YawProfile {
            yaw_rad: 0.0,
            yaw_rate_rad_s: yaw_rate,
            yaw_accel_rad_s2: 0.0,
        };

        let reference = flat_output_attitude_reference(&flat, yaw).expect("reference");
        let f_norm = (1.0_f64 + g * g).sqrt();
        let z_b_z = g / f_norm;
        let omega_x = -jerk_y / f_norm;
        let omega_z = yaw_rate * z_b_z;
        let expected_alpha_y = -omega_x * omega_z;

        assert_abs_diff_eq!(
            reference.alpha_body_rad_s2.y,
            expected_alpha_y,
            epsilon = 1.0e-12
        );
        assert!(reference.alpha_body_rad_s2.y.abs() > 1.0e-3);
    }

    #[test]
    fn factorial_ratio_matches_known_values() {
        assert_eq!(factorial_ratio(4, 0), 24);
        assert_eq!(factorial_ratio(7, 3), 7 * 6 * 5 * 4);
        assert_eq!(factorial_ratio(7, 7), 1);
    }

    #[test]
    fn segment_cost_block_recovers_known_pairs() {
        // Spot-check H_44 = 24·24/1 = 576/T^7 and H_77 = 840·840/7 = 100800/T^7.
        let block = segment_cost_block(1.0);
        assert_abs_diff_eq!(block[4][4], 576.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(block[7][7], 100_800.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(block[5][6], 120.0 * 360.0 / 4.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(block[5][7], 20_160.0, epsilon = 1.0e-12);
        // Above-diagonal symmetry.
        assert_abs_diff_eq!(block[4][7], block[7][4], epsilon = 1.0e-12);
    }
}
