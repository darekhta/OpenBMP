//! Successive-convexification substrate.
//!
//! This first WP-07.5 surface is a deterministic discrete affine SCvx core:
//! FOH-linearized dynamics, virtual control, box trust regions, and
//! state/control path constraints. Nonlinear 6-DoF/aero model generation is
//! layered above this module.

use alloc::vec;
use alloc::vec::Vec;

use clarabel::{
    algebra::CscMatrix,
    solver::{
        DefaultSolver, IPSolver, NonnegativeConeT, SecondOrderConeT, SolverStatus, SupportedConeT,
        ZeroConeT,
    },
};

use crate::iload::TrajoptError;
use crate::sqp::{ClarabelQpSolution, DenseQpProblem, deterministic_sqp_settings, solve_dense_qp};

/// Terminal state target for an SCvx solve.
///
/// The target is expressed only as a caller-owned local/state-vector target and
/// intentionally carries no latitude, longitude, range, aimpoint, or other
/// surface-coordinate targeting vocabulary.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxTerminalStateTarget {
    /// Target terminal state vector.
    pub state: Vec<f64>,
    /// Optional non-negative per-component terminal weights. `None` means all
    /// state components use unit terminal weight. A zero entry leaves that
    /// component untargeted.
    pub weights: Option<Vec<f64>>,
}

/// One FOH-discretized affine dynamics segment.
///
/// The segment represents
/// `x[k+1] = A x[k] + B_minus u[k] + B_plus u[k+1] + c + nu[k]`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxAffineDynamicsSegment {
    /// Row-major `state_dimension x state_dimension` state transition.
    pub state_transition: Vec<f64>,
    /// Row-major `state_dimension x control_dimension` left-control matrix.
    pub control_left: Vec<f64>,
    /// Row-major `state_dimension x control_dimension` right-control matrix.
    pub control_right: Vec<f64>,
    /// Affine offset.
    pub affine: Vec<f64>,
}

/// Linear state function `coefficients * x + offset`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxLinearStateFunction {
    /// Linear coefficients.
    pub coefficients: Vec<f64>,
    /// Constant offset.
    pub offset: f64,
}

/// Linear control function `coefficients * u + offset`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxLinearControlFunction {
    /// Linear coefficients.
    pub coefficients: Vec<f64>,
    /// Constant offset.
    pub offset: f64,
}

/// State-triggered linear constraint.
///
/// If `trigger(x[node]) <= 0`, the convex subproblem enforces
/// `constraint(x[node]) <= 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxStateTriggeredConstraint {
    /// Node index where the trigger and constraint are sampled.
    pub node: usize,
    /// Trigger function `h(x)`.
    pub trigger: ScvxLinearStateFunction,
    /// Constraint function `g(x)`.
    pub constraint: ScvxLinearStateFunction,
}

/// Always-active linear state path constraint sampled at one SCvx node.
///
/// The convex subproblem enforces `coefficients * x[node] + offset <= 0`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxLinearStateConstraint {
    /// Node index where the state is sampled.
    pub node: usize,
    /// Linear state function to enforce.
    pub function: ScvxLinearStateFunction,
}

/// Always-active linear control path constraint sampled at one SCvx node.
///
/// The convex subproblem enforces `coefficients * u[node] <= upper_bound`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxLinearControlConstraint {
    /// Node index where the control is sampled.
    pub node: usize,
    /// Linear coefficients with one entry per control component.
    pub coefficients: Vec<f64>,
    /// Inclusive upper bound for the linear control expression.
    pub upper_bound: f64,
}

/// Always-active second-order-cone state path constraint sampled at one node.
///
/// The convex subproblem enforces
/// `||vector[i](x[node])||_2 <= scalar(x[node])`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxSecondOrderConeStateConstraint {
    /// Node index where the state is sampled.
    pub node: usize,
    /// Scalar cone head.
    pub scalar: ScvxLinearStateFunction,
    /// Vector cone tail functions.
    pub vector: Vec<ScvxLinearStateFunction>,
}

/// Always-active second-order-cone control path constraint sampled at one node.
///
/// The convex subproblem enforces
/// `||vector[i](u[node])||_2 <= scalar(u[node])`.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxSecondOrderConeControlConstraint {
    /// Node index where the control is sampled.
    pub node: usize,
    /// Scalar cone head.
    pub scalar: ScvxLinearControlFunction,
    /// Vector cone tail functions.
    pub vector: Vec<ScvxLinearControlFunction>,
}

/// Discrete affine SCvx problem.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxDiscreteAffineProblem {
    /// Number of state variables per node.
    pub state_dimension: usize,
    /// Number of control variables per node.
    pub control_dimension: usize,
    /// Number of integration intervals.
    pub horizon_steps: usize,
    /// Initial state.
    pub initial_state: Vec<f64>,
    /// Terminal target.
    pub target: ScvxTerminalStateTarget,
    /// One affine dynamics segment per interval.
    pub dynamics: Vec<ScvxAffineDynamicsSegment>,
    /// Initial reference states, row-major by node.
    pub initial_states: Vec<f64>,
    /// Initial reference controls, row-major by node.
    pub initial_controls: Vec<f64>,
    /// Lower control bounds applied at every node.
    pub control_lower: Vec<f64>,
    /// Upper control bounds applied at every node.
    pub control_upper: Vec<f64>,
    /// Optional state-triggered constraints.
    pub state_triggered_constraints: Vec<ScvxStateTriggeredConstraint>,
    /// Always-active linear state path constraints.
    pub linear_state_constraints: Vec<ScvxLinearStateConstraint>,
    /// Always-active linear control path constraints.
    pub linear_control_constraints: Vec<ScvxLinearControlConstraint>,
    /// Always-active second-order-cone state path constraints.
    pub second_order_state_constraints: Vec<ScvxSecondOrderConeStateConstraint>,
    /// Always-active second-order-cone control path constraints.
    pub second_order_control_constraints: Vec<ScvxSecondOrderConeControlConstraint>,
}

/// Inputs for building a forward-Euler/FOH SCvx problem from continuous
/// nonlinear dynamics.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxForwardEulerFohProblemDefinition {
    /// Number of state variables per node.
    pub state_dimension: usize,
    /// Number of control variables per node.
    pub control_dimension: usize,
    /// Strictly increasing node times in seconds.
    pub times_s: Vec<f64>,
    /// Initial state pinned by the convex subproblem.
    pub initial_state: Vec<f64>,
    /// Terminal target.
    pub target: ScvxTerminalStateTarget,
    /// Reference states used for linearization, row-major by node.
    pub initial_states: Vec<f64>,
    /// Reference controls used for linearization, row-major by node.
    pub initial_controls: Vec<f64>,
    /// Lower control bounds applied at every node.
    pub control_lower: Vec<f64>,
    /// Upper control bounds applied at every node.
    pub control_upper: Vec<f64>,
    /// Optional state-triggered constraints copied into the SCvx problem.
    pub state_triggered_constraints: Vec<ScvxStateTriggeredConstraint>,
    /// Always-active linear state path constraints copied into the SCvx problem.
    pub linear_state_constraints: Vec<ScvxLinearStateConstraint>,
    /// Always-active linear control path constraints copied into the SCvx problem.
    pub linear_control_constraints: Vec<ScvxLinearControlConstraint>,
    /// Always-active second-order-cone state path constraints copied into the SCvx problem.
    pub second_order_state_constraints: Vec<ScvxSecondOrderConeStateConstraint>,
    /// Always-active second-order-cone control path constraints copied into the SCvx problem.
    pub second_order_control_constraints: Vec<ScvxSecondOrderConeControlConstraint>,
}

/// SCvx trust-region and penalty settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxSettings {
    /// Maximum outer SCvx iterations.
    pub max_iterations: usize,
    /// Terminal target-error convergence tolerance.
    pub terminal_tolerance: f64,
    /// Virtual-control convergence tolerance.
    pub virtual_control_tolerance: f64,
    /// State-triggered-constraint violation tolerance.
    pub constraint_tolerance: f64,
    /// Initial scalar infinity-norm trust-region radius.
    pub initial_trust_region_radius: f64,
    /// Minimum trust-region radius before reporting trust-region exhaustion.
    pub min_trust_region_radius: f64,
    /// Maximum trust-region radius after accepted expansion.
    pub max_trust_region_radius: f64,
    /// Trust-region expansion factor after accepted steps.
    pub trust_region_expand: f64,
    /// Trust-region shrink factor after rejected steps.
    pub trust_region_shrink: f64,
    /// Minimum accepted actual/predicted reduction ratio.
    pub accept_ratio: f64,
    /// Quadratic terminal target weight.
    pub terminal_weight: f64,
    /// Quadratic control effort weight.
    pub control_weight: f64,
    /// Quadratic virtual-control weight.
    pub virtual_control_weight: f64,
    /// Merit penalty for state-triggered violations.
    pub state_triggered_constraint_penalty: f64,
    /// Small non-negative diagonal regularization for QP conditioning.
    pub regularization: f64,
}

impl Default for ScvxSettings {
    fn default() -> Self {
        Self {
            max_iterations: 10,
            terminal_tolerance: 1.0e-6,
            virtual_control_tolerance: 1.0e-6,
            constraint_tolerance: 1.0e-8,
            initial_trust_region_radius: 1.0,
            min_trust_region_radius: 1.0e-6,
            max_trust_region_radius: 10.0,
            trust_region_expand: 1.5,
            trust_region_shrink: 0.5,
            accept_ratio: 0.1,
            terminal_weight: 1.0e6,
            control_weight: 1.0e-3,
            virtual_control_weight: 1.0e8,
            state_triggered_constraint_penalty: 1.0e8,
            regularization: 1.0e-9,
        }
    }
}

/// Settings for forward-Euler SCvx linearization of continuous dynamics.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxForwardEulerLinearizationSettings {
    /// Relative central finite-difference step for state/control Jacobians.
    pub finite_difference_step: f64,
}

impl Default for ScvxForwardEulerLinearizationSettings {
    fn default() -> Self {
        Self {
            finite_difference_step: 1.0e-6,
        }
    }
}

/// Flat 6-DoF rigid-body SCvx state layout.
///
/// The state order is `[r_eci(3), v_eci(3), q_body_to_eci_wxyz(4),
/// omega_body(3)]`. This matches the simulator convention: position and
/// velocity are inertial, the quaternion maps body-frame components to ECI, and
/// angular velocity is expressed in body axes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ScvxRigidBodyStateLayout;

impl ScvxRigidBodyStateLayout {
    /// Number of scalar state components.
    pub const DIMENSION: usize = 13;
    /// Start index of inertial position.
    pub const POSITION_ECI: usize = 0;
    /// Start index of inertial velocity.
    pub const VELOCITY_ECI: usize = 3;
    /// Start index of body-to-ECI quaternion, stored as `(w, x, y, z)`.
    pub const QUATERNION_BODY_TO_ECI_WXYZ: usize = 6;
    /// Start index of body-frame angular velocity.
    pub const ANGULAR_VELOCITY_BODY: usize = 10;
}

/// Flat 6-DoF rigid-body SCvx control layout.
///
/// The control order is `[force_eci_N(3), moment_body_N_m(3)]`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ScvxRigidBodyControlLayout;

impl ScvxRigidBodyControlLayout {
    /// Number of scalar control components.
    pub const DIMENSION: usize = 6;
    /// Start index of inertial force.
    pub const FORCE_ECI_N: usize = 0;
    /// Start index of body-frame moment.
    pub const MOMENT_BODY_N_M: usize = 3;
}

/// Powered-landing SCvx state layout.
///
/// The state order is `[mass, r_eci(3), v_eci(3), q_body_to_eci_wxyz(4),
/// omega_body(3)]`. This extends [`ScvxRigidBodyStateLayout`] with mass for
/// Szmuk-style powered-landing dynamics.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ScvxPoweredLandingStateLayout;

impl ScvxPoweredLandingStateLayout {
    /// Number of scalar state components.
    pub const DIMENSION: usize = 14;
    /// Mass index.
    pub const MASS: usize = 0;
    /// Start index of inertial position.
    pub const POSITION_ECI: usize = 1;
    /// Start index of inertial velocity.
    pub const VELOCITY_ECI: usize = 4;
    /// Start index of body-to-ECI quaternion, stored as `(w, x, y, z)`.
    pub const QUATERNION_BODY_TO_ECI_WXYZ: usize = 7;
    /// Start index of body-frame angular velocity.
    pub const ANGULAR_VELOCITY_BODY: usize = 11;
}

/// Powered-landing SCvx control layout.
///
/// The control order is body-frame thrust `[T_x, T_y, T_z]`.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ScvxPoweredLandingControlLayout;

impl ScvxPoweredLandingControlLayout {
    /// Number of scalar control components.
    pub const DIMENSION: usize = 3;
    /// Start index of body-frame thrust.
    pub const THRUST_BODY: usize = 0;
}

/// Parameters for the flat SCvx rigid-body dynamics helper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxRigidBodyDynamicsParameters {
    /// Vehicle mass, kg.
    pub mass_kg: f64,
    /// Row-major body-frame inertia tensor, kg*m^2.
    pub inertia_body_kg_m2: [f64; 9],
    /// Row-major body-frame inertia-rate tensor, kg*m^2/s.
    pub inertia_rate_body_kg_m2_s: [f64; 9],
    /// Non-control acceleration in ECI, m/s^2. Use this for gravity or
    /// externally supplied aero acceleration in this first substrate.
    pub external_acceleration_eci_m_s2: [f64; 3],
}

impl ScvxRigidBodyDynamicsParameters {
    /// Build parameters with zero inertia-rate and zero external acceleration.
    #[must_use]
    pub const fn new(mass_kg: f64, inertia_body_kg_m2: [f64; 9]) -> Self {
        Self {
            mass_kg,
            inertia_body_kg_m2,
            inertia_rate_body_kg_m2_s: [0.0; 9],
            external_acceleration_eci_m_s2: [0.0; 3],
        }
    }
}

/// Parameters for the powered-landing SCvx dynamics helper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxPoweredLandingDynamicsParameters {
    /// Non-control acceleration in ECI, m/s^2 or consistent non-dimensional
    /// acceleration units.
    pub gravity_eci_m_s2: [f64; 3],
    /// Row-major body-frame inertia tensor, kg*m^2 or consistent units.
    pub inertia_body_kg_m2: [f64; 9],
    /// Body-frame vector from center of mass to thrust application point.
    pub thrust_application_point_body_m: [f64; 3],
    /// Mass-flow coefficient multiplying body-thrust norm.
    pub mass_flow_coefficient_s_m: f64,
}

/// Trajectory-envelope limits for Szmuk-style powered-landing SCvx checks.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxPoweredLandingEnvelopeLimits {
    /// Inclusive lower mass bound.
    pub minimum_mass: f64,
    /// Inclusive upper mass bound.
    pub maximum_mass: f64,
    /// Inclusive lower body-thrust norm bound.
    pub minimum_thrust: f64,
    /// Inclusive upper body-thrust norm bound.
    pub maximum_thrust: f64,
    /// Maximum body-thrust gimbal angle from the body +x axis, radians.
    pub maximum_gimbal_angle_rad: f64,
    /// Maximum body +x tilt angle from the local up axis, radians.
    pub maximum_tilt_angle_rad: f64,
    /// Maximum body angular-rate norm, radians per trajectory time unit.
    pub maximum_angular_rate_rad_s: f64,
    /// Glideslope cone angle from the local horizontal, radians.
    pub glideslope_angle_rad: f64,
    /// Gravity direction used to define local up and glideslope altitude.
    pub gravity_eci: [f64; 3],
}

/// One powered-landing trajectory-envelope diagnostic sample.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxPoweredLandingEnvelopeSample {
    /// Physical/non-dimensional sample time.
    pub time_s: f64,
    /// Vehicle mass at the sample.
    pub mass: f64,
    /// Body-thrust norm at the sample.
    pub thrust_norm: f64,
    /// Body-thrust gimbal angle from body +x, radians.
    pub gimbal_angle_rad: f64,
    /// Body +x tilt angle from local up, radians.
    pub tilt_angle_rad: f64,
    /// Body angular-rate norm.
    pub angular_rate_norm_rad_s: f64,
    /// Glideslope signed margin, positive inside the cone.
    pub glideslope_margin: f64,
    /// Maximum scalar envelope violation at this sample.
    pub max_violation: f64,
}

/// Powered-landing trajectory-envelope report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxPoweredLandingEnvelopeReport {
    /// Number of checked samples.
    pub sample_count: usize,
    /// Maximum mass-bound violation.
    pub max_mass_violation: f64,
    /// Maximum thrust-bound violation.
    pub max_thrust_violation: f64,
    /// Maximum gimbal-angle violation.
    pub max_gimbal_angle_violation_rad: f64,
    /// Maximum tilt-angle violation.
    pub max_tilt_angle_violation_rad: f64,
    /// Maximum angular-rate violation.
    pub max_angular_rate_violation_rad_s: f64,
    /// Maximum glideslope violation.
    pub max_glideslope_violation: f64,
    /// Maximum scalar violation across all envelope rows.
    pub max_violation: f64,
    /// True when every envelope row is satisfied exactly.
    pub within_limits: bool,
    /// Per-sample diagnostics.
    pub samples: Vec<ScvxPoweredLandingEnvelopeSample>,
}

/// Linear state/control rows generated from the linearizable powered-landing
/// envelope quantities.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxPoweredLandingLinearEnvelopeConstraints {
    /// Per-node mass and glideslope halfspace rows.
    pub linear_state_constraints: Vec<ScvxLinearStateConstraint>,
    /// Per-node axial-thrust and gimbal halfspace rows.
    pub linear_control_constraints: Vec<ScvxLinearControlConstraint>,
}

/// Linear plus exact conic rows generated from powered-landing envelope quantities.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxPoweredLandingConicEnvelopeConstraints {
    /// Per-node mass-bound and reference-linearized tilt rows.
    pub linear_state_constraints: Vec<ScvxLinearStateConstraint>,
    /// Per-node axial minimum-thrust rows.
    pub linear_control_constraints: Vec<ScvxLinearControlConstraint>,
    /// Per-node glideslope and angular-rate norm cone rows.
    pub second_order_state_constraints: Vec<ScvxSecondOrderConeStateConstraint>,
    /// Per-node thrust-norm and gimbal cone rows.
    pub second_order_control_constraints: Vec<ScvxSecondOrderConeControlConstraint>,
}

/// Settings for powered-landing reference-linearized envelope row generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxPoweredLandingLinearizedEnvelopeSettings {
    /// Number of lateral cone facets used for glideslope and gimbal rows.
    pub cone_facets: usize,
    /// Number of unit-direction facets used for thrust and angular-rate norm rows.
    pub norm_facets: usize,
    /// Central finite-difference step for reference-linearized tilt rows.
    pub finite_difference_step: f64,
    /// Allowed absolute error from unit quaternion norm in reference states.
    pub quaternion_norm_tolerance: f64,
}

impl Default for ScvxPoweredLandingLinearizedEnvelopeSettings {
    fn default() -> Self {
        Self {
            cone_facets: 8,
            norm_facets: 14,
            finite_difference_step: 1.0e-6,
            quaternion_norm_tolerance: 1.0e-9,
        }
    }
}

/// Settings for powered-landing conic envelope row generation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxPoweredLandingConicEnvelopeSettings {
    /// Central finite-difference step for reference-linearized tilt rows.
    pub finite_difference_step: f64,
    /// Allowed absolute error from unit quaternion norm in reference states.
    pub quaternion_norm_tolerance: f64,
}

impl Default for ScvxPoweredLandingConicEnvelopeSettings {
    fn default() -> Self {
        Self {
            finite_difference_step: 1.0e-6,
            quaternion_norm_tolerance: 1.0e-9,
        }
    }
}

/// Validation settings for the flat SCvx rigid-body dynamics helper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxRigidBodyDynamicsSettings {
    /// Allowed absolute error from unit quaternion norm.
    pub quaternion_norm_tolerance: f64,
}

impl Default for ScvxRigidBodyDynamicsSettings {
    fn default() -> Self {
        Self {
            quaternion_norm_tolerance: 1.0e-9,
        }
    }
}

/// Reduced axisymmetric aerodynamic coefficients for the SCvx substrate.
///
/// The mapping matches `openbmp-aero` schema 1: `CD` drag acts along body `-x`,
/// `CN` normal force acts along body `-z`, and `CM` is a pitch moment about
/// body `+y`.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ScvxReducedAeroCoefficients {
    /// Normal-force coefficient.
    pub cn: f64,
    /// Drag coefficient.
    pub cd: f64,
    /// Pitching-moment coefficient.
    pub cm: f64,
}

/// Flight-condition and reference quantities for reduced aero composition.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxReducedAeroParameters {
    /// Dynamic pressure, Pa.
    pub dynamic_pressure_pa: f64,
    /// Aerodynamic reference area, m^2.
    pub reference_area_m2: f64,
    /// Aerodynamic reference length, m.
    pub reference_length_m: f64,
}

/// Reduced aero force/moment result for the SCvx rigid-body helper.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxReducedAeroForceMoment {
    /// Aerodynamic force in body axes, N.
    pub force_n_body: [f64; 3],
    /// Aerodynamic force rotated into ECI axes, N.
    pub force_n_eci: [f64; 3],
    /// Aerodynamic moment in body axes, N*m.
    pub moment_n_m_body: [f64; 3],
    /// Equivalent rigid-body control vector `[force_eci_N(3), moment_body_N_m(3)]`.
    pub rigid_body_control: [f64; ScvxRigidBodyControlLayout::DIMENSION],
}

/// Per-node reduced-aero profile used by the SCvx rigid-body FOH builder.
///
/// Interval linearization uses the profile sample at the interval's left node.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxReducedAeroReferenceProfile {
    /// Per-node reduced aerodynamic coefficients.
    pub coefficients: Vec<ScvxReducedAeroCoefficients>,
    /// Per-node dynamic-pressure/reference-quantity samples.
    pub parameters: Vec<ScvxReducedAeroParameters>,
}

/// Per-node `openbmp-aero` method context profile for the SCvx rigid-body FOH
/// builder.
#[cfg(feature = "aero")]
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxAeroMethodReferenceProfile {
    /// Per-node aerodynamic method contexts.
    pub contexts: Vec<openbmp_aero::AeroContext>,
}

/// SCvx termination status.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ScvxTerminationStatus {
    /// Converged to declared tolerances.
    Converged,
    /// Reached the configured iteration budget.
    MaxIterations,
    /// Trust region was repeatedly rejected below the configured minimum.
    TrustRegionTooSmall,
}

/// One SCvx outer-iteration diagnostic row.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxIterationReport {
    /// Zero-based iteration index.
    pub iteration: usize,
    /// Whether the convex-subproblem step was accepted.
    pub accepted: bool,
    /// Trust-region radius used to build the subproblem.
    pub trust_region_radius: f64,
    /// Predicted merit reduction.
    pub predicted_reduction: f64,
    /// Actual merit reduction.
    pub actual_reduction: f64,
    /// Actual/predicted reduction ratio.
    pub rho: f64,
    /// Trial merit after the step.
    pub merit: f64,
    /// Maximum interval virtual-control norm in the accepted/trial solution.
    pub max_virtual_control_norm: f64,
    /// Maximum active state-triggered constraint violation.
    pub max_state_triggered_violation: f64,
    /// Maximum always-active linear state path-constraint violation.
    pub max_linear_state_violation: f64,
    /// Maximum always-active linear control path-constraint violation.
    pub max_linear_control_violation: f64,
    /// Maximum always-active second-order-cone path-constraint violation.
    pub max_second_order_cone_violation: f64,
}

/// Deterministic SCvx solution report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxSolution {
    /// Clarabel status from the final accepted/trial subproblem.
    pub solver_status: SolverStatus,
    /// SCvx termination status.
    pub termination_status: ScvxTerminationStatus,
    /// Whether termination met all declared convergence tolerances.
    pub converged: bool,
    /// Accepted state trajectory, row-major by node.
    pub states: Vec<f64>,
    /// Accepted control trajectory, row-major by node.
    pub controls: Vec<f64>,
    /// Accepted virtual controls, row-major by interval.
    pub virtual_controls: Vec<f64>,
    /// Final merit.
    pub merit: f64,
    /// Terminal target-error norm.
    pub terminal_error_norm: f64,
    /// Maximum interval virtual-control norm.
    pub max_virtual_control_norm: f64,
    /// Maximum active state-triggered constraint violation.
    pub max_state_triggered_violation: f64,
    /// Maximum always-active linear state path-constraint violation.
    pub max_linear_state_violation: f64,
    /// Maximum always-active linear control path-constraint violation.
    pub max_linear_control_violation: f64,
    /// Maximum always-active second-order-cone path-constraint violation.
    pub max_second_order_cone_violation: f64,
    /// Outer-iteration trace.
    pub iterations: Vec<ScvxIterationReport>,
}

/// Settings for repeatedly rebuilding a continuous-dynamics FOH SCvx problem.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxContinuousFohSettings {
    /// Maximum continuous-dynamics relinearization iterations.
    pub max_outer_iterations: usize,
    /// Maximum allowed accepted-reference state/control update for convergence.
    pub reference_update_tolerance: f64,
    /// Settings for each discrete-affine SCvx subproblem.
    pub scvx_settings: ScvxSettings,
    /// Settings for each forward-Euler/FOH linearization pass.
    pub linearization_settings: ScvxForwardEulerLinearizationSettings,
}

impl Default for ScvxContinuousFohSettings {
    fn default() -> Self {
        Self {
            max_outer_iterations: 4,
            reference_update_tolerance: 1.0e-6,
            scvx_settings: ScvxSettings::default(),
            linearization_settings: ScvxForwardEulerLinearizationSettings::default(),
        }
    }
}

/// One continuous-dynamics FOH SCvx outer-loop diagnostic row.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohIterationReport {
    /// Zero-based outer-loop iteration.
    pub iteration: usize,
    /// Whether the rebuilt discrete-affine subproblem converged.
    pub affine_converged: bool,
    /// Number of inner discrete-affine SCvx iterations.
    pub affine_iteration_count: usize,
    /// Terminal target-error norm from the subproblem solution.
    pub terminal_error_norm: f64,
    /// Maximum interval virtual-control norm from the subproblem solution.
    pub max_virtual_control_norm: f64,
    /// Maximum state-reference update from the previous linearization point.
    pub max_state_update_norm: f64,
    /// Maximum control-reference update from the previous linearization point.
    pub max_control_update_norm: f64,
}

/// Continuous-dynamics FOH SCvx solve report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohSolution {
    /// Final discrete-affine SCvx solution.
    pub solution: ScvxSolution,
    /// True when the final subproblem converged and reference updates met the
    /// declared tolerance.
    pub converged: bool,
    /// Outer-loop relinearization diagnostics.
    pub outer_iterations: Vec<ScvxContinuousFohIterationReport>,
}

/// Deterministic golden-section time-of-flight search settings for SCvx.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxTimeOfFlightSearchSettings {
    /// Inclusive lower time-of-flight bound in seconds.
    pub lower_time_of_flight_s: f64,
    /// Inclusive upper time-of-flight bound in seconds.
    pub upper_time_of_flight_s: f64,
    /// Stop once the bracket width is no larger than this tolerance.
    pub tolerance_s: f64,
    /// Maximum number of outer golden-section iterations.
    pub max_iterations: usize,
    /// Non-negative time penalty added to the accepted SCvx merit.
    pub time_penalty_weight: f64,
    /// Settings used for each fixed-time SCvx candidate solve.
    pub scvx_settings: ScvxSettings,
}

/// Deterministic golden-section time-of-flight search settings for
/// caller-built continuous-FOH SCvx candidates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScvxContinuousFohTimeOfFlightSearchSettings {
    /// Inclusive lower time-of-flight bound in seconds.
    pub lower_time_of_flight_s: f64,
    /// Inclusive upper time-of-flight bound in seconds.
    pub upper_time_of_flight_s: f64,
    /// Stop once the bracket width is no larger than this tolerance.
    pub tolerance_s: f64,
    /// Maximum number of outer golden-section iterations.
    pub max_iterations: usize,
    /// Non-negative time penalty added to the accepted continuous-FOH merit.
    pub time_penalty_weight: f64,
}

/// One fixed-time candidate evaluated by deterministic SCvx free-final-time
/// search.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxTimeOfFlightCandidateReport {
    /// Candidate time of flight in seconds.
    pub time_of_flight_s: f64,
    /// Search objective for this candidate.
    pub objective: f64,
    /// Accepted SCvx merit before applying the time penalty.
    pub merit: f64,
    /// Terminal target error norm reported by the fixed-time solve.
    pub terminal_error_norm: f64,
    /// Maximum interval virtual-control norm reported by the fixed-time solve.
    pub max_virtual_control_norm: f64,
    /// Maximum always-active linear state path-constraint violation.
    pub max_linear_state_violation: f64,
    /// Maximum always-active linear control path-constraint violation.
    pub max_linear_control_violation: f64,
    /// Maximum always-active second-order-cone path-constraint violation.
    pub max_second_order_cone_violation: f64,
    /// Fixed-time SCvx iteration count.
    pub iteration_count: usize,
    /// Fixed-time SCvx termination status.
    pub termination_status: ScvxTerminationStatus,
}

/// One fixed-time continuous-FOH candidate evaluated by deterministic SCvx
/// free-final-time search.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohTimeOfFlightCandidateReport {
    /// Candidate time of flight in seconds.
    pub time_of_flight_s: f64,
    /// Search objective for this candidate.
    pub objective: f64,
    /// Accepted inner SCvx merit before applying the time penalty.
    pub merit: f64,
    /// Whether the continuous-FOH outer loop converged.
    pub converged: bool,
    /// Whether the final rebuilt affine subproblem converged.
    pub affine_converged: bool,
    /// Terminal target error norm reported by the final fixed-time solve.
    pub terminal_error_norm: f64,
    /// Maximum interval virtual-control norm reported by the final fixed-time solve.
    pub max_virtual_control_norm: f64,
    /// Maximum always-active linear state path-constraint violation.
    pub max_linear_state_violation: f64,
    /// Maximum always-active linear control path-constraint violation.
    pub max_linear_control_violation: f64,
    /// Maximum always-active second-order-cone path-constraint violation.
    pub max_second_order_cone_violation: f64,
    /// Continuous-FOH outer iteration count.
    pub outer_iteration_count: usize,
    /// Final fixed-time affine SCvx iteration count.
    pub affine_iteration_count: usize,
    /// Maximum state-reference update reported by the final outer iteration.
    pub max_state_update_norm: f64,
    /// Maximum control-reference update reported by the final outer iteration.
    pub max_control_update_norm: f64,
    /// Final fixed-time affine SCvx termination status.
    pub termination_status: ScvxTerminationStatus,
}

/// Evaluate one fixed-time candidate with the same settings used by
/// deterministic SCvx free-final-time search.
///
/// Unlike [`solve_scvx_time_of_flight_search`], this diagnostic helper returns
/// a report even when the fixed-time candidate does not converge. It is intended
/// for benchmark-gap audits and failed-window diagnostics.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings are malformed, `time_of_flight_s` is
/// outside the search bracket, the builder emits a malformed problem, the
/// fixed-time solve fails, or the candidate objective is non-finite.
pub fn evaluate_scvx_time_of_flight_candidate<F>(
    settings: ScvxTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
    mut build_problem: F,
) -> Result<ScvxTimeOfFlightCandidateReport, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxDiscreteAffineProblem, TrajoptError>,
{
    validate_scvx_time_search_settings(settings)?;
    validate_scvx_time_of_flight_candidate_time(settings, time_of_flight_s)?;
    Ok(evaluate_scvx_time_candidate(&mut build_problem, settings, time_of_flight_s)?.report())
}

/// Evaluate one fixed-time continuous-FOH candidate with the same settings used
/// by deterministic continuous-FOH SCvx free-final-time search.
///
/// Unlike [`solve_scvx_continuous_foh_time_of_flight_search`], this diagnostic
/// helper returns a report even when the fixed-time continuous relinearization
/// loop does not converge.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings are malformed, `time_of_flight_s` is
/// outside the search bracket, the candidate callback fails, or the candidate
/// objective is non-finite.
pub fn evaluate_scvx_continuous_foh_time_of_flight_candidate<F>(
    settings: ScvxContinuousFohTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
    mut solve_candidate: F,
) -> Result<ScvxContinuousFohTimeOfFlightCandidateReport, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxContinuousFohSolution, TrajoptError>,
{
    validate_scvx_continuous_foh_time_search_settings(settings)?;
    validate_scvx_continuous_foh_time_of_flight_candidate_time(settings, time_of_flight_s)?;
    Ok(evaluate_scvx_continuous_foh_time_candidate(
        &mut solve_candidate,
        settings,
        time_of_flight_s,
    )?
    .report())
}

/// Result of deterministic SCvx free-final-time search.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxTimeOfFlightSearchReport {
    /// Best fixed-time SCvx solution found by the search.
    pub solution: ScvxSolution,
    /// Search objective for `solution`.
    pub objective: f64,
    /// Time of flight associated with `solution`.
    pub best_time_of_flight_s: f64,
    /// Final lower bracket bound in seconds.
    pub final_lower_time_of_flight_s: f64,
    /// Final upper bracket bound in seconds.
    pub final_upper_time_of_flight_s: f64,
    /// Number of golden-section iterations completed.
    pub iterations: usize,
    /// Number of fixed-time SCvx candidate solves performed.
    pub candidate_count: usize,
    /// Ordered diagnostics for every converged fixed-time candidate evaluated.
    pub candidates: Vec<ScvxTimeOfFlightCandidateReport>,
}

/// Result of deterministic continuous-FOH SCvx free-final-time search.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohTimeOfFlightSearchReport {
    /// Best continuous-FOH SCvx solution found by the search.
    pub solution: ScvxContinuousFohSolution,
    /// Search objective for `solution`.
    pub objective: f64,
    /// Time of flight associated with `solution`.
    pub best_time_of_flight_s: f64,
    /// Final lower bracket bound in seconds.
    pub final_lower_time_of_flight_s: f64,
    /// Final upper bracket bound in seconds.
    pub final_upper_time_of_flight_s: f64,
    /// Number of golden-section iterations completed.
    pub iterations: usize,
    /// Number of fixed-time continuous-FOH candidate solves performed.
    pub candidate_count: usize,
    /// Ordered diagnostics for every converged fixed-time continuous-FOH
    /// candidate evaluated.
    pub candidates: Vec<ScvxContinuousFohTimeOfFlightCandidateReport>,
}

/// Settings for sweeping several continuous-FOH SCvx free-final-time search
/// windows.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohTimeOfFlightSweepSettings {
    /// Ordered center times for each search window.
    pub center_times_of_flight_s: Vec<f64>,
    /// Positive half-width applied around each center time.
    pub half_width_s: f64,
    /// Stop each window once its bracket width is no larger than this tolerance.
    pub tolerance_s: f64,
    /// Maximum number of golden-section iterations per window.
    pub max_iterations: usize,
    /// Non-negative time penalty added to each accepted continuous-FOH merit.
    pub time_penalty_weight: f64,
}

/// One continuous-FOH window evaluated by
/// [`solve_scvx_continuous_foh_time_of_flight_sweep`].
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohTimeOfFlightSweepWindowReport {
    /// Zero-based window index in the caller-provided center list.
    pub window_index: usize,
    /// Center time of flight for this window.
    pub center_time_of_flight_s: f64,
    /// Lower bound of this window.
    pub lower_time_of_flight_s: f64,
    /// Upper bound of this window.
    pub upper_time_of_flight_s: f64,
    /// Whether this window produced a converged search report.
    pub converged: bool,
    /// Search report for converged windows.
    pub search: Option<ScvxContinuousFohTimeOfFlightSearchReport>,
    /// Stable failure reason for windows that failed to produce a search report.
    pub failure_reason: Option<&'static str>,
    /// Fixed-time diagnostic retained for failed windows when it can be evaluated.
    pub failure_candidate: Option<ScvxContinuousFohTimeOfFlightCandidateReport>,
}

/// Deterministic multi-window continuous-FOH SCvx free-final-time sweep report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxContinuousFohTimeOfFlightSweepReport {
    /// Number of windows evaluated.
    pub window_count: usize,
    /// Number of windows that produced converged search reports.
    pub converged_window_count: usize,
    /// Index of the best converged window, if any.
    pub best_window_index: Option<usize>,
    /// Best time of flight across all converged windows, if any.
    pub best_time_of_flight_s: Option<f64>,
    /// Best objective across all converged windows, if any.
    pub best_objective: Option<f64>,
    /// Ordered per-window diagnostics.
    pub windows: Vec<ScvxContinuousFohTimeOfFlightSweepWindowReport>,
}

/// Settings for sweeping several SCvx free-final-time search windows.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxTimeOfFlightSweepSettings {
    /// Ordered center times for each search window.
    pub center_times_of_flight_s: Vec<f64>,
    /// Positive half-width applied around each center time.
    pub half_width_s: f64,
    /// Stop each window once its bracket width is no larger than this tolerance.
    pub tolerance_s: f64,
    /// Maximum number of golden-section iterations per window.
    pub max_iterations: usize,
    /// Non-negative time penalty added to each accepted SCvx merit.
    pub time_penalty_weight: f64,
    /// Settings used for each fixed-time SCvx candidate solve.
    pub scvx_settings: ScvxSettings,
}

/// One window evaluated by [`solve_scvx_time_of_flight_sweep`].
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxTimeOfFlightSweepWindowReport {
    /// Zero-based window index in the caller-provided center list.
    pub window_index: usize,
    /// Center time of flight for this window.
    pub center_time_of_flight_s: f64,
    /// Lower bound of this window.
    pub lower_time_of_flight_s: f64,
    /// Upper bound of this window.
    pub upper_time_of_flight_s: f64,
    /// Whether this window produced a converged search report.
    pub converged: bool,
    /// Search report for converged windows.
    pub search: Option<ScvxTimeOfFlightSearchReport>,
    /// Stable failure reason for windows that failed to produce a search report.
    pub failure_reason: Option<&'static str>,
    /// Fixed-time diagnostic retained for failed windows when it can be evaluated.
    pub failure_candidate: Option<ScvxTimeOfFlightCandidateReport>,
}

/// Deterministic multi-window SCvx free-final-time sweep report.
#[derive(Clone, Debug, PartialEq)]
pub struct ScvxTimeOfFlightSweepReport {
    /// Number of windows evaluated.
    pub window_count: usize,
    /// Number of windows that produced converged search reports.
    pub converged_window_count: usize,
    /// Index of the best converged window, if any.
    pub best_window_index: Option<usize>,
    /// Best time of flight across all converged windows, if any.
    pub best_time_of_flight_s: Option<f64>,
    /// Best objective across all converged windows, if any.
    pub best_objective: Option<f64>,
    /// Ordered per-window diagnostics.
    pub windows: Vec<ScvxTimeOfFlightSweepWindowReport>,
}

/// Solve a discrete affine SCvx problem with deterministic dense QP
/// subproblems.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the problem/settings are malformed or a
/// convex subproblem does not solve.
pub fn solve_scvx_discrete_affine(
    problem: &ScvxDiscreteAffineProblem,
    settings: ScvxSettings,
) -> Result<ScvxSolution, TrajoptError> {
    validate_scvx_problem(problem)?;
    validate_scvx_settings(settings)?;

    let layout = ScvxDecisionLayout::new(problem);
    let mut states = problem.initial_states.clone();
    let mut controls = problem.initial_controls.clone();
    let mut virtual_controls = dynamics_defects(problem, &states, &controls)?;
    let mut trust_region = settings.initial_trust_region_radius;
    let mut merit = scvx_merit(problem, &settings, &states, &controls, &virtual_controls)?;
    let mut iteration_reports = Vec::new();
    let mut last_status = SolverStatus::Unsolved;

    for iteration in 0..settings.max_iterations {
        let active_constraints = active_state_triggered_constraints(problem, &states)?;
        let qp = build_scvx_qp(
            problem,
            &settings,
            &layout,
            &states,
            &controls,
            trust_region,
            &active_constraints,
        )?;
        let qp_solution = solve_scvx_convex_subproblem(&qp, problem, &layout)?;
        last_status = qp_solution.status;
        let trial_states = decode_states(problem, &layout, &qp_solution.primal);
        let trial_controls = decode_controls(problem, &layout, &qp_solution.primal);
        let trial_virtual_controls = decode_virtual_controls(problem, &layout, &qp_solution.primal);
        let trial_merit = scvx_merit(
            problem,
            &settings,
            &trial_states,
            &trial_controls,
            &trial_virtual_controls,
        )?;
        let predicted_reduction = merit - trial_merit;
        let actual_reduction = merit - trial_merit;
        let rho = reduction_ratio(actual_reduction, predicted_reduction);
        let accepted =
            actual_reduction >= -settings.terminal_tolerance && rho >= settings.accept_ratio;

        let max_virtual = max_interval_norm(
            &trial_virtual_controls,
            problem.horizon_steps,
            problem.state_dimension,
        );
        let max_triggered_violation = max_state_triggered_violation(problem, &trial_states)?;
        let max_linear_state_violation = max_linear_state_violation(problem, &trial_states)?;
        let max_linear_control_violation = max_linear_control_violation(problem, &trial_controls)?;
        let max_second_order_cone_violation =
            max_second_order_cone_violation(problem, &trial_states, &trial_controls)?;
        iteration_reports.push(ScvxIterationReport {
            iteration,
            accepted,
            trust_region_radius: trust_region,
            predicted_reduction,
            actual_reduction,
            rho,
            merit: trial_merit,
            max_virtual_control_norm: max_virtual,
            max_state_triggered_violation: max_triggered_violation,
            max_linear_state_violation,
            max_linear_control_violation,
            max_second_order_cone_violation,
        });

        if accepted {
            states = trial_states;
            controls = trial_controls;
            virtual_controls = trial_virtual_controls;
            merit = trial_merit;
            trust_region =
                (trust_region * settings.trust_region_expand).min(settings.max_trust_region_radius);

            let terminal_error = terminal_error_norm(problem, &states);
            if terminal_error <= settings.terminal_tolerance
                && max_virtual <= settings.virtual_control_tolerance
                && max_triggered_violation <= settings.constraint_tolerance
                && max_linear_state_violation <= settings.constraint_tolerance
                && max_linear_control_violation <= settings.constraint_tolerance
                && max_second_order_cone_violation <= settings.constraint_tolerance
            {
                return Ok(solution_report(
                    problem,
                    last_status,
                    ScvxTerminationStatus::Converged,
                    true,
                    ScvxTrajectory {
                        states,
                        controls,
                        virtual_controls,
                    },
                    merit,
                    iteration_reports,
                ));
            }
        } else {
            trust_region *= settings.trust_region_shrink;
            if trust_region < settings.min_trust_region_radius {
                return Ok(solution_report(
                    problem,
                    last_status,
                    ScvxTerminationStatus::TrustRegionTooSmall,
                    false,
                    ScvxTrajectory {
                        states,
                        controls,
                        virtual_controls,
                    },
                    merit,
                    iteration_reports,
                ));
            }
        }
    }

    Ok(solution_report(
        problem,
        last_status,
        ScvxTerminationStatus::MaxIterations,
        false,
        ScvxTrajectory {
            states,
            controls,
            virtual_controls,
        },
        merit,
        iteration_reports,
    ))
}

/// Solve a continuous-dynamics forward-Euler/FOH SCvx problem by repeatedly
/// rebuilding the affine discretization around the accepted trajectory.
///
/// This is the nonlinear-model wrapper above [`build_scvx_forward_euler_foh_problem`]
/// and [`solve_scvx_discrete_affine`]. It stops once the rebuilt affine
/// subproblem converges and the accepted state/control reference update is below
/// `settings.reference_update_tolerance`.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings, the initial definition, a rebuilt
/// affine problem, or the continuous dynamics callback are malformed.
pub fn solve_scvx_forward_euler_foh<F>(
    definition: &ScvxForwardEulerFohProblemDefinition,
    dynamics: F,
    settings: ScvxContinuousFohSettings,
) -> Result<ScvxContinuousFohSolution, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
{
    solve_scvx_forward_euler_foh_with_reference_projection(
        definition,
        dynamics,
        settings,
        |_states| Ok(()),
    )
}

/// Solve a continuous-dynamics forward-Euler/FOH SCvx problem with a caller
/// supplied projection applied to the state reference before each
/// relinearization.
///
/// The projection is applied to the reference trajectory used for the next
/// affine rebuild. The returned [`ScvxSolution`] remains the accepted convex
/// subproblem solution, while `max_state_update_norm` is measured against the
/// projected next reference.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings, the initial definition, a rebuilt
/// affine problem, the continuous dynamics callback, or the reference-state
/// projection are malformed.
pub fn solve_scvx_forward_euler_foh_with_reference_projection<F, P>(
    definition: &ScvxForwardEulerFohProblemDefinition,
    dynamics: F,
    settings: ScvxContinuousFohSettings,
    mut project_reference_states: P,
) -> Result<ScvxContinuousFohSolution, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
    P: FnMut(&mut [f64]) -> Result<(), TrajoptError>,
{
    solve_scvx_forward_euler_foh_with_reference_update(
        definition,
        dynamics,
        settings,
        |reference| project_reference_states(&mut reference.initial_states),
    )
}

/// Solve a continuous-dynamics forward-Euler/FOH SCvx problem with a caller
/// supplied update applied to the full reference definition before each
/// relinearization.
///
/// The update is applied to the initial reference and to each accepted
/// state/control trajectory before the next affine rebuild. This supports
/// reference-dependent path rows, such as powered-landing body-tilt rows
/// linearized around the current quaternion reference. The returned
/// [`ScvxSolution`] remains the accepted convex subproblem solution, while
/// `max_state_update_norm` and `max_control_update_norm` are measured against
/// the updated next reference.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings, the initial definition, a rebuilt
/// affine problem, the continuous dynamics callback, or the reference update are
/// malformed.
pub fn solve_scvx_forward_euler_foh_with_reference_update<F, U>(
    definition: &ScvxForwardEulerFohProblemDefinition,
    mut dynamics: F,
    settings: ScvxContinuousFohSettings,
    mut update_reference: U,
) -> Result<ScvxContinuousFohSolution, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
    U: FnMut(&mut ScvxForwardEulerFohProblemDefinition) -> Result<(), TrajoptError>,
{
    validate_scvx_continuous_foh_settings(settings)?;
    validate_scvx_foh_problem_definition(definition, settings.linearization_settings)?;

    let mut reference = definition.clone();
    update_reference(&mut reference)?;
    validate_scvx_foh_problem_definition(&reference, settings.linearization_settings)?;
    let mut outer_iterations = Vec::with_capacity(settings.max_outer_iterations);
    let mut last_solution = None;

    for iteration in 0..settings.max_outer_iterations {
        let problem = build_scvx_forward_euler_foh_problem(
            &reference,
            &mut dynamics,
            settings.linearization_settings,
        )?;
        let solution = solve_scvx_discrete_affine(&problem, settings.scvx_settings)?;
        let mut next_reference = reference.clone();
        next_reference.initial_states = solution.states.clone();
        next_reference.initial_controls = solution.controls.clone();
        update_reference(&mut next_reference)?;
        validate_scvx_foh_problem_definition(&next_reference, settings.linearization_settings)?;
        let max_state_update_norm =
            max_abs_difference(&next_reference.initial_states, &reference.initial_states);
        let max_control_update_norm = max_abs_difference(
            &next_reference.initial_controls,
            &reference.initial_controls,
        );
        let converged = solution.converged
            && max_state_update_norm <= settings.reference_update_tolerance
            && max_control_update_norm <= settings.reference_update_tolerance;
        outer_iterations.push(ScvxContinuousFohIterationReport {
            iteration,
            affine_converged: solution.converged,
            affine_iteration_count: solution.iterations.len(),
            terminal_error_norm: solution.terminal_error_norm,
            max_virtual_control_norm: solution.max_virtual_control_norm,
            max_state_update_norm,
            max_control_update_norm,
        });

        if converged {
            return Ok(ScvxContinuousFohSolution {
                solution,
                converged: true,
                outer_iterations,
            });
        }
        if !solution.converged {
            return Ok(ScvxContinuousFohSolution {
                solution,
                converged: false,
                outer_iterations,
            });
        }

        reference = next_reference;
        last_solution = Some(solution);
    }

    let solution = last_solution.ok_or(TrajoptError::InvalidPayload {
        reason: "SCvx continuous FOH outer loop produced no solution",
    })?;
    Ok(ScvxContinuousFohSolution {
        solution,
        converged: false,
        outer_iterations,
    })
}

/// Solve a free-final-time SCvx problem by deterministic golden-section search
/// over fixed-time discrete-affine SCvx candidates.
///
/// `build_problem` receives the candidate time of flight and returns the
/// corresponding fixed-time [`ScvxDiscreteAffineProblem`]. The search objective
/// is `solution.merit + time_penalty_weight * time_of_flight_s`. Candidate
/// solves that do not converge return an error instead of being silently ranked.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings are malformed, the builder emits a
/// malformed problem, a candidate solve fails, or a candidate does not converge.
pub fn solve_scvx_time_of_flight_search<F>(
    settings: ScvxTimeOfFlightSearchSettings,
    mut build_problem: F,
) -> Result<ScvxTimeOfFlightSearchReport, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxDiscreteAffineProblem, TrajoptError>,
{
    validate_scvx_time_search_settings(settings)?;

    let inv_phi = (5.0_f64.sqrt() - 1.0) * 0.5;
    let inv_phi_sq = 1.0 - inv_phi;
    let mut lower = settings.lower_time_of_flight_s;
    let mut upper = settings.upper_time_of_flight_s;
    let mut left_time = lower + inv_phi_sq * (upper - lower);
    let mut right_time = lower + inv_phi * (upper - lower);
    let mut left = solve_scvx_with_time_of_flight(&mut build_problem, settings, left_time)?;
    let mut right = solve_scvx_with_time_of_flight(&mut build_problem, settings, right_time)?;
    let mut candidates = vec![left.report(), right.report()];
    let mut candidate_count = 2;
    let mut iterations = 0;
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
            left = solve_scvx_with_time_of_flight(&mut build_problem, settings, left_time)?;
            candidates.push(left.report());
            candidate_count += 1;
        } else {
            lower = left_time;
            left_time = right_time;
            left = right;
            right_time = lower + inv_phi * (upper - lower);
            right = solve_scvx_with_time_of_flight(&mut build_problem, settings, right_time)?;
            candidates.push(right.report());
            candidate_count += 1;
        }
        iterations += 1;

        let candidate = if left.objective <= right.objective {
            &left
        } else {
            &right
        };
        if candidate.objective < best.objective
            || (candidate.objective.to_bits() == best.objective.to_bits()
                && candidate.time_of_flight_s < best.time_of_flight_s)
        {
            best = candidate.clone();
        }
    }

    Ok(ScvxTimeOfFlightSearchReport {
        solution: best.solution,
        objective: best.objective,
        best_time_of_flight_s: best.time_of_flight_s,
        final_lower_time_of_flight_s: lower,
        final_upper_time_of_flight_s: upper,
        iterations,
        candidate_count,
        candidates,
    })
}

/// Solve a free-final-time continuous-FOH SCvx problem by deterministic
/// golden-section search over caller-built fixed-time continuous candidates.
///
/// `solve_candidate` receives the candidate time of flight and returns the
/// corresponding fixed-time [`ScvxContinuousFohSolution`], typically by calling
/// [`solve_scvx_forward_euler_foh`] or
/// [`solve_scvx_forward_euler_foh_with_reference_update`]. The search objective
/// is `solution.solution.merit + time_penalty_weight * time_of_flight_s`.
/// Candidate solves whose continuous outer loop does not converge return an
/// error instead of being silently ranked.
///
/// # Errors
///
/// Returns [`TrajoptError`] when settings are malformed, the callback fails, a
/// candidate objective is non-finite, or a candidate does not converge.
pub fn solve_scvx_continuous_foh_time_of_flight_search<F>(
    settings: ScvxContinuousFohTimeOfFlightSearchSettings,
    mut solve_candidate: F,
) -> Result<ScvxContinuousFohTimeOfFlightSearchReport, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxContinuousFohSolution, TrajoptError>,
{
    validate_scvx_continuous_foh_time_search_settings(settings)?;

    let inv_phi = (5.0_f64.sqrt() - 1.0) * 0.5;
    let inv_phi_sq = 1.0 - inv_phi;
    let mut lower = settings.lower_time_of_flight_s;
    let mut upper = settings.upper_time_of_flight_s;
    let mut left_time = lower + inv_phi_sq * (upper - lower);
    let mut right_time = lower + inv_phi * (upper - lower);
    let mut left =
        solve_scvx_continuous_foh_with_time_of_flight(&mut solve_candidate, settings, left_time)?;
    let mut right =
        solve_scvx_continuous_foh_with_time_of_flight(&mut solve_candidate, settings, right_time)?;
    let mut candidates = vec![left.report(), right.report()];
    let mut candidate_count = 2;
    let mut iterations = 0;
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
            left = solve_scvx_continuous_foh_with_time_of_flight(
                &mut solve_candidate,
                settings,
                left_time,
            )?;
            candidates.push(left.report());
            candidate_count += 1;
        } else {
            lower = left_time;
            left_time = right_time;
            left = right;
            right_time = lower + inv_phi * (upper - lower);
            right = solve_scvx_continuous_foh_with_time_of_flight(
                &mut solve_candidate,
                settings,
                right_time,
            )?;
            candidates.push(right.report());
            candidate_count += 1;
        }
        iterations += 1;

        let candidate = if left.objective <= right.objective {
            &left
        } else {
            &right
        };
        if candidate.objective < best.objective
            || (candidate.objective.to_bits() == best.objective.to_bits()
                && candidate.time_of_flight_s < best.time_of_flight_s)
        {
            best = candidate.clone();
        }
    }

    Ok(ScvxContinuousFohTimeOfFlightSearchReport {
        solution: best.solution,
        objective: best.objective,
        best_time_of_flight_s: best.time_of_flight_s,
        final_lower_time_of_flight_s: lower,
        final_upper_time_of_flight_s: upper,
        iterations,
        candidate_count,
        candidates,
    })
}

/// Sweep several continuous-FOH SCvx free-final-time search windows and report
/// convergence for each one.
///
/// Unlike [`solve_scvx_continuous_foh_time_of_flight_search`], a failed
/// candidate/window does not abort the whole sweep. Malformed sweep settings
/// still fail closed, but individual window failures are captured in the
/// returned diagnostics. When possible, the failed window retains the center
/// fixed-time continuous-FOH candidate report.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the sweep settings are malformed.
pub fn solve_scvx_continuous_foh_time_of_flight_sweep<F>(
    settings: ScvxContinuousFohTimeOfFlightSweepSettings,
    mut solve_candidate: F,
) -> Result<ScvxContinuousFohTimeOfFlightSweepReport, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxContinuousFohSolution, TrajoptError>,
{
    validate_scvx_continuous_foh_time_sweep_settings(&settings)?;

    let mut windows = Vec::with_capacity(settings.center_times_of_flight_s.len());
    let mut best_window_index = None;
    let mut best_time_of_flight_s = None;
    let mut best_objective = None;

    for (window_index, &center_time_of_flight_s) in
        settings.center_times_of_flight_s.iter().enumerate()
    {
        let lower_time_of_flight_s = center_time_of_flight_s - settings.half_width_s;
        let upper_time_of_flight_s = center_time_of_flight_s + settings.half_width_s;
        let search_settings = ScvxContinuousFohTimeOfFlightSearchSettings {
            lower_time_of_flight_s,
            upper_time_of_flight_s,
            tolerance_s: settings.tolerance_s,
            max_iterations: settings.max_iterations,
            time_penalty_weight: settings.time_penalty_weight,
        };

        match solve_scvx_continuous_foh_time_of_flight_search(search_settings, |time_of_flight_s| {
            solve_candidate(time_of_flight_s)
        }) {
            Ok(search) => {
                let is_best = match (best_objective, best_time_of_flight_s) {
                    (Some(current_objective), Some(current_time_of_flight_s)) => {
                        search.objective < current_objective
                            || (search.objective.to_bits() == current_objective.to_bits()
                                && search.best_time_of_flight_s < current_time_of_flight_s)
                    }
                    _ => true,
                };
                if is_best {
                    best_window_index = Some(window_index);
                    best_time_of_flight_s = Some(search.best_time_of_flight_s);
                    best_objective = Some(search.objective);
                }
                windows.push(ScvxContinuousFohTimeOfFlightSweepWindowReport {
                    window_index,
                    center_time_of_flight_s,
                    lower_time_of_flight_s,
                    upper_time_of_flight_s,
                    converged: true,
                    search: Some(search),
                    failure_reason: None,
                    failure_candidate: None,
                });
            }
            Err(error) => {
                let failure_candidate = evaluate_scvx_continuous_foh_time_candidate(
                    &mut solve_candidate,
                    search_settings,
                    center_time_of_flight_s,
                )
                .ok()
                .map(|candidate| candidate.report());
                windows.push(ScvxContinuousFohTimeOfFlightSweepWindowReport {
                    window_index,
                    center_time_of_flight_s,
                    lower_time_of_flight_s,
                    upper_time_of_flight_s,
                    converged: false,
                    search: None,
                    failure_reason: Some(trajopt_error_static_reason(&error)),
                    failure_candidate,
                });
            }
        }
    }

    Ok(ScvxContinuousFohTimeOfFlightSweepReport {
        window_count: windows.len(),
        converged_window_count: windows.iter().filter(|window| window.converged).count(),
        best_window_index,
        best_time_of_flight_s,
        best_objective,
        windows,
    })
}

/// Sweep several SCvx free-final-time search windows and report convergence for
/// each one.
///
/// Unlike [`solve_scvx_time_of_flight_search`], a failed candidate/window does
/// not abort the whole sweep. Malformed sweep settings still fail closed, but
/// individual window failures are captured in the returned diagnostics so
/// benchmark evidence can state exactly which initial-time windows converged.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the sweep settings are malformed.
pub fn solve_scvx_time_of_flight_sweep<F>(
    settings: ScvxTimeOfFlightSweepSettings,
    mut build_problem: F,
) -> Result<ScvxTimeOfFlightSweepReport, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxDiscreteAffineProblem, TrajoptError>,
{
    validate_scvx_time_sweep_settings(&settings)?;

    let mut windows = Vec::with_capacity(settings.center_times_of_flight_s.len());
    let mut converged_window_count = 0usize;
    let mut best_window_index = None;
    let mut best_time_of_flight_s = None;
    let mut best_objective = None;

    for (window_index, &center_time_of_flight_s) in
        settings.center_times_of_flight_s.iter().enumerate()
    {
        let lower_time_of_flight_s = center_time_of_flight_s - settings.half_width_s;
        let upper_time_of_flight_s = center_time_of_flight_s + settings.half_width_s;
        let search_settings = ScvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s,
            upper_time_of_flight_s,
            tolerance_s: settings.tolerance_s,
            max_iterations: settings.max_iterations,
            time_penalty_weight: settings.time_penalty_weight,
            scvx_settings: settings.scvx_settings,
        };

        match solve_scvx_time_of_flight_search(search_settings, |time_of_flight_s| {
            build_problem(time_of_flight_s)
        }) {
            Ok(search) => {
                let is_best = match (best_objective, best_time_of_flight_s) {
                    (Some(current_objective), Some(current_time_of_flight_s)) => {
                        search.objective < current_objective
                            || (search.objective.to_bits() == current_objective.to_bits()
                                && search.best_time_of_flight_s < current_time_of_flight_s)
                    }
                    _ => true,
                };
                if is_best {
                    best_window_index = Some(window_index);
                    best_time_of_flight_s = Some(search.best_time_of_flight_s);
                    best_objective = Some(search.objective);
                }
                converged_window_count += 1;
                windows.push(ScvxTimeOfFlightSweepWindowReport {
                    window_index,
                    center_time_of_flight_s,
                    lower_time_of_flight_s,
                    upper_time_of_flight_s,
                    converged: true,
                    search: Some(search),
                    failure_reason: None,
                    failure_candidate: None,
                });
            }
            Err(error) => {
                let failure_candidate = evaluate_scvx_time_candidate(
                    &mut build_problem,
                    search_settings,
                    center_time_of_flight_s,
                )
                .ok()
                .map(|candidate| candidate.report());
                windows.push(ScvxTimeOfFlightSweepWindowReport {
                    window_index,
                    center_time_of_flight_s,
                    lower_time_of_flight_s,
                    upper_time_of_flight_s,
                    converged: false,
                    search: None,
                    failure_reason: Some(trajopt_error_static_reason(&error)),
                    failure_candidate,
                });
            }
        }
    }

    Ok(ScvxTimeOfFlightSweepReport {
        window_count: windows.len(),
        converged_window_count,
        best_window_index,
        best_time_of_flight_s,
        best_objective,
        windows,
    })
}

/// Build a discrete affine SCvx problem by linearizing continuous dynamics over
/// a reference trajectory with forward-Euler state propagation and FOH controls.
///
/// This is the assembly counterpart to [`linearize_scvx_forward_euler_foh`]:
/// it validates the reference problem definition, linearizes the supplied
/// continuous dynamics, and copies the target, bounds, seeds, and
/// state-triggered, linear-state, and linear-control constraints into a
/// [`ScvxDiscreteAffineProblem`].
///
/// # Errors
///
/// Returns [`TrajoptError`] when the definition, settings, dynamics callback, or
/// generated affine problem is malformed.
pub fn build_scvx_forward_euler_foh_problem<F>(
    definition: &ScvxForwardEulerFohProblemDefinition,
    mut dynamics: F,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
{
    validate_scvx_foh_problem_definition(definition, linearization_settings)?;
    let dynamics_segments = linearize_scvx_forward_euler_foh(
        definition.state_dimension,
        definition.control_dimension,
        &definition.times_s,
        &definition.initial_states,
        &definition.initial_controls,
        &mut dynamics,
        linearization_settings,
    )?;
    let problem = ScvxDiscreteAffineProblem {
        state_dimension: definition.state_dimension,
        control_dimension: definition.control_dimension,
        horizon_steps: definition.times_s.len() - 1,
        initial_state: definition.initial_state.clone(),
        target: definition.target.clone(),
        dynamics: dynamics_segments,
        initial_states: definition.initial_states.clone(),
        initial_controls: definition.initial_controls.clone(),
        control_lower: definition.control_lower.clone(),
        control_upper: definition.control_upper.clone(),
        state_triggered_constraints: definition.state_triggered_constraints.clone(),
        linear_state_constraints: definition.linear_state_constraints.clone(),
        linear_control_constraints: definition.linear_control_constraints.clone(),
        second_order_state_constraints: definition.second_order_state_constraints.clone(),
        second_order_control_constraints: definition.second_order_control_constraints.clone(),
    };
    validate_scvx_problem(&problem)?;
    Ok(problem)
}

/// Build a flat 13-state/6-control rigid-body SCvx problem from a reference
/// trajectory.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the definition dimensions are not the SCvx
/// rigid-body layout, the rigid-body model rejects a reference sample, or the
/// generated affine problem is malformed.
pub fn build_scvx_rigid_body_foh_problem(
    definition: &ScvxForwardEulerFohProblemDefinition,
    parameters: &ScvxRigidBodyDynamicsParameters,
    dynamics_settings: ScvxRigidBodyDynamicsSettings,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError> {
    validate_scvx_rigid_body_problem_definition(definition)?;
    build_scvx_forward_euler_foh_problem(
        definition,
        |_time_s, state, control, output| {
            evaluate_scvx_rigid_body_dynamics(state, control, parameters, dynamics_settings, output)
        },
        linearization_settings,
    )
}

/// Build a powered-landing SCvx problem from a reference trajectory.
///
/// This helper uses the 14-state mass/rigid-body layout and 3-axis body-thrust
/// control used by Szmuk-style powered landing examples.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the definition dimensions are not the powered
/// landing layouts, the dynamics model rejects a reference sample, or the
/// generated affine problem is malformed.
pub fn build_scvx_powered_landing_foh_problem(
    definition: &ScvxForwardEulerFohProblemDefinition,
    parameters: &ScvxPoweredLandingDynamicsParameters,
    dynamics_settings: ScvxRigidBodyDynamicsSettings,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError> {
    validate_scvx_powered_landing_problem_definition(definition)?;
    build_scvx_forward_euler_foh_problem(
        definition,
        |_time_s, state, control, output| {
            evaluate_scvx_powered_landing_dynamics(
                state,
                control,
                parameters,
                dynamics_settings,
                output,
            )
        },
        linearization_settings,
    )
}

/// Build a reduced-aero-coupled powered-landing SCvx problem from a reference
/// trajectory and per-node aero profile.
///
/// Each interval uses the reduced-aero profile sample at its left node. The
/// resulting affine segments therefore include mass-depleting body thrust,
/// gravity, thrust-arm torque, and reduced aerodynamic force/moment loads.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the problem definition, aero profile, dynamics
/// model, linearization settings, or generated affine problem is malformed.
pub fn build_scvx_reduced_aero_powered_landing_foh_problem(
    definition: &ScvxForwardEulerFohProblemDefinition,
    aero_profile: &ScvxReducedAeroReferenceProfile,
    powered_landing_parameters: &ScvxPoweredLandingDynamicsParameters,
    dynamics_settings: ScvxRigidBodyDynamicsSettings,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError> {
    validate_scvx_powered_landing_problem_definition(definition)?;
    validate_scvx_reduced_aero_reference_profile(aero_profile, definition.times_s.len())?;
    build_scvx_forward_euler_foh_problem(
        definition,
        |time_s, state, control, output| {
            let node = scvx_left_node_for_time(&definition.times_s, time_s)?;
            evaluate_scvx_reduced_aero_powered_landing_dynamics(
                state,
                control,
                aero_profile.coefficients[node],
                aero_profile.parameters[node],
                powered_landing_parameters,
                dynamics_settings,
                output,
            )
        },
        linearization_settings,
    )
}

/// Build a reduced-aero-coupled flat rigid-body SCvx problem from a reference
/// trajectory and per-node aero profile.
///
/// Each interval uses the reduced-aero profile sample at its left node. The
/// resulting affine segments therefore include the force/moment rotation and
/// rigid-body load addition used by
/// [`evaluate_scvx_aero_coupled_rigid_body_dynamics`].
///
/// # Errors
///
/// Returns [`TrajoptError`] when the problem definition, aero profile, dynamics
/// model, linearization settings, or generated affine problem is malformed.
pub fn build_scvx_reduced_aero_rigid_body_foh_problem(
    definition: &ScvxForwardEulerFohProblemDefinition,
    aero_profile: &ScvxReducedAeroReferenceProfile,
    rigid_body_parameters: &ScvxRigidBodyDynamicsParameters,
    dynamics_settings: ScvxRigidBodyDynamicsSettings,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError> {
    validate_scvx_rigid_body_problem_definition(definition)?;
    validate_scvx_reduced_aero_reference_profile(aero_profile, definition.times_s.len())?;
    build_scvx_forward_euler_foh_problem(
        definition,
        |time_s, state, control, output| {
            let node = scvx_left_node_for_time(&definition.times_s, time_s)?;
            evaluate_scvx_aero_coupled_rigid_body_dynamics(
                state,
                control,
                aero_profile.coefficients[node],
                aero_profile.parameters[node],
                rigid_body_parameters,
                dynamics_settings,
                output,
            )
        },
        linearization_settings,
    )
}

/// Build an `openbmp-aero`-method-coupled flat rigid-body SCvx problem from a
/// reference trajectory and per-node aero contexts.
///
/// Each interval uses the context at its left node. This keeps deck,
/// hypersonic, hybrid, and future [`openbmp_aero::AeroMethod`] providers behind
/// the optional `openbmp-trajopt/aero` feature while generating the same
/// discrete affine SCvx problem form as the reduced-aero builder.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the problem definition, context profile,
/// aero-method evaluation, dynamics model, linearization settings, or generated
/// affine problem is malformed.
#[cfg(feature = "aero")]
pub fn build_scvx_aero_method_rigid_body_foh_problem<M>(
    definition: &ScvxForwardEulerFohProblemDefinition,
    aero_method: &M,
    aero_profile: &ScvxAeroMethodReferenceProfile,
    rigid_body_parameters: &ScvxRigidBodyDynamicsParameters,
    dynamics_settings: ScvxRigidBodyDynamicsSettings,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError>
where
    M: openbmp_aero::AeroMethod + ?Sized,
{
    validate_scvx_rigid_body_problem_definition(definition)?;
    validate_scvx_aero_method_reference_profile(aero_profile, definition.times_s.len())?;
    build_scvx_forward_euler_foh_problem(
        definition,
        |time_s, state, control, output| {
            let node = scvx_left_node_for_time(&definition.times_s, time_s)?;
            evaluate_scvx_aero_method_coupled_rigid_body_dynamics(
                state,
                control,
                aero_method,
                &aero_profile.contexts[node],
                rigid_body_parameters,
                dynamics_settings,
                output,
            )
        },
        linearization_settings,
    )
}

/// Build an `openbmp-aero`-method-coupled powered-landing SCvx problem from a
/// reference trajectory and per-node aero contexts.
///
/// Each interval uses the context at its left node. The resulting affine
/// segments include mass-depleting body thrust, gravity, thrust-arm torque, and
/// feature-gated aerodynamic force/moment loads while keeping mass depletion
/// tied only to commanded thrust.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the problem definition, context profile,
/// aero-method evaluation, dynamics model, linearization settings, or generated
/// affine problem is malformed.
#[cfg(feature = "aero")]
pub fn build_scvx_aero_method_powered_landing_foh_problem<M>(
    definition: &ScvxForwardEulerFohProblemDefinition,
    aero_method: &M,
    aero_profile: &ScvxAeroMethodReferenceProfile,
    powered_landing_parameters: &ScvxPoweredLandingDynamicsParameters,
    dynamics_settings: ScvxRigidBodyDynamicsSettings,
    linearization_settings: ScvxForwardEulerLinearizationSettings,
) -> Result<ScvxDiscreteAffineProblem, TrajoptError>
where
    M: openbmp_aero::AeroMethod + ?Sized,
{
    validate_scvx_powered_landing_problem_definition(definition)?;
    validate_scvx_aero_method_reference_profile(aero_profile, definition.times_s.len())?;
    build_scvx_forward_euler_foh_problem(
        definition,
        |time_s, state, control, output| {
            let node = scvx_left_node_for_time(&definition.times_s, time_s)?;
            evaluate_scvx_aero_method_powered_landing_dynamics(
                state,
                control,
                aero_method,
                &aero_profile.contexts[node],
                powered_landing_parameters,
                dynamics_settings,
                output,
            )
        },
        linearization_settings,
    )
}

/// Linearize continuous dynamics into FOH-control discrete affine segments.
///
/// The callback evaluates `x_dot = f(t, x, u)` into the supplied output slice.
/// The helper central-differences state and midpoint-control Jacobians at each
/// reference node and returns first-order forward-Euler segments:
/// `x[k+1] = A x[k] + B_minus u[k] + B_plus u[k+1] + c + nu[k]`.
///
/// # Errors
///
/// Returns [`TrajoptError`] if dimensions, times, reference values, settings, or
/// callback outputs are malformed.
pub fn linearize_scvx_forward_euler_foh<F>(
    state_dimension: usize,
    control_dimension: usize,
    times_s: &[f64],
    reference_states: &[f64],
    reference_controls: &[f64],
    mut dynamics: F,
    settings: ScvxForwardEulerLinearizationSettings,
) -> Result<Vec<ScvxAffineDynamicsSegment>, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
{
    validate_scvx_linearization_inputs(
        state_dimension,
        control_dimension,
        times_s,
        reference_states,
        reference_controls,
        settings,
    )?;
    let horizon_steps = times_s.len() - 1;
    let mut segments = Vec::with_capacity(horizon_steps);

    for interval in 0..horizon_steps {
        let dt = times_s[interval + 1] - times_s[interval];
        let state_start = interval * state_dimension;
        let control_start = interval * control_dimension;
        let next_control_start = (interval + 1) * control_dimension;
        let state_ref = &reference_states[state_start..state_start + state_dimension];
        let control_left = &reference_controls[control_start..control_start + control_dimension];
        let control_right =
            &reference_controls[next_control_start..next_control_start + control_dimension];
        let mut control_mid = vec![0.0; control_dimension];
        for component in 0..control_dimension {
            control_mid[component] = 0.5 * (control_left[component] + control_right[component]);
        }

        let base = evaluate_scvx_continuous_dynamics(
            &mut dynamics,
            times_s[interval],
            state_ref,
            &control_mid,
            state_dimension,
        )?;
        let state_jacobian = finite_difference_scvx_state_jacobian(
            &mut dynamics,
            times_s[interval],
            state_ref,
            &control_mid,
            settings.finite_difference_step,
            state_dimension,
        )?;
        let control_jacobian = finite_difference_scvx_control_jacobian(
            &mut dynamics,
            times_s[interval],
            state_ref,
            &control_mid,
            settings.finite_difference_step,
            state_dimension,
            control_dimension,
        )?;

        let mut state_transition = vec![0.0; state_dimension * state_dimension];
        for row in 0..state_dimension {
            for col in 0..state_dimension {
                let identity = if row == col { 1.0 } else { 0.0 };
                state_transition[row * state_dimension + col] =
                    identity + dt * state_jacobian[row * state_dimension + col];
            }
        }

        let mut control_left_matrix = vec![0.0; state_dimension * control_dimension];
        let mut control_right_matrix = vec![0.0; state_dimension * control_dimension];
        for row in 0..state_dimension {
            for col in 0..control_dimension {
                let value = 0.5 * dt * control_jacobian[row * control_dimension + col];
                control_left_matrix[row * control_dimension + col] = value;
                control_right_matrix[row * control_dimension + col] = value;
            }
        }

        let mut affine = vec![0.0; state_dimension];
        for row in 0..state_dimension {
            let mut value = dt * base[row];
            for col in 0..state_dimension {
                value -= dt * state_jacobian[row * state_dimension + col] * state_ref[col];
            }
            for col in 0..control_dimension {
                value -= dt * control_jacobian[row * control_dimension + col] * control_mid[col];
            }
            affine[row] = value;
        }

        segments.push(ScvxAffineDynamicsSegment {
            state_transition,
            control_left: control_left_matrix,
            control_right: control_right_matrix,
            affine,
        });
    }

    Ok(segments)
}

/// Evaluate the flat 6-DoF rigid-body continuous dynamics used by the SCvx
/// substrate.
///
/// The output order matches [`ScvxRigidBodyStateLayout`]. The equations mirror
/// the simulator's rigid-body convention:
///
/// - `r_dot = v`
/// - `v_dot = external_acceleration + force_eci / mass`
/// - `q_dot = 0.5 * q_body_to_eci * [0, omega_body]`
/// - `omega_dot = I^-1 * (moment - omega x I*omega - I_dot*omega)`
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions, finite values, mass, quaternion
/// norm, or inertia invertibility are invalid.
pub fn evaluate_scvx_rigid_body_dynamics(
    state: &[f64],
    control: &[f64],
    parameters: &ScvxRigidBodyDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError> {
    validate_scvx_rigid_body_dynamics_inputs(state, control, parameters, settings, output)?;

    let position = ScvxRigidBodyStateLayout::POSITION_ECI;
    let velocity = ScvxRigidBodyStateLayout::VELOCITY_ECI;
    let quaternion = ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let angular_velocity = ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY;
    let force = ScvxRigidBodyControlLayout::FORCE_ECI_N;
    let moment = ScvxRigidBodyControlLayout::MOMENT_BODY_N_M;

    output[position..position + 3].copy_from_slice(&state[velocity..velocity + 3]);
    for component in 0..3 {
        output[velocity + component] = parameters.external_acceleration_eci_m_s2[component]
            + control[force + component] / parameters.mass_kg;
    }

    let q_dot = quaternion_rate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        [
            state[angular_velocity],
            state[angular_velocity + 1],
            state[angular_velocity + 2],
        ],
    );
    output[quaternion..quaternion + 4].copy_from_slice(&q_dot);

    let omega = [
        state[angular_velocity],
        state[angular_velocity + 1],
        state[angular_velocity + 2],
    ];
    let i_omega = mat3_vec_mul(parameters.inertia_body_kg_m2, omega);
    let omega_cross_i_omega = cross3(omega, i_omega);
    let i_dot_omega = mat3_vec_mul(parameters.inertia_rate_body_kg_m2_s, omega);
    let net = [
        control[moment] - omega_cross_i_omega[0] - i_dot_omega[0],
        control[moment + 1] - omega_cross_i_omega[1] - i_dot_omega[1],
        control[moment + 2] - omega_cross_i_omega[2] - i_dot_omega[2],
    ];
    let inv_inertia =
        invert_mat3(parameters.inertia_body_kg_m2).ok_or(TrajoptError::InvalidPayload {
            reason: "SCvx rigid-body inertia tensor is not invertible",
        })?;
    let omega_dot = mat3_vec_mul(inv_inertia, net);
    output[angular_velocity..angular_velocity + 3].copy_from_slice(&omega_dot);

    Ok(())
}

/// Evaluate mass-depleting powered-landing 6-DoF continuous dynamics.
///
/// The output order matches [`ScvxPoweredLandingStateLayout`]. The equations are
/// the Szmuk-style single-thrust-vector rigid-body model:
///
/// - `m_dot = -alpha * ||T_body||`
/// - `r_dot = v`
/// - `v_dot = gravity + rotate_body_to_eci(q) * T_body / m`
/// - `q_dot = 0.5 * q_body_to_eci * [0, omega_body]`
/// - `omega_dot = I^-1 * ((r_T_body x T_body) - omega x I*omega)`
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions, finite values, mass, quaternion
/// norm, mass-flow coefficient, or inertia invertibility are invalid.
pub fn evaluate_scvx_powered_landing_dynamics(
    state: &[f64],
    control: &[f64],
    parameters: &ScvxPoweredLandingDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError> {
    validate_scvx_powered_landing_dynamics_inputs(state, control, parameters, settings, output)?;

    let mass = state[ScvxPoweredLandingStateLayout::MASS];
    let position = ScvxPoweredLandingStateLayout::POSITION_ECI;
    let velocity = ScvxPoweredLandingStateLayout::VELOCITY_ECI;
    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let angular_velocity = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
    let thrust = ScvxPoweredLandingControlLayout::THRUST_BODY;
    let thrust_body = [control[thrust], control[thrust + 1], control[thrust + 2]];

    output[ScvxPoweredLandingStateLayout::MASS] =
        -parameters.mass_flow_coefficient_s_m * norm3(thrust_body);
    output[position..position + 3].copy_from_slice(&state[velocity..velocity + 3]);

    let force_eci = rotate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        thrust_body,
    );
    for component in 0..3 {
        output[velocity + component] =
            parameters.gravity_eci_m_s2[component] + force_eci[component] / mass;
    }

    let q_dot = quaternion_rate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        [
            state[angular_velocity],
            state[angular_velocity + 1],
            state[angular_velocity + 2],
        ],
    );
    output[quaternion..quaternion + 4].copy_from_slice(&q_dot);

    let omega = [
        state[angular_velocity],
        state[angular_velocity + 1],
        state[angular_velocity + 2],
    ];
    let i_omega = mat3_vec_mul(parameters.inertia_body_kg_m2, omega);
    let omega_cross_i_omega = cross3(omega, i_omega);
    let thrust_moment = cross3(parameters.thrust_application_point_body_m, thrust_body);
    let net = [
        thrust_moment[0] - omega_cross_i_omega[0],
        thrust_moment[1] - omega_cross_i_omega[1],
        thrust_moment[2] - omega_cross_i_omega[2],
    ];
    let inv_inertia =
        invert_mat3(parameters.inertia_body_kg_m2).ok_or(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing inertia tensor is not invertible",
        })?;
    let omega_dot = mat3_vec_mul(inv_inertia, net);
    output[angular_velocity..angular_velocity + 3].copy_from_slice(&omega_dot);

    Ok(())
}

/// Evaluate mass-depleting powered-landing dynamics with reduced aerodynamic
/// force/moment added.
///
/// Mass depletion remains driven by the commanded body-thrust norm only. The
/// reduced aerodynamic body force is rotated into ECI and added to thrust for
/// translational acceleration; the reduced aerodynamic body moment is added to
/// the thrust-arm moment for angular acceleration.
///
/// # Errors
///
/// Returns [`TrajoptError`] when aero composition or powered-landing dynamics
/// validation fails.
pub fn evaluate_scvx_reduced_aero_powered_landing_dynamics(
    state: &[f64],
    control: &[f64],
    coefficients: ScvxReducedAeroCoefficients,
    aero_parameters: ScvxReducedAeroParameters,
    powered_landing_parameters: &ScvxPoweredLandingDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError> {
    validate_scvx_powered_landing_dynamics_inputs(
        state,
        control,
        powered_landing_parameters,
        settings,
        output,
    )?;

    let aero_state = powered_landing_state_as_rigid_body_state(state);
    let aero = evaluate_scvx_reduced_aero_force_moment(
        &aero_state,
        coefficients,
        aero_parameters,
        settings,
    )?;
    evaluate_scvx_powered_landing_dynamics_with_aero_load(
        state,
        control,
        &aero,
        powered_landing_parameters,
        settings,
        output,
    )
}

fn evaluate_scvx_powered_landing_dynamics_with_aero_load(
    state: &[f64],
    control: &[f64],
    aero: &ScvxReducedAeroForceMoment,
    powered_landing_parameters: &ScvxPoweredLandingDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError> {
    validate_scvx_powered_landing_dynamics_inputs(
        state,
        control,
        powered_landing_parameters,
        settings,
        output,
    )?;
    let mass = state[ScvxPoweredLandingStateLayout::MASS];
    let position = ScvxPoweredLandingStateLayout::POSITION_ECI;
    let velocity = ScvxPoweredLandingStateLayout::VELOCITY_ECI;
    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let angular_velocity = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
    let thrust = ScvxPoweredLandingControlLayout::THRUST_BODY;
    let thrust_body = [control[thrust], control[thrust + 1], control[thrust + 2]];

    output[ScvxPoweredLandingStateLayout::MASS] =
        -powered_landing_parameters.mass_flow_coefficient_s_m * norm3(thrust_body);
    output[position..position + 3].copy_from_slice(&state[velocity..velocity + 3]);

    let thrust_force_eci = rotate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        thrust_body,
    );
    for component in 0..3 {
        output[velocity + component] = powered_landing_parameters.gravity_eci_m_s2[component]
            + (thrust_force_eci[component] + aero.force_n_eci[component]) / mass;
    }

    let q_dot = quaternion_rate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        [
            state[angular_velocity],
            state[angular_velocity + 1],
            state[angular_velocity + 2],
        ],
    );
    output[quaternion..quaternion + 4].copy_from_slice(&q_dot);

    let omega = [
        state[angular_velocity],
        state[angular_velocity + 1],
        state[angular_velocity + 2],
    ];
    let i_omega = mat3_vec_mul(powered_landing_parameters.inertia_body_kg_m2, omega);
    let omega_cross_i_omega = cross3(omega, i_omega);
    let thrust_moment = cross3(
        powered_landing_parameters.thrust_application_point_body_m,
        thrust_body,
    );
    let net = [
        thrust_moment[0] + aero.moment_n_m_body[0] - omega_cross_i_omega[0],
        thrust_moment[1] + aero.moment_n_m_body[1] - omega_cross_i_omega[1],
        thrust_moment[2] + aero.moment_n_m_body[2] - omega_cross_i_omega[2],
    ];
    let inv_inertia = invert_mat3(powered_landing_parameters.inertia_body_kg_m2).ok_or(
        TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing inertia tensor is not invertible",
        },
    )?;
    let omega_dot = mat3_vec_mul(inv_inertia, net);
    output[angular_velocity..angular_velocity + 3].copy_from_slice(&omega_dot);

    Ok(())
}

/// Evaluate powered-landing trajectory envelope constraints.
///
/// This diagnostic checks the Szmuk-style published-case envelope quantities
/// that are independent of a particular convex subproblem: mass bounds, thrust
/// norm bounds, thrust gimbal angle, vehicle tilt angle, angular-rate norm, and
/// glideslope cone margin.
///
/// # Errors
///
/// Returns [`TrajoptError`] when trajectory dimensions, limits, sample times,
/// finite values, or sample quaternions are malformed.
pub fn evaluate_scvx_powered_landing_envelope(
    times_s: &[f64],
    states: &[f64],
    controls: &[f64],
    limits: ScvxPoweredLandingEnvelopeLimits,
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<ScvxPoweredLandingEnvelopeReport, TrajoptError> {
    validate_scvx_powered_landing_envelope_inputs(times_s, states, controls, limits, settings)?;
    let sample_count = times_s.len();
    let local_up = unit3(scale3(limits.gravity_eci, -1.0)).ok_or(TrajoptError::InvalidPayload {
        reason: "SCvx powered-landing envelope gravity must be nonzero",
    })?;
    let glideslope_tangent = limits.glideslope_angle_rad.tan();

    let mut samples = Vec::with_capacity(sample_count);
    let mut max_mass_violation = 0.0_f64;
    let mut max_thrust_violation = 0.0_f64;
    let mut max_gimbal_angle_violation_rad = 0.0_f64;
    let mut max_tilt_angle_violation_rad = 0.0_f64;
    let mut max_angular_rate_violation_rad_s = 0.0_f64;
    let mut max_glideslope_violation = 0.0_f64;
    let mut max_violation = 0.0_f64;

    for (sample_index, &time_s) in times_s.iter().enumerate() {
        let state_start = sample_index * ScvxPoweredLandingStateLayout::DIMENSION;
        let control_start = sample_index * ScvxPoweredLandingControlLayout::DIMENSION;
        let state = &states[state_start..state_start + ScvxPoweredLandingStateLayout::DIMENSION];
        let control =
            &controls[control_start..control_start + ScvxPoweredLandingControlLayout::DIMENSION];
        validate_scvx_powered_landing_sample_quaternion(state, settings)?;

        let mass = state[ScvxPoweredLandingStateLayout::MASS];
        let thrust_body = [control[0], control[1], control[2]];
        let thrust_norm = norm3(thrust_body);
        let lateral_thrust_norm =
            (thrust_body[1] * thrust_body[1] + thrust_body[2] * thrust_body[2]).sqrt();
        let gimbal_angle_rad = lateral_thrust_norm.atan2(thrust_body[0]);
        let body_axis_eci = rotate_body_to_eci_wxyz(
            [
                state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ],
                state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 1],
                state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 2],
                state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 3],
            ],
            [1.0, 0.0, 0.0],
        );
        let tilt_angle_rad = dot3(body_axis_eci, local_up).clamp(-1.0, 1.0).acos();
        let omega = [
            state[ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY],
            state[ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY + 1],
            state[ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY + 2],
        ];
        let angular_rate_norm_rad_s = norm3(omega);
        let position = [
            state[ScvxPoweredLandingStateLayout::POSITION_ECI],
            state[ScvxPoweredLandingStateLayout::POSITION_ECI + 1],
            state[ScvxPoweredLandingStateLayout::POSITION_ECI + 2],
        ];
        let altitude = dot3(position, local_up);
        let lateral = [
            position[0] - altitude * local_up[0],
            position[1] - altitude * local_up[1],
            position[2] - altitude * local_up[2],
        ];
        let glideslope_margin = altitude - glideslope_tangent * norm3(lateral);

        let mass_violation = positive_part(limits.minimum_mass - mass)
            .max(positive_part(mass - limits.maximum_mass));
        let thrust_violation = positive_part(limits.minimum_thrust - thrust_norm)
            .max(positive_part(thrust_norm - limits.maximum_thrust));
        let gimbal_violation = positive_part(gimbal_angle_rad - limits.maximum_gimbal_angle_rad);
        let tilt_violation = positive_part(tilt_angle_rad - limits.maximum_tilt_angle_rad);
        let angular_rate_violation =
            positive_part(angular_rate_norm_rad_s - limits.maximum_angular_rate_rad_s);
        let glideslope_violation = positive_part(-glideslope_margin);
        let sample_max_violation = mass_violation
            .max(thrust_violation)
            .max(gimbal_violation)
            .max(tilt_violation)
            .max(angular_rate_violation)
            .max(glideslope_violation);

        max_mass_violation = max_mass_violation.max(mass_violation);
        max_thrust_violation = max_thrust_violation.max(thrust_violation);
        max_gimbal_angle_violation_rad = max_gimbal_angle_violation_rad.max(gimbal_violation);
        max_tilt_angle_violation_rad = max_tilt_angle_violation_rad.max(tilt_violation);
        max_angular_rate_violation_rad_s =
            max_angular_rate_violation_rad_s.max(angular_rate_violation);
        max_glideslope_violation = max_glideslope_violation.max(glideslope_violation);
        max_violation = max_violation.max(sample_max_violation);

        samples.push(ScvxPoweredLandingEnvelopeSample {
            time_s,
            mass,
            thrust_norm,
            gimbal_angle_rad,
            tilt_angle_rad,
            angular_rate_norm_rad_s,
            glideslope_margin,
            max_violation: sample_max_violation,
        });
    }

    Ok(ScvxPoweredLandingEnvelopeReport {
        sample_count,
        max_mass_violation,
        max_thrust_violation,
        max_gimbal_angle_violation_rad,
        max_tilt_angle_violation_rad,
        max_angular_rate_violation_rad_s,
        max_glideslope_violation,
        max_violation,
        within_limits: max_violation == 0.0,
        samples,
    })
}

/// Build linear powered-landing envelope rows for every trajectory node.
///
/// The generated rows cover the envelope parts that are already linear or can be
/// represented as sampled halfspaces without a nonlinear subproblem: mass
/// bounds, glideslope halfspaces, axial thrust bounds, and body-thrust gimbal
/// halfspaces. Full thrust-norm upper bounds, vehicle-tilt bounds, and angular
/// rate norm bounds remain diagnostic-only in
/// [`evaluate_scvx_powered_landing_envelope`] until the SCvx substrate grows
/// conic or reference-linearized nonlinear path rows.
///
/// `cone_facets` controls the number of uniformly spaced lateral directions used
/// for both glideslope and gimbal halfspace approximations.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the node count, envelope limits, facet count, or
/// gimbal angle is not compatible with linear halfspace rows.
pub fn build_scvx_powered_landing_linear_envelope_constraints(
    node_count: usize,
    limits: ScvxPoweredLandingEnvelopeLimits,
    cone_facets: usize,
) -> Result<ScvxPoweredLandingLinearEnvelopeConstraints, TrajoptError> {
    validate_scvx_powered_landing_linear_envelope_inputs(node_count, limits, cone_facets)?;

    let local_up = unit3(scale3(limits.gravity_eci, -1.0)).ok_or(TrajoptError::InvalidPayload {
        reason: "SCvx powered-landing linear envelope gravity must be nonzero",
    })?;
    let (lateral_axis_a, lateral_axis_b) = powered_landing_lateral_basis(local_up)?;
    let glideslope_tangent = limits.glideslope_angle_rad.tan();
    let gimbal_tangent = limits.maximum_gimbal_angle_rad.tan();
    let mut linear_state_constraints = Vec::with_capacity(node_count * (2 + cone_facets));
    let mut linear_control_constraints = Vec::with_capacity(node_count * (2 + cone_facets));

    for node in 0..node_count {
        linear_state_constraints.push(powered_landing_mass_constraint(
            node,
            -1.0,
            limits.minimum_mass,
        ));
        linear_state_constraints.push(powered_landing_mass_constraint(
            node,
            1.0,
            -limits.maximum_mass,
        ));
        linear_control_constraints.push(powered_landing_axial_thrust_constraint(
            node,
            -1.0,
            -limits.minimum_thrust,
        ));
        linear_control_constraints.push(powered_landing_axial_thrust_constraint(
            node,
            1.0,
            limits.maximum_thrust,
        ));

        for facet in 0..cone_facets {
            let angle = 2.0 * core::f64::consts::PI * facet as f64 / cone_facets as f64;
            let (sin_angle, cos_angle) = angle.sin_cos();
            let lateral_direction = add3(
                scale3(lateral_axis_a, cos_angle),
                scale3(lateral_axis_b, sin_angle),
            );
            linear_state_constraints.push(powered_landing_glideslope_constraint(
                node,
                local_up,
                lateral_direction,
                glideslope_tangent,
            ));
            linear_control_constraints.push(powered_landing_gimbal_constraint(
                node,
                cos_angle,
                sin_angle,
                gimbal_tangent,
            ));
        }
    }

    Ok(ScvxPoweredLandingLinearEnvelopeConstraints {
        linear_state_constraints,
        linear_control_constraints,
    })
}

/// Build base linear plus reference-linearized powered-landing envelope rows.
///
/// This extends [`build_scvx_powered_landing_linear_envelope_constraints`] with
/// finite-facet upper-thrust-norm rows, finite-facet angular-rate-norm rows, and
/// one reference-linearized body-tilt row per trajectory node. These rows are a
/// deterministic SCvx substrate for the nonlinear envelope quantities; finite
/// facet counts remain approximations and do not replace an exact conic solve.
///
/// # Errors
///
/// Returns [`TrajoptError`] when reference-state dimensions, limits, settings, or
/// sample quaternions are malformed.
pub fn build_scvx_powered_landing_linearized_envelope_constraints(
    reference_states: &[f64],
    limits: ScvxPoweredLandingEnvelopeLimits,
    settings: ScvxPoweredLandingLinearizedEnvelopeSettings,
) -> Result<ScvxPoweredLandingLinearEnvelopeConstraints, TrajoptError> {
    validate_scvx_powered_landing_linearized_envelope_inputs(reference_states, limits, settings)?;
    let node_count = reference_states.len() / ScvxPoweredLandingStateLayout::DIMENSION;
    let mut constraints = build_scvx_powered_landing_linear_envelope_constraints(
        node_count,
        limits,
        settings.cone_facets,
    )?;
    let local_up = unit3(scale3(limits.gravity_eci, -1.0)).ok_or(TrajoptError::InvalidPayload {
        reason: "SCvx powered-landing linearized envelope gravity must be nonzero",
    })?;

    for node in 0..node_count {
        let state_start = node * ScvxPoweredLandingStateLayout::DIMENSION;
        let reference_state =
            &reference_states[state_start..state_start + ScvxPoweredLandingStateLayout::DIMENSION];
        constraints
            .linear_state_constraints
            .push(powered_landing_tilt_linearized_constraint(
                node,
                reference_state,
                local_up,
                limits.maximum_tilt_angle_rad,
                settings.finite_difference_step,
            )?);

        for facet in 0..settings.norm_facets {
            let direction = powered_landing_norm_facet_direction(facet, settings.norm_facets)?;
            constraints.linear_state_constraints.push(
                powered_landing_angular_rate_norm_facet_constraint(
                    node,
                    direction,
                    limits.maximum_angular_rate_rad_s,
                ),
            );
            constraints.linear_control_constraints.push(
                powered_landing_thrust_norm_facet_constraint(
                    node,
                    direction,
                    limits.maximum_thrust,
                ),
            );
        }
    }

    Ok(constraints)
}

/// Build powered-landing envelope rows with exact SOC constraints where convex.
///
/// This builder keeps mass bounds, minimum axial thrust, and the nonconvex body
/// tilt envelope on linear rows, with tilt linearized around each supplied
/// reference state. It emits exact second-order cones for the convex envelope
/// pieces: glideslope, upper thrust norm, body-thrust gimbal cone, and angular
/// rate norm.
///
/// # Errors
///
/// Returns [`TrajoptError`] when reference-state dimensions, limits, gimbal cone
/// angle, settings, or sample quaternions are malformed.
pub fn build_scvx_powered_landing_conic_envelope_constraints(
    reference_states: &[f64],
    limits: ScvxPoweredLandingEnvelopeLimits,
    settings: ScvxPoweredLandingConicEnvelopeSettings,
) -> Result<ScvxPoweredLandingConicEnvelopeConstraints, TrajoptError> {
    validate_scvx_powered_landing_conic_envelope_inputs(reference_states, limits, settings)?;
    let node_count = reference_states.len() / ScvxPoweredLandingStateLayout::DIMENSION;
    let local_up = unit3(scale3(limits.gravity_eci, -1.0)).ok_or(TrajoptError::InvalidPayload {
        reason: "SCvx powered-landing conic envelope gravity must be nonzero",
    })?;
    let (lateral_axis_a, lateral_axis_b) = powered_landing_lateral_basis(local_up)?;
    let glideslope_tangent = limits.glideslope_angle_rad.tan();
    let gimbal_tangent = limits.maximum_gimbal_angle_rad.tan();

    let mut linear_state_constraints = Vec::with_capacity(node_count * 3);
    let mut linear_control_constraints = Vec::with_capacity(node_count);
    let mut second_order_state_constraints = Vec::with_capacity(node_count * 2);
    let mut second_order_control_constraints = Vec::with_capacity(node_count * 2);

    for node in 0..node_count {
        let state_start = node * ScvxPoweredLandingStateLayout::DIMENSION;
        let reference_state =
            &reference_states[state_start..state_start + ScvxPoweredLandingStateLayout::DIMENSION];
        linear_state_constraints.push(powered_landing_mass_constraint(
            node,
            -1.0,
            limits.minimum_mass,
        ));
        linear_state_constraints.push(powered_landing_mass_constraint(
            node,
            1.0,
            -limits.maximum_mass,
        ));
        linear_state_constraints.push(powered_landing_tilt_linearized_constraint(
            node,
            reference_state,
            local_up,
            limits.maximum_tilt_angle_rad,
            settings.finite_difference_step,
        )?);
        linear_control_constraints.push(powered_landing_axial_thrust_constraint(
            node,
            -1.0,
            -limits.minimum_thrust,
        ));
        second_order_state_constraints.push(powered_landing_glideslope_soc_constraint(
            node,
            local_up,
            lateral_axis_a,
            lateral_axis_b,
            glideslope_tangent,
        ));
        second_order_state_constraints.push(powered_landing_angular_rate_soc_constraint(
            node,
            limits.maximum_angular_rate_rad_s,
        ));
        second_order_control_constraints.push(powered_landing_thrust_norm_soc_constraint(
            node,
            limits.maximum_thrust,
        ));
        second_order_control_constraints
            .push(powered_landing_gimbal_soc_constraint(node, gimbal_tangent));
    }

    Ok(ScvxPoweredLandingConicEnvelopeConstraints {
        linear_state_constraints,
        linear_control_constraints,
        second_order_state_constraints,
        second_order_control_constraints,
    })
}

/// Normalize row-major powered-landing reference-state quaternions in place.
///
/// The state order must match [`ScvxPoweredLandingStateLayout`]. The function
/// returns the largest absolute quaternion-norm error observed before
/// projection so callers can keep relinearization diagnostics explicit.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the state buffer is empty, not a whole number
/// of powered-landing states, contains non-finite values, or contains a
/// zero-norm/non-finite quaternion.
pub fn normalize_scvx_powered_landing_quaternion_reference_states(
    states: &mut [f64],
) -> Result<f64, TrajoptError> {
    if states.is_empty()
        || !states
            .len()
            .is_multiple_of(ScvxPoweredLandingStateLayout::DIMENSION)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing reference states must contain whole state rows",
        });
    }
    validate_finite(
        "SCvx powered-landing reference states must be finite",
        states,
    )?;

    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let mut max_norm_error = 0.0_f64;
    for state in states.chunks_exact_mut(ScvxPoweredLandingStateLayout::DIMENSION) {
        let norm = (state[quaternion] * state[quaternion]
            + state[quaternion + 1] * state[quaternion + 1]
            + state[quaternion + 2] * state[quaternion + 2]
            + state[quaternion + 3] * state[quaternion + 3])
            .sqrt();
        if !norm.is_finite() || norm <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx powered-landing reference quaternion norm must be positive",
            });
        }
        max_norm_error = max_norm_error.max((norm - 1.0).abs());
        state[quaternion] /= norm;
        state[quaternion + 1] /= norm;
        state[quaternion + 2] /= norm;
        state[quaternion + 3] /= norm;
    }
    Ok(max_norm_error)
}

/// Compose reduced `(CN, CD, CM)` aero coefficients into SCvx force/moment.
///
/// This mirrors the `openbmp-aero` schema-1 force/moment convention while
/// staying dependency-light inside `openbmp-trajopt`. The body-frame force is
/// `q*S*(-CD, 0, -CN)`, the body-frame moment is `q*S*L*(0, CM, 0)`, and the
/// force is rotated to ECI using the state quaternion.
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions, finite values, quaternion norm, or
/// aerodynamic reference quantities are invalid.
pub fn evaluate_scvx_reduced_aero_force_moment(
    state: &[f64],
    coefficients: ScvxReducedAeroCoefficients,
    parameters: ScvxReducedAeroParameters,
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<ScvxReducedAeroForceMoment, TrajoptError> {
    validate_scvx_reduced_aero_inputs(state, coefficients, parameters, settings)?;

    let q_s = parameters.dynamic_pressure_pa * parameters.reference_area_m2;
    let q_s_l = q_s * parameters.reference_length_m;
    if !q_s.is_finite() || !q_s_l.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx reduced aero scale is non-finite",
        });
    }
    let force_n_body = [-coefficients.cd * q_s, 0.0, -coefficients.cn * q_s];
    let moment_n_m_body = [0.0, coefficients.cm * q_s_l, 0.0];
    validate_array_finite("SCvx reduced aero body force must be finite", &force_n_body)?;
    validate_array_finite(
        "SCvx reduced aero body moment must be finite",
        &moment_n_m_body,
    )?;

    let quaternion = ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let force_n_eci = rotate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        force_n_body,
    );
    validate_array_finite("SCvx reduced aero ECI force must be finite", &force_n_eci)?;
    let rigid_body_control = [
        force_n_eci[0],
        force_n_eci[1],
        force_n_eci[2],
        moment_n_m_body[0],
        moment_n_m_body[1],
        moment_n_m_body[2],
    ];

    Ok(ScvxReducedAeroForceMoment {
        force_n_body,
        force_n_eci,
        moment_n_m_body,
        rigid_body_control,
    })
}

/// Evaluate rigid-body dynamics with reduced aerodynamic force/moment added to
/// the supplied baseline control vector.
///
/// The baseline `control` keeps the same layout as [`ScvxRigidBodyControlLayout`]
/// and may represent thrust or other non-aero generalized loads. The reduced
/// aero load is composed, rotated to ECI, added componentwise, and then passed
/// through [`evaluate_scvx_rigid_body_dynamics`].
///
/// # Errors
///
/// Returns [`TrajoptError`] when aero composition or rigid-body dynamics
/// validation fails.
pub fn evaluate_scvx_aero_coupled_rigid_body_dynamics(
    state: &[f64],
    control: &[f64],
    coefficients: ScvxReducedAeroCoefficients,
    aero_parameters: ScvxReducedAeroParameters,
    rigid_body_parameters: &ScvxRigidBodyDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError> {
    validate_len(
        "SCvx aero-coupled baseline control dimensions are inconsistent",
        control,
        ScvxRigidBodyControlLayout::DIMENSION,
    )?;
    validate_finite("SCvx aero-coupled baseline control must be finite", control)?;
    let aero =
        evaluate_scvx_reduced_aero_force_moment(state, coefficients, aero_parameters, settings)?;
    let mut coupled_control = [0.0; ScvxRigidBodyControlLayout::DIMENSION];
    for component in 0..ScvxRigidBodyControlLayout::DIMENSION {
        coupled_control[component] = control[component] + aero.rigid_body_control[component];
    }
    evaluate_scvx_rigid_body_dynamics(
        state,
        &coupled_control,
        rigid_body_parameters,
        settings,
        output,
    )
}

/// Evaluate an `openbmp-aero` method and convert its body-frame result into an
/// SCvx rigid-body control vector.
///
/// This is feature-gated behind `openbmp-trajopt/aero` so the default trajopt
/// build keeps no hard dependency on deck/hypersonic aero providers.
///
/// # Errors
///
/// Returns [`TrajoptError`] if the SCvx state/quaternion is malformed or the
/// aero method rejects the supplied context.
#[cfg(feature = "aero")]
pub fn evaluate_scvx_aero_method_force_moment<M>(
    state: &[f64],
    aero_method: &M,
    context: &openbmp_aero::AeroContext,
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<ScvxReducedAeroForceMoment, TrajoptError>
where
    M: openbmp_aero::AeroMethod + ?Sized,
{
    validate_scvx_aero_method_state(state, settings)?;
    let aero =
        aero_method
            .aero_force_moment_body(context)
            .map_err(|_| TrajoptError::InvalidPayload {
                reason: "SCvx aero method evaluation failed",
            })?;
    let force_n_body = [
        aero.force_n_body.x,
        aero.force_n_body.y,
        aero.force_n_body.z,
    ];
    let moment_n_m_body = [
        aero.moment_n_m_body.x,
        aero.moment_n_m_body.y,
        aero.moment_n_m_body.z,
    ];
    validate_array_finite("SCvx aero method body force must be finite", &force_n_body)?;
    validate_array_finite(
        "SCvx aero method body moment must be finite",
        &moment_n_m_body,
    )?;

    let quaternion = ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let force_n_eci = rotate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        force_n_body,
    );
    validate_array_finite("SCvx aero method ECI force must be finite", &force_n_eci)?;
    let rigid_body_control = [
        force_n_eci[0],
        force_n_eci[1],
        force_n_eci[2],
        moment_n_m_body[0],
        moment_n_m_body[1],
        moment_n_m_body[2],
    ];

    Ok(ScvxReducedAeroForceMoment {
        force_n_body,
        force_n_eci,
        moment_n_m_body,
        rigid_body_control,
    })
}

/// Evaluate rigid-body dynamics with a feature-gated `openbmp-aero` method
/// added to a baseline rigid-body control vector.
///
/// # Errors
///
/// Returns [`TrajoptError`] if baseline controls, aero-method evaluation, or
/// rigid-body dynamics validation fails.
#[cfg(feature = "aero")]
pub fn evaluate_scvx_aero_method_coupled_rigid_body_dynamics<M>(
    state: &[f64],
    control: &[f64],
    aero_method: &M,
    context: &openbmp_aero::AeroContext,
    rigid_body_parameters: &ScvxRigidBodyDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError>
where
    M: openbmp_aero::AeroMethod + ?Sized,
{
    validate_len(
        "SCvx aero-method baseline control dimensions are inconsistent",
        control,
        ScvxRigidBodyControlLayout::DIMENSION,
    )?;
    validate_finite("SCvx aero-method baseline control must be finite", control)?;
    let aero = evaluate_scvx_aero_method_force_moment(state, aero_method, context, settings)?;
    let mut coupled_control = [0.0; ScvxRigidBodyControlLayout::DIMENSION];
    for component in 0..ScvxRigidBodyControlLayout::DIMENSION {
        coupled_control[component] = control[component] + aero.rigid_body_control[component];
    }
    evaluate_scvx_rigid_body_dynamics(
        state,
        &coupled_control,
        rigid_body_parameters,
        settings,
        output,
    )
}

/// Evaluate mass-depleting powered-landing dynamics with a feature-gated
/// `openbmp-aero` method force/moment added.
///
/// Mass depletion remains driven by the commanded body-thrust norm only. The
/// aero method's body force is rotated into ECI and added to thrust for
/// translational acceleration; the body moment is added to the thrust-arm
/// moment for angular acceleration.
///
/// # Errors
///
/// Returns [`TrajoptError`] if powered-landing inputs, aero-method evaluation,
/// or load-coupled dynamics validation fails.
#[cfg(feature = "aero")]
pub fn evaluate_scvx_aero_method_powered_landing_dynamics<M>(
    state: &[f64],
    control: &[f64],
    aero_method: &M,
    context: &openbmp_aero::AeroContext,
    powered_landing_parameters: &ScvxPoweredLandingDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &mut [f64],
) -> Result<(), TrajoptError>
where
    M: openbmp_aero::AeroMethod + ?Sized,
{
    validate_scvx_powered_landing_dynamics_inputs(
        state,
        control,
        powered_landing_parameters,
        settings,
        output,
    )?;
    let aero_state = powered_landing_state_as_rigid_body_state(state);
    let aero = evaluate_scvx_aero_method_force_moment(&aero_state, aero_method, context, settings)?;
    evaluate_scvx_powered_landing_dynamics_with_aero_load(
        state,
        control,
        &aero,
        powered_landing_parameters,
        settings,
        output,
    )
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ScvxDecisionLayout {
    node_count: usize,
    state_offset: usize,
    control_offset: usize,
    virtual_offset: usize,
    variable_count: usize,
}

#[derive(Clone, Debug, PartialEq)]
struct ScvxTrajectory {
    states: Vec<f64>,
    controls: Vec<f64>,
    virtual_controls: Vec<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct ScvxTimeCandidate {
    time_of_flight_s: f64,
    objective: f64,
    solution: ScvxSolution,
}

#[derive(Clone, Debug, PartialEq)]
struct ScvxContinuousFohTimeCandidate {
    time_of_flight_s: f64,
    objective: f64,
    solution: ScvxContinuousFohSolution,
}

impl ScvxTimeCandidate {
    fn report(&self) -> ScvxTimeOfFlightCandidateReport {
        ScvxTimeOfFlightCandidateReport {
            time_of_flight_s: self.time_of_flight_s,
            objective: self.objective,
            merit: self.solution.merit,
            terminal_error_norm: self.solution.terminal_error_norm,
            max_virtual_control_norm: self.solution.max_virtual_control_norm,
            max_linear_state_violation: self.solution.max_linear_state_violation,
            max_linear_control_violation: self.solution.max_linear_control_violation,
            max_second_order_cone_violation: self.solution.max_second_order_cone_violation,
            iteration_count: self.solution.iterations.len(),
            termination_status: self.solution.termination_status,
        }
    }
}

impl ScvxContinuousFohTimeCandidate {
    fn report(&self) -> ScvxContinuousFohTimeOfFlightCandidateReport {
        let final_outer = self.solution.outer_iterations.last();
        ScvxContinuousFohTimeOfFlightCandidateReport {
            time_of_flight_s: self.time_of_flight_s,
            objective: self.objective,
            merit: self.solution.solution.merit,
            converged: self.solution.converged,
            affine_converged: self.solution.solution.converged,
            terminal_error_norm: self.solution.solution.terminal_error_norm,
            max_virtual_control_norm: self.solution.solution.max_virtual_control_norm,
            max_linear_state_violation: self.solution.solution.max_linear_state_violation,
            max_linear_control_violation: self.solution.solution.max_linear_control_violation,
            max_second_order_cone_violation: self.solution.solution.max_second_order_cone_violation,
            outer_iteration_count: self.solution.outer_iterations.len(),
            affine_iteration_count: self.solution.solution.iterations.len(),
            max_state_update_norm: final_outer
                .map(|iteration| iteration.max_state_update_norm)
                .unwrap_or(0.0),
            max_control_update_norm: final_outer
                .map(|iteration| iteration.max_control_update_norm)
                .unwrap_or(0.0),
            termination_status: self.solution.solution.termination_status,
        }
    }
}

impl ScvxDecisionLayout {
    fn new(problem: &ScvxDiscreteAffineProblem) -> Self {
        let node_count = problem.horizon_steps + 1;
        let state_count = node_count * problem.state_dimension;
        let control_count = node_count * problem.control_dimension;
        let virtual_count = problem.horizon_steps * problem.state_dimension;
        Self {
            node_count,
            state_offset: 0,
            control_offset: state_count,
            virtual_offset: state_count + control_count,
            variable_count: state_count + control_count + virtual_count,
        }
    }

    fn state(self, problem: &ScvxDiscreteAffineProblem, node: usize, component: usize) -> usize {
        self.state_offset + node * problem.state_dimension + component
    }

    fn control(self, problem: &ScvxDiscreteAffineProblem, node: usize, component: usize) -> usize {
        self.control_offset + node * problem.control_dimension + component
    }

    fn virtual_control(
        self,
        problem: &ScvxDiscreteAffineProblem,
        interval: usize,
        component: usize,
    ) -> usize {
        self.virtual_offset + interval * problem.state_dimension + component
    }
}

fn solve_scvx_with_time_of_flight<F>(
    build_problem: &mut F,
    settings: ScvxTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
) -> Result<ScvxTimeCandidate, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxDiscreteAffineProblem, TrajoptError>,
{
    let candidate = evaluate_scvx_time_candidate(build_problem, settings, time_of_flight_s)?;
    if !candidate.solution.converged {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight candidate did not converge",
        });
    }
    Ok(candidate)
}

fn evaluate_scvx_time_candidate<F>(
    build_problem: &mut F,
    settings: ScvxTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
) -> Result<ScvxTimeCandidate, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxDiscreteAffineProblem, TrajoptError>,
{
    let problem = build_problem(time_of_flight_s)?;
    let solution = solve_scvx_discrete_affine(&problem, settings.scvx_settings)?;
    if !solution.converged {
        let objective = solution.merit + settings.time_penalty_weight * time_of_flight_s;
        if !objective.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx time-of-flight candidate objective is non-finite",
            });
        }
        return Ok(ScvxTimeCandidate {
            time_of_flight_s,
            objective,
            solution,
        });
    }
    let objective = solution.merit + settings.time_penalty_weight * time_of_flight_s;
    if !objective.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight candidate objective is non-finite",
        });
    }
    Ok(ScvxTimeCandidate {
        time_of_flight_s,
        objective,
        solution,
    })
}

fn solve_scvx_continuous_foh_with_time_of_flight<F>(
    solve_candidate: &mut F,
    settings: ScvxContinuousFohTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
) -> Result<ScvxContinuousFohTimeCandidate, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxContinuousFohSolution, TrajoptError>,
{
    let candidate =
        evaluate_scvx_continuous_foh_time_candidate(solve_candidate, settings, time_of_flight_s)?;
    if !candidate.solution.converged {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight candidate did not converge",
        });
    }
    Ok(candidate)
}

fn evaluate_scvx_continuous_foh_time_candidate<F>(
    solve_candidate: &mut F,
    settings: ScvxContinuousFohTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
) -> Result<ScvxContinuousFohTimeCandidate, TrajoptError>
where
    F: FnMut(f64) -> Result<ScvxContinuousFohSolution, TrajoptError>,
{
    let solution = solve_candidate(time_of_flight_s)?;
    let objective = solution.solution.merit + settings.time_penalty_weight * time_of_flight_s;
    if !objective.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight candidate objective is non-finite",
        });
    }
    Ok(ScvxContinuousFohTimeCandidate {
        time_of_flight_s,
        objective,
        solution,
    })
}

fn build_scvx_qp(
    problem: &ScvxDiscreteAffineProblem,
    settings: &ScvxSettings,
    layout: &ScvxDecisionLayout,
    reference_states: &[f64],
    reference_controls: &[f64],
    trust_region: f64,
    active_constraints: &[usize],
) -> Result<DenseQpProblem, TrajoptError> {
    let mut hessian = vec![0.0; layout.variable_count * layout.variable_count];
    let mut linear = vec![0.0; layout.variable_count];

    for component in 0..problem.state_dimension {
        let col = layout.state(problem, problem.horizon_steps, component);
        let terminal_weight =
            settings.terminal_weight * terminal_component_weight(problem, component);
        hessian[col * layout.variable_count + col] += terminal_weight;
        linear[col] -= terminal_weight * problem.target.state[component];
    }
    for node in 0..layout.node_count {
        for component in 0..problem.control_dimension {
            let col = layout.control(problem, node, component);
            hessian[col * layout.variable_count + col] += settings.control_weight;
        }
    }
    for interval in 0..problem.horizon_steps {
        for component in 0..problem.state_dimension {
            let col = layout.virtual_control(problem, interval, component);
            hessian[col * layout.variable_count + col] += settings.virtual_control_weight;
        }
    }
    for diag in 0..layout.variable_count {
        hessian[diag * layout.variable_count + diag] += settings.regularization;
    }

    let mut equality_matrix = Vec::new();
    let mut equality_rhs = Vec::new();
    for component in 0..problem.state_dimension {
        push_dense_row(
            &mut equality_matrix,
            layout.variable_count,
            &[(layout.state(problem, 0, component), 1.0)],
        );
        equality_rhs.push(problem.initial_state[component]);
    }
    for interval in 0..problem.horizon_steps {
        let segment = &problem.dynamics[interval];
        for row in 0..problem.state_dimension {
            let mut entries = vec![(layout.state(problem, interval + 1, row), 1.0)];
            for col in 0..problem.state_dimension {
                let value = -segment.state_transition[row * problem.state_dimension + col];
                entries.push((layout.state(problem, interval, col), value));
            }
            for col in 0..problem.control_dimension {
                let left = -segment.control_left[row * problem.control_dimension + col];
                let right = -segment.control_right[row * problem.control_dimension + col];
                entries.push((layout.control(problem, interval, col), left));
                entries.push((layout.control(problem, interval + 1, col), right));
            }
            entries.push((layout.virtual_control(problem, interval, row), -1.0));
            push_dense_row(&mut equality_matrix, layout.variable_count, &entries);
            equality_rhs.push(segment.affine[row]);
        }
    }

    let mut inequality_matrix = Vec::new();
    let mut inequality_rhs = Vec::new();
    for node in 0..layout.node_count {
        for component in 0..problem.state_dimension {
            let index = node * problem.state_dimension + component;
            let variable = layout.state(problem, node, component);
            push_inequality(
                &mut inequality_matrix,
                &mut inequality_rhs,
                layout.variable_count,
                &[(variable, 1.0)],
                reference_states[index] + trust_region,
            );
            push_inequality(
                &mut inequality_matrix,
                &mut inequality_rhs,
                layout.variable_count,
                &[(variable, -1.0)],
                -reference_states[index] + trust_region,
            );
        }
        for component in 0..problem.control_dimension {
            let index = node * problem.control_dimension + component;
            let variable = layout.control(problem, node, component);
            push_inequality(
                &mut inequality_matrix,
                &mut inequality_rhs,
                layout.variable_count,
                &[(variable, 1.0)],
                reference_controls[index] + trust_region,
            );
            push_inequality(
                &mut inequality_matrix,
                &mut inequality_rhs,
                layout.variable_count,
                &[(variable, -1.0)],
                -reference_controls[index] + trust_region,
            );
            push_inequality(
                &mut inequality_matrix,
                &mut inequality_rhs,
                layout.variable_count,
                &[(variable, 1.0)],
                problem.control_upper[component],
            );
            push_inequality(
                &mut inequality_matrix,
                &mut inequality_rhs,
                layout.variable_count,
                &[(variable, -1.0)],
                -problem.control_lower[component],
            );
        }
    }
    for &constraint_index in active_constraints {
        let constraint = &problem.state_triggered_constraints[constraint_index];
        let mut entries = Vec::with_capacity(problem.state_dimension);
        for component in 0..problem.state_dimension {
            entries.push((
                layout.state(problem, constraint.node, component),
                constraint.constraint.coefficients[component],
            ));
        }
        push_inequality(
            &mut inequality_matrix,
            &mut inequality_rhs,
            layout.variable_count,
            &entries,
            -constraint.constraint.offset,
        );
    }
    for constraint in &problem.linear_state_constraints {
        let mut entries = Vec::with_capacity(problem.state_dimension);
        for component in 0..problem.state_dimension {
            entries.push((
                layout.state(problem, constraint.node, component),
                constraint.function.coefficients[component],
            ));
        }
        push_inequality(
            &mut inequality_matrix,
            &mut inequality_rhs,
            layout.variable_count,
            &entries,
            -constraint.function.offset,
        );
    }
    for constraint in &problem.linear_control_constraints {
        let mut entries = Vec::with_capacity(problem.control_dimension);
        for component in 0..problem.control_dimension {
            entries.push((
                layout.control(problem, constraint.node, component),
                constraint.coefficients[component],
            ));
        }
        push_inequality(
            &mut inequality_matrix,
            &mut inequality_rhs,
            layout.variable_count,
            &entries,
            constraint.upper_bound,
        );
    }

    DenseQpProblem::new(
        layout.variable_count,
        hessian,
        linear,
        equality_matrix,
        equality_rhs,
        inequality_matrix,
        inequality_rhs,
    )
}

fn solve_scvx_convex_subproblem(
    qp: &DenseQpProblem,
    problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
) -> Result<ClarabelQpSolution, TrajoptError> {
    if problem.second_order_state_constraints.is_empty()
        && problem.second_order_control_constraints.is_empty()
    {
        return solve_dense_qp(qp);
    }

    let (soc_matrix, soc_rhs, soc_dims) = scvx_second_order_cone_rows(problem, layout);
    let p = scvx_csc_from_dense_upper_triangle(&qp.hessian, qp.variable_count);
    let q = qp.linear.clone();
    let equality_rows = qp.equality_rhs.len();
    let inequality_rows = qp.inequality_rhs.len();
    let soc_rows = soc_rhs.len();
    let mut constraint_matrix =
        Vec::with_capacity((equality_rows + inequality_rows + soc_rows) * qp.variable_count);
    constraint_matrix.extend_from_slice(&qp.equality_matrix);
    constraint_matrix.extend_from_slice(&qp.inequality_matrix);
    constraint_matrix.extend_from_slice(&soc_matrix);
    let a = scvx_csc_from_dense_rows(
        &constraint_matrix,
        equality_rows + inequality_rows + soc_rows,
        qp.variable_count,
    );
    let mut b = qp.equality_rhs.clone();
    b.extend_from_slice(&qp.inequality_rhs);
    b.extend_from_slice(&soc_rhs);

    let mut cones: Vec<SupportedConeT<f64>> = Vec::new();
    if equality_rows > 0 {
        cones.push(ZeroConeT(equality_rows));
    }
    if inequality_rows > 0 {
        cones.push(NonnegativeConeT(inequality_rows));
    }
    for dim in soc_dims {
        cones.push(SecondOrderConeT(dim));
    }

    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, deterministic_sqp_settings());
    solver.solve();
    if solver.solution.status != SolverStatus::Solved {
        return Err(TrajoptError::InvalidPayload {
            reason: "Clarabel did not solve SCvx conic subproblem",
        });
    }

    Ok(ClarabelQpSolution {
        primal: solver.solution.x,
        equality_dual: solver.solution.z[..equality_rows].to_vec(),
        inequality_dual: solver.solution.z[equality_rows..equality_rows + inequality_rows].to_vec(),
        objective: solver.solution.obj_val,
        status: solver.solution.status,
    })
}

fn scvx_second_order_cone_rows(
    problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
) -> (Vec<f64>, Vec<f64>, Vec<usize>) {
    let mut matrix = Vec::new();
    let mut rhs = Vec::new();
    let mut dims = Vec::new();
    for constraint in &problem.second_order_state_constraints {
        push_soc_state_function_row(
            problem,
            layout,
            &mut matrix,
            &mut rhs,
            constraint.node,
            &constraint.scalar,
        );
        for function in &constraint.vector {
            push_soc_state_function_row(
                problem,
                layout,
                &mut matrix,
                &mut rhs,
                constraint.node,
                function,
            );
        }
        dims.push(1 + constraint.vector.len());
    }
    for constraint in &problem.second_order_control_constraints {
        push_soc_control_function_row(
            problem,
            layout,
            &mut matrix,
            &mut rhs,
            constraint.node,
            &constraint.scalar,
        );
        for function in &constraint.vector {
            push_soc_control_function_row(
                problem,
                layout,
                &mut matrix,
                &mut rhs,
                constraint.node,
                function,
            );
        }
        dims.push(1 + constraint.vector.len());
    }
    (matrix, rhs, dims)
}

fn push_soc_state_function_row(
    problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
    matrix: &mut Vec<f64>,
    rhs: &mut Vec<f64>,
    node: usize,
    function: &ScvxLinearStateFunction,
) {
    let entries = function
        .coefficients
        .iter()
        .enumerate()
        .map(|(component, coefficient)| (layout.state(problem, node, component), -coefficient))
        .collect::<Vec<_>>();
    push_dense_row(matrix, layout.variable_count, &entries);
    rhs.push(function.offset);
}

fn push_soc_control_function_row(
    problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
    matrix: &mut Vec<f64>,
    rhs: &mut Vec<f64>,
    node: usize,
    function: &ScvxLinearControlFunction,
) {
    let entries = function
        .coefficients
        .iter()
        .enumerate()
        .map(|(component, coefficient)| (layout.control(problem, node, component), -coefficient))
        .collect::<Vec<_>>();
    push_dense_row(matrix, layout.variable_count, &entries);
    rhs.push(function.offset);
}

fn scvx_csc_from_dense_upper_triangle(dense: &[f64], dim: usize) -> CscMatrix<f64> {
    let mut col_ptr = Vec::with_capacity(dim + 1);
    let mut row_idx = Vec::new();
    let mut values = Vec::new();
    col_ptr.push(0);
    for col in 0..dim {
        for row in 0..=col {
            let value = dense[row * dim + col];
            if value != 0.0 {
                row_idx.push(row);
                values.push(value);
            }
        }
        col_ptr.push(values.len());
    }
    CscMatrix::new(dim, dim, col_ptr, row_idx, values)
}

fn scvx_csc_from_dense_rows(dense: &[f64], rows: usize, cols: usize) -> CscMatrix<f64> {
    let mut col_ptr = Vec::with_capacity(cols + 1);
    let mut row_idx = Vec::new();
    let mut values = Vec::new();
    col_ptr.push(0);
    for col in 0..cols {
        for row in 0..rows {
            let value = dense[row * cols + col];
            if value != 0.0 {
                row_idx.push(row);
                values.push(value);
            }
        }
        col_ptr.push(values.len());
    }
    CscMatrix::new(rows, cols, col_ptr, row_idx, values)
}

fn active_state_triggered_constraints(
    problem: &ScvxDiscreteAffineProblem,
    states: &[f64],
) -> Result<Vec<usize>, TrajoptError> {
    let mut active = Vec::new();
    for (index, constraint) in problem.state_triggered_constraints.iter().enumerate() {
        let state = node_state(problem, states, constraint.node)?;
        if evaluate_linear_state_function(&constraint.trigger, state) <= 0.0 {
            active.push(index);
        }
    }
    Ok(active)
}

fn scvx_merit(
    problem: &ScvxDiscreteAffineProblem,
    settings: &ScvxSettings,
    states: &[f64],
    controls: &[f64],
    virtual_controls: &[f64],
) -> Result<f64, TrajoptError> {
    let terminal = terminal_error_norm(problem, states);
    let control_norm_sq = controls.iter().map(|value| value * value).sum::<f64>();
    let virtual_norm_sq = virtual_controls
        .iter()
        .map(|value| value * value)
        .sum::<f64>();
    let violation = max_state_triggered_violation(problem, states)?
        .max(max_linear_state_violation(problem, states)?)
        .max(max_linear_control_violation(problem, controls)?)
        .max(max_second_order_cone_violation(problem, states, controls)?);
    Ok(0.5 * settings.terminal_weight * terminal * terminal
        + 0.5 * settings.control_weight * control_norm_sq
        + 0.5 * settings.virtual_control_weight * virtual_norm_sq
        + 0.5 * settings.state_triggered_constraint_penalty * violation * violation)
}

fn dynamics_defects(
    problem: &ScvxDiscreteAffineProblem,
    states: &[f64],
    controls: &[f64],
) -> Result<Vec<f64>, TrajoptError> {
    let mut defects = vec![0.0; problem.horizon_steps * problem.state_dimension];
    for interval in 0..problem.horizon_steps {
        let segment = &problem.dynamics[interval];
        let current = node_state(problem, states, interval)?;
        let next = node_state(problem, states, interval + 1)?;
        let left_control = node_control(problem, controls, interval)?;
        let right_control = node_control(problem, controls, interval + 1)?;
        for row in 0..problem.state_dimension {
            let mut predicted = segment.affine[row];
            for (col, value) in current.iter().enumerate() {
                predicted += segment.state_transition[row * problem.state_dimension + col] * value;
            }
            for (col, value) in left_control.iter().enumerate() {
                predicted += segment.control_left[row * problem.control_dimension + col] * value;
            }
            for (col, value) in right_control.iter().enumerate() {
                predicted += segment.control_right[row * problem.control_dimension + col] * value;
            }
            defects[interval * problem.state_dimension + row] = next[row] - predicted;
        }
    }
    Ok(defects)
}

fn solution_report(
    problem: &ScvxDiscreteAffineProblem,
    solver_status: SolverStatus,
    termination_status: ScvxTerminationStatus,
    converged: bool,
    trajectory: ScvxTrajectory,
    merit: f64,
    iterations: Vec<ScvxIterationReport>,
) -> ScvxSolution {
    let ScvxTrajectory {
        states,
        controls,
        virtual_controls,
    } = trajectory;
    let terminal_error_norm = terminal_error_norm(problem, &states);
    let max_virtual_control_norm = max_interval_norm(
        &virtual_controls,
        problem.horizon_steps,
        problem.state_dimension,
    );
    let max_state_triggered_violation =
        max_state_triggered_violation(problem, &states).unwrap_or(f64::INFINITY);
    let max_linear_state_violation =
        max_linear_state_violation(problem, &states).unwrap_or(f64::INFINITY);
    let max_linear_control_violation =
        max_linear_control_violation(problem, &controls).unwrap_or(f64::INFINITY);
    let max_second_order_cone_violation =
        max_second_order_cone_violation(problem, &states, &controls).unwrap_or(f64::INFINITY);
    ScvxSolution {
        solver_status,
        termination_status,
        converged,
        states,
        controls,
        virtual_controls,
        merit,
        terminal_error_norm,
        max_virtual_control_norm,
        max_state_triggered_violation,
        max_linear_state_violation,
        max_linear_control_violation,
        max_second_order_cone_violation,
        iterations,
    }
}

fn max_abs_difference(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right)
        .map(|(left, right)| (left - right).abs())
        .fold(0.0_f64, f64::max)
}

fn decode_states(
    _problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
    primal: &[f64],
) -> Vec<f64> {
    primal[layout.state_offset..layout.control_offset].to_vec()
}

fn decode_controls(
    _problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
    primal: &[f64],
) -> Vec<f64> {
    primal[layout.control_offset..layout.virtual_offset].to_vec()
}

fn decode_virtual_controls(
    _problem: &ScvxDiscreteAffineProblem,
    layout: &ScvxDecisionLayout,
    primal: &[f64],
) -> Vec<f64> {
    primal[layout.virtual_offset..].to_vec()
}

fn terminal_error_norm(problem: &ScvxDiscreteAffineProblem, states: &[f64]) -> f64 {
    let terminal_offset = problem.horizon_steps * problem.state_dimension;
    let mut sum_sq = 0.0;
    for component in 0..problem.state_dimension {
        let error = states[terminal_offset + component] - problem.target.state[component];
        sum_sq += terminal_component_weight(problem, component) * error * error;
    }
    sum_sq.sqrt()
}

fn terminal_component_weight(problem: &ScvxDiscreteAffineProblem, component: usize) -> f64 {
    problem
        .target
        .weights
        .as_ref()
        .map_or(1.0, |weights| weights[component])
}

fn max_state_triggered_violation(
    problem: &ScvxDiscreteAffineProblem,
    states: &[f64],
) -> Result<f64, TrajoptError> {
    let mut max_violation: f64 = 0.0;
    for constraint in &problem.state_triggered_constraints {
        let state = node_state(problem, states, constraint.node)?;
        if evaluate_linear_state_function(&constraint.trigger, state) <= 0.0 {
            max_violation = max_violation.max(evaluate_linear_state_function(
                &constraint.constraint,
                state,
            ));
        }
    }
    Ok(max_violation.max(0.0))
}

fn max_linear_state_violation(
    problem: &ScvxDiscreteAffineProblem,
    states: &[f64],
) -> Result<f64, TrajoptError> {
    let mut max_violation: f64 = 0.0;
    for constraint in &problem.linear_state_constraints {
        let state = node_state(problem, states, constraint.node)?;
        max_violation =
            max_violation.max(evaluate_linear_state_function(&constraint.function, state));
    }
    Ok(max_violation.max(0.0))
}

fn max_linear_control_violation(
    problem: &ScvxDiscreteAffineProblem,
    controls: &[f64],
) -> Result<f64, TrajoptError> {
    let mut max_violation: f64 = 0.0;
    for constraint in &problem.linear_control_constraints {
        let control = node_control(problem, controls, constraint.node)?;
        let value = constraint
            .coefficients
            .iter()
            .zip(control)
            .map(|(coefficient, value)| coefficient * value)
            .sum::<f64>()
            - constraint.upper_bound;
        max_violation = max_violation.max(value);
    }
    Ok(max_violation.max(0.0))
}

fn max_second_order_cone_violation(
    problem: &ScvxDiscreteAffineProblem,
    states: &[f64],
    controls: &[f64],
) -> Result<f64, TrajoptError> {
    let mut max_violation: f64 = 0.0;
    for constraint in &problem.second_order_state_constraints {
        let state = node_state(problem, states, constraint.node)?;
        let scalar = evaluate_linear_state_function(&constraint.scalar, state);
        let vector_norm = constraint
            .vector
            .iter()
            .map(|function| evaluate_linear_state_function(function, state))
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        max_violation = max_violation.max(vector_norm - scalar);
    }
    for constraint in &problem.second_order_control_constraints {
        let control = node_control(problem, controls, constraint.node)?;
        let scalar = evaluate_linear_control_function(&constraint.scalar, control);
        let vector_norm = constraint
            .vector
            .iter()
            .map(|function| evaluate_linear_control_function(function, control))
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();
        max_violation = max_violation.max(vector_norm - scalar);
    }
    Ok(max_violation.max(0.0))
}

fn evaluate_linear_state_function(function: &ScvxLinearStateFunction, state: &[f64]) -> f64 {
    function.offset
        + function
            .coefficients
            .iter()
            .zip(state)
            .map(|(coefficient, value)| coefficient * value)
            .sum::<f64>()
}

fn evaluate_linear_control_function(function: &ScvxLinearControlFunction, control: &[f64]) -> f64 {
    function.offset
        + function
            .coefficients
            .iter()
            .zip(control)
            .map(|(coefficient, value)| coefficient * value)
            .sum::<f64>()
}

fn node_state<'a>(
    problem: &ScvxDiscreteAffineProblem,
    states: &'a [f64],
    node: usize,
) -> Result<&'a [f64], TrajoptError> {
    if node > problem.horizon_steps {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx node index is out of range",
        });
    }
    let start = node * problem.state_dimension;
    Ok(&states[start..start + problem.state_dimension])
}

fn node_control<'a>(
    problem: &ScvxDiscreteAffineProblem,
    controls: &'a [f64],
    node: usize,
) -> Result<&'a [f64], TrajoptError> {
    if node > problem.horizon_steps {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx control node index is out of range",
        });
    }
    let start = node * problem.control_dimension;
    Ok(&controls[start..start + problem.control_dimension])
}

fn max_interval_norm(values: &[f64], interval_count: usize, dimension: usize) -> f64 {
    let mut max_norm: f64 = 0.0;
    for interval in 0..interval_count {
        let mut sum_sq = 0.0;
        for component in 0..dimension {
            let value = values[interval * dimension + component];
            sum_sq += value * value;
        }
        max_norm = max_norm.max(sum_sq.sqrt());
    }
    max_norm
}

fn reduction_ratio(actual_reduction: f64, predicted_reduction: f64) -> f64 {
    if predicted_reduction.abs() <= 1.0e-12 {
        if actual_reduction >= -1.0e-12 {
            1.0
        } else {
            0.0
        }
    } else {
        actual_reduction / predicted_reduction
    }
}

fn scvx_left_node_for_time(times_s: &[f64], time_s: f64) -> Result<usize, TrajoptError> {
    for (node, &candidate) in times_s.iter().take(times_s.len() - 1).enumerate() {
        if candidate.to_bits() == time_s.to_bits() {
            return Ok(node);
        }
    }
    Err(TrajoptError::InvalidPayload {
        reason: "SCvx time is not an interval-left node",
    })
}

fn push_inequality(
    matrix: &mut Vec<f64>,
    rhs: &mut Vec<f64>,
    variable_count: usize,
    entries: &[(usize, f64)],
    bound: f64,
) {
    push_dense_row(matrix, variable_count, entries);
    rhs.push(bound);
}

fn push_dense_row(matrix: &mut Vec<f64>, variable_count: usize, entries: &[(usize, f64)]) {
    let row_start = matrix.len();
    matrix.resize(row_start + variable_count, 0.0);
    for &(column, value) in entries {
        matrix[row_start + column] += value;
    }
}

fn validate_scvx_problem(problem: &ScvxDiscreteAffineProblem) -> Result<(), TrajoptError> {
    if problem.state_dimension == 0 || problem.control_dimension == 0 || problem.horizon_steps == 0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx dimensions and horizon must be nonzero",
        });
    }
    let node_count = problem.horizon_steps + 1;
    validate_len(
        "SCvx initial state dimensions are inconsistent",
        &problem.initial_state,
        problem.state_dimension,
    )?;
    validate_len(
        "SCvx target dimensions are inconsistent",
        &problem.target.state,
        problem.state_dimension,
    )?;
    if let Some(weights) = &problem.target.weights {
        validate_len(
            "SCvx terminal target weight dimensions are inconsistent",
            weights,
            problem.state_dimension,
        )?;
        validate_finite("SCvx terminal target weights must be finite", weights)?;
        if weights.iter().any(|weight| *weight < 0.0) {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx terminal target weights must be non-negative",
            });
        }
    }
    validate_len(
        "SCvx initial state trajectory dimensions are inconsistent",
        &problem.initial_states,
        node_count * problem.state_dimension,
    )?;
    validate_len(
        "SCvx initial control trajectory dimensions are inconsistent",
        &problem.initial_controls,
        node_count * problem.control_dimension,
    )?;
    validate_len(
        "SCvx control lower bounds dimensions are inconsistent",
        &problem.control_lower,
        problem.control_dimension,
    )?;
    validate_len(
        "SCvx control upper bounds dimensions are inconsistent",
        &problem.control_upper,
        problem.control_dimension,
    )?;
    if problem.dynamics.len() != problem.horizon_steps {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx dynamics segment count must equal horizon_steps",
        });
    }
    validate_finite("SCvx initial state must be finite", &problem.initial_state)?;
    validate_finite("SCvx target state must be finite", &problem.target.state)?;
    validate_finite(
        "SCvx initial state trajectory must be finite",
        &problem.initial_states,
    )?;
    validate_finite(
        "SCvx initial control trajectory must be finite",
        &problem.initial_controls,
    )?;
    validate_finite(
        "SCvx control lower bounds must be finite",
        &problem.control_lower,
    )?;
    validate_finite(
        "SCvx control upper bounds must be finite",
        &problem.control_upper,
    )?;
    for component in 0..problem.control_dimension {
        if problem.control_lower[component] > problem.control_upper[component] {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx control bounds are inverted",
            });
        }
    }
    for segment in &problem.dynamics {
        validate_len(
            "SCvx state-transition dimensions are inconsistent",
            &segment.state_transition,
            problem.state_dimension * problem.state_dimension,
        )?;
        validate_len(
            "SCvx left-control dimensions are inconsistent",
            &segment.control_left,
            problem.state_dimension * problem.control_dimension,
        )?;
        validate_len(
            "SCvx right-control dimensions are inconsistent",
            &segment.control_right,
            problem.state_dimension * problem.control_dimension,
        )?;
        validate_len(
            "SCvx affine dynamics dimensions are inconsistent",
            &segment.affine,
            problem.state_dimension,
        )?;
        validate_finite(
            "SCvx state-transition values must be finite",
            &segment.state_transition,
        )?;
        validate_finite(
            "SCvx left-control values must be finite",
            &segment.control_left,
        )?;
        validate_finite(
            "SCvx right-control values must be finite",
            &segment.control_right,
        )?;
        validate_finite("SCvx affine values must be finite", &segment.affine)?;
    }
    for constraint in &problem.state_triggered_constraints {
        if constraint.node > problem.horizon_steps {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx state-triggered constraint node is out of range",
            });
        }
        validate_len(
            "SCvx trigger dimensions are inconsistent",
            &constraint.trigger.coefficients,
            problem.state_dimension,
        )?;
        validate_len(
            "SCvx state-triggered constraint dimensions are inconsistent",
            &constraint.constraint.coefficients,
            problem.state_dimension,
        )?;
        validate_finite(
            "SCvx trigger coefficients must be finite",
            &constraint.trigger.coefficients,
        )?;
        validate_finite(
            "SCvx state-triggered constraint coefficients must be finite",
            &constraint.constraint.coefficients,
        )?;
        if !constraint.trigger.offset.is_finite() || !constraint.constraint.offset.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx state-triggered offsets must be finite",
            });
        }
    }
    for constraint in &problem.linear_state_constraints {
        if constraint.node > problem.horizon_steps {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx linear state constraint node is out of range",
            });
        }
        validate_len(
            "SCvx linear state constraint dimensions are inconsistent",
            &constraint.function.coefficients,
            problem.state_dimension,
        )?;
        validate_finite(
            "SCvx linear state constraint coefficients must be finite",
            &constraint.function.coefficients,
        )?;
        if !constraint.function.offset.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx linear state constraint offset must be finite",
            });
        }
    }
    for constraint in &problem.linear_control_constraints {
        if constraint.node > problem.horizon_steps {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx linear control constraint node is out of range",
            });
        }
        validate_len(
            "SCvx linear control constraint dimensions are inconsistent",
            &constraint.coefficients,
            problem.control_dimension,
        )?;
        validate_finite(
            "SCvx linear control constraint coefficients must be finite",
            &constraint.coefficients,
        )?;
        if !constraint.upper_bound.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx linear control constraint bound must be finite",
            });
        }
    }
    for constraint in &problem.second_order_state_constraints {
        validate_scvx_second_order_state_constraint(
            constraint,
            problem.horizon_steps,
            problem.state_dimension,
            "SCvx",
        )?;
    }
    for constraint in &problem.second_order_control_constraints {
        validate_scvx_second_order_control_constraint(
            constraint,
            problem.horizon_steps,
            problem.control_dimension,
            "SCvx",
        )?;
    }
    Ok(())
}

fn validate_scvx_second_order_state_constraint(
    constraint: &ScvxSecondOrderConeStateConstraint,
    horizon_steps: usize,
    state_dimension: usize,
    prefix: &'static str,
) -> Result<(), TrajoptError> {
    if constraint.node > horizon_steps {
        return Err(TrajoptError::InvalidPayload {
            reason: if prefix == "SCvx FOH" {
                "SCvx FOH second-order state constraint node is out of range"
            } else {
                "SCvx second-order state constraint node is out of range"
            },
        });
    }
    if constraint.vector.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: if prefix == "SCvx FOH" {
                "SCvx FOH second-order state constraint vector must be nonempty"
            } else {
                "SCvx second-order state constraint vector must be nonempty"
            },
        });
    }
    validate_scvx_linear_state_function(
        &constraint.scalar,
        state_dimension,
        if prefix == "SCvx FOH" {
            "SCvx FOH second-order state scalar"
        } else {
            "SCvx second-order state scalar"
        },
    )?;
    for function in &constraint.vector {
        validate_scvx_linear_state_function(
            function,
            state_dimension,
            if prefix == "SCvx FOH" {
                "SCvx FOH second-order state vector"
            } else {
                "SCvx second-order state vector"
            },
        )?;
    }
    Ok(())
}

fn validate_scvx_second_order_control_constraint(
    constraint: &ScvxSecondOrderConeControlConstraint,
    horizon_steps: usize,
    control_dimension: usize,
    prefix: &'static str,
) -> Result<(), TrajoptError> {
    if constraint.node > horizon_steps {
        return Err(TrajoptError::InvalidPayload {
            reason: if prefix == "SCvx FOH" {
                "SCvx FOH second-order control constraint node is out of range"
            } else {
                "SCvx second-order control constraint node is out of range"
            },
        });
    }
    if constraint.vector.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: if prefix == "SCvx FOH" {
                "SCvx FOH second-order control constraint vector must be nonempty"
            } else {
                "SCvx second-order control constraint vector must be nonempty"
            },
        });
    }
    validate_scvx_linear_control_function(
        &constraint.scalar,
        control_dimension,
        if prefix == "SCvx FOH" {
            "SCvx FOH second-order control scalar"
        } else {
            "SCvx second-order control scalar"
        },
    )?;
    for function in &constraint.vector {
        validate_scvx_linear_control_function(
            function,
            control_dimension,
            if prefix == "SCvx FOH" {
                "SCvx FOH second-order control vector"
            } else {
                "SCvx second-order control vector"
            },
        )?;
    }
    Ok(())
}

fn validate_scvx_linear_state_function(
    function: &ScvxLinearStateFunction,
    state_dimension: usize,
    reason_prefix: &'static str,
) -> Result<(), TrajoptError> {
    validate_len(reason_prefix, &function.coefficients, state_dimension)?;
    validate_finite(reason_prefix, &function.coefficients)?;
    if !function.offset.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx linear state function offset must be finite",
        });
    }
    Ok(())
}

fn validate_scvx_linear_control_function(
    function: &ScvxLinearControlFunction,
    control_dimension: usize,
    reason_prefix: &'static str,
) -> Result<(), TrajoptError> {
    validate_len(reason_prefix, &function.coefficients, control_dimension)?;
    validate_finite(reason_prefix, &function.coefficients)?;
    if !function.offset.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx linear control function offset must be finite",
        });
    }
    Ok(())
}

fn validate_scvx_settings(settings: ScvxSettings) -> Result<(), TrajoptError> {
    if settings.max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx max_iterations must be >= 1",
        });
    }
    validate_positive(
        "SCvx terminal_tolerance must be positive",
        settings.terminal_tolerance,
    )?;
    validate_positive(
        "SCvx virtual_control_tolerance must be positive",
        settings.virtual_control_tolerance,
    )?;
    validate_positive(
        "SCvx constraint_tolerance must be positive",
        settings.constraint_tolerance,
    )?;
    validate_positive(
        "SCvx initial_trust_region_radius must be positive",
        settings.initial_trust_region_radius,
    )?;
    validate_positive(
        "SCvx min_trust_region_radius must be positive",
        settings.min_trust_region_radius,
    )?;
    validate_positive(
        "SCvx max_trust_region_radius must be positive",
        settings.max_trust_region_radius,
    )?;
    if settings.min_trust_region_radius > settings.max_trust_region_radius {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx trust-region bounds are inverted",
        });
    }
    validate_positive(
        "SCvx trust_region_expand must be positive",
        settings.trust_region_expand,
    )?;
    validate_positive(
        "SCvx trust_region_shrink must be positive",
        settings.trust_region_shrink,
    )?;
    if settings.trust_region_shrink >= 1.0 || settings.trust_region_expand <= 1.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx trust-region factors must shrink below one and expand above one",
        });
    }
    validate_positive("SCvx accept_ratio must be positive", settings.accept_ratio)?;
    if settings.accept_ratio > 1.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx accept_ratio must be <= 1",
        });
    }
    validate_positive(
        "SCvx terminal_weight must be positive",
        settings.terminal_weight,
    )?;
    validate_positive(
        "SCvx control_weight must be positive",
        settings.control_weight,
    )?;
    validate_positive(
        "SCvx virtual_control_weight must be positive",
        settings.virtual_control_weight,
    )?;
    validate_positive(
        "SCvx state_triggered_constraint_penalty must be positive",
        settings.state_triggered_constraint_penalty,
    )?;
    if !settings.regularization.is_finite() || settings.regularization < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx regularization must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_scvx_time_search_settings(
    settings: ScvxTimeOfFlightSearchSettings,
) -> Result<(), TrajoptError> {
    validate_positive(
        "SCvx lower_time_of_flight_s must be positive and finite",
        settings.lower_time_of_flight_s,
    )?;
    validate_positive(
        "SCvx upper_time_of_flight_s must be positive and finite",
        settings.upper_time_of_flight_s,
    )?;
    if settings.lower_time_of_flight_s >= settings.upper_time_of_flight_s {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight search bounds must be ordered",
        });
    }
    validate_positive(
        "SCvx time-of-flight search tolerance must be positive and finite",
        settings.tolerance_s,
    )?;
    if settings.max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight search max_iterations must be >= 1",
        });
    }
    if !settings.time_penalty_weight.is_finite() || settings.time_penalty_weight < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time_penalty_weight must be finite and non-negative",
        });
    }
    validate_scvx_settings(settings.scvx_settings)
}

fn validate_scvx_continuous_foh_time_search_settings(
    settings: ScvxContinuousFohTimeOfFlightSearchSettings,
) -> Result<(), TrajoptError> {
    validate_positive(
        "SCvx continuous FOH lower_time_of_flight_s must be positive and finite",
        settings.lower_time_of_flight_s,
    )?;
    validate_positive(
        "SCvx continuous FOH upper_time_of_flight_s must be positive and finite",
        settings.upper_time_of_flight_s,
    )?;
    if settings.lower_time_of_flight_s >= settings.upper_time_of_flight_s {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight search bounds must be ordered",
        });
    }
    validate_positive(
        "SCvx continuous FOH time-of-flight search tolerance must be positive and finite",
        settings.tolerance_s,
    )?;
    if settings.max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight search max_iterations must be >= 1",
        });
    }
    if !settings.time_penalty_weight.is_finite() || settings.time_penalty_weight < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time_penalty_weight must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_scvx_continuous_foh_time_sweep_settings(
    settings: &ScvxContinuousFohTimeOfFlightSweepSettings,
) -> Result<(), TrajoptError> {
    if settings.center_times_of_flight_s.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight sweep centers must be nonempty",
        });
    }
    validate_finite(
        "SCvx continuous FOH time-of-flight sweep centers must be finite",
        &settings.center_times_of_flight_s,
    )?;
    if !settings
        .center_times_of_flight_s
        .iter()
        .all(|center| *center > 0.0)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight sweep centers must be positive",
        });
    }
    if !settings
        .center_times_of_flight_s
        .windows(2)
        .all(|window| window[1] > window[0])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight sweep centers must be strictly increasing",
        });
    }
    validate_positive(
        "SCvx continuous FOH time-of-flight sweep half_width_s must be positive and finite",
        settings.half_width_s,
    )?;
    if settings
        .center_times_of_flight_s
        .iter()
        .any(|center| *center <= settings.half_width_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight sweep half_width_s must keep lower bounds positive",
        });
    }
    for &center_time_of_flight_s in &settings.center_times_of_flight_s {
        validate_scvx_continuous_foh_time_search_settings(
            ScvxContinuousFohTimeOfFlightSearchSettings {
                lower_time_of_flight_s: center_time_of_flight_s - settings.half_width_s,
                upper_time_of_flight_s: center_time_of_flight_s + settings.half_width_s,
                tolerance_s: settings.tolerance_s,
                max_iterations: settings.max_iterations,
                time_penalty_weight: settings.time_penalty_weight,
            },
        )?;
    }
    Ok(())
}

fn validate_scvx_time_of_flight_candidate_time(
    settings: ScvxTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
) -> Result<(), TrajoptError> {
    validate_positive(
        "SCvx time-of-flight candidate time must be positive and finite",
        time_of_flight_s,
    )?;
    if time_of_flight_s < settings.lower_time_of_flight_s
        || time_of_flight_s > settings.upper_time_of_flight_s
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight candidate time must be inside the search bracket",
        });
    }
    Ok(())
}

fn validate_scvx_continuous_foh_time_of_flight_candidate_time(
    settings: ScvxContinuousFohTimeOfFlightSearchSettings,
    time_of_flight_s: f64,
) -> Result<(), TrajoptError> {
    validate_positive(
        "SCvx continuous FOH time-of-flight candidate time must be positive and finite",
        time_of_flight_s,
    )?;
    if time_of_flight_s < settings.lower_time_of_flight_s
        || time_of_flight_s > settings.upper_time_of_flight_s
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH time-of-flight candidate time must be inside the search bracket",
        });
    }
    Ok(())
}

fn validate_scvx_time_sweep_settings(
    settings: &ScvxTimeOfFlightSweepSettings,
) -> Result<(), TrajoptError> {
    if settings.center_times_of_flight_s.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight sweep centers must be nonempty",
        });
    }
    validate_finite(
        "SCvx time-of-flight sweep centers must be finite",
        &settings.center_times_of_flight_s,
    )?;
    if !settings
        .center_times_of_flight_s
        .iter()
        .all(|center| *center > 0.0)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight sweep centers must be positive",
        });
    }
    if !settings
        .center_times_of_flight_s
        .windows(2)
        .all(|window| window[1] > window[0])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight sweep centers must be strictly increasing",
        });
    }
    validate_positive(
        "SCvx time-of-flight sweep half_width_s must be positive and finite",
        settings.half_width_s,
    )?;
    if settings
        .center_times_of_flight_s
        .iter()
        .any(|center| *center <= settings.half_width_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx time-of-flight sweep half_width_s must keep lower bounds positive",
        });
    }
    for &center_time_of_flight_s in &settings.center_times_of_flight_s {
        validate_scvx_time_search_settings(ScvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s: center_time_of_flight_s - settings.half_width_s,
            upper_time_of_flight_s: center_time_of_flight_s + settings.half_width_s,
            tolerance_s: settings.tolerance_s,
            max_iterations: settings.max_iterations,
            time_penalty_weight: settings.time_penalty_weight,
            scvx_settings: settings.scvx_settings,
        })?;
    }
    Ok(())
}

fn trajopt_error_static_reason(error: &TrajoptError) -> &'static str {
    match error {
        TrajoptError::InvalidPayload { reason } | TrajoptError::SingularSystem { reason } => reason,
        TrajoptError::Serialize(_) => "SCvx time-of-flight sweep serialization failed",
    }
}

fn validate_scvx_continuous_foh_settings(
    settings: ScvxContinuousFohSettings,
) -> Result<(), TrajoptError> {
    if settings.max_outer_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH outer iteration budget must be nonzero",
        });
    }
    if !settings.reference_update_tolerance.is_finite() || settings.reference_update_tolerance < 0.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx continuous FOH reference update tolerance must be finite",
        });
    }
    validate_scvx_settings(settings.scvx_settings)?;
    validate_positive(
        "SCvx continuous FOH finite-difference step must be positive",
        settings.linearization_settings.finite_difference_step,
    )
}

fn validate_scvx_foh_problem_definition(
    definition: &ScvxForwardEulerFohProblemDefinition,
    settings: ScvxForwardEulerLinearizationSettings,
) -> Result<(), TrajoptError> {
    validate_scvx_linearization_inputs(
        definition.state_dimension,
        definition.control_dimension,
        &definition.times_s,
        &definition.initial_states,
        &definition.initial_controls,
        settings,
    )?;
    let horizon_steps = definition.times_s.len() - 1;
    validate_len(
        "SCvx FOH initial state dimensions are inconsistent",
        &definition.initial_state,
        definition.state_dimension,
    )?;
    validate_len(
        "SCvx FOH target dimensions are inconsistent",
        &definition.target.state,
        definition.state_dimension,
    )?;
    if let Some(weights) = &definition.target.weights {
        validate_len(
            "SCvx FOH terminal target weight dimensions are inconsistent",
            weights,
            definition.state_dimension,
        )?;
        validate_finite("SCvx FOH terminal target weights must be finite", weights)?;
        if weights.iter().any(|weight| *weight < 0.0) {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH terminal target weights must be non-negative",
            });
        }
    }
    validate_len(
        "SCvx FOH control lower bounds dimensions are inconsistent",
        &definition.control_lower,
        definition.control_dimension,
    )?;
    validate_len(
        "SCvx FOH control upper bounds dimensions are inconsistent",
        &definition.control_upper,
        definition.control_dimension,
    )?;
    validate_finite(
        "SCvx FOH initial state must be finite",
        &definition.initial_state,
    )?;
    validate_finite("SCvx FOH target must be finite", &definition.target.state)?;
    validate_finite(
        "SCvx FOH control lower bounds must be finite",
        &definition.control_lower,
    )?;
    validate_finite(
        "SCvx FOH control upper bounds must be finite",
        &definition.control_upper,
    )?;
    for component in 0..definition.control_dimension {
        if definition.control_lower[component] > definition.control_upper[component] {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH control bounds are inverted",
            });
        }
    }
    for constraint in &definition.state_triggered_constraints {
        if constraint.node > horizon_steps {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH state-triggered constraint node is out of range",
            });
        }
        validate_len(
            "SCvx FOH trigger dimensions are inconsistent",
            &constraint.trigger.coefficients,
            definition.state_dimension,
        )?;
        validate_len(
            "SCvx FOH state-triggered constraint dimensions are inconsistent",
            &constraint.constraint.coefficients,
            definition.state_dimension,
        )?;
        validate_finite(
            "SCvx FOH trigger coefficients must be finite",
            &constraint.trigger.coefficients,
        )?;
        validate_finite(
            "SCvx FOH state-triggered constraint coefficients must be finite",
            &constraint.constraint.coefficients,
        )?;
        if !constraint.trigger.offset.is_finite() || !constraint.constraint.offset.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH state-triggered offsets must be finite",
            });
        }
    }
    for constraint in &definition.linear_state_constraints {
        if constraint.node > horizon_steps {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH linear state constraint node is out of range",
            });
        }
        validate_len(
            "SCvx FOH linear state constraint dimensions are inconsistent",
            &constraint.function.coefficients,
            definition.state_dimension,
        )?;
        validate_finite(
            "SCvx FOH linear state constraint coefficients must be finite",
            &constraint.function.coefficients,
        )?;
        if !constraint.function.offset.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH linear state constraint offset must be finite",
            });
        }
    }
    for constraint in &definition.linear_control_constraints {
        if constraint.node > horizon_steps {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH linear control constraint node is out of range",
            });
        }
        validate_len(
            "SCvx FOH linear control constraint dimensions are inconsistent",
            &constraint.coefficients,
            definition.control_dimension,
        )?;
        validate_finite(
            "SCvx FOH linear control constraint coefficients must be finite",
            &constraint.coefficients,
        )?;
        if !constraint.upper_bound.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx FOH linear control constraint bound must be finite",
            });
        }
    }
    for constraint in &definition.second_order_state_constraints {
        validate_scvx_second_order_state_constraint(
            constraint,
            horizon_steps,
            definition.state_dimension,
            "SCvx FOH",
        )?;
    }
    for constraint in &definition.second_order_control_constraints {
        validate_scvx_second_order_control_constraint(
            constraint,
            horizon_steps,
            definition.control_dimension,
            "SCvx FOH",
        )?;
    }
    Ok(())
}

fn validate_scvx_rigid_body_problem_definition(
    definition: &ScvxForwardEulerFohProblemDefinition,
) -> Result<(), TrajoptError> {
    if definition.state_dimension != ScvxRigidBodyStateLayout::DIMENSION
        || definition.control_dimension != ScvxRigidBodyControlLayout::DIMENSION
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx rigid-body FOH problem dimensions must match the rigid-body layouts",
        });
    }
    Ok(())
}

fn validate_scvx_powered_landing_problem_definition(
    definition: &ScvxForwardEulerFohProblemDefinition,
) -> Result<(), TrajoptError> {
    if definition.state_dimension != ScvxPoweredLandingStateLayout::DIMENSION
        || definition.control_dimension != ScvxPoweredLandingControlLayout::DIMENSION
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing FOH problem dimensions must match the powered-landing layouts",
        });
    }
    Ok(())
}

fn validate_scvx_reduced_aero_reference_profile(
    profile: &ScvxReducedAeroReferenceProfile,
    node_count: usize,
) -> Result<(), TrajoptError> {
    if profile.coefficients.len() != node_count || profile.parameters.len() != node_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx reduced-aero profile length must match the time grid",
        });
    }
    for coefficients in &profile.coefficients {
        if !coefficients.cn.is_finite()
            || !coefficients.cd.is_finite()
            || !coefficients.cm.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx reduced-aero profile coefficients must be finite",
            });
        }
    }
    for parameters in &profile.parameters {
        if !parameters.dynamic_pressure_pa.is_finite() || parameters.dynamic_pressure_pa < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx reduced-aero profile dynamic pressure must be finite and non-negative",
            });
        }
        validate_positive(
            "SCvx reduced-aero profile reference area must be positive",
            parameters.reference_area_m2,
        )?;
        validate_positive(
            "SCvx reduced-aero profile reference length must be positive",
            parameters.reference_length_m,
        )?;
    }
    Ok(())
}

#[cfg(feature = "aero")]
fn validate_scvx_aero_method_reference_profile(
    profile: &ScvxAeroMethodReferenceProfile,
    node_count: usize,
) -> Result<(), TrajoptError> {
    if profile.contexts.len() != node_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx aero-method context profile length must match the time grid",
        });
    }
    for context in &profile.contexts {
        if !context.mach.is_finite()
            || !context.alpha_deg.is_finite()
            || !context.beta_deg.is_finite()
            || !context.dynamic_pressure_pa.is_finite()
            || !context.knudsen.is_finite()
            || !context.reynolds_length.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx aero-method context profile values must be finite",
            });
        }
        if context.dynamic_pressure_pa < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx aero-method context dynamic pressure must be non-negative",
            });
        }
    }
    Ok(())
}

fn validate_scvx_linearization_inputs(
    state_dimension: usize,
    control_dimension: usize,
    times_s: &[f64],
    reference_states: &[f64],
    reference_controls: &[f64],
    settings: ScvxForwardEulerLinearizationSettings,
) -> Result<(), TrajoptError> {
    if state_dimension == 0 || control_dimension == 0 || times_s.len() < 2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx linearization dimensions and time grid must be nonzero",
        });
    }
    validate_finite("SCvx linearization times must be finite", times_s)?;
    for window in times_s.windows(2) {
        if window[1] <= window[0] {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx linearization times must be strictly increasing",
            });
        }
    }
    let node_count = times_s.len();
    validate_len(
        "SCvx linearization state trajectory dimensions are inconsistent",
        reference_states,
        node_count * state_dimension,
    )?;
    validate_len(
        "SCvx linearization control trajectory dimensions are inconsistent",
        reference_controls,
        node_count * control_dimension,
    )?;
    validate_finite(
        "SCvx linearization state trajectory must be finite",
        reference_states,
    )?;
    validate_finite(
        "SCvx linearization control trajectory must be finite",
        reference_controls,
    )?;
    validate_positive(
        "SCvx finite_difference_step must be positive",
        settings.finite_difference_step,
    )
}

fn evaluate_scvx_continuous_dynamics<F>(
    dynamics: &mut F,
    time_s: f64,
    state: &[f64],
    control: &[f64],
    state_dimension: usize,
) -> Result<Vec<f64>, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
{
    let mut output = vec![0.0; state_dimension];
    dynamics(time_s, state, control, &mut output)?;
    validate_finite("SCvx continuous dynamics output must be finite", &output)?;
    Ok(output)
}

fn finite_difference_scvx_state_jacobian<F>(
    dynamics: &mut F,
    time_s: f64,
    state: &[f64],
    control: &[f64],
    finite_difference_step: f64,
    state_dimension: usize,
) -> Result<Vec<f64>, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
{
    let mut jacobian = vec![0.0; state_dimension * state_dimension];
    for column in 0..state_dimension {
        let delta = finite_difference_step * (1.0 + state[column].abs());
        let mut plus_state = state.to_vec();
        let mut minus_state = state.to_vec();
        plus_state[column] += delta;
        minus_state[column] -= delta;
        let plus = evaluate_scvx_continuous_dynamics(
            dynamics,
            time_s,
            &plus_state,
            control,
            state_dimension,
        )?;
        let minus = evaluate_scvx_continuous_dynamics(
            dynamics,
            time_s,
            &minus_state,
            control,
            state_dimension,
        )?;
        for row in 0..state_dimension {
            jacobian[row * state_dimension + column] = (plus[row] - minus[row]) / (2.0 * delta);
        }
    }
    Ok(jacobian)
}

fn finite_difference_scvx_control_jacobian<F>(
    dynamics: &mut F,
    time_s: f64,
    state: &[f64],
    control: &[f64],
    finite_difference_step: f64,
    state_dimension: usize,
    control_dimension: usize,
) -> Result<Vec<f64>, TrajoptError>
where
    F: FnMut(f64, &[f64], &[f64], &mut [f64]) -> Result<(), TrajoptError>,
{
    let mut jacobian = vec![0.0; state_dimension * control_dimension];
    for column in 0..control_dimension {
        let delta = finite_difference_step * (1.0 + control[column].abs());
        let mut plus_control = control.to_vec();
        let mut minus_control = control.to_vec();
        plus_control[column] += delta;
        minus_control[column] -= delta;
        let plus = evaluate_scvx_continuous_dynamics(
            dynamics,
            time_s,
            state,
            &plus_control,
            state_dimension,
        )?;
        let minus = evaluate_scvx_continuous_dynamics(
            dynamics,
            time_s,
            state,
            &minus_control,
            state_dimension,
        )?;
        for row in 0..state_dimension {
            jacobian[row * control_dimension + column] = (plus[row] - minus[row]) / (2.0 * delta);
        }
    }
    Ok(jacobian)
}

fn validate_scvx_rigid_body_dynamics_inputs(
    state: &[f64],
    control: &[f64],
    parameters: &ScvxRigidBodyDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &[f64],
) -> Result<(), TrajoptError> {
    validate_len(
        "SCvx rigid-body state dimensions are inconsistent",
        state,
        ScvxRigidBodyStateLayout::DIMENSION,
    )?;
    validate_len(
        "SCvx rigid-body control dimensions are inconsistent",
        control,
        ScvxRigidBodyControlLayout::DIMENSION,
    )?;
    validate_len(
        "SCvx rigid-body output dimensions are inconsistent",
        output,
        ScvxRigidBodyStateLayout::DIMENSION,
    )?;
    validate_finite("SCvx rigid-body state must be finite", state)?;
    validate_finite("SCvx rigid-body control must be finite", control)?;
    validate_positive("SCvx rigid-body mass must be positive", parameters.mass_kg)?;
    validate_array_finite(
        "SCvx rigid-body inertia tensor must be finite",
        &parameters.inertia_body_kg_m2,
    )?;
    validate_array_finite(
        "SCvx rigid-body inertia-rate tensor must be finite",
        &parameters.inertia_rate_body_kg_m2_s,
    )?;
    validate_array_finite(
        "SCvx rigid-body external acceleration must be finite",
        &parameters.external_acceleration_eci_m_s2,
    )?;
    validate_positive(
        "SCvx rigid-body quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )?;
    let quaternion = ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let norm = (state[quaternion] * state[quaternion]
        + state[quaternion + 1] * state[quaternion + 1]
        + state[quaternion + 2] * state[quaternion + 2]
        + state[quaternion + 3] * state[quaternion + 3])
        .sqrt();
    if (norm - 1.0).abs() > settings.quaternion_norm_tolerance {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx rigid-body quaternion must be unit length",
        });
    }
    if invert_mat3(parameters.inertia_body_kg_m2).is_none() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx rigid-body inertia tensor is not invertible",
        });
    }
    Ok(())
}

fn validate_scvx_powered_landing_dynamics_inputs(
    state: &[f64],
    control: &[f64],
    parameters: &ScvxPoweredLandingDynamicsParameters,
    settings: ScvxRigidBodyDynamicsSettings,
    output: &[f64],
) -> Result<(), TrajoptError> {
    validate_len(
        "SCvx powered-landing state dimensions are inconsistent",
        state,
        ScvxPoweredLandingStateLayout::DIMENSION,
    )?;
    validate_len(
        "SCvx powered-landing control dimensions are inconsistent",
        control,
        ScvxPoweredLandingControlLayout::DIMENSION,
    )?;
    validate_len(
        "SCvx powered-landing output dimensions are inconsistent",
        output,
        ScvxPoweredLandingStateLayout::DIMENSION,
    )?;
    validate_finite("SCvx powered-landing state must be finite", state)?;
    validate_finite("SCvx powered-landing control must be finite", control)?;
    validate_positive(
        "SCvx powered-landing mass must be positive",
        state[ScvxPoweredLandingStateLayout::MASS],
    )?;
    validate_array_finite(
        "SCvx powered-landing gravity must be finite",
        &parameters.gravity_eci_m_s2,
    )?;
    validate_array_finite(
        "SCvx powered-landing inertia tensor must be finite",
        &parameters.inertia_body_kg_m2,
    )?;
    validate_array_finite(
        "SCvx powered-landing thrust application point must be finite",
        &parameters.thrust_application_point_body_m,
    )?;
    if !parameters.mass_flow_coefficient_s_m.is_finite()
        || parameters.mass_flow_coefficient_s_m < 0.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing mass-flow coefficient must be finite and non-negative",
        });
    }
    validate_positive(
        "SCvx powered-landing quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )?;
    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let norm = (state[quaternion] * state[quaternion]
        + state[quaternion + 1] * state[quaternion + 1]
        + state[quaternion + 2] * state[quaternion + 2]
        + state[quaternion + 3] * state[quaternion + 3])
        .sqrt();
    if (norm - 1.0).abs() > settings.quaternion_norm_tolerance {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing quaternion must be unit length",
        });
    }
    if invert_mat3(parameters.inertia_body_kg_m2).is_none() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing inertia tensor is not invertible",
        });
    }
    Ok(())
}

fn validate_scvx_reduced_aero_inputs(
    state: &[f64],
    coefficients: ScvxReducedAeroCoefficients,
    parameters: ScvxReducedAeroParameters,
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<(), TrajoptError> {
    validate_len(
        "SCvx reduced aero state dimensions are inconsistent",
        state,
        ScvxRigidBodyStateLayout::DIMENSION,
    )?;
    validate_finite("SCvx reduced aero state must be finite", state)?;
    validate_positive(
        "SCvx reduced aero quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )?;
    validate_array_finite(
        "SCvx reduced aero coefficients must be finite",
        &[coefficients.cn, coefficients.cd, coefficients.cm],
    )?;
    if !parameters.dynamic_pressure_pa.is_finite() || parameters.dynamic_pressure_pa < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx reduced aero dynamic pressure must be finite and non-negative",
        });
    }
    validate_positive(
        "SCvx reduced aero reference area must be positive",
        parameters.reference_area_m2,
    )?;
    validate_positive(
        "SCvx reduced aero reference length must be positive",
        parameters.reference_length_m,
    )?;
    validate_scvx_rigid_body_quaternion(state, settings)
}

#[cfg(feature = "aero")]
fn validate_scvx_aero_method_state(
    state: &[f64],
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<(), TrajoptError> {
    validate_len(
        "SCvx aero method state dimensions are inconsistent",
        state,
        ScvxRigidBodyStateLayout::DIMENSION,
    )?;
    validate_finite("SCvx aero method state must be finite", state)?;
    validate_positive(
        "SCvx aero method quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )?;
    validate_scvx_rigid_body_quaternion(state, settings)
}

fn validate_scvx_rigid_body_quaternion(
    state: &[f64],
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<(), TrajoptError> {
    let quaternion = ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let norm = (state[quaternion] * state[quaternion]
        + state[quaternion + 1] * state[quaternion + 1]
        + state[quaternion + 2] * state[quaternion + 2]
        + state[quaternion + 3] * state[quaternion + 3])
        .sqrt();
    if (norm - 1.0).abs() > settings.quaternion_norm_tolerance {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx rigid-body quaternion must be unit length",
        });
    }
    Ok(())
}

fn validate_scvx_powered_landing_envelope_inputs(
    times_s: &[f64],
    states: &[f64],
    controls: &[f64],
    limits: ScvxPoweredLandingEnvelopeLimits,
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<(), TrajoptError> {
    if times_s.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope samples must be nonempty",
        });
    }
    validate_finite(
        "SCvx powered-landing envelope times must be finite",
        times_s,
    )?;
    if !times_s.windows(2).all(|window| window[0] < window[1]) {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope times must be strictly increasing",
        });
    }
    validate_len(
        "SCvx powered-landing envelope state dimensions are inconsistent",
        states,
        times_s.len() * ScvxPoweredLandingStateLayout::DIMENSION,
    )?;
    validate_len(
        "SCvx powered-landing envelope control dimensions are inconsistent",
        controls,
        times_s.len() * ScvxPoweredLandingControlLayout::DIMENSION,
    )?;
    validate_finite(
        "SCvx powered-landing envelope states must be finite",
        states,
    )?;
    validate_finite(
        "SCvx powered-landing envelope controls must be finite",
        controls,
    )?;
    validate_scvx_powered_landing_envelope_limits(limits)?;
    validate_positive(
        "SCvx powered-landing envelope quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )
}

fn validate_scvx_powered_landing_envelope_limits(
    limits: ScvxPoweredLandingEnvelopeLimits,
) -> Result<(), TrajoptError> {
    if !limits.minimum_mass.is_finite()
        || !limits.maximum_mass.is_finite()
        || limits.minimum_mass <= 0.0
        || limits.maximum_mass < limits.minimum_mass
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope mass bounds are invalid",
        });
    }
    if !limits.minimum_thrust.is_finite()
        || !limits.maximum_thrust.is_finite()
        || limits.minimum_thrust < 0.0
        || limits.maximum_thrust < limits.minimum_thrust
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope thrust bounds are invalid",
        });
    }
    if !limits.maximum_gimbal_angle_rad.is_finite()
        || limits.maximum_gimbal_angle_rad < 0.0
        || limits.maximum_gimbal_angle_rad > core::f64::consts::PI
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope gimbal angle bound is invalid",
        });
    }
    if !limits.maximum_tilt_angle_rad.is_finite()
        || limits.maximum_tilt_angle_rad < 0.0
        || limits.maximum_tilt_angle_rad > core::f64::consts::PI
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope tilt angle bound is invalid",
        });
    }
    if !limits.maximum_angular_rate_rad_s.is_finite() || limits.maximum_angular_rate_rad_s < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope angular-rate bound is invalid",
        });
    }
    if !limits.glideslope_angle_rad.is_finite()
        || limits.glideslope_angle_rad < 0.0
        || limits.glideslope_angle_rad >= core::f64::consts::FRAC_PI_2
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope glideslope angle is invalid",
        });
    }
    validate_array_finite(
        "SCvx powered-landing envelope gravity must be finite",
        &limits.gravity_eci,
    )?;
    if norm3(limits.gravity_eci) == 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope gravity must be nonzero",
        });
    }
    Ok(())
}

fn validate_scvx_powered_landing_linear_envelope_inputs(
    node_count: usize,
    limits: ScvxPoweredLandingEnvelopeLimits,
    cone_facets: usize,
) -> Result<(), TrajoptError> {
    if node_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linear envelope node count must be nonzero",
        });
    }
    validate_scvx_powered_landing_envelope_limits(limits)?;
    if cone_facets < 4 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linear envelope cone facets must be at least four",
        });
    }
    if limits.maximum_gimbal_angle_rad >= core::f64::consts::FRAC_PI_2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linear envelope gimbal angle must be less than 90 degrees",
        });
    }
    Ok(())
}

fn validate_scvx_powered_landing_linearized_envelope_inputs(
    reference_states: &[f64],
    limits: ScvxPoweredLandingEnvelopeLimits,
    settings: ScvxPoweredLandingLinearizedEnvelopeSettings,
) -> Result<(), TrajoptError> {
    if reference_states.is_empty()
        || !reference_states
            .len()
            .is_multiple_of(ScvxPoweredLandingStateLayout::DIMENSION)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linearized envelope reference states are malformed",
        });
    }
    validate_finite(
        "SCvx powered-landing linearized envelope reference states must be finite",
        reference_states,
    )?;
    let node_count = reference_states.len() / ScvxPoweredLandingStateLayout::DIMENSION;
    validate_scvx_powered_landing_linear_envelope_inputs(node_count, limits, settings.cone_facets)?;
    if settings.norm_facets < 6 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linearized envelope norm facets must be at least six",
        });
    }
    validate_positive(
        "SCvx powered-landing linearized envelope finite-difference step must be positive",
        settings.finite_difference_step,
    )?;
    validate_positive(
        "SCvx powered-landing linearized envelope quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )?;
    for reference_state in reference_states.chunks_exact(ScvxPoweredLandingStateLayout::DIMENSION) {
        validate_scvx_powered_landing_sample_quaternion(
            reference_state,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: settings.quaternion_norm_tolerance,
            },
        )?;
    }
    Ok(())
}

fn validate_scvx_powered_landing_conic_envelope_inputs(
    reference_states: &[f64],
    limits: ScvxPoweredLandingEnvelopeLimits,
    settings: ScvxPoweredLandingConicEnvelopeSettings,
) -> Result<(), TrajoptError> {
    if reference_states.is_empty()
        || !reference_states
            .len()
            .is_multiple_of(ScvxPoweredLandingStateLayout::DIMENSION)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing conic envelope reference states are malformed",
        });
    }
    validate_finite(
        "SCvx powered-landing conic envelope reference states must be finite",
        reference_states,
    )?;
    validate_scvx_powered_landing_envelope_limits(limits)?;
    if limits.maximum_gimbal_angle_rad >= core::f64::consts::FRAC_PI_2 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing conic envelope gimbal angle must be less than 90 degrees",
        });
    }
    validate_positive(
        "SCvx powered-landing conic envelope finite-difference step must be positive",
        settings.finite_difference_step,
    )?;
    validate_positive(
        "SCvx powered-landing conic envelope quaternion tolerance must be positive",
        settings.quaternion_norm_tolerance,
    )?;
    for reference_state in reference_states.chunks_exact(ScvxPoweredLandingStateLayout::DIMENSION) {
        validate_scvx_powered_landing_sample_quaternion(
            reference_state,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: settings.quaternion_norm_tolerance,
            },
        )?;
    }
    Ok(())
}

fn powered_landing_mass_constraint(
    node: usize,
    coefficient: f64,
    offset: f64,
) -> ScvxLinearStateConstraint {
    let mut coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    coefficients[ScvxPoweredLandingStateLayout::MASS] = coefficient;
    ScvxLinearStateConstraint {
        node,
        function: ScvxLinearStateFunction {
            coefficients,
            offset,
        },
    }
}

fn powered_landing_glideslope_constraint(
    node: usize,
    local_up: [f64; 3],
    lateral_direction: [f64; 3],
    glideslope_tangent: f64,
) -> ScvxLinearStateConstraint {
    let mut coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    for component in 0..3 {
        coefficients[ScvxPoweredLandingStateLayout::POSITION_ECI + component] =
            glideslope_tangent * lateral_direction[component] - local_up[component];
    }
    ScvxLinearStateConstraint {
        node,
        function: ScvxLinearStateFunction {
            coefficients,
            offset: 0.0,
        },
    }
}

fn powered_landing_glideslope_soc_constraint(
    node: usize,
    local_up: [f64; 3],
    lateral_axis_a: [f64; 3],
    lateral_axis_b: [f64; 3],
    glideslope_tangent: f64,
) -> ScvxSecondOrderConeStateConstraint {
    let mut scalar_coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    let mut lateral_a_coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    let mut lateral_b_coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    for component in 0..3 {
        let state_component = ScvxPoweredLandingStateLayout::POSITION_ECI + component;
        scalar_coefficients[state_component] = local_up[component];
        lateral_a_coefficients[state_component] = glideslope_tangent * lateral_axis_a[component];
        lateral_b_coefficients[state_component] = glideslope_tangent * lateral_axis_b[component];
    }
    ScvxSecondOrderConeStateConstraint {
        node,
        scalar: ScvxLinearStateFunction {
            coefficients: scalar_coefficients,
            offset: 0.0,
        },
        vector: vec![
            ScvxLinearStateFunction {
                coefficients: lateral_a_coefficients,
                offset: 0.0,
            },
            ScvxLinearStateFunction {
                coefficients: lateral_b_coefficients,
                offset: 0.0,
            },
        ],
    }
}

fn powered_landing_angular_rate_norm_facet_constraint(
    node: usize,
    direction: [f64; 3],
    maximum_angular_rate_rad_s: f64,
) -> ScvxLinearStateConstraint {
    let omega = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
    let mut coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    coefficients[omega] = direction[0];
    coefficients[omega + 1] = direction[1];
    coefficients[omega + 2] = direction[2];
    ScvxLinearStateConstraint {
        node,
        function: ScvxLinearStateFunction {
            coefficients,
            offset: -maximum_angular_rate_rad_s,
        },
    }
}

fn powered_landing_angular_rate_soc_constraint(
    node: usize,
    maximum_angular_rate_rad_s: f64,
) -> ScvxSecondOrderConeStateConstraint {
    let scalar = ScvxLinearStateFunction {
        coefficients: vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION],
        offset: maximum_angular_rate_rad_s,
    };
    let omega = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
    let mut vector = Vec::with_capacity(3);
    for component in 0..3 {
        let mut coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        coefficients[omega + component] = 1.0;
        vector.push(ScvxLinearStateFunction {
            coefficients,
            offset: 0.0,
        });
    }
    ScvxSecondOrderConeStateConstraint {
        node,
        scalar,
        vector,
    }
}

fn powered_landing_tilt_linearized_constraint(
    node: usize,
    reference_state: &[f64],
    local_up: [f64; 3],
    maximum_tilt_angle_rad: f64,
    finite_difference_step: f64,
) -> Result<ScvxLinearStateConstraint, TrajoptError> {
    let base =
        powered_landing_tilt_constraint_value(reference_state, local_up, maximum_tilt_angle_rad);
    if !base.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing tilt linearization output must be finite",
        });
    }

    let mut coefficients = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    for component in 0..4 {
        let state_component = quaternion + component;
        let mut plus_state = reference_state.to_vec();
        let mut minus_state = reference_state.to_vec();
        plus_state[state_component] += finite_difference_step;
        minus_state[state_component] -= finite_difference_step;
        let plus =
            powered_landing_tilt_constraint_value(&plus_state, local_up, maximum_tilt_angle_rad);
        let minus =
            powered_landing_tilt_constraint_value(&minus_state, local_up, maximum_tilt_angle_rad);
        if !plus.is_finite() || !minus.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "SCvx powered-landing tilt finite-difference output must be finite",
            });
        }
        coefficients[state_component] = (plus - minus) / (2.0 * finite_difference_step);
    }
    let gradient_dot_reference = coefficients
        .iter()
        .zip(reference_state)
        .map(|(coefficient, value)| coefficient * value)
        .sum::<f64>();

    Ok(ScvxLinearStateConstraint {
        node,
        function: ScvxLinearStateFunction {
            coefficients,
            offset: base - gradient_dot_reference,
        },
    })
}

fn powered_landing_tilt_constraint_value(
    state: &[f64],
    local_up: [f64; 3],
    maximum_tilt_angle_rad: f64,
) -> f64 {
    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let body_axis_eci = rotate_body_to_eci_wxyz(
        [
            state[quaternion],
            state[quaternion + 1],
            state[quaternion + 2],
            state[quaternion + 3],
        ],
        [1.0, 0.0, 0.0],
    );
    maximum_tilt_angle_rad.cos() - dot3(body_axis_eci, local_up)
}

fn powered_landing_axial_thrust_constraint(
    node: usize,
    coefficient: f64,
    upper_bound: f64,
) -> ScvxLinearControlConstraint {
    let mut coefficients = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
    coefficients[ScvxPoweredLandingControlLayout::THRUST_BODY] = coefficient;
    ScvxLinearControlConstraint {
        node,
        coefficients,
        upper_bound,
    }
}

fn powered_landing_thrust_norm_facet_constraint(
    node: usize,
    direction: [f64; 3],
    maximum_thrust: f64,
) -> ScvxLinearControlConstraint {
    ScvxLinearControlConstraint {
        node,
        coefficients: direction.to_vec(),
        upper_bound: maximum_thrust,
    }
}

fn powered_landing_thrust_norm_soc_constraint(
    node: usize,
    maximum_thrust: f64,
) -> ScvxSecondOrderConeControlConstraint {
    let scalar = ScvxLinearControlFunction {
        coefficients: vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION],
        offset: maximum_thrust,
    };
    let thrust = ScvxPoweredLandingControlLayout::THRUST_BODY;
    let mut vector = Vec::with_capacity(3);
    for component in 0..3 {
        let mut coefficients = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
        coefficients[thrust + component] = 1.0;
        vector.push(ScvxLinearControlFunction {
            coefficients,
            offset: 0.0,
        });
    }
    ScvxSecondOrderConeControlConstraint {
        node,
        scalar,
        vector,
    }
}

fn powered_landing_gimbal_constraint(
    node: usize,
    lateral_y: f64,
    lateral_z: f64,
    gimbal_tangent: f64,
) -> ScvxLinearControlConstraint {
    let thrust = ScvxPoweredLandingControlLayout::THRUST_BODY;
    let mut coefficients = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
    coefficients[thrust] = -gimbal_tangent;
    coefficients[thrust + 1] = lateral_y;
    coefficients[thrust + 2] = lateral_z;
    ScvxLinearControlConstraint {
        node,
        coefficients,
        upper_bound: 0.0,
    }
}

fn powered_landing_gimbal_soc_constraint(
    node: usize,
    gimbal_tangent: f64,
) -> ScvxSecondOrderConeControlConstraint {
    let thrust = ScvxPoweredLandingControlLayout::THRUST_BODY;
    let mut scalar_coefficients = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
    scalar_coefficients[thrust] = gimbal_tangent;
    let mut lateral_y_coefficients = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
    lateral_y_coefficients[thrust + 1] = 1.0;
    let mut lateral_z_coefficients = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
    lateral_z_coefficients[thrust + 2] = 1.0;
    ScvxSecondOrderConeControlConstraint {
        node,
        scalar: ScvxLinearControlFunction {
            coefficients: scalar_coefficients,
            offset: 0.0,
        },
        vector: vec![
            ScvxLinearControlFunction {
                coefficients: lateral_y_coefficients,
                offset: 0.0,
            },
            ScvxLinearControlFunction {
                coefficients: lateral_z_coefficients,
                offset: 0.0,
            },
        ],
    }
}

fn powered_landing_lateral_basis(local_up: [f64; 3]) -> Result<([f64; 3], [f64; 3]), TrajoptError> {
    let reference_axis = if local_up[0].abs() < 0.9 {
        [1.0, 0.0, 0.0]
    } else {
        [0.0, 1.0, 0.0]
    };
    let lateral_axis_a =
        unit3(cross3(local_up, reference_axis)).ok_or(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linear envelope lateral basis is degenerate",
        })?;
    let lateral_axis_b =
        unit3(cross3(local_up, lateral_axis_a)).ok_or(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing linear envelope lateral basis is degenerate",
        })?;
    Ok((lateral_axis_a, lateral_axis_b))
}

fn powered_landing_norm_facet_direction(
    facet: usize,
    facet_count: usize,
) -> Result<[f64; 3], TrajoptError> {
    if facet >= facet_count || facet_count < 6 {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing norm facet index is out of range",
        });
    }
    const AXES: [[f64; 3]; 6] = [
        [1.0, 0.0, 0.0],
        [-1.0, 0.0, 0.0],
        [0.0, 1.0, 0.0],
        [0.0, -1.0, 0.0],
        [0.0, 0.0, 1.0],
        [0.0, 0.0, -1.0],
    ];
    if facet < AXES.len() {
        return Ok(AXES[facet]);
    }

    let extra_count = facet_count - AXES.len();
    let extra_index = facet - AXES.len();
    let z = 1.0 - 2.0 * (extra_index as f64 + 0.5) / extra_count as f64;
    let radius = (1.0 - z * z).max(0.0).sqrt();
    let angle = 2.0 * core::f64::consts::PI * extra_index as f64 / ((1.0 + 5.0_f64.sqrt()) * 0.5);
    unit3([radius * angle.cos(), radius * angle.sin(), z]).ok_or(TrajoptError::InvalidPayload {
        reason: "SCvx powered-landing norm facet direction is degenerate",
    })
}

fn validate_scvx_powered_landing_sample_quaternion(
    state: &[f64],
    settings: ScvxRigidBodyDynamicsSettings,
) -> Result<(), TrajoptError> {
    let quaternion = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
    let norm = (state[quaternion] * state[quaternion]
        + state[quaternion + 1] * state[quaternion + 1]
        + state[quaternion + 2] * state[quaternion + 2]
        + state[quaternion + 3] * state[quaternion + 3])
        .sqrt();
    if (norm - 1.0).abs() > settings.quaternion_norm_tolerance {
        return Err(TrajoptError::InvalidPayload {
            reason: "SCvx powered-landing envelope quaternion must be unit length",
        });
    }
    Ok(())
}

fn powered_landing_state_as_rigid_body_state(
    state: &[f64],
) -> [f64; ScvxRigidBodyStateLayout::DIMENSION] {
    let mut rigid_state = [0.0; ScvxRigidBodyStateLayout::DIMENSION];
    rigid_state[ScvxRigidBodyStateLayout::POSITION_ECI..ScvxRigidBodyStateLayout::POSITION_ECI + 3]
        .copy_from_slice(
            &state[ScvxPoweredLandingStateLayout::POSITION_ECI
                ..ScvxPoweredLandingStateLayout::POSITION_ECI + 3],
        );
    rigid_state[ScvxRigidBodyStateLayout::VELOCITY_ECI..ScvxRigidBodyStateLayout::VELOCITY_ECI + 3]
        .copy_from_slice(
            &state[ScvxPoweredLandingStateLayout::VELOCITY_ECI
                ..ScvxPoweredLandingStateLayout::VELOCITY_ECI + 3],
        );
    rigid_state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ
        ..ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 4]
        .copy_from_slice(
            &state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ
                ..ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 4],
        );
    rigid_state[ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY
        ..ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY + 3]
        .copy_from_slice(
            &state[ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY
                ..ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY + 3],
        );
    rigid_state
}

fn quaternion_rate_body_to_eci_wxyz(q: [f64; 4], omega_body: [f64; 3]) -> [f64; 4] {
    let [w, x, y, z] = q;
    let [p, q_rate, r] = omega_body;
    [
        -0.5 * (x * p + y * q_rate + z * r),
        0.5 * (w * p + y * r - z * q_rate),
        0.5 * (w * q_rate + z * p - x * r),
        0.5 * (w * r + x * q_rate - y * p),
    ]
}

fn rotate_body_to_eci_wxyz(q: [f64; 4], vector_body: [f64; 3]) -> [f64; 3] {
    let q_vector = [q[1], q[2], q[3]];
    let t = scale3(cross3(q_vector, vector_body), 2.0);
    add3(add3(vector_body, scale3(t, q[0])), cross3(q_vector, t))
}

fn mat3_vec_mul(matrix: [f64; 9], vector: [f64; 3]) -> [f64; 3] {
    [
        matrix[0] * vector[0] + matrix[1] * vector[1] + matrix[2] * vector[2],
        matrix[3] * vector[0] + matrix[4] * vector[1] + matrix[5] * vector[2],
        matrix[6] * vector[0] + matrix[7] * vector[1] + matrix[8] * vector[2],
    ]
}

fn add3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [left[0] + right[0], left[1] + right[1], left[2] + right[2]]
}

fn scale3(vector: [f64; 3], scale: f64) -> [f64; 3] {
    [vector[0] * scale, vector[1] * scale, vector[2] * scale]
}

fn norm3(vector: [f64; 3]) -> f64 {
    (vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]).sqrt()
}

fn dot3(left: [f64; 3], right: [f64; 3]) -> f64 {
    left[0] * right[0] + left[1] * right[1] + left[2] * right[2]
}

fn unit3(vector: [f64; 3]) -> Option<[f64; 3]> {
    let norm = norm3(vector);
    if norm.is_finite() && norm > 0.0 {
        Some(scale3(vector, 1.0 / norm))
    } else {
        None
    }
}

fn positive_part(value: f64) -> f64 {
    value.max(0.0)
}

fn cross3(left: [f64; 3], right: [f64; 3]) -> [f64; 3] {
    [
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    ]
}

fn invert_mat3(matrix: [f64; 9]) -> Option<[f64; 9]> {
    let a = matrix[0];
    let b = matrix[1];
    let c = matrix[2];
    let d = matrix[3];
    let e = matrix[4];
    let f = matrix[5];
    let g = matrix[6];
    let h = matrix[7];
    let i = matrix[8];
    let c00 = e * i - f * h;
    let c01 = -(d * i - f * g);
    let c02 = d * h - e * g;
    let c10 = -(b * i - c * h);
    let c11 = a * i - c * g;
    let c12 = -(a * h - b * g);
    let c20 = b * f - c * e;
    let c21 = -(a * f - c * d);
    let c22 = a * e - b * d;
    let determinant = a * c00 + b * c01 + c * c02;
    if !determinant.is_finite() || determinant.abs() <= 1.0e-15 {
        return None;
    }
    let inv_det = 1.0 / determinant;
    Some([
        c00 * inv_det,
        c10 * inv_det,
        c20 * inv_det,
        c01 * inv_det,
        c11 * inv_det,
        c21 * inv_det,
        c02 * inv_det,
        c12 * inv_det,
        c22 * inv_det,
    ])
}

fn validate_len(reason: &'static str, values: &[f64], expected: usize) -> Result<(), TrajoptError> {
    if values.len() == expected {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

fn validate_finite(reason: &'static str, values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

fn validate_array_finite<const N: usize>(
    reason: &'static str,
    values: &[f64; N],
) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

fn validate_positive(reason: &'static str, value: f64) -> Result<(), TrajoptError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
// Exact comparisons verify copied boundaries, sparse coefficients, and clamped values.
#[allow(clippy::float_cmp)]
mod tests {
    use super::*;
    use serde::Deserialize;

    const SZMUK_ACIKMESE_6DOF_FREE_FINAL_TIME: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/trajopt/szmuk-acikmese-6dof-free-final-time-v1.toml"
    ));

    #[derive(Debug, Deserialize)]
    struct SzmukBenchmarkFixture {
        dataset_id: String,
        vehicle: SzmukVehicle,
        environment: SzmukEnvironment,
        algorithm: SzmukAlgorithm,
        boundary_conditions: SzmukBoundaryConditions,
    }

    #[derive(Clone, Copy, Debug, Deserialize)]
    struct SzmukVehicle {
        wet_mass: f64,
        dry_mass: f64,
        minimum_thrust: f64,
        maximum_thrust: f64,
        maximum_gimbal_angle_deg: f64,
        maximum_tilt_angle_deg: f64,
        maximum_angular_rate_deg_per_time_unit: f64,
        inertia_body: [f64; 9],
        thrust_application_point_body: [f64; 3],
    }

    #[derive(Clone, Copy, Debug, Deserialize)]
    struct SzmukEnvironment {
        gravity: [f64; 3],
        glideslope_angle_deg: f64,
    }

    #[derive(Debug, Deserialize)]
    struct SzmukAlgorithm {
        virtual_control_weight: f64,
        state_control_trust_region_weight: f64,
        time_trust_region_weight: f64,
        virtual_control_tolerance: f64,
        trust_region_tolerance: f64,
        maximum_iterations: usize,
        discretization_points: usize,
        initial_time_guesses: Vec<f64>,
        published_in_plane_convergence_iteration: usize,
        published_out_of_plane_convergence_iteration: usize,
        published_time_of_flight_agreement: f64,
    }

    #[derive(Clone, Copy, Debug, Deserialize)]
    struct SzmukBoundaryConditions {
        initial_position: [f64; 3],
        in_plane_initial_velocity: [f64; 3],
        final_velocity: [f64; 3],
        initial_angular_velocity: [f64; 3],
        final_quaternion_body_to_inertial_wxyz: [f64; 4],
    }

    #[test]
    fn scvx_affine_integrator_converges_without_virtual_control() -> Result<(), TrajoptError> {
        let problem = scalar_integrator_problem(1.0);
        let solution = solve_scvx_discrete_affine(&problem, fast_settings())?;

        assert!(solution.converged);
        assert_eq!(
            solution.termination_status,
            ScvxTerminationStatus::Converged
        );
        assert_eq!(solution.solver_status, SolverStatus::Solved);
        assert!(solution.terminal_error_norm < 1.0e-6);
        assert!(solution.max_virtual_control_norm < 1.0e-6);
        assert!(
            solution
                .iterations
                .iter()
                .any(|iteration| iteration.accepted)
        );
        assert!(
            solution
                .iterations
                .iter()
                .all(|iteration| iteration.rho.is_finite())
        );
        assert!((solution.states[problem.horizon_steps] - 1.0).abs() < 1.0e-6);

        Ok(())
    }

    #[test]
    fn scvx_affine_integrator_is_repeat_deterministic() -> Result<(), TrajoptError> {
        let problem = scalar_integrator_problem(1.0);
        let first = solve_scvx_discrete_affine(&problem, fast_settings())?;
        let second = solve_scvx_discrete_affine(&problem, fast_settings())?;

        assert_eq!(first.converged, second.converged);
        assert_eq!(first.termination_status, second.termination_status);
        assert_eq!(first.states.len(), second.states.len());
        for (left, right) in first.states.iter().zip(&second.states) {
            assert_eq!(left.to_bits(), right.to_bits());
        }
        for (left, right) in first.controls.iter().zip(&second.controls) {
            assert_eq!(left.to_bits(), right.to_bits());
        }
        for (left, right) in first.virtual_controls.iter().zip(&second.virtual_controls) {
            assert_eq!(left.to_bits(), right.to_bits());
        }
        assert_eq!(first.iterations, second.iterations);

        Ok(())
    }

    #[test]
    fn scvx_time_of_flight_search_shrinks_bracket_and_finds_interior_time()
    -> Result<(), TrajoptError> {
        let settings = time_search_settings();
        let report = solve_scvx_time_of_flight_search(settings, |time| {
            scalar_integrator_time_problem(time)
        })?;
        let midpoint_time =
            0.5 * (settings.lower_time_of_flight_s + settings.upper_time_of_flight_s);
        let midpoint_objective = scvx_time_search_objective_at(midpoint_time, settings)?;

        assert_eq!(report.iterations, settings.max_iterations);
        assert_eq!(report.candidate_count, settings.max_iterations + 2);
        assert_eq!(report.candidates.len(), report.candidate_count);
        assert!(
            report.final_upper_time_of_flight_s - report.final_lower_time_of_flight_s
                < settings.upper_time_of_flight_s - settings.lower_time_of_flight_s
        );
        assert!(report.best_time_of_flight_s > settings.lower_time_of_flight_s);
        assert!(report.best_time_of_flight_s < settings.upper_time_of_flight_s);
        assert!(report.objective < midpoint_objective);
        assert!(report.solution.converged);
        assert!(report.solution.terminal_error_norm < 1.0e-6);
        assert!(report.solution.max_virtual_control_norm < 1.0e-6);
        assert!(report.candidates.iter().all(|candidate| {
            candidate.termination_status == ScvxTerminationStatus::Converged
                && candidate.terminal_error_norm < 1.0e-6
                && candidate.max_virtual_control_norm < 1.0e-6
        }));

        Ok(())
    }

    #[test]
    fn scvx_continuous_foh_time_of_flight_search_shrinks_bracket_and_reports_candidates()
    -> Result<(), TrajoptError> {
        let settings = continuous_foh_time_search_settings();
        let continuous_settings = continuous_foh_time_candidate_settings();
        let report = solve_scvx_continuous_foh_time_of_flight_search(settings, |time| {
            solve_scalar_integrator_continuous_foh_time_candidate(time, continuous_settings)
        })?;
        let diagnostic = evaluate_scvx_continuous_foh_time_of_flight_candidate(
            settings,
            report.best_time_of_flight_s,
            |time| solve_scalar_integrator_continuous_foh_time_candidate(time, continuous_settings),
        )?;

        assert_eq!(report.iterations, settings.max_iterations);
        assert_eq!(report.candidate_count, settings.max_iterations + 2);
        assert_eq!(report.candidates.len(), report.candidate_count);
        assert!(
            report.final_upper_time_of_flight_s - report.final_lower_time_of_flight_s
                < settings.upper_time_of_flight_s - settings.lower_time_of_flight_s
        );
        assert!(report.best_time_of_flight_s > settings.lower_time_of_flight_s);
        assert!(report.best_time_of_flight_s < settings.upper_time_of_flight_s);
        assert!(report.solution.converged);
        assert!(report.solution.solution.converged);
        assert!(report.solution.solution.terminal_error_norm < 1.0e-6);
        assert!(report.solution.solution.max_virtual_control_norm < 1.0e-6);
        assert!(report.candidates.iter().all(|candidate| {
            candidate.converged
                && candidate.affine_converged
                && candidate.termination_status == ScvxTerminationStatus::Converged
                && candidate.terminal_error_norm < 1.0e-6
                && candidate.max_virtual_control_norm < 1.0e-6
                && candidate.outer_iteration_count >= 1
                && candidate.affine_iteration_count >= 1
        }));
        assert!(diagnostic.converged);
        assert_eq!(
            diagnostic.time_of_flight_s.to_bits(),
            report.best_time_of_flight_s.to_bits()
        );
        Ok(())
    }

    #[test]
    fn scvx_continuous_foh_time_of_flight_sweep_reports_failed_windows() -> Result<(), TrajoptError>
    {
        let settings = continuous_foh_time_sweep_settings();
        let continuous_settings = continuous_foh_time_candidate_settings();
        let report = solve_scvx_continuous_foh_time_of_flight_sweep(settings.clone(), |time| {
            if time > 1.1 {
                Err(TrajoptError::InvalidPayload {
                    reason: "test SCvx continuous sweep window failed",
                })
            } else {
                solve_scalar_integrator_continuous_foh_time_candidate(time, continuous_settings)
            }
        })?;

        assert_eq!(report.window_count, 2);
        assert_eq!(report.windows.len(), 2);
        assert_eq!(report.converged_window_count, 1);
        assert_eq!(report.best_window_index, Some(0));
        assert!(report.best_time_of_flight_s.is_some());
        assert!(report.best_objective.is_some());
        assert!(report.windows[0].converged);
        assert!(report.windows[0].search.is_some());
        assert!(report.windows[0].failure_reason.is_none());
        assert!(report.windows[0].failure_candidate.is_none());
        assert!(!report.windows[1].converged);
        assert!(report.windows[1].search.is_none());
        assert_eq!(
            report.windows[1].failure_reason,
            Some("test SCvx continuous sweep window failed")
        );
        assert!(report.windows[1].failure_candidate.is_none());

        let bad_settings = ScvxContinuousFohTimeOfFlightSweepSettings {
            center_times_of_flight_s: vec![1.0, 1.0],
            ..settings
        };
        assert!(matches!(
            solve_scvx_continuous_foh_time_of_flight_sweep(bad_settings, |time| {
                solve_scalar_integrator_continuous_foh_time_candidate(time, continuous_settings)
            }),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        Ok(())
    }

    #[test]
    fn scvx_time_of_flight_search_is_repeat_deterministic() -> Result<(), TrajoptError> {
        let settings = time_search_settings();
        let first = solve_scvx_time_of_flight_search(settings, |time| {
            scalar_integrator_time_problem(time)
        })?;
        let second = solve_scvx_time_of_flight_search(settings, |time| {
            scalar_integrator_time_problem(time)
        })?;

        assert_eq!(
            first.best_time_of_flight_s.to_bits(),
            second.best_time_of_flight_s.to_bits()
        );
        assert_eq!(first.objective.to_bits(), second.objective.to_bits());
        assert_eq!(
            first.final_lower_time_of_flight_s.to_bits(),
            second.final_lower_time_of_flight_s.to_bits()
        );
        assert_eq!(
            first.final_upper_time_of_flight_s.to_bits(),
            second.final_upper_time_of_flight_s.to_bits()
        );
        assert_eq!(first.solution.states, second.solution.states);
        assert_eq!(first.solution.controls, second.solution.controls);
        assert_eq!(
            first.solution.virtual_controls,
            second.solution.virtual_controls
        );
        assert_eq!(first.candidates, second.candidates);

        Ok(())
    }

    #[test]
    fn scvx_time_of_flight_search_rejects_bad_settings() {
        let settings = ScvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s: 1.0,
            upper_time_of_flight_s: 1.0,
            ..time_search_settings()
        };
        assert!(matches!(
            solve_scvx_time_of_flight_search(settings, scalar_integrator_time_problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_scvx_time_of_flight_candidate(settings, 0.5, scalar_integrator_time_problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let settings = ScvxTimeOfFlightSearchSettings {
            time_penalty_weight: -1.0,
            ..time_search_settings()
        };
        assert!(matches!(
            solve_scvx_time_of_flight_search(settings, scalar_integrator_time_problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_scvx_time_of_flight_candidate(
                time_search_settings(),
                f64::NAN,
                scalar_integrator_time_problem,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let continuous_settings = ScvxContinuousFohTimeOfFlightSearchSettings {
            lower_time_of_flight_s: 1.0,
            upper_time_of_flight_s: 1.0,
            ..continuous_foh_time_search_settings()
        };
        assert!(matches!(
            solve_scvx_continuous_foh_time_of_flight_search(continuous_settings, |time| {
                solve_scalar_integrator_continuous_foh_time_candidate(
                    time,
                    continuous_foh_time_candidate_settings(),
                )
            }),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            evaluate_scvx_continuous_foh_time_of_flight_candidate(
                continuous_foh_time_search_settings(),
                f64::NAN,
                |time| solve_scalar_integrator_continuous_foh_time_candidate(
                    time,
                    continuous_foh_time_candidate_settings(),
                ),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_time_of_flight_sweep_reports_failed_windows() -> Result<(), TrajoptError> {
        let settings = ScvxTimeOfFlightSweepSettings {
            center_times_of_flight_s: vec![0.9, 1.2],
            half_width_s: 0.05,
            tolerance_s: 1.0e-3,
            max_iterations: 2,
            time_penalty_weight: 1.0,
            scvx_settings: time_search_settings().scvx_settings,
        };

        let report = solve_scvx_time_of_flight_sweep(settings.clone(), |time| {
            if time > 1.1 {
                Err(TrajoptError::InvalidPayload {
                    reason: "test SCvx sweep window failed",
                })
            } else {
                scalar_integrator_time_problem(time)
            }
        })?;

        assert_eq!(report.window_count, 2);
        assert_eq!(report.windows.len(), 2);
        assert_eq!(report.converged_window_count, 1);
        assert_eq!(report.best_window_index, Some(0));
        assert!(report.best_time_of_flight_s.is_some());
        assert!(report.best_objective.is_some());
        assert!(report.windows[0].converged);
        assert!(report.windows[0].search.is_some());
        assert!(report.windows[0].failure_reason.is_none());
        assert!(report.windows[0].failure_candidate.is_none());
        assert!(!report.windows[1].converged);
        assert!(report.windows[1].search.is_none());
        assert_eq!(
            report.windows[1].failure_reason,
            Some("test SCvx sweep window failed")
        );
        assert!(report.windows[1].failure_candidate.is_none());

        let bad_settings = ScvxTimeOfFlightSweepSettings {
            center_times_of_flight_s: vec![1.0, 1.0],
            ..settings
        };
        assert!(matches!(
            solve_scvx_time_of_flight_sweep(bad_settings, scalar_integrator_time_problem),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        Ok(())
    }

    #[test]
    fn scvx_state_triggered_constraint_is_inactive_above_and_active_below_trigger()
    -> Result<(), TrajoptError> {
        let inactive = state_triggered_problem([1.0, 2.0]);
        let inactive_solution = solve_scvx_discrete_affine(&inactive, triggered_settings())?;
        assert!(inactive_solution.converged);
        let inactive_terminal = node_state(&inactive, &inactive_solution.states, 1)?;
        assert!(inactive_terminal[1] > 1.5);
        assert!(inactive_solution.max_state_triggered_violation < 1.0e-8);

        let active = state_triggered_problem([-1.0, 2.0]);
        let active_solution = solve_scvx_discrete_affine(&active, triggered_settings())?;
        assert!(!active_solution.converged);
        assert_eq!(
            active_solution.termination_status,
            ScvxTerminationStatus::MaxIterations
        );
        let active_terminal = node_state(&active, &active_solution.states, 1)?;
        assert!(active_terminal[0] < -0.5);
        assert!(active_terminal[1] <= 0.5 + 1.0e-7);
        assert!(active_solution.max_state_triggered_violation < 1.0e-8);
        assert!(active_solution.max_virtual_control_norm < 1.0e-6);

        Ok(())
    }

    #[test]
    fn scvx_linear_state_constraint_is_enforced_at_node() -> Result<(), TrajoptError> {
        let mut problem = scalar_integrator_problem(1.0);
        problem.linear_state_constraints = vec![ScvxLinearStateConstraint {
            node: 2,
            function: ScvxLinearStateFunction {
                coefficients: vec![1.0],
                offset: -0.2,
            },
        }];

        let solution = solve_scvx_discrete_affine(&problem, fast_settings())?;
        let constrained_state = node_state(&problem, &solution.states, 2)?[0];

        assert!(solution.converged);
        assert!(solution.max_linear_state_violation <= 1.0e-8);
        assert!(constrained_state <= 0.2 + 1.0e-8);
        assert!(
            solution
                .iterations
                .iter()
                .all(|iteration| iteration.max_linear_state_violation.is_finite())
        );
        Ok(())
    }

    #[test]
    fn scvx_linear_control_constraint_is_enforced_at_node() -> Result<(), TrajoptError> {
        let mut problem = scalar_integrator_problem(1.0);
        problem.linear_control_constraints = vec![ScvxLinearControlConstraint {
            node: 0,
            coefficients: vec![1.0],
            upper_bound: 0.05,
        }];

        let solution = solve_scvx_discrete_affine(&problem, fast_settings())?;

        assert!(solution.converged);
        assert!(solution.max_linear_control_violation <= 1.0e-8);
        assert!(solution.controls[0] <= 0.05 + 1.0e-8);
        assert!(
            solution
                .iterations
                .iter()
                .all(|iteration| iteration.max_linear_control_violation.is_finite())
        );
        Ok(())
    }

    #[test]
    fn scvx_second_order_cone_constraints_are_enforced_at_node() -> Result<(), TrajoptError> {
        let mut problem = ScvxDiscreteAffineProblem {
            state_dimension: 2,
            control_dimension: 2,
            horizon_steps: 1,
            initial_state: vec![0.0, 0.0],
            target: ScvxTerminalStateTarget {
                state: vec![0.2, 0.0],
                weights: None,
            },
            dynamics: vec![ScvxAffineDynamicsSegment {
                state_transition: vec![1.0, 0.0, 0.0, 1.0],
                control_left: vec![0.5, 0.0, 0.0, 0.5],
                control_right: vec![0.5, 0.0, 0.0, 0.5],
                affine: vec![0.0, 0.0],
            }],
            initial_states: vec![0.0, 0.0, 0.2, 0.0],
            initial_controls: vec![0.2, 0.0, 0.2, 0.0],
            control_lower: vec![-1.0, -1.0],
            control_upper: vec![1.0, 1.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: vec![ScvxSecondOrderConeStateConstraint {
                node: 1,
                scalar: ScvxLinearStateFunction {
                    coefficients: vec![0.0, 0.0],
                    offset: 0.2,
                },
                vector: vec![
                    ScvxLinearStateFunction {
                        coefficients: vec![1.0, 0.0],
                        offset: 0.0,
                    },
                    ScvxLinearStateFunction {
                        coefficients: vec![0.0, 1.0],
                        offset: 0.0,
                    },
                ],
            }],
            second_order_control_constraints: Vec::new(),
        };
        for node in 0..=problem.horizon_steps {
            problem
                .second_order_control_constraints
                .push(ScvxSecondOrderConeControlConstraint {
                    node,
                    scalar: ScvxLinearControlFunction {
                        coefficients: vec![0.0, 0.0],
                        offset: 0.2,
                    },
                    vector: vec![
                        ScvxLinearControlFunction {
                            coefficients: vec![1.0, 0.0],
                            offset: 0.0,
                        },
                        ScvxLinearControlFunction {
                            coefficients: vec![0.0, 1.0],
                            offset: 0.0,
                        },
                    ],
                });
        }

        let solution = solve_scvx_discrete_affine(&problem, fast_settings())?;
        let terminal = node_state(&problem, &solution.states, 1)?;
        let control0 = node_control(&problem, &solution.controls, 0)?;
        let control1 = node_control(&problem, &solution.controls, 1)?;

        assert_eq!(solution.solver_status, SolverStatus::Solved);
        assert!(solution.max_second_order_cone_violation <= 1.0e-7);
        assert!((terminal[0] * terminal[0] + terminal[1] * terminal[1]).sqrt() <= 0.2 + 1.0e-8);
        assert!((control0[0] * control0[0] + control0[1] * control0[1]).sqrt() <= 0.2 + 1.0e-8);
        assert!((control1[0] * control1[0] + control1[1] * control1[1]).sqrt() <= 0.2 + 1.0e-8);
        assert!(
            solution
                .iterations
                .iter()
                .all(|iteration| iteration.max_second_order_cone_violation.is_finite())
        );
        Ok(())
    }

    #[test]
    fn scvx_terminal_weights_leave_components_untargeted() -> Result<(), TrajoptError> {
        let problem = weighted_terminal_problem();
        let solution = solve_scvx_discrete_affine(&problem, fast_settings())?;
        let terminal = node_state(&problem, &solution.states, problem.horizon_steps)?;

        assert!(solution.converged);
        assert!(solution.terminal_error_norm < 1.0e-6);
        assert!(
            (terminal[0] - problem.target.state[0]).abs() > 1.0,
            "zero-weight component should not be driven to its target"
        );
        assert!((terminal[1] - problem.target.state[1]).abs() < 1.0e-6);

        Ok(())
    }

    #[test]
    fn scvx_rejects_malformed_inputs() {
        let mut problem = scalar_integrator_problem(1.0);
        problem.horizon_steps = 0;
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = scalar_integrator_problem(1.0);
        problem.control_lower[0] = 1.0;
        problem.control_upper[0] = -1.0;
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut settings = fast_settings();
        settings.max_iterations = 0;
        assert!(matches!(
            solve_scvx_discrete_affine(&scalar_integrator_problem(1.0), settings),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = scalar_integrator_problem(1.0);
        problem.target.weights = Some(vec![-1.0]);
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = scalar_integrator_problem(1.0);
        problem.linear_state_constraints = vec![ScvxLinearStateConstraint {
            node: 0,
            function: ScvxLinearStateFunction {
                coefficients: vec![1.0, 0.0],
                offset: 0.0,
            },
        }];
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = scalar_integrator_problem(1.0);
        problem.linear_control_constraints = vec![ScvxLinearControlConstraint {
            node: 0,
            coefficients: vec![1.0, 0.0],
            upper_bound: 0.0,
        }];
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = scalar_integrator_problem(1.0);
        problem.second_order_state_constraints = vec![ScvxSecondOrderConeStateConstraint {
            node: 0,
            scalar: ScvxLinearStateFunction {
                coefficients: vec![0.0],
                offset: 1.0,
            },
            vector: Vec::new(),
        }];
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut problem = scalar_integrator_problem(1.0);
        problem.second_order_control_constraints = vec![ScvxSecondOrderConeControlConstraint {
            node: 0,
            scalar: ScvxLinearControlFunction {
                coefficients: vec![0.0],
                offset: 1.0,
            },
            vector: vec![ScvxLinearControlFunction {
                coefficients: vec![1.0, 0.0],
                offset: 0.0,
            }],
        }];
        assert!(matches!(
            solve_scvx_discrete_affine(&problem, fast_settings()),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_forward_euler_linearization_matches_affine_dynamics() -> Result<(), TrajoptError> {
        let segments = linearize_scvx_forward_euler_foh(
            1,
            1,
            &[0.0, 0.25],
            &[2.0, 3.5],
            &[1.0, 3.0],
            |_time_s, state, control, output| {
                output[0] = -2.0 * state[0] + 3.0 * control[0] + 4.0;
                Ok(())
            },
            ScvxForwardEulerLinearizationSettings::default(),
        )?;

        assert_eq!(segments.len(), 1);
        let segment = &segments[0];
        assert!((segment.state_transition[0] - 0.5).abs() < 1.0e-8);
        assert!((segment.control_left[0] - 0.375).abs() < 1.0e-8);
        assert!((segment.control_right[0] - 0.375).abs() < 1.0e-8);
        assert!((segment.affine[0] - 1.0).abs() < 1.0e-8);

        Ok(())
    }

    #[test]
    fn scvx_forward_euler_foh_outer_loop_refreshes_reference_to_convergence()
    -> Result<(), TrajoptError> {
        let definition = ScvxForwardEulerFohProblemDefinition {
            state_dimension: 1,
            control_dimension: 1,
            times_s: vec![0.0, 0.5, 1.0],
            initial_state: vec![0.0],
            target: ScvxTerminalStateTarget {
                state: vec![1.0],
                weights: None,
            },
            initial_states: vec![0.0, 0.0, 0.0],
            initial_controls: vec![0.0, 0.0, 0.0],
            control_lower: vec![-5.0],
            control_upper: vec![5.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        };
        let mut dynamics_call_count = 0usize;

        let report = solve_scvx_forward_euler_foh(
            &definition,
            |_time_s, _state, control, output| {
                dynamics_call_count += 1;
                output[0] = control[0];
                Ok(())
            },
            ScvxContinuousFohSettings {
                max_outer_iterations: 6,
                reference_update_tolerance: 1.0e-3,
                scvx_settings: ScvxSettings {
                    max_iterations: 4,
                    initial_trust_region_radius: 5.0,
                    max_trust_region_radius: 10.0,
                    terminal_tolerance: 1.0e-6,
                    virtual_control_tolerance: 1.0e-6,
                    terminal_weight: 1.0e8,
                    virtual_control_weight: 1.0e10,
                    ..ScvxSettings::default()
                },
                linearization_settings: ScvxForwardEulerLinearizationSettings::default(),
            },
        )?;

        assert!(
            report.converged,
            "powered-landing FOH solve did not converge: terminal={}, virtual={}, outer={:?}",
            report.solution.terminal_error_norm,
            report.solution.max_virtual_control_norm,
            report.outer_iterations
        );
        assert!(report.solution.converged);
        assert!(report.outer_iterations.len() >= 2);
        assert!(dynamics_call_count > definition.times_s.len() - 1);
        assert!(report.solution.terminal_error_norm < 1.0e-6);
        assert!(
            report
                .outer_iterations
                .last()
                .expect("outer iteration")
                .max_state_update_norm
                <= 1.0e-3
        );
        assert!(
            report
                .outer_iterations
                .last()
                .expect("outer iteration")
                .max_control_update_norm
                <= 1.0e-3
        );
        Ok(())
    }

    #[test]
    fn scvx_forward_euler_foh_outer_loop_rejects_bad_settings() {
        let definition = ScvxForwardEulerFohProblemDefinition {
            state_dimension: 1,
            control_dimension: 1,
            times_s: vec![0.0, 1.0],
            initial_state: vec![0.0],
            target: ScvxTerminalStateTarget {
                state: vec![1.0],
                weights: None,
            },
            initial_states: vec![0.0, 0.0],
            initial_controls: vec![0.0, 0.0],
            control_lower: vec![-5.0],
            control_upper: vec![5.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        };
        assert!(matches!(
            solve_scvx_forward_euler_foh(
                &definition,
                |_time_s, _state, control, output| {
                    output[0] = control[0];
                    Ok(())
                },
                ScvxContinuousFohSettings {
                    max_outer_iterations: 0,
                    ..ScvxContinuousFohSettings::default()
                },
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_forward_euler_linearization_rejects_bad_inputs() {
        assert!(matches!(
            linearize_scvx_forward_euler_foh(
                1,
                1,
                &[0.0, 0.0],
                &[0.0, 0.0],
                &[0.0, 0.0],
                |_time_s, _state, _control, output| {
                    output[0] = 0.0;
                    Ok(())
                },
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        assert!(matches!(
            linearize_scvx_forward_euler_foh(
                1,
                1,
                &[0.0, 1.0],
                &[0.0, 0.0],
                &[0.0, 0.0],
                |_time_s, _state, _control, output| {
                    output[0] = f64::NAN;
                    Ok(())
                },
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_rigid_body_dynamics_matches_quaternion_and_euler_equations() -> Result<(), TrajoptError>
    {
        let state = rigid_body_state([1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [1.0, 2.0, 3.0]);
        let control = vec![10.0, 20.0, 30.0, 1.0, 2.0, 3.0];
        let parameters = ScvxRigidBodyDynamicsParameters {
            mass_kg: 10.0,
            inertia_body_kg_m2: [2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0],
            inertia_rate_body_kg_m2_s: [0.1, 0.0, 0.0, 0.0, 0.2, 0.0, 0.0, 0.0, 0.3],
            external_acceleration_eci_m_s2: [0.0, -9.81, 0.0],
        };
        let mut output = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];

        evaluate_scvx_rigid_body_dynamics(
            &state,
            &control,
            &parameters,
            ScvxRigidBodyDynamicsSettings::default(),
            &mut output,
        )?;

        assert!((output[0] - 4.0).abs() < 1.0e-12);
        assert!((output[1] - 5.0).abs() < 1.0e-12);
        assert!((output[2] - 6.0).abs() < 1.0e-12);
        assert!((output[3] - 1.0).abs() < 1.0e-12);
        assert!((output[4] + 7.81).abs() < 1.0e-12);
        assert!((output[5] - 3.0).abs() < 1.0e-12);
        assert!((output[6] - 0.0).abs() < 1.0e-12);
        assert!((output[7] - 0.5).abs() < 1.0e-12);
        assert!((output[8] - 1.0).abs() < 1.0e-12);
        assert!((output[9] - 1.5).abs() < 1.0e-12);
        assert!((output[10] + 2.55).abs() < 1.0e-12);
        assert!((output[11] - 7.6 / 3.0).abs() < 1.0e-12);
        assert!((output[12] - 0.025).abs() < 1.0e-12);

        Ok(())
    }

    #[test]
    fn scvx_rigid_body_dynamics_rejects_bad_inputs() {
        let mut state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![0.0; ScvxRigidBodyControlLayout::DIMENSION];
        let mut output = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            1.0,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        );

        state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = 2.0;
        assert!(matches!(
            evaluate_scvx_rigid_body_dynamics(
                &state,
                &control,
                &parameters,
                ScvxRigidBodyDynamicsSettings::default(),
                &mut output,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let singular = ScvxRigidBodyDynamicsParameters::new(
            1.0,
            [1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
        );
        assert!(matches!(
            evaluate_scvx_rigid_body_dynamics(
                &state,
                &control,
                &singular,
                ScvxRigidBodyDynamicsSettings::default(),
                &mut output,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_powered_landing_dynamics_matches_szmuk_style_equations() -> Result<(), TrajoptError> {
        let state = powered_landing_state(2.0, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [1.0, 2.0, 3.0]);
        let control = vec![10.0, 20.0, 30.0];
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [-1.0, -2.0, -3.0],
            inertia_body_kg_m2: [2.0, 0.0, 0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 4.0],
            thrust_application_point_body_m: [-0.1, 0.2, 0.3],
            mass_flow_coefficient_s_m: 0.01,
        };
        let mut output = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];

        evaluate_scvx_powered_landing_dynamics(
            &state,
            &control,
            &parameters,
            ScvxRigidBodyDynamicsSettings::default(),
            &mut output,
        )?;

        assert!((output[0] + 0.01 * (1400.0_f64).sqrt()).abs() < 1.0e-12);
        assert!((output[1] - 4.0).abs() < 1.0e-12);
        assert!((output[2] - 5.0).abs() < 1.0e-12);
        assert!((output[3] - 6.0).abs() < 1.0e-12);
        assert!((output[4] - 4.0).abs() < 1.0e-12);
        assert!((output[5] - 8.0).abs() < 1.0e-12);
        assert!((output[6] - 12.0).abs() < 1.0e-12);
        assert!(output[7].abs() < 1.0e-12);
        assert!((output[8] - 0.5).abs() < 1.0e-12);
        assert!((output[9] - 1.0).abs() < 1.0e-12);
        assert!((output[10] - 1.5).abs() < 1.0e-12);
        assert!((output[11] + 3.0).abs() < 1.0e-12);
        assert!((output[12] - 4.0).abs() < 1.0e-12);
        assert!((output[13] + 1.5).abs() < 1.0e-12);

        Ok(())
    }

    #[test]
    fn scvx_powered_landing_dynamics_rejects_bad_inputs() {
        let mut state =
            powered_landing_state(1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![0.0; ScvxPoweredLandingControlLayout::DIMENSION];
        let mut output = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [0.0, 0.0, -1.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            thrust_application_point_body_m: [-0.1, 0.0, 0.0],
            mass_flow_coefficient_s_m: 0.0,
        };

        state[ScvxPoweredLandingStateLayout::MASS] = 0.0;
        assert!(matches!(
            evaluate_scvx_powered_landing_dynamics(
                &state,
                &control,
                &parameters,
                ScvxRigidBodyDynamicsSettings::default(),
                &mut output,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let mut state =
            powered_landing_state(1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = 2.0;
        assert!(matches!(
            evaluate_scvx_powered_landing_dynamics(
                &state,
                &control,
                &parameters,
                ScvxRigidBodyDynamicsSettings::default(),
                &mut output,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_builds_powered_landing_foh_problem_from_reference() -> Result<(), TrajoptError> {
        let state = powered_landing_state(2.0, [1.0, 2.0, 3.0], [4.0, 5.0, 6.0], [0.0, 0.0, 0.0]);
        let control = vec![2.0, 0.0, 0.0];
        let definition = powered_landing_problem_definition(
            [state.clone(), state.clone()].concat(),
            [control.clone(), control.clone()].concat(),
            state.clone(),
        );
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [-1.0, 0.0, 0.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.0],
            thrust_application_point_body_m: [-0.1, 0.0, 0.0],
            mass_flow_coefficient_s_m: 0.01,
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };

        let problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        assert_eq!(
            problem.state_dimension,
            ScvxPoweredLandingStateLayout::DIMENSION
        );
        assert_eq!(
            problem.control_dimension,
            ScvxPoweredLandingControlLayout::DIMENSION
        );
        let predicted = predict_segment(
            &problem.dynamics[0],
            ScvxPoweredLandingStateLayout::DIMENSION,
            ScvxPoweredLandingControlLayout::DIMENSION,
            &state,
            &control,
            &control,
        );
        let mut derivative = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        evaluate_scvx_powered_landing_dynamics(
            &state,
            &control,
            &parameters,
            dynamics_settings,
            &mut derivative,
        )?;
        for component in 0..ScvxPoweredLandingStateLayout::DIMENSION {
            assert!(
                (predicted[component] - (state[component] + 0.1 * derivative[component])).abs()
                    < 1.0e-7
            );
        }

        Ok(())
    }

    #[test]
    fn scvx_powered_landing_forward_euler_foh_solves_vertical_translation()
    -> Result<(), TrajoptError> {
        let node_count = 5usize;
        let final_time_s = 2.0;
        let mut times_s = Vec::with_capacity(node_count);
        let mut initial_states =
            Vec::with_capacity(node_count * ScvxPoweredLandingStateLayout::DIMENSION);
        let mut initial_controls =
            Vec::with_capacity(node_count * ScvxPoweredLandingControlLayout::DIMENSION);
        for node in 0..node_count {
            let fraction = node as f64 / (node_count - 1) as f64;
            let time_s = final_time_s * fraction;
            times_s.push(time_s);
            initial_states.extend(powered_landing_state(
                1.0,
                [1.0 - fraction, 0.0, 0.0],
                [-1.0 + fraction, 0.0, 0.0],
                [0.0, 0.0, 0.0],
            ));
            initial_controls.extend([1.5, 0.0, 0.0]);
        }
        let target = powered_landing_state(1.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let envelope_limits = ScvxPoweredLandingEnvelopeLimits {
            minimum_mass: 1.0,
            maximum_mass: 1.0,
            minimum_thrust: 0.0,
            maximum_thrust: 2.0,
            maximum_gimbal_angle_rad: 5.0_f64.to_radians(),
            maximum_tilt_angle_rad: 5.0_f64.to_radians(),
            maximum_angular_rate_rad_s: 1.0e-3,
            glideslope_angle_rad: 1.0_f64.to_radians(),
            gravity_eci: [-1.0, 0.0, 0.0],
        };
        let definition = ScvxForwardEulerFohProblemDefinition {
            state_dimension: ScvxPoweredLandingStateLayout::DIMENSION,
            control_dimension: ScvxPoweredLandingControlLayout::DIMENSION,
            times_s,
            initial_state: initial_states[..ScvxPoweredLandingStateLayout::DIMENSION].to_vec(),
            target: ScvxTerminalStateTarget {
                state: target,
                weights: None,
            },
            initial_states,
            initial_controls,
            control_lower: vec![0.0, 0.0, 0.0],
            control_upper: vec![2.0, 0.0, 0.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        };
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [-1.0, 0.0, 0.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            thrust_application_point_body_m: [-0.01, 0.0, 0.0],
            mass_flow_coefficient_s_m: 0.0,
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let report = solve_scvx_forward_euler_foh(
            &definition,
            |_time_s, state, control, output| {
                evaluate_scvx_powered_landing_dynamics(
                    state,
                    control,
                    &parameters,
                    dynamics_settings,
                    output,
                )
            },
            ScvxContinuousFohSettings {
                max_outer_iterations: 8,
                reference_update_tolerance: 1.0e-6,
                scvx_settings: ScvxSettings {
                    max_iterations: 8,
                    initial_trust_region_radius: 5.0,
                    max_trust_region_radius: 10.0,
                    terminal_tolerance: 1.0e-7,
                    virtual_control_tolerance: 1.0e-7,
                    terminal_weight: 1.0e8,
                    virtual_control_weight: 1.0e10,
                    control_weight: 1.0e-3,
                    ..ScvxSettings::default()
                },
                linearization_settings: ScvxForwardEulerLinearizationSettings::default(),
            },
        )?;

        assert!(
            report.converged,
            "powered-landing FOH outer loop did not converge: terminal={}, virtual={}, outer={:?}",
            report.solution.terminal_error_norm,
            report.solution.max_virtual_control_norm,
            report.outer_iterations
        );
        assert!(
            report.solution.converged,
            "powered-landing affine solve did not converge: status={:?}, terminal={}, virtual={}, iterations={:?}",
            report.solution.termination_status,
            report.solution.terminal_error_norm,
            report.solution.max_virtual_control_norm,
            report.solution.iterations
        );
        assert!(report.solution.terminal_error_norm < 1.0e-7);
        assert!(report.solution.max_virtual_control_norm < 1.0e-7);
        let terminal = &report.solution.states[4 * ScvxPoweredLandingStateLayout::DIMENSION
            ..5 * ScvxPoweredLandingStateLayout::DIMENSION];
        assert!(terminal[ScvxPoweredLandingStateLayout::POSITION_ECI].abs() < 1.0e-7);
        assert!(terminal[ScvxPoweredLandingStateLayout::VELOCITY_ECI].abs() < 1.0e-7);

        let envelope = evaluate_scvx_powered_landing_envelope(
            &definition.times_s,
            &report.solution.states,
            &report.solution.controls,
            envelope_limits,
            dynamics_settings,
        )?;
        assert!(envelope.max_violation < 1.0e-9);
        assert!(envelope.max_thrust_violation < 1.0e-12);
        assert!(envelope.max_gimbal_angle_violation_rad < 1.0e-12);
        assert!(envelope.max_tilt_angle_violation_rad < 1.0e-12);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_fixture_pins_constants_and_builds_powered_landing_reference()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        assert_eq!(fixture.dataset_id, "szmuk-acikmese-6dof-free-final-time-v1");
        assert!((fixture.vehicle.wet_mass - 2.0).abs() < 1.0e-12);
        assert!((fixture.vehicle.dry_mass - 1.0).abs() < 1.0e-12);
        assert!((fixture.vehicle.minimum_thrust - 0.30).abs() < 1.0e-12);
        assert!((fixture.vehicle.maximum_thrust - 5.00).abs() < 1.0e-12);
        assert!((fixture.vehicle.maximum_gimbal_angle_deg - 20.0).abs() < 1.0e-12);
        assert!((fixture.vehicle.maximum_tilt_angle_deg - 90.0).abs() < 1.0e-12);
        assert!((fixture.vehicle.maximum_angular_rate_deg_per_time_unit - 60.0).abs() < 1.0e-12);
        assert!((fixture.environment.glideslope_angle_deg - 20.0).abs() < 1.0e-12);
        assert!((fixture.algorithm.virtual_control_weight - 1.0e5).abs() < 1.0e-12);
        assert!((fixture.algorithm.state_control_trust_region_weight - 1.0e-3).abs() < 1.0e-12);
        assert!((fixture.algorithm.time_trust_region_weight - 1.0e-1).abs() < 1.0e-12);
        assert!((fixture.algorithm.virtual_control_tolerance - 1.0e-10).abs() < 1.0e-18);
        assert!((fixture.algorithm.trust_region_tolerance - 1.0e-3).abs() < 1.0e-12);
        assert_eq!(fixture.algorithm.discretization_points, 50);
        assert_eq!(fixture.algorithm.maximum_iterations, 15);
        assert_eq!(
            fixture.algorithm.published_in_plane_convergence_iteration,
            6
        );
        assert_eq!(
            fixture
                .algorithm
                .published_out_of_plane_convergence_iteration,
            9
        );
        assert!((fixture.algorithm.published_time_of_flight_agreement - 0.01).abs() < 1.0e-12);
        assert_eq!(
            fixture.algorithm.initial_time_guesses,
            vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0, 9.0, 10.0]
        );

        let definition = szmuk_in_plane_powered_landing_definition(&fixture, 4.0)?;
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: 0.0,
        };
        let problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        assert_eq!(
            problem.state_dimension,
            ScvxPoweredLandingStateLayout::DIMENSION
        );
        assert_eq!(
            problem.control_dimension,
            ScvxPoweredLandingControlLayout::DIMENSION
        );
        assert_eq!(
            problem.horizon_steps,
            fixture.algorithm.discretization_points - 1
        );
        assert_eq!(problem.dynamics.len(), problem.horizon_steps);
        assert_eq!(
            problem.initial_state[ScvxPoweredLandingStateLayout::MASS].to_bits(),
            fixture.vehicle.wet_mass.to_bits()
        );
        assert_eq!(
            problem
                .initial_controls
                .chunks_exact(ScvxPoweredLandingControlLayout::DIMENSION)
                .next()
                .expect("first control node exists"),
            &[2.0, 0.0, 0.0]
        );

        Ok(())
    }

    #[test]
    fn scvx_powered_landing_linear_envelope_constraints_feed_solver_rows()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let mut definition = szmuk_in_plane_powered_landing_definition(&fixture, 4.0)?;
        let cone_facets = 8usize;
        let node_count = definition.times_s.len();
        let constraints = build_scvx_powered_landing_linear_envelope_constraints(
            node_count,
            szmuk_powered_landing_envelope_limits(&fixture),
            cone_facets,
        )?;
        assert_eq!(
            constraints.linear_state_constraints.len(),
            node_count * (2 + cone_facets)
        );
        assert_eq!(
            constraints.linear_control_constraints.len(),
            node_count * (2 + cone_facets)
        );
        definition.linear_state_constraints = constraints.linear_state_constraints;
        definition.linear_control_constraints = constraints.linear_control_constraints;

        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(&fixture, 4.0),
        };
        let problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
            ScvxForwardEulerLinearizationSettings::default(),
        )?;

        assert_eq!(
            problem.linear_state_constraints.len(),
            node_count * (2 + cone_facets)
        );
        assert_eq!(
            problem.linear_control_constraints.len(),
            node_count * (2 + cone_facets)
        );
        assert!(max_linear_state_violation(&problem, &problem.initial_states)? <= 1.0e-12);
        assert!(max_linear_control_violation(&problem, &problem.initial_controls)? <= 1.0e-12);

        let mut bad_limits = szmuk_powered_landing_envelope_limits(&fixture);
        bad_limits.maximum_gimbal_angle_rad = core::f64::consts::FRAC_PI_2;
        assert!(matches!(
            build_scvx_powered_landing_linear_envelope_constraints(node_count, bad_limits, 8),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            build_scvx_powered_landing_linear_envelope_constraints(
                0,
                szmuk_powered_landing_envelope_limits(&fixture),
                8,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            build_scvx_powered_landing_linear_envelope_constraints(
                node_count,
                szmuk_powered_landing_envelope_limits(&fixture),
                3,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        Ok(())
    }

    #[test]
    fn scvx_powered_landing_linearized_envelope_constraints_feed_solver_rows()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let mut definition = szmuk_in_plane_powered_landing_definition(&fixture, 4.0)?;
        let settings = ScvxPoweredLandingLinearizedEnvelopeSettings {
            cone_facets: 8,
            norm_facets: 10,
            finite_difference_step: 1.0e-6,
            quaternion_norm_tolerance: 1.0e-4,
        };
        let node_count = definition.times_s.len();
        let constraints = build_scvx_powered_landing_linearized_envelope_constraints(
            &definition.initial_states,
            szmuk_powered_landing_envelope_limits(&fixture),
            settings,
        )?;
        assert_eq!(
            constraints.linear_state_constraints.len(),
            node_count * (3 + settings.cone_facets + settings.norm_facets)
        );
        assert_eq!(
            constraints.linear_control_constraints.len(),
            node_count * (2 + settings.cone_facets + settings.norm_facets)
        );
        definition.linear_state_constraints = constraints.linear_state_constraints;
        definition.linear_control_constraints = constraints.linear_control_constraints;

        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(&fixture, 4.0),
        };
        let problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
            ScvxForwardEulerLinearizationSettings::default(),
        )?;

        assert!(max_linear_state_violation(&problem, &problem.initial_states)? <= 1.0e-9);
        assert!(max_linear_control_violation(&problem, &problem.initial_controls)? <= 1.0e-9);

        let mut bad_states = definition.initial_states.clone();
        bad_states[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = 2.0;
        assert!(matches!(
            build_scvx_powered_landing_linearized_envelope_constraints(
                &bad_states,
                szmuk_powered_landing_envelope_limits(&fixture),
                settings,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            build_scvx_powered_landing_linearized_envelope_constraints(
                &definition.initial_states[..ScvxPoweredLandingStateLayout::DIMENSION - 1],
                szmuk_powered_landing_envelope_limits(&fixture),
                settings,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            build_scvx_powered_landing_linearized_envelope_constraints(
                &definition.initial_states,
                szmuk_powered_landing_envelope_limits(&fixture),
                ScvxPoweredLandingLinearizedEnvelopeSettings {
                    norm_facets: 5,
                    ..settings
                },
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        Ok(())
    }

    #[test]
    fn scvx_powered_landing_conic_envelope_constraints_feed_soc_solver_rows()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let mut definition = szmuk_in_plane_powered_landing_definition(&fixture, 4.0)?;
        let settings = ScvxPoweredLandingConicEnvelopeSettings {
            finite_difference_step: 1.0e-6,
            quaternion_norm_tolerance: 1.0e-4,
        };
        let node_count = definition.times_s.len();
        let constraints = build_scvx_powered_landing_conic_envelope_constraints(
            &definition.initial_states,
            szmuk_powered_landing_envelope_limits(&fixture),
            settings,
        )?;
        assert_eq!(constraints.linear_state_constraints.len(), node_count * 3);
        assert_eq!(constraints.linear_control_constraints.len(), node_count);
        assert_eq!(
            constraints.second_order_state_constraints.len(),
            node_count * 2
        );
        assert_eq!(
            constraints.second_order_control_constraints.len(),
            node_count * 2
        );
        assert_eq!(
            constraints.second_order_state_constraints[0].vector.len(),
            2
        );
        assert_eq!(
            constraints.second_order_state_constraints[1].vector.len(),
            3
        );
        assert_eq!(
            constraints.second_order_control_constraints[0].vector.len(),
            3
        );
        assert_eq!(
            constraints.second_order_control_constraints[1].vector.len(),
            2
        );

        definition.linear_state_constraints = constraints.linear_state_constraints;
        definition.linear_control_constraints = constraints.linear_control_constraints;
        definition.second_order_state_constraints = constraints.second_order_state_constraints;
        definition.second_order_control_constraints = constraints.second_order_control_constraints;

        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(&fixture, 4.0),
        };
        let problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
            ScvxForwardEulerLinearizationSettings::default(),
        )?;

        assert_eq!(problem.linear_state_constraints.len(), node_count * 3);
        assert_eq!(problem.linear_control_constraints.len(), node_count);
        assert_eq!(problem.second_order_state_constraints.len(), node_count * 2);
        assert_eq!(
            problem.second_order_control_constraints.len(),
            node_count * 2
        );
        assert!(max_linear_state_violation(&problem, &problem.initial_states)? <= 1.0e-9);
        assert!(max_linear_control_violation(&problem, &problem.initial_controls)? <= 1.0e-12);
        assert!(
            max_second_order_cone_violation(
                &problem,
                &problem.initial_states,
                &problem.initial_controls
            )? <= 1.0e-12
        );

        let mut bad_states = definition.initial_states.clone();
        bad_states[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = 2.0;
        assert!(matches!(
            build_scvx_powered_landing_conic_envelope_constraints(
                &bad_states,
                szmuk_powered_landing_envelope_limits(&fixture),
                settings,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        let mut bad_limits = szmuk_powered_landing_envelope_limits(&fixture);
        bad_limits.maximum_gimbal_angle_rad = core::f64::consts::FRAC_PI_2;
        assert!(matches!(
            build_scvx_powered_landing_conic_envelope_constraints(
                &definition.initial_states,
                bad_limits,
                settings,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        assert!(matches!(
            build_scvx_powered_landing_conic_envelope_constraints(
                &definition.initial_states[..ScvxPoweredLandingStateLayout::DIMENSION - 1],
                szmuk_powered_landing_envelope_limits(&fixture),
                settings,
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        Ok(())
    }

    #[test]
    fn scvx_powered_landing_conic_envelope_solve_reports_infeasible_target_honestly()
    -> Result<(), TrajoptError> {
        let initial = powered_landing_state(1.0, [1.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let target = powered_landing_state(1.0, [0.1, 1.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let limits = ScvxPoweredLandingEnvelopeLimits {
            minimum_mass: 1.0,
            maximum_mass: 1.0,
            minimum_thrust: 0.2,
            maximum_thrust: 10.0,
            maximum_gimbal_angle_rad: 20.0_f64.to_radians(),
            maximum_tilt_angle_rad: 5.0_f64.to_radians(),
            maximum_angular_rate_rad_s: 0.1,
            glideslope_angle_rad: 45.0_f64.to_radians(),
            gravity_eci: [-1.0, 0.0, 0.0],
        };
        let reference_states = [initial.clone(), target.clone()].concat();
        let conic_constraints = build_scvx_powered_landing_conic_envelope_constraints(
            &reference_states,
            limits,
            ScvxPoweredLandingConicEnvelopeSettings {
                finite_difference_step: 1.0e-6,
                quaternion_norm_tolerance: 1.0e-4,
            },
        )?;
        let mut state_transition = vec![
            0.0;
            ScvxPoweredLandingStateLayout::DIMENSION
                * ScvxPoweredLandingStateLayout::DIMENSION
        ];
        for component in 0..ScvxPoweredLandingStateLayout::DIMENSION {
            state_transition[component * ScvxPoweredLandingStateLayout::DIMENSION + component] =
                1.0;
        }
        let problem = ScvxDiscreteAffineProblem {
            state_dimension: ScvxPoweredLandingStateLayout::DIMENSION,
            control_dimension: ScvxPoweredLandingControlLayout::DIMENSION,
            horizon_steps: 1,
            initial_state: initial,
            target: ScvxTerminalStateTarget {
                state: target.clone(),
                weights: None,
            },
            dynamics: vec![ScvxAffineDynamicsSegment {
                state_transition,
                control_left: vec![
                    0.0;
                    ScvxPoweredLandingStateLayout::DIMENSION
                        * ScvxPoweredLandingControlLayout::DIMENSION
                ],
                control_right: vec![
                    0.0;
                    ScvxPoweredLandingStateLayout::DIMENSION
                        * ScvxPoweredLandingControlLayout::DIMENSION
                ],
                affine: vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION],
            }],
            initial_states: reference_states,
            initial_controls: vec![0.2, 0.0, 0.0, 0.2, 0.0, 0.0],
            control_lower: vec![0.0, -10.0, -10.0],
            control_upper: vec![10.0, 10.0, 10.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: conic_constraints.linear_state_constraints,
            linear_control_constraints: conic_constraints.linear_control_constraints,
            second_order_state_constraints: conic_constraints.second_order_state_constraints,
            second_order_control_constraints: conic_constraints.second_order_control_constraints,
        };

        let target_envelope = evaluate_scvx_powered_landing_envelope(
            &[0.0, 1.0],
            &problem.initial_states,
            &problem.initial_controls,
            limits,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
        )?;
        assert!(target_envelope.max_glideslope_violation > 0.0);

        let solution = solve_scvx_discrete_affine(
            &problem,
            ScvxSettings {
                max_iterations: 1,
                terminal_tolerance: 1.0e-8,
                virtual_control_tolerance: 1.0e-8,
                constraint_tolerance: 1.0e-8,
                initial_trust_region_radius: 5.0,
                max_trust_region_radius: 5.0,
                terminal_weight: 1.0e2,
                virtual_control_weight: 1.0,
                control_weight: 1.0e-3,
                ..ScvxSettings::default()
            },
        )?;

        assert_eq!(solution.solver_status, SolverStatus::Solved);
        assert!(!solution.converged);
        assert_eq!(
            solution.termination_status,
            ScvxTerminationStatus::MaxIterations
        );
        assert!(solution.terminal_error_norm > 1.0e-8);
        assert!(solution.max_second_order_cone_violation <= 1.0e-8);
        assert!(solution.max_linear_state_violation <= 1.0e-8);
        assert!(solution.max_linear_control_violation <= 1.0e-8);
        let terminal = &solution.states[ScvxPoweredLandingStateLayout::DIMENSION
            ..2 * ScvxPoweredLandingStateLayout::DIMENSION];
        let altitude = terminal[ScvxPoweredLandingStateLayout::POSITION_ECI];
        let lateral = terminal[ScvxPoweredLandingStateLayout::POSITION_ECI + 1].abs();
        assert!(lateral <= altitude + 1.0e-8);
        assert!(
            terminal[ScvxPoweredLandingStateLayout::POSITION_ECI + 1]
                < target[ScvxPoweredLandingStateLayout::POSITION_ECI + 1]
        );

        let solved_envelope = evaluate_scvx_powered_landing_envelope(
            &[0.0, 1.0],
            &solution.states,
            &solution.controls,
            limits,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
        )?;
        assert!(solved_envelope.max_violation <= 1.0e-8);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_in_plane_conic_envelope_affine_solve_enforces_rows() -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let initial_time_guess = 4.0;
        let definition =
            szmuk_in_plane_powered_landing_conic_envelope_definition(&fixture, initial_time_guess)?;

        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                &fixture,
                initial_time_guess,
            ),
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings::default(),
        )?;
        let initial_virtual_controls =
            dynamics_defects(&problem, &problem.initial_states, &problem.initial_controls)?;
        let initial_virtual_control_norm = max_interval_norm(
            &initial_virtual_controls,
            problem.horizon_steps,
            problem.state_dimension,
        );

        let settings = ScvxSettings {
            max_iterations: fixture.algorithm.maximum_iterations,
            terminal_tolerance: 1.0e-4,
            virtual_control_tolerance: 2.0e-2,
            constraint_tolerance: 1.0e-7,
            initial_trust_region_radius: 20.0,
            max_trust_region_radius: 40.0,
            terminal_weight: 1.0e8,
            virtual_control_weight: fixture.algorithm.virtual_control_weight,
            control_weight: fixture.algorithm.state_control_trust_region_weight,
            ..ScvxSettings::default()
        };
        let solution = solve_scvx_discrete_affine(&problem, settings)?;

        assert_eq!(solution.solver_status, SolverStatus::Solved);
        assert!(!solution.converged);
        assert_eq!(
            solution.termination_status,
            ScvxTerminationStatus::MaxIterations
        );
        assert!(
            solution.terminal_error_norm <= settings.terminal_tolerance,
            "Szmuk conic-envelope terminal error {:e} exceeded tolerance",
            solution.terminal_error_norm
        );
        assert!(
            solution.max_virtual_control_norm > settings.virtual_control_tolerance,
            "Szmuk conic-envelope solve unexpectedly closed the virtual-control gap"
        );
        assert!(
            solution.max_virtual_control_norm < initial_virtual_control_norm,
            "Szmuk conic-envelope solve virtual control {:e} did not improve from {:e}",
            solution.max_virtual_control_norm,
            initial_virtual_control_norm
        );
        assert!(solution.max_linear_state_violation <= 1.0e-7);
        assert!(solution.max_linear_control_violation <= 1.0e-7);
        assert!(solution.max_second_order_cone_violation <= 1.0e-7);

        let mut normalized_states = solution.states.clone();
        normalize_scvx_powered_landing_quaternion_reference_states(&mut normalized_states)?;
        let envelope = evaluate_scvx_powered_landing_envelope(
            &definition.times_s,
            &normalized_states,
            &solution.controls,
            szmuk_powered_landing_envelope_limits(&fixture),
            dynamics_settings,
        )?;
        assert!(envelope.max_mass_violation <= 1.0e-7);
        assert!(envelope.max_thrust_violation <= 1.0e-7);
        assert!(envelope.max_gimbal_angle_violation_rad <= 1.0e-7);
        assert!(envelope.max_angular_rate_violation_rad_s <= 1.0e-7);
        assert!(envelope.max_glideslope_violation <= 1.0e-7);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_reference_satisfies_powered_landing_envelope() -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let definition = szmuk_in_plane_powered_landing_definition(&fixture, 4.0)?;
        let report = evaluate_scvx_powered_landing_envelope(
            &definition.times_s,
            &definition.initial_states,
            &definition.initial_controls,
            szmuk_powered_landing_envelope_limits(&fixture),
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
        )?;

        assert_eq!(report.sample_count, fixture.algorithm.discretization_points);
        assert!(report.within_limits);
        assert_eq!(report.max_violation, 0.0);
        assert_eq!(report.max_mass_violation, 0.0);
        assert_eq!(report.max_thrust_violation, 0.0);
        assert_eq!(report.max_gimbal_angle_violation_rad, 0.0);
        assert_eq!(report.max_tilt_angle_violation_rad, 0.0);
        assert_eq!(report.max_angular_rate_violation_rad_s, 0.0);
        assert_eq!(report.max_glideslope_violation, 0.0);
        assert!((report.samples[0].mass - fixture.vehicle.wet_mass).abs() < 1.0e-12);
        assert!((report.samples[0].thrust_norm - fixture.vehicle.wet_mass).abs() < 1.0e-12);
        assert!(
            (report.samples.last().expect("last envelope sample").mass - fixture.vehicle.dry_mass)
                .abs()
                < 1.0e-12
        );
        assert!(
            report
                .samples
                .iter()
                .all(|sample| sample.glideslope_margin >= -1.0e-12)
        );
        Ok(())
    }

    #[test]
    fn scvx_powered_landing_quaternion_reference_projection_normalizes_and_rejects_bad_inputs()
    -> Result<(), TrajoptError> {
        let q = ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ;
        let mut states = vec![0.0; 2 * ScvxPoweredLandingStateLayout::DIMENSION];
        states[q] = 2.0;
        let second = ScvxPoweredLandingStateLayout::DIMENSION + q;
        states[second + 1] = 3.0;
        states[second + 2] = 4.0;

        let max_norm_error =
            normalize_scvx_powered_landing_quaternion_reference_states(&mut states)?;
        assert!((max_norm_error - 4.0).abs() < 1.0e-12);
        for state in states.chunks_exact(ScvxPoweredLandingStateLayout::DIMENSION) {
            let norm = (state[q] * state[q]
                + state[q + 1] * state[q + 1]
                + state[q + 2] * state[q + 2]
                + state[q + 3] * state[q + 3])
                .sqrt();
            assert!((norm - 1.0).abs() < 1.0e-12);
        }

        let mut bad_len = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION - 1];
        assert!(matches!(
            normalize_scvx_powered_landing_quaternion_reference_states(&mut bad_len),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        let mut zero_norm = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        assert!(matches!(
            normalize_scvx_powered_landing_quaternion_reference_states(&mut zero_norm),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        let mut non_finite = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        non_finite[q] = f64::NAN;
        assert!(matches!(
            normalize_scvx_powered_landing_quaternion_reference_states(&mut non_finite),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        Ok(())
    }

    #[test]
    fn scvx_szmuk_in_plane_powered_landing_affine_foh_solve_reduces_virtual_control_and_reports_envelope_gap()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let initial_time_guess = 4.0;
        let definition = szmuk_in_plane_powered_landing_definition(&fixture, initial_time_guess)?;
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                &fixture,
                initial_time_guess,
            ),
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let initial_problem = build_scvx_powered_landing_foh_problem(
            &definition,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings::default(),
        )?;
        let initial_virtual_controls = dynamics_defects(
            &initial_problem,
            &initial_problem.initial_states,
            &initial_problem.initial_controls,
        )?;
        let initial_virtual_control_norm = max_interval_norm(
            &initial_virtual_controls,
            initial_problem.horizon_steps,
            initial_problem.state_dimension,
        );

        let solution = solve_scvx_discrete_affine(
            &initial_problem,
            ScvxSettings {
                max_iterations: fixture.algorithm.maximum_iterations,
                terminal_tolerance: 1.0e-4,
                virtual_control_tolerance: 2.0e-2,
                initial_trust_region_radius: 20.0,
                max_trust_region_radius: 40.0,
                terminal_weight: 1.0e8,
                virtual_control_weight: fixture.algorithm.virtual_control_weight,
                control_weight: fixture.algorithm.state_control_trust_region_weight,
                ..ScvxSettings::default()
            },
        )?;

        assert!(
            solution.converged,
            "Szmuk in-plane affine solve did not converge: status={:?}, terminal={}, virtual={}, iterations={:?}",
            solution.termination_status,
            solution.terminal_error_norm,
            solution.max_virtual_control_norm,
            solution.iterations
        );
        assert!(
            solution.terminal_error_norm <= 1.0e-4,
            "Szmuk in-plane terminal error {:e} exceeded tolerance",
            solution.terminal_error_norm,
        );
        assert!(
            solution.max_virtual_control_norm <= 0.06 * initial_virtual_control_norm,
            "Szmuk in-plane virtual control {:e} did not improve from {:e}",
            solution.max_virtual_control_norm,
            initial_virtual_control_norm
        );
        let mut normalized_states = solution.states.clone();
        let max_quaternion_norm_error =
            normalize_scvx_powered_landing_quaternion_reference_states(&mut normalized_states)?;
        assert!(
            max_quaternion_norm_error > dynamics_settings.quaternion_norm_tolerance,
            "Szmuk in-plane affine solve quaternion norm drift {max_quaternion_norm_error:e} stayed inside relinearization tolerance",
        );
        assert!(
            max_quaternion_norm_error <= 9.0e-1,
            "Szmuk in-plane affine solve quaternion norm drift {max_quaternion_norm_error:e}"
        );
        let mut raw_relinearization = definition.clone();
        raw_relinearization.initial_states = solution.states.clone();
        raw_relinearization.initial_controls = solution.controls.clone();
        assert!(matches!(
            build_scvx_powered_landing_foh_problem(
                &raw_relinearization,
                &parameters,
                dynamics_settings,
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        let mut projected_relinearization = raw_relinearization.clone();
        projected_relinearization.initial_states = normalized_states.clone();
        let projected_problem = build_scvx_powered_landing_foh_problem(
            &projected_relinearization,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings::default(),
        )?;
        assert_eq!(
            projected_problem.horizon_steps,
            definition.times_s.len() - 1
        );
        assert_eq!(projected_problem.initial_states, normalized_states);

        let envelope = evaluate_scvx_powered_landing_envelope(
            &definition.times_s,
            &normalized_states,
            &solution.controls,
            szmuk_powered_landing_envelope_limits(&fixture),
            dynamics_settings,
        )?;
        assert!(!envelope.within_limits);
        assert!(envelope.max_violation > 0.0);
        assert!(envelope.max_violation <= 3.3);
        assert!(envelope.max_thrust_violation > 0.0);
        assert!(envelope.max_gimbal_angle_violation_rad > 0.0);
        assert!(envelope.max_angular_rate_violation_rad_s > 0.0);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_time_of_flight_search_shrinks_in_plane_window() -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let center_time = fixture.algorithm.initial_time_guesses[3];
        let half_width = 2.0 * fixture.algorithm.published_time_of_flight_agreement;
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let settings = ScvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s: center_time - half_width,
            upper_time_of_flight_s: center_time + half_width,
            tolerance_s: fixture.algorithm.published_time_of_flight_agreement,
            max_iterations: 3,
            time_penalty_weight: fixture.algorithm.time_trust_region_weight,
            scvx_settings: ScvxSettings {
                max_iterations: fixture.algorithm.maximum_iterations,
                terminal_tolerance: 1.0e-4,
                virtual_control_tolerance: 2.0e-2,
                initial_trust_region_radius: 20.0,
                max_trust_region_radius: 40.0,
                terminal_weight: 1.0e8,
                virtual_control_weight: fixture.algorithm.virtual_control_weight,
                control_weight: fixture.algorithm.state_control_trust_region_weight,
                ..ScvxSettings::default()
            },
        };

        let report = solve_scvx_time_of_flight_search(settings, |time_of_flight_s| {
            let definition = szmuk_in_plane_powered_landing_definition(&fixture, time_of_flight_s)?;
            let parameters = ScvxPoweredLandingDynamicsParameters {
                gravity_eci_m_s2: fixture.environment.gravity,
                inertia_body_kg_m2: fixture.vehicle.inertia_body,
                thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
                mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                    &fixture,
                    time_of_flight_s,
                ),
            };
            build_scvx_powered_landing_foh_problem(
                &definition,
                &parameters,
                dynamics_settings,
                ScvxForwardEulerLinearizationSettings::default(),
            )
        })?;

        assert_eq!(report.iterations, settings.max_iterations);
        assert_eq!(report.candidate_count, settings.max_iterations + 2);
        assert_eq!(report.candidates.len(), report.candidate_count);
        assert!(
            report.final_upper_time_of_flight_s - report.final_lower_time_of_flight_s
                <= fixture.algorithm.published_time_of_flight_agreement
        );
        assert!(
            (report.best_time_of_flight_s - center_time).abs() <= half_width,
            "Szmuk time search selected {} outside the local fixture window around {}",
            report.best_time_of_flight_s,
            center_time
        );
        assert!(report.solution.converged);
        assert!(report.solution.terminal_error_norm <= settings.scvx_settings.terminal_tolerance);
        assert!(
            report.solution.max_virtual_control_norm
                <= settings.scvx_settings.virtual_control_tolerance
        );
        assert!(report.candidates.iter().all(|candidate| {
            candidate.termination_status == ScvxTerminationStatus::Converged
                && candidate.terminal_error_norm <= settings.scvx_settings.terminal_tolerance
                && candidate.max_virtual_control_norm
                    <= settings.scvx_settings.virtual_control_tolerance
        }));
        assert!(report.candidates.iter().any(|candidate| {
            candidate.time_of_flight_s.to_bits() == report.best_time_of_flight_s.to_bits()
                && candidate.objective.to_bits() == report.objective.to_bits()
        }));
        Ok(())
    }

    #[test]
    fn scvx_szmuk_time_of_flight_sweep_reports_initial_guess_windows() -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let half_width = 2.0 * fixture.algorithm.published_time_of_flight_agreement;
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let settings = ScvxTimeOfFlightSweepSettings {
            center_times_of_flight_s: fixture.algorithm.initial_time_guesses.clone(),
            half_width_s: half_width,
            tolerance_s: fixture.algorithm.published_time_of_flight_agreement,
            max_iterations: 1,
            time_penalty_weight: fixture.algorithm.time_trust_region_weight,
            scvx_settings: ScvxSettings {
                max_iterations: fixture.algorithm.maximum_iterations,
                terminal_tolerance: 1.0e-4,
                virtual_control_tolerance: 2.0e-2,
                initial_trust_region_radius: 20.0,
                max_trust_region_radius: 40.0,
                terminal_weight: 1.0e8,
                virtual_control_weight: fixture.algorithm.virtual_control_weight,
                control_weight: fixture.algorithm.state_control_trust_region_weight,
                ..ScvxSettings::default()
            },
        };

        let report = solve_scvx_time_of_flight_sweep(settings.clone(), |time_of_flight_s| {
            let definition = szmuk_in_plane_powered_landing_definition(&fixture, time_of_flight_s)?;
            let parameters = ScvxPoweredLandingDynamicsParameters {
                gravity_eci_m_s2: fixture.environment.gravity,
                inertia_body_kg_m2: fixture.vehicle.inertia_body,
                thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
                mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                    &fixture,
                    time_of_flight_s,
                ),
            };
            build_scvx_powered_landing_foh_problem(
                &definition,
                &parameters,
                dynamics_settings,
                ScvxForwardEulerLinearizationSettings::default(),
            )
        })?;

        assert_eq!(report.window_count, settings.center_times_of_flight_s.len());
        assert_eq!(
            report.windows.len(),
            settings.center_times_of_flight_s.len()
        );
        assert!(report.converged_window_count >= 1);
        assert!(report.best_window_index.is_some());
        assert!(report.best_time_of_flight_s.is_some());
        assert!(report.best_objective.is_some());
        for (window_index, window) in report.windows.iter().enumerate() {
            assert_eq!(window.window_index, window_index);
            assert_eq!(
                window.center_time_of_flight_s.to_bits(),
                settings.center_times_of_flight_s[window_index].to_bits()
            );
            assert!(
                (window.lower_time_of_flight_s
                    - (window.center_time_of_flight_s - settings.half_width_s))
                    .abs()
                    < 1.0e-12
            );
            assert!(
                (window.upper_time_of_flight_s
                    - (window.center_time_of_flight_s + settings.half_width_s))
                    .abs()
                    < 1.0e-12
            );
            if let Some(search) = &window.search {
                assert!(window.converged);
                assert!(window.failure_reason.is_none());
                assert_eq!(search.candidates.len(), search.candidate_count);
                assert!(search.solution.converged);
                assert!(
                    search.solution.terminal_error_norm
                        <= settings.scvx_settings.terminal_tolerance
                );
                assert!(
                    search.solution.max_virtual_control_norm
                        <= settings.scvx_settings.virtual_control_tolerance
                );
            } else {
                assert!(!window.converged);
                assert!(window.failure_reason.is_some());
            }
        }
        let best_window = &report.windows[report.best_window_index.expect("best window")];
        let best_search = best_window.search.as_ref().expect("best search");
        assert_eq!(
            best_search.best_time_of_flight_s.to_bits(),
            report
                .best_time_of_flight_s
                .expect("best time of flight")
                .to_bits()
        );
        assert_eq!(
            best_search.objective.to_bits(),
            report.best_objective.expect("best objective").to_bits()
        );
        assert!(
            report.windows.iter().any(|window| {
                window.center_time_of_flight_s.to_bits()
                    == fixture.algorithm.initial_time_guesses[3].to_bits()
                    && window.converged
            }),
            "Szmuk sweep should keep the already-proven local published window converged"
        );

        Ok(())
    }

    #[test]
    fn scvx_szmuk_conic_envelope_time_sweep_reports_virtual_gap_window_failure()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let center_time = fixture.algorithm.initial_time_guesses[3];
        let half_width = 2.0 * fixture.algorithm.published_time_of_flight_agreement;
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let settings = ScvxTimeOfFlightSweepSettings {
            center_times_of_flight_s: vec![center_time],
            half_width_s: half_width,
            tolerance_s: fixture.algorithm.published_time_of_flight_agreement,
            max_iterations: 1,
            time_penalty_weight: fixture.algorithm.time_trust_region_weight,
            scvx_settings: ScvxSettings {
                max_iterations: fixture.algorithm.maximum_iterations,
                terminal_tolerance: 1.0e-4,
                virtual_control_tolerance: 2.0e-2,
                constraint_tolerance: 1.0e-7,
                initial_trust_region_radius: 20.0,
                max_trust_region_radius: 40.0,
                terminal_weight: 1.0e8,
                virtual_control_weight: fixture.algorithm.virtual_control_weight,
                control_weight: fixture.algorithm.state_control_trust_region_weight,
                ..ScvxSettings::default()
            },
        };

        let report = solve_scvx_time_of_flight_sweep(settings.clone(), |time_of_flight_s| {
            let definition = szmuk_in_plane_powered_landing_conic_envelope_definition(
                &fixture,
                time_of_flight_s,
            )?;
            let parameters = ScvxPoweredLandingDynamicsParameters {
                gravity_eci_m_s2: fixture.environment.gravity,
                inertia_body_kg_m2: fixture.vehicle.inertia_body,
                thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
                mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                    &fixture,
                    time_of_flight_s,
                ),
            };
            build_scvx_powered_landing_foh_problem(
                &definition,
                &parameters,
                dynamics_settings,
                ScvxForwardEulerLinearizationSettings::default(),
            )
        })?;

        assert_eq!(report.window_count, 1);
        assert_eq!(report.converged_window_count, 0);
        assert!(report.best_window_index.is_none());
        assert!(report.best_time_of_flight_s.is_none());
        assert!(report.best_objective.is_none());
        let window = &report.windows[0];
        assert_eq!(window.window_index, 0);
        assert_eq!(
            window.center_time_of_flight_s.to_bits(),
            center_time.to_bits()
        );
        assert!(!window.converged);
        assert!(window.search.is_none());
        assert_eq!(
            window.failure_reason,
            Some("SCvx time-of-flight candidate did not converge")
        );
        let candidate = window
            .failure_candidate
            .as_ref()
            .expect("failed Szmuk conic window should retain a candidate diagnostic");
        assert_eq!(candidate.time_of_flight_s.to_bits(), center_time.to_bits());
        assert!(candidate.objective.is_finite());
        assert_eq!(
            candidate.termination_status,
            ScvxTerminationStatus::MaxIterations
        );
        assert!(candidate.terminal_error_norm <= settings.scvx_settings.terminal_tolerance);
        assert!(
            candidate.max_virtual_control_norm > settings.scvx_settings.virtual_control_tolerance
        );
        assert!(candidate.max_linear_state_violation <= 1.0e-7);
        assert!(candidate.max_linear_control_violation <= 1.0e-7);
        assert!(candidate.max_second_order_cone_violation <= 1.0e-7);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_continuous_foh_projection_hook_relinearizes_after_quaternion_drift()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let initial_time_guess = 4.0;
        let definition = szmuk_in_plane_powered_landing_definition(&fixture, initial_time_guess)?;
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                &fixture,
                initial_time_guess,
            ),
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };

        let report = solve_scvx_forward_euler_foh_with_reference_projection(
            &definition,
            |_time_s, state, control, output| {
                evaluate_scvx_powered_landing_dynamics(
                    state,
                    control,
                    &parameters,
                    dynamics_settings,
                    output,
                )
            },
            ScvxContinuousFohSettings {
                max_outer_iterations: 2,
                reference_update_tolerance: 1.0e-9,
                scvx_settings: ScvxSettings {
                    max_iterations: fixture.algorithm.maximum_iterations,
                    terminal_tolerance: 1.0e-4,
                    virtual_control_tolerance: 2.0e-2,
                    initial_trust_region_radius: 20.0,
                    max_trust_region_radius: 40.0,
                    terminal_weight: 1.0e8,
                    virtual_control_weight: fixture.algorithm.virtual_control_weight,
                    control_weight: fixture.algorithm.state_control_trust_region_weight,
                    ..ScvxSettings::default()
                },
                linearization_settings: ScvxForwardEulerLinearizationSettings::default(),
            },
            |states| normalize_scvx_powered_landing_quaternion_reference_states(states).map(|_| ()),
        )?;

        assert!(!report.converged);
        assert_eq!(report.outer_iterations.len(), 2);
        assert!(report.outer_iterations[0].affine_converged);
        assert!(
            report.outer_iterations[0].max_state_update_norm
                > dynamics_settings.quaternion_norm_tolerance
        );
        assert!(report.outer_iterations[1].affine_iteration_count > 0);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_continuous_foh_reference_update_refreshes_conic_envelope_rows()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let initial_time_guess = 4.0;
        let definition = szmuk_in_plane_powered_landing_definition(&fixture, initial_time_guess)?;
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                &fixture,
                initial_time_guess,
            ),
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let mut update_count = 0usize;
        let mut refreshed_soc_state_row_counts = Vec::new();

        let report = solve_scvx_forward_euler_foh_with_reference_update(
            &definition,
            |_time_s, state, control, output| {
                evaluate_scvx_powered_landing_dynamics(
                    state,
                    control,
                    &parameters,
                    dynamics_settings,
                    output,
                )
            },
            ScvxContinuousFohSettings {
                max_outer_iterations: 2,
                reference_update_tolerance: 1.0e-9,
                scvx_settings: ScvxSettings {
                    max_iterations: fixture.algorithm.maximum_iterations,
                    terminal_tolerance: 1.0e-4,
                    virtual_control_tolerance: 1.0e-1,
                    constraint_tolerance: 1.0e-7,
                    initial_trust_region_radius: 20.0,
                    max_trust_region_radius: 40.0,
                    terminal_weight: 1.0e8,
                    virtual_control_weight: fixture.algorithm.virtual_control_weight,
                    control_weight: fixture.algorithm.state_control_trust_region_weight,
                    ..ScvxSettings::default()
                },
                linearization_settings: ScvxForwardEulerLinearizationSettings::default(),
            },
            |reference| {
                update_count += 1;
                normalize_scvx_powered_landing_quaternion_reference_states(
                    &mut reference.initial_states,
                )?;
                refresh_szmuk_powered_landing_conic_envelope_constraints(&fixture, reference)?;
                refreshed_soc_state_row_counts.push(reference.second_order_state_constraints.len());
                Ok(())
            },
        )?;

        assert!(!report.converged);
        assert_eq!(report.outer_iterations.len(), 2);
        assert_eq!(update_count, report.outer_iterations.len() + 1);
        assert!(
            refreshed_soc_state_row_counts
                .iter()
                .all(|&count| count == fixture.algorithm.discretization_points * 2)
        );
        assert!(report.outer_iterations[0].affine_converged);
        assert!(report.outer_iterations[1].affine_iteration_count > 0);
        assert!(report.outer_iterations[1].affine_converged);
        assert!(
            report.outer_iterations[1].max_virtual_control_norm
                < 0.5 * report.outer_iterations[0].max_virtual_control_norm
        );
        assert!(report.solution.converged);
        assert!(report.solution.max_virtual_control_norm <= 1.0e-1);
        assert!(report.solution.max_linear_state_violation <= 1.0e-7);
        assert!(report.solution.max_linear_control_violation <= 1.0e-7);
        assert!(report.solution.max_second_order_cone_violation <= 1.0e-7);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_continuous_foh_time_candidate_reports_conic_reference_gap()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let center_time = fixture.algorithm.initial_time_guesses[3];
        let half_width = 2.0 * fixture.algorithm.published_time_of_flight_agreement;
        let settings = ScvxContinuousFohTimeOfFlightSearchSettings {
            lower_time_of_flight_s: center_time - half_width,
            upper_time_of_flight_s: center_time + half_width,
            tolerance_s: fixture.algorithm.published_time_of_flight_agreement,
            max_iterations: 1,
            time_penalty_weight: fixture.algorithm.time_trust_region_weight,
        };

        let candidate =
            evaluate_scvx_continuous_foh_time_of_flight_candidate(settings, center_time, |time| {
                solve_szmuk_in_plane_conic_continuous_foh_candidate(&fixture, time)
            })?;

        assert_szmuk_continuous_conic_candidate_reports_reference_gap(&candidate, center_time);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_continuous_foh_time_sweep_reports_conic_reference_gap_window_failure()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let center_time = fixture.algorithm.initial_time_guesses[3];
        let half_width = 2.0 * fixture.algorithm.published_time_of_flight_agreement;
        let settings = ScvxContinuousFohTimeOfFlightSweepSettings {
            center_times_of_flight_s: vec![center_time],
            half_width_s: half_width,
            tolerance_s: fixture.algorithm.published_time_of_flight_agreement,
            max_iterations: 1,
            time_penalty_weight: fixture.algorithm.time_trust_region_weight,
        };

        let report = solve_scvx_continuous_foh_time_of_flight_sweep(settings.clone(), |time| {
            solve_szmuk_in_plane_conic_continuous_foh_candidate(&fixture, time)
        })?;

        assert_eq!(report.window_count, 1);
        assert_eq!(report.converged_window_count, 0);
        assert!(report.best_window_index.is_none());
        assert!(report.best_time_of_flight_s.is_none());
        assert!(report.best_objective.is_none());
        let window = &report.windows[0];
        assert_eq!(window.window_index, 0);
        assert_eq!(
            window.center_time_of_flight_s.to_bits(),
            center_time.to_bits()
        );
        assert!(!window.converged);
        assert!(window.search.is_none());
        assert_eq!(
            window.failure_reason,
            Some("SCvx continuous FOH time-of-flight candidate did not converge")
        );
        let candidate = window
            .failure_candidate
            .as_ref()
            .expect("failed Szmuk continuous-FOH window should retain a candidate diagnostic");
        assert_szmuk_continuous_conic_candidate_reports_reference_gap(candidate, center_time);
        Ok(())
    }

    #[test]
    fn scvx_szmuk_continuous_foh_time_sweep_retains_all_initial_window_diagnostics()
    -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let half_width = 2.0 * fixture.algorithm.published_time_of_flight_agreement;
        let settings = ScvxContinuousFohTimeOfFlightSweepSettings {
            center_times_of_flight_s: fixture.algorithm.initial_time_guesses.clone(),
            half_width_s: half_width,
            tolerance_s: fixture.algorithm.published_time_of_flight_agreement,
            max_iterations: 1,
            time_penalty_weight: fixture.algorithm.time_trust_region_weight,
        };

        let report = solve_scvx_continuous_foh_time_of_flight_sweep(settings.clone(), |time| {
            solve_szmuk_in_plane_conic_continuous_foh_candidate(&fixture, time)
        })?;

        assert_eq!(
            report.window_count,
            fixture.algorithm.initial_time_guesses.len()
        );
        assert_eq!(
            report.windows.len(),
            fixture.algorithm.initial_time_guesses.len()
        );
        assert_eq!(report.converged_window_count, 0);
        assert!(report.best_window_index.is_none());
        assert!(report.best_time_of_flight_s.is_none());
        assert!(report.best_objective.is_none());
        for (window_index, window) in report.windows.iter().enumerate() {
            let center_time = fixture.algorithm.initial_time_guesses[window_index];
            assert_eq!(window.window_index, window_index);
            assert_eq!(
                window.center_time_of_flight_s.to_bits(),
                center_time.to_bits()
            );
            assert!(
                (window.lower_time_of_flight_s - (center_time - settings.half_width_s)).abs()
                    < 1.0e-12
            );
            assert!(
                (window.upper_time_of_flight_s - (center_time + settings.half_width_s)).abs()
                    < 1.0e-12
            );
            assert!(!window.converged);
            assert!(window.search.is_none());
            assert_eq!(
                window.failure_reason,
                Some("SCvx continuous FOH time-of-flight candidate did not converge")
            );
            let candidate = window
                .failure_candidate
                .as_ref()
                .expect("failed Szmuk continuous-FOH window should retain a candidate diagnostic");
            assert_eq!(candidate.time_of_flight_s.to_bits(), center_time.to_bits());
            assert!(!candidate.converged);
            assert!(candidate.objective.is_finite());
            assert!(candidate.outer_iteration_count >= 1);
            assert!(candidate.terminal_error_norm.is_finite());
            assert!(candidate.max_virtual_control_norm.is_finite());
            assert!(candidate.max_linear_state_violation.is_finite());
            assert!(candidate.max_linear_control_violation.is_finite());
            assert!(candidate.max_second_order_cone_violation.is_finite());
        }
        Ok(())
    }

    #[test]
    fn scvx_powered_landing_envelope_reports_violations() -> Result<(), TrajoptError> {
        let fixture = szmuk_benchmark_fixture();
        let definition = szmuk_in_plane_powered_landing_definition(&fixture, 4.0)?;
        let mut controls = definition.initial_controls.clone();
        controls[ScvxPoweredLandingControlLayout::THRUST_BODY + 1] = fixture.vehicle.maximum_thrust;

        let report = evaluate_scvx_powered_landing_envelope(
            &definition.times_s,
            &definition.initial_states,
            &controls,
            szmuk_powered_landing_envelope_limits(&fixture),
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
        )?;

        assert!(!report.within_limits);
        assert!(report.max_gimbal_angle_violation_rad > 0.0);
        assert!(report.max_thrust_violation > 0.0);
        assert!(report.samples[0].max_violation > 0.0);

        let mut bad_limits = szmuk_powered_landing_envelope_limits(&fixture);
        bad_limits.gravity_eci = [0.0, 0.0, 0.0];
        assert!(matches!(
            evaluate_scvx_powered_landing_envelope(
                &definition.times_s,
                &definition.initial_states,
                &definition.initial_controls,
                bad_limits,
                ScvxRigidBodyDynamicsSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        Ok(())
    }

    #[test]
    fn scvx_forward_euler_linearizes_rigid_body_translation_force() -> Result<(), TrajoptError> {
        let state = rigid_body_state([10.0, 20.0, 30.0], [4.0, 5.0, 6.0], [0.0, 0.0, 0.0]);
        let reference_states = [state.clone(), state].concat();
        let reference_controls = vec![0.0; 2 * ScvxRigidBodyControlLayout::DIMENSION];
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            2.0,
            [1.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.0],
        );
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        let segments = linearize_scvx_forward_euler_foh(
            ScvxRigidBodyStateLayout::DIMENSION,
            ScvxRigidBodyControlLayout::DIMENSION,
            &[0.0, 0.1],
            &reference_states,
            &reference_controls,
            |_time_s, state, control, output| {
                evaluate_scvx_rigid_body_dynamics(
                    state,
                    control,
                    &parameters,
                    dynamics_settings,
                    output,
                )
            },
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        assert_eq!(segments.len(), 1);
        let segment = &segments[0];
        let px = ScvxRigidBodyStateLayout::POSITION_ECI;
        let vx = ScvxRigidBodyStateLayout::VELOCITY_ECI;
        let force_x = ScvxRigidBodyControlLayout::FORCE_ECI_N;
        assert!(
            (segment.state_transition[px * ScvxRigidBodyStateLayout::DIMENSION + vx] - 0.1).abs()
                < 1.0e-8
        );
        assert!(
            (segment.control_left[vx * ScvxRigidBodyControlLayout::DIMENSION + force_x] - 0.025)
                .abs()
                < 1.0e-8
        );
        assert!(
            (segment.control_right[vx * ScvxRigidBodyControlLayout::DIMENSION + force_x] - 0.025)
                .abs()
                < 1.0e-8
        );

        Ok(())
    }

    #[test]
    fn scvx_builds_rigid_body_foh_problem_from_reference() -> Result<(), TrajoptError> {
        let state = rigid_body_state([10.0, 20.0, 30.0], [4.0, 5.0, 6.0], [0.0, 0.0, 0.0]);
        let definition = rigid_body_problem_definition(
            [state.clone(), state.clone()].concat(),
            vec![0.0; 2 * ScvxRigidBodyControlLayout::DIMENSION],
            state,
        );
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            2.0,
            [1.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.0],
        );

        let problem = build_scvx_rigid_body_foh_problem(
            &definition,
            &parameters,
            ScvxRigidBodyDynamicsSettings {
                quaternion_norm_tolerance: 1.0e-4,
            },
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        assert_eq!(problem.state_dimension, ScvxRigidBodyStateLayout::DIMENSION);
        assert_eq!(
            problem.control_dimension,
            ScvxRigidBodyControlLayout::DIMENSION
        );
        assert_eq!(problem.horizon_steps, 1);
        assert_eq!(problem.initial_state, definition.initial_state);
        assert_eq!(problem.target, definition.target);
        assert_eq!(problem.dynamics.len(), 1);

        let segment = &problem.dynamics[0];
        let px = ScvxRigidBodyStateLayout::POSITION_ECI;
        let vx = ScvxRigidBodyStateLayout::VELOCITY_ECI;
        let force_x = ScvxRigidBodyControlLayout::FORCE_ECI_N;
        assert!(
            (segment.state_transition[px * ScvxRigidBodyStateLayout::DIMENSION + vx] - 0.1).abs()
                < 1.0e-8
        );
        assert!(
            (segment.control_left[vx * ScvxRigidBodyControlLayout::DIMENSION + force_x] - 0.025)
                .abs()
                < 1.0e-8
        );

        Ok(())
    }

    #[test]
    fn scvx_builds_reduced_aero_rigid_body_foh_problem_with_loads() -> Result<(), TrajoptError> {
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![0.0; ScvxRigidBodyControlLayout::DIMENSION];
        let definition = rigid_body_problem_definition(
            [state.clone(), state.clone()].concat(),
            [control.clone(), control.clone()].concat(),
            state.clone(),
        );
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            2.0,
            [1.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.0],
        );
        let coefficients = ScvxReducedAeroCoefficients {
            cn: 0.1,
            cd: 0.2,
            cm: 0.3,
        };
        let aero_parameters = ScvxReducedAeroParameters {
            dynamic_pressure_pa: 10.0,
            reference_area_m2: 2.0,
            reference_length_m: 5.0,
        };
        let aero_profile = ScvxReducedAeroReferenceProfile {
            coefficients: vec![coefficients, coefficients],
            parameters: vec![aero_parameters, aero_parameters],
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };

        let problem = build_scvx_reduced_aero_rigid_body_foh_problem(
            &definition,
            &aero_profile,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        let predicted = predict_segment(
            &problem.dynamics[0],
            ScvxRigidBodyStateLayout::DIMENSION,
            ScvxRigidBodyControlLayout::DIMENSION,
            &state,
            &control,
            &control,
        );
        let mut derivative = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];
        evaluate_scvx_aero_coupled_rigid_body_dynamics(
            &state,
            &control,
            coefficients,
            aero_parameters,
            &parameters,
            dynamics_settings,
            &mut derivative,
        )?;
        for component in 0..ScvxRigidBodyStateLayout::DIMENSION {
            assert!(
                (predicted[component] - (state[component] + 0.1 * derivative[component])).abs()
                    < 1.0e-7
            );
        }
        let velocity = ScvxRigidBodyStateLayout::VELOCITY_ECI;
        assert!((predicted[velocity] + 0.2).abs() < 1.0e-7);
        assert!((predicted[velocity + 2] + 0.1).abs() < 1.0e-7);

        Ok(())
    }

    #[test]
    fn scvx_reduced_aero_powered_landing_dynamics_adds_aero_loads() -> Result<(), TrajoptError> {
        let state = powered_landing_state(2.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![6.0, 0.0, 0.0];
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [0.0, 0.0, 0.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0],
            thrust_application_point_body_m: [0.0, 0.0, 1.0],
            mass_flow_coefficient_s_m: 0.01,
        };
        let mut output = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];

        evaluate_scvx_reduced_aero_powered_landing_dynamics(
            &state,
            &control,
            ScvxReducedAeroCoefficients {
                cn: 0.1,
                cd: 0.2,
                cm: 0.3,
            },
            ScvxReducedAeroParameters {
                dynamic_pressure_pa: 10.0,
                reference_area_m2: 2.0,
                reference_length_m: 5.0,
            },
            &parameters,
            ScvxRigidBodyDynamicsSettings::default(),
            &mut output,
        )?;

        let mass = ScvxPoweredLandingStateLayout::MASS;
        let velocity = ScvxPoweredLandingStateLayout::VELOCITY_ECI;
        let angular_velocity = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
        assert!((output[mass] + 0.06).abs() < 1.0e-12);
        assert!((output[velocity] - 1.0).abs() < 1.0e-12);
        assert!(output[velocity + 1].abs() < 1.0e-12);
        assert!((output[velocity + 2] + 1.0).abs() < 1.0e-12);
        assert!(output[angular_velocity].abs() < 1.0e-12);
        assert!((output[angular_velocity + 1] - 18.0).abs() < 1.0e-12);
        assert!(output[angular_velocity + 2].abs() < 1.0e-12);

        Ok(())
    }

    #[test]
    fn scvx_builds_reduced_aero_powered_landing_foh_problem_with_loads() -> Result<(), TrajoptError>
    {
        let state = powered_landing_state(2.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![6.0, 0.0, 0.0];
        let definition = powered_landing_problem_definition(
            [state.clone(), state.clone()].concat(),
            [control.clone(), control.clone()].concat(),
            state.clone(),
        );
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [0.0, 0.0, 0.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0],
            thrust_application_point_body_m: [0.0, 0.0, 1.0],
            mass_flow_coefficient_s_m: 0.01,
        };
        let coefficients = ScvxReducedAeroCoefficients {
            cn: 0.1,
            cd: 0.2,
            cm: 0.3,
        };
        let aero_parameters = ScvxReducedAeroParameters {
            dynamic_pressure_pa: 10.0,
            reference_area_m2: 2.0,
            reference_length_m: 5.0,
        };
        let aero_profile = ScvxReducedAeroReferenceProfile {
            coefficients: vec![coefficients, coefficients],
            parameters: vec![aero_parameters, aero_parameters],
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };

        let problem = build_scvx_reduced_aero_powered_landing_foh_problem(
            &definition,
            &aero_profile,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        let predicted = predict_segment(
            &problem.dynamics[0],
            ScvxPoweredLandingStateLayout::DIMENSION,
            ScvxPoweredLandingControlLayout::DIMENSION,
            &state,
            &control,
            &control,
        );
        let mut derivative = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        evaluate_scvx_reduced_aero_powered_landing_dynamics(
            &state,
            &control,
            coefficients,
            aero_parameters,
            &parameters,
            dynamics_settings,
            &mut derivative,
        )?;
        for component in 0..ScvxPoweredLandingStateLayout::DIMENSION {
            assert!(
                (predicted[component] - (state[component] + 0.1 * derivative[component])).abs()
                    < 1.0e-7
            );
        }
        let velocity = ScvxPoweredLandingStateLayout::VELOCITY_ECI;
        let angular_velocity = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
        assert!((predicted[velocity] - 0.1).abs() < 1.0e-7);
        assert!((predicted[velocity + 2] + 0.1).abs() < 1.0e-7);
        assert!((predicted[angular_velocity + 1] - 1.8).abs() < 1.0e-7);

        Ok(())
    }

    #[test]
    fn scvx_foh_problem_builders_reject_bad_inputs() {
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let mut definition = rigid_body_problem_definition(
            [state.clone(), state.clone()].concat(),
            vec![0.0; 2 * ScvxRigidBodyControlLayout::DIMENSION],
            state,
        );
        definition.control_dimension = ScvxRigidBodyControlLayout::DIMENSION - 1;
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            1.0,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        );
        assert!(matches!(
            build_scvx_rigid_body_foh_problem(
                &definition,
                &parameters,
                ScvxRigidBodyDynamicsSettings::default(),
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        definition.control_dimension = ScvxRigidBodyControlLayout::DIMENSION;
        let aero_profile = ScvxReducedAeroReferenceProfile {
            coefficients: vec![ScvxReducedAeroCoefficients::default()],
            parameters: vec![ScvxReducedAeroParameters {
                dynamic_pressure_pa: 0.0,
                reference_area_m2: 1.0,
                reference_length_m: 1.0,
            }],
        };
        assert!(matches!(
            build_scvx_reduced_aero_rigid_body_foh_problem(
                &definition,
                &aero_profile,
                &parameters,
                ScvxRigidBodyDynamicsSettings {
                    quaternion_norm_tolerance: 1.0e-4,
                },
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let powered_state =
            powered_landing_state(2.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let powered_definition = powered_landing_problem_definition(
            [powered_state.clone(), powered_state.clone()].concat(),
            vec![0.0; 2 * ScvxPoweredLandingControlLayout::DIMENSION],
            powered_state,
        );
        let powered_parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [0.0, 0.0, -1.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            thrust_application_point_body_m: [0.0, 0.0, 0.0],
            mass_flow_coefficient_s_m: 0.0,
        };
        assert!(matches!(
            build_scvx_reduced_aero_powered_landing_foh_problem(
                &powered_definition,
                &aero_profile,
                &powered_parameters,
                ScvxRigidBodyDynamicsSettings {
                    quaternion_norm_tolerance: 1.0e-4,
                },
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[test]
    fn scvx_reduced_aero_force_moment_matches_schema1_mapping_and_rotates_to_eci()
    -> Result<(), TrajoptError> {
        let mut state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let half_sqrt = 0.5_f64.sqrt();
        state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = half_sqrt;
        state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 1] = 0.0;
        state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 2] = 0.0;
        state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 3] = half_sqrt;

        let result = evaluate_scvx_reduced_aero_force_moment(
            &state,
            ScvxReducedAeroCoefficients {
                cn: 2.0,
                cd: 0.5,
                cm: 0.1,
            },
            ScvxReducedAeroParameters {
                dynamic_pressure_pa: 100.0,
                reference_area_m2: 2.0,
                reference_length_m: 3.0,
            },
            ScvxRigidBodyDynamicsSettings::default(),
        )?;

        assert!((result.force_n_body[0] + 100.0).abs() < 1.0e-12);
        assert!(result.force_n_body[1].abs() < 1.0e-12);
        assert!((result.force_n_body[2] + 400.0).abs() < 1.0e-12);
        assert!(result.force_n_eci[0].abs() < 1.0e-10);
        assert!((result.force_n_eci[1] + 100.0).abs() < 1.0e-10);
        assert!((result.force_n_eci[2] + 400.0).abs() < 1.0e-12);
        assert!(result.moment_n_m_body[0].abs() < 1.0e-12);
        assert!((result.moment_n_m_body[1] - 60.0).abs() < 1.0e-12);
        assert!(result.moment_n_m_body[2].abs() < 1.0e-12);
        assert_eq!(
            result.rigid_body_control[0].to_bits(),
            result.force_n_eci[0].to_bits()
        );
        assert_eq!(
            result.rigid_body_control[4].to_bits(),
            result.moment_n_m_body[1].to_bits()
        );

        Ok(())
    }

    #[test]
    fn scvx_aero_coupled_rigid_body_dynamics_adds_aero_loads() -> Result<(), TrajoptError> {
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            2.0,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        );
        let mut output = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];

        evaluate_scvx_aero_coupled_rigid_body_dynamics(
            &state,
            &control,
            ScvxReducedAeroCoefficients {
                cn: 0.1,
                cd: 0.2,
                cm: 0.3,
            },
            ScvxReducedAeroParameters {
                dynamic_pressure_pa: 10.0,
                reference_area_m2: 2.0,
                reference_length_m: 5.0,
            },
            &parameters,
            ScvxRigidBodyDynamicsSettings::default(),
            &mut output,
        )?;

        let velocity = ScvxRigidBodyStateLayout::VELOCITY_ECI;
        let angular_velocity = ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY;
        assert!((output[velocity] + 1.5).abs() < 1.0e-12);
        assert!((output[velocity + 1] - 1.0).abs() < 1.0e-12);
        assert!((output[velocity + 2] - 0.5).abs() < 1.0e-12);
        assert!((output[angular_velocity] - 4.0).abs() < 1.0e-12);
        assert!((output[angular_velocity + 1] - 35.0).abs() < 1.0e-12);
        assert!((output[angular_velocity + 2] - 6.0).abs() < 1.0e-12);

        Ok(())
    }

    #[test]
    fn scvx_reduced_aero_rejects_bad_inputs() {
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        assert!(matches!(
            evaluate_scvx_reduced_aero_force_moment(
                &state,
                ScvxReducedAeroCoefficients {
                    cn: 0.0,
                    cd: 0.0,
                    cm: 0.0,
                },
                ScvxReducedAeroParameters {
                    dynamic_pressure_pa: -1.0,
                    reference_area_m2: 1.0,
                    reference_length_m: 1.0,
                },
                ScvxRigidBodyDynamicsSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));
    }

    #[cfg(feature = "aero")]
    #[test]
    fn scvx_aero_method_bridge_consumes_deck_lookup() -> Result<(), TrajoptError> {
        let deck = openbmp_aero::AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![0.0],
            vec![0.1, 0.1, 2.0, 2.0],
            vec![0.5, 0.5, 0.5, 0.5],
            vec![0.0, 0.0, 0.2, 0.2],
            2.0,
            3.0,
        )
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "test aero deck should be valid",
        })?;
        let method = openbmp_aero::DeckLookup::new(deck);
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);

        let result = evaluate_scvx_aero_method_force_moment(
            &state,
            &method,
            &openbmp_aero::AeroContext {
                mach: 1.0,
                alpha_deg: 1.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 100.0,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            },
            ScvxRigidBodyDynamicsSettings::default(),
        )?;

        assert!((result.force_n_body[0] + 100.0).abs() < 1.0e-12);
        assert!((result.force_n_body[2] + 400.0).abs() < 1.0e-12);
        assert_eq!(
            result.force_n_body[0].to_bits(),
            result.force_n_eci[0].to_bits()
        );
        assert!((result.moment_n_m_body[1] - 120.0).abs() < 1.0e-12);

        Ok(())
    }

    #[cfg(feature = "aero")]
    #[test]
    fn scvx_aero_method_coupled_dynamics_adds_deck_loads() -> Result<(), TrajoptError> {
        let deck = openbmp_aero::AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![0.0],
            vec![0.1, 0.1, 0.2, 0.2],
            vec![0.3, 0.3, 0.3, 0.3],
            vec![0.4, 0.4, 0.4, 0.4],
            2.0,
            5.0,
        )
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "test aero deck should be valid",
        })?;
        let method = openbmp_aero::DeckLookup::new(deck);
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![1.0, 2.0, 3.0, 4.0, 5.0, 6.0];
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            2.0,
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        );
        let mut output = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];

        evaluate_scvx_aero_method_coupled_rigid_body_dynamics(
            &state,
            &control,
            &method,
            &openbmp_aero::AeroContext {
                mach: 1.0,
                alpha_deg: 1.0,
                beta_deg: 0.0,
                dynamic_pressure_pa: 10.0,
                airspeed_m_s: 100.0,
                body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
                knudsen: 0.0,
                reynolds_length: 0.0,
            },
            &parameters,
            ScvxRigidBodyDynamicsSettings::default(),
            &mut output,
        )?;

        let velocity = ScvxRigidBodyStateLayout::VELOCITY_ECI;
        let angular_velocity = ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY;
        assert!((output[velocity] + 2.5).abs() < 1.0e-12);
        assert!((output[velocity + 1] - 1.0).abs() < 1.0e-12);
        assert!((output[velocity + 2] + 0.5).abs() < 1.0e-12);
        assert!((output[angular_velocity + 1] - 45.0).abs() < 1.0e-12);

        Ok(())
    }

    #[cfg(feature = "aero")]
    #[test]
    fn scvx_aero_method_powered_landing_dynamics_adds_deck_loads() -> Result<(), TrajoptError> {
        let deck = openbmp_aero::AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![0.0],
            vec![0.1, 0.1, 0.2, 0.2],
            vec![0.3, 0.3, 0.3, 0.3],
            vec![0.4, 0.4, 0.4, 0.4],
            2.0,
            5.0,
        )
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "test aero deck should be valid",
        })?;
        let method = openbmp_aero::DeckLookup::new(deck);
        let state = powered_landing_state(2.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![6.0, 0.0, 0.0];
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [0.0, 0.0, 0.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0],
            thrust_application_point_body_m: [0.0, 0.0, 1.0],
            mass_flow_coefficient_s_m: 0.01,
        };
        let context = openbmp_aero::AeroContext {
            mach: 1.0,
            alpha_deg: 1.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 10.0,
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
        };
        let mut output = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];

        evaluate_scvx_aero_method_powered_landing_dynamics(
            &state,
            &control,
            &method,
            &context,
            &parameters,
            ScvxRigidBodyDynamicsSettings::default(),
            &mut output,
        )?;

        let mass = ScvxPoweredLandingStateLayout::MASS;
        let velocity = ScvxPoweredLandingStateLayout::VELOCITY_ECI;
        let angular_velocity = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
        assert!((output[mass] + 0.06).abs() < 1.0e-12);
        assert!(output[velocity].abs() < 1.0e-12);
        assert!(output[velocity + 1].abs() < 1.0e-12);
        assert!((output[velocity + 2] + 2.0).abs() < 1.0e-12);
        assert!(output[angular_velocity].abs() < 1.0e-12);
        assert!((output[angular_velocity + 1] - 23.0).abs() < 1.0e-12);
        assert!(output[angular_velocity + 2].abs() < 1.0e-12);

        Ok(())
    }

    #[cfg(feature = "aero")]
    #[test]
    fn scvx_aero_method_powered_landing_foh_problem_builder_consumes_deck_lookup()
    -> Result<(), TrajoptError> {
        let deck = openbmp_aero::AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![0.0],
            vec![0.1, 0.1, 0.2, 0.2],
            vec![0.3, 0.3, 0.3, 0.3],
            vec![0.4, 0.4, 0.4, 0.4],
            2.0,
            5.0,
        )
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "test aero deck should be valid",
        })?;
        let method = openbmp_aero::DeckLookup::new(deck);
        let state = powered_landing_state(2.0, [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![6.0, 0.0, 0.0];
        let definition = powered_landing_problem_definition(
            [state.clone(), state.clone()].concat(),
            [control.clone(), control.clone()].concat(),
            state.clone(),
        );
        let context = openbmp_aero::AeroContext {
            mach: 1.0,
            alpha_deg: 1.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 10.0,
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
        };
        let aero_profile = ScvxAeroMethodReferenceProfile {
            contexts: vec![context, context],
        };
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: [0.0, 0.0, 0.0],
            inertia_body_kg_m2: [1.0, 0.0, 0.0, 0.0, 2.0, 0.0, 0.0, 0.0, 4.0],
            thrust_application_point_body_m: [0.0, 0.0, 1.0],
            mass_flow_coefficient_s_m: 0.01,
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };

        let problem = build_scvx_aero_method_powered_landing_foh_problem(
            &definition,
            &method,
            &aero_profile,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        let predicted = predict_segment(
            &problem.dynamics[0],
            ScvxPoweredLandingStateLayout::DIMENSION,
            ScvxPoweredLandingControlLayout::DIMENSION,
            &state,
            &control,
            &control,
        );
        let mut derivative = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        evaluate_scvx_aero_method_powered_landing_dynamics(
            &state,
            &control,
            &method,
            &context,
            &parameters,
            dynamics_settings,
            &mut derivative,
        )?;
        for component in 0..ScvxPoweredLandingStateLayout::DIMENSION {
            assert!(
                (predicted[component] - (state[component] + 0.1 * derivative[component])).abs()
                    < 1.0e-7
            );
        }
        let velocity = ScvxPoweredLandingStateLayout::VELOCITY_ECI;
        let angular_velocity = ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY;
        assert!(predicted[velocity].abs() < 1.0e-7);
        assert!((predicted[velocity + 2] + 0.2).abs() < 1.0e-7);
        assert!((predicted[angular_velocity + 1] - 2.3).abs() < 1.0e-7);

        let malformed_profile = ScvxAeroMethodReferenceProfile {
            contexts: vec![context],
        };
        assert!(matches!(
            build_scvx_aero_method_powered_landing_foh_problem(
                &definition,
                &method,
                &malformed_profile,
                &parameters,
                dynamics_settings,
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        Ok(())
    }

    #[cfg(feature = "aero")]
    #[test]
    fn scvx_aero_method_foh_problem_builder_consumes_deck_lookup() -> Result<(), TrajoptError> {
        let deck = openbmp_aero::AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![0.0],
            vec![0.1, 0.1, 0.2, 0.2],
            vec![0.3, 0.3, 0.3, 0.3],
            vec![0.4, 0.4, 0.4, 0.4],
            2.0,
            5.0,
        )
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "test aero deck should be valid",
        })?;
        let method = openbmp_aero::DeckLookup::new(deck);
        let state = rigid_body_state([0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
        let control = vec![0.0; ScvxRigidBodyControlLayout::DIMENSION];
        let definition = rigid_body_problem_definition(
            [state.clone(), state.clone()].concat(),
            [control.clone(), control.clone()].concat(),
            state.clone(),
        );
        let context = openbmp_aero::AeroContext {
            mach: 1.0,
            alpha_deg: 1.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: 10.0,
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
        };
        let aero_profile = ScvxAeroMethodReferenceProfile {
            contexts: vec![context, context],
        };
        let parameters = ScvxRigidBodyDynamicsParameters::new(
            2.0,
            [1.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 2.0],
        );
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };

        let problem = build_scvx_aero_method_rigid_body_foh_problem(
            &definition,
            &method,
            &aero_profile,
            &parameters,
            dynamics_settings,
            ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        )?;

        let predicted = predict_segment(
            &problem.dynamics[0],
            ScvxRigidBodyStateLayout::DIMENSION,
            ScvxRigidBodyControlLayout::DIMENSION,
            &state,
            &control,
            &control,
        );
        let mut derivative = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];
        evaluate_scvx_aero_method_coupled_rigid_body_dynamics(
            &state,
            &control,
            &method,
            &context,
            &parameters,
            dynamics_settings,
            &mut derivative,
        )?;
        for component in 0..ScvxRigidBodyStateLayout::DIMENSION {
            assert!(
                (predicted[component] - (state[component] + 0.1 * derivative[component])).abs()
                    < 1.0e-7
            );
        }
        let velocity = ScvxRigidBodyStateLayout::VELOCITY_ECI;
        assert!((predicted[velocity] + 0.3).abs() < 1.0e-7);
        assert!((predicted[velocity + 2] + 0.2).abs() < 1.0e-7);

        let malformed_profile = ScvxAeroMethodReferenceProfile {
            contexts: vec![context],
        };
        assert!(matches!(
            build_scvx_aero_method_rigid_body_foh_problem(
                &definition,
                &method,
                &malformed_profile,
                &parameters,
                dynamics_settings,
                ScvxForwardEulerLinearizationSettings::default(),
            ),
            Err(TrajoptError::InvalidPayload { .. })
        ));

        Ok(())
    }

    fn scalar_integrator_problem(target: f64) -> ScvxDiscreteAffineProblem {
        let horizon_steps = 4;
        let dt = 1.0 / horizon_steps as f64;
        ScvxDiscreteAffineProblem {
            state_dimension: 1,
            control_dimension: 1,
            horizon_steps,
            initial_state: vec![0.0],
            target: ScvxTerminalStateTarget {
                state: vec![target],
                weights: None,
            },
            dynamics: (0..horizon_steps)
                .map(|_| ScvxAffineDynamicsSegment {
                    state_transition: vec![1.0],
                    control_left: vec![0.5 * dt],
                    control_right: vec![0.5 * dt],
                    affine: vec![0.0],
                })
                .collect(),
            initial_states: (0..=horizon_steps)
                .map(|node| target * node as f64 / horizon_steps as f64)
                .collect(),
            initial_controls: vec![0.0; horizon_steps + 1],
            control_lower: vec![-10.0],
            control_upper: vec![10.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        }
    }

    fn scalar_integrator_time_problem(
        time_of_flight_s: f64,
    ) -> Result<ScvxDiscreteAffineProblem, TrajoptError> {
        if !time_of_flight_s.is_finite() || time_of_flight_s <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "test SCvx time of flight must be positive",
            });
        }
        let target = 1.0;
        let horizon_steps = 4;
        let dt = time_of_flight_s / horizon_steps as f64;
        Ok(ScvxDiscreteAffineProblem {
            state_dimension: 1,
            control_dimension: 1,
            horizon_steps,
            initial_state: vec![0.0],
            target: ScvxTerminalStateTarget {
                state: vec![target],
                weights: None,
            },
            dynamics: (0..horizon_steps)
                .map(|_| ScvxAffineDynamicsSegment {
                    state_transition: vec![1.0],
                    control_left: vec![0.5 * dt],
                    control_right: vec![0.5 * dt],
                    affine: vec![0.0],
                })
                .collect(),
            initial_states: (0..=horizon_steps)
                .map(|node| target * node as f64 / horizon_steps as f64)
                .collect(),
            initial_controls: vec![1.0 / time_of_flight_s; horizon_steps + 1],
            control_lower: vec![-20.0],
            control_upper: vec![20.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        })
    }

    fn scalar_integrator_foh_definition(
        time_of_flight_s: f64,
    ) -> Result<ScvxForwardEulerFohProblemDefinition, TrajoptError> {
        if !time_of_flight_s.is_finite() || time_of_flight_s <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "test continuous FOH time of flight must be positive",
            });
        }
        let target = 1.0;
        let horizon_steps = 4;
        Ok(ScvxForwardEulerFohProblemDefinition {
            state_dimension: 1,
            control_dimension: 1,
            times_s: (0..=horizon_steps)
                .map(|node| time_of_flight_s * node as f64 / horizon_steps as f64)
                .collect(),
            initial_state: vec![0.0],
            target: ScvxTerminalStateTarget {
                state: vec![target],
                weights: None,
            },
            initial_states: (0..=horizon_steps)
                .map(|node| target * node as f64 / horizon_steps as f64)
                .collect(),
            initial_controls: vec![1.0 / time_of_flight_s; horizon_steps + 1],
            control_lower: vec![-20.0],
            control_upper: vec![20.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        })
    }

    fn solve_scalar_integrator_continuous_foh_time_candidate(
        time_of_flight_s: f64,
        settings: ScvxContinuousFohSettings,
    ) -> Result<ScvxContinuousFohSolution, TrajoptError> {
        let definition = scalar_integrator_foh_definition(time_of_flight_s)?;
        solve_scvx_forward_euler_foh(
            &definition,
            |_time_s, _state, control, output| {
                output[0] = control[0];
                Ok(())
            },
            settings,
        )
    }

    fn weighted_terminal_problem() -> ScvxDiscreteAffineProblem {
        ScvxDiscreteAffineProblem {
            state_dimension: 2,
            control_dimension: 2,
            horizon_steps: 1,
            initial_state: vec![0.0, 0.0],
            target: ScvxTerminalStateTarget {
                state: vec![10.0, 1.0],
                weights: Some(vec![0.0, 1.0]),
            },
            dynamics: vec![ScvxAffineDynamicsSegment {
                state_transition: vec![1.0, 0.0, 0.0, 1.0],
                control_left: vec![0.5, 0.0, 0.0, 0.5],
                control_right: vec![0.5, 0.0, 0.0, 0.5],
                affine: vec![0.0, 0.0],
            }],
            initial_states: vec![0.0, 0.0, 0.0, 1.0],
            initial_controls: vec![0.0, 0.0, 0.0, 0.0],
            control_lower: vec![-10.0, -10.0],
            control_upper: vec![10.0, 10.0],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        }
    }

    fn scvx_time_search_objective_at(
        time_of_flight_s: f64,
        settings: ScvxTimeOfFlightSearchSettings,
    ) -> Result<f64, TrajoptError> {
        let problem = scalar_integrator_time_problem(time_of_flight_s)?;
        let solution = solve_scvx_discrete_affine(&problem, settings.scvx_settings)?;
        Ok(solution.merit + settings.time_penalty_weight * time_of_flight_s)
    }

    fn state_triggered_problem(target: [f64; 2]) -> ScvxDiscreteAffineProblem {
        ScvxDiscreteAffineProblem {
            state_dimension: 2,
            control_dimension: 2,
            horizon_steps: 1,
            initial_state: vec![0.0, 0.0],
            target: ScvxTerminalStateTarget {
                state: target.to_vec(),
                weights: None,
            },
            dynamics: vec![ScvxAffineDynamicsSegment {
                state_transition: vec![1.0, 0.0, 0.0, 1.0],
                control_left: vec![0.5, 0.0, 0.0, 0.5],
                control_right: vec![0.5, 0.0, 0.0, 0.5],
                affine: vec![0.0, 0.0],
            }],
            initial_states: vec![0.0, 0.0, target[0], target[1]],
            initial_controls: vec![0.0, 0.0, 0.0, 0.0],
            control_lower: vec![-10.0, -10.0],
            control_upper: vec![10.0, 10.0],
            state_triggered_constraints: vec![ScvxStateTriggeredConstraint {
                node: 1,
                trigger: ScvxLinearStateFunction {
                    coefficients: vec![1.0, 0.0],
                    offset: 0.0,
                },
                constraint: ScvxLinearStateFunction {
                    coefficients: vec![0.0, 1.0],
                    offset: -0.5,
                },
            }],
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        }
    }

    fn fast_settings() -> ScvxSettings {
        ScvxSettings {
            max_iterations: 4,
            initial_trust_region_radius: 10.0,
            max_trust_region_radius: 20.0,
            terminal_tolerance: 1.0e-6,
            virtual_control_tolerance: 1.0e-6,
            ..ScvxSettings::default()
        }
    }

    fn time_search_settings() -> ScvxTimeOfFlightSearchSettings {
        ScvxTimeOfFlightSearchSettings {
            lower_time_of_flight_s: 0.8,
            upper_time_of_flight_s: 4.0,
            tolerance_s: 1.0e-3,
            max_iterations: 8,
            time_penalty_weight: 1.0,
            scvx_settings: ScvxSettings {
                max_iterations: 4,
                initial_trust_region_radius: 20.0,
                max_trust_region_radius: 40.0,
                terminal_tolerance: 1.0e-6,
                virtual_control_tolerance: 1.0e-6,
                terminal_weight: 1.0e7,
                control_weight: 1.0,
                ..ScvxSettings::default()
            },
        }
    }

    fn continuous_foh_time_search_settings() -> ScvxContinuousFohTimeOfFlightSearchSettings {
        ScvxContinuousFohTimeOfFlightSearchSettings {
            lower_time_of_flight_s: 0.8,
            upper_time_of_flight_s: 4.0,
            tolerance_s: 1.0e-3,
            max_iterations: 8,
            time_penalty_weight: 1.0,
        }
    }

    fn continuous_foh_time_sweep_settings() -> ScvxContinuousFohTimeOfFlightSweepSettings {
        ScvxContinuousFohTimeOfFlightSweepSettings {
            center_times_of_flight_s: vec![0.9, 1.2],
            half_width_s: 0.05,
            tolerance_s: 1.0e-3,
            max_iterations: 2,
            time_penalty_weight: 1.0,
        }
    }

    fn continuous_foh_time_candidate_settings() -> ScvxContinuousFohSettings {
        ScvxContinuousFohSettings {
            max_outer_iterations: 4,
            reference_update_tolerance: 1.0e-6,
            scvx_settings: time_search_settings().scvx_settings,
            linearization_settings: ScvxForwardEulerLinearizationSettings {
                finite_difference_step: 1.0e-6,
            },
        }
    }

    fn triggered_settings() -> ScvxSettings {
        ScvxSettings {
            max_iterations: 3,
            initial_trust_region_radius: 10.0,
            max_trust_region_radius: 20.0,
            terminal_tolerance: 1.0e-6,
            virtual_control_tolerance: 1.0e-6,
            state_triggered_constraint_penalty: 1.0e10,
            ..ScvxSettings::default()
        }
    }

    fn szmuk_benchmark_fixture() -> SzmukBenchmarkFixture {
        let fixture: SzmukBenchmarkFixture = toml::from_str(SZMUK_ACIKMESE_6DOF_FREE_FINAL_TIME)
            .expect("Szmuk benchmark fixture parses");
        assert!(fixture.vehicle.wet_mass.is_finite() && fixture.vehicle.wet_mass > 0.0);
        assert!(fixture.vehicle.dry_mass.is_finite() && fixture.vehicle.dry_mass > 0.0);
        assert!(fixture.vehicle.dry_mass < fixture.vehicle.wet_mass);
        assert!(fixture.vehicle.minimum_thrust > 0.0);
        assert!(fixture.vehicle.maximum_thrust > fixture.vehicle.minimum_thrust);
        assert!(
            fixture
                .vehicle
                .inertia_body
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            fixture
                .vehicle
                .thrust_application_point_body
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            fixture
                .environment
                .gravity
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(fixture.algorithm.discretization_points >= 2);
        assert_eq!(fixture.algorithm.initial_time_guesses.len(), 10);
        assert!(
            fixture
                .algorithm
                .initial_time_guesses
                .windows(2)
                .all(|window| window[1] > window[0])
        );
        assert!(
            fixture
                .boundary_conditions
                .initial_position
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            fixture
                .boundary_conditions
                .in_plane_initial_velocity
                .iter()
                .all(|value| value.is_finite())
        );
        assert!(
            fixture
                .boundary_conditions
                .final_velocity
                .iter()
                .all(|value| value.is_finite())
        );
        fixture
    }

    fn szmuk_in_plane_powered_landing_definition(
        fixture: &SzmukBenchmarkFixture,
        initial_time_guess: f64,
    ) -> Result<ScvxForwardEulerFohProblemDefinition, TrajoptError> {
        if !initial_time_guess.is_finite() || initial_time_guess <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Szmuk initial time guess must be positive",
            });
        }
        let node_count = fixture.algorithm.discretization_points;
        let horizon_steps = node_count - 1;
        let mut times_s = Vec::with_capacity(node_count);
        let mut initial_states =
            Vec::with_capacity(node_count * ScvxPoweredLandingStateLayout::DIMENSION);
        let mut initial_controls =
            Vec::with_capacity(node_count * ScvxPoweredLandingControlLayout::DIMENSION);

        for node in 0..node_count {
            let fraction = node as f64 / horizon_steps as f64;
            times_s.push(initial_time_guess * fraction);
            let alpha_start = 1.0 - fraction;
            let alpha_final = fraction;
            let mut state = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
            state[ScvxPoweredLandingStateLayout::MASS] =
                alpha_start * fixture.vehicle.wet_mass + alpha_final * fixture.vehicle.dry_mass;
            for component in 0..3 {
                state[ScvxPoweredLandingStateLayout::POSITION_ECI + component] =
                    alpha_start * fixture.boundary_conditions.initial_position[component];
                state[ScvxPoweredLandingStateLayout::VELOCITY_ECI + component] = alpha_start
                    * fixture.boundary_conditions.in_plane_initial_velocity[component]
                    + alpha_final * fixture.boundary_conditions.final_velocity[component];
                state[ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY + component] =
                    fixture.boundary_conditions.initial_angular_velocity[component];
            }
            state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ
                ..ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 4]
                .copy_from_slice(
                    &fixture
                        .boundary_conditions
                        .final_quaternion_body_to_inertial_wxyz,
                );

            let mass = state[ScvxPoweredLandingStateLayout::MASS];
            initial_controls.extend(
                fixture
                    .environment
                    .gravity
                    .iter()
                    .map(|component| -mass * component),
            );
            initial_states.extend(state);
        }

        let mut target = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        target[ScvxPoweredLandingStateLayout::MASS] = fixture.vehicle.dry_mass;
        target[ScvxPoweredLandingStateLayout::VELOCITY_ECI
            ..ScvxPoweredLandingStateLayout::VELOCITY_ECI + 3]
            .copy_from_slice(&fixture.boundary_conditions.final_velocity);
        target[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ
            ..ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ + 4]
            .copy_from_slice(
                &fixture
                    .boundary_conditions
                    .final_quaternion_body_to_inertial_wxyz,
            );

        Ok(ScvxForwardEulerFohProblemDefinition {
            state_dimension: ScvxPoweredLandingStateLayout::DIMENSION,
            control_dimension: ScvxPoweredLandingControlLayout::DIMENSION,
            times_s,
            initial_state: initial_states[..ScvxPoweredLandingStateLayout::DIMENSION].to_vec(),
            target: ScvxTerminalStateTarget {
                state: target,
                weights: Some(powered_landing_terminal_weights(false)),
            },
            initial_states,
            initial_controls,
            control_lower: vec![
                -fixture.vehicle.maximum_thrust;
                ScvxPoweredLandingControlLayout::DIMENSION
            ],
            control_upper: vec![
                fixture.vehicle.maximum_thrust;
                ScvxPoweredLandingControlLayout::DIMENSION
            ],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        })
    }

    fn szmuk_in_plane_powered_landing_conic_envelope_definition(
        fixture: &SzmukBenchmarkFixture,
        initial_time_guess: f64,
    ) -> Result<ScvxForwardEulerFohProblemDefinition, TrajoptError> {
        let mut definition =
            szmuk_in_plane_powered_landing_definition(fixture, initial_time_guess)?;
        refresh_szmuk_powered_landing_conic_envelope_constraints(fixture, &mut definition)?;
        Ok(definition)
    }

    fn assert_szmuk_continuous_conic_candidate_reports_reference_gap(
        candidate: &ScvxContinuousFohTimeOfFlightCandidateReport,
        time_of_flight_s: f64,
    ) {
        assert_eq!(
            candidate.time_of_flight_s.to_bits(),
            time_of_flight_s.to_bits()
        );
        assert!(!candidate.converged);
        assert!(candidate.affine_converged);
        assert_eq!(candidate.outer_iteration_count, 2);
        assert!(candidate.affine_iteration_count > 0);
        assert_eq!(
            candidate.termination_status,
            ScvxTerminationStatus::Converged
        );
        assert!(candidate.objective.is_finite());
        assert!(candidate.terminal_error_norm <= 1.0e-4);
        assert!(candidate.max_virtual_control_norm <= 1.0e-1);
        assert!(candidate.max_linear_state_violation <= 1.0e-7);
        assert!(candidate.max_linear_control_violation <= 1.0e-7);
        assert!(candidate.max_second_order_cone_violation <= 1.0e-7);
        assert!(candidate.max_state_update_norm > 1.0e-9);
    }

    fn solve_szmuk_in_plane_conic_continuous_foh_candidate(
        fixture: &SzmukBenchmarkFixture,
        time_of_flight_s: f64,
    ) -> Result<ScvxContinuousFohSolution, TrajoptError> {
        let definition = szmuk_in_plane_powered_landing_definition(fixture, time_of_flight_s)?;
        let parameters = ScvxPoweredLandingDynamicsParameters {
            gravity_eci_m_s2: fixture.environment.gravity,
            inertia_body_kg_m2: fixture.vehicle.inertia_body,
            thrust_application_point_body_m: fixture.vehicle.thrust_application_point_body,
            mass_flow_coefficient_s_m: szmuk_reference_mass_flow_coefficient(
                fixture,
                time_of_flight_s,
            ),
        };
        let dynamics_settings = ScvxRigidBodyDynamicsSettings {
            quaternion_norm_tolerance: 1.0e-4,
        };
        solve_scvx_forward_euler_foh_with_reference_update(
            &definition,
            |_time_s, state, control, output| {
                evaluate_scvx_powered_landing_dynamics(
                    state,
                    control,
                    &parameters,
                    dynamics_settings,
                    output,
                )
            },
            ScvxContinuousFohSettings {
                max_outer_iterations: 2,
                reference_update_tolerance: 1.0e-9,
                scvx_settings: ScvxSettings {
                    max_iterations: fixture.algorithm.maximum_iterations,
                    terminal_tolerance: 1.0e-4,
                    virtual_control_tolerance: 1.0e-1,
                    constraint_tolerance: 1.0e-7,
                    initial_trust_region_radius: 20.0,
                    max_trust_region_radius: 40.0,
                    terminal_weight: 1.0e8,
                    virtual_control_weight: fixture.algorithm.virtual_control_weight,
                    control_weight: fixture.algorithm.state_control_trust_region_weight,
                    ..ScvxSettings::default()
                },
                linearization_settings: ScvxForwardEulerLinearizationSettings::default(),
            },
            |reference| {
                normalize_scvx_powered_landing_quaternion_reference_states(
                    &mut reference.initial_states,
                )?;
                refresh_szmuk_powered_landing_conic_envelope_constraints(fixture, reference)
            },
        )
    }

    fn refresh_szmuk_powered_landing_conic_envelope_constraints(
        fixture: &SzmukBenchmarkFixture,
        definition: &mut ScvxForwardEulerFohProblemDefinition,
    ) -> Result<(), TrajoptError> {
        let conic_constraints = build_scvx_powered_landing_conic_envelope_constraints(
            &definition.initial_states,
            szmuk_powered_landing_envelope_limits(fixture),
            ScvxPoweredLandingConicEnvelopeSettings {
                finite_difference_step: 1.0e-6,
                quaternion_norm_tolerance: 1.0e-4,
            },
        )?;
        definition.linear_state_constraints = conic_constraints.linear_state_constraints;
        definition.linear_control_constraints = conic_constraints.linear_control_constraints;
        definition.second_order_state_constraints =
            conic_constraints.second_order_state_constraints;
        definition.second_order_control_constraints =
            conic_constraints.second_order_control_constraints;
        Ok(())
    }

    fn szmuk_powered_landing_envelope_limits(
        fixture: &SzmukBenchmarkFixture,
    ) -> ScvxPoweredLandingEnvelopeLimits {
        ScvxPoweredLandingEnvelopeLimits {
            minimum_mass: fixture.vehicle.dry_mass,
            maximum_mass: fixture.vehicle.wet_mass,
            minimum_thrust: fixture.vehicle.minimum_thrust,
            maximum_thrust: fixture.vehicle.maximum_thrust,
            maximum_gimbal_angle_rad: fixture.vehicle.maximum_gimbal_angle_deg.to_radians(),
            maximum_tilt_angle_rad: fixture.vehicle.maximum_tilt_angle_deg.to_radians(),
            maximum_angular_rate_rad_s: fixture
                .vehicle
                .maximum_angular_rate_deg_per_time_unit
                .to_radians(),
            glideslope_angle_rad: fixture.environment.glideslope_angle_deg.to_radians(),
            gravity_eci: fixture.environment.gravity,
        }
    }

    fn szmuk_reference_mass_flow_coefficient(
        fixture: &SzmukBenchmarkFixture,
        initial_time_guess: f64,
    ) -> f64 {
        let reference_thrust_integral =
            0.5 * (fixture.vehicle.wet_mass + fixture.vehicle.dry_mass) * initial_time_guess;
        (fixture.vehicle.wet_mass - fixture.vehicle.dry_mass) / reference_thrust_integral
    }

    fn rigid_body_problem_definition(
        initial_states: Vec<f64>,
        initial_controls: Vec<f64>,
        target: Vec<f64>,
    ) -> ScvxForwardEulerFohProblemDefinition {
        ScvxForwardEulerFohProblemDefinition {
            state_dimension: ScvxRigidBodyStateLayout::DIMENSION,
            control_dimension: ScvxRigidBodyControlLayout::DIMENSION,
            times_s: vec![0.0, 0.1],
            initial_state: initial_states[..ScvxRigidBodyStateLayout::DIMENSION].to_vec(),
            target: ScvxTerminalStateTarget {
                state: target,
                weights: None,
            },
            initial_states,
            initial_controls,
            control_lower: vec![-100.0; ScvxRigidBodyControlLayout::DIMENSION],
            control_upper: vec![100.0; ScvxRigidBodyControlLayout::DIMENSION],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        }
    }

    fn powered_landing_problem_definition(
        initial_states: Vec<f64>,
        initial_controls: Vec<f64>,
        target: Vec<f64>,
    ) -> ScvxForwardEulerFohProblemDefinition {
        ScvxForwardEulerFohProblemDefinition {
            state_dimension: ScvxPoweredLandingStateLayout::DIMENSION,
            control_dimension: ScvxPoweredLandingControlLayout::DIMENSION,
            times_s: vec![0.0, 0.1],
            initial_state: initial_states[..ScvxPoweredLandingStateLayout::DIMENSION].to_vec(),
            target: ScvxTerminalStateTarget {
                state: target,
                weights: None,
            },
            initial_states,
            initial_controls,
            control_lower: vec![-100.0; ScvxPoweredLandingControlLayout::DIMENSION],
            control_upper: vec![100.0; ScvxPoweredLandingControlLayout::DIMENSION],
            state_triggered_constraints: Vec::new(),
            linear_state_constraints: Vec::new(),
            linear_control_constraints: Vec::new(),
            second_order_state_constraints: Vec::new(),
            second_order_control_constraints: Vec::new(),
        }
    }

    fn powered_landing_terminal_weights(target_mass: bool) -> Vec<f64> {
        let mut weights = vec![1.0; ScvxPoweredLandingStateLayout::DIMENSION];
        if !target_mass {
            weights[ScvxPoweredLandingStateLayout::MASS] = 0.0;
        }
        weights
    }

    fn predict_segment(
        segment: &ScvxAffineDynamicsSegment,
        state_dimension: usize,
        control_dimension: usize,
        state: &[f64],
        control_left: &[f64],
        control_right: &[f64],
    ) -> Vec<f64> {
        let mut predicted = segment.affine.clone();
        for (row, prediction) in predicted.iter_mut().enumerate().take(state_dimension) {
            for (col, value) in state.iter().enumerate().take(state_dimension) {
                *prediction += segment.state_transition[row * state_dimension + col] * value;
            }
            for (col, value) in control_left.iter().enumerate().take(control_dimension) {
                *prediction += segment.control_left[row * control_dimension + col] * value;
            }
            for (col, value) in control_right.iter().enumerate().take(control_dimension) {
                *prediction += segment.control_right[row * control_dimension + col] * value;
            }
        }
        predicted
    }

    fn rigid_body_state(position: [f64; 3], velocity: [f64; 3], omega: [f64; 3]) -> Vec<f64> {
        let mut state = vec![0.0; ScvxRigidBodyStateLayout::DIMENSION];
        state[ScvxRigidBodyStateLayout::POSITION_ECI..ScvxRigidBodyStateLayout::POSITION_ECI + 3]
            .copy_from_slice(&position);
        state[ScvxRigidBodyStateLayout::VELOCITY_ECI..ScvxRigidBodyStateLayout::VELOCITY_ECI + 3]
            .copy_from_slice(&velocity);
        state[ScvxRigidBodyStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = 1.0;
        state[ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY
            ..ScvxRigidBodyStateLayout::ANGULAR_VELOCITY_BODY + 3]
            .copy_from_slice(&omega);
        state
    }

    fn powered_landing_state(
        mass: f64,
        position: [f64; 3],
        velocity: [f64; 3],
        omega: [f64; 3],
    ) -> Vec<f64> {
        let mut state = vec![0.0; ScvxPoweredLandingStateLayout::DIMENSION];
        state[ScvxPoweredLandingStateLayout::MASS] = mass;
        state[ScvxPoweredLandingStateLayout::POSITION_ECI
            ..ScvxPoweredLandingStateLayout::POSITION_ECI + 3]
            .copy_from_slice(&position);
        state[ScvxPoweredLandingStateLayout::VELOCITY_ECI
            ..ScvxPoweredLandingStateLayout::VELOCITY_ECI + 3]
            .copy_from_slice(&velocity);
        state[ScvxPoweredLandingStateLayout::QUATERNION_BODY_TO_ECI_WXYZ] = 1.0;
        state[ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY
            ..ScvxPoweredLandingStateLayout::ANGULAR_VELOCITY_BODY + 3]
            .copy_from_slice(&omega);
        state
    }
}
