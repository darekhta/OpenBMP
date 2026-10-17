//! Pseudospectral collocation primitives for trajectory optimization.
//!
//! This module starts the WP-07.3 Legendre-Gauss-Radau substrate. It exposes
//! deterministic left-Radau grids on `[-1, 1]`: the collocation points include
//! `-1`, exclude `+1`, and carry `+1` as the terminal support point for the
//! barycentric differentiation matrix.

use alloc::collections::BTreeMap;
use alloc::vec;
use alloc::vec::Vec;

use crate::collocation::{
    CollocationAutoDiffDynamics, CollocationAutoDiffDynamicsAdapter, CollocationAutoDiffScalar,
    CollocationDynamics, CollocationDynamicsJacobian, CollocationDynamicsJacobianSample,
    CollocationLagrange, CollocationPathFunction, SparseKktSqpIterationReport,
    SparseKktSqpSettings, scaled_decision_step, sparse_kkt_sqp_default_decision_bounds,
};
use crate::iload::ILoadOptimalityEvidenceKind;
use crate::iload::{
    GainTable, ILoadHeader, ILoadObjectiveClass, ILoadOptimalityCertificate, ILoadPayload,
    ReferenceProfileSample, SynthesisMetadata, TrajoptError,
};
use openbmp_physics::profile::TerminalCondition;

/// Left Legendre-Gauss-Radau grid on `[-1, 1]`.
///
/// `collocation_nodes` has length `N` and includes `-1`. `support_nodes` has
/// length `N + 1` and appends the terminal support node `+1`. The
/// `differentiation_matrix` is row-major with `N` rows and `N + 1` columns, so
/// multiplying it by values at every support node gives polynomial derivatives
/// at the collocation nodes.
#[derive(Clone, Debug, PartialEq)]
pub struct LegendreGaussRadauGrid {
    /// LGR collocation nodes, including `-1` and excluding `+1`.
    pub collocation_nodes: Vec<f64>,
    /// Terminal support node, fixed at `+1`.
    pub endpoint: f64,
    /// Collocation nodes followed by [`Self::endpoint`].
    pub support_nodes: Vec<f64>,
    /// LGR quadrature weights for values at [`Self::collocation_nodes`].
    pub quadrature_weights: Vec<f64>,
    /// Row-major `N x (N + 1)` barycentric differentiation matrix.
    pub differentiation_matrix: Vec<f64>,
}

/// One LGR pseudospectral segment on a physical time interval.
///
/// `support_states` is row-major over every LGR support node, including the
/// terminal endpoint. `collocation_controls` is row-major over collocation
/// nodes only. The segment residual is
/// `D * x_support - (duration / 2) * f(t_i, x_i, u_i)` at each collocation
/// node.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSegment {
    /// Initial physical time in seconds.
    pub initial_time_s: f64,
    /// Final physical time in seconds.
    pub final_time_s: f64,
    /// LGR grid on the normalized interval `[-1, 1]`.
    pub grid: LegendreGaussRadauGrid,
    /// Number of state components per support node.
    pub state_dimension: usize,
    /// Number of control components per collocation node.
    pub control_dimension: usize,
    /// Row-major support-node states, length `(N + 1) * state_dimension`.
    pub support_states: Vec<f64>,
    /// Row-major collocation-node controls, length `N * control_dimension`.
    pub collocation_controls: Vec<f64>,
}

impl LgrPseudospectralSegment {
    /// Segment duration in seconds.
    #[must_use]
    pub fn duration_s(&self) -> f64 {
        self.final_time_s - self.initial_time_s
    }

    /// Segment half-duration in seconds.
    #[must_use]
    pub fn half_duration_s(&self) -> f64 {
        0.5 * self.duration_s()
    }
}

/// Residual and quadrature report for one LGR pseudospectral segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSegmentReport {
    /// Physical collocation times in seconds, length `N`.
    pub collocation_times_s: Vec<f64>,
    /// Physical quadrature weights, equal to `(duration / 2) * w_i`.
    pub quadrature_weights_s: Vec<f64>,
    /// Row-major differential residuals, length `N * state_dimension`.
    pub residuals: Vec<f64>,
    /// Euclidean norm of [`Self::residuals`].
    pub residual_norm: f64,
    /// Maximum absolute differential residual.
    pub max_abs_residual: f64,
    /// LGR quadrature cost for the supplied running-cost integrand.
    pub quadrature_cost: f64,
}

/// Multi-segment LGR pseudospectral transcription.
///
/// Each segment carries its own normalized LGR grid and physical time interval.
/// Adjacent segments must have matching state/control dimensions and contiguous
/// physical time bounds. Continuity is reported as explicit residual rows
/// between the terminal support state of one segment and the initial support
/// state of the next.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralTranscription {
    /// Ordered LGR segments.
    pub segments: Vec<LgrPseudospectralSegment>,
}

/// Residual and quadrature report for a multi-segment LGR transcription.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralTranscriptionReport {
    /// Per-segment LGR evaluation reports.
    pub segment_reports: Vec<LgrPseudospectralSegmentReport>,
    /// Stacked differential residual rows followed by continuity residual rows.
    pub residuals: Vec<f64>,
    /// Number of stacked differential residual values.
    pub differential_residual_count: usize,
    /// Number of stacked continuity residual values.
    pub continuity_residual_count: usize,
    /// Euclidean norm of [`Self::residuals`].
    pub residual_norm: f64,
    /// Maximum absolute residual over differential and continuity rows.
    pub max_abs_residual: f64,
    /// Sum of all segment quadrature costs.
    pub quadrature_cost: f64,
}

/// Flat fixed-time decision-vector layout for an LGR pseudospectral transcription.
///
/// Decision vectors are ordered as:
///
/// ```text
/// [segment support states, segment collocation controls]
/// ```
///
/// State blocks are stored segment-by-segment over every LGR support node,
/// including each segment endpoint. Controls are stored segment-by-segment over
/// collocation nodes only.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralDecisionLayout {
    /// Physical mesh times in seconds, length `segment_count + 1`.
    pub time_nodes_s: Vec<f64>,
    /// Collocation-node count for each segment.
    pub segment_collocation_counts: Vec<usize>,
    /// LGR grid for each segment.
    pub segment_grids: Vec<LegendreGaussRadauGrid>,
    /// Number of state components per support node.
    pub state_dimension: usize,
    /// Number of control components per collocation node.
    pub control_dimension: usize,
    /// Absolute offsets of each segment's support-state block.
    pub segment_state_offsets: Vec<usize>,
    /// Absolute offsets of each segment's collocation-control block.
    pub segment_control_offsets: Vec<usize>,
    /// Number of support-state decision variables.
    pub state_variable_count: usize,
    /// Number of collocation-control decision variables.
    pub control_variable_count: usize,
    /// Total number of decision variables.
    pub variable_count: usize,
}

/// Decoded transcription and residual report from an LGR decision vector.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralDecisionReport {
    /// Multi-segment transcription decoded from the flat decision vector.
    pub transcription: LgrPseudospectralTranscription,
    /// Residual and quadrature report for the decoded transcription.
    pub transcription_report: LgrPseudospectralTranscriptionReport,
}

/// State-component mapping used when emitting an optimized LGR reference
/// profile from a fixed-mesh decision vector.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct LgrPseudospectralReferenceProfileMapping {
    /// State component containing inertial reference radius, m.
    pub radius_component: usize,
    /// State component containing inertial reference speed, m/s.
    pub speed_component: usize,
    /// State component containing flight-path angle, rad.
    pub flight_path_angle_component: usize,
}

/// Deterministic optimized reference profile emitted from an LGR fixed mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralReferenceProfileReport {
    /// Reference samples in strictly increasing time order.
    pub samples: Vec<ReferenceProfileSample>,
    /// Number of emitted samples.
    pub sample_count: usize,
    /// First sample time, s.
    pub start_time_s: f64,
    /// Last sample time, s.
    pub end_time_s: f64,
}

/// Fixed-state boundary residual applied to an LGR mesh node.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LgrPseudospectralBoundaryConstraint {
    /// Mesh node index in `[0, segment_count]`.
    pub mesh_node_index: usize,
    /// State component index in `[0, state_dimension)`.
    pub component: usize,
    /// Target state value.
    pub target: f64,
}

/// Scalar variable selected at an LGR collocation sample.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum LgrPseudospectralScalarVariable {
    /// State component at the collocation sample.
    StateComponent {
        /// State component index.
        component: usize,
    },
    /// Control component at the collocation sample.
    ControlComponent {
        /// Control component index.
        component: usize,
    },
}

/// Soft scalar path bound represented as an exterior-penalty residual.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LgrPseudospectralSoftPathConstraint {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Collocation node index in `[0, segment_collocation_count)`.
    pub collocation_index: usize,
    /// Scalar state/control component at the collocation node.
    pub variable: LgrPseudospectralScalarVariable,
    /// Optional lower bound.
    pub lower: Option<f64>,
    /// Optional upper bound.
    pub upper: Option<f64>,
    /// Positive residual weight. The reported residual is
    /// `sqrt(weight) * violation`.
    pub weight: f64,
}

/// Hard scalar path bound transcribed as a dense linear inequality row.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LgrPseudospectralHardPathConstraint {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Collocation node index in `[0, segment_collocation_count)`.
    pub collocation_index: usize,
    /// Scalar state/control component at the collocation node.
    pub variable: LgrPseudospectralScalarVariable,
    /// Optional lower bound.
    pub lower: Option<f64>,
    /// Optional upper bound.
    pub upper: Option<f64>,
}

/// Nonlinear vector path bound transcribed as hard inequality rows at an LGR
/// collocation sample.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralNonlinearHardPathConstraint {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Collocation node index in `[0, segment_collocation_count)`.
    pub collocation_index: usize,
    /// Optional lower bounds for each path-function component.
    pub lower_bounds: Vec<Option<f64>>,
    /// Optional upper bounds for each path-function component.
    pub upper_bounds: Vec<Option<f64>>,
}

/// LGR-quadrature-weighted quadratic control objective residual.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LgrPseudospectralQuadraticControlCost {
    /// Control component index.
    pub component: usize,
    /// Positive control-effort weight.
    pub weight: f64,
}

/// Mayer terminal-cost callback for fixed-time LGR objectives.
pub trait LgrPseudospectralMayerCost {
    /// Evaluate `phi(x_f, t_f)` at the terminal mesh node.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the terminal cost cannot be evaluated.
    fn eval(&mut self, terminal_time_s: f64, terminal_state: &[f64]) -> Result<f64, TrajoptError>;
}

impl<F> LgrPseudospectralMayerCost for F
where
    F: for<'state> FnMut(f64, &'state [f64]) -> Result<f64, TrajoptError>,
{
    fn eval(&mut self, terminal_time_s: f64, terminal_state: &[f64]) -> Result<f64, TrajoptError> {
        self(terminal_time_s, terminal_state)
    }
}

/// Fixed-time LGR scalar objective report.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralObjectiveReport {
    /// Terminal time used by the Mayer callback.
    pub terminal_time_s: f64,
    /// Terminal mesh-node state used by the Mayer callback.
    pub terminal_state: Vec<f64>,
    /// LGR quadrature of the caller-supplied Lagrange integrand.
    pub lagrange_quadrature_cost: f64,
    /// LGR quadrature of configured quadratic control-effort costs.
    pub quadratic_control_cost: f64,
    /// Terminal Mayer cost.
    pub mayer_cost: f64,
    /// Sum of Lagrange, quadratic-control, and Mayer costs.
    pub total_objective: f64,
}

/// Dense decision bounds after applying hard scalar LGR path constraints.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralDecisionBoundsReport {
    /// Number of flat LGR decision variables.
    pub variable_count: usize,
    /// Lower decision bounds.
    pub lower_bounds: Vec<f64>,
    /// Upper decision bounds.
    pub upper_bounds: Vec<f64>,
    /// Number of lower/upper hard-path bounds applied.
    pub applied_bound_count: usize,
    /// Number of distinct decision variables tightened by hard-path bounds.
    pub constrained_variable_count: usize,
}

/// Fixed-time LGR OCP residual report.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshResidualReport {
    /// Residual vector: differential, continuity, boundary, path, objective.
    pub residuals: Vec<f64>,
    /// Number of differential residual values.
    pub differential_residual_count: usize,
    /// Number of segment-boundary continuity residual values.
    pub continuity_residual_count: usize,
    /// Number of fixed-state boundary residual values.
    pub boundary_residual_count: usize,
    /// Number of soft path residual values.
    pub path_residual_count: usize,
    /// Number of quadratic-control objective residual values.
    pub objective_residual_count: usize,
    /// Euclidean norm of all residual values.
    pub residual_norm: f64,
    /// Maximum absolute residual over all rows.
    pub max_abs_residual: f64,
    /// LGR quadrature cost reported by the decoded transcription.
    pub quadrature_cost: f64,
    /// LGR quadrature of configured quadratic control costs.
    pub quadratic_control_cost: f64,
}

/// Dense row-major inequality transcription for hard LGR path bounds.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralPathInequalityReport {
    /// Number of flat LGR decision variables.
    pub variable_count: usize,
    /// Number of emitted inequality rows.
    pub inequality_count: usize,
    /// Row-major dense `Aineq` such that `Aineq * decision <= inequality_rhs`.
    pub inequality_matrix: Vec<f64>,
    /// Dense inequality right-hand side.
    pub inequality_rhs: Vec<f64>,
}

/// Nonlinear hard path inequalities evaluated as `g(z) <= 0` rows.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralNonlinearPathInequalityReport {
    /// Number of flat LGR decision variables.
    pub variable_count: usize,
    /// Number of path-function components evaluated before bound splitting.
    pub path_value_count: usize,
    /// Number of emitted scalar inequality rows.
    pub inequality_count: usize,
    /// Raw path-function values, constraint-major and component-major.
    pub path_values: Vec<f64>,
    /// Inequality row values. Feasible rows are `<= 0`.
    pub inequality_values: Vec<f64>,
    /// Maximum non-negative inequality-row violation.
    pub max_violation: f64,
}

/// Sparse CSC Jacobian for nonlinear hard LGR path inequalities.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparsePathInequalityJacobianReport {
    /// Number of inequality rows.
    pub row_count: usize,
    /// Number of decision-variable columns.
    pub column_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Central finite-difference step used for each column scale.
    pub finite_difference_step: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Sparse CSC residual Jacobian for a fixed-mesh LGR residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparseResidualJacobianReport {
    /// Number of residual rows.
    pub row_count: usize,
    /// Number of decision-variable columns.
    pub column_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Central finite-difference step used for each column scale.
    pub finite_difference_step: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Colored finite-difference sparse residual Jacobian report for fixed-mesh
/// LGR residual models.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralColoredSparseResidualJacobianReport {
    /// Sparse residual Jacobian evaluated with colored finite differences.
    pub sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    /// Greedy deterministic color for each decision-variable column.
    pub column_colors: Vec<usize>,
    /// Number of column colors used.
    pub color_count: usize,
    /// Number of residual evaluations used: one base evaluation plus two per
    /// color.
    pub residual_evaluation_count: usize,
    /// Conservative structural nonzero count before numeric drop tolerance.
    pub structural_nonzero_count: usize,
}

/// Sparse CSC Gauss-Newton Hessian / KKT normal block for a fixed-mesh LGR
/// residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparseGaussNewtonHessianReport {
    /// Number of Hessian rows.
    pub row_count: usize,
    /// Number of Hessian columns.
    pub column_count: usize,
    /// Number of residual rows in the source Jacobian.
    pub residual_row_count: usize,
    /// Number of nonzeros in the source residual Jacobian.
    pub source_jacobian_nonzero_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Non-negative diagonal regularization added to `J'J`.
    pub regularization: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Sparse CSC augmented KKT system for a linearized LGR least-squares step.
///
/// The system is the symmetric quasi-definite saddle matrix
/// `[regularization * I, J'; J, -I]`, where `J` is the sparse LGR residual
/// Jacobian. Solving it with right-hand side `[0, -r]` yields the same primal
/// step as the regularized normal equation `(J'J + regularization I) * step =
/// -J'r`, while exposing residual-space KKT variables for costate diagnostics.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparseAugmentedKktReport {
    /// Number of KKT rows.
    pub row_count: usize,
    /// Number of KKT columns.
    pub column_count: usize,
    /// Number of primal decision variables.
    pub variable_count: usize,
    /// Number of residual rows in the source Jacobian.
    pub residual_row_count: usize,
    /// Number of nonzeros in the source residual Jacobian.
    pub source_jacobian_nonzero_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Positive diagonal regularization in the primal block.
    pub regularization: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
}

/// Sparse CSC constrained KKT system for an active-hard-inequality LGR
/// linearized least-squares step.
///
/// The system is `[regularization * I, J', A'; J, -I, 0; A, 0, 0]`, where `J`
/// is the residual Jacobian and `A` contains active linearized hard-path rows.
/// Solving it with right-hand side `[0, -r, -g_active]` yields a primal step,
/// residual-space multipliers, and active inequality multipliers.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparseConstrainedKktReport {
    /// Number of KKT rows.
    pub row_count: usize,
    /// Number of KKT columns.
    pub column_count: usize,
    /// Number of primal decision variables.
    pub variable_count: usize,
    /// Number of residual rows in the source Jacobian.
    pub residual_row_count: usize,
    /// Number of active hard-inequality rows.
    pub active_inequality_row_count: usize,
    /// Number of nonzeros in the source residual Jacobian.
    pub source_jacobian_nonzero_count: usize,
    /// Number of nonzeros in the active hard-inequality Jacobian.
    pub source_inequality_nonzero_count: usize,
    /// Active hard-path row indices in the source inequality report.
    pub active_inequality_rows: Vec<usize>,
    /// Active inequality values `g(z) = A*z - b` at the linearization point.
    pub active_inequality_values: Vec<f64>,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
    /// Positive diagonal regularization in the primal block.
    pub regularization: f64,
    /// Absolute coefficient threshold used to drop structural zeros.
    pub drop_tolerance: f64,
    /// Active-row threshold used to select rows with `g(z) >= -tolerance`.
    pub active_tolerance: f64,
}

/// Conjugate-gradient solve report for a sparse LGR normal-equation block.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparseNormalSolveReport {
    /// Final solution vector.
    pub solution: Vec<f64>,
    /// Final linear residual `A * solution - rhs`.
    pub residual: Vec<f64>,
    /// Euclidean norm of the initial linear residual.
    pub initial_residual_norm: f64,
    /// Euclidean norm of the final linear residual.
    pub residual_norm: f64,
    /// Number of conjugate-gradient iterations executed.
    pub iterations: usize,
    /// Whether `residual_norm <= residual_tolerance`.
    pub converged: bool,
}

/// Deterministic LDL solve report for a sparse LGR augmented KKT matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSparseKktSolveReport {
    /// Final solution vector.
    pub solution: Vec<f64>,
    /// Final linear residual `A * solution - rhs`.
    pub residual: Vec<f64>,
    /// Euclidean norm of [`Self::residual`].
    pub residual_norm: f64,
    /// Diagonal entries of the no-pivot LDL factorization.
    pub factor_diagonal: Vec<f64>,
    /// Frobenius norm of `L * D * L' - A` for the dense factor workspace.
    pub factorization_residual_norm: f64,
}

/// Per-collocation KKT multipliers for LGR differential residual rows.
///
/// These are raw residual-space multipliers split according to the fixed-mesh
/// LGR differential row ordering. They are a deterministic substrate for later
/// PMP/costate cross-checks, not a complete covector mapping by themselves.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralKktCollocationMultiplierSample {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Collocation node index in the segment.
    pub collocation_index: usize,
    /// Physical collocation time in seconds.
    pub time_s: f64,
    /// Multipliers associated with the state-dimension differential rows.
    pub dynamics_defect_multipliers: Vec<f64>,
}

/// KKT residual multipliers split by fixed-mesh LGR residual row family.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralKktResidualMultiplierReport {
    /// Number of LGR segments in the source OCP.
    pub segment_count: usize,
    /// State dimension in the source OCP.
    pub state_dimension: usize,
    /// Total number of residual multipliers.
    pub residual_multiplier_count: usize,
    /// Number of differential residual multipliers.
    pub differential_multiplier_count: usize,
    /// Number of segment-boundary continuity multipliers.
    pub continuity_multiplier_count: usize,
    /// Number of fixed-state boundary residual multipliers.
    pub boundary_multiplier_count: usize,
    /// Number of soft path residual multipliers.
    pub path_multiplier_count: usize,
    /// Number of objective residual multipliers.
    pub objective_multiplier_count: usize,
    /// Per-collocation differential multiplier slices.
    pub collocation_samples: Vec<LgrPseudospectralKktCollocationMultiplierSample>,
    /// Segment-boundary continuity residual multipliers.
    pub continuity_multipliers: Vec<f64>,
    /// Fixed-state boundary residual multipliers.
    pub boundary_multipliers: Vec<f64>,
    /// Soft path residual multipliers.
    pub path_multipliers: Vec<f64>,
    /// Objective residual multipliers.
    pub objective_multipliers: Vec<f64>,
    /// Maximum absolute differential multiplier.
    pub max_abs_dynamics_multiplier: f64,
    /// Maximum absolute continuity multiplier.
    pub max_abs_continuity_multiplier: f64,
    /// Maximum absolute boundary multiplier.
    pub max_abs_boundary_multiplier: f64,
}

/// Collocation-level raw costate estimate from LGR differential multipliers.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralKktCollocationCostateEstimate {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Collocation node index in the segment.
    pub collocation_index: usize,
    /// Physical collocation time in seconds.
    pub time_s: f64,
    /// Raw costate estimate in state-component order.
    ///
    /// No sign flip or mesh scaling is applied here.
    pub costate: Vec<f64>,
}

/// Raw collocation-costate estimate report derived from KKT residual multipliers.
///
/// This is a deterministic mesh diagnostic for later PMP witnesses. It is not a
/// complete Betts-style covector mapping and does not apply final sign/scaling
/// conventions.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralKktCostateEstimateReport {
    /// Number of LGR segments in the source OCP.
    pub segment_count: usize,
    /// State dimension in the source OCP.
    pub state_dimension: usize,
    /// Total number of residual multipliers consumed.
    pub source_residual_multiplier_count: usize,
    /// Collocation-level raw costate estimates.
    pub collocation_samples: Vec<LgrPseudospectralKktCollocationCostateEstimate>,
    /// Maximum absolute collocation costate component.
    pub max_abs_costate: f64,
}

impl LgrPseudospectralKktCostateEstimateReport {
    /// Apply an explicit direct-method-to-PMP costate convention.
    ///
    /// The raw LGR collocation estimates remain unscaled diagnostics. This
    /// adapter only packages them into the common PMP comparison shape after the
    /// caller selects a sign/scale convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the raw collocation report is malformed or
    /// the selected PMP convention is invalid.
    pub fn pmp_costate_convention_report(
        &self,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        if self.segment_count == 0
            || self.state_dimension == 0
            || self.collocation_samples.len() < 2
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR PMP costate convention report dimensions are inconsistent",
            });
        }
        let mut times_s = Vec::with_capacity(self.collocation_samples.len());
        let mut raw_costates =
            Vec::with_capacity(self.collocation_samples.len() * self.state_dimension);
        for sample in &self.collocation_samples {
            if sample.segment_index >= self.segment_count
                || sample.costate.len() != self.state_dimension
            {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR PMP costate convention sample is inconsistent",
                });
            }
            times_s.push(sample.time_s);
            raw_costates.extend(sample.costate.iter().copied());
        }
        crate::pmp::apply_pmp_direct_costate_convention(
            self.state_dimension,
            &times_s,
            &raw_costates,
            convention,
        )
    }
}

/// Collocation-level Hamiltonian diagnostic from raw LGR KKT costate estimates.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralKktHamiltonianSample {
    /// Segment index in `[0, segment_count)`.
    pub segment_index: usize,
    /// Collocation node index in the segment.
    pub collocation_index: usize,
    /// Physical collocation time in seconds.
    pub time_s: f64,
    /// Raw collocation-costate estimate used for this sample.
    pub costate: Vec<f64>,
    /// Dynamics derivative `f(t, x, u)` at this sample.
    pub dynamics: Vec<f64>,
    /// Lagrange integrand `L(t, x, u)` at this sample.
    pub lagrange_value: f64,
    /// Inner product `lambda' * f`.
    pub costate_dot_dynamics: f64,
    /// Raw Hamiltonian diagnostic `L + lambda' * f`.
    pub hamiltonian: f64,
}

/// Raw Hamiltonian diagnostic report for later PMP witnesses.
///
/// This report evaluates `H = L + lambda' f` at LGR collocation nodes using the
/// unscaled collocation-costate estimates. It exposes deterministic diagnostic
/// evidence only; it is not a PMP constancy proof or final covector convention.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralKktHamiltonianReport {
    /// Number of LGR segments in the source OCP.
    pub segment_count: usize,
    /// State dimension in the source OCP.
    pub state_dimension: usize,
    /// Number of collocation Hamiltonian samples.
    pub sample_count: usize,
    /// Collocation-level Hamiltonian diagnostic samples.
    pub samples: Vec<LgrPseudospectralKktHamiltonianSample>,
    /// Maximum absolute Hamiltonian value.
    pub max_abs_hamiltonian: f64,
    /// Maximum absolute Hamiltonian drift relative to the first sample.
    pub max_abs_hamiltonian_drift: f64,
}

/// Sparse Gauss-Newton step report for a fixed-mesh LGR residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshSparseGaussNewtonStepReport {
    /// Residual report evaluated at the linearization point.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Sparse residual Jacobian at the linearization point.
    pub sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    /// Sparse normal-equation block `J'J + regularization I`.
    pub normal_hessian: LgrPseudospectralSparseGaussNewtonHessianReport,
    /// Sparse normal-equation right-hand side `-J'r`.
    pub rhs: Vec<f64>,
    /// Conjugate-gradient solve report for the sparse normal equation.
    pub solve_report: LgrPseudospectralSparseNormalSolveReport,
    /// Linearized residual `r + J * step`.
    pub linearized_residual: Vec<f64>,
    /// Euclidean norm of [`Self::linearized_residual`].
    pub linearized_residual_norm: f64,
    /// Euclidean norm of the solved step.
    pub step_norm: f64,
}

/// Sparse augmented-KKT step report for a fixed-mesh LGR residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshSparseKktStepReport {
    /// Residual report evaluated at the linearization point.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Sparse residual Jacobian at the linearization point.
    pub sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    /// Sparse augmented KKT matrix `[regularization * I, J'; J, -I]`.
    pub augmented_kkt: LgrPseudospectralSparseAugmentedKktReport,
    /// Augmented KKT right-hand side `[0, -r]`.
    pub rhs: Vec<f64>,
    /// LDL solve report for the augmented KKT system.
    pub solve_report: LgrPseudospectralSparseKktSolveReport,
    /// Primal decision step.
    pub step: Vec<f64>,
    /// Residual-space KKT variables, equal to the linearized residual.
    pub residual_multipliers: Vec<f64>,
    /// Linearized residual `r + J * step`.
    pub linearized_residual: Vec<f64>,
    /// Euclidean norm of [`Self::linearized_residual`].
    pub linearized_residual_norm: f64,
    /// Euclidean norm of the primal step.
    pub step_norm: f64,
}

/// Sparse constrained-KKT step report for active scalar hard-path rows.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshSparseConstrainedKktStepReport {
    /// Residual report evaluated at the linearization point.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Sparse residual Jacobian at the linearization point.
    pub sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    /// Hard scalar path inequality rows evaluated at the linearization point.
    pub hard_path_inequality_report: LgrPseudospectralPathInequalityReport,
    /// Sparse constrained KKT matrix with active inequality rows.
    pub constrained_kkt: LgrPseudospectralSparseConstrainedKktReport,
    /// Constrained KKT right-hand side `[0, -r, -g_active]`.
    pub rhs: Vec<f64>,
    /// LDL solve report for the constrained KKT system.
    pub solve_report: LgrPseudospectralSparseKktSolveReport,
    /// Primal decision step.
    pub step: Vec<f64>,
    /// Residual-space KKT variables, equal to the linearized residual.
    pub residual_multipliers: Vec<f64>,
    /// Active hard-inequality multipliers in active-row order.
    pub active_inequality_multipliers: Vec<f64>,
    /// Linearized residual `r + J * step`.
    pub linearized_residual: Vec<f64>,
    /// Euclidean norm of [`Self::linearized_residual`].
    pub linearized_residual_norm: f64,
    /// Linearized active inequality values `g_active + A_active * step`.
    pub linearized_active_inequality_values: Vec<f64>,
    /// Maximum positive value in [`Self::linearized_active_inequality_values`].
    pub max_linearized_active_inequality_violation: f64,
    /// Euclidean norm of the primal step.
    pub step_norm: f64,
}

/// Sparse augmented-KKT SQP solve report for a fixed-mesh LGR residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshSparseKktSqpReport {
    /// Accepted final decision vector.
    pub decision: Vec<f64>,
    /// Final residual report evaluated at [`Self::decision`].
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Final sparse augmented-KKT report assembled at [`Self::decision`].
    pub final_kkt_step_report: LgrPseudospectralFixedMeshSparseKktStepReport,
    /// Final sparse constrained-KKT report when a scalar hard-path row is active.
    pub final_constrained_kkt_step_report:
        Option<LgrPseudospectralFixedMeshSparseConstrainedKktStepReport>,
    /// Decision bounds after merging scalar hard-path constraints.
    pub decision_bounds: LgrPseudospectralDecisionBoundsReport,
    /// Maximum final violation of [`Self::decision_bounds`].
    pub max_decision_bound_violation: f64,
    /// Maximum final violation of scalar hard-path inequalities.
    pub max_hard_path_violation: f64,
    /// Final nonlinear hard-path inequality report, when evaluated by a sparse
    /// SQP entry point with a path-function callback.
    pub nonlinear_hard_path_report: Option<LgrPseudospectralNonlinearPathInequalityReport>,
    /// Maximum final violation of nonlinear hard-path inequalities.
    pub max_nonlinear_hard_path_violation: f64,
    /// Accepted iteration diagnostics.
    pub iterations: Vec<SparseKktSqpIterationReport>,
    /// True when the final residual norm satisfies the requested tolerance.
    pub converged: bool,
}

/// PMP/KKT certificate artifacts derived from one active constrained-KKT step.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshSparseConstrainedKktPmpKktCertificateReport {
    /// Sparse constrained-KKT report consumed by the PMP witness.
    pub constrained_kkt_step_report: LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
    /// Quadrature-normalized costates mapped through the selected PMP convention.
    pub costate_convention_report: crate::pmp::PmpDirectCostateConventionReport,
    /// PMP Hamiltonian profile evaluated from those convention-mapped costates.
    pub hamiltonian_report: crate::pmp::PmpHamiltonianReport,
    /// Optional Hamiltonian control-stationarity report attached to the certificate.
    pub control_stationarity_report: Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    /// Optional same-mesh PMP-vs-KKT costate comparison attached to the certificate.
    pub costate_comparison_report: Option<crate::pmp::PmpCostateComparisonReport>,
    /// Validated I-load optimality certificate for the Hamiltonian profile plus
    /// optional control-stationarity and costate-comparison summaries.
    pub optimality_certificate: ILoadOptimalityCertificate,
}

impl LgrPseudospectralFixedMeshSparseConstrainedKktPmpKktCertificateReport {
    /// Attach a validated same-mesh PMP-vs-KKT costate comparison.
    ///
    /// `pmp_times_s` and `pmp_costates` are the caller-provided PMP/oracle
    /// profile in the same PMP convention selected when this certificate report
    /// was built.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the supplied PMP profile is malformed, does
    /// not match this report's mesh/dimensions, or exceeds the declared
    /// comparison tolerance.
    pub fn with_pmp_costate_comparison(
        mut self,
        pmp_times_s: &[f64],
        pmp_costates: &[f64],
        settings: crate::pmp::PmpCostateComparisonSettings,
    ) -> Result<Self, TrajoptError> {
        attach_pmp_costate_comparison_to_certificate(
            &self.costate_convention_report,
            pmp_times_s,
            pmp_costates,
            settings,
            &mut self.costate_comparison_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }

    /// Attach a validated Hamiltonian control-stationarity summary.
    ///
    /// This evaluates `dH/du` from the certificate's solved PMP Hamiltonian
    /// profile and records the resulting tolerance-checked summary in the I-load
    /// optimality certificate.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity settings, callbacks, or
    /// resulting gradient summary are malformed or outside tolerance.
    pub fn with_pmp_control_stationarity<D, L>(
        mut self,
        settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<Self, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        attach_pmp_control_stationarity_to_certificate(
            &self.hamiltonian_report,
            settings,
            dynamics,
            lagrange,
            &mut self.control_stationarity_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }
}

/// PMP/KKT certificate artifacts derived from an accepted sparse-KKT SQP report.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshSparseKktSqpPmpKktCertificateReport {
    /// Final sparse augmented-KKT report from the accepted sparse SQP decision.
    pub sparse_kkt_step_report: LgrPseudospectralFixedMeshSparseKktStepReport,
    /// Final constrained sparse-KKT report, when a scalar hard-path row is active.
    pub final_constrained_kkt_step_report:
        Option<LgrPseudospectralFixedMeshSparseConstrainedKktStepReport>,
    /// True when the PMP witness used [`Self::final_constrained_kkt_step_report`].
    pub used_constrained_kkt_step_report: bool,
    /// Quadrature-normalized costates mapped through the selected PMP convention.
    pub costate_convention_report: crate::pmp::PmpDirectCostateConventionReport,
    /// PMP Hamiltonian profile evaluated from those convention-mapped costates.
    pub hamiltonian_report: crate::pmp::PmpHamiltonianReport,
    /// Optional Hamiltonian control-stationarity report attached to the certificate.
    pub control_stationarity_report: Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    /// Optional same-mesh PMP-vs-KKT costate comparison attached to the certificate.
    pub costate_comparison_report: Option<crate::pmp::PmpCostateComparisonReport>,
    /// Validated I-load optimality certificate for the Hamiltonian profile plus
    /// optional control-stationarity and costate-comparison summaries.
    pub optimality_certificate: ILoadOptimalityCertificate,
}

impl LgrPseudospectralFixedMeshSparseKktSqpPmpKktCertificateReport {
    /// Attach a validated same-mesh PMP-vs-KKT costate comparison.
    ///
    /// `pmp_times_s` and `pmp_costates` are the caller-provided PMP/oracle
    /// profile in the same PMP convention selected when this certificate report
    /// was built.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the supplied PMP profile is malformed, does
    /// not match this report's mesh/dimensions, or exceeds the declared
    /// comparison tolerance.
    pub fn with_pmp_costate_comparison(
        mut self,
        pmp_times_s: &[f64],
        pmp_costates: &[f64],
        settings: crate::pmp::PmpCostateComparisonSettings,
    ) -> Result<Self, TrajoptError> {
        attach_pmp_costate_comparison_to_certificate(
            &self.costate_convention_report,
            pmp_times_s,
            pmp_costates,
            settings,
            &mut self.costate_comparison_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }

    /// Attach a validated Hamiltonian control-stationarity summary.
    ///
    /// This evaluates `dH/du` from the certificate's solved PMP Hamiltonian
    /// profile and records the resulting tolerance-checked summary in the I-load
    /// optimality certificate.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity settings, callbacks, or
    /// resulting gradient summary are malformed or outside tolerance.
    pub fn with_pmp_control_stationarity<D, L>(
        mut self,
        settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<Self, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        attach_pmp_control_stationarity_to_certificate(
            &self.hamiltonian_report,
            settings,
            dynamics,
            lagrange,
            &mut self.control_stationarity_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }
}

fn attach_pmp_costate_comparison_to_certificate(
    convention_report: &crate::pmp::PmpDirectCostateConventionReport,
    pmp_times_s: &[f64],
    pmp_costates: &[f64],
    settings: crate::pmp::PmpCostateComparisonSettings,
    costate_comparison_report: &mut Option<crate::pmp::PmpCostateComparisonReport>,
    optimality_certificate: &mut ILoadOptimalityCertificate,
) -> Result<(), TrajoptError> {
    let comparison = crate::pmp::compare_pmp_costate_profiles(
        convention_report.state_dimension,
        pmp_times_s,
        pmp_costates,
        &convention_report.times_s,
        &convention_report.pmp_costates,
        settings,
    )?;
    let certificate = optimality_certificate
        .clone()
        .with_costate_comparison(&comparison)?;
    *costate_comparison_report = Some(comparison);
    *optimality_certificate = certificate;
    Ok(())
}

fn attach_pmp_control_stationarity_to_certificate<D, L>(
    hamiltonian_report: &crate::pmp::PmpHamiltonianReport,
    settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
    dynamics: &mut D,
    lagrange: &mut L,
    control_stationarity_report: &mut Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    optimality_certificate: &mut ILoadOptimalityCertificate,
) -> Result<(), TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    let stationarity = crate::pmp::evaluate_pmp_hamiltonian_control_stationarity_from_report(
        hamiltonian_report,
        settings,
        |time_s, state, control| dynamics.eval(time_s, state, control),
        |time_s, state, control| lagrange.eval(time_s, state, control),
    )?;
    let certificate = optimality_certificate
        .clone()
        .with_control_stationarity(&stationarity)?;
    *control_stationarity_report = Some(stationarity);
    *optimality_certificate = certificate;
    Ok(())
}

/// Result from the feature-gated dense QP fixed-mesh LGR solve wrapper.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshDenseQuadraticQpReport {
    /// Dense QP solution returned by Clarabel.
    pub qp_solution: crate::sqp::ClarabelQpSolution,
    /// Final residual report evaluated at the QP solution.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Hard path inequalities consumed by the dense QP.
    pub hard_path_inequality_report: LgrPseudospectralPathInequalityReport,
    /// Number of equality rows sent to the dense QP.
    pub equality_row_count: usize,
    /// Euclidean norm of differential, continuity, and boundary equality rows.
    pub equality_residual_norm: f64,
    /// Maximum violation over scalar hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// Result from the feature-gated dense SQP fixed-mesh LGR solve wrapper.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshDenseSqpReport {
    /// Dense SQP solution returned by the feature-gated solver lane.
    pub sqp_solution: crate::sqp::DenseSqpSolution,
    /// Final residual report evaluated at the accepted SQP variables.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: LgrPseudospectralDecisionBoundsReport,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over scalar hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// Result from the feature-gated dense SQP fixed-mesh LGR solve wrapper with a
/// linear terminal-state objective.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshTerminalObjectiveDenseSqpReport {
    /// Dense SQP objective solution returned by the feature-gated solver lane.
    pub sqp_solution: crate::sqp::DenseSqpObjectiveSolution,
    /// Terminal state component used in the linear objective.
    pub terminal_component: usize,
    /// Linear objective weight applied to that terminal component.
    pub terminal_objective_weight: f64,
    /// Accepted terminal component value.
    pub terminal_value: f64,
    /// Final residual report evaluated at the accepted SQP variables.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Euclidean norm of differential, continuity, and boundary equality rows.
    pub equality_residual_norm: f64,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: LgrPseudospectralDecisionBoundsReport,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over scalar hard path inequality rows.
    pub max_hard_path_violation: f64,
}

/// PMP/KKT certificate artifacts derived from an accepted dense SQP solution.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshDenseSqpPmpKktCertificateReport {
    /// Sparse augmented-KKT report assembled at the accepted dense SQP decision.
    pub sparse_kkt_step_report: LgrPseudospectralFixedMeshSparseKktStepReport,
    /// Quadrature-normalized costates mapped through the selected PMP convention.
    pub costate_convention_report: crate::pmp::PmpDirectCostateConventionReport,
    /// PMP Hamiltonian profile evaluated from those convention-mapped costates.
    pub hamiltonian_report: crate::pmp::PmpHamiltonianReport,
    /// Optional Hamiltonian control-stationarity report attached to the certificate.
    pub control_stationarity_report: Option<crate::pmp::PmpHamiltonianControlStationarityReport>,
    /// Optional same-mesh PMP-vs-KKT costate comparison attached to the certificate.
    pub costate_comparison_report: Option<crate::pmp::PmpCostateComparisonReport>,
    /// Validated I-load optimality certificate for the Hamiltonian profile plus
    /// optional control-stationarity and costate-comparison summaries.
    pub optimality_certificate: ILoadOptimalityCertificate,
}

#[cfg(feature = "sqp")]
impl LgrPseudospectralFixedMeshDenseSqpPmpKktCertificateReport {
    /// Attach a validated same-mesh PMP-vs-KKT costate comparison.
    ///
    /// `pmp_times_s` and `pmp_costates` are the caller-provided PMP/oracle
    /// profile in the same PMP convention selected when this certificate report
    /// was built.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the supplied PMP profile is malformed, does
    /// not match this report's mesh/dimensions, or exceeds the declared
    /// comparison tolerance.
    pub fn with_pmp_costate_comparison(
        mut self,
        pmp_times_s: &[f64],
        pmp_costates: &[f64],
        settings: crate::pmp::PmpCostateComparisonSettings,
    ) -> Result<Self, TrajoptError> {
        attach_pmp_costate_comparison_to_certificate(
            &self.costate_convention_report,
            pmp_times_s,
            pmp_costates,
            settings,
            &mut self.costate_comparison_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }

    /// Attach a validated Hamiltonian control-stationarity summary.
    ///
    /// This evaluates `dH/du` from the certificate's solved PMP Hamiltonian
    /// profile and records the resulting tolerance-checked summary in the I-load
    /// optimality certificate.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the stationarity settings, callbacks, or
    /// resulting gradient summary are malformed or outside tolerance.
    pub fn with_pmp_control_stationarity<D, L>(
        mut self,
        settings: crate::pmp::PmpHamiltonianControlStationaritySettings,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<Self, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        attach_pmp_control_stationarity_to_certificate(
            &self.hamiltonian_report,
            settings,
            dynamics,
            lagrange,
            &mut self.control_stationarity_report,
            &mut self.optimality_certificate,
        )?;
        Ok(self)
    }
}

/// Result from the feature-gated dense SQP fixed-mesh LGR solve wrapper with
/// nonlinear hard path inequalities.
#[cfg(feature = "sqp")]
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshDenseConstrainedSqpReport {
    /// Dense constrained SQP solution returned by the feature-gated solver lane.
    pub sqp_solution: crate::sqp::DenseConstrainedSqpSolution,
    /// Final residual report evaluated at the accepted SQP variables.
    pub residual_report: LgrPseudospectralFixedMeshResidualReport,
    /// Decision bounds consumed by the dense SQP solver after hard path merges.
    pub decision_bounds: LgrPseudospectralDecisionBoundsReport,
    /// Final nonlinear hard path inequality report.
    pub nonlinear_hard_path_report: LgrPseudospectralNonlinearPathInequalityReport,
    /// Maximum violation over the merged decision bounds.
    pub max_decision_bound_violation: f64,
    /// Maximum violation over scalar hard path inequality rows.
    pub max_hard_path_violation: f64,
    /// Maximum violation over nonlinear hard path inequality rows.
    pub max_nonlinear_hard_path_violation: f64,
}

/// Fixed-time LGR OCP residual model.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralFixedMeshOcp {
    /// Decision vector layout.
    pub layout: LgrPseudospectralDecisionLayout,
    /// Fixed-state boundary residuals.
    pub boundary_constraints: Vec<LgrPseudospectralBoundaryConstraint>,
    /// Soft scalar path bound residuals.
    pub path_constraints: Vec<LgrPseudospectralSoftPathConstraint>,
    /// Hard scalar path bounds transcribed as linear inequality rows.
    pub hard_path_constraints: Vec<LgrPseudospectralHardPathConstraint>,
    /// Nonlinear hard vector path bounds evaluated at collocation samples.
    pub nonlinear_hard_path_constraints: Vec<LgrPseudospectralNonlinearHardPathConstraint>,
    /// Quadratic control objective residuals.
    pub quadratic_control_costs: Vec<LgrPseudospectralQuadraticControlCost>,
}

/// Settings for deterministic LGR `ph` mesh-refinement recommendations.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralMeshRefinementSettings {
    /// Segment relative-error tolerance.
    pub relative_error_tolerance: f64,
    /// Largest collocation-node count allowed before an interval is split.
    pub max_collocation_count: usize,
    /// Number of collocation nodes to add for a degree-raise recommendation.
    pub degree_raise_step: usize,
    /// Number of child intervals recommended when the degree cap is reached.
    pub subdivision_factor: usize,
}

/// State-interpolation error estimate for one LGR segment.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralSegmentErrorEstimate {
    /// Segment index in the candidate transcription.
    pub segment_index: usize,
    /// Candidate segment collocation count.
    pub collocation_count: usize,
    /// Reference segment collocation count.
    pub reference_collocation_count: usize,
    /// Maximum absolute state interpolation difference.
    pub max_abs_state_error: f64,
    /// Error denominator, `1 + max(abs(state))`.
    pub state_scale: f64,
    /// Relative error used for the refinement decision.
    pub relative_error: f64,
}

/// Deterministic mesh-refinement action for one LGR segment.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum LgrPseudospectralMeshRefinementAction {
    /// Keep this segment unchanged.
    Accept,
    /// Increase the segment polynomial degree.
    RaiseDegree {
        /// Recommended next collocation-node count.
        collocation_count: usize,
    },
    /// Split the segment into child intervals.
    Split {
        /// Recommended number of child intervals.
        segment_count: usize,
        /// Collocation-node count to use on each child interval.
        collocation_count: usize,
    },
}

/// Refinement recommendation for one LGR segment.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LgrPseudospectralSegmentRefinementRecommendation {
    /// Segment index in the candidate transcription.
    pub segment_index: usize,
    /// Candidate segment collocation count.
    pub collocation_count: usize,
    /// Recommended refinement action.
    pub action: LgrPseudospectralMeshRefinementAction,
}

/// Deterministic `ph` mesh-refinement report for an LGR transcription.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralMeshRefinementReport {
    /// Per-segment state-interpolation error estimates.
    pub estimates: Vec<LgrPseudospectralSegmentErrorEstimate>,
    /// Per-segment accept/raise-degree/split recommendations.
    pub recommendations: Vec<LgrPseudospectralSegmentRefinementRecommendation>,
    /// Maximum segment relative error.
    pub max_relative_error: f64,
    /// True when every segment meets the configured relative-error tolerance.
    pub converged: bool,
    /// Number of segments with a non-accept recommendation.
    pub refined_segment_count: usize,
}

/// Mapping from a source segment to one seeded refined segment.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct LgrPseudospectralRefinedSegmentMapping {
    /// Segment index in the source transcription.
    pub source_segment_index: usize,
    /// Segment index in the refined transcription.
    pub refined_segment_index: usize,
    /// Child index within the source segment refinement.
    pub child_index: usize,
    /// Number of children emitted from the source segment.
    pub child_count: usize,
    /// Action used to seed the refined segment.
    pub action: LgrPseudospectralMeshRefinementAction,
}

/// Seeded transcription produced from ph-refinement recommendations.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralMeshRefinementApplication {
    /// Refined transcription seeded by state/control interpolation.
    pub transcription: LgrPseudospectralTranscription,
    /// Mapping from refined segments back to source segments.
    pub segment_mappings: Vec<LgrPseudospectralRefinedSegmentMapping>,
}

/// One iteration of the LGR `ph` mesh-refinement driver.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralMeshRefinementIterationReport {
    /// Zero-based refinement iteration index.
    pub iteration_index: usize,
    /// Candidate segment count before applying this iteration's recommendations.
    pub candidate_segment_count: usize,
    /// Reference segment count supplied for this iteration.
    pub reference_segment_count: usize,
    /// Per-segment error estimates and recommendations.
    pub refinement_report: LgrPseudospectralMeshRefinementReport,
    /// Source-to-refined segment mapping emitted by applying recommendations.
    pub segment_mappings: Vec<LgrPseudospectralRefinedSegmentMapping>,
}

/// Result of an iterative LGR `ph` mesh-refinement run.
#[derive(Clone, Debug, PartialEq)]
pub struct LgrPseudospectralMeshRefinementLoopReport {
    /// Final seeded transcription after zero or more refinement applications.
    pub transcription: LgrPseudospectralTranscription,
    /// Ordered iteration reports.
    pub iterations: Vec<LgrPseudospectralMeshRefinementIterationReport>,
    /// True when the final estimation pass met the tolerance before the
    /// iteration cap.
    pub converged: bool,
    /// Number of estimation passes that ran.
    pub iteration_count: usize,
    /// Final maximum segment relative error.
    pub max_relative_error: f64,
    /// Number of segments refined in the final estimation pass.
    pub refined_segment_count: usize,
}

impl LegendreGaussRadauGrid {
    /// Number of collocation nodes.
    #[must_use]
    pub fn collocation_count(&self) -> usize {
        self.collocation_nodes.len()
    }

    /// Number of interpolation support nodes.
    #[must_use]
    pub fn support_count(&self) -> usize {
        self.support_nodes.len()
    }

    /// Return one differentiation-matrix entry.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when `row` or `column` is out of range.
    pub fn differentiation_entry(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        let collocation_count = self.collocation_count();
        let support_count = self.support_count();
        if row >= collocation_count || column >= support_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR differentiation matrix index is out of range",
            });
        }
        Ok(self.differentiation_matrix[row * support_count + column])
    }

    /// Differentiate support-node values at the collocation nodes.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the input length does not match
    /// [`Self::support_count`] or contains non-finite values.
    pub fn differentiate_support_values(
        &self,
        support_values: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if support_values.len() != self.support_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR support value dimension is inconsistent",
            });
        }
        validate_finite("LGR support values must be finite", support_values)?;
        let support_count = self.support_count();
        let mut derivatives = vec![0.0; self.collocation_count()];
        for (row, derivative) in derivatives.iter_mut().enumerate() {
            let offset = row * support_count;
            *derivative = self.differentiation_matrix[offset..offset + support_count]
                .iter()
                .zip(support_values.iter())
                .map(|(entry, value)| entry * value)
                .sum();
        }
        Ok(derivatives)
    }

    /// Integrate collocation-node values using the LGR quadrature weights.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the input length does not match
    /// [`Self::collocation_count`] or contains non-finite values.
    pub fn integrate_collocation_values(
        &self,
        collocation_values: &[f64],
    ) -> Result<f64, TrajoptError> {
        if collocation_values.len() != self.collocation_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR collocation value dimension is inconsistent",
            });
        }
        validate_finite("LGR collocation values must be finite", collocation_values)?;
        Ok(self
            .quadrature_weights
            .iter()
            .zip(collocation_values.iter())
            .map(|(weight, value)| weight * value)
            .sum())
    }
}

/// Build a deterministic left Legendre-Gauss-Radau grid on `[-1, 1]`.
///
/// # Errors
///
/// Returns [`TrajoptError`] when `collocation_count` is zero or the Newton solve
/// for an interior node fails to converge.
pub fn legendre_gauss_radau_grid(
    collocation_count: usize,
) -> Result<LegendreGaussRadauGrid, TrajoptError> {
    if collocation_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR grid requires at least one collocation node",
        });
    }

    let collocation_nodes = lgr_collocation_nodes(collocation_count)?;
    let quadrature_weights = lgr_quadrature_weights(&collocation_nodes)?;
    let mut support_nodes = collocation_nodes.clone();
    support_nodes.push(1.0);
    let differentiation_matrix =
        barycentric_differentiation_matrix(&support_nodes, collocation_count)?;

    let grid = LegendreGaussRadauGrid {
        collocation_nodes,
        endpoint: 1.0,
        support_nodes,
        quadrature_weights,
        differentiation_matrix,
    };
    validate_grid(&grid)?;
    Ok(grid)
}

impl LgrPseudospectralDecisionLayout {
    /// Build a fixed-time LGR decision layout.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the mesh times are malformed, the
    /// collocation-count vector does not match the segment count, the state
    /// dimension is zero, or an LGR segment grid cannot be built.
    pub fn new(
        time_nodes_s: Vec<f64>,
        segment_collocation_counts: Vec<usize>,
        state_dimension: usize,
        control_dimension: usize,
    ) -> Result<Self, TrajoptError> {
        if time_nodes_s.len() < 2
            || segment_collocation_counts.len() + 1 != time_nodes_s.len()
            || !time_nodes_s.windows(2).all(|window| {
                window[0].is_finite() && window[1].is_finite() && window[0] < window[1]
            })
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout time nodes must be finite and strictly increasing",
            });
        }
        if state_dimension == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout state dimension must be nonzero",
            });
        }

        let mut segment_grids = Vec::with_capacity(segment_collocation_counts.len());
        let mut segment_state_offsets = Vec::with_capacity(segment_collocation_counts.len());
        let mut state_variable_count = 0_usize;
        for collocation_count in &segment_collocation_counts {
            let grid = legendre_gauss_radau_grid(*collocation_count)?;
            segment_state_offsets.push(state_variable_count);
            state_variable_count += grid.support_count() * state_dimension;
            segment_grids.push(grid);
        }

        let mut segment_control_offsets = Vec::with_capacity(segment_collocation_counts.len());
        let mut control_variable_count = 0_usize;
        for grid in &segment_grids {
            segment_control_offsets.push(state_variable_count + control_variable_count);
            control_variable_count += grid.collocation_count() * control_dimension;
        }
        let variable_count = state_variable_count + control_variable_count;

        let layout = Self {
            time_nodes_s,
            segment_collocation_counts,
            segment_grids,
            state_dimension,
            control_dimension,
            segment_state_offsets,
            segment_control_offsets,
            state_variable_count,
            control_variable_count,
            variable_count,
        };
        validate_lgr_decision_layout(&layout)?;
        Ok(layout)
    }

    /// Number of LGR segments.
    #[must_use]
    pub fn segment_count(&self) -> usize {
        self.segment_collocation_counts.len()
    }

    /// Number of state variables in one segment's support-state block.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the layout or segment index is invalid.
    pub fn support_state_count(&self, segment_index: usize) -> Result<usize, TrajoptError> {
        validate_lgr_decision_layout(self)?;
        if segment_index >= self.segment_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout segment index is out of range",
            });
        }
        Ok(self.segment_grids[segment_index].support_count() * self.state_dimension)
    }

    /// Number of control variables in one segment's collocation-control block.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the layout or segment index is invalid.
    pub fn collocation_control_count(&self, segment_index: usize) -> Result<usize, TrajoptError> {
        validate_lgr_decision_layout(self)?;
        if segment_index >= self.segment_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout segment index is out of range",
            });
        }
        Ok(self.segment_grids[segment_index].collocation_count() * self.control_dimension)
    }

    /// Pack row-major support states and collocation controls into this layout.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when slice lengths disagree with the layout or
    /// any value is non-finite.
    pub fn pack_decision_vector(
        &self,
        support_states: &[f64],
        collocation_controls: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_decision_layout(self)?;
        if support_states.len() != self.state_variable_count
            || collocation_controls.len() != self.control_variable_count
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision block dimensions are inconsistent",
            });
        }
        validate_finite("LGR decision support states must be finite", support_states)?;
        validate_finite(
            "LGR decision collocation controls must be finite",
            collocation_controls,
        )?;

        let mut decision = Vec::with_capacity(self.variable_count);
        decision.extend_from_slice(support_states);
        decision.extend_from_slice(collocation_controls);
        Ok(decision)
    }

    /// Segment support-state slice from a flat decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the vector length, layout, segment index,
    /// or support index is invalid.
    pub fn segment_support_state<'a>(
        &self,
        decision: &'a [f64],
        segment_index: usize,
        support_index: usize,
    ) -> Result<&'a [f64], TrajoptError> {
        validate_lgr_decision_vector(self, decision)?;
        if segment_index >= self.segment_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout segment index is out of range",
            });
        }
        if support_index >= self.segment_grids[segment_index].support_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision support index is out of range",
            });
        }
        let offset =
            self.segment_state_offsets[segment_index] + support_index * self.state_dimension;
        Ok(lgr_slice(decision, offset, self.state_dimension))
    }

    /// Mesh-node state slice from a flat decision vector.
    ///
    /// Interior mesh nodes are represented by the initial support node of the
    /// segment beginning at that mesh time; continuity residuals keep that value
    /// tied to the previous segment endpoint.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the vector length, layout, or mesh node
    /// index is invalid.
    pub fn mesh_node_state<'a>(
        &self,
        decision: &'a [f64],
        mesh_node_index: usize,
    ) -> Result<&'a [f64], TrajoptError> {
        validate_lgr_decision_vector(self, decision)?;
        if mesh_node_index > self.segment_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision mesh node index is out of range",
            });
        }
        if mesh_node_index == self.segment_count() {
            let segment_index = self.segment_count() - 1;
            return self.segment_support_state(
                decision,
                segment_index,
                self.segment_grids[segment_index].support_count() - 1,
            );
        }
        self.segment_support_state(decision, mesh_node_index, 0)
    }

    /// Segment collocation-control slice from a flat decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the vector length, layout, segment index,
    /// or collocation index is invalid.
    pub fn segment_collocation_control<'a>(
        &self,
        decision: &'a [f64],
        segment_index: usize,
        collocation_index: usize,
    ) -> Result<&'a [f64], TrajoptError> {
        validate_lgr_decision_vector(self, decision)?;
        if segment_index >= self.segment_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout segment index is out of range",
            });
        }
        if collocation_index >= self.segment_grids[segment_index].collocation_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision collocation index is out of range",
            });
        }
        let offset = self.segment_control_offsets[segment_index]
            + collocation_index * self.control_dimension;
        Ok(lgr_slice(decision, offset, self.control_dimension))
    }

    /// Decode a flat decision vector into a multi-segment LGR transcription.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the layout, decision vector, or decoded
    /// segment contents are invalid.
    pub fn transcription_from_decision(
        &self,
        decision: &[f64],
    ) -> Result<LgrPseudospectralTranscription, TrajoptError> {
        validate_lgr_decision_vector(self, decision)?;
        let mut segments = Vec::with_capacity(self.segment_count());

        for segment_index in 0..self.segment_count() {
            let grid = self.segment_grids[segment_index].clone();
            let state_count = grid.support_count() * self.state_dimension;
            let control_count = grid.collocation_count() * self.control_dimension;
            let state_offset = self.segment_state_offsets[segment_index];
            let control_offset = self.segment_control_offsets[segment_index];
            segments.push(LgrPseudospectralSegment {
                initial_time_s: self.time_nodes_s[segment_index],
                final_time_s: self.time_nodes_s[segment_index + 1],
                grid,
                state_dimension: self.state_dimension,
                control_dimension: self.control_dimension,
                support_states: lgr_slice(decision, state_offset, state_count).to_vec(),
                collocation_controls: lgr_slice(decision, control_offset, control_count).to_vec(),
            });
        }

        let transcription = LgrPseudospectralTranscription { segments };
        validate_lgr_transcription(&transcription)?;
        Ok(transcription)
    }

    /// Decode and evaluate a flat LGR decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector is malformed or the
    /// decoded transcription evaluation fails.
    pub fn evaluate_decision<D, L>(
        &self,
        decision: &[f64],
        dynamics: D,
        lagrange: L,
    ) -> Result<LgrPseudospectralDecisionReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let transcription = self.transcription_from_decision(decision)?;
        let transcription_report =
            evaluate_lgr_pseudospectral_transcription(&transcription, dynamics, lagrange)?;
        Ok(LgrPseudospectralDecisionReport {
            transcription,
            transcription_report,
        })
    }
}

impl LgrPseudospectralReferenceProfileMapping {
    /// Build a reference-profile state-component mapping.
    #[must_use]
    pub const fn new(
        radius_component: usize,
        speed_component: usize,
        flight_path_angle_component: usize,
    ) -> Self {
        Self {
            radius_component,
            speed_component,
            flight_path_angle_component,
        }
    }
}

impl LgrPseudospectralBoundaryConstraint {
    /// Build a finite mesh-node state boundary constraint.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when `target` is non-finite.
    pub fn new(
        mesh_node_index: usize,
        component: usize,
        target: f64,
    ) -> Result<Self, TrajoptError> {
        if !target.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR boundary target must be finite",
            });
        }
        Ok(Self {
            mesh_node_index,
            component,
            target,
        })
    }
}

impl LgrPseudospectralSoftPathConstraint {
    /// Build a soft scalar path bound at an LGR collocation node.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the bounds or weight are invalid.
    pub fn new(
        segment_index: usize,
        collocation_index: usize,
        variable: LgrPseudospectralScalarVariable,
        lower: Option<f64>,
        upper: Option<f64>,
        weight: f64,
    ) -> Result<Self, TrajoptError> {
        if !weight.is_finite() || weight <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR soft path weight must be finite and positive",
            });
        }
        validate_lgr_path_bounds(lower, upper)?;
        Ok(Self {
            segment_index,
            collocation_index,
            variable,
            lower,
            upper,
            weight,
        })
    }
}

impl LgrPseudospectralHardPathConstraint {
    /// Build a hard scalar path bound at an LGR collocation node.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the bounds are invalid.
    pub fn new(
        segment_index: usize,
        collocation_index: usize,
        variable: LgrPseudospectralScalarVariable,
        lower: Option<f64>,
        upper: Option<f64>,
    ) -> Result<Self, TrajoptError> {
        validate_lgr_hard_path_bounds(lower, upper)?;
        Ok(Self {
            segment_index,
            collocation_index,
            variable,
            lower,
            upper,
        })
    }

    /// Number of dense inequality rows emitted by this bound.
    #[must_use]
    pub fn inequality_count(&self) -> usize {
        usize::from(self.upper.is_some()) + usize::from(self.lower.is_some())
    }
}

impl LgrPseudospectralNonlinearHardPathConstraint {
    /// Build a nonlinear hard path constraint descriptor at an LGR collocation
    /// node.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions or bounds are invalid.
    pub fn new(
        segment_index: usize,
        collocation_index: usize,
        lower_bounds: Vec<Option<f64>>,
        upper_bounds: Vec<Option<f64>>,
    ) -> Result<Self, TrajoptError> {
        let constraint = Self {
            segment_index,
            collocation_index,
            lower_bounds,
            upper_bounds,
        };
        validate_lgr_nonlinear_path_bounds(&constraint)?;
        Ok(constraint)
    }

    /// Number of path-function components.
    #[must_use]
    pub fn output_dimension(&self) -> usize {
        self.lower_bounds.len()
    }

    /// Number of scalar inequality rows emitted by this descriptor.
    #[must_use]
    pub fn inequality_count(&self) -> usize {
        self.lower_bounds
            .iter()
            .zip(self.upper_bounds.iter())
            .map(|(lower, upper)| usize::from(upper.is_some()) + usize::from(lower.is_some()))
            .sum()
    }
}

impl LgrPseudospectralQuadraticControlCost {
    /// Build a quadratic control-effort objective term.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when `weight` is not finite and positive.
    pub fn new(component: usize, weight: f64) -> Result<Self, TrajoptError> {
        if !weight.is_finite() || weight <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR control-cost weight must be finite and positive",
            });
        }
        Ok(Self { component, weight })
    }
}

impl LgrPseudospectralPathInequalityReport {
    /// Evaluate `Aineq * decision`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn lhs(&self, decision: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_path_inequality_report(self)?;
        validate_lgr_dense_decision_vector(decision, self.variable_count)?;

        let mut lhs = Vec::with_capacity(self.inequality_count);
        for row in 0..self.inequality_count {
            let offset = row * self.variable_count;
            let value = self.inequality_matrix[offset..offset + self.variable_count]
                .iter()
                .zip(decision.iter())
                .map(|(coefficient, value)| coefficient * value)
                .sum();
            lhs.push(value);
        }
        Ok(lhs)
    }

    /// Evaluate non-negative hard-inequality violations.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn violations(&self, decision: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        let lhs = self.lhs(decision)?;
        Ok(lhs
            .iter()
            .zip(self.inequality_rhs.iter())
            .map(|(lhs, rhs)| (lhs - rhs).max(0.0))
            .collect())
    }

    /// Maximum non-negative hard-inequality violation.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn max_violation(&self, decision: &[f64]) -> Result<f64, TrajoptError> {
        Ok(self
            .violations(decision)?
            .into_iter()
            .fold(0.0_f64, f64::max))
    }
}

impl LgrPseudospectralDecisionBoundsReport {
    /// Evaluate the maximum non-negative bound violation for `decision`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or decision vector is invalid.
    pub fn max_violation(&self, decision: &[f64]) -> Result<f64, TrajoptError> {
        validate_lgr_decision_bounds_report(self)?;
        validate_lgr_dense_decision_vector(decision, self.variable_count)?;
        let mut max_violation = 0.0_f64;
        for ((value, lower), upper) in decision
            .iter()
            .zip(self.lower_bounds.iter())
            .zip(self.upper_bounds.iter())
        {
            max_violation = max_violation.max((lower - value).max(0.0));
            max_violation = max_violation.max((value - upper).max(0.0));
        }
        Ok(max_violation)
    }
}

impl LgrPseudospectralNonlinearPathInequalityReport {
    /// Evaluate non-negative hard-inequality violations.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report shape is invalid.
    pub fn violations(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_nonlinear_path_inequality_report(self)?;
        Ok(self
            .inequality_values
            .iter()
            .map(|value| value.max(0.0))
            .collect())
    }
}

impl LgrPseudospectralSparsePathInequalityJacobianReport {
    /// Return a single Jacobian coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_lgr_sparse_path_inequality_jacobian_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse path inequality Jacobian index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC Jacobian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_path_inequality_jacobian_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }
}

impl LgrPseudospectralSparseResidualJacobianReport {
    /// Return a single Jacobian coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_lgr_sparse_residual_jacobian_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse residual Jacobian index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC Jacobian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_residual_jacobian_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse Jacobian by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the column dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_residual_jacobian_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse residual Jacobian vector length is inconsistent",
            });
        }
        validate_finite("LGR sparse residual Jacobian vector must be finite", vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Assemble the sparse Gauss-Newton step right-hand side `-J' * r`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the Jacobian report is malformed, the
    /// residual length does not match the Jacobian row count, or any numeric
    /// value is non-finite.
    pub fn sparse_gauss_newton_step_rhs(
        &self,
        residuals: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_residual_jacobian_report(self)?;
        if residuals.len() != self.row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse Gauss-Newton RHS residual length is inconsistent",
            });
        }
        validate_finite(
            "LGR sparse Gauss-Newton RHS residuals must be finite",
            residuals,
        )?;

        let mut rhs = vec![0.0; self.column_count];
        for (column, rhs_column) in rhs.iter_mut().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                *rhs_column -= self.values[index] * residuals[self.row_indices[index]];
            }
        }
        Ok(rhs)
    }

    /// Assemble the sparse Gauss-Newton Hessian block `J'J + regularization I`.
    ///
    /// The output is the symmetric normal-equation block used by sparse
    /// least-squares SQP steps.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the source sparse Jacobian is malformed or
    /// the assembly settings are invalid.
    pub fn sparse_gauss_newton_hessian_report(
        &self,
        regularization: f64,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralSparseGaussNewtonHessianReport, TrajoptError> {
        validate_lgr_sparse_residual_jacobian_report(self)?;
        validate_lgr_sparse_hessian_settings(regularization, drop_tolerance)?;

        let mut residual_rows = vec![Vec::new(); self.row_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                residual_rows[self.row_indices[index]].push((column, self.values[index]));
            }
        }

        let mut coefficients = BTreeMap::<(usize, usize), f64>::new();
        for row_entries in &residual_rows {
            for (hessian_row, row_value) in row_entries {
                for (hessian_column, column_value) in row_entries {
                    *coefficients
                        .entry((*hessian_column, *hessian_row))
                        .or_insert(0.0) += row_value * column_value;
                }
            }
        }
        if regularization > 0.0 {
            for diagonal in 0..self.column_count {
                *coefficients.entry((diagonal, diagonal)).or_insert(0.0) += regularization;
            }
        }

        let mut column_offsets = Vec::with_capacity(self.column_count + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        let mut current_column = 0;
        column_offsets.push(0);
        for ((column, row), value) in coefficients {
            while current_column < column {
                column_offsets.push(values.len());
                current_column += 1;
            }
            if value.abs() > drop_tolerance {
                row_indices.push(row);
                values.push(value);
            }
        }
        while current_column < self.column_count {
            column_offsets.push(values.len());
            current_column += 1;
        }

        let report = LgrPseudospectralSparseGaussNewtonHessianReport {
            row_count: self.column_count,
            column_count: self.column_count,
            residual_row_count: self.row_count,
            source_jacobian_nonzero_count: self.values.len(),
            column_offsets,
            row_indices,
            values,
            regularization,
            drop_tolerance,
        };
        validate_lgr_sparse_gauss_newton_hessian_report(&report)?;
        Ok(report)
    }

    /// Assemble the sparse augmented KKT matrix `[regularization * I, J'; J, -I]`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the source sparse Jacobian is malformed or
    /// the assembly settings are invalid.
    pub fn sparse_augmented_kkt_report(
        &self,
        regularization: f64,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralSparseAugmentedKktReport, TrajoptError> {
        validate_lgr_sparse_residual_jacobian_report(self)?;
        validate_lgr_sparse_augmented_kkt_settings(regularization, drop_tolerance)?;

        let variable_count = self.column_count;
        let residual_row_count = self.row_count;
        let dimension = variable_count + residual_row_count;
        let mut coefficients = BTreeMap::<(usize, usize), f64>::new();

        for variable in 0..variable_count {
            coefficients.insert((variable, variable), regularization);
        }
        for residual in 0..residual_row_count {
            let index = variable_count + residual;
            coefficients.insert((index, index), -1.0);
        }
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let residual_row = self.row_indices[index];
                let value = self.values[index];
                let kkt_residual_index = variable_count + residual_row;
                *coefficients
                    .entry((column, kkt_residual_index))
                    .or_insert(0.0) += value;
                *coefficients
                    .entry((kkt_residual_index, column))
                    .or_insert(0.0) += value;
            }
        }

        let mut column_offsets = Vec::with_capacity(dimension + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        let mut current_column = 0;
        column_offsets.push(0);
        for ((column, row), value) in coefficients {
            while current_column < column {
                column_offsets.push(values.len());
                current_column += 1;
            }
            if value.abs() > drop_tolerance {
                row_indices.push(row);
                values.push(value);
            }
        }
        while current_column < dimension {
            column_offsets.push(values.len());
            current_column += 1;
        }

        let report = LgrPseudospectralSparseAugmentedKktReport {
            row_count: dimension,
            column_count: dimension,
            variable_count,
            residual_row_count,
            source_jacobian_nonzero_count: self.values.len(),
            column_offsets,
            row_indices,
            values,
            regularization,
            drop_tolerance,
        };
        validate_lgr_sparse_augmented_kkt_report(&report)?;
        Ok(report)
    }
}

impl LgrPseudospectralSparseGaussNewtonHessianReport {
    /// Return a single Hessian coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_lgr_sparse_gauss_newton_hessian_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse Gauss-Newton Hessian index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC Hessian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_gauss_newton_hessian_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse normal-equation block by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the Hessian dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_gauss_newton_hessian_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse Gauss-Newton Hessian vector length is inconsistent",
            });
        }
        validate_finite(
            "LGR sparse Gauss-Newton Hessian vector must be finite",
            vector,
        )?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Solve `A * x = rhs` with deterministic conjugate gradients.
    ///
    /// The matrix `A` is this sparse Gauss-Newton normal block. This assumes
    /// positive curvature for the supplied problem and is a sparse linear step
    /// substrate, not a full nonlinear NLP backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions/settings are invalid or the CG
    /// iteration detects non-positive curvature.
    pub fn solve_conjugate_gradient(
        &self,
        rhs: &[f64],
        initial_solution: &[f64],
        max_iterations: usize,
        residual_tolerance: f64,
    ) -> Result<LgrPseudospectralSparseNormalSolveReport, TrajoptError> {
        validate_lgr_sparse_cg_settings(
            self,
            rhs,
            initial_solution,
            max_iterations,
            residual_tolerance,
        )?;

        let mut solution = initial_solution.to_vec();
        let mut residual = rhs.to_vec();
        let product = self.multiply_vector(&solution)?;
        for index in 0..residual.len() {
            residual[index] -= product[index];
        }
        let initial_residual_norm = norm(&residual);
        if initial_residual_norm <= residual_tolerance {
            return Ok(LgrPseudospectralSparseNormalSolveReport {
                solution,
                residual: residual.iter().map(|value| -value).collect(),
                initial_residual_norm,
                residual_norm: initial_residual_norm,
                iterations: 0,
                converged: true,
            });
        }

        let mut direction = residual.clone();
        let mut residual_dot = lgr_dot(&residual, &residual);
        let mut iterations = 0;
        let mut converged = false;

        for iteration in 0..max_iterations {
            let product = self.multiply_vector(&direction)?;
            let curvature = lgr_dot(&direction, &product);
            if !curvature.is_finite() || curvature <= 0.0 {
                return Err(TrajoptError::SingularSystem {
                    reason: "LGR sparse Gauss-Newton CG detected non-positive curvature",
                });
            }
            let alpha = residual_dot / curvature;
            for index in 0..solution.len() {
                solution[index] += alpha * direction[index];
                residual[index] -= alpha * product[index];
            }
            iterations = iteration + 1;
            let new_residual_dot = lgr_dot(&residual, &residual);
            if !new_residual_dot.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR sparse Gauss-Newton CG residual is non-finite",
                });
            }
            if new_residual_dot.sqrt() <= residual_tolerance {
                converged = true;
                break;
            }
            let beta = new_residual_dot / residual_dot;
            for index in 0..direction.len() {
                direction[index] = residual[index] + beta * direction[index];
            }
            residual_dot = new_residual_dot;
        }

        let final_residual = self.multiply_vector(&solution)?;
        let final_residual = final_residual
            .iter()
            .zip(rhs.iter())
            .map(|(lhs, rhs_value)| lhs - rhs_value)
            .collect::<Vec<_>>();
        let residual_norm = norm(&final_residual);

        Ok(LgrPseudospectralSparseNormalSolveReport {
            solution,
            residual: final_residual,
            initial_residual_norm,
            residual_norm,
            iterations,
            converged: converged || residual_norm <= residual_tolerance,
        })
    }
}

impl LgrPseudospectralSparseAugmentedKktReport {
    /// Return a single augmented KKT coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_lgr_sparse_augmented_kkt_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse augmented KKT index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC augmented KKT matrix to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_augmented_kkt_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse augmented KKT matrix by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the KKT dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_augmented_kkt_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse augmented KKT vector length is inconsistent",
            });
        }
        validate_finite("LGR sparse augmented KKT vector must be finite", vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Solve the augmented KKT system with a deterministic no-pivot LDL factorization.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or right-hand side is malformed,
    /// or when the LDL factorization detects a singular pivot.
    pub fn solve_ldlt(
        &self,
        rhs: &[f64],
    ) -> Result<LgrPseudospectralSparseKktSolveReport, TrajoptError> {
        validate_lgr_sparse_augmented_kkt_report(self)?;
        if rhs.len() != self.row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse augmented KKT RHS length is inconsistent",
            });
        }
        validate_finite("LGR sparse augmented KKT RHS must be finite", rhs)?;

        let dense = self.to_dense_row_major()?;
        let factorization = lgr_dense_ldlt_factorization(&dense, self.row_count)?;
        let solution = factorization.solve(rhs)?;
        let product = self.multiply_vector(&solution)?;
        let residual = product
            .iter()
            .zip(rhs.iter())
            .map(|(lhs, rhs_value)| lhs - rhs_value)
            .collect::<Vec<_>>();
        let residual_norm = norm(&residual);

        Ok(LgrPseudospectralSparseKktSolveReport {
            solution,
            residual,
            residual_norm,
            factor_diagonal: factorization.diagonal,
            factorization_residual_norm: factorization.factorization_residual_norm,
        })
    }
}

impl LgrPseudospectralSparseConstrainedKktReport {
    /// Return a single constrained KKT coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape or requested index
    /// is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_lgr_sparse_constrained_kkt_report(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse constrained KKT index is out of range",
            });
        }
        let start = self.column_offsets[column];
        let end = self.column_offsets[column + 1];
        for index in start..end {
            if self.row_indices[index] == row {
                return Ok(self.values[index]);
            }
        }
        Ok(0.0)
    }

    /// Expand the CSC constrained KKT matrix to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_constrained_kkt_report(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC sparse constrained KKT matrix by `vector`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse report is malformed, the vector
    /// length does not match the KKT dimension, or any vector value is
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_sparse_constrained_kkt_report(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse constrained KKT vector length is inconsistent",
            });
        }
        validate_finite("LGR sparse constrained KKT vector must be finite", vector)?;

        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }

    /// Solve the constrained KKT system with deterministic no-pivot LDL.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the report or right-hand side is malformed,
    /// or when the LDL factorization detects a singular pivot.
    pub fn solve_ldlt(
        &self,
        rhs: &[f64],
    ) -> Result<LgrPseudospectralSparseKktSolveReport, TrajoptError> {
        validate_lgr_sparse_constrained_kkt_report(self)?;
        if rhs.len() != self.row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse constrained KKT RHS length is inconsistent",
            });
        }
        validate_finite("LGR sparse constrained KKT RHS must be finite", rhs)?;

        let dense = self.to_dense_row_major()?;
        let factorization = lgr_dense_ldlt_factorization(&dense, self.row_count)?;
        let solution = factorization.solve(rhs)?;
        let product = self.multiply_vector(&solution)?;
        let residual = product
            .iter()
            .zip(rhs.iter())
            .map(|(lhs, rhs_value)| lhs - rhs_value)
            .collect::<Vec<_>>();
        let residual_norm = norm(&residual);

        Ok(LgrPseudospectralSparseKktSolveReport {
            solution,
            residual,
            residual_norm,
            factor_diagonal: factorization.diagonal,
            factorization_residual_norm: factorization.factorization_residual_norm,
        })
    }
}

impl LgrPseudospectralColoredSparseResidualJacobianReport {
    /// Expand the CSC Jacobian to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the colored sparse report shape is
    /// invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_lgr_colored_sparse_residual_jacobian_report(self)?;
        self.sparse_jacobian.to_dense_row_major()
    }
}

impl LgrPseudospectralFixedMeshOcp {
    /// Build a fixed-time LGR OCP residual model.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the layout inputs or boundary constraints
    /// are invalid.
    pub fn new(
        time_nodes_s: Vec<f64>,
        segment_collocation_counts: Vec<usize>,
        state_dimension: usize,
        control_dimension: usize,
        boundary_constraints: Vec<LgrPseudospectralBoundaryConstraint>,
    ) -> Result<Self, TrajoptError> {
        let layout = LgrPseudospectralDecisionLayout::new(
            time_nodes_s,
            segment_collocation_counts,
            state_dimension,
            control_dimension,
        )?;
        for constraint in &boundary_constraints {
            validate_lgr_boundary_constraint(&layout, constraint)?;
        }
        Ok(Self {
            layout,
            boundary_constraints,
            path_constraints: Vec::new(),
            hard_path_constraints: Vec::new(),
            nonlinear_hard_path_constraints: Vec::new(),
            quadratic_control_costs: Vec::new(),
        })
    }

    /// Add soft path constraints to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a path constraint references an invalid
    /// collocation sample or component.
    pub fn with_path_constraints(
        mut self,
        path_constraints: Vec<LgrPseudospectralSoftPathConstraint>,
    ) -> Result<Self, TrajoptError> {
        for constraint in &path_constraints {
            validate_lgr_path_constraint(&self.layout, constraint)?;
        }
        self.path_constraints = path_constraints;
        Ok(self)
    }

    /// Add hard path constraints to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a hard path constraint references an
    /// invalid collocation sample or component.
    pub fn with_hard_path_constraints(
        mut self,
        hard_path_constraints: Vec<LgrPseudospectralHardPathConstraint>,
    ) -> Result<Self, TrajoptError> {
        for constraint in &hard_path_constraints {
            validate_lgr_hard_path_constraint(&self.layout, constraint)?;
        }
        self.hard_path_constraints = hard_path_constraints;
        Ok(self)
    }

    /// Add nonlinear hard path constraints to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a nonlinear hard path constraint
    /// references an invalid collocation sample or declares malformed bounds.
    pub fn with_nonlinear_hard_path_constraints(
        mut self,
        nonlinear_hard_path_constraints: Vec<LgrPseudospectralNonlinearHardPathConstraint>,
    ) -> Result<Self, TrajoptError> {
        for constraint in &nonlinear_hard_path_constraints {
            validate_lgr_nonlinear_hard_path_constraint(&self.layout, constraint)?;
        }
        self.nonlinear_hard_path_constraints = nonlinear_hard_path_constraints;
        Ok(self)
    }

    /// Add LGR-weighted quadratic control costs to this fixed mesh.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when a control-cost component is invalid.
    pub fn with_quadratic_control_costs(
        mut self,
        quadratic_control_costs: Vec<LgrPseudospectralQuadraticControlCost>,
    ) -> Result<Self, TrajoptError> {
        for cost in &quadratic_control_costs {
            validate_lgr_quadratic_control_cost(&self.layout, cost)?;
        }
        self.quadratic_control_costs = quadratic_control_costs;
        Ok(self)
    }

    /// Evaluate the scalar fixed-time LGR objective with no Mayer terminal
    /// cost.
    ///
    /// The objective is the LGR quadrature of the caller-supplied Lagrange
    /// integrand plus any configured quadratic control-effort costs.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, Lagrange callback,
    /// objective terms, or report shape are invalid.
    pub fn objective_report<L>(
        &self,
        decision: &[f64],
        lagrange: &mut L,
    ) -> Result<LgrPseudospectralObjectiveReport, TrajoptError>
    where
        L: CollocationLagrange,
    {
        let mut mayer = |_terminal_time_s: f64, _terminal_state: &[f64]| Ok(0.0);
        self.objective_report_with_mayer(decision, lagrange, &mut mayer)
    }

    /// Evaluate the scalar fixed-time LGR objective with a Mayer terminal cost.
    ///
    /// The objective is the LGR quadrature of the caller-supplied Lagrange
    /// integrand plus configured quadratic control-effort costs plus
    /// `phi(x_f, t_f)` from `mayer`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, Lagrange callback,
    /// Mayer callback, objective terms, or report shape are invalid.
    pub fn objective_report_with_mayer<L, M>(
        &self,
        decision: &[f64],
        lagrange: &mut L,
        mayer: &mut M,
    ) -> Result<LgrPseudospectralObjectiveReport, TrajoptError>
    where
        L: CollocationLagrange,
        M: LgrPseudospectralMayerCost,
    {
        validate_lgr_fixed_mesh_ocp(self)?;
        validate_lgr_decision_vector(&self.layout, decision)?;

        let lagrange_quadrature_cost = lgr_lagrange_quadrature_cost(self, decision, lagrange)?;
        let quadratic_control_cost = lgr_quadratic_control_cost(self, decision)?;
        let terminal_time_s =
            *self
                .layout
                .time_nodes_s
                .last()
                .ok_or(TrajoptError::InvalidPayload {
                    reason: "LGR objective terminal time is unavailable",
                })?;
        let terminal_state = self
            .layout
            .mesh_node_state(decision, self.layout.segment_count())?
            .to_vec();
        let mayer_cost = mayer.eval(terminal_time_s, &terminal_state)?;
        if !mayer_cost.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR Mayer objective value must be finite",
            });
        }
        let total_objective = lagrange_quadrature_cost + quadratic_control_cost + mayer_cost;
        let report = LgrPseudospectralObjectiveReport {
            terminal_time_s,
            terminal_state,
            lagrange_quadrature_cost,
            quadratic_control_cost,
            mayer_cost,
            total_objective,
        };
        validate_lgr_objective_report(&report)?;
        Ok(report)
    }

    /// Split KKT residual multipliers by fixed-mesh LGR row family.
    ///
    /// Differential residual multipliers are split segment-major and
    /// collocation-major, with one state-dimension vector per LGR collocation
    /// node. This exposes deterministic raw multiplier evidence for later
    /// costate/PMP witnesses while preserving the exact residual-row ordering.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when `residual_report` does not match this OCP
    /// layout or `residual_multipliers` has malformed dimensions/values.
    pub fn kkt_residual_multiplier_report(
        &self,
        residual_report: &LgrPseudospectralFixedMeshResidualReport,
        residual_multipliers: &[f64],
    ) -> Result<LgrPseudospectralKktResidualMultiplierReport, TrajoptError> {
        assemble_lgr_kkt_residual_multiplier_report(self, residual_report, residual_multipliers)
    }

    /// Split dense-QP equality duals by fixed-mesh LGR KKT row family.
    ///
    /// Dense quadratic-control QPs include differential, continuity, and
    /// boundary residuals as equality rows while quadratic-control costs live in
    /// the QP Hessian. This adapter zero-fills non-equality residual families
    /// and feeds solver-produced equality duals into the existing raw LGR KKT
    /// multiplier splitter.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report does not match this
    /// OCP layout or its equality dual vector is malformed.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_kkt_residual_multiplier_report(
        &self,
        report: &LgrPseudospectralFixedMeshDenseQuadraticQpReport,
    ) -> Result<LgrPseudospectralKktResidualMultiplierReport, TrajoptError> {
        validate_lgr_fixed_mesh_residual_report_for_ocp(self, &report.residual_report)?;
        let residual_multipliers = lgr_dense_qp_residual_multipliers_from_equality_duals(report)?;
        self.kkt_residual_multiplier_report(&report.residual_report, &residual_multipliers)
    }

    /// Estimate raw collocation costates from dense-QP equality duals.
    ///
    /// Returned values remain raw direct-method diagnostics; callers must choose
    /// an explicit PMP sign/scale convention before comparing them with an
    /// indirect solution.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP multiplier report is malformed
    /// or does not match this OCP layout.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_kkt_collocation_costate_estimate_report(
        &self,
        report: &LgrPseudospectralFixedMeshDenseQuadraticQpReport,
    ) -> Result<LgrPseudospectralKktCostateEstimateReport, TrajoptError> {
        let residual_multipliers = lgr_dense_qp_residual_multipliers_from_equality_duals(report)?;
        self.kkt_collocation_costate_estimate_report(&report.residual_report, &residual_multipliers)
    }

    /// Apply an explicit PMP convention to dense-QP-derived raw collocation
    /// costates.
    ///
    /// This is a bridge from solver-produced equality duals to the common PMP
    /// comparison shape. It does not infer or certify a production covector
    /// convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report is malformed or the PMP
    /// convention is invalid.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_pmp_costate_convention_report(
        &self,
        report: &LgrPseudospectralFixedMeshDenseQuadraticQpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        self.dense_qp_kkt_collocation_costate_estimate_report(report)?
            .pmp_costate_convention_report(convention)
    }

    /// Normalize dense-QP LGR collocation duals by Radau quadrature weight and
    /// apply an explicit PMP convention.
    ///
    /// LGR differential-row multipliers carry the collocation quadrature weight
    /// in the scalar minimum-energy benchmark. This adapter removes that known
    /// pseudospectral row weight before applying the caller-selected sign/scale
    /// convention, producing the common PMP comparison shape without inferring a
    /// production covector convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report, LGR grid weights, or
    /// selected PMP convention are malformed.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_pmp_quadrature_weight_normalized_costate_convention_report(
        &self,
        report: &LgrPseudospectralFixedMeshDenseQuadraticQpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        let costate_report = self.dense_qp_kkt_collocation_costate_estimate_report(report)?;
        lgr_quadrature_weight_normalized_costate_convention_report(
            &self.layout,
            &costate_report,
            convention,
        )
    }

    /// Evaluate a PMP Hamiltonian profile from quadrature-normalized dense-QP
    /// LGR collocation costates.
    ///
    /// This maps solver-produced equality duals through the LGR quadrature
    /// weight normalization plus caller-selected PMP costate convention, samples
    /// solved collocation states and controls, and evaluates
    /// `H = L + lambda' f` through the common PMP witness helper.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense-QP report, LGR grid weights,
    /// selected PMP convention, solved decision vector, or dynamics/Lagrange
    /// callback outputs are malformed.
    #[cfg(feature = "sqp")]
    pub fn dense_qp_pmp_quadrature_weight_normalized_hamiltonian_report<D, L>(
        &self,
        report: &LgrPseudospectralFixedMeshDenseQuadraticQpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let costate_report = self.dense_qp_kkt_collocation_costate_estimate_report(report)?;
        let convention_report = lgr_quadrature_weight_normalized_costate_convention_report(
            &self.layout,
            &costate_report,
            convention,
        )?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for (sample_index, sample) in costate_report.collocation_samples.iter().enumerate() {
            if sample.time_s.to_bits() != convention_report.times_s[sample_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR dense-QP PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(self.layout.segment_support_state(
                &report.qp_solution.primal,
                sample.segment_index,
                sample.collocation_index,
            )?);
            controls.extend_from_slice(self.layout.segment_collocation_control(
                &report.qp_solution.primal,
                sample.segment_index,
                sample.collocation_index,
            )?);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Estimate raw collocation costates from LGR differential multipliers.
    ///
    /// Each LGR collocation sample receives the raw multiplier vector associated
    /// with its differential residual rows. The returned values are unscaled so a
    /// later PMP verifier can apply the selected covector sign/scaling convention
    /// explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the residual or multiplier dimensions do not
    /// match this OCP layout.
    pub fn kkt_collocation_costate_estimate_report(
        &self,
        residual_report: &LgrPseudospectralFixedMeshResidualReport,
        residual_multipliers: &[f64],
    ) -> Result<LgrPseudospectralKktCostateEstimateReport, TrajoptError> {
        let multiplier_report =
            self.kkt_residual_multiplier_report(residual_report, residual_multipliers)?;
        assemble_lgr_kkt_collocation_costate_estimate_report(&multiplier_report)
    }

    /// Evaluate raw collocation Hamiltonian diagnostics from KKT costate estimates.
    ///
    /// This computes `H = L + lambda' f` at LGR collocation nodes using the
    /// unscaled costate estimates produced from the residual multipliers. It is a
    /// deterministic diagnostic substrate for a later PMP witness, not a proof
    /// that Hamiltonian constancy or a final covector convention holds.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, residual report,
    /// multipliers, dynamics, Lagrange callback, or generated dimensions are
    /// invalid.
    pub fn kkt_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        residual_report: &LgrPseudospectralFixedMeshResidualReport,
        residual_multipliers: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<LgrPseudospectralKktHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_decision_vector(&self.layout, decision)?;
        let costate_report =
            self.kkt_collocation_costate_estimate_report(residual_report, residual_multipliers)?;
        assemble_lgr_kkt_hamiltonian_report(self, decision, &costate_report, dynamics, lagrange)
    }

    /// Assemble and solve one sparse Gauss-Newton normal-equation step.
    ///
    /// This evaluates fixed-mesh residuals and deterministic central finite-
    /// difference sparse Jacobian at `decision`, forms `J'J + regularization I`,
    /// assembles `-J'r`, and solves the normal system with conjugate gradients.
    /// It exposes sparse matrix artifacts and the linearized residual norm.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian assembly, sparse normal
    /// block assembly, or the CG solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn sparse_gauss_newton_step_report<D, L>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        hessian_regularization: f64,
        hessian_drop_tolerance: f64,
        cg_max_iterations: usize,
        cg_residual_tolerance: f64,
    ) -> Result<LgrPseudospectralFixedMeshSparseGaussNewtonStepReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let residual_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, decision, dynamics, lagrange)?;
        let sparse_jacobian = self.sparse_residual_jacobian_report(
            decision,
            dynamics,
            lagrange,
            finite_difference_step,
            jacobian_drop_tolerance,
        )?;
        assemble_lgr_sparse_gauss_newton_step_report(
            residual_report,
            sparse_jacobian,
            hessian_regularization,
            hessian_drop_tolerance,
            cg_max_iterations,
            cg_residual_tolerance,
        )
    }

    /// Assemble and solve one sparse augmented-KKT linearized least-squares step.
    ///
    /// This evaluates residuals and a sparse finite-difference Jacobian at
    /// `decision`, builds `[regularization * I, J'; J, -I]`, and solves the KKT
    /// system with deterministic LDL factorization. The residual-space solution
    /// block is exposed as raw KKT multipliers for the LGR diagnostic methods.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian assembly, KKT assembly, or
    /// the LDL solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn sparse_augmented_kkt_step_report<D, L>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        kkt_regularization: f64,
        kkt_drop_tolerance: f64,
    ) -> Result<LgrPseudospectralFixedMeshSparseKktStepReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let residual_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, decision, dynamics, lagrange)?;
        let sparse_jacobian = self.sparse_residual_jacobian_report(
            decision,
            dynamics,
            lagrange,
            finite_difference_step,
            jacobian_drop_tolerance,
        )?;
        assemble_lgr_sparse_augmented_kkt_step_report(
            residual_report,
            sparse_jacobian,
            kkt_regularization,
            kkt_drop_tolerance,
        )
    }

    /// Assemble and solve one sparse constrained-KKT linearized least-squares
    /// step using active scalar hard-path inequalities.
    ///
    /// This evaluates residuals, a sparse residual Jacobian, and dense hard
    /// scalar path inequalities at `decision`. Rows with `Aineq * decision -
    /// rhs >= -active_tolerance` are inserted into the KKT system as active
    /// linearized equalities. The report exposes active row indices, active
    /// inequality multipliers, and linearized active-row closure. This is an
    /// active-set constrained sparse-KKT step substrate, not a full inequality
    /// globalization or production large-sparse NLP backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual/Jacobian/inequality assembly, KKT
    /// assembly, active-row selection, or the LDL solve fails.
    #[allow(clippy::too_many_arguments)]
    pub fn sparse_constrained_kkt_step_report<D, L>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        kkt_regularization: f64,
        kkt_drop_tolerance: f64,
        active_tolerance: f64,
    ) -> Result<LgrPseudospectralFixedMeshSparseConstrainedKktStepReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        if !active_tolerance.is_finite() || active_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse constrained KKT active tolerance must be finite and non-negative",
            });
        }
        let residual_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, decision, dynamics, lagrange)?;
        let sparse_jacobian = self.sparse_residual_jacobian_report(
            decision,
            dynamics,
            lagrange,
            finite_difference_step,
            jacobian_drop_tolerance,
        )?;
        let hard_path_inequality_report = self.hard_path_inequality_report()?;
        assemble_lgr_sparse_constrained_kkt_step_report(
            residual_report,
            sparse_jacobian,
            hard_path_inequality_report,
            decision,
            kkt_regularization,
            kkt_drop_tolerance,
            active_tolerance,
        )
    }

    /// Solve the fixed-mesh LGR residual model with a sparse augmented-KKT SQP loop.
    ///
    /// Each accepted iteration evaluates residuals/Jacobians at the current
    /// decision, solves the sparse augmented KKT system, and applies a
    /// backtracking line search on residual norm. The final report includes a
    /// sparse KKT solve assembled at the accepted final decision so residual
    /// multipliers come from the final sparse loop state. This convenience
    /// method uses a wide finite default decision-bound envelope before merging
    /// scalar hard-path constraints.
    ///
    /// This is a residual-only sparse SQP substrate with scalar hard-path bound
    /// enforcement; full inequality multiplier and constrained large-sparse NLP
    /// features remain separate work.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings, residual/Jacobian/KKT assembly,
    /// line-search evaluations, or final multiplier assembly fail.
    pub fn solve_sparse_kkt_sqp<D, L>(
        &self,
        initial_decision: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        settings: SparseKktSqpSettings,
    ) -> Result<LgrPseudospectralFixedMeshSparseKktSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let (default_lower_bounds, default_upper_bounds) =
            sparse_kkt_sqp_default_decision_bounds(self.layout.variable_count);
        self.solve_sparse_kkt_sqp_with_decision_bounds(
            initial_decision,
            &default_lower_bounds,
            &default_upper_bounds,
            dynamics,
            lagrange,
            settings,
        )
    }

    /// Solve the fixed-mesh LGR residual model with a bounded sparse
    /// augmented-KKT SQP loop.
    ///
    /// Scalar hard-path constraints are merged into the supplied finite default
    /// decision bounds. Initial decisions and line-search trial decisions must
    /// satisfy those merged bounds within the residual tolerance, so hard scalar
    /// path bounds are enforced even though the sparse KKT system itself still
    /// contains only residual rows.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings, decision bounds, residual/Jacobian
    /// assembly, line-search evaluations, or final multiplier assembly fail.
    pub fn solve_sparse_kkt_sqp_with_decision_bounds<D, L>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        settings: SparseKktSqpSettings,
    ) -> Result<LgrPseudospectralFixedMeshSparseKktSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        settings.validate()?;
        validate_lgr_decision_vector(&self.layout, initial_decision)?;
        if !self.nonlinear_hard_path_constraints.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse KKT SQP with nonlinear hard path constraints requires a path callback",
            });
        }
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        if decision_bounds.max_violation(initial_decision)? > settings.residual_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse KKT SQP initial decision violates bounds",
            });
        }

        let mut decision = initial_decision.to_vec();
        let mut residual_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, &decision, dynamics, lagrange)?;
        let mut iterations = Vec::new();
        let mut converged = residual_report.residual_norm <= settings.residual_tolerance;

        for iteration in 0..settings.max_iterations {
            if converged {
                break;
            }
            let step_report = self.sparse_augmented_kkt_step_report(
                &decision,
                dynamics,
                lagrange,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
            )?;
            if step_report.step_norm <= settings.step_tolerance {
                break;
            }

            let mut scale = 1.0_f64;
            let mut accepted = None;
            for _ in 0..settings.max_line_search_backtracks {
                let trial = scaled_decision_step(&decision, &step_report.step, scale)?;
                validate_lgr_decision_vector(&self.layout, &trial)?;
                if decision_bounds.max_violation(&trial)? > settings.residual_tolerance {
                    scale *= 0.5;
                    if scale < settings.min_line_search_scale {
                        break;
                    }
                    continue;
                }
                let trial_report = lgr_fixed_mesh_residual_report_with_callbacks(
                    self, &trial, dynamics, lagrange,
                )?;
                if trial_report.residual_norm < residual_report.residual_norm
                    || trial_report.residual_norm <= settings.residual_tolerance
                {
                    accepted = Some((trial, trial_report, scale));
                    break;
                }
                scale *= 0.5;
                if scale < settings.min_line_search_scale {
                    break;
                }
            }
            let Some((accepted_decision, accepted_residual_report, accepted_scale)) = accepted
            else {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR sparse KKT SQP line search failed to reduce residual",
                });
            };
            iterations.push(SparseKktSqpIterationReport {
                iteration,
                residual_norm: residual_report.residual_norm,
                linearized_residual_norm: step_report.linearized_residual_norm,
                step_norm: step_report.step_norm,
                accepted_step_scale: accepted_scale,
                accepted_residual_norm: accepted_residual_report.residual_norm,
            });
            decision = accepted_decision;
            residual_report = accepted_residual_report;
            converged = residual_report.residual_norm <= settings.residual_tolerance;
        }

        let final_kkt_step_report = self.sparse_augmented_kkt_step_report(
            &decision,
            dynamics,
            lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )?;
        let max_decision_bound_violation = decision_bounds.max_violation(&decision)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let final_constrained_kkt_step_report = lgr_sparse_constrained_kkt_step_report_if_active(
            &decision,
            settings,
            &hard_path_inequalities,
            &final_kkt_step_report,
        )?;
        let max_hard_path_violation = hard_path_inequalities.max_violation(&decision)?;
        Ok(LgrPseudospectralFixedMeshSparseKktSqpReport {
            decision,
            residual_report,
            final_kkt_step_report,
            final_constrained_kkt_step_report,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
            nonlinear_hard_path_report: None,
            max_nonlinear_hard_path_violation: 0.0,
            iterations,
            converged,
        })
    }

    /// Solve the fixed-mesh LGR residual model with a sparse augmented-KKT SQP
    /// loop while preserving nonlinear hard-path feasibility.
    ///
    /// Scalar hard-path constraints are merged into finite decision bounds.
    /// Nonlinear hard-path constraints are evaluated through `path_function` at
    /// the initial decision and every line-search trial. The sparse KKT system
    /// still contains only residual rows, so this is a feasibility-preserving
    /// constrained sparse-SQP substrate rather than a full inequality-multiplier
    /// large-sparse NLP solve.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings, decision bounds,
    /// residual/Jacobian/KKT assembly, nonlinear hard-path evaluation,
    /// line-search evaluations, or final multiplier assembly fail.
    #[allow(clippy::too_many_arguments)]
    pub fn solve_sparse_kkt_sqp_with_nonlinear_hard_path_constraints<D, L, P>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        path_function: &mut P,
        settings: SparseKktSqpSettings,
    ) -> Result<LgrPseudospectralFixedMeshSparseKktSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
        P: CollocationPathFunction,
    {
        settings.validate()?;
        validate_lgr_decision_vector(&self.layout, initial_decision)?;
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        if decision_bounds.max_violation(initial_decision)? > settings.residual_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse KKT SQP initial decision violates bounds",
            });
        }
        let mut nonlinear_hard_path_report =
            self.nonlinear_hard_path_inequality_report(initial_decision, path_function)?;
        if nonlinear_hard_path_report.max_violation > settings.residual_tolerance {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse KKT SQP initial decision violates nonlinear hard path constraints",
            });
        }

        let mut decision = initial_decision.to_vec();
        let mut residual_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, &decision, dynamics, lagrange)?;
        let mut iterations = Vec::new();
        let mut converged = residual_report.residual_norm <= settings.residual_tolerance
            && nonlinear_hard_path_report.max_violation <= settings.residual_tolerance;

        for iteration in 0..settings.max_iterations {
            if converged {
                break;
            }
            let step_report = self.sparse_augmented_kkt_step_report(
                &decision,
                dynamics,
                lagrange,
                settings.finite_difference_step,
                settings.jacobian_drop_tolerance,
                settings.kkt_regularization,
                settings.kkt_drop_tolerance,
            )?;
            if step_report.step_norm <= settings.step_tolerance {
                break;
            }

            let mut scale = 1.0_f64;
            let mut accepted = None;
            for _ in 0..settings.max_line_search_backtracks {
                let trial = scaled_decision_step(&decision, &step_report.step, scale)?;
                validate_lgr_decision_vector(&self.layout, &trial)?;
                if decision_bounds.max_violation(&trial)? > settings.residual_tolerance {
                    scale *= 0.5;
                    if scale < settings.min_line_search_scale {
                        break;
                    }
                    continue;
                }
                let trial_nonlinear_report =
                    self.nonlinear_hard_path_inequality_report(&trial, path_function)?;
                if trial_nonlinear_report.max_violation > settings.residual_tolerance {
                    scale *= 0.5;
                    if scale < settings.min_line_search_scale {
                        break;
                    }
                    continue;
                }
                let trial_report = lgr_fixed_mesh_residual_report_with_callbacks(
                    self, &trial, dynamics, lagrange,
                )?;
                if trial_report.residual_norm < residual_report.residual_norm
                    || trial_report.residual_norm <= settings.residual_tolerance
                {
                    accepted = Some((trial, trial_report, trial_nonlinear_report, scale));
                    break;
                }
                scale *= 0.5;
                if scale < settings.min_line_search_scale {
                    break;
                }
            }
            let Some((
                accepted_decision,
                accepted_residual_report,
                accepted_nonlinear_report,
                accepted_scale,
            )) = accepted
            else {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR sparse KKT SQP line search failed to reduce residual while preserving nonlinear hard path constraints",
                });
            };
            iterations.push(SparseKktSqpIterationReport {
                iteration,
                residual_norm: residual_report.residual_norm,
                linearized_residual_norm: step_report.linearized_residual_norm,
                step_norm: step_report.step_norm,
                accepted_step_scale: accepted_scale,
                accepted_residual_norm: accepted_residual_report.residual_norm,
            });
            decision = accepted_decision;
            residual_report = accepted_residual_report;
            nonlinear_hard_path_report = accepted_nonlinear_report;
            converged = residual_report.residual_norm <= settings.residual_tolerance
                && nonlinear_hard_path_report.max_violation <= settings.residual_tolerance;
        }

        let final_kkt_step_report = self.sparse_augmented_kkt_step_report(
            &decision,
            dynamics,
            lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )?;
        let max_decision_bound_violation = decision_bounds.max_violation(&decision)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let final_constrained_kkt_step_report = lgr_sparse_constrained_kkt_step_report_if_active(
            &decision,
            settings,
            &hard_path_inequalities,
            &final_kkt_step_report,
        )?;
        let max_hard_path_violation = hard_path_inequalities.max_violation(&decision)?;
        let max_nonlinear_hard_path_violation = nonlinear_hard_path_report.max_violation;
        Ok(LgrPseudospectralFixedMeshSparseKktSqpReport {
            decision,
            residual_report,
            final_kkt_step_report,
            final_constrained_kkt_step_report,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
            nonlinear_hard_path_report: Some(nonlinear_hard_path_report),
            max_nonlinear_hard_path_violation,
            iterations,
            converged,
        })
    }

    /// Split sparse augmented-KKT residual multipliers by fixed-mesh row family.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse step report does not match this
    /// OCP layout or contains malformed multiplier dimensions.
    pub fn sparse_kkt_residual_multiplier_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
    ) -> Result<LgrPseudospectralKktResidualMultiplierReport, TrajoptError> {
        validate_lgr_sparse_kkt_step_report_for_ocp(self, step_report)?;
        self.kkt_residual_multiplier_report(
            &step_report.residual_report,
            &step_report.residual_multipliers,
        )
    }

    /// Estimate raw collocation costates from sparse augmented-KKT multipliers.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse KKT step report does not match
    /// this OCP layout.
    pub fn sparse_kkt_collocation_costate_estimate_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
    ) -> Result<LgrPseudospectralKktCostateEstimateReport, TrajoptError> {
        validate_lgr_sparse_kkt_step_report_for_ocp(self, step_report)?;
        self.kkt_collocation_costate_estimate_report(
            &step_report.residual_report,
            &step_report.residual_multipliers,
        )
    }

    /// Apply an explicit PMP convention to sparse-KKT-derived raw collocation
    /// costates.
    ///
    /// This is the sparse augmented-KKT analogue of the dense-QP PMP costate
    /// adapter. It packages residual-multiplier-derived collocation costates
    /// into the common PMP comparison shape after the caller selects the
    /// sign/scale convention explicitly.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse KKT report is malformed or the
    /// PMP convention is invalid.
    pub fn sparse_kkt_pmp_costate_convention_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        self.sparse_kkt_collocation_costate_estimate_report(step_report)?
            .pmp_costate_convention_report(convention)
    }

    /// Normalize sparse-KKT LGR collocation costates by Radau quadrature weight
    /// and apply an explicit PMP convention.
    ///
    /// This mirrors the dense-QP LGR adapter for sparse augmented-KKT
    /// multipliers. It removes the collocation quadrature weight before applying
    /// the caller-selected sign/scale convention, without inferring a production
    /// covector convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse KKT report, LGR grid weights, or
    /// selected PMP convention are malformed.
    pub fn sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        let costate_report = self.sparse_kkt_collocation_costate_estimate_report(step_report)?;
        lgr_quadrature_weight_normalized_costate_convention_report(
            &self.layout,
            &costate_report,
            convention,
        )
    }

    /// Evaluate raw collocation Hamiltonian diagnostics from a sparse KKT step.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, sparse KKT report,
    /// dynamics callback, Lagrange callback, or generated dimensions are invalid.
    pub fn sparse_kkt_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<LgrPseudospectralKktHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_sparse_kkt_step_report_for_ocp(self, step_report)?;
        self.kkt_hamiltonian_report(
            decision,
            &step_report.residual_report,
            &step_report.residual_multipliers,
            dynamics,
            lagrange,
        )
    }

    /// Evaluate a PMP-convention Hamiltonian profile from sparse-KKT LGR
    /// collocation costates.
    ///
    /// This maps sparse augmented-KKT residual multipliers through the
    /// caller-selected PMP costate convention, samples collocation states and
    /// controls from `decision`, and evaluates `H = L + lambda' f` through the
    /// common PMP witness helper.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, sparse KKT report,
    /// PMP convention, dynamics callback, or Lagrange callback is malformed.
    pub fn sparse_kkt_pmp_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_sparse_kkt_step_report_for_ocp(self, step_report)?;
        validate_lgr_decision_vector(&self.layout, decision)?;
        let costate_report = self.sparse_kkt_collocation_costate_estimate_report(step_report)?;
        let convention_report = costate_report.pmp_costate_convention_report(convention)?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for (sample_index, sample) in costate_report.collocation_samples.iter().enumerate() {
            if sample.time_s.to_bits() != convention_report.times_s[sample_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR sparse-KKT PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(self.layout.segment_support_state(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
            controls.extend_from_slice(self.layout.segment_collocation_control(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Evaluate a PMP Hamiltonian profile from quadrature-normalized sparse-KKT
    /// LGR collocation costates.
    ///
    /// This maps sparse augmented-KKT residual multipliers through the LGR
    /// quadrature-weight normalization plus caller-selected PMP convention,
    /// samples collocation states and controls from `decision`, and evaluates
    /// `H = L + lambda' f` through the common PMP witness helper.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, sparse KKT report,
    /// LGR weights, PMP convention, dynamics callback, or Lagrange callback is
    /// malformed.
    pub fn sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_sparse_kkt_step_report_for_ocp(self, step_report)?;
        validate_lgr_decision_vector(&self.layout, decision)?;
        let costate_report = self.sparse_kkt_collocation_costate_estimate_report(step_report)?;
        let convention_report = lgr_quadrature_weight_normalized_costate_convention_report(
            &self.layout,
            &costate_report,
            convention,
        )?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for (sample_index, sample) in costate_report.collocation_samples.iter().enumerate() {
            if sample.time_s.to_bits() != convention_report.times_s[sample_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR sparse-KKT normalized PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(self.layout.segment_support_state(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
            controls.extend_from_slice(self.layout.segment_collocation_control(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Split constrained sparse-KKT residual multipliers by fixed-mesh row family.
    ///
    /// Active hard-inequality multipliers remain on the constrained step report;
    /// this adapter consumes the residual-space block so downstream diagnostics
    /// preserve the existing LGR row convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained step report does not match
    /// this OCP layout or contains malformed multiplier dimensions.
    pub fn sparse_constrained_kkt_residual_multiplier_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
    ) -> Result<LgrPseudospectralKktResidualMultiplierReport, TrajoptError> {
        validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, step_report)?;
        self.kkt_residual_multiplier_report(
            &step_report.residual_report,
            &step_report.residual_multipliers,
        )
    }

    /// Estimate raw collocation costates from constrained sparse-KKT multipliers.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained KKT step report does not
    /// match this OCP layout.
    pub fn sparse_constrained_kkt_collocation_costate_estimate_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
    ) -> Result<LgrPseudospectralKktCostateEstimateReport, TrajoptError> {
        validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, step_report)?;
        self.kkt_collocation_costate_estimate_report(
            &step_report.residual_report,
            &step_report.residual_multipliers,
        )
    }

    /// Apply an explicit PMP convention to constrained sparse-KKT raw costates.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained KKT report is malformed or the
    /// PMP convention is invalid.
    pub fn sparse_constrained_kkt_pmp_costate_convention_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        self.sparse_constrained_kkt_collocation_costate_estimate_report(step_report)?
            .pmp_costate_convention_report(convention)
    }

    /// Normalize constrained sparse-KKT LGR costates by Radau quadrature weight and
    /// apply an explicit PMP convention.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the constrained KKT report, LGR grid weights,
    /// or selected PMP convention are malformed.
    pub fn sparse_constrained_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
        &self,
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
    ) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
        let costate_report =
            self.sparse_constrained_kkt_collocation_costate_estimate_report(step_report)?;
        lgr_quadrature_weight_normalized_costate_convention_report(
            &self.layout,
            &costate_report,
            convention,
        )
    }

    /// Evaluate raw collocation Hamiltonian diagnostics from a constrained sparse
    /// KKT step.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT report,
    /// dynamics callback, Lagrange callback, or generated dimensions are invalid.
    pub fn sparse_constrained_kkt_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<LgrPseudospectralKktHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, step_report)?;
        self.kkt_hamiltonian_report(
            decision,
            &step_report.residual_report,
            &step_report.residual_multipliers,
            dynamics,
            lagrange,
        )
    }

    /// Evaluate a PMP-convention Hamiltonian profile from constrained sparse-KKT
    /// LGR collocation costates.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT report,
    /// PMP convention, dynamics callback, or Lagrange callback is malformed.
    pub fn sparse_constrained_kkt_pmp_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, step_report)?;
        validate_lgr_decision_vector(&self.layout, decision)?;
        let costate_report =
            self.sparse_constrained_kkt_collocation_costate_estimate_report(step_report)?;
        let convention_report = costate_report.pmp_costate_convention_report(convention)?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for (sample_index, sample) in costate_report.collocation_samples.iter().enumerate() {
            if sample.time_s.to_bits() != convention_report.times_s[sample_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR constrained sparse-KKT PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(self.layout.segment_support_state(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
            controls.extend_from_slice(self.layout.segment_collocation_control(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Evaluate a PMP Hamiltonian profile from quadrature-normalized constrained
    /// sparse-KKT LGR collocation costates.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT report,
    /// LGR weights, PMP convention, dynamics callback, or Lagrange callback is
    /// malformed.
    pub fn sparse_constrained_kkt_pmp_quadrature_weight_normalized_hamiltonian_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<crate::pmp::PmpHamiltonianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, step_report)?;
        validate_lgr_decision_vector(&self.layout, decision)?;
        let costate_report =
            self.sparse_constrained_kkt_collocation_costate_estimate_report(step_report)?;
        let convention_report = lgr_quadrature_weight_normalized_costate_convention_report(
            &self.layout,
            &costate_report,
            convention,
        )?;
        let mut states =
            Vec::with_capacity(convention_report.sample_count * self.layout.state_dimension);
        let mut controls =
            Vec::with_capacity(convention_report.sample_count * self.layout.control_dimension);
        for (sample_index, sample) in costate_report.collocation_samples.iter().enumerate() {
            if sample.time_s.to_bits() != convention_report.times_s[sample_index].to_bits() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR constrained sparse-KKT normalized PMP Hamiltonian sample time is inconsistent",
                });
            }
            states.extend_from_slice(self.layout.segment_support_state(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
            controls.extend_from_slice(self.layout.segment_collocation_control(
                decision,
                sample.segment_index,
                sample.collocation_index,
            )?);
        }
        crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: self.layout.state_dimension,
                control_dimension: self.layout.control_dimension,
                times_s: &convention_report.times_s,
                states: &states,
                controls: &controls,
                costates: &convention_report.pmp_costates,
            },
            |time_s, state, control| dynamics.eval(time_s, state, control),
            |time_s, state, control| lagrange.eval(time_s, state, control),
        )
    }

    /// Build quadrature-normalized PMP/KKT certificate artifacts from one active
    /// constrained-KKT step.
    ///
    /// This consumes an already assembled sparse constrained-KKT report, applies
    /// LGR quadrature-weight normalization and the caller-selected PMP costate
    /// convention, evaluates the common Hamiltonian witness, and packages a
    /// validated I-load certificate. The helper makes the active inequality
    /// multiplier handoff auditable without claiming a full production
    /// large-sparse NLP solve.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constrained KKT
    /// report, LGR weights, PMP convention, Hamiltonian callbacks, or declared
    /// certificate tolerance are malformed.
    pub fn sparse_constrained_kkt_pmp_kkt_quadrature_weight_normalized_certificate_report<D, L>(
        &self,
        decision: &[f64],
        step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        hamiltonian_drift_tolerance: f64,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<LgrPseudospectralFixedMeshSparseConstrainedKktPmpKktCertificateReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_decision_vector(&self.layout, decision)?;
        validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, step_report)?;
        let costate_convention_report = self
            .sparse_constrained_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                step_report,
                convention,
            )?;
        let hamiltonian_report = self
            .sparse_constrained_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                decision,
                step_report,
                convention,
                dynamics,
                lagrange,
            )?;
        let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt,
            &hamiltonian_report,
            hamiltonian_drift_tolerance,
        )?;
        Ok(
            LgrPseudospectralFixedMeshSparseConstrainedKktPmpKktCertificateReport {
                constrained_kkt_step_report: step_report.clone(),
                costate_convention_report,
                hamiltonian_report,
                control_stationarity_report: None,
                costate_comparison_report: None,
                optimality_certificate,
            },
        )
    }

    /// Build quadrature-normalized PMP/KKT certificate artifacts from an
    /// accepted dense SQP solution.
    ///
    /// This assembles a sparse augmented-KKT report at the final dense SQP
    /// decision, applies the LGR quadrature-weight normalization and the
    /// caller-selected PMP costate convention, evaluates the common Hamiltonian
    /// witness, and packages a validated I-load certificate. The helper is a
    /// final-solution audit surface for the dense SQP lane; it does not infer
    /// a covector sign convention or replace a production large-sparse NLP
    /// backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the dense SQP report, sparse KKT assembly,
    /// LGR weights, PMP convention, Hamiltonian callbacks, or declared
    /// certificate tolerance are malformed.
    #[cfg(feature = "sqp")]
    #[allow(clippy::too_many_arguments)]
    pub fn dense_sqp_pmp_kkt_quadrature_weight_normalized_certificate_report<D, L>(
        &self,
        report: &LgrPseudospectralFixedMeshDenseSqpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        hamiltonian_drift_tolerance: f64,
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        jacobian_drop_tolerance: f64,
        kkt_regularization: f64,
        kkt_drop_tolerance: f64,
    ) -> Result<LgrPseudospectralFixedMeshDenseSqpPmpKktCertificateReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_decision_vector(&self.layout, &report.sqp_solution.variables)?;
        let sparse_kkt_step_report = self.sparse_augmented_kkt_step_report(
            &report.sqp_solution.variables,
            dynamics,
            lagrange,
            finite_difference_step,
            jacobian_drop_tolerance,
            kkt_regularization,
            kkt_drop_tolerance,
        )?;
        let costate_convention_report = self
            .sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                &sparse_kkt_step_report,
                convention,
            )?;
        let hamiltonian_report = self
            .sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                &report.sqp_solution.variables,
                &sparse_kkt_step_report,
                convention,
                dynamics,
                lagrange,
            )?;
        let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt,
            &hamiltonian_report,
            hamiltonian_drift_tolerance,
        )?;
        Ok(LgrPseudospectralFixedMeshDenseSqpPmpKktCertificateReport {
            sparse_kkt_step_report,
            costate_convention_report,
            hamiltonian_report,
            control_stationarity_report: None,
            costate_comparison_report: None,
            optimality_certificate,
        })
    }

    /// Build quadrature-normalized PMP/KKT certificate artifacts from an
    /// accepted sparse-KKT SQP report.
    ///
    /// This consumes the final sparse-KKT multipliers stored by the sparse SQP
    /// outer loop, applies the LGR quadrature-weight normalization and the
    /// caller-selected PMP convention, evaluates the common Hamiltonian witness,
    /// and packages a validated I-load certificate. If the sparse SQP report
    /// carries a final constrained-KKT active-row report, the PMP witness uses
    /// those constrained multipliers; otherwise it uses the final augmented-KKT
    /// multipliers.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the sparse SQP report is malformed, does not
    /// match this OCP layout, the PMP convention or callbacks are invalid, or the
    /// declared certificate tolerance is not met.
    pub fn sparse_kkt_sqp_pmp_kkt_quadrature_weight_normalized_certificate_report<D, L>(
        &self,
        report: &LgrPseudospectralFixedMeshSparseKktSqpReport,
        convention: crate::pmp::PmpDirectCostateConvention,
        hamiltonian_drift_tolerance: f64,
        dynamics: &mut D,
        lagrange: &mut L,
    ) -> Result<LgrPseudospectralFixedMeshSparseKktSqpPmpKktCertificateReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_decision_vector(&self.layout, &report.decision)?;
        validate_lgr_sparse_kkt_step_report_for_ocp(self, &report.final_kkt_step_report)?;
        if report.final_kkt_step_report.residual_report != report.residual_report {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse SQP final KKT report does not match the final residual report",
            });
        }

        let (costate_convention_report, hamiltonian_report, used_constrained_kkt_step_report) =
            if let Some(constrained_report) = &report.final_constrained_kkt_step_report {
                validate_lgr_sparse_constrained_kkt_step_report_for_ocp(self, constrained_report)?;
                if constrained_report.residual_report != report.residual_report {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "LGR sparse SQP final constrained KKT report does not match the final residual report",
                    });
                }
                (
                    self
                        .sparse_constrained_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                            constrained_report,
                            convention,
                        )?,
                    self.sparse_constrained_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                        &report.decision,
                        constrained_report,
                        convention,
                        dynamics,
                        lagrange,
                    )?,
                    true,
                )
            } else {
                (
                    self.sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                        &report.final_kkt_step_report,
                        convention,
                    )?,
                    self.sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                        &report.decision,
                        &report.final_kkt_step_report,
                        convention,
                        dynamics,
                        lagrange,
                    )?,
                    false,
                )
            };

        let optimality_certificate = ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt,
            &hamiltonian_report,
            hamiltonian_drift_tolerance,
        )?;
        Ok(
            LgrPseudospectralFixedMeshSparseKktSqpPmpKktCertificateReport {
                sparse_kkt_step_report: report.final_kkt_step_report.clone(),
                final_constrained_kkt_step_report: report.final_constrained_kkt_step_report.clone(),
                used_constrained_kkt_step_report,
                costate_convention_report,
                hamiltonian_report,
                control_stationarity_report: None,
                costate_comparison_report: None,
                optimality_certificate,
            },
        )
    }

    /// Decode `decision` into a multi-segment LGR transcription.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector is invalid.
    pub fn transcription_from_decision(
        &self,
        decision: &[f64],
    ) -> Result<LgrPseudospectralTranscription, TrajoptError> {
        self.layout.transcription_from_decision(decision)
    }

    /// Emit a deterministic optimized reference profile from LGR support-state
    /// samples.
    ///
    /// Samples are ordered by physical time across every segment support node.
    /// Shared segment boundaries are emitted once, from the earlier segment
    /// endpoint. This is an output formatting surface only; it adds no target
    /// vocabulary.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector or state-component
    /// mapping is invalid, or emitted samples are malformed.
    pub fn optimized_reference_profile(
        &self,
        decision: &[f64],
        mapping: LgrPseudospectralReferenceProfileMapping,
    ) -> Result<LgrPseudospectralReferenceProfileReport, TrajoptError> {
        validate_lgr_fixed_mesh_ocp(self)?;
        validate_lgr_decision_vector(&self.layout, decision)?;
        validate_lgr_reference_profile_mapping(&self.layout, mapping)?;

        let mut samples = Vec::with_capacity(
            self.layout
                .segment_grids
                .iter()
                .map(|grid| grid.support_nodes.len())
                .sum::<usize>()
                - self.layout.segment_count().saturating_sub(1),
        );
        for segment_index in 0..self.layout.segment_count() {
            let initial_time_s = self.layout.time_nodes_s[segment_index];
            let final_time_s = self.layout.time_nodes_s[segment_index + 1];
            let first_support_index = usize::from(segment_index > 0);
            for support_index in
                first_support_index..self.layout.segment_grids[segment_index].support_nodes.len()
            {
                let normalized_node =
                    self.layout.segment_grids[segment_index].support_nodes[support_index];
                let time_s = physical_time(initial_time_s, final_time_s, normalized_node);
                samples.push(lgr_reference_profile_sample_from_state(
                    time_s,
                    self.layout
                        .segment_support_state(decision, segment_index, support_index)?,
                    mapping,
                )?);
            }
        }

        let sample_count = samples.len();
        let start_time_s = samples
            .first()
            .ok_or(TrajoptError::InvalidPayload {
                reason: "LGR reference profile must contain samples",
            })?
            .time_s;
        let end_time_s = samples
            .last()
            .ok_or(TrajoptError::InvalidPayload {
                reason: "LGR reference profile must contain samples",
            })?
            .time_s;
        let report = LgrPseudospectralReferenceProfileReport {
            samples,
            sample_count,
            start_time_s,
            end_time_s,
        };
        validate_lgr_reference_profile_report(&report)?;
        Ok(report)
    }

    /// Package an optimized fixed-mesh LGR reference profile as an I-load
    /// payload.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when reference-profile extraction or I-load
    /// payload validation fails.
    #[allow(clippy::too_many_arguments)]
    pub fn optimized_reference_iload_payload(
        &self,
        decision: &[f64],
        mapping: LgrPseudospectralReferenceProfileMapping,
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
        metadata: SynthesisMetadata,
        event_bindings_postcard: Vec<u8>,
        gain_tables: Vec<GainTable>,
    ) -> Result<ILoadPayload, TrajoptError> {
        self.optimized_reference_iload_payload_with_optional_certificate(
            decision,
            mapping,
            terminal_condition,
            synthesis_seed,
            producer,
            metadata,
            event_bindings_postcard,
            gain_tables,
            None,
        )
    }

    /// Package an optimized fixed-mesh LGR reference profile as an I-load
    /// payload with attached optimality evidence.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when reference-profile extraction, certificate
    /// validation, or I-load payload validation fails.
    #[allow(clippy::too_many_arguments)]
    pub fn optimized_reference_iload_payload_with_optimality_certificate(
        &self,
        decision: &[f64],
        mapping: LgrPseudospectralReferenceProfileMapping,
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
        metadata: SynthesisMetadata,
        event_bindings_postcard: Vec<u8>,
        gain_tables: Vec<GainTable>,
        optimality_certificate: ILoadOptimalityCertificate,
    ) -> Result<ILoadPayload, TrajoptError> {
        self.optimized_reference_iload_payload_with_optional_certificate(
            decision,
            mapping,
            terminal_condition,
            synthesis_seed,
            producer,
            metadata,
            event_bindings_postcard,
            gain_tables,
            Some(optimality_certificate),
        )
    }

    #[allow(clippy::too_many_arguments)]
    fn optimized_reference_iload_payload_with_optional_certificate(
        &self,
        decision: &[f64],
        mapping: LgrPseudospectralReferenceProfileMapping,
        terminal_condition: &TerminalCondition,
        synthesis_seed: u64,
        producer: String,
        metadata: SynthesisMetadata,
        event_bindings_postcard: Vec<u8>,
        gain_tables: Vec<GainTable>,
        optimality_certificate: Option<ILoadOptimalityCertificate>,
    ) -> Result<ILoadPayload, TrajoptError> {
        let reference_profile = self.optimized_reference_profile(decision, mapping)?.samples;
        let payload = ILoadPayload {
            header: ILoadHeader::new_with_objective(
                terminal_condition,
                ILoadObjectiveClass::OptimizedReference,
                synthesis_seed,
                producer,
            ),
            metadata,
            event_bindings_postcard,
            gain_tables,
            reference_profile,
            optimality_certificate,
        };
        payload.validate()?;
        Ok(payload)
    }

    /// Transcribe hard path bounds as dense linear inequalities.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when any hard path constraint is invalid.
    pub fn hard_path_inequality_report(
        &self,
    ) -> Result<LgrPseudospectralPathInequalityReport, TrajoptError> {
        validate_lgr_fixed_mesh_ocp(self)?;
        let inequality_count = self
            .hard_path_constraints
            .iter()
            .map(LgrPseudospectralHardPathConstraint::inequality_count)
            .sum();
        let variable_count = self.layout.variable_count;
        let mut inequality_matrix = vec![0.0; inequality_count * variable_count];
        let mut inequality_rhs = Vec::with_capacity(inequality_count);
        let mut row = 0;

        for constraint in &self.hard_path_constraints {
            let column = lgr_scalar_decision_index(
                &self.layout,
                constraint.segment_index,
                constraint.collocation_index,
                constraint.variable,
            )?;
            if let Some(upper) = constraint.upper {
                inequality_matrix[row * variable_count + column] = 1.0;
                inequality_rhs.push(upper);
                row += 1;
            }
            if let Some(lower) = constraint.lower {
                inequality_matrix[row * variable_count + column] = -1.0;
                inequality_rhs.push(-lower);
                row += 1;
            }
        }

        let report = LgrPseudospectralPathInequalityReport {
            variable_count,
            inequality_count,
            inequality_matrix,
            inequality_rhs,
        };
        validate_lgr_path_inequality_report(&report)?;
        Ok(report)
    }

    /// Merge hard scalar path constraints into finite decision-variable bounds.
    ///
    /// The returned bounds are directly consumable by the dense SQP lane. This
    /// is possible because current hard path constraints are scalar state or
    /// control bounds at LGR collocation decision variables.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the default bounds are malformed, any hard
    /// path constraint is invalid, or the merged bounds are contradictory.
    pub fn decision_bounds_with_hard_path_constraints(
        &self,
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
    ) -> Result<LgrPseudospectralDecisionBoundsReport, TrajoptError> {
        validate_lgr_decision_bounds(&self.layout, default_lower_bounds, default_upper_bounds)?;
        let variable_count = self.layout.variable_count;
        let mut lower_bounds = default_lower_bounds.to_vec();
        let mut upper_bounds = default_upper_bounds.to_vec();
        let mut constrained_variables = vec![false; variable_count];
        let mut applied_bound_count = 0;

        for constraint in &self.hard_path_constraints {
            validate_lgr_hard_path_constraint(&self.layout, constraint)?;
            let column = lgr_scalar_decision_index(
                &self.layout,
                constraint.segment_index,
                constraint.collocation_index,
                constraint.variable,
            )?;
            if let Some(lower) = constraint.lower {
                lower_bounds[column] = lower_bounds[column].max(lower);
                constrained_variables[column] = true;
                applied_bound_count += 1;
            }
            if let Some(upper) = constraint.upper {
                upper_bounds[column] = upper_bounds[column].min(upper);
                constrained_variables[column] = true;
                applied_bound_count += 1;
            }
            if lower_bounds[column] > upper_bounds[column] {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR hard path bounds conflict with decision bounds",
                });
            }
        }

        let report = LgrPseudospectralDecisionBoundsReport {
            variable_count,
            lower_bounds,
            upper_bounds,
            applied_bound_count,
            constrained_variable_count: constrained_variables
                .iter()
                .filter(|constrained| **constrained)
                .count(),
        };
        validate_lgr_decision_bounds_report(&report)?;
        Ok(report)
    }

    /// Solve the fixed-mesh equality-constrained quadratic-control QP.
    ///
    /// This wrapper forms a dense Clarabel QP from the LGR differential,
    /// continuity, and boundary residual rows; LGR-weighted quadratic control
    /// costs; and scalar hard path inequalities. The equality rows are
    /// linearized at the zero decision vector, so this is exact for affine
    /// dynamics/residual models and is not a general nonlinear NLP solve.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when quadratic controls are absent, unsupported
    /// soft/nonlinear path constraints are present, finite-difference Jacobian
    /// assembly fails, or Clarabel cannot solve the dense QP.
    #[cfg(feature = "sqp")]
    pub fn solve_dense_linearized_quadratic_control_qp<D>(
        &self,
        dynamics: &mut D,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralFixedMeshDenseQuadraticQpReport, TrajoptError>
    where
        D: CollocationDynamics,
    {
        if self.quadratic_control_costs.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR dense quadratic QP requires at least one control cost",
            });
        }
        if !self.path_constraints.is_empty() || !self.nonlinear_hard_path_constraints.is_empty() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR dense quadratic QP supports only scalar hard path constraints",
            });
        }

        let zero_decision = vec![0.0; self.layout.variable_count];
        let mut zero_lagrange = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(0.0);
        let zero_residual_report = lgr_fixed_mesh_residual_report_with_callbacks(
            self,
            &zero_decision,
            dynamics,
            &mut zero_lagrange,
        )?;
        let equality_row_count = zero_residual_report.differential_residual_count
            + zero_residual_report.continuity_residual_count
            + zero_residual_report.boundary_residual_count;
        let equality_rhs = zero_residual_report.residuals[..equality_row_count]
            .iter()
            .map(|residual| -residual)
            .collect::<Vec<_>>();
        let mut zero_lagrange = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(0.0);
        let jacobian = self.sparse_residual_jacobian_report(
            &zero_decision,
            dynamics,
            &mut zero_lagrange,
            finite_difference_step,
            drop_tolerance,
        )?;
        let dense_jacobian = jacobian.to_dense_row_major()?;
        let equality_matrix =
            dense_jacobian[..equality_row_count * self.layout.variable_count].to_vec();
        let hard_path_inequality_report = self.hard_path_inequality_report()?;
        let problem = crate::sqp::DenseQpProblem::new(
            self.layout.variable_count,
            lgr_dense_quadratic_control_hessian(self)?,
            vec![0.0; self.layout.variable_count],
            equality_matrix,
            equality_rhs,
            hard_path_inequality_report.inequality_matrix.clone(),
            hard_path_inequality_report.inequality_rhs.clone(),
        )?;
        let qp_solution = crate::sqp::solve_dense_qp(&problem)?;
        let mut zero_lagrange = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(0.0);
        let residual_report = lgr_fixed_mesh_residual_report_with_callbacks(
            self,
            &qp_solution.primal,
            dynamics,
            &mut zero_lagrange,
        )?;
        let equality_residual_norm = norm(&residual_report.residuals[..equality_row_count]);
        let max_hard_path_violation =
            hard_path_inequality_report.max_violation(&qp_solution.primal)?;

        Ok(LgrPseudospectralFixedMeshDenseQuadraticQpReport {
            qp_solution,
            residual_report,
            hard_path_inequality_report,
            equality_row_count,
            equality_residual_norm,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with the dense SQP lane.
    ///
    /// Hard scalar path constraints are merged into the supplied default
    /// decision bounds before calling the solver. The returned report includes
    /// both the solver result and final residual/bound violation evidence.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when bound merging, residual evaluation, hard
    /// path evaluation, or the dense SQP solve fails.
    #[cfg(feature = "sqp")]
    pub fn solve_dense_sqp<D, L>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        mut dynamics: D,
        mut lagrange: L,
    ) -> Result<LgrPseudospectralFixedMeshDenseSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        let sqp_solution = solver.solve(
            initial_decision,
            &decision_bounds.lower_bounds,
            &decision_bounds.upper_bounds,
            |decision: &[f64]| {
                Ok(lgr_fixed_mesh_residual_report_with_callbacks(
                    self,
                    decision,
                    &mut dynamics,
                    &mut lagrange,
                )?
                .residuals)
            },
        )?;
        let residual_report = lgr_fixed_mesh_residual_report_with_callbacks(
            self,
            &sqp_solution.variables,
            &mut dynamics,
            &mut lagrange,
        )?;
        let max_decision_bound_violation =
            decision_bounds.max_violation(&sqp_solution.variables)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let max_hard_path_violation =
            hard_path_inequalities.max_violation(&sqp_solution.variables)?;

        Ok(LgrPseudospectralFixedMeshDenseSqpReport {
            sqp_solution,
            residual_report,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with dense SQP and a linear
    /// terminal-state objective.
    ///
    /// Differential, continuity, and fixed-state boundary residuals are treated
    /// as equality rows. Scalar hard path constraints are merged into decision
    /// bounds before calling the solver. A negative objective weight maximizes
    /// the selected terminal component; a positive weight minimizes it. This is
    /// a fixed-mesh equality-plus-linear-objective lane, not a full constrained
    /// large-sparse NLP backend.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when unsupported soft/nonlinear/objective
    /// residual terms are present, terminal objective inputs are malformed,
    /// bound merging fails, residual evaluation fails, or the dense SQP
    /// objective solve fails.
    #[cfg(feature = "sqp")]
    #[allow(clippy::too_many_arguments)]
    pub fn solve_dense_sqp_with_terminal_linear_objective<D, L>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        mut dynamics: D,
        mut lagrange: L,
        terminal_component: usize,
        terminal_objective_weight: f64,
    ) -> Result<LgrPseudospectralFixedMeshTerminalObjectiveDenseSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        if !self.path_constraints.is_empty()
            || !self.nonlinear_hard_path_constraints.is_empty()
            || !self.quadratic_control_costs.is_empty()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR terminal-objective dense SQP supports only equality residuals and scalar hard path bounds",
            });
        }
        if terminal_component >= self.layout.state_dimension
            || !terminal_objective_weight.is_finite()
            || terminal_objective_weight == 0.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR terminal objective component or weight is invalid",
            });
        }
        validate_lgr_decision_vector(&self.layout, initial_decision)?;
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        let terminal_column = lgr_mesh_node_decision_index(
            &self.layout,
            self.layout.segment_count(),
            terminal_component,
        )?;
        let mut linear_objective = vec![0.0; self.layout.variable_count];
        linear_objective[terminal_column] = terminal_objective_weight;

        let sqp_solution = solver.solve_equalities_with_linear_objective(
            initial_decision,
            &decision_bounds.lower_bounds,
            &decision_bounds.upper_bounds,
            &linear_objective,
            |decision: &[f64]| {
                let report = lgr_fixed_mesh_residual_report_with_callbacks(
                    self,
                    decision,
                    &mut dynamics,
                    &mut lagrange,
                )?;
                let equality_row_count = report.differential_residual_count
                    + report.continuity_residual_count
                    + report.boundary_residual_count;
                Ok(report.residuals[..equality_row_count].to_vec())
            },
        )?;
        let residual_report = lgr_fixed_mesh_residual_report_with_callbacks(
            self,
            &sqp_solution.variables,
            &mut dynamics,
            &mut lagrange,
        )?;
        let equality_row_count = residual_report.differential_residual_count
            + residual_report.continuity_residual_count
            + residual_report.boundary_residual_count;
        let equality_residual_norm = norm(&residual_report.residuals[..equality_row_count]);
        let terminal_value = self
            .layout
            .mesh_node_state(&sqp_solution.variables, self.layout.segment_count())?
            [terminal_component];
        let max_decision_bound_violation =
            decision_bounds.max_violation(&sqp_solution.variables)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let max_hard_path_violation =
            hard_path_inequalities.max_violation(&sqp_solution.variables)?;

        Ok(LgrPseudospectralFixedMeshTerminalObjectiveDenseSqpReport {
            sqp_solution,
            terminal_component,
            terminal_objective_weight,
            terminal_value,
            residual_report,
            equality_residual_norm,
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
        })
    }

    /// Solve this fixed-mesh residual model with dense SQP and nonlinear hard
    /// path inequalities.
    ///
    /// Scalar hard path constraints are still merged into decision bounds.
    /// Nonlinear hard path constraints are evaluated as `g(z) <= 0`, linearized
    /// into each SQP QP subproblem, and reported at the accepted solution.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when bound merging, residual evaluation,
    /// nonlinear hard path evaluation, or the constrained dense SQP solve fails.
    #[cfg(feature = "sqp")]
    #[allow(clippy::too_many_arguments)]
    pub fn solve_dense_sqp_with_nonlinear_hard_path_constraints<D, L, P>(
        &self,
        initial_decision: &[f64],
        default_lower_bounds: &[f64],
        default_upper_bounds: &[f64],
        solver: &crate::sqp::DenseSqpSolver,
        mut dynamics: D,
        mut lagrange: L,
        mut path_function: P,
    ) -> Result<LgrPseudospectralFixedMeshDenseConstrainedSqpReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
        P: CollocationPathFunction,
    {
        let decision_bounds = self.decision_bounds_with_hard_path_constraints(
            default_lower_bounds,
            default_upper_bounds,
        )?;
        let sqp_solution = solver.solve_with_inequalities(
            initial_decision,
            &decision_bounds.lower_bounds,
            &decision_bounds.upper_bounds,
            |decision: &[f64]| {
                Ok(lgr_fixed_mesh_residual_report_with_callbacks(
                    self,
                    decision,
                    &mut dynamics,
                    &mut lagrange,
                )?
                .residuals)
            },
            |decision: &[f64]| {
                Ok(self
                    .nonlinear_hard_path_inequality_report(decision, &mut path_function)?
                    .inequality_values)
            },
        )?;
        let residual_report = lgr_fixed_mesh_residual_report_with_callbacks(
            self,
            &sqp_solution.variables,
            &mut dynamics,
            &mut lagrange,
        )?;
        let nonlinear_hard_path_report = self
            .nonlinear_hard_path_inequality_report(&sqp_solution.variables, &mut path_function)?;
        let max_decision_bound_violation =
            decision_bounds.max_violation(&sqp_solution.variables)?;
        let hard_path_inequalities = self.hard_path_inequality_report()?;
        let max_hard_path_violation =
            hard_path_inequalities.max_violation(&sqp_solution.variables)?;
        let max_nonlinear_hard_path_violation = nonlinear_hard_path_report.max_violation;

        Ok(LgrPseudospectralFixedMeshDenseConstrainedSqpReport {
            sqp_solution,
            residual_report,
            decision_bounds,
            nonlinear_hard_path_report,
            max_decision_bound_violation,
            max_hard_path_violation,
            max_nonlinear_hard_path_violation,
        })
    }

    /// Evaluate nonlinear hard path bounds as `g(z) <= 0` rows.
    ///
    /// For each path-function component, upper bounds emit `value - upper`
    /// rows and lower bounds emit `lower - value` rows.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, constraint
    /// descriptors, path-function outputs, or report shape are invalid.
    pub fn nonlinear_hard_path_inequality_report<P>(
        &self,
        decision: &[f64],
        path_function: &mut P,
    ) -> Result<LgrPseudospectralNonlinearPathInequalityReport, TrajoptError>
    where
        P: CollocationPathFunction,
    {
        validate_lgr_fixed_mesh_ocp(self)?;
        validate_lgr_decision_vector(&self.layout, decision)?;
        let mut path_values = Vec::new();
        let inequality_count = self
            .nonlinear_hard_path_constraints
            .iter()
            .map(LgrPseudospectralNonlinearHardPathConstraint::inequality_count)
            .sum();
        let mut inequality_values = Vec::with_capacity(inequality_count);

        for constraint in &self.nonlinear_hard_path_constraints {
            let (time_s, state, control) = lgr_sample_state_control(
                &self.layout,
                decision,
                constraint.segment_index,
                constraint.collocation_index,
            )?;
            let values = path_function.eval(time_s, state, control)?;
            if values.len() != constraint.output_dimension() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR nonlinear path output dimension is inconsistent",
                });
            }
            validate_finite("LGR nonlinear path output values must be finite", &values)?;
            for ((value, lower), upper) in values
                .iter()
                .zip(constraint.lower_bounds.iter())
                .zip(constraint.upper_bounds.iter())
            {
                if let Some(upper) = upper {
                    inequality_values.push(value - upper);
                }
                if let Some(lower) = lower {
                    inequality_values.push(lower - value);
                }
            }
            path_values.extend(values);
        }

        let max_violation = inequality_values
            .iter()
            .map(|value| value.max(0.0))
            .fold(0.0_f64, f64::max);
        let report = LgrPseudospectralNonlinearPathInequalityReport {
            variable_count: self.layout.variable_count,
            path_value_count: path_values.len(),
            inequality_count,
            path_values,
            inequality_values,
            max_violation,
        };
        validate_lgr_nonlinear_path_inequality_report(&report)?;
        Ok(report)
    }

    /// Evaluate the nonlinear hard path inequality Jacobian as CSC sparse
    /// storage.
    ///
    /// The derivative is computed with deterministic central finite
    /// differences and drops coefficients whose absolute value is at or below
    /// `drop_tolerance`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, finite-difference
    /// settings, path function, or inequality-row dimension stability is
    /// invalid.
    pub fn sparse_nonlinear_hard_path_inequality_jacobian_report<P>(
        &self,
        decision: &[f64],
        path_function: &mut P,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralSparsePathInequalityJacobianReport, TrajoptError>
    where
        P: CollocationPathFunction,
    {
        validate_lgr_finite_difference_settings(finite_difference_step, drop_tolerance)?;
        validate_lgr_decision_vector(&self.layout, decision)?;

        let base_report = self.nonlinear_hard_path_inequality_report(decision, path_function)?;
        let row_count = base_report.inequality_count;
        let column_count = self.layout.variable_count;
        let mut column_offsets = Vec::with_capacity(column_count + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        column_offsets.push(0);

        for column in 0..column_count {
            let step = finite_difference_step * decision[column].abs().max(1.0);
            let mut plus = decision.to_vec();
            let mut minus = decision.to_vec();
            plus[column] += step;
            minus[column] -= step;
            validate_lgr_decision_vector(&self.layout, &plus)?;
            validate_lgr_decision_vector(&self.layout, &minus)?;

            let plus_report = self.nonlinear_hard_path_inequality_report(&plus, path_function)?;
            validate_lgr_inequality_dimension(&plus_report.inequality_values, row_count)?;
            let minus_report = self.nonlinear_hard_path_inequality_report(&minus, path_function)?;
            validate_lgr_inequality_dimension(&minus_report.inequality_values, row_count)?;

            let scale = 2.0 * step;
            for row in 0..row_count {
                let coefficient = (plus_report.inequality_values[row]
                    - minus_report.inequality_values[row])
                    / scale;
                if coefficient.abs() > drop_tolerance {
                    row_indices.push(row);
                    values.push(coefficient);
                }
            }
            column_offsets.push(values.len());
        }

        let report = LgrPseudospectralSparsePathInequalityJacobianReport {
            row_count,
            column_count,
            column_offsets,
            row_indices,
            values,
            finite_difference_step,
            drop_tolerance,
        };
        validate_lgr_sparse_path_inequality_jacobian_report(&report)?;
        Ok(report)
    }

    /// Evaluate the fixed-mesh residual Jacobian as CSC sparse storage using
    /// caller-supplied analytic dynamics Jacobians.
    ///
    /// The assembled rows match [`Self::residual_report`]: LGR differential
    /// defects, continuity residuals, boundary residuals, soft path residuals,
    /// and quadratic control objective residuals. The returned sparse report
    /// uses `finite_difference_step = 0.0` to identify analytic derivative
    /// assembly.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, dynamics Jacobian
    /// samples, residual shape, or sparse report are invalid.
    pub fn analytic_sparse_residual_jacobian_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationDynamicsJacobian,
    {
        validate_lgr_analytic_residual_jacobian_drop_tolerance(drop_tolerance)?;
        validate_lgr_fixed_mesh_ocp(self)?;
        validate_lgr_decision_vector(&self.layout, decision)?;

        let row_count = lgr_fixed_mesh_residual_row_count(self);
        let column_count = self.layout.variable_count;
        let mut dense = vec![0.0; row_count * column_count];
        let mut row = 0;

        for segment_index in 0..self.layout.segment_count() {
            let grid = &self.layout.segment_grids[segment_index];
            let support_count = grid.support_count();
            let collocation_count = grid.collocation_count();
            let half_duration_s = 0.5
                * (self.layout.time_nodes_s[segment_index + 1]
                    - self.layout.time_nodes_s[segment_index]);
            let midpoint_time_s = 0.5
                * (self.layout.time_nodes_s[segment_index + 1]
                    + self.layout.time_nodes_s[segment_index]);
            let state_offset = self.layout.segment_state_offsets[segment_index];
            let control_offset = self.layout.segment_control_offsets[segment_index];

            for collocation_index in 0..collocation_count {
                let collocation_time_s =
                    midpoint_time_s + half_duration_s * grid.collocation_nodes[collocation_index];
                let state = self.layout.segment_support_state(
                    decision,
                    segment_index,
                    collocation_index,
                )?;
                let control = self.layout.segment_collocation_control(
                    decision,
                    segment_index,
                    collocation_index,
                )?;
                let sample = dynamics.eval_with_jacobian(collocation_time_s, state, control)?;
                validate_lgr_dynamics_jacobian_sample(
                    &sample,
                    self.layout.state_dimension,
                    self.layout.control_dimension,
                )?;

                for state_row in 0..self.layout.state_dimension {
                    let residual_row =
                        row + collocation_index * self.layout.state_dimension + state_row;
                    for support_index in 0..support_count {
                        let differentiation_coefficient = grid.differentiation_matrix
                            [collocation_index * support_count + support_index];
                        for state_column in 0..self.layout.state_dimension {
                            if state_row == state_column {
                                let column = state_offset
                                    + support_index * self.layout.state_dimension
                                    + state_column;
                                dense[residual_row * column_count + column] +=
                                    differentiation_coefficient;
                            }
                        }
                    }

                    for state_column in 0..self.layout.state_dimension {
                        let column = state_offset
                            + collocation_index * self.layout.state_dimension
                            + state_column;
                        let coefficient = sample.state_jacobian
                            [state_row * self.layout.state_dimension + state_column];
                        dense[residual_row * column_count + column] +=
                            -half_duration_s * coefficient;
                    }

                    for control_column in 0..self.layout.control_dimension {
                        let column = control_offset
                            + collocation_index * self.layout.control_dimension
                            + control_column;
                        let coefficient = sample.control_jacobian
                            [state_row * self.layout.control_dimension + control_column];
                        dense[residual_row * column_count + column] +=
                            -half_duration_s * coefficient;
                    }
                }
            }
            row += collocation_count * self.layout.state_dimension;
        }

        for segment_index in 1..self.layout.segment_count() {
            let previous_terminal_support =
                self.layout.segment_grids[segment_index - 1].support_count() - 1;
            for component in 0..self.layout.state_dimension {
                let previous_column = self.layout.segment_state_offsets[segment_index - 1]
                    + previous_terminal_support * self.layout.state_dimension
                    + component;
                let current_column = self.layout.segment_state_offsets[segment_index] + component;
                dense[row * column_count + previous_column] = -1.0;
                dense[row * column_count + current_column] = 1.0;
                row += 1;
            }
        }

        for constraint in &self.boundary_constraints {
            validate_lgr_boundary_constraint(&self.layout, constraint)?;
            let column = if constraint.mesh_node_index == self.layout.segment_count() {
                let segment_index = self.layout.segment_count() - 1;
                let terminal_support = self.layout.segment_grids[segment_index].support_count() - 1;
                self.layout.segment_state_offsets[segment_index]
                    + terminal_support * self.layout.state_dimension
                    + constraint.component
            } else {
                self.layout.segment_state_offsets[constraint.mesh_node_index] + constraint.component
            };
            dense[row * column_count + column] = 1.0;
            row += 1;
        }

        for constraint in &self.path_constraints {
            validate_lgr_path_constraint(&self.layout, constraint)?;
            let value = lgr_sample_scalar(
                &self.layout,
                decision,
                constraint.segment_index,
                constraint.collocation_index,
                constraint.variable,
            )?;
            let active = constraint.lower.is_some_and(|lower| value < lower)
                || constraint.upper.is_some_and(|upper| value > upper);
            if active {
                let column = lgr_scalar_decision_index(
                    &self.layout,
                    constraint.segment_index,
                    constraint.collocation_index,
                    constraint.variable,
                )?;
                dense[row * column_count + column] = constraint.weight.sqrt();
            }
            row += 1;
        }

        for segment_index in 0..self.layout.segment_count() {
            let duration_s = self.layout.time_nodes_s[segment_index + 1]
                - self.layout.time_nodes_s[segment_index];
            let grid = &self.layout.segment_grids[segment_index];
            for collocation_index in 0..grid.collocation_count() {
                let physical_weight_s =
                    0.5 * duration_s * grid.quadrature_weights[collocation_index];
                for cost in &self.quadratic_control_costs {
                    validate_lgr_quadratic_control_cost(&self.layout, cost)?;
                    let column = lgr_scalar_decision_index(
                        &self.layout,
                        segment_index,
                        collocation_index,
                        LgrPseudospectralScalarVariable::ControlComponent {
                            component: cost.component,
                        },
                    )?;
                    dense[row * column_count + column] = (cost.weight * physical_weight_s).sqrt();
                    row += 1;
                }
            }
        }

        if row != row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR analytic residual Jacobian row count is inconsistent",
            });
        }

        lgr_sparse_residual_report_from_dense(row_count, column_count, &dense, 0.0, drop_tolerance)
    }

    /// Evaluate the fixed-mesh residual Jacobian with forward-mode autodiff.
    ///
    /// The dynamics callback receives [`crate::CollocationAutoDiffScalar`]
    /// inputs at each LGR collocation sample. This method seeds state/control
    /// variables automatically and reuses the analytic sparse-Jacobian
    /// assembly path.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, AD dynamics output, or
    /// sparse report are invalid.
    pub fn autodiff_sparse_residual_jacobian_report<D>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationAutoDiffDynamics,
    {
        let borrowed = BorrowedLgrAutoDiffDynamics { dynamics };
        let mut adapter = CollocationAutoDiffDynamicsAdapter::new(
            borrowed,
            self.layout.state_dimension,
            self.layout.control_dimension,
        )?;
        self.analytic_sparse_residual_jacobian_report(decision, &mut adapter, drop_tolerance)
    }

    /// Evaluate the fixed-mesh residual Jacobian as CSC sparse storage.
    ///
    /// The derivative is computed with deterministic central finite
    /// differences and drops coefficients whose absolute value is at or below
    /// `drop_tolerance`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, finite-difference
    /// settings, residual evaluation, or residual dimension stability is
    /// invalid.
    pub fn sparse_residual_jacobian_report<D, L>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_finite_difference_settings(finite_difference_step, drop_tolerance)?;
        validate_lgr_decision_vector(&self.layout, decision)?;

        let base_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, decision, dynamics, lagrange)?;
        let row_count = base_report.residuals.len();
        let column_count = self.layout.variable_count;
        let mut column_offsets = Vec::with_capacity(column_count + 1);
        let mut row_indices = Vec::new();
        let mut values = Vec::new();
        column_offsets.push(0);

        for column in 0..column_count {
            let step = finite_difference_step * decision[column].abs().max(1.0);
            let mut plus = decision.to_vec();
            let mut minus = decision.to_vec();
            plus[column] += step;
            minus[column] -= step;
            validate_lgr_decision_vector(&self.layout, &plus)?;
            validate_lgr_decision_vector(&self.layout, &minus)?;

            let plus_report =
                lgr_fixed_mesh_residual_report_with_callbacks(self, &plus, dynamics, lagrange)?;
            validate_lgr_residual_dimension(&plus_report.residuals, row_count)?;
            let minus_report =
                lgr_fixed_mesh_residual_report_with_callbacks(self, &minus, dynamics, lagrange)?;
            validate_lgr_residual_dimension(&minus_report.residuals, row_count)?;

            let scale = 2.0 * step;
            for row in 0..row_count {
                let coefficient =
                    (plus_report.residuals[row] - minus_report.residuals[row]) / scale;
                if coefficient.abs() > drop_tolerance {
                    row_indices.push(row);
                    values.push(coefficient);
                }
            }
            column_offsets.push(values.len());
        }

        let report = LgrPseudospectralSparseResidualJacobianReport {
            row_count,
            column_count,
            column_offsets,
            row_indices,
            values,
            finite_difference_step,
            drop_tolerance,
        };
        validate_lgr_sparse_residual_jacobian_report(&report)?;
        Ok(report)
    }

    /// Evaluate the fixed-mesh residual Jacobian as CSC sparse storage using a
    /// deterministic greedy column coloring.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector, finite-difference
    /// settings, structural pattern, residual evaluation, or residual dimension
    /// stability is invalid.
    pub fn colored_sparse_residual_jacobian_report<D, L>(
        &self,
        decision: &[f64],
        dynamics: &mut D,
        lagrange: &mut L,
        finite_difference_step: f64,
        drop_tolerance: f64,
    ) -> Result<LgrPseudospectralColoredSparseResidualJacobianReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        validate_lgr_finite_difference_settings(finite_difference_step, drop_tolerance)?;
        validate_lgr_decision_vector(&self.layout, decision)?;

        let base_report =
            lgr_fixed_mesh_residual_report_with_callbacks(self, decision, dynamics, lagrange)?;
        let row_count = base_report.residuals.len();
        let column_count = self.layout.variable_count;
        let column_rows = self.residual_jacobian_structural_column_rows(row_count)?;
        let structural_nonzero_count = column_rows.iter().map(Vec::len).sum::<usize>();
        let column_colors = lgr_greedy_column_colors(&column_rows, row_count)?;
        let color_count = column_colors
            .iter()
            .copied()
            .max()
            .map_or(0, |color| color + 1);
        let mut columns_by_color = vec![Vec::new(); color_count];
        for (column, color) in column_colors.iter().copied().enumerate() {
            columns_by_color[color].push(column);
        }

        let mut dense = vec![0.0; row_count * column_count];
        for columns in &columns_by_color {
            let mut plus = decision.to_vec();
            let mut minus = decision.to_vec();
            let mut steps = Vec::with_capacity(columns.len());
            for column in columns {
                let step = finite_difference_step * decision[*column].abs().max(1.0);
                plus[*column] += step;
                minus[*column] -= step;
                steps.push(step);
            }
            validate_lgr_decision_vector(&self.layout, &plus)?;
            validate_lgr_decision_vector(&self.layout, &minus)?;

            let plus_report =
                lgr_fixed_mesh_residual_report_with_callbacks(self, &plus, dynamics, lagrange)?;
            validate_lgr_residual_dimension(&plus_report.residuals, row_count)?;
            let minus_report =
                lgr_fixed_mesh_residual_report_with_callbacks(self, &minus, dynamics, lagrange)?;
            validate_lgr_residual_dimension(&minus_report.residuals, row_count)?;

            for (column, step) in columns.iter().zip(steps.iter()) {
                let scale = 2.0 * step;
                for row in &column_rows[*column] {
                    dense[*row * column_count + *column] =
                        (plus_report.residuals[*row] - minus_report.residuals[*row]) / scale;
                }
            }
        }

        let sparse_jacobian = lgr_sparse_residual_report_from_dense(
            row_count,
            column_count,
            &dense,
            finite_difference_step,
            drop_tolerance,
        )?;
        let report = LgrPseudospectralColoredSparseResidualJacobianReport {
            sparse_jacobian,
            column_colors,
            color_count,
            residual_evaluation_count: 1 + 2 * color_count,
            structural_nonzero_count,
        };
        validate_lgr_colored_sparse_residual_jacobian_report(&report)?;
        Ok(report)
    }

    fn residual_jacobian_structural_column_rows(
        &self,
        row_count: usize,
    ) -> Result<Vec<Vec<usize>>, TrajoptError> {
        validate_lgr_fixed_mesh_ocp(self)?;
        let mut column_rows = vec![Vec::new(); self.layout.variable_count];
        let differential_rows = self
            .layout
            .segment_grids
            .iter()
            .map(|grid| grid.collocation_count() * self.layout.state_dimension)
            .sum::<usize>();
        let continuity_rows = (self.layout.segment_count() - 1) * self.layout.state_dimension;
        let objective_rows = self
            .layout
            .segment_grids
            .iter()
            .map(LegendreGaussRadauGrid::collocation_count)
            .sum::<usize>()
            * self.quadratic_control_costs.len();
        let expected_row_count = differential_rows
            + continuity_rows
            + self.boundary_constraints.len()
            + self.path_constraints.len()
            + objective_rows;
        if row_count != expected_row_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR residual structural pattern row count is inconsistent",
            });
        }

        let mut row = 0;
        for segment_index in 0..self.layout.segment_count() {
            let columns = lgr_segment_structural_columns(&self.layout, segment_index);
            let segment_row_count = self.layout.segment_grids[segment_index].collocation_count()
                * self.layout.state_dimension;
            for structural_row in row..row + segment_row_count {
                lgr_push_structural_row(&mut column_rows, structural_row, &columns)?;
            }
            row += segment_row_count;
        }

        for segment_index in 1..self.layout.segment_count() {
            let previous_terminal_support =
                self.layout.segment_grids[segment_index - 1].support_count() - 1;
            for component in 0..self.layout.state_dimension {
                let previous_column = self.layout.segment_state_offsets[segment_index - 1]
                    + previous_terminal_support * self.layout.state_dimension
                    + component;
                let current_column = self.layout.segment_state_offsets[segment_index] + component;
                lgr_push_structural_row(&mut column_rows, row, &[previous_column, current_column])?;
                row += 1;
            }
        }

        for constraint in &self.boundary_constraints {
            validate_lgr_boundary_constraint(&self.layout, constraint)?;
            let column = if constraint.mesh_node_index == self.layout.segment_count() {
                let segment_index = self.layout.segment_count() - 1;
                let terminal_support = self.layout.segment_grids[segment_index].support_count() - 1;
                self.layout.segment_state_offsets[segment_index]
                    + terminal_support * self.layout.state_dimension
                    + constraint.component
            } else {
                self.layout.segment_state_offsets[constraint.mesh_node_index] + constraint.component
            };
            lgr_push_structural_row(&mut column_rows, row, &[column])?;
            row += 1;
        }

        for constraint in &self.path_constraints {
            validate_lgr_path_constraint(&self.layout, constraint)?;
            let column = lgr_scalar_decision_index(
                &self.layout,
                constraint.segment_index,
                constraint.collocation_index,
                constraint.variable,
            )?;
            lgr_push_structural_row(&mut column_rows, row, &[column])?;
            row += 1;
        }

        for segment_index in 0..self.layout.segment_count() {
            let grid = &self.layout.segment_grids[segment_index];
            for collocation_index in 0..grid.collocation_count() {
                for cost in &self.quadratic_control_costs {
                    validate_lgr_quadratic_control_cost(&self.layout, cost)?;
                    let column = lgr_scalar_decision_index(
                        &self.layout,
                        segment_index,
                        collocation_index,
                        LgrPseudospectralScalarVariable::ControlComponent {
                            component: cost.component,
                        },
                    )?;
                    lgr_push_structural_row(&mut column_rows, row, &[column])?;
                    row += 1;
                }
            }
        }

        Ok(column_rows)
    }

    /// Evaluate differential, continuity, and boundary residuals.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when decision decoding, dynamics evaluation, or
    /// residual assembly fails.
    pub fn residual_report<D, L>(
        &self,
        decision: &[f64],
        dynamics: D,
        lagrange: L,
    ) -> Result<LgrPseudospectralFixedMeshResidualReport, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        let mut dynamics = dynamics;
        let mut lagrange = lagrange;
        lgr_fixed_mesh_residual_report_with_callbacks(self, decision, &mut dynamics, &mut lagrange)
    }

    /// Evaluate only the residual vector for a flat decision vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when residual assembly fails.
    pub fn residuals<D, L>(
        &self,
        decision: &[f64],
        dynamics: D,
        lagrange: L,
    ) -> Result<Vec<f64>, TrajoptError>
    where
        D: CollocationDynamics,
        L: CollocationLagrange,
    {
        Ok(self
            .residual_report(decision, dynamics, lagrange)?
            .residuals)
    }
}

/// Evaluate one LGR pseudospectral segment.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the segment dimensions or values are malformed,
/// the dynamics output dimensions change, or the Lagrange value is non-finite.
pub fn evaluate_lgr_pseudospectral_segment<D, L>(
    segment: &LgrPseudospectralSegment,
    mut dynamics: D,
    mut lagrange: L,
) -> Result<LgrPseudospectralSegmentReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    validate_lgr_segment(segment)?;
    evaluate_lgr_pseudospectral_segment_with_callbacks(segment, &mut dynamics, &mut lagrange)
}

/// Evaluate a multi-segment LGR pseudospectral transcription.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the transcription is empty, any segment is
/// malformed, adjacent segment times are not contiguous, segment dimensions do
/// not match, or any callback evaluation returns malformed values.
pub fn evaluate_lgr_pseudospectral_transcription<D, L>(
    transcription: &LgrPseudospectralTranscription,
    mut dynamics: D,
    mut lagrange: L,
) -> Result<LgrPseudospectralTranscriptionReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    evaluate_lgr_pseudospectral_transcription_with_callbacks(
        transcription,
        &mut dynamics,
        &mut lagrange,
    )
}

fn evaluate_lgr_pseudospectral_transcription_with_callbacks<D, L>(
    transcription: &LgrPseudospectralTranscription,
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<LgrPseudospectralTranscriptionReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    validate_lgr_transcription(transcription)?;
    let state_dimension = transcription.segments[0].state_dimension;
    let mut segment_reports = Vec::with_capacity(transcription.segments.len());
    let mut residuals = Vec::new();
    let mut continuity_residuals =
        Vec::with_capacity((transcription.segments.len() - 1) * state_dimension);
    let mut quadrature_cost = 0.0;

    for (segment_index, segment) in transcription.segments.iter().enumerate() {
        let report =
            evaluate_lgr_pseudospectral_segment_with_callbacks(segment, dynamics, lagrange)?;
        residuals.extend_from_slice(&report.residuals);
        quadrature_cost += report.quadrature_cost;

        if segment_index > 0 {
            let previous = &transcription.segments[segment_index - 1];
            let previous_terminal_offset =
                (previous.grid.support_count() - 1) * previous.state_dimension;
            for state_index in 0..state_dimension {
                continuity_residuals.push(
                    segment.support_states[state_index]
                        - previous.support_states[previous_terminal_offset + state_index],
                );
            }
        }

        segment_reports.push(report);
    }

    let differential_residual_count = residuals.len();
    let continuity_residual_count = continuity_residuals.len();
    residuals.extend_from_slice(&continuity_residuals);

    Ok(LgrPseudospectralTranscriptionReport {
        segment_reports,
        residual_norm: norm(&residuals),
        max_abs_residual: max_abs(&residuals),
        residuals,
        differential_residual_count,
        continuity_residual_count,
        quadrature_cost,
    })
}

fn lgr_fixed_mesh_residual_report_with_callbacks<D, L>(
    ocp: &LgrPseudospectralFixedMeshOcp,
    decision: &[f64],
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<LgrPseudospectralFixedMeshResidualReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    validate_lgr_fixed_mesh_ocp(ocp)?;
    let transcription = ocp.layout.transcription_from_decision(decision)?;
    let transcription_report = evaluate_lgr_pseudospectral_transcription_with_callbacks(
        &transcription,
        dynamics,
        lagrange,
    )?;
    let differential_residual_count = transcription_report.differential_residual_count;
    let continuity_residual_count = transcription_report.continuity_residual_count;
    let quadrature_cost = transcription_report.quadrature_cost;
    let mut residuals = transcription_report.residuals;

    for constraint in &ocp.boundary_constraints {
        let state = ocp
            .layout
            .mesh_node_state(decision, constraint.mesh_node_index)?;
        residuals.push(state[constraint.component] - constraint.target);
    }
    let path_residual_count = append_lgr_path_residuals(ocp, decision, &mut residuals)?;
    let quadratic_control_cost =
        append_lgr_quadratic_control_cost_residuals(ocp, decision, &mut residuals)?;

    Ok(LgrPseudospectralFixedMeshResidualReport {
        differential_residual_count,
        continuity_residual_count,
        boundary_residual_count: ocp.boundary_constraints.len(),
        path_residual_count,
        objective_residual_count: ocp.quadratic_control_costs.len()
            * ocp
                .layout
                .segment_grids
                .iter()
                .map(LegendreGaussRadauGrid::collocation_count)
                .sum::<usize>(),
        residual_norm: norm(&residuals),
        max_abs_residual: max_abs(&residuals),
        residuals,
        quadrature_cost,
        quadratic_control_cost,
    })
}

fn assemble_lgr_kkt_residual_multiplier_report(
    ocp: &LgrPseudospectralFixedMeshOcp,
    residual_report: &LgrPseudospectralFixedMeshResidualReport,
    residual_multipliers: &[f64],
) -> Result<LgrPseudospectralKktResidualMultiplierReport, TrajoptError> {
    validate_lgr_fixed_mesh_residual_report_for_ocp(ocp, residual_report)?;
    if residual_multipliers.len() != residual_report.residuals.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR KKT residual multiplier dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR KKT residual multipliers must be finite",
        residual_multipliers,
    )?;

    let mut offset = 0usize;
    let mut collocation_samples = Vec::new();
    let mut max_abs_dynamics_multiplier = 0.0_f64;
    for segment_index in 0..ocp.layout.segment_count() {
        let grid = &ocp.layout.segment_grids[segment_index];
        for collocation_index in 0..grid.collocation_count() {
            let end = offset + ocp.layout.state_dimension;
            let dynamics_defect_multipliers = residual_multipliers[offset..end].to_vec();
            max_abs_dynamics_multiplier =
                max_abs_dynamics_multiplier.max(max_abs(&dynamics_defect_multipliers));
            collocation_samples.push(LgrPseudospectralKktCollocationMultiplierSample {
                segment_index,
                collocation_index,
                time_s: lgr_physical_collocation_time(
                    &ocp.layout,
                    segment_index,
                    collocation_index,
                )?,
                dynamics_defect_multipliers,
            });
            offset = end;
        }
    }

    let continuity_end = offset + residual_report.continuity_residual_count;
    let continuity_multipliers = residual_multipliers[offset..continuity_end].to_vec();
    let max_abs_continuity_multiplier = max_abs(&continuity_multipliers);
    offset = continuity_end;

    let boundary_end = offset + residual_report.boundary_residual_count;
    let boundary_multipliers = residual_multipliers[offset..boundary_end].to_vec();
    let max_abs_boundary_multiplier = max_abs(&boundary_multipliers);
    offset = boundary_end;

    let path_end = offset + residual_report.path_residual_count;
    let path_multipliers = residual_multipliers[offset..path_end].to_vec();
    offset = path_end;

    let objective_end = offset + residual_report.objective_residual_count;
    let objective_multipliers = residual_multipliers[offset..objective_end].to_vec();
    if objective_end != residual_multipliers.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR KKT residual multiplier family split is inconsistent",
        });
    }

    Ok(LgrPseudospectralKktResidualMultiplierReport {
        segment_count: ocp.layout.segment_count(),
        state_dimension: ocp.layout.state_dimension,
        residual_multiplier_count: residual_multipliers.len(),
        differential_multiplier_count: residual_report.differential_residual_count,
        continuity_multiplier_count: residual_report.continuity_residual_count,
        boundary_multiplier_count: residual_report.boundary_residual_count,
        path_multiplier_count: residual_report.path_residual_count,
        objective_multiplier_count: residual_report.objective_residual_count,
        collocation_samples,
        continuity_multipliers,
        boundary_multipliers,
        path_multipliers,
        objective_multipliers,
        max_abs_dynamics_multiplier,
        max_abs_continuity_multiplier,
        max_abs_boundary_multiplier,
    })
}

#[cfg(feature = "sqp")]
fn lgr_dense_qp_residual_multipliers_from_equality_duals(
    report: &LgrPseudospectralFixedMeshDenseQuadraticQpReport,
) -> Result<Vec<f64>, TrajoptError> {
    let expected_equality_count = report.residual_report.differential_residual_count
        + report.residual_report.continuity_residual_count
        + report.residual_report.boundary_residual_count;
    if report.equality_row_count != expected_equality_count
        || report.qp_solution.equality_dual.len() != report.equality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR dense-QP equality dual dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR dense-QP equality duals must be finite",
        &report.qp_solution.equality_dual,
    )?;
    let mut residual_multipliers = vec![0.0; report.residual_report.residuals.len()];
    residual_multipliers[..report.equality_row_count]
        .copy_from_slice(&report.qp_solution.equality_dual);
    Ok(residual_multipliers)
}

fn lgr_quadrature_weight_normalized_costate_convention_report(
    layout: &LgrPseudospectralDecisionLayout,
    costate_report: &LgrPseudospectralKktCostateEstimateReport,
    convention: crate::pmp::PmpDirectCostateConvention,
) -> Result<crate::pmp::PmpDirectCostateConventionReport, TrajoptError> {
    if costate_report.segment_count != layout.segment_count()
        || costate_report.state_dimension != layout.state_dimension
        || costate_report.collocation_samples.is_empty()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR quadrature-normalized PMP costate dimensions are inconsistent",
        });
    }
    let mut times_s = Vec::with_capacity(costate_report.collocation_samples.len());
    let mut normalized_costates =
        Vec::with_capacity(costate_report.collocation_samples.len() * layout.state_dimension);
    for sample in &costate_report.collocation_samples {
        let grid =
            layout
                .segment_grids
                .get(sample.segment_index)
                .ok_or(TrajoptError::InvalidPayload {
                    reason: "LGR quadrature-normalized PMP costate segment is out of range",
                })?;
        let weight = grid
            .quadrature_weights
            .get(sample.collocation_index)
            .ok_or(TrajoptError::InvalidPayload {
                reason: "LGR quadrature-normalized PMP costate node is out of range",
            })?;
        if !weight.is_finite() || *weight <= 0.0 || sample.costate.len() != layout.state_dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR quadrature-normalized PMP costate sample is inconsistent",
            });
        }
        times_s.push(sample.time_s);
        normalized_costates.extend(sample.costate.iter().map(|value| value / weight));
    }
    crate::pmp::apply_pmp_direct_costate_convention(
        layout.state_dimension,
        &times_s,
        &normalized_costates,
        convention,
    )
}

fn assemble_lgr_kkt_collocation_costate_estimate_report(
    multiplier_report: &LgrPseudospectralKktResidualMultiplierReport,
) -> Result<LgrPseudospectralKktCostateEstimateReport, TrajoptError> {
    if multiplier_report.segment_count == 0
        || multiplier_report.state_dimension == 0
        || multiplier_report.collocation_samples.is_empty()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR KKT costate estimate multiplier dimensions are inconsistent",
        });
    }

    let mut collocation_samples = Vec::with_capacity(multiplier_report.collocation_samples.len());
    let mut max_abs_costate = 0.0_f64;
    for sample in &multiplier_report.collocation_samples {
        if sample.segment_index >= multiplier_report.segment_count
            || sample.dynamics_defect_multipliers.len() != multiplier_report.state_dimension
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR KKT costate estimate collocation sample is inconsistent",
            });
        }
        validate_finite(
            "LGR KKT costate estimate multipliers must be finite",
            &sample.dynamics_defect_multipliers,
        )?;
        let costate = sample.dynamics_defect_multipliers.clone();
        max_abs_costate = max_abs_costate.max(max_abs(&costate));
        collocation_samples.push(LgrPseudospectralKktCollocationCostateEstimate {
            segment_index: sample.segment_index,
            collocation_index: sample.collocation_index,
            time_s: sample.time_s,
            costate,
        });
    }

    Ok(LgrPseudospectralKktCostateEstimateReport {
        segment_count: multiplier_report.segment_count,
        state_dimension: multiplier_report.state_dimension,
        source_residual_multiplier_count: multiplier_report.residual_multiplier_count,
        collocation_samples,
        max_abs_costate,
    })
}

fn assemble_lgr_kkt_hamiltonian_report<D, L>(
    ocp: &LgrPseudospectralFixedMeshOcp,
    decision: &[f64],
    costate_report: &LgrPseudospectralKktCostateEstimateReport,
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<LgrPseudospectralKktHamiltonianReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    validate_lgr_fixed_mesh_ocp(ocp)?;
    validate_lgr_decision_vector(&ocp.layout, decision)?;
    if costate_report.segment_count != ocp.layout.segment_count()
        || costate_report.state_dimension != ocp.layout.state_dimension
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR KKT Hamiltonian costate report dimensions are inconsistent",
        });
    }

    let mut samples = Vec::with_capacity(costate_report.collocation_samples.len());
    let mut max_abs_hamiltonian = 0.0_f64;
    let mut max_abs_hamiltonian_drift = 0.0_f64;
    let mut reference_hamiltonian = None;
    for costate_sample in &costate_report.collocation_samples {
        if costate_sample.segment_index >= ocp.layout.segment_count()
            || costate_sample.collocation_index
                >= ocp.layout.segment_grids[costate_sample.segment_index].collocation_count()
            || costate_sample.costate.len() != ocp.layout.state_dimension
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR KKT Hamiltonian costate sample is inconsistent",
            });
        }
        validate_finite(
            "LGR KKT Hamiltonian costate values must be finite",
            &costate_sample.costate,
        )?;
        let expected_time_s = lgr_physical_collocation_time(
            &ocp.layout,
            costate_sample.segment_index,
            costate_sample.collocation_index,
        )?;
        if costate_sample.time_s.to_bits() != expected_time_s.to_bits() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR KKT Hamiltonian costate sample time is inconsistent",
            });
        }

        let state = ocp.layout.segment_support_state(
            decision,
            costate_sample.segment_index,
            costate_sample.collocation_index,
        )?;
        let control = ocp.layout.segment_collocation_control(
            decision,
            costate_sample.segment_index,
            costate_sample.collocation_index,
        )?;
        let dynamics_value = dynamics.eval(costate_sample.time_s, state, control)?;
        validate_lgr_dynamics_output(&dynamics_value, ocp.layout.state_dimension)?;
        let lagrange_value = lagrange.eval(costate_sample.time_s, state, control)?;
        if !lagrange_value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR KKT Hamiltonian Lagrange value must be finite",
            });
        }
        let costate_dot_dynamics = lgr_dot(&costate_sample.costate, &dynamics_value);
        let hamiltonian = lagrange_value + costate_dot_dynamics;
        if !costate_dot_dynamics.is_finite() || !hamiltonian.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR KKT Hamiltonian value must be finite",
            });
        }
        max_abs_hamiltonian = max_abs_hamiltonian.max(hamiltonian.abs());
        let first_hamiltonian = *reference_hamiltonian.get_or_insert(hamiltonian);
        max_abs_hamiltonian_drift =
            max_abs_hamiltonian_drift.max((hamiltonian - first_hamiltonian).abs());

        samples.push(LgrPseudospectralKktHamiltonianSample {
            segment_index: costate_sample.segment_index,
            collocation_index: costate_sample.collocation_index,
            time_s: costate_sample.time_s,
            costate: costate_sample.costate.clone(),
            dynamics: dynamics_value,
            lagrange_value,
            costate_dot_dynamics,
            hamiltonian,
        });
    }

    Ok(LgrPseudospectralKktHamiltonianReport {
        segment_count: ocp.layout.segment_count(),
        state_dimension: ocp.layout.state_dimension,
        sample_count: samples.len(),
        samples,
        max_abs_hamiltonian,
        max_abs_hamiltonian_drift,
    })
}

fn validate_lgr_fixed_mesh_residual_report_for_ocp(
    ocp: &LgrPseudospectralFixedMeshOcp,
    report: &LgrPseudospectralFixedMeshResidualReport,
) -> Result<(), TrajoptError> {
    validate_lgr_fixed_mesh_ocp(ocp)?;
    let expected_differential_count = ocp
        .layout
        .segment_grids
        .iter()
        .map(|grid| grid.collocation_count() * ocp.layout.state_dimension)
        .sum::<usize>();
    let expected_continuity_count = (ocp.layout.segment_count() - 1) * ocp.layout.state_dimension;
    let expected_objective_count = ocp
        .layout
        .segment_grids
        .iter()
        .map(LegendreGaussRadauGrid::collocation_count)
        .sum::<usize>()
        * ocp.quadratic_control_costs.len();
    let expected_total = report.differential_residual_count
        + report.continuity_residual_count
        + report.boundary_residual_count
        + report.path_residual_count
        + report.objective_residual_count;
    if report.differential_residual_count != expected_differential_count
        || report.continuity_residual_count != expected_continuity_count
        || report.boundary_residual_count != ocp.boundary_constraints.len()
        || report.path_residual_count != ocp.path_constraints.len()
        || report.objective_residual_count != expected_objective_count
        || report.residuals.len() != expected_total
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR fixed-mesh residual report does not match the OCP layout",
        });
    }
    validate_finite(
        "LGR fixed-mesh residual values must be finite",
        &report.residuals,
    )
}

fn validate_lgr_sparse_kkt_step_report_for_ocp(
    ocp: &LgrPseudospectralFixedMeshOcp,
    step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
) -> Result<(), TrajoptError> {
    validate_lgr_fixed_mesh_residual_report_for_ocp(ocp, &step_report.residual_report)?;
    validate_lgr_sparse_residual_jacobian_report(&step_report.sparse_jacobian)?;
    validate_lgr_sparse_augmented_kkt_report(&step_report.augmented_kkt)?;
    if step_report.sparse_jacobian.column_count != ocp.layout.variable_count
        || step_report.augmented_kkt.variable_count != ocp.layout.variable_count
        || step_report.augmented_kkt.residual_row_count != step_report.sparse_jacobian.row_count
        || step_report.sparse_jacobian.row_count != step_report.residual_report.residuals.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse KKT step report does not match the OCP layout",
        });
    }
    if step_report.rhs.len() != step_report.augmented_kkt.row_count
        || step_report.solve_report.solution.len() != step_report.augmented_kkt.row_count
        || step_report.step.len() != step_report.sparse_jacobian.column_count
        || step_report.residual_multipliers.len() != step_report.residual_report.residuals.len()
        || step_report.linearized_residual.len() != step_report.residual_report.residuals.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse KKT step report dimensions are inconsistent",
        });
    }
    validate_finite("LGR sparse KKT RHS values must be finite", &step_report.rhs)?;
    validate_finite(
        "LGR sparse KKT step values must be finite",
        &step_report.step,
    )?;
    validate_finite(
        "LGR sparse KKT residual multipliers must be finite",
        &step_report.residual_multipliers,
    )?;
    validate_finite(
        "LGR sparse KKT linearized residual values must be finite",
        &step_report.linearized_residual,
    )
}

fn validate_lgr_sparse_constrained_kkt_step_report_for_ocp(
    ocp: &LgrPseudospectralFixedMeshOcp,
    step_report: &LgrPseudospectralFixedMeshSparseConstrainedKktStepReport,
) -> Result<(), TrajoptError> {
    validate_lgr_fixed_mesh_residual_report_for_ocp(ocp, &step_report.residual_report)?;
    validate_lgr_sparse_residual_jacobian_report(&step_report.sparse_jacobian)?;
    validate_lgr_sparse_constrained_kkt_report(&step_report.constrained_kkt)?;
    validate_lgr_path_inequality_report(&step_report.hard_path_inequality_report)?;
    if step_report.sparse_jacobian.column_count != ocp.layout.variable_count
        || step_report.constrained_kkt.variable_count != ocp.layout.variable_count
        || step_report.hard_path_inequality_report.variable_count != ocp.layout.variable_count
        || step_report.constrained_kkt.residual_row_count != step_report.sparse_jacobian.row_count
        || step_report.sparse_jacobian.row_count != step_report.residual_report.residuals.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR constrained sparse KKT step report does not match the OCP layout",
        });
    }
    if step_report
        .constrained_kkt
        .active_inequality_rows
        .iter()
        .any(|row| *row >= step_report.hard_path_inequality_report.inequality_count)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR constrained sparse KKT active row index is invalid",
        });
    }
    if step_report.rhs.len() != step_report.constrained_kkt.row_count
        || step_report.solve_report.solution.len() != step_report.constrained_kkt.row_count
        || step_report.step.len() != step_report.sparse_jacobian.column_count
        || step_report.residual_multipliers.len() != step_report.residual_report.residuals.len()
        || step_report.active_inequality_multipliers.len()
            != step_report.constrained_kkt.active_inequality_row_count
        || step_report.linearized_residual.len() != step_report.residual_report.residuals.len()
        || step_report.linearized_active_inequality_values.len()
            != step_report.constrained_kkt.active_inequality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR constrained sparse KKT step report dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR constrained sparse KKT RHS values must be finite",
        &step_report.rhs,
    )?;
    validate_finite(
        "LGR constrained sparse KKT step values must be finite",
        &step_report.step,
    )?;
    validate_finite(
        "LGR constrained sparse KKT residual multipliers must be finite",
        &step_report.residual_multipliers,
    )?;
    validate_finite(
        "LGR constrained sparse KKT active multipliers must be finite",
        &step_report.active_inequality_multipliers,
    )?;
    validate_finite(
        "LGR constrained sparse KKT linearized residual values must be finite",
        &step_report.linearized_residual,
    )?;
    validate_finite(
        "LGR constrained sparse KKT linearized active values must be finite",
        &step_report.linearized_active_inequality_values,
    )
}

fn assemble_lgr_sparse_gauss_newton_step_report(
    residual_report: LgrPseudospectralFixedMeshResidualReport,
    sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    hessian_regularization: f64,
    hessian_drop_tolerance: f64,
    cg_max_iterations: usize,
    cg_residual_tolerance: f64,
) -> Result<LgrPseudospectralFixedMeshSparseGaussNewtonStepReport, TrajoptError> {
    validate_lgr_sparse_residual_jacobian_report(&sparse_jacobian)?;
    validate_lgr_residual_dimension(&residual_report.residuals, sparse_jacobian.row_count)?;
    let normal_hessian = sparse_jacobian
        .sparse_gauss_newton_hessian_report(hessian_regularization, hessian_drop_tolerance)?;
    let rhs = sparse_jacobian.sparse_gauss_newton_step_rhs(&residual_report.residuals)?;
    let initial_step = vec![0.0; sparse_jacobian.column_count];
    let solve_report = normal_hessian.solve_conjugate_gradient(
        &rhs,
        &initial_step,
        cg_max_iterations,
        cg_residual_tolerance,
    )?;
    let jacobian_step = sparse_jacobian.multiply_vector(&solve_report.solution)?;
    let linearized_residual = residual_report
        .residuals
        .iter()
        .zip(jacobian_step.iter())
        .map(|(residual, jacobian_step)| residual + jacobian_step)
        .collect::<Vec<_>>();
    let linearized_residual_norm = norm(&linearized_residual);
    let step_norm = norm(&solve_report.solution);

    Ok(LgrPseudospectralFixedMeshSparseGaussNewtonStepReport {
        residual_report,
        sparse_jacobian,
        normal_hessian,
        rhs,
        solve_report,
        linearized_residual,
        linearized_residual_norm,
        step_norm,
    })
}

fn assemble_lgr_sparse_augmented_kkt_step_report(
    residual_report: LgrPseudospectralFixedMeshResidualReport,
    sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    kkt_regularization: f64,
    kkt_drop_tolerance: f64,
) -> Result<LgrPseudospectralFixedMeshSparseKktStepReport, TrajoptError> {
    validate_lgr_sparse_residual_jacobian_report(&sparse_jacobian)?;
    validate_lgr_residual_dimension(&residual_report.residuals, sparse_jacobian.row_count)?;
    let augmented_kkt =
        sparse_jacobian.sparse_augmented_kkt_report(kkt_regularization, kkt_drop_tolerance)?;
    let mut rhs = vec![0.0; augmented_kkt.row_count];
    for (row, residual) in residual_report.residuals.iter().enumerate() {
        rhs[sparse_jacobian.column_count + row] = -*residual;
    }
    let solve_report = augmented_kkt.solve_ldlt(&rhs)?;
    let step = solve_report.solution[..sparse_jacobian.column_count].to_vec();
    let residual_multipliers = solve_report.solution[sparse_jacobian.column_count..].to_vec();
    let jacobian_step = sparse_jacobian.multiply_vector(&step)?;
    let linearized_residual = residual_report
        .residuals
        .iter()
        .zip(jacobian_step.iter())
        .map(|(residual, jacobian_step)| residual + jacobian_step)
        .collect::<Vec<_>>();
    let linearized_residual_norm = norm(&linearized_residual);
    let step_norm = norm(&step);

    Ok(LgrPseudospectralFixedMeshSparseKktStepReport {
        residual_report,
        sparse_jacobian,
        augmented_kkt,
        rhs,
        solve_report,
        step,
        residual_multipliers,
        linearized_residual,
        linearized_residual_norm,
        step_norm,
    })
}

fn assemble_lgr_sparse_constrained_kkt_step_report(
    residual_report: LgrPseudospectralFixedMeshResidualReport,
    sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport,
    hard_path_inequality_report: LgrPseudospectralPathInequalityReport,
    decision: &[f64],
    kkt_regularization: f64,
    kkt_drop_tolerance: f64,
    active_tolerance: f64,
) -> Result<LgrPseudospectralFixedMeshSparseConstrainedKktStepReport, TrajoptError> {
    validate_lgr_sparse_residual_jacobian_report(&sparse_jacobian)?;
    validate_lgr_residual_dimension(&residual_report.residuals, sparse_jacobian.row_count)?;
    validate_lgr_path_inequality_report(&hard_path_inequality_report)?;
    validate_lgr_dense_decision_vector(decision, hard_path_inequality_report.variable_count)?;
    validate_lgr_sparse_augmented_kkt_settings(kkt_regularization, kkt_drop_tolerance)?;
    if !active_tolerance.is_finite() || active_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active tolerance must be finite and non-negative",
        });
    }
    if hard_path_inequality_report.variable_count != sparse_jacobian.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT inequality dimensions are inconsistent",
        });
    }

    let inequality_values =
        lgr_hard_path_inequality_values(&hard_path_inequality_report, decision)?;
    let mut active_inequality_rows = Vec::new();
    let mut active_inequality_values = Vec::new();
    for (row, value) in inequality_values.iter().enumerate() {
        if *value >= -active_tolerance {
            active_inequality_rows.push(row);
            active_inequality_values.push(*value);
        }
    }
    if active_inequality_rows.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT requires at least one active hard path row",
        });
    }

    let constrained_kkt = assemble_lgr_sparse_constrained_kkt_report(
        &sparse_jacobian,
        &hard_path_inequality_report,
        active_inequality_rows,
        active_inequality_values,
        kkt_regularization,
        kkt_drop_tolerance,
        active_tolerance,
    )?;
    let mut rhs = vec![0.0; constrained_kkt.row_count];
    for (row, residual) in residual_report.residuals.iter().enumerate() {
        rhs[sparse_jacobian.column_count + row] = -*residual;
    }
    let active_offset = sparse_jacobian.column_count + sparse_jacobian.row_count;
    for (index, value) in constrained_kkt.active_inequality_values.iter().enumerate() {
        rhs[active_offset + index] = -*value;
    }

    let solve_report = constrained_kkt.solve_ldlt(&rhs)?;
    let variable_count = sparse_jacobian.column_count;
    let residual_end = variable_count + sparse_jacobian.row_count;
    let step = solve_report.solution[..variable_count].to_vec();
    let residual_multipliers = solve_report.solution[variable_count..residual_end].to_vec();
    let active_inequality_multipliers = solve_report.solution[residual_end..].to_vec();
    let jacobian_step = sparse_jacobian.multiply_vector(&step)?;
    let linearized_residual = residual_report
        .residuals
        .iter()
        .zip(jacobian_step.iter())
        .map(|(residual, jacobian_step)| residual + jacobian_step)
        .collect::<Vec<_>>();
    let linearized_residual_norm = norm(&linearized_residual);
    let linearized_active_inequality_values = lgr_linearized_active_inequality_values(
        &hard_path_inequality_report,
        &constrained_kkt.active_inequality_rows,
        &constrained_kkt.active_inequality_values,
        &step,
    )?;
    let max_linearized_active_inequality_violation = linearized_active_inequality_values
        .iter()
        .map(|value| value.max(0.0))
        .fold(0.0_f64, f64::max);
    let step_norm = norm(&step);

    Ok(LgrPseudospectralFixedMeshSparseConstrainedKktStepReport {
        residual_report,
        sparse_jacobian,
        hard_path_inequality_report,
        constrained_kkt,
        rhs,
        solve_report,
        step,
        residual_multipliers,
        active_inequality_multipliers,
        linearized_residual,
        linearized_residual_norm,
        linearized_active_inequality_values,
        max_linearized_active_inequality_violation,
        step_norm,
    })
}

fn assemble_lgr_sparse_constrained_kkt_report(
    sparse_jacobian: &LgrPseudospectralSparseResidualJacobianReport,
    hard_path_inequality_report: &LgrPseudospectralPathInequalityReport,
    active_inequality_rows: Vec<usize>,
    active_inequality_values: Vec<f64>,
    regularization: f64,
    drop_tolerance: f64,
    active_tolerance: f64,
) -> Result<LgrPseudospectralSparseConstrainedKktReport, TrajoptError> {
    validate_lgr_sparse_residual_jacobian_report(sparse_jacobian)?;
    validate_lgr_path_inequality_report(hard_path_inequality_report)?;
    validate_lgr_sparse_augmented_kkt_settings(regularization, drop_tolerance)?;
    if !active_tolerance.is_finite() || active_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active tolerance must be finite and non-negative",
        });
    }
    if active_inequality_rows.is_empty()
        || active_inequality_rows.len() != active_inequality_values.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active rows are inconsistent",
        });
    }
    if active_inequality_rows
        .windows(2)
        .any(|window| window[0] >= window[1])
        || active_inequality_rows
            .iter()
            .any(|row| *row >= hard_path_inequality_report.inequality_count)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active row index is invalid",
        });
    }
    validate_finite(
        "LGR sparse constrained KKT active values must be finite",
        &active_inequality_values,
    )?;
    if hard_path_inequality_report.variable_count != sparse_jacobian.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT inequality dimensions are inconsistent",
        });
    }

    let variable_count = sparse_jacobian.column_count;
    let residual_row_count = sparse_jacobian.row_count;
    let active_inequality_row_count = active_inequality_rows.len();
    let dimension = variable_count + residual_row_count + active_inequality_row_count;
    let mut coefficients = BTreeMap::<(usize, usize), f64>::new();

    for variable in 0..variable_count {
        coefficients.insert((variable, variable), regularization);
    }
    for residual in 0..residual_row_count {
        let index = variable_count + residual;
        coefficients.insert((index, index), -1.0);
    }
    for column in 0..sparse_jacobian.column_count {
        for index in
            sparse_jacobian.column_offsets[column]..sparse_jacobian.column_offsets[column + 1]
        {
            let residual_row = sparse_jacobian.row_indices[index];
            let value = sparse_jacobian.values[index];
            let kkt_residual_index = variable_count + residual_row;
            *coefficients
                .entry((column, kkt_residual_index))
                .or_insert(0.0) += value;
            *coefficients
                .entry((kkt_residual_index, column))
                .or_insert(0.0) += value;
        }
    }

    let mut source_inequality_nonzero_count = 0usize;
    for (active_index, source_row) in active_inequality_rows.iter().enumerate() {
        let active_kkt_index = variable_count + residual_row_count + active_index;
        let row_offset = *source_row * variable_count;
        for variable in 0..variable_count {
            let value = hard_path_inequality_report.inequality_matrix[row_offset + variable];
            if value.abs() > drop_tolerance {
                source_inequality_nonzero_count += 1;
                *coefficients
                    .entry((variable, active_kkt_index))
                    .or_insert(0.0) += value;
                *coefficients
                    .entry((active_kkt_index, variable))
                    .or_insert(0.0) += value;
            }
        }
    }
    if source_inequality_nonzero_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active rows have no coefficients",
        });
    }

    let mut column_offsets = Vec::with_capacity(dimension + 1);
    let mut row_indices = Vec::new();
    let mut values = Vec::new();
    let mut current_column = 0;
    column_offsets.push(0);
    for ((column, row), value) in coefficients {
        while current_column < column {
            column_offsets.push(values.len());
            current_column += 1;
        }
        if value.abs() > drop_tolerance {
            row_indices.push(row);
            values.push(value);
        }
    }
    while current_column < dimension {
        column_offsets.push(values.len());
        current_column += 1;
    }

    let report = LgrPseudospectralSparseConstrainedKktReport {
        row_count: dimension,
        column_count: dimension,
        variable_count,
        residual_row_count,
        active_inequality_row_count,
        source_jacobian_nonzero_count: sparse_jacobian.values.len(),
        source_inequality_nonzero_count,
        active_inequality_rows,
        active_inequality_values,
        column_offsets,
        row_indices,
        values,
        regularization,
        drop_tolerance,
        active_tolerance,
    };
    validate_lgr_sparse_constrained_kkt_report(&report)?;
    Ok(report)
}

fn lgr_hard_path_inequality_values(
    report: &LgrPseudospectralPathInequalityReport,
    decision: &[f64],
) -> Result<Vec<f64>, TrajoptError> {
    let lhs = report.lhs(decision)?;
    Ok(lhs
        .iter()
        .zip(report.inequality_rhs.iter())
        .map(|(lhs, rhs)| lhs - rhs)
        .collect())
}

fn lgr_sparse_constrained_kkt_step_report_if_active(
    decision: &[f64],
    settings: SparseKktSqpSettings,
    hard_path_inequality_report: &LgrPseudospectralPathInequalityReport,
    final_kkt_step_report: &LgrPseudospectralFixedMeshSparseKktStepReport,
) -> Result<Option<LgrPseudospectralFixedMeshSparseConstrainedKktStepReport>, TrajoptError> {
    if hard_path_inequality_report.inequality_count == 0 {
        return Ok(None);
    }
    let inequality_values = lgr_hard_path_inequality_values(hard_path_inequality_report, decision)?;
    if !inequality_values
        .iter()
        .any(|value| *value >= -settings.residual_tolerance)
    {
        return Ok(None);
    }

    assemble_lgr_sparse_constrained_kkt_step_report(
        final_kkt_step_report.residual_report.clone(),
        final_kkt_step_report.sparse_jacobian.clone(),
        hard_path_inequality_report.clone(),
        decision,
        settings.kkt_regularization,
        settings.kkt_drop_tolerance,
        settings.residual_tolerance,
    )
    .map(Some)
}

fn lgr_linearized_active_inequality_values(
    report: &LgrPseudospectralPathInequalityReport,
    active_rows: &[usize],
    active_values: &[f64],
    step: &[f64],
) -> Result<Vec<f64>, TrajoptError> {
    validate_lgr_path_inequality_report(report)?;
    validate_lgr_dense_decision_vector(step, report.variable_count)?;
    if active_rows.len() != active_values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active values are inconsistent",
        });
    }
    validate_finite(
        "LGR sparse constrained KKT active values must be finite",
        active_values,
    )?;
    let mut linearized = Vec::with_capacity(active_rows.len());
    for (active_index, source_row) in active_rows.iter().enumerate() {
        if *source_row >= report.inequality_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR sparse constrained KKT active row index is invalid",
            });
        }
        let row_offset = *source_row * report.variable_count;
        let step_contribution = report.inequality_matrix
            [row_offset..row_offset + report.variable_count]
            .iter()
            .zip(step.iter())
            .map(|(coefficient, step)| coefficient * step)
            .sum::<f64>();
        linearized.push(active_values[active_index] + step_contribution);
    }
    Ok(linearized)
}

#[derive(Clone, Debug)]
struct LgrDenseLdltFactorization {
    unit_lower: Vec<f64>,
    diagonal: Vec<f64>,
    factorization_residual_norm: f64,
}

impl LgrDenseLdltFactorization {
    fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        let dimension = self.diagonal.len();
        if rhs.len() != dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR LDL RHS length is inconsistent",
            });
        }
        validate_finite("LGR LDL RHS must be finite", rhs)?;

        let mut y = vec![0.0; dimension];
        for row in 0..dimension {
            let mut sum = rhs[row];
            for (column, y_column) in y.iter().enumerate().take(row) {
                sum -= self.unit_lower[row * dimension + column] * *y_column;
            }
            y[row] = sum;
        }

        let mut z = vec![0.0; dimension];
        for row in 0..dimension {
            let diagonal = self.diagonal[row];
            if !diagonal.is_finite() || diagonal.abs() <= 1.0e-14 {
                return Err(TrajoptError::SingularSystem {
                    reason: "LGR LDL solve detected a singular diagonal",
                });
            }
            z[row] = y[row] / diagonal;
        }

        let mut solution = vec![0.0; dimension];
        for row in (0..dimension).rev() {
            let mut sum = z[row];
            for (column, solution_column) in
                solution.iter().enumerate().take(dimension).skip(row + 1)
            {
                sum -= self.unit_lower[column * dimension + row] * *solution_column;
            }
            solution[row] = sum;
        }
        validate_finite("LGR LDL solution must be finite", &solution)?;
        Ok(solution)
    }
}

fn lgr_dense_ldlt_factorization(
    dense: &[f64],
    dimension: usize,
) -> Result<LgrDenseLdltFactorization, TrajoptError> {
    if dense.len() != dimension * dimension || dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR LDL factorization matrix dimensions are inconsistent",
        });
    }
    validate_finite("LGR LDL factorization matrix must be finite", dense)?;

    let mut unit_lower = vec![0.0; dimension * dimension];
    let mut diagonal = vec![0.0; dimension];
    for index in 0..dimension {
        unit_lower[index * dimension + index] = 1.0;
    }

    for pivot in 0..dimension {
        for column in 0..pivot {
            let mut value = dense[pivot * dimension + column];
            for inner in 0..column {
                value -= unit_lower[pivot * dimension + inner]
                    * diagonal[inner]
                    * unit_lower[column * dimension + inner];
            }
            let column_diagonal = diagonal[column];
            if !column_diagonal.is_finite() || column_diagonal.abs() <= 1.0e-14 {
                return Err(TrajoptError::SingularSystem {
                    reason: "LGR LDL factorization detected a singular pivot",
                });
            }
            unit_lower[pivot * dimension + column] = value / column_diagonal;
        }

        let mut pivot_diagonal = dense[pivot * dimension + pivot];
        for inner in 0..pivot {
            let lower = unit_lower[pivot * dimension + inner];
            pivot_diagonal -= lower * lower * diagonal[inner];
        }
        if !pivot_diagonal.is_finite() || pivot_diagonal.abs() <= 1.0e-14 {
            return Err(TrajoptError::SingularSystem {
                reason: "LGR LDL factorization detected a singular pivot",
            });
        }
        diagonal[pivot] = pivot_diagonal;
    }

    let factorization_residual_norm =
        lgr_dense_ldlt_factorization_residual_norm(dense, dimension, &unit_lower, &diagonal);
    if !factorization_residual_norm.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR LDL factorization residual is non-finite",
        });
    }

    Ok(LgrDenseLdltFactorization {
        unit_lower,
        diagonal,
        factorization_residual_norm,
    })
}

fn lgr_dense_ldlt_factorization_residual_norm(
    dense: &[f64],
    dimension: usize,
    unit_lower: &[f64],
    diagonal: &[f64],
) -> f64 {
    let mut squared_norm = 0.0;
    for row in 0..dimension {
        for column in 0..dimension {
            let mut reconstructed = 0.0;
            let shared = row.min(column);
            for inner in 0..=shared {
                reconstructed += unit_lower[row * dimension + inner]
                    * diagonal[inner]
                    * unit_lower[column * dimension + inner];
            }
            let residual = reconstructed - dense[row * dimension + column];
            squared_norm += residual * residual;
        }
    }
    squared_norm.sqrt()
}

fn lgr_physical_collocation_time(
    layout: &LgrPseudospectralDecisionLayout,
    segment_index: usize,
    collocation_index: usize,
) -> Result<f64, TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    if segment_index >= layout.segment_count()
        || collocation_index >= layout.segment_grids[segment_index].collocation_count()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR collocation time index is out of range",
        });
    }
    let initial_time_s = layout.time_nodes_s[segment_index];
    let final_time_s = layout.time_nodes_s[segment_index + 1];
    let midpoint_time_s = 0.5 * (initial_time_s + final_time_s);
    let half_duration_s = 0.5 * (final_time_s - initial_time_s);
    Ok(midpoint_time_s
        + half_duration_s
            * layout.segment_grids[segment_index].collocation_nodes[collocation_index])
}

struct BorrowedLgrAutoDiffDynamics<'a, D> {
    dynamics: &'a mut D,
}

impl<D> CollocationAutoDiffDynamics for BorrowedLgrAutoDiffDynamics<'_, D>
where
    D: CollocationAutoDiffDynamics,
{
    fn eval_dual(
        &mut self,
        time_s: f64,
        state: &[CollocationAutoDiffScalar],
        control: &[CollocationAutoDiffScalar],
    ) -> Result<Vec<CollocationAutoDiffScalar>, TrajoptError> {
        self.dynamics.eval_dual(time_s, state, control)
    }
}

fn lgr_fixed_mesh_residual_row_count(ocp: &LgrPseudospectralFixedMeshOcp) -> usize {
    let differential_rows = ocp
        .layout
        .segment_grids
        .iter()
        .map(|grid| grid.collocation_count() * ocp.layout.state_dimension)
        .sum::<usize>();
    let continuity_rows = (ocp.layout.segment_count() - 1) * ocp.layout.state_dimension;
    let objective_rows = ocp
        .layout
        .segment_grids
        .iter()
        .map(LegendreGaussRadauGrid::collocation_count)
        .sum::<usize>()
        * ocp.quadratic_control_costs.len();
    differential_rows
        + continuity_rows
        + ocp.boundary_constraints.len()
        + ocp.path_constraints.len()
        + objective_rows
}

fn lgr_sparse_residual_report_from_dense(
    row_count: usize,
    column_count: usize,
    dense: &[f64],
    finite_difference_step: f64,
    drop_tolerance: f64,
) -> Result<LgrPseudospectralSparseResidualJacobianReport, TrajoptError> {
    if dense.len() != row_count * column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR dense residual Jacobian dimensions are inconsistent",
        });
    }
    validate_finite("LGR dense residual Jacobian values must be finite", dense)?;
    let mut column_offsets = Vec::with_capacity(column_count + 1);
    let mut row_indices = Vec::new();
    let mut values = Vec::new();
    column_offsets.push(0);
    for column in 0..column_count {
        for row in 0..row_count {
            let coefficient = dense[row * column_count + column];
            if coefficient.abs() > drop_tolerance {
                row_indices.push(row);
                values.push(coefficient);
            }
        }
        column_offsets.push(values.len());
    }
    let report = LgrPseudospectralSparseResidualJacobianReport {
        row_count,
        column_count,
        column_offsets,
        row_indices,
        values,
        finite_difference_step,
        drop_tolerance,
    };
    validate_lgr_sparse_residual_jacobian_report(&report)?;
    Ok(report)
}

/// Estimate per-segment state interpolation errors and recommend LGR `ph`
/// mesh-refinement actions.
///
/// The `reference` transcription must share the same segment intervals and
/// dimensions as `candidate`, and each reference segment must have at least as
/// many collocation nodes as the candidate segment. The estimator interpolates
/// each reference segment's support-state polynomial at the candidate support
/// nodes and computes `max(abs(candidate - reference)) / (1 + max(abs(state)))`.
///
/// # Errors
///
/// Returns [`TrajoptError`] when either transcription or the settings are
/// malformed, segment counts or dimensions differ, segment time intervals do
/// not match, a reference segment has lower degree than its candidate segment,
/// or an interpolation produces a non-finite value.
pub fn estimate_lgr_pseudospectral_mesh_refinement(
    candidate: &LgrPseudospectralTranscription,
    reference: &LgrPseudospectralTranscription,
    settings: &LgrPseudospectralMeshRefinementSettings,
) -> Result<LgrPseudospectralMeshRefinementReport, TrajoptError> {
    validate_lgr_mesh_refinement_settings(settings)?;
    validate_lgr_transcription(candidate)?;
    validate_lgr_transcription(reference)?;
    if candidate.segments.len() != reference.segments.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement transcriptions must have the same segment count",
        });
    }

    let mut estimates = Vec::with_capacity(candidate.segments.len());
    let mut recommendations = Vec::with_capacity(candidate.segments.len());
    let mut max_relative_error = 0.0_f64;
    let mut refined_segment_count = 0_usize;

    for (segment_index, (candidate_segment, reference_segment)) in candidate
        .segments
        .iter()
        .zip(reference.segments.iter())
        .enumerate()
    {
        validate_lgr_refinement_segment_pair(candidate_segment, reference_segment)?;
        let estimate =
            estimate_lgr_segment_state_error(segment_index, candidate_segment, reference_segment)?;
        max_relative_error = max_relative_error.max(estimate.relative_error);
        let action = recommend_lgr_segment_refinement(&estimate, settings);
        if action != LgrPseudospectralMeshRefinementAction::Accept {
            refined_segment_count += 1;
        }
        recommendations.push(LgrPseudospectralSegmentRefinementRecommendation {
            segment_index,
            collocation_count: estimate.collocation_count,
            action,
        });
        estimates.push(estimate);
    }

    Ok(LgrPseudospectralMeshRefinementReport {
        estimates,
        recommendations,
        max_relative_error,
        converged: refined_segment_count == 0,
        refined_segment_count,
    })
}

/// Apply LGR `ph` mesh-refinement recommendations and seed a refined
/// transcription by interpolating the source states and controls.
///
/// Accepted segments are cloned unchanged. `RaiseDegree` recommendations keep
/// the source interval and interpolate support states and collocation controls
/// onto the requested LGR grid. `Split` recommendations subdivide the source
/// interval uniformly and seed every child interval from the source segment's
/// support-state and collocation-control polynomials.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the source transcription is malformed, the
/// report does not contain one matching recommendation per source segment, a
/// recommendation has invalid dimensions, an interpolated seed is non-finite, or
/// the resulting transcription is malformed.
pub fn apply_lgr_pseudospectral_mesh_refinement(
    source: &LgrPseudospectralTranscription,
    report: &LgrPseudospectralMeshRefinementReport,
) -> Result<LgrPseudospectralMeshRefinementApplication, TrajoptError> {
    validate_lgr_transcription(source)?;
    validate_lgr_refinement_report_for_source(source, report)?;

    let mut refined_segments = Vec::new();
    let mut segment_mappings = Vec::new();

    for (source_index, source_segment) in source.segments.iter().enumerate() {
        let recommendation = &report.recommendations[source_index];
        let action = recommendation.action.clone();
        match &recommendation.action {
            LgrPseudospectralMeshRefinementAction::Accept => {
                let refined_segment_index = refined_segments.len();
                refined_segments.push(source_segment.clone());
                segment_mappings.push(LgrPseudospectralRefinedSegmentMapping {
                    source_segment_index: source_index,
                    refined_segment_index,
                    child_index: 0,
                    child_count: 1,
                    action,
                });
            }
            LgrPseudospectralMeshRefinementAction::RaiseDegree { collocation_count } => {
                let refined_segment = seed_lgr_refined_segment(
                    source_segment,
                    source_segment.initial_time_s,
                    source_segment.final_time_s,
                    *collocation_count,
                )?;
                let refined_segment_index = refined_segments.len();
                refined_segments.push(refined_segment);
                segment_mappings.push(LgrPseudospectralRefinedSegmentMapping {
                    source_segment_index: source_index,
                    refined_segment_index,
                    child_index: 0,
                    child_count: 1,
                    action,
                });
            }
            LgrPseudospectralMeshRefinementAction::Split {
                segment_count,
                collocation_count,
            } => {
                let child_duration_s = source_segment.duration_s() / (*segment_count as f64);
                for child_index in 0..*segment_count {
                    let child_initial_time_s =
                        source_segment.initial_time_s + child_duration_s * (child_index as f64);
                    let child_final_time_s = if child_index + 1 == *segment_count {
                        source_segment.final_time_s
                    } else {
                        source_segment.initial_time_s
                            + child_duration_s * ((child_index + 1) as f64)
                    };
                    let refined_segment = seed_lgr_refined_segment(
                        source_segment,
                        child_initial_time_s,
                        child_final_time_s,
                        *collocation_count,
                    )?;
                    let refined_segment_index = refined_segments.len();
                    refined_segments.push(refined_segment);
                    segment_mappings.push(LgrPseudospectralRefinedSegmentMapping {
                        source_segment_index: source_index,
                        refined_segment_index,
                        child_index,
                        child_count: *segment_count,
                        action: action.clone(),
                    });
                }
            }
        }
    }

    let transcription = LgrPseudospectralTranscription {
        segments: refined_segments,
    };
    validate_lgr_transcription(&transcription)?;
    Ok(LgrPseudospectralMeshRefinementApplication {
        transcription,
        segment_mappings,
    })
}

/// Run an iterative LGR `ph` mesh-refinement loop.
///
/// `reference_builder` is called once per iteration with the current candidate
/// transcription and must return a same-interval reference transcription with
/// equal or higher segment degrees. This function estimates the mesh error,
/// applies the resulting recommendations, and repeats until convergence or the
/// iteration cap is reached.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the initial transcription, settings, iteration
/// cap, reference transcription, recommendation report, or seeded refined
/// transcription is malformed.
pub fn refine_lgr_pseudospectral_mesh_with_reference<F>(
    initial: &LgrPseudospectralTranscription,
    settings: &LgrPseudospectralMeshRefinementSettings,
    max_iterations: usize,
    mut reference_builder: F,
) -> Result<LgrPseudospectralMeshRefinementLoopReport, TrajoptError>
where
    F: FnMut(
        usize,
        &LgrPseudospectralTranscription,
    ) -> Result<LgrPseudospectralTranscription, TrajoptError>,
{
    validate_lgr_mesh_refinement_settings(settings)?;
    validate_lgr_transcription(initial)?;
    if max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement loop requires at least one iteration",
        });
    }

    let mut candidate = initial.clone();
    let mut iterations = Vec::with_capacity(max_iterations);
    let mut converged = false;
    let mut max_relative_error = 0.0;
    let mut refined_segment_count = 0;

    for iteration_index in 0..max_iterations {
        let reference = reference_builder(iteration_index, &candidate)?;
        let candidate_segment_count = candidate.segments.len();
        let reference_segment_count = reference.segments.len();
        let refinement_report =
            estimate_lgr_pseudospectral_mesh_refinement(&candidate, &reference, settings)?;
        converged = refinement_report.converged;
        max_relative_error = refinement_report.max_relative_error;
        refined_segment_count = refinement_report.refined_segment_count;
        let application = apply_lgr_pseudospectral_mesh_refinement(&candidate, &refinement_report)?;

        candidate = application.transcription;
        iterations.push(LgrPseudospectralMeshRefinementIterationReport {
            iteration_index,
            candidate_segment_count,
            reference_segment_count,
            refinement_report,
            segment_mappings: application.segment_mappings,
        });

        if converged {
            break;
        }
    }

    Ok(LgrPseudospectralMeshRefinementLoopReport {
        transcription: candidate,
        iteration_count: iterations.len(),
        iterations,
        converged,
        max_relative_error,
        refined_segment_count,
    })
}

fn evaluate_lgr_pseudospectral_segment_with_callbacks<D, L>(
    segment: &LgrPseudospectralSegment,
    dynamics: &mut D,
    lagrange: &mut L,
) -> Result<LgrPseudospectralSegmentReport, TrajoptError>
where
    D: CollocationDynamics,
    L: CollocationLagrange,
{
    let collocation_count = segment.grid.collocation_count();
    let support_count = segment.grid.support_count();
    let half_duration_s = segment.half_duration_s();
    let midpoint_time_s = 0.5 * (segment.initial_time_s + segment.final_time_s);
    let mut collocation_times_s = Vec::with_capacity(collocation_count);
    let mut quadrature_weights_s = Vec::with_capacity(collocation_count);
    let mut residuals = Vec::with_capacity(collocation_count * segment.state_dimension);
    let mut quadrature_cost = 0.0;

    for row in 0..collocation_count {
        let collocation_time_s =
            midpoint_time_s + half_duration_s * segment.grid.collocation_nodes[row];
        collocation_times_s.push(collocation_time_s);
        let physical_weight_s = half_duration_s * segment.grid.quadrature_weights[row];
        quadrature_weights_s.push(physical_weight_s);

        let state_offset = row * segment.state_dimension;
        let state = &segment.support_states[state_offset..state_offset + segment.state_dimension];
        let control_offset = row * segment.control_dimension;
        let control = &segment.collocation_controls
            [control_offset..control_offset + segment.control_dimension];
        let derivative = dynamics.eval(collocation_time_s, state, control)?;
        validate_lgr_dynamics_output(&derivative, segment.state_dimension)?;
        let lagrange_value = lagrange.eval(collocation_time_s, state, control)?;
        if !lagrange_value.is_finite() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR segment Lagrange value must be finite",
            });
        }
        quadrature_cost += physical_weight_s * lagrange_value;

        let differentiation_offset = row * support_count;
        for (state_index, derivative_value) in derivative.iter().enumerate() {
            let spectral_derivative = (0..support_count)
                .map(|support_index| {
                    segment.grid.differentiation_matrix[differentiation_offset + support_index]
                        * segment.support_states
                            [support_index * segment.state_dimension + state_index]
                })
                .sum::<f64>();
            residuals.push(spectral_derivative - half_duration_s * *derivative_value);
        }
    }

    Ok(LgrPseudospectralSegmentReport {
        collocation_times_s,
        quadrature_weights_s,
        residual_norm: norm(&residuals),
        max_abs_residual: max_abs(&residuals),
        residuals,
        quadrature_cost,
    })
}

fn lgr_collocation_nodes(collocation_count: usize) -> Result<Vec<f64>, TrajoptError> {
    let mut nodes = Vec::with_capacity(collocation_count);
    nodes.push(-1.0);
    for index in 1..collocation_count {
        let mut node = -(core::f64::consts::PI * (index as f64) / (collocation_count as f64)).cos();
        for _ in 0..64 {
            let value = legendre_polynomial(collocation_count, node)
                + legendre_polynomial(collocation_count - 1, node);
            let derivative = legendre_polynomial_derivative(collocation_count, node)
                + legendre_polynomial_derivative(collocation_count - 1, node);
            if !value.is_finite() || !derivative.is_finite() || derivative.abs() <= f64::EPSILON {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR interior node Newton solve became singular",
                });
            }
            let next = (node - value / derivative).clamp(-1.0 + 1.0e-14, 1.0 - 1.0e-14);
            if (next - node).abs() <= 1.0e-15 {
                node = next;
                break;
            }
            node = next;
        }
        let residual = (legendre_polynomial(collocation_count, node)
            + legendre_polynomial(collocation_count - 1, node))
        .abs();
        if residual > 1.0e-12 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR interior node Newton solve did not converge",
            });
        }
        nodes.push(node);
    }
    nodes.sort_by(|left, right| left.total_cmp(right));
    Ok(nodes)
}

fn lgr_quadrature_weights(nodes: &[f64]) -> Result<Vec<f64>, TrajoptError> {
    let collocation_count = nodes.len();
    let scale = (collocation_count * collocation_count) as f64;
    let mut weights = Vec::with_capacity(collocation_count);
    for node in nodes {
        let polynomial = legendre_polynomial(collocation_count - 1, *node);
        let denominator = scale * polynomial * polynomial;
        if !denominator.is_finite() || denominator <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR quadrature weight denominator is invalid",
            });
        }
        weights.push((1.0 - node) / denominator);
    }
    Ok(weights)
}

fn barycentric_differentiation_matrix(
    support_nodes: &[f64],
    collocation_count: usize,
) -> Result<Vec<f64>, TrajoptError> {
    let support_count = support_nodes.len();
    let barycentric_weights = barycentric_weights(support_nodes)?;
    let mut matrix = vec![0.0; collocation_count * support_count];
    for row in 0..collocation_count {
        let row_node = support_nodes[row];
        let row_weight = barycentric_weights[row];
        let mut diagonal = 0.0;
        for col in 0..support_count {
            if row == col {
                continue;
            }
            let entry = barycentric_weights[col] / row_weight / (row_node - support_nodes[col]);
            matrix[row * support_count + col] = entry;
            diagonal -= entry;
        }
        matrix[row * support_count + row] = diagonal;
    }
    Ok(matrix)
}

fn barycentric_weights(nodes: &[f64]) -> Result<Vec<f64>, TrajoptError> {
    let mut weights = Vec::with_capacity(nodes.len());
    for (index, node) in nodes.iter().enumerate() {
        let mut product = 1.0;
        for (other_index, other_node) in nodes.iter().enumerate() {
            if index != other_index {
                product *= node - other_node;
            }
        }
        if !product.is_finite() || product == 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR support nodes must be distinct and finite",
            });
        }
        weights.push(1.0 / product);
    }
    Ok(weights)
}

fn legendre_polynomial(degree: usize, node: f64) -> f64 {
    if degree == 0 {
        return 1.0;
    }
    let mut previous = 1.0;
    let mut current = node;
    for order in 2..=degree {
        let order_f = order as f64;
        let next = ((2.0 * order_f - 1.0) * node * current - (order_f - 1.0) * previous) / order_f;
        previous = current;
        current = next;
    }
    current
}

fn legendre_polynomial_derivative(degree: usize, node: f64) -> f64 {
    if degree == 0 {
        return 0.0;
    }
    let polynomial = legendre_polynomial(degree, node);
    let previous = legendre_polynomial(degree - 1, node);
    (degree as f64) * (node * polynomial - previous) / (node * node - 1.0)
}

fn validate_grid(grid: &LegendreGaussRadauGrid) -> Result<(), TrajoptError> {
    let collocation_count = grid.collocation_count();
    let support_count = grid.support_count();
    if support_count != collocation_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR support node dimension is inconsistent",
        });
    }
    if grid.quadrature_weights.len() != collocation_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR quadrature weight dimension is inconsistent",
        });
    }
    if grid.differentiation_matrix.len() != collocation_count * support_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR differentiation matrix dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR collocation nodes must be finite",
        &grid.collocation_nodes,
    )?;
    validate_finite("LGR support nodes must be finite", &grid.support_nodes)?;
    validate_finite(
        "LGR quadrature weights must be finite",
        &grid.quadrature_weights,
    )?;
    validate_finite(
        "LGR differentiation matrix values must be finite",
        &grid.differentiation_matrix,
    )?;
    if (grid.endpoint - 1.0).abs() > f64::EPSILON
        || (grid.collocation_nodes[0] + 1.0).abs() > f64::EPSILON
        || (grid.support_nodes[collocation_count] - 1.0).abs() > f64::EPSILON
        || !grid
            .support_nodes
            .windows(2)
            .all(|window| window[0] < window[1])
        || grid.quadrature_weights.iter().any(|weight| *weight <= 0.0)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR grid nodes and weights are invalid",
        });
    }
    Ok(())
}

fn validate_lgr_decision_layout(
    layout: &LgrPseudospectralDecisionLayout,
) -> Result<(), TrajoptError> {
    let segment_count = layout.segment_collocation_counts.len();
    if segment_count == 0
        || layout.time_nodes_s.len() != segment_count + 1
        || layout.segment_grids.len() != segment_count
        || layout.segment_state_offsets.len() != segment_count
        || layout.segment_control_offsets.len() != segment_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision layout dimensions are inconsistent",
        });
    }
    if layout.state_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision layout state dimension must be nonzero",
        });
    }
    validate_finite(
        "LGR decision layout time nodes must be finite",
        &layout.time_nodes_s,
    )?;
    if !layout
        .time_nodes_s
        .windows(2)
        .all(|window| window[0] < window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision layout time nodes must be strictly increasing",
        });
    }

    let mut expected_state_offset = 0_usize;
    for (segment_index, grid) in layout.segment_grids.iter().enumerate() {
        validate_grid(grid)?;
        if layout.segment_collocation_counts[segment_index] != grid.collocation_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout grid degree is inconsistent",
            });
        }
        if layout.segment_state_offsets[segment_index] != expected_state_offset {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout state offsets are inconsistent",
            });
        }
        expected_state_offset += grid.support_count() * layout.state_dimension;
    }
    if layout.state_variable_count != expected_state_offset {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision layout state variable count is inconsistent",
        });
    }

    let mut expected_control_count = 0_usize;
    for (segment_index, grid) in layout.segment_grids.iter().enumerate() {
        let expected_control_offset = layout.state_variable_count + expected_control_count;
        if layout.segment_control_offsets[segment_index] != expected_control_offset {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR decision layout control offsets are inconsistent",
            });
        }
        expected_control_count += grid.collocation_count() * layout.control_dimension;
    }
    if layout.control_variable_count != expected_control_count
        || layout.variable_count != layout.state_variable_count + layout.control_variable_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision layout variable count is inconsistent",
        });
    }
    Ok(())
}

fn validate_lgr_decision_vector(
    layout: &LgrPseudospectralDecisionLayout,
    decision: &[f64],
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    if decision.len() != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision vector dimension is inconsistent",
        });
    }
    validate_finite("LGR decision vector values must be finite", decision)
}

fn validate_lgr_boundary_constraint(
    layout: &LgrPseudospectralDecisionLayout,
    constraint: &LgrPseudospectralBoundaryConstraint,
) -> Result<(), TrajoptError> {
    if !constraint.target.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR boundary target must be finite",
        });
    }
    if constraint.mesh_node_index > layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR boundary mesh node is out of range",
        });
    }
    if constraint.component >= layout.state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR boundary component is out of range",
        });
    }
    Ok(())
}

fn validate_lgr_path_bounds(lower: Option<f64>, upper: Option<f64>) -> Result<(), TrajoptError> {
    if lower.is_none() && upper.is_none() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR soft path constraint requires at least one bound",
        });
    }
    if lower.is_some_and(|value| !value.is_finite())
        || upper.is_some_and(|value| !value.is_finite())
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR soft path bounds must be finite",
        });
    }
    if let (Some(lower), Some(upper)) = (lower, upper)
        && lower > upper
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR soft path lower bound exceeds upper bound",
        });
    }
    Ok(())
}

fn validate_lgr_hard_path_bounds(
    lower: Option<f64>,
    upper: Option<f64>,
) -> Result<(), TrajoptError> {
    if lower.is_none() && upper.is_none() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR hard path constraint requires at least one bound",
        });
    }
    if lower.is_some_and(|value| !value.is_finite())
        || upper.is_some_and(|value| !value.is_finite())
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR hard path bounds must be finite",
        });
    }
    if let (Some(lower), Some(upper)) = (lower, upper)
        && lower > upper
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR hard path lower bound exceeds upper bound",
        });
    }
    Ok(())
}

fn validate_lgr_nonlinear_path_bounds(
    constraint: &LgrPseudospectralNonlinearHardPathConstraint,
) -> Result<(), TrajoptError> {
    if constraint.lower_bounds.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear hard path dimension must be nonzero",
        });
    }
    if constraint.lower_bounds.len() != constraint.upper_bounds.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear hard path bound dimensions are inconsistent",
        });
    }
    let mut has_bound = false;
    for (lower, upper) in constraint
        .lower_bounds
        .iter()
        .zip(constraint.upper_bounds.iter())
    {
        if let Some(lower) = lower {
            has_bound = true;
            if !lower.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR nonlinear hard path lower bound must be finite",
                });
            }
        }
        if let Some(upper) = upper {
            has_bound = true;
            if !upper.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR nonlinear hard path upper bound must be finite",
                });
            }
        }
        if let (Some(lower), Some(upper)) = (lower, upper)
            && lower > upper
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR nonlinear hard path lower bound exceeds upper bound",
            });
        }
    }
    if !has_bound {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear hard path requires at least one bound",
        });
    }
    Ok(())
}

fn validate_lgr_path_constraint(
    layout: &LgrPseudospectralDecisionLayout,
    constraint: &LgrPseudospectralSoftPathConstraint,
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    validate_lgr_path_bounds(constraint.lower, constraint.upper)?;
    if !constraint.weight.is_finite() || constraint.weight <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR soft path weight must be finite and positive",
        });
    }
    if constraint.segment_index >= layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR soft path segment index is out of range",
        });
    }
    if constraint.collocation_index
        >= layout.segment_grids[constraint.segment_index].collocation_count()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR soft path collocation index is out of range",
        });
    }
    match constraint.variable {
        LgrPseudospectralScalarVariable::StateComponent { component } => {
            if component >= layout.state_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR soft path state component is out of range",
                });
            }
        }
        LgrPseudospectralScalarVariable::ControlComponent { component } => {
            if component >= layout.control_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR soft path control component is out of range",
                });
            }
        }
    }
    Ok(())
}

fn validate_lgr_hard_path_constraint(
    layout: &LgrPseudospectralDecisionLayout,
    constraint: &LgrPseudospectralHardPathConstraint,
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    validate_lgr_hard_path_bounds(constraint.lower, constraint.upper)?;
    if constraint.segment_index >= layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR hard path segment index is out of range",
        });
    }
    if constraint.collocation_index
        >= layout.segment_grids[constraint.segment_index].collocation_count()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR hard path collocation index is out of range",
        });
    }
    match constraint.variable {
        LgrPseudospectralScalarVariable::StateComponent { component } => {
            if component >= layout.state_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR hard path state component is out of range",
                });
            }
        }
        LgrPseudospectralScalarVariable::ControlComponent { component } => {
            if component >= layout.control_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR hard path control component is out of range",
                });
            }
        }
    }
    Ok(())
}

fn validate_lgr_nonlinear_hard_path_constraint(
    layout: &LgrPseudospectralDecisionLayout,
    constraint: &LgrPseudospectralNonlinearHardPathConstraint,
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    validate_lgr_nonlinear_path_bounds(constraint)?;
    if constraint.segment_index >= layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear hard path segment index is out of range",
        });
    }
    if constraint.collocation_index
        >= layout.segment_grids[constraint.segment_index].collocation_count()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear hard path collocation index is out of range",
        });
    }
    Ok(())
}

fn validate_lgr_quadratic_control_cost(
    layout: &LgrPseudospectralDecisionLayout,
    cost: &LgrPseudospectralQuadraticControlCost,
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    if !cost.weight.is_finite() || cost.weight <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR control-cost weight must be finite and positive",
        });
    }
    if cost.component >= layout.control_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR control-cost component is out of range",
        });
    }
    Ok(())
}

fn validate_lgr_reference_profile_mapping(
    layout: &LgrPseudospectralDecisionLayout,
    mapping: LgrPseudospectralReferenceProfileMapping,
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    if mapping.radius_component >= layout.state_dimension
        || mapping.speed_component >= layout.state_dimension
        || mapping.flight_path_angle_component >= layout.state_dimension
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR reference profile mapping component is out of range",
        });
    }
    Ok(())
}

fn validate_lgr_reference_profile_report(
    report: &LgrPseudospectralReferenceProfileReport,
) -> Result<(), TrajoptError> {
    if report.sample_count == 0 || report.samples.len() != report.sample_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR reference profile sample count is inconsistent",
        });
    }
    if report.start_time_s.to_bits() != report.samples[0].time_s.to_bits()
        || report.end_time_s.to_bits() != report.samples[report.samples.len() - 1].time_s.to_bits()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR reference profile time endpoints are inconsistent",
        });
    }
    if !report
        .samples
        .windows(2)
        .all(|window| window[0].time_s < window[1].time_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR reference profile times must be strictly increasing",
        });
    }
    for sample in &report.samples {
        validate_lgr_reference_profile_sample(sample)?;
    }
    Ok(())
}

fn validate_lgr_fixed_mesh_ocp(ocp: &LgrPseudospectralFixedMeshOcp) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(&ocp.layout)?;
    for constraint in &ocp.boundary_constraints {
        validate_lgr_boundary_constraint(&ocp.layout, constraint)?;
    }
    for constraint in &ocp.path_constraints {
        validate_lgr_path_constraint(&ocp.layout, constraint)?;
    }
    for constraint in &ocp.hard_path_constraints {
        validate_lgr_hard_path_constraint(&ocp.layout, constraint)?;
    }
    for constraint in &ocp.nonlinear_hard_path_constraints {
        validate_lgr_nonlinear_hard_path_constraint(&ocp.layout, constraint)?;
    }
    for cost in &ocp.quadratic_control_costs {
        validate_lgr_quadratic_control_cost(&ocp.layout, cost)?;
    }
    Ok(())
}

fn validate_lgr_path_inequality_report(
    report: &LgrPseudospectralPathInequalityReport,
) -> Result<(), TrajoptError> {
    if report.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR path inequality variable count must be nonzero",
        });
    }
    if report.inequality_matrix.len() != report.inequality_count * report.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR path inequality matrix dimensions are inconsistent",
        });
    }
    if report.inequality_rhs.len() != report.inequality_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR path inequality RHS dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR path inequality matrix values must be finite",
        &report.inequality_matrix,
    )?;
    validate_finite(
        "LGR path inequality RHS values must be finite",
        &report.inequality_rhs,
    )
}

fn validate_lgr_nonlinear_path_inequality_report(
    report: &LgrPseudospectralNonlinearPathInequalityReport,
) -> Result<(), TrajoptError> {
    if report.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear path inequality variable count must be nonzero",
        });
    }
    if report.path_values.len() != report.path_value_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear path value count is inconsistent",
        });
    }
    if report.inequality_values.len() != report.inequality_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear path inequality count is inconsistent",
        });
    }
    validate_finite(
        "LGR nonlinear path values must be finite",
        &report.path_values,
    )?;
    validate_finite(
        "LGR nonlinear path inequality values must be finite",
        &report.inequality_values,
    )?;
    let max_violation = report
        .inequality_values
        .iter()
        .map(|value| value.max(0.0))
        .fold(0.0_f64, f64::max);
    if !report.max_violation.is_finite()
        || (report.max_violation - max_violation).abs() > 1.0e-12 * max_violation.abs().max(1.0)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR nonlinear path max violation is inconsistent",
        });
    }
    Ok(())
}

fn validate_lgr_objective_report(
    report: &LgrPseudospectralObjectiveReport,
) -> Result<(), TrajoptError> {
    if !report.terminal_time_s.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR objective terminal time must be finite",
        });
    }
    if report.terminal_state.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR objective terminal state must be nonempty",
        });
    }
    validate_finite(
        "LGR objective terminal state values must be finite",
        &report.terminal_state,
    )?;
    let costs = [
        report.lagrange_quadrature_cost,
        report.quadratic_control_cost,
        report.mayer_cost,
        report.total_objective,
    ];
    validate_finite("LGR objective costs must be finite", &costs)?;
    let expected =
        report.lagrange_quadrature_cost + report.quadratic_control_cost + report.mayer_cost;
    if (report.total_objective - expected).abs() > 1.0e-12 * expected.abs().max(1.0) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR objective total is inconsistent",
        });
    }
    Ok(())
}

fn validate_lgr_finite_difference_settings(
    finite_difference_step: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !finite_difference_step.is_finite() || finite_difference_step <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian step must be finite and positive",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_lgr_sparse_path_inequality_jacobian_report(
    report: &LgrPseudospectralSparsePathInequalityJacobianReport,
) -> Result<(), TrajoptError> {
    if report.column_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse path inequality Jacobian column count must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse path inequality Jacobian column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse path inequality Jacobian column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse path inequality Jacobian row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse path inequality Jacobian row index is out of range",
        });
    }
    validate_lgr_finite_difference_settings(report.finite_difference_step, report.drop_tolerance)?;
    validate_finite(
        "LGR sparse path inequality Jacobian values must be finite",
        &report.values,
    )
}

fn validate_lgr_sparse_residual_jacobian_report(
    report: &LgrPseudospectralSparseResidualJacobianReport,
) -> Result<(), TrajoptError> {
    if report.column_count == 0 || report.row_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian dimensions must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian row index is out of range",
        });
    }
    if !report.finite_difference_step.is_finite() || report.finite_difference_step < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian step metadata must be finite and non-negative",
        });
    }
    if !report.drop_tolerance.is_finite() || report.drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian drop tolerance must be finite and non-negative",
        });
    }
    validate_finite(
        "LGR sparse residual Jacobian values must be finite",
        &report.values,
    )
}

fn validate_lgr_colored_sparse_residual_jacobian_report(
    report: &LgrPseudospectralColoredSparseResidualJacobianReport,
) -> Result<(), TrajoptError> {
    validate_lgr_sparse_residual_jacobian_report(&report.sparse_jacobian)?;
    if report.column_colors.len() != report.sparse_jacobian.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR colored sparse residual Jacobian colors must match columns",
        });
    }
    if report.color_count == 0
        || report
            .column_colors
            .iter()
            .any(|color| *color >= report.color_count)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR colored sparse residual Jacobian colors are inconsistent",
        });
    }
    if report.residual_evaluation_count != 1 + 2 * report.color_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR colored sparse residual Jacobian evaluation count is inconsistent",
        });
    }
    if report.structural_nonzero_count < report.sparse_jacobian.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR colored sparse residual Jacobian structural count is inconsistent",
        });
    }
    Ok(())
}

fn validate_lgr_inequality_dimension(
    inequality_values: &[f64],
    row_count: usize,
) -> Result<(), TrajoptError> {
    if inequality_values.len() != row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse path inequality Jacobian row dimension changed",
        });
    }
    validate_finite(
        "LGR sparse path inequality values must be finite",
        inequality_values,
    )
}

fn validate_lgr_residual_dimension(
    residuals: &[f64],
    row_count: usize,
) -> Result<(), TrajoptError> {
    if residuals.len() != row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse residual Jacobian residual dimension changed",
        });
    }
    validate_finite("LGR sparse residual values must be finite", residuals)
}

fn validate_lgr_sparse_hessian_settings(
    regularization: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !regularization.is_finite() || regularization < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian regularization must be finite and non-negative",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_lgr_sparse_augmented_kkt_settings(
    regularization: f64,
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !regularization.is_finite() || regularization <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT regularization must be finite and positive",
        });
    }
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_lgr_sparse_gauss_newton_hessian_report(
    report: &LgrPseudospectralSparseGaussNewtonHessianReport,
) -> Result<(), TrajoptError> {
    if report.row_count == 0 || report.column_count == 0 || report.row_count != report.column_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian dimensions must be square and nonzero",
        });
    }
    if report.residual_row_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian source residual count must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton Hessian row index is out of range",
        });
    }
    validate_lgr_sparse_hessian_settings(report.regularization, report.drop_tolerance)?;
    validate_finite(
        "LGR sparse Gauss-Newton Hessian values must be finite",
        &report.values,
    )
}

fn validate_lgr_sparse_augmented_kkt_report(
    report: &LgrPseudospectralSparseAugmentedKktReport,
) -> Result<(), TrajoptError> {
    if report.row_count == 0 || report.column_count == 0 || report.row_count != report.column_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT dimensions must be square and nonzero",
        });
    }
    if report.variable_count == 0 || report.residual_row_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT source dimensions must be nonzero",
        });
    }
    if report.row_count != report.variable_count + report.residual_row_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT dimensions are inconsistent",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse augmented KKT row index is out of range",
        });
    }
    validate_lgr_sparse_augmented_kkt_settings(report.regularization, report.drop_tolerance)?;
    validate_finite(
        "LGR sparse augmented KKT values must be finite",
        &report.values,
    )
}

fn validate_lgr_sparse_constrained_kkt_report(
    report: &LgrPseudospectralSparseConstrainedKktReport,
) -> Result<(), TrajoptError> {
    if report.row_count == 0 || report.column_count == 0 || report.row_count != report.column_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT dimensions must be square and nonzero",
        });
    }
    if report.variable_count == 0
        || report.residual_row_count == 0
        || report.active_inequality_row_count == 0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT source dimensions must be nonzero",
        });
    }
    if report.row_count
        != report.variable_count + report.residual_row_count + report.active_inequality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT dimensions are inconsistent",
        });
    }
    if report.active_inequality_rows.len() != report.active_inequality_row_count
        || report.active_inequality_values.len() != report.active_inequality_row_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active rows are inconsistent",
        });
    }
    if report
        .active_inequality_rows
        .windows(2)
        .any(|window| window[0] >= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active rows must be strictly increasing",
        });
    }
    if report.source_jacobian_nonzero_count == 0 || report.source_inequality_nonzero_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT source nonzero counts must be nonzero",
        });
    }
    if report.column_offsets.len() != report.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT column offsets are inconsistent",
        });
    }
    if report.column_offsets.first() != Some(&0)
        || report.column_offsets.last() != Some(&report.values.len())
        || !report
            .column_offsets
            .windows(2)
            .all(|window| window[0] <= window[1])
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT column offsets are invalid",
        });
    }
    if report.row_indices.len() != report.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT row/value lengths differ",
        });
    }
    if !report.row_indices.iter().all(|row| *row < report.row_count) {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT row index is out of range",
        });
    }
    validate_lgr_sparse_augmented_kkt_settings(report.regularization, report.drop_tolerance)?;
    if !report.active_tolerance.is_finite() || report.active_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse constrained KKT active tolerance must be finite and non-negative",
        });
    }
    validate_finite(
        "LGR sparse constrained KKT active values must be finite",
        &report.active_inequality_values,
    )?;
    validate_finite(
        "LGR sparse constrained KKT values must be finite",
        &report.values,
    )
}

fn validate_lgr_sparse_cg_settings(
    report: &LgrPseudospectralSparseGaussNewtonHessianReport,
    rhs: &[f64],
    initial_solution: &[f64],
    max_iterations: usize,
    residual_tolerance: f64,
) -> Result<(), TrajoptError> {
    validate_lgr_sparse_gauss_newton_hessian_report(report)?;
    if rhs.len() != report.row_count || initial_solution.len() != report.column_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton CG dimensions are inconsistent",
        });
    }
    if max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton CG iteration limit must be positive",
        });
    }
    if !residual_tolerance.is_finite() || residual_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR sparse Gauss-Newton CG tolerance must be finite and non-negative",
        });
    }
    validate_finite("LGR sparse Gauss-Newton CG RHS must be finite", rhs)?;
    validate_finite(
        "LGR sparse Gauss-Newton CG initial solution must be finite",
        initial_solution,
    )
}

fn validate_lgr_analytic_residual_jacobian_drop_tolerance(
    drop_tolerance: f64,
) -> Result<(), TrajoptError> {
    if !drop_tolerance.is_finite() || drop_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR analytic residual Jacobian drop tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_lgr_dynamics_jacobian_sample(
    sample: &CollocationDynamicsJacobianSample,
    state_dimension: usize,
    control_dimension: usize,
) -> Result<(), TrajoptError> {
    validate_lgr_dynamics_output(&sample.derivative, state_dimension)?;
    if sample.state_jacobian.len() != state_dimension * state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR dynamics state Jacobian dimensions are inconsistent",
        });
    }
    if sample.control_jacobian.len() != state_dimension * control_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR dynamics control Jacobian dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR dynamics state Jacobian values must be finite",
        &sample.state_jacobian,
    )?;
    validate_finite(
        "LGR dynamics control Jacobian values must be finite",
        &sample.control_jacobian,
    )
}

fn lgr_segment_structural_columns(
    layout: &LgrPseudospectralDecisionLayout,
    segment_index: usize,
) -> Vec<usize> {
    let grid = &layout.segment_grids[segment_index];
    let mut columns = Vec::with_capacity(
        grid.support_count() * layout.state_dimension
            + grid.collocation_count() * layout.control_dimension,
    );
    let state_offset = layout.segment_state_offsets[segment_index];
    for index in 0..grid.support_count() * layout.state_dimension {
        columns.push(state_offset + index);
    }
    let control_offset = layout.segment_control_offsets[segment_index];
    for index in 0..grid.collocation_count() * layout.control_dimension {
        columns.push(control_offset + index);
    }
    columns
}

fn lgr_push_structural_row(
    column_rows: &mut [Vec<usize>],
    row: usize,
    columns: &[usize],
) -> Result<(), TrajoptError> {
    for column in columns {
        let rows = column_rows
            .get_mut(*column)
            .ok_or(TrajoptError::InvalidPayload {
                reason: "LGR residual structural pattern column is out of range",
            })?;
        if rows.last() != Some(&row) {
            rows.push(row);
        }
    }
    Ok(())
}

fn lgr_greedy_column_colors(
    column_rows: &[Vec<usize>],
    row_count: usize,
) -> Result<Vec<usize>, TrajoptError> {
    let mut row_colors = vec![Vec::<usize>::new(); row_count];
    let mut column_colors = Vec::with_capacity(column_rows.len());
    for rows in column_rows {
        if rows.iter().any(|row| *row >= row_count) {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR residual structural pattern row is out of range",
            });
        }
        let mut color = 0;
        while rows.iter().any(|row| row_colors[*row].contains(&color)) {
            color += 1;
        }
        for row in rows {
            row_colors[*row].push(color);
        }
        column_colors.push(color);
    }
    Ok(column_colors)
}

fn validate_lgr_dense_decision_vector(
    decision: &[f64],
    variable_count: usize,
) -> Result<(), TrajoptError> {
    if decision.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR dense decision vector dimension is inconsistent",
        });
    }
    validate_finite("LGR dense decision vector values must be finite", decision)
}

fn validate_lgr_decision_bounds(
    layout: &LgrPseudospectralDecisionLayout,
    lower_bounds: &[f64],
    upper_bounds: &[f64],
) -> Result<(), TrajoptError> {
    validate_lgr_decision_layout(layout)?;
    if lower_bounds.len() != layout.variable_count || upper_bounds.len() != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision bounds must match variable count",
        });
    }
    validate_finite("LGR decision lower bounds must be finite", lower_bounds)?;
    validate_finite("LGR decision upper bounds must be finite", upper_bounds)?;
    if !lower_bounds
        .iter()
        .zip(upper_bounds.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision bounds are contradictory",
        });
    }
    Ok(())
}

fn validate_lgr_decision_bounds_report(
    report: &LgrPseudospectralDecisionBoundsReport,
) -> Result<(), TrajoptError> {
    if report.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision bounds variable count must be nonzero",
        });
    }
    if report.lower_bounds.len() != report.variable_count
        || report.upper_bounds.len() != report.variable_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision bounds report dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR decision lower bounds must be finite",
        &report.lower_bounds,
    )?;
    validate_finite(
        "LGR decision upper bounds must be finite",
        &report.upper_bounds,
    )?;
    if !report
        .lower_bounds
        .iter()
        .zip(report.upper_bounds.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision bounds report is contradictory",
        });
    }
    if report.constrained_variable_count > report.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR decision bounds constrained count is inconsistent",
        });
    }
    Ok(())
}

fn validate_lgr_segment(segment: &LgrPseudospectralSegment) -> Result<(), TrajoptError> {
    validate_grid(&segment.grid)?;
    if !segment.initial_time_s.is_finite()
        || !segment.final_time_s.is_finite()
        || segment.final_time_s <= segment.initial_time_s
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR segment times must be finite and strictly increasing",
        });
    }
    if segment.state_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR segment requires at least one state",
        });
    }
    let support_state_count = segment.grid.support_count() * segment.state_dimension;
    if segment.support_states.len() != support_state_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR segment support-state dimensions are inconsistent",
        });
    }
    let control_count = segment.grid.collocation_count() * segment.control_dimension;
    if segment.collocation_controls.len() != control_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR segment control dimensions are inconsistent",
        });
    }
    validate_finite(
        "LGR segment support states must be finite",
        &segment.support_states,
    )?;
    validate_finite(
        "LGR segment controls must be finite",
        &segment.collocation_controls,
    )
}

fn append_lgr_path_residuals(
    ocp: &LgrPseudospectralFixedMeshOcp,
    decision: &[f64],
    residuals: &mut Vec<f64>,
) -> Result<usize, TrajoptError> {
    for constraint in &ocp.path_constraints {
        let value = lgr_sample_scalar(
            &ocp.layout,
            decision,
            constraint.segment_index,
            constraint.collocation_index,
            constraint.variable,
        )?;
        let mut violation = 0.0_f64;
        if let Some(lower) = constraint.lower
            && value < lower
        {
            violation = value - lower;
        }
        if let Some(upper) = constraint.upper
            && value > upper
        {
            violation = value - upper;
        }
        residuals.push(constraint.weight.sqrt() * violation);
    }
    Ok(ocp.path_constraints.len())
}

fn append_lgr_quadratic_control_cost_residuals(
    ocp: &LgrPseudospectralFixedMeshOcp,
    decision: &[f64],
    residuals: &mut Vec<f64>,
) -> Result<f64, TrajoptError> {
    let mut cost_sum = 0.0;
    for segment_index in 0..ocp.layout.segment_count() {
        let duration_s =
            ocp.layout.time_nodes_s[segment_index + 1] - ocp.layout.time_nodes_s[segment_index];
        let grid = &ocp.layout.segment_grids[segment_index];
        for collocation_index in 0..grid.collocation_count() {
            let physical_weight_s = 0.5 * duration_s * grid.quadrature_weights[collocation_index];
            for cost in &ocp.quadratic_control_costs {
                let control = lgr_sample_scalar(
                    &ocp.layout,
                    decision,
                    segment_index,
                    collocation_index,
                    LgrPseudospectralScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                residuals.push((cost.weight * physical_weight_s).sqrt() * control);
                cost_sum += cost.weight * physical_weight_s * control * control;
            }
        }
    }
    Ok(cost_sum)
}

fn lgr_quadratic_control_cost(
    ocp: &LgrPseudospectralFixedMeshOcp,
    decision: &[f64],
) -> Result<f64, TrajoptError> {
    let mut cost_sum = 0.0;
    for segment_index in 0..ocp.layout.segment_count() {
        let duration_s =
            ocp.layout.time_nodes_s[segment_index + 1] - ocp.layout.time_nodes_s[segment_index];
        let grid = &ocp.layout.segment_grids[segment_index];
        for collocation_index in 0..grid.collocation_count() {
            let physical_weight_s = 0.5 * duration_s * grid.quadrature_weights[collocation_index];
            for cost in &ocp.quadratic_control_costs {
                let control = lgr_sample_scalar(
                    &ocp.layout,
                    decision,
                    segment_index,
                    collocation_index,
                    LgrPseudospectralScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                cost_sum += cost.weight * physical_weight_s * control * control;
            }
        }
    }
    Ok(cost_sum)
}

#[cfg(feature = "sqp")]
fn lgr_dense_quadratic_control_hessian(
    ocp: &LgrPseudospectralFixedMeshOcp,
) -> Result<Vec<f64>, TrajoptError> {
    validate_lgr_fixed_mesh_ocp(ocp)?;
    if ocp.quadratic_control_costs.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR dense quadratic Hessian requires at least one control cost",
        });
    }
    let variable_count = ocp.layout.variable_count;
    let mut hessian = vec![0.0; variable_count * variable_count];
    for segment_index in 0..ocp.layout.segment_count() {
        let duration_s =
            ocp.layout.time_nodes_s[segment_index + 1] - ocp.layout.time_nodes_s[segment_index];
        let grid = &ocp.layout.segment_grids[segment_index];
        for collocation_index in 0..grid.collocation_count() {
            let physical_weight_s = 0.5 * duration_s * grid.quadrature_weights[collocation_index];
            for cost in &ocp.quadratic_control_costs {
                validate_lgr_quadratic_control_cost(&ocp.layout, cost)?;
                let column = lgr_scalar_decision_index(
                    &ocp.layout,
                    segment_index,
                    collocation_index,
                    LgrPseudospectralScalarVariable::ControlComponent {
                        component: cost.component,
                    },
                )?;
                hessian[column * variable_count + column] += 2.0 * cost.weight * physical_weight_s;
            }
        }
    }
    Ok(hessian)
}

fn lgr_lagrange_quadrature_cost<L>(
    ocp: &LgrPseudospectralFixedMeshOcp,
    decision: &[f64],
    lagrange: &mut L,
) -> Result<f64, TrajoptError>
where
    L: CollocationLagrange,
{
    let mut cost_sum = 0.0;
    for segment_index in 0..ocp.layout.segment_count() {
        let duration_s =
            ocp.layout.time_nodes_s[segment_index + 1] - ocp.layout.time_nodes_s[segment_index];
        let grid = &ocp.layout.segment_grids[segment_index];
        for collocation_index in 0..grid.collocation_count() {
            let physical_weight_s = 0.5 * duration_s * grid.quadrature_weights[collocation_index];
            let (time_s, state, control) =
                lgr_sample_state_control(&ocp.layout, decision, segment_index, collocation_index)?;
            let value = lagrange.eval(time_s, state, control)?;
            if !value.is_finite() {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR objective Lagrange value must be finite",
                });
            }
            cost_sum += physical_weight_s * value;
        }
    }
    Ok(cost_sum)
}

fn lgr_reference_profile_sample_from_state(
    time_s: f64,
    state: &[f64],
    mapping: LgrPseudospectralReferenceProfileMapping,
) -> Result<ReferenceProfileSample, TrajoptError> {
    let sample = ReferenceProfileSample {
        time_s,
        radius_m: state[mapping.radius_component],
        speed_m_s: state[mapping.speed_component],
        flight_path_angle_rad: state[mapping.flight_path_angle_component],
    };
    validate_lgr_reference_profile_sample(&sample)?;
    Ok(sample)
}

fn validate_lgr_reference_profile_sample(
    sample: &ReferenceProfileSample,
) -> Result<(), TrajoptError> {
    if sample.time_s.is_finite()
        && sample.radius_m.is_finite()
        && sample.speed_m_s.is_finite()
        && sample.flight_path_angle_rad.is_finite()
    {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload {
            reason: "LGR reference profile samples must be finite",
        })
    }
}

fn lgr_sample_scalar(
    layout: &LgrPseudospectralDecisionLayout,
    decision: &[f64],
    segment_index: usize,
    collocation_index: usize,
    variable: LgrPseudospectralScalarVariable,
) -> Result<f64, TrajoptError> {
    validate_lgr_decision_vector(layout, decision)?;
    Ok(decision[lgr_scalar_decision_index(layout, segment_index, collocation_index, variable)?])
}

fn lgr_sample_state_control<'a>(
    layout: &LgrPseudospectralDecisionLayout,
    decision: &'a [f64],
    segment_index: usize,
    collocation_index: usize,
) -> Result<(f64, &'a [f64], &'a [f64]), TrajoptError> {
    validate_lgr_decision_vector(layout, decision)?;
    if segment_index >= layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR path sample segment index is out of range",
        });
    }
    let grid = &layout.segment_grids[segment_index];
    if collocation_index >= grid.collocation_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR path sample collocation index is out of range",
        });
    }
    let initial_time_s = layout.time_nodes_s[segment_index];
    let final_time_s = layout.time_nodes_s[segment_index + 1];
    let time_s = 0.5 * (initial_time_s + final_time_s)
        + 0.5 * (final_time_s - initial_time_s) * grid.collocation_nodes[collocation_index];
    let state = layout.segment_support_state(decision, segment_index, collocation_index)?;
    let control = layout.segment_collocation_control(decision, segment_index, collocation_index)?;
    Ok((time_s, state, control))
}

fn lgr_scalar_decision_index(
    layout: &LgrPseudospectralDecisionLayout,
    segment_index: usize,
    collocation_index: usize,
    variable: LgrPseudospectralScalarVariable,
) -> Result<usize, TrajoptError> {
    if segment_index >= layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR scalar sample segment index is out of range",
        });
    }
    if collocation_index >= layout.segment_grids[segment_index].collocation_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR scalar sample collocation index is out of range",
        });
    }
    match variable {
        LgrPseudospectralScalarVariable::StateComponent { component } => {
            if component >= layout.state_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR scalar state sample is out of range",
                });
            }
            Ok(layout.segment_state_offsets[segment_index]
                + collocation_index * layout.state_dimension
                + component)
        }
        LgrPseudospectralScalarVariable::ControlComponent { component } => {
            if component >= layout.control_dimension {
                return Err(TrajoptError::InvalidPayload {
                    reason: "LGR scalar control sample is out of range",
                });
            }
            Ok(layout.segment_control_offsets[segment_index]
                + collocation_index * layout.control_dimension
                + component)
        }
    }
}

#[cfg(feature = "sqp")]
fn lgr_mesh_node_decision_index(
    layout: &LgrPseudospectralDecisionLayout,
    mesh_node_index: usize,
    component: usize,
) -> Result<usize, TrajoptError> {
    if mesh_node_index > layout.segment_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-node decision index is out of range",
        });
    }
    if component >= layout.state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-node decision component is out of range",
        });
    }
    if mesh_node_index == layout.segment_count() {
        let segment_index = layout.segment_count() - 1;
        let terminal_support = layout.segment_grids[segment_index].support_count() - 1;
        Ok(layout.segment_state_offsets[segment_index]
            + terminal_support * layout.state_dimension
            + component)
    } else {
        Ok(layout.segment_state_offsets[mesh_node_index] + component)
    }
}

fn lgr_slice(values: &[f64], offset: usize, len: usize) -> &[f64] {
    &values[offset..offset + len]
}

fn validate_lgr_transcription(
    transcription: &LgrPseudospectralTranscription,
) -> Result<(), TrajoptError> {
    if transcription.segments.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR transcription requires at least one segment",
        });
    }

    let first_segment = &transcription.segments[0];
    validate_lgr_segment(first_segment)?;
    let state_dimension = first_segment.state_dimension;
    let control_dimension = first_segment.control_dimension;
    let mut previous_final_time_s = first_segment.final_time_s;

    for segment in transcription.segments.iter().skip(1) {
        validate_lgr_segment(segment)?;
        if segment.state_dimension != state_dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR transcription segment state dimensions must match",
            });
        }
        if segment.control_dimension != control_dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR transcription segment control dimensions must match",
            });
        }
        let tolerance_s = adjacent_time_tolerance(previous_final_time_s);
        if (segment.initial_time_s - previous_final_time_s).abs() > tolerance_s {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR transcription segment times must be contiguous",
            });
        }
        previous_final_time_s = segment.final_time_s;
    }

    Ok(())
}

fn validate_lgr_mesh_refinement_settings(
    settings: &LgrPseudospectralMeshRefinementSettings,
) -> Result<(), TrajoptError> {
    if !settings.relative_error_tolerance.is_finite()
        || settings.relative_error_tolerance <= 0.0
        || settings.max_collocation_count == 0
        || settings.degree_raise_step == 0
        || settings.subdivision_factor < 2
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement settings are invalid",
        });
    }
    Ok(())
}

fn validate_lgr_refinement_segment_pair(
    candidate: &LgrPseudospectralSegment,
    reference: &LgrPseudospectralSegment,
) -> Result<(), TrajoptError> {
    if candidate.state_dimension != reference.state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement state dimensions must match",
        });
    }
    if candidate.control_dimension != reference.control_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement control dimensions must match",
        });
    }
    if !same_time(candidate.initial_time_s, reference.initial_time_s)
        || !same_time(candidate.final_time_s, reference.final_time_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement segment intervals must match",
        });
    }
    if reference.grid.collocation_count() < candidate.grid.collocation_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement reference degree must not be lower than candidate degree",
        });
    }
    Ok(())
}

fn validate_lgr_refinement_report_for_source(
    source: &LgrPseudospectralTranscription,
    report: &LgrPseudospectralMeshRefinementReport,
) -> Result<(), TrajoptError> {
    if report.estimates.len() != source.segments.len()
        || report.recommendations.len() != source.segments.len()
        || !report.max_relative_error.is_finite()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement report dimensions are inconsistent",
        });
    }

    let mut max_relative_error = 0.0_f64;
    for (segment_index, (estimate, segment)) in report
        .estimates
        .iter()
        .zip(source.segments.iter())
        .enumerate()
    {
        if estimate.segment_index != segment_index
            || estimate.collocation_count != segment.grid.collocation_count()
            || estimate.reference_collocation_count < estimate.collocation_count
            || !estimate.max_abs_state_error.is_finite()
            || !estimate.state_scale.is_finite()
            || estimate.state_scale < 1.0
            || !estimate.relative_error.is_finite()
            || estimate.relative_error < 0.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR mesh-refinement estimate is inconsistent",
            });
        }
        max_relative_error = max_relative_error.max(estimate.relative_error);
    }

    let mut refined_segment_count = 0_usize;
    for (segment_index, (recommendation, segment)) in report
        .recommendations
        .iter()
        .zip(source.segments.iter())
        .enumerate()
    {
        if recommendation.segment_index != segment_index
            || recommendation.collocation_count != segment.grid.collocation_count()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR mesh-refinement recommendation is inconsistent",
            });
        }
        match recommendation.action {
            LgrPseudospectralMeshRefinementAction::Accept => {}
            LgrPseudospectralMeshRefinementAction::RaiseDegree { collocation_count } => {
                if collocation_count <= segment.grid.collocation_count() {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "LGR mesh-refinement degree raise must increase degree",
                    });
                }
                refined_segment_count += 1;
            }
            LgrPseudospectralMeshRefinementAction::Split {
                segment_count,
                collocation_count,
            } => {
                if segment_count < 2 || collocation_count == 0 {
                    return Err(TrajoptError::InvalidPayload {
                        reason: "LGR mesh-refinement split recommendation is invalid",
                    });
                }
                refined_segment_count += 1;
            }
        }
    }

    if report.refined_segment_count != refined_segment_count
        || report.converged != (refined_segment_count == 0)
        || (report.max_relative_error - max_relative_error).abs()
            > 1.0e-12 * max_relative_error.abs().max(1.0)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement report summary is inconsistent",
        });
    }

    Ok(())
}

fn seed_lgr_refined_segment(
    source: &LgrPseudospectralSegment,
    initial_time_s: f64,
    final_time_s: f64,
    collocation_count: usize,
) -> Result<LgrPseudospectralSegment, TrajoptError> {
    if !initial_time_s.is_finite()
        || !final_time_s.is_finite()
        || final_time_s <= initial_time_s
        || !time_in_segment(source, initial_time_s)
        || !time_in_segment(source, final_time_s)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR refined segment times are invalid",
        });
    }

    let grid = legendre_gauss_radau_grid(collocation_count)?;
    let source_state_weights = barycentric_weights(&source.grid.support_nodes)?;
    let source_control_weights = if source.control_dimension > 0 {
        barycentric_weights(&source.grid.collocation_nodes)?
    } else {
        Vec::new()
    };

    let mut support_states = Vec::with_capacity(grid.support_count() * source.state_dimension);
    for support_node in &grid.support_nodes {
        let time_s = physical_time(initial_time_s, final_time_s, *support_node);
        let source_node = normalized_time(source, time_s);
        for state_index in 0..source.state_dimension {
            support_states.push(interpolate_lgr_support_state(
                source,
                &source_state_weights,
                source_node,
                state_index,
            )?);
        }
    }

    let mut collocation_controls =
        Vec::with_capacity(grid.collocation_count() * source.control_dimension);
    for collocation_node in &grid.collocation_nodes {
        let time_s = physical_time(initial_time_s, final_time_s, *collocation_node);
        let source_node = normalized_time(source, time_s);
        for control_index in 0..source.control_dimension {
            collocation_controls.push(interpolate_lgr_collocation_control(
                source,
                &source_control_weights,
                source_node,
                control_index,
            )?);
        }
    }

    let segment = LgrPseudospectralSegment {
        initial_time_s,
        final_time_s,
        grid,
        state_dimension: source.state_dimension,
        control_dimension: source.control_dimension,
        support_states,
        collocation_controls,
    };
    validate_lgr_segment(&segment)?;
    Ok(segment)
}

fn estimate_lgr_segment_state_error(
    segment_index: usize,
    candidate: &LgrPseudospectralSegment,
    reference: &LgrPseudospectralSegment,
) -> Result<LgrPseudospectralSegmentErrorEstimate, TrajoptError> {
    let reference_barycentric_weights = barycentric_weights(&reference.grid.support_nodes)?;
    let mut max_abs_state_error = 0.0_f64;
    let mut max_abs_state = 0.0_f64;

    for (support_index, node) in candidate.grid.support_nodes.iter().enumerate() {
        for state_index in 0..candidate.state_dimension {
            let candidate_state =
                candidate.support_states[support_index * candidate.state_dimension + state_index];
            let reference_state = interpolate_lgr_support_state(
                reference,
                &reference_barycentric_weights,
                *node,
                state_index,
            )?;
            max_abs_state_error =
                max_abs_state_error.max((candidate_state - reference_state).abs());
            max_abs_state = max_abs_state
                .max(candidate_state.abs())
                .max(reference_state.abs());
        }
    }

    let state_scale = 1.0 + max_abs_state;
    let relative_error = max_abs_state_error / state_scale;
    if !relative_error.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement relative error must be finite",
        });
    }

    Ok(LgrPseudospectralSegmentErrorEstimate {
        segment_index,
        collocation_count: candidate.grid.collocation_count(),
        reference_collocation_count: reference.grid.collocation_count(),
        max_abs_state_error,
        state_scale,
        relative_error,
    })
}

fn interpolate_lgr_support_state(
    segment: &LgrPseudospectralSegment,
    barycentric_weights: &[f64],
    normalized_node: f64,
    state_index: usize,
) -> Result<f64, TrajoptError> {
    for (support_index, support_node) in segment.grid.support_nodes.iter().enumerate() {
        if same_node(normalized_node, *support_node) {
            return Ok(
                segment.support_states[support_index * segment.state_dimension + state_index]
            );
        }
    }

    let mut numerator = 0.0;
    let mut denominator = 0.0;
    for (support_index, support_node) in segment.grid.support_nodes.iter().enumerate() {
        let scaled_weight = barycentric_weights[support_index] / (normalized_node - support_node);
        numerator += scaled_weight
            * segment.support_states[support_index * segment.state_dimension + state_index];
        denominator += scaled_weight;
    }
    if !numerator.is_finite() || !denominator.is_finite() || denominator == 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement interpolation failed",
        });
    }
    Ok(numerator / denominator)
}

fn interpolate_lgr_collocation_control(
    segment: &LgrPseudospectralSegment,
    barycentric_weights: &[f64],
    normalized_node: f64,
    control_index: usize,
) -> Result<f64, TrajoptError> {
    for (collocation_index, collocation_node) in segment.grid.collocation_nodes.iter().enumerate() {
        if same_node(normalized_node, *collocation_node) {
            return Ok(segment.collocation_controls
                [collocation_index * segment.control_dimension + control_index]);
        }
    }

    let mut numerator = 0.0;
    let mut denominator = 0.0;
    for (collocation_index, collocation_node) in segment.grid.collocation_nodes.iter().enumerate() {
        let scaled_weight =
            barycentric_weights[collocation_index] / (normalized_node - collocation_node);
        numerator += scaled_weight
            * segment.collocation_controls
                [collocation_index * segment.control_dimension + control_index];
        denominator += scaled_weight;
    }
    if !numerator.is_finite() || !denominator.is_finite() || denominator == 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR mesh-refinement control interpolation failed",
        });
    }
    Ok(numerator / denominator)
}

fn recommend_lgr_segment_refinement(
    estimate: &LgrPseudospectralSegmentErrorEstimate,
    settings: &LgrPseudospectralMeshRefinementSettings,
) -> LgrPseudospectralMeshRefinementAction {
    if estimate.relative_error <= settings.relative_error_tolerance {
        return LgrPseudospectralMeshRefinementAction::Accept;
    }

    let raised_collocation_count = estimate
        .collocation_count
        .saturating_add(settings.degree_raise_step);
    if raised_collocation_count <= settings.max_collocation_count {
        return LgrPseudospectralMeshRefinementAction::RaiseDegree {
            collocation_count: raised_collocation_count,
        };
    }

    LgrPseudospectralMeshRefinementAction::Split {
        segment_count: settings.subdivision_factor,
        collocation_count: estimate
            .collocation_count
            .min(settings.max_collocation_count),
    }
}

fn adjacent_time_tolerance(time_s: f64) -> f64 {
    1.0e-12 * time_s.abs().max(1.0)
}

fn same_time(left: f64, right: f64) -> bool {
    (left - right).abs() <= adjacent_time_tolerance(left.abs().max(right.abs()))
}

fn same_node(left: f64, right: f64) -> bool {
    (left - right).abs() <= 1.0e-12
}

fn physical_time(initial_time_s: f64, final_time_s: f64, normalized_node: f64) -> f64 {
    let midpoint_time_s = 0.5 * (initial_time_s + final_time_s);
    let half_duration_s = 0.5 * (final_time_s - initial_time_s);
    midpoint_time_s + half_duration_s * normalized_node
}

fn normalized_time(segment: &LgrPseudospectralSegment, time_s: f64) -> f64 {
    (time_s - 0.5 * (segment.initial_time_s + segment.final_time_s)) / segment.half_duration_s()
}

fn time_in_segment(segment: &LgrPseudospectralSegment, time_s: f64) -> bool {
    let tolerance_s =
        adjacent_time_tolerance(segment.initial_time_s.abs().max(segment.final_time_s.abs()));
    time_s >= segment.initial_time_s - tolerance_s && time_s <= segment.final_time_s + tolerance_s
}

fn validate_lgr_dynamics_output(
    derivative: &[f64],
    state_dimension: usize,
) -> Result<(), TrajoptError> {
    if derivative.len() != state_dimension {
        return Err(TrajoptError::InvalidPayload {
            reason: "LGR segment dynamics output dimension is inconsistent",
        });
    }
    validate_finite("LGR segment dynamics output must be finite", derivative)
}

fn norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}

fn max_abs(values: &[f64]) -> f64 {
    values
        .iter()
        .map(|value| value.abs())
        .fold(0.0_f64, f64::max)
}

fn lgr_dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right.iter())
        .map(|(left_value, right_value)| left_value * right_value)
        .sum()
}

fn validate_finite(reason: &'static str, values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic)]
mod tests {
    use serde::Deserialize;
    use std::io::{Error, ErrorKind};

    use super::*;

    const LGR_SPECTRAL_CONVERGENCE_TOLERANCE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-two-body-apogee/lgr-spectral-convergence-tolerance.toml"
    ));
    const BRACHISTOCHRONE_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-brachistochrone/brachistochrone-cycloid-benchmark.toml"
    ));
    const GODDARD_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-goddard/goddard-cops-benchmark.toml"
    ));
    #[cfg(feature = "sqp")]
    const LGR_SCALAR_INTEGRATOR_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-two-body-apogee/lgr-scalar-integrator-benchmark.toml"
    ));
    #[cfg(feature = "sqp")]
    const BRYSON_DENHAM_BENCHMARK: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/trajopt-bryson-denham/bryson-denham-state-constrained-benchmark.toml"
    ));

    #[derive(Debug, Deserialize)]
    struct LgrSpectralConvergenceToleranceTable {
        case: String,
        validation: String,
        collocation_counts: Vec<usize>,
        dense_sample_count: usize,
        baseline_order: f64,
        minimum_error_ratio: f64,
        max_finest_interpolation_error: f64,
    }

    impl LgrSpectralConvergenceToleranceTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(LGR_SPECTRAL_CONVERGENCE_TOLERANCE)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "lgr-exp-spectral-convergence" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected LGR spectral-convergence case",
                ));
            }
            if self.validation != "validated-toy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR spectral convergence validation must be validated-toy",
                ));
            }
            if self.collocation_counts.len() < 3
                || !self
                    .collocation_counts
                    .windows(2)
                    .all(|window| window[0] >= 2 && window[1] > window[0])
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR spectral convergence degrees must strictly increase",
                ));
            }
            if self.dense_sample_count < 17
                || !self.baseline_order.is_finite()
                || self.baseline_order <= 0.0
                || !self.minimum_error_ratio.is_finite()
                || self.minimum_error_ratio <= 1.0
                || !self.max_finest_interpolation_error.is_finite()
                || self.max_finest_interpolation_error <= 0.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR spectral convergence tolerances are invalid",
                ));
            }
            Ok(())
        }
    }

    #[derive(Debug, Deserialize)]
    struct BrachistochroneBenchmarkTable {
        case: String,
        source: String,
        validation: String,
        problem: BrachistochroneBenchmarkProblem,
    }

    #[derive(Clone, Copy, Debug, Deserialize)]
    struct BrachistochroneBenchmarkProblem {
        endpoint_x_m: f64,
        endpoint_y_down_m: f64,
        gravity_m_s2: f64,
        theta_final_rad: f64,
        cycloid_scale_m: f64,
        optimal_time_s: f64,
        segment_count: usize,
        constant_tolerance: f64,
        max_terminal_error_m: f64,
        max_residual_norm: f64,
        relative_time_tolerance: f64,
        dense_sqp_max_residual_norm: f64,
        dense_sqp_max_solution_delta: f64,
        free_time_dense_sqp_max_residual_norm: f64,
        free_time_dense_sqp_relative_time_tolerance: f64,
    }

    impl BrachistochroneBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(BRACHISTOCHRONE_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "brachistochrone-cycloid-closed-form" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected brachistochrone benchmark case",
                ));
            }
            if !self.source.contains("Public closed-form brachistochrone") {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark source must describe the public closed form",
                ));
            }
            if self.validation != "research" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark validation must be research",
                ));
            }
            self.problem.require_valid()
        }
    }

    impl BrachistochroneBenchmarkProblem {
        fn require_valid(self) -> Result<(), Error> {
            let positive_values = [
                self.endpoint_x_m,
                self.endpoint_y_down_m,
                self.gravity_m_s2,
                self.theta_final_rad,
                self.cycloid_scale_m,
                self.optimal_time_s,
                self.constant_tolerance,
                self.max_terminal_error_m,
                self.max_residual_norm,
                self.relative_time_tolerance,
                self.dense_sqp_max_residual_norm,
                self.dense_sqp_max_solution_delta,
                self.free_time_dense_sqp_max_residual_norm,
                self.free_time_dense_sqp_relative_time_tolerance,
            ];
            if !positive_values
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
                || self.segment_count == 0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark values must be finite and positive",
                ));
            }
            let endpoint_x =
                self.cycloid_scale_m * (self.theta_final_rad - self.theta_final_rad.sin());
            let endpoint_y = self.cycloid_scale_m * (1.0 - self.theta_final_rad.cos());
            let optimal_time =
                (self.cycloid_scale_m / self.gravity_m_s2).sqrt() * self.theta_final_rad;
            if (endpoint_x - self.endpoint_x_m).abs() > self.constant_tolerance
                || (endpoint_y - self.endpoint_y_down_m).abs() > self.constant_tolerance
                || (optimal_time - self.optimal_time_s).abs() > self.constant_tolerance
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "brachistochrone benchmark constants are inconsistent",
                ));
            }
            Ok(())
        }
    }

    #[derive(Debug, Deserialize)]
    struct GoddardBenchmarkTable {
        case: String,
        source: String,
        singular_source: String,
        validation: String,
        problem: GoddardBenchmarkProblem,
    }

    #[derive(Clone, Copy, Debug, Deserialize)]
    struct GoddardBenchmarkProblem {
        initial_altitude: f64,
        initial_velocity: f64,
        initial_mass: f64,
        minimum_mass: f64,
        maximum_thrust: f64,
        exhaust_velocity: f64,
        drag_coefficient: f64,
        drag_altitude_scale: f64,
        initial_gravity: f64,
        sample_final_time: f64,
        published_final_time: f64,
        published_final_time_tolerance: f64,
        published_final_altitude: f64,
        published_final_altitude_tolerance: f64,
        published_max_constraint_violation: f64,
        segment_count: usize,
        lgr_collocation_count: usize,
        constant_tolerance: f64,
        max_boundary_residual_norm: f64,
        min_seed_dynamics_residual_norm: f64,
        max_seed_dynamics_residual_norm: f64,
        max_singular_arc_residual: f64,
        terminal_objective_min_altitude_gain: f64,
        terminal_objective_max_equality_residual_norm: f64,
    }

    impl GoddardBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(GODDARD_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "goddard-cops-max-altitude" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected Goddard benchmark case",
                ));
            }
            if !self
                .source
                .contains("COPS/Argonne Goddard Rocket benchmark")
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Goddard source must describe the COPS benchmark",
                ));
            }
            if !self
                .singular_source
                .contains("YAPSS Goddard Rocket example")
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Goddard singular source must describe the YAPSS example",
                ));
            }
            if self.validation != "research" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Goddard benchmark validation must be research",
                ));
            }
            self.problem.require_valid()
        }
    }

    impl GoddardBenchmarkProblem {
        fn require_valid(self) -> Result<(), Error> {
            let positive_values = [
                self.initial_altitude,
                self.initial_mass,
                self.minimum_mass,
                self.maximum_thrust,
                self.exhaust_velocity,
                self.drag_coefficient,
                self.drag_altitude_scale,
                self.initial_gravity,
                self.sample_final_time,
                self.published_final_time,
                self.published_final_time_tolerance,
                self.published_final_altitude,
                self.published_final_altitude_tolerance,
                self.published_max_constraint_violation,
                self.constant_tolerance,
                self.max_boundary_residual_norm,
                self.min_seed_dynamics_residual_norm,
                self.max_seed_dynamics_residual_norm,
                self.max_singular_arc_residual,
                self.terminal_objective_min_altitude_gain,
                self.terminal_objective_max_equality_residual_norm,
            ];
            if !positive_values
                .iter()
                .all(|value| value.is_finite() && *value > 0.0)
                || !self.initial_velocity.is_finite()
                || self.initial_velocity != 0.0
                || self.minimum_mass >= self.initial_mass
                || self.segment_count == 0
                || self.lgr_collocation_count < 2
                || self.min_seed_dynamics_residual_norm >= self.max_seed_dynamics_residual_norm
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Goddard benchmark values are invalid",
                ));
            }
            let expected_drag = 0.5 * 620.0 * self.initial_mass / self.initial_gravity;
            if (self.initial_altitude - 1.0).abs() > self.constant_tolerance
                || (self.initial_mass - 1.0).abs() > self.constant_tolerance
                || (self.minimum_mass - 0.6).abs() > self.constant_tolerance
                || (self.maximum_thrust - 3.5).abs() > self.constant_tolerance
                || (self.exhaust_velocity - 0.5).abs() > self.constant_tolerance
                || (self.drag_altitude_scale - 500.0).abs() > self.constant_tolerance
                || (self.initial_gravity - 1.0).abs() > self.constant_tolerance
                || (self.published_final_time - 0.073).abs() > self.published_final_time_tolerance
                || (self.published_final_altitude - 1.01283).abs()
                    > self.published_final_altitude_tolerance
                || (self.drag_coefficient - expected_drag).abs() > self.constant_tolerance
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "Goddard COPS constants are inconsistent",
                ));
            }
            Ok(())
        }
    }

    struct GoddardResidualLocalizedSplitSeed {
        application: LgrPseudospectralMeshRefinementApplication,
        peak_segment_index: usize,
        peak_segment_norm: f64,
        differential_norm: f64,
        recomposed_norm: f64,
    }

    #[cfg(feature = "sqp")]
    #[derive(Clone, Copy, Debug, PartialEq)]
    struct GoddardPublishedFinalAltitudeGapReport {
        candidate_final_time: f64,
        published_final_time: f64,
        absolute_final_time_error: f64,
        candidate_final_altitude: f64,
        published_final_altitude: f64,
        absolute_final_altitude_error: f64,
        equality_residual_norm: f64,
        within_published_time_band: bool,
        within_published_altitude_band: bool,
        within_published_residual_band: bool,
        published_closure_met: bool,
    }

    #[cfg(feature = "sqp")]
    #[derive(Debug, Deserialize)]
    struct LgrScalarIntegratorBenchmarkTable {
        case: String,
        source: String,
        validation: String,
        segment_count: usize,
        collocation_count: usize,
        expected_cost: f64,
        cost_tolerance: f64,
        max_equality_residual_norm: f64,
        max_state_error: f64,
        max_control_error: f64,
    }

    #[cfg(feature = "sqp")]
    impl LgrScalarIntegratorBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(LGR_SCALAR_INTEGRATOR_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "lgr-scalar-integrator-min-energy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected LGR scalar-integrator benchmark case",
                ));
            }
            if !self
                .source
                .contains("Synthetic scalar minimum-energy integrator OCP")
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR scalar-integrator benchmark source must describe the synthetic OCP",
                ));
            }
            if self.validation != "validated-toy" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR scalar-integrator benchmark validation must be validated-toy",
                ));
            }
            if self.segment_count == 0 || self.collocation_count < 2 {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR scalar-integrator benchmark mesh is invalid",
                ));
            }
            if !self.expected_cost.is_finite()
                || !self.cost_tolerance.is_finite()
                || self.cost_tolerance <= 0.0
                || !self.max_equality_residual_norm.is_finite()
                || self.max_equality_residual_norm <= 0.0
                || !self.max_state_error.is_finite()
                || self.max_state_error <= 0.0
                || !self.max_control_error.is_finite()
                || self.max_control_error <= 0.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR scalar-integrator benchmark tolerances must be finite and positive",
                ));
            }
            Ok(())
        }
    }

    #[cfg(feature = "sqp")]
    #[derive(Debug, Deserialize)]
    struct LgrBrysonDenhamBenchmarkTable {
        case: String,
        source: String,
        validation: String,
        problem: LgrBrysonDenhamBenchmarkProblem,
    }

    #[cfg(feature = "sqp")]
    #[derive(Clone, Copy, Debug, Deserialize)]
    struct LgrBrysonDenhamBenchmarkProblem {
        segment_count: usize,
        lgr_collocation_count: usize,
        state_bound: f64,
        entry_time_s: f64,
        exit_time_s: f64,
        expected_cost: f64,
        cost_tolerance: f64,
        max_residual_norm: f64,
        max_path_violation: f64,
        max_state_error: f64,
        max_control_error: f64,
    }

    #[cfg(feature = "sqp")]
    impl LgrBrysonDenhamBenchmarkTable {
        fn parse() -> Result<Self, Box<dyn std::error::Error>> {
            let table: Self = toml::from_str(BRYSON_DENHAM_BENCHMARK)?;
            table.require_valid()?;
            Ok(table)
        }

        fn require_valid(&self) -> Result<(), Error> {
            if self.case != "bryson-denham-state-constrained" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "unexpected LGR Bryson-Denham benchmark case",
                ));
            }
            if !self.source.contains("Public Bryson-Denham") {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR Bryson-Denham source must describe the public benchmark",
                ));
            }
            if self.validation != "research" {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR Bryson-Denham validation must be research",
                ));
            }
            self.problem.require_valid()
        }
    }

    #[cfg(feature = "sqp")]
    impl LgrBrysonDenhamBenchmarkProblem {
        fn require_valid(self) -> Result<(), Error> {
            let positive_values = [
                self.state_bound,
                self.entry_time_s,
                self.exit_time_s,
                self.expected_cost,
                self.cost_tolerance,
                self.max_residual_norm,
                self.max_path_violation,
                self.max_state_error,
                self.max_control_error,
            ];
            if self.segment_count == 0
                || self.lgr_collocation_count < 2
                || !positive_values
                    .iter()
                    .all(|value| value.is_finite() && *value > 0.0)
                || self.entry_time_s >= self.exit_time_s
                || self.exit_time_s >= 1.0
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR Bryson-Denham benchmark values are invalid",
                ));
            }
            if (self.state_bound - 1.0 / 9.0).abs() > self.max_state_error
                || (self.entry_time_s - 1.0 / 3.0).abs() > self.max_state_error
                || (self.exit_time_s - 2.0 / 3.0).abs() > self.max_state_error
            {
                return Err(Error::new(
                    ErrorKind::InvalidData,
                    "LGR Bryson-Denham benchmark constants are inconsistent",
                ));
            }
            Ok(())
        }
    }

    #[test]
    fn lgr_grid_matches_three_point_closed_form_reference() {
        let grid = legendre_gauss_radau_grid(3).expect("LGR grid builds");
        let sqrt_6 = 6.0_f64.sqrt();
        let expected_nodes = vec![-1.0, (1.0 - sqrt_6) / 5.0, (1.0 + sqrt_6) / 5.0];
        let expected_weights = vec![
            0.222_222_222_222_222_2,
            1.024_971_652_376_843_1,
            0.752_806_125_400_935_6,
        ];

        for (actual, expected) in grid.collocation_nodes.iter().zip(expected_nodes.iter()) {
            assert!(
                (actual - expected).abs() < 1.0e-14,
                "node {actual:e} != {expected:e}"
            );
        }
        for (actual, expected) in grid.quadrature_weights.iter().zip(expected_weights.iter()) {
            assert!(
                (actual - expected).abs() < 1.0e-14,
                "weight {actual:e} != {expected:e}"
            );
        }
        assert_eq!(grid.endpoint, 1.0);
        let expected_support = vec![-1.0, expected_nodes[1], expected_nodes[2], 1.0];
        for (actual, expected) in grid.support_nodes.iter().zip(expected_support.iter()) {
            assert!(
                (actual - expected).abs() < 1.0e-14,
                "support node {actual:e} != {expected:e}"
            );
        }
    }

    #[test]
    fn lgr_quadrature_is_exact_through_radau_degree() {
        let collocation_count = 5;
        let grid = legendre_gauss_radau_grid(collocation_count).expect("LGR grid builds");
        for degree in 0..=(2 * collocation_count - 2) {
            let values = grid
                .collocation_nodes
                .iter()
                .map(|node| node.powi(degree as i32))
                .collect::<Vec<_>>();
            let actual = grid
                .integrate_collocation_values(&values)
                .expect("quadrature integrates");
            let expected = if degree % 2 == 0 {
                2.0 / ((degree as f64) + 1.0)
            } else {
                0.0
            };
            assert!(
                (actual - expected).abs() < 1.0e-12,
                "degree {degree}: {actual:e} != {expected:e}"
            );
        }
    }

    #[test]
    fn lgr_differentiation_is_exact_for_support_polynomial() {
        let grid = legendre_gauss_radau_grid(4).expect("LGR grid builds");
        let values = grid
            .support_nodes
            .iter()
            .map(|node| 2.0 * node.powi(4) - 3.0 * node.powi(2) + node - 5.0)
            .collect::<Vec<_>>();
        let derivatives = grid
            .differentiate_support_values(&values)
            .expect("differentiation evaluates");

        for (node, derivative) in grid.collocation_nodes.iter().zip(derivatives.iter()) {
            let expected = 8.0 * node.powi(3) - 6.0 * node + 1.0;
            assert!(
                (derivative - expected).abs() < 1.0e-11,
                "node {node:e}: derivative {derivative:e} != {expected:e}"
            );
        }
    }

    #[test]
    fn lgr_interpolation_error_is_spectral_on_smooth_arc() {
        let table =
            LgrSpectralConvergenceToleranceTable::parse().expect("LGR spectral tolerance parses");
        let mut interpolation_errors = Vec::with_capacity(table.collocation_counts.len());

        for collocation_count in &table.collocation_counts {
            interpolation_errors.push(max_exp_interpolation_error(
                *collocation_count,
                table.dense_sample_count,
            ));
        }

        for ((degree_window, error_window), error_index) in table
            .collocation_counts
            .windows(2)
            .zip(interpolation_errors.windows(2))
            .zip(1..)
        {
            let ratio = error_window[0] / error_window[1];
            let baseline_ratio =
                ((degree_window[1] as f64) / (degree_window[0] as f64)).powf(table.baseline_order);
            let required_ratio = table.minimum_error_ratio.max(baseline_ratio);
            assert!(
                ratio >= required_ratio,
                "error ratio {ratio:e} at window {error_index} did not meet {required_ratio:e}; errors = {interpolation_errors:?}"
            );
        }

        let finest_error = *interpolation_errors
            .last()
            .expect("spectral fixture has samples");
        assert!(
            finest_error <= table.max_finest_interpolation_error,
            "finest LGR interpolation error {finest_error:e} exceeded {}",
            table.max_finest_interpolation_error
        );
    }

    #[test]
    fn lgr_brachistochrone_closed_form_benchmark_matches_cycloid() {
        let table = BrachistochroneBenchmarkTable::parse().expect("benchmark table parses");
        let problem = table.problem;
        let transcription = lgr_brachistochrone_transcription(problem, 4)
            .expect("LGR brachistochrone transcription builds");
        let mut residual_dynamics = move |_time_s: f64, state: &[f64], control: &[f64]| {
            brachistochrone_dynamics(problem.gravity_m_s2, state, control)
        };

        let report = evaluate_lgr_pseudospectral_transcription(
            &transcription,
            &mut residual_dynamics,
            unit_lagrange,
        )
        .expect("LGR brachistochrone transcription evaluates");

        assert!(
            report.residual_norm <= problem.max_residual_norm,
            "brachistochrone LGR residual norm {:e} exceeded {}",
            report.residual_norm,
            problem.max_residual_norm
        );
        let terminal = &transcription.segments[transcription.segments.len() - 1];
        let terminal_offset = (terminal.grid.support_count() - 1) * terminal.state_dimension;
        let terminal_x = terminal.support_states[terminal_offset];
        let terminal_y = terminal.support_states[terminal_offset + 1];
        assert!(
            (terminal_x - problem.endpoint_x_m).abs() <= problem.max_terminal_error_m,
            "brachistochrone LGR terminal x {terminal_x:e} expected {:e}",
            problem.endpoint_x_m
        );
        assert!(
            (terminal_y - problem.endpoint_y_down_m).abs() <= problem.max_terminal_error_m,
            "brachistochrone LGR terminal y {terminal_y:e} expected {:e}",
            problem.endpoint_y_down_m
        );
        let relative_time_error =
            ((report.quadrature_cost - problem.optimal_time_s) / problem.optimal_time_s).abs();
        assert!(
            relative_time_error <= problem.relative_time_tolerance,
            "brachistochrone LGR time relative error {relative_time_error:e} exceeded {}",
            problem.relative_time_tolerance
        );
    }

    #[test]
    fn lgr_goddard_cops_benchmark_pins_dynamics_and_singular_arc()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = GoddardBenchmarkTable::parse()?;
        let problem = table.problem;

        let initial_derivative = goddard_dynamics(
            problem,
            &[
                problem.initial_altitude,
                problem.initial_velocity,
                problem.initial_mass,
            ],
            &[problem.maximum_thrust],
        )?;
        assert_eq!(initial_derivative[0], 0.0);
        assert!(
            (initial_derivative[1] - (problem.maximum_thrust - problem.initial_gravity)).abs()
                <= problem.constant_tolerance
        );
        assert!(
            (initial_derivative[2] + problem.maximum_thrust / problem.exhaust_velocity).abs()
                <= problem.constant_tolerance
        );

        let singular_state = goddard_singular_arc_sample(problem)?;
        let residual = goddard_singular_arc_residual(problem, &singular_state)?;
        assert!(
            residual.abs() <= problem.max_singular_arc_residual,
            "Goddard singular-arc residual {residual:e} exceeded {}",
            problem.max_singular_arc_residual
        );
        Ok(())
    }

    #[test]
    fn lgr_goddard_cops_fixed_mesh_seed_reports_dynamics_defect()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = GoddardBenchmarkTable::parse()?;
        let problem = table.problem;
        let ocp = lgr_goddard_ocp(problem)?;
        let (support_states, collocation_controls) = lgr_goddard_seed_decision_blocks(problem)?;
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let report = ocp.residual_report(
            &decision,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
        )?;

        let boundary_start = report.differential_residual_count + report.continuity_residual_count;
        let boundary_end = boundary_start + report.boundary_residual_count;
        let boundary_norm = norm(&report.residuals[boundary_start..boundary_end]);
        assert!(
            boundary_norm <= problem.max_boundary_residual_norm,
            "Goddard fixed-mesh seed boundary norm {boundary_norm:e} exceeded {}",
            problem.max_boundary_residual_norm
        );
        let dynamics_norm = norm(&report.residuals[..report.differential_residual_count]);
        assert!(
            dynamics_norm >= problem.min_seed_dynamics_residual_norm
                && dynamics_norm <= problem.max_seed_dynamics_residual_norm,
            "Goddard fixed-mesh seed dynamics norm {dynamics_norm:e} outside [{}, {}]",
            problem.min_seed_dynamics_residual_norm,
            problem.max_seed_dynamics_residual_norm
        );
        assert_eq!(
            report.differential_residual_count,
            problem.segment_count * problem.lgr_collocation_count * 3
        );
        assert_eq!(report.boundary_residual_count, 4);
        Ok(())
    }

    #[test]
    fn lgr_goddard_seed_residual_localizes_split_refinement()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = GoddardBenchmarkTable::parse()?;
        let problem = table.problem;
        let split_seed = lgr_goddard_residual_localized_split_seed(problem)?;

        assert!(
            (split_seed.recomposed_norm - split_seed.differential_norm).abs() < 1.0e-12,
            "Goddard segment residual norms should recompose the differential norm"
        );
        assert!(
            split_seed.peak_segment_norm
                >= problem.min_seed_dynamics_residual_norm / (problem.segment_count as f64).sqrt(),
            "Goddard peak segment residual {:e} did not exceed the seed floor",
            split_seed.peak_segment_norm
        );
        let application = &split_seed.application;

        assert_eq!(
            application.transcription.segments.len(),
            problem.segment_count + 1
        );
        let peak_mappings = application
            .segment_mappings
            .iter()
            .filter(|mapping| mapping.source_segment_index == split_seed.peak_segment_index)
            .collect::<Vec<_>>();
        assert_eq!(peak_mappings.len(), 2);
        assert!(peak_mappings.iter().all(|mapping| {
            mapping.child_count == 2
                && matches!(
                    mapping.action,
                    LgrPseudospectralMeshRefinementAction::Split {
                        segment_count: 2,
                        collocation_count
                    } if collocation_count == problem.lgr_collocation_count
                )
        }));

        let refined_report = evaluate_lgr_pseudospectral_transcription(
            &application.transcription,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
        )?;
        assert_eq!(
            refined_report.segment_reports.len(),
            problem.segment_count + 1
        );
        assert!(refined_report.residual_norm.is_finite());
        assert!(refined_report.max_abs_residual.is_finite());
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_goddard_terminal_altitude_objective_dense_sqp_improves_seed()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = GoddardBenchmarkTable::parse()?;
        let problem = table.problem;
        let ocp = lgr_goddard_ocp(problem)?;
        let (support_states, collocation_controls) = lgr_goddard_seed_decision_blocks(problem)?;
        let initial = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let initial_terminal_altitude = ocp
            .layout
            .mesh_node_state(&initial, ocp.layout.segment_count())?[0];
        let initial_report = ocp.residual_report(
            &initial,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
        )?;
        let initial_equality_row_count = initial_report.differential_residual_count
            + initial_report.continuity_residual_count
            + initial_report.boundary_residual_count;
        let initial_equality_residual_norm =
            norm(&initial_report.residuals[..initial_equality_row_count]);
        let (lower, upper) = lgr_goddard_decision_bounds(problem, &ocp.layout);
        let solver_residual_tolerance = 0.5 * problem.terminal_objective_max_equality_residual_norm;
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 20,
            residual_tolerance: solver_residual_tolerance,
            step_tolerance: 1.0e-10,
            finite_difference_step: 1.0e-6,
            regularization: 1.0e-8,
            objective_residual_penalty: 10.0,
            max_line_search_backtracks: 20,
            min_line_search_scale: 1.0e-10,
            ..crate::sqp::DenseSqpSolver::default()
        };
        let report = ocp.solve_dense_sqp_with_terminal_linear_objective(
            &initial,
            &lower,
            &upper,
            &solver,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
            0,
            -1.0,
        )?;
        assert!(
            report.terminal_value
                >= initial_terminal_altitude + problem.terminal_objective_min_altitude_gain,
            "Goddard terminal altitude {} did not improve enough from {}",
            report.terminal_value,
            initial_terminal_altitude
        );
        assert!(
            report.equality_residual_norm <= problem.terminal_objective_max_equality_residual_norm,
            "Goddard terminal-objective equality residual {:e} exceeded {}",
            report.equality_residual_norm,
            problem.terminal_objective_max_equality_residual_norm
        );
        assert!(
            report.equality_residual_norm < initial_equality_residual_norm,
            "Goddard terminal-objective equality residual {:e} did not improve from seed {:e}",
            report.equality_residual_norm,
            initial_equality_residual_norm
        );
        assert!(report.max_decision_bound_violation <= 1.0e-9);
        assert!(report.max_hard_path_violation <= 1.0e-9);
        assert_eq!(report.terminal_component, 0);
        assert_eq!(report.terminal_objective_weight, -1.0);
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_goddard_split_refined_seed_feeds_terminal_altitude_objective_dense_sqp()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = GoddardBenchmarkTable::parse()?;
        let problem = table.problem;
        let split_seed = lgr_goddard_residual_localized_split_seed(problem)?;
        let refined_ocp =
            lgr_goddard_ocp_from_transcription(problem, &split_seed.application.transcription)?;
        let (support_states, collocation_controls) =
            lgr_decision_blocks_from_transcription(&split_seed.application.transcription)?;
        let initial = refined_ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let initial_terminal_altitude = refined_ocp
            .layout
            .mesh_node_state(&initial, refined_ocp.layout.segment_count())?[0];
        let initial_report = refined_ocp.residual_report(
            &initial,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
        )?;
        let initial_equality_row_count = initial_report.differential_residual_count
            + initial_report.continuity_residual_count
            + initial_report.boundary_residual_count;
        let initial_equality_residual_norm =
            norm(&initial_report.residuals[..initial_equality_row_count]);
        let (lower, upper) = lgr_goddard_decision_bounds(problem, &refined_ocp.layout);
        let solver_residual_tolerance = 0.5 * problem.terminal_objective_max_equality_residual_norm;
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 20,
            residual_tolerance: solver_residual_tolerance,
            step_tolerance: 1.0e-10,
            finite_difference_step: 1.0e-6,
            regularization: 1.0e-8,
            objective_residual_penalty: 10.0,
            max_line_search_backtracks: 20,
            min_line_search_scale: 1.0e-10,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = refined_ocp.solve_dense_sqp_with_terminal_linear_objective(
            &initial,
            &lower,
            &upper,
            &solver,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
            0,
            -1.0,
        )?;

        assert_eq!(
            refined_ocp.layout.segment_count(),
            problem.segment_count + 1
        );
        assert!(
            report.terminal_value
                >= initial_terminal_altitude + problem.terminal_objective_min_altitude_gain,
            "split-refined Goddard terminal altitude {} did not improve enough from {}",
            report.terminal_value,
            initial_terminal_altitude
        );
        assert!(
            report.equality_residual_norm <= problem.terminal_objective_max_equality_residual_norm,
            "split-refined Goddard equality residual {:e} exceeded {}",
            report.equality_residual_norm,
            problem.terminal_objective_max_equality_residual_norm
        );
        assert!(
            report.equality_residual_norm < initial_equality_residual_norm,
            "split-refined Goddard equality residual {:e} did not improve from seed {:e}",
            report.equality_residual_norm,
            initial_equality_residual_norm
        );
        assert!(report.max_decision_bound_violation <= 1.0e-9);
        assert!(report.max_hard_path_violation <= 1.0e-9);
        assert_eq!(report.terminal_component, 0);
        assert_eq!(report.terminal_objective_weight, -1.0);
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_goddard_split_refined_terminal_objective_reports_published_altitude_gap()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = GoddardBenchmarkTable::parse()?;
        let problem = table.problem;
        let split_seed = lgr_goddard_residual_localized_split_seed(problem)?;
        let refined_ocp =
            lgr_goddard_ocp_from_transcription(problem, &split_seed.application.transcription)?;
        let (support_states, collocation_controls) =
            lgr_decision_blocks_from_transcription(&split_seed.application.transcription)?;
        let initial = refined_ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let (lower, upper) = lgr_goddard_decision_bounds(problem, &refined_ocp.layout);
        let solver_residual_tolerance = 0.5 * problem.terminal_objective_max_equality_residual_norm;
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 20,
            residual_tolerance: solver_residual_tolerance,
            step_tolerance: 1.0e-10,
            finite_difference_step: 1.0e-6,
            regularization: 1.0e-8,
            objective_residual_penalty: 10.0,
            max_line_search_backtracks: 20,
            min_line_search_scale: 1.0e-10,
            ..crate::sqp::DenseSqpSolver::default()
        };
        let report = refined_ocp.solve_dense_sqp_with_terminal_linear_objective(
            &initial,
            &lower,
            &upper,
            &solver,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
            0,
            -1.0,
        )?;
        let gap = goddard_published_final_altitude_gap_report(
            problem,
            problem.sample_final_time,
            report.terminal_value,
            report.equality_residual_norm,
        )?;

        assert!(gap.candidate_final_time.is_finite());
        assert!(gap.candidate_final_altitude.is_finite());
        assert!(gap.equality_residual_norm.is_finite());
        assert!(!gap.within_published_time_band);
        assert!(!gap.within_published_altitude_band);
        assert!(!gap.within_published_residual_band);
        assert!(!gap.published_closure_met);
        assert!(gap.absolute_final_time_error > problem.published_final_time_tolerance);
        assert!(gap.absolute_final_altitude_error > problem.published_final_altitude_tolerance);
        assert_eq!(gap.published_final_time, problem.published_final_time);
        assert_eq!(
            gap.published_final_altitude,
            problem.published_final_altitude
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_brachistochrone_dense_sqp_restores_perturbed_cycloid()
    -> Result<(), Box<dyn std::error::Error>> {
        let table = BrachistochroneBenchmarkTable::parse()?;
        let mut problem = table.problem;
        problem.segment_count = 7;
        let ocp = lgr_brachistochrone_ocp(problem, 4)?;
        let (support_states, collocation_controls) =
            lgr_brachistochrone_decision_blocks(problem, 4)?;
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let initial = perturb_lgr_brachistochrone_control_decision(&ocp.layout, &exact);
        let (lower, upper) = lgr_brachistochrone_free_control_restoration_bounds(&ocp.layout);
        let exact_gravity_m_s2 = problem.gravity_m_s2;
        let exact_report = ocp.residual_report(
            &exact,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                brachistochrone_dynamics(exact_gravity_m_s2, state, control)
            },
            unit_lagrange,
        )?;
        let initial_gravity_m_s2 = problem.gravity_m_s2;
        let initial_report = ocp.residual_report(
            &initial,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                brachistochrone_dynamics(initial_gravity_m_s2, state, control)
            },
            unit_lagrange,
        )?;
        assert!(
            exact_report.residual_norm <= problem.dense_sqp_max_residual_norm,
            "sampled LGR brachistochrone cycloid residual norm {:e} exceeded {}",
            exact_report.residual_norm,
            problem.dense_sqp_max_residual_norm
        );
        assert!(
            initial_report.residual_norm > problem.dense_sqp_max_residual_norm,
            "perturbed LGR brachistochrone residual norm {:e} should start above {}",
            initial_report.residual_norm,
            problem.dense_sqp_max_residual_norm
        );
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 24,
            residual_tolerance: problem.dense_sqp_max_residual_norm,
            step_tolerance: 1.0e-12,
            finite_difference_step: 1.0e-8,
            regularization: 1.0e-6,
            max_line_search_backtracks: 24,
            min_line_search_scale: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let gravity_m_s2 = problem.gravity_m_s2;
        let report = ocp.solve_dense_sqp(
            &initial,
            &lower,
            &upper,
            &solver,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                brachistochrone_dynamics(gravity_m_s2, state, control)
            },
            unit_lagrange,
        )?;

        assert!(
            report.sqp_solution.converged,
            "LGR brachistochrone dense SQP did not converge: norm = {:e}, iterations = {}",
            report.sqp_solution.residual_norm, report.sqp_solution.iterations
        );
        assert!(
            report.residual_report.residual_norm <= problem.dense_sqp_max_residual_norm,
            "LGR brachistochrone dense SQP residual norm {:e} exceeded {}",
            report.residual_report.residual_norm,
            problem.dense_sqp_max_residual_norm
        );
        assert!(
            report.residual_report.residual_norm <= 0.05 * initial_report.residual_norm,
            "LGR brachistochrone dense SQP residual norm {:e} did not sufficiently improve from {:e}",
            report.residual_report.residual_norm,
            initial_report.residual_norm
        );
        assert!(report.max_decision_bound_violation <= 1.0e-10);
        let max_solution_delta = exact
            .iter()
            .zip(report.sqp_solution.variables.iter())
            .map(|(expected, actual)| (expected - actual).abs())
            .fold(0.0_f64, f64::max);
        assert!(
            max_solution_delta <= 2.0 * problem.dense_sqp_max_solution_delta,
            "LGR brachistochrone dense SQP solution delta {max_solution_delta:e} exceeded {}",
            2.0 * problem.dense_sqp_max_solution_delta
        );
        Ok(())
    }

    #[test]
    fn lgr_segment_transcription_is_exact_for_polynomial_integrator() {
        let grid = legendre_gauss_radau_grid(4).expect("LGR grid builds");
        let support_states = grid
            .support_nodes
            .iter()
            .map(|node| {
                let time_s = 1.0 + node;
                time_s * time_s
            })
            .collect::<Vec<_>>();
        let segment = LgrPseudospectralSegment {
            initial_time_s: 0.0,
            final_time_s: 2.0,
            grid,
            state_dimension: 1,
            control_dimension: 0,
            support_states,
            collocation_controls: Vec::new(),
        };

        let report = evaluate_lgr_pseudospectral_segment(
            &segment,
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
        )
        .expect("LGR segment evaluates");

        assert!(report.residual_norm < 1.0e-11);
        assert!(report.max_abs_residual < 1.0e-11);
        assert!((report.quadrature_cost - 8.0 / 3.0).abs() < 1.0e-12);
        assert_eq!(report.collocation_times_s.len(), 4);
        assert_eq!(report.quadrature_weights_s.len(), 4);
    }

    #[test]
    fn lgr_transcription_stacks_segments_and_continuity_for_polynomial_integrator() {
        let transcription = LgrPseudospectralTranscription {
            segments: vec![
                polynomial_lgr_segment(0.0, 1.0),
                polynomial_lgr_segment(1.0, 2.0),
            ],
        };

        let report = evaluate_lgr_pseudospectral_transcription(
            &transcription,
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
        )
        .expect("LGR transcription evaluates");

        assert_eq!(report.segment_reports.len(), 2);
        assert_eq!(report.differential_residual_count, 8);
        assert_eq!(report.continuity_residual_count, 1);
        assert_eq!(report.residuals.len(), 9);
        assert!(report.residual_norm < 1.0e-11);
        assert!(report.max_abs_residual < 1.0e-11);
        assert!((report.quadrature_cost - 8.0 / 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn lgr_decision_layout_decodes_and_evaluates_polynomial_transcription() {
        let layout = LgrPseudospectralDecisionLayout::new(vec![0.0, 1.0, 2.0], vec![4, 4], 1, 1)
            .expect("LGR decision layout builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&layout);
        let decision = layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        assert_eq!(layout.segment_count(), 2);
        assert_eq!(layout.support_state_count(0).expect("support count"), 5);
        assert_eq!(
            layout.collocation_control_count(1).expect("control count"),
            4
        );
        assert_eq!(layout.state_variable_count, 10);
        assert_eq!(layout.control_variable_count, 8);
        assert_eq!(decision.len(), layout.variable_count);
        assert_eq!(
            layout
                .segment_support_state(&decision, 0, 0)
                .expect("initial support state"),
            &[0.0]
        );
        assert_eq!(
            layout
                .segment_support_state(&decision, 0, 4)
                .expect("terminal support state"),
            &[1.0]
        );
        assert_eq!(
            layout
                .segment_collocation_control(&decision, 0, 0)
                .expect("initial control"),
            &[1.0]
        );

        let transcription = layout
            .transcription_from_decision(&decision)
            .expect("decision decodes");
        assert_eq!(transcription.segments.len(), 2);
        assert_eq!(transcription.segments[0].grid.collocation_count(), 4);
        assert_eq!(transcription.segments[1].initial_time_s, 1.0);

        let report = layout
            .evaluate_decision(
                &decision,
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
            )
            .expect("decision evaluates");

        assert_eq!(report.transcription.segments.len(), 2);
        assert_eq!(report.transcription_report.segment_reports.len(), 2);
        assert_eq!(report.transcription_report.differential_residual_count, 8);
        assert_eq!(report.transcription_report.continuity_residual_count, 1);
        assert!(report.transcription_report.residual_norm < 1.0e-11);
        assert!(report.transcription_report.max_abs_residual < 1.0e-11);
        assert!((report.transcription_report.quadrature_cost - 8.0 / 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn lgr_decision_layout_supports_zero_control_dimension() {
        let layout = LgrPseudospectralDecisionLayout::new(vec![0.0, 1.0], vec![3], 1, 0)
            .expect("zero-control LGR layout builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&layout);
        let decision = layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("zero-control decision packs");

        assert_eq!(layout.control_variable_count, 0);
        assert_eq!(collocation_controls.len(), 0);
        assert!(
            layout
                .segment_collocation_control(&decision, 0, 0)
                .expect("zero-control slice")
                .is_empty()
        );

        let report = layout
            .evaluate_decision(
                &decision,
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
            )
            .expect("zero-control decision evaluates");

        assert_eq!(report.transcription.segments[0].control_dimension, 0);
        assert!(
            report.transcription.segments[0]
                .collocation_controls
                .is_empty()
        );
        assert!(report.transcription_report.residual_norm < 1.0e-11);
        assert!((report.transcription_report.quadrature_cost - 1.0 / 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn lgr_decision_layout_rejects_malformed_inputs() {
        assert!(LgrPseudospectralDecisionLayout::new(vec![0.0], Vec::new(), 1, 0).is_err());
        assert!(LgrPseudospectralDecisionLayout::new(vec![0.0, 0.0], vec![2], 1, 0).is_err());
        assert!(LgrPseudospectralDecisionLayout::new(vec![0.0, 1.0, 2.0], vec![2], 1, 0).is_err());
        assert!(LgrPseudospectralDecisionLayout::new(vec![0.0, 1.0], vec![0], 1, 0).is_err());
        assert!(LgrPseudospectralDecisionLayout::new(vec![0.0, 1.0], vec![2], 0, 0).is_err());

        let layout = LgrPseudospectralDecisionLayout::new(vec![0.0, 1.0], vec![2], 1, 1)
            .expect("LGR decision layout builds");
        assert!(layout.pack_decision_vector(&[0.0], &[0.0, 0.0]).is_err());
        assert!(
            layout
                .pack_decision_vector(&[0.0, f64::NAN, 1.0], &[0.0, 0.0])
                .is_err()
        );
        assert!(
            layout
                .pack_decision_vector(&[0.0, 0.25, 1.0], &[f64::INFINITY, 0.0])
                .is_err()
        );

        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&layout);
        let decision = layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut short_decision = decision.clone();
        short_decision.pop();
        assert!(layout.transcription_from_decision(&short_decision).is_err());
        assert!(layout.segment_support_state(&decision, 1, 0).is_err());
        assert!(layout.segment_support_state(&decision, 0, 3).is_err());
        assert!(layout.segment_collocation_control(&decision, 0, 2).is_err());

        let mut inconsistent_layout = layout;
        inconsistent_layout.variable_count += 1;
        assert!(
            inconsistent_layout
                .transcription_from_decision(&decision)
                .is_err()
        );
    }

    #[test]
    fn lgr_fixed_mesh_ocp_appends_boundary_residuals_to_polynomial_transcription() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0, 2.0],
            vec![4, 4],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(2, 0, 4.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        assert_eq!(
            ocp.layout
                .mesh_node_state(&decision, 1)
                .expect("interior mesh state"),
            &[1.0]
        );
        assert_eq!(
            ocp.transcription_from_decision(&decision)
                .expect("decision decodes")
                .segments
                .len(),
            2
        );

        let report = ocp
            .residual_report(
                &decision,
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
            )
            .expect("OCP residuals evaluate");

        assert_eq!(report.differential_residual_count, 8);
        assert_eq!(report.continuity_residual_count, 1);
        assert_eq!(report.boundary_residual_count, 2);
        assert_eq!(report.residuals.len(), 11);
        assert!(report.residual_norm < 1.0e-11);
        assert!(report.max_abs_residual < 1.0e-11);
        assert!((report.quadrature_cost - 8.0 / 3.0).abs() < 1.0e-12);
        assert!(report.residuals[9].abs() < 1.0e-13);
        assert!(report.residuals[10].abs() < 1.0e-13);

        let residuals = ocp
            .residuals(
                &decision,
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
            )
            .expect("residual vector evaluates");
        assert_eq!(residuals, report.residuals);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_reports_boundary_mismatch() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            1,
            0,
            vec![LgrPseudospectralBoundaryConstraint::new(1, 0, 2.0).expect("terminal boundary")],
        )
        .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let report = ocp
            .residual_report(
                &decision,
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
            )
            .expect("OCP residuals evaluate");

        assert_eq!(report.differential_residual_count, 4);
        assert_eq!(report.continuity_residual_count, 0);
        assert_eq!(report.boundary_residual_count, 1);
        assert!((report.residuals[4] + 1.0).abs() < 1.0e-13);
        assert!((report.max_abs_residual - 1.0).abs() < 1.0e-13);
        assert!((report.residual_norm - 1.0).abs() < 1.0e-13);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_appends_path_and_quadratic_control_objective_residuals() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_path_constraints(vec![
                LgrPseudospectralSoftPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                    Some(0.2),
                    None,
                    9.0,
                )
                .expect("state path constraint"),
                LgrPseudospectralSoftPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
                    None,
                    Some(0.5),
                    4.0,
                )
                .expect("control path constraint"),
            ])
            .expect("path constraints attach")
            .with_quadratic_control_costs(vec![
                LgrPseudospectralQuadraticControlCost::new(0, 2.0).expect("control cost"),
            ])
            .expect("control costs attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let report = ocp
            .residual_report(
                &decision,
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
                |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
            )
            .expect("OCP residuals evaluate");

        assert_eq!(report.differential_residual_count, 4);
        assert_eq!(report.continuity_residual_count, 0);
        assert_eq!(report.boundary_residual_count, 0);
        assert_eq!(report.path_residual_count, 2);
        assert_eq!(report.objective_residual_count, 4);
        assert_eq!(report.residuals.len(), 10);
        assert!((report.quadrature_cost - 1.0 / 3.0).abs() < 1.0e-12);
        assert!((report.quadratic_control_cost - 14.0).abs() < 1.0e-12);

        let path_offset = report.differential_residual_count
            + report.continuity_residual_count
            + report.boundary_residual_count;
        assert!((report.residuals[path_offset] + 0.6).abs() < 1.0e-13);
        assert!((report.residuals[path_offset + 1] - 1.0).abs() < 1.0e-13);
        let objective_offset = path_offset + report.path_residual_count;
        let objective_norm_squared = report.residuals[objective_offset..]
            .iter()
            .map(|value| value * value)
            .sum::<f64>();
        assert!((objective_norm_squared - report.quadratic_control_cost).abs() < 1.0e-12);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_objective_report_transcribes_lagrange_mayer_and_quadratic_costs() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_quadratic_control_costs(vec![
                LgrPseudospectralQuadraticControlCost::new(0, 2.0).expect("control cost"),
            ])
            .expect("control costs attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut lagrange =
            |time_s: f64, state: &[f64], control: &[f64]| Ok(time_s + state[0] + 2.0 * control[0]);
        let mut mayer = |terminal_time_s: f64, terminal_state: &[f64]| {
            Ok(5.0 * terminal_state[0] - terminal_time_s)
        };

        let report = ocp
            .objective_report_with_mayer(&decision, &mut lagrange, &mut mayer)
            .expect("objective evaluates");

        let grid = &ocp.layout.segment_grids[0];
        let expected_lagrange = grid
            .collocation_nodes
            .iter()
            .zip(grid.quadrature_weights.iter())
            .map(|(node, weight)| {
                let time_s = test_physical_time(0.0, 1.0, *node);
                let state = time_s * time_s;
                let control = 1.0 + 3.0 * time_s;
                0.5 * weight * (time_s + state + 2.0 * control)
            })
            .sum::<f64>();

        assert_eq!(report.terminal_time_s, 1.0);
        assert_eq!(report.terminal_state, vec![1.0]);
        assert!((report.lagrange_quadrature_cost - expected_lagrange).abs() < 1.0e-12);
        assert!((report.quadratic_control_cost - 14.0).abs() < 1.0e-12);
        assert_eq!(report.mayer_cost, 4.0);
        assert!(
            (report.total_objective
                - (report.lagrange_quadrature_cost
                    + report.quadratic_control_cost
                    + report.mayer_cost))
                .abs()
                < 1.0e-12
        );

        let mut lagrange_only = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(2.0);
        let no_mayer = ocp
            .objective_report(&decision, &mut lagrange_only)
            .expect("objective without Mayer evaluates");
        assert_eq!(no_mayer.mayer_cost, 0.0);
        assert!((no_mayer.lagrange_quadrature_cost - 2.0).abs() < 1.0e-12);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_objective_report_inputs() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 1, Vec::new())
            .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut lagrange = zero_lagrange;
        assert!(
            ocp.objective_report(&decision[..decision.len() - 1], &mut lagrange)
                .is_err()
        );

        let mut nonfinite_lagrange = |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(f64::NAN);
        assert!(
            ocp.objective_report(&decision, &mut nonfinite_lagrange)
                .is_err()
        );

        let mut finite_lagrange = zero_lagrange;
        let mut nonfinite_mayer =
            |_terminal_time_s: f64, _terminal_state: &[f64]| Ok(f64::INFINITY);
        assert!(
            ocp.objective_report_with_mayer(&decision, &mut finite_lagrange, &mut nonfinite_mayer)
                .is_err()
        );

        let malformed_report = LgrPseudospectralObjectiveReport {
            terminal_time_s: 1.0,
            terminal_state: vec![1.0],
            lagrange_quadrature_cost: 1.0,
            quadratic_control_cost: 2.0,
            mayer_cost: 3.0,
            total_objective: 7.0,
        };
        assert!(validate_lgr_objective_report(&malformed_report).is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_slices_kkt_multipliers_and_reports_costate_hamiltonian_diagnostics() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0, 2.0],
            vec![2, 3],
            2,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(2, 1, 2.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds")
        .with_path_constraints(vec![
            LgrPseudospectralSoftPathConstraint::new(
                1,
                0,
                LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
                Some(0.0),
                None,
                4.0,
            )
            .expect("path constraint"),
        ])
        .expect("path constraints attach")
        .with_quadratic_control_costs(vec![
            LgrPseudospectralQuadraticControlCost::new(0, 0.5).expect("control cost"),
        ])
        .expect("control costs attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let residual_report = ocp
            .residual_report(&decision, linear_lgr_dynamics, affine_lgr_lagrange)
            .expect("residuals evaluate");
        let residual_multipliers = (0..residual_report.residuals.len())
            .map(|index| index as f64 + 0.25)
            .collect::<Vec<_>>();

        let multiplier_report = ocp
            .kkt_residual_multiplier_report(&residual_report, &residual_multipliers)
            .expect("LGR KKT multipliers slice");

        assert_eq!(multiplier_report.segment_count, 2);
        assert_eq!(multiplier_report.state_dimension, 2);
        assert_eq!(
            multiplier_report.residual_multiplier_count,
            residual_multipliers.len()
        );
        assert_eq!(multiplier_report.differential_multiplier_count, 10);
        assert_eq!(multiplier_report.continuity_multiplier_count, 2);
        assert_eq!(multiplier_report.boundary_multiplier_count, 2);
        assert_eq!(multiplier_report.path_multiplier_count, 1);
        assert_eq!(multiplier_report.objective_multiplier_count, 5);
        assert_eq!(multiplier_report.collocation_samples.len(), 5);
        assert_eq!(
            multiplier_report.collocation_samples[0].dynamics_defect_multipliers,
            vec![0.25, 1.25]
        );
        assert_eq!(
            multiplier_report.collocation_samples[2].dynamics_defect_multipliers,
            vec![4.25, 5.25]
        );
        assert_eq!(multiplier_report.continuity_multipliers, vec![10.25, 11.25]);
        assert_eq!(multiplier_report.boundary_multipliers, vec![12.25, 13.25]);
        assert_eq!(multiplier_report.path_multipliers, vec![14.25]);
        assert_eq!(
            multiplier_report.objective_multipliers,
            vec![15.25, 16.25, 17.25, 18.25, 19.25]
        );
        assert_eq!(multiplier_report.max_abs_dynamics_multiplier, 9.25);
        assert_eq!(multiplier_report.max_abs_continuity_multiplier, 11.25);
        assert_eq!(multiplier_report.max_abs_boundary_multiplier, 13.25);

        let costate_report = ocp
            .kkt_collocation_costate_estimate_report(&residual_report, &residual_multipliers)
            .expect("LGR costate estimate reports");
        assert_eq!(costate_report.segment_count, 2);
        assert_eq!(costate_report.state_dimension, 2);
        assert_eq!(costate_report.collocation_samples.len(), 5);
        assert_eq!(
            costate_report.collocation_samples[4].costate,
            vec![8.25, 9.25]
        );
        assert_eq!(costate_report.max_abs_costate, 9.25);

        let mut dynamics = linear_lgr_dynamics;
        let mut lagrange = affine_lgr_lagrange;
        let hamiltonian_report = ocp
            .kkt_hamiltonian_report(
                &decision,
                &residual_report,
                &residual_multipliers,
                &mut dynamics,
                &mut lagrange,
            )
            .expect("LGR Hamiltonian diagnostics evaluate");
        assert_eq!(hamiltonian_report.segment_count, 2);
        assert_eq!(hamiltonian_report.state_dimension, 2);
        assert_eq!(hamiltonian_report.sample_count, 5);
        assert_eq!(hamiltonian_report.samples.len(), 5);
        let mut expected_max_abs_hamiltonian = 0.0_f64;
        let mut expected_max_abs_hamiltonian_drift = 0.0_f64;
        let first_hamiltonian = hamiltonian_report.samples[0].hamiltonian;
        for sample in &hamiltonian_report.samples {
            let state = ocp
                .layout
                .segment_support_state(&decision, sample.segment_index, sample.collocation_index)
                .expect("sample state");
            let control = ocp
                .layout
                .segment_collocation_control(
                    &decision,
                    sample.segment_index,
                    sample.collocation_index,
                )
                .expect("sample control");
            let expected_dynamics =
                linear_lgr_dynamics(sample.time_s, state, control).expect("expected dynamics");
            let expected_lagrange =
                affine_lgr_lagrange(sample.time_s, state, control).expect("expected lagrange");
            let expected_dot = sample
                .costate
                .iter()
                .zip(expected_dynamics.iter())
                .map(|(costate, dynamics)| costate * dynamics)
                .sum::<f64>();
            assert_eq!(sample.dynamics, expected_dynamics);
            assert!((sample.lagrange_value - expected_lagrange).abs() < 1.0e-14);
            assert!((sample.costate_dot_dynamics - expected_dot).abs() < 1.0e-12);
            assert!((sample.hamiltonian - (expected_lagrange + expected_dot)).abs() < 1.0e-12);
            expected_max_abs_hamiltonian =
                expected_max_abs_hamiltonian.max(sample.hamiltonian.abs());
            expected_max_abs_hamiltonian_drift = expected_max_abs_hamiltonian_drift
                .max((sample.hamiltonian - first_hamiltonian).abs());
        }
        assert!(
            (hamiltonian_report.max_abs_hamiltonian - expected_max_abs_hamiltonian).abs() < 1.0e-12
        );
        assert!(
            (hamiltonian_report.max_abs_hamiltonian_drift - expected_max_abs_hamiltonian_drift)
                .abs()
                < 1.0e-12
        );

        let mut malformed_multipliers = residual_multipliers.clone();
        malformed_multipliers.pop();
        assert!(
            ocp.kkt_residual_multiplier_report(&residual_report, &malformed_multipliers)
                .is_err()
        );
        assert!(
            ocp.kkt_collocation_costate_estimate_report(&residual_report, &malformed_multipliers)
                .is_err()
        );
        let mut malformed_dynamics = linear_lgr_dynamics;
        let mut malformed_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.kkt_hamiltonian_report(
                &decision,
                &residual_report,
                &malformed_multipliers,
                &mut malformed_dynamics,
                &mut malformed_lagrange,
            )
            .is_err()
        );

        let mut malformed_report = residual_report;
        malformed_report.differential_residual_count += 1;
        assert!(
            ocp.kkt_residual_multiplier_report(&malformed_report, &residual_multipliers)
                .is_err()
        );
    }

    #[test]
    fn lgr_pmp_costate_convention_bridge_scales_collocation_profile() -> Result<(), TrajoptError> {
        let raw_costates = vec![1.0, -3.0, 2.0, 4.0];
        let costate_report = LgrPseudospectralKktCostateEstimateReport {
            segment_count: 1,
            state_dimension: 2,
            source_residual_multiplier_count: raw_costates.len(),
            collocation_samples: vec![
                LgrPseudospectralKktCollocationCostateEstimate {
                    segment_index: 0,
                    collocation_index: 0,
                    time_s: 0.0,
                    costate: raw_costates[0..2].to_vec(),
                },
                LgrPseudospectralKktCollocationCostateEstimate {
                    segment_index: 0,
                    collocation_index: 1,
                    time_s: 0.5,
                    costate: raw_costates[2..4].to_vec(),
                },
            ],
            max_abs_costate: 4.0,
        };

        let convention_report = costate_report.pmp_costate_convention_report(
            crate::pmp::PmpDirectCostateConvention {
                raw_to_pmp_scale: -2.0,
            },
        )?;

        assert_eq!(convention_report.state_dimension, 2);
        assert_eq!(convention_report.sample_count, 2);
        assert_eq!(convention_report.times_s, vec![0.0, 0.5]);
        assert_eq!(convention_report.raw_costates, raw_costates);
        assert_eq!(convention_report.pmp_costates, vec![-2.0, 6.0, -4.0, -8.0]);
        assert_eq!(convention_report.max_abs_raw_costate, 4.0);
        assert_eq!(convention_report.max_abs_pmp_costate, 8.0);
        Ok(())
    }

    #[test]
    fn lgr_fixed_mesh_ocp_sparse_gauss_newton_step_matches_dense_normal_system() {
        let ocp = lgr_sparse_linear_ocp().expect("LGR OCP builds");
        let decision = lgr_sparse_linear_decision(&ocp);
        let regularization = 1.0;
        let mut dynamics = linear_lgr_dynamics;
        let mut lagrange = affine_lgr_lagrange;
        let residual_report = ocp
            .residual_report(&decision, &mut dynamics, &mut lagrange)
            .expect("residuals evaluate");
        let jacobian = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-9,
            )
            .expect("sparse residual Jacobian evaluates");
        let dense_jacobian = jacobian.to_dense_row_major().expect("dense Jacobian");
        let hessian = jacobian
            .sparse_gauss_newton_hessian_report(regularization, 1.0e-12)
            .expect("sparse Gauss-Newton Hessian assembles");
        let expected_hessian = lgr_dense_gauss_newton_hessian(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let dense_hessian = hessian.to_dense_row_major().expect("dense Hessian");

        assert_eq!(hessian.row_count, jacobian.column_count);
        assert_eq!(hessian.column_count, jacobian.column_count);
        assert_eq!(hessian.residual_row_count, jacobian.row_count);
        assert_eq!(hessian.source_jacobian_nonzero_count, jacobian.values.len());
        for row in 0..hessian.row_count {
            for column in 0..hessian.column_count {
                let index = row * hessian.column_count + column;
                assert!(
                    (dense_hessian[index] - expected_hessian[index]).abs() < 1.0e-8,
                    "H({row}, {column}) expected {}, got {}",
                    expected_hessian[index],
                    dense_hessian[index]
                );
                assert!(
                    (hessian.coefficient(row, column).expect("coefficient")
                        - expected_hessian[index])
                        .abs()
                        < 1.0e-8,
                    "coefficient H({row}, {column}) mismatch"
                );
            }
        }

        let rhs = jacobian
            .sparse_gauss_newton_step_rhs(&residual_report.residuals)
            .expect("sparse RHS assembles");
        let expected_rhs = lgr_dense_gauss_newton_step_rhs(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            &residual_report.residuals,
        );
        for (index, (actual, expected)) in rhs.iter().zip(expected_rhs.iter()).enumerate() {
            assert!(
                (actual - expected).abs() < 1.0e-9,
                "rhs index {index} expected {expected}, got {actual}"
            );
        }

        let known_solution = (0..jacobian.column_count)
            .map(|index| {
                if index % 2 == 0 {
                    0.03 * (index as f64 + 1.0)
                } else {
                    -0.02 * (index as f64 + 1.0)
                }
            })
            .collect::<Vec<_>>();
        let sparse_product = hessian
            .multiply_vector(&known_solution)
            .expect("sparse matvec");
        let dense_product =
            lgr_dense_matrix_vector(jacobian.column_count, &expected_hessian, &known_solution);
        for (index, (actual, expected)) in
            sparse_product.iter().zip(dense_product.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-9,
                "matvec index {index} expected {expected}, got {actual}"
            );
        }
        let zero = vec![0.0; jacobian.column_count];
        let solve = hessian
            .solve_conjugate_gradient(&sparse_product, &zero, jacobian.column_count * 4, 1.0e-11)
            .expect("CG solves sparse normal system");
        assert!(solve.converged);
        assert!(solve.residual_norm < 1.0e-8);
        for (index, (actual, expected)) in
            solve.solution.iter().zip(known_solution.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "CG solution index {index} expected {expected}, got {actual}"
            );
        }

        let mut step_dynamics = linear_lgr_dynamics;
        let mut step_lagrange = affine_lgr_lagrange;
        let report = ocp
            .sparse_gauss_newton_step_report(
                &decision,
                &mut step_dynamics,
                &mut step_lagrange,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
                ocp.layout.variable_count * 4,
                1.0e-12,
            )
            .expect("sparse Gauss-Newton step assembles");
        let expected_step = crate::corrector::solve_linear_system(
            expected_hessian,
            expected_rhs,
            report.sparse_jacobian.column_count,
        )
        .expect("dense normal system solves");
        let expected_linearized_residual = lgr_dense_rectangular_matrix_vector(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &expected_step,
        )
        .iter()
        .zip(report.residual_report.residuals.iter())
        .map(|(jacobian_step, residual)| residual + jacobian_step)
        .collect::<Vec<_>>();

        assert!(report.solve_report.converged);
        assert!(report.solve_report.residual_norm < 1.0e-8);
        assert!((report.step_norm - norm(&expected_step)).abs() < 1.0e-8);
        for (index, (actual, expected)) in report
            .solve_report
            .solution
            .iter()
            .zip(expected_step.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "step index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .linearized_residual
            .iter()
            .zip(expected_linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "linearized residual index {index} expected {expected}, got {actual}"
            );
        }
    }

    #[test]
    fn lgr_fixed_mesh_ocp_sparse_augmented_kkt_step_reports_multipliers() {
        let ocp = lgr_sparse_linear_ocp().expect("LGR OCP builds");
        let decision = lgr_sparse_linear_decision(&ocp);
        let regularization = 1.0;
        let mut dynamics = linear_lgr_dynamics;
        let mut lagrange = affine_lgr_lagrange;
        let jacobian = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-9,
            )
            .expect("sparse residual Jacobian evaluates");
        let dense_jacobian = jacobian.to_dense_row_major().expect("dense Jacobian");
        let expected_kkt = lgr_dense_augmented_kkt(
            jacobian.row_count,
            jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let kkt = jacobian
            .sparse_augmented_kkt_report(regularization, 1.0e-12)
            .expect("sparse augmented KKT assembles");
        let dense_kkt = kkt.to_dense_row_major().expect("dense KKT");

        assert_eq!(kkt.row_count, jacobian.column_count + jacobian.row_count);
        assert_eq!(kkt.column_count, kkt.row_count);
        assert_eq!(kkt.variable_count, jacobian.column_count);
        assert_eq!(kkt.residual_row_count, jacobian.row_count);
        assert_eq!(kkt.source_jacobian_nonzero_count, jacobian.values.len());
        for row in 0..kkt.row_count {
            for column in 0..kkt.column_count {
                let index = row * kkt.column_count + column;
                assert!(
                    (dense_kkt[index] - expected_kkt[index]).abs() < 1.0e-9,
                    "KKT({row}, {column}) expected {}, got {}",
                    expected_kkt[index],
                    dense_kkt[index]
                );
                assert!(
                    (kkt.coefficient(row, column).expect("coefficient") - expected_kkt[index])
                        .abs()
                        < 1.0e-9,
                    "coefficient KKT({row}, {column}) mismatch"
                );
            }
        }

        let known_solution = (0..kkt.column_count)
            .map(|index| {
                if index % 2 == 0 {
                    0.02 * (index as f64 + 1.0)
                } else {
                    -0.01 * (index as f64 + 1.0)
                }
            })
            .collect::<Vec<_>>();
        let sparse_product = kkt.multiply_vector(&known_solution).expect("KKT matvec");
        let dense_product =
            lgr_dense_matrix_vector(kkt.column_count, &expected_kkt, &known_solution);
        for (index, (actual, expected)) in
            sparse_product.iter().zip(dense_product.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-9,
                "KKT matvec index {index} expected {expected}, got {actual}"
            );
        }
        let solve = kkt.solve_ldlt(&dense_product).expect("LDL solve");
        assert!(solve.residual_norm < 1.0e-8);
        assert!(solve.factorization_residual_norm < 1.0e-8);
        assert_eq!(solve.factor_diagonal.len(), kkt.column_count);
        for (index, (actual, expected)) in
            solve.solution.iter().zip(known_solution.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-8,
                "LDL solution index {index} expected {expected}, got {actual}"
            );
        }

        let mut step_dynamics = linear_lgr_dynamics;
        let mut step_lagrange = affine_lgr_lagrange;
        let report = ocp
            .sparse_augmented_kkt_step_report(
                &decision,
                &mut step_dynamics,
                &mut step_lagrange,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
            )
            .expect("sparse augmented KKT step assembles");
        let dense_hessian = lgr_dense_gauss_newton_hessian(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            regularization,
        );
        let dense_rhs = lgr_dense_gauss_newton_step_rhs(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &report.residual_report.residuals,
        );
        let expected_step = crate::corrector::solve_linear_system(
            dense_hessian,
            dense_rhs,
            report.sparse_jacobian.column_count,
        )
        .expect("dense normal system solves");
        let expected_linearized_residual = lgr_dense_rectangular_matrix_vector(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &expected_step,
        )
        .iter()
        .zip(report.residual_report.residuals.iter())
        .map(|(jacobian_step, residual)| residual + jacobian_step)
        .collect::<Vec<_>>();

        assert!(report.solve_report.residual_norm < 1.0e-8);
        assert!(report.solve_report.factorization_residual_norm < 1.0e-8);
        assert_eq!(report.rhs.len(), report.augmented_kkt.row_count);
        assert!((report.step_norm - norm(&expected_step)).abs() < 1.0e-8);
        for rhs in &report.rhs[..report.sparse_jacobian.column_count] {
            assert!(rhs.abs() < 1.0e-15);
        }
        for (index, residual) in report.residual_report.residuals.iter().enumerate() {
            assert!(
                (report.rhs[report.sparse_jacobian.column_count + index] + residual).abs()
                    < 1.0e-15,
                "KKT RHS residual block mismatch at {index}"
            );
        }
        for (index, (actual, expected)) in report.step.iter().zip(expected_step.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "KKT step index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .linearized_residual
            .iter()
            .zip(expected_linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "linearized residual index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .residual_multipliers
            .iter()
            .zip(expected_linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "KKT residual multiplier index {index} expected {expected}, got {actual}"
            );
        }

        let direct_multiplier_report = ocp
            .kkt_residual_multiplier_report(&report.residual_report, &report.residual_multipliers)
            .expect("direct KKT multiplier report slices");
        let step_multiplier_report = ocp
            .sparse_kkt_residual_multiplier_report(&report)
            .expect("step KKT multiplier report slices");
        assert_eq!(step_multiplier_report, direct_multiplier_report);
        assert_eq!(step_multiplier_report.segment_count, 2);
        assert_eq!(step_multiplier_report.state_dimension, 2);
        assert_eq!(
            step_multiplier_report.differential_multiplier_count,
            report.residual_report.differential_residual_count
        );
        assert_eq!(
            step_multiplier_report.continuity_multiplier_count,
            report.residual_report.continuity_residual_count
        );
        assert_eq!(
            step_multiplier_report.boundary_multiplier_count,
            report.residual_report.boundary_residual_count
        );
        assert_eq!(step_multiplier_report.collocation_samples.len(), 4);

        let costate_report = ocp
            .sparse_kkt_collocation_costate_estimate_report(&report)
            .expect("step KKT costate report evaluates");
        assert_eq!(costate_report.segment_count, 2);
        assert_eq!(costate_report.state_dimension, 2);
        assert_eq!(costate_report.collocation_samples.len(), 4);
        assert!(costate_report.max_abs_costate >= 0.0);
        let pmp_costate_report = ocp
            .sparse_kkt_pmp_costate_convention_report(
                &report,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
            )
            .expect("step KKT PMP costate convention report evaluates");
        assert_eq!(
            pmp_costate_report.sample_count,
            costate_report.collocation_samples.len()
        );
        assert_eq!(
            pmp_costate_report.state_dimension,
            costate_report.state_dimension
        );
        assert_eq!(pmp_costate_report.raw_to_pmp_scale, 1.0);
        for (pmp_sample, raw_sample) in pmp_costate_report
            .samples
            .iter()
            .zip(costate_report.collocation_samples.iter())
        {
            assert_eq!(pmp_sample.time_s.to_bits(), raw_sample.time_s.to_bits());
            assert_eq!(pmp_sample.raw_costate, raw_sample.costate);
            assert_eq!(pmp_sample.pmp_costate, raw_sample.costate);
        }
        let normalized_costate_report = ocp
            .sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                &report,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
            )
            .expect("step KKT normalized PMP costate convention report evaluates");
        assert_eq!(
            normalized_costate_report.sample_count,
            costate_report.collocation_samples.len()
        );
        assert_eq!(
            normalized_costate_report.state_dimension,
            costate_report.state_dimension
        );
        for (pmp_sample, raw_sample) in normalized_costate_report
            .samples
            .iter()
            .zip(costate_report.collocation_samples.iter())
        {
            let weight = ocp.layout.segment_grids[raw_sample.segment_index].quadrature_weights
                [raw_sample.collocation_index];
            assert_eq!(pmp_sample.time_s.to_bits(), raw_sample.time_s.to_bits());
            for (actual, expected_raw) in
                pmp_sample.raw_costate.iter().zip(raw_sample.costate.iter())
            {
                assert!((actual - expected_raw / weight).abs() < 1.0e-12);
            }
            assert_eq!(pmp_sample.pmp_costate, pmp_sample.raw_costate);
        }

        let mut hamiltonian_dynamics = linear_lgr_dynamics;
        let mut hamiltonian_lagrange = affine_lgr_lagrange;
        let hamiltonian_report = ocp
            .sparse_kkt_hamiltonian_report(
                &decision,
                &report,
                &mut hamiltonian_dynamics,
                &mut hamiltonian_lagrange,
            )
            .expect("step KKT Hamiltonian report evaluates");
        assert_eq!(hamiltonian_report.segment_count, 2);
        assert_eq!(hamiltonian_report.state_dimension, 2);
        assert_eq!(hamiltonian_report.sample_count, 4);
        assert!(hamiltonian_report.max_abs_hamiltonian >= 0.0);
        assert!(hamiltonian_report.max_abs_hamiltonian_drift >= 0.0);
        let mut pmp_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut pmp_hamiltonian_lagrange = affine_lgr_lagrange;
        let pmp_hamiltonian_report = ocp
            .sparse_kkt_pmp_hamiltonian_report(
                &decision,
                &report,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
                &mut pmp_hamiltonian_dynamics,
                &mut pmp_hamiltonian_lagrange,
            )
            .expect("step KKT PMP Hamiltonian report evaluates");
        assert_eq!(
            pmp_hamiltonian_report.sample_count,
            hamiltonian_report.sample_count
        );
        assert_eq!(
            pmp_hamiltonian_report.state_dimension,
            hamiltonian_report.state_dimension
        );
        assert_eq!(
            pmp_hamiltonian_report.control_dimension,
            ocp.layout.control_dimension
        );
        assert!(
            (pmp_hamiltonian_report.max_abs_hamiltonian - hamiltonian_report.max_abs_hamiltonian)
                .abs()
                < 1.0e-12
        );
        assert!(
            (pmp_hamiltonian_report.max_abs_hamiltonian_drift
                - hamiltonian_report.max_abs_hamiltonian_drift)
                .abs()
                < 1.0e-12
        );
        for (pmp_sample, raw_sample) in pmp_hamiltonian_report
            .samples
            .iter()
            .zip(hamiltonian_report.samples.iter())
        {
            assert_eq!(pmp_sample.time_s.to_bits(), raw_sample.time_s.to_bits());
            assert!((pmp_sample.hamiltonian - raw_sample.hamiltonian).abs() < 1.0e-12);
        }
        let mut normalized_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut normalized_hamiltonian_lagrange = affine_lgr_lagrange;
        let normalized_hamiltonian_report = ocp
            .sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                &decision,
                &report,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
                &mut normalized_hamiltonian_dynamics,
                &mut normalized_hamiltonian_lagrange,
            )
            .expect("step KKT normalized PMP Hamiltonian report evaluates");
        assert_eq!(
            normalized_hamiltonian_report.sample_count,
            normalized_costate_report.sample_count
        );
        assert_eq!(normalized_hamiltonian_report.state_dimension, 2);
        assert_eq!(
            normalized_hamiltonian_report.control_dimension,
            ocp.layout.control_dimension
        );
        for (hamiltonian_sample, costate_sample) in normalized_hamiltonian_report
            .samples
            .iter()
            .zip(normalized_costate_report.samples.iter())
        {
            assert_eq!(
                hamiltonian_sample.time_s.to_bits(),
                costate_sample.time_s.to_bits()
            );
            assert_eq!(hamiltonian_sample.costate, costate_sample.pmp_costate);
        }

        assert!(jacobian.sparse_gauss_newton_step_rhs(&[0.0]).is_err());
        assert!(
            jacobian
                .sparse_gauss_newton_hessian_report(-1.0, 1.0e-12)
                .is_err()
        );
        assert!(jacobian.sparse_augmented_kkt_report(0.0, 1.0e-12).is_err());
        assert!(kkt.multiply_vector(&[0.0]).is_err());
        assert!(kkt.solve_ldlt(&[0.0]).is_err());

        let mut malformed_step = report.clone();
        malformed_step.residual_multipliers.pop();
        assert!(
            ocp.sparse_kkt_residual_multiplier_report(&malformed_step)
                .is_err()
        );
        assert!(
            ocp.sparse_kkt_collocation_costate_estimate_report(&malformed_step)
                .is_err()
        );
        assert!(
            ocp.sparse_kkt_pmp_costate_convention_report(
                &malformed_step,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
            )
            .is_err()
        );
        assert!(
            ocp.sparse_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                &malformed_step,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
            )
            .is_err()
        );
        let mut malformed_dynamics = linear_lgr_dynamics;
        let mut malformed_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.sparse_kkt_hamiltonian_report(
                &decision,
                &malformed_step,
                &mut malformed_dynamics,
                &mut malformed_lagrange,
            )
            .is_err()
        );
        let mut malformed_pmp_dynamics = linear_lgr_dynamics;
        let mut malformed_pmp_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.sparse_kkt_pmp_hamiltonian_report(
                &decision,
                &malformed_step,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
                &mut malformed_pmp_dynamics,
                &mut malformed_pmp_lagrange,
            )
            .is_err()
        );
        let mut malformed_normalized_pmp_dynamics = linear_lgr_dynamics;
        let mut malformed_normalized_pmp_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.sparse_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                &decision,
                &malformed_step,
                crate::pmp::PmpDirectCostateConvention {
                    raw_to_pmp_scale: 1.0,
                },
                &mut malformed_normalized_pmp_dynamics,
                &mut malformed_normalized_pmp_lagrange,
            )
            .is_err()
        );

        let malformed_hessian = LgrPseudospectralSparseGaussNewtonHessianReport {
            row_count: 2,
            column_count: 2,
            residual_row_count: 1,
            source_jacobian_nonzero_count: 1,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![1.0],
            regularization: 0.0,
            drop_tolerance: 1.0e-12,
        };
        assert!(malformed_hessian.to_dense_row_major().is_err());

        let non_positive_curvature = LgrPseudospectralSparseGaussNewtonHessianReport {
            row_count: 1,
            column_count: 1,
            residual_row_count: 1,
            source_jacobian_nonzero_count: 1,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![-1.0],
            regularization: 0.0,
            drop_tolerance: 0.0,
        };
        assert!(
            non_positive_curvature
                .solve_conjugate_gradient(&[1.0], &[0.0], 4, 1.0e-9)
                .is_err()
        );

        let malformed_kkt = LgrPseudospectralSparseAugmentedKktReport {
            row_count: 3,
            column_count: 3,
            variable_count: 2,
            residual_row_count: 1,
            source_jacobian_nonzero_count: 1,
            column_offsets: vec![0, 1],
            row_indices: vec![0],
            values: vec![1.0],
            regularization: 1.0,
            drop_tolerance: 1.0e-12,
        };
        assert!(malformed_kkt.to_dense_row_major().is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_sparse_constrained_kkt_step_reports_active_hard_path_multipliers() {
        let ocp = lgr_sparse_linear_ocp()
            .expect("LGR OCP builds")
            .with_hard_path_constraints(vec![
                LgrPseudospectralHardPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                    None,
                    Some(0.0),
                )
                .expect("active state hard path constraint"),
            ])
            .expect("hard path constraints attach");
        let decision = lgr_sparse_linear_decision(&ocp);
        let regularization = 1.0;
        let mut dynamics = linear_lgr_dynamics;
        let mut lagrange = affine_lgr_lagrange;

        let report = ocp
            .sparse_constrained_kkt_step_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-9,
                regularization,
                1.0e-12,
                1.0e-12,
            )
            .expect("sparse constrained KKT step assembles");
        let dense_jacobian = report
            .sparse_jacobian
            .to_dense_row_major()
            .expect("dense Jacobian");
        let expected_kkt = lgr_dense_constrained_augmented_kkt(
            report.sparse_jacobian.row_count,
            report.sparse_jacobian.column_count,
            &dense_jacobian,
            &report.hard_path_inequality_report,
            &report.constrained_kkt.active_inequality_rows,
            regularization,
        );
        let dense_kkt = report
            .constrained_kkt
            .to_dense_row_major()
            .expect("dense constrained KKT");

        assert_eq!(report.constrained_kkt.active_inequality_row_count, 1);
        assert_eq!(report.constrained_kkt.active_inequality_rows, vec![0]);
        assert_eq!(report.constrained_kkt.active_inequality_values, vec![0.0]);
        assert_eq!(report.active_inequality_multipliers.len(), 1);
        assert_eq!(
            report.constrained_kkt.row_count,
            report.sparse_jacobian.column_count + report.sparse_jacobian.row_count + 1
        );
        for row in 0..report.constrained_kkt.row_count {
            for column in 0..report.constrained_kkt.column_count {
                let index = row * report.constrained_kkt.column_count + column;
                assert!(
                    (dense_kkt[index] - expected_kkt[index]).abs() < 1.0e-9,
                    "constrained KKT({row}, {column}) expected {}, got {}",
                    expected_kkt[index],
                    dense_kkt[index]
                );
                assert!(
                    (report
                        .constrained_kkt
                        .coefficient(row, column)
                        .expect("coefficient")
                        - expected_kkt[index])
                        .abs()
                        < 1.0e-9,
                    "constrained coefficient KKT({row}, {column}) mismatch"
                );
            }
        }

        let expected_solution = crate::corrector::solve_linear_system(
            expected_kkt.clone(),
            report.rhs.clone(),
            report.constrained_kkt.row_count,
        )
        .expect("dense constrained KKT solves");
        let expected_step = expected_solution[..report.sparse_jacobian.column_count].to_vec();
        let residual_end = report.sparse_jacobian.column_count + report.sparse_jacobian.row_count;
        let expected_residual_multipliers =
            expected_solution[report.sparse_jacobian.column_count..residual_end].to_vec();
        let expected_active_multipliers = expected_solution[residual_end..].to_vec();

        assert!(report.solve_report.residual_norm < 1.0e-8);
        assert!(report.solve_report.factorization_residual_norm < 1.0e-8);
        assert_eq!(report.rhs.len(), report.constrained_kkt.row_count);
        assert!((report.step_norm - norm(&expected_step)).abs() < 1.0e-8);
        for rhs in &report.rhs[..report.sparse_jacobian.column_count] {
            assert!(rhs.abs() < 1.0e-15);
        }
        for (index, residual) in report.residual_report.residuals.iter().enumerate() {
            assert!(
                (report.rhs[report.sparse_jacobian.column_count + index] + residual).abs()
                    < 1.0e-15,
                "constrained KKT RHS residual block mismatch at {index}"
            );
        }
        for (index, value) in report
            .constrained_kkt
            .active_inequality_values
            .iter()
            .enumerate()
        {
            let offset = report.sparse_jacobian.column_count + report.sparse_jacobian.row_count;
            assert!(
                (report.rhs[offset + index] + value).abs() < 1.0e-15,
                "constrained KKT RHS active block mismatch at {index}"
            );
        }
        for (index, (actual, expected)) in report.step.iter().zip(expected_step.iter()).enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "constrained KKT step index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .residual_multipliers
            .iter()
            .zip(expected_residual_multipliers.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "constrained KKT residual multiplier index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .active_inequality_multipliers
            .iter()
            .zip(expected_active_multipliers.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "constrained KKT active multiplier index {index} expected {expected}, got {actual}"
            );
        }
        for (index, (actual, expected)) in report
            .residual_multipliers
            .iter()
            .zip(report.linearized_residual.iter())
            .enumerate()
        {
            assert!(
                (actual - expected).abs() < 1.0e-7,
                "linearized residual multiplier index {index} expected {expected}, got {actual}"
            );
        }
        assert!(report.max_linearized_active_inequality_violation < 1.0e-9);
        for (index, value) in report
            .linearized_active_inequality_values
            .iter()
            .enumerate()
        {
            assert!(
                value.abs() < 1.0e-9,
                "linearized active inequality {index} expected closure, got {value}"
            );
        }

        let direct_multiplier_report = ocp
            .kkt_residual_multiplier_report(&report.residual_report, &report.residual_multipliers)
            .expect("direct constrained residual multipliers slice");
        let constrained_multiplier_report = ocp
            .sparse_constrained_kkt_residual_multiplier_report(&report)
            .expect("constrained sparse KKT residual multipliers slice");
        assert_eq!(constrained_multiplier_report, direct_multiplier_report);
        let constrained_costate_report = ocp
            .sparse_constrained_kkt_collocation_costate_estimate_report(&report)
            .expect("constrained sparse KKT costates evaluate");
        assert_eq!(constrained_costate_report.segment_count, 2);
        assert_eq!(constrained_costate_report.collocation_samples.len(), 4);
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: 1.0,
        };
        let constrained_pmp_costate_report = ocp
            .sparse_constrained_kkt_pmp_costate_convention_report(&report, pmp_convention)
            .expect("constrained sparse KKT PMP costates evaluate");
        assert_eq!(
            constrained_pmp_costate_report.sample_count,
            constrained_costate_report.collocation_samples.len()
        );
        let constrained_normalized_costate_report = ocp
            .sparse_constrained_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                &report,
                pmp_convention,
            )
            .expect("constrained sparse KKT normalized PMP costates evaluate");
        assert_eq!(
            constrained_normalized_costate_report.sample_count,
            constrained_costate_report.collocation_samples.len()
        );

        let mut constrained_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut constrained_hamiltonian_lagrange = affine_lgr_lagrange;
        let constrained_hamiltonian_report = ocp
            .sparse_constrained_kkt_hamiltonian_report(
                &decision,
                &report,
                &mut constrained_hamiltonian_dynamics,
                &mut constrained_hamiltonian_lagrange,
            )
            .expect("constrained sparse KKT Hamiltonian diagnostics evaluate");
        assert_eq!(
            constrained_hamiltonian_report.sample_count,
            constrained_costate_report.collocation_samples.len()
        );
        let mut constrained_pmp_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut constrained_pmp_hamiltonian_lagrange = affine_lgr_lagrange;
        let constrained_pmp_hamiltonian_report = ocp
            .sparse_constrained_kkt_pmp_hamiltonian_report(
                &decision,
                &report,
                pmp_convention,
                &mut constrained_pmp_hamiltonian_dynamics,
                &mut constrained_pmp_hamiltonian_lagrange,
            )
            .expect("constrained sparse KKT PMP Hamiltonian evaluates");
        assert_eq!(
            constrained_pmp_hamiltonian_report.sample_count,
            constrained_hamiltonian_report.sample_count
        );
        let mut constrained_normalized_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut constrained_normalized_hamiltonian_lagrange = affine_lgr_lagrange;
        let constrained_normalized_hamiltonian_report = ocp
            .sparse_constrained_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                &decision,
                &report,
                pmp_convention,
                &mut constrained_normalized_hamiltonian_dynamics,
                &mut constrained_normalized_hamiltonian_lagrange,
            )
            .expect("constrained sparse KKT normalized PMP Hamiltonian evaluates");
        assert_eq!(
            constrained_normalized_hamiltonian_report.sample_count,
            constrained_normalized_costate_report.sample_count
        );

        let mut malformed_report = report.clone();
        malformed_report.residual_multipliers.pop();
        assert!(
            ocp.sparse_constrained_kkt_residual_multiplier_report(&malformed_report)
                .is_err()
        );
        assert!(
            ocp.sparse_constrained_kkt_collocation_costate_estimate_report(&malformed_report)
                .is_err()
        );
        assert!(
            ocp.sparse_constrained_kkt_pmp_costate_convention_report(
                &malformed_report,
                pmp_convention,
            )
            .is_err()
        );
        assert!(
            ocp.sparse_constrained_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                &malformed_report,
                pmp_convention,
            )
            .is_err()
        );
        let mut malformed_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut malformed_hamiltonian_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.sparse_constrained_kkt_hamiltonian_report(
                &decision,
                &malformed_report,
                &mut malformed_hamiltonian_dynamics,
                &mut malformed_hamiltonian_lagrange,
            )
            .is_err()
        );
        let mut malformed_pmp_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut malformed_pmp_hamiltonian_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.sparse_constrained_kkt_pmp_hamiltonian_report(
                &decision,
                &malformed_report,
                pmp_convention,
                &mut malformed_pmp_hamiltonian_dynamics,
                &mut malformed_pmp_hamiltonian_lagrange,
            )
            .is_err()
        );
        let mut malformed_normalized_hamiltonian_dynamics = linear_lgr_dynamics;
        let mut malformed_normalized_hamiltonian_lagrange = affine_lgr_lagrange;
        assert!(
            ocp.sparse_constrained_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                &decision,
                &malformed_report,
                pmp_convention,
                &mut malformed_normalized_hamiltonian_dynamics,
                &mut malformed_normalized_hamiltonian_lagrange,
            )
            .is_err()
        );

        assert!(report.constrained_kkt.multiply_vector(&[0.0]).is_err());
        assert!(report.constrained_kkt.solve_ldlt(&[0.0]).is_err());

        let inactive_ocp = lgr_sparse_linear_ocp()
            .expect("LGR OCP builds")
            .with_hard_path_constraints(vec![
                LgrPseudospectralHardPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                    None,
                    Some(1.0),
                )
                .expect("inactive state hard path constraint"),
            ])
            .expect("hard path constraints attach");
        let mut inactive_dynamics = linear_lgr_dynamics;
        let mut inactive_lagrange = affine_lgr_lagrange;
        assert!(
            inactive_ocp
                .sparse_constrained_kkt_step_report(
                    &decision,
                    &mut inactive_dynamics,
                    &mut inactive_lagrange,
                    1.0e-6,
                    1.0e-9,
                    regularization,
                    1.0e-12,
                    1.0e-12,
                )
                .is_err()
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_sparse_kkt_sqp_solves_scalar_integrator_and_reports_final_multipliers() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds");
        let support_states = ocp.layout.segment_grids[0]
            .support_nodes
            .iter()
            .map(|node| test_physical_time(0.0, 1.0, *node))
            .collect::<Vec<_>>();
        let collocation_controls = vec![1.0; ocp.layout.segment_grids[0].collocation_count()];
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("exact decision packs");
        let mut initial = exact.clone();
        initial[1] += 0.1;
        initial[2] -= 0.05;
        initial[ocp.layout.state_variable_count] = 0.8;
        let mut dynamics = unit_control_lgr_dynamics;
        let mut lagrange = zero_lagrange;
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            kkt_regularization: 1.0e-3,
            ..SparseKktSqpSettings::default()
        };

        let report = ocp
            .solve_sparse_kkt_sqp(&initial, &mut dynamics, &mut lagrange, settings)
            .expect("LGR sparse KKT SQP solves scalar integrator");

        assert!(report.converged);
        assert!(!report.iterations.is_empty());
        assert!(report.residual_report.residual_norm <= settings.residual_tolerance);
        assert_eq!(report.decision_bounds.applied_bound_count, 0);
        assert!(report.max_decision_bound_violation <= settings.residual_tolerance);
        assert!(report.max_hard_path_violation <= settings.residual_tolerance);
        assert_eq!(
            report.final_kkt_step_report.residual_report,
            report.residual_report
        );
        assert_eq!(
            report.final_kkt_step_report.residual_multipliers.len(),
            report.residual_report.residuals.len()
        );
        assert!(report.final_constrained_kkt_step_report.is_none());
        let multiplier_report = ocp
            .sparse_kkt_residual_multiplier_report(&report.final_kkt_step_report)
            .expect("final LGR sparse-SQP multipliers slice");
        assert_eq!(
            multiplier_report.residual_multiplier_count,
            report.residual_report.residuals.len()
        );
        assert!(
            report
                .iterations
                .windows(2)
                .all(|window| window[1].accepted_residual_norm <= window[0].accepted_residual_norm)
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_sparse_kkt_sqp_enforces_hard_path_decision_bounds() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds")
        .with_hard_path_constraints(vec![
            LgrPseudospectralHardPathConstraint::new(
                0,
                0,
                LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                Some(0.0),
                Some(0.0),
            )
            .expect("state hard path constraint"),
        ])
        .expect("hard path constraints attach");
        let support_states = ocp.layout.segment_grids[0]
            .support_nodes
            .iter()
            .map(|node| test_physical_time(0.0, 1.0, *node))
            .collect::<Vec<_>>();
        let collocation_controls = vec![1.0; ocp.layout.segment_grids[0].collocation_count()];
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("exact decision packs");
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            kkt_regularization: 1.0e-3,
            ..SparseKktSqpSettings::default()
        };
        let mut dynamics = unit_control_lgr_dynamics;
        let mut lagrange = zero_lagrange;

        let report = ocp
            .solve_sparse_kkt_sqp(&exact, &mut dynamics, &mut lagrange, settings)
            .expect("bounded LGR sparse KKT SQP solves");

        assert!(report.converged);
        assert_eq!(report.decision_bounds.applied_bound_count, 2);
        assert_eq!(report.decision_bounds.constrained_variable_count, 1);
        assert!(report.max_decision_bound_violation <= settings.residual_tolerance);
        assert!(report.max_hard_path_violation <= settings.residual_tolerance);
        let constrained_report = report
            .final_constrained_kkt_step_report
            .as_ref()
            .expect("final constrained LGR sparse-KKT step report");
        assert_eq!(constrained_report.residual_report, report.residual_report);
        assert_eq!(
            constrained_report
                .constrained_kkt
                .active_inequality_row_count,
            2
        );
        assert_eq!(constrained_report.active_inequality_multipliers.len(), 2);
        assert!(
            constrained_report.max_linearized_active_inequality_violation
                <= settings.residual_tolerance
        );
        let constrained_multiplier_report = ocp
            .sparse_constrained_kkt_residual_multiplier_report(constrained_report)
            .expect("final constrained LGR multipliers slice");
        assert_eq!(
            constrained_multiplier_report.residual_multiplier_count,
            report.residual_report.residuals.len()
        );

        let mut outside_initial = exact;
        outside_initial[0] = 0.1;
        let mut dynamics = unit_control_lgr_dynamics;
        let mut lagrange = zero_lagrange;
        assert!(
            ocp.solve_sparse_kkt_sqp(&outside_initial, &mut dynamics, &mut lagrange, settings)
                .is_err()
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_sparse_kkt_sqp_preserves_nonlinear_hard_path_feasibility() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds")
        .with_nonlinear_hard_path_constraints(vec![
            LgrPseudospectralNonlinearHardPathConstraint::new(
                0,
                0,
                vec![Some(0.9)],
                vec![Some(1.1)],
            )
            .expect("nonlinear hard path constraint"),
        ])
        .expect("nonlinear hard path constraints attach");
        let support_states = ocp.layout.segment_grids[0]
            .support_nodes
            .iter()
            .map(|node| test_physical_time(0.0, 1.0, *node))
            .collect::<Vec<_>>();
        let collocation_controls = vec![1.0; ocp.layout.segment_grids[0].collocation_count()];
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("exact decision packs");
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            kkt_regularization: 1.0e-3,
            ..SparseKktSqpSettings::default()
        };
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];
        let mut dynamics = unit_control_lgr_dynamics;
        let mut lagrange = zero_lagrange;
        let mut path_function = control_lgr_path_function;

        let report = ocp
            .solve_sparse_kkt_sqp_with_nonlinear_hard_path_constraints(
                &exact,
                &lower,
                &upper,
                &mut dynamics,
                &mut lagrange,
                &mut path_function,
                settings,
            )
            .expect("nonlinear-feasible LGR sparse KKT SQP solves");

        assert!(report.converged);
        assert!(report.max_decision_bound_violation <= settings.residual_tolerance);
        assert!(report.max_hard_path_violation <= settings.residual_tolerance);
        assert!(report.final_constrained_kkt_step_report.is_none());
        assert!(report.max_nonlinear_hard_path_violation <= settings.residual_tolerance);
        assert!(
            report
                .nonlinear_hard_path_report
                .as_ref()
                .expect("nonlinear report")
                .max_violation
                <= settings.residual_tolerance
        );

        let mut dynamics = unit_control_lgr_dynamics;
        let mut lagrange = zero_lagrange;
        assert!(
            ocp.solve_sparse_kkt_sqp(&exact, &mut dynamics, &mut lagrange, settings)
                .is_err()
        );

        let mut outside_initial = exact;
        outside_initial[ocp.layout.state_variable_count] = 1.2;
        let mut dynamics = unit_control_lgr_dynamics;
        let mut lagrange = zero_lagrange;
        let mut path_function = control_lgr_path_function;
        assert!(
            ocp.solve_sparse_kkt_sqp_with_nonlinear_hard_path_constraints(
                &outside_initial,
                &lower,
                &upper,
                &mut dynamics,
                &mut lagrange,
                &mut path_function,
                settings,
            )
            .is_err()
        );
    }

    #[test]
    fn lgr_fixed_mesh_ocp_merges_hard_path_bounds_for_dense_solver() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_hard_path_constraints(vec![
                LgrPseudospectralHardPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                    Some(0.0),
                    Some(0.0),
                )
                .expect("state hard path constraint"),
                LgrPseudospectralHardPathConstraint::new(
                    0,
                    1,
                    LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
                    Some(0.5),
                    Some(1.5),
                )
                .expect("control hard path constraint"),
            ])
            .expect("hard path constraints attach");
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];

        let report = ocp
            .decision_bounds_with_hard_path_constraints(&lower, &upper)
            .expect("decision bounds merge");

        let state_column = lgr_scalar_decision_index(
            &ocp.layout,
            0,
            0,
            LgrPseudospectralScalarVariable::StateComponent { component: 0 },
        )
        .expect("state column");
        let control_column = lgr_scalar_decision_index(
            &ocp.layout,
            0,
            1,
            LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
        )
        .expect("control column");
        assert_eq!(report.variable_count, ocp.layout.variable_count);
        assert_eq!(report.applied_bound_count, 4);
        assert_eq!(report.constrained_variable_count, 2);
        assert_eq!(report.lower_bounds[state_column], 0.0);
        assert_eq!(report.upper_bounds[state_column], 0.0);
        assert_eq!(report.lower_bounds[control_column], 0.5);
        assert_eq!(report.upper_bounds[control_column], 1.5);

        let mut violating_decision = vec![0.0; ocp.layout.variable_count];
        violating_decision[control_column] = 2.0;
        assert!(
            (report
                .max_violation(&violating_decision)
                .expect("max violation")
                - 0.5)
                .abs()
                < 1.0e-14
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_dense_sqp_wrapper_restores_scalar_integrator_and_is_repeat_deterministic()
    {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds")
        .with_hard_path_constraints(vec![
            LgrPseudospectralHardPathConstraint::new(
                0,
                0,
                LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                Some(0.0),
                Some(0.0),
            )
            .expect("state hard path constraint"),
        ])
        .expect("hard path constraints attach");
        let support_states = ocp.layout.segment_grids[0]
            .support_nodes
            .iter()
            .map(|node| test_physical_time(0.0, 1.0, *node))
            .collect::<Vec<_>>();
        let collocation_controls = vec![1.0; ocp.layout.segment_grids[0].collocation_count()];
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut initial = exact.clone();
        initial[1] += 0.1;
        initial[2] -= 0.05;
        initial[ocp.layout.state_variable_count - 1] -= 0.1;
        let mut lower = vec![-10.0; ocp.layout.variable_count];
        let mut upper = vec![10.0; ocp.layout.variable_count];
        for index in ocp.layout.state_variable_count..ocp.layout.variable_count {
            lower[index] = 1.0;
            upper[index] = 1.0;
        }
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 12,
            residual_tolerance: 1.0e-9,
            step_tolerance: 1.0e-12,
            regularization: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let first = ocp
            .solve_dense_sqp(
                &initial,
                &lower,
                &upper,
                &solver,
                unit_control_lgr_dynamics,
                zero_lagrange,
            )
            .expect("first LGR dense SQP solve succeeds");
        let second = ocp
            .solve_dense_sqp(
                &initial,
                &lower,
                &upper,
                &solver,
                unit_control_lgr_dynamics,
                zero_lagrange,
            )
            .expect("second LGR dense SQP solve succeeds");

        assert!(first.sqp_solution.converged);
        assert!(first.residual_report.residual_norm < 1.0e-9);
        assert!(first.max_decision_bound_violation < 1.0e-10);
        assert!(first.max_hard_path_violation < 1.0e-10);
        assert_eq!(first.decision_bounds.applied_bound_count, 2);
        assert_eq!(first.sqp_solution.variables, second.sqp_solution.variables);
        assert_eq!(first.sqp_solution.residual, second.sqp_solution.residual);
        assert_eq!(
            first.sqp_solution.residual_norm.to_bits(),
            second.sqp_solution.residual_norm.to_bits()
        );
        assert_eq!(first.residual_report, second.residual_report);
        let max_solution_delta = first
            .sqp_solution
            .variables
            .iter()
            .zip(exact.iter())
            .map(|(actual, expected)| (actual - expected).abs())
            .fold(0.0_f64, f64::max);
        assert!(max_solution_delta < 1.0e-7);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_dense_sqp_sparse_kkt_certificate_attaches_to_optimized_reference_iload()
    -> Result<(), TrajoptError> {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            3,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0)?,
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0)?,
                LgrPseudospectralBoundaryConstraint::new(0, 1, 10.0)?,
                LgrPseudospectralBoundaryConstraint::new(1, 1, 10.0)?,
                LgrPseudospectralBoundaryConstraint::new(0, 2, 0.1)?,
                LgrPseudospectralBoundaryConstraint::new(1, 2, 0.1)?,
            ],
        )?;
        let mut support_states = Vec::new();
        for node in &ocp.layout.segment_grids[0].support_nodes {
            let time_s = test_physical_time(0.0, 1.0, *node);
            support_states.extend_from_slice(&[time_s, 10.0, 0.1]);
        }
        let collocation_controls = vec![1.0; ocp.layout.segment_grids[0].collocation_count()];
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let mut initial = exact.clone();
        initial[3] += 0.05;
        initial[7] -= 0.05;
        let mut lower = vec![-20.0; ocp.layout.variable_count];
        let mut upper = vec![20.0; ocp.layout.variable_count];
        for index in ocp.layout.state_variable_count..ocp.layout.variable_count {
            lower[index] = 1.0;
            upper[index] = 1.0;
        }
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 12,
            residual_tolerance: 1.0e-10,
            step_tolerance: 1.0e-12,
            regularization: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };
        let sqp_report = ocp.solve_dense_sqp(
            &initial,
            &lower,
            &upper,
            &solver,
            three_state_lgr_reference_dynamics,
            zero_lagrange,
        )?;
        let mut certificate_dynamics = three_state_lgr_reference_dynamics;
        let mut certificate_lagrange = zero_lagrange;
        let certificate_report = ocp
            .dense_sqp_pmp_kkt_quadrature_weight_normalized_certificate_report(
                &sqp_report,
                crate::pmp::PmpDirectCostateConvention::default(),
                1.0e-9,
                &mut certificate_dynamics,
                &mut certificate_lagrange,
                1.0e-6,
                1.0e-12,
                2.5e-1,
                1.0e-12,
            )?;
        let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
        let pmp_costates = certificate_report
            .costate_convention_report
            .pmp_costates
            .clone();
        let certificate_report = certificate_report.with_pmp_costate_comparison(
            &pmp_times_s,
            &pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )?;
        let mut stationarity_dynamics = three_state_lgr_reference_dynamics;
        let mut stationarity_lagrange = zero_lagrange;
        let certificate_report = certificate_report.with_pmp_control_stationarity(
            crate::pmp::PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            &mut stationarity_dynamics,
            &mut stationarity_lagrange,
        )?;

        assert!(sqp_report.sqp_solution.converged);
        assert!(sqp_report.residual_report.residual_norm < 1.0e-10);
        assert_eq!(
            certificate_report
                .sparse_kkt_step_report
                .residual_multipliers
                .len(),
            sqp_report.residual_report.residuals.len()
        );
        assert_eq!(
            certificate_report.costate_convention_report.sample_count,
            ocp.layout.segment_grids[0].collocation_count()
        );
        assert_eq!(
            certificate_report.hamiltonian_report.sample_count,
            ocp.layout.segment_grids[0].collocation_count()
        );
        assert!(
            certificate_report
                .costate_comparison_report
                .as_ref()
                .expect("costate comparison attached")
                .within_tolerance
        );
        assert!(
            certificate_report
                .control_stationarity_report
                .as_ref()
                .expect("stationarity attached")
                .within_tolerance
        );
        assert_eq!(
            certificate_report
                .optimality_certificate
                .max_costate_norm_difference,
            Some(0.0)
        );

        let payload = ocp.optimized_reference_iload_payload_with_optimality_certificate(
            &sqp_report.sqp_solution.variables,
            LgrPseudospectralReferenceProfileMapping::new(0, 1, 2),
            &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
            202,
            "openbmp-trajopt-lgr-dense-sqp-test".to_owned(),
            SynthesisMetadata {
                scenario_digest: "sha256:lgr-dense-sqp-pmp-kkt-certificate".to_owned(),
                source_revision: "test".to_owned(),
                method: "lgr_dense_sqp_sparse_kkt_pmp_certificate".to_owned(),
            },
            Vec::new(),
            Vec::new(),
            certificate_report.optimality_certificate.clone(),
        )?;
        let decoded =
            crate::iload::decode_iload_payload(&crate::iload::encode_iload_payload(&payload)?)?;
        assert_eq!(decoded.metadata.method, payload.metadata.method);
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate_report.optimality_certificate
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .max_control_stationarity_gradient_norm,
            Some(
                certificate_report
                    .control_stationarity_report
                    .as_ref()
                    .expect("stationarity attached")
                    .max_gradient_norm
            )
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_sparse_kkt_sqp_pmp_kkt_certificate_attaches_to_optimized_reference_iload()
    -> Result<(), TrajoptError> {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            3,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0)?,
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0)?,
                LgrPseudospectralBoundaryConstraint::new(0, 1, 10.0)?,
                LgrPseudospectralBoundaryConstraint::new(1, 1, 10.0)?,
                LgrPseudospectralBoundaryConstraint::new(0, 2, 0.1)?,
                LgrPseudospectralBoundaryConstraint::new(1, 2, 0.1)?,
            ],
        )?
        .with_hard_path_constraints(vec![LgrPseudospectralHardPathConstraint::new(
            0,
            0,
            LgrPseudospectralScalarVariable::StateComponent { component: 0 },
            None,
            Some(0.0),
        )?])?;
        let mut support_states = Vec::new();
        for node in &ocp.layout.segment_grids[0].support_nodes {
            let time_s = test_physical_time(0.0, 1.0, *node);
            support_states.extend_from_slice(&[time_s, 10.0, 0.1]);
        }
        let collocation_controls = vec![1.0; ocp.layout.segment_grids[0].collocation_count()];
        let exact = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let mut lower = vec![-20.0; ocp.layout.variable_count];
        let mut upper = vec![20.0; ocp.layout.variable_count];
        for index in ocp.layout.state_variable_count..ocp.layout.variable_count {
            lower[index] = 1.0;
            upper[index] = 1.0;
        }
        let settings = SparseKktSqpSettings {
            max_iterations: 12,
            residual_tolerance: 1.0e-10,
            step_tolerance: 1.0e-12,
            kkt_regularization: 2.5e-1,
            ..SparseKktSqpSettings::default()
        };
        let decision_bounds = ocp.decision_bounds_with_hard_path_constraints(&lower, &upper)?;
        let max_decision_bound_violation = decision_bounds.max_violation(&exact)?;
        let hard_path_inequalities = ocp.hard_path_inequality_report()?;
        let max_hard_path_violation = hard_path_inequalities.max_violation(&exact)?;
        let residual_report =
            ocp.residual_report(&exact, three_state_lgr_reference_dynamics, zero_lagrange)?;
        let mut kkt_dynamics = three_state_lgr_reference_dynamics;
        let mut kkt_lagrange = zero_lagrange;
        let final_kkt_step_report = ocp.sparse_augmented_kkt_step_report(
            &exact,
            &mut kkt_dynamics,
            &mut kkt_lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
        )?;
        let mut constrained_dynamics = three_state_lgr_reference_dynamics;
        let mut constrained_lagrange = zero_lagrange;
        let final_constrained_kkt_step_report = ocp.sparse_constrained_kkt_step_report(
            &exact,
            &mut constrained_dynamics,
            &mut constrained_lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
            settings.residual_tolerance,
        )?;
        let sqp_report = LgrPseudospectralFixedMeshSparseKktSqpReport {
            decision: exact.clone(),
            residual_report,
            final_kkt_step_report,
            final_constrained_kkt_step_report: Some(final_constrained_kkt_step_report),
            decision_bounds,
            max_decision_bound_violation,
            max_hard_path_violation,
            nonlinear_hard_path_report: None,
            max_nonlinear_hard_path_violation: 0.0,
            iterations: Vec::new(),
            converged: true,
        };
        let mut certificate_dynamics = three_state_lgr_reference_dynamics;
        let mut certificate_lagrange = zero_lagrange;
        let certificate_report = ocp
            .sparse_kkt_sqp_pmp_kkt_quadrature_weight_normalized_certificate_report(
                &sqp_report,
                crate::pmp::PmpDirectCostateConvention::default(),
                1.0e-9,
                &mut certificate_dynamics,
                &mut certificate_lagrange,
            )?;
        let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
        let pmp_costates = certificate_report
            .costate_convention_report
            .pmp_costates
            .clone();
        let certificate_report = certificate_report.with_pmp_costate_comparison(
            &pmp_times_s,
            &pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-12,
                ..Default::default()
            },
        )?;
        let mut stationarity_dynamics = three_state_lgr_reference_dynamics;
        let mut stationarity_lagrange = zero_lagrange;
        let certificate_report = certificate_report.with_pmp_control_stationarity(
            crate::pmp::PmpHamiltonianControlStationaritySettings {
                gradient_tolerance: 1.0e-9,
                ..Default::default()
            },
            &mut stationarity_dynamics,
            &mut stationarity_lagrange,
        )?;

        assert!(sqp_report.converged);
        assert!(sqp_report.residual_report.residual_norm < settings.residual_tolerance);
        assert!(sqp_report.final_constrained_kkt_step_report.is_some());
        assert!(certificate_report.used_constrained_kkt_step_report);
        assert!(
            certificate_report
                .final_constrained_kkt_step_report
                .as_ref()
                .expect("constrained report")
                .constrained_kkt
                .active_inequality_row_count
                > 0
        );
        assert_eq!(
            certificate_report.costate_convention_report.sample_count,
            ocp.layout.segment_grids[0].collocation_count()
        );
        assert_eq!(
            certificate_report.hamiltonian_report.sample_count,
            ocp.layout.segment_grids[0].collocation_count()
        );
        assert!(
            certificate_report
                .costate_comparison_report
                .as_ref()
                .expect("costate comparison attached")
                .within_tolerance
        );
        assert!(
            certificate_report
                .control_stationarity_report
                .as_ref()
                .expect("stationarity attached")
                .within_tolerance
        );
        assert_eq!(
            certificate_report
                .optimality_certificate
                .max_costate_norm_difference,
            Some(0.0)
        );

        let payload = ocp.optimized_reference_iload_payload_with_optimality_certificate(
            &sqp_report.decision,
            LgrPseudospectralReferenceProfileMapping::new(0, 1, 2),
            &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
            212,
            "openbmp-trajopt-lgr-sparse-sqp-test".to_owned(),
            SynthesisMetadata {
                scenario_digest: "sha256:lgr-sparse-sqp-pmp-kkt-certificate".to_owned(),
                source_revision: "test".to_owned(),
                method: "lgr_sparse_sqp_pmp_kkt_certificate".to_owned(),
            },
            Vec::new(),
            Vec::new(),
            certificate_report.optimality_certificate.clone(),
        )?;
        let decoded =
            crate::iload::decode_iload_payload(&crate::iload::encode_iload_payload(&payload)?)?;
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate_report.optimality_certificate
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .max_control_stationarity_gradient_norm,
            Some(
                certificate_report
                    .control_stationarity_report
                    .as_ref()
                    .expect("stationarity attached")
                    .max_gradient_norm
            )
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_scalar_integrator_min_energy_benchmark_matches_analytic_solution() {
        let table = LgrScalarIntegratorBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = unit_interval_lgr_integrator_ocp(&table)
            .expect("LGR scalar-integrator OCP builds")
            .with_quadratic_control_costs(vec![
                LgrPseudospectralQuadraticControlCost::new(0, 1.0).expect("control cost"),
            ])
            .expect("control costs attach");
        let mut dynamics = unit_control_lgr_dynamics;

        let report = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)
            .expect("LGR scalar-integrator dense QP solves");
        let cost = lgr_quadratic_control_cost(&ocp, &report.qp_solution.primal)
            .expect("LGR scalar-integrator cost evaluates");

        assert!(report.equality_residual_norm <= table.max_equality_residual_norm);
        assert!(report.max_hard_path_violation <= 1.0e-10);
        assert!((cost - table.expected_cost).abs() <= table.cost_tolerance);
        assert!(
            (report.residual_report.quadratic_control_cost - table.expected_cost).abs()
                <= table.cost_tolerance
        );
        assert_lgr_scalar_integrator_analytic_solution(&ocp, &report.qp_solution.primal, &table);
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_bryson_denham_dense_quadratic_qp_wrapper_matches_active_arc() {
        let table = LgrBrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp =
            lgr_bryson_denham_quadratic_ocp(table.problem).expect("LGR Bryson-Denham OCP builds");
        let mut dynamics = lgr_bryson_denham_dynamics;

        let report = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)
            .expect("LGR Bryson-Denham dense QP solves");
        let cost = lgr_quadratic_control_cost(&ocp, &report.qp_solution.primal)
            .expect("LGR Bryson-Denham cost evaluates");

        assert!(
            report.equality_residual_norm <= table.problem.max_residual_norm,
            "LGR Bryson-Denham equality norm {} exceeded {}",
            report.equality_residual_norm,
            table.problem.max_residual_norm
        );
        assert!(
            report.max_hard_path_violation <= table.problem.max_path_violation,
            "LGR Bryson-Denham path violation {} exceeded {}",
            report.max_hard_path_violation,
            table.problem.max_path_violation
        );
        assert!(
            (cost - table.problem.expected_cost).abs() <= table.problem.cost_tolerance,
            "LGR Bryson-Denham cost {} expected {}",
            cost,
            table.problem.expected_cost
        );
        assert!(
            (report.residual_report.quadratic_control_cost - table.problem.expected_cost).abs()
                <= table.problem.cost_tolerance
        );
        assert_lgr_bryson_denham_active_arc_solution(
            &ocp,
            &report.qp_solution.primal,
            table.problem,
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_bryson_denham_dense_solution_feeds_sparse_constrained_kkt_report()
    -> Result<(), TrajoptError> {
        let table = LgrBrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = lgr_bryson_denham_quadratic_ocp(table.problem)?;
        let mut dynamics = lgr_bryson_denham_dynamics;
        let dense_report =
            ocp.solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)?;
        let settings = SparseKktSqpSettings {
            residual_tolerance: table.problem.max_path_violation,
            ..SparseKktSqpSettings::default()
        };
        let mut sparse_dynamics = lgr_bryson_denham_dynamics;
        let mut sparse_lagrange = zero_lagrange;

        let sparse_report = ocp.sparse_constrained_kkt_step_report(
            &dense_report.qp_solution.primal,
            &mut sparse_dynamics,
            &mut sparse_lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
            settings.residual_tolerance,
        )?;
        let equality_row_count = sparse_report.residual_report.differential_residual_count
            + sparse_report.residual_report.continuity_residual_count
            + sparse_report.residual_report.boundary_residual_count;
        let equality_residual_norm =
            norm(&sparse_report.residual_report.residuals[..equality_row_count]);
        let cost = lgr_quadratic_control_cost(&ocp, &dense_report.qp_solution.primal)?;
        let constrained_multiplier_report =
            ocp.sparse_constrained_kkt_residual_multiplier_report(&sparse_report)?;

        assert!(
            equality_residual_norm <= table.problem.max_residual_norm,
            "LGR Bryson-Denham sparse-KKT equality norm {equality_residual_norm} exceeded {}",
            table.problem.max_residual_norm
        );
        assert!(
            sparse_report
                .hard_path_inequality_report
                .max_violation(&dense_report.qp_solution.primal)?
                <= table.problem.max_path_violation
        );
        assert!(
            (cost - table.problem.expected_cost).abs() <= table.problem.cost_tolerance,
            "LGR Bryson-Denham sparse-KKT cost {} expected {}",
            cost,
            table.problem.expected_cost
        );
        assert!(
            sparse_report.constrained_kkt.active_inequality_row_count > 0,
            "LGR Bryson-Denham sparse-KKT report should retain active path rows"
        );
        assert_eq!(
            sparse_report.active_inequality_multipliers.len(),
            sparse_report.constrained_kkt.active_inequality_row_count
        );
        assert!(
            sparse_report.max_linearized_active_inequality_violation
                <= table.problem.max_path_violation
        );
        assert_eq!(
            constrained_multiplier_report.residual_multiplier_count,
            sparse_report.residual_report.residuals.len()
        );
        assert_eq!(
            constrained_multiplier_report.objective_multiplier_count,
            sparse_report.residual_report.objective_residual_count
        );
        assert_lgr_bryson_denham_active_arc_solution(
            &ocp,
            &dense_report.qp_solution.primal,
            table.problem,
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_bryson_denham_sparse_constrained_kkt_feeds_pmp_diagnostics()
    -> Result<(), TrajoptError> {
        let table = LgrBrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = lgr_bryson_denham_quadratic_ocp(table.problem)?;
        let mut dynamics = lgr_bryson_denham_dynamics;
        let dense_report =
            ocp.solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)?;
        let settings = SparseKktSqpSettings {
            residual_tolerance: table.problem.max_path_violation,
            ..SparseKktSqpSettings::default()
        };
        let mut sparse_dynamics = lgr_bryson_denham_dynamics;
        let mut sparse_lagrange = zero_lagrange;
        let sparse_report = ocp.sparse_constrained_kkt_step_report(
            &dense_report.qp_solution.primal,
            &mut sparse_dynamics,
            &mut sparse_lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
            settings.residual_tolerance,
        )?;
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: 1.0,
        };

        let costate_report =
            ocp.sparse_constrained_kkt_collocation_costate_estimate_report(&sparse_report)?;
        let raw_pmp_costate_report = ocp
            .sparse_constrained_kkt_pmp_costate_convention_report(&sparse_report, pmp_convention)?;
        let normalized_pmp_costate_report = ocp
            .sparse_constrained_kkt_pmp_quadrature_weight_normalized_costate_convention_report(
                &sparse_report,
                pmp_convention,
            )?;
        let mut hamiltonian_dynamics = lgr_bryson_denham_dynamics;
        let mut hamiltonian_lagrange = zero_lagrange;
        let hamiltonian_report = ocp
            .sparse_constrained_kkt_pmp_quadrature_weight_normalized_hamiltonian_report(
                &dense_report.qp_solution.primal,
                &sparse_report,
                pmp_convention,
                &mut hamiltonian_dynamics,
                &mut hamiltonian_lagrange,
            )?;

        assert!(sparse_report.constrained_kkt.active_inequality_row_count > 0);
        assert_eq!(
            costate_report.collocation_samples.len(),
            table.problem.segment_count * table.problem.lgr_collocation_count
        );
        assert_eq!(
            raw_pmp_costate_report.sample_count,
            costate_report.collocation_samples.len()
        );
        assert_eq!(
            normalized_pmp_costate_report.sample_count,
            costate_report.collocation_samples.len()
        );
        assert_eq!(normalized_pmp_costate_report.state_dimension, 2);
        assert_eq!(
            hamiltonian_report.sample_count,
            normalized_pmp_costate_report.sample_count
        );
        assert_eq!(hamiltonian_report.state_dimension, 2);
        assert_eq!(hamiltonian_report.control_dimension, 1);
        assert!(hamiltonian_report.max_abs_hamiltonian.is_finite());
        assert!(hamiltonian_report.max_abs_hamiltonian_drift.is_finite());
        assert!(
            normalized_pmp_costate_report
                .pmp_costates
                .iter()
                .all(|value| value.is_finite())
        );
        assert_lgr_bryson_denham_active_arc_solution(
            &ocp,
            &dense_report.qp_solution.primal,
            table.problem,
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_bryson_denham_sparse_constrained_kkt_certificate_attaches_to_iload()
    -> Result<(), TrajoptError> {
        let table = LgrBrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = lgr_bryson_denham_quadratic_ocp(table.problem)?;
        let mut dynamics = lgr_bryson_denham_dynamics;
        let dense_report =
            ocp.solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)?;
        let settings = SparseKktSqpSettings {
            residual_tolerance: table.problem.max_path_violation,
            ..SparseKktSqpSettings::default()
        };
        let mut sparse_dynamics = lgr_bryson_denham_dynamics;
        let mut sparse_lagrange = zero_lagrange;
        let sparse_report = ocp.sparse_constrained_kkt_step_report(
            &dense_report.qp_solution.primal,
            &mut sparse_dynamics,
            &mut sparse_lagrange,
            settings.finite_difference_step,
            settings.jacobian_drop_tolerance,
            settings.kkt_regularization,
            settings.kkt_drop_tolerance,
            settings.residual_tolerance,
        )?;
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: 1.0,
        };
        let mut certificate_dynamics = lgr_bryson_denham_dynamics;
        let mut certificate_lagrange = zero_lagrange;
        let certificate_report = ocp
            .sparse_constrained_kkt_pmp_kkt_quadrature_weight_normalized_certificate_report(
                &dense_report.qp_solution.primal,
                &sparse_report,
                pmp_convention,
                1.0e12,
                &mut certificate_dynamics,
                &mut certificate_lagrange,
            )?;
        let pmp_times_s = certificate_report.costate_convention_report.times_s.clone();
        let pmp_costates = certificate_report
            .costate_convention_report
            .pmp_costates
            .clone();
        let certificate_report = certificate_report.with_pmp_costate_comparison(
            &pmp_times_s,
            &pmp_costates,
            crate::pmp::PmpCostateComparisonSettings::default(),
        )?;
        let payload = ocp.optimized_reference_iload_payload_with_optimality_certificate(
            &dense_report.qp_solution.primal,
            LgrPseudospectralReferenceProfileMapping::new(0, 1, 1),
            &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
            78,
            "openbmp-trajopt-lgr-bryson-denham-test".to_string(),
            SynthesisMetadata {
                scenario_digest: "sha256:lgr-bryson-denham-constrained-kkt-certificate".to_string(),
                source_revision: "test-revision".to_string(),
                method: "lgr_bryson_denham_constrained_kkt_pmp_certificate".to_string(),
            },
            Vec::new(),
            Vec::new(),
            certificate_report.optimality_certificate.clone(),
        )?;
        let decoded = crate::iload::decode_iload_payload(
            &crate::iload::encode_iload_payload(&payload)
                .expect("certified LGR Bryson-Denham payload encodes"),
        )?;

        assert!(
            certificate_report
                .constrained_kkt_step_report
                .constrained_kkt
                .active_inequality_row_count
                > 0
        );
        assert_eq!(
            certificate_report.costate_convention_report.sample_count,
            table.problem.segment_count * table.problem.lgr_collocation_count
        );
        assert_eq!(
            certificate_report.hamiltonian_report.sample_count,
            certificate_report.costate_convention_report.sample_count
        );
        assert_eq!(
            certificate_report
                .costate_comparison_report
                .as_ref()
                .expect("comparison present")
                .max_costate_norm_difference,
            0.0
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate_report.optimality_certificate
        );
        assert_eq!(
            decoded
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .evidence_kind,
            crate::iload::ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt
        );
        assert_lgr_bryson_denham_active_arc_solution(
            &ocp,
            &dense_report.qp_solution.primal,
            table.problem,
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_bryson_denham_benchmark_qp_is_repeat_deterministic() {
        let table = LgrBrysonDenhamBenchmarkTable::parse().expect("benchmark table parses");
        let ocp =
            lgr_bryson_denham_quadratic_ocp(table.problem).expect("LGR Bryson-Denham OCP builds");
        let mut first_dynamics = lgr_bryson_denham_dynamics;
        let mut second_dynamics = lgr_bryson_denham_dynamics;

        let first = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut first_dynamics, 1.0e-6, 1.0e-12)
            .expect("first LGR Bryson-Denham QP solves");
        let second = ocp
            .solve_dense_linearized_quadratic_control_qp(&mut second_dynamics, 1.0e-6, 1.0e-12)
            .expect("second LGR Bryson-Denham QP solves");

        assert_eq!(first.qp_solution.primal, second.qp_solution.primal);
        assert_eq!(
            first.qp_solution.equality_dual,
            second.qp_solution.equality_dual
        );
        assert_eq!(
            first.qp_solution.inequality_dual,
            second.qp_solution.inequality_dual
        );
        assert_eq!(
            first.residual_report.residuals,
            second.residual_report.residuals
        );
        assert_lgr_bryson_denham_active_arc_solution(
            &ocp,
            &first.qp_solution.primal,
            table.problem,
        );
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_dense_qp_duals_feed_pmp_costate_convention_bridge() -> Result<(), TrajoptError> {
        let table = LgrScalarIntegratorBenchmarkTable::parse().expect("benchmark table parses");
        let ocp = unit_interval_lgr_integrator_ocp(&table)?.with_quadratic_control_costs(vec![
            LgrPseudospectralQuadraticControlCost::new(0, 1.0)?,
        ])?;
        let mut dynamics = unit_control_lgr_dynamics;

        let report =
            ocp.solve_dense_linearized_quadratic_control_qp(&mut dynamics, 1.0e-6, 1.0e-12)?;
        let multiplier_report = ocp.dense_qp_kkt_residual_multiplier_report(&report)?;
        let costate_report = ocp.dense_qp_kkt_collocation_costate_estimate_report(&report)?;
        let pmp_solution = crate::pmp::solve_pmp_scalar_min_energy_shooting(
            crate::pmp::PmpScalarMinEnergyProblem {
                initial_time_s: 0.0,
                final_time_s: 1.0,
                initial_state: 0.0,
                terminal_state: 1.0,
                control_weight: 2.0,
            },
            crate::pmp::PmpScalarMinEnergyShootingSettings {
                sample_count: costate_report.collocation_samples.len(),
                ..Default::default()
            },
        )?;
        let pmp_convention = crate::pmp::PmpDirectCostateConvention {
            raw_to_pmp_scale: -1.0,
        };
        let convention_report = ocp
            .dense_qp_pmp_quadrature_weight_normalized_costate_convention_report(
                &report,
                pmp_convention,
            )?;
        let pmp_costates = vec![pmp_solution.initial_costate; convention_report.sample_count];
        let comparison = crate::pmp::compare_pmp_costate_profiles(
            costate_report.state_dimension,
            &convention_report.times_s,
            &pmp_costates,
            &convention_report.times_s,
            &convention_report.pmp_costates,
            crate::pmp::PmpCostateComparisonSettings {
                costate_tolerance: 1.0e-7,
                ..Default::default()
            },
        )?;
        let mut hamiltonian_dynamics = unit_control_lgr_dynamics;
        let mut hamiltonian_lagrange =
            |_: f64, _: &[f64], control: &[f64]| Ok(control[0] * control[0]);
        let hamiltonian_report = ocp.dense_qp_pmp_quadrature_weight_normalized_hamiltonian_report(
            &report,
            pmp_convention,
            &mut hamiltonian_dynamics,
            &mut hamiltonian_lagrange,
        )?;

        assert!(pmp_solution.converged);
        assert_eq!(
            report.equality_row_count,
            report.residual_report.differential_residual_count
                + report.residual_report.continuity_residual_count
                + report.residual_report.boundary_residual_count
        );
        assert_eq!(
            multiplier_report.residual_multiplier_count,
            report.residual_report.residuals.len()
        );
        assert_eq!(multiplier_report.path_multiplier_count, 0);
        assert_eq!(
            multiplier_report.objective_multipliers,
            vec![0.0; multiplier_report.objective_multiplier_count]
        );
        assert_eq!(
            costate_report.collocation_samples.len(),
            table.segment_count * table.collocation_count
        );
        assert_eq!(
            convention_report.sample_count,
            costate_report.collocation_samples.len()
        );
        assert!(convention_report.max_abs_raw_costate > 0.0);
        assert!(comparison.within_tolerance);
        assert!(hamiltonian_report.max_abs_hamiltonian_drift < 1.0e-7);
        let certificate = crate::iload::ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            crate::iload::ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt,
            &hamiltonian_report,
            1.0e-7,
        )?
        .with_costate_comparison(&comparison)?;
        assert_eq!(certificate.sample_count, convention_report.sample_count);
        assert_eq!(
            certificate.max_costate_norm_difference,
            Some(comparison.max_costate_norm_difference)
        );
        let certified_payload = crate::iload::ILoadPayload {
            header: crate::iload::ILoadHeader::new_with_objective(
                &TerminalCondition::ApogeeRadius { radius_m: 1.0 },
                crate::iload::ILoadObjectiveClass::OptimizedReference,
                102,
                "openbmp-trajopt-lgr-dense-qp-test".to_owned(),
            ),
            metadata: crate::iload::SynthesisMetadata {
                scenario_digest: "sha256:lgr-dense-qp-pmp-certificate".to_owned(),
                source_revision: "test".to_owned(),
                method: "lgr_dense_qp_pmp_certificate".to_owned(),
            },
            event_bindings_postcard: Vec::new(),
            gain_tables: Vec::new(),
            reference_profile: hamiltonian_report
                .samples
                .iter()
                .map(|sample| crate::iload::ReferenceProfileSample {
                    time_s: sample.time_s,
                    radius_m: sample.state[0],
                    speed_m_s: sample.control[0],
                    flight_path_angle_rad: 0.0,
                })
                .collect(),
            optimality_certificate: Some(certificate.clone()),
        };
        let decoded_payload = crate::iload::decode_iload_payload(
            &crate::iload::encode_iload_payload(&certified_payload)?,
        )?;
        assert_eq!(
            decoded_payload
                .optimality_certificate
                .as_ref()
                .expect("certificate present"),
            &certificate
        );
        assert!(
            (hamiltonian_report.initial_hamiltonian
                - pmp_solution.hamiltonian_report.initial_hamiltonian)
                .abs()
                < 1.0e-7
        );
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn lgr_fixed_mesh_ocp_dense_constrained_sqp_wrapper_consumes_nonlinear_hard_path_rows() {
        let base_ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![4],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(1, 0, 0.5).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds");
        let nonlinear_constraints = (0..base_ocp.layout.segment_grids[0].collocation_count())
            .map(|collocation_index| {
                LgrPseudospectralNonlinearHardPathConstraint::new(
                    0,
                    collocation_index,
                    vec![None],
                    vec![Some(0.5)],
                )
                .expect("nonlinear hard path constraint")
            })
            .collect::<Vec<_>>();
        let ocp = base_ocp
            .with_nonlinear_hard_path_constraints(nonlinear_constraints)
            .expect("nonlinear hard path constraints attach");
        let exact_support_states = ocp.layout.segment_grids[0]
            .support_nodes
            .iter()
            .map(|node| 0.5 * test_physical_time(0.0, 1.0, *node))
            .collect::<Vec<_>>();
        let exact_controls = vec![0.5; ocp.layout.segment_grids[0].collocation_count()];
        let initial_support_states = ocp.layout.segment_grids[0]
            .support_nodes
            .iter()
            .map(|node| 0.4 * test_physical_time(0.0, 1.0, *node))
            .collect::<Vec<_>>();
        let initial_controls = vec![0.4; ocp.layout.segment_grids[0].collocation_count()];
        let initial = ocp
            .layout
            .pack_decision_vector(&initial_support_states, &initial_controls)
            .expect("initial decision packs");
        let exact = ocp
            .layout
            .pack_decision_vector(&exact_support_states, &exact_controls)
            .expect("exact decision packs");
        let lower = vec![-10.0; ocp.layout.variable_count];
        let upper = vec![10.0; ocp.layout.variable_count];
        let solver = crate::sqp::DenseSqpSolver {
            max_iterations: 16,
            residual_tolerance: 1.0e-4,
            inequality_tolerance: 1.0e-8,
            step_tolerance: 1.0e-12,
            regularization: 1.0e-12,
            ..crate::sqp::DenseSqpSolver::default()
        };

        let report = ocp
            .solve_dense_sqp_with_nonlinear_hard_path_constraints(
                &initial,
                &lower,
                &upper,
                &solver,
                unit_control_lgr_dynamics,
                zero_lagrange,
                control_lgr_path_function,
            )
            .expect("fixed-mesh constrained LGR dense SQP wrapper solves");

        assert!(
            report.sqp_solution.converged,
            "constrained LGR SQP did not converge: residual {:e}, max inequality {:e}, iterations {}",
            report.sqp_solution.residual_norm,
            report.sqp_solution.max_inequality_violation,
            report.sqp_solution.iterations
        );
        assert!(
            report.residual_report.residual_norm < 1.0e-4,
            "constrained LGR residual norm {:e}",
            report.residual_report.residual_norm
        );
        assert!(report.max_decision_bound_violation < 1.0e-10);
        assert!(report.max_hard_path_violation < 1.0e-10);
        assert!(report.max_nonlinear_hard_path_violation < 1.0e-8);
        assert!(report.nonlinear_hard_path_report.max_violation < 1.0e-8);
        let max_solution_delta = report
            .sqp_solution
            .variables
            .iter()
            .zip(exact.iter())
            .map(|(actual, expected)| (actual - expected).abs())
            .fold(0.0_f64, f64::max);
        assert!(max_solution_delta < 1.0e-4);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_emits_optimized_reference_profile_and_iload_payload() {
        let ocp =
            LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0, 2.0], vec![2, 2], 3, 0, Vec::new())
                .expect("LGR OCP builds");
        let mut support_states = Vec::with_capacity(ocp.layout.state_variable_count);
        for (segment_index, grid) in ocp.layout.segment_grids.iter().enumerate() {
            let initial_time_s = ocp.layout.time_nodes_s[segment_index];
            let final_time_s = ocp.layout.time_nodes_s[segment_index + 1];
            for node in &grid.support_nodes {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                support_states.extend_from_slice(&[100.0 + time_s, 10.0 + time_s, 0.1 * time_s]);
            }
        }
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &[])
            .expect("decision packs");
        let mapping = LgrPseudospectralReferenceProfileMapping::new(0, 1, 2);

        let report = ocp
            .optimized_reference_profile(&decision, mapping)
            .expect("reference profile emits");

        assert_eq!(report.sample_count, 5);
        assert_eq!(report.start_time_s, 0.0);
        assert_eq!(report.end_time_s, 2.0);
        assert_eq!(report.samples[0].time_s, 0.0);
        assert_eq!(report.samples[2].time_s, 1.0);
        assert_eq!(report.samples[4].time_s, 2.0);
        assert_eq!(report.samples[2].radius_m, 101.0);
        assert_eq!(report.samples[2].speed_m_s, 11.0);
        assert_eq!(report.samples[2].flight_path_angle_rad, 0.1);
        assert!(
            report
                .samples
                .windows(2)
                .all(|window| window[0].time_s < window[1].time_s)
        );

        let payload = ocp
            .optimized_reference_iload_payload(
                &decision,
                mapping,
                &TerminalCondition::ApogeeRadius { radius_m: 120.0 },
                43,
                "openbmp-trajopt-lgr-test".to_string(),
                SynthesisMetadata {
                    scenario_digest: "scenario-sha256".to_string(),
                    source_revision: "test-revision".to_string(),
                    method: "lgr_fixed_mesh".to_string(),
                },
                Vec::new(),
                Vec::new(),
            )
            .expect("I-load payload emits");
        let encoded = crate::iload::encode_iload_payload(&payload).expect("payload encodes");
        let decoded = crate::iload::decode_iload_payload(&encoded).expect("payload decodes");

        assert_eq!(decoded.header.synthesis_seed, 43);
        assert_eq!(decoded.metadata.method, "lgr_fixed_mesh");
        assert_eq!(decoded.reference_profile, report.samples);

        let certificate_hamiltonian = crate::pmp::evaluate_pmp_hamiltonian_profile(
            crate::pmp::PmpHamiltonianProfile {
                state_dimension: 3,
                control_dimension: 1,
                times_s: &[0.0, 0.5, 1.0, 1.5, 2.0],
                states: &[0.0; 15],
                controls: &[0.0; 5],
                costates: &[0.0; 15],
            },
            |_time_s, _state, _control| Ok(vec![0.0, 0.0, 0.0]),
            |_time_s, _state, _control| Ok(0.0),
        )
        .expect("certificate Hamiltonian evaluates");
        let certificate = crate::iload::ILoadOptimalityCertificate::pmp_hamiltonian_constancy(
            crate::iload::ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt,
            &certificate_hamiltonian,
            1.0e-12,
        )
        .expect("certificate validates");
        let certified_payload = ocp
            .optimized_reference_iload_payload_with_optimality_certificate(
                &decision,
                mapping,
                &TerminalCondition::ApogeeRadius { radius_m: 120.0 },
                44,
                "openbmp-trajopt-lgr-test".to_string(),
                SynthesisMetadata {
                    scenario_digest: "scenario-sha256".to_string(),
                    source_revision: "test-revision".to_string(),
                    method: "lgr_fixed_mesh_with_pmp_certificate".to_string(),
                },
                Vec::new(),
                Vec::new(),
                certificate,
            )
            .expect("certified I-load payload emits");
        let decoded_certified = crate::iload::decode_iload_payload(
            &crate::iload::encode_iload_payload(&certified_payload)
                .expect("certified payload encodes"),
        )
        .expect("certified payload decodes");
        assert_eq!(
            decoded_certified
                .optimality_certificate
                .as_ref()
                .expect("certificate present")
                .evidence_kind,
            crate::iload::ILoadOptimalityEvidenceKind::LgrPseudospectralPmpKkt
        );
    }

    #[test]
    fn lgr_fixed_mesh_ocp_reference_profile_rejects_bad_mapping_and_payload() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 3, 0, Vec::new())
            .expect("LGR OCP builds");
        let support_states = vec![100.0, 10.0, 0.0, 110.0, 11.0, 0.1, 120.0, 12.0, 0.2];
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &[])
            .expect("decision packs");

        assert!(
            ocp.optimized_reference_profile(
                &decision,
                LgrPseudospectralReferenceProfileMapping::new(0, 1, 3),
            )
            .is_err()
        );

        let mut nonfinite_decision = decision.clone();
        nonfinite_decision[1] = f64::NAN;
        assert!(
            ocp.optimized_reference_profile(
                &nonfinite_decision,
                LgrPseudospectralReferenceProfileMapping::new(0, 1, 2),
            )
            .is_err()
        );

        let malformed_report = LgrPseudospectralReferenceProfileReport {
            samples: vec![
                ReferenceProfileSample {
                    time_s: 1.0,
                    radius_m: 100.0,
                    speed_m_s: 10.0,
                    flight_path_angle_rad: 0.0,
                },
                ReferenceProfileSample {
                    time_s: 1.0,
                    radius_m: 101.0,
                    speed_m_s: 11.0,
                    flight_path_angle_rad: 0.1,
                },
            ],
            sample_count: 2,
            start_time_s: 1.0,
            end_time_s: 1.0,
        };
        assert!(validate_lgr_reference_profile_report(&malformed_report).is_err());

        assert!(
            ocp.optimized_reference_iload_payload(
                &decision,
                LgrPseudospectralReferenceProfileMapping::new(0, 1, 2),
                &TerminalCondition::ApogeeRadius { radius_m: 120.0 },
                43,
                " ".to_string(),
                SynthesisMetadata {
                    scenario_digest: "scenario-sha256".to_string(),
                    source_revision: "test-revision".to_string(),
                    method: "lgr_fixed_mesh".to_string(),
                },
                Vec::new(),
                Vec::new(),
            )
            .is_err()
        );
    }

    #[test]
    fn lgr_fixed_mesh_ocp_transcribes_hard_path_bounds_to_dense_inequalities() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_hard_path_constraints(vec![
                LgrPseudospectralHardPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                    Some(0.2),
                    Some(0.75),
                )
                .expect("state hard path constraint"),
                LgrPseudospectralHardPathConstraint::new(
                    0,
                    0,
                    LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
                    None,
                    Some(0.5),
                )
                .expect("control hard path constraint"),
            ])
            .expect("hard path constraints attach");
        let report = ocp
            .hard_path_inequality_report()
            .expect("hard inequalities transcribe");

        assert_eq!(report.variable_count, 9);
        assert_eq!(report.inequality_count, 3);
        assert_eq!(report.inequality_rhs, vec![0.75, -0.2, 0.5]);
        assert_eq!(report.inequality_matrix[0], 1.0);
        assert_eq!(report.inequality_matrix[9], -1.0);
        assert_eq!(report.inequality_matrix[23], 1.0);

        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        assert_eq!(report.lhs(&decision).expect("lhs"), vec![0.0, -0.0, 1.0]);
        assert_eq!(
            report.violations(&decision).expect("violations"),
            vec![0.0, 0.2, 0.5]
        );
        assert!((report.max_violation(&decision).expect("max violation") - 0.5).abs() < 1.0e-14);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_evaluates_nonlinear_hard_path_inequalities() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_nonlinear_hard_path_constraints(vec![
                LgrPseudospectralNonlinearHardPathConstraint::new(
                    0,
                    0,
                    vec![Some(0.5), None],
                    vec![Some(0.75), Some(0.25)],
                )
                .expect("nonlinear hard path constraint"),
            ])
            .expect("nonlinear hard path constraints attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut path_function = |time_s: f64, state: &[f64], control: &[f64]| {
            Ok(vec![time_s + state[0] + control[0], control[0] - state[0]])
        };

        let report = ocp
            .nonlinear_hard_path_inequality_report(&decision, &mut path_function)
            .expect("nonlinear hard path inequalities evaluate");

        assert_eq!(report.variable_count, 9);
        assert_eq!(report.path_value_count, 2);
        assert_eq!(report.inequality_count, 3);
        assert_eq!(report.path_values, vec![1.0, 1.0]);
        assert_eq!(report.inequality_values, vec![0.25, -0.5, 0.75]);
        assert_eq!(
            report.violations().expect("violations"),
            vec![0.25, 0.0, 0.75]
        );
        assert!((report.max_violation - 0.75).abs() < 1.0e-14);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_nonlinear_hard_path_terms() {
        assert!(
            LgrPseudospectralNonlinearHardPathConstraint::new(0, 0, Vec::new(), Vec::new())
                .is_err()
        );
        assert!(
            LgrPseudospectralNonlinearHardPathConstraint::new(0, 0, vec![Some(0.0)], Vec::new(),)
                .is_err()
        );
        assert!(
            LgrPseudospectralNonlinearHardPathConstraint::new(0, 0, vec![None], vec![None],)
                .is_err()
        );
        assert!(
            LgrPseudospectralNonlinearHardPathConstraint::new(
                0,
                0,
                vec![Some(f64::NAN)],
                vec![None],
            )
            .is_err()
        );
        assert!(
            LgrPseudospectralNonlinearHardPathConstraint::new(
                0,
                0,
                vec![Some(1.0)],
                vec![Some(0.0)],
            )
            .is_err()
        );

        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 1, Vec::new())
            .expect("LGR OCP builds");
        assert!(
            ocp.clone()
                .with_nonlinear_hard_path_constraints(vec![
                    LgrPseudospectralNonlinearHardPathConstraint::new(
                        1,
                        0,
                        vec![Some(0.0)],
                        vec![None],
                    )
                    .expect("nonlinear hard path constraint")
                ])
                .is_err()
        );
        assert!(
            ocp.clone()
                .with_nonlinear_hard_path_constraints(vec![
                    LgrPseudospectralNonlinearHardPathConstraint::new(
                        0,
                        2,
                        vec![Some(0.0)],
                        vec![None],
                    )
                    .expect("nonlinear hard path constraint")
                ])
                .is_err()
        );

        let ocp = ocp
            .with_nonlinear_hard_path_constraints(vec![
                LgrPseudospectralNonlinearHardPathConstraint::new(
                    0,
                    0,
                    vec![Some(0.0)],
                    vec![None],
                )
                .expect("nonlinear hard path constraint"),
            ])
            .expect("nonlinear hard path constraints attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut wrong_dimension_path =
            |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![0.0, 1.0]);
        assert!(
            ocp.nonlinear_hard_path_inequality_report(&decision, &mut wrong_dimension_path)
                .is_err()
        );
        let mut nonfinite_path =
            |_time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![f64::NAN]);
        assert!(
            ocp.nonlinear_hard_path_inequality_report(&decision, &mut nonfinite_path)
                .is_err()
        );

        let malformed_report = LgrPseudospectralNonlinearPathInequalityReport {
            variable_count: 0,
            path_value_count: 1,
            inequality_count: 1,
            path_values: vec![0.0],
            inequality_values: vec![0.0],
            max_violation: 0.0,
        };
        assert!(malformed_report.violations().is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_sparse_nonlinear_path_jacobian_matches_linear_reference() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_nonlinear_hard_path_constraints(vec![
                LgrPseudospectralNonlinearHardPathConstraint::new(
                    0,
                    0,
                    vec![Some(-4.0)],
                    vec![Some(-2.0)],
                )
                .expect("nonlinear hard path constraint"),
            ])
            .expect("nonlinear hard path constraints attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut path_function = |time_s: f64, state: &[f64], control: &[f64]| {
            Ok(vec![time_s + 2.0 * state[0] - 3.0 * control[0]])
        };

        let jacobian = ocp
            .sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision,
                &mut path_function,
                1.0e-6,
                1.0e-12,
            )
            .expect("sparse nonlinear path Jacobian evaluates");

        assert_eq!(jacobian.row_count, 2);
        assert_eq!(jacobian.column_count, 9);
        assert_eq!(jacobian.finite_difference_step, 1.0e-6);
        assert_eq!(jacobian.drop_tolerance, 1.0e-12);
        assert!((jacobian.coefficient(0, 0).expect("upper state") - 2.0).abs() < 1.0e-9);
        assert!((jacobian.coefficient(1, 0).expect("lower state") + 2.0).abs() < 1.0e-9);
        assert!((jacobian.coefficient(0, 5).expect("upper control") + 3.0).abs() < 1.0e-9);
        assert!((jacobian.coefficient(1, 5).expect("lower control") - 3.0).abs() < 1.0e-9);
        assert_eq!(jacobian.coefficient(0, 1).expect("unrelated support"), 0.0);

        let dense = jacobian.to_dense_row_major().expect("dense expansion");
        assert_eq!(dense.len(), jacobian.row_count * jacobian.column_count);
        assert!(jacobian.coefficient(2, 0).is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_sparse_nonlinear_path_jacobian_inputs() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_nonlinear_hard_path_constraints(vec![
                LgrPseudospectralNonlinearHardPathConstraint::new(
                    0,
                    0,
                    vec![Some(-1.0)],
                    vec![None],
                )
                .expect("nonlinear hard path constraint"),
            ])
            .expect("nonlinear hard path constraints attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut path_function = |_time_s: f64, state: &[f64], _control: &[f64]| Ok(vec![state[0]]);

        assert!(
            ocp.sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision,
                &mut path_function,
                0.0,
                1.0e-12,
            )
            .is_err()
        );
        assert!(
            ocp.sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision,
                &mut path_function,
                1.0e-6,
                -1.0,
            )
            .is_err()
        );
        assert!(
            ocp.sparse_nonlinear_hard_path_inequality_jacobian_report(
                &decision[..decision.len() - 1],
                &mut path_function,
                1.0e-6,
                1.0e-12,
            )
            .is_err()
        );

        let malformed = LgrPseudospectralSparsePathInequalityJacobianReport {
            row_count: 1,
            column_count: 0,
            column_offsets: vec![0],
            row_indices: Vec::new(),
            values: Vec::new(),
            finite_difference_step: 1.0e-6,
            drop_tolerance: 0.0,
        };
        assert!(malformed.to_dense_row_major().is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_sparse_residual_jacobian_matches_linear_reference() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![3],
            1,
            0,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(1, 0, 1.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut dynamics = |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]);
        let mut lagrange = zero_lagrange;

        let jacobian = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-12,
            )
            .expect("sparse residual Jacobian evaluates");

        assert_eq!(jacobian.row_count, 5);
        assert_eq!(jacobian.column_count, 4);
        assert_eq!(jacobian.column_offsets.len(), 5);
        assert_eq!(jacobian.finite_difference_step, 1.0e-6);
        assert_eq!(jacobian.drop_tolerance, 1.0e-12);

        let dense = jacobian.to_dense_row_major().expect("dense expansion");
        let grid = &ocp.layout.segment_grids[0];
        for row in 0..grid.collocation_count() {
            for column in 0..grid.support_count() {
                let expected = grid.differentiation_matrix[row * grid.support_count() + column];
                let actual = dense[row * jacobian.column_count + column];
                assert!((actual - expected).abs() < 1.0e-8);
            }
        }
        assert!((jacobian.coefficient(3, 0).expect("initial boundary") - 1.0).abs() < 1.0e-10);
        assert!((jacobian.coefficient(4, 3).expect("terminal boundary") - 1.0).abs() < 1.0e-10);
        assert!(jacobian.coefficient(5, 0).is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_analytic_sparse_residual_jacobian_matches_finite_difference_report() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0, 2.0],
            vec![3, 2],
            2,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("initial boundary"),
                LgrPseudospectralBoundaryConstraint::new(2, 1, 2.0).expect("terminal boundary"),
            ],
        )
        .expect("LGR OCP builds")
        .with_path_constraints(vec![
            LgrPseudospectralSoftPathConstraint::new(
                0,
                0,
                LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                Some(0.2),
                None,
                4.0,
            )
            .expect("active lower soft path"),
            LgrPseudospectralSoftPathConstraint::new(
                1,
                0,
                LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
                None,
                Some(1.2),
                9.0,
            )
            .expect("active upper soft path"),
        ])
        .expect("soft paths attach")
        .with_quadratic_control_costs(vec![
            LgrPseudospectralQuadraticControlCost::new(0, 0.25).expect("control cost"),
        ])
        .expect("control costs attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut finite_difference_dynamics = linear_lgr_dynamics;
        let mut lagrange = zero_lagrange;
        let finite_difference = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut finite_difference_dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-10,
            )
            .expect("finite-difference sparse residual Jacobian evaluates");
        let mut analytic_dynamics = LinearLgrDynamicsJacobian;
        let analytic = ocp
            .analytic_sparse_residual_jacobian_report(&decision, &mut analytic_dynamics, 1.0e-12)
            .expect("analytic sparse residual Jacobian evaluates");

        assert_eq!(analytic.row_count, finite_difference.row_count);
        assert_eq!(analytic.column_count, finite_difference.column_count);
        assert_eq!(analytic.finite_difference_step, 0.0);
        assert_eq!(analytic.drop_tolerance, 1.0e-12);

        let finite_difference_dense = finite_difference.to_dense_row_major().expect("FD dense");
        let analytic_dense = analytic.to_dense_row_major().expect("analytic dense");
        assert_eq!(finite_difference_dense.len(), analytic_dense.len());
        for (finite_difference, analytic) in
            finite_difference_dense.iter().zip(analytic_dense.iter())
        {
            assert!((finite_difference - analytic).abs() < 1.0e-8);
        }
    }

    #[test]
    fn lgr_fixed_mesh_ocp_autodiff_sparse_residual_jacobian_matches_analytic_report() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![4], 2, 1, Vec::new())
            .expect("LGR OCP builds")
            .with_quadratic_control_costs(vec![
                LgrPseudospectralQuadraticControlCost::new(0, 1.0).expect("control cost"),
            ])
            .expect("control costs attach");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut analytic_dynamics = LinearLgrDynamicsJacobian;
        let analytic = ocp
            .analytic_sparse_residual_jacobian_report(&decision, &mut analytic_dynamics, 1.0e-12)
            .expect("analytic sparse residual Jacobian evaluates");
        let mut autodiff_dynamics =
            |time_s: f64,
             state: &[CollocationAutoDiffScalar],
             control: &[CollocationAutoDiffScalar]| {
                Ok(vec![
                    state[0].clone() * 2.0 - state[1].clone() * 0.5
                        + control[0].clone() * 3.0
                        + time_s,
                    -state[0].clone() + state[1].clone() * 4.0 - control[0].clone() * 2.0
                        + 0.5 * time_s,
                ])
            };
        let autodiff = ocp
            .autodiff_sparse_residual_jacobian_report(&decision, &mut autodiff_dynamics, 1.0e-12)
            .expect("autodiff sparse residual Jacobian evaluates");

        assert_eq!(autodiff.finite_difference_step, 0.0);
        let analytic_dense = analytic.to_dense_row_major().expect("analytic dense");
        let autodiff_dense = autodiff.to_dense_row_major().expect("autodiff dense");
        assert_eq!(analytic_dense, autodiff_dense);
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_analytic_and_autodiff_residual_jacobians() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 2, 1, Vec::new())
            .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut analytic_dynamics = LinearLgrDynamicsJacobian;
        assert!(
            ocp.analytic_sparse_residual_jacobian_report(
                &decision[..decision.len() - 1],
                &mut analytic_dynamics,
                1.0e-12,
            )
            .is_err()
        );
        assert!(
            ocp.analytic_sparse_residual_jacobian_report(&decision, &mut analytic_dynamics, -1.0,)
                .is_err()
        );

        let mut malformed_analytic = MalformedLgrDynamicsJacobian;
        assert!(
            ocp.analytic_sparse_residual_jacobian_report(
                &decision,
                &mut malformed_analytic,
                1.0e-12,
            )
            .is_err()
        );

        let mut malformed_autodiff =
            |_time_s: f64,
             state: &[CollocationAutoDiffScalar],
             _control: &[CollocationAutoDiffScalar]| { Ok(vec![state[0].clone()]) };
        assert!(
            ocp.autodiff_sparse_residual_jacobian_report(
                &decision,
                &mut malformed_autodiff,
                1.0e-12,
            )
            .is_err()
        );
    }

    #[test]
    fn lgr_fixed_mesh_ocp_colored_sparse_residual_jacobian_matches_columnwise_report() {
        let ocp =
            LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0, 2.0], vec![3, 3], 1, 0, Vec::new())
                .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut columnwise_dynamics =
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]);
        let mut columnwise_lagrange = zero_lagrange;
        let columnwise = ocp
            .sparse_residual_jacobian_report(
                &decision,
                &mut columnwise_dynamics,
                &mut columnwise_lagrange,
                1.0e-6,
                1.0e-12,
            )
            .expect("columnwise sparse residual Jacobian evaluates");

        let mut colored_dynamics =
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]);
        let mut colored_lagrange = zero_lagrange;
        let colored = ocp
            .colored_sparse_residual_jacobian_report(
                &decision,
                &mut colored_dynamics,
                &mut colored_lagrange,
                1.0e-6,
                1.0e-12,
            )
            .expect("colored sparse residual Jacobian evaluates");

        assert_eq!(colored.sparse_jacobian.row_count, columnwise.row_count);
        assert_eq!(
            colored.sparse_jacobian.column_count,
            columnwise.column_count
        );
        assert_eq!(colored.column_colors.len(), columnwise.column_count);
        assert!(colored.color_count < columnwise.column_count);
        assert_eq!(
            colored.residual_evaluation_count,
            1 + 2 * colored.color_count
        );
        assert!(colored.structural_nonzero_count >= colored.sparse_jacobian.values.len());

        let columnwise_dense = columnwise.to_dense_row_major().expect("columnwise dense");
        let colored_dense = colored.to_dense_row_major().expect("colored dense");
        assert_eq!(columnwise_dense.len(), colored_dense.len());
        for (columnwise, colored) in columnwise_dense.iter().zip(colored_dense.iter()) {
            assert!((columnwise - colored).abs() < 1.0e-8);
        }
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_sparse_residual_jacobian_inputs() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 0, Vec::new())
            .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut dynamics = zero_dynamics;
        let mut lagrange = zero_lagrange;
        assert!(
            ocp.sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                0.0,
                1.0e-12,
            )
            .is_err()
        );
        assert!(
            ocp.sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                -1.0,
            )
            .is_err()
        );
        assert!(
            ocp.sparse_residual_jacobian_report(
                &decision[..decision.len() - 1],
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-12,
            )
            .is_err()
        );

        let malformed = LgrPseudospectralSparseResidualJacobianReport {
            row_count: 0,
            column_count: 1,
            column_offsets: vec![0, 0],
            row_indices: Vec::new(),
            values: Vec::new(),
            finite_difference_step: 1.0e-6,
            drop_tolerance: 0.0,
        };
        assert!(malformed.to_dense_row_major().is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_colored_sparse_residual_jacobian_inputs() {
        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 0, Vec::new())
            .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");

        let mut dynamics = zero_dynamics;
        let mut lagrange = zero_lagrange;
        assert!(
            ocp.colored_sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                0.0,
                1.0e-12,
            )
            .is_err()
        );
        assert!(
            ocp.colored_sparse_residual_jacobian_report(
                &decision,
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                -1.0,
            )
            .is_err()
        );
        assert!(
            ocp.colored_sparse_residual_jacobian_report(
                &decision[..decision.len() - 1],
                &mut dynamics,
                &mut lagrange,
                1.0e-6,
                1.0e-12,
            )
            .is_err()
        );

        let malformed = LgrPseudospectralColoredSparseResidualJacobianReport {
            sparse_jacobian: LgrPseudospectralSparseResidualJacobianReport {
                row_count: 1,
                column_count: 1,
                column_offsets: vec![0, 0],
                row_indices: Vec::new(),
                values: Vec::new(),
                finite_difference_step: 1.0e-6,
                drop_tolerance: 0.0,
            },
            column_colors: Vec::new(),
            color_count: 1,
            residual_evaluation_count: 3,
            structural_nonzero_count: 0,
        };
        assert!(malformed.to_dense_row_major().is_err());
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_boundaries_and_decisions() {
        assert!(LgrPseudospectralBoundaryConstraint::new(0, 0, f64::NAN).is_err());
        assert!(
            LgrPseudospectralFixedMeshOcp::new(vec![0.0, 0.0], vec![2], 1, 0, Vec::new()).is_err()
        );
        assert!(
            LgrPseudospectralFixedMeshOcp::new(
                vec![0.0, 1.0],
                vec![2],
                1,
                0,
                vec![LgrPseudospectralBoundaryConstraint::new(2, 0, 0.0).expect("boundary")],
            )
            .is_err()
        );
        assert!(
            LgrPseudospectralFixedMeshOcp::new(
                vec![0.0, 1.0],
                vec![2],
                1,
                0,
                vec![LgrPseudospectralBoundaryConstraint::new(1, 1, 0.0).expect("boundary")],
            )
            .is_err()
        );

        let ocp = LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0],
            vec![2],
            1,
            0,
            vec![LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0).expect("boundary")],
        )
        .expect("LGR OCP builds");
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        let mut short_decision = decision.clone();
        short_decision.pop();
        assert!(
            ocp.residual_report(&short_decision, zero_dynamics, zero_lagrange)
                .is_err()
        );

        let wrong_dynamics = |_: f64, _: &[f64], _: &[f64]| Ok(Vec::new());
        assert!(
            ocp.residual_report(&decision, wrong_dynamics, zero_lagrange)
                .is_err()
        );

        let mut inconsistent_ocp = ocp;
        inconsistent_ocp.boundary_constraints[0].component = 2;
        assert!(
            inconsistent_ocp
                .residual_report(&decision, zero_dynamics, zero_lagrange)
                .is_err()
        );
    }

    #[test]
    fn lgr_fixed_mesh_ocp_rejects_malformed_path_and_objective_terms() {
        let state_component = LgrPseudospectralScalarVariable::StateComponent { component: 0 };
        let control_component = LgrPseudospectralScalarVariable::ControlComponent { component: 0 };
        assert!(
            LgrPseudospectralSoftPathConstraint::new(0, 0, state_component, None, None, 1.0)
                .is_err()
        );
        assert!(
            LgrPseudospectralSoftPathConstraint::new(
                0,
                0,
                state_component,
                Some(1.0),
                Some(0.0),
                1.0,
            )
            .is_err()
        );
        assert!(
            LgrPseudospectralSoftPathConstraint::new(0, 0, state_component, Some(0.0), None, 0.0,)
                .is_err()
        );
        assert!(
            LgrPseudospectralHardPathConstraint::new(0, 0, state_component, None, None).is_err()
        );
        assert!(
            LgrPseudospectralHardPathConstraint::new(0, 0, state_component, Some(1.0), Some(0.0),)
                .is_err()
        );
        assert!(LgrPseudospectralQuadraticControlCost::new(0, 0.0).is_err());

        let zero_control_ocp =
            LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 0, Vec::new())
                .expect("zero-control OCP builds");
        assert!(
            zero_control_ocp
                .clone()
                .with_path_constraints(vec![
                    LgrPseudospectralSoftPathConstraint::new(
                        0,
                        0,
                        control_component,
                        Some(0.0),
                        None,
                        1.0,
                    )
                    .expect("control path constraint")
                ])
                .is_err()
        );
        assert!(
            zero_control_ocp
                .clone()
                .with_hard_path_constraints(vec![
                    LgrPseudospectralHardPathConstraint::new(
                        0,
                        0,
                        control_component,
                        Some(0.0),
                        None,
                    )
                    .expect("hard path constraint")
                ])
                .is_err()
        );
        assert!(
            zero_control_ocp
                .with_quadratic_control_costs(vec![
                    LgrPseudospectralQuadraticControlCost::new(0, 1.0).expect("control cost")
                ])
                .is_err()
        );

        let ocp = LgrPseudospectralFixedMeshOcp::new(vec![0.0, 1.0], vec![2], 1, 1, Vec::new())
            .expect("LGR OCP builds");
        assert!(
            ocp.clone()
                .with_hard_path_constraints(vec![
                    LgrPseudospectralHardPathConstraint::new(
                        1,
                        0,
                        state_component,
                        Some(0.0),
                        None,
                    )
                    .expect("hard path constraint")
                ])
                .is_err()
        );
        assert!(
            ocp.clone()
                .with_hard_path_constraints(vec![
                    LgrPseudospectralHardPathConstraint::new(
                        0,
                        2,
                        state_component,
                        Some(0.0),
                        None,
                    )
                    .expect("hard path constraint")
                ])
                .is_err()
        );
        assert!(
            ocp.clone()
                .with_hard_path_constraints(vec![
                    LgrPseudospectralHardPathConstraint::new(
                        0,
                        0,
                        LgrPseudospectralScalarVariable::StateComponent { component: 1 },
                        Some(0.0),
                        None,
                    )
                    .expect("hard path constraint")
                ])
                .is_err()
        );
        assert!(
            ocp.clone()
                .with_path_constraints(vec![
                    LgrPseudospectralSoftPathConstraint::new(
                        1,
                        0,
                        state_component,
                        Some(0.0),
                        None,
                        1.0,
                    )
                    .expect("path constraint")
                ])
                .is_err()
        );
        assert!(
            ocp.clone()
                .with_path_constraints(vec![
                    LgrPseudospectralSoftPathConstraint::new(
                        0,
                        2,
                        state_component,
                        Some(0.0),
                        None,
                        1.0,
                    )
                    .expect("path constraint")
                ])
                .is_err()
        );
        assert!(
            ocp.clone()
                .with_path_constraints(vec![
                    LgrPseudospectralSoftPathConstraint::new(
                        0,
                        0,
                        LgrPseudospectralScalarVariable::StateComponent { component: 1 },
                        Some(0.0),
                        None,
                        1.0,
                    )
                    .expect("path constraint")
                ])
                .is_err()
        );

        let mut inconsistent_ocp = ocp
            .clone()
            .with_path_constraints(vec![
                LgrPseudospectralSoftPathConstraint::new(
                    0,
                    0,
                    state_component,
                    Some(0.0),
                    None,
                    1.0,
                )
                .expect("path constraint"),
            ])
            .expect("path constraints attach");
        let (support_states, collocation_controls) =
            polynomial_lgr_decision_blocks(&inconsistent_ocp.layout);
        let decision = inconsistent_ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("decision packs");
        inconsistent_ocp.path_constraints[0].collocation_index = 3;
        assert!(
            inconsistent_ocp
                .residual_report(&decision, zero_dynamics, zero_lagrange)
                .is_err()
        );

        let mut inconsistent_hard_ocp = ocp
            .with_hard_path_constraints(vec![
                LgrPseudospectralHardPathConstraint::new(0, 0, state_component, Some(0.0), None)
                    .expect("hard path constraint"),
            ])
            .expect("hard path constraints attach");
        inconsistent_hard_ocp.hard_path_constraints[0].collocation_index = 3;
        assert!(inconsistent_hard_ocp.hard_path_inequality_report().is_err());

        let malformed_report = LgrPseudospectralPathInequalityReport {
            variable_count: 0,
            inequality_count: 1,
            inequality_matrix: vec![0.0],
            inequality_rhs: vec![0.0],
        };
        assert!(malformed_report.max_violation(&decision).is_err());
    }

    #[test]
    fn lgr_mesh_refinement_accepts_exact_candidate_against_reference() {
        let candidate = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        let reference = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 6)],
        };
        let settings = mesh_refinement_settings();

        let report = estimate_lgr_pseudospectral_mesh_refinement(&candidate, &reference, &settings)
            .expect("mesh refinement estimates");

        assert!(report.converged);
        assert_eq!(report.refined_segment_count, 0);
        assert!(report.max_relative_error < 1.0e-13);
        assert_eq!(report.estimates.len(), 1);
        assert_eq!(
            report.recommendations[0].action,
            LgrPseudospectralMeshRefinementAction::Accept
        );
    }

    #[test]
    fn lgr_mesh_refinement_recommends_degree_raise_then_split() {
        let reference = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 8)],
        };
        let degree_raise_candidate = LgrPseudospectralTranscription {
            segments: vec![shifted_polynomial_lgr_segment(0.0, 1.0, 4, 1.0e-2)],
        };
        let split_candidate = LgrPseudospectralTranscription {
            segments: vec![shifted_polynomial_lgr_segment(0.0, 1.0, 6, 1.0e-2)],
        };
        let settings = mesh_refinement_settings();

        let degree_raise_report = estimate_lgr_pseudospectral_mesh_refinement(
            &degree_raise_candidate,
            &reference,
            &settings,
        )
        .expect("degree-raise estimate");
        assert!(!degree_raise_report.converged);
        assert_eq!(degree_raise_report.refined_segment_count, 1);
        assert!(
            degree_raise_report.estimates[0].relative_error > settings.relative_error_tolerance
        );
        assert_eq!(
            degree_raise_report.recommendations[0].action,
            LgrPseudospectralMeshRefinementAction::RaiseDegree {
                collocation_count: 6
            }
        );

        let split_report =
            estimate_lgr_pseudospectral_mesh_refinement(&split_candidate, &reference, &settings)
                .expect("split estimate");
        assert!(!split_report.converged);
        assert_eq!(
            split_report.recommendations[0].action,
            LgrPseudospectralMeshRefinementAction::Split {
                segment_count: 2,
                collocation_count: 6
            }
        );
    }

    #[test]
    fn lgr_mesh_refinement_application_raises_degree_and_preserves_seed() {
        let source = LgrPseudospectralTranscription {
            segments: vec![controlled_polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        let report = mesh_refinement_report_for(
            &source,
            LgrPseudospectralMeshRefinementAction::RaiseDegree {
                collocation_count: 6,
            },
        );

        let application = apply_lgr_pseudospectral_mesh_refinement(&source, &report)
            .expect("mesh refinement applies");

        assert_eq!(application.transcription.segments.len(), 1);
        assert_eq!(application.segment_mappings.len(), 1);
        assert_eq!(application.segment_mappings[0].source_segment_index, 0);
        assert_eq!(application.segment_mappings[0].refined_segment_index, 0);
        assert_eq!(application.segment_mappings[0].child_count, 1);
        let segment = &application.transcription.segments[0];
        assert_eq!(segment.grid.collocation_count(), 6);
        assert_polynomial_state_and_linear_control_seed(segment);
    }

    #[test]
    fn lgr_mesh_refinement_application_splits_segment_and_preserves_seed() {
        let source = LgrPseudospectralTranscription {
            segments: vec![controlled_polynomial_lgr_segment_with_count(0.0, 2.0, 5)],
        };
        let report = mesh_refinement_report_for(
            &source,
            LgrPseudospectralMeshRefinementAction::Split {
                segment_count: 2,
                collocation_count: 4,
            },
        );

        let application = apply_lgr_pseudospectral_mesh_refinement(&source, &report)
            .expect("mesh refinement applies");

        assert_eq!(application.transcription.segments.len(), 2);
        assert_eq!(application.segment_mappings.len(), 2);
        assert_eq!(application.segment_mappings[0].child_index, 0);
        assert_eq!(application.segment_mappings[1].child_index, 1);
        assert_eq!(application.segment_mappings[0].child_count, 2);
        assert_eq!(application.transcription.segments[0].initial_time_s, 0.0);
        assert_eq!(application.transcription.segments[0].final_time_s, 1.0);
        assert_eq!(application.transcription.segments[1].initial_time_s, 1.0);
        assert_eq!(application.transcription.segments[1].final_time_s, 2.0);
        for segment in &application.transcription.segments {
            assert_polynomial_state_and_linear_control_seed(segment);
        }

        let report = evaluate_lgr_pseudospectral_transcription(
            &application.transcription,
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(vec![2.0 * time_s]),
            |time_s: f64, _state: &[f64], _control: &[f64]| Ok(time_s * time_s),
        )
        .expect("refined transcription evaluates");
        assert!(report.residual_norm < 1.0e-11);
        assert!((report.quadrature_cost - 8.0 / 3.0).abs() < 1.0e-12);
    }

    #[test]
    fn lgr_mesh_refinement_loop_refines_until_reference_converges() {
        let source = LgrPseudospectralTranscription {
            segments: vec![controlled_polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        let settings = mesh_refinement_settings();
        let mut reference_calls = 0;

        let report = refine_lgr_pseudospectral_mesh_with_reference(
            &source,
            &settings,
            3,
            |iteration_index, candidate| {
                reference_calls += 1;
                let state_shift = if iteration_index == 0 { 1.0e-2 } else { 0.0 };
                Ok(polynomial_reference_for_candidate(
                    candidate,
                    2,
                    state_shift,
                ))
            },
        )
        .expect("mesh-refinement loop runs");

        assert!(report.converged);
        assert_eq!(report.iteration_count, 2);
        assert_eq!(reference_calls, 2);
        assert_eq!(report.transcription.segments.len(), 1);
        assert_eq!(report.transcription.segments[0].grid.collocation_count(), 6);
        assert_eq!(report.iterations[0].candidate_segment_count, 1);
        assert_eq!(report.iterations[0].reference_segment_count, 1);
        assert_eq!(
            report.iterations[0].refinement_report.refined_segment_count,
            1
        );
        assert!(report.iterations[1].refinement_report.converged);
        assert_eq!(report.refined_segment_count, 0);
        assert_polynomial_state_and_linear_control_seed(&report.transcription.segments[0]);
    }

    #[test]
    fn lgr_mesh_refinement_loop_reports_iteration_cap_without_convergence() {
        let source = LgrPseudospectralTranscription {
            segments: vec![controlled_polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        let settings = mesh_refinement_settings();

        let report = refine_lgr_pseudospectral_mesh_with_reference(
            &source,
            &settings,
            1,
            |_iteration_index, candidate| {
                Ok(polynomial_reference_for_candidate(candidate, 2, 1.0e-2))
            },
        )
        .expect("mesh-refinement loop runs");

        assert!(!report.converged);
        assert_eq!(report.iteration_count, 1);
        assert!(report.max_relative_error > settings.relative_error_tolerance);
        assert_eq!(report.refined_segment_count, 1);
        assert_eq!(report.transcription.segments[0].grid.collocation_count(), 6);
    }

    #[test]
    fn lgr_grid_rejects_bad_dimensions() {
        assert!(legendre_gauss_radau_grid(0).is_err());
        let grid = legendre_gauss_radau_grid(2).expect("LGR grid builds");
        assert!(grid.differentiation_entry(2, 0).is_err());
        assert!(grid.differentiation_entry(0, 3).is_err());
        assert!(grid.integrate_collocation_values(&[0.0]).is_err());
        assert!(grid.differentiate_support_values(&[0.0, 1.0]).is_err());
        assert!(
            grid.differentiate_support_values(&[0.0, f64::NAN, 1.0])
                .is_err()
        );
    }

    #[test]
    fn lgr_segment_rejects_malformed_inputs() {
        let grid = legendre_gauss_radau_grid(2).expect("LGR grid builds");
        let base_segment = LgrPseudospectralSegment {
            initial_time_s: 0.0,
            final_time_s: 1.0,
            grid,
            state_dimension: 1,
            control_dimension: 1,
            support_states: vec![0.0, 0.5, 1.0],
            collocation_controls: vec![0.0, 0.0],
        };

        let mut bad_time = base_segment.clone();
        bad_time.final_time_s = 0.0;
        assert!(
            evaluate_lgr_pseudospectral_segment(&bad_time, zero_dynamics, zero_lagrange).is_err()
        );

        let mut bad_state_dimension = base_segment.clone();
        bad_state_dimension.state_dimension = 0;
        assert!(
            evaluate_lgr_pseudospectral_segment(&bad_state_dimension, zero_dynamics, zero_lagrange)
                .is_err()
        );

        let mut bad_support = base_segment.clone();
        bad_support.support_states.pop();
        assert!(
            evaluate_lgr_pseudospectral_segment(&bad_support, zero_dynamics, zero_lagrange)
                .is_err()
        );

        let mut bad_control = base_segment.clone();
        bad_control.collocation_controls.push(0.0);
        assert!(
            evaluate_lgr_pseudospectral_segment(&bad_control, zero_dynamics, zero_lagrange)
                .is_err()
        );

        let wrong_dynamics = |_: f64, _: &[f64], _: &[f64]| Ok(Vec::new());
        assert!(
            evaluate_lgr_pseudospectral_segment(&base_segment, wrong_dynamics, zero_lagrange)
                .is_err()
        );

        let nonfinite_lagrange = |_: f64, _: &[f64], _: &[f64]| Ok(f64::NAN);
        assert!(
            evaluate_lgr_pseudospectral_segment(&base_segment, zero_dynamics, nonfinite_lagrange)
                .is_err()
        );
    }

    #[test]
    fn lgr_transcription_rejects_nonadjacent_or_mismatched_segments() {
        let first_segment = polynomial_lgr_segment(0.0, 1.0);
        let second_segment = polynomial_lgr_segment(1.0, 2.0);

        assert!(
            evaluate_lgr_pseudospectral_transcription(
                &LgrPseudospectralTranscription {
                    segments: Vec::new()
                },
                zero_dynamics,
                zero_lagrange,
            )
            .is_err()
        );

        let mut nonadjacent_segment = second_segment.clone();
        nonadjacent_segment.initial_time_s = 1.1;
        assert!(
            evaluate_lgr_pseudospectral_transcription(
                &LgrPseudospectralTranscription {
                    segments: vec![first_segment.clone(), nonadjacent_segment]
                },
                zero_dynamics,
                zero_lagrange,
            )
            .is_err()
        );

        let mut mismatched_state_segment = second_segment.clone();
        mismatched_state_segment.state_dimension = 2;
        mismatched_state_segment.support_states =
            two_state_polynomial_support(&mismatched_state_segment.grid, 1.0, 2.0);
        assert!(
            evaluate_lgr_pseudospectral_transcription(
                &LgrPseudospectralTranscription {
                    segments: vec![first_segment.clone(), mismatched_state_segment]
                },
                zero_dynamics,
                zero_lagrange,
            )
            .is_err()
        );

        let mut mismatched_control_segment = second_segment;
        mismatched_control_segment.control_dimension = 1;
        mismatched_control_segment.collocation_controls =
            vec![0.0; mismatched_control_segment.grid.collocation_count()];
        assert!(
            evaluate_lgr_pseudospectral_transcription(
                &LgrPseudospectralTranscription {
                    segments: vec![first_segment, mismatched_control_segment]
                },
                zero_dynamics,
                zero_lagrange,
            )
            .is_err()
        );
    }

    #[test]
    fn lgr_mesh_refinement_rejects_malformed_inputs() {
        let candidate = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        let reference = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 6)],
        };

        let mut bad_settings = mesh_refinement_settings();
        bad_settings.relative_error_tolerance = 0.0;
        assert!(
            estimate_lgr_pseudospectral_mesh_refinement(&candidate, &reference, &bad_settings)
                .is_err()
        );

        let empty_reference = LgrPseudospectralTranscription {
            segments: Vec::new(),
        };
        assert!(
            estimate_lgr_pseudospectral_mesh_refinement(
                &candidate,
                &empty_reference,
                &mesh_refinement_settings(),
            )
            .is_err()
        );

        let lower_degree_reference = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 3)],
        };
        assert!(
            estimate_lgr_pseudospectral_mesh_refinement(
                &candidate,
                &lower_degree_reference,
                &mesh_refinement_settings(),
            )
            .is_err()
        );

        let mismatched_interval_reference = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.1, 6)],
        };
        assert!(
            estimate_lgr_pseudospectral_mesh_refinement(
                &candidate,
                &mismatched_interval_reference,
                &mesh_refinement_settings(),
            )
            .is_err()
        );
    }

    #[test]
    fn lgr_mesh_refinement_application_rejects_inconsistent_report() {
        let source = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        let mut mismatched_index_report = mesh_refinement_report_for(
            &source,
            LgrPseudospectralMeshRefinementAction::RaiseDegree {
                collocation_count: 6,
            },
        );
        mismatched_index_report.recommendations[0].segment_index = 1;
        assert!(
            apply_lgr_pseudospectral_mesh_refinement(&source, &mismatched_index_report).is_err()
        );

        let bad_raise_report = mesh_refinement_report_for(
            &source,
            LgrPseudospectralMeshRefinementAction::RaiseDegree {
                collocation_count: 4,
            },
        );
        assert!(apply_lgr_pseudospectral_mesh_refinement(&source, &bad_raise_report).is_err());
    }

    #[test]
    fn lgr_mesh_refinement_loop_rejects_bad_budget_or_reference() {
        let source = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.0, 4)],
        };
        assert!(
            refine_lgr_pseudospectral_mesh_with_reference(
                &source,
                &mesh_refinement_settings(),
                0,
                |_iteration_index, candidate| {
                    Ok(polynomial_reference_for_candidate(candidate, 2, 0.0))
                },
            )
            .is_err()
        );

        let bad_reference = LgrPseudospectralTranscription {
            segments: vec![polynomial_lgr_segment_with_count(0.0, 1.1, 6)],
        };
        assert!(
            refine_lgr_pseudospectral_mesh_with_reference(
                &source,
                &mesh_refinement_settings(),
                1,
                |_iteration_index, _candidate| Ok(bad_reference.clone()),
            )
            .is_err()
        );
    }

    fn max_exp_interpolation_error(collocation_count: usize, dense_sample_count: usize) -> f64 {
        let grid = legendre_gauss_radau_grid(collocation_count).expect("LGR grid builds");
        let support_states = grid
            .support_nodes
            .iter()
            .map(|node| {
                let time_s = test_physical_time(0.0, 1.0, *node);
                time_s.exp()
            })
            .collect::<Vec<_>>();
        let segment = LgrPseudospectralSegment {
            initial_time_s: 0.0,
            final_time_s: 1.0,
            grid,
            state_dimension: 1,
            control_dimension: 0,
            support_states,
            collocation_controls: Vec::new(),
        };
        let barycentric_weights =
            barycentric_weights(&segment.grid.support_nodes).expect("barycentric weights build");

        let mut max_error = 0.0_f64;
        for sample_index in 0..dense_sample_count {
            let time_s = (sample_index as f64) / ((dense_sample_count - 1) as f64);
            let normalized_node = 2.0 * time_s - 1.0;
            let interpolated =
                interpolate_lgr_support_state(&segment, &barycentric_weights, normalized_node, 0)
                    .expect("support interpolation evaluates");
            max_error = max_error.max((interpolated - time_s.exp()).abs());
        }
        max_error
    }

    fn lgr_brachistochrone_transcription(
        problem: BrachistochroneBenchmarkProblem,
        collocation_count: usize,
    ) -> Result<LgrPseudospectralTranscription, TrajoptError> {
        let mut segments = Vec::with_capacity(problem.segment_count);
        for segment_index in 0..problem.segment_count {
            let initial_time_s =
                problem.optimal_time_s * (segment_index as f64) / (problem.segment_count as f64);
            let final_time_s = problem.optimal_time_s * ((segment_index + 1) as f64)
                / (problem.segment_count as f64);
            segments.push(lgr_brachistochrone_segment(
                problem,
                initial_time_s,
                final_time_s,
                collocation_count,
            )?);
        }
        let transcription = LgrPseudospectralTranscription { segments };
        validate_lgr_transcription(&transcription)?;
        Ok(transcription)
    }

    #[cfg(feature = "sqp")]
    fn lgr_brachistochrone_ocp(
        problem: BrachistochroneBenchmarkProblem,
        collocation_count: usize,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        let time_nodes_s = lgr_uniform_time_nodes(problem.segment_count, problem.optimal_time_s);
        let segment_collocation_counts = vec![collocation_count; problem.segment_count];
        let boundary_constraints = vec![
            LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0)?,
            LgrPseudospectralBoundaryConstraint::new(0, 1, 0.0)?,
            LgrPseudospectralBoundaryConstraint::new(0, 2, 0.0)?,
            LgrPseudospectralBoundaryConstraint::new(
                problem.segment_count,
                0,
                problem.endpoint_x_m,
            )?,
            LgrPseudospectralBoundaryConstraint::new(
                problem.segment_count,
                1,
                problem.endpoint_y_down_m,
            )?,
        ];
        LgrPseudospectralFixedMeshOcp::new(
            time_nodes_s,
            segment_collocation_counts,
            3,
            1,
            boundary_constraints,
        )
    }

    fn lgr_uniform_time_nodes(segment_count: usize, final_time_s: f64) -> Vec<f64> {
        let mut time_nodes_s = Vec::with_capacity(segment_count + 1);
        for index in 0..=segment_count {
            time_nodes_s.push(final_time_s * (index as f64) / (segment_count as f64));
        }
        time_nodes_s
    }

    #[cfg(feature = "sqp")]
    fn lgr_brachistochrone_decision_blocks(
        problem: BrachistochroneBenchmarkProblem,
        collocation_count: usize,
    ) -> Result<(Vec<f64>, Vec<f64>), TrajoptError> {
        let transcription = lgr_brachistochrone_transcription(problem, collocation_count)?;
        let support_state_count = transcription
            .segments
            .iter()
            .map(|segment| segment.support_states.len())
            .sum::<usize>();
        let collocation_control_count = transcription
            .segments
            .iter()
            .map(|segment| segment.collocation_controls.len())
            .sum::<usize>();
        let mut support_states = Vec::with_capacity(support_state_count);
        let mut collocation_controls = Vec::with_capacity(collocation_control_count);
        for segment in transcription.segments {
            support_states.extend(segment.support_states);
            collocation_controls.extend(segment.collocation_controls);
        }
        Ok((support_states, collocation_controls))
    }

    #[cfg(feature = "sqp")]
    fn perturb_lgr_brachistochrone_control_decision(
        layout: &LgrPseudospectralDecisionLayout,
        exact: &[f64],
    ) -> Vec<f64> {
        let mut perturbed = exact.to_vec();
        for (segment_index, grid) in layout.segment_grids.iter().enumerate() {
            for collocation_index in 0..grid.collocation_count() {
                let offset = layout.segment_control_offsets[segment_index]
                    + collocation_index * layout.control_dimension;
                let phase = (segment_index as f64) + 0.5 * (collocation_index as f64) + 1.0;
                perturbed[offset] += 0.001 * phase.sin();
            }
        }
        perturbed
    }

    #[cfg(feature = "sqp")]
    fn lgr_brachistochrone_free_control_restoration_bounds(
        layout: &LgrPseudospectralDecisionLayout,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut lower = vec![-10.0; layout.variable_count];
        let mut upper = vec![10.0; layout.variable_count];
        for (segment_index, grid) in layout.segment_grids.iter().enumerate() {
            for support_index in 0..grid.support_count() {
                let offset = layout.segment_state_offsets[segment_index]
                    + support_index * layout.state_dimension;
                lower[offset] = -0.1;
                upper[offset] = 1.2;
                lower[offset + 1] = -0.1;
                upper[offset + 1] = 1.2;
                lower[offset + 2] = 0.0;
                upper[offset + 2] = 10.0;
            }
            for collocation_index in 0..grid.collocation_count() {
                let offset = layout.segment_control_offsets[segment_index]
                    + collocation_index * layout.control_dimension;
                lower[offset] = 0.0;
                upper[offset] = core::f64::consts::PI;
            }
        }
        (lower, upper)
    }

    fn lgr_goddard_ocp(
        problem: GoddardBenchmarkProblem,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        let time_nodes_s = lgr_uniform_time_nodes(problem.segment_count, problem.sample_final_time);
        let segment_collocation_counts = vec![problem.lgr_collocation_count; problem.segment_count];
        lgr_goddard_ocp_from_mesh(problem, time_nodes_s, segment_collocation_counts)
    }

    fn lgr_goddard_ocp_from_mesh(
        problem: GoddardBenchmarkProblem,
        time_nodes_s: Vec<f64>,
        segment_collocation_counts: Vec<usize>,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        let boundary_constraints = vec![
            LgrPseudospectralBoundaryConstraint::new(0, 0, problem.initial_altitude)?,
            LgrPseudospectralBoundaryConstraint::new(0, 1, problem.initial_velocity)?,
            LgrPseudospectralBoundaryConstraint::new(0, 2, problem.initial_mass)?,
            LgrPseudospectralBoundaryConstraint::new(
                segment_collocation_counts.len(),
                2,
                problem.minimum_mass,
            )?,
        ];
        LgrPseudospectralFixedMeshOcp::new(
            time_nodes_s,
            segment_collocation_counts,
            3,
            1,
            boundary_constraints,
        )
    }

    fn lgr_goddard_seed_decision_blocks(
        problem: GoddardBenchmarkProblem,
    ) -> Result<(Vec<f64>, Vec<f64>), TrajoptError> {
        let ocp = lgr_goddard_ocp(problem)?;
        let mut support_states = Vec::with_capacity(ocp.layout.state_variable_count);
        let mut collocation_controls = Vec::with_capacity(ocp.layout.control_variable_count);
        for segment_index in 0..ocp.layout.segment_count() {
            let initial_time_s = ocp.layout.time_nodes_s[segment_index];
            let final_time_s = ocp.layout.time_nodes_s[segment_index + 1];
            let grid = &ocp.layout.segment_grids[segment_index];
            for node in &grid.support_nodes {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                support_states.extend_from_slice(&goddard_seed_state(problem, time_s));
            }
            for node in &grid.collocation_nodes {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                collocation_controls.push(goddard_seed_thrust(problem, time_s));
            }
        }
        validate_finite(
            "Goddard seed support states must be finite",
            &support_states,
        )?;
        validate_finite(
            "Goddard seed collocation controls must be finite",
            &collocation_controls,
        )?;
        Ok((support_states, collocation_controls))
    }

    fn lgr_goddard_residual_localized_split_seed(
        problem: GoddardBenchmarkProblem,
    ) -> Result<GoddardResidualLocalizedSplitSeed, TrajoptError> {
        let ocp = lgr_goddard_ocp(problem)?;
        let (support_states, collocation_controls) = lgr_goddard_seed_decision_blocks(problem)?;
        let decision = ocp
            .layout
            .pack_decision_vector(&support_states, &collocation_controls)?;
        let transcription = ocp.layout.transcription_from_decision(&decision)?;
        let transcription_report = evaluate_lgr_pseudospectral_transcription(
            &transcription,
            move |_time_s: f64, state: &[f64], control: &[f64]| {
                goddard_dynamics(problem, state, control)
            },
            zero_lagrange,
        )?;

        let segment_norms = transcription_report
            .segment_reports
            .iter()
            .map(|segment| segment.residual_norm)
            .collect::<Vec<_>>();
        let (peak_segment_index, &peak_segment_norm) = segment_norms
            .iter()
            .enumerate()
            .max_by(|left, right| {
                left.1
                    .partial_cmp(right.1)
                    .expect("Goddard segment residuals are finite")
            })
            .expect("Goddard segment residuals exist");
        let differential_norm = norm(
            &transcription_report.residuals[..transcription_report.differential_residual_count],
        );
        let recomposed_norm = segment_norms
            .iter()
            .map(|value| value * value)
            .sum::<f64>()
            .sqrt();

        let estimates = transcription
            .segments
            .iter()
            .enumerate()
            .map(|(segment_index, segment)| {
                let relative_error = if segment_index == peak_segment_index {
                    peak_segment_norm
                } else {
                    0.0
                };
                LgrPseudospectralSegmentErrorEstimate {
                    segment_index,
                    collocation_count: segment.grid.collocation_count(),
                    reference_collocation_count: segment.grid.collocation_count(),
                    max_abs_state_error: relative_error,
                    state_scale: 1.0,
                    relative_error,
                }
            })
            .collect::<Vec<_>>();
        let recommendations = transcription
            .segments
            .iter()
            .enumerate()
            .map(|(segment_index, segment)| {
                let action = if segment_index == peak_segment_index {
                    LgrPseudospectralMeshRefinementAction::Split {
                        segment_count: 2,
                        collocation_count: segment.grid.collocation_count(),
                    }
                } else {
                    LgrPseudospectralMeshRefinementAction::Accept
                };
                LgrPseudospectralSegmentRefinementRecommendation {
                    segment_index,
                    collocation_count: segment.grid.collocation_count(),
                    action,
                }
            })
            .collect::<Vec<_>>();
        let refinement_report = LgrPseudospectralMeshRefinementReport {
            estimates,
            recommendations,
            max_relative_error: peak_segment_norm,
            converged: false,
            refined_segment_count: 1,
        };
        let application =
            apply_lgr_pseudospectral_mesh_refinement(&transcription, &refinement_report)?;

        Ok(GoddardResidualLocalizedSplitSeed {
            application,
            peak_segment_index,
            peak_segment_norm,
            differential_norm,
            recomposed_norm,
        })
    }

    #[cfg(feature = "sqp")]
    fn lgr_goddard_ocp_from_transcription(
        problem: GoddardBenchmarkProblem,
        transcription: &LgrPseudospectralTranscription,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        validate_lgr_transcription(transcription)?;
        let mut time_nodes_s = Vec::with_capacity(transcription.segments.len() + 1);
        time_nodes_s.push(transcription.segments[0].initial_time_s);
        time_nodes_s.extend(
            transcription
                .segments
                .iter()
                .map(|segment| segment.final_time_s),
        );
        let segment_collocation_counts = transcription
            .segments
            .iter()
            .map(|segment| segment.grid.collocation_count())
            .collect::<Vec<_>>();
        lgr_goddard_ocp_from_mesh(problem, time_nodes_s, segment_collocation_counts)
    }

    #[cfg(feature = "sqp")]
    fn lgr_decision_blocks_from_transcription(
        transcription: &LgrPseudospectralTranscription,
    ) -> Result<(Vec<f64>, Vec<f64>), TrajoptError> {
        validate_lgr_transcription(transcription)?;
        let support_state_count = transcription
            .segments
            .iter()
            .map(|segment| segment.support_states.len())
            .sum::<usize>();
        let collocation_control_count = transcription
            .segments
            .iter()
            .map(|segment| segment.collocation_controls.len())
            .sum::<usize>();
        let mut support_states = Vec::with_capacity(support_state_count);
        let mut collocation_controls = Vec::with_capacity(collocation_control_count);
        for segment in &transcription.segments {
            support_states.extend_from_slice(&segment.support_states);
            collocation_controls.extend_from_slice(&segment.collocation_controls);
        }
        validate_finite(
            "LGR transcription support states must be finite",
            &support_states,
        )?;
        validate_finite(
            "LGR transcription collocation controls must be finite",
            &collocation_controls,
        )?;
        Ok((support_states, collocation_controls))
    }

    #[cfg(feature = "sqp")]
    fn lgr_goddard_decision_bounds(
        problem: GoddardBenchmarkProblem,
        layout: &LgrPseudospectralDecisionLayout,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut lower = vec![-1.0e3; layout.variable_count];
        let mut upper = vec![1.0e3; layout.variable_count];
        for segment_index in 0..layout.segment_count() {
            let grid = &layout.segment_grids[segment_index];
            for support_index in 0..grid.support_count() {
                let offset = layout.segment_state_offsets[segment_index]
                    + support_index * layout.state_dimension;
                lower[offset] = problem.initial_altitude;
                upper[offset] = 2.0;
                lower[offset + 1] = 0.0;
                upper[offset + 1] = 2.0;
                lower[offset + 2] = problem.minimum_mass;
                upper[offset + 2] = problem.initial_mass;
            }
            for collocation_index in 0..grid.collocation_count() {
                let offset = layout.segment_control_offsets[segment_index]
                    + collocation_index * layout.control_dimension;
                lower[offset] = 0.0;
                upper[offset] = problem.maximum_thrust;
            }
        }
        (lower, upper)
    }

    fn goddard_seed_state(problem: GoddardBenchmarkProblem, time_s: f64) -> [f64; 3] {
        let tau = (time_s / problem.sample_final_time).clamp(0.0, 1.0);
        let altitude = problem.initial_altitude + 0.04 * tau * tau;
        let velocity = 0.08 * tau * (1.0 - tau);
        let mass = problem.initial_mass - (problem.initial_mass - problem.minimum_mass) * tau;
        [altitude, velocity, mass]
    }

    fn goddard_seed_thrust(problem: GoddardBenchmarkProblem, _time_s: f64) -> f64 {
        problem.exhaust_velocity * (problem.initial_mass - problem.minimum_mass)
            / problem.sample_final_time
    }

    #[cfg(feature = "sqp")]
    fn goddard_published_final_altitude_gap_report(
        problem: GoddardBenchmarkProblem,
        candidate_final_time: f64,
        candidate_final_altitude: f64,
        equality_residual_norm: f64,
    ) -> Result<GoddardPublishedFinalAltitudeGapReport, TrajoptError> {
        let values = [
            candidate_final_time,
            candidate_final_altitude,
            equality_residual_norm,
            problem.published_final_time,
            problem.published_final_altitude,
            problem.published_final_time_tolerance,
            problem.published_final_altitude_tolerance,
            problem.published_max_constraint_violation,
        ];
        validate_finite("Goddard published-band gap values must be finite", &values)?;
        if candidate_final_time <= 0.0
            || candidate_final_altitude <= 0.0
            || equality_residual_norm < 0.0
            || problem.published_final_time <= 0.0
            || problem.published_final_altitude <= 0.0
            || problem.published_final_time_tolerance < 0.0
            || problem.published_final_altitude_tolerance < 0.0
            || problem.published_max_constraint_violation < 0.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "Goddard published-band gap values are outside benchmark bounds",
            });
        }
        let absolute_final_time_error = (candidate_final_time - problem.published_final_time).abs();
        let absolute_final_altitude_error =
            (candidate_final_altitude - problem.published_final_altitude).abs();
        let within_published_time_band =
            absolute_final_time_error <= problem.published_final_time_tolerance;
        let within_published_altitude_band =
            absolute_final_altitude_error <= problem.published_final_altitude_tolerance;
        let within_published_residual_band =
            equality_residual_norm <= problem.published_max_constraint_violation;
        Ok(GoddardPublishedFinalAltitudeGapReport {
            candidate_final_time,
            published_final_time: problem.published_final_time,
            absolute_final_time_error,
            candidate_final_altitude,
            published_final_altitude: problem.published_final_altitude,
            absolute_final_altitude_error,
            equality_residual_norm,
            within_published_time_band,
            within_published_altitude_band,
            within_published_residual_band,
            published_closure_met: within_published_time_band
                && within_published_altitude_band
                && within_published_residual_band,
        })
    }

    fn goddard_dynamics(
        problem: GoddardBenchmarkProblem,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if state.len() != 3 || control.len() != 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Goddard dynamics dimensions are inconsistent",
            });
        }
        validate_finite("Goddard state must be finite", state)?;
        validate_finite("Goddard control must be finite", control)?;
        let altitude = state[0];
        let velocity = state[1];
        let mass = state[2];
        let thrust = control[0];
        if altitude <= 0.0 || mass <= 0.0 || thrust < 0.0 || thrust > problem.maximum_thrust {
            return Err(TrajoptError::InvalidPayload {
                reason: "Goddard dynamics values are outside benchmark bounds",
            });
        }
        let drag = goddard_drag(problem, altitude, velocity);
        let gravity = goddard_gravity(problem, altitude);
        Ok(vec![
            velocity,
            (thrust - drag) / mass - gravity,
            -thrust / problem.exhaust_velocity,
        ])
    }

    fn goddard_drag(problem: GoddardBenchmarkProblem, altitude: f64, velocity: f64) -> f64 {
        problem.drag_coefficient
            * velocity
            * velocity
            * (-problem.drag_altitude_scale * (altitude - problem.initial_altitude)
                / problem.initial_altitude)
                .exp()
    }

    fn goddard_gravity(problem: GoddardBenchmarkProblem, altitude: f64) -> f64 {
        problem.initial_gravity
            * (problem.initial_altitude / altitude)
            * (problem.initial_altitude / altitude)
    }

    fn goddard_singular_arc_residual(
        problem: GoddardBenchmarkProblem,
        state: &[f64; 3],
    ) -> Result<f64, TrajoptError> {
        let altitude = state[0];
        let velocity = state[1];
        let mass = state[2];
        if altitude <= 0.0 || velocity < 0.0 || mass <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "Goddard singular-arc state is outside benchmark bounds",
            });
        }
        let drag = goddard_drag(problem, altitude, velocity);
        let gravity = goddard_gravity(problem, altitude);
        Ok(mass * gravity - (1.0 + velocity / problem.exhaust_velocity) * drag)
    }

    fn goddard_singular_arc_sample(
        problem: GoddardBenchmarkProblem,
    ) -> Result<[f64; 3], TrajoptError> {
        let altitude = 1.005;
        let mass = 0.8;
        let mut lower_velocity = 1.0e-6;
        let mut upper_velocity = 0.2;
        for _ in 0..80 {
            let mid_velocity = 0.5 * (lower_velocity + upper_velocity);
            let state = [altitude, mid_velocity, mass];
            let residual = goddard_singular_arc_residual(problem, &state)?;
            if residual > 0.0 {
                lower_velocity = mid_velocity;
            } else {
                upper_velocity = mid_velocity;
            }
        }
        Ok([altitude, 0.5 * (lower_velocity + upper_velocity), mass])
    }

    fn lgr_brachistochrone_segment(
        problem: BrachistochroneBenchmarkProblem,
        initial_time_s: f64,
        final_time_s: f64,
        collocation_count: usize,
    ) -> Result<LgrPseudospectralSegment, TrajoptError> {
        let grid = legendre_gauss_radau_grid(collocation_count)?;
        let mut support_states = Vec::with_capacity(grid.support_count() * 3);
        for node in &grid.support_nodes {
            let time_s = test_physical_time(initial_time_s, final_time_s, *node);
            let theta = problem.theta_final_rad * time_s / problem.optimal_time_s;
            let (state, _control) = brachistochrone_sample(problem, theta);
            support_states.extend_from_slice(&state);
        }
        let mut collocation_controls = Vec::with_capacity(grid.collocation_count());
        for node in &grid.collocation_nodes {
            let time_s = test_physical_time(initial_time_s, final_time_s, *node);
            let theta = problem.theta_final_rad * time_s / problem.optimal_time_s;
            let (_state, control) = brachistochrone_sample(problem, theta);
            collocation_controls.push(control);
        }
        let segment = LgrPseudospectralSegment {
            initial_time_s,
            final_time_s,
            grid,
            state_dimension: 3,
            control_dimension: 1,
            support_states,
            collocation_controls,
        };
        validate_lgr_segment(&segment)?;
        Ok(segment)
    }

    fn brachistochrone_sample(
        problem: BrachistochroneBenchmarkProblem,
        theta: f64,
    ) -> ([f64; 3], f64) {
        let x = problem.cycloid_scale_m * (theta - theta.sin());
        let y_down = problem.cycloid_scale_m * (1.0 - theta.cos());
        let speed = (2.0 * problem.gravity_m_s2 * y_down).sqrt();
        let control_angle_rad = core::f64::consts::FRAC_PI_2 - 0.5 * theta;
        ([x, y_down, speed], control_angle_rad)
    }

    fn brachistochrone_dynamics(
        gravity_m_s2: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if state.len() != 3 || control.len() != 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "brachistochrone dynamics dimensions are inconsistent",
            });
        }
        let speed = state[2];
        let control_angle_rad = control[0];
        if !gravity_m_s2.is_finite()
            || gravity_m_s2 <= 0.0
            || !speed.is_finite()
            || speed < 0.0
            || !control_angle_rad.is_finite()
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "brachistochrone dynamics values are invalid",
            });
        }
        Ok(vec![
            speed * control_angle_rad.cos(),
            speed * control_angle_rad.sin(),
            gravity_m_s2 * control_angle_rad.sin(),
        ])
    }

    fn polynomial_lgr_segment(initial_time_s: f64, final_time_s: f64) -> LgrPseudospectralSegment {
        polynomial_lgr_segment_with_count(initial_time_s, final_time_s, 4)
    }

    fn polynomial_lgr_segment_with_count(
        initial_time_s: f64,
        final_time_s: f64,
        collocation_count: usize,
    ) -> LgrPseudospectralSegment {
        let grid = legendre_gauss_radau_grid(collocation_count).expect("LGR grid builds");
        let support_states = polynomial_support(&grid, initial_time_s, final_time_s);
        LgrPseudospectralSegment {
            initial_time_s,
            final_time_s,
            grid,
            state_dimension: 1,
            control_dimension: 0,
            support_states,
            collocation_controls: Vec::new(),
        }
    }

    fn shifted_polynomial_lgr_segment(
        initial_time_s: f64,
        final_time_s: f64,
        collocation_count: usize,
        state_shift: f64,
    ) -> LgrPseudospectralSegment {
        let mut segment =
            polynomial_lgr_segment_with_count(initial_time_s, final_time_s, collocation_count);
        segment.support_states[0] += state_shift;
        segment
    }

    fn polynomial_support(
        grid: &LegendreGaussRadauGrid,
        initial_time_s: f64,
        final_time_s: f64,
    ) -> Vec<f64> {
        grid.support_nodes
            .iter()
            .map(|node| {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                time_s * time_s
            })
            .collect::<Vec<_>>()
    }

    fn controlled_polynomial_lgr_segment_with_count(
        initial_time_s: f64,
        final_time_s: f64,
        collocation_count: usize,
    ) -> LgrPseudospectralSegment {
        let grid = legendre_gauss_radau_grid(collocation_count).expect("LGR grid builds");
        let support_states = polynomial_support(&grid, initial_time_s, final_time_s);
        let collocation_controls = grid
            .collocation_nodes
            .iter()
            .map(|node| {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                1.0 + 3.0 * time_s
            })
            .collect::<Vec<_>>();
        LgrPseudospectralSegment {
            initial_time_s,
            final_time_s,
            grid,
            state_dimension: 1,
            control_dimension: 1,
            support_states,
            collocation_controls,
        }
    }

    fn polynomial_lgr_decision_blocks(
        layout: &LgrPseudospectralDecisionLayout,
    ) -> (Vec<f64>, Vec<f64>) {
        let mut support_states = Vec::with_capacity(layout.state_variable_count);
        let mut collocation_controls = Vec::with_capacity(layout.control_variable_count);

        for (segment_index, grid) in layout.segment_grids.iter().enumerate() {
            let initial_time_s = layout.time_nodes_s[segment_index];
            let final_time_s = layout.time_nodes_s[segment_index + 1];
            for node in &grid.support_nodes {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                for state_index in 0..layout.state_dimension {
                    support_states.push(time_s * time_s + state_index as f64);
                }
            }
            for node in &grid.collocation_nodes {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                for control_index in 0..layout.control_dimension {
                    collocation_controls.push(1.0 + 3.0 * time_s + control_index as f64);
                }
            }
        }

        (support_states, collocation_controls)
    }

    fn lgr_sparse_linear_ocp() -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        LgrPseudospectralFixedMeshOcp::new(
            vec![0.0, 1.0, 2.0],
            vec![2, 2],
            2,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0)?,
                LgrPseudospectralBoundaryConstraint::new(2, 1, 5.0)?,
            ],
        )?
        .with_path_constraints(vec![LgrPseudospectralSoftPathConstraint::new(
            0,
            0,
            LgrPseudospectralScalarVariable::ControlComponent { component: 0 },
            None,
            Some(0.5),
            4.0,
        )?])?
        .with_quadratic_control_costs(vec![LgrPseudospectralQuadraticControlCost::new(0, 0.25)?])
    }

    fn lgr_sparse_linear_decision(ocp: &LgrPseudospectralFixedMeshOcp) -> Vec<f64> {
        let (support_states, collocation_controls) = polynomial_lgr_decision_blocks(&ocp.layout);
        ocp.layout
            .pack_decision_vector(&support_states, &collocation_controls)
            .expect("LGR sparse linear decision packs")
    }

    fn mesh_refinement_settings() -> LgrPseudospectralMeshRefinementSettings {
        LgrPseudospectralMeshRefinementSettings {
            relative_error_tolerance: 1.0e-6,
            max_collocation_count: 6,
            degree_raise_step: 2,
            subdivision_factor: 2,
        }
    }

    fn polynomial_reference_for_candidate(
        candidate: &LgrPseudospectralTranscription,
        degree_increment: usize,
        state_shift: f64,
    ) -> LgrPseudospectralTranscription {
        let segments = candidate
            .segments
            .iter()
            .map(|segment| {
                let collocation_count = segment.grid.collocation_count() + degree_increment;
                let mut reference = if segment.control_dimension == 0 {
                    polynomial_lgr_segment_with_count(
                        segment.initial_time_s,
                        segment.final_time_s,
                        collocation_count,
                    )
                } else {
                    controlled_polynomial_lgr_segment_with_count(
                        segment.initial_time_s,
                        segment.final_time_s,
                        collocation_count,
                    )
                };
                reference.support_states[0] += state_shift;
                reference
            })
            .collect::<Vec<_>>();
        LgrPseudospectralTranscription { segments }
    }

    fn mesh_refinement_report_for(
        source: &LgrPseudospectralTranscription,
        action: LgrPseudospectralMeshRefinementAction,
    ) -> LgrPseudospectralMeshRefinementReport {
        let collocation_count = source.segments[0].grid.collocation_count();
        let refined = action != LgrPseudospectralMeshRefinementAction::Accept;
        LgrPseudospectralMeshRefinementReport {
            estimates: vec![LgrPseudospectralSegmentErrorEstimate {
                segment_index: 0,
                collocation_count,
                reference_collocation_count: collocation_count + 2,
                max_abs_state_error: if refined { 1.0e-3 } else { 0.0 },
                state_scale: 1.0,
                relative_error: if refined { 1.0e-3 } else { 0.0 },
            }],
            recommendations: vec![LgrPseudospectralSegmentRefinementRecommendation {
                segment_index: 0,
                collocation_count,
                action,
            }],
            max_relative_error: if refined { 1.0e-3 } else { 0.0 },
            converged: !refined,
            refined_segment_count: if refined { 1 } else { 0 },
        }
    }

    fn assert_polynomial_state_and_linear_control_seed(segment: &LgrPseudospectralSegment) {
        for (support_index, node) in segment.grid.support_nodes.iter().enumerate() {
            let time_s = test_physical_time(segment.initial_time_s, segment.final_time_s, *node);
            let actual = segment.support_states[support_index * segment.state_dimension];
            assert!(
                (actual - time_s * time_s).abs() < 1.0e-11,
                "support state {actual:e} != {:e}",
                time_s * time_s
            );
        }
        for (collocation_index, node) in segment.grid.collocation_nodes.iter().enumerate() {
            let time_s = test_physical_time(segment.initial_time_s, segment.final_time_s, *node);
            let actual =
                segment.collocation_controls[collocation_index * segment.control_dimension];
            assert!(
                (actual - (1.0 + 3.0 * time_s)).abs() < 1.0e-11,
                "control {actual:e} != {:e}",
                1.0 + 3.0 * time_s
            );
        }
    }

    fn two_state_polynomial_support(
        grid: &LegendreGaussRadauGrid,
        initial_time_s: f64,
        final_time_s: f64,
    ) -> Vec<f64> {
        let mut support_states = Vec::with_capacity(grid.support_count() * 2);
        for node in &grid.support_nodes {
            let time_s = test_physical_time(initial_time_s, final_time_s, *node);
            support_states.push(time_s * time_s);
            support_states.push(time_s);
        }
        support_states
    }

    fn test_physical_time(initial_time_s: f64, final_time_s: f64, normalized_node: f64) -> f64 {
        let midpoint_time_s = 0.5 * (initial_time_s + final_time_s);
        let half_duration_s = 0.5 * (final_time_s - initial_time_s);
        midpoint_time_s + half_duration_s * normalized_node
    }

    fn zero_dynamics(
        _time_s: f64,
        state: &[f64],
        _control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![0.0; state.len()])
    }

    fn linear_lgr_dynamics(
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![
            2.0 * state[0] - 0.5 * state[1] + 3.0 * control[0] + time_s,
            -state[0] + 4.0 * state[1] - 2.0 * control[0] + 0.5 * time_s,
        ])
    }

    fn affine_lgr_lagrange(
        time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<f64, TrajoptError> {
        Ok(time_s + 0.25 * state[0] - 0.5 * state[1] + control[0])
    }

    fn lgr_dense_gauss_newton_hessian(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        regularization: f64,
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        let mut hessian = vec![0.0; column_count * column_count];
        for residual_row in 0..row_count {
            for row in 0..column_count {
                let row_value = dense_jacobian[residual_row * column_count + row];
                for column in 0..column_count {
                    hessian[row * column_count + column] +=
                        row_value * dense_jacobian[residual_row * column_count + column];
                }
            }
        }
        for diagonal in 0..column_count {
            hessian[diagonal * column_count + diagonal] += regularization;
        }
        hessian
    }

    fn lgr_dense_gauss_newton_step_rhs(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        residuals: &[f64],
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        assert_eq!(residuals.len(), row_count);
        let mut rhs = vec![0.0; column_count];
        for row in 0..row_count {
            for column in 0..column_count {
                rhs[column] -= dense_jacobian[row * column_count + column] * residuals[row];
            }
        }
        rhs
    }

    fn lgr_dense_augmented_kkt(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        regularization: f64,
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        let dimension = row_count + column_count;
        let mut kkt = vec![0.0; dimension * dimension];
        for variable in 0..column_count {
            kkt[variable * dimension + variable] = regularization;
        }
        for residual in 0..row_count {
            let index = column_count + residual;
            kkt[index * dimension + index] = -1.0;
        }
        for residual in 0..row_count {
            for variable in 0..column_count {
                let value = dense_jacobian[residual * column_count + variable];
                let residual_index = column_count + residual;
                kkt[variable * dimension + residual_index] = value;
                kkt[residual_index * dimension + variable] = value;
            }
        }
        kkt
    }

    fn lgr_dense_constrained_augmented_kkt(
        row_count: usize,
        column_count: usize,
        dense_jacobian: &[f64],
        hard_path_inequality_report: &LgrPseudospectralPathInequalityReport,
        active_rows: &[usize],
        regularization: f64,
    ) -> Vec<f64> {
        assert_eq!(dense_jacobian.len(), row_count * column_count);
        assert_eq!(hard_path_inequality_report.variable_count, column_count);
        let dimension = row_count + column_count + active_rows.len();
        let mut kkt = vec![0.0; dimension * dimension];
        for variable in 0..column_count {
            kkt[variable * dimension + variable] = regularization;
        }
        for residual in 0..row_count {
            let index = column_count + residual;
            kkt[index * dimension + index] = -1.0;
        }
        for residual in 0..row_count {
            for variable in 0..column_count {
                let value = dense_jacobian[residual * column_count + variable];
                let residual_index = column_count + residual;
                kkt[variable * dimension + residual_index] = value;
                kkt[residual_index * dimension + variable] = value;
            }
        }
        for (active_index, source_row) in active_rows.iter().enumerate() {
            let active_kkt_index = column_count + row_count + active_index;
            let row_offset = *source_row * column_count;
            for variable in 0..column_count {
                let value = hard_path_inequality_report.inequality_matrix[row_offset + variable];
                kkt[variable * dimension + active_kkt_index] = value;
                kkt[active_kkt_index * dimension + variable] = value;
            }
        }
        kkt
    }

    fn lgr_dense_matrix_vector(dimension: usize, dense_matrix: &[f64], vector: &[f64]) -> Vec<f64> {
        assert_eq!(dense_matrix.len(), dimension * dimension);
        assert_eq!(vector.len(), dimension);
        let mut product = vec![0.0; dimension];
        for row in 0..dimension {
            for column in 0..dimension {
                product[row] += dense_matrix[row * dimension + column] * vector[column];
            }
        }
        product
    }

    fn lgr_dense_rectangular_matrix_vector(
        row_count: usize,
        column_count: usize,
        dense_matrix: &[f64],
        vector: &[f64],
    ) -> Vec<f64> {
        assert_eq!(dense_matrix.len(), row_count * column_count);
        assert_eq!(vector.len(), column_count);
        let mut product = vec![0.0; row_count];
        for row in 0..row_count {
            for column in 0..column_count {
                product[row] += dense_matrix[row * column_count + column] * vector[column];
            }
        }
        product
    }

    #[cfg(feature = "sqp")]
    fn unit_control_lgr_dynamics(
        _time_s: f64,
        _state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![control[0]])
    }

    #[cfg(feature = "sqp")]
    fn three_state_lgr_reference_dynamics(
        _time_s: f64,
        _state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![control[0], 0.0, 0.0])
    }

    #[cfg(feature = "sqp")]
    fn unit_interval_lgr_integrator_ocp(
        table: &LgrScalarIntegratorBenchmarkTable,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        let time_nodes_s = (0..=table.segment_count)
            .map(|index| (index as f64) / (table.segment_count as f64))
            .collect::<Vec<_>>();
        LgrPseudospectralFixedMeshOcp::new(
            time_nodes_s,
            vec![table.collocation_count; table.segment_count],
            1,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0)?,
                LgrPseudospectralBoundaryConstraint::new(table.segment_count, 0, 1.0)?,
            ],
        )
    }

    #[cfg(feature = "sqp")]
    fn lgr_bryson_denham_ocp(
        problem: LgrBrysonDenhamBenchmarkProblem,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        let time_nodes_s = (0..=problem.segment_count)
            .map(|index| (index as f64) / (problem.segment_count as f64))
            .collect::<Vec<_>>();
        let mut hard_path_constraints =
            Vec::with_capacity(problem.segment_count * problem.lgr_collocation_count);
        for segment_index in 0..problem.segment_count {
            for collocation_index in 0..problem.lgr_collocation_count {
                hard_path_constraints.push(LgrPseudospectralHardPathConstraint::new(
                    segment_index,
                    collocation_index,
                    LgrPseudospectralScalarVariable::StateComponent { component: 0 },
                    None,
                    Some(problem.state_bound),
                )?);
            }
        }

        LgrPseudospectralFixedMeshOcp::new(
            time_nodes_s,
            vec![problem.lgr_collocation_count; problem.segment_count],
            2,
            1,
            vec![
                LgrPseudospectralBoundaryConstraint::new(0, 0, 0.0)?,
                LgrPseudospectralBoundaryConstraint::new(0, 1, 1.0)?,
                LgrPseudospectralBoundaryConstraint::new(problem.segment_count, 0, 0.0)?,
                LgrPseudospectralBoundaryConstraint::new(problem.segment_count, 1, -1.0)?,
            ],
        )?
        .with_hard_path_constraints(hard_path_constraints)
    }

    #[cfg(feature = "sqp")]
    fn lgr_bryson_denham_quadratic_ocp(
        problem: LgrBrysonDenhamBenchmarkProblem,
    ) -> Result<LgrPseudospectralFixedMeshOcp, TrajoptError> {
        lgr_bryson_denham_ocp(problem)?
            .with_quadratic_control_costs(vec![LgrPseudospectralQuadraticControlCost::new(0, 0.5)?])
    }

    #[cfg(feature = "sqp")]
    fn lgr_bryson_denham_dynamics(
        _time_s: f64,
        state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        if state.len() != 2 || control.len() != 1 {
            return Err(TrajoptError::InvalidPayload {
                reason: "LGR Bryson-Denham dynamics dimensions are inconsistent",
            });
        }
        Ok(vec![state[1], control[0]])
    }

    #[cfg(feature = "sqp")]
    fn assert_lgr_scalar_integrator_analytic_solution(
        ocp: &LgrPseudospectralFixedMeshOcp,
        decision: &[f64],
        table: &LgrScalarIntegratorBenchmarkTable,
    ) {
        for segment_index in 0..ocp.layout.segment_count() {
            let initial_time_s = ocp.layout.time_nodes_s[segment_index];
            let final_time_s = ocp.layout.time_nodes_s[segment_index + 1];
            let grid = &ocp.layout.segment_grids[segment_index];
            for (support_index, node) in grid.support_nodes.iter().enumerate() {
                let expected_time_s = test_physical_time(initial_time_s, final_time_s, *node);
                let state = ocp
                    .layout
                    .segment_support_state(decision, segment_index, support_index)
                    .expect("support state")[0];
                assert!(
                    (state - expected_time_s).abs() <= table.max_state_error,
                    "segment {segment_index} support {support_index} expected {expected_time_s}, got {state}"
                );
            }
            for collocation_index in 0..grid.collocation_count() {
                let control = ocp
                    .layout
                    .segment_collocation_control(decision, segment_index, collocation_index)
                    .expect("collocation control")[0];
                assert!(
                    (control - 1.0).abs() <= table.max_control_error,
                    "segment {segment_index} collocation {collocation_index} expected 1, got {control}"
                );
            }
        }
    }

    #[cfg(feature = "sqp")]
    fn assert_lgr_bryson_denham_active_arc_solution(
        ocp: &LgrPseudospectralFixedMeshOcp,
        decision: &[f64],
        problem: LgrBrysonDenhamBenchmarkProblem,
    ) {
        for segment_index in 0..ocp.layout.segment_count() {
            let initial_time_s = ocp.layout.time_nodes_s[segment_index];
            let final_time_s = ocp.layout.time_nodes_s[segment_index + 1];
            let grid = &ocp.layout.segment_grids[segment_index];
            for (support_index, node) in grid.support_nodes.iter().enumerate() {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                let state = ocp
                    .layout
                    .segment_support_state(decision, segment_index, support_index)
                    .expect("support state");
                let (expected_x, expected_v, _) =
                    lgr_bryson_denham_active_arc_sample(time_s, problem);
                assert!(
                    (state[0] - expected_x).abs() <= problem.max_state_error,
                    "LGR Bryson-Denham x({time_s}) expected {expected_x}, got {}",
                    state[0]
                );
                assert!(
                    (state[1] - expected_v).abs() <= problem.max_state_error,
                    "LGR Bryson-Denham v({time_s}) expected {expected_v}, got {}",
                    state[1]
                );
            }
            for (collocation_index, node) in grid.collocation_nodes.iter().enumerate() {
                let time_s = test_physical_time(initial_time_s, final_time_s, *node);
                let control = ocp
                    .layout
                    .segment_collocation_control(decision, segment_index, collocation_index)
                    .expect("collocation control")[0];
                let (_, _, expected_u) = lgr_bryson_denham_active_arc_sample(time_s, problem);
                assert!(
                    (control - expected_u).abs() <= problem.max_control_error,
                    "LGR Bryson-Denham u({time_s}) expected {expected_u}, got {control}"
                );
            }
        }
    }

    #[cfg(feature = "sqp")]
    fn lgr_bryson_denham_active_arc_sample(
        time_s: f64,
        problem: LgrBrysonDenhamBenchmarkProblem,
    ) -> (f64, f64, f64) {
        if time_s <= problem.entry_time_s + 1.0e-14 {
            let x = time_s - 3.0 * time_s * time_s + 3.0 * time_s * time_s * time_s;
            let v = 1.0 - 6.0 * time_s + 9.0 * time_s * time_s;
            let u = 18.0 * time_s - 6.0;
            (x, v, u)
        } else if time_s <= problem.exit_time_s + 1.0e-14 {
            (problem.state_bound, 0.0, 0.0)
        } else {
            let tau = time_s - problem.exit_time_s;
            let x = problem.state_bound - 3.0 * tau * tau * tau;
            let v = -9.0 * tau * tau;
            let u = -18.0 * tau;
            (x, v, u)
        }
    }

    #[cfg(feature = "sqp")]
    fn control_lgr_path_function(
        _time_s: f64,
        _state: &[f64],
        control: &[f64],
    ) -> Result<Vec<f64>, TrajoptError> {
        Ok(vec![control[0]])
    }

    struct LinearLgrDynamicsJacobian;

    impl CollocationDynamicsJacobian for LinearLgrDynamicsJacobian {
        fn eval_with_jacobian(
            &mut self,
            time_s: f64,
            state: &[f64],
            control: &[f64],
        ) -> Result<CollocationDynamicsJacobianSample, TrajoptError> {
            CollocationDynamicsJacobianSample::new(
                linear_lgr_dynamics(time_s, state, control)?,
                vec![2.0, -0.5, -1.0, 4.0],
                vec![3.0, -2.0],
                2,
                1,
            )
        }
    }

    struct MalformedLgrDynamicsJacobian;

    impl CollocationDynamicsJacobian for MalformedLgrDynamicsJacobian {
        fn eval_with_jacobian(
            &mut self,
            time_s: f64,
            state: &[f64],
            control: &[f64],
        ) -> Result<CollocationDynamicsJacobianSample, TrajoptError> {
            Ok(CollocationDynamicsJacobianSample {
                derivative: linear_lgr_dynamics(time_s, state, control)?,
                state_jacobian: vec![0.0],
                control_jacobian: vec![0.0],
            })
        }
    }

    fn zero_lagrange(_time_s: f64, _state: &[f64], _control: &[f64]) -> Result<f64, TrajoptError> {
        Ok(0.0)
    }

    fn unit_lagrange(_time_s: f64, _state: &[f64], _control: &[f64]) -> Result<f64, TrajoptError> {
        Ok(1.0)
    }
}
