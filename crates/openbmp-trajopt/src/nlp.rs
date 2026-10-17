//! Generic sparse-NLP contracts for direct trajectory optimization.
//!
//! This module is the public solver-facing substrate for WP-07.2/WP-07.3
//! transcriptions. It deliberately stops at validated problem assembly and
//! backend traits; it does not claim a production sparse NLP solver.

use alloc::vec;
use alloc::vec::Vec;

use openbmp_physics::profile::TerminalCondition;

use crate::iload::TrajoptError;
#[cfg(feature = "sqp")]
use crate::sqp::{DenseQpProblem, solve_dense_qp};

/// State/control/path dimensions for an optimal-control problem.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProblemDims {
    /// State dimension.
    pub state_dimension: usize,
    /// Control dimension.
    pub control_dimension: usize,
    /// Path-constraint output dimension. Zero is valid for unconstrained arcs.
    pub path_dimension: usize,
}

impl ProblemDims {
    /// Build validated OCP dimensions.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when state or control dimensions are zero.
    pub fn new(
        state_dimension: usize,
        control_dimension: usize,
        path_dimension: usize,
    ) -> Result<Self, TrajoptError> {
        let dims = Self {
            state_dimension,
            control_dimension,
            path_dimension,
        };
        validate_problem_dims(dims)?;
        Ok(dims)
    }
}

/// Decision-vector and constraint-row dimensions for a sparse NLP.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DecisionLayout {
    /// Number of scalar decision variables.
    pub variable_count: usize,
    /// Number of equality rows.
    pub equality_count: usize,
    /// Number of inequality rows.
    pub inequality_count: usize,
}

impl DecisionLayout {
    /// Build a validated sparse-NLP layout.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the decision vector is empty.
    pub fn new(
        variable_count: usize,
        equality_count: usize,
        inequality_count: usize,
    ) -> Result<Self, TrajoptError> {
        let layout = Self {
            variable_count,
            equality_count,
            inequality_count,
        };
        validate_decision_layout(layout)?;
        Ok(layout)
    }

    /// Total number of scalar constraint rows.
    #[must_use]
    pub fn constraint_count(self) -> usize {
        self.equality_count + self.inequality_count
    }
}

/// Sparse vector with strictly increasing indices.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseVec {
    /// Dense vector dimension.
    pub dimension: usize,
    /// Strictly increasing nonzero indices.
    pub indices: Vec<usize>,
    /// Nonzero values.
    pub values: Vec<f64>,
}

impl SparseVec {
    /// Build a validated sparse vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when indices/values are malformed.
    pub fn new(
        dimension: usize,
        indices: Vec<usize>,
        values: Vec<f64>,
    ) -> Result<Self, TrajoptError> {
        let vector = Self {
            dimension,
            indices,
            values,
        };
        validate_sparse_vec(&vector)?;
        Ok(vector)
    }

    /// Return one coefficient, or zero when the index is structurally absent.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the vector shape or index is invalid.
    pub fn coefficient(&self, index: usize) -> Result<f64, TrajoptError> {
        validate_sparse_vec(self)?;
        if index >= self.dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP vector index is out of range",
            });
        }
        for (entry_index, value) in self.indices.iter().zip(&self.values) {
            if *entry_index == index {
                return Ok(*value);
            }
        }
        Ok(0.0)
    }

    /// Expand the sparse vector to dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the vector shape is invalid.
    pub fn to_dense(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_sparse_vec(self)?;
        let mut dense = vec![0.0; self.dimension];
        for (index, value) in self.indices.iter().zip(&self.values) {
            dense[*index] = *value;
        }
        Ok(dense)
    }
}

/// Generic column-compressed sparse matrix consumed by sparse NLP backends.
#[derive(Clone, Debug, PartialEq)]
pub struct CscMatrix {
    /// Number of rows.
    pub row_count: usize,
    /// Number of columns.
    pub column_count: usize,
    /// CSC column offsets, length `column_count + 1`.
    pub column_offsets: Vec<usize>,
    /// CSC row indices, strictly increasing within each column.
    pub row_indices: Vec<usize>,
    /// CSC nonzero values.
    pub values: Vec<f64>,
}

impl CscMatrix {
    /// Build a validated CSC matrix.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the CSC shape or values are malformed.
    pub fn new(
        row_count: usize,
        column_count: usize,
        column_offsets: Vec<usize>,
        row_indices: Vec<usize>,
        values: Vec<f64>,
    ) -> Result<Self, TrajoptError> {
        let matrix = Self {
            row_count,
            column_count,
            column_offsets,
            row_indices,
            values,
        };
        validate_csc_matrix(&matrix)?;
        Ok(matrix)
    }

    /// Return a single matrix coefficient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the CSC shape or requested index is invalid.
    pub fn coefficient(&self, row: usize, column: usize) -> Result<f64, TrajoptError> {
        validate_csc_matrix(self)?;
        if row >= self.row_count || column >= self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP CSC matrix index is out of range",
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

    /// Expand the matrix to row-major dense storage.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the CSC shape is invalid.
    pub fn to_dense_row_major(&self) -> Result<Vec<f64>, TrajoptError> {
        validate_csc_matrix(self)?;
        let mut dense = vec![0.0; self.row_count * self.column_count];
        for column in 0..self.column_count {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                let row = self.row_indices[index];
                dense[row * self.column_count + column] = self.values[index];
            }
        }
        Ok(dense)
    }

    /// Multiply this CSC matrix by a dense vector.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions are inconsistent or values are
    /// non-finite.
    pub fn multiply_vector(&self, vector: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        validate_csc_matrix(self)?;
        if vector.len() != self.column_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP CSC matrix-vector dimensions are inconsistent",
            });
        }
        validate_finite("sparse NLP matrix-vector input must be finite", vector)?;
        let mut product = vec![0.0; self.row_count];
        for (column, vector_value) in vector.iter().enumerate() {
            for index in self.column_offsets[column]..self.column_offsets[column + 1] {
                product[self.row_indices[index]] += self.values[index] * *vector_value;
            }
        }
        Ok(product)
    }
}

/// Sparse NLP residual stack and Jacobian.
#[derive(Clone, Debug, PartialEq)]
pub struct NlpResidual {
    /// Equality residuals `c(z) = 0`.
    pub equalities: Vec<f64>,
    /// Inequality-row values `d(z)`.
    pub inequalities: Vec<f64>,
    /// Inequality lower bounds. Infinite one-sided bounds are allowed.
    pub inequality_lower_bounds: Vec<f64>,
    /// Inequality upper bounds. Infinite one-sided bounds are allowed.
    pub inequality_upper_bounds: Vec<f64>,
    /// CSC Jacobian of `[c(z); d(z)]` with respect to the decision vector.
    pub jacobian: CscMatrix,
}

impl NlpResidual {
    /// Build a validated sparse-NLP residual report.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when row dimensions, bounds, or Jacobian shape
    /// are malformed.
    pub fn new(
        equalities: Vec<f64>,
        inequalities: Vec<f64>,
        inequality_lower_bounds: Vec<f64>,
        inequality_upper_bounds: Vec<f64>,
        jacobian: CscMatrix,
    ) -> Result<Self, TrajoptError> {
        let residual = Self {
            equalities,
            inequalities,
            inequality_lower_bounds,
            inequality_upper_bounds,
            jacobian,
        };
        validate_nlp_residual(&residual)?;
        Ok(residual)
    }

    /// Total number of residual rows.
    #[must_use]
    pub fn row_count(&self) -> usize {
        self.equalities.len() + self.inequalities.len()
    }

    /// Maximum equality or inequality-bound violation.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the residual report is malformed.
    pub fn max_constraint_violation(&self) -> Result<f64, TrajoptError> {
        validate_nlp_residual(self)?;
        let equality_violation = self
            .equalities
            .iter()
            .fold(0.0_f64, |maximum, value| maximum.max(value.abs()));
        let inequality_violation = self
            .inequalities
            .iter()
            .zip(&self.inequality_lower_bounds)
            .zip(&self.inequality_upper_bounds)
            .fold(0.0_f64, |maximum, ((value, lower), upper)| {
                let lower_violation = if value < lower { lower - value } else { 0.0 };
                let upper_violation = if value > upper { value - upper } else { 0.0 };
                maximum.max(lower_violation).max(upper_violation)
            });
        Ok(equality_violation.max(inequality_violation))
    }
}

/// Sparse-NLP objective assembly report.
#[derive(Clone, Debug, PartialEq)]
pub struct NlpAssemblyReport {
    /// Sparse-NLP decision and row layout.
    pub layout: DecisionLayout,
    /// Sparse-NLP residual and Jacobian.
    pub residual: NlpResidual,
    /// Objective value `F(z)`.
    pub objective: f64,
    /// Sparse objective gradient.
    pub objective_gradient: SparseVec,
}

/// Validation settings for checked sparse-NLP backend solves.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct NlpSolveValidationSettings {
    /// Allowed absolute mismatch between the backend-reported objective and the
    /// objective reassembled at the returned decision vector.
    pub objective_tolerance: f64,
}

impl NlpSolveValidationSettings {
    /// Build validated checked-solve settings.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the objective tolerance is malformed.
    pub fn new(objective_tolerance: f64) -> Result<Self, TrajoptError> {
        let settings = Self {
            objective_tolerance,
        };
        validate_nlp_solve_validation_settings(settings)?;
        Ok(settings)
    }
}

impl Default for NlpSolveValidationSettings {
    fn default() -> Self {
        Self {
            objective_tolerance: 1.0e-9,
        }
    }
}

/// Checked sparse-NLP solve report around a backend adapter call.
#[derive(Clone, Debug, PartialEq)]
pub struct NlpCheckedSolveReport {
    /// Validated sparse-NLP assembly at the initial decision vector.
    pub initial_assembly: NlpAssemblyReport,
    /// Backend solution after dimension and numeric validation.
    pub solution: NlpSolution,
    /// Validated sparse-NLP assembly at [`NlpSolution::z`].
    pub final_assembly: NlpAssemblyReport,
    /// Absolute objective mismatch between [`NlpSolution::objective`] and the
    /// final reassembled objective.
    pub objective_mismatch: f64,
    /// Maximum equality or inequality-bound violation in the final assembly.
    pub final_max_constraint_violation: f64,
}

/// Settings for the feature-gated dense SQP sparse-NLP adapter.
#[cfg(feature = "sqp")]
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseSqpNlpSettings {
    /// Maximum accepted SQP iterations before returning a non-converged report.
    pub max_iterations: usize,
    /// Maximum allowed constraint violation for convergence.
    pub constraint_tolerance: f64,
    /// Maximum infinity norm of `grad F + J^T lambda` for convergence.
    pub stationarity_tolerance: f64,
    /// Minimum accepted step norm used as a non-converged stopping condition.
    pub step_tolerance: f64,
    /// Positive diagonal Hessian regularization for the linearized QP step.
    pub regularization: f64,
    /// Positive penalty applied to equality and inequality violations in the
    /// line-search merit function.
    pub merit_constraint_penalty: f64,
    /// Maximum merit-function backtracks per SQP step.
    pub max_line_search_backtracks: usize,
    /// Smallest line-search scale allowed before the step is rejected.
    pub min_line_search_scale: f64,
}

#[cfg(feature = "sqp")]
impl Default for DenseSqpNlpSettings {
    fn default() -> Self {
        Self {
            max_iterations: 25,
            constraint_tolerance: 1.0e-8,
            stationarity_tolerance: 1.0e-8,
            step_tolerance: 1.0e-10,
            regularization: 1.0,
            merit_constraint_penalty: 100.0,
            max_line_search_backtracks: 12,
            min_line_search_scale: 1.0e-8,
        }
    }
}

/// Deterministic dense SQP adapter for the public sparse-NLP contract.
///
/// This is a checked host-side bridge from [`Transcription`] / [`OptimalControlProblem`]
/// into the existing Clarabel-backed dense QP lane. It reassembles the sparse
/// NLP at each iterate, forms a first-order QP with hard linearized equality
/// rows, linearized inequality-bound rows, the sparse objective gradient, and a
/// diagonal regularization Hessian, then accepts steps through an exact
/// objective-plus-constraint-violation merit function.
///
/// The adapter is intentionally bounded and dense. It is useful for small public
/// benchmark and backend-contract tests, but it is not the production
/// large-sparse NLP/IPOPT backend.
#[cfg(feature = "sqp")]
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct DenseSqpNlpSolver {
    /// Adapter settings.
    pub settings: DenseSqpNlpSettings,
}

#[cfg(feature = "sqp")]
impl DenseSqpNlpSolver {
    /// Build a dense SQP sparse-NLP adapter with explicit settings.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when settings are malformed.
    pub fn new(settings: DenseSqpNlpSettings) -> Result<Self, TrajoptError> {
        validate_dense_sqp_nlp_settings(settings)?;
        Ok(Self { settings })
    }
}

/// Sparse-NLP backend solution.
#[derive(Clone, Debug, PartialEq)]
pub struct NlpSolution {
    /// Solved decision vector.
    pub z: Vec<f64>,
    /// KKT multipliers for the assembled constraint rows.
    pub multipliers: Vec<f64>,
    /// Final objective value.
    pub objective: f64,
    /// Final KKT residual norm.
    pub kkt_residual: f64,
    /// Whether the backend declared convergence.
    pub converged: bool,
    /// Backend iteration count.
    pub iterations: usize,
}

impl NlpSolution {
    /// Build a validated sparse-NLP solution.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when numeric fields are malformed.
    pub fn new(
        z: Vec<f64>,
        multipliers: Vec<f64>,
        objective: f64,
        kkt_residual: f64,
        converged: bool,
        iterations: usize,
    ) -> Result<Self, TrajoptError> {
        let solution = Self {
            z,
            multipliers,
            objective,
            kkt_residual,
            converged,
            iterations,
        };
        validate_nlp_solution(&solution)?;
        Ok(solution)
    }

    /// Validate this solution against a known decision/row layout.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when solution dimensions do not match `layout`.
    pub fn validate_against_layout(&self, layout: DecisionLayout) -> Result<(), TrajoptError> {
        validate_decision_layout(layout)?;
        validate_nlp_solution(self)?;
        if self.z.len() != layout.variable_count {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP solution decision dimensions are inconsistent",
            });
        }
        if self.multipliers.len() != layout.constraint_count() {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP solution multiplier dimensions are inconsistent",
            });
        }
        Ok(())
    }
}

/// Forward-only optimal-control problem consumed by direct transcriptions.
pub trait OptimalControlProblem {
    /// State, control, and path dimensions.
    fn dims(&self) -> ProblemDims;

    /// Evaluate `x_dot = f(x, u, t)`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dynamics cannot be evaluated.
    fn dynamics(
        &self,
        state: &[f64],
        control: &[f64],
        time_s: f64,
    ) -> Result<Vec<f64>, TrajoptError>;

    /// Evaluate the running-cost integrand `L(x, u, t)`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the running cost cannot be evaluated.
    fn running_cost(
        &self,
        state: &[f64],
        control: &[f64],
        time_s: f64,
    ) -> Result<f64, TrajoptError>;

    /// Evaluate the Mayer terminal cost `phi(x_f, t_f)`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the terminal cost cannot be evaluated.
    fn terminal_cost(&self, final_state: &[f64], final_time_s: f64) -> Result<f64, TrajoptError>;

    /// Evaluate path-constraint values.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when path constraints cannot be evaluated.
    fn path_constraints(
        &self,
        state: &[f64],
        control: &[f64],
        time_s: f64,
    ) -> Result<Vec<f64>, TrajoptError>;

    /// Closed terminal-condition vocabulary for target semantics.
    fn terminal_condition(&self) -> &TerminalCondition;

    /// Central-body gravitational parameter.
    fn mu_m3_s2(&self) -> f64;
}

/// Mesh transcription that lowers an OCP to sparse-NLP arrays.
pub trait Transcription {
    /// Decision-vector and constraint-row layout.
    fn layout(&self) -> DecisionLayout;

    /// Assemble constraint residuals and Jacobian.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the transcription cannot be assembled.
    fn assemble(
        &self,
        problem: &dyn OptimalControlProblem,
        z: &[f64],
    ) -> Result<NlpResidual, TrajoptError>;

    /// Assemble objective value and sparse gradient.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the objective cannot be assembled.
    fn objective(
        &self,
        problem: &dyn OptimalControlProblem,
        z: &[f64],
    ) -> Result<(f64, SparseVec), TrajoptError>;
}

/// Sparse NLP backend contract.
pub trait NlpSolver {
    /// Solve a transcribed sparse NLP from `z0`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when assembly or backend solve fails.
    fn solve(
        &self,
        transcription: &dyn Transcription,
        problem: &dyn OptimalControlProblem,
        z0: &[f64],
    ) -> Result<NlpSolution, TrajoptError>;
}

#[cfg(feature = "sqp")]
impl NlpSolver for DenseSqpNlpSolver {
    fn solve(
        &self,
        transcription: &dyn Transcription,
        problem: &dyn OptimalControlProblem,
        z0: &[f64],
    ) -> Result<NlpSolution, TrajoptError> {
        validate_dense_sqp_nlp_settings(self.settings)?;
        let initial = assemble_sparse_nlp(transcription, problem, z0)?;
        let layout = initial.layout;
        let mut z = z0.to_vec();
        let mut multipliers = vec![0.0; layout.constraint_count()];
        let mut converged = false;
        let mut accepted_iterations = 0;

        for iteration in 0..self.settings.max_iterations {
            let assembly = assemble_sparse_nlp(transcription, problem, &z)?;
            if assembly.layout != layout {
                return Err(TrajoptError::InvalidPayload {
                    reason: "dense SQP sparse NLP layout changed during solve",
                });
            }
            let max_violation = assembly.residual.max_constraint_violation()?;
            let stationarity =
                nlp_stationarity_inf_norm(&assembly, &multipliers, layout.variable_count)?;
            if max_violation <= self.settings.constraint_tolerance
                && stationarity <= self.settings.stationarity_tolerance
            {
                converged = true;
                break;
            }

            let step_report = dense_sqp_nlp_step(&assembly, self.settings.regularization)?;
            let step_norm = dense_norm(&step_report.step);
            if step_norm <= self.settings.step_tolerance {
                break;
            }

            let current_merit =
                dense_sqp_nlp_merit(&assembly, self.settings.merit_constraint_penalty)?;
            let accepted = dense_sqp_nlp_accept_step(
                transcription,
                problem,
                layout,
                &z,
                &step_report.step,
                current_merit,
                self.settings,
            )?;
            z = accepted;
            multipliers = step_report.multipliers;
            accepted_iterations = iteration + 1;
        }

        let final_assembly = assemble_sparse_nlp(transcription, problem, &z)?;
        if final_assembly.layout != layout {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP sparse NLP final layout changed during solve",
            });
        }
        let final_violation = final_assembly.residual.max_constraint_violation()?;
        let stationarity =
            nlp_stationarity_inf_norm(&final_assembly, &multipliers, layout.variable_count)?;
        converged |= final_violation <= self.settings.constraint_tolerance
            && stationarity <= self.settings.stationarity_tolerance;
        NlpSolution::new(
            z,
            multipliers,
            final_assembly.objective,
            final_violation.max(stationarity),
            converged,
            accepted_iterations,
        )
    }
}

/// Assemble, solve, and validate a sparse-NLP backend result with default
/// checked-solve settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when preflight assembly, backend solve, solution
/// shape, final assembly, or backend-reported objective consistency fails.
pub fn solve_sparse_nlp_checked(
    solver: &dyn NlpSolver,
    transcription: &dyn Transcription,
    problem: &dyn OptimalControlProblem,
    z0: &[f64],
) -> Result<NlpCheckedSolveReport, TrajoptError> {
    solve_sparse_nlp_checked_with_settings(
        solver,
        transcription,
        problem,
        z0,
        NlpSolveValidationSettings::default(),
    )
}

/// Assemble, solve, and validate a sparse-NLP backend result.
///
/// This is the checked boundary a concrete sparse backend can use before and
/// after crossing its solver API. It intentionally validates only generic
/// sparse-NLP shape and objective consistency; convergence policy stays with
/// the concrete backend/report.
///
/// # Errors
///
/// Returns [`TrajoptError`] when preflight assembly, backend solve, solution
/// shape, final assembly, or backend-reported objective consistency fails.
pub fn solve_sparse_nlp_checked_with_settings(
    solver: &dyn NlpSolver,
    transcription: &dyn Transcription,
    problem: &dyn OptimalControlProblem,
    z0: &[f64],
    settings: NlpSolveValidationSettings,
) -> Result<NlpCheckedSolveReport, TrajoptError> {
    validate_nlp_solve_validation_settings(settings)?;
    let initial_assembly = assemble_sparse_nlp(transcription, problem, z0)?;
    let solution = solver.solve(transcription, problem, z0)?;
    solution.validate_against_layout(initial_assembly.layout)?;
    let final_assembly = assemble_sparse_nlp(transcription, problem, &solution.z)?;
    if final_assembly.layout != initial_assembly.layout {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP final assembly layout changed across backend solve",
        });
    }
    let objective_mismatch = (solution.objective - final_assembly.objective).abs();
    if objective_mismatch > settings.objective_tolerance {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP backend objective is inconsistent with final assembly",
        });
    }
    let final_max_constraint_violation = final_assembly.residual.max_constraint_violation()?;
    Ok(NlpCheckedSolveReport {
        initial_assembly,
        solution,
        final_assembly,
        objective_mismatch,
        final_max_constraint_violation,
    })
}

/// Assemble and validate the sparse-NLP arrays for a transcription/problem pair.
///
/// This is the common preflight check a future IPOPT or sparse-SQP adapter can
/// share before crossing an FFI/backend boundary.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the OCP, initial decision vector, residual
/// shape, or objective gradient is malformed.
pub fn assemble_sparse_nlp(
    transcription: &dyn Transcription,
    problem: &dyn OptimalControlProblem,
    z: &[f64],
) -> Result<NlpAssemblyReport, TrajoptError> {
    let dims = problem.dims();
    validate_problem_dims(dims)?;
    problem
        .terminal_condition()
        .validate()
        .map_err(|_| TrajoptError::InvalidPayload {
            reason: "sparse NLP terminal condition is invalid",
        })?;
    if !problem.mu_m3_s2().is_finite() || problem.mu_m3_s2() <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP gravity parameter must be finite and positive",
        });
    }

    let layout = transcription.layout();
    validate_decision_layout(layout)?;
    if z.len() != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP decision-vector dimensions are inconsistent",
        });
    }
    validate_finite("sparse NLP decision vector must be finite", z)?;

    let residual = transcription.assemble(problem, z)?;
    validate_residual_against_layout(&residual, layout)?;
    let (objective, objective_gradient) = transcription.objective(problem, z)?;
    if !objective.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP objective must be finite",
        });
    }
    validate_sparse_vec(&objective_gradient)?;
    if objective_gradient.dimension != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP objective-gradient dimensions are inconsistent",
        });
    }

    Ok(NlpAssemblyReport {
        layout,
        residual,
        objective,
        objective_gradient,
    })
}

fn validate_problem_dims(dims: ProblemDims) -> Result<(), TrajoptError> {
    if dims.state_dimension == 0 || dims.control_dimension == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP OCP state/control dimensions must be nonzero",
        });
    }
    Ok(())
}

fn validate_decision_layout(layout: DecisionLayout) -> Result<(), TrajoptError> {
    if layout.variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP decision layout must contain variables",
        });
    }
    Ok(())
}

fn validate_sparse_vec(vector: &SparseVec) -> Result<(), TrajoptError> {
    if vector.indices.len() != vector.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP vector indices and values are inconsistent",
        });
    }
    let mut previous = None;
    for index in &vector.indices {
        if *index >= vector.dimension {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP vector index is out of range",
            });
        }
        if previous.is_some_and(|previous_index| *index <= previous_index) {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP vector indices must be strictly increasing",
            });
        }
        previous = Some(*index);
    }
    validate_finite("sparse NLP vector values must be finite", &vector.values)
}

fn validate_csc_matrix(matrix: &CscMatrix) -> Result<(), TrajoptError> {
    if matrix.column_offsets.len() != matrix.column_count + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP CSC column offsets are inconsistent",
        });
    }
    if matrix.column_offsets.first() != Some(&0)
        || matrix.column_offsets.last() != Some(&matrix.values.len())
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP CSC column offsets do not bracket values",
        });
    }
    if matrix.row_indices.len() != matrix.values.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP CSC rows and values are inconsistent",
        });
    }
    for window in matrix.column_offsets.windows(2) {
        if window[0] > window[1] {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP CSC column offsets must be monotonic",
            });
        }
        if window[1] > matrix.values.len() {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP CSC column offset is out of range",
            });
        }
        let rows = &matrix.row_indices[window[0]..window[1]];
        let mut previous = None;
        for row in rows {
            if *row >= matrix.row_count {
                return Err(TrajoptError::InvalidPayload {
                    reason: "sparse NLP CSC row index is out of range",
                });
            }
            if previous.is_some_and(|previous_row| *row <= previous_row) {
                return Err(TrajoptError::InvalidPayload {
                    reason: "sparse NLP CSC rows must be strictly increasing within each column",
                });
            }
            previous = Some(*row);
        }
    }
    validate_finite("sparse NLP CSC values must be finite", &matrix.values)
}

fn validate_nlp_residual(residual: &NlpResidual) -> Result<(), TrajoptError> {
    validate_finite(
        "sparse NLP equality residuals must be finite",
        &residual.equalities,
    )?;
    validate_finite(
        "sparse NLP inequality values must be finite",
        &residual.inequalities,
    )?;
    validate_bounds(
        &residual.inequality_lower_bounds,
        &residual.inequality_upper_bounds,
    )?;
    if residual.inequality_lower_bounds.len() != residual.inequalities.len()
        || residual.inequality_upper_bounds.len() != residual.inequalities.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP inequality bounds are inconsistent",
        });
    }
    validate_csc_matrix(&residual.jacobian)?;
    if residual.jacobian.row_count != residual.row_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP residual Jacobian row dimensions are inconsistent",
        });
    }
    Ok(())
}

fn validate_residual_against_layout(
    residual: &NlpResidual,
    layout: DecisionLayout,
) -> Result<(), TrajoptError> {
    validate_nlp_residual(residual)?;
    if residual.equalities.len() != layout.equality_count
        || residual.inequalities.len() != layout.inequality_count
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP residual row dimensions are inconsistent",
        });
    }
    if residual.jacobian.column_count != layout.variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP residual Jacobian column dimensions are inconsistent",
        });
    }
    Ok(())
}

fn validate_nlp_solution(solution: &NlpSolution) -> Result<(), TrajoptError> {
    validate_finite(
        "sparse NLP solution decision vector must be finite",
        &solution.z,
    )?;
    validate_finite(
        "sparse NLP solution multipliers must be finite",
        &solution.multipliers,
    )?;
    if !solution.objective.is_finite() {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP solution objective must be finite",
        });
    }
    if !solution.kkt_residual.is_finite() || solution.kkt_residual < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP solution KKT residual must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_nlp_solve_validation_settings(
    settings: NlpSolveValidationSettings,
) -> Result<(), TrajoptError> {
    if !settings.objective_tolerance.is_finite() || settings.objective_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP checked-solve objective tolerance must be finite and non-negative",
        });
    }
    Ok(())
}

fn validate_bounds(lower_bounds: &[f64], upper_bounds: &[f64]) -> Result<(), TrajoptError> {
    if lower_bounds.len() != upper_bounds.len() {
        return Err(TrajoptError::InvalidPayload {
            reason: "sparse NLP inequality bound dimensions are inconsistent",
        });
    }
    for (lower, upper) in lower_bounds.iter().zip(upper_bounds) {
        if lower.is_nan() || upper.is_nan() || lower > upper {
            return Err(TrajoptError::InvalidPayload {
                reason: "sparse NLP inequality bounds are invalid",
            });
        }
    }
    Ok(())
}

fn validate_finite(reason: &'static str, values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

#[cfg(feature = "sqp")]
struct DenseSqpNlpStepReport {
    step: Vec<f64>,
    multipliers: Vec<f64>,
}

#[cfg(feature = "sqp")]
fn validate_dense_sqp_nlp_settings(settings: DenseSqpNlpSettings) -> Result<(), TrajoptError> {
    if settings.max_iterations == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP requires at least one iteration",
        });
    }
    if !settings.constraint_tolerance.is_finite() || settings.constraint_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP constraint tolerance must be finite and non-negative",
        });
    }
    if !settings.stationarity_tolerance.is_finite() || settings.stationarity_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP stationarity tolerance must be finite and non-negative",
        });
    }
    if !settings.step_tolerance.is_finite() || settings.step_tolerance < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP step tolerance must be finite and non-negative",
        });
    }
    if !settings.regularization.is_finite() || settings.regularization <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP regularization must be finite and positive",
        });
    }
    if !settings.merit_constraint_penalty.is_finite() || settings.merit_constraint_penalty <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP merit penalty must be finite and positive",
        });
    }
    if settings.max_line_search_backtracks == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP line search requires at least one backtrack",
        });
    }
    if !settings.min_line_search_scale.is_finite()
        || settings.min_line_search_scale <= 0.0
        || settings.min_line_search_scale > 1.0
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP minimum line-search scale must be in (0, 1]",
        });
    }
    Ok(())
}

#[cfg(feature = "sqp")]
fn dense_sqp_nlp_step(
    assembly: &NlpAssemblyReport,
    regularization: f64,
) -> Result<DenseSqpNlpStepReport, TrajoptError> {
    let variable_count = assembly.layout.variable_count;
    let mut hessian = vec![0.0; variable_count * variable_count];
    for index in 0..variable_count {
        hessian[index * variable_count + index] = regularization;
    }
    let linear = assembly.objective_gradient.to_dense()?;
    let jacobian = assembly.residual.jacobian.to_dense_row_major()?;
    let equality_count = assembly.layout.equality_count;
    let equality_matrix = jacobian[..equality_count * variable_count].to_vec();
    let equality_rhs = assembly
        .residual
        .equalities
        .iter()
        .map(|value| -*value)
        .collect::<Vec<_>>();

    let mut inequality_matrix = Vec::new();
    let mut inequality_rhs = Vec::new();
    let mut converted_rows = Vec::new();
    for row in 0..assembly.layout.inequality_count {
        let value = assembly.residual.inequalities[row];
        let row_offset = (equality_count + row) * variable_count;
        let row_slice = &jacobian[row_offset..row_offset + variable_count];
        let lower = assembly.residual.inequality_lower_bounds[row];
        let upper = assembly.residual.inequality_upper_bounds[row];
        if lower.is_finite() {
            inequality_matrix.extend(row_slice.iter().map(|coefficient| -*coefficient));
            inequality_rhs.push(value - lower);
            converted_rows.push((row, -1.0));
        }
        if upper.is_finite() {
            inequality_matrix.extend_from_slice(row_slice);
            inequality_rhs.push(upper - value);
            converted_rows.push((row, 1.0));
        }
    }

    let qp = DenseQpProblem::new(
        variable_count,
        hessian,
        linear,
        equality_matrix,
        equality_rhs,
        inequality_matrix,
        inequality_rhs,
    )?;
    let solution = solve_dense_qp(&qp)?;
    let mut multipliers = vec![0.0; assembly.layout.constraint_count()];
    multipliers[..equality_count].copy_from_slice(&solution.equality_dual);
    for ((row, sign), dual) in converted_rows.iter().zip(&solution.inequality_dual) {
        multipliers[equality_count + *row] += sign * *dual;
    }

    Ok(DenseSqpNlpStepReport {
        step: solution.primal,
        multipliers,
    })
}

#[cfg(feature = "sqp")]
fn dense_sqp_nlp_accept_step(
    transcription: &dyn Transcription,
    problem: &dyn OptimalControlProblem,
    layout: DecisionLayout,
    z: &[f64],
    step: &[f64],
    current_merit: f64,
    settings: DenseSqpNlpSettings,
) -> Result<Vec<f64>, TrajoptError> {
    let mut scale = 1.0;
    for _ in 0..settings.max_line_search_backtracks {
        let trial = z
            .iter()
            .zip(step)
            .map(|(value, delta)| value + scale * delta)
            .collect::<Vec<_>>();
        let assembly = assemble_sparse_nlp(transcription, problem, &trial)?;
        if assembly.layout != layout {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP sparse NLP trial layout changed during line search",
            });
        }
        let trial_merit = dense_sqp_nlp_merit(&assembly, settings.merit_constraint_penalty)?;
        if trial_merit < current_merit
            || assembly.residual.max_constraint_violation()? <= settings.constraint_tolerance
        {
            return Ok(trial);
        }
        scale *= 0.5;
        if scale < settings.min_line_search_scale {
            break;
        }
    }
    Err(TrajoptError::InvalidPayload {
        reason: "dense SQP sparse NLP line search failed to reduce merit",
    })
}

#[cfg(feature = "sqp")]
fn dense_sqp_nlp_merit(
    assembly: &NlpAssemblyReport,
    constraint_penalty: f64,
) -> Result<f64, TrajoptError> {
    Ok(assembly.objective + constraint_penalty * nlp_constraint_violation_l1(&assembly.residual)?)
}

#[cfg(feature = "sqp")]
fn nlp_constraint_violation_l1(residual: &NlpResidual) -> Result<f64, TrajoptError> {
    validate_nlp_residual(residual)?;
    let equality_sum = residual
        .equalities
        .iter()
        .map(|value| value.abs())
        .sum::<f64>();
    let inequality_sum = residual
        .inequalities
        .iter()
        .zip(&residual.inequality_lower_bounds)
        .zip(&residual.inequality_upper_bounds)
        .map(|((value, lower), upper)| {
            let lower_violation = if value < lower { lower - value } else { 0.0 };
            let upper_violation = if value > upper { value - upper } else { 0.0 };
            lower_violation + upper_violation
        })
        .sum::<f64>();
    Ok(equality_sum + inequality_sum)
}

#[cfg(feature = "sqp")]
fn nlp_stationarity_inf_norm(
    assembly: &NlpAssemblyReport,
    multipliers: &[f64],
    variable_count: usize,
) -> Result<f64, TrajoptError> {
    if multipliers.len() != assembly.layout.constraint_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP sparse NLP multiplier dimensions are inconsistent",
        });
    }
    let mut stationarity = assembly.objective_gradient.to_dense()?;
    for (column, stationarity_value) in stationarity.iter_mut().enumerate().take(variable_count) {
        for index in assembly.residual.jacobian.column_offsets[column]
            ..assembly.residual.jacobian.column_offsets[column + 1]
        {
            let row = assembly.residual.jacobian.row_indices[index];
            *stationarity_value += assembly.residual.jacobian.values[index] * multipliers[row];
        }
    }
    Ok(stationarity
        .iter()
        .map(|value| value.abs())
        .fold(0.0_f64, f64::max))
}

#[cfg(feature = "sqp")]
fn dense_norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Clone, Debug)]
    struct ToyOcp {
        terminal_condition: TerminalCondition,
    }

    impl ToyOcp {
        fn new() -> Self {
            Self {
                terminal_condition: TerminalCondition::ApogeeRadius {
                    radius_m: 7_000_000.0,
                },
            }
        }
    }

    impl OptimalControlProblem for ToyOcp {
        fn dims(&self) -> ProblemDims {
            ProblemDims {
                state_dimension: 1,
                control_dimension: 1,
                path_dimension: 1,
            }
        }

        fn dynamics(
            &self,
            _state: &[f64],
            control: &[f64],
            _time_s: f64,
        ) -> Result<Vec<f64>, TrajoptError> {
            Ok(vec![control[0]])
        }

        fn running_cost(
            &self,
            _state: &[f64],
            control: &[f64],
            _time_s: f64,
        ) -> Result<f64, TrajoptError> {
            Ok(0.5 * control[0] * control[0])
        }

        fn terminal_cost(
            &self,
            final_state: &[f64],
            _final_time_s: f64,
        ) -> Result<f64, TrajoptError> {
            Ok(final_state[0])
        }

        fn path_constraints(
            &self,
            state: &[f64],
            _control: &[f64],
            _time_s: f64,
        ) -> Result<Vec<f64>, TrajoptError> {
            Ok(vec![state[0]])
        }

        fn terminal_condition(&self) -> &TerminalCondition {
            &self.terminal_condition
        }

        fn mu_m3_s2(&self) -> f64 {
            3.986_004_418e14
        }
    }

    struct ToyTranscription;

    impl Transcription for ToyTranscription {
        fn layout(&self) -> DecisionLayout {
            DecisionLayout {
                variable_count: 2,
                equality_count: 1,
                inequality_count: 1,
            }
        }

        fn assemble(
            &self,
            problem: &dyn OptimalControlProblem,
            z: &[f64],
        ) -> Result<NlpResidual, TrajoptError> {
            let dynamics = problem.dynamics(&z[0..1], &z[1..2], 0.0)?;
            let path = problem.path_constraints(&z[0..1], &z[1..2], 0.0)?;
            let jacobian =
                CscMatrix::new(2, 2, vec![0, 1, 3], vec![1, 0, 1], vec![1.0, -1.0, 1.0])?;
            NlpResidual::new(
                vec![dynamics[0] - z[0]],
                path,
                vec![f64::NEG_INFINITY],
                vec![2.0],
                jacobian,
            )
        }

        fn objective(
            &self,
            problem: &dyn OptimalControlProblem,
            z: &[f64],
        ) -> Result<(f64, SparseVec), TrajoptError> {
            let running = problem.running_cost(&z[0..1], &z[1..2], 0.0)?;
            let terminal = problem.terminal_cost(&z[0..1], 1.0)?;
            Ok((
                running + terminal,
                SparseVec::new(2, vec![0, 1], vec![1.0, z[1]])?,
            ))
        }
    }

    struct ToySolver {
        z: Vec<f64>,
        multipliers: Vec<f64>,
        objective_offset: f64,
    }

    impl NlpSolver for ToySolver {
        fn solve(
            &self,
            transcription: &dyn Transcription,
            problem: &dyn OptimalControlProblem,
            _z0: &[f64],
        ) -> Result<NlpSolution, TrajoptError> {
            let assembly = assemble_sparse_nlp(transcription, problem, &self.z)?;
            NlpSolution::new(
                self.z.clone(),
                self.multipliers.clone(),
                assembly.objective + self.objective_offset,
                assembly.residual.max_constraint_violation()?,
                true,
                3,
            )
        }
    }

    #[test]
    fn sparse_nlp_csc_and_residual_reports_validate() -> Result<(), TrajoptError> {
        let matrix = CscMatrix::new(2, 3, vec![0, 1, 2, 3], vec![0, 1, 1], vec![1.0, -2.0, 3.0])?;
        assert_eq!(matrix.coefficient(1, 1)?.to_bits(), (-2.0_f64).to_bits());
        assert_eq!(matrix.coefficient(0, 2)?.to_bits(), 0.0_f64.to_bits());
        assert_eq!(matrix.multiply_vector(&[2.0, 3.0, 4.0])?, vec![2.0, 6.0]);
        assert_eq!(
            matrix.to_dense_row_major()?,
            vec![1.0, 0.0, 0.0, 0.0, -2.0, 3.0]
        );

        let gradient = SparseVec::new(3, vec![0, 2], vec![4.0, -1.0])?;
        assert_eq!(gradient.coefficient(1)?.to_bits(), 0.0_f64.to_bits());
        assert_eq!(gradient.to_dense()?, vec![4.0, 0.0, -1.0]);

        let residual = NlpResidual::new(
            vec![0.5],
            vec![1.2],
            vec![0.0],
            vec![1.0],
            CscMatrix::new(2, 3, vec![0, 1, 2, 3], vec![0, 1, 1], vec![1.0, -2.0, 3.0])?,
        )?;
        assert_eq!(residual.row_count(), 2);
        assert_eq!(
            residual.max_constraint_violation()?.to_bits(),
            0.5_f64.to_bits()
        );
        Ok(())
    }

    #[test]
    fn sparse_nlp_assembly_accepts_closed_terminal_condition_contract() -> Result<(), TrajoptError>
    {
        let report = assemble_sparse_nlp(&ToyTranscription, &ToyOcp::new(), &[1.0, 1.5])?;
        assert_eq!(report.layout.constraint_count(), 2);
        assert_eq!(report.residual.equalities, vec![0.5]);
        assert_eq!(report.residual.inequalities, vec![1.0]);
        assert_eq!(report.objective.to_bits(), 2.125_f64.to_bits());
        assert_eq!(report.objective_gradient.to_dense()?, vec![1.0, 1.5]);
        assert_eq!(
            report.residual.max_constraint_violation()?.to_bits(),
            0.5_f64.to_bits()
        );
        Ok(())
    }

    #[test]
    fn sparse_nlp_assembly_rejects_malformed_backend_shapes() -> Result<(), TrajoptError> {
        let bad_matrix = CscMatrix::new(2, 2, vec![0, 2, 1], vec![0], vec![1.0]);
        assert!(matches!(
            bad_matrix,
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let bad_vector = SparseVec::new(3, vec![2, 2], vec![1.0, 2.0]);
        assert!(matches!(
            bad_vector,
            Err(TrajoptError::InvalidPayload { .. })
        ));

        let layout = DecisionLayout::new(2, 1, 1)?;
        let solution = NlpSolution::new(vec![1.0], vec![0.0, 0.0], 1.0, 0.0, true, 2)?;
        assert!(matches!(
            solution.validate_against_layout(layout),
            Err(TrajoptError::InvalidPayload { .. })
        ));
        Ok(())
    }

    #[test]
    fn sparse_nlp_checked_solve_validates_backend_output() -> Result<(), TrajoptError> {
        let solver = ToySolver {
            z: vec![1.0, 1.0],
            multipliers: vec![0.25, -0.5],
            objective_offset: 0.0,
        };

        let report =
            solve_sparse_nlp_checked(&solver, &ToyTranscription, &ToyOcp::new(), &[1.0, 1.5])?;
        assert_eq!(
            report.initial_assembly.objective.to_bits(),
            2.125_f64.to_bits()
        );
        assert_eq!(report.solution.iterations, 3);
        assert_eq!(report.final_assembly.objective.to_bits(), 1.5_f64.to_bits());
        assert_eq!(report.objective_mismatch.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            report.final_max_constraint_violation.to_bits(),
            0.0_f64.to_bits()
        );
        Ok(())
    }

    #[test]
    fn sparse_nlp_checked_solve_rejects_objective_mismatch() -> Result<(), TrajoptError> {
        let solver = ToySolver {
            z: vec![1.0, 1.0],
            multipliers: vec![0.25, -0.5],
            objective_offset: 1.0e-3,
        };

        let report = solve_sparse_nlp_checked_with_settings(
            &solver,
            &ToyTranscription,
            &ToyOcp::new(),
            &[1.0, 1.5],
            NlpSolveValidationSettings::new(1.0e-6)?,
        );
        assert!(matches!(report, Err(TrajoptError::InvalidPayload { .. })));
        Ok(())
    }

    #[cfg(feature = "sqp")]
    struct UnconstrainedQuadraticTranscription;

    #[cfg(feature = "sqp")]
    impl Transcription for UnconstrainedQuadraticTranscription {
        fn layout(&self) -> DecisionLayout {
            DecisionLayout {
                variable_count: 1,
                equality_count: 0,
                inequality_count: 0,
            }
        }

        fn assemble(
            &self,
            _problem: &dyn OptimalControlProblem,
            _z: &[f64],
        ) -> Result<NlpResidual, TrajoptError> {
            NlpResidual::new(
                Vec::new(),
                Vec::new(),
                Vec::new(),
                Vec::new(),
                CscMatrix::new(0, 1, vec![0, 0], Vec::new(), Vec::new())?,
            )
        }

        fn objective(
            &self,
            _problem: &dyn OptimalControlProblem,
            z: &[f64],
        ) -> Result<(f64, SparseVec), TrajoptError> {
            let residual = z[0] - 2.0;
            Ok((
                0.5 * residual * residual,
                SparseVec::new(1, vec![0], vec![residual])?,
            ))
        }
    }

    #[cfg(feature = "sqp")]
    struct EqualityBoundTranscription;

    #[cfg(feature = "sqp")]
    impl Transcription for EqualityBoundTranscription {
        fn layout(&self) -> DecisionLayout {
            DecisionLayout {
                variable_count: 1,
                equality_count: 1,
                inequality_count: 1,
            }
        }

        fn assemble(
            &self,
            _problem: &dyn OptimalControlProblem,
            z: &[f64],
        ) -> Result<NlpResidual, TrajoptError> {
            NlpResidual::new(
                vec![z[0] - 1.0],
                vec![z[0]],
                vec![0.0],
                vec![1.5],
                CscMatrix::new(2, 1, vec![0, 2], vec![0, 1], vec![1.0, 1.0])?,
            )
        }

        fn objective(
            &self,
            _problem: &dyn OptimalControlProblem,
            z: &[f64],
        ) -> Result<(f64, SparseVec), TrajoptError> {
            Ok((0.5 * z[0] * z[0], SparseVec::new(1, vec![0], vec![z[0]])?))
        }
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn dense_sqp_nlp_solver_reduces_unconstrained_quadratic() -> Result<(), TrajoptError> {
        let solver = DenseSqpNlpSolver::new(DenseSqpNlpSettings {
            max_iterations: 8,
            regularization: 1.0,
            ..DenseSqpNlpSettings::default()
        })?;

        let report = solve_sparse_nlp_checked(
            &solver,
            &UnconstrainedQuadraticTranscription,
            &ToyOcp::new(),
            &[0.0],
        )?;

        assert!((report.solution.z[0] - 2.0).abs() <= 1.0e-8);
        assert!(report.solution.converged);
        assert!(report.solution.objective <= 1.0e-16);
        assert!(report.objective_mismatch <= 1.0e-12);
        Ok(())
    }

    #[cfg(feature = "sqp")]
    #[test]
    fn dense_sqp_nlp_solver_restores_equality_and_bound_feasibility() -> Result<(), TrajoptError> {
        let solver = DenseSqpNlpSolver::new(DenseSqpNlpSettings {
            max_iterations: 8,
            regularization: 1.0,
            stationarity_tolerance: 2.0,
            ..DenseSqpNlpSettings::default()
        })?;

        let report =
            solve_sparse_nlp_checked(&solver, &EqualityBoundTranscription, &ToyOcp::new(), &[0.2])?;

        assert!((report.solution.z[0] - 1.0).abs() <= 1.0e-8);
        assert!(report.solution.converged);
        assert!(report.final_max_constraint_violation <= 1.0e-8);
        assert_eq!(report.solution.multipliers.len(), 2);
        Ok(())
    }
}
