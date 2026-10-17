//! SQP subproblem primitives for trajectory optimization.
//!
//! This module is feature-gated behind `sqp`. It provides a deterministic
//! Clarabel-backed dense QP lane plus a bounded nonlinear least-squares SQP
//! driver for the WP-07.2 fixed-mesh SQP path. The public API accepts dense
//! row-major data and converts it to Clarabel CSC matrices internally; the
//! sparse/AD transcription path remains a later WP-07.2 layer.

use alloc::vec;
use alloc::vec::Vec;

use clarabel::{
    algebra::CscMatrix,
    solver::{DefaultSolver, IPSolver, NonnegativeConeT, SolverStatus, ZeroConeT},
};
use openbmp_convex::deterministic_clarabel_settings;

use crate::iload::TrajoptError;

/// Residual callback used by the dense nonlinear least-squares SQP driver.
pub trait SqpResidualModel {
    /// Evaluate residuals at `variables`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the residual model cannot be evaluated.
    fn residual(&mut self, variables: &[f64]) -> Result<Vec<f64>, TrajoptError>;
}

impl<F> SqpResidualModel for F
where
    F: for<'variables> FnMut(&'variables [f64]) -> Result<Vec<f64>, TrajoptError>,
{
    fn residual(&mut self, variables: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        self(variables)
    }
}

/// Inequality callback used by the constrained dense SQP driver.
pub trait SqpInequalityModel {
    /// Evaluate inequality rows `g(variables) <= 0`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the inequality model cannot be evaluated.
    fn inequality(&mut self, variables: &[f64]) -> Result<Vec<f64>, TrajoptError>;
}

impl<F> SqpInequalityModel for F
where
    F: for<'variables> FnMut(&'variables [f64]) -> Result<Vec<f64>, TrajoptError>,
{
    fn inequality(&mut self, variables: &[f64]) -> Result<Vec<f64>, TrajoptError> {
        self(variables)
    }
}

/// Clarabel settings type used by the trajectory-optimization SQP lane.
pub type TrajoptClarabelSettings = openbmp_convex::ClarabelSettings;

/// Returns the deterministic Clarabel settings profile for offline trajopt QPs.
#[must_use]
pub fn deterministic_sqp_settings() -> TrajoptClarabelSettings {
    deterministic_clarabel_settings()
}

/// Dense convex quadratic program in Clarabel-compatible inequality form.
///
/// The problem is:
///
/// ```text
/// minimize 0.5 * x' H x + q' x
/// subject to Aeq x = beq
///            Aineq x <= bineq
/// ```
///
/// Matrices are row-major. Lower bounds can be represented by negating an
/// inequality row.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseQpProblem {
    /// Number of decision variables.
    pub variable_count: usize,
    /// Row-major symmetric Hessian `H`, length `variable_count^2`.
    pub hessian: Vec<f64>,
    /// Linear objective vector `q`.
    pub linear: Vec<f64>,
    /// Row-major equality matrix `Aeq`.
    pub equality_matrix: Vec<f64>,
    /// Equality right-hand side `beq`.
    pub equality_rhs: Vec<f64>,
    /// Row-major inequality matrix `Aineq`.
    pub inequality_matrix: Vec<f64>,
    /// Inequality right-hand side `bineq`.
    pub inequality_rhs: Vec<f64>,
}

impl DenseQpProblem {
    /// Build a dense QP and validate dimensions and finite values.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when dimensions disagree, values are non-finite,
    /// or the Hessian is not symmetric.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        variable_count: usize,
        hessian: Vec<f64>,
        linear: Vec<f64>,
        equality_matrix: Vec<f64>,
        equality_rhs: Vec<f64>,
        inequality_matrix: Vec<f64>,
        inequality_rhs: Vec<f64>,
    ) -> Result<Self, TrajoptError> {
        let problem = Self {
            variable_count,
            hessian,
            linear,
            equality_matrix,
            equality_rhs,
            inequality_matrix,
            inequality_rhs,
        };
        validate_dense_qp(&problem)?;
        Ok(problem)
    }
}

/// Linearized least-squares SQP step with box bounds.
///
/// This forms the convex QP:
///
/// ```text
/// minimize 0.5 * ||J step + residual||^2 + 0.5 * regularization * ||step||^2
/// subject to lower <= step <= upper
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct LinearizedLeastSquaresStepProblem {
    /// Row-major Jacobian `J`, with `residual.len()` rows and one column per
    /// step variable.
    pub jacobian: Vec<f64>,
    /// Current linearized residual.
    pub residual: Vec<f64>,
    /// Lower step bounds.
    pub step_lower: Vec<f64>,
    /// Upper step bounds.
    pub step_upper: Vec<f64>,
    /// Non-negative diagonal Hessian regularization.
    pub regularization: f64,
}

impl LinearizedLeastSquaresStepProblem {
    /// Number of step variables.
    #[must_use]
    pub fn variable_count(&self) -> usize {
        self.step_lower.len()
    }
}

/// Linearized least-squares SQP step with box bounds and a linear objective.
///
/// This forms the convex QP:
///
/// ```text
/// minimize 0.5 * ||J step + residual||^2
///        + linear_objective' step
///        + 0.5 * regularization * ||step||^2
/// subject to lower <= step <= upper
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct LinearizedLeastSquaresObjectiveStepProblem {
    /// Row-major Jacobian `J`, with `residual.len()` rows and one column per
    /// step variable.
    pub jacobian: Vec<f64>,
    /// Current linearized residual.
    pub residual: Vec<f64>,
    /// Extra linear objective term in step coordinates.
    pub linear_objective: Vec<f64>,
    /// Lower step bounds.
    pub step_lower: Vec<f64>,
    /// Upper step bounds.
    pub step_upper: Vec<f64>,
    /// Non-negative diagonal Hessian regularization.
    pub regularization: f64,
}

impl LinearizedLeastSquaresObjectiveStepProblem {
    /// Number of step variables.
    #[must_use]
    pub fn variable_count(&self) -> usize {
        self.step_lower.len()
    }
}

/// Linearized equality-constrained SQP step with box bounds and a linear
/// objective.
///
/// This forms the convex QP:
///
/// ```text
/// minimize linear_objective' step + 0.5 * regularization * ||step||^2
/// subject to J step = -residual
///            lower <= step <= upper
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct LinearizedEqualityObjectiveStepProblem {
    /// Row-major equality Jacobian `J`, with `residual.len()` rows and one
    /// column per step variable.
    pub jacobian: Vec<f64>,
    /// Current equality residual.
    pub residual: Vec<f64>,
    /// Linear objective term in step coordinates.
    pub linear_objective: Vec<f64>,
    /// Lower step bounds.
    pub step_lower: Vec<f64>,
    /// Upper step bounds.
    pub step_upper: Vec<f64>,
    /// Non-negative diagonal Hessian regularization.
    pub regularization: f64,
}

impl LinearizedEqualityObjectiveStepProblem {
    /// Number of step variables.
    #[must_use]
    pub fn variable_count(&self) -> usize {
        self.step_lower.len()
    }
}

/// Linearized equality-constrained SQP step with finite residual slack, box
/// bounds, and a linear objective.
///
/// This forms the convex QP:
///
/// ```text
/// minimize linear_objective' step
///        + slack_penalty * 1' (slack_plus + slack_minus)
///        + 0.5 * regularization * ||step||^2
/// subject to J step + slack_plus - slack_minus = -residual
///            lower <= step <= upper
///            0 <= slack_plus, slack_minus <= slack_upper
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct LinearizedElasticEqualityObjectiveStepProblem {
    /// Row-major equality Jacobian `J`, with `residual.len()` rows and one
    /// column per step variable.
    pub jacobian: Vec<f64>,
    /// Current equality residual.
    pub residual: Vec<f64>,
    /// Linear objective term in step coordinates.
    pub linear_objective: Vec<f64>,
    /// Lower step bounds.
    pub step_lower: Vec<f64>,
    /// Upper step bounds.
    pub step_upper: Vec<f64>,
    /// Non-negative diagonal Hessian regularization on step variables.
    pub regularization: f64,
    /// Positive L1 penalty on each nonnegative slack variable.
    pub slack_penalty: f64,
    /// Finite upper bound applied to each slack variable.
    pub slack_upper: f64,
}

impl LinearizedElasticEqualityObjectiveStepProblem {
    /// Number of original step variables.
    #[must_use]
    pub fn variable_count(&self) -> usize {
        self.step_lower.len()
    }

    /// Number of slack variables in the generated dense QP.
    #[must_use]
    pub fn slack_variable_count(&self) -> usize {
        2 * self.residual.len()
    }
}

/// Linearized least-squares SQP step with box bounds and inequality rows.
///
/// This forms the convex QP:
///
/// ```text
/// minimize 0.5 * ||Jr step + residual||^2 + 0.5 * regularization * ||step||^2
/// subject to lower <= step <= upper
///            Jg step <= -g
/// ```
#[derive(Clone, Debug, PartialEq)]
pub struct LinearizedConstrainedLeastSquaresStepProblem {
    /// Row-major residual Jacobian `Jr`.
    pub residual_jacobian: Vec<f64>,
    /// Current residual vector.
    pub residual: Vec<f64>,
    /// Row-major inequality Jacobian `Jg`.
    pub inequality_jacobian: Vec<f64>,
    /// Current inequality values `g <= 0`.
    pub inequality_values: Vec<f64>,
    /// Lower step bounds.
    pub step_lower: Vec<f64>,
    /// Upper step bounds.
    pub step_upper: Vec<f64>,
    /// Non-negative diagonal Hessian regularization.
    pub regularization: f64,
}

impl LinearizedConstrainedLeastSquaresStepProblem {
    /// Number of step variables.
    #[must_use]
    pub fn variable_count(&self) -> usize {
        self.step_lower.len()
    }
}

/// Clarabel solution for a trajectory-optimization QP.
#[derive(Clone, Debug, PartialEq)]
pub struct ClarabelQpSolution {
    /// Primal decision vector.
    pub primal: Vec<f64>,
    /// Dual variables for equality rows, in caller-supplied row order.
    pub equality_dual: Vec<f64>,
    /// Dual variables for inequality rows, in caller-supplied row order.
    pub inequality_dual: Vec<f64>,
    /// Solver primal objective.
    pub objective: f64,
    /// Clarabel termination status.
    pub status: SolverStatus,
}

/// Configuration for the dense nonlinear least-squares SQP driver.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseSqpSolver {
    /// Maximum SQP iterations before reporting non-convergence.
    pub max_iterations: usize,
    /// Convergence threshold on the residual Euclidean norm.
    pub residual_tolerance: f64,
    /// Convergence threshold on `max(g, 0)` for constrained solves.
    pub inequality_tolerance: f64,
    /// Termination threshold on the accepted step norm.
    pub step_tolerance: f64,
    /// Relative forward/backward finite-difference step for Jacobian columns.
    pub finite_difference_step: f64,
    /// Non-negative diagonal regularization added to each least-squares QP.
    pub regularization: f64,
    /// Positive residual-norm penalty used by equality-objective SQP merit.
    pub objective_residual_penalty: f64,
    /// Maximum merit-function backtracking attempts per SQP step.
    pub max_line_search_backtracks: usize,
    /// Smallest line-search scale allowed before reporting non-convergence.
    pub min_line_search_scale: f64,
}

impl Default for DenseSqpSolver {
    fn default() -> Self {
        Self {
            max_iterations: 25,
            residual_tolerance: 1.0e-8,
            inequality_tolerance: 1.0e-8,
            step_tolerance: 1.0e-10,
            finite_difference_step: 1.0e-6,
            regularization: 1.0e-10,
            objective_residual_penalty: 1.0,
            max_line_search_backtracks: 12,
            min_line_search_scale: 1.0e-8,
        }
    }
}

/// Result from [`DenseSqpSolver::solve`].
#[derive(Clone, Debug, PartialEq)]
pub struct DenseSqpSolution {
    /// Final decision vector.
    pub variables: Vec<f64>,
    /// Final residual vector.
    pub residual: Vec<f64>,
    /// Final residual Euclidean norm.
    pub residual_norm: f64,
    /// Number of accepted SQP iterations.
    pub iterations: usize,
    /// Whether [`DenseSqpSolver::residual_tolerance`] was reached.
    pub converged: bool,
}

/// Result from [`DenseSqpSolver::solve_with_inequalities`].
#[derive(Clone, Debug, PartialEq)]
pub struct DenseConstrainedSqpSolution {
    /// Final decision vector.
    pub variables: Vec<f64>,
    /// Final residual vector.
    pub residual: Vec<f64>,
    /// Final residual Euclidean norm.
    pub residual_norm: f64,
    /// Final inequality rows `g <= 0`.
    pub inequality_values: Vec<f64>,
    /// Maximum non-negative inequality violation.
    pub max_inequality_violation: f64,
    /// Number of accepted SQP iterations.
    pub iterations: usize,
    /// Whether residual and inequality tolerances were both reached.
    pub converged: bool,
}

/// Result from [`DenseSqpSolver::solve_with_linear_objective`].
#[derive(Clone, Debug, PartialEq)]
pub struct DenseSqpObjectiveSolution {
    /// Final decision vector.
    pub variables: Vec<f64>,
    /// Final residual vector.
    pub residual: Vec<f64>,
    /// Final residual Euclidean norm.
    pub residual_norm: f64,
    /// Final linear objective value.
    pub linear_objective_value: f64,
    /// Final merit used by the solver lane.
    pub merit: f64,
    /// Number of accepted SQP iterations.
    pub iterations: usize,
    /// Whether the final linearized objective step is below
    /// [`DenseSqpSolver::step_tolerance`].
    pub converged: bool,
}

struct AcceptedConstrainedStep {
    variables: Vec<f64>,
    residual: Vec<f64>,
    inequality_values: Vec<f64>,
    residual_norm: f64,
}

type AcceptedStep = (Vec<f64>, Vec<f64>, f64);

impl DenseSqpSolver {
    /// Solve a bounded nonlinear least-squares problem with dense SQP steps.
    ///
    /// The merit function is `0.5 * ||residual(x)||^2`. Each SQP iteration
    /// finite-differences a dense Jacobian, solves a box-bounded linearized
    /// least-squares QP with Clarabel, and accepts the step through backtracking
    /// when the merit function decreases.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the configuration, starting point, bounds,
    /// residual values, or generated QP are invalid.
    pub fn solve<M>(
        &self,
        initial_variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        mut model: M,
    ) -> Result<DenseSqpSolution, TrajoptError>
    where
        M: SqpResidualModel,
    {
        self.validate_config()?;
        validate_sqp_inputs(initial_variables, lower_bounds, upper_bounds)?;

        let mut variables = initial_variables.to_vec();
        let mut residual = model.residual(&variables)?;
        validate_residual(&residual)?;
        let residual_count = residual.len();
        let mut residual_norm = norm(&residual);
        if residual_norm <= self.residual_tolerance {
            return Ok(DenseSqpSolution {
                variables,
                residual,
                residual_norm,
                iterations: 0,
                converged: true,
            });
        }

        for iteration in 0..self.max_iterations {
            let jacobian = finite_difference_jacobian(
                &mut model,
                &variables,
                lower_bounds,
                upper_bounds,
                &residual,
                residual_count,
                self.finite_difference_step,
            )?;
            let step_lower = lower_bounds
                .iter()
                .zip(variables.iter())
                .map(|(lower, value)| lower - value)
                .collect::<Vec<_>>();
            let step_upper = upper_bounds
                .iter()
                .zip(variables.iter())
                .map(|(upper, value)| upper - value)
                .collect::<Vec<_>>();
            let step_problem = LinearizedLeastSquaresStepProblem {
                jacobian,
                residual: residual.clone(),
                step_lower,
                step_upper,
                regularization: self.regularization,
            };
            let step_solution = solve_linearized_least_squares_step(&step_problem)?;
            let step_norm = norm(&step_solution.primal);
            if step_norm <= self.step_tolerance {
                return Ok(DenseSqpSolution {
                    variables,
                    residual,
                    residual_norm,
                    iterations: iteration,
                    converged: false,
                });
            }

            let (accepted_variables, accepted_residual, accepted_norm) = self.accept_step(
                &mut model,
                &variables,
                lower_bounds,
                upper_bounds,
                &step_solution.primal,
                residual_norm,
                residual_count,
            )?;
            variables = accepted_variables;
            residual = accepted_residual;
            residual_norm = accepted_norm;

            if residual_norm <= self.residual_tolerance {
                return Ok(DenseSqpSolution {
                    variables,
                    residual,
                    residual_norm,
                    iterations: iteration + 1,
                    converged: true,
                });
            }
        }

        Ok(DenseSqpSolution {
            variables,
            residual,
            residual_norm,
            iterations: self.max_iterations,
            converged: false,
        })
    }

    /// Solve a bounded nonlinear least-squares objective with an extra linear
    /// objective term.
    ///
    /// The merit function is
    /// `0.5 * ||residual(x)||^2 + linear_objective' x`. Each SQP iteration
    /// finite-differences a dense Jacobian, solves a box-bounded linearized
    /// least-squares QP with the same linear objective term, and accepts the
    /// step through backtracking when the merit function decreases.
    ///
    /// Unlike [`Self::solve`], this method does not stop only because the
    /// residual is already small: the linear objective can still define a
    /// descent direction inside the variable bounds.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the configuration, starting point, bounds,
    /// linear objective, residual values, or generated QP are invalid.
    pub fn solve_with_linear_objective<M>(
        &self,
        initial_variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        linear_objective: &[f64],
        mut model: M,
    ) -> Result<DenseSqpObjectiveSolution, TrajoptError>
    where
        M: SqpResidualModel,
    {
        self.validate_config()?;
        validate_sqp_inputs(initial_variables, lower_bounds, upper_bounds)?;
        validate_linear_objective(linear_objective, initial_variables.len())?;

        let mut variables = initial_variables.to_vec();
        let mut residual = model.residual(&variables)?;
        validate_residual(&residual)?;
        let residual_count = residual.len();
        let mut residual_norm = norm(&residual);
        let mut linear_objective_value = dot(linear_objective, &variables);
        let mut merit = objective_merit(residual_norm, linear_objective_value);

        for iteration in 0..self.max_iterations {
            let jacobian = finite_difference_jacobian(
                &mut model,
                &variables,
                lower_bounds,
                upper_bounds,
                &residual,
                residual_count,
                self.finite_difference_step,
            )?;
            let step_lower = lower_bounds
                .iter()
                .zip(variables.iter())
                .map(|(lower, value)| lower - value)
                .collect::<Vec<_>>();
            let step_upper = upper_bounds
                .iter()
                .zip(variables.iter())
                .map(|(upper, value)| upper - value)
                .collect::<Vec<_>>();
            let step_problem = LinearizedLeastSquaresObjectiveStepProblem {
                jacobian,
                residual: residual.clone(),
                linear_objective: linear_objective.to_vec(),
                step_lower,
                step_upper,
                regularization: self.regularization,
            };
            let step_solution = solve_linearized_least_squares_objective_step(&step_problem)?;
            let step_norm = norm(&step_solution.primal);
            if step_norm <= self.step_tolerance {
                return Ok(DenseSqpObjectiveSolution {
                    variables,
                    residual,
                    residual_norm,
                    linear_objective_value,
                    merit,
                    iterations: iteration,
                    converged: true,
                });
            }

            let accepted = self.accept_objective_step(
                &mut model,
                &variables,
                lower_bounds,
                upper_bounds,
                linear_objective,
                &step_solution.primal,
                merit,
                residual_count,
            )?;
            variables = accepted.0;
            residual = accepted.1;
            residual_norm = accepted.2;
            linear_objective_value = dot(linear_objective, &variables);
            merit = objective_merit(residual_norm, linear_objective_value);
        }

        Ok(DenseSqpObjectiveSolution {
            variables,
            residual,
            residual_norm,
            linear_objective_value,
            merit,
            iterations: self.max_iterations,
            converged: false,
        })
    }

    /// Solve bounded nonlinear equality residuals with an extra linear
    /// objective term.
    ///
    /// Each SQP iteration finite-differences a dense equality Jacobian, solves
    /// a box-bounded QP with `J step = -residual`, and accepts the step through
    /// backtracking on
    /// `linear_objective' x + objective_residual_penalty * ||residual(x)||`.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the configuration, starting point, bounds,
    /// linear objective, residual values, generated equality QP, or line-search
    /// evaluations are invalid.
    pub fn solve_equalities_with_linear_objective<M>(
        &self,
        initial_variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        linear_objective: &[f64],
        mut model: M,
    ) -> Result<DenseSqpObjectiveSolution, TrajoptError>
    where
        M: SqpResidualModel,
    {
        self.validate_config()?;
        validate_sqp_inputs(initial_variables, lower_bounds, upper_bounds)?;
        validate_linear_objective(linear_objective, initial_variables.len())?;

        let mut variables = initial_variables.to_vec();
        let mut residual = model.residual(&variables)?;
        validate_residual(&residual)?;
        let residual_count = residual.len();
        let mut residual_norm = norm(&residual);
        let mut linear_objective_value = dot(linear_objective, &variables);
        let mut merit = equality_objective_merit(
            residual_norm,
            linear_objective_value,
            self.objective_residual_penalty,
        );

        for iteration in 0..self.max_iterations {
            let jacobian = finite_difference_jacobian(
                &mut model,
                &variables,
                lower_bounds,
                upper_bounds,
                &residual,
                residual_count,
                self.finite_difference_step,
            )?;
            let step_lower = lower_bounds
                .iter()
                .zip(variables.iter())
                .map(|(lower, value)| lower - value)
                .collect::<Vec<_>>();
            let step_upper = upper_bounds
                .iter()
                .zip(variables.iter())
                .map(|(upper, value)| upper - value)
                .collect::<Vec<_>>();
            let step_problem = LinearizedEqualityObjectiveStepProblem {
                jacobian,
                residual: residual.clone(),
                linear_objective: linear_objective.to_vec(),
                step_lower,
                step_upper,
                regularization: self.regularization,
            };
            let step = self.solve_equality_objective_step_with_elastic_fallback(&step_problem)?;
            let step_norm = norm(&step);
            if step_norm <= self.step_tolerance {
                return Ok(DenseSqpObjectiveSolution {
                    variables,
                    residual,
                    residual_norm,
                    linear_objective_value,
                    merit,
                    iterations: iteration,
                    converged: residual_norm <= self.residual_tolerance,
                });
            }

            let accepted = self.accept_equality_objective_step(
                &mut model,
                &variables,
                lower_bounds,
                upper_bounds,
                linear_objective,
                &step,
                merit,
                residual_count,
                self.objective_residual_penalty,
            )?;
            variables = accepted.0;
            residual = accepted.1;
            residual_norm = accepted.2;
            linear_objective_value = dot(linear_objective, &variables);
            merit = equality_objective_merit(
                residual_norm,
                linear_objective_value,
                self.objective_residual_penalty,
            );

            let restored = if residual_norm > self.residual_tolerance {
                self.try_restoration_step(
                    &mut model,
                    &variables,
                    lower_bounds,
                    upper_bounds,
                    &residual,
                    residual_norm,
                    residual_count,
                )?
            } else {
                None
            };
            if let Some(restored) = restored {
                variables = restored.0;
                residual = restored.1;
                residual_norm = restored.2;
                linear_objective_value = dot(linear_objective, &variables);
                merit = equality_objective_merit(
                    residual_norm,
                    linear_objective_value,
                    self.objective_residual_penalty,
                );
            }
        }

        Ok(DenseSqpObjectiveSolution {
            variables,
            residual,
            residual_norm,
            linear_objective_value,
            merit,
            iterations: self.max_iterations,
            converged: false,
        })
    }

    /// Solve a bounded nonlinear least-squares problem with nonlinear
    /// inequalities `g(x) <= 0`.
    ///
    /// Each SQP iteration finite-differences residual and inequality Jacobians,
    /// solves a Clarabel QP with box-bounded steps and linearized inequality
    /// rows, and accepts the step using a residual-plus-violation merit
    /// function.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when configuration, inputs, residuals,
    /// inequalities, generated QPs, or line-search evaluations are invalid.
    pub fn solve_with_inequalities<R, G>(
        &self,
        initial_variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        mut residual_model: R,
        mut inequality_model: G,
    ) -> Result<DenseConstrainedSqpSolution, TrajoptError>
    where
        R: SqpResidualModel,
        G: SqpInequalityModel,
    {
        self.validate_config()?;
        validate_sqp_inputs(initial_variables, lower_bounds, upper_bounds)?;

        let mut variables = initial_variables.to_vec();
        let mut residual = residual_model.residual(&variables)?;
        validate_residual(&residual)?;
        let residual_count = residual.len();
        let mut residual_norm = norm(&residual);
        let mut inequality_values = inequality_model.inequality(&variables)?;
        validate_inequality_values(&inequality_values)?;
        let inequality_count = inequality_values.len();
        let mut max_inequality_violation = max_violation(&inequality_values);
        if residual_norm <= self.residual_tolerance
            && max_inequality_violation <= self.inequality_tolerance
        {
            return Ok(DenseConstrainedSqpSolution {
                variables,
                residual,
                residual_norm,
                inequality_values,
                max_inequality_violation,
                iterations: 0,
                converged: true,
            });
        }

        for iteration in 0..self.max_iterations {
            let residual_jacobian = finite_difference_jacobian(
                &mut residual_model,
                &variables,
                lower_bounds,
                upper_bounds,
                &residual,
                residual_count,
                self.finite_difference_step,
            )?;
            let inequality_jacobian = finite_difference_inequality_jacobian(
                &mut inequality_model,
                &variables,
                lower_bounds,
                upper_bounds,
                &inequality_values,
                inequality_count,
                self.finite_difference_step,
            )?;
            let step_lower = lower_bounds
                .iter()
                .zip(variables.iter())
                .map(|(lower, value)| lower - value)
                .collect::<Vec<_>>();
            let step_upper = upper_bounds
                .iter()
                .zip(variables.iter())
                .map(|(upper, value)| upper - value)
                .collect::<Vec<_>>();
            let step_problem = LinearizedConstrainedLeastSquaresStepProblem {
                residual_jacobian,
                residual: residual.clone(),
                inequality_jacobian,
                inequality_values: inequality_values.clone(),
                step_lower,
                step_upper,
                regularization: self.regularization,
            };
            let step_solution = solve_linearized_constrained_least_squares_step(&step_problem)?;
            let step_norm = norm(&step_solution.primal);
            if step_norm <= self.step_tolerance {
                return Ok(DenseConstrainedSqpSolution {
                    variables,
                    residual,
                    residual_norm,
                    inequality_values,
                    max_inequality_violation,
                    iterations: iteration,
                    converged: false,
                });
            }

            let accepted = self.accept_constrained_step(
                &mut residual_model,
                &mut inequality_model,
                &variables,
                lower_bounds,
                upper_bounds,
                &step_solution.primal,
                constrained_merit(residual_norm, &inequality_values),
                residual_count,
                inequality_count,
            )?;
            variables = accepted.variables;
            residual = accepted.residual;
            inequality_values = accepted.inequality_values;
            residual_norm = accepted.residual_norm;
            max_inequality_violation = max_violation(&inequality_values);

            if residual_norm <= self.residual_tolerance
                && max_inequality_violation <= self.inequality_tolerance
            {
                return Ok(DenseConstrainedSqpSolution {
                    variables,
                    residual,
                    residual_norm,
                    inequality_values,
                    max_inequality_violation,
                    iterations: iteration + 1,
                    converged: true,
                });
            }
        }

        Ok(DenseConstrainedSqpSolution {
            variables,
            residual,
            residual_norm,
            inequality_values,
            max_inequality_violation,
            iterations: self.max_iterations,
            converged: false,
        })
    }

    fn validate_config(&self) -> Result<(), TrajoptError> {
        if self.max_iterations == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP requires at least one iteration",
            });
        }
        if !self.residual_tolerance.is_finite() || self.residual_tolerance <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP residual tolerance must be finite and positive",
            });
        }
        if !self.inequality_tolerance.is_finite() || self.inequality_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP inequality tolerance must be finite and non-negative",
            });
        }
        if !self.step_tolerance.is_finite() || self.step_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP step tolerance must be finite and non-negative",
            });
        }
        if !self.finite_difference_step.is_finite() || self.finite_difference_step <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP finite-difference step must be finite and positive",
            });
        }
        if !self.regularization.is_finite() || self.regularization < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP regularization must be finite and non-negative",
            });
        }
        if !self.objective_residual_penalty.is_finite() || self.objective_residual_penalty <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP objective residual penalty must be finite and positive",
            });
        }
        if self.max_line_search_backtracks == 0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP line search requires at least one backtrack",
            });
        }
        if !self.min_line_search_scale.is_finite()
            || self.min_line_search_scale <= 0.0
            || self.min_line_search_scale > 1.0
        {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP minimum line-search scale must be in (0, 1]",
            });
        }
        Ok(())
    }

    fn solve_equality_objective_step_with_elastic_fallback(
        &self,
        problem: &LinearizedEqualityObjectiveStepProblem,
    ) -> Result<Vec<f64>, TrajoptError> {
        match solve_linearized_equality_objective_step(problem) {
            Ok(solution) => Ok(solution.primal),
            Err(_) => {
                let elastic_problem = LinearizedElasticEqualityObjectiveStepProblem {
                    jacobian: problem.jacobian.clone(),
                    residual: problem.residual.clone(),
                    linear_objective: problem.linear_objective.clone(),
                    step_lower: problem.step_lower.clone(),
                    step_upper: problem.step_upper.clone(),
                    regularization: problem.regularization,
                    slack_penalty: self.objective_residual_penalty,
                    slack_upper: elastic_slack_upper(&problem.residual),
                };
                let solution = solve_linearized_elastic_equality_objective_step(&elastic_problem)?;
                Ok(solution.primal[..problem.variable_count()].to_vec())
            }
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn accept_step<M>(
        &self,
        model: &mut M,
        variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        step: &[f64],
        current_norm: f64,
        residual_count: usize,
    ) -> Result<(Vec<f64>, Vec<f64>, f64), TrajoptError>
    where
        M: SqpResidualModel,
    {
        let current_merit = 0.5 * current_norm * current_norm;
        let mut scale = 1.0;
        for _ in 0..self.max_line_search_backtracks {
            let trial = bounded_trial(variables, lower_bounds, upper_bounds, step, scale)?;
            let trial_residual = model.residual(&trial)?;
            validate_residual_dimension(&trial_residual, residual_count)?;
            let trial_norm = norm(&trial_residual);
            let trial_merit = 0.5 * trial_norm * trial_norm;
            if trial_merit < current_merit || trial_norm <= self.residual_tolerance {
                return Ok((trial, trial_residual, trial_norm));
            }
            scale *= 0.5;
            if scale < self.min_line_search_scale {
                break;
            }
        }
        Err(TrajoptError::InvalidPayload {
            reason: "dense SQP line search failed to reduce merit",
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn accept_objective_step<M>(
        &self,
        model: &mut M,
        variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        linear_objective: &[f64],
        step: &[f64],
        current_merit: f64,
        residual_count: usize,
    ) -> Result<(Vec<f64>, Vec<f64>, f64), TrajoptError>
    where
        M: SqpResidualModel,
    {
        let mut scale = 1.0;
        for _ in 0..self.max_line_search_backtracks {
            let trial = bounded_trial(variables, lower_bounds, upper_bounds, step, scale)?;
            let trial_residual = model.residual(&trial)?;
            validate_residual_dimension(&trial_residual, residual_count)?;
            let trial_norm = norm(&trial_residual);
            let trial_linear_objective = dot(linear_objective, &trial);
            let trial_merit = objective_merit(trial_norm, trial_linear_objective);
            if trial_merit < current_merit {
                return Ok((trial, trial_residual, trial_norm));
            }
            scale *= 0.5;
            if scale < self.min_line_search_scale {
                break;
            }
        }
        Err(TrajoptError::InvalidPayload {
            reason: "dense SQP objective line search failed to reduce merit",
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn accept_equality_objective_step<M>(
        &self,
        model: &mut M,
        variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        linear_objective: &[f64],
        step: &[f64],
        current_merit: f64,
        residual_count: usize,
        residual_penalty: f64,
    ) -> Result<(Vec<f64>, Vec<f64>, f64), TrajoptError>
    where
        M: SqpResidualModel,
    {
        let mut scale = 1.0;
        for _ in 0..self.max_line_search_backtracks {
            let trial = bounded_trial(variables, lower_bounds, upper_bounds, step, scale)?;
            let trial_residual = model.residual(&trial)?;
            validate_residual_dimension(&trial_residual, residual_count)?;
            let trial_norm = norm(&trial_residual);
            let trial_linear_objective = dot(linear_objective, &trial);
            let trial_merit =
                equality_objective_merit(trial_norm, trial_linear_objective, residual_penalty);
            if trial_merit < current_merit {
                return Ok((trial, trial_residual, trial_norm));
            }
            scale *= 0.5;
            if scale < self.min_line_search_scale {
                break;
            }
        }
        Err(TrajoptError::InvalidPayload {
            reason: "dense SQP equality-objective line search failed to reduce merit",
        })
    }

    #[allow(clippy::too_many_arguments)]
    fn try_restoration_step<M>(
        &self,
        model: &mut M,
        variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        residual: &[f64],
        residual_norm: f64,
        residual_count: usize,
    ) -> Result<Option<AcceptedStep>, TrajoptError>
    where
        M: SqpResidualModel,
    {
        let jacobian = finite_difference_jacobian(
            model,
            variables,
            lower_bounds,
            upper_bounds,
            residual,
            residual_count,
            self.finite_difference_step,
        )?;
        let step_lower = lower_bounds
            .iter()
            .zip(variables.iter())
            .map(|(lower, value)| lower - value)
            .collect::<Vec<_>>();
        let step_upper = upper_bounds
            .iter()
            .zip(variables.iter())
            .map(|(upper, value)| upper - value)
            .collect::<Vec<_>>();
        let step_problem = LinearizedLeastSquaresStepProblem {
            jacobian,
            residual: residual.to_vec(),
            step_lower,
            step_upper,
            regularization: self.regularization,
        };
        let step_solution = solve_linearized_least_squares_step(&step_problem)?;
        if norm(&step_solution.primal) <= self.step_tolerance {
            return Ok(None);
        }
        let accepted = match self.accept_step(
            model,
            variables,
            lower_bounds,
            upper_bounds,
            &step_solution.primal,
            residual_norm,
            residual_count,
        ) {
            Ok(accepted) => accepted,
            Err(_) => return Ok(None),
        };
        if accepted.2 < residual_norm {
            Ok(Some(accepted))
        } else {
            Ok(None)
        }
    }

    #[allow(clippy::too_many_arguments)]
    fn accept_constrained_step<R, G>(
        &self,
        residual_model: &mut R,
        inequality_model: &mut G,
        variables: &[f64],
        lower_bounds: &[f64],
        upper_bounds: &[f64],
        step: &[f64],
        current_merit: f64,
        residual_count: usize,
        inequality_count: usize,
    ) -> Result<AcceptedConstrainedStep, TrajoptError>
    where
        R: SqpResidualModel,
        G: SqpInequalityModel,
    {
        let mut scale = 1.0;
        for _ in 0..self.max_line_search_backtracks {
            let trial = bounded_trial(variables, lower_bounds, upper_bounds, step, scale)?;
            let trial_residual = residual_model.residual(&trial)?;
            validate_residual_dimension(&trial_residual, residual_count)?;
            let trial_inequality = inequality_model.inequality(&trial)?;
            validate_inequality_dimension(&trial_inequality, inequality_count)?;
            let trial_norm = norm(&trial_residual);
            let trial_merit = constrained_merit(trial_norm, &trial_inequality);
            if trial_merit < current_merit
                || (trial_norm <= self.residual_tolerance
                    && max_violation(&trial_inequality) <= self.inequality_tolerance)
            {
                return Ok(AcceptedConstrainedStep {
                    variables: trial,
                    residual: trial_residual,
                    inequality_values: trial_inequality,
                    residual_norm: trial_norm,
                });
            }
            scale *= 0.5;
            if scale < self.min_line_search_scale {
                break;
            }
        }
        Err(TrajoptError::InvalidPayload {
            reason: "dense constrained SQP line search failed to reduce merit",
        })
    }
}

/// Solve a dense convex QP with deterministic Clarabel settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation fails or Clarabel does not report
/// [`SolverStatus::Solved`].
pub fn solve_dense_qp(problem: &DenseQpProblem) -> Result<ClarabelQpSolution, TrajoptError> {
    validate_dense_qp(problem)?;

    let p = csc_from_dense_upper_triangle(&problem.hessian, problem.variable_count);
    let q = problem.linear.clone();
    let a = csc_from_constraints(problem);
    let mut b = problem.equality_rhs.clone();
    b.extend_from_slice(&problem.inequality_rhs);

    let mut cones = Vec::new();
    if !problem.equality_rhs.is_empty() {
        cones.push(ZeroConeT(problem.equality_rhs.len()));
    }
    if !problem.inequality_rhs.is_empty() {
        cones.push(NonnegativeConeT(problem.inequality_rhs.len()));
    }

    let mut solver = DefaultSolver::new(&p, &q, &a, &b, &cones, deterministic_sqp_settings());
    solver.solve();
    if solver.solution.status != SolverStatus::Solved {
        return Err(TrajoptError::InvalidPayload {
            reason: "Clarabel did not solve trajectory QP",
        });
    }

    let equality_dual = solver.solution.z[..problem.equality_rhs.len()].to_vec();
    let inequality_dual = solver.solution.z[problem.equality_rhs.len()..].to_vec();

    Ok(ClarabelQpSolution {
        primal: solver.solution.x,
        equality_dual,
        inequality_dual,
        objective: solver.solution.obj_val,
        status: solver.solution.status,
    })
}

/// Solve a linearized least-squares SQP step with deterministic Clarabel
/// settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation fails or Clarabel does not solve the
/// generated QP.
pub fn solve_linearized_least_squares_step(
    problem: &LinearizedLeastSquaresStepProblem,
) -> Result<ClarabelQpSolution, TrajoptError> {
    let qp = least_squares_step_qp(problem)?;
    solve_dense_qp(&qp)
}

/// Solve a linearized least-squares SQP step with an extra linear objective
/// term and deterministic Clarabel settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation fails or Clarabel does not solve the
/// generated QP.
pub fn solve_linearized_least_squares_objective_step(
    problem: &LinearizedLeastSquaresObjectiveStepProblem,
) -> Result<ClarabelQpSolution, TrajoptError> {
    let qp = least_squares_objective_step_qp(problem)?;
    solve_dense_qp(&qp)
}

/// Solve a linearized equality-constrained SQP step with an extra linear
/// objective term and deterministic Clarabel settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation fails or Clarabel does not solve the
/// generated QP.
pub fn solve_linearized_equality_objective_step(
    problem: &LinearizedEqualityObjectiveStepProblem,
) -> Result<ClarabelQpSolution, TrajoptError> {
    let qp = equality_objective_step_qp(problem)?;
    solve_dense_qp(&qp)
}

/// Solve a linearized elastic equality-constrained SQP step with an extra
/// linear objective term and deterministic Clarabel settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation fails or Clarabel does not solve the
/// generated QP.
pub fn solve_linearized_elastic_equality_objective_step(
    problem: &LinearizedElasticEqualityObjectiveStepProblem,
) -> Result<ClarabelQpSolution, TrajoptError> {
    let qp = elastic_equality_objective_step_qp(problem)?;
    solve_dense_qp(&qp)
}

/// Solve a linearized constrained least-squares SQP step with deterministic
/// Clarabel settings.
///
/// # Errors
///
/// Returns [`TrajoptError`] when validation fails or Clarabel does not solve the
/// generated QP.
pub fn solve_linearized_constrained_least_squares_step(
    problem: &LinearizedConstrainedLeastSquaresStepProblem,
) -> Result<ClarabelQpSolution, TrajoptError> {
    let qp = constrained_least_squares_step_qp(problem)?;
    solve_dense_qp(&qp)
}

/// Convert a linearized least-squares step into the equivalent dense QP.
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions or numeric values are invalid.
pub fn least_squares_step_qp(
    problem: &LinearizedLeastSquaresStepProblem,
) -> Result<DenseQpProblem, TrajoptError> {
    validate_least_squares_step(problem)?;
    let variable_count = problem.variable_count();
    let residual_count = problem.residual.len();

    let mut hessian = vec![0.0; variable_count * variable_count];
    let mut linear = vec![0.0; variable_count];
    for row in 0..residual_count {
        let residual = problem.residual[row];
        for col in 0..variable_count {
            let j_col = problem.jacobian[row * variable_count + col];
            linear[col] += j_col * residual;
            for other_col in 0..variable_count {
                let j_other = problem.jacobian[row * variable_count + other_col];
                hessian[col * variable_count + other_col] += j_col * j_other;
            }
        }
    }
    for diag in 0..variable_count {
        hessian[diag * variable_count + diag] += problem.regularization;
    }

    let mut inequality_matrix = vec![0.0; 2 * variable_count * variable_count];
    let mut inequality_rhs = vec![0.0; 2 * variable_count];
    for index in 0..variable_count {
        inequality_matrix[index * variable_count + index] = 1.0;
        inequality_rhs[index] = problem.step_upper[index];

        let lower_row = variable_count + index;
        inequality_matrix[lower_row * variable_count + index] = -1.0;
        inequality_rhs[lower_row] = -problem.step_lower[index];
    }

    DenseQpProblem::new(
        variable_count,
        hessian,
        linear,
        Vec::new(),
        Vec::new(),
        inequality_matrix,
        inequality_rhs,
    )
}

/// Convert a linearized least-squares objective step into the equivalent dense
/// QP.
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions or numeric values are invalid.
pub fn least_squares_objective_step_qp(
    problem: &LinearizedLeastSquaresObjectiveStepProblem,
) -> Result<DenseQpProblem, TrajoptError> {
    validate_least_squares_objective_step(problem)?;
    let variable_count = problem.variable_count();
    let residual_count = problem.residual.len();

    let mut hessian = vec![0.0; variable_count * variable_count];
    let mut linear = problem.linear_objective.clone();
    for row in 0..residual_count {
        let residual = problem.residual[row];
        for col in 0..variable_count {
            let j_col = problem.jacobian[row * variable_count + col];
            linear[col] += j_col * residual;
            for other_col in 0..variable_count {
                let j_other = problem.jacobian[row * variable_count + other_col];
                hessian[col * variable_count + other_col] += j_col * j_other;
            }
        }
    }
    for diag in 0..variable_count {
        hessian[diag * variable_count + diag] += problem.regularization;
    }

    let mut inequality_matrix = vec![0.0; 2 * variable_count * variable_count];
    let mut inequality_rhs = vec![0.0; 2 * variable_count];
    for index in 0..variable_count {
        inequality_matrix[index * variable_count + index] = 1.0;
        inequality_rhs[index] = problem.step_upper[index];

        let lower_row = variable_count + index;
        inequality_matrix[lower_row * variable_count + index] = -1.0;
        inequality_rhs[lower_row] = -problem.step_lower[index];
    }

    DenseQpProblem::new(
        variable_count,
        hessian,
        linear,
        Vec::new(),
        Vec::new(),
        inequality_matrix,
        inequality_rhs,
    )
}

/// Convert a linearized equality-constrained objective step into the
/// equivalent dense QP.
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions or numeric values are invalid.
pub fn equality_objective_step_qp(
    problem: &LinearizedEqualityObjectiveStepProblem,
) -> Result<DenseQpProblem, TrajoptError> {
    validate_equality_objective_step(problem)?;
    let variable_count = problem.variable_count();

    let mut hessian = vec![0.0; variable_count * variable_count];
    for diag in 0..variable_count {
        hessian[diag * variable_count + diag] = problem.regularization;
    }

    let mut equality_rhs = Vec::with_capacity(problem.residual.len());
    equality_rhs.extend(problem.residual.iter().map(|residual| -residual));

    let mut inequality_matrix = vec![0.0; 2 * variable_count * variable_count];
    let mut inequality_rhs = vec![0.0; 2 * variable_count];
    for index in 0..variable_count {
        inequality_matrix[index * variable_count + index] = 1.0;
        inequality_rhs[index] = problem.step_upper[index];

        let lower_row = variable_count + index;
        inequality_matrix[lower_row * variable_count + index] = -1.0;
        inequality_rhs[lower_row] = -problem.step_lower[index];
    }

    DenseQpProblem::new(
        variable_count,
        hessian,
        problem.linear_objective.clone(),
        problem.jacobian.clone(),
        equality_rhs,
        inequality_matrix,
        inequality_rhs,
    )
}

/// Convert a linearized elastic equality-constrained objective step into the
/// equivalent dense QP.
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions or numeric values are invalid.
pub fn elastic_equality_objective_step_qp(
    problem: &LinearizedElasticEqualityObjectiveStepProblem,
) -> Result<DenseQpProblem, TrajoptError> {
    validate_elastic_equality_objective_step(problem)?;
    let step_variable_count = problem.variable_count();
    let residual_count = problem.residual.len();
    let variable_count = step_variable_count + problem.slack_variable_count();
    let slack_plus_offset = step_variable_count;
    let slack_minus_offset = step_variable_count + residual_count;

    let mut hessian = vec![0.0; variable_count * variable_count];
    for diag in 0..step_variable_count {
        hessian[diag * variable_count + diag] = problem.regularization;
    }

    let mut linear = vec![problem.slack_penalty; variable_count];
    linear[..step_variable_count].copy_from_slice(&problem.linear_objective);

    let mut equality_matrix = vec![0.0; residual_count * variable_count];
    let mut equality_rhs = Vec::with_capacity(residual_count);
    for row in 0..residual_count {
        let src = row * step_variable_count;
        let dst = row * variable_count;
        equality_matrix[dst..dst + step_variable_count]
            .copy_from_slice(&problem.jacobian[src..src + step_variable_count]);
        equality_matrix[dst + slack_plus_offset + row] = 1.0;
        equality_matrix[dst + slack_minus_offset + row] = -1.0;
        equality_rhs.push(-problem.residual[row]);
    }

    let mut lower_bounds = vec![0.0; variable_count];
    let mut upper_bounds = vec![problem.slack_upper; variable_count];
    lower_bounds[..step_variable_count].copy_from_slice(&problem.step_lower);
    upper_bounds[..step_variable_count].copy_from_slice(&problem.step_upper);

    let mut inequality_matrix = vec![0.0; 2 * variable_count * variable_count];
    let mut inequality_rhs = vec![0.0; 2 * variable_count];
    for index in 0..variable_count {
        inequality_matrix[index * variable_count + index] = 1.0;
        inequality_rhs[index] = upper_bounds[index];

        let lower_row = variable_count + index;
        inequality_matrix[lower_row * variable_count + index] = -1.0;
        inequality_rhs[lower_row] = -lower_bounds[index];
    }

    DenseQpProblem::new(
        variable_count,
        hessian,
        linear,
        equality_matrix,
        equality_rhs,
        inequality_matrix,
        inequality_rhs,
    )
}

/// Convert a linearized constrained least-squares step into a dense QP.
///
/// # Errors
///
/// Returns [`TrajoptError`] when dimensions or numeric values are invalid.
pub fn constrained_least_squares_step_qp(
    problem: &LinearizedConstrainedLeastSquaresStepProblem,
) -> Result<DenseQpProblem, TrajoptError> {
    validate_constrained_least_squares_step(problem)?;
    let variable_count = problem.variable_count();
    let residual_count = problem.residual.len();
    let inequality_count = problem.inequality_values.len();

    let mut hessian = vec![0.0; variable_count * variable_count];
    let mut linear = vec![0.0; variable_count];
    for row in 0..residual_count {
        let residual = problem.residual[row];
        for col in 0..variable_count {
            let j_col = problem.residual_jacobian[row * variable_count + col];
            linear[col] += j_col * residual;
            for other_col in 0..variable_count {
                let j_other = problem.residual_jacobian[row * variable_count + other_col];
                hessian[col * variable_count + other_col] += j_col * j_other;
            }
        }
    }
    for diag in 0..variable_count {
        hessian[diag * variable_count + diag] += problem.regularization;
    }

    let inequality_rows = 2 * variable_count + inequality_count;
    let mut inequality_matrix = vec![0.0; inequality_rows * variable_count];
    let mut inequality_rhs = vec![0.0; inequality_rows];
    for index in 0..variable_count {
        inequality_matrix[index * variable_count + index] = 1.0;
        inequality_rhs[index] = problem.step_upper[index];

        let lower_row = variable_count + index;
        inequality_matrix[lower_row * variable_count + index] = -1.0;
        inequality_rhs[lower_row] = -problem.step_lower[index];
    }
    let nonlinear_offset = 2 * variable_count;
    for row in 0..inequality_count {
        let target_row = nonlinear_offset + row;
        let src = row * variable_count;
        let dst = target_row * variable_count;
        inequality_matrix[dst..dst + variable_count]
            .copy_from_slice(&problem.inequality_jacobian[src..src + variable_count]);
        inequality_rhs[target_row] = -problem.inequality_values[row];
    }

    DenseQpProblem::new(
        variable_count,
        hessian,
        linear,
        Vec::new(),
        Vec::new(),
        inequality_matrix,
        inequality_rhs,
    )
}

fn validate_dense_qp(problem: &DenseQpProblem) -> Result<(), TrajoptError> {
    let variable_count = problem.variable_count;
    if variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "trajectory QP requires at least one decision variable",
        });
    }
    if problem.hessian.len() != variable_count * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "trajectory QP Hessian dimensions are inconsistent",
        });
    }
    if problem.linear.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "trajectory QP linear vector dimension is inconsistent",
        });
    }
    if problem.equality_matrix.len() != problem.equality_rhs.len() * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "trajectory QP equality matrix dimensions are inconsistent",
        });
    }
    if problem.inequality_matrix.len() != problem.inequality_rhs.len() * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "trajectory QP inequality matrix dimensions are inconsistent",
        });
    }
    validate_finite(
        "trajectory QP Hessian values must be finite",
        &problem.hessian,
    )?;
    validate_finite(
        "trajectory QP linear values must be finite",
        &problem.linear,
    )?;
    validate_finite(
        "trajectory QP equality matrix values must be finite",
        &problem.equality_matrix,
    )?;
    validate_finite(
        "trajectory QP equality RHS values must be finite",
        &problem.equality_rhs,
    )?;
    validate_finite(
        "trajectory QP inequality matrix values must be finite",
        &problem.inequality_matrix,
    )?;
    validate_finite(
        "trajectory QP inequality RHS values must be finite",
        &problem.inequality_rhs,
    )?;
    validate_symmetric_hessian(&problem.hessian, variable_count)
}

fn validate_linear_objective(
    linear_objective: &[f64],
    variable_count: usize,
) -> Result<(), TrajoptError> {
    if linear_objective.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP linear objective dimension must match variables",
        });
    }
    validate_finite(
        "dense SQP linear objective values must be finite",
        linear_objective,
    )
}

fn validate_least_squares_step(
    problem: &LinearizedLeastSquaresStepProblem,
) -> Result<(), TrajoptError> {
    let variable_count = problem.variable_count();
    if variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares step requires at least one variable",
        });
    }
    if problem.step_upper.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares step bounds must have equal length",
        });
    }
    if problem.residual.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares step requires at least one residual",
        });
    }
    if problem.jacobian.len() != problem.residual.len() * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares Jacobian dimensions are inconsistent",
        });
    }
    if !problem.regularization.is_finite() || problem.regularization < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares regularization must be finite and non-negative",
        });
    }
    validate_finite(
        "linearized least-squares Jacobian values must be finite",
        &problem.jacobian,
    )?;
    validate_finite(
        "linearized least-squares residual values must be finite",
        &problem.residual,
    )?;
    validate_finite(
        "linearized least-squares lower bounds must be finite",
        &problem.step_lower,
    )?;
    validate_finite(
        "linearized least-squares upper bounds must be finite",
        &problem.step_upper,
    )?;
    if !problem
        .step_lower
        .iter()
        .zip(problem.step_upper.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares lower bounds must not exceed upper bounds",
        });
    }
    Ok(())
}

fn validate_least_squares_objective_step(
    problem: &LinearizedLeastSquaresObjectiveStepProblem,
) -> Result<(), TrajoptError> {
    let base_problem = LinearizedLeastSquaresStepProblem {
        jacobian: problem.jacobian.clone(),
        residual: problem.residual.clone(),
        step_lower: problem.step_lower.clone(),
        step_upper: problem.step_upper.clone(),
        regularization: problem.regularization,
    };
    validate_least_squares_step(&base_problem)?;
    if problem.linear_objective.len() != problem.variable_count() {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized least-squares objective vector dimension is inconsistent",
        });
    }
    validate_finite(
        "linearized least-squares objective values must be finite",
        &problem.linear_objective,
    )
}

fn validate_equality_objective_step(
    problem: &LinearizedEqualityObjectiveStepProblem,
) -> Result<(), TrajoptError> {
    let variable_count = problem.variable_count();
    if variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective step requires at least one variable",
        });
    }
    if problem.step_upper.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective step bounds must have equal length",
        });
    }
    if problem.residual.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective step requires at least one residual",
        });
    }
    if problem.jacobian.len() != problem.residual.len() * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective Jacobian dimensions are inconsistent",
        });
    }
    if problem.linear_objective.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective vector dimension is inconsistent",
        });
    }
    if !problem.regularization.is_finite() || problem.regularization < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective regularization must be finite and non-negative",
        });
    }
    validate_finite(
        "linearized equality objective Jacobian values must be finite",
        &problem.jacobian,
    )?;
    validate_finite(
        "linearized equality objective residual values must be finite",
        &problem.residual,
    )?;
    validate_finite(
        "linearized equality objective values must be finite",
        &problem.linear_objective,
    )?;
    validate_finite(
        "linearized equality objective lower bounds must be finite",
        &problem.step_lower,
    )?;
    validate_finite(
        "linearized equality objective upper bounds must be finite",
        &problem.step_upper,
    )?;
    if !problem
        .step_lower
        .iter()
        .zip(problem.step_upper.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized equality objective lower bounds must not exceed upper bounds",
        });
    }
    Ok(())
}

fn validate_elastic_equality_objective_step(
    problem: &LinearizedElasticEqualityObjectiveStepProblem,
) -> Result<(), TrajoptError> {
    let base_problem = LinearizedEqualityObjectiveStepProblem {
        jacobian: problem.jacobian.clone(),
        residual: problem.residual.clone(),
        linear_objective: problem.linear_objective.clone(),
        step_lower: problem.step_lower.clone(),
        step_upper: problem.step_upper.clone(),
        regularization: problem.regularization,
    };
    validate_equality_objective_step(&base_problem)?;
    if !problem.slack_penalty.is_finite() || problem.slack_penalty <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized elastic equality objective slack penalty must be finite and positive",
        });
    }
    if !problem.slack_upper.is_finite() || problem.slack_upper <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized elastic equality objective slack upper bound must be finite and positive",
        });
    }
    Ok(())
}

fn validate_constrained_least_squares_step(
    problem: &LinearizedConstrainedLeastSquaresStepProblem,
) -> Result<(), TrajoptError> {
    let variable_count = problem.variable_count();
    if variable_count == 0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained least-squares step requires at least one variable",
        });
    }
    if problem.step_upper.len() != variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained least-squares step bounds must have equal length",
        });
    }
    if problem.residual.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained least-squares step requires at least one residual",
        });
    }
    if problem.inequality_values.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained least-squares step requires at least one inequality",
        });
    }
    if problem.residual_jacobian.len() != problem.residual.len() * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained residual Jacobian dimensions are inconsistent",
        });
    }
    if problem.inequality_jacobian.len() != problem.inequality_values.len() * variable_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained inequality Jacobian dimensions are inconsistent",
        });
    }
    if !problem.regularization.is_finite() || problem.regularization < 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained least-squares regularization must be finite and non-negative",
        });
    }
    validate_finite(
        "linearized constrained residual Jacobian values must be finite",
        &problem.residual_jacobian,
    )?;
    validate_finite(
        "linearized constrained residual values must be finite",
        &problem.residual,
    )?;
    validate_finite(
        "linearized constrained inequality Jacobian values must be finite",
        &problem.inequality_jacobian,
    )?;
    validate_finite(
        "linearized constrained inequality values must be finite",
        &problem.inequality_values,
    )?;
    validate_finite(
        "linearized constrained lower bounds must be finite",
        &problem.step_lower,
    )?;
    validate_finite(
        "linearized constrained upper bounds must be finite",
        &problem.step_upper,
    )?;
    if !problem
        .step_lower
        .iter()
        .zip(problem.step_upper.iter())
        .all(|(lower, upper)| lower <= upper)
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "linearized constrained lower bounds must not exceed upper bounds",
        });
    }
    Ok(())
}

fn validate_sqp_inputs(
    initial_variables: &[f64],
    lower_bounds: &[f64],
    upper_bounds: &[f64],
) -> Result<(), TrajoptError> {
    if initial_variables.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP requires at least one variable",
        });
    }
    if lower_bounds.len() != initial_variables.len()
        || upper_bounds.len() != initial_variables.len()
    {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP bounds must match variable dimension",
        });
    }
    validate_finite(
        "dense SQP initial variables must be finite",
        initial_variables,
    )?;
    validate_finite("dense SQP lower bounds must be finite", lower_bounds)?;
    validate_finite("dense SQP upper bounds must be finite", upper_bounds)?;
    for ((value, lower), upper) in initial_variables
        .iter()
        .zip(lower_bounds.iter())
        .zip(upper_bounds.iter())
    {
        if lower > upper {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP lower bounds must not exceed upper bounds",
            });
        }
        if value < lower || value > upper {
            return Err(TrajoptError::InvalidPayload {
                reason: "dense SQP initial variables must be inside bounds",
            });
        }
    }
    Ok(())
}

fn finite_difference_jacobian<M>(
    model: &mut M,
    variables: &[f64],
    lower_bounds: &[f64],
    upper_bounds: &[f64],
    residual: &[f64],
    residual_count: usize,
    finite_difference_step: f64,
) -> Result<Vec<f64>, TrajoptError>
where
    M: SqpResidualModel,
{
    let variable_count = variables.len();
    let mut jacobian = vec![0.0; residual_count * variable_count];
    for col in 0..variable_count {
        let scale = finite_difference_step * variables[col].abs().max(1.0);
        let forward_room = upper_bounds[col] - variables[col];
        let backward_room = variables[col] - lower_bounds[col];
        if forward_room > 0.0 {
            let step = scale.min(forward_room);
            let mut perturbed = variables.to_vec();
            perturbed[col] += step;
            let perturbed_residual = model.residual(&perturbed)?;
            validate_residual_dimension(&perturbed_residual, residual_count)?;
            for row in 0..residual_count {
                jacobian[row * variable_count + col] =
                    (perturbed_residual[row] - residual[row]) / step;
            }
        } else if backward_room > 0.0 {
            let step = scale.min(backward_room);
            let mut perturbed = variables.to_vec();
            perturbed[col] -= step;
            let perturbed_residual = model.residual(&perturbed)?;
            validate_residual_dimension(&perturbed_residual, residual_count)?;
            for row in 0..residual_count {
                jacobian[row * variable_count + col] =
                    (residual[row] - perturbed_residual[row]) / step;
            }
        }
    }
    Ok(jacobian)
}

fn finite_difference_inequality_jacobian<M>(
    model: &mut M,
    variables: &[f64],
    lower_bounds: &[f64],
    upper_bounds: &[f64],
    inequality_values: &[f64],
    inequality_count: usize,
    finite_difference_step: f64,
) -> Result<Vec<f64>, TrajoptError>
where
    M: SqpInequalityModel,
{
    let variable_count = variables.len();
    let mut jacobian = vec![0.0; inequality_count * variable_count];
    for col in 0..variable_count {
        let scale = finite_difference_step * variables[col].abs().max(1.0);
        let forward_room = upper_bounds[col] - variables[col];
        let backward_room = variables[col] - lower_bounds[col];
        if forward_room > 0.0 {
            let step = scale.min(forward_room);
            let mut perturbed = variables.to_vec();
            perturbed[col] += step;
            let perturbed_inequality = model.inequality(&perturbed)?;
            validate_inequality_dimension(&perturbed_inequality, inequality_count)?;
            for row in 0..inequality_count {
                jacobian[row * variable_count + col] =
                    (perturbed_inequality[row] - inequality_values[row]) / step;
            }
        } else if backward_room > 0.0 {
            let step = scale.min(backward_room);
            let mut perturbed = variables.to_vec();
            perturbed[col] -= step;
            let perturbed_inequality = model.inequality(&perturbed)?;
            validate_inequality_dimension(&perturbed_inequality, inequality_count)?;
            for row in 0..inequality_count {
                jacobian[row * variable_count + col] =
                    (inequality_values[row] - perturbed_inequality[row]) / step;
            }
        }
    }
    Ok(jacobian)
}

fn validate_residual(residual: &[f64]) -> Result<(), TrajoptError> {
    if residual.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP residual dimension must be nonzero",
        });
    }
    validate_finite("dense SQP residual values must be finite", residual)
}

fn validate_residual_dimension(
    residual: &[f64],
    residual_count: usize,
) -> Result<(), TrajoptError> {
    if residual.len() != residual_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP residual dimension changed during solve",
        });
    }
    validate_residual(residual)
}

fn validate_inequality_values(inequality_values: &[f64]) -> Result<(), TrajoptError> {
    if inequality_values.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense constrained SQP inequality dimension must be nonzero",
        });
    }
    validate_finite(
        "dense constrained SQP inequality values must be finite",
        inequality_values,
    )
}

fn validate_inequality_dimension(
    inequality_values: &[f64],
    inequality_count: usize,
) -> Result<(), TrajoptError> {
    if inequality_values.len() != inequality_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense constrained SQP inequality dimension changed during solve",
        });
    }
    validate_inequality_values(inequality_values)
}

fn norm(values: &[f64]) -> f64 {
    values.iter().map(|value| value * value).sum::<f64>().sqrt()
}

fn dot(left: &[f64], right: &[f64]) -> f64 {
    left.iter()
        .zip(right.iter())
        .map(|(left_value, right_value)| left_value * right_value)
        .sum()
}

fn objective_merit(residual_norm: f64, linear_objective_value: f64) -> f64 {
    0.5 * residual_norm * residual_norm + linear_objective_value
}

fn equality_objective_merit(
    residual_norm: f64,
    linear_objective_value: f64,
    residual_penalty: f64,
) -> f64 {
    linear_objective_value + residual_penalty * residual_norm
}

fn elastic_slack_upper(residual: &[f64]) -> f64 {
    residual
        .iter()
        .map(|value| value.abs())
        .fold(1.0_f64, f64::max)
}

fn max_violation(inequality_values: &[f64]) -> f64 {
    inequality_values
        .iter()
        .map(|value| value.max(0.0))
        .fold(0.0_f64, f64::max)
}

fn constrained_merit(residual_norm: f64, inequality_values: &[f64]) -> f64 {
    let violation_sum_sq = inequality_values
        .iter()
        .map(|value| value.max(0.0))
        .map(|violation| violation * violation)
        .sum::<f64>();
    0.5 * residual_norm * residual_norm + violation_sum_sq
}

fn bounded_trial(
    variables: &[f64],
    lower_bounds: &[f64],
    upper_bounds: &[f64],
    step: &[f64],
    scale: f64,
) -> Result<Vec<f64>, TrajoptError> {
    if !scale.is_finite() || scale <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "dense SQP trial scale must be finite and positive",
        });
    }
    let mut trial = Vec::with_capacity(variables.len());
    for index in 0..variables.len() {
        let value = variables[index] + scale * step[index];
        trial.push(value.clamp(lower_bounds[index], upper_bounds[index]));
    }
    Ok(trial)
}

fn validate_finite(reason: &'static str, values: &[f64]) -> Result<(), TrajoptError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TrajoptError::InvalidPayload { reason })
    }
}

fn validate_symmetric_hessian(hessian: &[f64], variable_count: usize) -> Result<(), TrajoptError> {
    for row in 0..variable_count {
        for col in (row + 1)..variable_count {
            let forward = hessian[row * variable_count + col];
            let reverse = hessian[col * variable_count + row];
            let tolerance = 1.0e-12 * forward.abs().max(reverse.abs()).max(1.0);
            if (forward - reverse).abs() > tolerance {
                return Err(TrajoptError::InvalidPayload {
                    reason: "trajectory QP Hessian must be symmetric",
                });
            }
        }
    }
    Ok(())
}

fn csc_from_dense_upper_triangle(dense: &[f64], dim: usize) -> CscMatrix<f64> {
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

fn csc_from_constraints(problem: &DenseQpProblem) -> CscMatrix<f64> {
    let rows = problem.equality_rhs.len() + problem.inequality_rhs.len();
    let cols = problem.variable_count;
    let mut col_ptr = Vec::with_capacity(cols + 1);
    let mut row_idx = Vec::new();
    let mut values = Vec::new();
    col_ptr.push(0);
    for col in 0..cols {
        for row in 0..problem.equality_rhs.len() {
            let value = problem.equality_matrix[row * cols + col];
            if value != 0.0 {
                row_idx.push(row);
                values.push(value);
            }
        }
        for row in 0..problem.inequality_rhs.len() {
            let value = problem.inequality_matrix[row * cols + col];
            if value != 0.0 {
                row_idx.push(problem.equality_rhs.len() + row);
                values.push(value);
            }
        }
        col_ptr.push(values.len());
    }
    CscMatrix::new(rows, cols, col_ptr, row_idx, values)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn dense_qp_solves_equality_constrained_projection() {
        let problem = DenseQpProblem::new(
            2,
            vec![1.0, 0.0, 0.0, 1.0],
            vec![-1.0, -3.0],
            vec![1.0, 1.0],
            vec![3.0],
            Vec::new(),
            Vec::new(),
        )
        .expect("valid QP");

        let solution = solve_dense_qp(&problem).expect("QP solves");
        assert_eq!(solution.status, SolverStatus::Solved);
        assert!((solution.primal[0] - 0.5).abs() < 1.0e-6);
        assert!((solution.primal[1] - 2.5).abs() < 1.0e-6);
    }

    #[test]
    fn dense_qp_solves_upper_bound_inequality() {
        let problem = DenseQpProblem::new(
            1,
            vec![1.0],
            vec![-2.0],
            Vec::new(),
            Vec::new(),
            vec![1.0],
            vec![1.0],
        )
        .expect("valid QP");

        let solution = solve_dense_qp(&problem).expect("QP solves");
        assert!((solution.primal[0] - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn linearized_least_squares_step_respects_box_bounds() {
        let problem = LinearizedLeastSquaresStepProblem {
            jacobian: vec![1.0],
            residual: vec![-2.0],
            step_lower: vec![-0.25],
            step_upper: vec![1.0],
            regularization: 0.0,
        };

        let solution = solve_linearized_least_squares_step(&problem).expect("QP solves");
        assert!((solution.primal[0] - 1.0).abs() < 1.0e-6);
    }

    #[test]
    fn linearized_least_squares_objective_step_adds_linear_term() {
        let problem = LinearizedLeastSquaresObjectiveStepProblem {
            jacobian: vec![1.0],
            residual: vec![-1.0],
            linear_objective: vec![0.25],
            step_lower: vec![0.0],
            step_upper: vec![2.0],
            regularization: 0.0,
        };

        let qp = least_squares_objective_step_qp(&problem).expect("QP converts");
        assert_eq!(qp.hessian, vec![1.0]);
        assert_eq!(qp.linear, vec![-0.75]);

        let solution = solve_linearized_least_squares_objective_step(&problem).expect("QP solves");
        assert!((solution.primal[0] - 0.75).abs() < 1.0e-6);
    }

    #[test]
    fn linearized_least_squares_objective_step_rejects_bad_objective() {
        let bad_dimension = LinearizedLeastSquaresObjectiveStepProblem {
            jacobian: vec![1.0],
            residual: vec![0.0],
            linear_objective: Vec::new(),
            step_lower: vec![-1.0],
            step_upper: vec![1.0],
            regularization: 0.0,
        };
        assert!(least_squares_objective_step_qp(&bad_dimension).is_err());

        let nonfinite_objective = LinearizedLeastSquaresObjectiveStepProblem {
            jacobian: vec![1.0],
            residual: vec![0.0],
            linear_objective: vec![f64::INFINITY],
            step_lower: vec![-1.0],
            step_upper: vec![1.0],
            regularization: 0.0,
        };
        assert!(least_squares_objective_step_qp(&nonfinite_objective).is_err());
    }

    #[test]
    fn linearized_equality_objective_step_moves_along_nullspace() {
        let problem = LinearizedEqualityObjectiveStepProblem {
            jacobian: vec![1.0, 1.0],
            residual: vec![0.0],
            linear_objective: vec![1.0, 0.0],
            step_lower: vec![-0.5, -0.5],
            step_upper: vec![0.5, 0.5],
            regularization: 1.0e-10,
        };

        let qp = equality_objective_step_qp(&problem).expect("QP converts");
        assert_eq!(qp.equality_rhs, vec![-0.0]);
        assert_eq!(qp.equality_matrix, vec![1.0, 1.0]);

        let solution = solve_linearized_equality_objective_step(&problem).expect("QP solves");
        assert!((solution.primal[0] + 0.5).abs() < 1.0e-6);
        assert!((solution.primal[1] - 0.5).abs() < 1.0e-6);
    }

    #[test]
    fn linearized_equality_objective_step_rejects_bad_dimensions() {
        let bad_problem = LinearizedEqualityObjectiveStepProblem {
            jacobian: vec![1.0],
            residual: vec![0.0],
            linear_objective: vec![1.0, 0.0],
            step_lower: vec![-1.0, -1.0],
            step_upper: vec![1.0, 1.0],
            regularization: 0.0,
        };
        assert!(equality_objective_step_qp(&bad_problem).is_err());
    }

    #[test]
    fn linearized_elastic_equality_objective_step_uses_slack_when_bounds_block_residual() {
        let problem = LinearizedElasticEqualityObjectiveStepProblem {
            jacobian: vec![1.0],
            residual: vec![1.0],
            linear_objective: vec![0.0],
            step_lower: vec![0.0],
            step_upper: vec![0.0],
            regularization: 0.0,
            slack_penalty: 10.0,
            slack_upper: 2.0,
        };

        let qp = elastic_equality_objective_step_qp(&problem).expect("QP converts");
        assert_eq!(qp.variable_count, 3);
        assert_eq!(qp.equality_rhs, vec![-1.0]);
        assert_eq!(qp.equality_matrix, vec![1.0, 1.0, -1.0]);

        let solution =
            solve_linearized_elastic_equality_objective_step(&problem).expect("QP solves");
        assert!(solution.primal[0].abs() < 1.0e-8);
        assert!(solution.primal[1].abs() < 1.0e-8);
        assert!((solution.primal[2] - 1.0).abs() < 1.0e-8);
    }

    #[test]
    fn linearized_elastic_equality_objective_step_rejects_bad_slack_settings() {
        let base_problem = LinearizedElasticEqualityObjectiveStepProblem {
            jacobian: vec![1.0],
            residual: vec![0.0],
            linear_objective: vec![0.0],
            step_lower: vec![-1.0],
            step_upper: vec![1.0],
            regularization: 0.0,
            slack_penalty: 1.0,
            slack_upper: 1.0,
        };

        let mut bad_penalty = base_problem.clone();
        bad_penalty.slack_penalty = 0.0;
        assert!(elastic_equality_objective_step_qp(&bad_penalty).is_err());

        let mut bad_upper = base_problem;
        bad_upper.slack_upper = f64::INFINITY;
        assert!(elastic_equality_objective_step_qp(&bad_upper).is_err());
    }

    #[test]
    fn linearized_constrained_least_squares_step_respects_inequality_rows() {
        let problem = LinearizedConstrainedLeastSquaresStepProblem {
            residual_jacobian: vec![1.0],
            residual: vec![-2.0],
            inequality_jacobian: vec![1.0],
            inequality_values: vec![-0.25],
            step_lower: vec![-1.0],
            step_upper: vec![2.0],
            regularization: 0.0,
        };

        let solution =
            solve_linearized_constrained_least_squares_step(&problem).expect("QP solves");
        assert!((solution.primal[0] - 0.25).abs() < 1.0e-6);

        let qp = constrained_least_squares_step_qp(&problem).expect("QP converts");
        assert_eq!(qp.inequality_rhs, vec![2.0, 1.0, 0.25]);
        assert_eq!(qp.inequality_matrix, vec![1.0, -1.0, 1.0]);
    }

    #[test]
    fn dense_qp_solution_is_repeat_deterministic() {
        let problem = DenseQpProblem::new(
            2,
            vec![2.0, 0.25, 0.25, 4.0],
            vec![-1.0, -2.0],
            vec![1.0, -1.0],
            vec![0.0],
            vec![1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 0.0, -1.0],
            vec![1.0, 1.0, 1.0, 1.0],
        )
        .expect("valid QP");

        let first = solve_dense_qp(&problem).expect("first solve");
        let second = solve_dense_qp(&problem).expect("second solve");
        for (first_value, second_value) in first.primal.iter().zip(second.primal.iter()) {
            assert!((first_value - second_value).abs() < 1.0e-12);
        }
        assert_eq!(first.equality_dual.len(), second.equality_dual.len());
        for (first_value, second_value) in
            first.equality_dual.iter().zip(second.equality_dual.iter())
        {
            assert!((first_value - second_value).abs() < 1.0e-12);
        }
        assert_eq!(first.inequality_dual.len(), second.inequality_dual.len());
        for (first_value, second_value) in first
            .inequality_dual
            .iter()
            .zip(second.inequality_dual.iter())
        {
            assert!((first_value - second_value).abs() < 1.0e-12);
        }
        assert!((first.objective - second.objective).abs() < 1.0e-12);
    }

    #[test]
    fn dense_qp_rejects_bad_dimensions_and_values() {
        let bad_hessian = DenseQpProblem::new(
            2,
            vec![1.0, 0.0, 0.0],
            vec![0.0, 0.0],
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        );
        assert!(bad_hessian.is_err());

        let asymmetric = DenseQpProblem::new(
            2,
            vec![1.0, 0.5, 0.0, 1.0],
            vec![0.0, 0.0],
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        );
        assert!(asymmetric.is_err());

        let nonfinite_step = LinearizedLeastSquaresStepProblem {
            jacobian: vec![f64::NAN],
            residual: vec![0.0],
            step_lower: vec![-1.0],
            step_upper: vec![1.0],
            regularization: 0.0,
        };
        assert!(least_squares_step_qp(&nonfinite_step).is_err());
    }

    #[test]
    fn dense_sqp_outer_loop_solves_nonlinear_scalar_residual() {
        let solver = DenseSqpSolver {
            residual_tolerance: 1.0e-10,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve(&[1.0], &[0.0], &[2.0], |variables: &[f64]| {
                Ok(vec![variables[0] * variables[0] - 2.0])
            })
            .expect("SQP solves");

        assert!(solution.converged);
        assert!(solution.iterations > 0);
        assert!((solution.variables[0] - 2.0_f64.sqrt()).abs() < 1.0e-8);
        assert!(solution.residual_norm < 1.0e-10);
    }

    #[test]
    fn dense_sqp_outer_loop_respects_variable_bounds() {
        let solver = DenseSqpSolver {
            max_iterations: 4,
            residual_tolerance: 1.0e-12,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve(&[0.0], &[0.0], &[1.0], |variables: &[f64]| {
                Ok(vec![variables[0] - 3.0])
            })
            .expect("SQP reports bounded best effort");

        assert!(!solution.converged);
        assert!(solution.variables[0] <= 1.0 + 1.0e-9);
        assert!((solution.variables[0] - 1.0).abs() < 1.0e-7);
        assert!((solution.residual[0] + 2.0).abs() < 1.0e-7);
    }

    #[test]
    fn dense_sqp_linear_objective_solves_penalized_scalar_residual() {
        let solver = DenseSqpSolver {
            residual_tolerance: 1.0e-12,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve_with_linear_objective(&[0.0], &[0.0], &[2.0], &[0.25], |variables: &[f64]| {
                Ok(vec![variables[0] - 1.0])
            })
            .expect("objective SQP solves");

        assert!(solution.converged);
        assert!(solution.iterations > 0);
        assert!((solution.variables[0] - 0.75).abs() < 1.0e-6);
        assert!((solution.residual[0] + 0.25).abs() < 1.0e-6);
        assert!((solution.linear_objective_value - 0.1875).abs() < 1.0e-6);
        assert!((solution.merit - 0.21875).abs() < 1.0e-6);
    }

    #[test]
    fn dense_sqp_linear_objective_continues_from_feasible_residual() {
        let solver = DenseSqpSolver {
            residual_tolerance: 1.0e-12,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve_with_linear_objective(&[1.0], &[0.0], &[1.0], &[1.0], |_variables: &[f64]| {
                Ok(vec![0.0])
            })
            .expect("objective SQP solves");

        assert!(solution.converged);
        assert!(solution.iterations > 0);
        assert!(solution.residual_norm <= 1.0e-12);
        assert!(solution.variables[0].abs() < 1.0e-7);
        assert!(solution.linear_objective_value.abs() < 1.0e-7);
        assert!(solution.merit.abs() < 1.0e-7);
    }

    #[test]
    fn dense_sqp_equality_objective_preserves_linear_residual() {
        let solver = DenseSqpSolver {
            residual_tolerance: 1.0e-12,
            step_tolerance: 1.0e-9,
            regularization: 1.0e-10,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve_equalities_with_linear_objective(
                &[0.5, 0.5],
                &[0.0, 0.0],
                &[1.0, 1.0],
                &[1.0, 0.0],
                |variables: &[f64]| Ok(vec![variables[0] + variables[1] - 1.0]),
            )
            .expect("equality objective SQP solves");

        assert!(solution.converged);
        assert!(solution.residual_norm < 1.0e-12);
        assert!(solution.variables[0].abs() < 1.0e-7);
        assert!((solution.variables[1] - 1.0).abs() < 1.0e-7);
        assert!(solution.linear_objective_value.abs() < 1.0e-7);
    }

    #[test]
    fn dense_sqp_equality_objective_accepts_nonlinear_tangent_step() {
        let solver = DenseSqpSolver {
            max_iterations: 1,
            residual_tolerance: 1.0e-8,
            step_tolerance: 1.0e-8,
            regularization: 1.0e-8,
            objective_residual_penalty: 1.0,
            max_line_search_backtracks: 20,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve_equalities_with_linear_objective(
                &[0.5, 0.25],
                &[-1.0, 0.0],
                &[1.0, 1.0],
                &[1.0, 0.0],
                |variables: &[f64]| Ok(vec![variables[1] - variables[0] * variables[0]]),
            )
            .expect("nonlinear equality objective SQP solves");

        assert!(!solution.converged);
        assert_eq!(solution.iterations, 1);
        assert!(
            solution.residual_norm < 1.0e-2,
            "variables {:?}, residual {:e}, merit {:e}, iterations {}",
            solution.variables,
            solution.residual_norm,
            solution.merit,
            solution.iterations
        );
        assert!(
            solution.variables[0] < 0.5,
            "variables {:?}, residual {:e}, merit {:e}, iterations {}",
            solution.variables,
            solution.residual_norm,
            solution.merit,
            solution.iterations
        );
        assert!(solution.merit < 0.5);
        assert!(solution.linear_objective_value < 0.5);
    }

    #[test]
    fn dense_sqp_equality_objective_elastic_fallback_handles_blocked_linearized_residual() {
        let solver = DenseSqpSolver {
            max_iterations: 1,
            residual_tolerance: 1.0e-12,
            step_tolerance: 1.0e-10,
            ..DenseSqpSolver::default()
        };
        let solution = solver
            .solve_equalities_with_linear_objective(
                &[0.0],
                &[0.0],
                &[0.0],
                &[0.0],
                |variables: &[f64]| Ok(vec![variables[0] + 1.0]),
            )
            .expect("elastic fallback reports bounded best effort");

        assert!(!solution.converged);
        assert_eq!(solution.iterations, 0);
        assert_eq!(solution.variables, vec![0.0]);
        assert!((solution.residual_norm - 1.0).abs() < 1.0e-12);
    }

    #[test]
    fn dense_constrained_sqp_outer_loop_respects_nonlinear_inequality() {
        let solver = DenseSqpSolver {
            max_iterations: 8,
            residual_tolerance: 1.0e-12,
            inequality_tolerance: 1.0e-10,
            ..DenseSqpSolver::default()
        };

        let solution = solver
            .solve_with_inequalities(
                &[0.0],
                &[0.0],
                &[2.0],
                |variables: &[f64]| Ok(vec![variables[0] - 1.0]),
                |variables: &[f64]| Ok(vec![variables[0] * variables[0] - 0.25]),
            )
            .expect("constrained SQP reports constrained best effort");

        assert!(!solution.converged);
        assert!((solution.variables[0] - 0.5).abs() < 1.0e-6);
        assert!((solution.residual[0] + 0.5).abs() < 1.0e-6);
        assert!(solution.max_inequality_violation < 1.0e-8);
        assert!(solution.inequality_values[0].abs() < 1.0e-8);
    }

    #[test]
    fn dense_sqp_rejects_unstable_or_nonfinite_residuals() {
        let solver = DenseSqpSolver::default();
        let mut calls = 0usize;
        let unstable = solver.solve(&[1.0], &[0.0], &[2.0], |variables: &[f64]| {
            calls += 1;
            if calls == 1 {
                Ok(vec![variables[0] - 1.5])
            } else {
                Ok(vec![variables[0] - 1.5, 0.0])
            }
        });
        assert!(unstable.is_err());

        let nonfinite = solver.solve(&[1.0], &[0.0], &[2.0], |_variables: &[f64]| {
            Ok(vec![f64::NAN])
        });
        assert!(nonfinite.is_err());

        let bad_inequality_dimension = solver.solve_with_inequalities(
            &[0.0],
            &[0.0],
            &[1.0],
            |_variables: &[f64]| Ok(vec![0.0]),
            |_variables: &[f64]| Ok(Vec::new()),
        );
        assert!(bad_inequality_dimension.is_err());

        let bad_constrained_step = LinearizedConstrainedLeastSquaresStepProblem {
            residual_jacobian: vec![1.0],
            residual: vec![0.0],
            inequality_jacobian: Vec::new(),
            inequality_values: vec![0.0],
            step_lower: vec![-1.0],
            step_upper: vec![1.0],
            regularization: 0.0,
        };
        assert!(constrained_least_squares_step_qp(&bad_constrained_step).is_err());
    }
}
