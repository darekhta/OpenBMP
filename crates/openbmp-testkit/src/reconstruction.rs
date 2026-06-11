//! Linear-Gaussian reconstruction and filter-consistency helpers.
//!
//! These routines are intentionally independent of `openbmp-fc`: they
//! provide deterministic evidence fixtures and reducers that estimator
//! implementations can feed without creating a dependency edge from the
//! shared testkit back into flight-controller code.

use nalgebra::{DMatrix, DVector};

use crate::TestkitError;

/// One scalar or vector consistency statistic acceptance band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ChiSquareBounds {
    /// Degrees of freedom of the chi-square statistic.
    pub dof: usize,
    /// Two-sided confidence level covered by `[lower, upper]`.
    pub confidence: f64,
    /// Lower chi-square quantile.
    pub lower: f64,
    /// Upper chi-square quantile.
    pub upper: f64,
}

/// Fractional in-bounds report for a NEES or NIS statistic stream.
#[derive(Clone, Debug, PartialEq)]
pub struct ConsistencyReport {
    /// Statistic label, for example `nees` or `nis`.
    pub label: String,
    /// Chi-square acceptance band.
    pub bounds: ChiSquareBounds,
    /// Number of evaluated statistics.
    pub samples: usize,
    /// Number of statistics inside `[lower, upper]`.
    pub in_bounds: usize,
    /// `in_bounds / samples`.
    pub in_bounds_fraction: f64,
}

/// A filtered state/covariance sample plus one-step prediction data
/// needed by the Rauch-Tung-Striebel backward pass.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearFilterStep {
    /// `x_k|k`.
    pub filtered_state: DVector<f64>,
    /// `P_k|k`.
    pub filtered_covariance: DMatrix<f64>,
    /// `x_k|k-1`.
    pub predicted_state: DVector<f64>,
    /// `P_k|k-1`.
    pub predicted_covariance: DMatrix<f64>,
    /// Transition matrix from this step to the next step, `F_k`.
    pub transition_to_next: DMatrix<f64>,
}

/// One smoothed state/covariance sample.
#[derive(Clone, Debug, PartialEq)]
pub struct LinearSmootherStep {
    /// `x_k|N`.
    pub smoothed_state: DVector<f64>,
    /// `P_k|N`.
    pub smoothed_covariance: DMatrix<f64>,
}

/// One weighted linearized observation for a batch Gauss-Newton
/// normal-equation solve.
#[derive(Clone, Debug, PartialEq)]
pub struct BatchObservation {
    /// Observation sensitivity row `H_i` as a column vector.
    pub sensitivity: DVector<f64>,
    /// Scalar residual `z_i - h_i(x)`.
    pub residual: f64,
    /// Scalar observation variance.
    pub variance: f64,
}

/// Result of one linear batch Gauss-Newton solve.
#[derive(Clone, Debug, PartialEq)]
pub struct BatchGaussNewtonReport {
    /// Estimated state increment.
    pub update: DVector<f64>,
    /// Posterior covariance, equal to the inverse normal matrix.
    pub covariance: DMatrix<f64>,
    /// Normal matrix `H^T R^-1 H`.
    pub normal_matrix: DMatrix<f64>,
    /// Weighted residual norm before applying the update.
    pub weighted_residual_norm: f64,
}

/// Synthetic linear-Gaussian reconstruction evidence report.
#[derive(Clone, Debug, PartialEq)]
pub struct SyntheticReconstructionReport {
    /// NEES consistency report.
    pub nees: ConsistencyReport,
    /// NIS consistency report.
    pub nis: ConsistencyReport,
    /// Smoothed scalar states from the RTS backward pass.
    pub smoothed_states: Vec<f64>,
    /// Batch Gauss-Newton update estimate.
    pub batch_update: Vec<f64>,
    /// Euclidean error in the batch update against the known truth.
    pub batch_update_error_norm: f64,
    /// Whether both consistency fractions meet the WP-11.6 first-slice
    /// band `[0.93, 0.97]`.
    pub passed: bool,
}

/// Return the checked-in 95% two-sided chi-square band for common
/// estimator dimensions.
///
/// # Errors
///
/// Returns [`TestkitError`] for unsupported degrees of freedom.
pub fn chi_square_95_bounds(dof: usize) -> Result<ChiSquareBounds, TestkitError> {
    let (lower, upper) = match dof {
        1 => (9.820_691_170_756e-4, 5.023_886_187_315),
        2 => (5.063_561_596_858e-2, 7.377_758_908_228),
        3 => (2.157_952_826_239e-1, 9.348_403_604_496),
        6 => (1.237_344_245_791, 14.449_375_335_448),
        15 => (6.262_137_795_043, 27.488_392_863_442),
        _ => {
            return Err(TestkitError::InvalidReconstructionInput {
                field: "dof",
                rule: "supported chi-square 95% bounds are tabulated for dof 1, 2, 3, 6, and 15",
            });
        }
    };
    Ok(ChiSquareBounds {
        dof,
        confidence: 0.95,
        lower,
        upper,
    })
}

/// Compute `e^T P^-1 e` for a positive-definite covariance `P`.
///
/// # Errors
///
/// Returns [`TestkitError`] for shape mismatch, non-finite data, or
/// non-positive-definite covariance.
pub fn normalized_error_squared(
    error: &DVector<f64>,
    covariance: &DMatrix<f64>,
) -> Result<f64, TestkitError> {
    require_square_covariance("covariance", covariance)?;
    if covariance.nrows() != error.len() {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "error",
            rule: "error length must match covariance dimension",
        });
    }
    require_finite_vector("error", error)?;
    require_finite_matrix("covariance", covariance)?;
    let cholesky =
        covariance
            .clone()
            .cholesky()
            .ok_or(TestkitError::ReconstructionSolveFailed {
                field: "covariance",
            })?;
    let solved = cholesky.solve(error);
    Ok(error.dot(&solved))
}

/// Summarize a stream of NEES/NIS statistics against chi-square bounds.
///
/// # Errors
///
/// Returns [`TestkitError`] when the statistic stream is empty or
/// contains a non-finite value.
pub fn consistency_report(
    label: impl Into<String>,
    statistics: &[f64],
    bounds: ChiSquareBounds,
) -> Result<ConsistencyReport, TestkitError> {
    if statistics.is_empty() {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "statistics",
            rule: "at least one statistic is required",
        });
    }
    if !statistics.iter().all(|value| value.is_finite()) {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "statistics",
            rule: "all statistics must be finite",
        });
    }
    let in_bounds = statistics
        .iter()
        .filter(|value| (bounds.lower..=bounds.upper).contains(value))
        .count();
    Ok(ConsistencyReport {
        label: label.into(),
        bounds,
        samples: statistics.len(),
        in_bounds,
        in_bounds_fraction: in_bounds as f64 / statistics.len() as f64,
    })
}

/// Run a Rauch-Tung-Striebel backward smoother over filtered samples.
///
/// # Errors
///
/// Returns [`TestkitError`] for empty input, inconsistent dimensions,
/// non-finite matrices, or singular predicted covariance.
pub fn rts_smooth(steps: &[LinearFilterStep]) -> Result<Vec<LinearSmootherStep>, TestkitError> {
    validate_filter_steps(steps)?;
    let mut smoothed: Vec<LinearSmootherStep> = steps
        .iter()
        .map(|step| LinearSmootherStep {
            smoothed_state: step.filtered_state.clone(),
            smoothed_covariance: step.filtered_covariance.clone(),
        })
        .collect();

    for k in (0..steps.len() - 1).rev() {
        let next_pred_cov = &steps[k + 1].predicted_covariance;
        let cholesky =
            next_pred_cov
                .clone()
                .cholesky()
                .ok_or(TestkitError::ReconstructionSolveFailed {
                    field: "predicted_covariance",
                })?;
        let rhs = &steps[k].transition_to_next * &steps[k].filtered_covariance;
        let gain = cholesky.solve(&rhs).transpose();
        let state_delta = &smoothed[k + 1].smoothed_state - &steps[k + 1].predicted_state;
        let cov_delta = &smoothed[k + 1].smoothed_covariance - next_pred_cov;
        smoothed[k].smoothed_state = &steps[k].filtered_state + &gain * state_delta;
        smoothed[k].smoothed_covariance =
            &steps[k].filtered_covariance + &gain * cov_delta * gain.transpose();
    }

    Ok(smoothed)
}

/// Solve one linear batch Gauss-Newton normal equation.
///
/// # Errors
///
/// Returns [`TestkitError`] when inputs are empty, malformed, or the
/// normal matrix is singular.
pub fn batch_gauss_newton(
    state_dimension: usize,
    observations: &[BatchObservation],
) -> Result<BatchGaussNewtonReport, TestkitError> {
    if state_dimension == 0 {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "state_dimension",
            rule: "state dimension must be positive",
        });
    }
    if observations.is_empty() {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "observations",
            rule: "at least one observation is required",
        });
    }

    let mut normal = DMatrix::<f64>::zeros(state_dimension, state_dimension);
    let mut rhs = DVector::<f64>::zeros(state_dimension);
    let mut weighted_residual_sum = 0.0;
    for observation in observations {
        if observation.sensitivity.len() != state_dimension {
            return Err(TestkitError::InvalidReconstructionInput {
                field: "sensitivity",
                rule: "sensitivity length must match state dimension",
            });
        }
        require_finite_vector("sensitivity", &observation.sensitivity)?;
        if !observation.residual.is_finite()
            || !observation.variance.is_finite()
            || observation.variance <= 0.0
        {
            return Err(TestkitError::InvalidReconstructionInput {
                field: "observation",
                rule: "residual must be finite and variance must be positive finite",
            });
        }
        let weight = 1.0 / observation.variance;
        normal += (&observation.sensitivity * observation.sensitivity.transpose()) * weight;
        rhs += &observation.sensitivity * (observation.residual * weight);
        weighted_residual_sum += observation.residual * observation.residual * weight;
    }

    let cholesky = normal
        .clone()
        .cholesky()
        .ok_or(TestkitError::ReconstructionSolveFailed {
            field: "normal_matrix",
        })?;
    let update = cholesky.solve(&rhs);
    let identity = DMatrix::<f64>::identity(state_dimension, state_dimension);
    let covariance = cholesky.solve(&identity);
    Ok(BatchGaussNewtonReport {
        update,
        covariance,
        normal_matrix: normal,
        weighted_residual_norm: weighted_residual_sum.sqrt(),
    })
}

/// Build the deterministic synthetic WP-11.6 first-slice evidence
/// report.
///
/// # Errors
///
/// Returns [`TestkitError`] if any helper rejects its constructed
/// evidence. This should not happen unless the fixture is edited.
pub fn synthetic_linear_reconstruction_report()
-> Result<SyntheticReconstructionReport, TestkitError> {
    let bounds = chi_square_95_bounds(1)?;
    let nees_statistics = synthetic_statistics(1.0, 2.5);
    let nis_statistics = synthetic_statistics(0.8, 2.6);
    let nees = consistency_report("nees", &nees_statistics, bounds)?;
    let nis = consistency_report("nis", &nis_statistics, bounds)?;

    let smoothed = rts_smooth(&synthetic_filter_steps())?;
    let smoothed_states = smoothed
        .iter()
        .map(|step| step.smoothed_state[0])
        .collect::<Vec<_>>();

    let truth = DVector::from_vec(vec![1.25, -0.2]);
    let observations = synthetic_batch_observations(&truth);
    let batch = batch_gauss_newton(2, &observations)?;
    let batch_error = &batch.update - truth;
    let batch_update_error_norm = batch_error.norm();

    let passed = (0.93..=0.97).contains(&nees.in_bounds_fraction)
        && (0.93..=0.97).contains(&nis.in_bounds_fraction)
        && batch_update_error_norm <= 1.0e-12;

    Ok(SyntheticReconstructionReport {
        nees,
        nis,
        smoothed_states,
        batch_update: batch.update.iter().copied().collect(),
        batch_update_error_norm,
        passed,
    })
}

fn synthetic_statistics(inlier_sigma: f64, outlier_sigma: f64) -> Vec<f64> {
    let mut values = Vec::with_capacity(100);
    values.extend((0..95).map(|_| inlier_sigma * inlier_sigma));
    values.extend((0..5).map(|_| outlier_sigma * outlier_sigma));
    values
}

fn synthetic_filter_steps() -> Vec<LinearFilterStep> {
    let f = DMatrix::from_vec(1, 1, vec![1.0]);
    vec![
        LinearFilterStep {
            filtered_state: DVector::from_vec(vec![0.0]),
            filtered_covariance: DMatrix::from_vec(1, 1, vec![0.6]),
            predicted_state: DVector::from_vec(vec![0.0]),
            predicted_covariance: DMatrix::from_vec(1, 1, vec![1.0]),
            transition_to_next: f.clone(),
        },
        LinearFilterStep {
            filtered_state: DVector::from_vec(vec![1.1]),
            filtered_covariance: DMatrix::from_vec(1, 1, vec![0.4]),
            predicted_state: DVector::from_vec(vec![0.2]),
            predicted_covariance: DMatrix::from_vec(1, 1, vec![0.9]),
            transition_to_next: f.clone(),
        },
        LinearFilterStep {
            filtered_state: DVector::from_vec(vec![1.9]),
            filtered_covariance: DMatrix::from_vec(1, 1, vec![0.25]),
            predicted_state: DVector::from_vec(vec![1.2]),
            predicted_covariance: DMatrix::from_vec(1, 1, vec![0.65]),
            transition_to_next: f,
        },
    ]
}

fn synthetic_batch_observations(truth: &DVector<f64>) -> Vec<BatchObservation> {
    [0.0, 1.0, 2.0, 3.0]
        .into_iter()
        .map(|time_s| {
            let sensitivity = DVector::from_vec(vec![1.0, time_s]);
            let residual = sensitivity.dot(truth);
            BatchObservation {
                sensitivity,
                residual,
                variance: 1.0,
            }
        })
        .collect()
}

fn validate_filter_steps(steps: &[LinearFilterStep]) -> Result<(), TestkitError> {
    if steps.is_empty() {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "steps",
            rule: "at least one filter step is required",
        });
    }
    let dimension = steps[0].filtered_state.len();
    if dimension == 0 {
        return Err(TestkitError::InvalidReconstructionInput {
            field: "filtered_state",
            rule: "state dimension must be positive",
        });
    }
    for step in steps {
        require_vector_dimension("filtered_state", &step.filtered_state, dimension)?;
        require_vector_dimension("predicted_state", &step.predicted_state, dimension)?;
        require_matrix_dimension("filtered_covariance", &step.filtered_covariance, dimension)?;
        require_matrix_dimension(
            "predicted_covariance",
            &step.predicted_covariance,
            dimension,
        )?;
        require_matrix_dimension("transition_to_next", &step.transition_to_next, dimension)?;
        require_finite_vector("filtered_state", &step.filtered_state)?;
        require_finite_vector("predicted_state", &step.predicted_state)?;
        require_finite_matrix("filtered_covariance", &step.filtered_covariance)?;
        require_finite_matrix("predicted_covariance", &step.predicted_covariance)?;
        require_finite_matrix("transition_to_next", &step.transition_to_next)?;
    }
    Ok(())
}

fn require_square_covariance(
    field: &'static str,
    covariance: &DMatrix<f64>,
) -> Result<(), TestkitError> {
    if covariance.nrows() == 0 || covariance.nrows() != covariance.ncols() {
        return Err(TestkitError::InvalidReconstructionInput {
            field,
            rule: "matrix must be non-empty and square",
        });
    }
    Ok(())
}

fn require_vector_dimension(
    field: &'static str,
    vector: &DVector<f64>,
    dimension: usize,
) -> Result<(), TestkitError> {
    if vector.len() != dimension {
        return Err(TestkitError::InvalidReconstructionInput {
            field,
            rule: "vector dimension mismatch",
        });
    }
    Ok(())
}

fn require_matrix_dimension(
    field: &'static str,
    matrix: &DMatrix<f64>,
    dimension: usize,
) -> Result<(), TestkitError> {
    if matrix.nrows() != dimension || matrix.ncols() != dimension {
        return Err(TestkitError::InvalidReconstructionInput {
            field,
            rule: "matrix dimension mismatch",
        });
    }
    Ok(())
}

fn require_finite_vector(field: &'static str, vector: &DVector<f64>) -> Result<(), TestkitError> {
    if vector.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TestkitError::InvalidReconstructionInput {
            field,
            rule: "all vector entries must be finite",
        })
    }
}

fn require_finite_matrix(field: &'static str, matrix: &DMatrix<f64>) -> Result<(), TestkitError> {
    if matrix.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(TestkitError::InvalidReconstructionInput {
            field,
            rule: "all matrix entries must be finite",
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn normalized_error_squared_matches_scalar_nees() -> Result<(), TestkitError> {
        let stat = normalized_error_squared(
            &DVector::from_vec(vec![2.0]),
            &DMatrix::from_vec(1, 1, vec![4.0]),
        )?;
        assert!((stat - 1.0).abs() <= 1.0e-12);
        Ok(())
    }

    #[test]
    fn consistency_report_counts_expected_fraction() -> Result<(), TestkitError> {
        let bounds = chi_square_95_bounds(1)?;
        let report = consistency_report("nees", &synthetic_statistics(1.0, 2.5), bounds)?;
        assert_eq!(report.samples, 100);
        assert_eq!(report.in_bounds, 95);
        assert!((report.in_bounds_fraction - 0.95).abs() <= 1.0e-12);
        Ok(())
    }

    #[test]
    fn rts_smoother_preserves_terminal_filtered_state() -> Result<(), TestkitError> {
        let smoothed = rts_smooth(&synthetic_filter_steps())?;
        assert_eq!(smoothed.len(), 3);
        assert!((smoothed[2].smoothed_state[0] - 1.9).abs() <= 1.0e-12);
        assert!(smoothed[0].smoothed_state[0] > 0.0);
        Ok(())
    }

    #[test]
    fn batch_gauss_newton_recovers_linear_truth() -> Result<(), TestkitError> {
        let truth = DVector::from_vec(vec![1.25, -0.2]);
        let report = batch_gauss_newton(2, &synthetic_batch_observations(&truth))?;
        let error = report.update - truth;
        assert!(error.norm() <= 1.0e-12);
        Ok(())
    }

    #[test]
    fn synthetic_reconstruction_report_passes_first_slice_gate() -> Result<(), TestkitError> {
        let report = synthetic_linear_reconstruction_report()?;
        assert!(report.passed);
        assert!((0.93..=0.97).contains(&report.nees.in_bounds_fraction));
        assert!((0.93..=0.97).contains(&report.nis.in_bounds_fraction));
        Ok(())
    }
}
