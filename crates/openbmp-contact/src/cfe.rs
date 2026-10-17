//! Constraint Force Equation helpers.

use crate::{
    ContactError,
    error::{require_finite, require_non_negative, require_positive},
};

/// Baumgarte stabilization gains for an acceleration-level scalar constraint.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BaumgarteCfeConfig {
    alpha_1_per_s: f64,
    beta_1_per_s: f64,
}

impl BaumgarteCfeConfig {
    /// Creates Baumgarte gains for `phi_ddot + 2*alpha*phi_dot + beta^2*phi = 0`.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when either gain is
    /// non-finite or negative.
    pub fn new(alpha_1_per_s: f64, beta_1_per_s: f64) -> Result<Self, ContactError> {
        Ok(Self {
            alpha_1_per_s: require_non_negative("alpha_1_per_s", alpha_1_per_s)?,
            beta_1_per_s: require_non_negative("beta_1_per_s", beta_1_per_s)?,
        })
    }

    /// Returns the first-order damping gain.
    #[must_use]
    pub const fn alpha_1_per_s(self) -> f64 {
        self.alpha_1_per_s
    }

    /// Returns the second-order frequency gain.
    #[must_use]
    pub const fn beta_1_per_s(self) -> f64 {
        self.beta_1_per_s
    }

    fn desired_constraint_acceleration(self, constraint: ScalarCfeConstraint) -> f64 {
        -2.0 * self.alpha_1_per_s * constraint.velocity_residual_per_s
            - self.beta_1_per_s * self.beta_1_per_s * constraint.position_residual
    }
}

/// Scalar acceleration-level CFE input assembled by a higher-level multibody system.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarCfeConstraint {
    /// Constraint position residual `phi`.
    pub position_residual: f64,
    /// Constraint velocity residual `phi_dot`.
    pub velocity_residual_per_s: f64,
    /// Constraint acceleration without the constraint force, `J*qdd_free`.
    pub unconstrained_acceleration_per_s2: f64,
    /// Effective inverse mass `J*M^-1*J^T`; must be positive.
    pub effective_inverse_mass: f64,
}

impl ScalarCfeConstraint {
    /// Creates a scalar CFE input.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] for non-finite residuals or a
    /// non-positive effective inverse mass.
    pub fn new(
        position_residual: f64,
        velocity_residual_per_s: f64,
        unconstrained_acceleration_per_s2: f64,
        effective_inverse_mass: f64,
    ) -> Result<Self, ContactError> {
        Ok(Self {
            position_residual: require_finite("position_residual", position_residual)?,
            velocity_residual_per_s: require_finite(
                "velocity_residual_per_s",
                velocity_residual_per_s,
            )?,
            unconstrained_acceleration_per_s2: require_finite(
                "unconstrained_acceleration_per_s2",
                unconstrained_acceleration_per_s2,
            )?,
            effective_inverse_mass: require_positive(
                "effective_inverse_mass",
                effective_inverse_mass,
            )?,
        })
    }
}

/// Scalar CFE solve result.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ScalarCfeSolution {
    /// Baumgarte-requested constraint acceleration.
    pub desired_constraint_acceleration_per_s2: f64,
    /// Constraint Lagrange multiplier in the caller's generalized-force units.
    pub lagrange_multiplier: f64,
    /// Constraint acceleration after applying the Lagrange multiplier.
    pub stabilized_constraint_acceleration_per_s2: f64,
    /// Difference between stabilized and desired constraint acceleration.
    pub acceleration_residual_per_s2: f64,
}

/// Caller-owned dense acceleration-level CFE system.
///
/// The effective inverse mass matrix is row-major with dimension
/// `constraint_count x constraint_count`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseCfeSystem<'a> {
    position_residuals: &'a [f64],
    velocity_residuals_per_s: &'a [f64],
    unconstrained_accelerations_per_s2: &'a [f64],
    effective_inverse_mass_row_major: &'a [f64],
}

impl<'a> DenseCfeSystem<'a> {
    /// Creates a validated dense CFE system view.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when the slices are empty,
    /// have inconsistent dimensions, or contain non-finite values.
    pub fn new(
        position_residuals: &'a [f64],
        velocity_residuals_per_s: &'a [f64],
        unconstrained_accelerations_per_s2: &'a [f64],
        effective_inverse_mass_row_major: &'a [f64],
    ) -> Result<Self, ContactError> {
        let constraint_count = position_residuals.len();
        if constraint_count == 0 {
            return Err(ContactError::InvalidParameter {
                field: "dense_cfe_system",
                reason: "must contain at least one constraint",
            });
        }
        if velocity_residuals_per_s.len() != constraint_count
            || unconstrained_accelerations_per_s2.len() != constraint_count
        {
            return Err(ContactError::InvalidParameter {
                field: "dense_cfe_system",
                reason: "residual vector lengths must match",
            });
        }
        let matrix_len = constraint_count.checked_mul(constraint_count).ok_or(
            ContactError::InvalidParameter {
                field: "dense_cfe_system",
                reason: "constraint count is too large",
            },
        )?;
        if effective_inverse_mass_row_major.len() != matrix_len {
            return Err(ContactError::InvalidParameter {
                field: "effective_inverse_mass_row_major",
                reason: "matrix length must be constraint_count squared",
            });
        }

        require_all_finite("position_residuals", position_residuals)?;
        require_all_finite("velocity_residuals_per_s", velocity_residuals_per_s)?;
        require_all_finite(
            "unconstrained_accelerations_per_s2",
            unconstrained_accelerations_per_s2,
        )?;
        require_all_finite(
            "effective_inverse_mass_row_major",
            effective_inverse_mass_row_major,
        )?;

        Ok(Self {
            position_residuals,
            velocity_residuals_per_s,
            unconstrained_accelerations_per_s2,
            effective_inverse_mass_row_major,
        })
    }

    /// Returns the number of scalar constraint rows.
    #[must_use]
    pub const fn constraint_count(self) -> usize {
        self.position_residuals.len()
    }
}

/// Caller-owned workspace and outputs for a dense CFE solve.
#[derive(Debug, PartialEq)]
pub struct DenseCfeSolveBuffers<'a> {
    matrix_scratch_row_major: &'a mut [f64],
    rhs_scratch: &'a mut [f64],
    lagrange_multipliers: &'a mut [f64],
    stabilized_constraint_accelerations_per_s2: &'a mut [f64],
    acceleration_residuals_per_s2: &'a mut [f64],
}

impl<'a> DenseCfeSolveBuffers<'a> {
    /// Creates a validated dense CFE buffer view.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when any slice length is
    /// inconsistent with `constraint_count`.
    pub fn new(
        constraint_count: usize,
        matrix_scratch_row_major: &'a mut [f64],
        rhs_scratch: &'a mut [f64],
        lagrange_multipliers: &'a mut [f64],
        stabilized_constraint_accelerations_per_s2: &'a mut [f64],
        acceleration_residuals_per_s2: &'a mut [f64],
    ) -> Result<Self, ContactError> {
        if constraint_count == 0 {
            return Err(ContactError::InvalidParameter {
                field: "constraint_count",
                reason: "must be positive",
            });
        }
        let matrix_len = constraint_count.checked_mul(constraint_count).ok_or(
            ContactError::InvalidParameter {
                field: "constraint_count",
                reason: "is too large",
            },
        )?;
        if matrix_scratch_row_major.len() != matrix_len {
            return Err(ContactError::InvalidParameter {
                field: "matrix_scratch_row_major",
                reason: "length must be constraint_count squared",
            });
        }
        if rhs_scratch.len() != constraint_count
            || lagrange_multipliers.len() != constraint_count
            || stabilized_constraint_accelerations_per_s2.len() != constraint_count
            || acceleration_residuals_per_s2.len() != constraint_count
        {
            return Err(ContactError::InvalidParameter {
                field: "dense_cfe_buffers",
                reason: "vector lengths must match constraint_count",
            });
        }
        Ok(Self {
            matrix_scratch_row_major,
            rhs_scratch,
            lagrange_multipliers,
            stabilized_constraint_accelerations_per_s2,
            acceleration_residuals_per_s2,
        })
    }
}

/// Summary returned after a dense CFE solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DenseCfeSolutionSummary {
    /// Number of scalar constraint rows solved.
    pub constraint_count: usize,
    /// Maximum absolute acceleration residual across all rows.
    pub max_abs_acceleration_residual_per_s2: f64,
}

/// Tree-facing CFE loop system assembled by a higher-level multibody crate.
///
/// The caller supplies the row-space unconstrained accelerations and effective
/// inverse mass matrix from the tree dynamics assembly, plus the constraint
/// Jacobian used to project solved Lagrange multipliers back to generalized
/// forces through `G^T * lambda`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeCfeLoopSystem<'a> {
    generalized_dof_count: usize,
    position_residuals: &'a [f64],
    velocity_residuals_per_s: &'a [f64],
    unconstrained_constraint_accelerations_per_s2: &'a [f64],
    effective_inverse_mass_row_major: &'a [f64],
    constraint_jacobian_row_major: &'a [f64],
}

impl<'a> TreeCfeLoopSystem<'a> {
    /// Creates a validated tree-facing loop-constraint system view.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when dimensions are
    /// inconsistent, the generalized DOF count is zero, or any entry is
    /// non-finite.
    pub fn new(
        generalized_dof_count: usize,
        position_residuals: &'a [f64],
        velocity_residuals_per_s: &'a [f64],
        unconstrained_constraint_accelerations_per_s2: &'a [f64],
        effective_inverse_mass_row_major: &'a [f64],
        constraint_jacobian_row_major: &'a [f64],
    ) -> Result<Self, ContactError> {
        if generalized_dof_count == 0 {
            return Err(ContactError::InvalidParameter {
                field: "generalized_dof_count",
                reason: "must be positive",
            });
        }
        let dense = DenseCfeSystem::new(
            position_residuals,
            velocity_residuals_per_s,
            unconstrained_constraint_accelerations_per_s2,
            effective_inverse_mass_row_major,
        )?;
        let constraint_count = dense.constraint_count();
        let expected_jacobian_len = constraint_count.checked_mul(generalized_dof_count).ok_or(
            ContactError::InvalidParameter {
                field: "tree_cfe_loop_system",
                reason: "constraint count times generalized DOF count is too large",
            },
        )?;
        if constraint_jacobian_row_major.len() != expected_jacobian_len {
            return Err(ContactError::InvalidParameter {
                field: "constraint_jacobian_row_major",
                reason: "length must be constraint_count times generalized_dof_count",
            });
        }
        require_all_finite(
            "constraint_jacobian_row_major",
            constraint_jacobian_row_major,
        )?;

        Ok(Self {
            generalized_dof_count,
            position_residuals,
            velocity_residuals_per_s,
            unconstrained_constraint_accelerations_per_s2,
            effective_inverse_mass_row_major,
            constraint_jacobian_row_major,
        })
    }

    /// Number of generalized coordinates receiving projected constraint force.
    #[must_use]
    pub const fn generalized_dof_count(self) -> usize {
        self.generalized_dof_count
    }

    /// Number of scalar loop-constraint rows.
    #[must_use]
    pub const fn constraint_count(self) -> usize {
        self.position_residuals.len()
    }
}

/// Caller-owned workspace and outputs for a tree-facing CFE loop solve.
#[derive(Debug, PartialEq)]
pub struct TreeCfeLoopSolveBuffers<'a> {
    matrix_scratch_row_major: &'a mut [f64],
    rhs_scratch: &'a mut [f64],
    lagrange_multipliers: &'a mut [f64],
    stabilized_constraint_accelerations_per_s2: &'a mut [f64],
    acceleration_residuals_per_s2: &'a mut [f64],
    generalized_constraint_forces: &'a mut [f64],
}

impl<'a> TreeCfeLoopSolveBuffers<'a> {
    /// Creates a validated tree-facing CFE buffer view.
    ///
    /// # Errors
    ///
    /// Returns [`ContactError::InvalidParameter`] when any slice length is
    /// inconsistent with the constraint or generalized DOF counts.
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        constraint_count: usize,
        generalized_dof_count: usize,
        matrix_scratch_row_major: &'a mut [f64],
        rhs_scratch: &'a mut [f64],
        lagrange_multipliers: &'a mut [f64],
        stabilized_constraint_accelerations_per_s2: &'a mut [f64],
        acceleration_residuals_per_s2: &'a mut [f64],
        generalized_constraint_forces: &'a mut [f64],
    ) -> Result<Self, ContactError> {
        if generalized_dof_count == 0 {
            return Err(ContactError::InvalidParameter {
                field: "generalized_dof_count",
                reason: "must be positive",
            });
        }
        if generalized_constraint_forces.len() != generalized_dof_count {
            return Err(ContactError::InvalidParameter {
                field: "generalized_constraint_forces",
                reason: "length must match generalized_dof_count",
            });
        }
        let dense = DenseCfeSolveBuffers::new(
            constraint_count,
            matrix_scratch_row_major,
            rhs_scratch,
            lagrange_multipliers,
            stabilized_constraint_accelerations_per_s2,
            acceleration_residuals_per_s2,
        )?;
        Ok(Self {
            matrix_scratch_row_major: dense.matrix_scratch_row_major,
            rhs_scratch: dense.rhs_scratch,
            lagrange_multipliers: dense.lagrange_multipliers,
            stabilized_constraint_accelerations_per_s2: dense
                .stabilized_constraint_accelerations_per_s2,
            acceleration_residuals_per_s2: dense.acceleration_residuals_per_s2,
            generalized_constraint_forces,
        })
    }
}

/// Summary returned after a tree-facing CFE loop solve.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TreeCfeLoopSolutionSummary {
    /// Number of scalar constraint rows solved.
    pub constraint_count: usize,
    /// Number of generalized coordinates receiving projected force.
    pub generalized_dof_count: usize,
    /// Maximum absolute acceleration residual across all rows.
    pub max_abs_acceleration_residual_per_s2: f64,
    /// Maximum absolute Lagrange multiplier magnitude.
    pub max_abs_lagrange_multiplier: f64,
    /// Maximum absolute generalized constraint-force component.
    pub max_abs_generalized_constraint_force: f64,
}

/// Solves one scalar Baumgarte-stabilized CFE row.
///
/// The caller owns the tree dynamics assembly and supplies `J*qdd_free` plus
/// `J*M^-1*J^T`. This helper solves
/// `J*M^-1*J^T * lambda = desired_phi_ddot - J*qdd_free` deterministically for
/// one scalar row.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] if the effective inverse mass is
/// invalid or if the result is non-finite.
pub fn solve_scalar_baumgarte_cfe(
    config: BaumgarteCfeConfig,
    constraint: ScalarCfeConstraint,
) -> Result<ScalarCfeSolution, ContactError> {
    let desired_constraint_acceleration_per_s2 = config.desired_constraint_acceleration(constraint);
    let lagrange_multiplier = (desired_constraint_acceleration_per_s2
        - constraint.unconstrained_acceleration_per_s2)
        / constraint.effective_inverse_mass;
    let stabilized_constraint_acceleration_per_s2 = constraint.unconstrained_acceleration_per_s2
        + constraint.effective_inverse_mass * lagrange_multiplier;
    let acceleration_residual_per_s2 =
        stabilized_constraint_acceleration_per_s2 - desired_constraint_acceleration_per_s2;

    if !desired_constraint_acceleration_per_s2.is_finite()
        || !lagrange_multiplier.is_finite()
        || !stabilized_constraint_acceleration_per_s2.is_finite()
        || !acceleration_residual_per_s2.is_finite()
    {
        return Err(ContactError::InvalidParameter {
            field: "scalar_cfe_solution",
            reason: "must be finite",
        });
    }

    Ok(ScalarCfeSolution {
        desired_constraint_acceleration_per_s2,
        lagrange_multiplier,
        stabilized_constraint_acceleration_per_s2,
        acceleration_residual_per_s2,
    })
}

/// Solves a dense fixed-pivot Baumgarte-stabilized CFE system.
///
/// The caller owns multibody assembly and supplies the row-major effective
/// inverse mass matrix `J*M^-1*J^T`. This helper computes the desired
/// Baumgarte acceleration for each row, solves the symmetric positive-definite
/// system with deterministic no-pivot LDL^T factorization, writes
/// Lagrange-multiplier and residual outputs into caller-owned buffers, and
/// leaves integration of `J^T*lambda` to higher layers.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] for invalid dimensions,
/// non-positive LDL^T pivots, or non-finite outputs.
pub fn solve_dense_baumgarte_cfe(
    config: BaumgarteCfeConfig,
    system: DenseCfeSystem<'_>,
    buffers: DenseCfeSolveBuffers<'_>,
) -> Result<DenseCfeSolutionSummary, ContactError> {
    let constraint_count = system.constraint_count();
    let buffers = DenseCfeSolveBuffers::new(
        constraint_count,
        buffers.matrix_scratch_row_major,
        buffers.rhs_scratch,
        buffers.lagrange_multipliers,
        buffers.stabilized_constraint_accelerations_per_s2,
        buffers.acceleration_residuals_per_s2,
    )?;

    buffers
        .matrix_scratch_row_major
        .copy_from_slice(system.effective_inverse_mass_row_major);
    for row in 0..constraint_count {
        let desired = config.desired_constraint_acceleration(ScalarCfeConstraint::new(
            system.position_residuals[row],
            system.velocity_residuals_per_s[row],
            system.unconstrained_accelerations_per_s2[row],
            1.0,
        )?);
        buffers.rhs_scratch[row] = desired - system.unconstrained_accelerations_per_s2[row];
    }

    ldlt_factor_in_place(buffers.matrix_scratch_row_major, constraint_count)?;
    ldlt_solve_factored(
        buffers.matrix_scratch_row_major,
        buffers.rhs_scratch,
        buffers.lagrange_multipliers,
        constraint_count,
    )?;

    let mut max_abs_acceleration_residual_per_s2 = 0.0_f64;
    for row in 0..constraint_count {
        let desired = config.desired_constraint_acceleration(ScalarCfeConstraint::new(
            system.position_residuals[row],
            system.velocity_residuals_per_s[row],
            system.unconstrained_accelerations_per_s2[row],
            1.0,
        )?);
        let mut stabilized = system.unconstrained_accelerations_per_s2[row];
        for column in 0..constraint_count {
            stabilized += system.effective_inverse_mass_row_major[row * constraint_count + column]
                * buffers.lagrange_multipliers[column];
        }
        let residual = stabilized - desired;
        if !stabilized.is_finite() || !residual.is_finite() {
            return Err(ContactError::InvalidParameter {
                field: "dense_cfe_solution",
                reason: "must be finite",
            });
        }
        buffers.stabilized_constraint_accelerations_per_s2[row] = stabilized;
        buffers.acceleration_residuals_per_s2[row] = residual;
        max_abs_acceleration_residual_per_s2 =
            max_abs_acceleration_residual_per_s2.max(residual.abs());
    }

    Ok(DenseCfeSolutionSummary {
        constraint_count,
        max_abs_acceleration_residual_per_s2,
    })
}

/// Solves tree-facing Baumgarte CFE loop rows and projects generalized forces.
///
/// The caller still owns tree assembly and integration. This helper closes the
/// deterministic handoff between row-space CFE solves and the tree generalized
/// force vector by writing `G^T * lambda` into `generalized_constraint_forces`.
///
/// # Errors
///
/// Returns [`ContactError::InvalidParameter`] for invalid dimensions,
/// non-positive LDL^T pivots, or non-finite solved forces.
pub fn solve_tree_loop_baumgarte_cfe(
    config: BaumgarteCfeConfig,
    system: TreeCfeLoopSystem<'_>,
    buffers: TreeCfeLoopSolveBuffers<'_>,
) -> Result<TreeCfeLoopSolutionSummary, ContactError> {
    let constraint_count = system.constraint_count();
    let generalized_dof_count = system.generalized_dof_count();
    let buffers = TreeCfeLoopSolveBuffers::new(
        constraint_count,
        generalized_dof_count,
        buffers.matrix_scratch_row_major,
        buffers.rhs_scratch,
        buffers.lagrange_multipliers,
        buffers.stabilized_constraint_accelerations_per_s2,
        buffers.acceleration_residuals_per_s2,
        buffers.generalized_constraint_forces,
    )?;
    let dense_system = DenseCfeSystem::new(
        system.position_residuals,
        system.velocity_residuals_per_s,
        system.unconstrained_constraint_accelerations_per_s2,
        system.effective_inverse_mass_row_major,
    )?;
    let dense_buffers = DenseCfeSolveBuffers::new(
        constraint_count,
        buffers.matrix_scratch_row_major,
        buffers.rhs_scratch,
        buffers.lagrange_multipliers,
        buffers.stabilized_constraint_accelerations_per_s2,
        buffers.acceleration_residuals_per_s2,
    )?;
    let dense_summary = solve_dense_baumgarte_cfe(config, dense_system, dense_buffers)?;

    buffers.generalized_constraint_forces.fill(0.0);
    let mut max_abs_lagrange_multiplier = 0.0_f64;
    for row in 0..constraint_count {
        let lagrange_multiplier = buffers.lagrange_multipliers[row];
        max_abs_lagrange_multiplier = max_abs_lagrange_multiplier.max(lagrange_multiplier.abs());
        for dof in 0..generalized_dof_count {
            buffers.generalized_constraint_forces[dof] += system.constraint_jacobian_row_major
                [row * generalized_dof_count + dof]
                * lagrange_multiplier;
        }
    }

    let mut max_abs_generalized_constraint_force = 0.0_f64;
    for force in buffers.generalized_constraint_forces.iter() {
        if !force.is_finite() {
            return Err(ContactError::InvalidParameter {
                field: "generalized_constraint_forces",
                reason: "entries must be finite",
            });
        }
        max_abs_generalized_constraint_force =
            max_abs_generalized_constraint_force.max(force.abs());
    }

    Ok(TreeCfeLoopSolutionSummary {
        constraint_count: dense_summary.constraint_count,
        generalized_dof_count,
        max_abs_acceleration_residual_per_s2: dense_summary.max_abs_acceleration_residual_per_s2,
        max_abs_lagrange_multiplier,
        max_abs_generalized_constraint_force,
    })
}

fn ldlt_factor_in_place(matrix: &mut [f64], constraint_count: usize) -> Result<(), ContactError> {
    for column in 0..constraint_count {
        for row in column..constraint_count {
            let mut value = matrix[row * constraint_count + column];
            for previous in 0..column {
                value -= matrix[row * constraint_count + previous]
                    * matrix[previous * constraint_count + previous]
                    * matrix[column * constraint_count + previous];
            }
            if row == column {
                if !value.is_finite() || value <= 0.0 {
                    return Err(ContactError::InvalidParameter {
                        field: "effective_inverse_mass_row_major",
                        reason: "must be symmetric positive definite",
                    });
                }
                matrix[column * constraint_count + column] = value;
            } else {
                let pivot = matrix[column * constraint_count + column];
                matrix[row * constraint_count + column] = value / pivot;
            }
        }
    }
    Ok(())
}

fn ldlt_solve_factored(
    matrix: &[f64],
    rhs_and_work: &mut [f64],
    solution: &mut [f64],
    constraint_count: usize,
) -> Result<(), ContactError> {
    for row in 0..constraint_count {
        let mut value = rhs_and_work[row];
        for column in 0..row {
            value -= matrix[row * constraint_count + column] * rhs_and_work[column];
        }
        rhs_and_work[row] = value;
    }
    for row in 0..constraint_count {
        rhs_and_work[row] /= matrix[row * constraint_count + row];
    }
    for row in (0..constraint_count).rev() {
        let mut value = rhs_and_work[row];
        for column in (row + 1)..constraint_count {
            value -= matrix[column * constraint_count + row] * solution[column];
        }
        if !value.is_finite() {
            return Err(ContactError::InvalidParameter {
                field: "lagrange_multipliers",
                reason: "must be finite",
            });
        }
        solution[row] = value;
    }
    Ok(())
}

fn require_all_finite(field: &'static str, values: &[f64]) -> Result<(), ContactError> {
    for value in values {
        if !value.is_finite() {
            return Err(ContactError::InvalidParameter {
                field,
                reason: "entries must be finite",
            });
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;

    #[test]
    fn scalar_baumgarte_cfe_matches_closed_form_multiplier() -> Result<(), ContactError> {
        let config = BaumgarteCfeConfig::new(4.0, 5.0)?;
        let constraint = ScalarCfeConstraint::new(0.01, -0.02, 1.2, 0.5)?;

        let solution = solve_scalar_baumgarte_cfe(config, constraint)?;

        assert_abs_diff_eq!(
            solution.desired_constraint_acceleration_per_s2,
            -0.09,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(solution.lagrange_multiplier, -2.58, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            solution.stabilized_constraint_acceleration_per_s2,
            -0.09,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            solution.acceleration_residual_per_s2,
            0.0,
            epsilon = 1.0e-15
        );
        Ok(())
    }

    #[test]
    fn scalar_baumgarte_cfe_drives_positive_position_residual_back() -> Result<(), ContactError> {
        let config = BaumgarteCfeConfig::new(0.0, 5.0)?;
        let constraint = ScalarCfeConstraint::new(0.1, 0.0, 0.0, 2.0)?;

        let solution = solve_scalar_baumgarte_cfe(config, constraint)?;

        assert!(solution.desired_constraint_acceleration_per_s2 < 0.0);
        assert_abs_diff_eq!(
            solution.desired_constraint_acceleration_per_s2,
            -2.5,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(solution.lagrange_multiplier, -1.25, epsilon = 1.0e-15);
        Ok(())
    }

    #[test]
    fn scalar_baumgarte_cfe_rejects_invalid_inputs() {
        assert!(BaumgarteCfeConfig::new(-1.0, 1.0).is_err());
        assert!(ScalarCfeConstraint::new(0.0, 0.0, 0.0, 0.0).is_err());
        assert!(ScalarCfeConstraint::new(f64::NAN, 0.0, 0.0, 1.0).is_err());
    }

    #[test]
    fn dense_baumgarte_cfe_matches_two_row_closed_form() -> Result<(), ContactError> {
        let system = DenseCfeSystem::new(
            &[0.1, -0.04],
            &[0.0, 0.02],
            &[0.3, -0.2],
            &[2.0, 0.5, 0.5, 1.5],
        )?;
        let mut matrix_scratch = [0.0; 4];
        let mut rhs_scratch = [0.0; 2];
        let mut lagrange_multipliers = [0.0; 2];
        let mut stabilized_accelerations = [0.0; 2];
        let mut residuals = [0.0; 2];
        let buffers = DenseCfeSolveBuffers::new(
            system.constraint_count(),
            &mut matrix_scratch,
            &mut rhs_scratch,
            &mut lagrange_multipliers,
            &mut stabilized_accelerations,
            &mut residuals,
        )?;

        let summary =
            solve_dense_baumgarte_cfe(BaumgarteCfeConfig::new(2.0, 3.0)?, system, buffers)?;

        assert_eq!(summary.constraint_count, 2);
        assert_abs_diff_eq!(lagrange_multipliers[0], -2.04 / 2.75, epsilon = 1.0e-15);
        assert_abs_diff_eq!(lagrange_multipliers[1], 1.56 / 2.75, epsilon = 1.0e-15);
        assert_abs_diff_eq!(stabilized_accelerations[0], -0.9, epsilon = 1.0e-15);
        assert_abs_diff_eq!(stabilized_accelerations[1], 0.28, epsilon = 1.0e-15);
        assert_abs_diff_eq!(residuals[0], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(residuals[1], 0.0, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            summary.max_abs_acceleration_residual_per_s2,
            0.0,
            epsilon = 1.0e-15
        );
        Ok(())
    }

    #[test]
    fn dense_baumgarte_cfe_matches_scalar_row() -> Result<(), ContactError> {
        let config = BaumgarteCfeConfig::new(4.0, 5.0)?;
        let constraint = ScalarCfeConstraint::new(0.01, -0.02, 1.2, 0.5)?;
        let scalar = solve_scalar_baumgarte_cfe(config, constraint)?;
        let system = DenseCfeSystem::new(&[0.01], &[-0.02], &[1.2], &[0.5])?;
        let mut matrix_scratch = [0.0; 1];
        let mut rhs_scratch = [0.0; 1];
        let mut lagrange_multipliers = [0.0; 1];
        let mut stabilized_accelerations = [0.0; 1];
        let mut residuals = [0.0; 1];
        let buffers = DenseCfeSolveBuffers::new(
            system.constraint_count(),
            &mut matrix_scratch,
            &mut rhs_scratch,
            &mut lagrange_multipliers,
            &mut stabilized_accelerations,
            &mut residuals,
        )?;

        solve_dense_baumgarte_cfe(config, system, buffers)?;

        assert_abs_diff_eq!(
            lagrange_multipliers[0],
            scalar.lagrange_multiplier,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            stabilized_accelerations[0],
            scalar.stabilized_constraint_acceleration_per_s2,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            residuals[0],
            scalar.acceleration_residual_per_s2,
            epsilon = 1.0e-15
        );
        Ok(())
    }

    #[test]
    fn tree_loop_baumgarte_cfe_projects_multipliers_to_generalized_forces()
    -> Result<(), ContactError> {
        let system = TreeCfeLoopSystem::new(
            3,
            &[0.1, -0.04],
            &[0.0, 0.02],
            &[0.3, -0.2],
            &[2.0, 0.5, 0.5, 1.5],
            &[1.0, 0.0, 2.0, 0.0, -1.0, 1.0],
        )?;
        let mut matrix_scratch = [0.0; 4];
        let mut rhs_scratch = [0.0; 2];
        let mut lagrange_multipliers = [0.0; 2];
        let mut stabilized_accelerations = [0.0; 2];
        let mut residuals = [0.0; 2];
        let mut generalized_forces = [9.0; 3];
        let buffers = TreeCfeLoopSolveBuffers::new(
            system.constraint_count(),
            system.generalized_dof_count(),
            &mut matrix_scratch,
            &mut rhs_scratch,
            &mut lagrange_multipliers,
            &mut stabilized_accelerations,
            &mut residuals,
            &mut generalized_forces,
        )?;

        let summary =
            solve_tree_loop_baumgarte_cfe(BaumgarteCfeConfig::new(2.0, 3.0)?, system, buffers)?;

        assert_eq!(summary.constraint_count, 2);
        assert_eq!(summary.generalized_dof_count, 3);
        assert_abs_diff_eq!(lagrange_multipliers[0], -2.04 / 2.75, epsilon = 1.0e-15);
        assert_abs_diff_eq!(lagrange_multipliers[1], 1.56 / 2.75, epsilon = 1.0e-15);
        let expected_third_force = 2.0_f64 * (-2.04 / 2.75) + 1.56 / 2.75;
        assert_abs_diff_eq!(generalized_forces[0], -2.04 / 2.75, epsilon = 1.0e-15);
        assert_abs_diff_eq!(generalized_forces[1], -1.56 / 2.75, epsilon = 1.0e-15);
        assert_abs_diff_eq!(
            generalized_forces[2],
            expected_third_force,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            summary.max_abs_acceleration_residual_per_s2,
            0.0,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            summary.max_abs_lagrange_multiplier,
            2.04 / 2.75,
            epsilon = 1.0e-15
        );
        assert_abs_diff_eq!(
            summary.max_abs_generalized_constraint_force,
            expected_third_force.abs(),
            epsilon = 1.0e-15
        );
        Ok(())
    }

    #[test]
    fn tree_loop_baumgarte_cfe_rejects_bad_dimensions() -> Result<(), ContactError> {
        assert!(TreeCfeLoopSystem::new(0, &[0.0], &[0.0], &[0.0], &[1.0], &[1.0]).is_err());
        assert!(TreeCfeLoopSystem::new(2, &[0.0], &[0.0], &[0.0], &[1.0], &[1.0]).is_err());
        assert!(TreeCfeLoopSystem::new(1, &[0.0], &[0.0], &[0.0], &[1.0], &[f64::NAN]).is_err());

        let mut matrix_scratch = [0.0; 1];
        let mut rhs_scratch = [0.0; 1];
        let mut lagrange_multipliers = [0.0; 1];
        let mut stabilized_accelerations = [0.0; 1];
        let mut residuals = [0.0; 1];
        let mut generalized_forces = [0.0; 2];
        assert!(
            TreeCfeLoopSolveBuffers::new(
                1,
                3,
                &mut matrix_scratch,
                &mut rhs_scratch,
                &mut lagrange_multipliers,
                &mut stabilized_accelerations,
                &mut residuals,
                &mut generalized_forces,
            )
            .is_err()
        );
        Ok(())
    }

    #[test]
    fn dense_baumgarte_cfe_rejects_bad_dimensions_and_indefinite_matrix() -> Result<(), ContactError>
    {
        assert!(DenseCfeSystem::new(&[], &[], &[], &[]).is_err());
        assert!(DenseCfeSystem::new(&[0.0], &[], &[0.0], &[1.0]).is_err());

        let system =
            DenseCfeSystem::new(&[0.0, 0.0], &[0.0, 0.0], &[0.0, 0.0], &[1.0, 2.0, 2.0, 1.0])?;
        let mut matrix_scratch = [0.0; 4];
        let mut rhs_scratch = [0.0; 2];
        let mut lagrange_multipliers = [0.0; 2];
        let mut stabilized_accelerations = [0.0; 2];
        let mut residuals = [0.0; 2];
        let buffers = DenseCfeSolveBuffers::new(
            system.constraint_count(),
            &mut matrix_scratch,
            &mut rhs_scratch,
            &mut lagrange_multipliers,
            &mut stabilized_accelerations,
            &mut residuals,
        )?;

        assert!(
            solve_dense_baumgarte_cfe(BaumgarteCfeConfig::new(1.0, 1.0)?, system, buffers).is_err()
        );
        Ok(())
    }
}
