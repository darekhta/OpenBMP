//! Multiple-shooting continuity defects for T1 trajectory optimization.
//!
//! The first implementation evaluates fixed-duration two-body segment
//! continuity and the block-bidiagonal Jacobian built from each segment STM.
//! It is a solver substrate, not a new target surface.

use alloc::vec;
use alloc::vec::Vec;

use crate::corrector::solve_linear_system;
use crate::iload::TrajoptError;
use crate::stm::{
    TwoBodyCartesianState, TwoBodyVariationalPropagation, propagate_two_body_variational,
};

/// One node in a multiple-shooting mesh.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultipleShootingNode {
    /// Cartesian state at the mesh node.
    pub state: TwoBodyCartesianState,
}

impl MultipleShootingNode {
    /// Construct a node from a Cartesian state.
    #[must_use]
    pub const fn new(state: TwoBodyCartesianState) -> Self {
        Self { state }
    }
}

/// Continuity-defect report for a fixed-duration multiple-shooting mesh.
#[derive(Clone, Debug, PartialEq)]
pub struct MultipleShootingContinuityReport {
    /// Stacked continuity defects, six rows per segment:
    /// `phi_i(x_i) - x_{i+1}`.
    pub defects: Vec<f64>,
    /// Row-major Jacobian of `defects` with respect to all node states.
    ///
    /// Columns are grouped as `[x_0, x_1, ..., x_M]`, six variables per node.
    /// Each continuity block contains `[STM_i, -I]`.
    pub jacobian: Vec<f64>,
    /// Euclidean norm of [`Self::defects`].
    pub defect_norm: f64,
    /// Number of propagated segments.
    pub segment_count: usize,
    /// Number of free node-state variables.
    pub free_state_count: usize,
}

/// Configuration for fixed-endpoint multiple-shooting continuity correction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MultipleShootingCorrector {
    /// Maximum Gauss-Newton iterations before reporting non-convergence.
    pub max_iterations: usize,
    /// Convergence threshold on the stacked continuity-defect norm.
    pub defect_tolerance: f64,
    /// Levenberg-Marquardt damping added to the normal-equation diagonal.
    pub levenberg_marquardt_damping: f64,
    /// Cap on the Euclidean norm of a single interior-node correction step.
    pub max_step_norm: f64,
}

impl Default for MultipleShootingCorrector {
    fn default() -> Self {
        Self {
            max_iterations: 20,
            defect_tolerance: 1.0e-6,
            levenberg_marquardt_damping: 1.0e-9,
            max_step_norm: f64::INFINITY,
        }
    }
}

/// Result of a fixed-endpoint multiple-shooting continuity correction.
#[derive(Clone, Debug, PartialEq)]
pub struct MultipleShootingCorrection {
    /// Corrected node sequence. The first and last nodes are held fixed.
    pub nodes: Vec<MultipleShootingNode>,
    /// Final continuity report for [`Self::nodes`].
    pub report: MultipleShootingContinuityReport,
    /// Number of Gauss-Newton iterations taken.
    pub iterations: usize,
    /// Whether the continuity-defect norm reached the configured tolerance.
    pub converged: bool,
}

impl MultipleShootingCorrector {
    /// Correct interior nodes while holding the first and last node fixed.
    ///
    /// This is the first solve loop for WP-07.1: it improves an M-segment
    /// two-body shooting mesh by using the STM continuity Jacobian. It does not
    /// introduce controls, path constraints, terminal targeting, or any new
    /// surface-coordinate vocabulary.
    ///
    /// # Errors
    ///
    /// Returns [`TrajoptError`] when the configuration or mesh is invalid,
    /// segment propagation fails, or the damped normal equations are singular.
    pub fn solve_two_body_fixed_endpoint_continuity(
        &self,
        initial_nodes: &[MultipleShootingNode],
        segment_durations_s: &[f64],
        step_s: f64,
        mu_m3_s2: f64,
    ) -> Result<MultipleShootingCorrection, TrajoptError> {
        self.validate_config()?;
        if initial_nodes.len() < 3 {
            return Err(TrajoptError::InvalidPayload {
                reason: "fixed-endpoint multiple shooting requires at least one interior node",
            });
        }
        let mut nodes = initial_nodes.to_vec();
        let mut report =
            evaluate_two_body_multiple_shooting(&nodes, segment_durations_s, step_s, mu_m3_s2)?;
        let mut iterations = 0;

        while report.defect_norm > self.defect_tolerance {
            if iterations >= self.max_iterations {
                return Ok(MultipleShootingCorrection {
                    nodes,
                    report,
                    iterations,
                    converged: false,
                });
            }

            let free_count = interior_free_state_count(&report)?;
            let jacobian = interior_node_jacobian(&report, free_count)?;
            let step = gauss_newton_step(
                &jacobian,
                &report.defects,
                report.defects.len(),
                free_count,
                self.levenberg_marquardt_damping,
            )?;
            apply_interior_step(&mut nodes, &step, self.max_step_norm)?;

            iterations += 1;
            report =
                evaluate_two_body_multiple_shooting(&nodes, segment_durations_s, step_s, mu_m3_s2)?;
        }

        Ok(MultipleShootingCorrection {
            nodes,
            report,
            iterations,
            converged: true,
        })
    }

    fn validate_config(&self) -> Result<(), TrajoptError> {
        if !self.defect_tolerance.is_finite() || self.defect_tolerance < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "multiple shooting defect tolerance must be finite and non-negative",
            });
        }
        if !self.levenberg_marquardt_damping.is_finite() || self.levenberg_marquardt_damping < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "multiple shooting damping must be finite and non-negative",
            });
        }
        if self.max_step_norm.is_nan() || self.max_step_norm <= 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "multiple shooting maximum step norm must be positive",
            });
        }
        Ok(())
    }
}

/// Generate a dynamically consistent two-body multiple-shooting node seed.
///
/// # Errors
///
/// Returns [`TrajoptError`] when any segment propagation fails.
pub fn seed_two_body_multiple_shooting_nodes(
    initial_state: TwoBodyCartesianState,
    segment_durations_s: &[f64],
    step_s: f64,
    mu_m3_s2: f64,
) -> Result<Vec<MultipleShootingNode>, TrajoptError> {
    let mut nodes = Vec::with_capacity(segment_durations_s.len() + 1);
    nodes.push(MultipleShootingNode::new(initial_state));
    let mut current = initial_state;
    for &duration_s in segment_durations_s {
        current =
            propagate_two_body_variational(current, duration_s, step_s, mu_m3_s2)?.terminal_state;
        nodes.push(MultipleShootingNode::new(current));
    }
    Ok(nodes)
}

/// Evaluate two-body multiple-shooting continuity and STM Jacobian.
///
/// # Errors
///
/// Returns [`TrajoptError`] when the mesh shape is invalid or any segment
/// propagation fails.
pub fn evaluate_two_body_multiple_shooting(
    nodes: &[MultipleShootingNode],
    segment_durations_s: &[f64],
    step_s: f64,
    mu_m3_s2: f64,
) -> Result<MultipleShootingContinuityReport, TrajoptError> {
    validate_mesh(nodes, segment_durations_s)?;
    let segment_count = segment_durations_s.len();
    let free_state_count = nodes.len() * 6;
    let defect_count = segment_count * 6;
    let mut defects = vec![0.0_f64; defect_count];
    let mut jacobian = vec![0.0_f64; defect_count * free_state_count];

    for (segment_index, &duration_s) in segment_durations_s.iter().enumerate() {
        let propagation = propagate_two_body_variational(
            nodes[segment_index].state,
            duration_s,
            step_s,
            mu_m3_s2,
        )?;
        insert_segment_defect(
            segment_index,
            &propagation,
            nodes[segment_index + 1].state,
            free_state_count,
            &mut defects,
            &mut jacobian,
        );
    }
    let defect_norm = defects
        .iter()
        .map(|value| value * value)
        .sum::<f64>()
        .sqrt();
    Ok(MultipleShootingContinuityReport {
        defects,
        jacobian,
        defect_norm,
        segment_count,
        free_state_count,
    })
}

fn validate_mesh(
    nodes: &[MultipleShootingNode],
    segment_durations_s: &[f64],
) -> Result<(), TrajoptError> {
    if segment_durations_s.is_empty() {
        return Err(TrajoptError::InvalidPayload {
            reason: "multiple shooting requires at least one segment",
        });
    }
    if nodes.len() != segment_durations_s.len() + 1 {
        return Err(TrajoptError::InvalidPayload {
            reason: "multiple shooting node count must equal segment count plus one",
        });
    }
    for node in nodes {
        TwoBodyCartesianState::from_array(node.state.to_array())?;
    }
    for &duration_s in segment_durations_s {
        if !duration_s.is_finite() || duration_s < 0.0 {
            return Err(TrajoptError::InvalidPayload {
                reason: "multiple shooting segment duration must be finite and non-negative",
            });
        }
    }
    Ok(())
}

fn insert_segment_defect(
    segment_index: usize,
    propagation: &TwoBodyVariationalPropagation,
    next_node: TwoBodyCartesianState,
    free_state_count: usize,
    defects: &mut [f64],
    jacobian: &mut [f64],
) {
    let row_offset = segment_index * 6;
    let current_col_offset = segment_index * 6;
    let next_col_offset = (segment_index + 1) * 6;
    let terminal = propagation.terminal_state.to_array();
    let next = next_node.to_array();
    for row in 0..6 {
        defects[row_offset + row] = terminal[row] - next[row];
        for column in 0..6 {
            jacobian[(row_offset + row) * free_state_count + current_col_offset + column] =
                propagation.stm.get(row, column);
        }
        jacobian[(row_offset + row) * free_state_count + next_col_offset + row] = -1.0;
    }
}

fn interior_free_state_count(
    report: &MultipleShootingContinuityReport,
) -> Result<usize, TrajoptError> {
    let node_count = report.free_state_count / 6;
    if node_count < 3 || report.free_state_count != node_count * 6 {
        return Err(TrajoptError::InvalidPayload {
            reason: "multiple shooting report must contain endpoint and interior node states",
        });
    }
    Ok((node_count - 2) * 6)
}

fn interior_node_jacobian(
    report: &MultipleShootingContinuityReport,
    free_count: usize,
) -> Result<Vec<f64>, TrajoptError> {
    let node_count = report.free_state_count / 6;
    let expected_free_count = (node_count - 2) * 6;
    if free_count != expected_free_count {
        return Err(TrajoptError::InvalidPayload {
            reason: "multiple shooting interior free-state count is inconsistent",
        });
    }
    let residual_count = report.defects.len();
    let mut jacobian = vec![0.0_f64; residual_count * free_count];
    for row in 0..residual_count {
        for interior_node in 1..(node_count - 1) {
            for component in 0..6 {
                let source_column = interior_node * 6 + component;
                let target_column = (interior_node - 1) * 6 + component;
                jacobian[row * free_count + target_column] =
                    report.jacobian[row * report.free_state_count + source_column];
            }
        }
    }
    Ok(jacobian)
}

fn gauss_newton_step(
    jacobian: &[f64],
    residual: &[f64],
    residual_count: usize,
    free_count: usize,
    damping: f64,
) -> Result<Vec<f64>, TrajoptError> {
    let mut normal = vec![0.0_f64; free_count * free_count];
    let mut rhs = vec![0.0_f64; free_count];
    for column in 0..free_count {
        for other in 0..free_count {
            let mut accumulator = 0.0_f64;
            for row in 0..residual_count {
                accumulator +=
                    jacobian[row * free_count + column] * jacobian[row * free_count + other];
            }
            normal[column * free_count + other] = accumulator;
        }
        normal[column * free_count + column] += damping;
        let mut gradient = 0.0_f64;
        for row in 0..residual_count {
            gradient += jacobian[row * free_count + column] * residual[row];
        }
        rhs[column] = -gradient;
    }
    solve_linear_system(normal, rhs, free_count)
}

fn apply_interior_step(
    nodes: &mut [MultipleShootingNode],
    step: &[f64],
    max_step_norm: f64,
) -> Result<(), TrajoptError> {
    let mut norm_squared = 0.0_f64;
    for &value in step {
        norm_squared += value * value;
    }
    let norm = norm_squared.sqrt();
    let scale = if max_step_norm.is_finite() && norm > max_step_norm && norm > 0.0 {
        max_step_norm / norm
    } else {
        1.0
    };
    let interior_count = nodes.len().saturating_sub(2);
    if step.len() != interior_count * 6 {
        return Err(TrajoptError::InvalidPayload {
            reason: "multiple shooting correction step dimension is inconsistent",
        });
    }
    let last_node_index = nodes.len() - 1;
    for (node_index, node) in nodes[1..last_node_index].iter_mut().enumerate() {
        let mut state = node.state.to_array();
        for component in 0..6 {
            state[component] += scale * step[node_index * 6 + component];
        }
        node.state = TwoBodyCartesianState::from_array(state)?;
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use openbmp_physics::WGS84_MU_M3_S2;

    #[test]
    fn dynamically_seeded_nodes_have_zero_continuity_defect() -> Result<(), TrajoptError> {
        let initial =
            TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let durations = [40.0, 40.0, 40.0];
        let nodes =
            seed_two_body_multiple_shooting_nodes(initial, &durations, 10.0, WGS84_MU_M3_S2)?;

        let report = evaluate_two_body_multiple_shooting(&nodes, &durations, 10.0, WGS84_MU_M3_S2)?;

        assert_eq!(report.segment_count, 3);
        assert_eq!(report.free_state_count, 24);
        assert_eq!(report.defects.len(), 18);
        assert_eq!(report.jacobian.len(), 18 * 24);
        assert!(report.defect_norm < 1.0e-8, "{report:?}");
        Ok(())
    }

    #[test]
    fn continuity_defect_detects_perturbed_interior_node() -> Result<(), TrajoptError> {
        let initial =
            TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let durations = [60.0, 60.0];
        let mut nodes =
            seed_two_body_multiple_shooting_nodes(initial, &durations, 10.0, WGS84_MU_M3_S2)?;
        nodes[1].state.position_eci_m[0] += 10.0;

        let report = evaluate_two_body_multiple_shooting(&nodes, &durations, 10.0, WGS84_MU_M3_S2)?;

        assert!(report.defect_norm > 1.0);
        Ok(())
    }

    #[test]
    fn continuity_jacobian_has_block_bidiagonal_shape() -> Result<(), TrajoptError> {
        let initial =
            TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let durations = [120.0];
        let nodes =
            seed_two_body_multiple_shooting_nodes(initial, &durations, 10.0, WGS84_MU_M3_S2)?;

        let report = evaluate_two_body_multiple_shooting(&nodes, &durations, 10.0, WGS84_MU_M3_S2)?;

        for row in 0..6 {
            let identity_entry = report.jacobian[row * report.free_state_count + 6 + row];
            assert!((identity_entry + 1.0).abs() < f64::EPSILON);
        }
        let nonzero_outside_blocks = report
            .jacobian
            .iter()
            .enumerate()
            .filter(|(index, value)| {
                let column = index % report.free_state_count;
                column >= 12 && value.abs() > 0.0
            })
            .count();
        assert_eq!(nonzero_outside_blocks, 0);
        Ok(())
    }

    #[test]
    fn fixed_endpoint_corrector_restores_perturbed_interior_nodes() -> Result<(), TrajoptError> {
        let initial =
            TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let durations = [40.0, 40.0, 40.0];
        let truth =
            seed_two_body_multiple_shooting_nodes(initial, &durations, 10.0, WGS84_MU_M3_S2)?;
        let mut perturbed = truth.clone();
        perturbed[1].state.position_eci_m[0] += 50.0;
        perturbed[1].state.velocity_eci_m_s[1] -= 0.05;
        perturbed[2].state.position_eci_m[1] -= 25.0;
        perturbed[2].state.velocity_eci_m_s[0] += 0.02;

        let initial_report =
            evaluate_two_body_multiple_shooting(&perturbed, &durations, 10.0, WGS84_MU_M3_S2)?;
        let corrector = MultipleShootingCorrector {
            defect_tolerance: 1.0e-8,
            max_step_norm: 100.0,
            ..MultipleShootingCorrector::default()
        };

        let correction = corrector.solve_two_body_fixed_endpoint_continuity(
            &perturbed,
            &durations,
            10.0,
            WGS84_MU_M3_S2,
        )?;

        assert!(initial_report.defect_norm > 1.0);
        assert!(correction.converged, "{correction:?}");
        assert!(correction.iterations > 0);
        assert!(correction.report.defect_norm < corrector.defect_tolerance);
        assert_eq!(correction.nodes[0], truth[0]);
        assert_eq!(
            correction.nodes[correction.nodes.len() - 1],
            truth[truth.len() - 1]
        );
        Ok(())
    }

    #[test]
    fn fixed_endpoint_corrector_reports_non_convergence_for_inconsistent_endpoint()
    -> Result<(), TrajoptError> {
        let initial =
            TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let durations = [60.0, 60.0];
        let mut nodes =
            seed_two_body_multiple_shooting_nodes(initial, &durations, 10.0, WGS84_MU_M3_S2)?;
        nodes[2].state.position_eci_m[0] += 10.0;
        let corrector = MultipleShootingCorrector {
            max_iterations: 3,
            defect_tolerance: 1.0e-10,
            max_step_norm: 100.0,
            ..MultipleShootingCorrector::default()
        };

        let correction = corrector.solve_two_body_fixed_endpoint_continuity(
            &nodes,
            &durations,
            10.0,
            WGS84_MU_M3_S2,
        )?;

        assert!(!correction.converged);
        assert_eq!(correction.iterations, corrector.max_iterations);
        assert_eq!(correction.nodes[0], nodes[0]);
        assert_eq!(
            correction.nodes[correction.nodes.len() - 1],
            nodes[nodes.len() - 1]
        );
        Ok(())
    }

    #[test]
    fn fixed_endpoint_corrector_requires_an_interior_node() -> Result<(), TrajoptError> {
        let initial =
            TwoBodyCartesianState::new([6_778_000.0, 0.0, 0.0], [0.0, 7_668.635_675, 0.0])?;
        let durations = [60.0];
        let nodes =
            seed_two_body_multiple_shooting_nodes(initial, &durations, 10.0, WGS84_MU_M3_S2)?;

        let result = MultipleShootingCorrector::default().solve_two_body_fixed_endpoint_continuity(
            &nodes,
            &durations,
            10.0,
            WGS84_MU_M3_S2,
        );

        assert!(result.is_err());
        Ok(())
    }
}
