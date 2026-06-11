//! Multiple-shooting continuity defects for T1 trajectory optimization.
//!
//! The first implementation evaluates fixed-duration two-body segment
//! continuity and the block-bidiagonal Jacobian built from each segment STM.
//! It is a solver substrate, not a new target surface.

use alloc::vec;
use alloc::vec::Vec;

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
}
