//! Coupled gain scheduling and bumpless transfer helpers.
//!
//! The module is intentionally small and table-driven: offline tools
//! synthesize LQR/LQG gains at scalar flight-condition nodes, the FC
//! validates the I-load through `tables.rs`, and the runtime only
//! interpolates gains plus resets declared integral states at schedule
//! boundaries.

#[cfg(not(feature = "std"))]
use alloc::vec;
use std::string::{String, ToString};
use std::vec::Vec;

#[cfg(not(feature = "std"))]
use num_traits::Float;
use thiserror::Error;

use crate::tables::{GainInterp, GainScheduleTable, RobustControllerTable, Table};

/// Errors raised by gain-schedule interpolation and transition logic.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum ScheduleError {
    /// A table failed its structural validation.
    #[error("gain schedule table is invalid: {reason}")]
    InvalidTable {
        /// Human-readable validation failure.
        reason: String,
    },
    /// Query coordinate was not finite.
    #[error("gain schedule query must be finite; got {value}")]
    NonFiniteQuery {
        /// Offending schedule value.
        value: f64,
    },
    /// A runtime vector contained a non-finite value.
    #[error("gain schedule vector {label} must contain only finite values")]
    NonFiniteVector {
        /// Vector label.
        label: &'static str,
    },
    /// Matrix/vector length did not match the declared shape.
    #[error("gain schedule matrix {label} length mismatch: expected {expected}, got {actual}")]
    MatrixLengthMismatch {
        /// Matrix label.
        label: &'static str,
        /// Expected row-major entry count.
        expected: usize,
        /// Actual entry count.
        actual: usize,
    },
    /// A declared dimension is outside the bounded FC envelope.
    #[error("gain schedule dimension {label} must be in 1..={max}; got {value}")]
    InvalidDimension {
        /// Dimension label.
        label: &'static str,
        /// Offending value.
        value: usize,
        /// Maximum accepted value.
        max: usize,
    },
    /// Linear solve failed during bumpless transfer.
    #[error("gain schedule linear solve failed for {label}")]
    LinearSolveFailed {
        /// Solve label.
        label: &'static str,
    },
}

/// Interpolated schedule lookup.
#[derive(Clone, Debug, PartialEq)]
pub struct ScheduledGainSet {
    /// Lower node index used for interpolation.
    pub lower_index: usize,
    /// Upper node index used for interpolation.
    pub upper_index: usize,
    /// Interpolation fraction after applying the selected interpolation law.
    pub lambda: f64,
    /// Row-major controller gain `K`, shaped `controls × states`.
    pub gain_row_major: Vec<f64>,
    /// Optional row-major observer gain `L`, shaped `states × measurements`.
    pub observer_gain_row_major: Option<Vec<f64>>,
}

/// Interpolated imported robust-controller state-space matrices.
#[derive(Clone, Debug, PartialEq)]
pub struct RobustControllerMatrices {
    /// Lower node index used for interpolation.
    pub lower_index: usize,
    /// Upper node index used for interpolation.
    pub upper_index: usize,
    /// Interpolation fraction after applying the selected interpolation law.
    pub lambda: f64,
    /// Row-major state matrix `A_c`, shaped `states × states`.
    pub a_row_major: Vec<f64>,
    /// Row-major input matrix `B_c`, shaped `states × measurements`.
    pub b_row_major: Vec<f64>,
    /// Row-major output matrix `C_c`, shaped `controls × states`.
    pub c_row_major: Vec<f64>,
    /// Row-major feedthrough matrix `D_c`, shaped `controls × measurements`.
    pub d_row_major: Vec<f64>,
}

/// One deterministic imported robust-controller step.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportedRobustControllerStep {
    /// Interpolated controller matrices used for this step.
    pub matrices: RobustControllerMatrices,
    /// Command `u[k] = C_c x_c[k] + D_c y[k]`.
    pub command: Vec<f64>,
    /// Controller state after the step, `x_c[k+1]`.
    pub next_state: Vec<f64>,
}

/// Result from a bumpless boundary reset.
#[derive(Clone, Debug, PartialEq)]
pub struct BumplessTransferReport {
    /// Command immediately before the transfer, `u = -K_old x_old`.
    pub command_before: Vec<f64>,
    /// Command immediately after resetting integral states and applying `K_new`.
    pub command_after: Vec<f64>,
    /// Maximum absolute command jump.
    pub max_command_jump: f64,
}

/// Classical SISO loop margins from a discrete state-space open loop.
#[derive(Clone, Debug, PartialEq)]
pub struct SisoLoopMargins {
    /// Gain margin in dB. `f64::INFINITY` means no `-180 deg` phase
    /// crossing was found on `[0, pi]`.
    pub gain_margin_db: f64,
    /// Phase margin in degrees. `f64::INFINITY` means no unity-gain
    /// crossing was found on `[0, pi]`.
    pub phase_margin_deg: f64,
}

/// Minimum SISO margins over a schedule sweep.
#[derive(Clone, Debug, PartialEq)]
pub struct SisoMarginSweep {
    /// Minimum gain margin in dB.
    pub min_gain_margin_db: f64,
    /// Minimum phase margin in degrees.
    pub min_phase_margin_deg: f64,
}

/// Borrowed SISO discrete state-space model.
///
/// The response is `G(z) = C (zI - A)^-1 B + D`, with `A` row-major
/// `states × states`, `B` length `states`, `C` length `states`, and scalar `D`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SisoStateSpaceRef<'a> {
    /// State dimension.
    pub states: usize,
    /// Row-major state matrix.
    pub a_row_major: &'a [f64],
    /// Input matrix column.
    pub b_row_major: &'a [f64],
    /// Output matrix row.
    pub c_row_major: &'a [f64],
    /// Feedthrough scalar.
    pub d: f64,
}

/// Scheduled coupled controller state.
#[derive(Clone, Debug, PartialEq)]
pub struct ScheduledController {
    table: GainScheduleTable,
    state: Vec<f64>,
    integral_indices: Vec<usize>,
}

/// Runtime state for an imported robust state-space controller.
#[derive(Clone, Debug, PartialEq)]
pub struct ImportedRobustController {
    table: RobustControllerTable,
    state: Vec<f64>,
}

impl ImportedRobustController {
    /// Builds an imported controller from a validated robust-controller table.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError`] if the table is invalid, the initial state
    /// length does not match `table.states`, or the state contains a non-finite
    /// value.
    pub fn new(
        table: RobustControllerTable,
        initial_state: Vec<f64>,
    ) -> Result<Self, ScheduleError> {
        table
            .validate()
            .map_err(|reason| ScheduleError::InvalidTable { reason })?;
        if initial_state.len() != table.states {
            return Err(ScheduleError::MatrixLengthMismatch {
                label: "robust_controller_state",
                expected: table.states,
                actual: initial_state.len(),
            });
        }
        if initial_state.iter().any(|value| !value.is_finite()) {
            return Err(ScheduleError::NonFiniteVector {
                label: "robust_controller_state",
            });
        }
        Ok(Self {
            table,
            state: initial_state,
        })
    }

    /// Returns the controller state.
    #[must_use]
    pub fn state(&self) -> &[f64] {
        &self.state
    }

    /// Interpolates robust-controller matrices at schedule value `s`.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError`] if the table is invalid or `s` is non-finite.
    pub fn matrices_at(&self, s: f64) -> Result<RobustControllerMatrices, ScheduleError> {
        interpolate_robust_controller_table(&self.table, s)
    }

    /// Advances one imported state-space controller step.
    ///
    /// The command is computed from the pre-step state and supplied measurement:
    /// `u[k] = C_c x_c[k] + D_c y[k]`; the internal state is then replaced with
    /// `A_c x_c[k] + B_c y[k]`.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError`] when interpolation fails or measurement shape
    /// validation fails.
    pub fn step(
        &mut self,
        s: f64,
        measurement: &[f64],
    ) -> Result<ImportedRobustControllerStep, ScheduleError> {
        if measurement.len() != self.table.measurements {
            return Err(ScheduleError::MatrixLengthMismatch {
                label: "robust_controller_measurement",
                expected: self.table.measurements,
                actual: measurement.len(),
            });
        }
        if measurement.iter().any(|value| !value.is_finite()) {
            return Err(ScheduleError::NonFiniteVector {
                label: "robust_controller_measurement",
            });
        }
        let matrices = self.matrices_at(s)?;
        let command = robust_controller_command(
            self.table.states,
            self.table.measurements,
            self.table.controls,
            &matrices.c_row_major,
            &matrices.d_row_major,
            &self.state,
            measurement,
        )?;
        let next_state = robust_controller_next_state(
            self.table.states,
            self.table.measurements,
            &matrices.a_row_major,
            &matrices.b_row_major,
            &self.state,
            measurement,
        )?;
        self.state.clone_from(&next_state);
        Ok(ImportedRobustControllerStep {
            matrices,
            command,
            next_state,
        })
    }
}

impl ScheduledController {
    /// Builds a scheduled controller with the supplied initial state
    /// and the state indices that are allowed to move during bumpless
    /// transfer.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError`] if the schedule table is invalid, the
    /// state length does not match `table.states`, or an integral index
    /// is out of range.
    pub fn new(
        table: GainScheduleTable,
        initial_state: Vec<f64>,
        integral_indices: Vec<usize>,
    ) -> Result<Self, ScheduleError> {
        table
            .validate()
            .map_err(|reason| ScheduleError::InvalidTable { reason })?;
        if initial_state.len() != table.states {
            return Err(ScheduleError::MatrixLengthMismatch {
                label: "state",
                expected: table.states,
                actual: initial_state.len(),
            });
        }
        if initial_state.iter().any(|value| !value.is_finite()) {
            return Err(ScheduleError::LinearSolveFailed {
                label: "non-finite state",
            });
        }
        for index in &integral_indices {
            if *index >= table.states {
                return Err(ScheduleError::InvalidDimension {
                    label: "integral_index",
                    value: *index,
                    max: table.states.saturating_sub(1),
                });
            }
        }
        Ok(Self {
            table,
            state: initial_state,
            integral_indices,
        })
    }

    /// Returns the controller state.
    #[must_use]
    pub fn state(&self) -> &[f64] {
        &self.state
    }

    /// Returns an interpolated gain set at schedule value `s`.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError::NonFiniteQuery`] when `s` is NaN or infinite.
    pub fn gains_at(&self, s: f64) -> Result<ScheduledGainSet, ScheduleError> {
        interpolate_gain_schedule(&self.table, s)
    }

    /// Computes `u = -K(s)x` for the current state.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError`] if the schedule lookup fails.
    pub fn command_at(&self, s: f64) -> Result<Vec<f64>, ScheduleError> {
        let gains = self.gains_at(s)?;
        Ok(command_from_gain(
            self.table.states,
            self.table.controls,
            &gains.gain_row_major,
            &self.state,
        ))
    }

    /// Resets declared integral states so switching from `old_gain`
    /// to `new_gain` preserves the current command as closely as the
    /// integral subspace allows.
    ///
    /// # Errors
    ///
    /// Returns [`ScheduleError`] on dimension mismatch or singular
    /// integral-state influence.
    pub fn on_schedule_boundary(
        &mut self,
        old_gain_row_major: &[f64],
        new_gain_row_major: &[f64],
    ) -> Result<BumplessTransferReport, ScheduleError> {
        let before = command_from_gain(
            self.table.states,
            self.table.controls,
            old_gain_row_major,
            &self.state,
        );
        let adjusted = reset_integral_states_for_bumpless_transfer(
            self.table.states,
            self.table.controls,
            old_gain_row_major,
            new_gain_row_major,
            &self.state,
            &self.integral_indices,
        )?;
        let after = command_from_gain(
            self.table.states,
            self.table.controls,
            new_gain_row_major,
            &adjusted,
        );
        let max_command_jump = before
            .iter()
            .zip(&after)
            .map(|(lhs, rhs)| (lhs - rhs).abs())
            .fold(0.0_f64, f64::max);
        self.state = adjusted;
        Ok(BumplessTransferReport {
            command_before: before,
            command_after: after,
            max_command_jump,
        })
    }
}

/// Interpolates a validated imported robust-controller schedule.
///
/// # Errors
///
/// Returns [`ScheduleError`] when the table is invalid or `s` is non-finite.
pub fn interpolate_robust_controller_table(
    table: &RobustControllerTable,
    s: f64,
) -> Result<RobustControllerMatrices, ScheduleError> {
    table
        .validate()
        .map_err(|reason| ScheduleError::InvalidTable { reason })?;
    if !s.is_finite() {
        return Err(ScheduleError::NonFiniteQuery { value: s });
    }

    let first = &table.nodes[0];
    if s <= first.schedule_value {
        return Ok(RobustControllerMatrices {
            lower_index: 0,
            upper_index: 0,
            lambda: 0.0,
            a_row_major: first.a_row_major.clone(),
            b_row_major: first.b_row_major.clone(),
            c_row_major: first.c_row_major.clone(),
            d_row_major: first.d_row_major.clone(),
        });
    }

    let last_index = table.nodes.len() - 1;
    let last = &table.nodes[last_index];
    if s >= last.schedule_value {
        return Ok(RobustControllerMatrices {
            lower_index: last_index,
            upper_index: last_index,
            lambda: 0.0,
            a_row_major: last.a_row_major.clone(),
            b_row_major: last.b_row_major.clone(),
            c_row_major: last.c_row_major.clone(),
            d_row_major: last.d_row_major.clone(),
        });
    }

    for lower_index in 0..last_index {
        let lower = &table.nodes[lower_index];
        let upper = &table.nodes[lower_index + 1];
        if (lower.schedule_value..=upper.schedule_value).contains(&s) {
            let raw_lambda =
                (s - lower.schedule_value) / (upper.schedule_value - lower.schedule_value);
            let lambda = match table.interp {
                GainInterp::Linear => raw_lambda,
                GainInterp::Smoothstep => raw_lambda * raw_lambda * (3.0 - 2.0 * raw_lambda),
            };
            return Ok(RobustControllerMatrices {
                lower_index,
                upper_index: lower_index + 1,
                lambda,
                a_row_major: blend(&lower.a_row_major, &upper.a_row_major, lambda),
                b_row_major: blend(&lower.b_row_major, &upper.b_row_major, lambda),
                c_row_major: blend(&lower.c_row_major, &upper.c_row_major, lambda),
                d_row_major: blend(&lower.d_row_major, &upper.d_row_major, lambda),
            });
        }
    }

    Err(ScheduleError::InvalidTable {
        reason: "robust controller interval lookup failed despite validated nodes".to_string(),
    })
}

/// Interpolates a validated scalar gain schedule.
///
/// # Errors
///
/// Returns [`ScheduleError::InvalidTable`] if the table is invalid or
/// [`ScheduleError::NonFiniteQuery`] if `s` is not finite.
pub fn interpolate_gain_schedule(
    table: &GainScheduleTable,
    s: f64,
) -> Result<ScheduledGainSet, ScheduleError> {
    table
        .validate()
        .map_err(|reason| ScheduleError::InvalidTable { reason })?;
    if !s.is_finite() {
        return Err(ScheduleError::NonFiniteQuery { value: s });
    }

    let first = &table.nodes[0];
    if s <= first.schedule_value {
        return Ok(ScheduledGainSet {
            lower_index: 0,
            upper_index: 0,
            lambda: 0.0,
            gain_row_major: first.gain_row_major.clone(),
            observer_gain_row_major: first.observer_gain_row_major.clone(),
        });
    }

    let last_index = table.nodes.len() - 1;
    let last = &table.nodes[last_index];
    if s >= last.schedule_value {
        return Ok(ScheduledGainSet {
            lower_index: last_index,
            upper_index: last_index,
            lambda: 0.0,
            gain_row_major: last.gain_row_major.clone(),
            observer_gain_row_major: last.observer_gain_row_major.clone(),
        });
    }

    for lower_index in 0..last_index {
        let lower = &table.nodes[lower_index];
        let upper = &table.nodes[lower_index + 1];
        if (lower.schedule_value..=upper.schedule_value).contains(&s) {
            let raw_lambda =
                (s - lower.schedule_value) / (upper.schedule_value - lower.schedule_value);
            let lambda = match table.interp {
                GainInterp::Linear => raw_lambda,
                GainInterp::Smoothstep => raw_lambda * raw_lambda * (3.0 - 2.0 * raw_lambda),
            };
            return Ok(ScheduledGainSet {
                lower_index,
                upper_index: lower_index + 1,
                lambda,
                gain_row_major: blend(&lower.gain_row_major, &upper.gain_row_major, lambda),
                observer_gain_row_major: match (
                    &lower.observer_gain_row_major,
                    &upper.observer_gain_row_major,
                ) {
                    (Some(lhs), Some(rhs)) => Some(blend(lhs, rhs, lambda)),
                    _ => None,
                },
            });
        }
    }

    Err(ScheduleError::InvalidTable {
        reason: "schedule interval lookup failed despite validated nodes".to_string(),
    })
}

/// Adjusts integral states so a schedule boundary does not introduce
/// a command jump.
///
/// The solve is least-squares when there are more integral states than
/// controls and exact when the integral submatrix is square and full-rank.
///
/// # Errors
///
/// Returns [`ScheduleError`] on dimension mismatch, empty integral
/// subspace, or singular normal equations.
pub fn reset_integral_states_for_bumpless_transfer(
    states: usize,
    controls: usize,
    old_gain_row_major: &[f64],
    new_gain_row_major: &[f64],
    state: &[f64],
    integral_indices: &[usize],
) -> Result<Vec<f64>, ScheduleError> {
    validate_gain_shape("old_gain", states, controls, old_gain_row_major)?;
    validate_gain_shape("new_gain", states, controls, new_gain_row_major)?;
    if state.len() != states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "state",
            expected: states,
            actual: state.len(),
        });
    }
    if integral_indices.is_empty() {
        return Err(ScheduleError::InvalidDimension {
            label: "integral_indices",
            value: 0,
            max: states,
        });
    }
    for index in integral_indices {
        if *index >= states {
            return Err(ScheduleError::InvalidDimension {
                label: "integral_index",
                value: *index,
                max: states.saturating_sub(1),
            });
        }
    }

    let old_command = command_from_gain(states, controls, old_gain_row_major, state);
    let new_command = command_from_gain(states, controls, new_gain_row_major, state);
    let mut rhs = vec![0.0_f64; controls];
    for control in 0..controls {
        rhs[control] = new_command[control] - old_command[control];
    }

    let columns = integral_indices.len();
    let mut influence = vec![0.0_f64; controls * columns];
    for control in 0..controls {
        for (col, state_index) in integral_indices.iter().enumerate() {
            influence[control * columns + col] = new_gain_row_major[control * states + state_index];
        }
    }
    let delta = least_squares(&influence, controls, columns, &rhs)?;
    let mut adjusted = state.to_vec();
    for (index, value) in integral_indices.iter().zip(delta) {
        adjusted[*index] += value;
    }
    Ok(adjusted)
}

/// Conservative closed-loop stability margin for `A - B K`.
///
/// The returned value is `1 - max_i Σ_j |A_cl[i,j]|`; positive values
/// prove Schur stability by the induced infinity-norm bound. It is
/// intentionally conservative and deterministic, suitable for
/// schedule-grid tripwires.
///
/// # Errors
///
/// Returns [`ScheduleError`] on dimension mismatch.
pub fn closed_loop_row_sum_margin(
    states: usize,
    controls: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    k_row_major: &[f64],
) -> Result<f64, ScheduleError> {
    if a_row_major.len() != states * states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "A",
            expected: states * states,
            actual: a_row_major.len(),
        });
    }
    if b_row_major.len() != states * controls {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "B",
            expected: states * controls,
            actual: b_row_major.len(),
        });
    }
    validate_gain_shape("K", states, controls, k_row_major)?;

    let mut worst = 0.0_f64;
    for row in 0..states {
        let mut sum = 0.0_f64;
        for col in 0..states {
            let mut value = a_row_major[row * states + col];
            for control in 0..controls {
                value -=
                    b_row_major[row * controls + control] * k_row_major[control * states + col];
            }
            sum += value.abs();
        }
        worst = worst.max(sum);
    }
    Ok(1.0 - worst)
}

/// Sweeps a schedule at caller-supplied nodes/interior points and
/// returns the minimum conservative row-sum margin.
///
/// # Errors
///
/// Returns [`ScheduleError`] if interpolation or a matrix check fails.
pub fn sweep_closed_loop_row_sum_margin(
    table: &GainScheduleTable,
    a_row_major: &[f64],
    b_row_major: &[f64],
    sample_values: &[f64],
) -> Result<f64, ScheduleError> {
    if sample_values.is_empty() {
        return Err(ScheduleError::InvalidDimension {
            label: "sample_values",
            value: 0,
            max: usize::MAX,
        });
    }
    let mut minimum = f64::INFINITY;
    for sample in sample_values {
        let gain = interpolate_gain_schedule(table, *sample)?;
        let margin = closed_loop_row_sum_margin(
            table.states,
            table.controls,
            a_row_major,
            b_row_major,
            &gain.gain_row_major,
        )?;
        minimum = minimum.min(margin);
    }
    Ok(minimum)
}

/// Computes classical SISO gain/phase margins for
/// `L(z) = K (zI - A)^-1 B`.
///
/// This diagnostic is used for toy schedule-grid tripwires. It is not
/// a substitute for offline MIMO robustness analysis of a real vehicle.
///
/// # Errors
///
/// Returns [`ScheduleError`] on dimension mismatch, non-SISO inputs, or
/// singular frequency-response solves.
pub fn siso_discrete_loop_margins(
    states: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    k_row_major: &[f64],
    frequency_samples: usize,
) -> Result<SisoLoopMargins, ScheduleError> {
    if frequency_samples < 3 {
        return Err(ScheduleError::InvalidDimension {
            label: "frequency_samples",
            value: frequency_samples,
            max: usize::MAX,
        });
    }
    if a_row_major.len() != states * states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "A",
            expected: states * states,
            actual: a_row_major.len(),
        });
    }
    if b_row_major.len() != states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "B",
            expected: states,
            actual: b_row_major.len(),
        });
    }
    validate_gain_shape("K", states, 1, k_row_major)?;

    let mut magnitudes = Vec::with_capacity(frequency_samples + 1);
    let mut phases = Vec::with_capacity(frequency_samples + 1);
    let mut previous_phase = 0.0_f64;
    for index in 0..=frequency_samples {
        let omega = core::f64::consts::PI * (index as f64) / (frequency_samples as f64);
        let response = siso_loop_response(states, a_row_major, b_row_major, k_row_major, omega)?;
        let magnitude = response.abs();
        let mut phase = response.arg();
        if index > 0 {
            while phase - previous_phase > core::f64::consts::PI {
                phase -= 2.0 * core::f64::consts::PI;
            }
            while previous_phase - phase > core::f64::consts::PI {
                phase += 2.0 * core::f64::consts::PI;
            }
        }
        previous_phase = phase;
        magnitudes.push(magnitude);
        phases.push(phase);
    }

    let phase_margin_deg = find_gain_crossover_phase_margin(&magnitudes, &phases);
    let gain_margin_db = find_phase_crossover_gain_margin(&magnitudes, &phases);
    Ok(SisoLoopMargins {
        gain_margin_db,
        phase_margin_deg,
    })
}

/// Sweeps SISO margins over a scalar gain schedule.
///
/// # Errors
///
/// Returns [`ScheduleError`] if schedule interpolation or a frequency
/// response solve fails.
pub fn sweep_siso_discrete_loop_margins(
    table: &GainScheduleTable,
    a_row_major: &[f64],
    b_row_major: &[f64],
    sample_values: &[f64],
    frequency_samples: usize,
) -> Result<SisoMarginSweep, ScheduleError> {
    if table.controls != 1 {
        return Err(ScheduleError::InvalidDimension {
            label: "controls",
            value: table.controls,
            max: 1,
        });
    }
    if sample_values.is_empty() {
        return Err(ScheduleError::InvalidDimension {
            label: "sample_values",
            value: 0,
            max: usize::MAX,
        });
    }
    let mut min_gain_margin_db = f64::INFINITY;
    let mut min_phase_margin_deg = f64::INFINITY;
    for sample in sample_values {
        let gain = interpolate_gain_schedule(table, *sample)?;
        let margins = siso_discrete_loop_margins(
            table.states,
            a_row_major,
            b_row_major,
            &gain.gain_row_major,
            frequency_samples,
        )?;
        min_gain_margin_db = min_gain_margin_db.min(margins.gain_margin_db);
        min_phase_margin_deg = min_phase_margin_deg.min(margins.phase_margin_deg);
    }
    Ok(SisoMarginSweep {
        min_gain_margin_db,
        min_phase_margin_deg,
    })
}

/// Computes classical SISO margins for an open-loop product
/// `L(z) = plant(z) controller(z)`.
///
/// This is intended for deterministic toy/public-benchmark margin
/// tripwires. It does not replace external MIMO H∞/μ validation.
///
/// # Errors
///
/// Returns [`ScheduleError`] on dimension mismatch, non-finite model
/// entries, too few frequency samples, or singular frequency-response
/// solves.
pub fn siso_state_space_loop_margins(
    plant: SisoStateSpaceRef<'_>,
    controller: SisoStateSpaceRef<'_>,
    frequency_samples: usize,
) -> Result<SisoLoopMargins, ScheduleError> {
    if frequency_samples < 3 {
        return Err(ScheduleError::InvalidDimension {
            label: "frequency_samples",
            value: frequency_samples,
            max: usize::MAX,
        });
    }
    validate_siso_state_space(
        plant,
        "plant_states",
        "plant_A",
        "plant_B",
        "plant_C",
        "plant_D",
    )?;
    validate_siso_state_space(
        controller,
        "controller_states",
        "controller_A",
        "controller_B",
        "controller_C",
        "controller_D",
    )?;

    let mut magnitudes = Vec::with_capacity(frequency_samples + 1);
    let mut phases = Vec::with_capacity(frequency_samples + 1);
    let mut previous_phase = 0.0_f64;
    for index in 0..=frequency_samples {
        let omega = core::f64::consts::PI * (index as f64) / (frequency_samples as f64);
        let response = siso_state_space_response(plant, omega)?
            * siso_state_space_response(controller, omega)?;
        let magnitude = response.abs();
        let mut phase = response.arg();
        if index > 0 {
            while phase - previous_phase > core::f64::consts::PI {
                phase -= 2.0 * core::f64::consts::PI;
            }
            while previous_phase - phase > core::f64::consts::PI {
                phase += 2.0 * core::f64::consts::PI;
            }
        }
        previous_phase = phase;
        magnitudes.push(magnitude);
        phases.push(phase);
    }

    Ok(SisoLoopMargins {
        gain_margin_db: find_phase_crossover_gain_margin(&magnitudes, &phases),
        phase_margin_deg: find_gain_crossover_phase_margin(&magnitudes, &phases),
    })
}

/// Sweeps SISO margins for an imported robust-controller schedule against
/// a caller-supplied SISO plant.
///
/// The imported controller must be SISO (`measurements == controls == 1`).
/// The sweep samples declared nodes/interior points chosen by the caller and
/// returns the minimum classical gain/phase margin over those points.
///
/// # Errors
///
/// Returns [`ScheduleError`] if the table is invalid, not SISO, the plant is
/// malformed, a sample is non-finite, or a frequency-response solve fails.
pub fn sweep_imported_robust_controller_siso_margins(
    table: &RobustControllerTable,
    plant: SisoStateSpaceRef<'_>,
    sample_values: &[f64],
    frequency_samples: usize,
) -> Result<SisoMarginSweep, ScheduleError> {
    table
        .validate()
        .map_err(|reason| ScheduleError::InvalidTable { reason })?;
    if table.measurements != 1 {
        return Err(ScheduleError::InvalidDimension {
            label: "robust_controller_measurements",
            value: table.measurements,
            max: 1,
        });
    }
    if table.controls != 1 {
        return Err(ScheduleError::InvalidDimension {
            label: "robust_controller_controls",
            value: table.controls,
            max: 1,
        });
    }
    if sample_values.is_empty() {
        return Err(ScheduleError::InvalidDimension {
            label: "sample_values",
            value: 0,
            max: usize::MAX,
        });
    }

    let mut min_gain_margin_db = f64::INFINITY;
    let mut min_phase_margin_deg = f64::INFINITY;
    for sample in sample_values {
        let matrices = interpolate_robust_controller_table(table, *sample)?;
        let controller = SisoStateSpaceRef {
            states: table.states,
            a_row_major: &matrices.a_row_major,
            b_row_major: &matrices.b_row_major,
            c_row_major: &matrices.c_row_major,
            d: matrices.d_row_major[0],
        };
        let margins = siso_state_space_loop_margins(plant, controller, frequency_samples)?;
        min_gain_margin_db = min_gain_margin_db.min(margins.gain_margin_db);
        min_phase_margin_deg = min_phase_margin_deg.min(margins.phase_margin_deg);
    }
    Ok(SisoMarginSweep {
        min_gain_margin_db,
        min_phase_margin_deg,
    })
}

fn blend(lhs: &[f64], rhs: &[f64], lambda: f64) -> Vec<f64> {
    lhs.iter()
        .zip(rhs)
        .map(|(left, right)| (1.0 - lambda) * left + lambda * right)
        .collect()
}

fn robust_controller_command(
    states: usize,
    measurements: usize,
    controls: usize,
    c_row_major: &[f64],
    d_row_major: &[f64],
    state: &[f64],
    measurement: &[f64],
) -> Result<Vec<f64>, ScheduleError> {
    if c_row_major.len() != controls * states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_C",
            expected: controls * states,
            actual: c_row_major.len(),
        });
    }
    if d_row_major.len() != controls * measurements {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_D",
            expected: controls * measurements,
            actual: d_row_major.len(),
        });
    }
    if state.len() != states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_controller_state",
            expected: states,
            actual: state.len(),
        });
    }
    if measurement.len() != measurements {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_controller_measurement",
            expected: measurements,
            actual: measurement.len(),
        });
    }

    let mut command = vec![0.0_f64; controls];
    for control in 0..controls {
        let mut value = 0.0_f64;
        for state_index in 0..states {
            value += c_row_major[control * states + state_index] * state[state_index];
        }
        for measurement_index in 0..measurements {
            value += d_row_major[control * measurements + measurement_index]
                * measurement[measurement_index];
        }
        command[control] = value;
    }
    Ok(command)
}

fn robust_controller_next_state(
    states: usize,
    measurements: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    state: &[f64],
    measurement: &[f64],
) -> Result<Vec<f64>, ScheduleError> {
    if a_row_major.len() != states * states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_A",
            expected: states * states,
            actual: a_row_major.len(),
        });
    }
    if b_row_major.len() != states * measurements {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_B",
            expected: states * measurements,
            actual: b_row_major.len(),
        });
    }
    if state.len() != states {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_controller_state",
            expected: states,
            actual: state.len(),
        });
    }
    if measurement.len() != measurements {
        return Err(ScheduleError::MatrixLengthMismatch {
            label: "robust_controller_measurement",
            expected: measurements,
            actual: measurement.len(),
        });
    }

    let mut next_state = vec![0.0_f64; states];
    for row in 0..states {
        let mut value = 0.0_f64;
        for col in 0..states {
            value += a_row_major[row * states + col] * state[col];
        }
        for measurement_index in 0..measurements {
            value += b_row_major[row * measurements + measurement_index]
                * measurement[measurement_index];
        }
        next_state[row] = value;
    }
    Ok(next_state)
}

fn find_gain_crossover_phase_margin(magnitudes: &[f64], phases: &[f64]) -> f64 {
    for index in 0..(magnitudes.len() - 1) {
        let lhs = magnitudes[index] - 1.0;
        let rhs = magnitudes[index + 1] - 1.0;
        if lhs == 0.0 {
            return 180.0 + phases[index].to_degrees();
        }
        if lhs.signum() != rhs.signum() {
            let denom = magnitudes[index + 1] - magnitudes[index];
            let lambda = if denom.abs() > 0.0 {
                (1.0 - magnitudes[index]) / denom
            } else {
                0.0
            };
            let phase = (1.0 - lambda) * phases[index] + lambda * phases[index + 1];
            return 180.0 + phase.to_degrees();
        }
    }
    f64::INFINITY
}

fn find_phase_crossover_gain_margin(magnitudes: &[f64], phases: &[f64]) -> f64 {
    let target = -core::f64::consts::PI;
    for index in 0..(phases.len() - 1) {
        let lhs = phases[index] - target;
        let rhs = phases[index + 1] - target;
        if lhs == 0.0 {
            return -20.0 * magnitudes[index].log10();
        }
        if lhs.signum() != rhs.signum() {
            let denom = phases[index + 1] - phases[index];
            let lambda = if denom.abs() > 0.0 {
                (target - phases[index]) / denom
            } else {
                0.0
            };
            let magnitude = (1.0 - lambda) * magnitudes[index] + lambda * magnitudes[index + 1];
            return -20.0 * magnitude.log10();
        }
    }
    f64::INFINITY
}

fn siso_loop_response(
    states: usize,
    a_row_major: &[f64],
    b_row_major: &[f64],
    k_row_major: &[f64],
    omega_rad: f64,
) -> Result<Complex64, ScheduleError> {
    let z = Complex64 {
        re: omega_rad.cos(),
        im: omega_rad.sin(),
    };
    let mut matrix = vec![Complex64::ZERO; states * states];
    for row in 0..states {
        for col in 0..states {
            let mut value = Complex64 {
                re: -a_row_major[row * states + col],
                im: 0.0,
            };
            if row == col {
                value = value + z;
            }
            matrix[row * states + col] = value;
        }
    }
    let mut rhs = b_row_major
        .iter()
        .map(|value| Complex64 {
            re: *value,
            im: 0.0,
        })
        .collect::<Vec<_>>();
    let state_response = solve_complex_square("zI-A", &mut matrix, states, &mut rhs)?;
    let mut response = Complex64::ZERO;
    for (gain, state) in k_row_major.iter().zip(state_response) {
        response = response + Complex64 { re: *gain, im: 0.0 } * state;
    }
    Ok(response)
}

fn siso_state_space_response(
    model: SisoStateSpaceRef<'_>,
    omega_rad: f64,
) -> Result<Complex64, ScheduleError> {
    let z = Complex64 {
        re: omega_rad.cos(),
        im: omega_rad.sin(),
    };
    let mut matrix = vec![Complex64::ZERO; model.states * model.states];
    for row in 0..model.states {
        for col in 0..model.states {
            let mut value = Complex64 {
                re: -model.a_row_major[row * model.states + col],
                im: 0.0,
            };
            if row == col {
                value = value + z;
            }
            matrix[row * model.states + col] = value;
        }
    }
    let mut rhs = model
        .b_row_major
        .iter()
        .map(|value| Complex64 {
            re: *value,
            im: 0.0,
        })
        .collect::<Vec<_>>();
    let state_response =
        solve_complex_square("siso state-space zI-A", &mut matrix, model.states, &mut rhs)?;
    let mut response = Complex64 {
        re: model.d,
        im: 0.0,
    };
    for (out, state) in model.c_row_major.iter().zip(state_response) {
        response = response + Complex64 { re: *out, im: 0.0 } * state;
    }
    Ok(response)
}

fn validate_siso_state_space(
    model: SisoStateSpaceRef<'_>,
    states_label: &'static str,
    a_label: &'static str,
    b_label: &'static str,
    c_label: &'static str,
    d_label: &'static str,
) -> Result<(), ScheduleError> {
    if model.states == 0 || model.states > 16 {
        return Err(ScheduleError::InvalidDimension {
            label: states_label,
            value: model.states,
            max: 16,
        });
    }
    validate_finite_row_major(a_label, model.states * model.states, model.a_row_major)?;
    validate_finite_row_major(b_label, model.states, model.b_row_major)?;
    validate_finite_row_major(c_label, model.states, model.c_row_major)?;
    if !model.d.is_finite() {
        return Err(ScheduleError::NonFiniteVector { label: d_label });
    }
    Ok(())
}

fn validate_finite_row_major(
    label: &'static str,
    expected: usize,
    values: &[f64],
) -> Result<(), ScheduleError> {
    if values.len() != expected {
        return Err(ScheduleError::MatrixLengthMismatch {
            label,
            expected,
            actual: values.len(),
        });
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err(ScheduleError::NonFiniteVector { label });
    }
    Ok(())
}

#[derive(Copy, Clone, Debug, Default, PartialEq)]
struct Complex64 {
    re: f64,
    im: f64,
}

impl Complex64 {
    const ZERO: Self = Self { re: 0.0, im: 0.0 };

    fn abs(self) -> f64 {
        (self.re * self.re + self.im * self.im).sqrt()
    }

    fn arg(self) -> f64 {
        self.im.atan2(self.re)
    }
}

impl core::ops::Add for Complex64 {
    type Output = Self;

    fn add(self, rhs: Self) -> Self::Output {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }
}

impl core::ops::Sub for Complex64 {
    type Output = Self;

    fn sub(self, rhs: Self) -> Self::Output {
        Self {
            re: self.re - rhs.re,
            im: self.im - rhs.im,
        }
    }
}

impl core::ops::Mul for Complex64 {
    type Output = Self;

    fn mul(self, rhs: Self) -> Self::Output {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }
}

impl core::ops::Div for Complex64 {
    type Output = Self;

    fn div(self, rhs: Self) -> Self::Output {
        let denom = rhs.re * rhs.re + rhs.im * rhs.im;
        Self {
            re: (self.re * rhs.re + self.im * rhs.im) / denom,
            im: (self.im * rhs.re - self.re * rhs.im) / denom,
        }
    }
}

fn solve_complex_square(
    label: &'static str,
    matrix_row_major: &mut [Complex64],
    size: usize,
    rhs: &mut [Complex64],
) -> Result<Vec<Complex64>, ScheduleError> {
    const PIVOT_TOL: f64 = 1.0e-14;
    for pivot in 0..size {
        let mut best = pivot;
        let mut best_abs = matrix_row_major[pivot * size + pivot].abs();
        for row in (pivot + 1)..size {
            let candidate = matrix_row_major[row * size + pivot].abs();
            if candidate > best_abs {
                best = row;
                best_abs = candidate;
            }
        }
        if !best_abs.is_finite() || best_abs <= PIVOT_TOL {
            return Err(ScheduleError::LinearSolveFailed { label });
        }
        if best != pivot {
            for col in 0..size {
                matrix_row_major.swap(pivot * size + col, best * size + col);
            }
            rhs.swap(pivot, best);
        }
        for row in (pivot + 1)..size {
            let factor =
                matrix_row_major[row * size + pivot] / matrix_row_major[pivot * size + pivot];
            matrix_row_major[row * size + pivot] = Complex64::ZERO;
            for col in (pivot + 1)..size {
                matrix_row_major[row * size + col] = matrix_row_major[row * size + col]
                    - factor * matrix_row_major[pivot * size + col];
            }
            rhs[row] = rhs[row] - factor * rhs[pivot];
        }
    }

    let mut x = vec![Complex64::ZERO; size];
    for row_rev in 0..size {
        let row = size - 1 - row_rev;
        let mut sum = rhs[row];
        for col in (row + 1)..size {
            sum = sum - matrix_row_major[row * size + col] * x[col];
        }
        let diag = matrix_row_major[row * size + row];
        if diag.abs() <= PIVOT_TOL {
            return Err(ScheduleError::LinearSolveFailed { label });
        }
        x[row] = sum / diag;
    }
    Ok(x)
}

fn validate_gain_shape(
    label: &'static str,
    states: usize,
    controls: usize,
    gain_row_major: &[f64],
) -> Result<(), ScheduleError> {
    let expected = states * controls;
    if gain_row_major.len() != expected {
        return Err(ScheduleError::MatrixLengthMismatch {
            label,
            expected,
            actual: gain_row_major.len(),
        });
    }
    if gain_row_major.iter().any(|value| !value.is_finite()) {
        return Err(ScheduleError::LinearSolveFailed {
            label: "non-finite gain",
        });
    }
    Ok(())
}

fn command_from_gain(
    states: usize,
    controls: usize,
    gain_row_major: &[f64],
    state: &[f64],
) -> Vec<f64> {
    let mut out = vec![0.0_f64; controls];
    for control in 0..controls {
        let mut sum = 0.0_f64;
        for state_index in 0..states {
            sum += gain_row_major[control * states + state_index] * state[state_index];
        }
        out[control] = -sum;
    }
    out
}

fn least_squares(
    matrix_row_major: &[f64],
    rows: usize,
    cols: usize,
    rhs: &[f64],
) -> Result<Vec<f64>, ScheduleError> {
    let mut ata = vec![0.0_f64; cols * cols];
    let mut atb = vec![0.0_f64; cols];
    for row in 0..rows {
        for col_i in 0..cols {
            let a_i = matrix_row_major[row * cols + col_i];
            atb[col_i] += a_i * rhs[row];
            for col_j in 0..cols {
                ata[col_i * cols + col_j] += a_i * matrix_row_major[row * cols + col_j];
            }
        }
    }
    solve_square("bumpless normal equations", &mut ata, cols, &mut atb)
}

fn solve_square(
    label: &'static str,
    matrix_row_major: &mut [f64],
    size: usize,
    rhs: &mut [f64],
) -> Result<Vec<f64>, ScheduleError> {
    const PIVOT_TOL: f64 = 1.0e-14;
    for pivot in 0..size {
        let mut best = pivot;
        let mut best_abs = matrix_row_major[pivot * size + pivot].abs();
        for row in (pivot + 1)..size {
            let candidate = matrix_row_major[row * size + pivot].abs();
            if candidate > best_abs {
                best = row;
                best_abs = candidate;
            }
        }
        if !best_abs.is_finite() || best_abs <= PIVOT_TOL {
            return Err(ScheduleError::LinearSolveFailed { label });
        }
        if best != pivot {
            for col in 0..size {
                matrix_row_major.swap(pivot * size + col, best * size + col);
            }
            rhs.swap(pivot, best);
        }
        for row in (pivot + 1)..size {
            let factor =
                matrix_row_major[row * size + pivot] / matrix_row_major[pivot * size + pivot];
            matrix_row_major[row * size + pivot] = 0.0;
            for col in (pivot + 1)..size {
                matrix_row_major[row * size + col] -= factor * matrix_row_major[pivot * size + col];
            }
            rhs[row] -= factor * rhs[pivot];
        }
    }

    let mut x = vec![0.0_f64; size];
    for row_rev in 0..size {
        let row = size - 1 - row_rev;
        let mut sum = rhs[row];
        for col in (row + 1)..size {
            sum -= matrix_row_major[row * size + col] * x[col];
        }
        let diag = matrix_row_major[row * size + row];
        if diag.abs() <= PIVOT_TOL {
            return Err(ScheduleError::LinearSolveFailed { label });
        }
        x[row] = sum / diag;
    }
    Ok(x)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::tables::{
        CoupledGainNode, GainInterp, RobustControllerNode, RobustControllerTable, ScheduleVariable,
    };
    use toml::Value;

    const GAIN_SCHEDULE_LQG_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/gain-schedule-lqg-tolerance-v1.toml");
    const ROBUST_CONTROLLER_SISO_MARGIN_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/robust-controller-siso-margin-tolerance-v1.toml");

    fn node(schedule_value: f64, gain: Vec<f64>) -> CoupledGainNode {
        CoupledGainNode {
            node_id: format!("node_{schedule_value:.1}"),
            schedule_value,
            flight_condition_tag: "mach".to_string(),
            states: 2,
            controls: 1,
            measurements: 1,
            gain_row_major: gain,
            observer_gain_row_major: Some(vec![0.1 + schedule_value, 0.2 + schedule_value]),
        }
    }

    fn table(interp: GainInterp) -> GainScheduleTable {
        GainScheduleTable {
            variable: ScheduleVariable::Mach,
            interp,
            states: 2,
            controls: 1,
            measurements: 1,
            nodes: vec![node(0.0, vec![2.0, 1.0]), node(1.0, vec![4.0, 3.0])],
        }
    }

    fn robust_controller_table(interp: GainInterp) -> RobustControllerTable {
        RobustControllerTable {
            controller_id: "robust_ascent_controller".to_string(),
            source_id: "synthetic_hinf_reduced_order_v1".to_string(),
            source_sha256_hex: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
                .to_string(),
            variable: ScheduleVariable::DynamicPressurePa,
            interp,
            states: 2,
            measurements: 1,
            controls: 1,
            nodes: vec![
                RobustControllerNode {
                    node_id: "low_q".to_string(),
                    schedule_value: 10_000.0,
                    flight_condition_tag: "qbar_mach_mass".to_string(),
                    states: 2,
                    measurements: 1,
                    controls: 1,
                    a_row_major: vec![1.0, 0.1, 0.0, 0.9],
                    b_row_major: vec![0.0, 0.2],
                    c_row_major: vec![1.0, 0.0],
                    d_row_major: vec![0.1],
                },
                RobustControllerNode {
                    node_id: "high_q".to_string(),
                    schedule_value: 20_000.0,
                    flight_condition_tag: "qbar_mach_mass".to_string(),
                    states: 2,
                    measurements: 1,
                    controls: 1,
                    a_row_major: vec![0.8, 0.0, 0.1, 0.7],
                    b_row_major: vec![0.1, 0.3],
                    c_row_major: vec![0.5, 0.5],
                    d_row_major: vec![0.2],
                },
            ],
        }
    }

    fn toml_table<'a>(value: &'a Value, key: &str) -> &'a toml::map::Map<String, Value> {
        value
            .get(key)
            .and_then(Value::as_table)
            .unwrap_or_else(|| panic!("missing TOML table {key}"))
    }

    fn toml_f64(value: &toml::map::Map<String, Value>, key: &str) -> f64 {
        value
            .get(key)
            .and_then(Value::as_float)
            .unwrap_or_else(|| panic!("missing TOML float {key}"))
    }

    fn toml_usize(value: &toml::map::Map<String, Value>, key: &str) -> usize {
        value
            .get(key)
            .and_then(Value::as_integer)
            .and_then(|raw| usize::try_from(raw).ok())
            .unwrap_or_else(|| panic!("missing TOML usize {key}"))
    }

    fn toml_f64_array(value: &toml::map::Map<String, Value>, key: &str) -> Vec<f64> {
        value
            .get(key)
            .and_then(Value::as_array)
            .unwrap_or_else(|| panic!("missing TOML array {key}"))
            .iter()
            .map(|entry| {
                entry
                    .as_float()
                    .unwrap_or_else(|| panic!("TOML array {key} contains non-float"))
            })
            .collect()
    }

    fn toml_usize_array(value: &toml::map::Map<String, Value>, key: &str) -> Vec<usize> {
        value
            .get(key)
            .and_then(Value::as_array)
            .unwrap_or_else(|| panic!("missing TOML array {key}"))
            .iter()
            .map(|entry| {
                entry
                    .as_integer()
                    .and_then(|raw| usize::try_from(raw).ok())
                    .unwrap_or_else(|| panic!("TOML array {key} contains non-usize"))
            })
            .collect()
    }

    fn assert_close_vec(actual: &[f64], expected: &[f64], epsilon: f64, label: &str) {
        assert_eq!(actual.len(), expected.len(), "{label} length");
        for (index, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (*actual - *expected).abs() <= epsilon,
                "{label}[{index}] actual {actual:e}, expected {expected:e}, epsilon {epsilon:e}"
            );
        }
    }

    fn table_from_fixture(data: &Value) -> GainScheduleTable {
        let schedule = toml_table(data, "schedule");
        let variable = match schedule
            .get("variable")
            .and_then(Value::as_str)
            .expect("schedule variable")
        {
            "Mach" => ScheduleVariable::Mach,
            other => panic!("unsupported schedule variable {other}"),
        };
        let interp = match schedule
            .get("interp")
            .and_then(Value::as_str)
            .expect("schedule interp")
        {
            "Linear" => GainInterp::Linear,
            "Smoothstep" => GainInterp::Smoothstep,
            other => panic!("unsupported schedule interp {other}"),
        };
        let states = toml_usize(schedule, "states");
        let controls = toml_usize(schedule, "controls");
        let measurements = toml_usize(schedule, "measurements");
        let nodes = schedule
            .get("node")
            .and_then(Value::as_array)
            .expect("schedule nodes")
            .iter()
            .map(|node| {
                let node = node.as_table().expect("node table");
                CoupledGainNode {
                    node_id: node
                        .get("node_id")
                        .and_then(Value::as_str)
                        .expect("node_id")
                        .to_string(),
                    schedule_value: toml_f64(node, "schedule_value"),
                    flight_condition_tag: node
                        .get("flight_condition_tag")
                        .and_then(Value::as_str)
                        .expect("flight_condition_tag")
                        .to_string(),
                    states,
                    controls,
                    measurements,
                    gain_row_major: toml_f64_array(node, "gain_row_major"),
                    observer_gain_row_major: Some(toml_f64_array(node, "observer_gain_row_major")),
                }
            })
            .collect();
        GainScheduleTable {
            variable,
            interp,
            states,
            controls,
            measurements,
            nodes,
        }
    }

    fn robust_controller_table_from_fixture(data: &Value) -> RobustControllerTable {
        let schedule = toml_table(data, "robust_controller");
        let variable = match schedule
            .get("variable")
            .and_then(Value::as_str)
            .expect("robust schedule variable")
        {
            "DynamicPressurePa" => ScheduleVariable::DynamicPressurePa,
            other => panic!("unsupported robust schedule variable {other}"),
        };
        let interp = match schedule
            .get("interp")
            .and_then(Value::as_str)
            .expect("robust schedule interp")
        {
            "Linear" => GainInterp::Linear,
            "Smoothstep" => GainInterp::Smoothstep,
            other => panic!("unsupported robust schedule interp {other}"),
        };
        let states = toml_usize(schedule, "states");
        let measurements = toml_usize(schedule, "measurements");
        let controls = toml_usize(schedule, "controls");
        let nodes = schedule
            .get("node")
            .and_then(Value::as_array)
            .expect("robust nodes")
            .iter()
            .map(|node| {
                let node = node.as_table().expect("robust node table");
                RobustControllerNode {
                    node_id: node
                        .get("node_id")
                        .and_then(Value::as_str)
                        .expect("node_id")
                        .to_string(),
                    schedule_value: toml_f64(node, "schedule_value"),
                    flight_condition_tag: node
                        .get("flight_condition_tag")
                        .and_then(Value::as_str)
                        .expect("flight_condition_tag")
                        .to_string(),
                    states,
                    measurements,
                    controls,
                    a_row_major: toml_f64_array(node, "a_row_major"),
                    b_row_major: toml_f64_array(node, "b_row_major"),
                    c_row_major: toml_f64_array(node, "c_row_major"),
                    d_row_major: toml_f64_array(node, "d_row_major"),
                }
            })
            .collect();
        RobustControllerTable {
            controller_id: schedule
                .get("controller_id")
                .and_then(Value::as_str)
                .expect("controller_id")
                .to_string(),
            source_id: schedule
                .get("source_id")
                .and_then(Value::as_str)
                .expect("source_id")
                .to_string(),
            source_sha256_hex: schedule
                .get("source_sha256_hex")
                .and_then(Value::as_str)
                .expect("source_sha256_hex")
                .to_string(),
            variable,
            interp,
            states,
            measurements,
            controls,
            nodes,
        }
    }

    #[test]
    fn gain_schedule_interpolates_and_clamps() {
        let table = table(GainInterp::Linear);
        let mid = interpolate_gain_schedule(&table, 0.5).unwrap();
        assert_eq!(mid.lower_index, 0);
        assert_eq!(mid.upper_index, 1);
        assert_eq!(mid.gain_row_major, vec![3.0, 2.0]);
        let observer_gain = mid.observer_gain_row_major.unwrap();
        assert!((observer_gain[0] - 0.6).abs() < 1.0e-12);
        assert!((observer_gain[1] - 0.7).abs() < 1.0e-12);

        let low = interpolate_gain_schedule(&table, -1.0).unwrap();
        assert_eq!(low.gain_row_major, vec![2.0, 1.0]);
        let high = interpolate_gain_schedule(&table, 2.0).unwrap();
        assert_eq!(high.gain_row_major, vec![4.0, 3.0]);
    }

    #[test]
    fn gain_schedule_lqg_tolerance_table_matches_schedule_and_bumpless() {
        let data: Value =
            toml::from_str(GAIN_SCHEDULE_LQG_TOLERANCE_TABLE).expect("schedule table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_gain_schedule_lqg_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let gain_epsilon = toml_f64(tolerance, "absolute_gain");
        let state_epsilon = toml_f64(tolerance, "absolute_state");
        let min_margin = toml_f64(tolerance, "minimum_row_sum_margin");
        let table = table_from_fixture(&data);
        let schedule = toml_table(&data, "schedule");
        let lookup = interpolate_gain_schedule(&table, toml_f64(schedule, "query")).unwrap();
        assert_close_vec(
            &lookup.gain_row_major,
            &toml_f64_array(schedule, "expected_gain_row_major"),
            gain_epsilon,
            "interpolated K",
        );
        assert_close_vec(
            lookup.observer_gain_row_major.as_ref().unwrap(),
            &toml_f64_array(schedule, "expected_observer_gain_row_major"),
            gain_epsilon,
            "interpolated L",
        );

        let sweep = toml_table(&data, "closed_loop_sweep");
        let margin = sweep_closed_loop_row_sum_margin(
            &table,
            &toml_f64_array(sweep, "a_row_major"),
            &toml_f64_array(sweep, "b_row_major"),
            &toml_f64_array(sweep, "sample_values"),
        )
        .unwrap();
        assert!(margin >= min_margin);
        assert!((margin - toml_f64(sweep, "expected_min_row_sum_margin")).abs() < 1.0e-12);

        let siso = toml_table(&data, "siso_margin");
        let margins = sweep_siso_discrete_loop_margins(
            &table,
            &toml_f64_array(siso, "a_row_major"),
            &toml_f64_array(siso, "b_row_major"),
            &toml_f64_array(siso, "sample_values"),
            toml_usize(siso, "frequency_samples"),
        )
        .unwrap();
        assert!(margins.min_gain_margin_db >= toml_f64(siso, "min_gain_margin_db"));
        assert!(margins.min_phase_margin_deg >= toml_f64(siso, "min_phase_margin_deg"));

        let bumpless = toml_table(&data, "bumpless");
        let adjusted = reset_integral_states_for_bumpless_transfer(
            toml_usize(bumpless, "states"),
            toml_usize(bumpless, "controls"),
            &toml_f64_array(bumpless, "old_gain_row_major"),
            &toml_f64_array(bumpless, "new_gain_row_major"),
            &toml_f64_array(bumpless, "state"),
            &toml_usize_array(bumpless, "integral_indices"),
        )
        .unwrap();
        assert_close_vec(
            &adjusted,
            &toml_f64_array(bumpless, "expected_adjusted_state"),
            state_epsilon,
            "bumpless state",
        );
        let command = command_from_gain(
            toml_usize(bumpless, "states"),
            toml_usize(bumpless, "controls"),
            &toml_f64_array(bumpless, "new_gain_row_major"),
            &adjusted,
        );
        assert_close_vec(
            &command,
            &toml_f64_array(bumpless, "expected_command"),
            gain_epsilon,
            "bumpless command",
        );
    }

    #[test]
    fn gain_schedule_smoothstep_uses_pinned_fraction() {
        let table = table(GainInterp::Smoothstep);
        let gain = interpolate_gain_schedule(&table, 0.25).unwrap();
        // smoothstep(0.25) = 0.15625
        assert!((gain.lambda - 0.15625).abs() < f64::EPSILON);
        assert_eq!(gain.gain_row_major, vec![2.3125, 1.3125]);
    }

    #[test]
    fn robust_controller_interpolates_and_steps_state_space() {
        let table = robust_controller_table(GainInterp::Linear);
        let matrices = interpolate_robust_controller_table(&table, 15_000.0).unwrap();
        assert_eq!(matrices.lower_index, 0);
        assert_eq!(matrices.upper_index, 1);
        assert!((matrices.lambda - 0.5).abs() < f64::EPSILON);
        assert_close_vec(
            &matrices.a_row_major,
            &[0.9, 0.05, 0.05, 0.8],
            1.0e-14,
            "robust A",
        );
        assert_close_vec(&matrices.b_row_major, &[0.05, 0.25], 1.0e-14, "robust B");
        assert_close_vec(&matrices.c_row_major, &[0.75, 0.25], 1.0e-14, "robust C");
        assert_close_vec(&matrices.d_row_major, &[0.15], 1.0e-14, "robust D");

        let mut controller =
            ImportedRobustController::new(table, vec![1.0, -1.0]).expect("controller builds");
        let step = controller
            .step(15_000.0, &[2.0])
            .expect("controller step succeeds");
        assert_close_vec(&step.command, &[0.8], 1.0e-14, "robust command");
        assert_close_vec(&step.next_state, &[0.95, -0.25], 1.0e-14, "robust state");
        assert_close_vec(controller.state(), &[0.95, -0.25], 1.0e-14, "stored state");
    }

    #[test]
    fn robust_controller_siso_margin_tolerance_table_matches() {
        let data: Value = toml::from_str(ROBUST_CONTROLLER_SISO_MARGIN_TOLERANCE_TABLE)
            .expect("robust controller margin table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_robust_controller_siso_margin_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let plant_data = toml_table(&data, "plant");
        let plant_a = toml_f64_array(plant_data, "a_row_major");
        let plant_b = toml_f64_array(plant_data, "b_row_major");
        let plant_c = toml_f64_array(plant_data, "c_row_major");
        let plant = SisoStateSpaceRef {
            states: toml_usize(plant_data, "states"),
            a_row_major: &plant_a,
            b_row_major: &plant_b,
            c_row_major: &plant_c,
            d: toml_f64(plant_data, "d"),
        };
        let sweep = toml_table(&data, "sweep");
        let table = robust_controller_table_from_fixture(&data);
        let margins = sweep_imported_robust_controller_siso_margins(
            &table,
            plant,
            &toml_f64_array(sweep, "sample_values"),
            toml_usize(sweep, "frequency_samples"),
        )
        .unwrap();

        assert!(margins.min_gain_margin_db >= toml_f64(tolerance, "min_gain_margin_db"));
        assert!(margins.min_phase_margin_deg >= toml_f64(tolerance, "min_phase_margin_deg"));
    }

    #[test]
    fn robust_controller_siso_margin_rejects_bad_inputs() {
        let table = robust_controller_table(GainInterp::Linear);
        let plant_a = [0.5];
        let plant_b = [1.0];
        let plant_c = [1.0];
        let plant = SisoStateSpaceRef {
            states: 1,
            a_row_major: &plant_a,
            b_row_major: &plant_b,
            c_row_major: &plant_c,
            d: 0.0,
        };
        assert!(matches!(
            sweep_imported_robust_controller_siso_margins(&table, plant, &[15_000.0], 2)
                .unwrap_err(),
            ScheduleError::InvalidDimension {
                label: "frequency_samples",
                ..
            }
        ));

        let bad_c = [f64::NAN];
        let bad_plant = SisoStateSpaceRef {
            c_row_major: &bad_c,
            ..plant
        };
        assert!(matches!(
            sweep_imported_robust_controller_siso_margins(&table, bad_plant, &[15_000.0], 128)
                .unwrap_err(),
            ScheduleError::NonFiniteVector { label: "plant_C" }
        ));
    }

    #[test]
    fn robust_controller_rejects_bad_runtime_inputs() {
        let table = robust_controller_table(GainInterp::Linear);
        assert!(
            ImportedRobustController::new(table.clone(), vec![f64::NAN, 0.0])
                .unwrap_err()
                .to_string()
                .contains("finite")
        );

        let mut controller =
            ImportedRobustController::new(table, vec![0.0, 0.0]).expect("controller builds");
        assert!(matches!(
            controller.step(15_000.0, &[1.0, 2.0]).unwrap_err(),
            ScheduleError::MatrixLengthMismatch { .. }
        ));
        assert!(matches!(
            controller.step(f64::INFINITY, &[1.0]).unwrap_err(),
            ScheduleError::NonFiniteQuery { .. }
        ));
    }

    #[test]
    fn scheduled_controller_boundary_reset_is_bumpless() {
        let mut controller =
            ScheduledController::new(table(GainInterp::Linear), vec![0.5, 0.25], vec![1]).unwrap();
        let report = controller
            .on_schedule_boundary(&[2.0, 1.0], &[4.0, 2.0])
            .unwrap();
        assert!(report.max_command_jump < 1.0e-12);
        assert_eq!(report.command_before, report.command_after);
        assert!((controller.state()[1] + 0.375).abs() < 1.0e-12);
    }

    #[test]
    fn schedule_row_sum_margin_sweeps_nodes_and_interior() {
        let table = table(GainInterp::Linear);
        let margin = sweep_closed_loop_row_sum_margin(
            &table,
            &[0.8, 0.0, 0.0, 0.7],
            &[0.1, 0.0],
            &[0.0, 0.5, 1.0],
        )
        .unwrap();
        assert!(margin > 0.0);
    }

    #[test]
    fn gain_schedule_table_rejects_bad_shapes_and_target_vocabulary() {
        let mut bad = table(GainInterp::Linear);
        bad.nodes[1].schedule_value = 0.0;
        assert!(bad.validate().unwrap_err().contains("strictly sorted"));

        let mut bad = table(GainInterp::Linear);
        bad.nodes[0].flight_condition_tag = "target_range".to_string();
        assert!(bad.validate().unwrap_err().contains("target vocabulary"));
    }
}
