//! Validated-then-activated typed table registry.
//!
//! Pattern adapted from NASA cFE TBL: a table is an immutable typed
//! blob (gain schedule, mass/CG sweep, schedule definition) that the
//! ground / runner uplinks into a *pending* slot, asks the registry
//! to validate, and then atomically activates by swapping the pending
//! and active cells.
//!
//! # Why two buffers
//!
//! Modules read from the active buffer continuously. The runner can
//! stage a candidate update in the pending buffer, run validation
//! synchronously, and only swap on success. A failed validation
//! leaves the active buffer untouched — the controller never reads a
//! half-initialised table.
//!
//! # Determinism
//!
//! Active-table iteration order is registration order. The two-buffer
//! swap is deterministic: pending becomes active, the previous active
//! is dropped. Validation runs synchronously on the calling thread.

use std::any::{Any, TypeId};
use std::boxed::Box;
use std::string::{String, ToString};
use std::vec::Vec;

use crate::error::TableError;
use crate::filters::{BiquadSection, prewarped_tustin_notch};
use crate::stable_map::StableIndexMap;

/// Marker trait implemented by every typed table.
pub trait Table: 'static + Clone {
    /// Canonical table name. Convention: `snake_case`, namespaced by
    /// owning module.
    const NAME: &'static str;

    /// Returns `Ok(())` if the candidate value is structurally valid,
    /// or `Err(reason)` if it must be rejected. Reasons are
    /// human-readable strings recorded in [`TableError::ValidationFailed`].
    ///
    /// # Errors
    ///
    /// Returns a human-readable validation failure string when the
    /// table content is rejected.
    fn validate(&self) -> Result<(), String>;
}

/// Runtime table registry.
#[derive(Default)]
pub struct Tables {
    cells: StableIndexMap<TypeId, TableCell>,
}

struct TableCell {
    name: &'static str,
    active: Box<dyn Any>,
    pending: Option<Box<dyn Any>>,
}

/// Registered table descriptor.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct TableInfo {
    /// Canonical table name (`Table::NAME`).
    pub name: &'static str,
    /// `true` if a pending value is staged.
    pub has_pending: bool,
}

/// Offline-synthesised coupled LQR gain node.
///
/// The gain matrix is row-major `controls × states` and is intended
/// for table-driven runtime use as `u = -K x`. This table carries a
/// flight-condition tag only; it does not encode targets, ranges, or
/// mission geometry.
#[derive(Clone, Debug, PartialEq)]
pub struct CoupledGainNode {
    /// Stable node identifier, usually a gain-schedule knot id.
    pub node_id: String,
    /// Scalar schedule knot value (Mach, qbar, mass fraction, or time).
    pub schedule_value: f64,
    /// Non-targeting flight-condition tag such as `mach_qbar_mass`.
    pub flight_condition_tag: String,
    /// State dimension.
    pub states: usize,
    /// Control dimension.
    pub controls: usize,
    /// Measurement dimension for an optional LQG observer gain.
    pub measurements: usize,
    /// Row-major gain matrix `K`, shaped `controls × states`.
    pub gain_row_major: Vec<f64>,
    /// Optional row-major observer gain `L`, shaped `states × measurements`.
    pub observer_gain_row_major: Option<Vec<f64>>,
}

impl Table for CoupledGainNode {
    const NAME: &'static str = "lqr.coupled_gain_node";

    fn validate(&self) -> Result<(), String> {
        if self.node_id.trim().is_empty() {
            return Err("node_id must be non-empty".to_string());
        }
        if self.flight_condition_tag.trim().is_empty() {
            return Err("flight_condition_tag must be non-empty".to_string());
        }
        if !self.schedule_value.is_finite() {
            return Err("schedule_value must be finite".to_string());
        }
        if self.states == 0 || self.states > 16 {
            return Err("states must be in 1..=16".to_string());
        }
        if self.controls == 0 || self.controls > 16 {
            return Err("controls must be in 1..=16".to_string());
        }
        if self.measurements > 16 {
            return Err("measurements must be in 0..=16".to_string());
        }
        let expected = self.states * self.controls;
        if self.gain_row_major.len() != expected {
            return Err("gain_row_major length must equal controls * states".to_string());
        }
        if self.gain_row_major.iter().any(|value| !value.is_finite()) {
            return Err("gain_row_major entries must be finite".to_string());
        }
        if let Some(observer_gain) = &self.observer_gain_row_major {
            if self.measurements == 0 {
                return Err("observer gain requires measurements > 0".to_string());
            }
            if observer_gain.len() != self.states * self.measurements {
                return Err(
                    "observer_gain_row_major length must equal states * measurements".to_string(),
                );
            }
            if observer_gain.iter().any(|value| !value.is_finite()) {
                return Err("observer_gain_row_major entries must be finite".to_string());
            }
        }
        Ok(())
    }
}

/// Scalar scheduling variable for a coupled gain table.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum ScheduleVariable {
    /// Mach number.
    Mach,
    /// Dynamic pressure, Pa.
    DynamicPressurePa,
    /// Vehicle mass fraction.
    MassFraction,
    /// Mission elapsed time, s.
    TimeS,
    /// Nondimensional time or ascent progress fraction.
    NondimensionalTime,
}

/// Interpolation law between adjacent schedule nodes.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub enum GainInterp {
    /// Linear interpolation.
    Linear,
    /// Smoothstep interpolation, `λ²(3 − 2λ)`.
    Smoothstep,
}

/// One physical bending/slosh filter section scheduled at a table node.
///
/// Tables carry mode, zero, pole, and damping parameters only. Runtime
/// IIR taps are regenerated from these physical fields so schedules do
/// not interpolate raw filter coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct BendingFilterSectionSpec {
    /// Mode frequency used for prewarping, rad/s.
    pub mode_frequency_rad_s: f64,
    /// Continuous numerator frequency, rad/s.
    pub zero_frequency_rad_s: f64,
    /// Continuous numerator damping ratio.
    pub zero_damping_ratio: f64,
    /// Continuous denominator frequency, rad/s.
    pub pole_frequency_rad_s: f64,
    /// Continuous denominator damping ratio.
    pub pole_damping_ratio: f64,
}

impl BendingFilterSectionSpec {
    /// Discretize this physical section with prewarped Tustin.
    ///
    /// # Errors
    ///
    /// Returns a validation string when the physical parameters are
    /// invalid or generate unstable digital poles.
    pub fn discretize(&self, sample_interval_s: f64) -> Result<BiquadSection, String> {
        prewarped_tustin_notch(
            self.zero_frequency_rad_s,
            self.zero_damping_ratio,
            self.pole_frequency_rad_s,
            self.pole_damping_ratio,
            self.mode_frequency_rad_s,
            sample_interval_s,
        )
        .map_err(|_| "bending filter section must discretize to stable poles".to_string())
    }

    fn validate(&self, sample_interval_s: f64) -> Result<(), String> {
        if !self.mode_frequency_rad_s.is_finite()
            || !self.zero_frequency_rad_s.is_finite()
            || !self.zero_damping_ratio.is_finite()
            || !self.pole_frequency_rad_s.is_finite()
            || !self.pole_damping_ratio.is_finite()
        {
            return Err("bending filter physical parameters must be finite".to_string());
        }
        if self.mode_frequency_rad_s <= 0.0
            || self.zero_frequency_rad_s <= 0.0
            || self.pole_frequency_rad_s <= 0.0
        {
            return Err("bending filter frequencies must be positive".to_string());
        }
        if self.zero_damping_ratio <= 0.0 || self.pole_damping_ratio <= 0.0 {
            return Err("bending filter damping ratios must be positive".to_string());
        }
        if self.zero_damping_ratio >= self.pole_damping_ratio {
            return Err("bending filter zero damping must be lower than pole damping".to_string());
        }
        self.discretize(sample_interval_s)?;
        Ok(())
    }

    fn interpolate(&self, rhs: &Self, lambda: f64) -> Self {
        Self {
            mode_frequency_rad_s: lerp(self.mode_frequency_rad_s, rhs.mode_frequency_rad_s, lambda),
            zero_frequency_rad_s: lerp(self.zero_frequency_rad_s, rhs.zero_frequency_rad_s, lambda),
            zero_damping_ratio: lerp(self.zero_damping_ratio, rhs.zero_damping_ratio, lambda),
            pole_frequency_rad_s: lerp(self.pole_frequency_rad_s, rhs.pole_frequency_rad_s, lambda),
            pole_damping_ratio: lerp(self.pole_damping_ratio, rhs.pole_damping_ratio, lambda),
        }
    }
}

/// A scalar schedule knot for a bending/slosh filter array.
#[derive(Clone, Debug, PartialEq)]
pub struct BendingFilterNode {
    /// Stable node identifier, usually a gain-schedule knot id.
    pub node_id: String,
    /// Scalar schedule knot value (Mach, qbar, mass fraction, or time).
    pub schedule_value: f64,
    /// Filter sections associated with this knot.
    pub sections: Vec<BendingFilterSectionSpec>,
}

/// Offline-synthesised scalar bending/slosh filter schedule.
#[derive(Clone, Debug, PartialEq)]
pub struct BendingFilterTable {
    /// Schedule variable used by `nodes[*].schedule_value`.
    pub variable: ScheduleVariable,
    /// Interpolation law for physical frequencies and damping ratios.
    pub interp: GainInterp,
    /// Controller sample interval, s.
    pub sample_interval_s: f64,
    /// Strictly ascending filter nodes.
    pub nodes: Vec<BendingFilterNode>,
}

impl Table for BendingFilterTable {
    const NAME: &'static str = "filters.bending_filter_schedule";

    fn validate(&self) -> Result<(), String> {
        if !self.sample_interval_s.is_finite() || self.sample_interval_s <= 0.0 {
            return Err("sample_interval_s must be finite and positive".to_string());
        }
        if self.nodes.len() < 2 || self.nodes.len() > 64 {
            return Err("nodes length must be in 2..=64".to_string());
        }
        let section_count = self
            .nodes
            .first()
            .map(|node| node.sections.len())
            .unwrap_or(0);
        if section_count == 0 || section_count > 16 {
            return Err("sections per node must be in 1..=16".to_string());
        }

        let mut previous = f64::NEG_INFINITY;
        for node in &self.nodes {
            if node.node_id.trim().is_empty() {
                return Err("node_id must be non-empty".to_string());
            }
            if !node.schedule_value.is_finite() {
                return Err("schedule_value must be finite".to_string());
            }
            if node.schedule_value <= previous {
                return Err("nodes must be strictly sorted by schedule_value".to_string());
            }
            previous = node.schedule_value;
            if node.sections.len() != section_count {
                return Err("section count must match at every bending filter node".to_string());
            }
            for section in &node.sections {
                section.validate(self.sample_interval_s)?;
            }
        }
        Ok(())
    }
}

/// Offline-synthesised scalar gain schedule for coupled LQR/LQG gains.
#[derive(Clone, Debug, PartialEq)]
pub struct GainScheduleTable {
    /// Schedule variable used by `nodes[*].schedule_value`.
    pub variable: ScheduleVariable,
    /// Interpolation law.
    pub interp: GainInterp,
    /// State dimension shared by every node.
    pub states: usize,
    /// Control dimension shared by every node.
    pub controls: usize,
    /// Measurement dimension for optional observer gains.
    pub measurements: usize,
    /// Strictly ascending schedule nodes.
    pub nodes: Vec<CoupledGainNode>,
}

impl Table for GainScheduleTable {
    const NAME: &'static str = "lqr.gain_schedule";

    fn validate(&self) -> Result<(), String> {
        if self.states == 0 || self.states > 16 {
            return Err("states must be in 1..=16".to_string());
        }
        if self.controls == 0 || self.controls > 16 {
            return Err("controls must be in 1..=16".to_string());
        }
        if self.measurements > 16 {
            return Err("measurements must be in 0..=16".to_string());
        }
        if self.nodes.len() < 2 || self.nodes.len() > 64 {
            return Err("nodes length must be in 2..=64".to_string());
        }

        let mut previous = f64::NEG_INFINITY;
        let mut observer_presence: Option<bool> = None;
        for node in &self.nodes {
            node.validate()?;
            if node.states != self.states
                || node.controls != self.controls
                || node.measurements != self.measurements
            {
                return Err("node dimensions must match the schedule table".to_string());
            }
            if node.schedule_value <= previous {
                return Err("nodes must be strictly sorted by schedule_value".to_string());
            }
            previous = node.schedule_value;

            let has_observer = node.observer_gain_row_major.is_some();
            match observer_presence {
                Some(expected) if expected != has_observer => {
                    return Err(
                        "observer gains must be present either at every node or no node"
                            .to_string(),
                    );
                }
                None => observer_presence = Some(has_observer),
                _ => {}
            }

            let tag = node.flight_condition_tag.to_ascii_lowercase();
            for forbidden in ["target", "range", "aimpoint", "latitude", "longitude"] {
                if tag.contains(forbidden) {
                    return Err("flight_condition_tag must not carry target vocabulary".to_string());
                }
            }
        }
        Ok(())
    }
}

/// One scheduled imported robust controller state-space node.
///
/// Matrices are row-major and use the discrete controller convention
/// `x_c[k+1] = A_c x_c[k] + B_c y[k]`, `u[k] = C_c x_c[k] + D_c y[k]`.
/// The node carries only flight-condition metadata; targeting, range,
/// aimpoint, latitude, and longitude vocabulary are rejected by the
/// containing [`RobustControllerTable`].
#[derive(Clone, Debug, PartialEq)]
pub struct RobustControllerNode {
    /// Stable node identifier, usually a robust-controller schedule knot id.
    pub node_id: String,
    /// Scalar schedule knot value (Mach, qbar, mass fraction, or time).
    pub schedule_value: f64,
    /// Non-targeting flight-condition tag such as `mach_qbar_mass`.
    pub flight_condition_tag: String,
    /// Controller state dimension.
    pub states: usize,
    /// Measurement/input dimension.
    pub measurements: usize,
    /// Command/output dimension.
    pub controls: usize,
    /// Row-major state matrix `A_c`, shaped `states × states`.
    pub a_row_major: Vec<f64>,
    /// Row-major input matrix `B_c`, shaped `states × measurements`.
    pub b_row_major: Vec<f64>,
    /// Row-major output matrix `C_c`, shaped `controls × states`.
    pub c_row_major: Vec<f64>,
    /// Row-major feedthrough matrix `D_c`, shaped `controls × measurements`.
    pub d_row_major: Vec<f64>,
}

impl RobustControllerNode {
    fn validate(&self) -> Result<(), String> {
        if self.node_id.trim().is_empty() {
            return Err("node_id must be non-empty".to_string());
        }
        if self.flight_condition_tag.trim().is_empty() {
            return Err("flight_condition_tag must be non-empty".to_string());
        }
        if !self.schedule_value.is_finite() {
            return Err("schedule_value must be finite".to_string());
        }
        if self.states == 0 || self.states > 16 {
            return Err("states must be in 1..=16".to_string());
        }
        if self.measurements == 0 || self.measurements > 16 {
            return Err("measurements must be in 1..=16".to_string());
        }
        if self.controls == 0 || self.controls > 16 {
            return Err("controls must be in 1..=16".to_string());
        }
        validate_row_major("a_row_major", self.states * self.states, &self.a_row_major)?;
        validate_row_major(
            "b_row_major",
            self.states * self.measurements,
            &self.b_row_major,
        )?;
        validate_row_major(
            "c_row_major",
            self.controls * self.states,
            &self.c_row_major,
        )?;
        validate_row_major(
            "d_row_major",
            self.controls * self.measurements,
            &self.d_row_major,
        )?;
        Ok(())
    }
}

/// Offline-synthesised robust controller schedule.
///
/// This table is the FC-local import substrate for reduced discrete
/// state-space controllers produced by external robust-control tools. It
/// records provenance metadata and validates dimensions before activation,
/// but it does not synthesize the controller or claim robustness margins by
/// itself.
#[derive(Clone, Debug, PartialEq)]
pub struct RobustControllerTable {
    /// Stable controller identifier.
    pub controller_id: String,
    /// Source artifact or tolerance-table identifier.
    pub source_id: String,
    /// SHA-256 digest of the source artifact, lowercase or uppercase hex.
    pub source_sha256_hex: String,
    /// Schedule variable used by `nodes[*].schedule_value`.
    pub variable: ScheduleVariable,
    /// Interpolation law for state-space matrices.
    pub interp: GainInterp,
    /// Controller state dimension shared by every node.
    pub states: usize,
    /// Measurement/input dimension shared by every node.
    pub measurements: usize,
    /// Command/output dimension shared by every node.
    pub controls: usize,
    /// Strictly ascending robust-controller nodes.
    pub nodes: Vec<RobustControllerNode>,
}

impl Table for RobustControllerTable {
    const NAME: &'static str = "robust.state_space_controller";

    fn validate(&self) -> Result<(), String> {
        if self.controller_id.trim().is_empty() {
            return Err("controller_id must be non-empty".to_string());
        }
        if self.source_id.trim().is_empty() {
            return Err("source_id must be non-empty".to_string());
        }
        if !is_sha256_hex(&self.source_sha256_hex) {
            return Err("source_sha256_hex must be 64 hex characters".to_string());
        }
        if self.states == 0 || self.states > 16 {
            return Err("states must be in 1..=16".to_string());
        }
        if self.measurements == 0 || self.measurements > 16 {
            return Err("measurements must be in 1..=16".to_string());
        }
        if self.controls == 0 || self.controls > 16 {
            return Err("controls must be in 1..=16".to_string());
        }
        if self.nodes.len() < 2 || self.nodes.len() > 64 {
            return Err("nodes length must be in 2..=64".to_string());
        }

        let mut previous = f64::NEG_INFINITY;
        for node in &self.nodes {
            node.validate()?;
            if node.states != self.states
                || node.measurements != self.measurements
                || node.controls != self.controls
            {
                return Err("node dimensions must match the robust controller table".to_string());
            }
            if node.schedule_value <= previous {
                return Err("nodes must be strictly sorted by schedule_value".to_string());
            }
            previous = node.schedule_value;

            let tag = node.flight_condition_tag.to_ascii_lowercase();
            for forbidden in ["target", "range", "aimpoint", "latitude", "longitude"] {
                if tag.contains(forbidden) {
                    return Err("flight_condition_tag must not carry target vocabulary".to_string());
                }
            }
        }
        Ok(())
    }
}

fn validate_row_major(_label: &'static str, expected: usize, values: &[f64]) -> Result<(), String> {
    if values.len() != expected {
        return Err("row-major matrix length must match declared dimensions".to_string());
    }
    if values.iter().any(|value| !value.is_finite()) {
        return Err("row-major matrix entries must be finite".to_string());
    }
    Ok(())
}

fn is_sha256_hex(value: &str) -> bool {
    value.len() == 64 && value.chars().all(|ch| ch.is_ascii_hexdigit())
}

/// Interpolate a bending/slosh filter schedule and discretize the
/// resulting physical sections.
///
/// # Errors
///
/// Returns a validation string when the table is invalid or the
/// interpolated physical section generates unstable poles.
pub fn interpolate_bending_filter_table(
    table: &BendingFilterTable,
    schedule_value: f64,
) -> Result<Vec<BiquadSection>, String> {
    table.validate()?;
    if !schedule_value.is_finite() {
        return Err("schedule_value must be finite".to_string());
    }

    let (lower_index, upper_index, lambda) = bending_schedule_window(table, schedule_value)
        .ok_or_else(|| "empty schedule".to_string())?;
    let lower = &table.nodes[lower_index];
    let upper = &table.nodes[upper_index];
    let mut sections = Vec::with_capacity(lower.sections.len());
    for (lower_section, upper_section) in lower.sections.iter().zip(&upper.sections) {
        let physical = lower_section.interpolate(upper_section, lambda);
        sections.push(physical.discretize(table.sample_interval_s)?);
    }
    Ok(sections)
}

fn bending_schedule_window(
    table: &BendingFilterTable,
    schedule_value: f64,
) -> Option<(usize, usize, f64)> {
    if schedule_value <= table.nodes.first()?.schedule_value {
        return Some((0, 0, 0.0));
    }
    let last_index = table.nodes.len() - 1;
    if schedule_value >= table.nodes[last_index].schedule_value {
        return Some((last_index, last_index, 0.0));
    }
    for index in 0..last_index {
        let lower = table.nodes[index].schedule_value;
        let upper = table.nodes[index + 1].schedule_value;
        if schedule_value >= lower && schedule_value <= upper {
            let raw_lambda = (schedule_value - lower) / (upper - lower);
            return Some((
                index,
                index + 1,
                interpolation_fraction(table.interp, raw_lambda),
            ));
        }
    }
    None
}

fn interpolation_fraction(interp: GainInterp, raw_lambda: f64) -> f64 {
    match interp {
        GainInterp::Linear => raw_lambda,
        GainInterp::Smoothstep => raw_lambda * raw_lambda * (3.0 - 2.0 * raw_lambda),
    }
}

fn lerp(lower: f64, upper: f64, lambda: f64) -> f64 {
    lower + lambda * (upper - lower)
}

impl Tables {
    /// Constructs an empty table registry.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Declares a table and seeds it with its initial active value.
    /// The initial value is validated before being installed.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::DuplicateTable`] if the table was already
    /// declared, [`TableError::ValidationFailed`] if the initial value
    /// fails validation.
    pub fn declare<T: Table>(&mut self, initial: T) -> Result<(), TableError> {
        let id = TypeId::of::<T>();
        if self.cells.contains_key(&id) {
            return Err(TableError::DuplicateTable { table: T::NAME });
        }
        if let Err(reason) = initial.validate() {
            return Err(TableError::ValidationFailed {
                table: T::NAME,
                reason,
            });
        }
        self.cells.insert(
            id,
            TableCell {
                name: T::NAME,
                active: Box::new(initial),
                pending: None,
            },
        );
        Ok(())
    }

    /// Stages a candidate value in the pending buffer. Validation
    /// runs synchronously; on failure the pending buffer is cleared
    /// and the original error is returned.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared, [`TableError::ValidationFailed`] if validation fails.
    pub fn stage<T: Table>(&mut self, candidate: T) -> Result<(), TableError> {
        let id = TypeId::of::<T>();
        let cell = self
            .cells
            .get_mut(&id)
            .ok_or(TableError::UnknownTable { table: T::NAME })?;
        if let Err(reason) = candidate.validate() {
            cell.pending = None;
            return Err(TableError::ValidationFailed {
                table: T::NAME,
                reason,
            });
        }
        cell.pending = Some(Box::new(candidate));
        Ok(())
    }

    /// Atomically activates the staged pending buffer. Has no effect
    /// if no pending value is staged.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared.
    pub fn activate<T: Table>(&mut self) -> Result<bool, TableError> {
        let id = TypeId::of::<T>();
        let cell = self
            .cells
            .get_mut(&id)
            .ok_or(TableError::UnknownTable { table: T::NAME })?;
        if let Some(pending) = cell.pending.take() {
            cell.active = pending;
            Ok(true)
        } else {
            Ok(false)
        }
    }

    /// Stages and activates in one call. Convenience for boot-time
    /// loads that have no separate review step.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared, [`TableError::ValidationFailed`] if validation fails.
    pub fn replace<T: Table>(&mut self, candidate: T) -> Result<(), TableError> {
        self.stage::<T>(candidate)?;
        self.activate::<T>()?;
        Ok(())
    }

    /// Returns a reference to the active value of a table.
    ///
    /// # Errors
    ///
    /// Returns [`TableError::UnknownTable`] if the table was not
    /// declared.
    pub fn active<T: Table>(&self) -> Result<&T, TableError> {
        let id = TypeId::of::<T>();
        let cell = self
            .cells
            .get(&id)
            .ok_or(TableError::UnknownTable { table: T::NAME })?;
        cell.active
            .downcast_ref::<T>()
            .ok_or(TableError::UnknownTable { table: T::NAME })
    }

    /// Returns descriptors for every declared table in declaration
    /// order.
    #[must_use]
    pub fn tables(&self) -> Vec<TableInfo> {
        self.cells
            .values()
            .map(|c| TableInfo {
                name: c.name,
                has_pending: c.pending.is_some(),
            })
            .collect()
    }
}

impl std::fmt::Debug for Tables {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Tables")
            .field("tables", &self.tables())
            .finish()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use toml::Value;

    const BENDING_FILTER_TUSTIN_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/bending-filter-tustin-tolerance-v1.toml");

    #[derive(Clone, Debug, PartialEq)]
    struct GainSchedule {
        kp: Vec<f64>,
    }

    impl Table for GainSchedule {
        const NAME: &'static str = "test.gain_schedule";
        fn validate(&self) -> Result<(), String> {
            if self.kp.is_empty() {
                return Err("kp must be non-empty".to_string());
            }
            if self.kp.iter().any(|k| !k.is_finite() || *k < 0.0) {
                return Err("kp entries must be finite and non-negative".to_string());
            }
            Ok(())
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

    fn toml_str<'a>(value: &'a toml::map::Map<String, Value>, key: &str) -> &'a str {
        value
            .get(key)
            .and_then(Value::as_str)
            .unwrap_or_else(|| panic!("missing TOML string {key}"))
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

    fn assert_close_array(actual: &[f64], expected: &[f64], epsilon: f64, label: &str) {
        assert_eq!(actual.len(), expected.len(), "{label} length");
        for (index, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (*actual - *expected).abs() <= epsilon,
                "{label}[{index}] actual {actual:e}, expected {expected:e}, epsilon {epsilon:e}"
            );
        }
    }

    fn bending_table_from_fixture(data: &Value) -> BendingFilterTable {
        let schedule = toml_table(data, "schedule");
        let variable = match toml_str(schedule, "variable") {
            "Mach" => ScheduleVariable::Mach,
            other => panic!("unsupported bending schedule variable {other}"),
        };
        let interp = match toml_str(schedule, "interp") {
            "Linear" => GainInterp::Linear,
            "Smoothstep" => GainInterp::Smoothstep,
            other => panic!("unsupported bending schedule interp {other}"),
        };
        let nodes = schedule
            .get("node")
            .and_then(Value::as_array)
            .expect("bending filter nodes")
            .iter()
            .map(|node| {
                let node = node.as_table().expect("node table");
                let sections = node
                    .get("section")
                    .and_then(Value::as_array)
                    .expect("bending filter node sections")
                    .iter()
                    .map(|section| {
                        let section = section.as_table().expect("section table");
                        BendingFilterSectionSpec {
                            mode_frequency_rad_s: toml_f64(section, "mode_frequency_rad_s"),
                            zero_frequency_rad_s: toml_f64(section, "zero_frequency_rad_s"),
                            zero_damping_ratio: toml_f64(section, "zero_damping_ratio"),
                            pole_frequency_rad_s: toml_f64(section, "pole_frequency_rad_s"),
                            pole_damping_ratio: toml_f64(section, "pole_damping_ratio"),
                        }
                    })
                    .collect();
                BendingFilterNode {
                    node_id: toml_str(node, "node_id").to_string(),
                    schedule_value: toml_f64(node, "schedule_value"),
                    sections,
                }
            })
            .collect();
        BendingFilterTable {
            variable,
            interp,
            sample_interval_s: toml_f64(schedule, "dt_s"),
            nodes,
        }
    }

    #[test]
    fn declare_and_read_active() {
        let mut tables = Tables::new();
        tables.declare(GainSchedule { kp: vec![1.0, 2.0] }).unwrap();
        let active = tables.active::<GainSchedule>().unwrap();
        assert_eq!(active.kp, vec![1.0, 2.0]);
    }

    #[test]
    fn stage_then_activate() {
        let mut tables = Tables::new();
        tables.declare(GainSchedule { kp: vec![1.0] }).unwrap();
        tables.stage(GainSchedule { kp: vec![3.0, 4.0] }).unwrap();
        // Active still old until activate.
        assert_eq!(tables.active::<GainSchedule>().unwrap().kp, vec![1.0]);
        let activated = tables.activate::<GainSchedule>().unwrap();
        assert!(activated);
        assert_eq!(tables.active::<GainSchedule>().unwrap().kp, vec![3.0, 4.0]);
    }

    #[test]
    fn validation_failure_keeps_active_intact() {
        let mut tables = Tables::new();
        tables.declare(GainSchedule { kp: vec![1.0] }).unwrap();
        let err = tables.stage(GainSchedule { kp: vec![] }).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
        // Active untouched.
        assert_eq!(tables.active::<GainSchedule>().unwrap().kp, vec![1.0]);
    }

    #[test]
    fn declare_with_invalid_initial_rejected() {
        let mut tables = Tables::new();
        let err = tables.declare(GainSchedule { kp: vec![] }).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
    }

    #[test]
    fn coupled_gain_node_declares_and_rejects_bad_shapes() {
        let mut tables = Tables::new();
        let initial = CoupledGainNode {
            node_id: "mach0p8_qbar20".to_string(),
            schedule_value: 0.8,
            flight_condition_tag: "mach_qbar_mass".to_string(),
            states: 3,
            controls: 2,
            measurements: 0,
            gain_row_major: vec![1.0, 0.1, -0.2, 0.0, 0.5, 0.3],
            observer_gain_row_major: None,
        };
        tables
            .declare(initial.clone())
            .expect("coupled gain node is valid");
        assert_eq!(
            tables.active::<CoupledGainNode>().unwrap().gain_row_major,
            initial.gain_row_major
        );

        let err = tables
            .stage(CoupledGainNode {
                node_id: "bad".to_string(),
                schedule_value: 1.0,
                flight_condition_tag: "mach_qbar_mass".to_string(),
                states: 3,
                controls: 2,
                measurements: 0,
                gain_row_major: vec![1.0; 5],
                observer_gain_row_major: None,
            })
            .unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
    }

    #[test]
    fn gain_schedule_table_declares_and_rejects_bad_nodes() {
        let initial = GainScheduleTable {
            variable: ScheduleVariable::Mach,
            interp: GainInterp::Linear,
            states: 2,
            controls: 1,
            measurements: 1,
            nodes: vec![
                CoupledGainNode {
                    node_id: "low".to_string(),
                    schedule_value: 0.0,
                    flight_condition_tag: "mach".to_string(),
                    states: 2,
                    controls: 1,
                    measurements: 1,
                    gain_row_major: vec![1.0, 0.2],
                    observer_gain_row_major: Some(vec![0.1, 0.2]),
                },
                CoupledGainNode {
                    node_id: "high".to_string(),
                    schedule_value: 1.0,
                    flight_condition_tag: "mach".to_string(),
                    states: 2,
                    controls: 1,
                    measurements: 1,
                    gain_row_major: vec![2.0, 0.4],
                    observer_gain_row_major: Some(vec![0.3, 0.4]),
                },
            ],
        };
        let mut tables = Tables::new();
        tables
            .declare(initial.clone())
            .expect("gain schedule table is valid");
        assert_eq!(
            tables.active::<GainScheduleTable>().unwrap().nodes[1].schedule_value,
            1.0
        );

        let mut bad = initial;
        bad.nodes[1].schedule_value = 0.0;
        let err = tables.stage(bad).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
    }

    #[test]
    fn robust_controller_table_declares_and_rejects_bad_nodes() {
        let initial = RobustControllerTable {
            controller_id: "robust_ascent_controller".to_string(),
            source_id: "synthetic_hinf_reduced_order_v1".to_string(),
            source_sha256_hex: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
                .to_string(),
            variable: ScheduleVariable::DynamicPressurePa,
            interp: GainInterp::Linear,
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
        };
        let mut tables = Tables::new();
        tables
            .declare(initial.clone())
            .expect("robust controller table is valid");
        assert_eq!(
            tables.active::<RobustControllerTable>().unwrap().source_id,
            "synthetic_hinf_reduced_order_v1"
        );

        let mut bad = initial.clone();
        bad.nodes[1].a_row_major.pop();
        let err = tables.stage(bad).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));

        let mut bad = initial.clone();
        bad.nodes[0].flight_condition_tag = "target_range".to_string();
        let err = tables.stage(bad).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));

        let mut bad = initial;
        bad.source_sha256_hex = "not-a-sha".to_string();
        let err = tables.stage(bad).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
    }

    #[test]
    fn bending_filter_tustin_tolerance_table_matches_schedule_and_stability() {
        let data: Value = toml::from_str(BENDING_FILTER_TUSTIN_TOLERANCE_TABLE)
            .expect("bending filter table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_bending_filter_tustin_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let coefficient_epsilon = toml_f64(tolerance, "absolute_coefficient");
        let magnitude_epsilon = toml_f64(tolerance, "absolute_magnitude");
        let table = bending_table_from_fixture(&data);

        let mut registry = Tables::new();
        registry
            .declare(table.clone())
            .expect("bending filter table is valid");
        assert_eq!(
            registry.active::<BendingFilterTable>().unwrap().nodes.len(),
            2
        );

        let query = toml_table(&data, "query");
        let sections = interpolate_bending_filter_table(&table, toml_f64(query, "schedule_value"))
            .expect("query discretizes");
        assert_eq!(sections.len(), 1);
        assert!(sections[0].has_stable_poles());
        assert_close_array(
            &sections[0].b,
            &toml_f64_array(query, "expected_b"),
            coefficient_epsilon,
            "query b",
        );
        assert_close_array(
            &sections[0].a,
            &toml_f64_array(query, "expected_a"),
            coefficient_epsilon,
            "query a",
        );

        let margin = toml_table(&data, "margin_sweep");
        let schedule_values = toml_f64_array(margin, "sample_values");
        let mode_frequencies = toml_f64_array(margin, "expected_mode_frequencies_rad_s");
        let mode_magnitudes = toml_f64_array(margin, "expected_mode_magnitudes");
        let min_attenuation_db = toml_f64(margin, "min_notch_attenuation_db");
        for (index, schedule_value) in schedule_values.iter().enumerate() {
            let sections = interpolate_bending_filter_table(&table, *schedule_value)
                .expect("sample discretizes");
            let magnitude = sections[0]
                .magnitude_at_rad_s(mode_frequencies[index], table.sample_interval_s)
                .expect("mode response evaluates");
            assert!(
                (magnitude - mode_magnitudes[index]).abs() <= magnitude_epsilon,
                "mode magnitude[{index}] was {magnitude}"
            );
            let attenuation_db = -20.0 * magnitude.max(1.0e-12).log10();
            assert!(
                attenuation_db >= min_attenuation_db,
                "attenuation[{index}] was {attenuation_db} dB"
            );
        }

        let mut bad = table;
        bad.nodes[0].sections[0].pole_damping_ratio = 0.0;
        let err = registry.stage(bad).unwrap_err();
        assert!(matches!(err, TableError::ValidationFailed { .. }));
    }
}
