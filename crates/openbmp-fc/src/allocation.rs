//! Control-allocation primitives.
//!
//! The actuator-channel mapping in `mixer.rs` assumes a 1:1
//! correspondence between the autopilot's semantic channel surface
//! (`aileron / elevator / rudder / body_flap`) and a single
//! `direct_torque` effector per channel. With more than one
//! effector contributing to a body axis — a 4-thruster RCS bank
//! with redundant roll authority, or a multi-engine cluster sharing
//! pitch/yaw — that 1:1 assumption breaks. The allocators consume
//! the per-axis command demand from
//! [`crate::topics::ActuatorCommand`] and emit a per-effector
//! [`crate::topics::EffectorCommandSet`] honouring each effector's
//! symmetric box bound.
//!
//! # Scope
//!
//! - The legacy per-axis allocator surface remains `direct_torque`
//!   only: each legacy effector contributes to exactly one body axis.
//!   The allocator's job per axis is then "distribute the demand
//!   across the assigned effectors"; inter-axis priority is reported
//!   but does not change the outcome on a single-axis-effector
//!   scenario.
//! - The dense-`B` substrate accepts arbitrary virtual command rows and
//!   effector columns, including coupled gimbaled clusters. Dense WLS consumes
//!   semantic three-axis actuator demand; dense INCA consumes desired/measured
//!   pseudo-control from `autopilot.pseudo_control_cmd` when the `indi` feature
//!   is enabled. Both are wired through the v3 scenario schema and mixer.
//! - Legacy symmetric box bounds: every per-axis effector's `[min, max]` is
//!   required to satisfy `max == −min` and `max > 0`. Asymmetric
//!   limits are rejected at allocator construction. Dense-`B` bounds
//!   accept asymmetric position limits plus per-tick rate tightening.
//! - The pseudo-inverse allocator named in
//!   `openbmp_scenario::FcAutopilotAllocationKind::PseudoInverse`
//!   is consumed for the current `direct_torque` surface. It solves
//!   the diagonal body-axis case with a bounded weighted
//!   pseudo-inverse; a later slice can extend the same public strategy
//!   to coupled `G_eff` rows when the scenario schema can express
//!   them.
//!
//! # Math (per axis, simple single-axis-effector case)
//!
//! For axis `a` with assigned effectors group(a) = `{ e₁, …, eₙ }`,
//! per-effector symmetric bound `Lᵢ = max_abs(eᵢ)`, and demand
//! `τ_a` from the autopilot:
//!
//! 1. Apply any phase-authority mask from the mixer. Disallowed
//!    effectors receive an explicit zero command and do not contribute
//!    to capacity.
//! 2. `Σ L = Σᵢ Lᵢ` over the allowed effectors. If `Σ L = 0` the axis
//!    has no authority and every effector receives zero (saturation
//!    reported when demand is non-zero).
//! 3. Else if `|τ_a| ≤ Σ L`: distribute proportionally —
//!    `uᵢ = τ_a · Lᵢ / Σ L`. No saturation.
//! 4. Else: clamp at total capacity, every allowed effector gets
//!    `sign(τ_a) · Lᵢ`. Saturation reported on this axis.
//!
//! The output is sign-consistent across the group (every effector
//! pulls in the same direction as `τ_a`), and the per-effector
//! magnitude never exceeds `Lᵢ`. Saturation reporting per axis
//! preserves the autopilot's `saturated` bit so downstream FDIR
//! can react.
//!
//! # Determinism
//!
//! The arithmetic is purely scalar with no floating-point reductions
//! that depend on iteration order; running the allocator twice on
//! identical inputs produces bit-identical outputs.

#[cfg(not(feature = "std"))]
use alloc::vec;
use std::{collections::BTreeSet, vec::Vec};

use thiserror::Error;

use openbmp_core::EffectorId;

/// Body-frame axis label used by the allocator. Mirrors
/// `openbmp_scenario::TorqueAxis` one-for-one (the scenario layer
/// owns the parser type; this is the FC-local equivalent so the
/// `openbmp-fc` crate does not depend on `openbmp-scenario`).
#[derive(Copy, Clone, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum BodyAxis {
    /// Roll body-frame moment about +x.
    Roll,
    /// Pitch body-frame moment about +y.
    Pitch,
    /// Yaw body-frame moment about +z.
    Yaw,
}

impl BodyAxis {
    /// Iterates the three axes in `[Roll, Pitch, Yaw]` order.
    #[must_use]
    pub const fn all() -> [Self; 3] {
        [Self::Roll, Self::Pitch, Self::Yaw]
    }

    /// `0` for roll, `1` for pitch, `2` for yaw.
    #[must_use]
    pub const fn index(self) -> usize {
        match self {
            Self::Roll => 0,
            Self::Pitch => 1,
            Self::Yaw => 2,
        }
    }
}

/// Errors raised by allocator constructors.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum AllocatorError {
    /// Same effector id appears twice in the assignment list.
    #[error("allocator: duplicate effector {effector_id:?}")]
    DuplicateEffector {
        /// Offending effector id.
        effector_id: EffectorId,
    },
    /// An effector's `max_abs` (symmetric box bound) is non-positive
    /// or non-finite.
    #[error("allocator: effector {effector_id:?} max_abs must be > 0 and finite; got {value}")]
    NonPositiveOrNonFiniteLimit {
        /// Offending effector id.
        effector_id: EffectorId,
        /// Offending value.
        value: f64,
    },
    /// Asymmetric box bound — `min ≠ −max`.
    #[error(
        "allocator: effector {effector_id:?} requires symmetric limits; \
         got min = {min}, max = {max}"
    )]
    AsymmetricLimits {
        /// Offending effector id.
        effector_id: EffectorId,
        /// Configured `min`.
        min: f64,
        /// Configured `max`.
        max: f64,
    },
    /// `axis_priority` does not contain exactly the three body axes
    /// (each appearing exactly once).
    #[error("allocator: axis_priority must list each of [roll, pitch, yaw] exactly once")]
    InvalidAxisPriority,
    /// The configured priority list selects an axis that has no
    /// assigned effectors. Permitted only for axes that the
    /// autopilot never commands, so the allocator emits a fail-
    /// closed error instead.
    #[error("allocator: axis {axis:?} appears in axis_priority but has no assigned effectors")]
    NoEffectorForAxis {
        /// Offending axis.
        axis: BodyAxis,
    },
    /// Dense effectiveness matrix has inconsistent dimensions.
    #[error(
        "allocator: dense matrix shape is invalid: rows={rows}, effectors={effectors}, coefficients={coefficients}"
    )]
    InvalidDenseMatrixShape {
        /// Virtual-command row count.
        rows: usize,
        /// Effector-column count.
        effectors: usize,
        /// Row-major coefficient count.
        coefficients: usize,
    },
    /// Dense allocator vector dimensions do not match the matrix.
    #[error("allocator: dense WLS dimension mismatch: {reason}")]
    DenseDimensionMismatch {
        /// Human-readable reason.
        reason: &'static str,
    },
    /// Dense allocator field is non-finite.
    #[error("allocator: dense WLS field {field} must be finite; got {value}")]
    DenseNonFinite {
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// Dense allocator field must be positive and finite.
    #[error("allocator: dense WLS field {field} must be positive and finite; got {value}")]
    DenseNonPositive {
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
    },
    /// Dense allocator bounds are invalid for one effector.
    #[error(
        "allocator: dense WLS bounds for {effector_id:?} are invalid: min={min}, max={max}, rate_limit={rate_limit}"
    )]
    InvalidDenseBounds {
        /// Effector id.
        effector_id: EffectorId,
        /// Position lower bound.
        min: f64,
        /// Position upper bound.
        max: f64,
        /// Per-second rate limit.
        rate_limit: f64,
    },
    /// Dense allocator rate tightening produced an empty interval.
    #[error(
        "allocator: dense WLS tightened bounds for {effector_id:?} are empty: lower={lower}, upper={upper}"
    )]
    DenseRateBoundsInfeasible {
        /// Effector id.
        effector_id: EffectorId,
        /// Tightened lower bound.
        lower: f64,
        /// Tightened upper bound.
        upper: f64,
    },
    /// Dense WLS normal equation solve became singular.
    #[error("allocator: dense WLS normal-equation system is singular")]
    DenseSingularSystem,
    /// Dense WLS active-set loop exceeded its deterministic iteration cap.
    #[error("allocator: dense WLS active-set iteration limit exceeded")]
    DenseIterationLimit,
}

/// Per-effector axis assignment consumed by the allocator.
///
/// Each `direct_torque` effector contributes to exactly one body
/// axis with a symmetric box bound. The runner derives this list
/// from `vehicle.assembly.effectors` at scenario load.
#[derive(Clone, Debug, PartialEq)]
pub struct EffectorAxisAssignment {
    /// Effector id (matches the kernel-side rack entry).
    pub effector_id: EffectorId,
    /// Body axis driven by this effector.
    pub axis: BodyAxis,
    /// Symmetric box bound. The allocator output satisfies
    /// `|u| ≤ max_abs`. Must be `> 0` and finite.
    pub max_abs: f64,
}

/// Prioritised redistributed allocator.
///
/// Built from a body-axis priority permutation and a list of
/// per-effector axis assignments; constructed once at scenario
/// load and reused on every tick.
///
/// Construction validates the assignment list and the priority
/// permutation; per-tick `allocate` is a pure scalar arithmetic
/// pass that runs in `O(N)` for `N` effectors. The allocator does
/// not own any mutable state — the same instance can be reused
/// across all ticks.
#[derive(Clone, Debug)]
pub struct PrioritisedRedistributedAllocator {
    axis_priority: [BodyAxis; 3],
    /// Axis-grouped capacity table: `per_axis[axis_index] = vec of
    /// (effector_id, max_abs)`. Indexed via [`BodyAxis::index`].
    per_axis: [Vec<(EffectorId, f64)>; 3],
}

/// Weighted pseudo-inverse allocator over direct body-axis effectors.
///
/// For the current scenario surface every `direct_torque` effector
/// contributes to exactly one body axis, so the allocation matrix is a
/// sparse selector matrix. The unconstrained solution for each axis is:
///
/// `u_i = tau_axis * L_i^2 / sum(L_j^2)`
///
/// where `L_i` is the effector's symmetric command limit. This is the
/// weighted pseudo-inverse solution that minimizes normalized command
/// effort. Bounds are enforced with a deterministic active-set pass:
/// any tentative command outside its limit is pinned, removed from the
/// free set, and the residual demand is redistributed over the
/// remaining free effectors.
#[derive(Clone, Debug)]
pub struct WeightedPseudoInverseAllocator {
    per_axis: [Vec<(EffectorId, f64)>; 3],
}

/// Dense control-effectiveness matrix for weighted-least-squares allocation.
///
/// The matrix has `rows` virtual-command rows and one column per effector,
/// stored in deterministic row-major order.
#[derive(Clone, Debug, PartialEq)]
pub struct EffectivenessMatrix {
    rows: usize,
    row_major: Vec<f64>,
    effectors: Vec<EffectorId>,
}

/// Position and rate bounds for one dense WLS allocation step.
#[derive(Clone, Debug, PartialEq)]
pub struct EffectorBounds {
    /// Hard lower position bound for each effector command.
    pub u_min_pos: Vec<f64>,
    /// Hard upper position bound for each effector command.
    pub u_max_pos: Vec<f64>,
    /// Absolute rate limit in command-units per second for each effector.
    pub rate_limit: Vec<f64>,
}

/// Dense-`B` Härkegård-style bounded weighted-least-squares allocator.
#[derive(Clone, Debug)]
pub struct WlsActiveSetAllocator {
    matrix: EffectivenessMatrix,
    w_v: Vec<f64>,
    w_u: Vec<f64>,
    gamma: f64,
    u_pref: Vec<f64>,
    attainable_tolerance: f64,
}

/// Stateful dense WLS control allocator for the mixer-facing three-axis surface.
#[derive(Clone, Debug)]
pub struct DenseWlsControlAllocator {
    allocator: WlsActiveSetAllocator,
    bounds: EffectorBounds,
    u_prev: Vec<f64>,
    dt_s: f64,
}

/// Stateful dense INCA allocator for INDI pseudo-control rows.
#[cfg(feature = "indi")]
#[derive(Clone, Debug)]
pub struct DenseIncaControlAllocator {
    controller: crate::indi::DenseIncaController,
}

/// Result of one dense WLS allocation step.
#[derive(Clone, Debug, PartialEq)]
pub struct WlsAllocationOutput {
    /// Per-effector commands in the matrix column order.
    pub u: Vec<(EffectorId, f64)>,
    /// `true` where the returned command lies on a tightened bound.
    pub active_at_bound: Vec<bool>,
    /// Virtual-command residual `v - B u`, in row order.
    pub virtual_residual: Vec<f64>,
    /// `true` when every virtual residual is inside the allocator tolerance.
    pub attainable: bool,
    /// Active-set iterations used by this allocation.
    pub iterations: u16,
}

/// Runtime control allocator selected by scenario I-load.
#[derive(Clone, Debug)]
pub enum ControlAllocator {
    /// Harkegard-style prioritized redistributed allocator.
    PrioritisedRedistributed(PrioritisedRedistributedAllocator),
    /// Bounded weighted pseudo-inverse allocator.
    WeightedPseudoInverse(WeightedPseudoInverseAllocator),
    /// Dense bounded WLS allocator over `[roll, pitch, yaw]` virtual rows.
    DenseWls(DenseWlsControlAllocator),
    /// Dense INCA allocator over `[roll, pitch, yaw]` pseudo-control rows.
    #[cfg(feature = "indi")]
    DenseInca(DenseIncaControlAllocator),
}

/// Result of one allocator step.
#[derive(Clone, Debug, PartialEq)]
pub struct AllocationOutput {
    /// Per-effector torque commands, ordered as the allocator's
    /// internal flat assignment list (axis-grouped). Caller
    /// iterates and routes each entry to the matching effector.
    pub commands: Vec<(EffectorId, f64)>,
    /// `true` for axes whose demand exceeded the allocator's total
    /// capacity. Bit `i` corresponds to [`BodyAxis::index`] `i`.
    pub saturated_axes: [bool; 3],
}

impl From<PrioritisedRedistributedAllocator> for ControlAllocator {
    fn from(value: PrioritisedRedistributedAllocator) -> Self {
        Self::PrioritisedRedistributed(value)
    }
}

impl From<WeightedPseudoInverseAllocator> for ControlAllocator {
    fn from(value: WeightedPseudoInverseAllocator) -> Self {
        Self::WeightedPseudoInverse(value)
    }
}

impl From<DenseWlsControlAllocator> for ControlAllocator {
    fn from(value: DenseWlsControlAllocator) -> Self {
        Self::DenseWls(value)
    }
}

#[cfg(feature = "indi")]
impl From<DenseIncaControlAllocator> for ControlAllocator {
    fn from(value: DenseIncaControlAllocator) -> Self {
        Self::DenseInca(value)
    }
}

impl ControlAllocator {
    /// Allocates a per-axis demand without phase masking.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when the dense WLS variant fails closed.
    pub fn allocate(&mut self, demand: [f64; 3]) -> Result<AllocationOutput, AllocatorError> {
        match self {
            Self::PrioritisedRedistributed(allocator) => Ok(allocator.allocate(demand)),
            Self::WeightedPseudoInverse(allocator) => Ok(allocator.allocate(demand)),
            Self::DenseWls(allocator) => allocator.allocate(demand),
            #[cfg(feature = "indi")]
            Self::DenseInca(_) => Err(AllocatorError::DenseDimensionMismatch {
                reason: "dense_inca allocator requires pseudo-control demand",
            }),
        }
    }

    /// Allocates a per-axis demand while removing disallowed
    /// effectors from the available authority set.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when the dense WLS variant fails closed.
    pub fn allocate_with_allowed_effectors(
        &mut self,
        demand: [f64; 3],
        allowed_effectors: &[EffectorId],
    ) -> Result<AllocationOutput, AllocatorError> {
        match self {
            Self::PrioritisedRedistributed(allocator) => {
                Ok(allocator.allocate_with_allowed_effectors(demand, allowed_effectors))
            }
            Self::WeightedPseudoInverse(allocator) => {
                Ok(allocator.allocate_with_allowed_effectors(demand, allowed_effectors))
            }
            Self::DenseWls(allocator) => {
                allocator.allocate_with_allowed_effectors(demand, allowed_effectors)
            }
            #[cfg(feature = "indi")]
            Self::DenseInca(_) => Err(AllocatorError::DenseDimensionMismatch {
                reason: "dense_inca allocator requires pseudo-control demand",
            }),
        }
    }

    /// Returns `true` when this allocator consumes desired/measured
    /// pseudo-control instead of direct semantic actuator demand.
    #[must_use]
    pub const fn uses_pseudo_control(&self) -> bool {
        match self {
            #[cfg(feature = "indi")]
            Self::DenseInca(_) => true,
            _ => false,
        }
    }

    /// Allocates from desired/measured angular-acceleration pseudo-control
    /// while removing disallowed effectors from the available authority set.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when the dense-INCA path is not selected or
    /// the underlying dense WLS solve fails closed.
    pub fn allocate_pseudo_control_with_allowed_effectors(
        &mut self,
        _omega_dot_des_rad_s2: [f64; 3],
        _omega_dot_meas_rad_s2: [f64; 3],
        _allowed_effectors: &[EffectorId],
    ) -> Result<AllocationOutput, AllocatorError> {
        match self {
            #[cfg(feature = "indi")]
            Self::DenseInca(allocator) => allocator.allocate_with_allowed_effectors(
                _omega_dot_des_rad_s2,
                _omega_dot_meas_rad_s2,
                _allowed_effectors,
            ),
            _ => Err(AllocatorError::DenseDimensionMismatch {
                reason: "pseudo-control allocation requires dense_inca allocator",
            }),
        }
    }
}

impl EffectivenessMatrix {
    /// Build a dense effectiveness matrix from row-major coefficients.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when the dimensions are empty or inconsistent,
    /// an effector id is duplicated, or any coefficient is non-finite.
    pub fn new(
        rows: usize,
        effectors: Vec<EffectorId>,
        row_major: Vec<f64>,
    ) -> Result<Self, AllocatorError> {
        let columns = effectors.len();
        if rows == 0 || columns == 0 || row_major.len() != rows * columns {
            return Err(AllocatorError::InvalidDenseMatrixShape {
                rows,
                effectors: columns,
                coefficients: row_major.len(),
            });
        }
        let mut by_id: BTreeSet<EffectorId> = BTreeSet::new();
        for effector_id in &effectors {
            if !by_id.insert(*effector_id) {
                return Err(AllocatorError::DuplicateEffector {
                    effector_id: *effector_id,
                });
            }
        }
        for value in &row_major {
            require_dense_finite(*value, "EffectivenessMatrix::row_major")?;
        }
        Ok(Self {
            rows,
            row_major,
            effectors,
        })
    }

    /// Number of virtual-command rows.
    #[must_use]
    pub const fn rows(&self) -> usize {
        self.rows
    }

    /// Number of effector columns.
    #[must_use]
    pub fn columns(&self) -> usize {
        self.effectors.len()
    }

    /// Effector ids in matrix column order.
    #[must_use]
    pub fn effectors(&self) -> &[EffectorId] {
        &self.effectors
    }

    /// Return one matrix coefficient, or `None` when the index is outside the matrix.
    #[must_use]
    pub fn coefficient(&self, row: usize, column: usize) -> Option<f64> {
        (row < self.rows && column < self.columns())
            .then(|| self.coefficient_unchecked(row, column))
    }

    fn coefficient_unchecked(&self, row: usize, column: usize) -> f64 {
        self.row_major[row * self.columns() + column]
    }

    fn multiply(&self, u: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.rows];
        for (row, out_value) in out.iter_mut().enumerate() {
            let mut sum = 0.0;
            for (column, command) in u.iter().enumerate() {
                sum += self.coefficient_unchecked(row, column) * command;
            }
            *out_value = sum;
        }
        out
    }
}

impl EffectorBounds {
    fn require_valid(
        &self,
        effectors: &[EffectorId],
        u_prev: &[f64],
        dt_s: f64,
    ) -> Result<(Vec<f64>, Vec<f64>), AllocatorError> {
        let columns = effectors.len();
        if self.u_min_pos.len() != columns
            || self.u_max_pos.len() != columns
            || self.rate_limit.len() != columns
            || u_prev.len() != columns
        {
            return Err(AllocatorError::DenseDimensionMismatch {
                reason: "bounds and previous command lengths must match effector columns",
            });
        }
        require_dense_positive(dt_s, "dt_s")?;

        let mut lower = vec![0.0; columns];
        let mut upper = vec![0.0; columns];
        for column in 0..columns {
            let min = self.u_min_pos[column];
            let max = self.u_max_pos[column];
            let rate = self.rate_limit[column];
            require_dense_finite(u_prev[column], "u_prev")?;
            if !min.is_finite() || !max.is_finite() || !rate.is_finite() || rate < 0.0 || min > max
            {
                return Err(AllocatorError::InvalidDenseBounds {
                    effector_id: effectors[column],
                    min,
                    max,
                    rate_limit: rate,
                });
            }
            lower[column] = min.max(u_prev[column] - rate * dt_s);
            upper[column] = max.min(u_prev[column] + rate * dt_s);
            if lower[column] > upper[column] {
                return Err(AllocatorError::DenseRateBoundsInfeasible {
                    effector_id: effectors[column],
                    lower: lower[column],
                    upper: upper[column],
                });
            }
        }
        Ok((lower, upper))
    }
}

impl WlsActiveSetAllocator {
    /// Build a dense bounded-WLS allocator.
    ///
    /// `w_v` has one positive finite entry per virtual-command row. `w_u`
    /// and `u_pref` have one entry per effector column. `gamma` is the
    /// positive finite primary-objective weight.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when dimensions mismatch or any scalar is
    /// non-finite / outside its allowed range.
    pub fn new(
        matrix: EffectivenessMatrix,
        w_v: Vec<f64>,
        w_u: Vec<f64>,
        gamma: f64,
        u_pref: Vec<f64>,
    ) -> Result<Self, AllocatorError> {
        if w_v.len() != matrix.rows() {
            return Err(AllocatorError::DenseDimensionMismatch {
                reason: "w_v length must match dense matrix rows",
            });
        }
        if w_u.len() != matrix.columns() || u_pref.len() != matrix.columns() {
            return Err(AllocatorError::DenseDimensionMismatch {
                reason: "w_u and u_pref lengths must match dense matrix columns",
            });
        }
        for value in &w_v {
            require_dense_positive(*value, "w_v")?;
        }
        for value in &w_u {
            require_dense_positive(*value, "w_u")?;
        }
        for value in &u_pref {
            require_dense_finite(*value, "u_pref")?;
        }
        require_dense_positive(gamma, "gamma")?;

        Ok(Self {
            matrix,
            w_v,
            w_u,
            gamma,
            u_pref,
            attainable_tolerance: 1.0e-9,
        })
    }

    /// Borrow the dense effectiveness matrix.
    #[must_use]
    pub const fn matrix(&self) -> &EffectivenessMatrix {
        &self.matrix
    }

    /// Allocate one dense WLS step with per-tick rate-tightened bounds.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when vector dimensions mismatch, inputs are
    /// non-finite, bounds are invalid, or the active-set solve cannot converge.
    pub fn allocate(
        &self,
        v: &[f64],
        u_prev: &[f64],
        bounds: &EffectorBounds,
        dt_s: f64,
    ) -> Result<WlsAllocationOutput, AllocatorError> {
        if v.len() != self.matrix.rows() {
            return Err(AllocatorError::DenseDimensionMismatch {
                reason: "virtual command length must match dense matrix rows",
            });
        }
        for value in v {
            require_dense_finite(*value, "virtual command")?;
        }
        let (lower, upper) = bounds.require_valid(self.matrix.effectors(), u_prev, dt_s)?;
        let (commands, active_at_bound, iterations) = self.solve_active_set(v, &lower, &upper)?;
        let virtual_residual = self.virtual_residual(v, &commands);
        let attainable = virtual_residual
            .iter()
            .all(|value| value.abs() <= self.attainable_tolerance);
        let u = self
            .matrix
            .effectors()
            .iter()
            .zip(commands)
            .map(|(id, command)| (*id, command))
            .collect();

        Ok(WlsAllocationOutput {
            u,
            active_at_bound,
            virtual_residual,
            attainable,
            iterations,
        })
    }

    fn solve_active_set(
        &self,
        v: &[f64],
        lower: &[f64],
        upper: &[f64],
    ) -> Result<(Vec<f64>, Vec<bool>, u16), AllocatorError> {
        let columns = self.matrix.columns();
        let mut active: Vec<Option<f64>> = vec![None; columns];
        let mut commands = vec![0.0; columns];
        let max_iterations = columns.saturating_mul(2).saturating_add(1);
        let mut iterations = 0usize;

        loop {
            if iterations >= max_iterations {
                return Err(AllocatorError::DenseIterationLimit);
            }
            iterations += 1;
            self.solve_free_commands(v, &active, &mut commands)?;

            if let Some((column, value)) =
                first_dense_bound_violation(&commands, &active, lower, upper)
            {
                active[column] = Some(value);
                continue;
            }

            let gradient = self.gradient(v, &commands);
            if let Some(column) = first_dense_active_release(&active, lower, upper, &gradient) {
                active[column] = None;
                continue;
            }

            let active_at_bound = active.iter().map(Option::is_some).collect();
            return Ok((commands, active_at_bound, iterations as u16));
        }
    }

    fn solve_free_commands(
        &self,
        v: &[f64],
        active: &[Option<f64>],
        commands: &mut [f64],
    ) -> Result<(), AllocatorError> {
        let free: Vec<usize> = active
            .iter()
            .enumerate()
            .filter_map(|(column, fixed)| fixed.is_none().then_some(column))
            .collect();
        for (column, fixed) in active.iter().enumerate() {
            if let Some(value) = fixed {
                commands[column] = *value;
            }
        }
        if free.is_empty() {
            return Ok(());
        }

        let virtual_rows = self.matrix.rows();
        let equation_rows = virtual_rows + free.len();
        let mut augmented = vec![0.0; equation_rows * free.len()];
        let mut rhs = vec![0.0; equation_rows];

        let gamma_sqrt = <f64 as num_traits::Float>::sqrt(self.gamma);
        for (row, target) in v.iter().enumerate().take(virtual_rows) {
            let mut fixed_sum = 0.0;
            for (column, fixed) in active.iter().enumerate() {
                if let Some(value) = fixed {
                    fixed_sum += self.matrix.coefficient_unchecked(row, column) * value;
                }
            }
            let row_weight = self.w_v[row];
            let residual_target = *target - fixed_sum;
            rhs[row] = row_weight * residual_target;
            for (i_free, column_i) in free.iter().enumerate() {
                let b_i = self.matrix.coefficient_unchecked(row, *column_i);
                augmented[row * free.len() + i_free] = row_weight * b_i;
            }
        }

        for (i_free, column) in free.iter().enumerate() {
            let row = virtual_rows + i_free;
            let effort_weight = self.w_u[*column] / gamma_sqrt;
            augmented[row * free.len() + i_free] = effort_weight;
            rhs[row] = effort_weight * self.u_pref[*column];
        }

        let solution = solve_dense_least_squares(equation_rows, free.len(), augmented, rhs)?;
        for (column, value) in free.iter().zip(solution) {
            require_dense_finite(value, "dense WLS solution")?;
            commands[*column] = value;
        }
        Ok(())
    }

    fn virtual_residual(&self, v: &[f64], commands: &[f64]) -> Vec<f64> {
        let achieved = self.matrix.multiply(commands);
        v.iter()
            .zip(achieved)
            .map(|(target, actual)| target - actual)
            .collect()
    }

    fn gradient(&self, v: &[f64], commands: &[f64]) -> Vec<f64> {
        let achieved = self.matrix.multiply(commands);
        let mut gradient = vec![0.0; self.matrix.columns()];
        for (row, (actual, target)) in achieved.iter().zip(v).enumerate() {
            let weighted_residual = self.gamma * self.w_v[row] * self.w_v[row] * (actual - target);
            for (column, gradient_value) in gradient.iter_mut().enumerate() {
                *gradient_value +=
                    weighted_residual * self.matrix.coefficient_unchecked(row, column);
            }
        }
        for (column, gradient_value) in gradient.iter_mut().enumerate() {
            let effort_weight = self.w_u[column] * self.w_u[column];
            *gradient_value += effort_weight * (commands[column] - self.u_pref[column]);
        }
        gradient
    }
}

impl DenseWlsControlAllocator {
    /// Build a mixer-facing dense WLS allocator for `[roll, pitch, yaw]` demand rows.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when the dense WLS matrix does not have exactly
    /// three rows, when bounds/previous commands have inconsistent dimensions, or
    /// when `dt_s` is not positive and finite.
    pub fn new(
        allocator: WlsActiveSetAllocator,
        bounds: EffectorBounds,
        initial_u: Vec<f64>,
        dt_s: f64,
    ) -> Result<Self, AllocatorError> {
        if allocator.matrix().rows() != 3 {
            return Err(AllocatorError::DenseDimensionMismatch {
                reason: "mixer-facing dense WLS allocator must have [roll, pitch, yaw] rows",
            });
        }
        bounds.require_valid(allocator.matrix().effectors(), &initial_u, dt_s)?;
        Ok(Self {
            allocator,
            bounds,
            u_prev: initial_u,
            dt_s,
        })
    }

    /// Allocate without phase masking.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] if the dense WLS solve fails.
    pub fn allocate(&mut self, demand: [f64; 3]) -> Result<AllocationOutput, AllocatorError> {
        self.allocate_inner(demand, None)
    }

    /// Allocate while treating disallowed effectors as fixed-zero commands.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] if the dense WLS solve fails.
    pub fn allocate_with_allowed_effectors(
        &mut self,
        demand: [f64; 3],
        allowed_effectors: &[EffectorId],
    ) -> Result<AllocationOutput, AllocatorError> {
        self.allocate_inner(demand, Some(allowed_effectors))
    }

    fn allocate_inner(
        &mut self,
        demand: [f64; 3],
        allowed_effectors: Option<&[EffectorId]>,
    ) -> Result<AllocationOutput, AllocatorError> {
        let mut step_bounds = self.bounds.clone();
        let mut step_prev = self.u_prev.clone();
        if let Some(allowed) = allowed_effectors {
            for (column, effector_id) in self.allocator.matrix().effectors().iter().enumerate() {
                if !allowed.contains(effector_id) {
                    step_bounds.u_min_pos[column] = 0.0;
                    step_bounds.u_max_pos[column] = 0.0;
                    step_bounds.rate_limit[column] = 0.0;
                    step_prev[column] = 0.0;
                }
            }
        }

        let output = self
            .allocator
            .allocate(&demand, &step_prev, &step_bounds, self.dt_s)?;
        let commands: Vec<(EffectorId, f64)> = output.u;
        self.u_prev = commands.iter().map(|(_, command)| *command).collect();

        let mut saturated_axes = [false; 3];
        for (axis, residual) in output.virtual_residual.iter().enumerate().take(3) {
            saturated_axes[axis] = residual.abs() > 1.0e-9;
        }
        Ok(AllocationOutput {
            commands,
            saturated_axes,
        })
    }
}

#[cfg(feature = "indi")]
impl DenseIncaControlAllocator {
    /// Build a mixer-facing dense INCA allocator for `[roll, pitch, yaw]`
    /// pseudo-control rows.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] when the dense matrix does not have exactly
    /// three rows, when bounds/previous commands have inconsistent dimensions,
    /// or when `dt_s` is not positive and finite.
    pub fn new(
        allocator: WlsActiveSetAllocator,
        bounds: EffectorBounds,
        initial_u: Vec<f64>,
        dt_s: f64,
    ) -> Result<Self, AllocatorError> {
        if allocator.matrix().rows() != 3 {
            return Err(AllocatorError::DenseDimensionMismatch {
                reason: "mixer-facing dense INCA allocator must have [roll, pitch, yaw] rows",
            });
        }
        crate::indi::DenseIncaController::new(allocator, bounds, initial_u, dt_s)
            .map(|controller| Self { controller })
            .map_err(dense_inca_error_to_allocator_error)
    }

    /// Allocate from desired/measured angular acceleration without phase masking.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] if the dense INCA solve fails.
    pub fn allocate(
        &mut self,
        omega_dot_des_rad_s2: [f64; 3],
        omega_dot_meas_rad_s2: [f64; 3],
    ) -> Result<AllocationOutput, AllocatorError> {
        self.controller
            .step(&omega_dot_des_rad_s2, &omega_dot_meas_rad_s2)
            .map(dense_inca_output_to_allocation)
            .map_err(dense_inca_error_to_allocator_error)
    }

    /// Allocate from desired/measured angular acceleration while treating
    /// disallowed effectors as fixed-zero commands.
    ///
    /// # Errors
    ///
    /// Returns [`AllocatorError`] if the dense INCA solve fails.
    pub fn allocate_with_allowed_effectors(
        &mut self,
        omega_dot_des_rad_s2: [f64; 3],
        omega_dot_meas_rad_s2: [f64; 3],
        allowed_effectors: &[EffectorId],
    ) -> Result<AllocationOutput, AllocatorError> {
        self.controller
            .step_with_allowed_effectors(
                &omega_dot_des_rad_s2,
                &omega_dot_meas_rad_s2,
                allowed_effectors,
            )
            .map(dense_inca_output_to_allocation)
            .map_err(dense_inca_error_to_allocator_error)
    }
}

#[cfg(feature = "indi")]
fn dense_inca_output_to_allocation(output: crate::indi::DenseIncaOutput) -> AllocationOutput {
    let mut saturated_axes = [false; 3];
    for (axis, deficit) in output.pseudo_control_deficit.iter().enumerate().take(3) {
        saturated_axes[axis] = deficit.abs() > 1.0e-9;
    }
    if output.saturated && !saturated_axes.iter().any(|saturated| *saturated) {
        saturated_axes = [true; 3];
    }
    AllocationOutput {
        commands: output.u,
        saturated_axes,
    }
}

#[cfg(feature = "indi")]
fn dense_inca_error_to_allocator_error(err: crate::indi::DenseIncaError) -> AllocatorError {
    match err {
        crate::indi::DenseIncaError::Allocator(err) => err,
        crate::indi::DenseIncaError::DimensionMismatch { reason } => {
            AllocatorError::DenseDimensionMismatch { reason }
        }
        crate::indi::DenseIncaError::NonFinite { field, value } => {
            AllocatorError::DenseNonFinite { field, value }
        }
        crate::indi::DenseIncaError::NonPositive { field, value } => {
            AllocatorError::DenseNonPositive { field, value }
        }
    }
}

impl PrioritisedRedistributedAllocator {
    /// Build an allocator from a priority permutation and an
    /// assignment list.
    ///
    /// # Errors
    ///
    /// Returns an [`AllocatorError`] when:
    /// - `axis_priority` is not a permutation of `[Roll, Pitch, Yaw]`;
    /// - any effector appears twice in `assignments`;
    /// - any `max_abs` is non-positive or non-finite;
    /// - any axis named in `axis_priority` has no matching
    ///   effector (fail closed: a priority entry signals "I expect
    ///   to allocate here").
    pub fn new(
        axis_priority: [BodyAxis; 3],
        assignments: Vec<EffectorAxisAssignment>,
    ) -> Result<Self, AllocatorError> {
        // Permutation check.
        let mut seen = [false; 3];
        for axis in axis_priority {
            let i = axis.index();
            if seen[i] {
                return Err(AllocatorError::InvalidAxisPriority);
            }
            seen[i] = true;
        }
        if !seen.iter().all(|&v| v) {
            return Err(AllocatorError::InvalidAxisPriority);
        }
        // Duplicate-effector check.
        let mut by_id: BTreeSet<EffectorId> = BTreeSet::new();
        for assignment in &assignments {
            if !by_id.insert(assignment.effector_id) {
                return Err(AllocatorError::DuplicateEffector {
                    effector_id: assignment.effector_id,
                });
            }
            if !assignment.max_abs.is_finite() || assignment.max_abs <= 0.0 {
                return Err(AllocatorError::NonPositiveOrNonFiniteLimit {
                    effector_id: assignment.effector_id,
                    value: assignment.max_abs,
                });
            }
        }
        // Group by axis.
        let mut per_axis: [Vec<(EffectorId, f64)>; 3] = Default::default();
        for assignment in assignments {
            per_axis[assignment.axis.index()].push((assignment.effector_id, assignment.max_abs));
        }
        // Each priority axis must have at least one effector.
        for axis in axis_priority {
            if per_axis[axis.index()].is_empty() {
                return Err(AllocatorError::NoEffectorForAxis { axis });
            }
        }
        Ok(Self {
            axis_priority,
            per_axis,
        })
    }

    /// Distribute the per-axis torque demand across the assigned
    /// effectors. `demand[i]` is the autopilot's requested torque
    /// on axis `BodyAxis::index() == i` (`0 = roll, 1 = pitch, 2 =
    /// yaw`).
    #[must_use]
    pub fn allocate(&self, demand: [f64; 3]) -> AllocationOutput {
        self.allocate_inner(demand, None)
    }

    /// Distribute demand while treating only `allowed_effectors` as
    /// available capacity. Effectors omitted from `allowed_effectors`
    /// are still emitted with zero commands so downstream racks receive
    /// an explicit authority withdrawal for that tick.
    #[must_use]
    pub fn allocate_with_allowed_effectors(
        &self,
        demand: [f64; 3],
        allowed_effectors: &[EffectorId],
    ) -> AllocationOutput {
        self.allocate_inner(demand, Some(allowed_effectors))
    }

    fn allocate_inner(
        &self,
        demand: [f64; 3],
        allowed_effectors: Option<&[EffectorId]>,
    ) -> AllocationOutput {
        let mut commands: Vec<(EffectorId, f64)> = Vec::new();
        let mut saturated_axes = [false; 3];
        // Visit axes in priority order. With single-axis effectors
        // priority does not change the outcome (axes are
        // independent), but the iteration order keeps the output
        // deterministic and prepares the API for a future slice
        // that adds inter-axis coupling.
        for axis in self.axis_priority {
            let group = &self.per_axis[axis.index()];
            let axis_demand = demand[axis.index()];
            let total_capacity: f64 = group
                .iter()
                .filter(|(id, _)| effector_allowed(*id, allowed_effectors))
                .map(|(_, l)| *l)
                .sum();
            if total_capacity <= 0.0 {
                // No allowed authority on this axis.
                for (id, _) in group {
                    commands.push((*id, 0.0));
                }
                saturated_axes[axis.index()] = axis_demand != 0.0;
                continue;
            }
            if axis_demand.abs() <= total_capacity {
                // Proportional split — each effector takes its
                // share of the demand. Sign of `u_i` matches sign
                // of `axis_demand`.
                for (id, capacity) in group {
                    let u = if effector_allowed(*id, allowed_effectors) {
                        axis_demand * capacity / total_capacity
                    } else {
                        0.0
                    };
                    commands.push((*id, u));
                }
            } else {
                // Saturated — every effector pulls at its limit in
                // the direction of the demand.
                let sign = axis_demand.signum();
                for (id, capacity) in group {
                    let u = if effector_allowed(*id, allowed_effectors) {
                        sign * capacity
                    } else {
                        0.0
                    };
                    commands.push((*id, u));
                }
                saturated_axes[axis.index()] = true;
            }
        }
        AllocationOutput {
            commands,
            saturated_axes,
        }
    }

    /// Read-only access to the configured axis priority.
    #[must_use]
    pub fn axis_priority(&self) -> [BodyAxis; 3] {
        self.axis_priority
    }
}

fn effector_allowed(effector_id: EffectorId, allowed_effectors: Option<&[EffectorId]>) -> bool {
    allowed_effectors.is_none_or(|allowed| allowed.contains(&effector_id))
}

impl WeightedPseudoInverseAllocator {
    /// Build a bounded weighted pseudo-inverse allocator from a list of
    /// direct-axis assignments.
    ///
    /// # Errors
    ///
    /// Returns an [`AllocatorError`] when:
    /// - any effector appears twice in `assignments`;
    /// - any `max_abs` is non-positive or non-finite;
    /// - any body axis has no matching effector.
    pub fn new(assignments: Vec<EffectorAxisAssignment>) -> Result<Self, AllocatorError> {
        let mut by_id: BTreeSet<EffectorId> = BTreeSet::new();
        for assignment in &assignments {
            if !by_id.insert(assignment.effector_id) {
                return Err(AllocatorError::DuplicateEffector {
                    effector_id: assignment.effector_id,
                });
            }
            if !assignment.max_abs.is_finite() || assignment.max_abs <= 0.0 {
                return Err(AllocatorError::NonPositiveOrNonFiniteLimit {
                    effector_id: assignment.effector_id,
                    value: assignment.max_abs,
                });
            }
        }

        let mut per_axis: [Vec<(EffectorId, f64)>; 3] = Default::default();
        for assignment in assignments {
            per_axis[assignment.axis.index()].push((assignment.effector_id, assignment.max_abs));
        }
        for axis in BodyAxis::all() {
            if per_axis[axis.index()].is_empty() {
                return Err(AllocatorError::NoEffectorForAxis { axis });
            }
        }
        Ok(Self { per_axis })
    }

    /// Allocates a per-axis command demand without phase masking.
    #[must_use]
    pub fn allocate(&self, demand: [f64; 3]) -> AllocationOutput {
        self.allocate_inner(demand, None)
    }

    /// Allocates a per-axis demand while treating only
    /// `allowed_effectors` as available authority.
    #[must_use]
    pub fn allocate_with_allowed_effectors(
        &self,
        demand: [f64; 3],
        allowed_effectors: &[EffectorId],
    ) -> AllocationOutput {
        self.allocate_inner(demand, Some(allowed_effectors))
    }

    fn allocate_inner(
        &self,
        demand: [f64; 3],
        allowed_effectors: Option<&[EffectorId]>,
    ) -> AllocationOutput {
        let mut commands: Vec<(EffectorId, f64)> = Vec::new();
        let mut saturated_axes = [false; 3];
        for axis in BodyAxis::all() {
            let group = &self.per_axis[axis.index()];
            let axis_demand = demand[axis.index()];
            let (axis_commands, saturated) =
                allocate_axis_weighted_pseudo_inverse(group, axis_demand, allowed_effectors);
            commands.extend(axis_commands);
            saturated_axes[axis.index()] = saturated;
        }
        AllocationOutput {
            commands,
            saturated_axes,
        }
    }
}

fn allocate_axis_weighted_pseudo_inverse(
    group: &[(EffectorId, f64)],
    demand: f64,
    allowed_effectors: Option<&[EffectorId]>,
) -> (Vec<(EffectorId, f64)>, bool) {
    let mut commands = vec![0.0; group.len()];
    let mut free: Vec<usize> = group
        .iter()
        .enumerate()
        .filter_map(|(index, (id, _))| effector_allowed(*id, allowed_effectors).then_some(index))
        .collect();
    let mut residual = demand;
    const EPS: f64 = 1.0e-12;

    while !free.is_empty() {
        let denom: f64 = free
            .iter()
            .map(|index| {
                let limit = group[*index].1;
                limit * limit
            })
            .sum();
        if denom <= EPS {
            break;
        }

        let mut clamped_any = false;
        let mut next_free: Vec<usize> = Vec::new();
        for index in &free {
            let limit = group[*index].1;
            let tentative = residual * limit * limit / denom;
            if tentative.abs() > limit + EPS {
                let pinned = tentative.signum() * limit;
                commands[*index] += pinned;
                residual -= pinned;
                clamped_any = true;
            } else {
                next_free.push(*index);
            }
        }
        if !clamped_any {
            for index in &free {
                let limit = group[*index].1;
                let u = residual * limit * limit / denom;
                commands[*index] += u;
            }
            residual = 0.0;
            break;
        }
        free = next_free;
    }

    let saturated = residual.abs() > EPS;
    (
        group
            .iter()
            .zip(commands)
            .map(|((id, _), command)| (*id, command))
            .collect(),
        saturated,
    )
}

fn require_dense_finite(value: f64, field: &'static str) -> Result<(), AllocatorError> {
    if !value.is_finite() {
        return Err(AllocatorError::DenseNonFinite { field, value });
    }
    Ok(())
}

fn require_dense_positive(value: f64, field: &'static str) -> Result<(), AllocatorError> {
    require_dense_finite(value, field)?;
    if value <= 0.0 {
        return Err(AllocatorError::DenseNonPositive { field, value });
    }
    Ok(())
}

fn first_dense_bound_violation(
    commands: &[f64],
    active: &[Option<f64>],
    lower: &[f64],
    upper: &[f64],
) -> Option<(usize, f64)> {
    const BOUND_EPS: f64 = 1.0e-12;
    for (column, command) in commands.iter().enumerate() {
        if active[column].is_some() {
            continue;
        }
        if *command < lower[column] - BOUND_EPS {
            return Some((column, lower[column]));
        }
        if *command > upper[column] + BOUND_EPS {
            return Some((column, upper[column]));
        }
    }
    None
}

fn first_dense_active_release(
    active: &[Option<f64>],
    lower: &[f64],
    upper: &[f64],
    gradient: &[f64],
) -> Option<usize> {
    const KKT_EPS: f64 = 1.0e-10;
    for (column, fixed) in active.iter().enumerate() {
        if let Some(value) = fixed {
            if (upper[column] - lower[column]).abs() <= KKT_EPS {
                continue;
            }
            if (*value - lower[column]).abs() <= KKT_EPS && gradient[column] < -KKT_EPS {
                return Some(column);
            }
            if (*value - upper[column]).abs() <= KKT_EPS && gradient[column] > KKT_EPS {
                return Some(column);
            }
        }
    }
    None
}

fn solve_dense_least_squares(
    rows: usize,
    columns: usize,
    mut row_major: Vec<f64>,
    mut rhs: Vec<f64>,
) -> Result<Vec<f64>, AllocatorError> {
    if columns == 0 {
        return Ok(Vec::new());
    }
    if rows < columns || row_major.len() != rows * columns || rhs.len() != rows {
        return Err(AllocatorError::DenseDimensionMismatch {
            reason: "least-squares matrix dimensions are inconsistent",
        });
    }

    for pivot in 0..columns {
        let mut norm_sq = 0.0;
        for row in pivot..rows {
            let value = row_major[row * columns + pivot];
            norm_sq += value * value;
        }
        let norm = <f64 as num_traits::Float>::sqrt(norm_sq);
        if norm <= 1.0e-14 {
            return Err(AllocatorError::DenseSingularSystem);
        }

        let diagonal = row_major[pivot * columns + pivot];
        let alpha = if diagonal >= 0.0 { -norm } else { norm };
        let householder_len = rows - pivot;
        let mut householder = vec![0.0; householder_len];
        householder[0] = diagonal - alpha;
        for row in (pivot + 1)..rows {
            householder[row - pivot] = row_major[row * columns + pivot];
        }

        let beta_denominator: f64 = householder.iter().map(|value| value * value).sum();
        if beta_denominator <= 1.0e-28 {
            continue;
        }
        let beta = 2.0 / beta_denominator;

        for column in pivot..columns {
            let mut projection = 0.0;
            for (offset, householder_value) in householder.iter().enumerate() {
                projection += householder_value * row_major[(pivot + offset) * columns + column];
            }
            let scale = beta * projection;
            for (offset, householder_value) in householder.iter().enumerate() {
                row_major[(pivot + offset) * columns + column] -= scale * householder_value;
            }
        }

        let mut rhs_projection = 0.0;
        for (offset, householder_value) in householder.iter().enumerate() {
            rhs_projection += householder_value * rhs[pivot + offset];
        }
        let rhs_scale = beta * rhs_projection;
        for (offset, householder_value) in householder.iter().enumerate() {
            rhs[pivot + offset] -= rhs_scale * householder_value;
        }

        row_major[pivot * columns + pivot] = alpha;
        for row in (pivot + 1)..rows {
            row_major[row * columns + pivot] = 0.0;
        }
    }

    let mut solution = vec![0.0; columns];
    for row in (0..columns).rev() {
        let mut sum = rhs[row];
        for column in (row + 1)..columns {
            sum -= row_major[row * columns + column] * solution[column];
        }
        let diagonal = row_major[row * columns + row];
        if diagonal.abs() <= 1.0e-14 {
            return Err(AllocatorError::DenseSingularSystem);
        }
        solution[row] = sum / diagonal;
    }
    Ok(solution)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]
mod tests {
    use std::collections::BTreeMap;

    use approx::assert_abs_diff_eq;
    use toml::Value;

    use super::*;

    const QCAT_WLS_TOLERANCE_TABLE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/gnc/qcat-wls-allocation-tolerance-v1.toml"
    ));

    fn eid(path: &str) -> EffectorId {
        EffectorId::from_path(path)
    }

    fn nominal_assignments() -> Vec<EffectorAxisAssignment> {
        vec![
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-a"),
                axis: BodyAxis::Roll,
                max_abs: 0.2,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-b"),
                axis: BodyAxis::Roll,
                max_abs: 0.2,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.pitch"),
                axis: BodyAxis::Pitch,
                max_abs: 0.35,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.yaw"),
                axis: BodyAxis::Yaw,
                max_abs: 0.35,
            },
        ]
    }

    fn nominal_priority() -> [BodyAxis; 3] {
        [BodyAxis::Roll, BodyAxis::Pitch, BodyAxis::Yaw]
    }

    #[test]
    fn constructor_accepts_nominal_assignments() {
        PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
            .expect("nominal allocator constructs");
    }

    #[test]
    fn constructor_rejects_axis_priority_with_repeats() {
        let bad = [BodyAxis::Roll, BodyAxis::Roll, BodyAxis::Yaw];
        assert!(matches!(
            PrioritisedRedistributedAllocator::new(bad, nominal_assignments()),
            Err(AllocatorError::InvalidAxisPriority)
        ));
    }

    #[test]
    fn constructor_rejects_duplicate_effector_ids() {
        let mut a = nominal_assignments();
        a.push(EffectorAxisAssignment {
            effector_id: a[0].effector_id,
            axis: BodyAxis::Pitch,
            max_abs: 0.1,
        });
        assert!(matches!(
            PrioritisedRedistributedAllocator::new(nominal_priority(), a),
            Err(AllocatorError::DuplicateEffector { .. })
        ));
    }

    #[test]
    fn constructor_rejects_non_positive_max_abs() {
        let mut a = nominal_assignments();
        a[0].max_abs = 0.0;
        assert!(matches!(
            PrioritisedRedistributedAllocator::new(nominal_priority(), a),
            Err(AllocatorError::NonPositiveOrNonFiniteLimit { .. })
        ));
        let mut a = nominal_assignments();
        a[1].max_abs = -0.1;
        assert!(matches!(
            PrioritisedRedistributedAllocator::new(nominal_priority(), a),
            Err(AllocatorError::NonPositiveOrNonFiniteLimit { .. })
        ));
        let mut a = nominal_assignments();
        a[2].max_abs = f64::NAN;
        assert!(matches!(
            PrioritisedRedistributedAllocator::new(nominal_priority(), a),
            Err(AllocatorError::NonPositiveOrNonFiniteLimit { .. })
        ));
    }

    #[test]
    fn constructor_rejects_axis_priority_referencing_unassigned_axis() {
        // Drop the yaw effector; priority still names it.
        let mut a = nominal_assignments();
        a.retain(|x| x.axis != BodyAxis::Yaw);
        assert!(matches!(
            PrioritisedRedistributedAllocator::new(nominal_priority(), a),
            Err(AllocatorError::NoEffectorForAxis {
                axis: BodyAxis::Yaw
            })
        ));
    }

    #[test]
    fn allocate_zero_demand_emits_zeros_with_no_saturation() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        let out = alloc.allocate([0.0, 0.0, 0.0]);
        for (_, u) in &out.commands {
            assert_eq!(*u, 0.0);
        }
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    #[test]
    fn allocate_splits_roll_demand_proportionally_across_two_effectors() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        // Roll demand of 0.3, two roll effectors with equal limit.
        // Each should get half.
        let out = alloc.allocate([0.3, 0.0, 0.0]);
        let mut roll_a = None;
        let mut roll_b = None;
        for (id, u) in &out.commands {
            if *id == eid("vehicle.assembly.effectors.roll-a") {
                roll_a = Some(*u);
            } else if *id == eid("vehicle.assembly.effectors.roll-b") {
                roll_b = Some(*u);
            }
        }
        let ra = roll_a.expect("roll-a present");
        let rb = roll_b.expect("roll-b present");
        assert_abs_diff_eq!(ra, 0.15, epsilon = 1.0e-12);
        assert_abs_diff_eq!(rb, 0.15, epsilon = 1.0e-12);
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    #[test]
    fn allocate_splits_proportionally_to_unequal_capacities() {
        let assignments = vec![
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-a"),
                axis: BodyAxis::Roll,
                max_abs: 0.1,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-b"),
                axis: BodyAxis::Roll,
                max_abs: 0.3,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.pitch"),
                axis: BodyAxis::Pitch,
                max_abs: 0.35,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.yaw"),
                axis: BodyAxis::Yaw,
                max_abs: 0.35,
            },
        ];
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), assignments).expect("ok");
        // Roll demand 0.4, total capacity 0.4. Should distribute
        // 0.1 → 0.1 (saturated at roll-a) and 0.3 → 0.3 (saturated
        // at roll-b). Total exactly = capacity → not saturated
        // (boundary case `|τ| == Σ L` falls into the proportional
        // branch).
        let out = alloc.allocate([0.4, 0.0, 0.0]);
        for (id, u) in &out.commands {
            if *id == eid("vehicle.assembly.effectors.roll-a") {
                assert_abs_diff_eq!(*u, 0.1, epsilon = 1.0e-12);
            } else if *id == eid("vehicle.assembly.effectors.roll-b") {
                assert_abs_diff_eq!(*u, 0.3, epsilon = 1.0e-12);
            }
        }
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    #[test]
    fn allocate_saturates_when_demand_exceeds_capacity() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        // Roll demand 1.0, total roll capacity 0.4. Should saturate.
        let out = alloc.allocate([1.0, 0.0, 0.0]);
        for (id, u) in &out.commands {
            if *id == eid("vehicle.assembly.effectors.roll-a")
                || *id == eid("vehicle.assembly.effectors.roll-b")
            {
                // Each at +0.2 (their limit).
                assert_abs_diff_eq!(*u, 0.2, epsilon = 1.0e-12);
            }
        }
        assert!(out.saturated_axes[BodyAxis::Roll.index()]);
        assert!(!out.saturated_axes[BodyAxis::Pitch.index()]);
        assert!(!out.saturated_axes[BodyAxis::Yaw.index()]);
    }

    #[test]
    fn allocate_negative_demand_drives_negative_commands() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        let out = alloc.allocate([-0.3, 0.0, 0.0]);
        for (id, u) in &out.commands {
            if *id == eid("vehicle.assembly.effectors.roll-a")
                || *id == eid("vehicle.assembly.effectors.roll-b")
            {
                assert_abs_diff_eq!(*u, -0.15, epsilon = 1.0e-12);
            }
        }
    }

    #[test]
    fn allocate_passes_through_single_effector_axis() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        // Pitch axis has exactly one effector with max_abs = 0.35.
        let out = alloc.allocate([0.0, 0.2, 0.0]);
        for (id, u) in &out.commands {
            if *id == eid("vehicle.assembly.effectors.pitch") {
                assert_abs_diff_eq!(*u, 0.2, epsilon = 1.0e-12);
            }
        }
    }

    #[test]
    fn allocate_handles_all_three_axes_independently() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        let out = alloc.allocate([0.3, 0.2, -0.1]);
        let by_id: BTreeMap<_, _> = out.commands.iter().map(|(id, u)| (*id, *u)).collect();
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-a")],
            0.15,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-b")],
            0.15,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.pitch")],
            0.2,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.yaw")],
            -0.1,
            epsilon = 1.0e-12
        );
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    #[test]
    fn allocate_is_deterministic_across_reruns() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        let a = alloc.allocate([0.123, -0.456, 0.0789]);
        let b = alloc.allocate([0.123, -0.456, 0.0789]);
        assert_eq!(a.commands.len(), b.commands.len());
        for (lhs, rhs) in a.commands.iter().zip(&b.commands) {
            assert_eq!(lhs.0, rhs.0);
            assert_eq!(lhs.1.to_bits(), rhs.1.to_bits());
        }
    }

    #[test]
    fn allocate_with_allowed_effectors_excludes_disallowed_capacity() {
        let alloc =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        let allowed = [
            eid("vehicle.assembly.effectors.roll-b"),
            eid("vehicle.assembly.effectors.pitch"),
            eid("vehicle.assembly.effectors.yaw"),
        ];
        let out = alloc.allocate_with_allowed_effectors([0.3, 0.0, 0.0], &allowed);
        let by_id: BTreeMap<_, _> = out.commands.iter().map(|(id, u)| (*id, *u)).collect();
        assert_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-a")].to_bits(),
            0.0_f64.to_bits()
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-b")],
            0.2,
            epsilon = 1.0e-12
        );
        assert!(out.saturated_axes[BodyAxis::Roll.index()]);
    }

    #[test]
    fn priority_does_not_change_outcome_in_single_axis_case() {
        let alloc_rpy =
            PrioritisedRedistributedAllocator::new(nominal_priority(), nominal_assignments())
                .expect("ok");
        let alloc_yrp = PrioritisedRedistributedAllocator::new(
            [BodyAxis::Yaw, BodyAxis::Roll, BodyAxis::Pitch],
            nominal_assignments(),
        )
        .expect("ok");
        let demand = [0.2, 0.1, -0.05];
        let a = alloc_rpy.allocate(demand);
        let b = alloc_yrp.allocate(demand);
        let by_id_a: BTreeMap<_, _> = a.commands.iter().map(|(id, u)| (*id, *u)).collect();
        let by_id_b: BTreeMap<_, _> = b.commands.iter().map(|(id, u)| (*id, *u)).collect();
        for (id, u) in &by_id_a {
            assert_eq!(by_id_a[id].to_bits(), by_id_b[id].to_bits(), "id {id:?}");
            let _ = u;
        }
        assert_eq!(a.saturated_axes, b.saturated_axes);
    }

    #[test]
    fn pseudo_inverse_splits_by_squared_capacity() {
        let assignments = vec![
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-a"),
                axis: BodyAxis::Roll,
                max_abs: 0.1,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-b"),
                axis: BodyAxis::Roll,
                max_abs: 0.3,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.pitch"),
                axis: BodyAxis::Pitch,
                max_abs: 0.35,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.yaw"),
                axis: BodyAxis::Yaw,
                max_abs: 0.35,
            },
        ];
        let alloc = WeightedPseudoInverseAllocator::new(assignments).expect("ok");
        let out = alloc.allocate([0.2, 0.0, 0.0]);
        let by_id: BTreeMap<_, _> = out.commands.iter().map(|(id, u)| (*id, *u)).collect();
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-a")],
            0.02,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-b")],
            0.18,
            epsilon = 1.0e-12
        );
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    #[test]
    fn pseudo_inverse_redistributes_after_bound_hit() {
        let assignments = vec![
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-a"),
                axis: BodyAxis::Roll,
                max_abs: 0.1,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.roll-b"),
                axis: BodyAxis::Roll,
                max_abs: 0.3,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.pitch"),
                axis: BodyAxis::Pitch,
                max_abs: 0.35,
            },
            EffectorAxisAssignment {
                effector_id: eid("vehicle.assembly.effectors.yaw"),
                axis: BodyAxis::Yaw,
                max_abs: 0.35,
            },
        ];
        let alloc = WeightedPseudoInverseAllocator::new(assignments).expect("ok");
        let out = alloc.allocate([0.4, 0.0, 0.0]);
        let by_id: BTreeMap<_, _> = out.commands.iter().map(|(id, u)| (*id, *u)).collect();
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-a")],
            0.1,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-b")],
            0.3,
            epsilon = 1.0e-12
        );
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    #[test]
    fn pseudo_inverse_reports_saturation_when_capacity_is_exceeded() {
        let alloc = WeightedPseudoInverseAllocator::new(nominal_assignments()).expect("ok");
        let out = alloc.allocate([1.0, 0.0, 0.0]);
        let by_id: BTreeMap<_, _> = out.commands.iter().map(|(id, u)| (*id, *u)).collect();
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-a")],
            0.2,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-b")],
            0.2,
            epsilon = 1.0e-12
        );
        assert!(out.saturated_axes[BodyAxis::Roll.index()]);
    }

    #[test]
    fn control_allocator_dispatches_pseudo_inverse_with_phase_mask() {
        let mut alloc: ControlAllocator =
            WeightedPseudoInverseAllocator::new(nominal_assignments())
                .expect("ok")
                .into();
        let allowed = [
            eid("vehicle.assembly.effectors.roll-b"),
            eid("vehicle.assembly.effectors.pitch"),
            eid("vehicle.assembly.effectors.yaw"),
        ];
        let out = alloc
            .allocate_with_allowed_effectors([0.1, 0.0, 0.0], &allowed)
            .expect("allocation succeeds");
        let by_id: BTreeMap<_, _> = out.commands.iter().map(|(id, u)| (*id, *u)).collect();
        assert_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-a")].to_bits(),
            0.0_f64.to_bits()
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.roll-b")],
            0.1,
            epsilon = 1.0e-12
        );
        assert_eq!(out.saturated_axes, [false; 3]);
    }

    fn dense_cluster_matrix() -> EffectivenessMatrix {
        EffectivenessMatrix::new(
            2,
            vec![
                eid("vehicle.assembly.effectors.engine-left"),
                eid("vehicle.assembly.effectors.engine-right"),
                eid("vehicle.assembly.effectors.engine-center"),
            ],
            vec![
                1.0, 0.0, 1.0, //
                0.0, 1.0, 1.0,
            ],
        )
        .expect("dense matrix constructs")
    }

    fn loose_dense_bounds(columns: usize) -> EffectorBounds {
        EffectorBounds {
            u_min_pos: vec![-10.0; columns],
            u_max_pos: vec![10.0; columns],
            rate_limit: vec![100.0; columns],
        }
    }

    fn toml_table<'a>(value: &'a Value, key: &str) -> &'a toml::Table {
        value
            .get(key)
            .and_then(Value::as_table)
            .unwrap_or_else(|| panic!("missing table {key}"))
    }

    fn toml_array<'a>(table: &'a toml::Table, key: &str) -> &'a [Value] {
        table
            .get(key)
            .and_then(Value::as_array)
            .map(Vec::as_slice)
            .unwrap_or_else(|| panic!("missing array {key}"))
    }

    fn toml_string_array(table: &toml::Table, key: &str) -> Vec<String> {
        toml_array(table, key)
            .iter()
            .map(|value| {
                value
                    .as_str()
                    .unwrap_or_else(|| panic!("{key} entry must be a string"))
                    .to_owned()
            })
            .collect()
    }

    fn toml_f64(table: &toml::Table, key: &str) -> f64 {
        table
            .get(key)
            .and_then(Value::as_float)
            .unwrap_or_else(|| panic!("missing float {key}"))
    }

    fn toml_usize(table: &toml::Table, key: &str) -> usize {
        table
            .get(key)
            .and_then(Value::as_integer)
            .and_then(|value| usize::try_from(value).ok())
            .unwrap_or_else(|| panic!("missing usize {key}"))
    }

    fn toml_i8_array(table: &toml::Table, key: &str) -> Vec<i8> {
        toml_array(table, key)
            .iter()
            .map(|value| {
                value
                    .as_integer()
                    .and_then(|integer| i8::try_from(integer).ok())
                    .unwrap_or_else(|| panic!("{key} entry must be an i8"))
            })
            .collect()
    }

    fn toml_f64_array(table: &toml::Table, key: &str) -> Vec<f64> {
        toml_array(table, key)
            .iter()
            .map(|value| {
                value
                    .as_float()
                    .unwrap_or_else(|| panic!("{key} entry must be a float"))
            })
            .collect()
    }

    fn dense_active_set_signs(output: &WlsAllocationOutput, bounds: &EffectorBounds) -> Vec<i8> {
        output
            .active_at_bound
            .iter()
            .enumerate()
            .map(|(index, active)| {
                if !active {
                    return 0;
                }
                let command = output.u[index].1;
                if (command - bounds.u_min_pos[index]).abs() <= 1.0e-9 {
                    -1
                } else if (command - bounds.u_max_pos[index]).abs() <= 1.0e-9 {
                    1
                } else {
                    panic!(
                        "active command {command} is not on [{}, {}]",
                        bounds.u_min_pos[index], bounds.u_max_pos[index]
                    );
                }
            })
            .collect()
    }

    #[test]
    fn dense_wls_allocator_splits_coupled_column_by_minimum_effort() {
        let matrix = dense_cluster_matrix();
        let allocator = WlsActiveSetAllocator::new(
            matrix,
            vec![1.0, 1.0],
            vec![1.0, 1.0, 1.0],
            1.0e12,
            vec![0.0, 0.0, 0.0],
        )
        .expect("allocator constructs");

        let output = allocator
            .allocate(&[1.0, 1.0], &[0.0, 0.0, 0.0], &loose_dense_bounds(3), 1.0)
            .expect("allocation succeeds");
        let by_id: BTreeMap<_, _> = output.u.iter().map(|(id, u)| (*id, *u)).collect();

        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.engine-left")],
            1.0 / 3.0,
            epsilon = 1.0e-6
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.engine-right")],
            1.0 / 3.0,
            epsilon = 1.0e-6
        );
        assert_abs_diff_eq!(
            by_id[&eid("vehicle.assembly.effectors.engine-center")],
            2.0 / 3.0,
            epsilon = 1.0e-6
        );
        assert!(output.attainable);
        assert_eq!(output.active_at_bound, vec![false, false, false]);
        assert!(usize::from(output.iterations) <= 3);
    }

    #[test]
    fn dense_wls_allocator_applies_rate_tightened_bounds() {
        let matrix =
            EffectivenessMatrix::new(1, vec![eid("vehicle.assembly.effectors.engine")], vec![1.0])
                .expect("dense matrix constructs");
        let allocator = WlsActiveSetAllocator::new(matrix, vec![1.0], vec![1.0], 1.0e9, vec![0.0])
            .expect("allocator constructs");
        let bounds = EffectorBounds {
            u_min_pos: vec![-10.0],
            u_max_pos: vec![10.0],
            rate_limit: vec![0.2],
        };

        let output = allocator
            .allocate(&[1.0], &[0.0], &bounds, 1.0)
            .expect("allocation succeeds");

        assert_abs_diff_eq!(output.u[0].1, 0.2, epsilon = 1.0e-12);
        assert_eq!(output.active_at_bound, vec![true]);
        assert_abs_diff_eq!(output.virtual_residual[0], 0.8, epsilon = 1.0e-12);
        assert!(!output.attainable);
    }

    #[test]
    fn dense_wls_allocator_matches_qcat_admire_and_synthetic_tolerance_tables() {
        let data: Value = toml::from_str(QCAT_WLS_TOLERANCE_TABLE).expect("QCAT WLS table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_allocation_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let command_epsilon = toml_f64(tolerance, "absolute_command");
        let residual_epsilon = toml_f64(tolerance, "absolute_residual");
        let problems = data
            .get("problem")
            .and_then(Value::as_array)
            .expect("problem array");

        for problem in problems {
            let problem = problem.as_table().expect("problem table");
            let problem_name = problem
                .get("name")
                .and_then(Value::as_str)
                .expect("problem name");
            let effectors: Vec<EffectorId> = toml_string_array(problem, "effectors")
                .iter()
                .map(|id| eid(&format!("vehicle.assembly.effectors.{id}")))
                .collect();
            let matrix = EffectivenessMatrix::new(
                toml_usize(problem, "rows"),
                effectors,
                toml_f64_array(problem, "row_major"),
            )
            .expect("QCAT matrix constructs");
            let bounds = EffectorBounds {
                u_min_pos: toml_f64_array(problem, "u_min"),
                u_max_pos: toml_f64_array(problem, "u_max"),
                rate_limit: toml_f64_array(problem, "rate_limit"),
            };
            let allocator = WlsActiveSetAllocator::new(
                matrix,
                toml_f64_array(problem, "virtual_weights"),
                toml_f64_array(problem, "effort_weights"),
                toml_f64(problem, "gamma"),
                toml_f64_array(problem, "preferred"),
            )
            .expect("QCAT allocator constructs");
            let u_prev = toml_f64_array(problem, "u_prev");
            let dt_s = toml_f64(problem, "dt_s");

            for case in toml_array(problem, "case") {
                let case = case.as_table().expect("case table");
                let case_name = case.get("name").and_then(Value::as_str).expect("case name");
                let expected_u = toml_f64_array(case, "expected_u");
                let expected_residual = toml_f64_array(case, "expected_residual");
                let output = allocator
                    .allocate(
                        &toml_f64_array(case, "virtual_command"),
                        &u_prev,
                        &bounds,
                        dt_s,
                    )
                    .unwrap_or_else(|err| {
                        panic!("{problem_name}/{case_name} allocation failed: {err}")
                    });

                for ((_, actual), expected) in output.u.iter().zip(expected_u) {
                    assert!(
                        (*actual - expected).abs() <= command_epsilon,
                        "{problem_name} / {case_name} command: actual {actual}, expected \
                         {expected}, epsilon {command_epsilon}"
                    );
                }
                for (actual, expected) in output.virtual_residual.iter().zip(expected_residual) {
                    assert!(
                        (*actual - expected).abs() <= residual_epsilon,
                        "{problem_name} / {case_name} residual: actual {actual}, expected \
                         {expected}, epsilon {residual_epsilon}"
                    );
                }
                assert_eq!(
                    dense_active_set_signs(&output, &bounds),
                    toml_i8_array(case, "expected_active_set"),
                    "{problem_name} / {case_name} active set"
                );
                assert_eq!(
                    usize::from(output.iterations),
                    toml_usize(case, "expected_iterations"),
                    "{problem_name} / {case_name} iterations"
                );
                assert_eq!(
                    output.attainable,
                    case.get("expected_attainable")
                        .and_then(Value::as_bool)
                        .expect("expected_attainable bool"),
                    "{problem_name} / {case_name} attainability"
                );
            }
        }
    }

    #[test]
    fn dense_control_allocator_updates_previous_command_and_phase_gates_zero() {
        let matrix = EffectivenessMatrix::new(
            3,
            vec![eid("vehicle.assembly.effectors.roll")],
            vec![1.0, 0.0, 0.0],
        )
        .expect("dense matrix constructs");
        let wls =
            WlsActiveSetAllocator::new(matrix, vec![1.0, 1.0, 1.0], vec![1.0], 1.0e9, vec![0.0])
                .expect("wls constructs");
        let bounds = EffectorBounds {
            u_min_pos: vec![-1.0],
            u_max_pos: vec![1.0],
            rate_limit: vec![0.25],
        };
        let mut allocator = DenseWlsControlAllocator::new(wls, bounds, vec![0.0], 1.0)
            .expect("dense control allocator constructs");

        let first = allocator
            .allocate([1.0, 0.0, 0.0])
            .expect("first allocation succeeds");
        assert_abs_diff_eq!(first.commands[0].1, 0.25, epsilon = 1.0e-12);
        assert!(first.saturated_axes[BodyAxis::Roll.index()]);

        let second = allocator
            .allocate([1.0, 0.0, 0.0])
            .expect("second allocation succeeds");
        assert_abs_diff_eq!(second.commands[0].1, 0.5, epsilon = 1.0e-12);

        let gated = allocator
            .allocate_with_allowed_effectors([1.0, 0.0, 0.0], &[])
            .expect("phase-gated allocation succeeds");
        assert_eq!(gated.commands[0].1.to_bits(), 0.0_f64.to_bits());
        assert!(gated.saturated_axes[BodyAxis::Roll.index()]);
    }

    #[test]
    fn dense_wls_allocator_is_deterministic_across_reruns() {
        let allocator = WlsActiveSetAllocator::new(
            dense_cluster_matrix(),
            vec![1.0, 2.0],
            vec![1.0, 1.5, 2.0],
            1.0e6,
            vec![0.1, -0.1, 0.0],
        )
        .expect("allocator constructs");
        let bounds = EffectorBounds {
            u_min_pos: vec![-0.5, -0.5, -0.25],
            u_max_pos: vec![0.5, 0.5, 0.25],
            rate_limit: vec![1.0, 1.0, 1.0],
        };

        let first = allocator
            .allocate(&[0.75, -0.1], &[0.0, 0.0, 0.0], &bounds, 1.0)
            .expect("first allocation succeeds");
        let second = allocator
            .allocate(&[0.75, -0.1], &[0.0, 0.0, 0.0], &bounds, 1.0)
            .expect("second allocation succeeds");

        assert_eq!(first.active_at_bound, second.active_at_bound);
        assert_eq!(first.iterations, second.iterations);
        for ((lhs_id, lhs), (rhs_id, rhs)) in first.u.iter().zip(&second.u) {
            assert_eq!(lhs_id, rhs_id);
            assert_eq!(lhs.to_bits(), rhs.to_bits());
        }
        for (lhs, rhs) in first.virtual_residual.iter().zip(&second.virtual_residual) {
            assert_eq!(lhs.to_bits(), rhs.to_bits());
        }
    }

    #[test]
    fn dense_wls_allocator_rejects_invalid_inputs() {
        assert!(matches!(
            EffectivenessMatrix::new(
                2,
                vec![eid("vehicle.assembly.effectors.a")],
                vec![1.0, f64::NAN],
            ),
            Err(AllocatorError::DenseNonFinite { .. })
        ));
        assert!(matches!(
            EffectivenessMatrix::new(
                1,
                vec![
                    eid("vehicle.assembly.effectors.a"),
                    eid("vehicle.assembly.effectors.a"),
                ],
                vec![1.0, 2.0],
            ),
            Err(AllocatorError::DuplicateEffector { .. })
        ));

        let allocator = WlsActiveSetAllocator::new(
            dense_cluster_matrix(),
            vec![1.0, 1.0],
            vec![1.0, 1.0, 1.0],
            1.0e6,
            vec![0.0, 0.0, 0.0],
        )
        .expect("allocator constructs");
        let mut bounds = loose_dense_bounds(3);
        bounds.rate_limit[1] = -1.0;
        assert!(matches!(
            allocator.allocate(&[1.0, 1.0], &[0.0, 0.0, 0.0], &bounds, 1.0),
            Err(AllocatorError::InvalidDenseBounds { .. })
        ));
        let bounds = loose_dense_bounds(3);
        assert!(matches!(
            allocator.allocate(&[1.0, f64::INFINITY], &[0.0, 0.0, 0.0], &bounds, 1.0),
            Err(AllocatorError::DenseNonFinite { .. })
        ));
    }

    #[test]
    fn body_axis_index_matches_array_layout() {
        assert_eq!(BodyAxis::Roll.index(), 0);
        assert_eq!(BodyAxis::Pitch.index(), 1);
        assert_eq!(BodyAxis::Yaw.index(), 2);
    }

    #[test]
    fn body_axis_all_visits_each_axis_once() {
        let v = BodyAxis::all();
        assert_eq!(v, [BodyAxis::Roll, BodyAxis::Pitch, BodyAxis::Yaw]);
    }
}
