//! Phase-5.A.5 control-allocation primitives.
//!
//! The Phase-4 actuator-channel mapping in `mixer.rs` assumes a 1:1
//! correspondence between the autopilot's semantic channel surface
//! (`aileron / elevator / rudder / body_flap`) and a single
//! `direct_torque` effector per channel. With more than one
//! effector contributing to a body axis — a 4-thruster RCS bank
//! with redundant roll authority, or a multi-engine cluster sharing
//! pitch/yaw — that 1:1 assumption breaks. Phase 5.A.5 ships a
//! prioritised redistributed allocator that consumes the
//! per-axis torque demand from
//! [`crate::topics::ActuatorCommand`] and emits a per-effector
//! [`crate::topics::EffectorCommandSet`] honouring each effector's
//! symmetric box bound.
//!
//! # Scope (Phase 5.A.5)
//!
//! - `direct_torque` effectors only: each effector contributes to
//!   exactly one body axis. The allocator's job per axis is then
//!   "distribute the demand across the assigned effectors";
//!   inter-axis priority is reported but does not change the
//!   outcome on a single-axis-effector scenario (the case shipped
//!   here). Future slices can extend the allocator to coupled
//!   effectors (gimbaled engine clusters, etc.) where priority
//!   genuinely affects redistribution.
//! - Symmetric box bounds: every effector's `[min, max]` is
//!   required to satisfy `max == −min` and `max > 0`. Asymmetric
//!   limits are rejected at allocator construction. (The shipped
//!   scenarios all use symmetric bounds.)
//! - The pseudo-inverse allocator named in
//!   [`openbmp_scenario::FcAutopilotAllocationKind::PseudoInverse`]
//!   is parsed but not yet consumed: a later slice will add the
//!   general `G_eff` path.
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

use std::collections::BTreeSet;

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

/// Errors raised by [`PrioritisedRedistributedAllocator::new`].
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
        "allocator: effector {effector_id:?} requires symmetric limits in Phase 5.A.5; \
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

/// Phase-5.A.5 prioritised redistributed allocator.
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

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::panic)]
mod tests {
    use std::collections::BTreeMap;

    use approx::assert_abs_diff_eq;

    use super::*;

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
