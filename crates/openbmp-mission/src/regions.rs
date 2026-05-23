//! Orthogonal concurrent regions (Phase 5.X.D).
//!
//! A [`Region`] is an independent state machine instance that ticks
//! concurrently with the canonical mission region. Phase 5.X.D
//! lands the type-level vocabulary for the four canonical regions
//! (`mission`, `health`, `comms`, `estimator_regime`) and the
//! cross-region transition guard expression that composes with
//! triggers via AND semantics.
//!
//! # Architecture
//!
//! - [`Region`] owns its declared flat state set and current state.
//!   The canonical `mission` region mirrors the mission graph; the
//!   `health` region is updated by the FC commander's safety logic.
//! - [`RegionSet`] is the per-scenario registry of regions, indexed
//!   by [`crate::RegionId`]. Production commander code owns a
//!   `RegionSet`, keeps the mission region in sync with the active
//!   mission state, and derives `safe_state_requested` from the
//!   health region.
//! - [`CrossRegionGuard`] expresses a precondition on another
//!   region's current state. Composes with the trigger via AND
//!   semantics.
//! - [`CanonicalRegions`] enumerates the four canonical region ids
//!   so consumers reference them by name (`mission`, `health`,
//!   `comms`, `estimator_regime`) without hard-coding the
//!   FNV-1a-64 values.
//!
//! # Determinism contract
//!
//! - Region ids are FNV-1a-64 of canonical paths
//!   (`"mission.regions.<name>"`).
//! - Per-region tick order is locked by canonical-form sort
//!   (`RegionId.value()` ascending).
//! - Cross-region guards never produce a non-deterministic
//!   transition: a guard either fires consistently or doesn't fire
//!   for a given `(transition, region-state)` pair.

use std::collections::BTreeMap;

use std::borrow::Cow;

use thiserror::Error;

use crate::{MissionPhaseGraph, Phase, PhaseId, RegionId};

// ---------------------------------------------------------------------
// Canonical region ids
// ---------------------------------------------------------------------

/// The four canonical orthogonal regions Phase 5.X.D reserves.
///
/// Their ids are FNV-1a-64 of the canonical scenario region paths;
/// reordering region declarations in a scenario file cannot shift
/// any region's id. Phase 5.X.F's runner supplies these four regions
/// when `[[mission.regions]]` is omitted.
#[derive(Debug)]
pub struct CanonicalRegions;

impl CanonicalRegions {
    /// The `mission` region's id — the existing flat-DAG mission
    /// FSM, upgraded to hierarchical in 5.X.F. Always present.
    #[must_use]
    pub const fn mission() -> RegionId {
        RegionId::from_path("mission.regions.mission")
    }

    /// The `health` region's id — `Nominal` / `Degraded` /
    /// `AbortRequested` / `SafedOnFault`.
    #[must_use]
    pub const fn health() -> RegionId {
        RegionId::from_path("mission.regions.health")
    }

    /// The `comms` region's id — `Linked` / `Degraded` /
    /// `LossOfSignal` / `SafedOnLossOfSignal`.
    #[must_use]
    pub const fn comms() -> RegionId {
        RegionId::from_path("mission.regions.comms")
    }

    /// The `estimator_regime` region's id — populated by the IMM
    /// Phase 5.B estimator. Observed-but-not-decided by the
    /// commander.
    #[must_use]
    pub const fn estimator_regime() -> RegionId {
        RegionId::from_path("mission.regions.estimator_regime")
    }

    /// Phase-6 reserved: aerodynamic-regime region (`subsonic`,
    /// `transonic`, `supersonic`, `hypersonic`, `rarefied`). Not
    /// instantiated until 6.D.
    #[must_use]
    pub const fn aerodynamic_regime() -> RegionId {
        RegionId::from_path("mission.regions.aerodynamic_regime")
    }
}

/// Canonical region-state ids for the built-in orthogonal regions.
#[derive(Debug)]
pub struct CanonicalRegionStates;

impl CanonicalRegionStates {
    /// `health.nominal`.
    #[must_use]
    pub const fn health_nominal() -> PhaseId {
        PhaseId::from_path("mission.regions.health.nominal")
    }

    /// `health.degraded`.
    #[must_use]
    pub const fn health_degraded() -> PhaseId {
        PhaseId::from_path("mission.regions.health.degraded")
    }

    /// `health.abort_requested`.
    #[must_use]
    pub const fn health_abort_requested() -> PhaseId {
        PhaseId::from_path("mission.regions.health.abort_requested")
    }

    /// `health.safed_on_fault`.
    #[must_use]
    pub const fn health_safed_on_fault() -> PhaseId {
        PhaseId::from_path("mission.regions.health.safed_on_fault")
    }

    /// `comms.linked`.
    #[must_use]
    pub const fn comms_linked() -> PhaseId {
        PhaseId::from_path("mission.regions.comms.linked")
    }

    /// `estimator_regime.boost_mode`.
    #[must_use]
    pub const fn estimator_boost_mode() -> PhaseId {
        PhaseId::from_path("mission.regions.estimator_regime.boost_mode")
    }
}

// ---------------------------------------------------------------------
// Cross-region guard
// ---------------------------------------------------------------------

/// Cross-region transition guard expression.
///
/// Composes with the trigger via AND: the transition fires when the
/// trigger AND the guard both hold. A guard either consistently
/// fires or consistently doesn't fire for a given
/// `(transition, region-state)` pair — never flaps.
#[derive(Clone, Debug, PartialEq, Eq, Hash)]
pub struct CrossRegionGuard {
    /// Region whose current state is the guard's left-hand side.
    pub region: RegionId,
    /// State the region must currently be in for the guard to hold.
    /// Encoded as a [`PhaseId`] until the v4 scenario format lifts
    /// region states to hierarchical ids in 5.X.F.
    pub state: PhaseId,
}

impl CrossRegionGuard {
    /// Construct a guard requiring `region` to currently be in
    /// `state`.
    #[must_use]
    pub const fn new(region: RegionId, state: PhaseId) -> Self {
        Self { region, state }
    }

    /// Evaluate the guard against a region-current-state snapshot.
    /// Returns `true` if `region` is in `state` (guard holds).
    #[must_use]
    pub fn evaluate(&self, region_states: &BTreeMap<RegionId, PhaseId>) -> bool {
        region_states
            .get(&self.region)
            .copied()
            .is_some_and(|s| s == self.state)
    }
}

// ---------------------------------------------------------------------
// Region
// ---------------------------------------------------------------------

/// One orthogonal concurrent region.
///
/// Phase 5.X models a region as `(id, states, current_state)`. The
/// `mission` region's state set comes from the mission graph. The
/// `health`, `comms`, and `estimator_regime` regions use canonical
/// state ids supplied by the runner unless a scenario declares an
/// explicit non-mission region.
#[derive(Clone, Debug, PartialEq)]
pub struct Region {
    /// Path-derived stable id.
    pub id: RegionId,
    /// The region's declared flat state set.
    pub states: Vec<Phase>,
    /// Active state at the start of the tick.
    pub current_state: PhaseId,
}

impl Region {
    /// Construct a region from an existing flat mission graph.
    #[must_use]
    pub fn new(id: RegionId, graph: MissionPhaseGraph) -> Self {
        let current_state = graph.initial;
        Self {
            id,
            states: graph.phases,
            current_state,
        }
    }

    /// Construct a region from explicit states and an initial state.
    ///
    /// # Errors
    ///
    /// Returns [`RegionError`] when ids are duplicated or `initial`
    /// is not in `states`.
    pub fn from_states(
        id: RegionId,
        mut states: Vec<Phase>,
        initial: PhaseId,
    ) -> Result<Self, RegionError> {
        let mut seen = std::collections::BTreeSet::new();
        for state in &states {
            if !seen.insert(state.id) {
                return Err(RegionError::DuplicateState {
                    region: id,
                    state: state.id,
                });
            }
        }
        if !seen.contains(&initial) {
            return Err(RegionError::UnknownState {
                region: id,
                state: initial,
                in_field: Cow::Borrowed("region.initial_state"),
            });
        }
        states.sort_by_key(|state| state.id.value());
        Ok(Self {
            id,
            states,
            current_state: initial,
        })
    }

    /// Returns `true` if this region declares `state`.
    #[must_use]
    pub fn contains_state(&self, state: PhaseId) -> bool {
        self.states.iter().any(|s| s.id == state)
    }

    /// Set the current state for this region.
    ///
    /// # Errors
    ///
    /// Returns [`RegionError::UnknownState`] when `state` is not
    /// declared in this region.
    pub fn set_current_state(&mut self, state: PhaseId) -> Result<(), RegionError> {
        if !self.contains_state(state) {
            return Err(RegionError::UnknownState {
                region: self.id,
                state,
                in_field: Cow::Borrowed("region.current_state"),
            });
        }
        self.current_state = state;
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Region set
// ---------------------------------------------------------------------

/// Per-scenario registry of orthogonal regions.
///
/// The FC commander owns this registry in production. Region order is
/// locked by `RegionId.value()` ascending (canonical-form sort).
#[derive(Clone, Debug, Default, PartialEq)]
pub struct RegionSet {
    /// Regions keyed by id; `BTreeMap` iteration order is locked
    /// by `RegionId.value()` ascending.
    pub regions: BTreeMap<RegionId, Region>,
}

impl RegionSet {
    /// Construct an empty region set.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Add a region to the set. Returns the previous region for
    /// `id` if present (typically `None`).
    pub fn insert(&mut self, region: Region) -> Option<Region> {
        self.regions.insert(region.id, region)
    }

    /// Current state for one region.
    #[must_use]
    pub fn current_state(&self, region: RegionId) -> Option<PhaseId> {
        self.regions.get(&region).map(|r| r.current_state)
    }

    /// Set a region's current state.
    ///
    /// # Errors
    ///
    /// Returns [`RegionError`] when the region or state is unknown.
    pub fn set_current_state(
        &mut self,
        region: RegionId,
        state: PhaseId,
    ) -> Result<(), RegionError> {
        let Some(region_entry) = self.regions.get_mut(&region) else {
            return Err(RegionError::UnknownRegion { region });
        };
        region_entry.set_current_state(state)
    }

    /// Snapshot the current state of every region. Used by the
    /// commander to publish to per-region bus topics and to
    /// evaluate cross-region guards.
    #[must_use]
    pub fn current_states(&self) -> BTreeMap<RegionId, PhaseId> {
        self.regions
            .iter()
            .map(|(id, r)| (*id, r.current_state))
            .collect()
    }
}

/// Errors produced while constructing or ticking orthogonal regions.
#[derive(Debug, Clone, PartialEq, Error)]
pub enum RegionError {
    /// Two states in the same region share an id.
    #[error("region {region:?} has duplicate state id {state:?}")]
    DuplicateState {
        /// Region that contains the duplicate.
        region: RegionId,
        /// Duplicated state id.
        state: PhaseId,
    },
    /// Region id was not declared.
    #[error("unknown region {region:?}")]
    UnknownRegion {
        /// Unknown region id.
        region: RegionId,
    },
    /// State id was not declared in the referenced region.
    #[error("unknown state {state:?} for region {region:?} in {in_field}")]
    UnknownState {
        /// Region containing the reference.
        region: RegionId,
        /// Unknown state id.
        state: PhaseId,
        /// Field that made the reference.
        in_field: Cow<'static, str>,
    },
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{Phase, PhaseTransition};

    #[test]
    fn canonical_region_ids_are_stable_and_distinct() {
        let mission = CanonicalRegions::mission();
        let health = CanonicalRegions::health();
        let comms = CanonicalRegions::comms();
        let estimator = CanonicalRegions::estimator_regime();
        let aero = CanonicalRegions::aerodynamic_regime();
        let ids = [mission, health, comms, estimator, aero];
        let unique: std::collections::BTreeSet<_> = ids.iter().copied().collect();
        assert_eq!(unique.len(), ids.len(), "all canonical region ids distinct");
        assert_ne!(
            CanonicalRegionStates::health_nominal(),
            CanonicalRegionStates::health_abort_requested()
        );
    }

    #[test]
    fn guard_evaluates_against_region_snapshot() {
        let health = CanonicalRegions::health();
        let nominal = PhaseId::from_path("mission.regions.health.nominal");
        let degraded = PhaseId::from_path("mission.regions.health.degraded");

        let guard = CrossRegionGuard::new(health, nominal);

        let mut snapshot = BTreeMap::new();
        snapshot.insert(health, nominal);
        assert!(guard.evaluate(&snapshot));

        snapshot.insert(health, degraded);
        assert!(!guard.evaluate(&snapshot));

        snapshot.clear();
        assert!(!guard.evaluate(&snapshot));
    }

    #[test]
    fn region_set_iterates_in_canonical_order() {
        let mission_phase = PhaseId::from_path("mission.regions.mission.initial");
        let health_phase = PhaseId::from_path("mission.regions.health.nominal");
        let mission_graph = MissionPhaseGraph::new(
            vec![Phase {
                id: mission_phase,
                label: "initial".into(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            }],
            Vec::new(),
            mission_phase,
            &[],
        )
        .expect("mission graph");
        let health_graph = MissionPhaseGraph::new(
            vec![Phase {
                id: health_phase,
                label: "nominal".into(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            }],
            Vec::new(),
            health_phase,
            &[],
        )
        .expect("health graph");

        let mut set = RegionSet::new();
        set.insert(Region::new(CanonicalRegions::mission(), mission_graph));
        set.insert(Region::new(CanonicalRegions::health(), health_graph));

        let states = set.current_states();
        assert_eq!(states.len(), 2);
        assert_eq!(
            states.get(&CanonicalRegions::mission()).copied(),
            Some(mission_phase),
        );
        assert_eq!(
            states.get(&CanonicalRegions::health()).copied(),
            Some(health_phase),
        );
    }

    #[test]
    fn empty_region_set_has_no_states() {
        let set = RegionSet::new();
        assert!(set.current_states().is_empty());
    }

    #[test]
    fn region_rejects_unknown_current_state() {
        let health = CanonicalRegions::health();
        let nominal = CanonicalRegionStates::health_nominal();
        let abort = CanonicalRegionStates::health_abort_requested();
        let mut region = Region::from_states(
            health,
            vec![Phase {
                id: nominal,
                label: "nominal".into(),
                allowed_effectors: Vec::new(),
                allowed_engines: Vec::new(),
            }],
            nominal,
        )
        .expect("region");
        let err = region.set_current_state(abort).expect_err("unknown state");
        assert!(matches!(err, RegionError::UnknownState { state, .. } if state == abort));
    }

    // PhaseTransition is held live to confirm the symbol re-exports
    // remain usable from this module for future 5.X.F wiring; this
    // is a compile-time check, not a runtime test.
    #[allow(dead_code)]
    const _: Option<PhaseTransition> = None;
}
