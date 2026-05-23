//! Phase 5.X.G property tests for orthogonal regions + cross-region
//! guards.
//!
//! Asserts:
//!
//! 1. Canonical region ids are stable across compilation reruns
//!    (FNV-1a-64 of canonical path).
//! 2. `CrossRegionGuard::evaluate` is deterministic: a fixed
//!    `(guard, region_state)` pair always yields the same result.
//! 3. `RegionSet::current_states` iteration order is locked by
//!    `RegionId.value()` ascending, regardless of insertion order.
//! 4. Round-trip: inserting regions in any permutation produces an
//!    identical `current_states` map.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use std::collections::BTreeMap;

use openbmp_mission::{
    CanonicalRegions, CrossRegionGuard, MissionPhaseGraph, Phase, PhaseId, Region, RegionId,
    RegionSet,
};
use proptest::prelude::*;

fn build_single_state_graph(region_path: &str, state_path: &str) -> MissionPhaseGraph {
    let phase_id = PhaseId::from_path(state_path);
    MissionPhaseGraph::new(
        vec![Phase {
            id: phase_id,
            label: format!("{region_path}.{state_path}"),
            allowed_effectors: Vec::new(),
            allowed_engines: Vec::new(),
        }],
        Vec::new(),
        phase_id,
        &[],
    )
    .expect("single-state graph")
}

#[test]
fn canonical_region_ids_are_compile_time_const_stable() {
    // Stability rerun: read twice; should be identical.
    let m1 = CanonicalRegions::mission();
    let m2 = CanonicalRegions::mission();
    assert_eq!(m1, m2);
    // Distinct from sibling canonical regions.
    let siblings = [
        CanonicalRegions::health(),
        CanonicalRegions::comms(),
        CanonicalRegions::estimator_regime(),
        CanonicalRegions::aerodynamic_regime(),
    ];
    for s in siblings {
        assert_ne!(m1, s);
    }
}

proptest! {
    /// Guard evaluation is deterministic — same input snapshot always
    /// yields the same boolean result.
    #[test]
    fn guard_evaluation_deterministic(
        guard_region in 0u64..16,
        guard_state in 0u64..16,
        snap_region in 0u64..16,
        snap_state in 0u64..16,
    ) {
        let guard = CrossRegionGuard::new(
            RegionId::new(guard_region),
            PhaseId::new(guard_state),
        );
        let mut snap = BTreeMap::new();
        snap.insert(RegionId::new(snap_region), PhaseId::new(snap_state));

        let result1 = guard.evaluate(&snap);
        let result2 = guard.evaluate(&snap);
        prop_assert_eq!(result1, result2, "guard evaluation not deterministic");

        // Expected semantics: guard fires iff snap contains the
        // guard's (region, state) pair.
        let expected = guard_region == snap_region && guard_state == snap_state;
        prop_assert_eq!(result1, expected);
    }

    /// `current_states` iteration order is locked by RegionId.value()
    /// ascending, regardless of insertion order.
    #[test]
    fn current_states_order_independent_of_insert_order(
        seed in any::<u64>(),
    ) {
        let region_ids = [
            CanonicalRegions::mission(),
            CanonicalRegions::health(),
            CanonicalRegions::comms(),
            CanonicalRegions::estimator_regime(),
        ];
        // Insert in two different orders.
        let mut a = RegionSet::new();
        let mut b = RegionSet::new();
        for &id in &region_ids {
            let graph = build_single_state_graph(
                &format!("{id:?}"),
                &format!("mission.states.{id:?}.initial"),
            );
            a.insert(Region::new(id, graph));
        }
        // Shuffle order deterministically.
        let mut shuffled: Vec<_> = region_ids.iter().copied().collect();
        let mut rng_state = seed | 1;
        for i in (1..shuffled.len()).rev() {
            rng_state = rng_state.wrapping_mul(6364136223846793005).wrapping_add(1);
            let j = ((rng_state >> 33) as usize) % (i + 1);
            shuffled.swap(i, j);
        }
        for &id in &shuffled {
            let graph = build_single_state_graph(
                &format!("{id:?}"),
                &format!("mission.states.{id:?}.initial"),
            );
            b.insert(Region::new(id, graph));
        }

        let states_a: Vec<_> = a.current_states().into_iter().collect();
        let states_b: Vec<_> = b.current_states().into_iter().collect();
        prop_assert_eq!(states_a, states_b,
            "current_states order shifted under reinsertion");
    }

    /// Empty guard region in snapshot — guard never fires.
    #[test]
    fn guard_misses_when_region_absent(
        guard_state in 0u64..16,
    ) {
        let guard = CrossRegionGuard::new(
            CanonicalRegions::health(),
            PhaseId::new(guard_state),
        );
        let snap = BTreeMap::new();
        prop_assert!(!guard.evaluate(&snap), "guard should not fire on empty snapshot");
    }
}
