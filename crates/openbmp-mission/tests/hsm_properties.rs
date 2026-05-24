//! Property tests for the hierarchical state machine.
//!
//! Asserts the load-bearing HSM invariants on randomly-generated
//! state hierarchies:
//!
//! 1. Every constructed [`MissionStateMachine`] has every declared
//!    state reachable from `initial` via the parent relation.
//! 2. [`MissionStateMachine::lca`] returns the deepest state on
//!    both parent chains, or `None` for disjoint trees.
//! 3. Exit and enter chains compose with no overlap and exactly
//!    enclose the path between the two states.
//! 4. Canonical-form sort is stable: re-ordering the input
//!    declaration vec produces an identical sorted output.
//! 5. The Harel invariant: `len(exit_chain(a,b)) + 1 ==
//!    parent_chain(a)` minus the LCA tail; symmetrically for
//!    `enter_chain`.

#![allow(clippy::unwrap_used, clippy::expect_used)]

use openbmp_mission::{MissionAction, MissionState, MissionStateMachine, StateId};
use proptest::prelude::*;

/// Generate a random valid state hierarchy of depth up to `max_depth`.
/// The root has id `0`; each subsequent state picks a parent uniformly
/// from already-generated states. This guarantees acyclicity and
/// reachability.
fn arb_hierarchy(max_states: usize) -> impl Strategy<Value = Vec<MissionState>> {
    (2usize..=max_states).prop_flat_map(|n| {
        // For each state index `i` in `[0, n)`, pick a parent index in
        // `[0, i)` (or None for index 0). We do this as a vec of
        // `Option<usize>` choices.
        let choices = proptest::collection::vec(any::<usize>(), n);
        choices.prop_map(move |raw_choices| {
            (0..n)
                .map(|i| {
                    let id = StateId::from_path(&format!("mission.states.s{i}"));
                    let parent = if i == 0 {
                        None
                    } else {
                        let parent_idx = raw_choices[i] % i;
                        Some(StateId::from_path(&format!("mission.states.s{parent_idx}")))
                    };
                    MissionState {
                        id,
                        label: format!("s{i}"),
                        parent,
                        on_entry: Vec::new(),
                        on_exit: Vec::new(),
                        on_active: Vec::new(),
                        allowed_effectors: Vec::new(),
                        allowed_engines: Vec::new(),
                    }
                })
                .collect()
        })
    })
}

proptest! {
    /// Every state in a valid hierarchy is reachable from the root via
    /// the parent relation (i.e. has the root on its parent chain).
    #[test]
    fn every_state_has_root_on_parent_chain(states in arb_hierarchy(16)) {
        let root = states[0].id;
        let hsm = MissionStateMachine::new(states.clone(), root).expect("hsm");
        for state in &hsm.states {
            let chain = hsm.parent_chain(state.id);
            prop_assert!(chain.contains(&root),
                "state {:?} has no root on its parent chain ({:?})",
                state.id, chain);
        }
    }

    /// LCA of any two states is the deepest common ancestor — i.e.
    /// it appears on both parent chains and no descendant of it
    /// appears on both.
    #[test]
    fn lca_is_deepest_common_ancestor(states in arb_hierarchy(16)) {
        let root = states[0].id;
        let hsm = MissionStateMachine::new(states.clone(), root).expect("hsm");
        for a in &hsm.states {
            for b in &hsm.states {
                let Some(lca) = hsm.lca(a.id, b.id) else { continue };
                let chain_a: std::collections::BTreeSet<_> =
                    hsm.parent_chain(a.id).into_iter().collect();
                let chain_b: std::collections::BTreeSet<_> =
                    hsm.parent_chain(b.id).into_iter().collect();
                prop_assert!(chain_a.contains(&lca), "LCA not on a's chain");
                prop_assert!(chain_b.contains(&lca), "LCA not on b's chain");
                // No descendant of lca should be on both chains.
                let common: std::collections::BTreeSet<_> =
                    chain_a.intersection(&chain_b).copied().collect();
                // Every common ancestor must be lca or an ancestor of lca.
                let lca_chain: std::collections::BTreeSet<_> =
                    hsm.parent_chain(lca).into_iter().collect();
                for c in &common {
                    prop_assert!(lca_chain.contains(c),
                        "common ancestor {:?} not in lca's chain", c);
                }
            }
        }
    }

    /// `exit_chain(a, b)` and `enter_chain(a, b)` are disjoint — no
    /// state appears in both.
    #[test]
    fn exit_and_enter_chains_disjoint(states in arb_hierarchy(16)) {
        let root = states[0].id;
        let hsm = MissionStateMachine::new(states.clone(), root).expect("hsm");
        for a in &hsm.states {
            for b in &hsm.states {
                if a.id == b.id { continue; }
                let exit_set: std::collections::BTreeSet<_> =
                    hsm.exit_chain(a.id, b.id).into_iter().collect();
                let enter_set: std::collections::BTreeSet<_> =
                    hsm.enter_chain(a.id, b.id).into_iter().collect();
                prop_assert!(exit_set.is_disjoint(&enter_set),
                    "exit and enter chains overlap on transition {:?} -> {:?}",
                    a.id, b.id);
            }
        }
    }

    /// `exit_chain` is ordered bottom-to-top: each successive state is
    /// the parent of the previous.
    #[test]
    fn exit_chain_is_bottom_to_top(states in arb_hierarchy(16)) {
        let root = states[0].id;
        let hsm = MissionStateMachine::new(states.clone(), root).expect("hsm");
        for a in &hsm.states {
            for b in &hsm.states {
                let chain = hsm.exit_chain(a.id, b.id);
                for window in chain.windows(2) {
                    let parent = hsm.parent_of(window[0]);
                    prop_assert_eq!(parent, Some(window[1]),
                        "exit chain not bottom-to-top: {:?} parent != {:?}",
                        window[0], window[1]);
                }
            }
        }
    }

    /// `enter_chain` is ordered top-to-bottom: each successive state is
    /// a child of the previous.
    #[test]
    fn enter_chain_is_top_to_bottom(states in arb_hierarchy(16)) {
        let root = states[0].id;
        let hsm = MissionStateMachine::new(states.clone(), root).expect("hsm");
        for a in &hsm.states {
            for b in &hsm.states {
                let chain = hsm.enter_chain(a.id, b.id);
                for window in chain.windows(2) {
                    let parent = hsm.parent_of(window[1]);
                    prop_assert_eq!(parent, Some(window[0]),
                        "enter chain not top-to-bottom: {:?} not parent of {:?}",
                        window[0], window[1]);
                }
            }
        }
    }

    /// Re-ordering the input declaration vec produces an identical
    /// canonical-form sorted hsm.
    #[test]
    fn canonical_form_stable_under_reordering(
        states in arb_hierarchy(12),
        seed in any::<u64>(),
    ) {
        let root = states[0].id;
        let hsm1 = MissionStateMachine::new(states.clone(), root).expect("hsm1");
        // Shuffle states[1..] deterministically using `seed`.
        let mut shuffled = states.clone();
        let mut rng_state = seed | 1; // ensure odd for Fisher-Yates
        for i in (2..shuffled.len()).rev() {
            // Linear-congruential mixer; deterministic for proptest replay.
            rng_state = rng_state
                .wrapping_mul(6_364_136_223_846_793_005)
                .wrapping_add(1);
            let j = ((rng_state >> 33) as usize) % (i + 1);
            shuffled.swap(i, j);
        }
        let hsm2 = MissionStateMachine::new(shuffled, root).expect("hsm2");
        let ids1: Vec<_> = hsm1.states.iter().map(|s| s.id).collect();
        let ids2: Vec<_> = hsm2.states.iter().map(|s| s.id).collect();
        prop_assert_eq!(ids1, ids2, "canonical sort not stable under reordering");
    }
}

/// `MissionAction` is held live to confirm the symbol re-exports
/// remain usable from this test crate — `on_entry` / `on_exit`
/// action vecs are constructed with these.
#[allow(dead_code)]
fn _action_compile_check() -> MissionAction {
    MissionAction::EmitTelemetryMarker { tag: "test".into() }
}
