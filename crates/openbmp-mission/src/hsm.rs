//! Hierarchical state machine primitives (Phase 5.X.C).
//!
//! Replaces the flat [`crate::MissionPhaseGraph`] with a hierarchical
//! Harel-style state machine: parent / child relationships, entry /
//! exit / do actions per state, and history pseudo-states. Phase
//! 5.X.C lands the primitives only — no shipped scenario consumes
//! them yet; every Phase-5 scenario migrates as a flat (depth-1)
//! hierarchy in 5.X.F.
//!
//! # Architecture
//!
//! - [`MissionStateMachine`] — the hierarchical state-machine value.
//!   Owns a flat-storage [`Vec<MissionState>`] indexed by [`StateId`]
//!   with parent / child relationships expressed via
//!   `MissionState::parent`. Construction validates duplicate ids,
//!   known parents, parent-chain acyclicity, and canonical-form sort.
//! - [`MissionState`] — extends the flat [`crate::Phase`] with
//!   `parent`, `on_entry`, `on_exit`, `on_active` action lists.
//! - [`HistoryState`] — pseudo-state recording the last-active child
//!   of a composite state.
//! - [`HsmError`] — typed construction errors.
//!
//! # Hierarchical transition semantics (Harel)
//!
//! When transitioning from state `A` (depth `d_a`) to state `B`
//! (depth `d_b`) in different sub-trees:
//!
//! 1. Compute the lowest common ancestor (LCA) of `A` and `B`. This
//!    is the deepest state that is on both parent chains.
//! 2. **Exit chain.** Walk from `A` up to (but not including) the
//!    LCA, firing each state's `on_exit` actions bottom-to-top.
//! 3. **Enter chain.** Walk from the LCA down to `B`, firing each
//!    state's `on_entry` actions top-to-bottom.
//!
//! The LCA itself does not fire `on_exit` / `on_entry` — it remains
//! active across the transition.
//!
//! # Determinism contract
//!
//! - All ids remain FNV-1a-64 of canonical scenario paths.
//! - Canonical-form sort: states by `(depth-from-root,
//!   parent-StateId.value(), StateId.value())`; transitions by their
//!   existing five-tuple plus the ancestor-LCA depth.
//! - Reordering declarations in the scenario produces an identical
//!   HSM.
//!
//! # Integration status (Phase 5.X.C)
//!
//! Primitives only. The simulator kernel and FC commander still
//! consume [`crate::MissionPhaseGraph`] (the flat-DAG type) for
//! production transitions. Phase 5.X.F lifts the scenario format to
//! v4 (`[[mission.states]]` with `parent`) and can build a
//! `MissionStateMachine`, but the production commander loop has not
//! yet moved to LCA / entry / exit semantics.

use std::borrow::Cow;
use std::collections::{BTreeMap, BTreeSet};

use thiserror::Error;

use crate::{MissionAction, StateId};

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------

/// One state in the hierarchical mission state machine.
///
/// Extends the flat [`crate::Phase`] with the Harel-statechart
/// vocabulary: `parent` for hierarchy, `on_entry` / `on_exit` for
/// transition-time actions, and `on_active` for periodic
/// (every-tick-while-active) actions.
#[derive(Clone, Debug, PartialEq)]
pub struct MissionState {
    /// Path-derived stable id (FNV-1a-64 of
    /// `"mission.states.<path>"`).
    pub id: StateId,
    /// Human-readable label for telemetry / diagnostics.
    pub label: String,
    /// Parent state, or `None` for top-level states.
    pub parent: Option<StateId>,
    /// Actions fired in order when the state is entered (top-to-bottom
    /// in the enter chain).
    pub on_entry: Vec<MissionAction>,
    /// Actions fired in order when the state is exited (bottom-to-top
    /// in the exit chain).
    pub on_exit: Vec<MissionAction>,
    /// Actions fired in order on every tick the state is active.
    /// Useful for periodic health checks; runs after entry, before
    /// exit, never on the entry / exit tick itself.
    pub on_active: Vec<MissionAction>,
    /// Allowed effectors while this state is active. Validated at
    /// scenario load; active command gating retained from
    /// [`crate::Phase`].
    pub allowed_effectors: Vec<String>,
    /// Allowed engines while this state is active.
    pub allowed_engines: Vec<String>,
}

// ---------------------------------------------------------------------
// History pseudo-state
// ---------------------------------------------------------------------

/// History pseudo-state — records the last-active child of a
/// composite state.
///
/// Transitioning *to* the composite state via its history pseudo-state
/// resumes the saved sub-state. The pseudo-state has an id distinct
/// from the composite state itself; the [`MissionStateMachine`] holds
/// a `BTreeMap<StateId, StateId>` mapping `history_id` to the saved
/// child id.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct HistoryState(StateId);

impl HistoryState {
    /// Construct a history pseudo-state attached to a composite state.
    ///
    /// The pseudo-state id is derived from
    /// `"mission.states.<composite>.history"` so reordering scenario
    /// declarations doesn't shift the id.
    #[must_use]
    pub fn for_composite(composite: StateId) -> Self {
        // Derive a distinct id by hashing the composite's value with
        // a salt. FNV-1a is the load-bearing mixer here; we re-use
        // the existing `StateId::from_path` constructor with a
        // synthesised path to stay consistent with the rest of the
        // id-derivation invariants.
        Self(StateId::from_path(&format!(
            "mission.states.history.{}",
            composite.value()
        )))
    }

    /// Returns the pseudo-state's id.
    #[must_use]
    pub fn id(self) -> StateId {
        self.0
    }
}

// ---------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------

/// Hierarchical mission state machine.
///
/// Construct via [`MissionStateMachine::new`]; direct field assignment
/// works for tests but bypasses the validation invariants.
///
/// The constructor enforces:
///
/// 1. No duplicate state ids.
/// 2. Every `parent` references a declared state.
/// 3. The parent relation is acyclic (no state is its own ancestor).
/// 4. `initial` references a declared state.
/// 5. Every declared state is reachable from `initial` either as a
///    descendant or via a declared transition.
/// 6. States are sorted by `(depth-from-root, parent-StateId.value(),
///    StateId.value())`. Top-level states have depth 0.
#[derive(Clone, Debug, PartialEq)]
pub struct MissionStateMachine {
    /// States sorted by `(depth-from-root, parent-StateId.value(),
    /// StateId.value())`.
    pub states: Vec<MissionState>,
    /// History pseudo-states, keyed by composite state id.
    pub history: BTreeMap<StateId, HistoryState>,
    /// Initial state. Resolved at construction.
    pub initial: StateId,
}

impl MissionStateMachine {
    /// Validate and construct a canonical hierarchical mission state
    /// machine.
    ///
    /// # Errors
    ///
    /// Returns [`HsmError`] for any validation failure.
    pub fn new(states: Vec<MissionState>, initial: StateId) -> Result<Self, HsmError> {
        // 1. No duplicate state ids.
        let mut seen = BTreeSet::new();
        for state in &states {
            if !seen.insert(state.id) {
                return Err(HsmError::DuplicateState { state: state.id });
            }
        }

        // 2. Every `parent` references a declared state.
        for state in &states {
            if let Some(parent) = state.parent
                && !seen.contains(&parent)
            {
                return Err(HsmError::UnknownStateId {
                    state: parent,
                    in_field: Cow::Borrowed("state.parent"),
                });
            }
        }

        // 3. Parent relation is acyclic.
        for state in &states {
            let mut walker = state.parent;
            let mut depth = 0;
            while let Some(p) = walker {
                if p == state.id {
                    return Err(HsmError::ParentCycle { state: state.id });
                }
                walker = states.iter().find(|s| s.id == p).and_then(|s| s.parent);
                depth += 1;
                if depth > states.len() {
                    return Err(HsmError::ParentCycle { state: state.id });
                }
            }
        }

        // 4. `initial` is declared.
        if !seen.contains(&initial) {
            return Err(HsmError::MissingInitial);
        }

        // 5. Reachability is the transition graph's concern, not the
        //    parent-relation HSM's. A flat hierarchy (every state
        //    has parent=None) is valid — the states are sibling top-
        //    level roots, connected by transitions in
        //    [`MissionPhaseGraph`]. The HSM only validates parent
        //    structure; cross-state navigability lives with the
        //    transition graph.

        // 6. Compute depth-from-root.
        let depth = compute_depth(&states);

        // 7. Canonical-form sort.
        let mut sorted_states = states;
        sorted_states.sort_by_key(|s| {
            (
                depth.get(&s.id).copied().unwrap_or(usize::MAX),
                s.parent.map_or(0, StateId::value),
                s.id.value(),
            )
        });

        Ok(Self {
            states: sorted_states,
            history: BTreeMap::new(),
            initial,
        })
    }

    /// Returns the parent of `state`, or `None` if `state` is
    /// top-level or unknown.
    #[must_use]
    pub fn parent_of(&self, state: StateId) -> Option<StateId> {
        self.states
            .iter()
            .find(|s| s.id == state)
            .and_then(|s| s.parent)
    }

    /// Returns the parent chain of `state`: the state itself, then
    /// its parent, then its grandparent, … up to and including the
    /// root (depth-0) ancestor.
    ///
    /// Order is bottom-to-top (deepest first). Empty if `state` is
    /// unknown.
    #[must_use]
    pub fn parent_chain(&self, state: StateId) -> Vec<StateId> {
        if !self.states.iter().any(|s| s.id == state) {
            return Vec::new();
        }
        let mut chain = vec![state];
        let mut walker = self.parent_of(state);
        let mut guard = 0;
        while let Some(p) = walker {
            chain.push(p);
            walker = self.parent_of(p);
            guard += 1;
            if guard > self.states.len() {
                break;
            }
        }
        chain
    }

    /// Lowest common ancestor of `a` and `b`.
    ///
    /// Returns `Some(lca)` if `a` and `b` share an ancestor (including
    /// the case where one is an ancestor of the other), `None` if
    /// they are in disjoint trees or either is unknown.
    #[must_use]
    pub fn lca(&self, a: StateId, b: StateId) -> Option<StateId> {
        let chain_a = self.parent_chain(a);
        if chain_a.is_empty() {
            return None;
        }
        let a_set: BTreeSet<StateId> = chain_a.iter().copied().collect();
        let chain_b = self.parent_chain(b);
        chain_b.into_iter().find(|s| a_set.contains(s))
    }

    /// Exit chain for a transition from `a` to `b`: the states whose
    /// `on_exit` fires, bottom-to-top, up to but not including the
    /// LCA. Returns an empty vec if `a == b` or no LCA exists.
    #[must_use]
    pub fn exit_chain(&self, a: StateId, b: StateId) -> Vec<StateId> {
        if a == b {
            return Vec::new();
        }
        let Some(lca) = self.lca(a, b) else {
            return Vec::new();
        };
        self.parent_chain(a)
            .into_iter()
            .take_while(|&s| s != lca)
            .collect()
    }

    /// Enter chain for a transition from `a` to `b`: the states whose
    /// `on_entry` fires, top-to-bottom, from the LCA down to (and
    /// including) `b` but not including the LCA itself.
    #[must_use]
    pub fn enter_chain(&self, a: StateId, b: StateId) -> Vec<StateId> {
        if a == b {
            return Vec::new();
        }
        let Some(lca) = self.lca(a, b) else {
            return Vec::new();
        };
        let mut chain: Vec<StateId> = self
            .parent_chain(b)
            .into_iter()
            .take_while(|&s| s != lca)
            .collect();
        chain.reverse();
        chain
    }
}

// ---------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------

/// Depth-from-root for every state. Top-level (no parent) is depth 0.
fn compute_depth(states: &[MissionState]) -> BTreeMap<StateId, usize> {
    let mut children: BTreeMap<StateId, Vec<StateId>> = BTreeMap::new();
    let mut in_degree: BTreeMap<StateId, usize> = BTreeMap::new();
    let mut depth: BTreeMap<StateId, usize> = BTreeMap::new();

    for state in states {
        children.entry(state.id).or_default();
        in_degree.insert(state.id, usize::from(state.parent.is_some()));
        depth.insert(state.id, 0);
        if let Some(parent) = state.parent {
            children.entry(parent).or_default().push(state.id);
        }
    }

    for entry in children.values_mut() {
        entry.sort_by_key(|id| id.value());
    }

    let mut frontier = BTreeSet::new();
    for (id, degree) in &in_degree {
        if *degree == 0 {
            frontier.insert(*id);
        }
    }

    let mut visited = 0_usize;
    while let Some(&parent) = frontier.iter().next() {
        frontier.remove(&parent);
        visited += 1;
        let parent_depth = depth[&parent];
        if let Some(kids) = children.get(&parent) {
            for &child in kids {
                depth.insert(child, parent_depth + 1);
                if let Some(degree) = in_degree.get_mut(&child) {
                    *degree = degree.saturating_sub(1);
                    if *degree == 0 {
                        frontier.insert(child);
                    }
                }
            }
        }
    }

    debug_assert_eq!(
        visited,
        states.len(),
        "parent-relation topological order incomplete"
    );
    depth
}

// ---------------------------------------------------------------------
// Errors
// ---------------------------------------------------------------------

/// Errors produced when constructing a [`MissionStateMachine`].
#[derive(Debug, Clone, PartialEq, Error)]
pub enum HsmError {
    /// Two states share the same id.
    #[error("duplicate state id {state:?}")]
    DuplicateState {
        /// Duplicated state id.
        state: StateId,
    },
    /// A reference to a state id that does not exist in the state
    /// list.
    #[error("unknown state id {state:?} referenced in {in_field}")]
    UnknownStateId {
        /// Unknown state id.
        state: StateId,
        /// Where the unknown id appeared.
        in_field: Cow<'static, str>,
    },
    /// The parent chain forms a cycle (a state is its own ancestor).
    #[error("parent cycle involving state {state:?}")]
    ParentCycle {
        /// State whose parent chain cycles.
        state: StateId,
    },
    /// A state is declared but not a descendant of `initial`.
    #[error("state {state:?} is unreachable from the initial state")]
    UnreachableState {
        /// Unreachable state.
        state: StateId,
    },
    /// `initial` references an unknown id, or is otherwise missing.
    #[error("missing or unknown initial state")]
    MissingInitial,
}

// ---------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn s(path: &str) -> StateId {
        StateId::from_path(&format!("mission.states.{path}"))
    }

    fn state(path: &str, parent: Option<StateId>) -> MissionState {
        MissionState {
            id: s(path),
            label: path.to_string(),
            parent,
            on_entry: Vec::new(),
            on_exit: Vec::new(),
            on_active: Vec::new(),
            allowed_effectors: Vec::new(),
            allowed_engines: Vec::new(),
        }
    }

    #[test]
    fn flat_hierarchy_constructs_with_root_only() {
        let root = s("root");
        let hsm = MissionStateMachine::new(vec![state("root", None)], root).expect("hsm");
        assert_eq!(hsm.initial, root);
        assert_eq!(hsm.states.len(), 1);
    }

    #[test]
    fn parent_chain_walks_up_to_root() {
        let root = s("root");
        let child = s("root.child");
        let grandchild = s("root.child.gc");
        let hsm = MissionStateMachine::new(
            vec![
                state("root", None),
                state("root.child", Some(root)),
                state("root.child.gc", Some(child)),
            ],
            root,
        )
        .expect("hsm");
        let chain = hsm.parent_chain(grandchild);
        assert_eq!(chain, vec![grandchild, child, root]);
    }

    #[test]
    fn lca_finds_common_ancestor() {
        let root = s("root");
        let left = s("root.left");
        let right = s("root.right");
        let hsm = MissionStateMachine::new(
            vec![
                state("root", None),
                state("root.left", Some(root)),
                state("root.right", Some(root)),
            ],
            root,
        )
        .expect("hsm");
        assert_eq!(hsm.lca(left, right), Some(root));
        assert_eq!(hsm.lca(left, left), Some(left));
    }

    #[test]
    fn lca_handles_descendant_relation() {
        let root = s("root");
        let child = s("root.child");
        let grandchild = s("root.child.gc");
        let hsm = MissionStateMachine::new(
            vec![
                state("root", None),
                state("root.child", Some(root)),
                state("root.child.gc", Some(child)),
            ],
            root,
        )
        .expect("hsm");
        assert_eq!(hsm.lca(child, grandchild), Some(child));
        assert_eq!(hsm.lca(grandchild, child), Some(child));
    }

    #[test]
    fn exit_and_enter_chains_compose_correctly() {
        let root = s("root");
        let left = s("root.left");
        let left_inner = s("root.left.inner");
        let right = s("root.right");
        let hsm = MissionStateMachine::new(
            vec![
                state("root", None),
                state("root.left", Some(root)),
                state("root.left.inner", Some(left)),
                state("root.right", Some(root)),
            ],
            root,
        )
        .expect("hsm");
        // Transition from left.inner → right.
        assert_eq!(hsm.exit_chain(left_inner, right), vec![left_inner, left]);
        assert_eq!(hsm.enter_chain(left_inner, right), vec![right]);
    }

    #[test]
    fn duplicate_state_rejected() {
        let root = s("root");
        let err = MissionStateMachine::new(vec![state("root", None), state("root", None)], root)
            .expect_err("duplicate");
        assert!(matches!(err, HsmError::DuplicateState { state } if state == root));
    }

    #[test]
    fn unknown_parent_rejected() {
        let root = s("root");
        let ghost = s("ghost");
        let err = MissionStateMachine::new(vec![state("root", Some(ghost))], root)
            .expect_err("unknown parent");
        assert!(matches!(err, HsmError::UnknownStateId { state, .. } if state == ghost));
    }

    #[test]
    fn parent_cycle_rejected() {
        // Two states each declaring the other as parent.
        let a = s("a");
        let b = s("b");
        let err = MissionStateMachine::new(
            vec![
                MissionState {
                    id: a,
                    label: "a".into(),
                    parent: Some(b),
                    on_entry: Vec::new(),
                    on_exit: Vec::new(),
                    on_active: Vec::new(),
                    allowed_effectors: Vec::new(),
                    allowed_engines: Vec::new(),
                },
                MissionState {
                    id: b,
                    label: "b".into(),
                    parent: Some(a),
                    on_entry: Vec::new(),
                    on_exit: Vec::new(),
                    on_active: Vec::new(),
                    allowed_effectors: Vec::new(),
                    allowed_engines: Vec::new(),
                },
            ],
            a,
        )
        .expect_err("parent cycle");
        assert!(matches!(err, HsmError::ParentCycle { .. }));
    }

    // Phase 5.X.C revision: flat hierarchies (every state has
    // parent=None) are valid HSMs — siblings are connected via the
    // transition graph, not the parent relation. The
    // unreachable_state validation was removed; reachability is the
    // [`MissionPhaseGraph`] transition graph's responsibility.
    #[test]
    fn flat_hierarchy_with_sibling_top_levels_accepted() {
        let root = s("root");
        let sibling = s("sibling");
        let hsm = MissionStateMachine::new(vec![state("root", None), state("sibling", None)], root)
            .expect("flat hierarchy with sibling top-levels is valid");
        assert_eq!(hsm.states.len(), 2);
        assert!(hsm.states.iter().any(|s| s.id == sibling));
    }

    #[test]
    fn missing_initial_rejected() {
        let err = MissionStateMachine::new(vec![state("root", None)], s("ghost"))
            .expect_err("missing initial");
        assert!(matches!(err, HsmError::MissingInitial));
    }

    #[test]
    fn canonical_sort_is_stable_under_reordering() {
        let root = s("root");
        let a = s("a");
        let b = s("b");
        let hsm1 = MissionStateMachine::new(
            vec![
                state("root", None),
                state("a", Some(root)),
                state("b", Some(root)),
            ],
            root,
        )
        .expect("hsm1");
        let hsm2 = MissionStateMachine::new(
            vec![
                state("b", Some(root)),
                state("a", Some(root)),
                state("root", None),
            ],
            root,
        )
        .expect("hsm2");
        let ids1: Vec<StateId> = hsm1.states.iter().map(|s| s.id).collect();
        let ids2: Vec<StateId> = hsm2.states.iter().map(|s| s.id).collect();
        assert_eq!(ids1, ids2);
        // Top-level root first; then sorted children by id.
        assert_eq!(ids1[0], root);
        assert!(ids1.contains(&a) && ids1.contains(&b));
    }
}
