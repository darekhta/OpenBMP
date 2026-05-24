//! `MissionPhaseGraph` validation + ordering tests.
//!
//! Covers:
//!
//! - Valid graph construction and canonical ordering.
//! - Order-stable construction: shuffling the input vectors produces
//!   bit-identical output graphs.
//! - Negative tests: cycle, self-loop, unreachable phase, unknown
//!   phase id (initial / transition.from / transition.to), unknown
//!   event id in transition, duplicate phase id, missing initial.
//!
//! These cover the validation + canonicalisation contract described
//! at [`openbmp_sim::MissionPhaseGraph`] and in
//! `docs/scenario-format.md § Mission blocks`.

#![allow(
    clippy::expect_used,
    clippy::float_cmp,
    clippy::panic,
    clippy::unwrap_used
)]

use openbmp_sim::{EventId, MissionGraphError, MissionPhaseGraph, Phase, PhaseId, PhaseTransition};

fn phase(id: &str, label: &str) -> Phase {
    Phase {
        id: PhaseId::from_path(id),
        label: label.to_owned(),
        allowed_effectors: Vec::new(),
        allowed_engines: Vec::new(),
    }
}

fn transition(from: &str, to: &str, event: &str) -> PhaseTransition {
    PhaseTransition {
        from: PhaseId::from_path(from),
        to: PhaseId::from_path(to),
        event: EventId::from_path(event),
    }
}

fn linear_three_phase_inputs() -> (Vec<Phase>, Vec<PhaseTransition>, PhaseId, Vec<EventId>) {
    let phases = vec![
        phase("ascent", "ascent"),
        phase("pre_launch", "pre-launch"),
        phase("descent", "descent"),
    ];
    let transitions = vec![
        transition("pre_launch", "ascent", "ignition"),
        transition("ascent", "descent", "at_apogee_marker"),
    ];
    let initial = PhaseId::from_path("pre_launch");
    let events = vec![
        EventId::from_path("ignition"),
        EventId::from_path("at_apogee_marker"),
    ];
    (phases, transitions, initial, events)
}

#[test]
fn valid_linear_graph_constructs() {
    let (phases, transitions, initial, events) = linear_three_phase_inputs();
    let g = MissionPhaseGraph::new(phases, transitions, initial, &events).expect("valid");
    assert_eq!(g.initial, PhaseId::from_path("pre_launch"));
    assert_eq!(g.phases.len(), 3);
    assert_eq!(g.transitions.len(), 2);
    // Canonical order: depth 0 then 1 then 2.
    assert_eq!(g.phases[0].id, PhaseId::from_path("pre_launch"));
    assert_eq!(g.phases[1].id, PhaseId::from_path("ascent"));
    assert_eq!(g.phases[2].id, PhaseId::from_path("descent"));
}

#[test]
fn graph_construction_is_input_order_independent() {
    let (phases_a, transitions_a, initial, events) = linear_three_phase_inputs();
    let g_a = MissionPhaseGraph::new(phases_a, transitions_a, initial, &events).expect("a");

    // Same phases / transitions, reversed input order.
    let (mut phases_b, mut transitions_b, _, _) = linear_three_phase_inputs();
    phases_b.reverse();
    transitions_b.reverse();
    let mut events_rev = events.clone();
    events_rev.reverse();
    let g_b = MissionPhaseGraph::new(phases_b, transitions_b, initial, &events_rev).expect("b");

    assert_eq!(g_a, g_b, "graphs must be order-independent");
}

#[test]
fn duplicate_phase_id_rejected() {
    let phases = vec![phase("ascent", "ascent #1"), phase("ascent", "ascent #2")];
    let transitions = Vec::new();
    let initial = PhaseId::from_path("ascent");
    let events = Vec::new();
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(err, MissionGraphError::DuplicatePhase { .. }));
}

#[test]
fn missing_initial_rejected() {
    let phases = vec![phase("ascent", "ascent"), phase("descent", "descent")];
    let transitions = Vec::new();
    let initial = PhaseId::from_path("nope");
    let events = Vec::new();
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(err, MissionGraphError::MissingInitial));
}

#[test]
fn unknown_phase_in_transition_from_rejected() {
    let phases = vec![phase("ascent", "ascent"), phase("descent", "descent")];
    let transitions = vec![transition("ghost", "descent", "at_apogee_marker")];
    let initial = PhaseId::from_path("ascent");
    let events = vec![EventId::from_path("at_apogee_marker")];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(
        err,
        MissionGraphError::UnknownPhaseId { ref in_field, .. }
            if in_field.as_ref() == "transition.from"
    ));
}

#[test]
fn unknown_phase_in_transition_to_rejected() {
    let phases = vec![phase("ascent", "ascent"), phase("descent", "descent")];
    let transitions = vec![transition("ascent", "ghost", "at_apogee_marker")];
    let initial = PhaseId::from_path("ascent");
    let events = vec![EventId::from_path("at_apogee_marker")];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(
        err,
        MissionGraphError::UnknownPhaseId { ref in_field, .. }
            if in_field.as_ref() == "transition.to"
    ));
}

#[test]
fn unknown_event_in_transition_rejected() {
    let phases = vec![phase("ascent", "ascent"), phase("descent", "descent")];
    let transitions = vec![transition("ascent", "descent", "ghost")];
    let initial = PhaseId::from_path("ascent");
    let events = vec![EventId::from_path("at_apogee_marker")];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(err, MissionGraphError::UnknownEvent { .. }));
}

#[test]
fn duplicate_event_id_rejected() {
    let phases = vec![phase("ascent", "ascent"), phase("descent", "descent")];
    let transitions = vec![transition("ascent", "descent", "at_apogee_marker")];
    let initial = PhaseId::from_path("ascent");
    let events = vec![
        EventId::from_path("at_apogee_marker"),
        EventId::from_path("at_apogee_marker"),
    ];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(err, MissionGraphError::DuplicateEvent { .. }));
}

#[test]
fn ambiguous_transition_rejected() {
    let phases = vec![
        phase("ascent", "ascent"),
        phase("descent", "descent"),
        phase("abort", "abort"),
    ];
    let transitions = vec![
        transition("ascent", "descent", "at_apogee_marker"),
        transition("ascent", "abort", "at_apogee_marker"),
    ];
    let initial = PhaseId::from_path("ascent");
    let events = vec![EventId::from_path("at_apogee_marker")];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(err, MissionGraphError::AmbiguousTransition { .. }));
}

#[test]
fn cycle_rejected() {
    let phases = vec![phase("a", "a"), phase("b", "b"), phase("c", "c")];
    let transitions = vec![
        transition("a", "b", "e1"),
        transition("b", "c", "e2"),
        transition("c", "a", "e3"), // cycle
    ];
    let initial = PhaseId::from_path("a");
    let events = vec![
        EventId::from_path("e1"),
        EventId::from_path("e2"),
        EventId::from_path("e3"),
    ];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    let MissionGraphError::Cycle { involving } = err else {
        panic!("expected Cycle, got {err:?}");
    };
    assert_eq!(involving.len(), 3);
}

#[test]
fn self_loop_rejected() {
    let phases = vec![phase("a", "a")];
    let transitions = vec![transition("a", "a", "loop")];
    let initial = PhaseId::from_path("a");
    let events = vec![EventId::from_path("loop")];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    let MissionGraphError::Cycle { involving } = err else {
        panic!("expected Cycle, got {err:?}");
    };
    assert_eq!(involving, vec![PhaseId::from_path("a")]);
}

#[test]
fn self_loop_on_middle_phase_rejected() {
    let phases = vec![phase("a", "a"), phase("b", "b"), phase("c", "c")];
    let transitions = vec![
        transition("a", "b", "e1"),
        transition("b", "b", "loop"),
        transition("b", "c", "e2"),
    ];
    let initial = PhaseId::from_path("a");
    let events = vec![
        EventId::from_path("e1"),
        EventId::from_path("loop"),
        EventId::from_path("e2"),
    ];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    let MissionGraphError::Cycle { involving } = err else {
        panic!("expected Cycle, got {err:?}");
    };
    assert_eq!(involving, vec![PhaseId::from_path("b")]);
}

#[test]
fn unreachable_phase_rejected() {
    // `descent` is declared but no transition leads to it from
    // `ascent`.
    let phases = vec![
        phase("ascent", "ascent"),
        phase("descent", "descent"),
        phase("orphan", "orphan"),
    ];
    let transitions = vec![transition("ascent", "descent", "at_apogee_marker")];
    let initial = PhaseId::from_path("ascent");
    let events = vec![EventId::from_path("at_apogee_marker")];
    let err = MissionPhaseGraph::new(phases, transitions, initial, &events).unwrap_err();
    assert!(matches!(
        err,
        MissionGraphError::UnreachablePhase { phase: id }
            if id == PhaseId::from_path("orphan")
    ));
}

#[test]
fn empty_transitions_with_single_phase_accepted() {
    // Minimal valid graph: one phase, no transitions.
    let phases = vec![phase("only", "only phase")];
    let transitions = Vec::new();
    let initial = PhaseId::from_path("only");
    let events = Vec::new();
    let g = MissionPhaseGraph::new(phases, transitions, initial, &events).expect("valid");
    assert_eq!(g.phases.len(), 1);
    assert_eq!(g.transitions.len(), 0);
}

#[test]
fn diamond_graph_canonical_depth_ordering() {
    // pre_launch → ascent → apogee → descent
    //              ascent → coast → descent
    //
    // Depth(pre_launch) = 0
    // Depth(ascent)     = 1
    // Depth(apogee)     = 2
    // Depth(coast)      = 2
    // Depth(descent)    = 3
    //
    // The canonical order is by (depth, PhaseId.value()).
    let phases = vec![
        phase("pre_launch", "pre"),
        phase("ascent", "asc"),
        phase("apogee", "apo"),
        phase("coast", "coa"),
        phase("descent", "des"),
    ];
    let transitions = vec![
        transition("pre_launch", "ascent", "ignition"),
        transition("ascent", "apogee", "at_apogee_marker"),
        transition("ascent", "coast", "burnout"),
        transition("apogee", "descent", "deploy_drogue"),
        transition("coast", "descent", "deploy_drogue"),
    ];
    let initial = PhaseId::from_path("pre_launch");
    let events = vec![
        EventId::from_path("ignition"),
        EventId::from_path("at_apogee_marker"),
        EventId::from_path("burnout"),
        EventId::from_path("deploy_drogue"),
    ];
    let g = MissionPhaseGraph::new(phases, transitions, initial, &events).expect("valid");
    // First and last positions are pinned by depth.
    assert_eq!(g.phases[0].id, PhaseId::from_path("pre_launch"));
    assert_eq!(g.phases[1].id, PhaseId::from_path("ascent"));
    // Two depth-2 phases are tied; sorted by PhaseId.value() — depends
    // on the FNV hash. We just assert both are present at indices 2/3.
    let depth2: std::collections::BTreeSet<PhaseId> =
        [g.phases[2].id, g.phases[3].id].into_iter().collect();
    assert_eq!(
        depth2,
        [PhaseId::from_path("apogee"), PhaseId::from_path("coast")]
            .into_iter()
            .collect()
    );
    assert_eq!(g.phases[4].id, PhaseId::from_path("descent"));
}
