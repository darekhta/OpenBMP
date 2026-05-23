//! Event triggers and mission-state machine — re-exported.
//!
//! Phase-3.14.B moved the event / mission graph data shapes to
//! `openbmp-mission` so flight-controller code (Phase 4) and HAL
//! adopters can consume them without depending on the simulator.
//! Phase 5.X.A added [`openbmp_scenario_script::ScenarioScriptAction`]
//! (the simulator-only physics-override action type) which the
//! simulator re-exports here for ergonomic access.
//! This module is now a thin re-export to preserve every existing
//! `openbmp_sim::events::*` import path.

pub use openbmp_mission::AlarmCode;
pub use openbmp_mission::{
    BuiltInEventTrigger, EventBinding, EventEvalState, EventId, EventScalars, EventTrigger,
    FiredEvent, MissionAction, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition, RegionId, StateId,
};
pub use openbmp_scenario_script::ScenarioScriptAction;
