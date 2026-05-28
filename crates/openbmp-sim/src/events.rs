//! Event triggers and mission-state machine — re-exported.
//!
//! The event / mission graph data shapes live in
//! `openbmp-mission` so flight-controller code and HAL
//! adopters can consume them without depending on the simulator.
//! [`openbmp_scenario_script::ScenarioScriptAction`]
//! (the simulator-only physics-override action type) is
//! re-exported here for ergonomic access.
//! This module is now a thin re-export to preserve every existing
//! `openbmp_sim::events::*` import path.

pub use openbmp_mission::AlarmCode;
pub use openbmp_mission::{
    BuiltInEventTrigger, EventBinding, EventEvalState, EventId, EventScalars, EventTrigger,
    FiredEvent, MissionAction, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition, RegionId, RelativeDistanceKey, StateId,
};
pub use openbmp_scenario_script::ScenarioScriptAction;
