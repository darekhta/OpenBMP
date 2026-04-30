//! Event triggers and mission-phase graph — re-exported from
//! `openbmp-mission`.
//!
//! Phase-3.14.B moved the event / mission graph data shapes to
//! `openbmp-mission` so flight-controller code (Phase 4) and HAL
//! adopters can consume them without depending on the simulator.
//! This module is now a thin re-export to preserve every existing
//! `openbmp_sim::events::*` import path.

pub use openbmp_mission::{
    BuiltInEventTrigger, EventAction, EventBinding, EventEvalState, EventId, EventScalars,
    EventTrigger, FiredEvent, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition,
};
