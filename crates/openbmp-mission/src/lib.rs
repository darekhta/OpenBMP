//! `openbmp-mission` — hardware-portable mission state machine.
//!
//! Phase-3.14.B: extracted from `openbmp-sim` so any flight code that
//! consumes the `MissionPhaseGraph` / `EventTrigger` /
//! `EventBinding` vocabulary does not transitively depend on the
//! simulation kernel.
//!
//! Phase 5.X.A: the action taxonomy was split. This crate ships only
//! the HAL-portable [`MissionAction`] enum; the simulator-only
//! scenario-script actions (engine / effector / separation /
//! recovery) live in the separate `openbmp-scenario-script` crate.
//! [`EventBinding`] and [`FiredEvent`] are now generic over the
//! action type.
//!
//! The simulator-side event evaluator stays in `openbmp-sim`; this
//! crate ships only the data shapes + the trigger trait + the graph
//! validator, which are the items a real-hardware adopter would also
//! need.

mod events;
mod hsm;
mod regions;

pub use events::{
    BuiltInEventTrigger, EventBinding, EventEvalState, EventId, EventScalars, EventTrigger,
    FiredEvent, MissionAction, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition, RegionId, StateId,
};
pub use hsm::{HistoryState, HsmError, MissionState, MissionStateMachine};
pub use regions::{CanonicalRegions, CrossRegionGuard, Region, RegionSet};
