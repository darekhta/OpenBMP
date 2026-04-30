//! `openbmp-mission` — hardware-portable mission state machine.
//!
//! Phase-3.14.B: extracted from `openbmp-sim` so any flight code that
//! consumes the `MissionPhaseGraph` / `EventTrigger` /
//! `EventBinding` vocabulary does not transitively depend on the
//! simulation kernel.
//!
//! The simulator-side event evaluator stays in `openbmp-sim`; this
//! crate ships only the data shapes + the trigger trait + the graph
//! validator, which are the items a real-hardware adopter would also
//! need.

mod events;

pub use events::{
    BuiltInEventTrigger, EventAction, EventBinding, EventEvalState, EventId, EventScalars,
    EventTrigger, FiredEvent, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition,
};
