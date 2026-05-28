//! `openbmp-mission` — hardware-portable mission state machine.
//!
//! Lives apart from `openbmp-sim` so any flight code that
//! consumes the `MissionPhaseGraph` / `EventTrigger` /
//! `EventBinding` vocabulary does not transitively depend on the
//! simulation kernel.
//!
//! The action taxonomy is split: this crate ships only
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
    AlarmCode, BuiltInEventTrigger, EventBinding, EventEvalState, EventId, EventScalars,
    EventTrigger, FiredEvent, MissionAction, MissionGraphError, MissionPhaseGraph, Phase, PhaseId,
    PhaseTransition, RegionId, RelativeDistanceKey, StateId,
};
pub use hsm::{HistoryState, HsmError, MissionState, MissionStateMachine};
pub use regions::{
    CanonicalRegionStates, CanonicalRegions, CrossRegionGuard, Region, RegionError, RegionSet,
};
