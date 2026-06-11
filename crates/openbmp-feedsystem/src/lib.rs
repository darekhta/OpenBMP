//! `openbmp-feedsystem` — deterministic feed-network primitives.
//!
//! This crate is the L2 propulsion feed-system boundary. It owns reduced
//! tank/valve/chamber network solves and later transient line/turbopump/POGO
//! feed models. It depends only on `openbmp-core` plus local error plumbing; it
//! has no dependency on `openbmp-sim`, `openbmp-runner`, or `openbmp-fc`.
//!
//! The first shipped models are intentionally small: single-fluid
//! tank-valve-chamber equilibrium solves, a steady node/branch graph, and a
//! lumped transient chamber-pressure stepper. They stay off the runner hot path
//! unless a scenario block opts into them.

pub mod chamber;
pub mod control;
pub mod error;
pub mod graph;
pub mod line;
pub mod network;
pub mod pogo;
pub mod pump;
pub mod transient;

pub use chamber::{
    ChamberFeedInput, TransientChamber, TransientChamberConfig, TransientChamberSnapshot,
    TransientChamberState,
};
pub use control::{
    ThrottleMixtureController, ThrottleMixtureControllerConfig, ThrottleMixtureControllerSnapshot,
    ThrottleMixtureControllerState, ThrottleMixtureMeasurement, ThrottleMixtureSetpoint,
    ValveCommandPair,
};
pub use error::FeedSystemError;
pub use graph::{
    FeedGraphNode, FeedGraphNodeKind, SteadyFeedGraph, SteadyFeedGraphConfig,
    SteadyFeedGraphSnapshot, ValveBranch,
};
pub use line::{MocLine, MocLineBoundary, MocLineConfig, MocLineSnapshot, MocLineState};
pub use network::{
    FeedCommand, FeedNetwork, FeedNetworkSnapshot, TankValveChamberConfig, TankValveChamberNetwork,
};
pub use pogo::{
    PogoFeedCouplingConfig, PogoModeConfig, PogoStability, PogoStabilityConfig,
    PogoStabilitySnapshot, PogoStabilityVerdict,
};
pub use pump::{
    NormalizedPumpMap, PumpCavitationState, Turbopump, TurbopumpConfig, TurbopumpDesignPoint,
    TurbopumpOperatingPoint, TurbopumpSnapshot,
};
pub use transient::{
    FeedLegPressures, TransientDualValveFeedNetwork, TransientDualValveFeedNetworkConfig,
    TransientDualValveFeedNetworkSnapshot, TransientDualValveFeedNetworkState, ValveFeedLegConfig,
};
