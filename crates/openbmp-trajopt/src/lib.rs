//! Offline trajectory optimization and I-load synthesis scaffolding.
//!
//! This crate is deliberately L4 / host-side. It packages the
//! forward-only terminal-condition vocabulary from
//! `openbmp-physics`, provides residual calculations for orbital /
//! inertial terminal conditions, and emits versioned postcard I-load
//! payloads for a flight controller to validate before use. It is not
//! linked by `openbmp-fc`.

#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

extern crate alloc;

pub mod corrector;
pub mod driver;
pub mod iload;
pub mod shooting;
pub mod stm;
pub mod target;

pub use corrector::{DifferentialCorrection, DifferentialCorrector};
pub use driver::{TwoBodyApogeeReport, TwoBodyApogeeTargeting, correct_two_body_apogee};
pub use iload::{
    GainAxis, GainTable, ILoadHeader, ILoadPayload, ILoadSchemaVersion, ReferenceProfileSample,
    SynthesisMetadata, TrajoptError, encode_iload_payload,
};
pub use openbmp_physics::profile::TerminalCondition;
pub use shooting::{
    MultipleShootingContinuityReport, MultipleShootingNode, evaluate_two_body_multiple_shooting,
    seed_two_body_multiple_shooting_nodes,
};
pub use stm::{
    StateTransitionMatrix, TwoBodyCartesianState, TwoBodyVariationalPropagation,
    propagate_two_body_variational,
};
pub use target::{TerminalResidual, terminal_residual};
