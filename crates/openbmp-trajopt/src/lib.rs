//! Offline trajectory optimization and I-load synthesis scaffolding.
//!
//! This crate is deliberately L4 / host-side. It packages the
//! forward-only terminal-condition vocabulary from
//! `openbmp-physics`, provides residual calculations for orbital /
//! inertial terminal conditions, and emits versioned postcard I-load
//! payloads for a flight controller to validate before use. It is not
//! linked by `openbmp-fc`.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

extern crate alloc;

pub mod corrector;
pub mod iload;
pub mod target;

pub use corrector::{DifferentialCorrection, DifferentialCorrector};
pub use iload::{
    GainAxis, GainTable, ILoadHeader, ILoadPayload, ILoadSchemaVersion, ReferenceProfileSample,
    SynthesisMetadata, TrajoptError, encode_iload_payload,
};
pub use openbmp_physics::profile::TerminalCondition;
pub use target::{TerminalResidual, terminal_residual};
