//! `openbmp-fc` — OpenBMP flight controller.
//!
//! A simulator-local autopilot composed of a lockstep clock, an
//! internal pub/sub bus, a cyclic scheduler with budget enforcement,
//! a commander, a parameter registry, a sensor voter, a health /
//! arming gate, a phase-gated actuator mixer, and the academic
//! algorithms (EKF / MEKF / three-loop autopilot / mission FSM /
//! guidance / FDIR). The architecture is hardware-portable: the
//! same binary, linked against a downstream HAL crate, may be
//! deployable in that adopter's repository — but the OpenBMP
//! repository itself ships only simulator-local validation and
//! makes no compliance claim under IEC 61508, ISO 26262, DO-178C,
//! or equivalent regimes.
//!
//! **No targeting, no terminal-homing, no real-world location
//! guidance, no hardware protocols.**
//!
//! The FC provides a sigma-point UKF (with a square-root 15-state
//! variant), the kernel↔FC runner bridge, WGS84-J2 gravity, WMM 2025
//! magnetic model selection through the FC runner, feature-gated
//! Clarabel QP / SOCP primitives, receding-horizon MPC, `LCvxLD` /
//! `SCvx` trajectory reproduction, and multi-instance estimator
//! routing.
//!
//! See `docs/software-architecture.md § Flight Controller` for the
//! architectural contract.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

#[cfg(not(feature = "std"))]
extern crate alloc;

#[cfg(not(feature = "std"))]
extern crate self as std;

#[cfg(not(feature = "std"))]
pub use alloc::format;

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod any {
    pub use core::any::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod boxed {
    pub use alloc::boxed::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod borrow {
    pub use alloc::borrow::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod cell {
    pub use core::cell::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod cmp {
    pub use core::cmp::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod collections {
    pub use alloc::collections::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod f64 {
    pub use core::f64::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod fmt {
    pub use core::fmt::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod ops {
    pub use core::ops::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod string {
    pub use alloc::string::*;
}

#[cfg(not(feature = "std"))]
#[doc(hidden)]
pub mod vec {
    pub use alloc::vec::*;
}

pub mod allocation;
pub mod anti_windup;
pub mod autopilot;
pub mod bus;
pub mod clock;
pub mod commander;
pub mod controller;
pub mod dictionary;
pub mod error;
pub mod estimator;
pub mod estimator_lanes;
pub mod fdir;
pub mod filters;
pub mod glrt;
pub mod guidance;
pub mod health;
pub mod imm;
#[cfg(feature = "indi")]
pub mod indi;
#[cfg(feature = "l1-adaptive")]
pub mod l1_adaptive_full;
#[cfg(feature = "mpc")]
pub mod landing;
#[cfg(feature = "lqr")]
pub mod lqr;
pub mod mixer;
#[cfg(feature = "mpc")]
pub mod mpc;
mod nav_metrics;
pub mod params;
pub mod replay;
pub mod scheduler;
pub mod sensor_ingest;
pub mod sr_ukf;
mod stable_map;
pub mod tables;
pub mod topics;
pub mod trajectory;
#[cfg(feature = "square-root-ekf")]
pub mod ud;
pub mod voter;

pub use bus::{Bus, Sequence, Topic};
pub use clock::{Clock, FixedClock, SimulatedClock};
pub use controller::{FlightController, FlightControllerBuilder};
pub use dictionary::{Dictionary, DictionaryError};
pub use error::{
    AutopilotError, BusError, CommanderError, ControllerError, EstimatorError, GuidanceError,
    ParamError, SchedulerError, TableError,
};
pub use scheduler::{
    DispatchSummary, Job, JobContext, JobInfo, OverrunEvent, Priority, Scheduler, Trigger,
};
