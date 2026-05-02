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
//! Phase 4.C implemented a real 6-state sigma-point UKF, the
//! kernel↔FC runner bridge, WGS84-J2 gravity, WMM 2025 magnetic
//! model selection through the FC runner, and feature-gated Clarabel
//! QP / SOCP primitives. Full 15-state / square-root UKF, full
//! receding-horizon MPC, full `LCvxLD` / `SCvx` trajectory
//! reproduction, NRLMSISE-00, and multi-instance estimator routing
//! are Phase 5 / downstream scope.
//!
//! See `docs/software-architecture.md § Flight Controller` for the
//! architectural contract and `docs/phase-5-plan.md` for the
//! deferred SOTA work.

#![forbid(unsafe_code)]

pub mod autopilot;
pub mod bus;
pub mod clock;
pub mod commander;
pub mod controller;
pub mod dictionary;
pub mod error;
pub mod estimator;
pub mod fdir;
pub mod filters;
pub mod guidance;
pub mod health;
#[cfg(feature = "l1-adaptive")]
pub mod l1_adaptive_full;
#[cfg(feature = "mpc")]
pub mod landing;
pub mod mixer;
#[cfg(feature = "mpc")]
pub mod mpc;
pub mod params;
pub mod replay;
pub mod scheduler;
pub mod sensor_ingest;
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
    AutopilotError, BusError, CommanderError, ControllerError, EstimatorError, ParamError,
    SchedulerError, TableError,
};
pub use scheduler::{
    DispatchSummary, Job, JobContext, JobInfo, OverrunEvent, Priority, Scheduler, Trigger,
};
