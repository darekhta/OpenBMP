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
//! Phase 4.C deferrals (not in this crate today): real sigma-point
//! UKF, real MPC backed by a vetted convex-QP solver, real
//! `LCvxLD` / `SCvx` powered-descent guidance backed by a vetted SOCP
//! solver, full WMM 2025 spherical-harmonic field model.
//!
//! See `docs/phase-4-plan.md` (during Phase 4) and
//! `docs/software-architecture.md § Flight Controller` (after closure)
//! for the architectural contract.

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
pub mod guidance;
pub mod health;
pub mod magnetic;
pub mod mixer;
pub mod params;
pub mod replay;
pub mod scheduler;
pub mod sensor_ingest;
pub mod tables;
pub mod topics;
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
