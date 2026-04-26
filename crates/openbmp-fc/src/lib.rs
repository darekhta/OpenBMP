//! `openbmp-fc` — OpenBMP virtual flight controller.
//!
//! Simulator-local controller framework. Composed of:
//! - `Estimator` — `Ekf` (15-state error-state), `Mekf`, `Ukf`.
//! - `Autopilot` — three-loop, gain-scheduled.
//! - `MissionStateMachine` — academic phases.
//! - Academic guidance laws — attitude tracking, scripted reference
//!   state, scenario-waypoint navigation.
//! - `Fdir` — fault detection / isolation / recovery framework for
//!   simulated faults.
//!
//! **No targeting, no terminal-homing, no real-world location
//! guidance, no hardware protocols.**
//!
//! **Status:** Phase 4 stub.
