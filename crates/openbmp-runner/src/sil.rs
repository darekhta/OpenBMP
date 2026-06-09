//! Software-in-the-loop observation boundary.
//!
//! This module exposes a read-only, per-tick view of the in-loop flight
//! controller so a host SIL bench can compare what the flight software
//! *estimates* against the simulated *truth* the bridge already computes.
//! It is the observe half of a real SIL bench; it adds no behaviour to a
//! normal run (the hook is `None` by default and the observation is never
//! collected unless a monitor is installed).
//!
//! Forward-only by construction: [`SilMonitor::observe`] is handed shared
//! references and returns nothing, so it can neither steer the controller
//! nor feed back into any stimulus path — the bench observes forward
//! dynamics, it never drives them.

use openbmp_core::StepIndex;
use openbmp_fc::topics::{
    AttitudeEstimate, EstimatorMode, EstimatorStatus, FdirStatus, PositionEstimate, ReferenceState,
};
use openbmp_sensors::SensorTruth;

/// Owned, lifetime-free snapshot of the flight controller's most recent
/// published estimates and health, assembled AFTER the controller steps on
/// a given tick.
///
/// Every field is `Option` because a topic may not have been published yet,
/// and [`Self::mode`] is additionally `None` for a single-lane estimator
/// (e.g. a plain EKF) that does not run the IMM mode filter — callers must
/// treat a `None` mode as "not applicable", never as a failure.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct FcObservation {
    /// Latest attitude estimate (`q_body_to_eci`, debiased rate, gyro bias).
    pub attitude: Option<AttitudeEstimate>,
    /// Latest translational state estimate (ECI position/velocity, accel bias).
    pub position: Option<PositionEstimate>,
    /// Latest estimator-health snapshot (chi², dead-reckoning, innovations).
    pub estimator: Option<EstimatorStatus>,
    /// Latest FDIR status (triggered flag, tripped-detector mask).
    pub fdir: Option<FdirStatus>,
    /// Latest IMM mode snapshot; `None` for a single-lane estimator.
    pub mode: Option<EstimatorMode>,
    /// Latest guidance reference the autopilot is tracking.
    pub reference: Option<ReferenceState>,
}

/// A read-only per-tick observer of the in-loop flight controller.
///
/// [`Self::observe`] runs once per kernel tick, AFTER the controller has
/// stepped (so the estimate is fresh) and BEFORE the controller's commands
/// reach the racks. It receives shared references and returns nothing: by
/// construction a monitor cannot influence the controller, the kernel, or
/// any stimulus — it is an observation tap, not a feedback path.
pub trait SilMonitor {
    /// Observe one tick of the in-loop controller against truth.
    fn observe(&mut self, step: StepIndex, truth: &SensorTruth, observation: &FcObservation);
}
