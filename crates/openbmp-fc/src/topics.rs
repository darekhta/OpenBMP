//! Canonical bus topics published and consumed across the controller.
//!
//! The autopilot's modules communicate exclusively through the bus.
//! Topics are typed Rust structures implementing the [`Topic`](crate::bus::Topic)
//! trait. This module is the single registry of every topic the
//! controller binary speaks; the dictionary generator walks this
//! module to produce the build-time JSON dictionary.
//!
//! Topic naming convention: `<subsystem>.<topic>`. Subsystems include
//! `sensor`, `estimator`, `commander`, `autopilot`, `actuator`,
//! `health`, `fdir`, `mission`, `scheduler`, `parameters`. The
//! dictionary generator validates that every registered topic uses
//! the convention.

use nalgebra::Vector3;
use openbmp_core::SimTime;

use crate::bus::Topic;

// ---------------------------------------------------------------------
// Sensor sample topics (Phase 4.2)
// ---------------------------------------------------------------------

/// Inertial-measurement-unit sample at the sensor's native cadence.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ImuSample {
    /// Capture timestamp.
    pub time: SimTime,
    /// Body-frame angular velocity (rad/s).
    pub gyro_rad_s: Vector3<f64>,
    /// Body-frame specific force (m/s²).
    pub accel_m_s2: Vector3<f64>,
    /// `true` if a synthetic-side fault disabled this sample (the
    /// voter then masks it).
    pub healthy: bool,
}

impl Topic for ImuSample {
    const NAME: &'static str = "sensor.imu";
}

/// Barometric-altimeter sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BarometerSample {
    /// Capture timestamp.
    pub time: SimTime,
    /// Static pressure (Pa).
    pub pressure_pa: f64,
    /// Bias estimate (Pa) from the synthetic / HAL backend.
    pub bias_pa: f64,
    /// `true` if the sample is valid this cycle.
    pub healthy: bool,
}

impl Topic for BarometerSample {
    const NAME: &'static str = "sensor.barometer";
}

/// GNSS receiver sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct GnssSample {
    /// Capture timestamp.
    pub time: SimTime,
    /// ECI position (m).
    pub position_eci_m: Vector3<f64>,
    /// ECI velocity (m/s).
    pub velocity_eci_m_s: Vector3<f64>,
    /// Bias estimate (m) from the synthetic / HAL backend.
    pub position_bias_eci_m: Vector3<f64>,
    /// `true` if the sample is a valid fix.
    pub healthy: bool,
}

impl Topic for GnssSample {
    const NAME: &'static str = "sensor.gnss";
}

/// Magnetometer sample.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct MagnetometerSample {
    /// Capture timestamp.
    pub time: SimTime,
    /// Body-frame magnetic field (nT).
    pub field_body_nt: Vector3<f64>,
    /// Hard-iron bias estimate (nT) from the synthetic / HAL backend.
    pub hard_iron_body_nt: Vector3<f64>,
    /// `true` if the sample is valid this cycle.
    pub healthy: bool,
}

impl Topic for MagnetometerSample {
    const NAME: &'static str = "sensor.magnetometer";
}

/// Star-tracker sample (attitude only).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct StarTrackerSample {
    /// Capture timestamp.
    pub time: SimTime,
    /// Quaternion (x, y, z, w) rotating ECI vectors into the body
    /// frame.
    pub q_eci_to_body_xyzw: [f64; 4],
    /// `true` if a fix was acquired this cycle.
    pub healthy: bool,
}

impl Topic for StarTrackerSample {
    const NAME: &'static str = "sensor.star_tracker";
}

// ---------------------------------------------------------------------
// Estimator output topics (Phase 4.3 / 4.8)
// ---------------------------------------------------------------------

/// Attitude estimate published by an [`Estimator`](crate::estimator::Estimator).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct AttitudeEstimate {
    /// Estimate timestamp.
    pub time: SimTime,
    /// Quaternion (x, y, z, w) rotating body-frame vectors into ECI.
    pub q_body_to_eci_xyzw: [f64; 4],
    /// Body-frame angular velocity (rad/s) — debiased.
    pub omega_body_rad_s: Vector3<f64>,
    /// Estimated gyro bias (rad/s) in the body frame.
    pub gyro_bias_body_rad_s: Vector3<f64>,
}

impl Topic for AttitudeEstimate {
    const NAME: &'static str = "estimator.attitude";
}

/// Translational state estimate published by an
/// [`Estimator`](crate::estimator::Estimator).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PositionEstimate {
    /// Estimate timestamp.
    pub time: SimTime,
    /// ECI position (m).
    pub position_eci_m: Vector3<f64>,
    /// ECI velocity (m/s).
    pub velocity_eci_m_s: Vector3<f64>,
    /// Estimated accelerometer bias (m/s²) in the body frame.
    pub accel_bias_body_m_s2: Vector3<f64>,
}

impl Topic for PositionEstimate {
    const NAME: &'static str = "estimator.position";
}

/// Diagnostic snapshot of estimator health.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EstimatorStatus {
    /// Estimate timestamp.
    pub time: SimTime,
    /// `true` while propagation has been valid for at least one full
    /// `predict` step.
    pub initialized: bool,
    /// `true` if the filter has not received a corrective measurement
    /// for longer than its configured dead-reckoning timeout.
    pub dead_reckoning: bool,
    /// Innovation chi-square ratio for the most recent IMU update.
    pub imu_chi2: f64,
    /// Innovation chi-square ratio for the most recent GNSS update.
    pub gnss_chi2: f64,
    /// Innovation chi-square ratio for the most recent baro update.
    pub baro_chi2: f64,
    /// Innovation chi-square ratio for the most recent magnetometer update.
    pub mag_chi2: f64,
    /// Innovation chi-square ratio for the most recent star tracker update.
    pub star_tracker_chi2: f64,
    /// `true` if any innovation in the most recent update exceeded
    /// its configured chi-square gate.
    pub innovation_rejected: bool,
}

impl Topic for EstimatorStatus {
    const NAME: &'static str = "estimator.status";
}

// ---------------------------------------------------------------------
// Commander / mission topics (Phase 4.4)
// ---------------------------------------------------------------------

/// Vehicle status published by the
/// [`Commander`](crate::commander::Commander).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct VehicleStatus {
    /// Active mission phase (`PhaseId::value()`).
    pub phase_id: u64,
    /// `true` while the commander has armed the vehicle. Actuator
    /// authority is gated on this in addition to the per-phase mask.
    pub armed: bool,
    /// `true` while the binary believes the vehicle is in flight
    /// (post-liftoff, pre-landed).
    pub in_flight: bool,
    /// `true` once an FDIR trip has demanded a safe-state transition.
    /// Latches; cleared only when the scenario explicitly transitions
    /// the commander into a safe-state phase via an `EventBinding`
    /// (Phase 4.C).
    pub safe_state_requested: bool,
}

impl Topic for VehicleStatus {
    const NAME: &'static str = "commander.vehicle_status";
}

/// Aggregate failsafe-flag bitfield published by the
/// [`HealthMonitor`](crate::health::HealthMonitor).
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
#[allow(clippy::struct_excessive_bools)]
pub struct FailsafeFlags {
    /// `true` if the IMU is unhealthy or stale.
    pub imu_unhealthy: bool,
    /// `true` if the barometer is unhealthy or stale.
    pub baro_unhealthy: bool,
    /// `true` if GNSS is unhealthy or stale.
    pub gnss_unhealthy: bool,
    /// `true` if the magnetometer is unhealthy or stale.
    pub mag_unhealthy: bool,
    /// `true` if the estimator has flagged dead-reckoning.
    pub estimator_dead_reckoning: bool,
    /// `true` if the scheduler emitted a sustained overrun.
    pub scheduler_overrun: bool,
    /// `true` if any registered FDIR detector tripped.
    pub fdir_triggered: bool,
}

impl FailsafeFlags {
    /// Returns `true` if any failsafe condition is asserted.
    #[must_use]
    pub fn any(self) -> bool {
        self.imu_unhealthy
            || self.baro_unhealthy
            || self.gnss_unhealthy
            || self.mag_unhealthy
            || self.estimator_dead_reckoning
            || self.scheduler_overrun
            || self.fdir_triggered
    }
}

impl Topic for FailsafeFlags {
    const NAME: &'static str = "health.failsafe_flags";
}

// ---------------------------------------------------------------------
// Autopilot output topics (Phase 4.5)
// ---------------------------------------------------------------------

/// Demanded actuator deflection emitted by the autopilot, before the
/// mixer applies phase-gated authority.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ActuatorCommand {
    /// Demand timestamp.
    pub time: SimTime,
    /// Commanded elevator-equivalent deflection (rad).
    pub elevator_rad: f64,
    /// Commanded aileron-equivalent deflection (rad).
    pub aileron_rad: f64,
    /// Commanded rudder-equivalent deflection (rad).
    pub rudder_rad: f64,
    /// Commanded body-flap deflection (rad).
    pub body_flap_rad: f64,
    /// `true` if any commanded surface saturated this cycle.
    pub saturated: bool,
}

impl Topic for ActuatorCommand {
    const NAME: &'static str = "autopilot.actuator_cmd";
}

/// Demanded engine throttle / gimbal command emitted by the autopilot,
/// before the mixer applies phase-gated authority.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EngineDemand {
    /// Demand timestamp.
    pub time: SimTime,
    /// Commanded throttle in `[0, 1]`.
    pub throttle_unit: f64,
    /// Commanded pitch gimbal angle (rad).
    pub gimbal_pitch_rad: f64,
    /// Commanded yaw gimbal angle (rad).
    pub gimbal_yaw_rad: f64,
    /// `true` if the autopilot is requesting ignition.
    pub ignite: bool,
    /// `true` if the autopilot is requesting shutdown.
    pub shutdown: bool,
}

impl Topic for EngineDemand {
    const NAME: &'static str = "autopilot.engine_cmd";
}

// ---------------------------------------------------------------------
// FDIR / health topics (Phase 4.6)
// ---------------------------------------------------------------------

/// FDIR module's published status.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct FdirStatus {
    /// `true` if any detector is currently flagging a fault.
    pub triggered: bool,
    /// Bitmask of detector indices that have tripped (capacity 64).
    pub tripped_mask: u64,
    /// Time-since-trip in monotonic ticks of the most recent trip.
    pub ticks_since_trip: u64,
}

impl Topic for FdirStatus {
    const NAME: &'static str = "fdir.status";
}

// ---------------------------------------------------------------------
// Reference / guidance topics (Phase 4.7)
// ---------------------------------------------------------------------

/// Reference state the autopilot tracks. Populated by the guidance
/// module and consumed by the autopilot.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct ReferenceState {
    /// Reference timestamp.
    pub time: SimTime,
    /// Reference quaternion (x, y, z, w) rotating body to ECI.
    pub q_body_to_eci_xyzw: [f64; 4],
    /// Reference body-frame angular velocity (rad/s).
    pub omega_body_rad_s: Vector3<f64>,
    /// Reference ECI position (m).
    pub position_eci_m: Vector3<f64>,
    /// Reference ECI velocity (m/s).
    pub velocity_eci_m_s: Vector3<f64>,
}

impl Topic for ReferenceState {
    const NAME: &'static str = "guidance.reference";
}
