//! Canonical bus topics published and consumed across the controller.
//!
//! The autopilot's modules communicate exclusively through the bus.
//! Topics are typed Rust structures implementing the [`Topic`]
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

/// Maximum number of redundant lanes represented in a per-kind
/// sensor-status publication.
pub const MAX_SENSOR_STATUS_LANES: usize = 8;

/// Sensor class represented by [`SensorStatus`].
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub enum SensorKind {
    /// IMU lane status.
    #[default]
    Imu,
    /// Barometer lane status.
    Barometer,
    /// GNSS lane status.
    Gnss,
    /// Magnetometer lane status.
    Magnetometer,
}

/// Per-lane voter status for one redundant sensor source.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct SensorLaneStatus {
    /// `SensorId::value()` for this lane.
    pub sensor_id: u64,
    /// Lane index in deterministic ingest order.
    pub lane_index: u8,
    /// `true` if `read()` returned a measurement of the expected kind.
    pub healthy: bool,
    /// `true` if this lane diverged from the voted value this tick.
    pub divergent: bool,
}

/// Per-sensor voter status emitted by voted ingest jobs.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SensorStatus {
    /// Publication timestamp.
    pub time: SimTime,
    /// Sensor class covered by this status sample.
    pub kind: SensorKind,
    /// Number of valid entries in [`SensorStatus::lanes`].
    pub lane_count: u8,
    /// Fixed-capacity lane records. Entries beyond `lane_count` are
    /// zero-filled.
    pub lanes: [SensorLaneStatus; MAX_SENSOR_STATUS_LANES],
    /// `true` if any lane diverged from the voted value this tick.
    pub any_divergent: bool,
    /// `true` if more lanes existed than fit in this fixed record.
    pub overflowed: bool,
}

impl Default for SensorStatus {
    fn default() -> Self {
        Self {
            time: SimTime::ZERO,
            kind: SensorKind::default(),
            lane_count: 0,
            lanes: [SensorLaneStatus::default(); MAX_SENSOR_STATUS_LANES],
            any_divergent: false,
            overflowed: false,
        }
    }
}

impl Topic for SensorStatus {
    const NAME: &'static str = "sensor.status";
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
#[allow(clippy::struct_excessive_bools)] // Phase-5.B.4 added per-sensor `*_updated_this_tick` flags
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
    /// Phase-5.B.4 — Cholesky-whitened GNSS innovation `ν̃ = L⁻¹ ν`
    /// where `L` is the Cholesky lower-triangular factor of the GNSS
    /// innovation covariance `S = L Lᵀ`. Under H₀ the whitened
    /// residual is `N(0, I_6)` and `‖ν̃‖² = chi2`. Populated only when
    /// `gnss_updated_this_tick` is `true`; zero otherwise.
    pub gnss_innovation_whitened: [f64; 6],
    /// `true` if a GNSS innovation was evaluated and whitened on this
    /// tick. The measurement may still have been gate-rejected before
    /// state correction.
    pub gnss_updated_this_tick: bool,
    /// Phase-5.B.4 — Cholesky-whitened baro innovation. The baro
    /// measurement is scalar (altitude-only), so the whitened residual
    /// is `ν / √S` and `(ν̃)² = chi2`. Populated only when
    /// `baro_updated_this_tick` is `true`; zero otherwise.
    pub baro_innovation_whitened: f64,
    /// `true` if a baro innovation was evaluated and whitened on this
    /// tick. The measurement may still have been gate-rejected before
    /// state correction.
    pub baro_updated_this_tick: bool,
    /// Phase-5.B.4 — Cholesky-whitened magnetometer innovation
    /// `ν̃ = L⁻¹ ν` for the 3-axis body-frame magnetic-vector
    /// innovation. Under H₀ the whitened residual is `N(0, I_3)` and
    /// `‖ν̃‖² = chi2`. Populated only when `mag_updated_this_tick` is
    /// `true`; zero otherwise.
    pub mag_innovation_whitened: [f64; 3],
    /// `true` if a magnetometer innovation was evaluated and whitened
    /// on this tick. The measurement may still have been gate-rejected
    /// before state correction.
    pub mag_updated_this_tick: bool,
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

/// Diagnostic status emitted by the autopilot.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq)]
pub struct AutopilotStatus {
    /// `true` when the differential-flatness trajectory loop is active.
    pub differential_flatness_active: bool,
    /// `true` when the flat-output attitude reference was suppressed
    /// because the desired specific force had no well-defined direction.
    pub differential_flatness_reference_suppressed: bool,
}

impl Topic for AutopilotStatus {
    const NAME: &'static str = "autopilot.status";
}

/// Maximum number of effector-specific commands published by the FC
/// mixer in one tick. Kept above the legacy four-channel surface so
/// over-actuated direct-torque demos do not silently truncate as soon
/// as future scenarios add modest redundancy.
pub const MAX_EFFECTOR_COMMANDS: usize = 16;

/// One effector-specific command after mixer phase gating.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EffectorCommand {
    /// `EffectorId::value()` for the target effectors rack entry.
    pub effector_id: u64,
    /// Scalar command in the effector's declared units.
    pub command: f64,
    /// `true` if the originating autopilot loop saturated.
    pub saturated: bool,
}

/// Effector-specific command set emitted by the mixer for kernel
/// consumption.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EffectorCommandSet {
    /// Demand timestamp.
    pub time: SimTime,
    /// Number of valid entries in [`EffectorCommandSet::commands`].
    pub count: u8,
    /// Fixed-capacity command records.
    pub commands: [EffectorCommand; MAX_EFFECTOR_COMMANDS],
    /// `true` if any originating loop saturated.
    pub saturated: bool,
}

impl Default for EffectorCommandSet {
    fn default() -> Self {
        Self {
            time: SimTime::ZERO,
            count: 0,
            commands: [EffectorCommand::default(); MAX_EFFECTOR_COMMANDS],
            saturated: false,
        }
    }
}

impl Topic for EffectorCommandSet {
    const NAME: &'static str = "actuator.effector_cmds";
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

/// One engine-specific command after mixer phase gating.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EngineCommand {
    /// `EngineId::value()` for the target engine-rack entry.
    pub engine_id: u64,
    /// Commanded throttle in `[0, 1]`.
    pub throttle_unit: f64,
    /// Commanded pitch gimbal angle (rad).
    pub gimbal_pitch_rad: f64,
    /// Commanded yaw gimbal angle (rad).
    pub gimbal_yaw_rad: f64,
    /// `true` if ignition is requested.
    pub ignite: bool,
    /// `true` if shutdown is requested.
    pub shutdown: bool,
}

/// Maximum number of engine-specific commands published by the FC
/// mixer in one tick.
pub const MAX_ENGINE_COMMANDS: usize = 8;

/// Engine-specific command set emitted by the mixer for kernel
/// consumption.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EngineCommandSet {
    /// Demand timestamp.
    pub time: SimTime,
    /// Number of valid entries in [`EngineCommandSet::commands`].
    pub count: u8,
    /// Fixed-capacity command records.
    pub commands: [EngineCommand; MAX_ENGINE_COMMANDS],
}

impl Default for EngineCommandSet {
    fn default() -> Self {
        Self {
            time: SimTime::ZERO,
            count: 0,
            commands: [EngineCommand::default(); MAX_ENGINE_COMMANDS],
        }
    }
}

impl Topic for EngineCommandSet {
    const NAME: &'static str = "actuator.engine_cmds";
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

/// Phase-5.B.4 — diagnostic slot published when the windowed
/// mean-shift GLRT detector trips. Carries the maximised test
/// statistic, the Bonferroni-corrected trip threshold, and the
/// FDIR-step index that maximised `Λ(τ)`. Multiple sensors trip in
/// the same tick produce one diagnostic per tick — the latest sensor
/// in the dispatch order (gnss, baro, mag) wins the diagnostic slot;
/// the full per-sensor mask still latches on `FdirStatus`.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct FdirGlrtDiagnostic {
    /// Sensor that tripped (one of the `FDIR_BIT_*` masks). Combined
    /// masks are not used — exactly one sensor bit is set per
    /// diagnostic.
    pub sensor_mask: u64,
    /// FDIR-step index that maximised `Λ(τ)` (`1`-based, monotonic).
    /// Translates to absolute step time via the runner's tick clock.
    pub estimated_jump_step: u64,
    /// Maximised statistic `Λ(τ̂) = ‖Σ ν̃_i‖² / N_τ`.
    pub statistic: f64,
    /// Configured Bonferroni-corrected trip threshold.
    pub threshold: f64,
}

impl Topic for FdirGlrtDiagnostic {
    const NAME: &'static str = "fdir.glrt.diagnostic";
}

/// Phase-5.B.3 — IMM mode-probability snapshot published every tick
/// by [`crate::imm::ImmEstimator`]. Carries the active mode (the
/// `argmax_j μ_j` index), the full posterior probability vector
/// (zero-padded to [`crate::imm::MAX_IMM_MODES`]), and the count of
/// valid entries.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct EstimatorMode {
    /// Estimate timestamp.
    pub time: SimTime,
    /// `argmax_j μ_j` — currently most-probable mode index
    /// (`< mode_count`).
    pub active_mode: u8,
    /// Posterior mode probabilities. Indices `[0, mode_count)` carry
    /// the live values; indices `[mode_count, MAX_IMM_MODES)` are
    /// zero-padded.
    pub mode_probabilities: [f64; 4],
    /// Number of valid entries in `mode_probabilities`. `2 ≤
    /// mode_count ≤ 4`.
    pub mode_count: u8,
}

impl Topic for EstimatorMode {
    const NAME: &'static str = "estimator.mode";
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
