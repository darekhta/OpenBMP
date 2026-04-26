//! `SyntheticSensor` trait, truth-bag, and measurement enum.
//!
//! The kernel-side adapter (Phase 2.10) constructs a [`SensorTruth`]
//! once per step from the kernel's state, hands it to each registered
//! [`SyntheticSensor::measure`] implementation, and routes the
//! returned [`SensorMeasurement`] into telemetry.
//!
//! Phase 2.7 ships three sensors: [`crate::ideal::IdealStateSensor`],
//! [`crate::barometer::SyntheticBarometer`], [`crate::imu::SyntheticImu`].

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::{Eci, Position3, SensorId, SimTime, StepIndex, Velocity3};

use crate::error::SensorError;

// ---------------------------------------------------------------------
// SensorTruth
// ---------------------------------------------------------------------

/// Snapshot of the simulation truth state needed by Phase-2 sensors.
///
/// All fields are in SI units in their named frame. The kernel-side
/// adapter at Phase 2.10 fills this from its `RigidBodyState` plus
/// the active env-model outputs (atmosphere sample for barometer
/// pressure / altitude). Phase-2 sensors never read the kernel state
/// directly — they only consume `SensorTruth` — so the propulsion /
/// sensors / aero crates stay L2 and never depend on `openbmp-sim`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct SensorTruth {
    /// Vehicle inertial position (m, ECI).
    pub position_eci: Position3<Eci>,
    /// Vehicle inertial velocity (m/s, ECI).
    pub velocity_eci: Velocity3<Eci>,
    /// Attitude rotating ECI into Body.
    pub attitude_eci_to_body: UnitQuaternion<f64>,
    /// Body-frame angular velocity (rad/s).
    pub angular_velocity_body_rad_s: Vector3<f64>,
    /// Body-frame **specific force** (m/s²) — i.e., total
    /// non-gravitational acceleration of the body, which is what an
    /// accelerometer actually measures.
    pub specific_force_body_m_s2: Vector3<f64>,
    /// Local atmospheric static pressure (Pa).
    pub static_pressure_pa: f64,
    /// Geometric altitude above the model reference surface (m).
    pub altitude_geometric_m: f64,
    /// Simulation wall-clock at this measurement.
    pub time: SimTime,
}

// ---------------------------------------------------------------------
// SensorMeasurement
// ---------------------------------------------------------------------

/// A typed measurement returned by a [`SyntheticSensor`].
#[derive(Clone, Debug, PartialEq)]
pub enum SensorMeasurement {
    /// Bit-equal echo of the supplied [`SensorTruth`]. Useful for
    /// analytic-toy validation and golden-output regression where
    /// the noise channels are intentionally muted.
    IdealState(SensorTruth),
    /// Barometric pressure measurement (Pa) including additive
    /// Gaussian noise and the slowly-drifting OU bias state.
    Barometer {
        /// Reported static pressure, in Pa.
        pressure_pa: f64,
        /// Current bias state (Pa) — useful for telemetry and audit.
        bias_pa: f64,
    },
    /// IMU triaxial gyro + triaxial accelerometer measurement after
    /// the IEEE 952 five-component noise model.
    Imu {
        /// Reported angular rate in body frame (rad/s).
        gyro_rad_s: Vector3<f64>,
        /// Reported specific force in body frame (m/s²).
        accel_m_s2: Vector3<f64>,
    },
}

// ---------------------------------------------------------------------
// SyntheticSensor trait
// ---------------------------------------------------------------------

/// Trait implemented by Phase-2 synthetic sensors.
///
/// Each call to [`measure`](Self::measure) advances any per-sensor
/// random state (OU bias, RRW walk) by one step, draws fresh noise
/// from a per-component RNG sub-stream keyed by
/// `(scenario_seed, step, sensor_id, component_id)`, and produces
/// the typed [`SensorMeasurement`].
///
/// The mutable receiver (`&mut self`) is required because OU and
/// RRW carry state across calls.
pub trait SyntheticSensor {
    /// Stable identifier for this sensor instance, derived from the
    /// canonical scenario sensor path via [`SensorId::from_path`].
    fn sensor_id(&self) -> SensorId;

    /// Produce one measurement at simulation step `step` from the
    /// supplied truth bag.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::NonFinite`] when an arithmetic step
    /// produces a non-finite value or when the supplied truth carries
    /// a non-finite component the sensor would propagate.
    fn measure(
        &mut self,
        truth: &SensorTruth,
        step: StepIndex,
        scenario_seed: u64,
    ) -> Result<SensorMeasurement, SensorError>;
}

// ---------------------------------------------------------------------
// Helpers used by sensor implementations
// ---------------------------------------------------------------------

/// Validate that every component of the supplied truth bag is
/// finite. Returns [`SensorError::NonFinite`] on the first
/// non-finite component.
pub(crate) fn require_truth_finite(truth: &SensorTruth) -> Result<(), SensorError> {
    let p = truth.position_eci.vector;
    let v = truth.velocity_eci.vector;
    let q = truth.attitude_eci_to_body.into_inner();
    for x in [
        p.x,
        p.y,
        p.z,
        v.x,
        v.y,
        v.z,
        q.i,
        q.j,
        q.k,
        q.w,
        truth.angular_velocity_body_rad_s.x,
        truth.angular_velocity_body_rad_s.y,
        truth.angular_velocity_body_rad_s.z,
        truth.specific_force_body_m_s2.x,
        truth.specific_force_body_m_s2.y,
        truth.specific_force_body_m_s2.z,
        truth.static_pressure_pa,
        truth.altitude_geometric_m,
    ] {
        if !x.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "sensor truth contains a non-finite component",
            });
        }
    }
    Ok(())
}
