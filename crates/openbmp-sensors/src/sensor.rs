//! Sensor traits, truth-bag, and measurement enum.
//!
//! The kernel-side adapter (Phase 2.10) constructs a [`SensorTruth`]
//! once per step from the kernel's state, hands it to each registered
//! synthetic sensor implementation, and routes the returned
//! [`SensorMeasurement`] into telemetry.
//!
//! Phase 2.7 ships three synthetic sensors: `IdealStateSensor`,
//! `SyntheticBarometer`, and `SyntheticImu`.

use nalgebra::{UnitQuaternion, Vector3};
#[cfg(feature = "synthetic")]
use openbmp_core::StepIndex;
use openbmp_core::{Eci, Position3, SensorId, SimTime, Velocity3};

#[cfg(feature = "synthetic")]
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
    /// Phase-3.10.C body-frame magnetic flux density (nT). The
    /// runner-side adapter rotates the geodetic-NED WMM truth field
    /// at the vehicle's position and time through the attitude into
    /// the body frame and packs the result here. Defaults to zero
    /// for legacy / non-magnetometer scenarios.
    pub magnetic_field_body_nt: Vector3<f64>,
    /// Simulation wall-clock at this measurement.
    pub time: SimTime,
}

// ---------------------------------------------------------------------
// SensorMeasurement
// ---------------------------------------------------------------------

/// A typed measurement returned by a synthetic or hardware-backed sensor.
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
    /// Phase-3.10.B GNSS receiver measurement: per-axis position
    /// and velocity in ECI with additive Gaussian noise plus an OU
    /// bias drift on the position channels. IS-GPS-200 nominal
    /// noise budget; no satellite geometry, no pseudorange.
    Gnss {
        /// Reported position in ECI (m).
        position_eci_m: Vector3<f64>,
        /// Reported velocity in ECI (m/s).
        velocity_eci_m_s: Vector3<f64>,
        /// Per-axis OU bias state in ECI (m). Useful for telemetry
        /// audit and for downstream estimators that benefit from
        /// observability into the slowly-drifting bias.
        position_bias_eci_m: Vector3<f64>,
    },
    /// Phase-3.10.C body-frame magnetometer measurement: WMM truth
    /// field rotated into the body frame, with constant soft-iron
    /// (3×3) and hard-iron (3-vector) biases plus per-axis
    /// Gaussian noise.
    Magnetometer {
        /// Reported body-frame magnetic flux density (nT).
        field_body_nt: Vector3<f64>,
        /// Constant body-frame hard-iron offset (nT) declared by
        /// the budget; reported for telemetry audit.
        hard_iron_body_nt: Vector3<f64>,
    },
    /// Phase-3.10.D star-tracker attitude measurement: the truth
    /// `attitude_eci_to_body` quaternion with a small-angle
    /// Gaussian rotation-vector perturbation applied via right-
    /// multiplication. Unit-norm by construction.
    StarTracker {
        /// Reported body-to-ECI attitude (unit quaternion).
        attitude_eci_to_body: UnitQuaternion<f64>,
    },
}

// ---------------------------------------------------------------------
// Sensor (hardware-portable) + SyntheticSensor (sim-side) traits
// ---------------------------------------------------------------------

/// Hardware-portable sensor abstraction.
///
/// Phase-3.14.C extracted this from the simulator-side sensor trait so a real
/// flight controller — and a downstream HAL adopter — can reason
/// about a sensor by its stable id and output type without depending
/// on the simulator's truth-port + per-step RNG mechanism. The
/// controller subscribes to a `dyn Sensor<Output = …>` instance and
/// the runner (sim) or HAL (hardware) drives the measurement
/// pipeline through implementation-specific channels.
pub trait Sensor {
    /// Output type produced by this sensor.
    type Output;

    /// Stable identifier for this sensor instance, derived from the
    /// canonical scenario sensor path via [`SensorId::from_path`] in
    /// sim-side use; HAL adopters typically derive it from a
    /// hardware-channel name.
    fn sensor_id(&self) -> SensorId;
}

/// Trait implemented by simulator-side synthetic sensors.
///
/// Each call to [`measure`](Self::measure) advances any per-sensor
/// random state (OU bias, RRW walk) by one step, draws fresh noise
/// from a per-component RNG sub-stream keyed by
/// `(scenario_seed, step, sensor_id, component_id)`, and produces
/// the typed [`SensorMeasurement`].
///
/// The mutable receiver (`&mut self`) is required because OU and
/// RRW carry state across calls. The `(truth, step, scenario_seed)`
/// arguments are the simulation-side context — a HAL adopter does
/// not implement this trait; they implement the supertrait
/// [`Sensor`] directly and read from real hardware in their own
/// runtime.
#[cfg(feature = "synthetic")]
pub trait SyntheticSensor: Sensor<Output = SensorMeasurement> {
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

/// Validate that every component of the supplied truth bag is finite,
/// and that the simulation timestamp is valid. Returns
/// [`SensorError::NonFinite`] / [`SensorError::InvalidParameter`] on
/// the first invalid component.
#[cfg(feature = "synthetic")]
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
        truth.magnetic_field_body_nt.x,
        truth.magnetic_field_body_nt.y,
        truth.magnetic_field_body_nt.z,
    ] {
        if !x.is_finite() {
            return Err(SensorError::NonFinite {
                reason: "sensor truth contains a non-finite component",
            });
        }
    }
    if !truth.time.is_finite() {
        return Err(SensorError::NonFinite {
            reason: "sensor truth time is NaN or infinite",
        });
    }
    if truth.time.as_seconds() < 0.0 {
        return Err(SensorError::InvalidParameter {
            reason: "sensor truth time must not be negative",
        });
    }
    Ok(())
}
