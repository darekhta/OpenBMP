//! Sensor traits, synthetic truth bag, and measurement enum.
//!
//! Simulator-side code constructs a [`SensorTruth`] once per tick from
//! the simulated vehicle state, hands it to each registered synthetic
//! sensor implementation, and routes the returned [`SensorMeasurement`]
//! into telemetry. HAL-backed code implements [`Sensor`] directly and
//! does not need the synthetic truth port.
//!
//! Provides three synthetic sensors: `IdealStateSensor`,
//! `SyntheticBarometer`, and `SyntheticImu`.

use nalgebra::{UnitQuaternion, Vector3};
#[cfg(feature = "synthetic")]
use openbmp_core::StepIndex;
use openbmp_core::{Eci, Position3, SensorId, SimTime, Velocity3};

use crate::error::SensorError;

// ---------------------------------------------------------------------
// SensorTruth
// ---------------------------------------------------------------------

/// Snapshot of simulator truth state needed by synthetic sensors.
///
/// All fields are in SI units in their named frame. The simulator
/// fills this from its vehicle-state snapshot plus active environment
/// outputs (atmosphere sample for barometer pressure / altitude).
/// Synthetic sensors never read simulator state directly — they only
/// consume `SensorTruth` — so the sensors crate does not depend on
/// `openbmp-sim`.
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
    /// Body-frame magnetic flux density (nT). The
    /// runner-side adapter rotates the geodetic-NED WMM truth field
    /// at the vehicle's position and time through the attitude into
    /// the body frame and packs the result here. Defaults to zero
    /// for legacy / non-magnetometer scenarios.
    pub magnetic_field_body_nt: Vector3<f64>,
    /// Truth timestamp on the caller-owned monotonic timeline.
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
    /// GNSS receiver measurement: per-axis position
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
    /// Body-frame magnetometer measurement: WMM truth
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
    /// Star-tracker attitude measurement: the truth
    /// `attitude_eci_to_body` quaternion with a small-angle
    /// Gaussian rotation-vector perturbation applied via right-
    /// multiplication. Unit-norm by construction.
    StarTracker {
        /// Reported body-to-ECI attitude (unit quaternion).
        attitude_eci_to_body: UnitQuaternion<f64>,
    },
}

// ---------------------------------------------------------------------
// Timestamped<T> — measurement + capture-time wrapper
// ---------------------------------------------------------------------

/// A measurement value paired with the time it was captured.
///
/// This lets the controller-side
/// [`Sensor::read`] surface carry a measurement timestamp without
/// committing the controller to a particular time source. Sim-side
/// the `time` is elapsed scenario time; HAL adopters typically fill
/// it from a hardware monotonic counter normalised to controller
/// start.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Timestamped<T> {
    /// Sample-capture time.
    pub time: SimTime,
    /// Measurement value.
    pub value: T,
}

impl<T> Timestamped<T> {
    /// Construct from `(time, value)`.
    #[must_use]
    pub const fn new(time: SimTime, value: T) -> Self {
        Self { time, value }
    }
}

// ---------------------------------------------------------------------
// Sensor (hardware-portable) + SyntheticSensor (sim-side) traits
// ---------------------------------------------------------------------

/// Hardware-portable sensor abstraction.
///
/// This is separate from the simulator-side sensor trait so
/// a real flight controller — and a downstream HAL adopter — can
/// reason about a sensor by its stable id and output type without
/// depending on the simulator's truth-port + per-tick RNG mechanism.
///
/// The [`read`](Self::read) acquisition method drives acquisition:
/// the controller polls each sensor every controller tick, gets a
/// [`Timestamped<Self::Output>`] back, and updates its estimator.
/// Sim-side, [`SyntheticSensorAdapter`] wraps a [`SyntheticSensor`]
/// with a runner-pushed truth port + tick / seed pair, and forwards
/// `read()` calls to the synthetic `measure()`. HAL adopters
/// implement `read()` directly by reading from real hardware in their
/// own runtime.
///
/// The `&mut self` receiver allows stateful HAL impls (DMA buffer
/// rotation, last-good-frame caching, sample-and-hold).
pub trait Sensor {
    /// Output type produced by this sensor.
    type Output;

    /// Stable identifier for this sensor instance, derived from the
    /// canonical scenario sensor path via [`SensorId::from_path`] in
    /// sim-side use; HAL adopters typically derive it from a
    /// hardware-channel name.
    fn sensor_id(&self) -> SensorId;

    /// Pull one measurement from this sensor at its native cadence.
    ///
    /// Implementations are expected to be non-blocking and to return
    /// the latest available sample. A controller calls `read()` once
    /// per controller tick; if the sensor has no new sample available
    /// (rate mismatch, hardware missed-frame) the implementation
    /// returns either the most recently captured value (sample-and-
    /// hold) or [`SensorError::NoSample`].
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::NoSample`] when no measurement is
    /// available, [`SensorError::NonFinite`] for non-finite arithmetic
    /// in the synthetic path, or implementation-specific
    /// [`SensorError::InvalidParameter`] when the underlying hardware
    /// or noise budget is misconfigured.
    fn read(&mut self) -> Result<Timestamped<Self::Output>, SensorError>;
}

/// Trait implemented by simulator-side synthetic sensors.
///
/// Each call to [`measure`](Self::measure) advances any per-sensor
/// random state (OU bias, RRW walk) by one tick, draws fresh noise
/// from a per-component RNG sub-stream keyed by
/// `(scenario_seed, tick, sensor_id, component_id)`, and produces
/// the typed [`SensorMeasurement`].
///
/// The mutable receiver (`&mut self`) is required because OU and
/// RRW carry state across calls. The `(truth, step, scenario_seed)`
/// arguments are the simulation-side context — a HAL adopter does
/// not implement this trait; they implement [`Sensor`] directly and
/// read from real hardware in their own runtime. A controller that
/// needs to consume `dyn Sensor<Output = SensorMeasurement>` over
/// either real-hardware or synthetic sensors uses
/// [`SyntheticSensorAdapter`] to bridge the two surfaces.
#[cfg(feature = "synthetic")]
pub trait SyntheticSensor {
    /// Stable identifier for this synthetic sensor, derived from
    /// the canonical scenario sensor path via
    /// [`SensorId::from_path`].
    fn sensor_id(&self) -> SensorId;

    /// Produce one measurement at simulator tick `step` from the
    /// supplied truth bag.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::NonFinite`] when an arithmetic tick
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
// SyntheticSensorAdapter — bridges `SyntheticSensor` to `Sensor`
// ---------------------------------------------------------------------

/// Runner-side bridge that exposes a [`SyntheticSensor`] through the
/// hardware-portable [`Sensor`] surface.
///
/// This exists so a controller written against
/// `dyn Sensor<Output = SensorMeasurement>` can be instantiated
/// against either real-hardware impls (which implement [`Sensor`]
/// directly) or simulator-side synthetic impls (wrapped here).
///
/// The simulator runner calls [`prime`](Self::prime) once per base tick
/// with the per-tick truth + tick + seed; the next [`Sensor::read`]
/// call consumes that primed state, calls
/// [`SyntheticSensor::measure`], and returns the [`Timestamped`]
/// measurement. Reading without a primed truth port returns
/// [`SensorError::NoSample`] — this matches the HAL contract where a
/// real sensor with no available sample returns the same error.
#[cfg(feature = "synthetic")]
#[derive(Debug)]
pub struct SyntheticSensorAdapter<S> {
    inner: S,
    pending: Option<PendingSyntheticInput>,
}

#[cfg(feature = "synthetic")]
#[derive(Copy, Clone, Debug)]
struct PendingSyntheticInput {
    truth: SensorTruth,
    step: StepIndex,
    scenario_seed: u64,
}

#[cfg(feature = "synthetic")]
impl<S> SyntheticSensorAdapter<S> {
    /// Wrap a [`SyntheticSensor`] for runner consumption.
    #[must_use]
    pub const fn new(inner: S) -> Self {
        Self {
            inner,
            pending: None,
        }
    }

    /// Push the per-tick truth + tick + seed pair the next
    /// [`Sensor::read`] call will consume. Idempotent — calling
    /// `prime` twice without an intervening `read` discards the
    /// earlier input (mirrors the typical "latest-wins" semantics of
    /// a HAL sample-and-hold register).
    pub fn prime(&mut self, truth: SensorTruth, step: StepIndex, scenario_seed: u64) {
        self.pending = Some(PendingSyntheticInput {
            truth,
            step,
            scenario_seed,
        });
    }

    /// Borrow the wrapped [`SyntheticSensor`] (for telemetry / audit /
    /// determinism oracle access).
    #[must_use]
    pub const fn inner(&self) -> &S {
        &self.inner
    }

    /// Mutably borrow the wrapped [`SyntheticSensor`]. Useful for
    /// scenario-injected fault application that the synthetic impl
    /// exposes via its own surface.
    pub const fn inner_mut(&mut self) -> &mut S {
        &mut self.inner
    }

    /// Consume the adapter and return the wrapped [`SyntheticSensor`].
    #[must_use]
    pub fn into_inner(self) -> S {
        self.inner
    }
}

#[cfg(feature = "synthetic")]
impl<S: SyntheticSensor> Sensor for SyntheticSensorAdapter<S> {
    type Output = SensorMeasurement;

    fn sensor_id(&self) -> SensorId {
        self.inner.sensor_id()
    }

    fn read(&mut self) -> Result<Timestamped<Self::Output>, SensorError> {
        let pending = self.pending.take().ok_or(SensorError::NoSample)?;
        let measurement =
            self.inner
                .measure(&pending.truth, pending.step, pending.scenario_seed)?;
        Ok(Timestamped::new(pending.truth.time, measurement))
    }
}

// ---------------------------------------------------------------------
// Helpers used by sensor implementations
// ---------------------------------------------------------------------

/// Validate that every component of the supplied truth bag is finite,
/// and that the truth timestamp is valid. Returns
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

#[cfg(all(test, feature = "synthetic"))]
#[allow(clippy::unwrap_used)]
mod tests {
    use nalgebra::{UnitQuaternion, Vector3};
    use openbmp_core::{Position3, SensorId, SimTime, StepIndex, Velocity3};

    use super::{
        Sensor, SensorError, SensorMeasurement, SensorTruth, SyntheticSensor,
        SyntheticSensorAdapter,
    };

    #[derive(Debug)]
    struct EchoSyntheticSensor {
        id: SensorId,
        expected_step: StepIndex,
        expected_seed: u64,
    }

    impl SyntheticSensor for EchoSyntheticSensor {
        fn sensor_id(&self) -> SensorId {
            self.id
        }

        fn measure(
            &mut self,
            truth: &SensorTruth,
            step: StepIndex,
            scenario_seed: u64,
        ) -> Result<SensorMeasurement, SensorError> {
            assert_eq!(step, self.expected_step);
            assert_eq!(scenario_seed, self.expected_seed);
            Ok(SensorMeasurement::IdealState(*truth))
        }
    }

    fn echo_sensor(expected_step: StepIndex, expected_seed: u64) -> EchoSyntheticSensor {
        EchoSyntheticSensor {
            id: SensorId::from_path("sensors.echo"),
            expected_step,
            expected_seed,
        }
    }

    fn truth_at(time_s: f64) -> SensorTruth {
        SensorTruth {
            position_eci: Position3::new(0.0, 0.0, 0.0),
            velocity_eci: Velocity3::new(0.0, 0.0, 0.0),
            attitude_eci_to_body: UnitQuaternion::identity(),
            angular_velocity_body_rad_s: Vector3::zeros(),
            specific_force_body_m_s2: Vector3::zeros(),
            static_pressure_pa: 101_325.0,
            altitude_geometric_m: 0.0,
            magnetic_field_body_nt: Vector3::zeros(),
            time: SimTime::from_seconds(time_s),
        }
    }

    #[test]
    fn synthetic_adapter_read_without_prime_returns_no_sample() {
        let mut adapter = SyntheticSensorAdapter::new(echo_sensor(StepIndex::new(0), 0xCAFE_BABE));

        assert_eq!(adapter.read(), Err(SensorError::NoSample));
    }

    #[test]
    fn synthetic_adapter_forwards_sensor_id() {
        let adapter = SyntheticSensorAdapter::new(echo_sensor(StepIndex::new(0), 0xCAFE_BABE));

        assert_eq!(adapter.sensor_id(), SensorId::from_path("sensors.echo"));
    }

    #[test]
    fn synthetic_adapter_read_consumes_primed_truth_with_timestamp() {
        let step = StepIndex::new(17);
        let seed = 0xABCD_EF01;
        let truth = truth_at(12.5);
        let mut adapter = SyntheticSensorAdapter::new(echo_sensor(step, seed));

        adapter.prime(truth, step, seed);
        let sample = adapter.read().unwrap();

        assert_eq!(sample.time, truth.time);
        assert_eq!(sample.value, SensorMeasurement::IdealState(truth));
        assert_eq!(adapter.read(), Err(SensorError::NoSample));
    }

    #[test]
    fn synthetic_adapter_prime_is_latest_wins() {
        let step = StepIndex::new(2);
        let seed = 0x1234_5678;
        let first = truth_at(1.0);
        let second = truth_at(2.0);
        let mut adapter = SyntheticSensorAdapter::new(echo_sensor(step, seed));

        adapter.prime(first, StepIndex::new(1), seed);
        adapter.prime(second, step, seed);
        let sample = adapter.read().unwrap();

        assert_eq!(sample.time, second.time);
        assert_eq!(sample.value, SensorMeasurement::IdealState(second));
    }
}
