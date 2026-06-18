//! Sensor ingest jobs.
//!
//! Each per-sensor-kind ingest job pulls measurements from its
//! `Sensor::read()` source, votes redundant lanes via the
//! [`crate::voter`] module, and publishes the voted result on the
//! corresponding sensor topic. The simulator's runner constructs
//! these jobs around `SyntheticSensorAdapter` instances; a downstream
//! HAL adopter constructs them around real driver impls.
//!
//! Ingest jobs for the IMU, barometer, air-data, GNSS,
//! magnetometer, and star tracker. Each job is a [`crate::Job`]
//! ready to register with the scheduler.

use std::fmt;
use std::vec::Vec;

use nalgebra::Vector3;
use openbmp_core::SimTime;
use openbmp_sensors::{Sensor, SensorMeasurement, Timestamped};

use crate::error::ControllerError;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    AirDataSample, BarometerSample, GnssSample, ImuSample, MAX_SENSOR_STATUS_LANES,
    MagnetometerSample, SensorKind, SensorStatus, StarTrackerSample,
};
use crate::voter::Voter;

macro_rules! ingest_debug_impl {
    ($ty:ident) => {
        impl<S> fmt::Debug for $ty<S>
        where
            S: Sensor<Output = SensorMeasurement>,
        {
            fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
                f.debug_struct(stringify!($ty))
                    .field("name", &self.name)
                    .finish()
            }
        }
    };
}

ingest_debug_impl!(ImuIngest);
ingest_debug_impl!(BarometerIngest);
ingest_debug_impl!(AirDataIngest);
ingest_debug_impl!(GnssIngest);
ingest_debug_impl!(MagnetometerIngest);
ingest_debug_impl!(StarTrackerIngest);

fn unhealthy_imu_sample(time: SimTime) -> ImuSample {
    ImuSample {
        time,
        gyro_rad_s: Vector3::zeros(),
        accel_m_s2: Vector3::zeros(),
        healthy: false,
    }
}

fn unhealthy_barometer_sample(time: SimTime) -> BarometerSample {
    BarometerSample {
        time,
        pressure_pa: 0.0,
        bias_pa: 0.0,
        healthy: false,
    }
}

fn unhealthy_airdata_sample(time: SimTime) -> AirDataSample {
    AirDataSample {
        time,
        static_pressure_pa: 0.0,
        impact_pressure_pa: 0.0,
        mach: 0.0,
        calibrated_airspeed_m_s: 0.0,
        true_airspeed_m_s: 0.0,
        angle_of_attack_rad: 0.0,
        sideslip_rad: 0.0,
        pressure_altitude_m: 0.0,
        healthy: false,
    }
}

fn unhealthy_gnss_sample(time: SimTime) -> GnssSample {
    GnssSample {
        time,
        position_eci_m: Vector3::zeros(),
        velocity_eci_m_s: Vector3::zeros(),
        position_bias_eci_m: Vector3::zeros(),
        healthy: false,
    }
}

fn unhealthy_magnetometer_sample(time: SimTime) -> MagnetometerSample {
    MagnetometerSample {
        time,
        field_body_nt: Vector3::zeros(),
        hard_iron_body_nt: Vector3::zeros(),
        healthy: false,
    }
}

fn unhealthy_star_tracker_sample(time: SimTime) -> StarTrackerSample {
    StarTrackerSample {
        time,
        q_eci_to_body_xyzw: [0.0, 0.0, 0.0, 1.0],
        healthy: false,
    }
}

/// Ingest job for an IMU `Sensor` source. Reads via
/// [`Sensor::read`] and publishes [`ImuSample`] on the bus.
pub struct ImuIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    sensor: S,
    name: &'static str,
}

impl<S> ImuIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    /// Constructs the ingest job around the given sensor.
    pub fn new(sensor: S) -> Self {
        Self {
            sensor,
            name: "sensor_ingest.imu",
        }
    }
}

impl<S> Job for ImuIngest<S>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }

    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Timestamped {
            time,
            value:
                SensorMeasurement::Imu {
                    gyro_rad_s,
                    accel_m_s2,
                },
        }) = self.sensor.read()
        {
            let _ = ctx.bus.publish(ImuSample {
                time,
                gyro_rad_s,
                accel_m_s2,
                healthy: true,
            })?;
        } else {
            ctx.bus.publish(unhealthy_imu_sample(ctx.clock.now()))?;
        }
        Ok(())
    }
}

/// Ingest job for a barometer `Sensor` source.
pub struct BarometerIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    sensor: S,
    name: &'static str,
}

impl<S> BarometerIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    /// Constructs the ingest job around the given sensor.
    pub fn new(sensor: S) -> Self {
        Self {
            sensor,
            name: "sensor_ingest.barometer",
        }
    }
}

impl<S> Job for BarometerIngest<S>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Timestamped {
            time,
            value:
                SensorMeasurement::Barometer {
                    pressure_pa,
                    bias_pa,
                },
        }) = self.sensor.read()
        {
            let _ = ctx.bus.publish(BarometerSample {
                time,
                pressure_pa,
                bias_pa,
                healthy: true,
            })?;
        } else {
            ctx.bus
                .publish(unhealthy_barometer_sample(ctx.clock.now()))?;
        }
        Ok(())
    }
}

/// Ingest job for an air-data `Sensor` source.
pub struct AirDataIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    sensor: S,
    name: &'static str,
}

impl<S> AirDataIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    /// Constructs the ingest job around the given sensor.
    pub fn new(sensor: S) -> Self {
        Self {
            sensor,
            name: "sensor_ingest.airdata",
        }
    }
}

impl<S> Job for AirDataIngest<S>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Timestamped {
            time,
            value:
                SensorMeasurement::AirData {
                    static_pressure_pa,
                    impact_pressure_pa,
                    mach,
                    calibrated_airspeed_m_s,
                    true_airspeed_m_s,
                    angle_of_attack_rad,
                    sideslip_rad,
                    pressure_altitude_m,
                },
        }) = self.sensor.read()
        {
            let _ = ctx.bus.publish(AirDataSample {
                time,
                static_pressure_pa,
                impact_pressure_pa,
                mach,
                calibrated_airspeed_m_s,
                true_airspeed_m_s,
                angle_of_attack_rad,
                sideslip_rad,
                pressure_altitude_m,
                healthy: true,
            })?;
        } else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
        }
        Ok(())
    }
}

/// Ingest job for a GNSS `Sensor` source.
pub struct GnssIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    sensor: S,
    name: &'static str,
}

impl<S> GnssIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    /// Constructs the ingest job around the given sensor.
    pub fn new(sensor: S) -> Self {
        Self {
            sensor,
            name: "sensor_ingest.gnss",
        }
    }
}

impl<S> Job for GnssIngest<S>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Timestamped {
            time,
            value:
                SensorMeasurement::Gnss {
                    position_eci_m,
                    velocity_eci_m_s,
                    position_bias_eci_m,
                },
        }) = self.sensor.read()
        {
            let _ = ctx.bus.publish(GnssSample {
                time,
                position_eci_m,
                velocity_eci_m_s,
                position_bias_eci_m,
                healthy: true,
            })?;
        } else {
            ctx.bus.publish(unhealthy_gnss_sample(ctx.clock.now()))?;
        }
        Ok(())
    }
}

/// Ingest job for a magnetometer `Sensor` source.
pub struct MagnetometerIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    sensor: S,
    name: &'static str,
}

impl<S> MagnetometerIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    /// Constructs the ingest job around the given sensor.
    pub fn new(sensor: S) -> Self {
        Self {
            sensor,
            name: "sensor_ingest.magnetometer",
        }
    }
}

impl<S> Job for MagnetometerIngest<S>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Timestamped {
            time,
            value:
                SensorMeasurement::Magnetometer {
                    field_body_nt,
                    hard_iron_body_nt,
                },
        }) = self.sensor.read()
        {
            let _ = ctx.bus.publish(MagnetometerSample {
                time,
                field_body_nt,
                hard_iron_body_nt,
                healthy: true,
            })?;
        } else {
            ctx.bus
                .publish(unhealthy_magnetometer_sample(ctx.clock.now()))?;
        }
        Ok(())
    }
}

/// Ingest job for a star-tracker `Sensor` source.
pub struct StarTrackerIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    sensor: S,
    name: &'static str,
}

impl<S> StarTrackerIngest<S>
where
    S: Sensor<Output = SensorMeasurement>,
{
    /// Constructs the ingest job around the given sensor.
    pub fn new(sensor: S) -> Self {
        Self {
            sensor,
            name: "sensor_ingest.star_tracker",
        }
    }
}

impl<S> Job for StarTrackerIngest<S>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        if let Ok(Timestamped {
            time,
            value:
                SensorMeasurement::StarTracker {
                    attitude_eci_to_body,
                },
        }) = self.sensor.read()
        {
            let q = attitude_eci_to_body.into_inner();
            let _ = ctx.bus.publish(StarTrackerSample {
                time,
                q_eci_to_body_xyzw: [q.i, q.j, q.k, q.w],
                healthy: true,
            })?;
        } else {
            ctx.bus
                .publish(unhealthy_star_tracker_sample(ctx.clock.now()))?;
        }
        Ok(())
    }
}

// ---------------------------------------------------------------------
// Voted ingest jobs
// ---------------------------------------------------------------------
//
// Voted ingest jobs own `Vec<S>` of redundant sensor lanes plus a
// `V: Voter<f64>`. On each tick, every lane is read; the per-axis
// scalar voter is run independently across the lanes; the voted value
// is published, with the `healthy` field set to `false` when *any*
// axis flags divergence so the consumer can fall through to GNSS-only
// operation.

fn vote_vec3<V: Voter<f64>>(voter: &V, samples: &[Vector3<f64>]) -> Option<(Vector3<f64>, bool)> {
    if samples.is_empty() {
        return None;
    }
    let count = samples.len().min(MAX_SENSOR_STATUS_LANES);
    let mut xs = [0.0_f64; MAX_SENSOR_STATUS_LANES];
    let mut ys = [0.0_f64; MAX_SENSOR_STATUS_LANES];
    let mut zs = [0.0_f64; MAX_SENSOR_STATUS_LANES];
    for (i, sample) in samples.iter().take(count).enumerate() {
        xs[i] = sample.x;
        ys[i] = sample.y;
        zs[i] = sample.z;
    }
    let vx = voter.vote(&xs[..count])?;
    let vy = voter.vote(&ys[..count])?;
    let vz = voter.vote(&zs[..count])?;
    Some((
        Vector3::new(vx.value, vy.value, vz.value),
        vx.divergent || vy.divergent || vz.divergent,
    ))
}

fn vec3_sample_diverges<V: Voter<f64>>(
    voter: &V,
    sample: Vector3<f64>,
    fused: Vector3<f64>,
) -> bool {
    (0..3).any(|axis| voter.sample_diverges(sample[axis], fused[axis]))
}

/// Voted ingest job for redundant IMU lanes. Reads every source on
/// each tick, votes per-axis using the supplied `Voter<f64>`, and
/// publishes the voted [`ImuSample`].
pub struct VotedImuIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    sensors: Vec<S>,
    voter: V,
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedImuIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        Self {
            sensors,
            voter,
            name: "sensor_ingest.voted_imu",
        }
    }
}

impl<S, V> fmt::Debug for VotedImuIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VotedImuIngest")
            .field("name", &self.name)
            .field("sources", &self.sensors.len())
            .finish_non_exhaustive()
    }
}

impl<S, V> Job for VotedImuIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
    V: Voter<f64> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        let mut gyros = [Vector3::zeros(); MAX_SENSOR_STATUS_LANES];
        let mut accels = [Vector3::zeros(); MAX_SENSOR_STATUS_LANES];
        let mut sample_lanes = [0_usize; MAX_SENSOR_STATUS_LANES];
        let mut sample_count = 0_usize;
        let mut status = SensorStatus {
            time: ctx.clock.now(),
            kind: SensorKind::Imu,
            lane_count: u8::try_from(self.sensors.len().min(MAX_SENSOR_STATUS_LANES))
                .unwrap_or(u8::MAX),
            overflowed: self.sensors.len() > MAX_SENSOR_STATUS_LANES,
            ..SensorStatus::default()
        };
        let mut last_time = openbmp_core::SimTime::ZERO;
        for (lane_index, sensor) in self.sensors.iter_mut().enumerate() {
            if lane_index < MAX_SENSOR_STATUS_LANES {
                status.lanes[lane_index].sensor_id = sensor.sensor_id().value();
                status.lanes[lane_index].lane_index = u8::try_from(lane_index).unwrap_or(u8::MAX);
            }
            if let Ok(Timestamped {
                time,
                value:
                    SensorMeasurement::Imu {
                        gyro_rad_s,
                        accel_m_s2,
                    },
            }) = sensor.read()
            {
                if lane_index < MAX_SENSOR_STATUS_LANES {
                    status.lanes[lane_index].healthy = true;
                }
                if sample_count < MAX_SENSOR_STATUS_LANES {
                    gyros[sample_count] = gyro_rad_s;
                    accels[sample_count] = accel_m_s2;
                    sample_lanes[sample_count] = lane_index;
                    sample_count += 1;
                }
                last_time = time;
            }
        }
        let Some((gyro, gyro_divergent)) = vote_vec3(&self.voter, &gyros[..sample_count]) else {
            ctx.bus.publish(unhealthy_imu_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some((accel, accel_divergent)) = vote_vec3(&self.voter, &accels[..sample_count]) else {
            ctx.bus.publish(unhealthy_imu_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        for sample_index in 0..sample_count {
            let lane_index = sample_lanes[sample_index];
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let gyro_lane_divergent = (0..3).any(|axis| {
                self.voter
                    .sample_diverges(gyros[sample_index][axis], gyro[axis])
            });
            let accel_lane_divergent = (0..3).any(|axis| {
                self.voter
                    .sample_diverges(accels[sample_index][axis], accel[axis])
            });
            let lane_divergent = gyro_lane_divergent || accel_lane_divergent;
            status.lanes[lane_index].divergent = lane_divergent;
            status.any_divergent |= lane_divergent;
        }
        let _ = ctx.bus.publish(ImuSample {
            time: last_time,
            gyro_rad_s: gyro,
            accel_m_s2: accel,
            healthy: !(gyro_divergent || accel_divergent),
        });
        let _ = ctx.bus.publish(status);
        Ok(())
    }
}

/// Voted ingest job for redundant barometer lanes.
pub struct VotedBarometerIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    sensors: Vec<S>,
    voter: V,
    pressures: Vec<f64>,
    biases: Vec<f64>,
    sample_lanes: Vec<usize>,
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedBarometerIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        let sensor_count = sensors.len();
        Self {
            sensors,
            voter,
            pressures: Vec::with_capacity(sensor_count),
            biases: Vec::with_capacity(sensor_count),
            sample_lanes: Vec::with_capacity(sensor_count),
            name: "sensor_ingest.voted_barometer",
        }
    }
}

impl<S, V> fmt::Debug for VotedBarometerIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VotedBarometerIngest")
            .field("name", &self.name)
            .field("sources", &self.sensors.len())
            .finish_non_exhaustive()
    }
}

impl<S, V> Job for VotedBarometerIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
    V: Voter<f64> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.pressures.clear();
        self.biases.clear();
        self.sample_lanes.clear();
        let mut status = SensorStatus {
            time: ctx.clock.now(),
            kind: SensorKind::Barometer,
            lane_count: u8::try_from(self.sensors.len().min(MAX_SENSOR_STATUS_LANES))
                .unwrap_or(u8::MAX),
            overflowed: self.sensors.len() > MAX_SENSOR_STATUS_LANES,
            ..SensorStatus::default()
        };
        let mut last_time = openbmp_core::SimTime::ZERO;
        for (lane_index, sensor) in self.sensors.iter_mut().enumerate() {
            if lane_index < MAX_SENSOR_STATUS_LANES {
                status.lanes[lane_index].sensor_id = sensor.sensor_id().value();
                status.lanes[lane_index].lane_index = u8::try_from(lane_index).unwrap_or(u8::MAX);
            }
            if let Ok(Timestamped {
                time,
                value:
                    SensorMeasurement::Barometer {
                        pressure_pa,
                        bias_pa,
                    },
            }) = sensor.read()
            {
                if lane_index < MAX_SENSOR_STATUS_LANES {
                    status.lanes[lane_index].healthy = true;
                }
                self.pressures.push(pressure_pa);
                self.biases.push(bias_pa);
                self.sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some(p) = self.voter.vote(&self.pressures) else {
            ctx.bus
                .publish(unhealthy_barometer_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(b) = self.voter.vote(&self.biases) else {
            ctx.bus
                .publish(unhealthy_barometer_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        for (sample_index, lane_index) in self.sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent = self
                .voter
                .sample_diverges(self.pressures[sample_index], p.value)
                || self
                    .voter
                    .sample_diverges(self.biases[sample_index], b.value);
            status.lanes[lane_index].divergent = lane_divergent;
            status.any_divergent |= lane_divergent;
        }
        let _ = ctx.bus.publish(BarometerSample {
            time: last_time,
            pressure_pa: p.value,
            bias_pa: b.value,
            healthy: !(p.divergent || b.divergent),
        });
        let _ = ctx.bus.publish(status);
        Ok(())
    }
}

/// Voted ingest job for redundant air-data lanes.
pub struct VotedAirDataIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    sensors: Vec<S>,
    voter: V,
    static_pressures: Vec<f64>,
    impact_pressures: Vec<f64>,
    machs: Vec<f64>,
    calibrated_airspeeds: Vec<f64>,
    true_airspeeds: Vec<f64>,
    angles_of_attack: Vec<f64>,
    sideslips: Vec<f64>,
    pressure_altitudes: Vec<f64>,
    sample_lanes: Vec<usize>,
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedAirDataIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        let sensor_count = sensors.len();
        Self {
            sensors,
            voter,
            static_pressures: Vec::with_capacity(sensor_count),
            impact_pressures: Vec::with_capacity(sensor_count),
            machs: Vec::with_capacity(sensor_count),
            calibrated_airspeeds: Vec::with_capacity(sensor_count),
            true_airspeeds: Vec::with_capacity(sensor_count),
            angles_of_attack: Vec::with_capacity(sensor_count),
            sideslips: Vec::with_capacity(sensor_count),
            pressure_altitudes: Vec::with_capacity(sensor_count),
            sample_lanes: Vec::with_capacity(sensor_count),
            name: "sensor_ingest.voted_airdata",
        }
    }
}

impl<S, V> fmt::Debug for VotedAirDataIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VotedAirDataIngest")
            .field("name", &self.name)
            .field("sources", &self.sensors.len())
            .finish_non_exhaustive()
    }
}

impl<S, V> Job for VotedAirDataIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
    V: Voter<f64> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.static_pressures.clear();
        self.impact_pressures.clear();
        self.machs.clear();
        self.calibrated_airspeeds.clear();
        self.true_airspeeds.clear();
        self.angles_of_attack.clear();
        self.sideslips.clear();
        self.pressure_altitudes.clear();
        self.sample_lanes.clear();
        let mut status = SensorStatus {
            time: ctx.clock.now(),
            kind: SensorKind::AirData,
            lane_count: u8::try_from(self.sensors.len().min(MAX_SENSOR_STATUS_LANES))
                .unwrap_or(u8::MAX),
            overflowed: self.sensors.len() > MAX_SENSOR_STATUS_LANES,
            ..SensorStatus::default()
        };
        let mut last_time = openbmp_core::SimTime::ZERO;
        for (lane_index, sensor) in self.sensors.iter_mut().enumerate() {
            if lane_index < MAX_SENSOR_STATUS_LANES {
                status.lanes[lane_index].sensor_id = sensor.sensor_id().value();
                status.lanes[lane_index].lane_index = u8::try_from(lane_index).unwrap_or(u8::MAX);
            }
            if let Ok(Timestamped {
                time,
                value:
                    SensorMeasurement::AirData {
                        static_pressure_pa,
                        impact_pressure_pa,
                        mach,
                        calibrated_airspeed_m_s,
                        true_airspeed_m_s,
                        angle_of_attack_rad,
                        sideslip_rad,
                        pressure_altitude_m,
                    },
            }) = sensor.read()
            {
                if lane_index < MAX_SENSOR_STATUS_LANES {
                    status.lanes[lane_index].healthy = true;
                }
                self.static_pressures.push(static_pressure_pa);
                self.impact_pressures.push(impact_pressure_pa);
                self.machs.push(mach);
                self.calibrated_airspeeds.push(calibrated_airspeed_m_s);
                self.true_airspeeds.push(true_airspeed_m_s);
                self.angles_of_attack.push(angle_of_attack_rad);
                self.sideslips.push(sideslip_rad);
                self.pressure_altitudes.push(pressure_altitude_m);
                self.sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some(static_pressure) = self.voter.vote(&self.static_pressures) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(impact_pressure) = self.voter.vote(&self.impact_pressures) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(mach) = self.voter.vote(&self.machs) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(calibrated_airspeed) = self.voter.vote(&self.calibrated_airspeeds) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(true_airspeed) = self.voter.vote(&self.true_airspeeds) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(angle_of_attack) = self.voter.vote(&self.angles_of_attack) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(sideslip) = self.voter.vote(&self.sideslips) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some(pressure_altitude) = self.voter.vote(&self.pressure_altitudes) else {
            ctx.bus.publish(unhealthy_airdata_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        for (sample_index, lane_index) in self.sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent = self
                .voter
                .sample_diverges(self.static_pressures[sample_index], static_pressure.value)
                || self
                    .voter
                    .sample_diverges(self.impact_pressures[sample_index], impact_pressure.value)
                || self
                    .voter
                    .sample_diverges(self.machs[sample_index], mach.value)
                || self.voter.sample_diverges(
                    self.calibrated_airspeeds[sample_index],
                    calibrated_airspeed.value,
                )
                || self
                    .voter
                    .sample_diverges(self.true_airspeeds[sample_index], true_airspeed.value)
                || self
                    .voter
                    .sample_diverges(self.angles_of_attack[sample_index], angle_of_attack.value)
                || self
                    .voter
                    .sample_diverges(self.sideslips[sample_index], sideslip.value)
                || self.voter.sample_diverges(
                    self.pressure_altitudes[sample_index],
                    pressure_altitude.value,
                );
            status.lanes[lane_index].divergent = lane_divergent;
            status.any_divergent |= lane_divergent;
        }
        let _ = ctx.bus.publish(AirDataSample {
            time: last_time,
            static_pressure_pa: static_pressure.value,
            impact_pressure_pa: impact_pressure.value,
            mach: mach.value,
            calibrated_airspeed_m_s: calibrated_airspeed.value,
            true_airspeed_m_s: true_airspeed.value,
            angle_of_attack_rad: angle_of_attack.value,
            sideslip_rad: sideslip.value,
            pressure_altitude_m: pressure_altitude.value,
            healthy: !(static_pressure.divergent
                || impact_pressure.divergent
                || mach.divergent
                || calibrated_airspeed.divergent
                || true_airspeed.divergent
                || angle_of_attack.divergent
                || sideslip.divergent
                || pressure_altitude.divergent),
        });
        let _ = ctx.bus.publish(status);
        Ok(())
    }
}

/// Voted ingest job for redundant GNSS lanes.
pub struct VotedGnssIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    sensors: Vec<S>,
    voter: V,
    positions: Vec<Vector3<f64>>,
    velocities: Vec<Vector3<f64>>,
    biases: Vec<Vector3<f64>>,
    sample_lanes: Vec<usize>,
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedGnssIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        let sensor_count = sensors.len();
        Self {
            sensors,
            voter,
            positions: Vec::with_capacity(sensor_count),
            velocities: Vec::with_capacity(sensor_count),
            biases: Vec::with_capacity(sensor_count),
            sample_lanes: Vec::with_capacity(sensor_count),
            name: "sensor_ingest.voted_gnss",
        }
    }
}

impl<S, V> fmt::Debug for VotedGnssIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VotedGnssIngest")
            .field("name", &self.name)
            .field("sources", &self.sensors.len())
            .finish_non_exhaustive()
    }
}

impl<S, V> Job for VotedGnssIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
    V: Voter<f64> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.positions.clear();
        self.velocities.clear();
        self.biases.clear();
        self.sample_lanes.clear();
        let mut status = SensorStatus {
            time: ctx.clock.now(),
            kind: SensorKind::Gnss,
            lane_count: u8::try_from(self.sensors.len().min(MAX_SENSOR_STATUS_LANES))
                .unwrap_or(u8::MAX),
            overflowed: self.sensors.len() > MAX_SENSOR_STATUS_LANES,
            ..SensorStatus::default()
        };
        let mut last_time = openbmp_core::SimTime::ZERO;
        for (lane_index, sensor) in self.sensors.iter_mut().enumerate() {
            if lane_index < MAX_SENSOR_STATUS_LANES {
                status.lanes[lane_index].sensor_id = sensor.sensor_id().value();
                status.lanes[lane_index].lane_index = u8::try_from(lane_index).unwrap_or(u8::MAX);
            }
            if let Ok(Timestamped {
                time,
                value:
                    SensorMeasurement::Gnss {
                        position_eci_m,
                        velocity_eci_m_s,
                        position_bias_eci_m,
                    },
            }) = sensor.read()
            {
                if lane_index < MAX_SENSOR_STATUS_LANES {
                    status.lanes[lane_index].healthy = true;
                }
                self.positions.push(position_eci_m);
                self.velocities.push(velocity_eci_m_s);
                self.biases.push(position_bias_eci_m);
                self.sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some((position, position_div)) = vote_vec3(&self.voter, &self.positions) else {
            ctx.bus.publish(unhealthy_gnss_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some((velocity, velocity_div)) = vote_vec3(&self.voter, &self.velocities) else {
            ctx.bus.publish(unhealthy_gnss_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some((bias, bias_div)) = vote_vec3(&self.voter, &self.biases) else {
            ctx.bus.publish(unhealthy_gnss_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        for (sample_index, lane_index) in self.sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent =
                vec3_sample_diverges(&self.voter, self.positions[sample_index], position)
                    || vec3_sample_diverges(&self.voter, self.velocities[sample_index], velocity)
                    || vec3_sample_diverges(&self.voter, self.biases[sample_index], bias);
            status.lanes[lane_index].divergent = lane_divergent;
            status.any_divergent |= lane_divergent;
        }
        let _ = ctx.bus.publish(GnssSample {
            time: last_time,
            position_eci_m: position,
            velocity_eci_m_s: velocity,
            position_bias_eci_m: bias,
            healthy: !(position_div || velocity_div || bias_div),
        });
        let _ = ctx.bus.publish(status);
        Ok(())
    }
}

/// Voted ingest job for redundant magnetometer lanes.
pub struct VotedMagnetometerIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    sensors: Vec<S>,
    voter: V,
    fields: Vec<Vector3<f64>>,
    hard_irons: Vec<Vector3<f64>>,
    sample_lanes: Vec<usize>,
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedMagnetometerIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        let sensor_count = sensors.len();
        Self {
            sensors,
            voter,
            fields: Vec::with_capacity(sensor_count),
            hard_irons: Vec::with_capacity(sensor_count),
            sample_lanes: Vec::with_capacity(sensor_count),
            name: "sensor_ingest.voted_magnetometer",
        }
    }
}

impl<S, V> fmt::Debug for VotedMagnetometerIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("VotedMagnetometerIngest")
            .field("name", &self.name)
            .field("sources", &self.sensors.len())
            .finish_non_exhaustive()
    }
}

impl<S, V> Job for VotedMagnetometerIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement> + 'static,
    V: Voter<f64> + 'static,
{
    fn name(&self) -> &'static str {
        self.name
    }
    fn run(&mut self, ctx: &JobContext<'_>) -> Result<(), ControllerError> {
        self.fields.clear();
        self.hard_irons.clear();
        self.sample_lanes.clear();
        let mut status = SensorStatus {
            time: ctx.clock.now(),
            kind: SensorKind::Magnetometer,
            lane_count: u8::try_from(self.sensors.len().min(MAX_SENSOR_STATUS_LANES))
                .unwrap_or(u8::MAX),
            overflowed: self.sensors.len() > MAX_SENSOR_STATUS_LANES,
            ..SensorStatus::default()
        };
        let mut last_time = openbmp_core::SimTime::ZERO;
        for (lane_index, sensor) in self.sensors.iter_mut().enumerate() {
            if lane_index < MAX_SENSOR_STATUS_LANES {
                status.lanes[lane_index].sensor_id = sensor.sensor_id().value();
                status.lanes[lane_index].lane_index = u8::try_from(lane_index).unwrap_or(u8::MAX);
            }
            if let Ok(Timestamped {
                time,
                value:
                    SensorMeasurement::Magnetometer {
                        field_body_nt,
                        hard_iron_body_nt,
                    },
            }) = sensor.read()
            {
                if lane_index < MAX_SENSOR_STATUS_LANES {
                    status.lanes[lane_index].healthy = true;
                }
                self.fields.push(field_body_nt);
                self.hard_irons.push(hard_iron_body_nt);
                self.sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some((field, field_div)) = vote_vec3(&self.voter, &self.fields) else {
            ctx.bus
                .publish(unhealthy_magnetometer_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        let Some((hard_iron, hard_iron_div)) = vote_vec3(&self.voter, &self.hard_irons) else {
            ctx.bus
                .publish(unhealthy_magnetometer_sample(ctx.clock.now()))?;
            let _ = ctx.bus.publish(status);
            return Ok(());
        };
        for (sample_index, lane_index) in self.sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent =
                vec3_sample_diverges(&self.voter, self.fields[sample_index], field)
                    || vec3_sample_diverges(&self.voter, self.hard_irons[sample_index], hard_iron);
            status.lanes[lane_index].divergent = lane_divergent;
            status.any_divergent |= lane_divergent;
        }
        let _ = ctx.bus.publish(MagnetometerSample {
            time: last_time,
            field_body_nt: field,
            hard_iron_body_nt: hard_iron,
            healthy: !(field_div || hard_iron_div),
        });
        let _ = ctx.bus.publish(status);
        Ok(())
    }
}
