//! Sensor ingest jobs.
//!
//! Each per-sensor-kind ingest job pulls measurements from its
//! `Sensor::read()` source, votes redundant lanes via the
//! [`crate::voter`] module, and publishes the voted result on the
//! corresponding sensor topic. The simulator's runner constructs
//! these jobs around `SyntheticSensorAdapter` instances; a downstream
//! HAL adopter constructs them around real driver impls.
//!
//! Phase 4.2: ingest jobs for the IMU, barometer, GNSS,
//! magnetometer, and star tracker. Each job is a [`crate::Job`]
//! ready to register with the scheduler.

use std::fmt;

use nalgebra::Vector3;
use openbmp_sensors::{Sensor, SensorMeasurement, Timestamped};

use crate::error::ControllerError;
use crate::scheduler::{Job, JobContext};
use crate::topics::{
    BarometerSample, GnssSample, ImuSample, MAX_SENSOR_STATUS_LANES, MagnetometerSample,
    SensorKind, SensorStatus, StarTrackerSample,
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
ingest_debug_impl!(GnssIngest);
ingest_debug_impl!(MagnetometerIngest);
ingest_debug_impl!(StarTrackerIngest);

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
        match self.sensor.read() {
            Ok(Timestamped { time, value }) => {
                if let SensorMeasurement::Imu {
                    gyro_rad_s,
                    accel_m_s2,
                } = value
                {
                    let _ = ctx.bus.publish(ImuSample {
                        time,
                        gyro_rad_s,
                        accel_m_s2,
                        healthy: true,
                    })?;
                }
                Ok(())
            }
            Err(_) => Ok(()),
        }
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
        match self.sensor.read() {
            Ok(Timestamped { time, value }) => {
                if let SensorMeasurement::Barometer {
                    pressure_pa,
                    bias_pa,
                } = value
                {
                    let _ = ctx.bus.publish(BarometerSample {
                        time,
                        pressure_pa,
                        bias_pa,
                        healthy: true,
                    })?;
                }
                Ok(())
            }
            Err(_) => Ok(()),
        }
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
        match self.sensor.read() {
            Ok(Timestamped { time, value }) => {
                if let SensorMeasurement::Gnss {
                    position_eci_m,
                    velocity_eci_m_s,
                    position_bias_eci_m,
                } = value
                {
                    let _ = ctx.bus.publish(GnssSample {
                        time,
                        position_eci_m,
                        velocity_eci_m_s,
                        position_bias_eci_m,
                        healthy: true,
                    })?;
                }
                Ok(())
            }
            Err(_) => Ok(()),
        }
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
        match self.sensor.read() {
            Ok(Timestamped { time, value }) => {
                if let SensorMeasurement::Magnetometer {
                    field_body_nt,
                    hard_iron_body_nt,
                } = value
                {
                    let _ = ctx.bus.publish(MagnetometerSample {
                        time,
                        field_body_nt,
                        hard_iron_body_nt,
                        healthy: true,
                    })?;
                }
                Ok(())
            }
            Err(_) => Ok(()),
        }
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
        match self.sensor.read() {
            Ok(Timestamped { time, value }) => {
                if let SensorMeasurement::StarTracker {
                    attitude_eci_to_body,
                } = value
                {
                    let q = attitude_eci_to_body.into_inner();
                    let _ = ctx.bus.publish(StarTrackerSample {
                        time,
                        q_eci_to_body_xyzw: [q.i, q.j, q.k, q.w],
                        healthy: true,
                    })?;
                }
                Ok(())
            }
            Err(_) => Ok(()),
        }
    }
}

// ---------------------------------------------------------------------
// Voted ingest jobs (Phase 4.B)
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
            return Ok(());
        };
        let Some((accel, accel_divergent)) = vote_vec3(&self.voter, &accels[..sample_count]) else {
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
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedBarometerIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        Self {
            sensors,
            voter,
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
        let mut pressures = Vec::with_capacity(self.sensors.len());
        let mut biases = Vec::with_capacity(self.sensors.len());
        let mut sample_lanes = Vec::with_capacity(self.sensors.len());
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
                pressures.push(pressure_pa);
                biases.push(bias_pa);
                sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some(p) = self.voter.vote(&pressures) else {
            return Ok(());
        };
        let Some(b) = self.voter.vote(&biases) else {
            return Ok(());
        };
        for (sample_index, lane_index) in sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent = self.voter.sample_diverges(pressures[sample_index], p.value)
                || self.voter.sample_diverges(biases[sample_index], b.value);
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

/// Voted ingest job for redundant GNSS lanes.
pub struct VotedGnssIngest<S, V>
where
    S: Sensor<Output = SensorMeasurement>,
    V: Voter<f64>,
{
    sensors: Vec<S>,
    voter: V,
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedGnssIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        Self {
            sensors,
            voter,
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
        let mut positions = Vec::with_capacity(self.sensors.len());
        let mut velocities = Vec::with_capacity(self.sensors.len());
        let mut biases = Vec::with_capacity(self.sensors.len());
        let mut sample_lanes = Vec::with_capacity(self.sensors.len());
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
                positions.push(position_eci_m);
                velocities.push(velocity_eci_m_s);
                biases.push(position_bias_eci_m);
                sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some((position, position_div)) = vote_vec3(&self.voter, &positions) else {
            return Ok(());
        };
        let Some((velocity, velocity_div)) = vote_vec3(&self.voter, &velocities) else {
            return Ok(());
        };
        let Some((bias, bias_div)) = vote_vec3(&self.voter, &biases) else {
            return Ok(());
        };
        for (sample_index, lane_index) in sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent =
                vec3_sample_diverges(&self.voter, positions[sample_index], position)
                    || vec3_sample_diverges(&self.voter, velocities[sample_index], velocity)
                    || vec3_sample_diverges(&self.voter, biases[sample_index], bias);
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
    name: &'static str,
}

impl<S: Sensor<Output = SensorMeasurement>, V: Voter<f64>> VotedMagnetometerIngest<S, V> {
    /// Constructs the voted ingest job.
    pub fn new(sensors: Vec<S>, voter: V) -> Self {
        Self {
            sensors,
            voter,
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
        let mut fields = Vec::with_capacity(self.sensors.len());
        let mut hard_irons = Vec::with_capacity(self.sensors.len());
        let mut sample_lanes = Vec::with_capacity(self.sensors.len());
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
                fields.push(field_body_nt);
                hard_irons.push(hard_iron_body_nt);
                sample_lanes.push(lane_index);
                last_time = time;
            }
        }
        let Some((field, field_div)) = vote_vec3(&self.voter, &fields) else {
            return Ok(());
        };
        let Some((hard_iron, hard_iron_div)) = vote_vec3(&self.voter, &hard_irons) else {
            return Ok(());
        };
        for (sample_index, lane_index) in sample_lanes.iter().copied().enumerate() {
            if lane_index >= MAX_SENSOR_STATUS_LANES {
                continue;
            }
            let lane_divergent = vec3_sample_diverges(&self.voter, fields[sample_index], field)
                || vec3_sample_diverges(&self.voter, hard_irons[sample_index], hard_iron);
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
