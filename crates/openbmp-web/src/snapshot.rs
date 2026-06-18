//! Flat numeric state snapshot for render hosts.
//!
//! A renderer polls the simulation many times per second across a
//! marshalling boundary (WebAssembly → JavaScript), so the per-frame
//! state is encoded as one flat `f64` array with a layout fixed at
//! construction. [`SnapshotLayout::to_json`] describes every index plus
//! the scenario-derived constants a renderer needs (body dimensions,
//! engine mounts, tank capacities, phase labels) — the front-end builds
//! the vehicle *from the scenario*, never from a hard-coded model.

use openbmp_core::{BodyId, EngineId};
use openbmp_runner::Session;
use openbmp_scenario::ScenarioDocument;
use serde::Serialize;
use uom::si::f64::Mass;
use uom::si::mass::kilogram;

use crate::session::{body_render_dimensions, tank_volume_m3};

/// Width of one separated-body slot in the snapshot.
const SEPARATED_SLOT_WIDTH: usize = 15;
/// Width of one engine slot in the snapshot.
const ENGINE_SLOT_WIDTH: usize = 6;
/// Width of the fixed header block (`separated_count` is its last
/// scalar; separated-body slots start right after).
const HEADER_WIDTH: usize = 91;

/// Fixed scalar indices within the snapshot header.
#[derive(Clone, Copy, Debug, Serialize)]
pub struct FieldIndices {
    /// Simulation time (s).
    pub time_s: usize,
    /// Kernel step index.
    pub step: usize,
    /// Truth ECI position (m), 3 values.
    pub position_eci_m: usize,
    /// Truth ECI velocity (m/s), 3 values.
    pub velocity_eci_m_s: usize,
    /// Truth body→ECI quaternion (x, y, z, w), 4 values.
    pub attitude_xyzw: usize,
    /// Truth body angular velocity (rad/s), 3 values.
    pub angular_velocity_body_rad_s: usize,
    /// Stack mass (kg).
    pub mass_kg: usize,
    /// Local atmospheric density (kg/m³).
    pub atmosphere_density_kg_m3: usize,
    /// Local atmospheric static pressure (Pa).
    pub atmosphere_pressure_pa: usize,
    /// Guidance time-to-go (s); `NaN` when not published.
    pub guidance_time_to_go_s: usize,
    /// 1.0 when the estimator blocks below are valid.
    pub estimate_valid: usize,
    /// Estimated ECI position (m), 3 values.
    pub estimate_position_eci_m: usize,
    /// Estimated ECI velocity (m/s), 3 values.
    pub estimate_velocity_eci_m_s: usize,
    /// Estimated body→ECI quaternion (x, y, z, w), 4 values.
    pub estimate_attitude_xyzw: usize,
    /// Estimated gyro bias (rad/s), 3 values.
    pub estimate_gyro_bias_rad_s: usize,
    /// 1.0 while the estimator is dead-reckoning.
    pub dead_reckoning: usize,
    /// GNSS innovation chi-square ratio.
    pub gnss_chi2: usize,
    /// IMU innovation chi-square ratio.
    pub imu_chi2: usize,
    /// Star-tracker innovation chi-square ratio.
    pub star_tracker_chi2: usize,
    /// 1.0 while FDIR is triggered.
    pub fdir_triggered: usize,
    /// 1.0 when the guidance-reference quaternion is valid.
    pub reference_valid: usize,
    /// Guidance reference body→ECI quaternion (x, y, z, w), 4 values.
    pub reference_attitude_xyzw: usize,
    // ---- estimator covariance / uncertainty (1σ²) ----
    /// EKF position covariance diagonal (m²), ECI, 3 values.
    pub estimate_position_var: usize,
    /// EKF velocity covariance diagonal (m²/s²), ECI, 3 values.
    pub estimate_velocity_var: usize,
    /// EKF attitude covariance diagonal (rad²), 3 values.
    pub estimate_attitude_var: usize,
    /// Estimated accelerometer bias (m/s²), body, 3 values.
    pub estimate_accel_bias: usize,
    // ---- per-sensor innovations + gating ----
    /// Whitened GNSS innovation (pos 0..3, vel 3..6), 6 values.
    pub gnss_innovation: usize,
    /// Whitened barometric innovation (scalar).
    pub baro_innovation: usize,
    /// Whitened magnetometer innovation, 3 values.
    pub mag_innovation: usize,
    /// Barometric innovation chi-square ratio.
    pub baro_chi2: usize,
    /// Magnetometer innovation chi-square ratio.
    pub mag_chi2: usize,
    /// 1.0 when GNSS updated the filter this tick (else gated/none).
    pub gnss_updated: usize,
    /// 1.0 when baro updated this tick.
    pub baro_updated: usize,
    /// 1.0 when mag updated this tick.
    pub mag_updated: usize,
    /// 1.0 when any measurement was rejected by the innovation gate.
    pub innovation_rejected: usize,
    /// 1.0 while attitude is under-observable.
    pub attitude_under_observable: usize,
    /// Covariance condition proxy.
    pub covariance_condition_proxy: usize,
    // ---- FDIR ----
    /// Bitmask of tripped FDIR detectors.
    pub fdir_tripped_mask: usize,
    /// Ticks since the most recent FDIR trip.
    pub fdir_ticks_since_trip: usize,
    // ---- IMM estimator mode ----
    /// Active estimator mode index (IMM).
    pub estimator_active_mode: usize,
    /// Number of active estimator modes (0/1 for single-model).
    pub estimator_mode_count: usize,
    /// IMM mode posterior probabilities, 4 values.
    pub estimator_mode_probabilities: usize,
    // ---- guidance reference state ----
    /// Guidance reference ECI position (m), 3 values.
    pub reference_position_eci_m: usize,
    /// Guidance reference ECI velocity (m/s), 3 values.
    pub reference_velocity_eci_m_s: usize,
    /// Guidance reference body angular velocity (rad/s), 3 values.
    pub reference_omega_body_rad_s: usize,
    /// Number of separated bodies currently tracked.
    pub separated_count: usize,
    /// First separated-body slot (see `separated_slot_width`).
    pub separated_base: usize,
    /// First engine slot (see `engine_slot_width`).
    pub engines_base: usize,
}

/// One renderable body constant derived from the scenario assembly.
#[derive(Clone, Debug, Serialize)]
pub struct BodyInfo {
    /// Scenario body id.
    pub id: String,
    /// Render length (m).
    pub length_m: f64,
    /// Render diameter (m).
    pub diameter_m: f64,
    /// Dry mass (kg).
    pub dry_mass_kg: f64,
}

/// One engine constant derived from the scenario assembly.
#[derive(Clone, Debug, Serialize)]
pub struct EngineInfo {
    /// Scenario engine id.
    pub id: String,
    /// Owning body id (empty for single-body stacks).
    pub mounted_to: String,
    /// Body-frame mount point (m).
    pub mount_point_body_m: [f64; 3],
    /// Maximum thrust (N) — normalises plume/throttle display.
    pub max_thrust_n: f64,
    /// Maximum gimbal deflection (rad).
    pub max_gimbal_rad: f64,
    /// Fuel tank id, when a propellant budget is declared.
    pub fuel_tank: String,
}

/// One tank constant derived from the scenario assembly.
#[derive(Clone, Debug, Serialize)]
pub struct TankInfo {
    /// Scenario tank id.
    pub id: String,
    /// Owning body id.
    pub mounted_to: String,
    /// Initial propellant load (kg) = volume × density × fill.
    pub initial_propellant_kg: f64,
}

/// One mission phase label.
#[derive(Clone, Debug, Serialize)]
pub struct PhaseInfo {
    /// Stable phase id.
    pub id: String,
    /// Human-readable label.
    pub label: String,
    /// Engine ids permitted in this phase (informational).
    pub allowed_engines: Vec<String>,
}

/// Snapshot layout plus scenario-derived render constants.
#[derive(Clone, Debug, Serialize)]
pub struct SnapshotLayout {
    /// Layout schema version.
    pub version: u32,
    /// Embedded scenario key.
    pub key: String,
    /// Embedded scenario picker title.
    pub title: String,
    /// Scenario `[meta] name`.
    pub name: String,
    /// Scenario `[meta] description`.
    pub description: String,
    /// Scenario `[meta] validation` label, verbatim.
    pub validation: String,
    /// Scenario stop time (s).
    pub stop_s: f64,
    /// Kernel step (s).
    pub dt_s: f64,
    /// Flight-controller base rate (Hz); 0 when no `[fc]` block.
    pub fc_rate_hz: u32,
    /// Total snapshot length in `f64` values.
    pub stride: usize,
    /// Fixed header indices.
    pub fields: FieldIndices,
    /// Width of one separated-body slot.
    pub separated_slot_width: usize,
    /// Width of one engine slot.
    pub engine_slot_width: usize,
    /// Bodies in scenario declaration order (separated-body slot `i`
    /// reports `bodies[i]` when that body has departed the stack).
    pub bodies: Vec<BodyInfo>,
    /// Engines in scenario declaration order (engine slot `i` reports
    /// `engines[i]`).
    pub engines: Vec<EngineInfo>,
    /// Tanks in scenario declaration order.
    pub tanks: Vec<TankInfo>,
    /// Mission phases in scenario declaration order.
    pub phases: Vec<PhaseInfo>,
    /// Declared sensor keys.
    pub sensors: Vec<String>,
}

impl SnapshotLayout {
    /// Derive the layout and render constants from a parsed scenario.
    #[must_use]
    pub fn from_document(key: &str, title: &str, document: &ScenarioDocument) -> Self {
        let assembly = &document.vehicle.assembly;
        let bodies: Vec<BodyInfo> = assembly
            .bodies
            .iter()
            .map(|body| {
                let (length_m, diameter_m) = body_render_dimensions(&body.geometry);
                BodyInfo {
                    id: body.id.clone(),
                    length_m,
                    diameter_m,
                    dry_mass_kg: body.dry_mass_kg,
                }
            })
            .collect();
        let engines: Vec<EngineInfo> = assembly
            .engines
            .iter()
            .map(|engine| EngineInfo {
                id: engine.id.clone(),
                mounted_to: engine.mounted_to.clone().unwrap_or_default(),
                mount_point_body_m: engine.mount_point_body_m,
                max_thrust_n: engine.limits.max_thrust_n,
                max_gimbal_rad: engine.limits.max_gimbal_rad,
                fuel_tank: engine
                    .propellant
                    .as_ref()
                    .map(|propellant| propellant.fuel_tank.clone())
                    .unwrap_or_default(),
            })
            .collect();
        let tanks: Vec<TankInfo> = assembly
            .tanks
            .iter()
            .map(|tank| TankInfo {
                id: tank.id.clone(),
                mounted_to: tank.mounted_to.clone(),
                initial_propellant_kg: tank_volume_m3(&tank.geometry)
                    * tank.propellant.density_kg_m3
                    * tank.initial_fill_fraction,
            })
            .collect();
        let phases: Vec<PhaseInfo> = document
            .mission
            .as_ref()
            .map(|mission| {
                mission
                    .phases
                    .iter()
                    .map(|phase| PhaseInfo {
                        id: phase.id.clone(),
                        label: phase.label.clone(),
                        allowed_engines: phase.allowed_engines.clone(),
                    })
                    .collect()
            })
            .unwrap_or_default();
        let sensors: Vec<String> = document
            .sensors
            .as_ref()
            .map(|sensors| sensors.keys().cloned().collect())
            .unwrap_or_default();

        let fields = FieldIndices {
            time_s: 0,
            step: 1,
            position_eci_m: 2,
            velocity_eci_m_s: 5,
            attitude_xyzw: 8,
            angular_velocity_body_rad_s: 12,
            mass_kg: 15,
            atmosphere_density_kg_m3: 16,
            atmosphere_pressure_pa: 17,
            guidance_time_to_go_s: 18,
            estimate_valid: 19,
            estimate_position_eci_m: 20,
            estimate_velocity_eci_m_s: 23,
            estimate_attitude_xyzw: 26,
            estimate_gyro_bias_rad_s: 30,
            dead_reckoning: 33,
            gnss_chi2: 34,
            imu_chi2: 35,
            star_tracker_chi2: 36,
            fdir_triggered: 37,
            reference_valid: 38,
            reference_attitude_xyzw: 39,
            estimate_position_var: 43,
            estimate_velocity_var: 46,
            estimate_attitude_var: 49,
            estimate_accel_bias: 52,
            gnss_innovation: 55,
            baro_innovation: 61,
            mag_innovation: 62,
            baro_chi2: 65,
            mag_chi2: 66,
            gnss_updated: 67,
            baro_updated: 68,
            mag_updated: 69,
            innovation_rejected: 70,
            attitude_under_observable: 71,
            covariance_condition_proxy: 72,
            fdir_tripped_mask: 73,
            fdir_ticks_since_trip: 74,
            estimator_active_mode: 75,
            estimator_mode_count: 76,
            estimator_mode_probabilities: 77,
            reference_position_eci_m: 81,
            reference_velocity_eci_m_s: 84,
            reference_omega_body_rad_s: 87,
            separated_count: HEADER_WIDTH - 1,
            separated_base: HEADER_WIDTH,
            engines_base: HEADER_WIDTH + bodies.len() * SEPARATED_SLOT_WIDTH,
        };
        let stride = fields.engines_base + engines.len() * ENGINE_SLOT_WIDTH;

        Self {
            version: 2,
            key: key.to_owned(),
            title: title.to_owned(),
            name: document.meta.name.clone(),
            description: document.meta.description.clone(),
            validation: document.meta.validation.as_label().to_owned(),
            stop_s: document.time.stop_s,
            dt_s: document.time.dt_s,
            fc_rate_hz: document.fc.as_ref().map_or(0, |fc| fc.base_rate_hz),
            stride,
            fields,
            separated_slot_width: SEPARATED_SLOT_WIDTH,
            engine_slot_width: ENGINE_SLOT_WIDTH,
            bodies,
            engines,
            tanks,
            phases,
            sensors,
        }
    }

    /// Serialise the layout as JSON.
    #[must_use]
    pub fn to_json(&self) -> String {
        serde_json::to_string(self)
            .unwrap_or_else(|_| unreachable!("layout serialisation is infallible"))
    }

    /// Encode the session's current state into `out` (length
    /// [`Self::stride`]).
    pub fn encode(
        &self,
        session: &Session,
        engine_ids: &[(String, EngineId)],
        body_ids: &[(String, BodyId)],
        out: &mut Vec<f64>,
    ) {
        out.clear();
        out.resize(self.stride, 0.0);
        let f = &self.fields;

        out[f.time_s] = session.time_s();
        #[allow(clippy::cast_precision_loss)] // step counts stay far below 2^52
        {
            out[f.step] = session.step_index() as f64;
        }

        let state = session.state();
        write_vector3(out, f.position_eci_m, &state.position.vector);
        write_vector3(out, f.velocity_eci_m_s, &state.velocity.vector);
        write_quaternion_xyzw(out, f.attitude_xyzw, state.orientation.q.coords.as_slice());
        write_vector3(
            out,
            f.angular_velocity_body_rad_s,
            &state.angular_velocity.vector,
        );
        out[f.mass_kg] = mass_kg(state.mass_props.mass);

        if let Ok(environment) = session.environment_sample() {
            out[f.atmosphere_density_kg_m3] = environment.atmosphere_density_kg_m3;
            out[f.atmosphere_pressure_pa] = environment.atmosphere_pressure_pa;
        }

        out[f.guidance_time_to_go_s] = session
            .latest_guidance_cutoff()
            .map_or(f64::NAN, |cutoff| cutoff.time_to_go_s);

        if let Some(observation) = session.fc_observation() {
            if let Some(position) = observation.position {
                out[f.estimate_valid] = 1.0;
                write_vector3(out, f.estimate_position_eci_m, &position.position_eci_m);
                write_vector3(out, f.estimate_velocity_eci_m_s, &position.velocity_eci_m_s);
                write_vector3(out, f.estimate_accel_bias, &position.accel_bias_body_m_s2);
            }
            if let Some(attitude) = observation.attitude {
                write_quaternion_xyzw(out, f.estimate_attitude_xyzw, &attitude.q_body_to_eci_xyzw);
                write_vector3(
                    out,
                    f.estimate_gyro_bias_rad_s,
                    &attitude.gyro_bias_body_rad_s,
                );
            }
            if let Some(estimator) = observation.estimator {
                out[f.dead_reckoning] = f64::from(u8::from(estimator.dead_reckoning));
                out[f.gnss_chi2] = estimator.gnss_chi2;
                out[f.imu_chi2] = estimator.imu_chi2;
                out[f.star_tracker_chi2] = estimator.star_tracker_chi2;
                out[f.baro_chi2] = estimator.baro_chi2;
                out[f.mag_chi2] = estimator.mag_chi2;
                out[f.estimate_position_var..f.estimate_position_var + 3]
                    .copy_from_slice(&estimator.position_variance_eci_m2);
                out[f.estimate_velocity_var..f.estimate_velocity_var + 3]
                    .copy_from_slice(&estimator.velocity_variance_eci_m2_s2);
                out[f.estimate_attitude_var..f.estimate_attitude_var + 3]
                    .copy_from_slice(&estimator.attitude_variance_rad2);
                out[f.gnss_innovation..f.gnss_innovation + 6]
                    .copy_from_slice(&estimator.gnss_innovation_whitened);
                out[f.baro_innovation] = estimator.baro_innovation_whitened;
                out[f.mag_innovation..f.mag_innovation + 3]
                    .copy_from_slice(&estimator.mag_innovation_whitened);
                out[f.gnss_updated] = f64::from(u8::from(estimator.gnss_updated_this_tick));
                out[f.baro_updated] = f64::from(u8::from(estimator.baro_updated_this_tick));
                out[f.mag_updated] = f64::from(u8::from(estimator.mag_updated_this_tick));
                out[f.innovation_rejected] = f64::from(u8::from(estimator.innovation_rejected));
                out[f.attitude_under_observable] =
                    f64::from(u8::from(estimator.attitude_under_observable));
                out[f.covariance_condition_proxy] = estimator.covariance_condition_proxy;
            }
            if let Some(fdir) = observation.fdir {
                out[f.fdir_triggered] = f64::from(u8::from(fdir.triggered));
                #[allow(clippy::cast_precision_loss)]
                {
                    out[f.fdir_tripped_mask] = fdir.tripped_mask as f64;
                    out[f.fdir_ticks_since_trip] = fdir.ticks_since_trip as f64;
                }
            }
            if let Some(mode) = observation.mode {
                out[f.estimator_active_mode] = f64::from(mode.active_mode);
                out[f.estimator_mode_count] = f64::from(mode.mode_count);
                out[f.estimator_mode_probabilities..f.estimator_mode_probabilities + 4]
                    .copy_from_slice(&mode.mode_probabilities);
            }
            if let Some(reference) = observation.reference {
                out[f.reference_valid] = 1.0;
                write_quaternion_xyzw(
                    out,
                    f.reference_attitude_xyzw,
                    &reference.q_body_to_eci_xyzw,
                );
                write_vector3(out, f.reference_position_eci_m, &reference.position_eci_m);
                write_vector3(
                    out,
                    f.reference_velocity_eci_m_s,
                    &reference.velocity_eci_m_s,
                );
                write_vector3(
                    out,
                    f.reference_omega_body_rad_s,
                    &reference.omega_body_rad_s,
                );
            }
        }

        let separated = session.separated_bodies();
        #[allow(clippy::cast_precision_loss)] // body counts are tiny
        {
            out[f.separated_count] = separated.len() as f64;
        }
        for (slot, (_, body_id)) in body_ids.iter().enumerate() {
            let base = f.separated_base + slot * SEPARATED_SLOT_WIDTH;
            let Some(lane) = separated.iter().find(|lane| lane.body == *body_id) else {
                continue;
            };
            out[base] = 1.0;
            out[base + 1] = f64::from(u8::from(lane.propagating));
            write_vector3(out, base + 2, &lane.state.position.vector);
            write_vector3(out, base + 5, &lane.state.velocity.vector);
            write_quaternion_xyzw(out, base + 8, lane.state.orientation.q.coords.as_slice());
            write_vector3(out, base + 12, &lane.state.angular_velocity.vector);
        }

        let snapshots = session.engine_snapshots();
        for (slot, (_, engine_id)) in engine_ids.iter().enumerate() {
            let base = f.engines_base + slot * ENGINE_SLOT_WIDTH;
            let Some(snapshot) = snapshots.get(engine_id) else {
                continue;
            };
            write_vector3(out, base, &snapshot.thrust_body);
            out[base + 3] = f64::from(snapshot.lifecycle_state_index);
            out[base + 4] = snapshot.consumed_kg;
            out[base + 5] = snapshot.mass_flow_kg_per_s;
        }
    }
}

fn write_vector3(out: &mut [f64], base: usize, vector: &nalgebra::Vector3<f64>) {
    out[base] = vector.x;
    out[base + 1] = vector.y;
    out[base + 2] = vector.z;
}

fn write_quaternion_xyzw(out: &mut [f64], base: usize, xyzw: &[f64]) {
    out[base..base + 4].copy_from_slice(&xyzw[..4]);
}

fn mass_kg(mass: Mass) -> f64 {
    mass.get::<kilogram>()
}
