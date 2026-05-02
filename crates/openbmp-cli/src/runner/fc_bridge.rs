//! Kernel-side flight-controller bridge.
//!
//! `fc.rs` constructs a standalone controller from a parsed `[fc]`
//! block. This module is the runtime bridge between a kernel run and
//! that controller: it owns synthetic sensor adapters, translates the
//! kernel state into `openbmp-sensors::SensorTruth`, steps the FC once
//! per kernel tick, then pushes gated FC commands back into the
//! runner-side effector / engine racks.

use std::collections::BTreeMap;

use nalgebra::{UnitQuaternion, Vector3};
use openbmp_core::{Position3, SensorId, StepIndex, Velocity3};
use openbmp_fc::topics::{
    BarometerSample, GnssSample, ImuSample, MagnetometerSample, StarTrackerSample,
};
use openbmp_physics::atmosphere::{AtmosphereModel, UsStandard1976};
use openbmp_physics::magnetic::{EarthDipoleField, MagneticFieldEci, Wmm2025};
use openbmp_scenario::{
    FcConfig, FcEstimatorKind, FcMagFieldKind, ResolvedFile, Scenario, ScenarioDocument,
    SensorConfig,
};
use openbmp_sensors::{
    GnssNoiseBudget, IdealStateSensor, ImuNoiseBudget, MagnetometerNoiseBudget,
    Sensor as SensorTrait, SensorMeasurement, SensorTruth, StarTrackerNoiseBudget,
    SyntheticBarometer, SyntheticGnss, SyntheticImu, SyntheticMagnetometer, SyntheticSensorAdapter,
    SyntheticStarTracker,
};
use openbmp_state::{PointMassState, RigidBodyState};

use crate::error::CliError;
use crate::runner::fc::{FcAutopilotLqrContext, FcRunner};

const DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR: f64 = 2025.0;

/// Optional FC bridge. Absent when the scenario has no `[fc]` block.
#[derive(Debug)]
pub struct FcBridge {
    runner: FcRunner,
    sensors: Vec<BridgeSensor>,
    atmosphere: UsStandard1976,
    magnetic: Box<dyn MagneticFieldEci>,
    scenario_seed: u64,
    previous_velocity_eci_m_s: Option<Vector3<f64>>,
    previous_time_s: Option<f64>,
}

impl FcBridge {
    /// Build the bridge when `[fc]` is present; otherwise return
    /// `None` so legacy scenarios remain byte-stable.
    ///
    /// # Errors
    ///
    /// Returns [`CliError`] when the FC config, mission graph, frame
    /// profile, or synthetic sensor config cannot be resolved.
    pub fn maybe_new(
        scenario: &Scenario,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Option<Self>, CliError> {
        let Some(fc_config) = &scenario.document.fc else {
            return Ok(None);
        };
        require_bridge_frame(&scenario.document)?;
        let Some(mission) = &scenario.document.mission else {
            return Err(CliError::UnsupportedScenario {
                what:
                    "[fc] requires a [mission] graph so the FC commander and kernel share phase ids"
                        .to_owned(),
            });
        };
        let (bindings, graph) = crate::runner::mission::build_mission_runtime(mission)?;
        let start_phase = graph.initial;
        let magnetic = build_magnetic_field(fc_config)?;
        let lqr_ctx = build_autopilot_lqr_context(scenario)?;
        let runner =
            FcRunner::new(fc_config, graph, bindings, start_phase, lqr_ctx).map_err(|err| {
                CliError::UnsupportedScenario {
                    what: format!("flight-controller construction failed: {err}"),
                }
            })?;
        let sensors = build_sensors(&scenario.document, resolved_files)?;
        Ok(Some(Self {
            runner,
            sensors,
            atmosphere: UsStandard1976::new(),
            magnetic,
            scenario_seed: scenario.document.time.seed,
            previous_velocity_eci_m_s: None,
            previous_time_s: None,
        }))
    }

    /// Run one point-mass bridge tick and push FC commands into the
    /// runner-side racks.
    ///
    /// # Errors
    ///
    /// Propagates sensor, controller, or rack errors as [`CliError`].
    pub fn tick_point_mass(
        &mut self,
        state: &PointMassState,
        step: StepIndex,
        gravity_eci_m_s2: Vector3<f64>,
        effectors: &mut crate::runner::effectors::EffectorRack,
        engines: &mut crate::runner::engines::EngineRack,
    ) -> Result<(), CliError> {
        let truth = self.point_mass_truth(state, gravity_eci_m_s2);
        self.step_from_truth(truth, step, effectors, engines)
    }

    /// Run one rigid-body bridge tick and push FC commands into the
    /// runner-side racks.
    ///
    /// # Errors
    ///
    /// Propagates sensor, controller, or rack errors as [`CliError`].
    pub fn tick_rigid_body(
        &mut self,
        state: &RigidBodyState,
        step: StepIndex,
        gravity_eci_m_s2: Vector3<f64>,
        effectors: &mut crate::runner::effectors::EffectorRack,
        engines: &mut crate::runner::engines::EngineRack,
    ) -> Result<(), CliError> {
        let truth = self.rigid_body_truth(state, gravity_eci_m_s2);
        self.step_from_truth(truth, step, effectors, engines)
    }

    fn step_from_truth(
        &mut self,
        truth: SensorTruth,
        step: StepIndex,
        effectors: &mut crate::runner::effectors::EffectorRack,
        engines: &mut crate::runner::engines::EngineRack,
    ) -> Result<(), CliError> {
        for index in 0..self.sensors.len() {
            let measurement = {
                let sensor = &mut self.sensors[index];
                sensor.prime(truth, step, self.scenario_seed);
                sensor.read()?
            };
            self.publish_measurement(&measurement);
        }
        self.runner
            .step(truth.time, step)
            .map_err(|err| CliError::UnsupportedScenario {
                what: format!("flight-controller tick failed: {err}"),
            })?;
        if let Some(commands) = self.runner.latest_effector_command_set()
            && !effectors.is_empty()
        {
            effectors.apply_fc_commands(&commands)?;
        }
        if let Some(commands) = self.runner.latest_engine_command_set()
            && !engines.is_empty()
        {
            engines.apply_fc_commands(&commands)?;
        }
        Ok(())
    }

    fn publish_measurement(&self, measurement: &openbmp_sensors::Timestamped<SensorMeasurement>) {
        match measurement.value {
            SensorMeasurement::IdealState(_) => {}
            SensorMeasurement::Imu {
                gyro_rad_s,
                accel_m_s2,
            } => self.runner.publish_imu(ImuSample {
                time: measurement.time,
                gyro_rad_s,
                accel_m_s2,
                healthy: true,
            }),
            SensorMeasurement::Barometer {
                pressure_pa,
                bias_pa,
            } => self.runner.publish_barometer(BarometerSample {
                time: measurement.time,
                pressure_pa,
                bias_pa,
                healthy: true,
            }),
            SensorMeasurement::Gnss {
                position_eci_m,
                velocity_eci_m_s,
                position_bias_eci_m,
            } => self.runner.publish_gnss(GnssSample {
                time: measurement.time,
                position_eci_m,
                velocity_eci_m_s,
                position_bias_eci_m,
                healthy: true,
            }),
            SensorMeasurement::Magnetometer {
                field_body_nt,
                hard_iron_body_nt,
            } => self.runner.publish_magnetometer(MagnetometerSample {
                time: measurement.time,
                field_body_nt,
                hard_iron_body_nt,
                healthy: true,
            }),
            SensorMeasurement::StarTracker {
                attitude_eci_to_body,
            } => {
                let q = attitude_eci_to_body.into_inner();
                self.runner.publish_star_tracker(StarTrackerSample {
                    time: measurement.time,
                    q_eci_to_body_xyzw: [q.i, q.j, q.k, q.w],
                    healthy: true,
                });
            }
        }
    }

    fn point_mass_truth(
        &mut self,
        state: &PointMassState,
        gravity_eci_m_s2: Vector3<f64>,
    ) -> SensorTruth {
        let attitude_body_to_eci = UnitQuaternion::identity();
        let specific_force_eci = self.specific_force_eci(
            state.velocity.vector,
            state.time.as_seconds(),
            gravity_eci_m_s2,
        );
        self.truth_common(
            state.position,
            state.velocity,
            attitude_body_to_eci,
            Vector3::zeros(),
            specific_force_eci,
            state.time,
        )
    }

    fn rigid_body_truth(
        &mut self,
        state: &RigidBodyState,
        gravity_eci_m_s2: Vector3<f64>,
    ) -> SensorTruth {
        let attitude_body_to_eci = state.orientation.q;
        let specific_force_eci = self.specific_force_eci(
            state.velocity.vector,
            state.time.as_seconds(),
            gravity_eci_m_s2,
        );
        self.truth_common(
            state.position,
            state.velocity,
            attitude_body_to_eci,
            state.angular_velocity.vector,
            specific_force_eci,
            state.time,
        )
    }

    fn truth_common(
        &self,
        position: Position3<openbmp_core::Eci>,
        velocity: Velocity3<openbmp_core::Eci>,
        attitude_body_to_eci: UnitQuaternion<f64>,
        angular_velocity_body_rad_s: Vector3<f64>,
        specific_force_eci_m_s2: Vector3<f64>,
        time: openbmp_core::SimTime,
    ) -> SensorTruth {
        let altitude_m = position.vector.z.max(0.0);
        let static_pressure_pa = self.atmosphere.sample(altitude_m, time).map_or(
            openbmp_physics::atmosphere::USSA76_SEA_LEVEL_PRESSURE_PA,
            |s| s.pressure_pa,
        );
        let attitude_eci_to_body = attitude_body_to_eci.inverse();
        let specific_force_body_m_s2 = attitude_eci_to_body * specific_force_eci_m_s2;
        let magnetic_field_body_nt =
            attitude_eci_to_body * self.magnetic.field_eci_nt(position.vector, time);
        SensorTruth {
            position_eci: position,
            velocity_eci: velocity,
            attitude_eci_to_body,
            angular_velocity_body_rad_s,
            specific_force_body_m_s2,
            static_pressure_pa,
            altitude_geometric_m: altitude_m,
            magnetic_field_body_nt,
            time,
        }
    }

    fn specific_force_eci(
        &mut self,
        velocity_eci_m_s: Vector3<f64>,
        time_s: f64,
        gravity_eci_m_s2: Vector3<f64>,
    ) -> Vector3<f64> {
        let total_accel = match (self.previous_velocity_eci_m_s, self.previous_time_s) {
            (Some(prev_v), Some(prev_t)) if time_s > prev_t => {
                (velocity_eci_m_s - prev_v) / (time_s - prev_t)
            }
            _ => Vector3::zeros(),
        };
        self.previous_velocity_eci_m_s = Some(velocity_eci_m_s);
        self.previous_time_s = Some(time_s);
        total_accel - gravity_eci_m_s2
    }
}

#[derive(Debug)]
enum BridgeSensor {
    Imu(SyntheticSensorAdapter<SyntheticImu>),
    Barometer(SyntheticSensorAdapter<SyntheticBarometer>),
    Gnss(SyntheticSensorAdapter<SyntheticGnss>),
    Magnetometer(SyntheticSensorAdapter<SyntheticMagnetometer>),
    StarTracker(SyntheticSensorAdapter<SyntheticStarTracker>),
    Ideal(SyntheticSensorAdapter<IdealStateSensor>),
}

impl BridgeSensor {
    fn prime(&mut self, truth: SensorTruth, step: StepIndex, seed: u64) {
        match self {
            Self::Imu(s) => s.prime(truth, step, seed),
            Self::Barometer(s) => s.prime(truth, step, seed),
            Self::Gnss(s) => s.prime(truth, step, seed),
            Self::Magnetometer(s) => s.prime(truth, step, seed),
            Self::StarTracker(s) => s.prime(truth, step, seed),
            Self::Ideal(s) => s.prime(truth, step, seed),
        }
    }

    fn read(&mut self) -> Result<openbmp_sensors::Timestamped<SensorMeasurement>, CliError> {
        match self {
            Self::Imu(s) => s.read(),
            Self::Barometer(s) => s.read(),
            Self::Gnss(s) => s.read(),
            Self::Magnetometer(s) => s.read(),
            Self::StarTracker(s) => s.read(),
            Self::Ideal(s) => s.read(),
        }
        .map_err(|err| CliError::UnsupportedScenario {
            what: format!("synthetic sensor read failed: {err}"),
        })
    }
}

fn require_bridge_frame(document: &ScenarioDocument) -> Result<(), CliError> {
    if document.environment.frame_profile == "toy-fixed-earth" {
        return Ok(());
    }
    Err(CliError::UnsupportedScenario {
        what: format!(
            "[fc] bridge requires frame_profile = \"toy-fixed-earth\"; got `{}`",
            document.environment.frame_profile
        ),
    })
}

/// Phase 5.A.3.B helper: extract the diagonal moments of inertia
/// from a single-body assembly so the runner can solve the per-axis
/// LQR DARE at scenario load. Multi-body assemblies fail closed until
/// a later slice solves gains against the full assembled mass
/// properties.
/// Returns `Ok(None)` when the FC scenario does not request the
/// LQR rate loop, or when the vehicle has no inertia matrix
/// declared (e.g. a point-mass kernel). Fails closed if LQR is
/// requested but the assembly is multi-body or the inertia matrix is
/// non-diagonal.
fn build_autopilot_lqr_context(
    scenario: &Scenario,
) -> Result<Option<FcAutopilotLqrContext>, CliError> {
    let Some(fc_config) = &scenario.document.fc else {
        return Ok(None);
    };
    let Some(autopilot_params) = fc_config.autopilot_params.as_ref() else {
        return Ok(None);
    };
    if autopilot_params.rate_loop_kind != Some(openbmp_scenario::FcRateLoopKind::Lqr) {
        return Ok(None);
    }
    let bodies = &scenario.document.vehicle.assembly.bodies;
    if bodies.len() != 1 {
        return Err(CliError::UnsupportedScenario {
            what: format!(
                "rate_loop_kind = \"lqr\" requires exactly one [[vehicle.assembly.bodies]] entry \
                 with diagonal inertia in Phase 5.A.3.B; got {} bodies",
                bodies.len()
            ),
        });
    }
    let body = bodies
        .first()
        .ok_or_else(|| CliError::UnsupportedScenario {
            what:
                "rate_loop_kind = \"lqr\" requires at least one [[vehicle.assembly.bodies]] entry"
                    .to_owned(),
        })?;
    let inertia_matrix =
        body.dry_inertia_body_kg_m2
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: "rate_loop_kind = \"lqr\" requires \
                   vehicle.assembly.bodies[0].dry_inertia_body_kg_m2 to be declared"
                    .to_owned(),
            })?;
    // Reject non-diagonal inertia: per-axis LQR depends on
    // axis-decoupled rotational dynamics, which only holds for
    // diagonal J in body axes.
    for (i, row) in inertia_matrix.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if i != j && *value != 0.0 {
                return Err(CliError::UnsupportedScenario {
                    what: format!(
                        "rate_loop_kind = \"lqr\" requires diagonal inertia; body[0] \
                         dry_inertia_body_kg_m2[{i}][{j}] = {value} ≠ 0"
                    ),
                });
            }
        }
    }
    Ok(Some(FcAutopilotLqrContext {
        dt_s: scenario.document.time.dt_s,
        diagonal_inertia_kg_m2: [
            inertia_matrix[0][0],
            inertia_matrix[1][1],
            inertia_matrix[2][2],
        ],
    }))
}

fn build_magnetic_field(config: &FcConfig) -> Result<Box<dyn MagneticFieldEci>, CliError> {
    let (kind, epoch) = magnetic_field_settings(config);
    match kind {
        FcMagFieldKind::EarthDipole => Ok(Box::new(EarthDipoleField::default())),
        FcMagFieldKind::Wmm2025 => {
            let model = Wmm2025::new_for_decimal_year(epoch).map_err(|err| {
                CliError::UnsupportedScenario {
                    what: format!("WMM 2025 magnetic model rejected epoch {epoch}: {err}"),
                }
            })?;
            Ok(Box::new(model))
        }
    }
}

fn magnetic_field_settings(config: &FcConfig) -> (FcMagFieldKind, f64) {
    match config.estimator {
        FcEstimatorKind::Ekf => {
            let cfg = config.ekf.as_ref();
            (
                cfg.and_then(|c| c.mag_field).unwrap_or_default(),
                cfg.and_then(|c| c.mag_epoch_decimal_year)
                    .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR),
            )
        }
        FcEstimatorKind::Mekf => {
            let cfg = config.mekf.as_ref();
            (
                cfg.and_then(|c| c.mag_field).unwrap_or_default(),
                cfg.and_then(|c| c.mag_epoch_decimal_year)
                    .unwrap_or(DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR),
            )
        }
    }
}

fn build_sensors(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Vec<BridgeSensor>, CliError> {
    let mut sensors = Vec::new();
    let Some(configs) = &document.sensors else {
        return Ok(sensors);
    };
    for (name, config) in configs {
        sensors.push(build_sensor(
            name,
            config,
            document.time.dt_s,
            resolved_files,
        )?);
    }
    Ok(sensors)
}

fn build_sensor(
    name: &str,
    config: &SensorConfig,
    dt_s: f64,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<BridgeSensor, CliError> {
    let id = SensorId::from_path(&format!("sensors.{name}"));
    match config.kind.as_str() {
        "ideal_state" => Ok(BridgeSensor::Ideal(SyntheticSensorAdapter::new(
            IdealStateSensor::new(id),
        ))),
        "imu" => {
            let budget = ImuNoiseBudget::load_from_str(sensor_text(name, resolved_files)?)?;
            let sensor = SyntheticImu::new(id, budget)?;
            Ok(BridgeSensor::Imu(SyntheticSensorAdapter::new(sensor)))
        }
        "barometer" => {
            let budget = parse_baro_budget(sensor_text(name, resolved_files)?, dt_s)?;
            let sensor = SyntheticBarometer::new(id, budget.0, budget.1, budget.2, dt_s)?;
            Ok(BridgeSensor::Barometer(SyntheticSensorAdapter::new(sensor)))
        }
        "gnss" => {
            let budget = parse_gnss_budget(sensor_text(name, resolved_files)?, dt_s)?;
            let sensor = SyntheticGnss::new(id, budget)?;
            Ok(BridgeSensor::Gnss(SyntheticSensorAdapter::new(sensor)))
        }
        "magnetometer" => {
            let budget = parse_magnetometer_budget(sensor_text(name, resolved_files)?)?;
            let sensor = SyntheticMagnetometer::new(id, budget);
            Ok(BridgeSensor::Magnetometer(SyntheticSensorAdapter::new(
                sensor,
            )))
        }
        "star_tracker" => {
            let budget = parse_star_tracker_budget(sensor_text(name, resolved_files)?)?;
            let sensor = SyntheticStarTracker::new(id, budget);
            Ok(BridgeSensor::StarTracker(SyntheticSensorAdapter::new(
                sensor,
            )))
        }
        other => Err(CliError::UnsupportedScenario {
            what: format!("unsupported FC bridge sensor kind `{other}`"),
        }),
    }
}

fn sensor_text<'a>(
    name: &str,
    resolved_files: &'a BTreeMap<String, ResolvedFile>,
) -> Result<&'a str, CliError> {
    let key = format!("sensors.{name}.file");
    let resolved = resolved_files
        .get(&key)
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: format!("sensor budget `{key}` was not resolved"),
        })?;
    std::str::from_utf8(&resolved.bytes).map_err(|err| CliError::UnsupportedScenario {
        what: format!("sensor budget `{key}` is not UTF-8: {err}"),
    })
}

fn parse_toml_budget(text: &str) -> Result<toml::Value, CliError> {
    // Phase-5.A.2.A: in `toml` 1.x `text.parse::<toml::Value>()`
    // expects a single TOML scalar / inline-table / array, not a
    // top-level document — leading comments + a `key = value` line
    // surface as "unexpected content, expected nothing". Parse as a
    // `Table` (the canonical document shape) and wrap it back into a
    // `Value` so the existing `as_table` consumers keep working.
    let table =
        toml::from_str::<toml::Table>(text).map_err(|err| CliError::UnsupportedScenario {
            what: format!("sensor budget TOML parse failed: {err}"),
        })?;
    Ok(toml::Value::Table(table))
}

fn toml_number_as_f64(value: &toml::Value) -> Option<f64> {
    value.as_float().or_else(|| {
        value
            .as_integer()
            .and_then(|integer| integer.to_string().parse::<f64>().ok())
    })
}

fn array3(table: &toml::value::Table, key: &str) -> Result<[f64; 3], CliError> {
    let value = table
        .get(key)
        .and_then(toml::Value::as_array)
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: format!("sensor budget missing array `{key}`"),
        })?;
    if value.len() != 3 {
        return Err(CliError::UnsupportedScenario {
            what: format!("sensor budget `{key}` must contain 3 values"),
        });
    }
    let mut out = [0.0; 3];
    for (slot, item) in out.iter_mut().zip(value) {
        *slot = toml_number_as_f64(item).ok_or_else(|| CliError::UnsupportedScenario {
            what: format!("sensor budget `{key}` contains a non-number"),
        })?;
    }
    Ok(out)
}

fn matrix3(table: &toml::value::Table, key: &str) -> Result<[[f64; 3]; 3], CliError> {
    let value = table
        .get(key)
        .and_then(toml::Value::as_array)
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: format!("sensor budget missing matrix `{key}`"),
        })?;
    if value.len() != 3 {
        return Err(CliError::UnsupportedScenario {
            what: format!("sensor budget `{key}` must contain 3 rows"),
        });
    }
    let mut out = [[0.0; 3]; 3];
    for (row, item) in out.iter_mut().zip(value) {
        let row_values = item
            .as_array()
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-array row"),
            })?;
        if row_values.len() != 3 {
            return Err(CliError::UnsupportedScenario {
                what: format!("sensor budget `{key}` rows must contain 3 values"),
            });
        }
        for (slot, item) in row.iter_mut().zip(row_values) {
            *slot = toml_number_as_f64(item).ok_or_else(|| CliError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-number"),
            })?;
        }
    }
    Ok(out)
}

fn budget_table(value: &toml::Value) -> Result<&toml::value::Table, CliError> {
    value
        .get("budget")
        .and_then(toml::Value::as_table)
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: "sensor budget missing [budget] table".to_owned(),
        })
}

fn parse_gnss_budget(text: &str, dt_s: f64) -> Result<GnssNoiseBudget, CliError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    GnssNoiseBudget::new(
        array3(table, "sigma_position_m")?,
        array3(table, "sigma_velocity_m_s")?,
        array3(table, "position_bias_ou_theta_per_s")?,
        array3(table, "position_bias_ou_sigma_m_sqrt_s")?,
        dt_s,
    )
    .map_err(|err| CliError::UnsupportedScenario {
        what: format!("GNSS budget rejected: {err}"),
    })
}

fn parse_magnetometer_budget(text: &str) -> Result<MagnetometerNoiseBudget, CliError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    MagnetometerNoiseBudget::new(
        array3(table, "sigma_body_nt")?,
        array3(table, "hard_iron_body_nt")?,
        matrix3(table, "soft_iron_body")?,
    )
    .map_err(|err| CliError::UnsupportedScenario {
        what: format!("magnetometer budget rejected: {err}"),
    })
}

fn parse_star_tracker_budget(text: &str) -> Result<StarTrackerNoiseBudget, CliError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let sigma_arcsec = table
        .get("sigma_per_axis_arcsec")
        .and_then(toml_number_as_f64)
        .ok_or_else(|| CliError::UnsupportedScenario {
            what: "star-tracker budget missing `sigma_per_axis_arcsec`".to_owned(),
        })?;
    StarTrackerNoiseBudget::from_arcsec(sigma_arcsec).map_err(|err| CliError::UnsupportedScenario {
        what: format!("star-tracker budget rejected: {err}"),
    })
}

fn parse_baro_budget(text: &str, dt_s: f64) -> Result<(f64, f64, f64), CliError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let get = |key: &str| -> Result<f64, CliError> {
        table
            .get(key)
            .and_then(toml_number_as_f64)
            .ok_or_else(|| CliError::UnsupportedScenario {
                what: format!("barometer budget missing `{key}`"),
            })
    };
    let measurement_stddev_pa = get("measurement_stddev_pa")?;
    let bias_theta = get("bias_ou_theta_per_s")?;
    let bias_sigma = get("bias_ou_sigma_pa_sqrt_s")?;
    if dt_s <= 0.0 {
        return Err(CliError::UnsupportedScenario {
            what: "barometer sensor dt must be positive".to_owned(),
        });
    }
    Ok((measurement_stddev_pa, bias_theta, bias_sigma))
}

impl From<openbmp_sensors::SensorError> for CliError {
    fn from(value: openbmp_sensors::SensorError) -> Self {
        Self::UnsupportedScenario {
            what: format!("sensor bridge error: {value}"),
        }
    }
}
