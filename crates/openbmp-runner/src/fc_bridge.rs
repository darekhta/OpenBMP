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
    BarometerSample, EnvironmentEstimate, GnssSample, ImuSample, MagnetometerSample,
    PropellantState, StarTrackerSample,
};
pub(crate) use openbmp_fc::topics::{GuidanceCutoff, ReferenceState};
use openbmp_mission::{FiredEvent, MissionAction};
use openbmp_physics::atmosphere::{AtmosphereModel, ExoatmosphericPolicy, UsStandard1976};
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

use crate::atmosphere::{
    RuntimeAtmosphere, atmosphere_altitude_m_with_surface_radius,
    build_document_runtime_atmosphere, document_geocentric_surface_radius_m,
    is_runtime_atmosphere_kind, scenario_atmosphere_kind,
};
use crate::error::RunnerError;
use crate::fc::{EstimatorSeed, FcAutopilotLqrContext, FcRunner, FcRunnerMission};

const DEFAULT_WMM_2025_EPOCH_DECIMAL_YEAR: f64 = 2025.0;

/// Optional FC bridge. Absent when the scenario has no `[fc]` block.
#[derive(Debug)]
pub struct FcBridge {
    runner: FcRunner,
    sensors: Vec<BridgeSensor>,
    atmosphere: Option<RuntimeAtmosphere>,
    fallback_atmosphere: UsStandard1976,
    geocentric_surface_radius_m: Option<f64>,
    magnetic: Box<dyn MagneticFieldEci>,
    scenario_seed: u64,
    previous_velocity_eci_m_s: Option<Vector3<f64>>,
    previous_time_s: Option<f64>,
    previous_angular_velocity_body_rad_s: Option<Vector3<f64>>,
    previous_angular_time_s: Option<f64>,
    last_mission_action_sequence: Option<u64>,
}

impl FcBridge {
    /// Build the bridge when `[fc]` is present; otherwise return
    /// `None` so legacy scenarios remain byte-stable.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the FC config, mission graph, frame
    /// profile, or synthetic sensor config cannot be resolved.
    pub fn maybe_new(
        scenario: &Scenario,
        resolved_files: &BTreeMap<String, ResolvedFile>,
    ) -> Result<Option<Self>, RunnerError> {
        let Some(fc_config) = &scenario.document.fc else {
            return Ok(None);
        };
        require_bridge_frame(&scenario.document)?;
        let Some(mission) = &scenario.document.mission else {
            return Err(RunnerError::UnsupportedScenario {
                what:
                    "[fc] requires a [mission] graph so the FC commander and kernel share phase ids"
                        .to_owned(),
            });
        };
        let mission_runtime = crate::mission::build_mission_runtime_typed(mission)?;
        let start_phase = mission_runtime.graph.initial;
        let fc_mission = FcRunnerMission::new(
            mission_runtime.graph,
            mission_runtime.hsm,
            mission_runtime.regions,
            mission_runtime.mission_bindings,
            start_phase,
        );
        let magnetic = build_magnetic_field(fc_config)?;
        let lqr_ctx = build_autopilot_lqr_context(scenario)?;
        let allocator = build_autopilot_allocator(scenario)?;
        // Seed the estimator from the scenario's known initial state so
        // the navigation filter starts near truth. Without this the EKF
        // seeds at the ECI origin and its first GNSS fix lands outside
        // the innovation gate (the vehicle is thousands of km away), so
        // it rejects every correction and dead-reckons from zero — any
        // guidance keyed off the position estimate is then wrong for the
        // whole flight.
        let v = &scenario.document.vehicle;
        let estimator_seed = EstimatorSeed {
            position_eci_m: Vector3::from(v.initial_position_eci_m),
            velocity_eci_m_s: Vector3::from(v.initial_velocity_eci_m_s),
            attitude_body_to_eci: v.initial_quaternion_body_to_eci_xyzw.map_or_else(
                UnitQuaternion::identity,
                |q| {
                    UnitQuaternion::from_quaternion(nalgebra::Quaternion::new(
                        q[3], q[0], q[1], q[2],
                    ))
                },
            ),
        };
        let runner = FcRunner::new(
            fc_config,
            fc_mission,
            lqr_ctx,
            scenario.document.time.dt_s,
            allocator,
            estimator_seed,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("flight-controller construction failed: {err}"),
        })?;
        let sensors = build_sensors(&scenario.document, resolved_files)?;
        let atmosphere_kind = scenario_atmosphere_kind(&scenario.document);
        let atmosphere = if is_runtime_atmosphere_kind(atmosphere_kind) {
            Some(build_document_runtime_atmosphere(&scenario.document)?)
        } else {
            None
        };
        Ok(Some(Self {
            runner,
            sensors,
            atmosphere,
            fallback_atmosphere: UsStandard1976::with_exoatmospheric_policy(
                ExoatmosphericPolicy::ZeroDensityAboveCeiling,
            ),
            geocentric_surface_radius_m: document_geocentric_surface_radius_m(&scenario.document),
            magnetic,
            scenario_seed: scenario.document.time.seed,
            previous_velocity_eci_m_s: None,
            previous_time_s: None,
            previous_angular_velocity_body_rad_s: None,
            previous_angular_time_s: None,
            last_mission_action_sequence: None,
        }))
    }

    /// Returns the FC commander's most recent
    /// mission-state publication, or `None` when no FC is wired or
    /// the commander has not yet ticked. The scenario runner reads
    /// this between FC and kernel ticks and forwards into
    /// `kernel.set_external_mission_state` so the kernel observes
    /// (rather than duplicates) the FC's mission-state ownership.
    #[must_use]
    pub fn latest_mission_state_id(&self) -> Option<u64> {
        self.runner
            .latest_mission_state()
            .map(|s| s.mission_state_id)
    }

    /// Drain FC-fired mission actions published since the previous
    /// bridge read.
    #[must_use]
    pub fn drain_mission_actions(&mut self) -> Vec<FiredEvent<MissionAction>> {
        let Some((batch, sequence)) = self.runner.latest_mission_action_batch() else {
            return Vec::new();
        };
        if self.last_mission_action_sequence == Some(sequence) {
            return Vec::new();
        }
        self.last_mission_action_sequence = Some(sequence);
        batch.events
    }

    /// Returns the most recent guidance reference published by the FC.
    #[must_use]
    pub fn latest_reference_state(&self) -> Option<ReferenceState> {
        self.runner.latest_reference_state()
    }

    /// Returns the most recent guidance cutoff estimate published by the FC.
    #[must_use]
    pub fn latest_guidance_cutoff(&self) -> Option<GuidanceCutoff> {
        self.runner.latest_guidance_cutoff()
    }

    /// Run one point-mass bridge tick and push FC commands into the
    /// runner-side racks.
    ///
    /// # Errors
    ///
    /// Propagates sensor, controller, or rack errors as [`RunnerError`].
    pub fn tick_point_mass(
        &mut self,
        state: &PointMassState,
        step: StepIndex,
        gravity_eci_m_s2: Vector3<f64>,
        propellant_state: Option<PropellantState>,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
    ) -> Result<(), RunnerError> {
        let truth = self.point_mass_truth(state, gravity_eci_m_s2);
        self.step_from_truth(truth, step, propellant_state, effectors, engines)
    }

    /// Run one rigid-body bridge tick and push FC commands into the
    /// runner-side racks.
    ///
    /// # Errors
    ///
    /// Propagates sensor, controller, or rack errors as [`RunnerError`].
    #[allow(clippy::too_many_arguments)]
    pub fn tick_rigid_body(
        &mut self,
        state: &RigidBodyState,
        step: StepIndex,
        gravity_eci_m_s2: Vector3<f64>,
        propellant_state: Option<PropellantState>,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
        gyro_pickup_rad_s: Vector3<f64>,
    ) -> Result<(), RunnerError> {
        let truth = self.rigid_body_truth(state, gravity_eci_m_s2, gyro_pickup_rad_s);
        self.step_from_truth(truth, step, propellant_state, effectors, engines)
    }

    fn step_from_truth(
        &mut self,
        truth: SensorTruth,
        step: StepIndex,
        propellant_state: Option<PropellantState>,
        effectors: &mut crate::effectors::EffectorRack,
        engines: &mut crate::engines::EngineRack,
    ) -> Result<(), RunnerError> {
        for index in 0..self.sensors.len() {
            let measurement = {
                let sensor = &mut self.sensors[index];
                sensor.prime(truth, step, self.scenario_seed);
                sensor.read()?
            };
            self.publish_measurement(&measurement);
        }
        self.runner.publish_environment(EnvironmentEstimate {
            time: truth.time,
            density_kg_m3: self
                .sample_atmosphere(truth.altitude_geometric_m, truth.time)
                .density_kg_m3,
        });
        if let Some(state) = propellant_state {
            self.runner.publish_propellant_state(state);
        }
        self.runner
            .step(truth.time, step)
            .map_err(|err| RunnerError::UnsupportedScenario {
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
            Vector3::zeros(),
            specific_force_eci,
            state.time,
        )
    }

    fn rigid_body_truth(
        &mut self,
        state: &RigidBodyState,
        gravity_eci_m_s2: Vector3<f64>,
        gyro_pickup_rad_s: Vector3<f64>,
    ) -> SensorTruth {
        let attitude_body_to_eci = state.orientation.q;
        let specific_force_eci = self.specific_force_eci(
            state.velocity.vector,
            state.time.as_seconds(),
            gravity_eci_m_s2,
        );
        // The rate gyro senses the rigid-body rate PLUS the local structural
        // bending slope rate (zero for a rigid vehicle). This is what lets the
        // autopilot — and its gyro notch — interact with the flex mode.
        let angular_velocity_body_rad_s = state.angular_velocity.vector + gyro_pickup_rad_s;
        let angular_acceleration_body_rad_s2 =
            self.angular_acceleration_body(angular_velocity_body_rad_s, state.time.as_seconds());
        self.truth_common(
            state.position,
            state.velocity,
            attitude_body_to_eci,
            angular_velocity_body_rad_s,
            angular_acceleration_body_rad_s2,
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
        angular_acceleration_body_rad_s2: Vector3<f64>,
        specific_force_eci_m_s2: Vector3<f64>,
        time: openbmp_core::SimTime,
    ) -> SensorTruth {
        let altitude_m =
            bridge_sensor_altitude_m(position.vector, self.geocentric_surface_radius_m);
        let atmosphere = self.sample_atmosphere(altitude_m, time);
        let static_pressure_pa = atmosphere.pressure_pa;
        let attitude_eci_to_body = attitude_body_to_eci.inverse();
        let specific_force_body_m_s2 = attitude_eci_to_body * specific_force_eci_m_s2;
        let magnetic_field_body_nt =
            attitude_eci_to_body * self.magnetic.field_eci_nt(position.vector, time);
        SensorTruth {
            position_eci: position,
            velocity_eci: velocity,
            attitude_eci_to_body,
            angular_velocity_body_rad_s,
            angular_acceleration_body_rad_s2,
            specific_force_body_m_s2,
            static_pressure_pa,
            altitude_geometric_m: altitude_m,
            magnetic_field_body_nt,
            time,
        }
    }

    fn sample_atmosphere(
        &self,
        altitude_m: f64,
        time: openbmp_core::SimTime,
    ) -> openbmp_physics::AtmosphereSample {
        let sample = self.atmosphere.as_ref().map_or_else(
            || self.fallback_atmosphere.sample(altitude_m, time),
            |atmosphere| atmosphere.sample(altitude_m, time),
        );
        sample.unwrap_or_else(|_| openbmp_physics::AtmosphereSample {
            pressure_pa: openbmp_physics::atmosphere::USSA76_SEA_LEVEL_PRESSURE_PA,
            ..openbmp_physics::AtmosphereSample::default()
        })
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

    fn angular_acceleration_body(
        &mut self,
        angular_velocity_body_rad_s: Vector3<f64>,
        time_s: f64,
    ) -> Vector3<f64> {
        let angular_accel = match (
            self.previous_angular_velocity_body_rad_s,
            self.previous_angular_time_s,
        ) {
            (Some(prev_omega), Some(prev_t)) if time_s > prev_t => {
                (angular_velocity_body_rad_s - prev_omega) / (time_s - prev_t)
            }
            _ => Vector3::zeros(),
        };
        self.previous_angular_velocity_body_rad_s = Some(angular_velocity_body_rad_s);
        self.previous_angular_time_s = Some(time_s);
        angular_accel
    }
}

pub(crate) fn propellant_state_from_tanks(
    time: openbmp_core::SimTime,
    tanks: &BTreeMap<openbmp_core::TankId, openbmp_vehicle::PropellantTankState>,
) -> Option<PropellantState> {
    if tanks.is_empty() {
        return None;
    }
    let mut initial_kg = 0.0;
    let mut remaining_kg = 0.0;
    for tank in tanks.values() {
        if !tank.initial_fluid_mass_kg.is_finite() || !tank.fluid_remaining_kg.is_finite() {
            return None;
        }
        initial_kg += tank.initial_fluid_mass_kg.max(0.0);
        remaining_kg += tank.fluid_remaining_kg.max(0.0);
    }
    if !initial_kg.is_finite() || initial_kg <= 0.0 {
        return None;
    }
    let remaining_kg = remaining_kg.min(initial_kg);
    let mass_fraction = (remaining_kg / initial_kg).clamp(0.0, 1.0);
    Some(PropellantState {
        time,
        mass_fraction,
        mass_remaining_kg: remaining_kg,
        mass_initial_kg: initial_kg,
        depleted: remaining_kg <= f64::EPSILON * initial_kg.max(1.0),
    })
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

    fn read(&mut self) -> Result<openbmp_sensors::Timestamped<SensorMeasurement>, RunnerError> {
        match self {
            Self::Imu(s) => s.read(),
            Self::Barometer(s) => s.read(),
            Self::Gnss(s) => s.read(),
            Self::Magnetometer(s) => s.read(),
            Self::StarTracker(s) => s.read(),
            Self::Ideal(s) => s.read(),
        }
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("synthetic sensor read failed: {err}"),
        })
    }
}

fn require_bridge_frame(document: &ScenarioDocument) -> Result<(), RunnerError> {
    let has_barometer = document
        .sensors
        .as_ref()
        .is_some_and(|sensors| sensors.values().any(|s| s.kind == "barometer"));
    bridge_frame_supported(
        &document.environment.frame_profile,
        &document.environment.atmosphere,
        has_barometer,
    )
    .map_err(|what| RunnerError::UnsupportedScenario { what })
}

fn bridge_sensor_altitude_m(
    position_eci_m: Vector3<f64>,
    geocentric_surface_radius_m: Option<f64>,
) -> f64 {
    atmosphere_altitude_m_with_surface_radius(position_eci_m, geocentric_surface_radius_m)
}

/// Pure decision for whether the FC sensor bridge supports a given
/// `(frame_profile, atmosphere, has_barometer)` configuration. Extracted
/// from [`require_bridge_frame`] so the policy is unit-testable without
/// constructing a full [`ScenarioDocument`].
///
/// `wgs84-uniform-rotation` differs from `toy-fixed-earth` only by a
/// rotation of the ECEF frame about the inertial +z axis. The kernel
/// integrates the dynamics in ECI and the bridge derives sensor truth
/// entirely in ECI: the IMU specific force is `d/dt v_eci - g_eci` (so it
/// senses atmospheric drag correctly through the truth velocity), GNSS and
/// magnetometer truth are ECI quantities, and barometric pressure now uses
/// the same frame-aware altitude helper as the kernel atmosphere path.
///
/// `iers-tabulated` is treated identically: it differs from
/// `wgs84-uniform-rotation` only in the ECI↔ECEF transform (IAU precession +
/// nutation, tabulated UT1-UTC / polar motion / LOD), which the KERNEL
/// applies to the dynamics (atmospheric co-rotation, gravity). The bridge
/// sensor truth is derived in ECI and so is identical under either rotating
/// frame.
fn bridge_frame_supported(
    frame_profile: &str,
    _atmosphere: &str,
    _has_barometer: bool,
) -> Result<(), String> {
    match frame_profile {
        "toy-fixed-earth" => Ok(()),
        "wgs84-uniform-rotation" | "iers-tabulated" => Ok(()),
        other => Err(format!(
            "[fc] bridge requires frame_profile = \"toy-fixed-earth\", \
             \"wgs84-uniform-rotation\", or \"iers-tabulated\"; got `{other}`"
        )),
    }
}

/// Precondition: per-axis rate loops (LQR
/// and INDI) require a single-body assembly with diagonal inertia
/// in body axes. Multi-body assemblies fail closed because solving
/// gains / parameters against the full assembled mass
/// properties is not supported; non-diagonal inertia breaks the per-axis decoupling
/// assumption both rate loops are built on.
///
/// Returns `Ok([Jxx, Jyy, Jzz])` after passing the precondition.
/// Callers only invoke this helper after a per-axis rate loop has
/// been selected. Fails closed when the precondition is violated;
/// the error includes the offending rate-loop kind label.
fn verify_per_axis_rate_loop_preconditions(
    scenario: &Scenario,
    kind_label: &str,
) -> Result<[f64; 3], RunnerError> {
    let bodies = &scenario.document.vehicle.assembly.bodies;
    if bodies.len() != 1 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "rate_loop_kind = \"{kind_label}\" requires exactly one \
                 [[vehicle.assembly.bodies]] entry with diagonal inertia; \
                 got {} bodies",
                bodies.len()
            ),
        });
    }
    let body = bodies
        .first()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "rate_loop_kind = \"{kind_label}\" requires at least one \
                 [[vehicle.assembly.bodies]] entry"
            ),
        })?;
    let inertia_matrix =
        body.dry_inertia_body_kg_m2
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "rate_loop_kind = \"{kind_label}\" requires \
                     vehicle.assembly.bodies[0].dry_inertia_body_kg_m2 to be declared"
                ),
            })?;
    // Reject non-diagonal inertia: per-axis decoupling only holds
    // for diagonal J in body axes.
    for (i, row) in inertia_matrix.iter().enumerate() {
        for (j, value) in row.iter().enumerate() {
            if i != j && *value != 0.0 {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "rate_loop_kind = \"{kind_label}\" requires diagonal inertia; body[0] \
                         dry_inertia_body_kg_m2[{i}][{j}] = {value} ≠ 0"
                    ),
                });
            }
        }
    }
    Ok([
        inertia_matrix[0][0],
        inertia_matrix[1][1],
        inertia_matrix[2][2],
    ])
}

/// Helper to extract the diagonal moments of inertia
/// from a single-body assembly so the runner can solve the per-axis
/// LQR DARE at scenario load. Returns `Ok(None)` when the FC
/// scenario does not request a rate loop that needs the precondition
/// check. Fails closed when LQR is requested but the precondition
/// (single body, diagonal inertia) is violated; the matching INDI
/// precondition runs through [`verify_per_axis_rate_loop_preconditions`]
/// for its fail-closed side effect.
fn build_autopilot_lqr_context(
    scenario: &Scenario,
) -> Result<Option<FcAutopilotLqrContext>, RunnerError> {
    let Some(fc_config) = &scenario.document.fc else {
        return Ok(None);
    };
    let Some(autopilot_params) = fc_config.autopilot_params.as_ref() else {
        return Ok(None);
    };
    let kind = autopilot_params.rate_loop_kind;
    if kind == Some(openbmp_scenario::FcRateLoopKind::Lqr) {
        let diagonal_inertia_kg_m2 = verify_per_axis_rate_loop_preconditions(scenario, "lqr")?;
        return Ok(Some(FcAutopilotLqrContext {
            dt_s: scenario.document.time.dt_s,
            diagonal_inertia_kg_m2,
        }));
    }
    if kind == Some(openbmp_scenario::FcRateLoopKind::Indi) {
        // INDI shares the precondition (single body, diagonal
        // inertia) but uses scenario-config inertia for its
        // inversion, not the truth-side body inertia. Run the
        // check for its side-effect; LQR-context is None.
        let _ = verify_per_axis_rate_loop_preconditions(scenario, "indi")?;
        return Ok(None);
    }
    Ok(None)
}

/// Helper to derive a `PrioritisedRedistributedAllocator`
/// from `[fc.autopilot_allocation]` plus the
/// `[[vehicle.assembly.effectors]]` declarations.
///
/// Returns `Ok(None)` when the FC config has no allocation block, or
/// when the configured kind is not yet wired in this slice
/// (`pseudo_inverse` is parsed but consumed by a future slice; for
/// now its presence triggers fail-closed). For the consumed
/// `prioritised_redistributed` kind, walks every `direct_torque`
/// effector in the assembly and groups them by axis. The optional
/// scenario `axis_priority` is honoured in priority order; absent →
/// the documented default `[roll, yaw, pitch]`.
fn build_autopilot_allocator(
    scenario: &Scenario,
) -> Result<Option<openbmp_fc::allocation::PrioritisedRedistributedAllocator>, RunnerError> {
    use openbmp_fc::allocation::{BodyAxis, EffectorAxisAssignment};
    let Some(fc_config) = &scenario.document.fc else {
        return Ok(None);
    };
    let Some(alloc_cfg) = fc_config.autopilot_allocation.as_ref() else {
        return Ok(None);
    };
    match alloc_cfg.kind {
        openbmp_scenario::FcAutopilotAllocationKind::PrioritisedRedistributed => {}
        openbmp_scenario::FcAutopilotAllocationKind::PseudoInverse => {
            return Err(RunnerError::UnsupportedScenario {
                what: "fc.autopilot_allocation.kind = \"pseudo_inverse\" is parsed but not yet \
                       consumed; use \"prioritised_redistributed\" or remove the \
                       block until the pseudo-inverse path lands"
                    .to_owned(),
            });
        }
    }
    // Walk effectors and pull out direct_torque assignments.
    let mut assignments: Vec<EffectorAxisAssignment> = Vec::new();
    for effector in &scenario.document.vehicle.assembly.effectors {
        let openbmp_scenario::EffectorKindConfig::DirectTorque { axis, .. } = effector.kind else {
            continue;
        };
        let body_axis = match axis {
            openbmp_scenario::TorqueAxis::Roll => BodyAxis::Roll,
            openbmp_scenario::TorqueAxis::Pitch => BodyAxis::Pitch,
            openbmp_scenario::TorqueAxis::Yaw => BodyAxis::Yaw,
        };
        // Symmetric box check: the allocator consumes one positive
        // capacity per effector, so asymmetric authority must fail
        // closed instead of being hidden by a tolerance.
        if !limits_are_exactly_symmetric(effector.limits.min, effector.limits.max) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "fc.autopilot_allocation = \"prioritised_redistributed\" requires symmetric \
                     effector limits; effector \"{}\" has min = {}, max = {}",
                    effector.id, effector.limits.min, effector.limits.max
                ),
            });
        }
        assignments.push(EffectorAxisAssignment {
            effector_id: openbmp_core::EffectorId::from_path(&format!(
                "vehicle.assembly.effectors.{}",
                effector.id
            )),
            axis: body_axis,
            max_abs: effector.limits.max,
        });
    }
    if assignments.is_empty() {
        return Err(RunnerError::UnsupportedScenario {
            what: "fc.autopilot_allocation = \"prioritised_redistributed\" requires at least one \
                   direct_torque effector in vehicle.assembly.effectors"
                .to_owned(),
        });
    }
    // Resolve axis priority. Default per the scenario block:
    // [roll, yaw, pitch]. Any axis named in `axis_priority` must
    // appear; absent → fall back to the default permutation.
    let priority = if let Some(priority_strs) = alloc_cfg.axis_priority.as_ref() {
        let mut axes = [BodyAxis::Roll, BodyAxis::Yaw, BodyAxis::Pitch];
        // The scenario validator already requires unique entries
        // drawn from {roll, pitch, yaw}; we still defend in depth.
        if priority_strs.len() != 3 {
            return Err(RunnerError::UnsupportedScenario {
                what: "fc.autopilot_allocation.axis_priority must list each of \
                       [roll, pitch, yaw] exactly once"
                    .to_owned(),
            });
        }
        for (slot, label) in axes.iter_mut().zip(priority_strs.iter()) {
            *slot = match label.as_str() {
                "roll" => BodyAxis::Roll,
                "pitch" => BodyAxis::Pitch,
                "yaw" => BodyAxis::Yaw,
                other => {
                    return Err(RunnerError::UnsupportedScenario {
                        what: format!(
                            "fc.autopilot_allocation.axis_priority entry \"{other}\" is not a \
                             body axis"
                        ),
                    });
                }
            };
        }
        axes
    } else {
        [BodyAxis::Roll, BodyAxis::Yaw, BodyAxis::Pitch]
    };
    let allocator =
        openbmp_fc::allocation::PrioritisedRedistributedAllocator::new(priority, assignments)
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!("control allocator construction failed: {err}"),
            })?;
    Ok(Some(allocator))
}

fn limits_are_exactly_symmetric(min: f64, max: f64) -> bool {
    max.to_bits() == (-min).to_bits()
}

fn build_magnetic_field(config: &FcConfig) -> Result<Box<dyn MagneticFieldEci>, RunnerError> {
    let (kind, epoch) = magnetic_field_settings(config);
    match kind {
        FcMagFieldKind::EarthDipole => Ok(Box::new(EarthDipoleField::default())),
        FcMagFieldKind::Wmm2025 => {
            let model = Wmm2025::new_for_decimal_year(epoch).map_err(|err| {
                RunnerError::UnsupportedScenario {
                    what: format!("WMM 2025 magnetic model rejected epoch {epoch}: {err}"),
                }
            })?;
            Ok(Box::new(model))
        }
    }
}

fn magnetic_field_settings(config: &FcConfig) -> (FcMagFieldKind, f64) {
    if let Some(lanes) = config.estimator_lanes.as_ref()
        && let Some(first_lane) = lanes.lanes.first()
    {
        // `[fc.estimator_lanes]` replaces the top-level estimator
        // selector. The synthetic magnetometer bridge follows the
        // same declaration-order contract as the lane runner and uses
        // the first lane's estimator family to choose the shared truth
        // field model.
        return magnetic_field_settings_for_estimator(config, first_lane.estimator);
    }
    magnetic_field_settings_for_estimator(config, config.estimator)
}

fn magnetic_field_settings_for_estimator(
    config: &FcConfig,
    estimator: FcEstimatorKind,
) -> (FcMagFieldKind, f64) {
    match estimator {
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
        FcEstimatorKind::Imm | FcEstimatorKind::SrUkf | FcEstimatorKind::SrUkfAttitude => {
            // IMM and SR-UKF (full and attitude
            // variants) all use the [fc.ekf] base for the magnetic-field
            // model; per-mode / per-lane overrides do not alter the
            // field-evaluation reference.
            let cfg = config.ekf.as_ref();
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
) -> Result<Vec<BridgeSensor>, RunnerError> {
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
) -> Result<BridgeSensor, RunnerError> {
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
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("unsupported FC bridge sensor kind `{other}`"),
        }),
    }
}

fn sensor_text<'a>(
    name: &str,
    resolved_files: &'a BTreeMap<String, ResolvedFile>,
) -> Result<&'a str, RunnerError> {
    let key = format!("sensors.{name}.file");
    let resolved = resolved_files
        .get(&key)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` was not resolved"),
        })?;
    std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("sensor budget `{key}` is not UTF-8: {err}"),
    })
}

fn parse_toml_budget(text: &str) -> Result<toml::Value, RunnerError> {
    // In `toml` 1.x `text.parse::<toml::Value>()`
    // expects a single TOML scalar / inline-table / array, not a
    // top-level document — leading comments + a `key = value` line
    // surface as "unexpected content, expected nothing". Parse as a
    // `Table` (the canonical document shape) and wrap it back into a
    // `Value` so the existing `as_table` consumers keep working.
    let table =
        toml::from_str::<toml::Table>(text).map_err(|err| RunnerError::UnsupportedScenario {
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

fn array3(table: &toml::value::Table, key: &str) -> Result<[f64; 3], RunnerError> {
    let value = table
        .get(key)
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget missing array `{key}`"),
        })?;
    if value.len() != 3 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` must contain 3 values"),
        });
    }
    let mut out = [0.0; 3];
    for (slot, item) in out.iter_mut().zip(value) {
        *slot = toml_number_as_f64(item).ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` contains a non-number"),
        })?;
    }
    Ok(out)
}

fn matrix3(table: &toml::value::Table, key: &str) -> Result<[[f64; 3]; 3], RunnerError> {
    let value = table
        .get(key)
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("sensor budget missing matrix `{key}`"),
        })?;
    if value.len() != 3 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("sensor budget `{key}` must contain 3 rows"),
        });
    }
    let mut out = [[0.0; 3]; 3];
    for (row, item) in out.iter_mut().zip(value) {
        let row_values = item
            .as_array()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-array row"),
            })?;
        if row_values.len() != 3 {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` rows must contain 3 values"),
            });
        }
        for (slot, item) in row.iter_mut().zip(row_values) {
            *slot = toml_number_as_f64(item).ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("sensor budget `{key}` contains a non-number"),
            })?;
        }
    }
    Ok(out)
}

fn budget_table(value: &toml::Value) -> Result<&toml::value::Table, RunnerError> {
    value
        .get("budget")
        .and_then(toml::Value::as_table)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "sensor budget missing [budget] table".to_owned(),
        })
}

fn parse_gnss_budget(text: &str, dt_s: f64) -> Result<GnssNoiseBudget, RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    GnssNoiseBudget::new(
        array3(table, "sigma_position_m")?,
        array3(table, "sigma_velocity_m_s")?,
        array3(table, "position_bias_ou_theta_per_s")?,
        array3(table, "position_bias_ou_sigma_m_sqrt_s")?,
        dt_s,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("GNSS budget rejected: {err}"),
    })
}

fn parse_magnetometer_budget(text: &str) -> Result<MagnetometerNoiseBudget, RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    MagnetometerNoiseBudget::new(
        array3(table, "sigma_body_nt")?,
        array3(table, "hard_iron_body_nt")?,
        matrix3(table, "soft_iron_body")?,
    )
    .map_err(|err| RunnerError::UnsupportedScenario {
        what: format!("magnetometer budget rejected: {err}"),
    })
}

fn parse_star_tracker_budget(text: &str) -> Result<StarTrackerNoiseBudget, RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let sigma_arcsec = table
        .get("sigma_per_axis_arcsec")
        .and_then(toml_number_as_f64)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "star-tracker budget missing `sigma_per_axis_arcsec`".to_owned(),
        })?;
    StarTrackerNoiseBudget::from_arcsec(sigma_arcsec).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("star-tracker budget rejected: {err}"),
        }
    })
}

fn parse_baro_budget(text: &str, dt_s: f64) -> Result<(f64, f64, f64), RunnerError> {
    let value = parse_toml_budget(text)?;
    let table = budget_table(&value)?;
    let get = |key: &str| -> Result<f64, RunnerError> {
        table.get(key).and_then(toml_number_as_f64).ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: format!("barometer budget missing `{key}`"),
            }
        })
    };
    let measurement_stddev_pa = get("measurement_stddev_pa")?;
    let bias_theta = get("bias_ou_theta_per_s")?;
    let bias_sigma = get("bias_ou_sigma_pa_sqrt_s")?;
    if dt_s <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "barometer sensor dt must be positive".to_owned(),
        });
    }
    Ok((measurement_stddev_pa, bias_theta, bias_sigma))
}

impl From<openbmp_sensors::SensorError> for RunnerError {
    fn from(value: openbmp_sensors::SensorError) -> Self {
        Self::UnsupportedScenario {
            what: format!("sensor bridge error: {value}"),
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;

    const ALLOCATOR_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/diff-flatness-figure-eight-allocator/scenario.toml"
    ));
    const FC_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/closed-loop-attitude-hold/scenario.toml"
    ));

    #[test]
    fn allocator_builder_rejects_even_tiny_asymmetric_limits() {
        let toml = ALLOCATOR_SCENARIO.replacen(
            "limits           = { min = -0.2, max = 0.2",
            "limits           = { min = -0.2000000000001, max = 0.2",
            1,
        );
        let scenario = Scenario::from_toml_str(&toml).expect("scenario parses");
        let err = build_autopilot_allocator(&scenario).expect_err("asymmetric limits rejected");
        assert!(
            matches!(err, RunnerError::UnsupportedScenario { ref what } if what.contains("requires symmetric effector limits")),
            "expected symmetric-limit UnsupportedScenario, got {err:?}"
        );
    }

    #[test]
    fn magnetic_field_settings_follow_first_estimator_lane() {
        let lanes = r#"
[fc.estimator_lanes]
voter = "simplex_pass_through"

[[fc.estimator_lanes.lane]]
id        = "ekf_lane"
estimator = "ekf"
"#;
        let toml = format!(
            "{}\n{lanes}",
            FC_SCENARIO
                .replace("openbmp.scenario = 2", "openbmp.scenario = 3")
                .replace("estimator        = \"ekf\"", "estimator        = \"mekf\"")
        );
        let scenario = Scenario::from_toml_str(&toml).expect("lane scenario validates");
        let fc = scenario.document.fc.as_ref().expect("fc block present");
        let (kind, epoch) = magnetic_field_settings(fc);
        assert_eq!(kind, FcMagFieldKind::Wmm2025);
        assert_eq!(epoch.to_bits(), 2025.0f64.to_bits());
    }

    #[test]
    fn bridge_sensor_altitude_is_frame_aware() {
        // Local-frame launch (near origin): preserve the legacy flat-earth
        // altitude convention.
        assert_eq!(
            bridge_sensor_altitude_m(Vector3::new(0.0, 0.0, 1_000.0), None),
            1_000.0
        );

        // Geocentric launch: use radius above the Earth model, not ECI z.
        let surface = Vector3::new(6_371_000.0, 0.0, 0.0);
        assert_eq!(bridge_sensor_altitude_m(surface, None), 0.0);
        let up_100km = Vector3::new(6_371_000.0 + 100_000.0, 0.0, 0.0);
        assert!((bridge_sensor_altitude_m(up_100km, None) - 100_000.0).abs() < 1.0e-6);

        let rounded_surface = Vector3::new(6_370_000.0, 0.0, 0.0);
        assert_eq!(
            bridge_sensor_altitude_m(rounded_surface, Some(6_370_000.0)),
            0.0
        );
        let rounded_up_1km = Vector3::new(6_371_000.0, 0.0, 0.0);
        assert!(
            (bridge_sensor_altitude_m(rounded_up_1km, Some(6_370_000.0)) - 1_000.0).abs() < 1.0e-6
        );
    }

    #[test]
    fn propellant_state_from_tanks_reports_remaining_fraction() {
        let tanks = BTreeMap::from([
            (
                openbmp_core::TankId::from_path("vehicle.assembly.tanks.fuel"),
                openbmp_vehicle::PropellantTankState {
                    fluid_remaining_kg: 30.0,
                    initial_fluid_mass_kg: 80.0,
                    volume_m3: 1.0,
                    density_kg_m3: 800.0,
                    has_ullage: false,
                    ullage_gamma: 1.4,
                },
            ),
            (
                openbmp_core::TankId::from_path("vehicle.assembly.tanks.oxidizer"),
                openbmp_vehicle::PropellantTankState {
                    fluid_remaining_kg: 20.0,
                    initial_fluid_mass_kg: 120.0,
                    volume_m3: 1.0,
                    density_kg_m3: 1_000.0,
                    has_ullage: false,
                    ullage_gamma: 1.4,
                },
            ),
        ]);

        let state = propellant_state_from_tanks(openbmp_core::SimTime::from_seconds(3.0), &tanks)
            .expect("propellant state");

        assert_eq!(state.time, openbmp_core::SimTime::from_seconds(3.0));
        assert_eq!(state.mass_remaining_kg.to_bits(), 50.0_f64.to_bits());
        assert_eq!(state.mass_initial_kg.to_bits(), 200.0_f64.to_bits());
        assert_eq!(state.mass_fraction.to_bits(), 0.25_f64.to_bits());
        assert!(!state.depleted);
    }

    #[test]
    fn bridge_frame_policy_allows_rotating_frame_with_barometer() {
        // toy-fixed-earth is always supported, with or without a baro.
        assert!(bridge_frame_supported("toy-fixed-earth", "none", false).is_ok());
        assert!(bridge_frame_supported("toy-fixed-earth", "us_standard_1976", true).is_ok());

        // wgs84-uniform-rotation in vacuum is fine regardless of baro.
        assert!(bridge_frame_supported("wgs84-uniform-rotation", "none", false).is_ok());
        assert!(bridge_frame_supported("wgs84-uniform-rotation", "none", true).is_ok());

        // wgs84-uniform-rotation + atmosphere is supported with and without
        // a barometer because bridge sensor truth now uses frame-aware
        // altitude for pressure.
        assert!(
            bridge_frame_supported("wgs84-uniform-rotation", "us_standard_1976", false).is_ok()
        );
        assert!(bridge_frame_supported("wgs84-uniform-rotation", "us_standard_1976", true).is_ok());

        // iers-tabulated shares the rotating-Earth bridge contract: the
        // sensor truth is frame-invariant in ECI, so it is accepted under
        // the same conditions as wgs84-uniform-rotation.
        assert!(bridge_frame_supported("iers-tabulated", "none", true).is_ok());
        assert!(bridge_frame_supported("iers-tabulated", "us_standard_1976", false).is_ok());
        assert!(bridge_frame_supported("iers-tabulated", "us_standard_1976", true).is_ok());

        // A genuinely unsupported frame is still rejected.
        let err = bridge_frame_supported("spice-reference", "none", false)
            .expect_err("spice-reference is rejected");
        assert!(err.contains("toy-fixed-earth"), "unexpected message: {err}");
    }
}
