//! Runner-side terminal landing throttle control for separated lanes.
//!
//! This is a deterministic scenario-director controller. It is not
//! flight software and does not claim propulsive-landing fidelity. The
//! controller lets SIL scenarios replace a terminal open-loop burn with
//! a closed-loop throttle command keyed to the propagated separated
//! body's altitude and radial velocity.

use nalgebra::Vector3;
use openbmp_core::{BodyId, EngineId, SimTime};
use openbmp_fc::topics::{EngineCommand, EngineCommandSet, MAX_ENGINE_COMMANDS};
use openbmp_scenario::{MultiBodyLandingControllerConfig, ScenarioDocument};
use openbmp_sim::SeparatedRigidBody;

use crate::error::RunnerError;

/// Collection of separated-body terminal landing controllers.
#[derive(Clone, Debug, Default)]
pub struct SeparatedLandingControllers {
    controllers: Vec<SeparatedLandingController>,
}

impl SeparatedLandingControllers {
    /// Build from `[multi_body]` config. Empty when no landing
    /// controllers are declared.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(multi_body) = &document.multi_body else {
            return Ok(Self::default());
        };
        let mut controllers = Vec::with_capacity(multi_body.landing_controllers.len());
        for config in &multi_body.landing_controllers {
            controllers.push(SeparatedLandingController::from_config(config, document)?);
        }
        Ok(Self { controllers })
    }

    /// `true` when no terminal landing controllers are configured.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.controllers.is_empty()
    }

    /// Evaluate every active terminal controller and apply one-tick
    /// engine commands.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when a controller cannot compute altitude
    /// or emits an invalid engine command.
    pub fn apply(
        &self,
        separated_bodies: &[SeparatedRigidBody],
        time: SimTime,
        ground_radius_m: Option<f64>,
        engines: &mut crate::engines::EngineRack,
    ) -> Result<(), RunnerError> {
        if self.controllers.is_empty() {
            return Ok(());
        }
        let ground_radius_m = ground_radius_m.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "multi_body.landing_controller requires a near-surface geocentric initial radius"
                .to_owned(),
        })?;
        let mut command_set = EngineCommandSet {
            time,
            ..EngineCommandSet::default()
        };
        for controller in &self.controllers {
            let Some(body) = separated_bodies
                .iter()
                .find(|body| body.body == controller.body && body.propagating)
            else {
                continue;
            };
            controller.append_command(body, ground_radius_m, &mut command_set)?;
        }
        if command_set.count > 0 {
            engines.apply_fc_commands(&command_set)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct SeparatedLandingController {
    body: BodyId,
    engine: EngineId,
    max_thrust_n: f64,
    start_altitude_m: f64,
    target_altitude_m: f64,
    target_vertical_speed_m_s: f64,
    target_position_eci_m: Option<Vector3<f64>>,
    lateral_kp_s2: f64,
    lateral_kd_s: f64,
    max_lateral_accel_m_s2: f64,
    gravity_margin_m_s2: f64,
    min_throttle_unit: f64,
    max_throttle_unit: f64,
}

impl SeparatedLandingController {
    fn from_config(
        config: &MultiBodyLandingControllerConfig,
        document: &ScenarioDocument,
    ) -> Result<Self, RunnerError> {
        let engine = document
            .vehicle
            .assembly
            .engines
            .iter()
            .find(|engine| engine.id == config.engine_id)
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "multi_body.landing_controller engine `{}` was not found",
                    config.engine_id
                ),
            })?;
        Ok(Self {
            body: BodyId::from_path(&format!("vehicle.assembly.bodies.{}", config.body_id)),
            engine: EngineId::from_path(&format!("vehicle.assembly.engines.{}", config.engine_id)),
            max_thrust_n: engine.limits.max_thrust_n,
            start_altitude_m: config.start_altitude_m,
            target_altitude_m: config.target_altitude_m,
            target_vertical_speed_m_s: config.target_vertical_speed_m_s,
            target_position_eci_m: config
                .target_position_eci_m
                .map(|v| Vector3::new(v[0], v[1], v[2])),
            lateral_kp_s2: config.lateral_kp_s2,
            lateral_kd_s: config.lateral_kd_s,
            max_lateral_accel_m_s2: config.max_lateral_accel_m_s2,
            gravity_margin_m_s2: config.gravity_margin_m_s2,
            min_throttle_unit: config.min_throttle_unit,
            max_throttle_unit: config.max_throttle_unit,
        })
    }

    fn append_command(
        &self,
        body: &SeparatedRigidBody,
        ground_radius_m: f64,
        command_set: &mut EngineCommandSet,
    ) -> Result<(), RunnerError> {
        let radius_m = body.state.position.vector.norm();
        if !radius_m.is_finite() || radius_m <= 0.0 {
            return Err(RunnerError::UnsupportedScenario {
                what: "multi_body.landing_controller requires finite non-zero body position"
                    .to_owned(),
            });
        }
        let altitude_m = radius_m - ground_radius_m;
        if altitude_m > self.start_altitude_m {
            return Ok(());
        }
        if altitude_m <= self.target_altitude_m {
            return push_engine_command(command_set, self.engine, 0.0, 0.0, 0.0, false, true);
        }

        let radial_eci = body.state.position.vector / radius_m;
        let radial_velocity_m_s = body.state.velocity.vector.dot(&radial_eci);
        let mass_kg = body.state.mass_props.mass_kg();
        let max_accel_m_s2 = self.max_thrust_n / mass_kg;
        if !max_accel_m_s2.is_finite() || max_accel_m_s2 <= 0.0 {
            return Err(RunnerError::UnsupportedScenario {
                what: "multi_body.landing_controller requires positive finite thrust acceleration"
                    .to_owned(),
            });
        }
        let remaining_altitude_m = (altitude_m - self.target_altitude_m).max(1.0e-3);
        let braking_accel_m_s2 = ((radial_velocity_m_s * radial_velocity_m_s
            - self.target_vertical_speed_m_s * self.target_vertical_speed_m_s)
            / (2.0 * remaining_altitude_m))
            .max(0.0);
        let vertical_accel_m_s2 = braking_accel_m_s2 + self.gravity_margin_m_s2;
        let lateral_accel_eci = self.lateral_accel_eci(body, radial_eci, radial_velocity_m_s)?;
        let desired_accel_eci = radial_eci * vertical_accel_m_s2 + lateral_accel_eci;
        let desired_accel_m_s2 = desired_accel_eci.norm();
        if !desired_accel_m_s2.is_finite() || desired_accel_m_s2 <= 0.0 {
            return Err(RunnerError::UnsupportedScenario {
                what: "multi_body.landing_controller produced invalid desired acceleration"
                    .to_owned(),
            });
        }
        let throttle = (desired_accel_m_s2 / max_accel_m_s2)
            .clamp(self.min_throttle_unit, self.max_throttle_unit);
        let (gimbal_pitch_rad, gimbal_yaw_rad) = if self.target_position_eci_m.is_some() {
            gimbal_for_desired_accel_body(body, desired_accel_eci)?
        } else {
            (0.0, 0.0)
        };
        push_engine_command(
            command_set,
            self.engine,
            throttle,
            gimbal_pitch_rad,
            gimbal_yaw_rad,
            true,
            false,
        )
    }

    fn lateral_accel_eci(
        &self,
        body: &SeparatedRigidBody,
        radial_eci: Vector3<f64>,
        radial_velocity_m_s: f64,
    ) -> Result<Vector3<f64>, RunnerError> {
        let Some(target_position_eci_m) = self.target_position_eci_m else {
            return Ok(Vector3::zeros());
        };
        let to_target_eci = target_position_eci_m - body.state.position.vector;
        let lateral_error_eci = to_target_eci - radial_eci * to_target_eci.dot(&radial_eci);
        let lateral_velocity_eci = body.state.velocity.vector - radial_eci * radial_velocity_m_s;
        let raw_accel_eci =
            lateral_error_eci * self.lateral_kp_s2 - lateral_velocity_eci * self.lateral_kd_s;
        let tangent_accel_eci = raw_accel_eci - radial_eci * raw_accel_eci.dot(&radial_eci);
        let norm = tangent_accel_eci.norm();
        if !norm.is_finite() {
            return Err(RunnerError::UnsupportedScenario {
                what: "multi_body.landing_controller produced non-finite lateral acceleration"
                    .to_owned(),
            });
        }
        if norm <= self.max_lateral_accel_m_s2 || norm <= f64::EPSILON {
            Ok(tangent_accel_eci)
        } else {
            Ok(tangent_accel_eci * (self.max_lateral_accel_m_s2 / norm))
        }
    }
}

fn gimbal_for_desired_accel_body(
    body: &SeparatedRigidBody,
    desired_accel_eci: Vector3<f64>,
) -> Result<(f64, f64), RunnerError> {
    let norm = desired_accel_eci.norm();
    if !norm.is_finite() || norm <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "multi_body.landing_controller cannot gimbal toward invalid acceleration"
                .to_owned(),
        });
    }
    let direction_eci = desired_accel_eci / norm;
    let direction_body = body.state.orientation.q.inverse() * direction_eci;
    let body_norm = direction_body.norm();
    if !body_norm.is_finite() || body_norm <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "multi_body.landing_controller produced invalid body-frame thrust direction"
                .to_owned(),
        });
    }
    let unit = direction_body / body_norm;
    let gimbal_pitch_rad = unit.x.atan2(unit.z);
    let gimbal_yaw_rad = (-unit.y).atan2((unit.x * unit.x + unit.z * unit.z).sqrt());
    Ok((gimbal_pitch_rad, gimbal_yaw_rad))
}

fn push_engine_command(
    set: &mut EngineCommandSet,
    engine_id: EngineId,
    throttle_unit: f64,
    gimbal_pitch_rad: f64,
    gimbal_yaw_rad: f64,
    ignite: bool,
    shutdown: bool,
) -> Result<(), RunnerError> {
    if usize::from(set.count) >= MAX_ENGINE_COMMANDS {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "multi_body.landing_controller emitted more than {MAX_ENGINE_COMMANDS} engine commands"
            ),
        });
    }
    if !throttle_unit.is_finite() || !gimbal_pitch_rad.is_finite() || !gimbal_yaw_rad.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: "multi_body.landing_controller produced a non-finite engine command".to_owned(),
        });
    }
    let index = usize::from(set.count);
    set.commands[index] = EngineCommand {
        engine_id: engine_id.value(),
        throttle_unit,
        gimbal_pitch_rad,
        gimbal_yaw_rad,
        ignite,
        shutdown,
    };
    set.count += 1;
    Ok(())
}
