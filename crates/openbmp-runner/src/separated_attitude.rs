//! Runner-side attitude target tracking for separated rigid-body lanes.
//!
//! This is intentionally a simulator-side controller, not recovered
//! flight data. It lets local diagnostics exercise post-separation
//! attitude dynamics with the same direct-torque effector path used by
//! closed-loop FC scenarios.

use nalgebra::{Unit, Vector3};
use openbmp_core::{BodyId, EffectorId, SimTime};
use openbmp_fc::topics::{EffectorCommand, EffectorCommandSet, MAX_EFFECTOR_COMMANDS};
use openbmp_scenario::{
    MultiBodyAttitudeTargetConfig, MultiBodyAttitudeTargetKindConfig, ScenarioDocument,
};
use openbmp_sim::SeparatedRigidBody;

use crate::error::RunnerError;

/// Collection of separated-body attitude target controllers.
#[derive(Clone, Debug, Default)]
pub struct SeparatedAttitudeTargets {
    targets: Vec<SeparatedAttitudeTarget>,
}

impl SeparatedAttitudeTargets {
    /// Build from `[multi_body]` config. Empty when no attitude targets
    /// are declared.
    pub fn build(document: &ScenarioDocument) -> Result<Self, RunnerError> {
        let Some(multi_body) = &document.multi_body else {
            return Ok(Self::default());
        };
        let mut targets = Vec::with_capacity(multi_body.attitude_targets.len());
        for config in &multi_body.attitude_targets {
            targets.push(SeparatedAttitudeTarget::from_config(config)?);
        }
        Ok(Self { targets })
    }

    /// `true` when no separated-body attitude targets are configured.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.targets.is_empty()
    }

    /// Evaluate every active target against the current separated
    /// lanes and push one-tick direct-torque effector commands.
    pub fn apply(
        &self,
        separated_bodies: &[SeparatedRigidBody],
        time: SimTime,
        effectors: &mut crate::effectors::EffectorRack,
    ) -> Result<(), RunnerError> {
        if self.targets.is_empty() {
            return Ok(());
        }
        let mut command_set = EffectorCommandSet {
            time,
            ..EffectorCommandSet::default()
        };
        for target in &self.targets {
            if !target.active_at(time) {
                continue;
            }
            let Some(body) = separated_bodies
                .iter()
                .find(|body| body.body == target.body && body.propagating)
            else {
                continue;
            };
            target.append_commands(body, &mut command_set)?;
        }
        if command_set.count > 0 {
            effectors.apply_fc_commands(&command_set)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
struct SeparatedAttitudeTarget {
    body: BodyId,
    start_time_s: Option<f64>,
    end_time_s: Option<f64>,
    body_axis_body: Vector3<f64>,
    effectors: [Option<EffectorId>; 3],
    kp: f64,
    kd: f64,
    max_command: Option<f64>,
    target: AttitudeTargetKind,
}

impl SeparatedAttitudeTarget {
    fn from_config(config: &MultiBodyAttitudeTargetConfig) -> Result<Self, RunnerError> {
        let body_axis_body = normalized(Vector3::from(
            config.body_axis_body.unwrap_or([0.0, 0.0, 1.0]),
        ))
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "multi_body.attitude_target.body_axis_body must have non-zero norm".to_owned(),
        })?;
        Ok(Self {
            body: BodyId::from_path(&format!("vehicle.assembly.bodies.{}", config.body_id)),
            start_time_s: config.start_time_s,
            end_time_s: config.end_time_s,
            body_axis_body,
            effectors: [
                config
                    .roll_effector
                    .as_ref()
                    .map(|id| EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}"))),
                config
                    .pitch_effector
                    .as_ref()
                    .map(|id| EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}"))),
                config
                    .yaw_effector
                    .as_ref()
                    .map(|id| EffectorId::from_path(&format!("vehicle.assembly.effectors.{id}"))),
            ],
            kp: config.kp,
            kd: config.kd,
            max_command: config.max_command,
            target: AttitudeTargetKind::from_config(&config.target),
        })
    }

    fn active_at(&self, time: SimTime) -> bool {
        let t = time.as_seconds();
        if let Some(start) = self.start_time_s
            && t < start
        {
            return false;
        }
        if let Some(end) = self.end_time_s
            && t >= end
        {
            return false;
        }
        true
    }

    fn append_commands(
        &self,
        body: &SeparatedRigidBody,
        command_set: &mut EffectorCommandSet,
    ) -> Result<(), RunnerError> {
        let target_eci = self.target.target_eci(&body.state)?;
        let axis_eci = body.state.orientation.q * self.body_axis_body;
        let error_eci = axis_eci.cross(&target_eci);
        let error_body = body.state.orientation.q.inverse() * error_eci;
        let raw = self.kp * error_body - self.kd * body.state.angular_velocity.vector;
        for (axis, effector) in self.effectors.iter().enumerate() {
            let Some(effector) = effector else {
                continue;
            };
            let command = if let Some(limit) = self.max_command {
                raw[axis].clamp(-limit, limit)
            } else {
                raw[axis]
            };
            push_effector_command(command_set, *effector, command)?;
        }
        Ok(())
    }
}

#[derive(Clone, Debug)]
enum AttitudeTargetKind {
    EciVector {
        vector_eci: Vector3<f64>,
    },
    SurfaceRelativeAxes {
        radial: f64,
        downrange: f64,
        crossrange: f64,
        downrange_axis_eci: Vector3<f64>,
    },
}

impl AttitudeTargetKind {
    fn from_config(config: &MultiBodyAttitudeTargetKindConfig) -> Self {
        match *config {
            MultiBodyAttitudeTargetKindConfig::EciVector { vector_eci } => Self::EciVector {
                vector_eci: Vector3::from(vector_eci),
            },
            MultiBodyAttitudeTargetKindConfig::SurfaceRelativeAxes {
                radial,
                downrange,
                crossrange,
                downrange_axis_eci,
            } => Self::SurfaceRelativeAxes {
                radial,
                downrange,
                crossrange,
                downrange_axis_eci: Vector3::from(downrange_axis_eci),
            },
        }
    }

    fn target_eci(
        &self,
        state: &openbmp_state::RigidBodyState,
    ) -> Result<Vector3<f64>, RunnerError> {
        match *self {
            Self::EciVector { vector_eci } => {
                normalized(vector_eci).ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "multi_body.attitude_target.target.vector_eci must have non-zero norm"
                        .to_owned(),
                })
            }
            Self::SurfaceRelativeAxes {
                radial,
                downrange,
                crossrange,
                downrange_axis_eci,
            } => {
                let up = normalized(state.position.vector).ok_or_else(|| {
                    RunnerError::UnsupportedScenario {
                        what: "surface-relative attitude target requires non-zero position"
                            .to_owned(),
                    }
                })?;
                let downrange_raw = downrange_axis_eci - up * downrange_axis_eci.dot(&up);
                let downrange_unit =
                    normalized(downrange_raw).ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "surface-relative attitude target downrange_axis_eci must not be \
                               parallel to the local radial direction"
                            .to_owned(),
                    })?;
                let crossrange_unit = normalized(up.cross(&downrange_unit)).ok_or_else(|| {
                    RunnerError::UnsupportedScenario {
                        what: "surface-relative attitude target basis construction failed"
                            .to_owned(),
                    }
                })?;
                let target =
                    radial * up + downrange * downrange_unit + crossrange * crossrange_unit;
                normalized(target).ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "surface-relative attitude target vector must have non-zero norm"
                        .to_owned(),
                })
            }
        }
    }
}

fn push_effector_command(
    set: &mut EffectorCommandSet,
    effector_id: EffectorId,
    command: f64,
) -> Result<(), RunnerError> {
    if usize::from(set.count) >= MAX_EFFECTOR_COMMANDS {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "multi_body.attitude_target emitted more than {MAX_EFFECTOR_COMMANDS} effector commands"
            ),
        });
    }
    if !command.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: "multi_body.attitude_target produced a non-finite effector command".to_owned(),
        });
    }
    let index = usize::from(set.count);
    set.commands[index] = EffectorCommand {
        effector_id: effector_id.value(),
        command,
        saturated: false,
    };
    set.count += 1;
    Ok(())
}

fn normalized(vector: Vector3<f64>) -> Option<Vector3<f64>> {
    if !vector.iter().all(|value| value.is_finite()) {
        return None;
    }
    Unit::try_new(vector, 1.0e-12).map(Unit::into_inner)
}
