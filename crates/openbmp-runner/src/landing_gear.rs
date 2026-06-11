//! Runner-side landing-gear rack for WP-14.4.
//!
//! The rack is opt-in through `[vehicle.landing_gear]`. It keeps legacy
//! scenarios byte-identical: no config means no rack, no force model, and no
//! telemetry channels.

use std::borrow::Cow;
use std::sync::{Arc, Mutex};

use nalgebra::Vector3;
use openbmp_contact::ContactGeometry;
use openbmp_core::{BodyId, ChannelId, ModelId, SimTime, ValidationStatus};
use openbmp_scenario::{
    ContactGeometryConfig, LandingGearConfig, LandingGearLegConfig, ScenarioDocument,
};
use openbmp_sim::{ForceContext, ForceModel, ModelEvalError, MomentContext, MomentModel};
use openbmp_state::RigidBodyState;
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};
use openbmp_vehicle::{CrushCore, LandingGearLeg, OleoStage};

use crate::RunnerError;

/// Runner-owned, cloneable landing-gear runtime.
#[derive(Clone)]
pub(crate) struct LandingGearRuntime {
    inner: Arc<Mutex<LandingGearRuntimeInner>>,
}

impl std::fmt::Debug for LandingGearRuntime {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let leg_count = self.inner.lock().map_or(0, |inner| inner.legs.len());
        f.debug_struct("LandingGearRuntime")
            .field("leg_count", &leg_count)
            .finish()
    }
}

impl LandingGearRuntime {
    /// Build from a scenario document. Returns `None` when landing gear is not
    /// declared.
    pub(crate) fn maybe_build(document: &ScenarioDocument) -> Result<Option<Self>, RunnerError> {
        document
            .vehicle
            .landing_gear
            .as_ref()
            .map(Self::build)
            .transpose()
    }

    fn build(config: &LandingGearConfig) -> Result<Self, RunnerError> {
        let mut legs = Vec::with_capacity(config.legs.len());
        for leg in &config.legs {
            legs.push(RuntimeLeg::from_config(leg)?);
        }
        Ok(Self {
            inner: Arc::new(Mutex::new(LandingGearRuntimeInner {
                ground_altitude_m: config.ground_altitude_m,
                legs,
                last: None,
            })),
        })
    }

    fn force_eval(
        &self,
        state: &RigidBodyState,
        time: SimTime,
        active_body: Option<BodyId>,
    ) -> Result<LandingGearEvaluation, ModelEvalError> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| ModelEvalError::InvalidState {
                model: LANDING_GEAR_INTERNAL_MODEL_ID,
                reason: Cow::Borrowed("landing gear runtime lock poisoned"),
            })?;
        inner.evaluate(state, time, active_body, true)
    }

    fn moment_eval(
        &self,
        state: &RigidBodyState,
        time: SimTime,
        active_body: Option<BodyId>,
    ) -> Result<LandingGearEvaluation, ModelEvalError> {
        let mut inner = self
            .inner
            .lock()
            .map_err(|_| ModelEvalError::InvalidState {
                model: LANDING_GEAR_INTERNAL_MODEL_ID,
                reason: Cow::Borrowed("landing gear runtime lock poisoned"),
            })?;
        let signature = EvaluationSignature::new(state, time, active_body);
        if let Some(last) = &inner.last
            && last.signature == signature
        {
            return Ok(last.evaluation.clone());
        }
        inner.evaluate(state, time, active_body, false)
    }

    /// Latest per-leg samples, in scenario-declared order.
    pub(crate) fn samples(&self) -> Result<Vec<LandingGearLegSample>, RunnerError> {
        let inner = self
            .inner
            .lock()
            .map_err(|_| RunnerError::UnsupportedScenario {
                what: "landing gear runtime lock poisoned".to_owned(),
            })?;
        Ok(inner
            .last
            .as_ref()
            .map_or_else(Vec::new, |last| last.evaluation.samples.clone()))
    }
}

const LANDING_GEAR_INTERNAL_MODEL_ID: ModelId = ModelId::new(390);

#[derive(Clone, Debug)]
struct LandingGearRuntimeInner {
    ground_altitude_m: f64,
    legs: Vec<RuntimeLeg>,
    last: Option<CachedEvaluation>,
}

impl LandingGearRuntimeInner {
    fn evaluate(
        &mut self,
        state: &RigidBodyState,
        time: SimTime,
        active_body: Option<BodyId>,
        mutate: bool,
    ) -> Result<LandingGearEvaluation, ModelEvalError> {
        let signature = EvaluationSignature::new(state, time, active_body);
        let mut force_eci = Vector3::zeros();
        let mut moment_body = Vector3::zeros();
        let mut samples = Vec::with_capacity(self.legs.len());
        for leg in &mut self.legs {
            let sample = leg.evaluate(state, self.ground_altitude_m, active_body, mutate)?;
            force_eci += sample.force_eci_n;
            moment_body += sample.moment_body_n_m;
            samples.push(sample.public);
        }
        let evaluation = LandingGearEvaluation {
            force_eci_n: force_eci,
            moment_body_n_m: moment_body,
            samples,
        };
        if mutate {
            self.last = Some(CachedEvaluation {
                signature,
                evaluation: evaluation.clone(),
            });
        }
        Ok(evaluation)
    }
}

#[derive(Clone, Debug)]
struct RuntimeLeg {
    id: String,
    owner: BodyId,
    leg: LandingGearLeg,
    crush: Option<CrushCore>,
}

impl RuntimeLeg {
    fn from_config(config: &LandingGearLegConfig) -> Result<Self, RunnerError> {
        let footpad = match config.footpad {
            ContactGeometryConfig::Point => ContactGeometry::Point,
            ContactGeometryConfig::Sphere => {
                ContactGeometry::sphere(config.footpad_radius_m.unwrap_or(0.0)).map_err(|err| {
                    RunnerError::UnsupportedScenario {
                        what: format!(
                            "invalid vehicle.landing_gear leg `{}` footpad: {err}",
                            config.id
                        ),
                    }
                })?
            }
        };
        let oleo = config
            .oleo
            .map(|oleo| {
                OleoStage::new(
                    oleo.p0_pa,
                    oleo.v0_m3,
                    oleo.gamma_unit,
                    oleo.orifice_c_n_s2_m2,
                    oleo.stroke_max_m,
                    oleo.piston_area_m2,
                )
            })
            .transpose()
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "invalid vehicle.landing_gear leg `{}` oleo: {err}",
                    config.id
                ),
            })?;
        let crush = config
            .crush
            .map(|crush| CrushCore::new(crush.f_crush_n, crush.stroke_max_m, crush.k_elastic_n_m))
            .transpose()
            .map_err(|err| RunnerError::UnsupportedScenario {
                what: format!(
                    "invalid vehicle.landing_gear leg `{}` crush: {err}",
                    config.id
                ),
            })?;
        let leg = LandingGearLeg::new(
            config.attach_body_m,
            config.strut_axis_body,
            config.free_length_m,
            oleo,
            crush,
            footpad,
        )
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("invalid vehicle.landing_gear leg `{}`: {err}", config.id),
        })?;
        Ok(Self {
            id: config.id.clone(),
            owner: body_id_from_scenario_text(&config.mounted_to),
            leg,
            crush,
        })
    }

    fn evaluate(
        &mut self,
        state: &RigidBodyState,
        ground_altitude_m: f64,
        active_body: Option<BodyId>,
        mutate: bool,
    ) -> Result<InternalLegSample, ModelEvalError> {
        if active_body.is_some_and(|active| active != self.owner) {
            return Ok(InternalLegSample::zero(self.id.clone()));
        }

        let attach = array_to_vec(self.leg.attach_body_m());
        let axis = array_to_vec(self.leg.strut_axis_body());
        let foot_body = attach + axis * self.leg.free_length_m();
        let foot_eci_offset = state.orientation.q * foot_body;
        let foot_eci = state.position.vector + foot_eci_offset;
        let omega_eci = state.orientation.q * state.angular_velocity.vector;
        let foot_velocity_eci = state.velocity.vector + omega_eci.cross(&foot_eci_offset);
        let radius_m = footpad_radius_m(self.leg.footpad());
        let gap_m = foot_eci.z - ground_altitude_m - radius_m;
        let raw_stroke_m = (-gap_m).max(0.0);
        let compression_rate_m_s = (-foot_velocity_eci.z).max(0.0);

        if raw_stroke_m <= 0.0 {
            return Ok(InternalLegSample::zero_with_gap(self.id.clone(), gap_m));
        }

        let oleo_stroke_max = self.leg.oleo().map_or(0.0, OleoStage::stroke_max_m);
        let crush_stroke_max = self.crush.map_or(0.0, CrushCore::stroke_max_m);
        let stroke_limit_m = (oleo_stroke_max + crush_stroke_max).max(0.0);
        let stroke_m = if stroke_limit_m > 0.0 {
            raw_stroke_m.min(stroke_limit_m)
        } else {
            raw_stroke_m
        };

        let mut force_n = 0.0;
        if let Some(oleo) = self.leg.oleo() {
            force_n += oleo
                .force_n(stroke_m.min(oleo.stroke_max_m()), compression_rate_m_s)
                .map_err(|err| ModelEvalError::InvalidState {
                    model: LANDING_GEAR_INTERNAL_MODEL_ID,
                    reason: Cow::Owned(err.to_string()),
                })?;
        }
        if let Some(crush) = &mut self.crush {
            let crush_stroke_m = (stroke_m - oleo_stroke_max)
                .max(0.0)
                .min(crush.stroke_max_m());
            let pre_force_n = crush.force_at_stroke_n(crush_stroke_m).map_err(|err| {
                ModelEvalError::InvalidState {
                    model: LANDING_GEAR_INTERNAL_MODEL_ID,
                    reason: Cow::Owned(err.to_string()),
                }
            })?;
            let mut crush_force_n = pre_force_n;
            if mutate && crush_stroke_m > crush.crushed_m() {
                let energy_j = (crush_stroke_m - crush.crushed_m()) * crush.f_crush_n();
                let response = crush.absorb_energy_j(energy_j).map_err(|err| {
                    ModelEvalError::InvalidState {
                        model: LANDING_GEAR_INTERNAL_MODEL_ID,
                        reason: Cow::Owned(err.to_string()),
                    }
                })?;
                crush_force_n = crush_force_n.max(response.force_n);
            }
            force_n += crush_force_n;
        }

        if !force_n.is_finite() {
            return Err(ModelEvalError::NonFinite {
                model: LANDING_GEAR_INTERNAL_MODEL_ID,
            });
        }
        let force_eci_n = Vector3::new(0.0, 0.0, force_n);
        let force_body_n = state.orientation.q.inverse() * force_eci_n;
        let moment_body_n_m = foot_body.cross(&force_body_n);
        Ok(InternalLegSample {
            force_eci_n,
            moment_body_n_m,
            public: LandingGearLegSample {
                id: self.id.clone(),
                stroke_m,
                gap_m,
                compression_rate_m_s,
                force_n,
                crushed_m: self.crush.map_or(0.0, CrushCore::crushed_m),
                in_contact: true,
            },
        })
    }
}

#[derive(Clone, Debug)]
struct CachedEvaluation {
    signature: EvaluationSignature,
    evaluation: LandingGearEvaluation,
}

#[derive(Clone, Debug)]
struct LandingGearEvaluation {
    force_eci_n: Vector3<f64>,
    moment_body_n_m: Vector3<f64>,
    samples: Vec<LandingGearLegSample>,
}

#[derive(Clone, Debug)]
struct InternalLegSample {
    force_eci_n: Vector3<f64>,
    moment_body_n_m: Vector3<f64>,
    public: LandingGearLegSample,
}

impl InternalLegSample {
    fn zero(id: String) -> Self {
        Self::zero_with_gap(id, f64::INFINITY)
    }

    fn zero_with_gap(id: String, gap_m: f64) -> Self {
        Self {
            force_eci_n: Vector3::zeros(),
            moment_body_n_m: Vector3::zeros(),
            public: LandingGearLegSample {
                id,
                stroke_m: 0.0,
                gap_m,
                compression_rate_m_s: 0.0,
                force_n: 0.0,
                crushed_m: 0.0,
                in_contact: false,
            },
        }
    }
}

/// Public per-leg diagnostic sample used by telemetry and tests.
#[derive(Clone, Debug, PartialEq)]
pub(crate) struct LandingGearLegSample {
    pub(crate) id: String,
    pub(crate) stroke_m: f64,
    pub(crate) gap_m: f64,
    pub(crate) compression_rate_m_s: f64,
    pub(crate) force_n: f64,
    pub(crate) crushed_m: f64,
    pub(crate) in_contact: bool,
}

#[derive(Clone, Debug, PartialEq)]
struct EvaluationSignature {
    time_s: f64,
    active_body: Option<BodyId>,
    position_m: [f64; 3],
    velocity_m_s: [f64; 3],
    quaternion_xyzw: [f64; 4],
    omega_body_rad_s: [f64; 3],
}

impl EvaluationSignature {
    fn new(state: &RigidBodyState, time: SimTime, active_body: Option<BodyId>) -> Self {
        let q = state.orientation.q.into_inner();
        Self {
            time_s: time.as_seconds(),
            active_body,
            position_m: [
                state.position.vector.x,
                state.position.vector.y,
                state.position.vector.z,
            ],
            velocity_m_s: [
                state.velocity.vector.x,
                state.velocity.vector.y,
                state.velocity.vector.z,
            ],
            quaternion_xyzw: [q.coords.x, q.coords.y, q.coords.z, q.coords.w],
            omega_body_rad_s: [
                state.angular_velocity.vector.x,
                state.angular_velocity.vector.y,
                state.angular_velocity.vector.z,
            ],
        }
    }
}

/// Kernel force adapter for the landing-gear rack.
#[derive(Clone, Debug)]
pub(crate) struct LandingGearForceAdapter {
    runtime: LandingGearRuntime,
    model_id: ModelId,
}

impl LandingGearForceAdapter {
    #[must_use]
    pub(crate) fn new(runtime: LandingGearRuntime, model_id: ModelId) -> Self {
        Self { runtime, model_id }
    }
}

impl ForceModel<RigidBodyState> for LandingGearForceAdapter {
    fn force_n_eci(
        &self,
        ctx: ForceContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let force = self
            .runtime
            .force_eval(ctx.state, ctx.time, ctx.active_body)?
            .force_eci_n;
        if force.x.is_finite() && force.y.is_finite() && force.z.is_finite() {
            Ok(force)
        } else {
            Err(ModelEvalError::NonFinite {
                model: self.model_id,
            })
        }
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::ValidatedToy
    }
}

/// Kernel moment adapter for the landing-gear rack.
#[derive(Clone, Debug)]
pub(crate) struct LandingGearMomentAdapter {
    runtime: LandingGearRuntime,
    model_id: ModelId,
}

impl LandingGearMomentAdapter {
    #[must_use]
    pub(crate) fn new(runtime: LandingGearRuntime, model_id: ModelId) -> Self {
        Self { runtime, model_id }
    }
}

impl MomentModel<RigidBodyState> for LandingGearMomentAdapter {
    fn moment_n_m_body(
        &self,
        ctx: MomentContext<'_, RigidBodyState>,
    ) -> Result<Vector3<f64>, ModelEvalError> {
        let moment = self
            .runtime
            .moment_eval(ctx.state, ctx.time, ctx.active_body)?
            .moment_body_n_m;
        if moment.x.is_finite() && moment.y.is_finite() && moment.z.is_finite() {
            Ok(moment)
        } else {
            Err(ModelEvalError::NonFinite {
                model: self.model_id,
            })
        }
    }

    fn validation(&self) -> ValidationStatus {
        ValidationStatus::ValidatedToy
    }
}

/// Per-leg landing-gear telemetry channels.
#[derive(Debug)]
pub(crate) struct LandingGearTelemetryChannels {
    per_leg: Vec<LandingGearLegTelemetryChannels>,
}

impl LandingGearTelemetryChannels {
    pub(crate) fn new<F>(document: &ScenarioDocument, alloc: &mut F) -> Result<Self, RunnerError>
    where
        F: FnMut() -> ChannelId,
    {
        let per_leg = document
            .vehicle
            .landing_gear
            .as_ref()
            .map(|gear| {
                gear.legs
                    .iter()
                    .map(|leg| LandingGearLegTelemetryChannels::new(&leg.id, alloc))
                    .collect::<Result<Vec<_>, _>>()
            })
            .transpose()?
            .unwrap_or_default();
        Ok(Self { per_leg })
    }

    pub(crate) fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        for leg in &self.per_leg {
            leg.push_metadata(channels);
        }
    }

    pub(crate) fn insert(
        &self,
        row: &mut TelemetryRow,
        samples: &[LandingGearLegSample],
    ) -> Result<(), RunnerError> {
        for channels in &self.per_leg {
            let sample = samples
                .iter()
                .find(|sample| sample.id == channels.id)
                .cloned()
                .unwrap_or_else(|| LandingGearLegSample {
                    id: channels.id.clone(),
                    stroke_m: 0.0,
                    gap_m: f64::INFINITY,
                    compression_rate_m_s: 0.0,
                    force_n: 0.0,
                    crushed_m: 0.0,
                    in_contact: false,
                });
            channels.insert(row, &sample)?;
        }
        Ok(())
    }
}

#[derive(Debug)]
struct LandingGearLegTelemetryChannels {
    id: String,
    stroke: TelemetryChannel<f64>,
    gap: TelemetryChannel<f64>,
    compression_rate: TelemetryChannel<f64>,
    force: TelemetryChannel<f64>,
    crushed: TelemetryChannel<f64>,
    in_contact: TelemetryChannel<bool>,
}

impl LandingGearLegTelemetryChannels {
    fn new<F>(id: &str, alloc: &mut F) -> Result<Self, RunnerError>
    where
        F: FnMut() -> ChannelId,
    {
        let prefix = format!("landing_gear.{id}");
        Ok(Self {
            id: id.to_owned(),
            stroke: TelemetryChannel::<f64>::new(
                alloc(),
                format!("{prefix}.stroke_m"),
                "m",
                None::<&str>,
            )?,
            gap: TelemetryChannel::<f64>::new(
                alloc(),
                format!("{prefix}.gap_m"),
                "m",
                None::<&str>,
            )?,
            compression_rate: TelemetryChannel::<f64>::new(
                alloc(),
                format!("{prefix}.compression_rate_m_s"),
                "m/s",
                None::<&str>,
            )?,
            force: TelemetryChannel::<f64>::new(
                alloc(),
                format!("{prefix}.force_n"),
                "N",
                None::<&str>,
            )?,
            crushed: TelemetryChannel::<f64>::new(
                alloc(),
                format!("{prefix}.crushed_m"),
                "m",
                None::<&str>,
            )?,
            in_contact: TelemetryChannel::<bool>::new(
                alloc(),
                format!("{prefix}.in_contact"),
                "bool",
                None::<&str>,
            )?,
        })
    }

    fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        channels.push(self.stroke.metadata().clone());
        channels.push(self.gap.metadata().clone());
        channels.push(self.compression_rate.metadata().clone());
        channels.push(self.force.metadata().clone());
        channels.push(self.crushed.metadata().clone());
        channels.push(self.in_contact.metadata().clone());
    }

    fn insert(
        &self,
        row: &mut TelemetryRow,
        sample: &LandingGearLegSample,
    ) -> Result<(), RunnerError> {
        row.insert(&self.stroke, sample.stroke_m)?;
        row.insert(&self.gap, finite_or_zero(sample.gap_m))?;
        row.insert(&self.compression_rate, sample.compression_rate_m_s)?;
        row.insert(&self.force, sample.force_n)?;
        row.insert(&self.crushed, sample.crushed_m)?;
        row.insert(&self.in_contact, sample.in_contact)?;
        Ok(())
    }
}

fn array_to_vec(value: [f64; 3]) -> Vector3<f64> {
    Vector3::new(value[0], value[1], value[2])
}

fn footpad_radius_m(geometry: ContactGeometry) -> f64 {
    match geometry {
        ContactGeometry::Point => 0.0,
        ContactGeometry::Sphere { radius_m } => radius_m,
    }
}

fn body_id_from_scenario_text(id: &str) -> BodyId {
    BodyId::from_path(&format!("vehicle.assembly.bodies.{id}"))
}

fn finite_or_zero(value: f64) -> f64 {
    if value.is_finite() { value } else { 0.0 }
}

/// Build a zero-height rigid body state for unit tests.
#[cfg(test)]
fn test_state(z_m: f64, vz_m_s: f64) -> RigidBodyState {
    let q = nalgebra::UnitQuaternion::identity();
    RigidBodyState::new(
        SimTime::from_seconds(0.0),
        openbmp_core::Position3::new(0.0, 0.0, z_m),
        openbmp_core::Velocity3::new(0.0, 0.0, vz_m_s),
        openbmp_core::Quaternion::<openbmp_core::Body, openbmp_core::Eci>::from_unit_quaternion(q),
        openbmp_core::AngularVelocity3::<openbmp_core::Body>::new(0.0, 0.0, 0.0),
        openbmp_state::MassProperties::new(
            uom::si::f64::Mass::new::<uom::si::mass::kilogram>(1000.0),
            openbmp_core::Position3::<openbmp_core::Body>::new(0.0, 0.0, 0.0),
            nalgebra::Matrix3::identity(),
        ),
    )
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn landing_gear_runtime_reports_four_leg_loads() {
        let config = LandingGearConfig {
            data_file: None,
            data_file_sha256: None,
            ground_altitude_m: 0.0,
            legs: ["fl", "fr", "rl", "rr"]
                .into_iter()
                .map(|id| LandingGearLegConfig {
                    id: id.to_owned(),
                    mounted_to: "core".to_owned(),
                    attach_body_m: [0.0, 0.0, 0.0],
                    strut_axis_body: [0.0, 0.0, -1.0],
                    free_length_m: 1.0,
                    footpad: ContactGeometryConfig::Sphere,
                    footpad_radius_m: Some(0.05),
                    oleo: Some(openbmp_scenario::LandingGearOleoConfig {
                        p0_pa: 100_000.0,
                        v0_m3: 0.01,
                        gamma_unit: 1.2,
                        orifice_c_n_s2_m2: 1_000.0,
                        stroke_max_m: 0.2,
                        piston_area_m2: 0.01,
                    }),
                    crush: None,
                })
                .collect(),
        };
        let runtime = LandingGearRuntime::build(&config).unwrap();
        let state = test_state(0.9, -1.0);
        let eval = runtime
            .force_eval(&state, SimTime::from_seconds(0.0), None)
            .unwrap();

        assert!(eval.force_eci_n.z > 0.0);
        assert_eq!(runtime.samples().unwrap().len(), 4);
        assert!(
            runtime
                .samples()
                .unwrap()
                .iter()
                .all(|sample| sample.in_contact)
        );
    }
}
