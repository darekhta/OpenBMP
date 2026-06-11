//! Runner-side construction, diagnostics, and reports for compliant contact.

use openbmp_contact::{
    AnchoredStictionFriction, AnchoredStictionState, ContactEnergyAudit, ContactError,
    ContactGeometry, ContactPair, HalfSpace, HertzNormal, HuntCrossleyNormal, KelvinVoigtNormal,
    NormalLaw, RegularizedCoulombFriction, RestDetector, RestDetectorConfig, RestDetectorState,
    RestStatus, half_space_kinematics,
};
use openbmp_core::{ChannelId, ModelId};
use openbmp_scenario::{
    ContactConfig, ContactFrictionLawConfig, ContactGeometryConfig, ContactNormalLawConfig,
    ScenarioDocument,
};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};
use openbmp_vehicle::HalfSpaceContactForceAdapter;
use std::sync::Mutex;

use crate::RunnerError;

const REST_GAP_TOLERANCE_M: f64 = 1.0e-9;
const REST_SPEED_TOLERANCE_M_S: f64 = 1.0e-9;
const REST_HOLD_SAMPLES: u32 = 3;

/// Returns the runner kernel step size after contact fixed sub-stepping.
#[must_use]
pub(crate) fn kernel_step_s(document: &ScenarioDocument) -> f64 {
    document
        .contact
        .as_ref()
        .map_or(document.time.dt_s, |contact| {
            document.time.dt_s / f64::from(contact.substeps)
        })
}

/// Contact classification emitted in [`crate::RunOutcome`].
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContactOutcomeKind {
    /// The configured contact pair never penetrated the surface.
    NoContact,
    /// The final sample is penetrating and normal motion is below the
    /// deterministic rest tolerance.
    Rest,
    /// The run ended while the pair was still moving or otherwise not at rest.
    Unsettled,
}

/// One deterministic contact diagnostic sample at a telemetry row boundary.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactStepDiagnostics {
    /// Signed gap in meters; negative values mean penetration.
    pub gap_m: f64,
    /// Non-negative penetration depth in meters.
    pub penetration_m: f64,
    /// Relative normal velocity in meters per second.
    pub normal_velocity_m_s: f64,
    /// Relative tangential speed in meters per second.
    pub tangential_speed_m_s: f64,
    /// Non-negative scalar normal force in newtons.
    pub normal_force_n: f64,
    /// Elastic energy stored in the normal law and tangential anchor.
    pub elastic_energy_j: f64,
    /// `true` when the contact is statically stuck for rest detection.
    pub sticking: bool,
    /// Instantaneous damping/friction dissipation power in watts.
    pub dissipated_power_w: f64,
    /// Instantaneous contact-force power on the vehicle in watts.
    pub contact_power_on_vehicle_w: f64,
}

/// Deterministic run-level contact energy balance.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactRunEnergyAudit {
    /// Elastic energy stored at the first contact diagnostic sample.
    pub initial_elastic_energy_j: f64,
    /// Elastic energy stored at the final contact diagnostic sample.
    pub final_elastic_energy_j: f64,
    /// Integrated work done by contact forces on the vehicle.
    pub contact_work_on_vehicle_j: f64,
    /// Integrated energy dissipated by normal damping and tangential friction.
    pub dissipated_energy_j: f64,
    /// Signed closure error for `ΔE_elastic + D + W_vehicle = 0`.
    pub closure_error_j: f64,
    /// Absolute closure error normalized by the largest energy scale.
    pub relative_closure_error: f64,
}

/// Run-level contact summary emitted outside canonical telemetry bytes.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ContactRunReport {
    /// Closed classification of the run endpoint.
    pub outcome: ContactOutcomeKind,
    /// Number of diagnostic samples accumulated.
    pub samples: u64,
    /// Maximum penetration depth observed in meters.
    pub max_penetration_m: f64,
    /// Maximum scalar normal force observed in newtons.
    pub max_normal_force_n: f64,
    /// Final row's contact diagnostics.
    pub final_diagnostics: ContactStepDiagnostics,
    /// Run-level contact energy balance.
    pub energy: ContactRunEnergyAudit,
}

pub(crate) struct ContactDiagnosticsEvaluator {
    half_space: HalfSpace,
    geometry: ContactGeometry,
    normal_law: NormalLaw,
    friction: ContactDiagnosticsFriction,
}

impl ContactDiagnosticsEvaluator {
    pub(crate) fn from_config(config: &ContactConfig) -> Result<Self, RunnerError> {
        let parts = build_half_space_contact_parts(config)?;
        Ok(Self {
            half_space: parts.half_space,
            geometry: parts.geometry,
            normal_law: parts.normal_law,
            friction: ContactDiagnosticsFriction::from_runtime(parts.friction),
        })
    }

    pub(crate) fn diagnostics_from_state_vectors(
        &self,
        position_m: nalgebra::Vector3<f64>,
        velocity_m_s: nalgebra::Vector3<f64>,
        time_s: f64,
    ) -> Result<ContactStepDiagnostics, RunnerError> {
        let position = [position_m.x, position_m.y, position_m.z];
        let velocity = [velocity_m_s.x, velocity_m_s.y, velocity_m_s.z];
        let kinematics = half_space_kinematics(self.half_space, self.geometry, position, velocity)
            .map_err(contact_build_error)?;
        let normal = self.normal_law.evaluate(kinematics);
        let friction = self
            .friction
            .evaluate(kinematics, normal.normal_force_n, time_s)?;
        let total_force_n = add(
            scale(self.half_space.normal(), normal.normal_force_n),
            friction.force_n,
        );
        let dissipated_power_w = normal.damping_power_w + friction.dissipated_power_w;
        let contact_power_on_vehicle_w = dot(total_force_n, velocity);
        Ok(ContactStepDiagnostics {
            gap_m: kinematics.gap_m,
            penetration_m: kinematics.penetration_m(),
            normal_velocity_m_s: kinematics.normal_velocity_m_s,
            tangential_speed_m_s: kinematics.tangential_speed_m_s(),
            normal_force_n: normal.normal_force_n,
            elastic_energy_j: normal.elastic_energy_j + friction.elastic_energy_j,
            sticking: friction.sticking,
            dissipated_power_w,
            contact_power_on_vehicle_w,
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct ContactFrictionDiagnostics {
    force_n: [f64; 3],
    elastic_energy_j: f64,
    sticking: bool,
    dissipated_power_w: f64,
}

enum ContactDiagnosticsFriction {
    RegularizedCoulomb(RegularizedCoulombFriction),
    AnchoredStiction {
        friction: AnchoredStictionFriction,
        state: Mutex<TimeGatedStictionState>,
    },
}

impl ContactDiagnosticsFriction {
    fn from_runtime(friction: ContactFrictionRuntime) -> Self {
        match friction {
            ContactFrictionRuntime::RegularizedCoulomb(friction) => {
                Self::RegularizedCoulomb(friction)
            }
            ContactFrictionRuntime::AnchoredStiction(friction) => Self::AnchoredStiction {
                friction,
                state: Mutex::new(TimeGatedStictionState::default()),
            },
        }
    }

    fn evaluate(
        &self,
        kinematics: openbmp_contact::ContactKinematics,
        normal_force_n: f64,
        time_s: f64,
    ) -> Result<ContactFrictionDiagnostics, RunnerError> {
        match self {
            Self::RegularizedCoulomb(friction) => {
                let force = friction.force_n(kinematics, normal_force_n);
                Ok(ContactFrictionDiagnostics {
                    force_n: force,
                    elastic_energy_j: 0.0,
                    sticking: normal_force_n > 0.0
                        && kinematics.tangential_speed_m_s() <= REST_SPEED_TOLERANCE_M_S,
                    dissipated_power_w: (0.0 - dot(force, kinematics.tangential_velocity_m_s))
                        .max(0.0),
                })
            }
            Self::AnchoredStiction { friction, state } => {
                let mut state = state.lock().map_err(|_| RunnerError::UnsupportedScenario {
                    what: "anchored stiction diagnostics state lock poisoned".to_owned(),
                })?;
                let dt_s = state.advance_dt_s(time_s);
                let response = friction
                    .evaluate(
                        &mut state.stiction,
                        normal_force_n,
                        kinematics.tangential_velocity_m_s,
                        dt_s,
                    )
                    .map_err(contact_build_error)?;
                let power_w = if dt_s > 0.0 {
                    response.dissipated_energy_j / dt_s
                } else {
                    (0.0 - dot(
                        response.friction_force_n,
                        kinematics.tangential_velocity_m_s,
                    ))
                    .max(0.0)
                };
                Ok(ContactFrictionDiagnostics {
                    force_n: response.friction_force_n,
                    elastic_energy_j: response.elastic_energy_j,
                    sticking: response.sticking,
                    dissipated_power_w: power_w,
                })
            }
        }
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
struct TimeGatedStictionState {
    stiction: AnchoredStictionState,
    last_time_s: Option<f64>,
}

impl TimeGatedStictionState {
    fn advance_dt_s(&mut self, time_s: f64) -> f64 {
        match self.last_time_s {
            Some(last_time_s) if time_s > last_time_s => {
                self.last_time_s = Some(time_s);
                time_s - last_time_s
            }
            Some(_) => 0.0,
            None => {
                self.last_time_s = Some(time_s);
                0.0
            }
        }
    }
}

#[derive(Clone, Copy, Debug)]
struct ContactEnergySample {
    time_s: f64,
    dissipated_power_w: f64,
    contact_power_on_vehicle_w: f64,
}

#[derive(Debug)]
pub(crate) struct ContactRunAccumulator {
    samples: u64,
    max_penetration_m: f64,
    max_normal_force_n: f64,
    effective_mass_kg: f64,
    rest_detector: RestDetector,
    rest_state: RestDetectorState,
    final_rest_status: RestStatus,
    initial_elastic_energy_j: Option<f64>,
    final_elastic_energy_j: f64,
    contact_work_on_vehicle_j: f64,
    dissipated_energy_j: f64,
    previous_energy_sample: Option<ContactEnergySample>,
    final_diagnostics: Option<ContactStepDiagnostics>,
}

impl ContactRunAccumulator {
    pub(crate) fn from_config(config: &ContactConfig) -> Result<Self, RunnerError> {
        let rest_kinetic_energy_floor_j =
            0.5 * config.effective_mass_kg * REST_SPEED_TOLERANCE_M_S * REST_SPEED_TOLERANCE_M_S;
        let rest_config = RestDetectorConfig::new(rest_kinetic_energy_floor_j, REST_HOLD_SAMPLES)
            .map_err(contact_build_error)?;
        Ok(Self {
            samples: 0,
            max_penetration_m: 0.0,
            max_normal_force_n: 0.0,
            effective_mass_kg: config.effective_mass_kg,
            rest_detector: RestDetector::new(rest_config),
            rest_state: RestDetectorState::new(),
            final_rest_status: RestStatus {
                quiet_steps: 0,
                at_rest: false,
            },
            initial_elastic_energy_j: None,
            final_elastic_energy_j: 0.0,
            contact_work_on_vehicle_j: 0.0,
            dissipated_energy_j: 0.0,
            previous_energy_sample: None,
            final_diagnostics: None,
        })
    }

    pub(crate) fn record(&mut self, time_s: f64, diagnostics: ContactStepDiagnostics) {
        self.samples += 1;
        self.max_penetration_m = self.max_penetration_m.max(diagnostics.penetration_m);
        self.max_normal_force_n = self.max_normal_force_n.max(diagnostics.normal_force_n);
        self.initial_elastic_energy_j
            .get_or_insert(diagnostics.elastic_energy_j);
        self.final_elastic_energy_j = diagnostics.elastic_energy_j;
        let sample = ContactEnergySample {
            time_s,
            dissipated_power_w: diagnostics.dissipated_power_w,
            contact_power_on_vehicle_w: diagnostics.contact_power_on_vehicle_w,
        };
        if let Some(previous) = self.previous_energy_sample {
            let dt_s = (sample.time_s - previous.time_s).max(0.0);
            self.contact_work_on_vehicle_j += 0.5
                * (previous.contact_power_on_vehicle_w + sample.contact_power_on_vehicle_w)
                * dt_s;
            self.dissipated_energy_j +=
                0.5 * (previous.dissipated_power_w + sample.dissipated_power_w) * dt_s;
        }
        self.previous_energy_sample = Some(sample);
        let kinetic_energy_j = 0.5
            * self.effective_mass_kg
            * (diagnostics.normal_velocity_m_s * diagnostics.normal_velocity_m_s
                + diagnostics.tangential_speed_m_s * diagnostics.tangential_speed_m_s);
        self.final_rest_status = self
            .rest_detector
            .update(
                &mut self.rest_state,
                kinetic_energy_j,
                diagnostics.sticking && diagnostics.penetration_m > REST_GAP_TOLERANCE_M,
            )
            .unwrap_or(RestStatus {
                quiet_steps: 0,
                at_rest: false,
            });
        self.final_diagnostics = Some(diagnostics);
    }

    pub(crate) fn finish(self) -> Result<Option<ContactRunReport>, RunnerError> {
        let Some(final_diagnostics) = self.final_diagnostics else {
            return Ok(None);
        };
        let outcome = classify_contact_outcome(
            final_diagnostics,
            self.max_penetration_m,
            self.final_rest_status,
            REST_GAP_TOLERANCE_M,
        );
        let initial_elastic_energy_j = self.initial_elastic_energy_j.unwrap_or(0.0);
        let audit = ContactEnergyAudit::new(
            initial_elastic_energy_j,
            self.final_elastic_energy_j,
            -self.contact_work_on_vehicle_j,
            self.dissipated_energy_j,
        )
        .map_err(contact_build_error)?;
        let energy = ContactRunEnergyAudit {
            initial_elastic_energy_j,
            final_elastic_energy_j: self.final_elastic_energy_j,
            contact_work_on_vehicle_j: self.contact_work_on_vehicle_j,
            dissipated_energy_j: self.dissipated_energy_j,
            closure_error_j: audit.closure_error_j(),
            relative_closure_error: audit.relative_closure_error(),
        };
        Ok(Some(ContactRunReport {
            outcome,
            samples: self.samples,
            max_penetration_m: self.max_penetration_m,
            max_normal_force_n: self.max_normal_force_n,
            final_diagnostics,
            energy,
        }))
    }
}

#[derive(Debug)]
pub(crate) struct ContactTelemetryChannels {
    gap: TelemetryChannel<f64>,
    penetration: TelemetryChannel<f64>,
    normal_velocity: TelemetryChannel<f64>,
    normal_force: TelemetryChannel<f64>,
}

impl ContactTelemetryChannels {
    pub(crate) fn new<F>(alloc: &mut F) -> Result<Self, RunnerError>
    where
        F: FnMut() -> ChannelId,
    {
        Ok(Self {
            gap: TelemetryChannel::<f64>::new(alloc(), "contact.gap_m", "m", None::<&str>)?,
            penetration: TelemetryChannel::<f64>::new(
                alloc(),
                "contact.penetration_m",
                "m",
                None::<&str>,
            )?,
            normal_velocity: TelemetryChannel::<f64>::new(
                alloc(),
                "contact.normal_velocity_m_s",
                "m/s",
                None::<&str>,
            )?,
            normal_force: TelemetryChannel::<f64>::new(
                alloc(),
                "contact.normal_force_n",
                "N",
                None::<&str>,
            )?,
        })
    }

    pub(crate) fn push_metadata(&self, channels: &mut Vec<ChannelMetadata>) {
        channels.push(self.gap.metadata().clone());
        channels.push(self.penetration.metadata().clone());
        channels.push(self.normal_velocity.metadata().clone());
        channels.push(self.normal_force.metadata().clone());
    }

    pub(crate) fn insert(
        &self,
        row: &mut TelemetryRow,
        diagnostics: ContactStepDiagnostics,
    ) -> Result<(), RunnerError> {
        row.insert(&self.gap, diagnostics.gap_m)?;
        row.insert(&self.penetration, diagnostics.penetration_m)?;
        row.insert(&self.normal_velocity, diagnostics.normal_velocity_m_s)?;
        row.insert(&self.normal_force, diagnostics.normal_force_n)?;
        Ok(())
    }
}

/// Build a half-space contact force adapter from a parsed `[contact]`
/// block.
pub(crate) fn build_half_space_contact_force_adapter(
    config: &ContactConfig,
    model_id: ModelId,
) -> Result<HalfSpaceContactForceAdapter, RunnerError> {
    let parts = build_half_space_contact_parts(config)?;
    Ok(match parts.friction {
        ContactFrictionRuntime::RegularizedCoulomb(friction) => HalfSpaceContactForceAdapter::new(
            parts.half_space,
            ContactPair::new(parts.geometry, parts.normal_law, friction),
            model_id,
        ),
        ContactFrictionRuntime::AnchoredStiction(friction) => {
            HalfSpaceContactForceAdapter::new_anchored_stiction(
                parts.half_space,
                parts.geometry,
                parts.normal_law,
                friction,
                model_id,
            )
        }
    })
}

struct HalfSpaceContactParts {
    half_space: HalfSpace,
    geometry: ContactGeometry,
    normal_law: NormalLaw,
    friction: ContactFrictionRuntime,
}

#[derive(Clone, Copy, Debug)]
enum ContactFrictionRuntime {
    RegularizedCoulomb(RegularizedCoulombFriction),
    AnchoredStiction(AnchoredStictionFriction),
}

fn build_half_space_contact_parts(
    config: &ContactConfig,
) -> Result<HalfSpaceContactParts, RunnerError> {
    let half_space =
        HalfSpace::new([0.0, 0.0, 1.0], config.ground_altitude_m).map_err(contact_build_error)?;
    let geometry = match config.geometry {
        ContactGeometryConfig::Point => ContactGeometry::Point,
        ContactGeometryConfig::Sphere => {
            ContactGeometry::sphere(require_config(config.radius_m, "contact.radius_m")?)
                .map_err(contact_build_error)?
        }
    };
    let normal_law = build_normal_law(config)?;
    let friction = build_friction_law(config)?;
    Ok(HalfSpaceContactParts {
        half_space,
        geometry,
        normal_law,
        friction,
    })
}

fn build_normal_law(config: &ContactConfig) -> Result<NormalLaw, RunnerError> {
    match config.normal_law {
        ContactNormalLawConfig::KelvinVoigt => Ok(NormalLaw::KelvinVoigt(
            KelvinVoigtNormal::new(
                require_config(config.stiffness_n_m, "contact.stiffness_n_m")?,
                config.damping_n_s_m.unwrap_or(0.0),
            )
            .map_err(contact_build_error)?,
        )),
        ContactNormalLawConfig::Hertz => Ok(NormalLaw::Hertz(
            HertzNormal::new(require_config(
                config.stiffness_n_m_3_2,
                "contact.stiffness_n_m_3_2",
            )?)
            .map_err(contact_build_error)?,
        )),
        ContactNormalLawConfig::HuntCrossley => {
            let stiffness_n_m_3_2 =
                require_config(config.stiffness_n_m_3_2, "contact.stiffness_n_m_3_2")?;
            let law = if let Some(damping_factor_s_m) = config.damping_factor_s_m {
                HuntCrossleyNormal::new(stiffness_n_m_3_2, damping_factor_s_m)
            } else {
                HuntCrossleyNormal::from_restitution(
                    stiffness_n_m_3_2,
                    require_config(config.restitution, "contact.restitution")?,
                    require_config(
                        config.reference_impact_speed_m_s,
                        "contact.reference_impact_speed_m_s",
                    )?,
                )
            }
            .map_err(contact_build_error)?;
            Ok(NormalLaw::HuntCrossley(law))
        }
    }
}

fn build_friction_law(config: &ContactConfig) -> Result<ContactFrictionRuntime, RunnerError> {
    match config.friction_law {
        ContactFrictionLawConfig::RegularizedCoulomb => {
            Ok(ContactFrictionRuntime::RegularizedCoulomb(
                RegularizedCoulombFriction::new(
                    config.friction_coefficient,
                    config.friction_regularization_speed_m_s(),
                )
                .map_err(contact_build_error)?,
            ))
        }
        ContactFrictionLawConfig::AnchoredStiction => Ok(ContactFrictionRuntime::AnchoredStiction(
            AnchoredStictionFriction::new(
                require_config(
                    config.static_friction_coefficient,
                    "contact.static_friction_coefficient",
                )?,
                require_config(
                    config.kinetic_friction_coefficient,
                    "contact.kinetic_friction_coefficient",
                )?,
                require_config(
                    config.tangential_stiffness_n_m,
                    "contact.tangential_stiffness_n_m",
                )?,
                config.tangential_damping_n_s_m.unwrap_or(0.0),
                require_config(config.restick_speed_m_s, "contact.restick_speed_m_s")?,
            )
            .map_err(contact_build_error)?,
        )),
    }
}

fn require_config(value: Option<f64>, field: &'static str) -> Result<f64, RunnerError> {
    value.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("{field} is required by contact force construction"),
    })
}

fn contact_build_error(err: ContactError) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("invalid [contact] block: {err}"),
    }
}

fn dot(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn scale(a: [f64; 3], scalar: f64) -> [f64; 3] {
    [a[0] * scalar, a[1] * scalar, a[2] * scalar]
}

fn add(a: [f64; 3], b: [f64; 3]) -> [f64; 3] {
    [a[0] + b[0], a[1] + b[1], a[2] + b[2]]
}

fn classify_contact_outcome(
    final_diagnostics: ContactStepDiagnostics,
    max_penetration_m: f64,
    final_rest_status: RestStatus,
    gap_tolerance_m: f64,
) -> ContactOutcomeKind {
    if max_penetration_m <= gap_tolerance_m && final_diagnostics.gap_m >= -gap_tolerance_m {
        ContactOutcomeKind::NoContact
    } else if final_diagnostics.penetration_m > gap_tolerance_m && final_rest_status.at_rest {
        ContactOutcomeKind::Rest
    } else {
        ContactOutcomeKind::Unsettled
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn contact_outcome_classifier_distinguishes_no_contact_rest_and_unsettled() {
        let no_contact = classify_contact_outcome(
            ContactStepDiagnostics {
                gap_m: 1.0,
                penetration_m: 0.0,
                normal_velocity_m_s: 0.0,
                tangential_speed_m_s: 0.0,
                normal_force_n: 0.0,
                elastic_energy_j: 0.0,
                sticking: false,
                dissipated_power_w: 0.0,
                contact_power_on_vehicle_w: 0.0,
            },
            0.0,
            RestStatus {
                quiet_steps: 0,
                at_rest: false,
            },
            REST_GAP_TOLERANCE_M,
        );
        assert_eq!(no_contact, ContactOutcomeKind::NoContact);

        let rest = classify_contact_outcome(
            ContactStepDiagnostics {
                gap_m: -0.01,
                penetration_m: 0.01,
                normal_velocity_m_s: 0.0,
                tangential_speed_m_s: 0.0,
                normal_force_n: 20.0,
                elastic_energy_j: 0.1,
                sticking: true,
                dissipated_power_w: 0.0,
                contact_power_on_vehicle_w: 0.0,
            },
            0.01,
            RestStatus {
                quiet_steps: REST_HOLD_SAMPLES,
                at_rest: true,
            },
            REST_GAP_TOLERANCE_M,
        );
        assert_eq!(rest, ContactOutcomeKind::Rest);

        let unsettled = classify_contact_outcome(
            ContactStepDiagnostics {
                gap_m: -0.01,
                penetration_m: 0.01,
                normal_velocity_m_s: 1.0e-3,
                tangential_speed_m_s: 0.0,
                normal_force_n: 20.0,
                elastic_energy_j: 0.1,
                sticking: true,
                dissipated_power_w: 0.0,
                contact_power_on_vehicle_w: 0.02,
            },
            0.01,
            RestStatus {
                quiet_steps: 0,
                at_rest: false,
            },
            REST_GAP_TOLERANCE_M,
        );
        assert_eq!(unsettled, ContactOutcomeKind::Unsettled);
    }

    #[test]
    fn contact_diagnostics_evaluator_matches_kelvin_voigt_golden_row() {
        let config = ContactConfig {
            kind: "half_space".to_owned(),
            ground_altitude_m: 0.0,
            geometry: ContactGeometryConfig::Point,
            radius_m: None,
            normal_law: ContactNormalLawConfig::KelvinVoigt,
            stiffness_n_m: Some(2000.0),
            stiffness_n_m_3_2: None,
            damping_n_s_m: Some(0.0),
            damping_factor_s_m: None,
            restitution: None,
            reference_impact_speed_m_s: None,
            stability_stiffness_n_m: None,
            friction_coefficient: 0.0,
            friction_law: ContactFrictionLawConfig::RegularizedCoulomb,
            friction_regularization_speed_m_s: None,
            static_friction_coefficient: None,
            kinetic_friction_coefficient: None,
            tangential_stiffness_n_m: None,
            tangential_damping_n_s_m: None,
            restick_speed_m_s: None,
            effective_mass_kg: 1.0,
            substeps: 1,
        };
        let diagnostics = ContactDiagnosticsEvaluator::from_config(&config)
            .unwrap()
            .diagnostics_from_state_vectors(
                nalgebra::Vector3::new(0.0, 0.0, -0.01),
                nalgebra::Vector3::zeros(),
                0.0,
            )
            .unwrap();

        assert_eq!(diagnostics.gap_m.to_bits(), (-0.01_f64).to_bits());
        assert_eq!(diagnostics.penetration_m.to_bits(), 0.01_f64.to_bits());
        assert_eq!(diagnostics.normal_velocity_m_s.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            diagnostics.tangential_speed_m_s.to_bits(),
            0.0_f64.to_bits()
        );
        assert!((diagnostics.normal_force_n - 20.0).abs() <= 1.0e-12);
        assert!((diagnostics.elastic_energy_j - 0.1).abs() <= 1.0e-15);
        assert!(diagnostics.sticking);
        assert_eq!(diagnostics.dissipated_power_w.to_bits(), 0.0_f64.to_bits());
        assert_eq!(
            diagnostics.contact_power_on_vehicle_w.to_bits(),
            0.0_f64.to_bits()
        );
    }

    #[test]
    fn contact_diagnostics_anchored_stiction_time_gates_static_anchor_energy() {
        let config = ContactConfig {
            kind: "half_space".to_owned(),
            ground_altitude_m: 0.0,
            geometry: ContactGeometryConfig::Point,
            radius_m: None,
            normal_law: ContactNormalLawConfig::KelvinVoigt,
            stiffness_n_m: Some(2000.0),
            stiffness_n_m_3_2: None,
            damping_n_s_m: Some(0.0),
            damping_factor_s_m: None,
            restitution: None,
            reference_impact_speed_m_s: None,
            stability_stiffness_n_m: None,
            friction_coefficient: 0.0,
            friction_law: ContactFrictionLawConfig::AnchoredStiction,
            friction_regularization_speed_m_s: None,
            static_friction_coefficient: Some(1.0),
            kinetic_friction_coefficient: Some(0.5),
            tangential_stiffness_n_m: Some(1000.0),
            tangential_damping_n_s_m: Some(0.0),
            restick_speed_m_s: Some(0.01),
            effective_mass_kg: 1.0,
            substeps: 1,
        };
        let evaluator = ContactDiagnosticsEvaluator::from_config(&config).unwrap();
        let position = nalgebra::Vector3::new(0.0, 0.0, -0.01);
        let velocity = nalgebra::Vector3::new(0.001, 0.0, 0.0);

        let first = evaluator
            .diagnostics_from_state_vectors(position, velocity, 0.0)
            .unwrap();
        let second = evaluator
            .diagnostics_from_state_vectors(position, velocity, 0.001)
            .unwrap();
        let repeated = evaluator
            .diagnostics_from_state_vectors(position, velocity, 0.001)
            .unwrap();

        assert!(first.sticking);
        assert!(second.sticking);
        assert!(repeated.sticking);
        assert_eq!(first.tangential_speed_m_s.to_bits(), 0.001_f64.to_bits());
        assert!((first.elastic_energy_j - 0.1).abs() <= 1.0e-15);
        assert!((second.elastic_energy_j - 0.100_000_000_5).abs() <= 1.0e-15);
        assert_eq!(
            second.elastic_energy_j.to_bits(),
            repeated.elastic_energy_j.to_bits()
        );
        assert!(second.contact_power_on_vehicle_w < 0.0);
        assert_eq!(second.dissipated_power_w.to_bits(), 0.0_f64.to_bits());
    }
}
