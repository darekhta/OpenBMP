//! Runner-side construction, diagnostics, and reports for compliant contact.

use openbmp_contact::{
    ContactError, ContactGeometry, ContactPair, HalfSpace, HertzNormal, HuntCrossleyNormal,
    KelvinVoigtNormal, NormalLaw, RegularizedCoulombFriction, half_space_kinematics,
};
use openbmp_core::{ChannelId, ModelId};
use openbmp_scenario::{ContactConfig, ContactGeometryConfig, ContactNormalLawConfig};
use openbmp_telemetry::{ChannelMetadata, TelemetryChannel, TelemetryRow};
use openbmp_vehicle::HalfSpaceContactForceAdapter;

use crate::RunnerError;

const REST_GAP_TOLERANCE_M: f64 = 1.0e-9;
const REST_SPEED_TOLERANCE_M_S: f64 = 1.0e-9;

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
    /// Non-negative scalar normal force in newtons.
    pub normal_force_n: f64,
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
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ContactDiagnosticsEvaluator {
    half_space: HalfSpace,
    geometry: ContactGeometry,
    pair: ContactPair,
}

impl ContactDiagnosticsEvaluator {
    pub(crate) fn from_config(config: &ContactConfig) -> Result<Self, RunnerError> {
        let (half_space, geometry, pair) = build_half_space_contact_parts(config)?;
        Ok(Self {
            half_space,
            geometry,
            pair,
        })
    }

    pub(crate) fn diagnostics_from_state_vectors(
        &self,
        position_m: nalgebra::Vector3<f64>,
        velocity_m_s: nalgebra::Vector3<f64>,
    ) -> Result<ContactStepDiagnostics, RunnerError> {
        let position = [position_m.x, position_m.y, position_m.z];
        let velocity = [velocity_m_s.x, velocity_m_s.y, velocity_m_s.z];
        let kinematics = half_space_kinematics(self.half_space, self.geometry, position, velocity)
            .map_err(contact_build_error)?;
        let force = self
            .pair
            .evaluate_half_space(self.half_space, position, velocity)
            .map_err(contact_build_error)?;
        Ok(ContactStepDiagnostics {
            gap_m: kinematics.gap_m,
            penetration_m: kinematics.penetration_m(),
            normal_velocity_m_s: kinematics.normal_velocity_m_s,
            normal_force_n: force.normal_force_n,
        })
    }
}

#[derive(Debug, Default)]
pub(crate) struct ContactRunAccumulator {
    samples: u64,
    max_penetration_m: f64,
    max_normal_force_n: f64,
    final_diagnostics: Option<ContactStepDiagnostics>,
}

impl ContactRunAccumulator {
    pub(crate) fn record(&mut self, diagnostics: ContactStepDiagnostics) {
        self.samples += 1;
        self.max_penetration_m = self.max_penetration_m.max(diagnostics.penetration_m);
        self.max_normal_force_n = self.max_normal_force_n.max(diagnostics.normal_force_n);
        self.final_diagnostics = Some(diagnostics);
    }

    pub(crate) fn finish(self) -> Option<ContactRunReport> {
        let final_diagnostics = self.final_diagnostics?;
        let outcome = classify_contact_outcome(
            final_diagnostics,
            self.max_penetration_m,
            REST_GAP_TOLERANCE_M,
            REST_SPEED_TOLERANCE_M_S,
        );
        Some(ContactRunReport {
            outcome,
            samples: self.samples,
            max_penetration_m: self.max_penetration_m,
            max_normal_force_n: self.max_normal_force_n,
            final_diagnostics,
        })
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
    let (half_space, _, pair) = build_half_space_contact_parts(config)?;
    Ok(HalfSpaceContactForceAdapter::new(
        half_space, pair, model_id,
    ))
}

fn build_half_space_contact_parts(
    config: &ContactConfig,
) -> Result<(HalfSpace, ContactGeometry, ContactPair), RunnerError> {
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
    let friction = RegularizedCoulombFriction::new(
        config.friction_coefficient,
        config.friction_regularization_speed_m_s(),
    )
    .map_err(contact_build_error)?;
    Ok((
        half_space,
        geometry,
        ContactPair::new(geometry, normal_law, friction),
    ))
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

fn classify_contact_outcome(
    final_diagnostics: ContactStepDiagnostics,
    max_penetration_m: f64,
    gap_tolerance_m: f64,
    speed_tolerance_m_s: f64,
) -> ContactOutcomeKind {
    if max_penetration_m <= gap_tolerance_m && final_diagnostics.gap_m >= -gap_tolerance_m {
        ContactOutcomeKind::NoContact
    } else if final_diagnostics.penetration_m > gap_tolerance_m
        && final_diagnostics.normal_velocity_m_s.abs() <= speed_tolerance_m_s
    {
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
                normal_force_n: 0.0,
            },
            0.0,
            REST_GAP_TOLERANCE_M,
            REST_SPEED_TOLERANCE_M_S,
        );
        assert_eq!(no_contact, ContactOutcomeKind::NoContact);

        let rest = classify_contact_outcome(
            ContactStepDiagnostics {
                gap_m: -0.01,
                penetration_m: 0.01,
                normal_velocity_m_s: 0.0,
                normal_force_n: 20.0,
            },
            0.01,
            REST_GAP_TOLERANCE_M,
            REST_SPEED_TOLERANCE_M_S,
        );
        assert_eq!(rest, ContactOutcomeKind::Rest);

        let unsettled = classify_contact_outcome(
            ContactStepDiagnostics {
                gap_m: -0.01,
                penetration_m: 0.01,
                normal_velocity_m_s: 1.0e-3,
                normal_force_n: 20.0,
            },
            0.01,
            REST_GAP_TOLERANCE_M,
            REST_SPEED_TOLERANCE_M_S,
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
            friction_regularization_speed_m_s: None,
            effective_mass_kg: 1.0,
            substeps: 1,
        };
        let diagnostics = ContactDiagnosticsEvaluator::from_config(&config)
            .unwrap()
            .diagnostics_from_state_vectors(
                nalgebra::Vector3::new(0.0, 0.0, -0.01),
                nalgebra::Vector3::zeros(),
            )
            .unwrap();

        assert_eq!(diagnostics.gap_m.to_bits(), (-0.01_f64).to_bits());
        assert_eq!(diagnostics.penetration_m.to_bits(), 0.01_f64.to_bits());
        assert_eq!(diagnostics.normal_velocity_m_s.to_bits(), 0.0_f64.to_bits());
        assert!((diagnostics.normal_force_n - 20.0).abs() <= 1.0e-12);
    }
}
