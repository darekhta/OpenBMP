//! Runner-side construction of compliant contact force adapters.

use openbmp_contact::{
    ContactError, ContactGeometry, ContactPair, HalfSpace, HertzNormal, HuntCrossleyNormal,
    KelvinVoigtNormal, NormalLaw, RegularizedCoulombFriction,
};
use openbmp_core::ModelId;
use openbmp_scenario::{ContactConfig, ContactGeometryConfig, ContactNormalLawConfig};
use openbmp_vehicle::HalfSpaceContactForceAdapter;

use crate::RunnerError;

/// Build a half-space contact force adapter from a parsed `[contact]`
/// block.
pub(crate) fn build_half_space_contact_force_adapter(
    config: &ContactConfig,
    model_id: ModelId,
) -> Result<HalfSpaceContactForceAdapter, RunnerError> {
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
    Ok(HalfSpaceContactForceAdapter::new(
        half_space,
        ContactPair::new(geometry, normal_law, friction),
        model_id,
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
