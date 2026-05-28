//! Runner-side atmosphere dispatch.
//!
//! Wraps the concrete atmosphere models that the runners support
//! ([`UsStandard1976`] for the historical sounding-rocket / drop-test
//! scenarios, [`PiecewiseExponentialAtmosphere`] for
//! engineering LEO drag estimation) so the rest of the runner code
//! can hold a single concrete type instead of being generic over
//! [`AtmosphereModel`].
//!
//! The enum dispatch keeps the legacy `us_standard_1976` in-envelope
//! contract intact and uses the explicit
//! `ZeroDensityAboveCeiling` exoatmospheric policy for runner calls
//! above the 86 km USSA76 implementation ceiling.

use std::borrow::Cow;

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{Ecef, ModelId, Ned, SimTime, Velocity3};
use openbmp_physics::{
    AtmosphereModel, AtmosphereSample, ExoatmosphericPolicy, FrameContext, FrameProfile,
    Nrlmsis2Compat, Nrlmsise00Full, Nrlmsise00Inputs, PhysicsError, PiecewiseExponentialAtmosphere,
    UsStandard1976,
};
use openbmp_scenario::{AtmosphereConfig, ScenarioDocument};
use openbmp_sim::{EnvironmentModel, EnvironmentQuery, EnvironmentSample, ModelEvalError};

use crate::error::RunnerError;

const RUNNER_ENVIRONMENT_MODEL_ID: ModelId = ModelId::new(900);

/// Atmosphere model selected by the scenario.
#[derive(Copy, Clone, Debug)]
pub enum RuntimeAtmosphere {
    /// `us_standard_1976` — layered model (0-86 km).
    UsStandard1976(UsStandard1976),
    /// `piecewise_exponential` — engineering layered
    /// exponential model (0-1000 km).
    PiecewiseExponential(PiecewiseExponentialAtmosphere),
    /// `nrlmsise00` — full empirical coefficient model (0-1000 km).
    Nrlmsise00(Nrlmsise00Full),
    /// `nrlmsis2_compat` — OpenBMP compatibility profile for
    /// NRLMSIS-2-family scenario selectors.
    Nrlmsis2Compat(Nrlmsis2Compat),
}

/// Kernel environment wrapper used for event-scalar evaluation.
///
/// Force models still own their atmosphere instances. This wrapper
/// exposes the same scenario atmosphere density through
/// [`EnvironmentSample`] so kernel-owned mission triggers can compute
/// dynamic pressure without reaching into force-model internals.
#[derive(Clone, Debug, Default)]
pub struct RuntimeEnvironment {
    atmosphere: Option<RuntimeAtmosphere>,
    frame: FrameContext,
}

impl RuntimeEnvironment {
    /// Build a runtime environment from the scenario atmosphere when
    /// the runner supports that atmosphere kind. Unsupported or
    /// `none` atmospheres preserve the legacy null-environment
    /// density of zero.
    ///
    /// # Errors
    ///
    /// Returns [`RunnerError`] when the selected runner atmosphere is
    /// supported but model-specific construction fails.
    pub fn from_document(
        document: &ScenarioDocument,
        frame: &FrameContext,
    ) -> Result<Self, RunnerError> {
        validate_wind_frame(document, frame)?;
        let atmosphere_kind = scenario_atmosphere_kind(document);
        let atmosphere = if is_runtime_atmosphere_kind(atmosphere_kind) {
            Some(build_document_runtime_atmosphere(document)?)
        } else {
            None
        };
        Ok(Self {
            atmosphere,
            frame: frame.clone(),
        })
    }
}

impl EnvironmentModel for RuntimeEnvironment {
    fn sample(&self, query: EnvironmentQuery) -> Result<EnvironmentSample, ModelEvalError> {
        let mut sample = EnvironmentSample::default();
        populate_frame_motion(&self.frame, query, &mut sample)?;
        let Some(atmosphere) = &self.atmosphere else {
            return Ok(sample);
        };
        let altitude_m = query.position_eci.vector.z.max(0.0);
        let atmosphere_sample = atmosphere.sample(altitude_m, query.time).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: RUNNER_ENVIRONMENT_MODEL_ID,
                reason: Cow::Borrowed("atmosphere out of envelope at altitude"),
            }
        })?;
        sample.atmosphere_density_kg_m3 = atmosphere_sample.density_kg_m3;
        Ok(sample)
    }
}

fn populate_frame_motion(
    frame: &FrameContext,
    query: EnvironmentQuery,
    sample: &mut EnvironmentSample,
) -> Result<(), ModelEvalError> {
    let p_ecef = frame.eci_to_ecef_position(query.time, query.position_eci);
    let still_air_eci = frame
        .ecef_to_eci_velocity(query.time, Velocity3::<Ecef>::zero(), p_ecef)
        .vector;
    sample.atmosphere_velocity_eci_m_s = still_air_eci;
    sample.wind_ned_to_eci = wind_ned_to_eci_matrix(frame, query.time, p_ecef, still_air_eci)?;
    Ok(())
}

fn wind_ned_to_eci_matrix(
    frame: &FrameContext,
    time: SimTime,
    p_ecef: openbmp_core::Position3<Ecef>,
    still_air_eci_m_s: Vector3<f64>,
) -> Result<Matrix3<f64>, ModelEvalError> {
    if frame.profile() == FrameProfile::ToyFixedEarth {
        return Ok(Matrix3::from_columns(&[
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(0.0, 0.0, -1.0),
        ]));
    }
    if frame.local_origin().is_none() {
        return Ok(Matrix3::zeros());
    }

    let basis = [
        Velocity3::<Ned>::new(1.0, 0.0, 0.0),
        Velocity3::<Ned>::new(0.0, 1.0, 0.0),
        Velocity3::<Ned>::new(0.0, 0.0, 1.0),
    ];
    let mut matrix = Matrix3::zeros();
    for (column, wind_ned) in basis.into_iter().enumerate() {
        let wind_ecef =
            frame
                .ned_to_ecef_velocity(wind_ned)
                .map_err(|_| ModelEvalError::InvalidState {
                    model: RUNNER_ENVIRONMENT_MODEL_ID,
                    reason: Cow::Borrowed("NED wind requires a local geodetic origin"),
                })?;
        let wind_eci =
            frame.ecef_to_eci_velocity(time, wind_ecef, p_ecef).vector - still_air_eci_m_s;
        matrix.set_column(column, &wind_eci);
    }
    Ok(matrix)
}

fn validate_wind_frame(
    document: &ScenarioDocument,
    frame: &FrameContext,
) -> Result<(), RunnerError> {
    let wind_active = document
        .wind
        .as_ref()
        .is_some_and(|wind| wind.kind.as_str() != "none");
    if wind_active
        && frame.profile() != FrameProfile::ToyFixedEarth
        && frame.local_origin().is_none()
    {
        return Err(RunnerError::UnsupportedScenario {
            what: "[wind] with a rotating Earth frame requires [frames.local_origin] so NED wind can be transformed to ECI for air-relative aerodynamics"
                .to_owned(),
        });
    }
    Ok(())
}

impl AtmosphereModel for RuntimeAtmosphere {
    fn sample(
        &self,
        altitude_geometric_m: f64,
        time: SimTime,
    ) -> Result<AtmosphereSample, PhysicsError> {
        match self {
            Self::UsStandard1976(a) => a.sample(altitude_geometric_m, time),
            Self::PiecewiseExponential(a) => a.sample(altitude_geometric_m, time),
            Self::Nrlmsise00(a) => a.sample(altitude_geometric_m, time),
            Self::Nrlmsis2Compat(a) => a.sample(altitude_geometric_m, time),
        }
    }
}

/// Atmosphere kind names that the runner can construct.
const SUPPORTED_ATMOSPHERE_KINDS: &[&str] = &[
    "us_standard_1976",
    "piecewise_exponential",
    "nrlmsise00",
    "nrlmsis2_compat",
];

/// Resolve the atmosphere kind named by the scenario into a runtime
/// dispatch enum. Returns [`RunnerError::UnsupportedScenario`] for kinds
/// the runner does not yet wire.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] when `atmosphere_kind`
/// is not in `SUPPORTED_ATMOSPHERE_KINDS`.
pub fn build_runtime_atmosphere(atmosphere_kind: &str) -> Result<RuntimeAtmosphere, RunnerError> {
    build_runtime_atmosphere_with_config(atmosphere_kind, None)
}

/// Resolve a scenario document into a runtime atmosphere.
///
/// This preserves the structured `[atmosphere]` block's model-specific
/// inputs for MSIS-family models; callers with only a legacy kind
/// string continue to get deterministic mid-condition defaults.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] when the selected kind
/// is not wired or model-specific inputs fail validation.
pub fn build_document_runtime_atmosphere(
    document: &ScenarioDocument,
) -> Result<RuntimeAtmosphere, RunnerError> {
    build_runtime_atmosphere_with_config(
        scenario_atmosphere_kind(document),
        document.atmosphere.as_ref(),
    )
}

fn build_runtime_atmosphere_with_config(
    atmosphere_kind: &str,
    atmosphere: Option<&AtmosphereConfig>,
) -> Result<RuntimeAtmosphere, RunnerError> {
    match atmosphere_kind {
        "us_standard_1976" => Ok(RuntimeAtmosphere::UsStandard1976(
            UsStandard1976::with_exoatmospheric_policy(
                ExoatmosphericPolicy::ZeroDensityAboveCeiling,
            ),
        )),
        "piecewise_exponential" => Ok(RuntimeAtmosphere::PiecewiseExponential(
            PiecewiseExponentialAtmosphere::new(),
        )),
        "nrlmsise00" => {
            let inputs = msis_inputs("nrlmsise00", atmosphere)?;
            let model =
                Nrlmsise00Full::new(inputs).map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("Nrlmsise00Full construction failed: {err}"),
                })?;
            Ok(RuntimeAtmosphere::Nrlmsise00(model))
        }
        "nrlmsis2_compat" => {
            let inputs = msis_inputs("nrlmsis2_compat", atmosphere)?;
            let model =
                Nrlmsis2Compat::new(inputs).map_err(|err| RunnerError::UnsupportedScenario {
                    what: format!("Nrlmsis2Compat construction failed: {err}"),
                })?;
            Ok(RuntimeAtmosphere::Nrlmsis2Compat(model))
        }
        other => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{other}` is not wired (supported: {})",
                SUPPORTED_ATMOSPHERE_KINDS.join(", ")
            ),
        }),
    }
}

fn msis_inputs(
    model_kind: &str,
    atmosphere: Option<&AtmosphereConfig>,
) -> Result<Nrlmsise00Inputs, RunnerError> {
    let Some(atmosphere) = atmosphere else {
        return Ok(Nrlmsise00Inputs::mid_conditions(0.0));
    };
    let mut inputs = Nrlmsise00Inputs::mid_conditions(0.0);
    if let Some(year) = atmosphere.year {
        inputs.year = year;
    }
    if let Some(day_of_year) = atmosphere.day_of_year {
        inputs.day_of_year = day_of_year;
    }
    if let Some(utc_s) = atmosphere.utc_s {
        inputs.utc_seconds = utc_s;
    }
    if let Some(latitude_deg) = atmosphere.latitude_deg {
        inputs.latitude_rad = latitude_deg.to_radians();
    }
    if let Some(longitude_deg) = atmosphere.longitude_deg {
        inputs.longitude_rad = longitude_deg.to_radians();
    }
    if let Some(local_solar_time) = atmosphere.local_apparent_solar_time_h {
        inputs.local_apparent_solar_time_hours = local_solar_time;
    }
    if let Some(f107_average) = atmosphere.f107_average_81day_sfu {
        inputs.f107_average_81day = f107_average;
    }
    if let Some(f107_yesterday_sfu) = atmosphere.f107_yesterday_sfu {
        inputs.f107_yesterday = f107_yesterday_sfu;
    }
    if let Some(ap_average) = atmosphere.ap_average {
        inputs.ap_average = ap_average;
    }
    inputs
        .validate_full_path()
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("invalid {model_kind} atmosphere inputs: {err}"),
        })?;
    Ok(inputs)
}

/// Whether `kind` is one of the layered atmospheres the runner
/// supports for aero / recovery drag and for telemetry sampling.
#[must_use]
pub fn is_runtime_atmosphere_kind(kind: &str) -> bool {
    SUPPORTED_ATMOSPHERE_KINDS.contains(&kind)
}

/// Resolve the atmosphere kind named in either the structured
/// `[atmosphere]` block or the legacy `environment.atmosphere`
/// selector — the structured kind wins when both are present
/// (`EnvironmentConfig::validate` rejects disagreement).
#[must_use]
pub fn scenario_atmosphere_kind(document: &ScenarioDocument) -> &str {
    document
        .atmosphere
        .as_ref()
        .map_or(document.environment.atmosphere.as_str(), |a| {
            a.kind.as_str()
        })
}

#[cfg(test)]
mod tests {
    use super::*;
    use openbmp_core::Position3;
    use openbmp_physics::{LocalGeodeticOrigin, WGS84_A_M, WGS84_OMEGA_RAD_S};

    #[test]
    fn wgs84_environment_reports_corotating_still_air_velocity() {
        let environment = RuntimeEnvironment {
            atmosphere: None,
            frame: FrameContext::wgs84_uniform_rotation(None),
        };
        let sample = environment
            .sample(EnvironmentQuery {
                time: SimTime::ZERO,
                position_eci: Position3::new(WGS84_A_M, 0.0, 0.0),
            })
            .unwrap();

        let expected = WGS84_OMEGA_RAD_S * WGS84_A_M;
        assert!(sample.atmosphere_velocity_eci_m_s.x.abs() < 1.0e-12);
        assert!((sample.atmosphere_velocity_eci_m_s.y - expected).abs() < 1.0e-9);
        assert!(sample.atmosphere_velocity_eci_m_s.z.abs() < 1.0e-12);
    }

    #[test]
    fn wgs84_environment_maps_ned_wind_to_eci_relative_to_still_air() {
        let origin = LocalGeodeticOrigin::new_degrees(0.0, 0.0, 0.0).unwrap();
        let environment = RuntimeEnvironment {
            atmosphere: None,
            frame: FrameContext::wgs84_uniform_rotation(Some(origin)),
        };
        let sample = environment
            .sample(EnvironmentQuery {
                time: SimTime::ZERO,
                position_eci: Position3::new(WGS84_A_M, 0.0, 0.0),
            })
            .unwrap();

        let mut with_wind = sample;
        with_wind.set_wind_ned_m_s(Vector3::new(0.0, 12.0, 0.0));
        assert!(with_wind.wind_eci_m_s.x.abs() < 1.0e-12);
        assert!((with_wind.wind_eci_m_s.y - 12.0).abs() < 1.0e-12);
        assert!(with_wind.wind_eci_m_s.z.abs() < 1.0e-12);
    }

    #[test]
    fn toy_environment_keeps_flat_ned_mapping() {
        let environment = RuntimeEnvironment {
            atmosphere: None,
            frame: FrameContext::toy_fixed_earth(),
        };
        let mut sample = environment
            .sample(EnvironmentQuery {
                time: SimTime::ZERO,
                position_eci: Position3::origin(),
            })
            .unwrap();

        sample.set_wind_ned_m_s(Vector3::new(1.0, 2.0, 3.0));
        assert_eq!(sample.atmosphere_velocity_eci_m_s, Vector3::zeros());
        assert_eq!(sample.wind_eci_m_s, Vector3::new(1.0, 2.0, -3.0));
    }
}
