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
use std::collections::BTreeMap;

use nalgebra::{Matrix3, Vector3};
use openbmp_core::{Ecef, ModelId, Ned, SimTime, Velocity3};
use openbmp_physics::{
    AtmosphereModel, AtmosphereSample, ConstantGravity, ExoatmosphericPolicy, FrameContext,
    FrameProfile, GravityModel, J2Gravity, Nrlmsis2Compat, Nrlmsise00Full, Nrlmsise00Inputs,
    PhysicsError, PiecewiseExponentialAtmosphere, PointMassGravity, UsStandard1976, WGS84_J2,
};
use openbmp_scenario::{AtmosphereConfig, ResolvedFile, ScenarioDocument};
use openbmp_sim::{EnvironmentModel, EnvironmentQuery, EnvironmentSample, ModelEvalError};

use crate::error::RunnerError;

const RUNNER_ENVIRONMENT_MODEL_ID: ModelId = ModelId::new(900);
const RUNNER_ENVIRONMENT_GRAVITY_MODEL_ID: ModelId = ModelId::new(901);

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

/// Gravity model selected by the scenario for environment sampling.
#[derive(Clone, Debug)]
pub(crate) enum RuntimeGravity {
    /// Constant ECI -z gravity.
    Constant(ConstantGravity),
    /// Point-mass central gravity.
    PointMass(PointMassGravity),
    /// J2 central gravity.
    J2(J2Gravity),
    /// Pinned or coefficient-file EGM2008 gravity.
    Egm2008(crate::celestial::RuntimeEgm2008Gravity),
    /// Frame-coupled degree-2 tesseral/sectoral Earth gravity.
    Tesseral(openbmp_physics::EarthFixedGravity<openbmp_physics::TesseralGravity>),
    /// Central gravity plus configured third-body perturbations.
    ThirdBody(crate::celestial::RuntimeThirdBodyGravity),
}

impl GravityModel for RuntimeGravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: openbmp_core::Position3<openbmp_core::Eci>,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        match self {
            Self::Constant(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::PointMass(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::J2(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::Egm2008(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::Tesseral(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::ThirdBody(model) => model.gravity_eci_m_s2(position_eci, time),
        }
    }
}

/// Kernel environment wrapper used for event-scalar evaluation.
///
/// Force models still own their atmosphere instances. This wrapper
/// exposes the same scenario atmosphere density through
/// [`EnvironmentSample`] so kernel-owned mission triggers can compute
/// dynamic pressure without reaching into force-model internals.
#[derive(Clone, Debug)]
pub struct RuntimeEnvironment {
    atmosphere: Option<RuntimeAtmosphere>,
    gravity: RuntimeGravity,
    frame: FrameContext,
    geocentric_surface_radius_m: Option<f64>,
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
        resolved_files: &BTreeMap<String, ResolvedFile>,
        frame: &FrameContext,
    ) -> Result<Self, RunnerError> {
        validate_wind_frame(document, frame)?;
        let atmosphere_kind = scenario_atmosphere_kind(document);
        let atmosphere = if is_runtime_atmosphere_kind(atmosphere_kind) {
            Some(build_document_runtime_atmosphere(document)?)
        } else {
            None
        };
        let gravity = build_document_runtime_gravity(document, resolved_files)?;
        Ok(Self {
            atmosphere,
            gravity,
            frame: frame.clone(),
            geocentric_surface_radius_m: document_geocentric_surface_radius_m(document),
        })
    }
}

impl EnvironmentModel for RuntimeEnvironment {
    fn sample(&self, query: EnvironmentQuery) -> Result<EnvironmentSample, ModelEvalError> {
        let mut sample = EnvironmentSample {
            gravity_eci_m_s2: self
                .gravity
                .gravity_eci_m_s2(query.position_eci, query.time)
                .map_err(|_| ModelEvalError::OutOfEnvelope {
                    model: RUNNER_ENVIRONMENT_GRAVITY_MODEL_ID,
                    reason: Cow::Borrowed("gravity model out of envelope"),
                })?,
            ..EnvironmentSample::default()
        };
        populate_frame_motion(&self.frame, query, &mut sample)?;
        let Some(atmosphere) = &self.atmosphere else {
            return Ok(sample);
        };
        let altitude_m = atmosphere_altitude_m_with_surface_radius(
            query.position_eci.vector,
            self.geocentric_surface_radius_m,
        );
        let atmosphere_sample = atmosphere.sample(altitude_m, query.time).map_err(|_| {
            ModelEvalError::OutOfEnvelope {
                model: RUNNER_ENVIRONMENT_MODEL_ID,
                reason: Cow::Borrowed("atmosphere out of envelope at altitude"),
            }
        })?;
        sample.atmosphere_density_kg_m3 = atmosphere_sample.density_kg_m3;
        sample.atmosphere_pressure_pa = atmosphere_sample.pressure_pa;
        Ok(sample)
    }
}

/// Resolve the scenario gravity block into the runtime environment gravity
/// dispatcher used to populate [`EnvironmentSample::gravity_eci_m_s2`].
///
/// # Errors
///
/// Returns [`RunnerError`] when the scenario's gravity selector or required
/// fields cannot be constructed by the runner.
pub(crate) fn build_document_runtime_gravity(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RuntimeGravity, RunnerError> {
    match document.environment.gravity.as_str() {
        "constant" => {
            let g = document.environment.gravity_m_s2.ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: "environment.gravity_m_s2 missing for constant gravity".to_owned(),
                }
            })?;
            if g < 0.0 {
                return Err(RunnerError::UnsupportedScenario {
                    what: "environment.gravity_m_s2 must be a non-negative magnitude; \
                         constant gravity is -z in ECI"
                        .to_owned(),
                });
            }
            Ok(RuntimeGravity::Constant(ConstantGravity::down_z(g)?))
        }
        "point_mass" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for point_mass gravity".to_owned(),
                    })?;
            Ok(RuntimeGravity::PointMass(PointMassGravity::new(mu)?))
        }
        "j2" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for j2 gravity".to_owned(),
                    })?;
            let r_e =
                document
                    .environment
                    .r_e_m
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.r_e_m missing for j2 gravity".to_owned(),
                    })?;
            let j2 = document.environment.j2.unwrap_or(WGS84_J2);
            Ok(RuntimeGravity::J2(J2Gravity::new(mu, r_e, j2)?))
        }
        "egm2008" => Ok(RuntimeGravity::Egm2008(
            crate::celestial::build_egm2008_gravity(document, resolved_files)?,
        )),
        "tesseral" => Ok(RuntimeGravity::Tesseral(
            crate::celestial::build_tesseral_gravity(document, resolved_files)?,
        )),
        "third_body" => Ok(RuntimeGravity::ThirdBody(
            crate::celestial::build_third_body_gravity(document, resolved_files)?,
        )),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("environment.gravity = {other} is not wired"),
        }),
    }
}

/// Geometric altitude (m) for atmosphere sampling, frame-aware.
///
/// For a geocentric configuration — position well beyond any
/// flat-earth / local-frame launch radius — altitude is the height
/// above the inferred launch surface radius, `|r| - R_surface`, or
/// above the 6 371 km mean Earth fallback when no near-surface launch
/// radius is available. A near-origin local launch (sounding rocket,
/// drop test) keeps the legacy flat-earth `+z` reading, where `+z` is
/// the local vertical. The 1e6 m threshold matches the geocentric/local
/// split used by `openbmp-sim`'s `vertical_climb_rate` and the FC
/// commander, so a geocentric launch (e.g. an equatorial ascent that
/// stays near `z=0`) no longer reads sea-level density all the way to
/// orbit.
pub(crate) fn atmosphere_altitude_m_with_surface_radius(
    position_eci_m: Vector3<f64>,
    geocentric_surface_radius_m: Option<f64>,
) -> f64 {
    const GEOCENTRIC_RADIUS_THRESHOLD_M: f64 = 1.0e6;
    const DEFAULT_EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;
    let rn = position_eci_m.norm();
    if rn > GEOCENTRIC_RADIUS_THRESHOLD_M {
        let surface_radius_m = geocentric_surface_radius_m.unwrap_or(DEFAULT_EARTH_MEAN_RADIUS_M);
        (rn - surface_radius_m).max(0.0)
    } else {
        position_eci_m.z.max(0.0)
    }
}

pub(crate) fn document_geocentric_surface_radius_m(document: &ScenarioDocument) -> Option<f64> {
    infer_geocentric_surface_radius_m(Vector3::new(
        document.vehicle.initial_position_eci_m[0],
        document.vehicle.initial_position_eci_m[1],
        document.vehicle.initial_position_eci_m[2],
    ))
}

pub(crate) fn infer_geocentric_surface_radius_m(position_eci_m: Vector3<f64>) -> Option<f64> {
    let radius_m = position_eci_m.norm();
    // Sea-level Earth radii are roughly 6.357e6..6.378e6 m. Include a
    // little margin for rounded synthetic launch radii, but avoid
    // treating high-altitude entry/orbit initial states as the ground.
    if radius_m.is_finite() && (6_330_000.0..=6_390_000.0).contains(&radius_m) {
        Some(radius_m)
    } else {
        None
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
#[allow(clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::Position3;
    use openbmp_physics::{LocalGeodeticOrigin, WGS84_A_M, WGS84_OMEGA_RAD_S};
    use openbmp_scenario::Scenario;

    fn zero_gravity() -> RuntimeGravity {
        RuntimeGravity::Constant(ConstantGravity::down_z(0.0).unwrap())
    }

    #[test]
    fn atmosphere_altitude_is_frame_aware() {
        // Local-frame launch (near origin): altitude is the flat-earth +z.
        assert_eq!(
            atmosphere_altitude_m_with_surface_radius(Vector3::new(0.0, 0.0, 1_000.0), None),
            1_000.0
        );
        assert_eq!(
            atmosphere_altitude_m_with_surface_radius(Vector3::new(10.0, 20.0, 0.0), None),
            0.0
        );
        // Geocentric launch (|r| ~ Earth radius): altitude is |r| - R_earth,
        // independent of which axis the position lies on. An equatorial
        // launch at +x must NOT read sea level all the way up.
        let surface = Vector3::new(6_371_000.0, 0.0, 0.0);
        assert_eq!(
            atmosphere_altitude_m_with_surface_radius(surface, None),
            0.0
        );
        let up_100km = Vector3::new(6_471_000.0, 0.0, 0.0);
        assert!(
            (atmosphere_altitude_m_with_surface_radius(up_100km, None) - 100_000.0).abs() < 1.0e-6
        );
        // Same altitude regardless of orbital-plane orientation (z-axis launch).
        let polar_100km = Vector3::new(0.0, 0.0, 6_471_000.0);
        assert!(
            (atmosphere_altitude_m_with_surface_radius(polar_100km, None) - 100_000.0).abs()
                < 1.0e-6
        );

        let rounded_surface = Vector3::new(6_370_000.0, 0.0, 0.0);
        assert_eq!(
            atmosphere_altitude_m_with_surface_radius(rounded_surface, Some(6_370_000.0)),
            0.0
        );
        let rounded_up_1km = Vector3::new(6_371_000.0, 0.0, 0.0);
        assert!(
            (atmosphere_altitude_m_with_surface_radius(rounded_up_1km, Some(6_370_000.0))
                - 1_000.0)
                .abs()
                < 1.0e-6
        );
    }

    #[test]
    fn wgs84_environment_reports_corotating_still_air_velocity() {
        let environment = RuntimeEnvironment {
            atmosphere: None,
            gravity: zero_gravity(),
            frame: FrameContext::wgs84_uniform_rotation(None),
            geocentric_surface_radius_m: None,
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
            gravity: zero_gravity(),
            frame: FrameContext::wgs84_uniform_rotation(Some(origin)),
            geocentric_surface_radius_m: None,
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
            gravity: zero_gravity(),
            frame: FrameContext::toy_fixed_earth(),
            geocentric_surface_radius_m: None,
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

    #[test]
    fn environment_sample_reports_configured_gravity() {
        let environment = RuntimeEnvironment {
            atmosphere: None,
            gravity: RuntimeGravity::Constant(ConstantGravity::down_z(9.80665).unwrap()),
            frame: FrameContext::toy_fixed_earth(),
            geocentric_surface_radius_m: None,
        };
        let sample = environment
            .sample(EnvironmentQuery {
                time: SimTime::ZERO,
                position_eci: Position3::origin(),
            })
            .unwrap();

        assert_eq!(sample.gravity_eci_m_s2, Vector3::new(0.0, 0.0, -9.80665));
    }

    #[test]
    fn environment_sample_reports_tesseral_gravity() {
        let scenario = Scenario::from_toml_str(
            r#"
openbmp.scenario = 3

[meta]
name = "tesseral-environment-test"
description = "Synthetic runtime-environment tesseral gravity test."
validation = "validated-toy"
provenance = "synthetic runner unit test"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 1.0
seed = 1

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6900000.0, 400000.0, 300000.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "tesseral-environment-test"

[[vehicle.assembly.bodies]]
id = "mass"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "wgs84-uniform-rotation"
gravity = "tesseral"
mu_m3_s2 = 3.986004418e14
r_e_m = 6378137.0
tesseral_degree = 2
tesseral_order = 2
tesseral_c20 = -1.082626683e-3
tesseral_c21 = 2.0e-7
tesseral_s21 = -3.0e-7
tesseral_c22 = 1.0e-7
tesseral_s22 = -2.0e-7
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/tesseral-environment-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#,
        )
        .unwrap();
        let frame = FrameContext::wgs84_uniform_rotation(None);
        let environment =
            RuntimeEnvironment::from_document(&scenario.document, &BTreeMap::new(), &frame)
                .unwrap();
        let sample = environment
            .sample(EnvironmentQuery {
                time: SimTime::ZERO,
                position_eci: Position3::new(6_900_000.0, 400_000.0, 300_000.0),
            })
            .unwrap();

        assert!(sample.gravity_eci_m_s2.y.is_finite());
        assert!(sample.gravity_eci_m_s2.y.abs() > 1.0e-6);
    }
}
