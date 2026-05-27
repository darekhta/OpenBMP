//! Runner-side atmosphere dispatch.
//!
//! Wraps the concrete atmosphere models that the runners support
//! ([`UsStandard1976`] for the historical sounding-rocket / drop-test
//! scenarios, [`PiecewiseExponentialAtmosphere`] for
//! engineering LEO drag estimation) so the rest of the runner code
//! can hold a single concrete type instead of being generic over
//! [`AtmosphereModel`].
//!
//! The enum dispatch keeps the legacy `us_standard_1976` byte-stable
//! contract intact: when the scenario selects `us_standard_1976` the
//! enum forwards to the existing `UsStandard1976` implementation
//! verbatim, so no existing tolerance evidence shifts.

use openbmp_core::SimTime;
use openbmp_physics::{
    AtmosphereModel, AtmosphereSample, Nrlmsis2Compat, Nrlmsise00Full, Nrlmsise00Inputs,
    PhysicsError, PiecewiseExponentialAtmosphere, UsStandard1976,
};
use openbmp_scenario::{AtmosphereConfig, ScenarioDocument};

use crate::error::RunnerError;

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
        "us_standard_1976" => Ok(RuntimeAtmosphere::UsStandard1976(UsStandard1976::new())),
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
