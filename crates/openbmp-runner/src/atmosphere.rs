//! Runner-side atmosphere dispatch.
//!
//! Wraps the concrete atmosphere models that the runners support
//! ([`UsStandard1976`] for the historical sounding-rocket / drop-test
//! scenarios, [`PiecewiseExponentialAtmosphere`] from Phase 5.C.1 for
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
    AtmosphereModel, AtmosphereSample, PhysicsError, PiecewiseExponentialAtmosphere, UsStandard1976,
};
use openbmp_scenario::ScenarioDocument;

use crate::error::RunnerError;

/// Atmosphere model selected by the scenario.
#[derive(Copy, Clone, Debug)]
pub enum RuntimeAtmosphere {
    /// `us_standard_1976` — Phase 2.3 layered model (0-86 km).
    UsStandard1976(UsStandard1976),
    /// `piecewise_exponential` — Phase 5.C.1 engineering layered
    /// exponential model (0-1000 km).
    PiecewiseExponential(PiecewiseExponentialAtmosphere),
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
        }
    }
}

/// Atmosphere kind names that the runner can construct.
const SUPPORTED_ATMOSPHERE_KINDS: &[&str] = &["us_standard_1976", "piecewise_exponential"];

/// Resolve the atmosphere kind named by the scenario into a runtime
/// dispatch enum. Returns [`RunnerError::UnsupportedScenario`] for kinds
/// the runner does not yet wire.
///
/// # Errors
///
/// Returns [`RunnerError::UnsupportedScenario`] when `atmosphere_kind`
/// is not in [`SUPPORTED_ATMOSPHERE_KINDS`].
pub fn build_runtime_atmosphere(atmosphere_kind: &str) -> Result<RuntimeAtmosphere, RunnerError> {
    match atmosphere_kind {
        "us_standard_1976" => Ok(RuntimeAtmosphere::UsStandard1976(UsStandard1976::new())),
        "piecewise_exponential" => Ok(RuntimeAtmosphere::PiecewiseExponential(
            PiecewiseExponentialAtmosphere::new(),
        )),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "atmosphere `{other}` is not wired (supported: {})",
                SUPPORTED_ATMOSPHERE_KINDS.join(", ")
            ),
        }),
    }
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
