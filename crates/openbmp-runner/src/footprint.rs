//! Offline range-safety landing-footprint helpers.
//!
//! This module is post-processing only: it turns a parsed
//! `[landing_footprint]` scenario block plus a caller-supplied
//! ballistic state into a forward landing prediction. It is not
//! registered as a flight-controller job and it never produces an
//! actuator command.

use openbmp_physics::profile::{
    BallisticState, ConstantGravityRangeSafetyFootprint, FootprintDispersionInput,
    FootprintEnvironment, FootprintGeodeticOrigin, LandingFootprint, RangeSafetyFootprint,
};
use openbmp_scenario::{
    LandingFootprintConfig, LandingFootprintMethod, ModelRole, Scenario, ScenarioDocument,
    ScenarioError,
};

use crate::RunnerError;

/// Compute the configured offline landing footprint for `state`.
///
/// Returns `Ok(None)` when the scenario does not declare a
/// `[landing_footprint]` block.
///
/// # Errors
///
/// Returns [`RunnerError`] when the scenario block is internally
/// inconsistent or the physics model rejects the supplied state /
/// environment.
pub fn landing_footprint_for_state(
    scenario: &Scenario,
    state: &BallisticState,
) -> Result<Option<LandingFootprint>, RunnerError> {
    let Some(config) = scenario.document.landing_footprint.as_ref() else {
        return Ok(None);
    };
    let env = footprint_environment(&scenario.document, config)?;
    let footprint = match config.method {
        LandingFootprintMethod::ConstantGravity => {
            ConstantGravityRangeSafetyFootprint.landing_footprint(state, &env)?
        }
    };
    Ok(Some(footprint))
}

fn footprint_environment(
    document: &ScenarioDocument,
    config: &LandingFootprintConfig,
) -> Result<FootprintEnvironment, RunnerError> {
    let gravity_m_s2 = constant_gravity_m_s2(document)?;
    let launch_origin_eci_m = [
        document.vehicle.initial_position_eci_m[0],
        document.vehicle.initial_position_eci_m[1],
        config.cull_altitude_m,
    ];
    let geodetic_origin = if config.include_geodetic {
        let origin = document
            .frames
            .as_ref()
            .and_then(|frames| frames.local_origin.as_ref())
            .ok_or_else(|| {
                RunnerError::Scenario(ScenarioError::InconsistentSection {
                    field_a: "landing_footprint.include_geodetic".to_owned(),
                    value_a: "true".to_owned(),
                    field_b: "frames.local_origin".to_owned(),
                    value_b: "missing".to_owned(),
                })
            })?;
        Some(FootprintGeodeticOrigin {
            latitude_deg: origin.latitude_deg,
            longitude_deg: origin.longitude_deg,
            height_m: origin.height_m,
        })
    } else {
        None
    };
    Ok(FootprintEnvironment {
        cull_altitude_m: config.cull_altitude_m,
        gravity_m_s2,
        launch_origin_eci_m,
        geodetic_origin,
        dispersion: config
            .dispersion
            .as_ref()
            .map(|dispersion| FootprintDispersionInput {
                one_sigma_semi_major_m: dispersion.one_sigma_semi_major_m,
                one_sigma_semi_minor_m: dispersion.one_sigma_semi_minor_m,
                orientation_rad: dispersion.orientation_rad,
            }),
    })
}

fn constant_gravity_m_s2(document: &ScenarioDocument) -> Result<f64, RunnerError> {
    if document.environment.gravity != "constant" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "landing_footprint.method = \"constant_gravity\" requires \
                 environment.gravity = \"constant\"; got `{}`",
                document.environment.gravity
            ),
        });
    }
    document.environment.gravity_m_s2.ok_or_else(|| {
        RunnerError::Scenario(ScenarioError::MissingRequiredField {
            field: "environment.gravity_m_s2".to_owned(),
            role: ModelRole::Gravity,
            name: "constant".to_owned(),
        })
    })
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;

    const COAST_FOOTPRINT_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../openbmp-scenario/tests/fixtures/coast-footprint-valid.toml"
    ));
    const MINIMAL_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

    #[test]
    fn runner_computes_configured_landing_footprint() {
        let scenario = Scenario::from_toml_str(COAST_FOOTPRINT_SCENARIO).unwrap();
        let state = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [5.0, 2.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let footprint = landing_footprint_for_state(&scenario, &state)
            .unwrap()
            .unwrap();
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.crossrange_m > 0.0);
        assert!(footprint.dispersion_ellipse.is_some());
    }

    #[test]
    fn runner_returns_none_without_footprint_config() {
        let scenario = Scenario::from_toml_str(MINIMAL_SCENARIO).unwrap();
        let state = BallisticState {
            position_eci_m: [0.0, 0.0, 100.0],
            velocity_eci_m_s: [0.0, 0.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        assert!(
            landing_footprint_for_state(&scenario, &state)
                .unwrap()
                .is_none()
        );
    }
}
