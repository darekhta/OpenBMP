//! Offline range-safety landing-footprint helpers.
//!
//! This module is post-processing only: it turns a parsed
//! `[landing_footprint]` scenario block plus a caller-supplied
//! ballistic state into a forward landing prediction. It is not
//! registered as a flight-controller job and it never produces an
//! actuator command.

use openbmp_physics::profile::{
    BallisticState, ConstantGravityRangeSafetyFootprint, FootprintDispersionInput,
    FootprintEnvironment, FootprintGeodeticOrigin, LandingFootprint,
    NumericalGravityRangeSafetyFootprint, RangeSafetyFootprint,
};
use openbmp_physics::{Egm2008ZonalGravity, J2Gravity, STANDARD_GRAVITY_M_S2, WGS84_J2};
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
        LandingFootprintMethod::J2 => {
            let gravity = build_j2_gravity(&scenario.document)?;
            NumericalGravityRangeSafetyFootprint::new(gravity).landing_footprint(state, &env)?
        }
        LandingFootprintMethod::Egm2008 => {
            let gravity = Egm2008ZonalGravity::wgs84_egm2008_zonal();
            NumericalGravityRangeSafetyFootprint::new(gravity).landing_footprint(state, &env)?
        }
    };
    Ok(Some(footprint))
}

fn footprint_environment(
    document: &ScenarioDocument,
    config: &LandingFootprintConfig,
) -> Result<FootprintEnvironment, RunnerError> {
    let gravity_m_s2 = match config.method {
        LandingFootprintMethod::ConstantGravity => constant_gravity_m_s2(document)?,
        LandingFootprintMethod::J2 | LandingFootprintMethod::Egm2008 => STANDARD_GRAVITY_M_S2,
    };
    let launch_origin_eci_m = launch_origin_eci_m(document, config);
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

fn launch_origin_eci_m(document: &ScenarioDocument, config: &LandingFootprintConfig) -> [f64; 3] {
    match config.method {
        LandingFootprintMethod::ConstantGravity => [
            document.vehicle.initial_position_eci_m[0],
            document.vehicle.initial_position_eci_m[1],
            config.cull_altitude_m,
        ],
        LandingFootprintMethod::J2 | LandingFootprintMethod::Egm2008 => {
            if config.include_geodetic
                && let Some(origin) = document
                    .frames
                    .as_ref()
                    .and_then(|frames| frames.local_origin.as_ref())
                && let Ok(origin) = openbmp_physics::LocalGeodeticOrigin::new_degrees(
                    origin.latitude_deg,
                    origin.longitude_deg,
                    origin.height_m,
                )
            {
                let ecef = origin.to_ecef_position();
                [ecef.vector.x, ecef.vector.y, ecef.vector.z]
            } else {
                document.vehicle.initial_position_eci_m
            }
        }
    }
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

fn build_j2_gravity(document: &ScenarioDocument) -> Result<J2Gravity, RunnerError> {
    let mu = document
        .environment
        .mu_m3_s2
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.mu_m3_s2 missing for j2 footprint gravity".to_owned(),
        })?;
    let r_e = document
        .environment
        .r_e_m
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.r_e_m missing for j2 footprint gravity".to_owned(),
        })?;
    let j2 = document.environment.j2.unwrap_or(WGS84_J2);
    Ok(J2Gravity::new(mu, r_e, j2)?)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::SimTime;
    use openbmp_physics::WGS84_A_M;

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
    fn runner_computes_j2_landing_footprint() {
        let surface_position = format!("initial_position_eci_m = [{WGS84_A_M:.1}, 0.0, 0.0]");
        let gravity_config = format!("mu_m3_s2 = 398600441800000.0\nr_e_m = {WGS84_A_M:.1}");
        let toml = COAST_FOOTPRINT_SCENARIO
            .replace(
                "initial_position_eci_m = [0.0, 0.0, 0.0]",
                &surface_position,
            )
            .replace(r#"gravity = "constant""#, r#"gravity = "j2""#)
            .replace("gravity_m_s2 = 9.80665", &gravity_config)
            .replace(r#"method = "constant_gravity""#, r#"method = "j2""#);
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let state = BallisticState {
            position_eci_m: [WGS84_A_M + 1_000.0, 0.0, 0.0],
            velocity_eci_m_s: [0.0, 100.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let footprint = landing_footprint_for_state(&scenario, &state)
            .unwrap()
            .unwrap();
        assert!(footprint.time_to_cull_s > 0.0);
        assert!(footprint.downrange_m > 0.0);
        assert!(footprint.dispersion_ellipse.is_some());
    }

    #[test]
    fn runner_computes_egm2008_landing_footprint() {
        let surface_position = format!("initial_position_eci_m = [{WGS84_A_M:.1}, 0.0, 0.0]");
        let toml = COAST_FOOTPRINT_SCENARIO
            .replace(
                "initial_position_eci_m = [0.0, 0.0, 0.0]",
                &surface_position,
            )
            .replace(r#"gravity = "constant""#, r#"gravity = "egm2008""#)
            .replace("gravity_m_s2 = 9.80665\n", "")
            .replace(r#"method = "constant_gravity""#, r#"method = "egm2008""#);
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let state = BallisticState {
            position_eci_m: [WGS84_A_M + 1_000.0, 0.0, 0.0],
            velocity_eci_m_s: [0.0, 100.0, 0.0],
            ballistic_coefficient_m2_kg: 0.0,
            time: SimTime::from_seconds(0.0),
        };
        let footprint = landing_footprint_for_state(&scenario, &state)
            .unwrap()
            .unwrap();
        assert!(footprint.time_to_cull_s > 0.0);
        assert!(footprint.downrange_m > 0.0);
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
