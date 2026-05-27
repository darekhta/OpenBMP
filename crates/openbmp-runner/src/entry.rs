//! Offline descent / entry profile helpers.
//!
//! This module consumes the schema-v3 `[entry_profile]` block and a
//! caller-supplied entry sample. It reports forward physics diagnostics
//! and corridor-driven bank references only; it never accepts a target
//! location and never emits actuator commands.

use openbmp_aerothermal::SuttonGraves;
use openbmp_physics::profile::{
    BandLimitedEntryCorridorReference, EntryCorridor, EntryCorridorReference, EntryState,
};
use openbmp_physics::{AllenEggers, EntryInterfaceBuilder, Vinh, VinhState, VinhStateDerivative};
use openbmp_scenario::{EntryProfileConfig, EntryProfileMode, Scenario};

use crate::RunnerError;

/// State sample supplied to entry-profile post-processing.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EntryProfileSample {
    /// Geometric altitude (m).
    pub altitude_m: f64,
    /// Inertial speed magnitude (m/s).
    pub velocity_m_s: f64,
    /// Flight-path angle (rad), positive upward.
    pub flight_path_angle_rad: f64,
    /// Heading angle (rad), measured east from north for the Vinh
    /// reference equations.
    pub heading_rad: f64,
    /// Ballistic coefficient `B = C_D * A / m` (m²/kg).
    pub ballistic_coefficient_m2_kg: f64,
    /// Current stagnation-point heat-rate estimate (W/m²).
    pub heat_rate_w_m2: f64,
    /// Current deceleration load factor (g).
    pub load_factor_g: f64,
}

/// Entry-profile report produced from a configured scenario block.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum EntryProfileReport {
    /// Ballistic-entry closed-form diagnostics.
    Ballistic(BallisticEntryReport),
    /// Lifting-entry corridor reference and Vinh derivative.
    Lifting(LiftingEntryReport),
}

/// Ballistic-entry diagnostics from Allen-Eggers.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BallisticEntryReport {
    /// Radius at the supplied sample altitude (m).
    pub radius_m: f64,
    /// Entry flight-path angle below local horizon (rad).
    pub flight_path_angle_below_horizon_rad: f64,
    /// Altitude of peak deceleration (m).
    pub peak_deceleration_altitude_m: f64,
    /// Peak deceleration magnitude (m/s²).
    pub peak_deceleration_m_s2: f64,
    /// Peak deceleration load factor (g).
    pub peak_deceleration_g: f64,
    /// Peak stagnation-point convective heat flux (W/m²), when
    /// `entry_profile.nose_radius_m` is configured.
    pub peak_convective_heat_flux_w_m2: Option<f64>,
    /// Altitude of peak stagnation-point convective heating (m), when
    /// `entry_profile.nose_radius_m` is configured.
    pub peak_heat_flux_altitude_m: Option<f64>,
    /// Integrated stagnation-point convective heat load (J/m²), when
    /// `entry_profile.nose_radius_m` is configured.
    pub convective_heat_load_j_m2: Option<f64>,
}

/// Lifting-entry report from the corridor reference and Vinh RHS.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LiftingEntryReport {
    /// Radius at the supplied sample altitude (m).
    pub radius_m: f64,
    /// Reference bank angle (rad).
    pub bank_angle_rad: f64,
    /// Heat-rate margin to the configured corridor limit (W/m²).
    pub heat_rate_margin_w_m2: f64,
    /// Load-factor margin to the configured corridor limit (g).
    pub load_factor_margin_g: f64,
    /// Flight-path-angle margin to the configured corridor band (rad).
    pub flight_path_angle_margin_rad: f64,
    /// Vinh six-state derivative at the supplied sample.
    pub derivative: VinhStateDerivative,
}

/// Compute the configured entry-profile report for `sample`.
///
/// Returns `Ok(None)` when the scenario has no `[entry_profile]`
/// block.
///
/// # Errors
///
/// Returns [`RunnerError`] when the sample or configured physics
/// envelope is invalid.
pub fn entry_profile_for_sample(
    scenario: &Scenario,
    sample: &EntryProfileSample,
) -> Result<Option<EntryProfileReport>, RunnerError> {
    let Some(config) = scenario.document.entry_profile.as_ref() else {
        return Ok(None);
    };
    let report = match config.mode {
        EntryProfileMode::Ballistic => {
            EntryProfileReport::Ballistic(ballistic_entry_report(config, sample)?)
        }
        EntryProfileMode::Lifting => EntryProfileReport::Lifting(lifting_entry_report(
            config,
            sample,
            scenario.document.environment.mu_m3_s2,
        )?),
    };
    Ok(Some(report))
}

fn ballistic_entry_report(
    config: &EntryProfileConfig,
    sample: &EntryProfileSample,
) -> Result<BallisticEntryReport, RunnerError> {
    validate_sample_common(sample)?;
    if sample.ballistic_coefficient_m2_kg <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "ballistic entry requires a positive ballistic coefficient".to_owned(),
        });
    }
    let flight_path_angle_below_horizon_rad = -sample.flight_path_angle_rad;
    if flight_path_angle_below_horizon_rad <= 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "ballistic entry sample must be descending".to_owned(),
        });
    }
    let builder = entry_builder_from_sample(sample, flight_path_angle_below_horizon_rad)?;
    let model = AllenEggers {
        rho_s_kg_m3: config.surface_density_kg_m3,
        beta_inv_m: 1.0 / config.scale_height_m,
        entry_velocity_m_s: sample.velocity_m_s,
        flight_path_angle_rad: flight_path_angle_below_horizon_rad,
        ballistic_coefficient_m2_kg: sample.ballistic_coefficient_m2_kg,
    };
    model.validate()?;
    let heating = config
        .nose_radius_m
        .map(|nose_radius_m| SuttonGraves::default().allen_eggers_heating(&model, nose_radius_m))
        .transpose()
        .map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("ballistic entry heating diagnostic failed: {err}"),
        })?;
    Ok(BallisticEntryReport {
        radius_m: builder.radius_m(),
        flight_path_angle_below_horizon_rad,
        peak_deceleration_altitude_m: model.peak_decel_altitude_m(),
        peak_deceleration_m_s2: model.peak_deceleration_m_s2(),
        peak_deceleration_g: model.peak_deceleration_g(),
        peak_convective_heat_flux_w_m2: heating.map(|h| h.peak_convective_heat_flux_w_m2),
        peak_heat_flux_altitude_m: heating.map(|h| h.peak_heat_flux_altitude_m),
        convective_heat_load_j_m2: heating.map(|h| h.convective_heat_load_j_m2),
    })
}

fn lifting_entry_report(
    config: &EntryProfileConfig,
    sample: &EntryProfileSample,
    mu_m3_s2: Option<f64>,
) -> Result<LiftingEntryReport, RunnerError> {
    validate_sample_common(sample)?;
    let corridor_config =
        config
            .corridor
            .as_ref()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "lifting entry requires entry_profile.corridor".to_owned(),
            })?;
    let corridor = EntryCorridor {
        max_heat_rate_w_m2: corridor_config.max_heat_rate_w_m2,
        max_load_factor_g: corridor_config.max_load_factor_g,
        flight_path_angle_band_rad: corridor_config.flight_path_angle_band_rad,
    };
    let state = EntryState {
        altitude_m: sample.altitude_m,
        velocity_m_s: sample.velocity_m_s,
        flight_path_angle_rad: sample.flight_path_angle_rad,
        heat_rate_w_m2: sample.heat_rate_w_m2,
        load_factor_g: sample.load_factor_g,
    };
    let reference = BandLimitedEntryCorridorReference::new(
        corridor_config.nominal_bank_rad,
        corridor_config.max_bank_rad,
    )?;
    let output =
        reference.bank_reference(&corridor, &state, openbmp_core::SimTime::from_seconds(0.0))?;
    let builder = entry_builder_from_sample(sample, -sample.flight_path_angle_rad)?;
    let mut model = Vinh {
        ballistic_coefficient_m2_kg: sample.ballistic_coefficient_m2_kg,
        lift_to_drag_ratio: config.lift_to_drag_ratio.ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: "lifting entry requires entry_profile.lift_to_drag_ratio".to_owned(),
            }
        })?,
        bank_angle_rad: output.bank_angle_rad,
        rho_s_kg_m3: config.surface_density_kg_m3,
        beta_inv_m: 1.0 / config.scale_height_m,
        ..Vinh::default()
    };
    if let Some(mu_m3_s2) = mu_m3_s2 {
        model.mu_m3_s2 = mu_m3_s2;
    }
    let derivative = model.derivative(VinhState {
        r_m: builder.radius_m(),
        theta_rad: 0.0,
        phi_rad: 0.0,
        velocity_m_s: sample.velocity_m_s,
        flight_path_angle_rad: sample.flight_path_angle_rad,
        heading_rad: sample.heading_rad,
    });
    Ok(LiftingEntryReport {
        radius_m: builder.radius_m(),
        bank_angle_rad: output.bank_angle_rad,
        heat_rate_margin_w_m2: output.heat_rate_margin_w_m2,
        load_factor_margin_g: output.load_factor_margin_g,
        flight_path_angle_margin_rad: output.flight_path_angle_margin_rad,
        derivative,
    })
}

fn entry_builder_from_sample(
    sample: &EntryProfileSample,
    flight_path_angle_below_horizon_rad: f64,
) -> Result<EntryInterfaceBuilder, RunnerError> {
    let builder = EntryInterfaceBuilder {
        entry_altitude_m: sample.altitude_m,
        entry_velocity_m_s: sample.velocity_m_s,
        flight_path_angle_below_horizon_rad,
        heading_rad: sample.heading_rad,
    };
    builder.validate()?;
    Ok(builder)
}

fn validate_sample_common(sample: &EntryProfileSample) -> Result<(), RunnerError> {
    let entry_state = EntryState {
        altitude_m: sample.altitude_m,
        velocity_m_s: sample.velocity_m_s,
        flight_path_angle_rad: sample.flight_path_angle_rad,
        heat_rate_w_m2: sample.heat_rate_w_m2,
        load_factor_g: sample.load_factor_g,
    };
    entry_state.validate()?;
    if !sample.heading_rad.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: "entry heading angle must be finite".to_owned(),
        });
    }
    if !sample.ballistic_coefficient_m2_kg.is_finite() || sample.ballistic_coefficient_m2_kg < 0.0 {
        return Err(RunnerError::UnsupportedScenario {
            what: "entry ballistic coefficient must be finite and non-negative".to_owned(),
        });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;

    const ENTRY_PROFILE_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../openbmp-scenario/tests/fixtures/entry-profile-valid.toml"
    ));
    const MINIMAL_SCENARIO: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../scenarios/analytic-toy/constant-acceleration-drop.toml"
    ));

    fn entry_sample() -> EntryProfileSample {
        EntryProfileSample {
            altitude_m: 122_000.0,
            velocity_m_s: 7_800.0,
            flight_path_angle_rad: -5.0_f64.to_radians(),
            heading_rad: 0.0,
            ballistic_coefficient_m2_kg: 0.004,
            heat_rate_w_m2: 100_000.0,
            load_factor_g: 1.5,
        }
    }

    #[test]
    fn runner_computes_configured_ballistic_entry_report() {
        let scenario = Scenario::from_toml_str(ENTRY_PROFILE_SCENARIO).unwrap();
        let report = entry_profile_for_sample(&scenario, &entry_sample())
            .unwrap()
            .unwrap();
        let EntryProfileReport::Ballistic(report) = report else {
            panic!("expected ballistic report");
        };
        assert!(report.radius_m > 6.0e6);
        assert!(report.peak_deceleration_m_s2 > 0.0);
        assert!(report.peak_deceleration_altitude_m.is_finite());
        assert!(report.peak_convective_heat_flux_w_m2.unwrap() > 0.0);
        assert!(report.peak_heat_flux_altitude_m.unwrap() > report.peak_deceleration_altitude_m);
        assert!(report.convective_heat_load_j_m2.unwrap() > 0.0);
    }

    #[test]
    fn runner_returns_none_without_entry_profile_config() {
        let scenario = Scenario::from_toml_str(MINIMAL_SCENARIO).unwrap();
        assert!(
            entry_profile_for_sample(&scenario, &entry_sample())
                .unwrap()
                .is_none()
        );
    }
}
