//! Runner-side propulsion loaders shared by point-mass and rigid-body
//! paths.

use std::collections::BTreeMap;

use openbmp_physics::{
    IdealStagingBudgetAnalysis, StageMassProperties, StagingBudgetAnalysis, StagingBudgetInput,
};
use openbmp_propulsion::{
    AmbientPressureCorrection, BatesGrain, EndBurnerGrain, EquilibriumInternalBallistics,
    GrainPropellant, GrainRegressionMode, GrainRegressionModel, MotorError,
    NozzleSeparationCriterion, SolidMotor, TabulatedGrain, Validation,
};
use openbmp_scenario::{
    FeedModeConfig, GrainGeometryConfig, GrainRegressionModeConfig, MotorConfig, MotorGrainConfig,
    NozzleAmbientPressureCorrectionConfig, NozzleSeparationConfig, PropulsionThermochemConfig,
    ResolvedFile, ScenarioDocument, StagingAnalysisMode,
};
use openbmp_thermochem::{ThermochemDeck, ThermochemQuery, ThermochemTable};
use openbmp_vehicle::{EnginePropellantBinding, FeedMode, PropellantBudget};

use crate::error::RunnerError;

/// Load or synthesize the scenario's single solid motor, if present.
pub(crate) fn load_solid_motor(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<SolidMotor>, RunnerError> {
    let Some(propulsion) = document.propulsion.as_ref() else {
        return Ok(None);
    };
    let Some(config) = propulsion.motor.as_ref() else {
        return Ok(None);
    };
    let mut motor = load_motor_config(config, propulsion.thermochem.as_ref(), resolved_files)?;
    if let Some(nozzle) = document
        .propulsion
        .as_ref()
        .and_then(|propulsion| propulsion.nozzle.as_ref())
    {
        let correction = match nozzle.ambient_pressure_correction {
            NozzleAmbientPressureCorrectionConfig::Constant => AmbientPressureCorrection::Constant,
            NozzleAmbientPressureCorrectionConfig::PressureThrust => {
                AmbientPressureCorrection::PressureThrust
            }
        };
        motor = motor.with_ambient_pressure_correction(correction)?;
        let separation = match nozzle.separation {
            NozzleSeparationConfig::Off => NozzleSeparationCriterion::Off,
            NozzleSeparationConfig::Summerfield => NozzleSeparationCriterion::Summerfield,
            NozzleSeparationConfig::Schmucker => NozzleSeparationCriterion::Schmucker,
        };
        motor = motor.with_nozzle_separation(separation)?;
    }
    Ok(Some(motor))
}

/// Build the optional vehicle-side propellant budget from engine
/// declarations.
pub(crate) fn build_propellant_budget(
    document: &ScenarioDocument,
) -> Result<Option<PropellantBudget>, RunnerError> {
    let mut bindings = Vec::new();
    for engine in &document.vehicle.assembly.engines {
        let Some(propellant) = &engine.propellant else {
            continue;
        };
        let engine_id = openbmp_core::EngineId::from_path(&format!(
            "vehicle.assembly.engines.{id}",
            id = engine.id
        ));
        let fuel_tank = openbmp_core::TankId::from_path(&format!(
            "vehicle.assembly.tanks.{id}",
            id = propellant.fuel_tank
        ));
        let oxidizer_tank = propellant
            .oxidizer_tank
            .as_ref()
            .map(|id| openbmp_core::TankId::from_path(&format!("vehicle.assembly.tanks.{id}")));
        let feed = match propellant.feed {
            FeedModeConfig::Regulated => FeedMode::Regulated,
            FeedModeConfig::Blowdown => FeedMode::Blowdown,
        };
        bindings.push(EnginePropellantBinding {
            engine_id,
            oxidizer_fuel_ratio: propellant.oxidizer_fuel_ratio,
            fuel_tank,
            oxidizer_tank,
            feed,
            residual_reserve_kg: propellant.residual_reserve_kg,
        });
    }
    if bindings.is_empty() {
        Ok(None)
    } else {
        PropellantBudget::new(bindings)
            .map(Some)
            .map_err(|err| RunnerError::Engine {
                field: "vehicle.assembly.engines[*].propellant".to_owned(),
                reason: err.to_string(),
            })
    }
}

/// Evaluate optional `[staging_analysis]` and write a compact report
/// into telemetry schema metadata.
pub(crate) fn append_staging_analysis_metadata(
    document: &ScenarioDocument,
    metadata: &mut BTreeMap<String, String>,
) -> Result<(), RunnerError> {
    let Some(config) = &document.staging_analysis else {
        return Ok(());
    };
    let input = StagingBudgetInput {
        stages: config
            .stages
            .iter()
            .map(|stage| StageMassProperties {
                isp_s: stage.isp_s,
                structural_coefficient: stage.structural_coefficient,
                structural_mass_kg: stage.structural_mass_kg,
                propellant_mass_kg: stage.propellant_mass_kg,
            })
            .collect(),
        payload_mass_kg: config.payload_mass_kg,
        delta_v_budget_m_s: config.delta_v_budget_m_s,
    };
    let report = IdealStagingBudgetAnalysis::default()
        .analyze(&input)
        .map_err(RunnerError::Env)?;
    let mode = match config.mode {
        StagingAnalysisMode::Budget => "budget",
        StagingAnalysisMode::Optimal => "optimal",
    };
    metadata.insert("openbmp.staging_analysis.mode".to_owned(), mode.to_owned());
    metadata.insert(
        "openbmp.staging_analysis.ideal_loss_free".to_owned(),
        report.ideal_loss_free.to_string(),
    );
    metadata.insert(
        "openbmp.staging_analysis.total_delta_v_m_s".to_owned(),
        report.total_delta_v_m_s.to_string(),
    );
    metadata.insert(
        "openbmp.staging_analysis.gross_liftoff_mass_kg".to_owned(),
        report.gross_liftoff_mass_kg.to_string(),
    );
    metadata.insert(
        "openbmp.staging_analysis.payload_fraction".to_owned(),
        report.payload_fraction.to_string(),
    );
    for stage in &report.stages {
        let prefix = format!("openbmp.staging_analysis.stages.{}", stage.stage_index);
        metadata.insert(
            format!("{prefix}.delta_v_m_s"),
            stage.delta_v_m_s.to_string(),
        );
        metadata.insert(format!("{prefix}.mass_ratio"), stage.mass_ratio.to_string());
        metadata.insert(
            format!("{prefix}.payload_ratio"),
            stage.payload_ratio.to_string(),
        );
    }
    Ok(())
}

fn load_motor_config(
    config: &MotorConfig,
    thermochem: Option<&PropulsionThermochemConfig>,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<SolidMotor, RunnerError> {
    if config.file.is_some() {
        let resolved = resolved_files.get("propulsion.motor.file").ok_or_else(|| {
            RunnerError::UnsupportedScenario {
                what: "internal invariant: resolved file `propulsion.motor.file` missing after pin verification".to_owned(),
            }
        })?;
        let text = std::str::from_utf8(&resolved.bytes).map_err(|e| {
            RunnerError::Motor(MotorError::Io {
                reason: format!(
                    "could not read motor file {} as UTF-8: {e}",
                    resolved.path.display()
                ),
            })
        })?;
        return Ok(SolidMotor::load_from_str(text)?);
    }
    let grain = config
        .grain
        .as_ref()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "[propulsion.motor] declared without file or grain".to_owned(),
        })?;
    build_grain_motor(grain, thermochem, resolved_files)
}

fn build_grain_motor(
    config: &MotorGrainConfig,
    thermochem: Option<&PropulsionThermochemConfig>,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<SolidMotor, RunnerError> {
    let mut propellant = GrainPropellant {
        label: config.propellant.label.clone(),
        density_kg_m3: config.propellant.density_kg_m3,
        burn_rate_a: config.propellant.burn_rate_a,
        burn_rate_n: config.propellant.burn_rate_n,
        c_star_m_s: config.propellant.c_star_m_s,
        gamma: config.propellant.gamma,
    };
    if let Some(thermochem) = thermochem {
        let resolved =
            resolved_files
                .get("propulsion.thermochem.file")
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: "internal invariant: resolved file `propulsion.thermochem.file` missing after pin verification".to_owned(),
                })?;
        let text =
            std::str::from_utf8(&resolved.bytes).map_err(|e| RunnerError::UnsupportedScenario {
                what: format!(
                    "could not read thermochemistry deck {} as UTF-8: {e}",
                    resolved.path.display()
                ),
            })?;
        let deck = ThermochemTable::load_from_str(text)?;
        let state = deck.lookup(ThermochemQuery {
            chamber_pressure_pa: thermochem.chamber_pressure_pa,
            mixture_ratio: thermochem.mixture_ratio,
        })?;
        propellant.c_star_m_s = state.c_star_m_s;
        propellant.gamma = state.gamma;
    }
    let throat_area_m2 = std::f64::consts::PI * config.throat_radius_m * config.throat_radius_m;
    let name = config
        .name
        .clone()
        .unwrap_or_else(|| format!("grain-{}", config.propellant.label));
    let provenance = config.provenance.clone().unwrap_or_else(|| {
        "synthetic/textbook inline grain regression declared in scenario".to_owned()
    });
    let mode = match config.mode {
        GrainRegressionModeConfig::QuasiStatic => GrainRegressionMode::QuasiStatic,
        GrainRegressionModeConfig::Transient => GrainRegressionMode::Transient,
    };
    let solver = EquilibriumInternalBallistics::new(
        propellant,
        throat_area_m2,
        config.expansion_ratio,
        config.propellant.web_steps,
        config.dry_mass_kg,
        name,
        provenance,
        Validation::Research,
    )?
    .with_regression_mode(mode);
    match config.geometry {
        GrainGeometryConfig::EndBurner => solver
            .regress(&EndBurnerGrain::new(
                config
                    .cross_section_area_m2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain end_burner missing cross_section_area_m2".to_owned(),
                    })?,
                config
                    .length_m
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain end_burner missing length_m".to_owned(),
                    })?,
            )?)
            .map_err(RunnerError::Motor),
        GrainGeometryConfig::Bates => solver
            .regress(&BatesGrain::new(
                config
                    .segments
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain bates missing segments".to_owned(),
                    })?,
                config
                    .outer_radius_m
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain bates missing outer_radius_m".to_owned(),
                    })?,
                config
                    .core_radius_m
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain bates missing core_radius_m".to_owned(),
                    })?,
                config
                    .segment_length_m
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain bates missing segment_length_m".to_owned(),
                    })?,
            )?)
            .map_err(RunnerError::Motor),
        GrainGeometryConfig::Tabulated => solver
            .regress(&TabulatedGrain::new(
                config
                    .points
                    .clone()
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain tabulated missing points".to_owned(),
                    })?,
                config
                    .propellant_volume_m3
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "validated grain tabulated missing propellant_volume_m3".to_owned(),
                    })?,
            )?)
            .map_err(RunnerError::Motor),
    }
}
