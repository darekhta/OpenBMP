//! Offline trajectory-optimization CLI commands.

use std::io::BufWriter;
use std::path::{Path, PathBuf};

use openbmp_core::ChannelId;
use openbmp_physics::WGS84_MU_M3_S2;
use openbmp_physics::profile::{
    BallisticState, ForwardSimulationProvenance, ForwardSimulationSource, TerminalCondition,
};
use openbmp_physics::wind::{
    MeasuredWindProfile, MeasuredWindSample, SpliceRule, SplicedWindProfile, WindProfileProvenance,
    WindSource,
};
use openbmp_runner::RunOutcome;
use openbmp_scenario::{
    FcAscentReferenceConfig, FcAscentReferenceMethod, FcGuidanceKind, ModelRegistry, Scenario,
};
use openbmp_sil::{
    MissionPackage, ParameterOverride, SilStimulation, SilTestbench, write_evidence_bundle,
};
use openbmp_telemetry::{TelemetryRow, TelemetryTable, TelemetryValue};
use openbmp_trajopt::{
    DifferentialCorrector, DolColaScreeningReport, DolCommitReadinessInput,
    DolCommitReadinessReport, DolLoadWindow, DolPersistenceKnockdownReport, DolRedesignInput,
    DolReportCard, DolReportCardInput, DolVerificationIndicator, GainAxis, GainTable, ILoadHeader,
    ILoadPayload, ILoadUpdateArtifact, ReferenceProfileSample, SynthesisMetadata, TrajoptError,
    TwoBodyApogeeTargeting, build_dol_commit_readiness, build_dol_report_card,
    decode_iload_payload, design_dol_iload_update, dol_commit_readiness_csv, dol_report_card_csv,
    encode_iload_payload,
};
use serde::Deserialize;
use serde::de::DeserializeOwned;
use sha2::{Digest, Sha256};

use super::output::ensure_parent_dir;
use crate::CliError;
use crate::commands::run as run_command;

/// Summary returned by `openbmp trajopt correct-apogee`.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrectApogeeCliReport {
    /// Corrected tangential speed, in m/s.
    pub corrected_speed_m_s: f64,
    /// Final residual norm, in m.
    pub residual_norm_m: f64,
    /// Corrector iterations.
    pub iterations: usize,
    /// Number of reference-profile samples emitted into the I-load.
    pub reference_samples: usize,
    /// Serialized I-load size, in bytes.
    pub iload_bytes: usize,
    /// Written I-load path.
    pub output_iload: PathBuf,
}

/// Arguments for `openbmp trajopt correct-apogee`.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrectApogeeArgs {
    /// Initial inertial radius on the x-axis, in m.
    pub initial_radius_m: f64,
    /// Target apogee radius from the central body, in m.
    pub target_apogee_radius_m: f64,
    /// Optional initial tangential speed guess, in m/s.
    pub initial_speed_m_s: Option<f64>,
    /// Coast duration before residual evaluation, in s.
    pub coast_duration_s: f64,
    /// Fixed RK4 propagation step, in s.
    pub step_s: f64,
    /// Central-body gravitational parameter, in m^3/s^2.
    pub mu_m3_s2: f64,
    /// Residual convergence tolerance, in m.
    pub residual_tolerance_m: f64,
    /// Maximum Gauss-Newton iterations.
    pub max_iterations: usize,
    /// Deterministic synthesis seed recorded in the I-load.
    pub synthesis_seed: u64,
    /// Opaque scenario or driver digest.
    pub scenario_digest: String,
    /// Source revision recorded in the I-load.
    pub source_revision: String,
    /// Producer name recorded in the I-load.
    pub producer: String,
}

/// Arguments for `openbmp trajopt correct-apogee-scenario`.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrectApogeeScenarioArgs {
    /// Scenario TOML file used as the runner forward map.
    pub scenario: PathBuf,
    /// Target apogee radius from the central body, in m.
    pub target_apogee_radius_m: f64,
    /// Optional initial speed guess, in m/s.
    pub initial_speed_m_s: Option<f64>,
    /// Optional central-body gravitational parameter for terminal residuals.
    pub mu_m3_s2: Option<f64>,
    /// Residual convergence tolerance, in m.
    pub residual_tolerance_m: f64,
    /// Maximum Gauss-Newton iterations.
    pub max_iterations: usize,
    /// Deterministic synthesis seed recorded in the I-load.
    pub synthesis_seed: u64,
    /// Optional scenario digest; omitted values are derived from the scenario file bytes.
    pub scenario_digest: Option<String>,
    /// Source revision recorded in the I-load.
    pub source_revision: String,
    /// Producer name recorded in the I-load.
    pub producer: String,
}

/// Summary returned by `openbmp trajopt dol-redesign`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolRedesignCliReport {
    /// DOL-specific update artifact.
    pub artifact: ILoadUpdateArtifact,
    /// Serialized I-load size, in bytes.
    pub iload_bytes: usize,
    /// SHA-256 digest of the encoded I-load bytes.
    pub iload_sha256_hex: String,
    /// Written I-load path.
    pub output_iload: PathBuf,
    /// Optional written report JSON path.
    pub report_json: Option<PathBuf>,
}

/// Arguments for `openbmp trajopt dol-redesign`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolRedesignArgs {
    /// Scenario TOML file containing `[day_of_launch.wind]`.
    pub scenario: PathBuf,
    /// Target apogee radius from the central body, in m.
    pub target_apogee_radius_m: f64,
    /// Load-window lower altitude, m.
    pub load_window_min_altitude_m: f64,
    /// Load-window upper altitude, m.
    pub load_window_max_altitude_m: f64,
    /// Reference design airspeed through the load window, m/s.
    pub airspeed_m_s: f64,
    /// Desired mean angle of attack, rad.
    pub alpha_reference_rad: f64,
    /// Desired mean sideslip, rad.
    pub beta_reference_rad: f64,
    /// Existing pitch steering bias, rad.
    pub stale_pitch_bias_rad: f64,
    /// Existing yaw steering bias, rad.
    pub stale_yaw_bias_rad: f64,
    /// Scalar load-angle limit for the toy q-alpha/q-beta surrogate, rad.
    pub load_limit_angle_rad: f64,
    /// Terminal/insertion residual norm supplied by the design pass, m.
    pub insertion_residual_norm_m: f64,
    /// Accepted terminal/insertion residual limit, m.
    pub insertion_residual_limit_m: f64,
    /// Deterministic synthesis seed recorded in the I-load.
    pub synthesis_seed: u64,
    /// Optional scenario digest; omitted values are derived from the scenario file bytes.
    pub scenario_digest: Option<String>,
    /// Optional design-profile digest; omitted values are derived from the scenario file bytes.
    pub design_profile_digest: Option<String>,
    /// Source revision recorded in the I-load metadata.
    pub source_revision: String,
    /// Producer name recorded in the I-load header.
    pub producer: String,
    /// Solver provenance recorded in the report.
    pub solver_provenance: String,
}

/// Summary returned by `openbmp trajopt dol-report-card`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolReportCardCliReport {
    /// Deterministic report-card artifact.
    pub report_card: DolReportCard,
    /// Written CSV report-card path.
    pub output_csv: PathBuf,
    /// Optional written JSON report-card path.
    pub report_json: Option<PathBuf>,
}

/// Arguments for `openbmp trajopt dol-report-card`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolReportCardArgs {
    /// DOL update artifact JSON produced by `dol-redesign`.
    pub update_report_json: PathBuf,
    /// Runner/SIL verification indicator CSV.
    pub indicators_csv: PathBuf,
    /// Stable report identifier.
    pub report_id: String,
    /// Verification provenance label.
    pub verification_provenance: String,
    /// Minimum accepted redesigned load margin, rad.
    pub min_required_margin_rad: f64,
    /// Minimum accepted redesigned not-worse sample fraction.
    pub min_not_worse_fraction: f64,
}

/// Summary returned by `openbmp trajopt dol-commit-readiness`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolCommitReadinessCliReport {
    /// Deterministic commit-readiness artifact.
    pub readiness: DolCommitReadinessReport,
    /// Written CSV readiness path.
    pub output_csv: PathBuf,
    /// Optional written JSON readiness path.
    pub report_json: Option<PathBuf>,
}

/// Arguments for `openbmp trajopt dol-commit-readiness`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolCommitReadinessArgs {
    /// DOL report-card JSON.
    pub report_card_json: PathBuf,
    /// DOL persistence knockdown report JSON.
    pub persistence_json: PathBuf,
    /// DOL launch-COLA screening report JSON.
    pub cola_json: PathBuf,
    /// Stable readiness identifier.
    pub readiness_id: String,
}

/// Summary returned by `openbmp trajopt dol-materialize-indicators`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolIndicatorMaterializationReport {
    /// Verification indicators written for report-card consumption.
    pub indicators: Vec<DolVerificationIndicator>,
    /// Written indicator CSV path.
    pub output_csv: PathBuf,
}

/// Arguments for `openbmp trajopt dol-materialize-indicators`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolIndicatorMaterializationArgs {
    /// Native runner or SIL telemetry CSV.
    pub telemetry_csv: PathBuf,
    /// Indicator-spec CSV mapping DOL indicator names to telemetry channels.
    pub spec_csv: PathBuf,
}

/// How the authenticated DOL I-load was applied before the verifier run.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DolIloadApplicationReport {
    /// Stable application mode label.
    pub mode: String,
    /// Number of ascent-reference configs updated.
    pub updated_ascent_reference_count: usize,
    /// Number of pitch-bias schedule points applied to each updated config.
    pub pitch_bias_points: usize,
    /// Number of yaw-bias schedule points applied to each updated config.
    pub yaw_bias_points: usize,
}

/// Summary returned by `openbmp trajopt dol-verify-run`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolVerifyRunCliReport {
    /// Final runner step reached by the verifier scenario.
    pub run_final_step: u64,
    /// Final runner time reached by the verifier scenario.
    pub run_final_time_s: f64,
    /// Runner stop label.
    pub run_stop_label: String,
    /// Design-profile digest used to bind the verifier scenario to the update.
    pub verifier_design_profile_digest: String,
    /// SHA-256 digest of the updated I-load verified against the update artifact.
    pub updated_iload_sha256_hex: String,
    /// Authenticated I-load application evidence for the verifier scenario.
    pub iload_application: DolIloadApplicationReport,
    /// Verification indicators recovered from runner/SIL telemetry.
    pub indicators: Vec<DolVerificationIndicator>,
    /// Deterministic report-card artifact.
    pub report_card: DolReportCard,
    /// Written runner/SIL telemetry CSV path.
    pub telemetry_csv: PathBuf,
    /// Written materialized indicator CSV path.
    pub indicators_csv: PathBuf,
    /// Written report-card CSV path.
    pub report_csv: PathBuf,
    /// Optional written report-card JSON path.
    pub report_json: Option<PathBuf>,
}

/// Arguments for `openbmp trajopt dol-verify-run`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolVerifyRunArgs {
    /// Verification scenario to run through the native runner.
    pub scenario: PathBuf,
    /// DOL update artifact JSON produced by `dol-redesign`.
    pub update_report_json: PathBuf,
    /// Updated I-load payload produced by `dol-redesign`.
    pub updated_iload: PathBuf,
    /// Optional verifier design-profile digest; omitted values use the scenario
    /// file SHA-256.
    pub verifier_design_profile_digest: Option<String>,
    /// Indicator-spec CSV mapping DOL indicator names to telemetry channels.
    pub indicator_spec_csv: PathBuf,
    /// Stable report identifier.
    pub report_id: String,
    /// Verification provenance label.
    pub verification_provenance: String,
    /// Minimum accepted redesigned load margin, rad.
    pub min_required_margin_rad: f64,
    /// Minimum accepted redesigned not-worse sample fraction.
    pub min_not_worse_fraction: f64,
}

/// Summary returned by `openbmp trajopt dol-verify-sil`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolVerifySilCliReport {
    /// SIL package id.
    pub package_id: String,
    /// SIL case id selected by the package/testbench.
    pub case_id: String,
    /// Final runner step reached by the SIL verifier package.
    pub run_final_step: u64,
    /// Final runner time reached by the SIL verifier package.
    pub run_final_time_s: f64,
    /// Runner stop label.
    pub run_stop_label: String,
    /// Design-profile digest used to bind the verifier package to the update.
    pub verifier_design_profile_digest: String,
    /// SHA-256 digest of the updated I-load verified against the update artifact.
    pub updated_iload_sha256_hex: String,
    /// Authenticated I-load application evidence for the verifier package.
    pub iload_application: DolIloadApplicationReport,
    /// Number of SIL stimulation records emitted by I-load application.
    pub sil_stimuli: usize,
    /// Verification indicators recovered from SIL telemetry.
    pub indicators: Vec<DolVerificationIndicator>,
    /// Deterministic report-card artifact.
    pub report_card: DolReportCard,
    /// Written SIL telemetry CSV path.
    pub telemetry_csv: PathBuf,
    /// Written materialized indicator CSV path.
    pub indicators_csv: PathBuf,
    /// Written report-card CSV path.
    pub report_csv: PathBuf,
    /// Written SIL evidence manifest path.
    pub evidence_manifest: PathBuf,
    /// Optional written report-card JSON path.
    pub report_json: Option<PathBuf>,
}

/// Arguments for `openbmp trajopt dol-verify-sil`.
#[derive(Clone, Debug, PartialEq)]
pub struct DolVerifySilArgs {
    /// SIL mission package manifest to run through the native testbench.
    pub package: PathBuf,
    /// Optional package case id. Omitted values use the package default rule.
    pub case: Option<String>,
    /// DOL update artifact JSON produced by `dol-redesign`.
    pub update_report_json: PathBuf,
    /// Updated I-load payload produced by `dol-redesign`.
    pub updated_iload: PathBuf,
    /// Optional verifier design-profile digest; omitted values use the package
    /// manifest SHA-256.
    pub verifier_design_profile_digest: Option<String>,
    /// Indicator-spec CSV mapping DOL indicator names to telemetry channels.
    pub indicator_spec_csv: PathBuf,
    /// Stable report identifier.
    pub report_id: String,
    /// Verification provenance label.
    pub verification_provenance: String,
    /// Minimum accepted redesigned load margin, rad.
    pub min_required_margin_rad: f64,
    /// Minimum accepted redesigned not-worse sample fraction.
    pub min_not_worse_fraction: f64,
}

/// Run the T0 two-body apogee corrector and write a postcard I-load.
///
/// # Errors
///
/// Returns [`CliError`] when the trajectory corrector rejects inputs,
/// reports non-convergence, or the I-load cannot be written.
pub fn run_correct_apogee(
    args: CorrectApogeeArgs,
    output_iload: &Path,
) -> Result<CorrectApogeeCliReport, CliError> {
    let initial_speed_m_s = args.initial_speed_m_s.unwrap_or_else(|| {
        if args.mu_m3_s2 > 0.0 && args.initial_radius_m > 0.0 {
            (args.mu_m3_s2 / args.initial_radius_m).sqrt()
        } else {
            f64::NAN
        }
    });
    let mut config = TwoBodyApogeeTargeting::wgs84(
        args.initial_radius_m,
        args.target_apogee_radius_m,
        initial_speed_m_s,
    );
    config.coast_duration_s = args.coast_duration_s;
    config.step_s = args.step_s;
    config.mu_m3_s2 = args.mu_m3_s2;
    config.corrector = DifferentialCorrector {
        residual_tolerance: args.residual_tolerance_m,
        max_iterations: args.max_iterations,
        ..DifferentialCorrector::default()
    };
    config.synthesis_seed = args.synthesis_seed;
    config.scenario_digest = args.scenario_digest;
    config.source_revision = args.source_revision;
    config.producer = args.producer;

    let report = openbmp_trajopt::correct_two_body_apogee(&config)?;
    if !report.correction.converged {
        return Err(CliError::Trajopt {
            summary: format!(
                "two-body apogee correction did not converge: residual_norm_m={:.12e}, iterations={}",
                report.correction.residual.norm, report.correction.iterations
            ),
        });
    }
    let Some(encoded) = report.encoded_iload else {
        return Err(CliError::Trajopt {
            summary: "two-body apogee correction converged without encoded I-load".to_owned(),
        });
    };
    std::fs::write(output_iload, &encoded).map_err(|source| CliError::Io {
        path: output_iload.to_path_buf(),
        source,
    })?;
    let Some(&corrected_speed_m_s) = report.correction.free_variables.first() else {
        return Err(CliError::Trajopt {
            summary: "two-body apogee correction returned no speed variable".to_owned(),
        });
    };
    Ok(CorrectApogeeCliReport {
        corrected_speed_m_s,
        residual_norm_m: report.correction.residual.norm,
        iterations: report.correction.iterations,
        reference_samples: report.reference_profile.len(),
        iload_bytes: encoded.len(),
        output_iload: output_iload.to_path_buf(),
    })
}

/// Run the scenario-backed T0 apogee corrector and write a postcard I-load.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario cannot run, the trajectory corrector
/// rejects inputs, reports non-convergence, or the I-load cannot be written.
pub fn run_correct_apogee_scenario(
    args: CorrectApogeeScenarioArgs,
    output_iload: &Path,
) -> Result<CorrectApogeeCliReport, CliError> {
    let base_scenario = Scenario::from_file(&args.scenario)?;
    let initial_velocity = base_scenario.document.vehicle.initial_velocity_eci_m_s;
    let initial_speed_from_scenario = norm3(initial_velocity);
    if initial_speed_from_scenario <= f64::EPSILON || !initial_speed_from_scenario.is_finite() {
        return Err(CliError::Trajopt {
            summary: "scenario initial velocity must be finite and non-zero".to_owned(),
        });
    }
    let velocity_direction = scale3(initial_velocity, 1.0 / initial_speed_from_scenario);
    let initial_speed_m_s = args
        .initial_speed_m_s
        .unwrap_or(initial_speed_from_scenario);
    let mu_m3_s2 = args
        .mu_m3_s2
        .or(base_scenario.document.environment.mu_m3_s2)
        .unwrap_or(WGS84_MU_M3_S2);
    if !mu_m3_s2.is_finite() || mu_m3_s2 <= 0.0 {
        return Err(CliError::Trajopt {
            summary: "terminal residual gravity parameter must be finite and positive".to_owned(),
        });
    }

    let condition = TerminalCondition::ApogeeRadius {
        radius_m: args.target_apogee_radius_m,
    };
    let corrector = DifferentialCorrector {
        residual_tolerance: args.residual_tolerance_m,
        max_iterations: args.max_iterations,
        ..DifferentialCorrector::default()
    };
    let mut forward_failure = None::<String>;
    let correction = corrector
        .solve(&condition, mu_m3_s2, &[initial_speed_m_s], |free| {
            let speed = first_speed_variable(free)?;
            let scenario = scenario_with_speed(&base_scenario, velocity_direction, speed);
            match runner_trajectory(&scenario) {
                Ok((terminal_state, _reference_profile)) => Ok(terminal_state),
                Err(summary) => {
                    forward_failure = Some(summary);
                    Err(TrajoptError::InvalidPayload {
                        reason: "scenario runner forward map failed",
                    })
                }
            }
        })
        .map_err(|source| {
            if let Some(summary) = forward_failure {
                CliError::Trajopt {
                    summary: format!("scenario runner forward map failed: {summary}"),
                }
            } else {
                CliError::TrajoptLib(source)
            }
        })?;

    if !correction.converged {
        return Err(CliError::Trajopt {
            summary: format!(
                "scenario apogee correction did not converge: residual_norm_m={:.12e}, iterations={}",
                correction.residual.norm, correction.iterations
            ),
        });
    }
    let Some(&corrected_speed_m_s) = correction.free_variables.first() else {
        return Err(CliError::Trajopt {
            summary: "scenario apogee correction returned no speed variable".to_owned(),
        });
    };
    let final_scenario =
        scenario_with_speed(&base_scenario, velocity_direction, corrected_speed_m_s);
    let (_terminal_state, reference_profile) =
        runner_trajectory(&final_scenario).map_err(|summary| CliError::Trajopt {
            summary: format!("scenario runner final propagation failed: {summary}"),
        })?;
    let payload = ILoadPayload {
        header: ILoadHeader::new(&condition, args.synthesis_seed, args.producer),
        metadata: SynthesisMetadata {
            scenario_digest: args
                .scenario_digest
                .map_or_else(|| digest_file(&args.scenario), Ok)?,
            source_revision: args.source_revision,
            method: "runner_scenario_apogee_single_shooting".to_owned(),
        },
        event_bindings_postcard: Vec::new(),
        gain_tables: Vec::new(),
        reference_profile,
        optimality_certificate: None,
    };
    let encoded = encode_iload_payload(&payload)?;
    std::fs::write(output_iload, &encoded).map_err(|source| CliError::Io {
        path: output_iload.to_path_buf(),
        source,
    })?;
    Ok(CorrectApogeeCliReport {
        corrected_speed_m_s,
        residual_norm_m: correction.residual.norm,
        iterations: correction.iterations,
        reference_samples: payload.reference_profile.len(),
        iload_bytes: encoded.len(),
        output_iload: output_iload.to_path_buf(),
    })
}

/// Run the day-of-launch spliced-wind recentering helper and write an I-load.
///
/// # Errors
///
/// Returns [`CliError`] when the scenario cannot be read, `[day_of_launch.wind]`
/// is absent or not spliced, the profile cannot be materialized, the redesign
/// helper rejects inputs, or output files cannot be written.
pub fn run_dol_redesign(
    args: DolRedesignArgs,
    output_iload: &Path,
    report_json: Option<&Path>,
) -> Result<DolRedesignCliReport, CliError> {
    let scenario = Scenario::from_file(&args.scenario)?;
    let spliced_wind = spliced_day_of_launch_wind(&scenario)?;
    let scenario_digest = args
        .scenario_digest
        .map_or_else(|| digest_file(&args.scenario), Ok)?;
    let design_profile_digest = args
        .design_profile_digest
        .map_or_else(|| digest_file(&args.scenario), Ok)?;
    let mut update = design_dol_iload_update(DolRedesignInput {
        spliced_wind: &spliced_wind,
        terminal_condition: TerminalCondition::ApogeeRadius {
            radius_m: args.target_apogee_radius_m,
        },
        synthesis_seed: args.synthesis_seed,
        scenario_digest,
        source_revision: args.source_revision,
        producer: args.producer,
        design_profile_digest,
        solver_provenance: args.solver_provenance,
        load_window: DolLoadWindow::new(
            args.load_window_min_altitude_m,
            args.load_window_max_altitude_m,
        )?,
        airspeed_m_s: args.airspeed_m_s,
        alpha_reference_rad: args.alpha_reference_rad,
        beta_reference_rad: args.beta_reference_rad,
        stale_pitch_bias_rad: args.stale_pitch_bias_rad,
        stale_yaw_bias_rad: args.stale_yaw_bias_rad,
        load_limit_angle_rad: args.load_limit_angle_rad,
        insertion_residual_norm_m: args.insertion_residual_norm_m,
        insertion_residual_limit_m: args.insertion_residual_limit_m,
    })?;
    let encoded = encode_iload_payload(&update.payload)?;
    let iload_sha256_hex = sha256_hex(&encoded);
    update.artifact.iload_sha256_hex = iload_sha256_hex.clone();
    std::fs::write(output_iload, &encoded).map_err(|source| CliError::Io {
        path: output_iload.to_path_buf(),
        source,
    })?;
    let report_path = if let Some(path) = report_json {
        let bytes = serde_json::to_vec_pretty(&update.artifact).map_err(|source| {
            CliError::TrajoptReportJson {
                path: path.to_path_buf(),
                source,
            }
        })?;
        std::fs::write(path, bytes).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Some(path.to_path_buf())
    } else {
        None
    };
    Ok(DolRedesignCliReport {
        artifact: update.artifact,
        iload_bytes: encoded.len(),
        iload_sha256_hex,
        output_iload: output_iload.to_path_buf(),
        report_json: report_path,
    })
}

/// Build a DOL report card from a DOL update artifact and verifier indicators.
///
/// # Errors
///
/// Returns [`CliError`] when JSON/CSV inputs cannot be read, inputs are
/// malformed, the report-card builder rejects the comparison, or output files
/// cannot be written.
pub fn run_dol_report_card(
    args: DolReportCardArgs,
    output_csv: &Path,
    report_json: Option<&Path>,
) -> Result<DolReportCardCliReport, CliError> {
    let update_artifact = read_dol_update_artifact(&args.update_report_json)?;
    let indicators = read_verification_indicators_csv(&args.indicators_csv)?;
    let report_card = build_dol_report_card(DolReportCardInput {
        report_id: args.report_id,
        verification_provenance: args.verification_provenance,
        update_artifact: &update_artifact,
        indicators: &indicators,
        min_required_margin_rad: args.min_required_margin_rad,
        min_not_worse_fraction: args.min_not_worse_fraction,
    })?;
    let csv = dol_report_card_csv(&report_card);
    std::fs::write(output_csv, csv).map_err(|source| CliError::Io {
        path: output_csv.to_path_buf(),
        source,
    })?;
    let report_path = if let Some(path) = report_json {
        let bytes = serde_json::to_vec_pretty(&report_card).map_err(|source| {
            CliError::TrajoptReportJson {
                path: path.to_path_buf(),
                source,
            }
        })?;
        std::fs::write(path, bytes).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Some(path.to_path_buf())
    } else {
        None
    };
    Ok(DolReportCardCliReport {
        report_card,
        output_csv: output_csv.to_path_buf(),
        report_json: report_path,
    })
}

fn read_dol_update_artifact(path: &Path) -> Result<ILoadUpdateArtifact, CliError> {
    read_dol_json_report(path)
}

fn read_dol_json_report<T: DeserializeOwned>(path: &Path) -> Result<T, CliError> {
    let artifact_bytes = std::fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    serde_json::from_slice(&artifact_bytes).map_err(|source| CliError::TrajoptReportJson {
        path: path.to_path_buf(),
        source,
    })
}

/// Build a DOL commit-readiness report from report-card, persistence, and COLA artifacts.
///
/// # Errors
///
/// Returns [`CliError`] when JSON inputs cannot be read, the readiness builder
/// rejects inconsistent gate evidence, or output files cannot be written.
pub fn run_dol_commit_readiness(
    args: DolCommitReadinessArgs,
    output_csv: &Path,
    report_json: Option<&Path>,
) -> Result<DolCommitReadinessCliReport, CliError> {
    let report_card: DolReportCard = read_dol_json_report(&args.report_card_json)?;
    let persistence: DolPersistenceKnockdownReport = read_dol_json_report(&args.persistence_json)?;
    let cola: DolColaScreeningReport = read_dol_json_report(&args.cola_json)?;
    let readiness = build_dol_commit_readiness(DolCommitReadinessInput {
        readiness_id: args.readiness_id,
        report_card: &report_card,
        persistence: &persistence,
        cola: &cola,
    })?;
    let csv = dol_commit_readiness_csv(&readiness);
    std::fs::write(output_csv, csv).map_err(|source| CliError::Io {
        path: output_csv.to_path_buf(),
        source,
    })?;
    let report_path = if let Some(path) = report_json {
        let bytes = serde_json::to_vec_pretty(&readiness).map_err(|source| {
            CliError::TrajoptReportJson {
                path: path.to_path_buf(),
                source,
            }
        })?;
        std::fs::write(path, bytes).map_err(|source| CliError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        Some(path.to_path_buf())
    } else {
        None
    };
    Ok(DolCommitReadinessCliReport {
        readiness,
        output_csv: output_csv.to_path_buf(),
        report_json: report_path,
    })
}

#[derive(Debug, Deserialize)]
struct DolVerificationIndicatorCsvRow {
    name: String,
    design_value: f64,
    verified_value: f64,
    allowed_abs_error: f64,
}

fn read_verification_indicators_csv(
    path: &Path,
) -> Result<Vec<DolVerificationIndicator>, CliError> {
    let mut reader = csv::Reader::from_path(path).map_err(|source| CliError::Csv {
        path: path.to_path_buf(),
        source,
    })?;
    let mut indicators = Vec::new();
    for row in reader.deserialize() {
        let row: DolVerificationIndicatorCsvRow = row.map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
        indicators.push(DolVerificationIndicator {
            name: row.name,
            design_value: row.design_value,
            verified_value: row.verified_value,
            allowed_abs_error: row.allowed_abs_error,
        });
    }
    Ok(indicators)
}

/// Materialize DOL report-card indicators from a runner/SIL telemetry CSV.
///
/// # Errors
///
/// Returns [`CliError`] when the telemetry/spec CSVs are malformed, a declared
/// telemetry column is absent, no finite samples are available in a declared
/// window, or the output CSV cannot be written.
pub fn run_dol_materialize_indicators(
    args: DolIndicatorMaterializationArgs,
    output_csv: &Path,
) -> Result<DolIndicatorMaterializationReport, CliError> {
    let specs = read_telemetry_indicator_specs_csv(&args.spec_csv)?;
    let indicators = materialize_indicators_from_telemetry(&args.telemetry_csv, &specs)?;
    write_verification_indicators_csv(output_csv, &indicators)?;
    Ok(DolIndicatorMaterializationReport {
        indicators,
        output_csv: output_csv.to_path_buf(),
    })
}

/// Run a DOL verifier scenario, materialize indicators, and write a report card.
///
/// # Errors
///
/// Returns [`CliError`] when the verifier scenario fails, telemetry cannot be
/// written, indicator materialization fails closed, the report-card builder
/// rejects the inputs, or output artifacts cannot be written.
pub fn run_dol_verify_run(
    args: DolVerifyRunArgs,
    output_telemetry_csv: &Path,
    output_indicators_csv: &Path,
    output_report_csv: &Path,
    report_json: Option<&Path>,
) -> Result<DolVerifyRunCliReport, CliError> {
    let verifier_design_profile_digest = args
        .verifier_design_profile_digest
        .map_or_else(|| digest_file(&args.scenario), Ok)?;
    let (verifier_design_profile_digest, updated_iload_sha256_hex, updated_iload_payload) =
        load_verified_dol_iload(
            &args.update_report_json,
            &args.updated_iload,
            verifier_design_profile_digest,
            "DOL verify-run",
        )?;
    let mut verifier_scenario = Scenario::from_file(&args.scenario)?;
    let iload_application =
        apply_dol_iload_to_verifier_scenario(&mut verifier_scenario, &updated_iload_payload)?;
    let run_data = run_command::RunDataOptions::new(None, None, String::new(), None, None, None);
    let run_report = run_command::run_loaded_with_options(
        &args.scenario,
        verifier_scenario,
        &run_command::OutputOverrides::new(Some(output_telemetry_csv.to_path_buf()), None, None),
        &run_data,
    )?;
    let indicator_report = run_dol_materialize_indicators(
        DolIndicatorMaterializationArgs {
            telemetry_csv: output_telemetry_csv.to_path_buf(),
            spec_csv: args.indicator_spec_csv,
        },
        output_indicators_csv,
    )?;
    let report_card_report = run_dol_report_card(
        DolReportCardArgs {
            update_report_json: args.update_report_json,
            indicators_csv: output_indicators_csv.to_path_buf(),
            report_id: args.report_id,
            verification_provenance: args.verification_provenance,
            min_required_margin_rad: args.min_required_margin_rad,
            min_not_worse_fraction: args.min_not_worse_fraction,
        },
        output_report_csv,
        report_json,
    )?;

    Ok(DolVerifyRunCliReport {
        run_final_step: run_report.final_step,
        run_final_time_s: run_report.final_time_s,
        run_stop_label: run_report.stop_label,
        verifier_design_profile_digest,
        updated_iload_sha256_hex,
        iload_application,
        indicators: indicator_report.indicators,
        report_card: report_card_report.report_card,
        telemetry_csv: output_telemetry_csv.to_path_buf(),
        indicators_csv: output_indicators_csv.to_path_buf(),
        report_csv: report_card_report.output_csv,
        report_json: report_card_report.report_json,
    })
}

/// Run a SIL package, apply an authenticated DOL I-load through SIL
/// stimulation, materialize indicators, and write a report card.
///
/// # Errors
///
/// Returns [`CliError`] when package loading, authenticated I-load binding,
/// SIL stimulation/run, telemetry output, indicator materialization, evidence
/// writing, or report-card generation fails.
pub fn run_dol_verify_sil(
    args: DolVerifySilArgs,
    output_telemetry_csv: &Path,
    output_indicators_csv: &Path,
    output_report_csv: &Path,
    output_evidence_dir: &Path,
    report_json: Option<&Path>,
) -> Result<DolVerifySilCliReport, CliError> {
    let verifier_design_profile_digest = args
        .verifier_design_profile_digest
        .map_or_else(|| digest_file(&args.package), Ok)?;
    let (verifier_design_profile_digest, updated_iload_sha256_hex, updated_iload_payload) =
        load_verified_dol_iload(
            &args.update_report_json,
            &args.updated_iload,
            verifier_design_profile_digest,
            "DOL verify-sil",
        )?;

    let bench = SilTestbench::load(&args.package)?;
    let case = args.case.as_deref();
    let scenario_path = sil_case_scenario_path(bench.package(), case)?;
    let base_scenario = Scenario::from_file(&scenario_path)?;
    let (stimulation, iload_application) =
        dol_iload_stimulation_for_verifier_scenario(&base_scenario, &updated_iload_payload)?;
    if stimulation.parameter_overrides.is_empty() {
        return Err(CliError::Trajopt {
            summary:
                "DOL verify-sil updated I-load application produced no SIL parameter overrides"
                    .to_owned(),
        });
    }

    let sil_report = bench.run_with_stimulation(case, &stimulation)?;
    write_telemetry_csv(&sil_report.telemetry, output_telemetry_csv)?;
    let evidence_manifest = write_evidence_bundle(output_evidence_dir, &sil_report.evidence)?;
    let indicator_report = run_dol_materialize_indicators(
        DolIndicatorMaterializationArgs {
            telemetry_csv: output_telemetry_csv.to_path_buf(),
            spec_csv: args.indicator_spec_csv,
        },
        output_indicators_csv,
    )?;
    let report_card_report = run_dol_report_card(
        DolReportCardArgs {
            update_report_json: args.update_report_json,
            indicators_csv: output_indicators_csv.to_path_buf(),
            report_id: args.report_id,
            verification_provenance: args.verification_provenance,
            min_required_margin_rad: args.min_required_margin_rad,
            min_not_worse_fraction: args.min_not_worse_fraction,
        },
        output_report_csv,
        report_json,
    )?;

    Ok(DolVerifySilCliReport {
        package_id: sil_report.package_id,
        case_id: sil_report.case_id,
        run_final_step: sil_report.final_step,
        run_final_time_s: sil_report.final_time_s,
        run_stop_label: sil_report.stop_label,
        verifier_design_profile_digest,
        updated_iload_sha256_hex,
        iload_application,
        sil_stimuli: sil_report.evidence.stimuli.len(),
        indicators: indicator_report.indicators,
        report_card: report_card_report.report_card,
        telemetry_csv: output_telemetry_csv.to_path_buf(),
        indicators_csv: output_indicators_csv.to_path_buf(),
        report_csv: report_card_report.output_csv,
        evidence_manifest,
        report_json: report_card_report.report_json,
    })
}

fn load_verified_dol_iload(
    update_report_json: &Path,
    updated_iload: &Path,
    verifier_design_profile_digest: String,
    command_label: &str,
) -> Result<(String, String, ILoadPayload), CliError> {
    let update_artifact = read_dol_update_artifact(update_report_json)?;
    if verifier_design_profile_digest != update_artifact.design_profile_digest {
        return Err(CliError::Trajopt {
            summary: format!(
                "{command_label} design-profile digest mismatch: update={}, verifier={}",
                update_artifact.design_profile_digest, verifier_design_profile_digest
            ),
        });
    }
    let updated_iload_bytes = std::fs::read(updated_iload).map_err(|source| CliError::Io {
        path: updated_iload.to_path_buf(),
        source,
    })?;
    let updated_iload_sha256_hex = sha256_hex(&updated_iload_bytes);
    if updated_iload_sha256_hex != update_artifact.iload_sha256_hex {
        return Err(CliError::Trajopt {
            summary: format!(
                "{command_label} updated I-load digest mismatch: update={}, verifier={}",
                update_artifact.iload_sha256_hex, updated_iload_sha256_hex
            ),
        });
    }
    let updated_iload_payload = decode_iload_payload(&updated_iload_bytes)?;
    Ok((
        verifier_design_profile_digest,
        updated_iload_sha256_hex,
        updated_iload_payload,
    ))
}

fn sil_case_scenario_path(
    package: &MissionPackage,
    case_id: Option<&str>,
) -> Result<PathBuf, CliError> {
    let scenario = if let Some(case_id) = case_id {
        let case = package
            .manifest
            .test_cases
            .iter()
            .find(|case| case.id == case_id)
            .ok_or_else(|| CliError::Trajopt {
                summary: format!("DOL verify-sil unknown SIL test case `{case_id}`"),
            })?;
        case.scenario
            .as_ref()
            .unwrap_or(&package.manifest.files.scenario)
    } else if package.manifest.test_cases.len() == 1 {
        package.manifest.test_cases[0]
            .scenario
            .as_ref()
            .unwrap_or(&package.manifest.files.scenario)
    } else {
        &package.manifest.files.scenario
    };
    Ok(resolve_package_path(package, scenario))
}

fn resolve_package_path(package: &MissionPackage, path: &Path) -> PathBuf {
    if path.is_absolute() {
        path.to_path_buf()
    } else {
        package.root.join(path)
    }
}

fn dol_iload_stimulation_for_verifier_scenario(
    scenario: &Scenario,
    payload: &ILoadPayload,
) -> Result<(SilStimulation, DolIloadApplicationReport), CliError> {
    let mut mutated = scenario.clone();
    let iload_application = apply_dol_iload_to_verifier_scenario(&mut mutated, payload)?;
    let mut parameter_overrides = Vec::new();

    if let Some(fc) = mutated.document.fc.as_ref() {
        if let Some(ascent_reference) = fc.ascent_reference.as_ref() {
            push_dol_ascent_reference_overrides(
                "fc.ascent_reference",
                ascent_reference,
                &mut parameter_overrides,
            )?;
        }
        if let Some(by_phase) = fc.ascent_reference_by_phase.as_ref() {
            for (phase, ascent_reference) in by_phase {
                if !is_safe_toml_path_segment(phase) {
                    return Err(CliError::Trajopt {
                        summary: format!(
                            "DOL verify-sil ascent_reference_by_phase key `{phase}` cannot be addressed by SIL parameter overrides"
                        ),
                    });
                }
                push_dol_ascent_reference_overrides(
                    &format!("fc.ascent_reference_by_phase.{phase}"),
                    ascent_reference,
                    &mut parameter_overrides,
                )?;
            }
        }
    }

    Ok((
        SilStimulation {
            parameter_overrides,
            ..SilStimulation::default()
        },
        iload_application,
    ))
}

fn push_dol_ascent_reference_overrides(
    prefix: &str,
    config: &FcAscentReferenceConfig,
    parameter_overrides: &mut Vec<ParameterOverride>,
) -> Result<(), CliError> {
    let schedule_s = config
        .schedule_s
        .as_deref()
        .ok_or_else(|| CliError::Trajopt {
            summary: format!(
                "DOL verify-sil applied ascent reference `{prefix}` is missing schedule_s"
            ),
        })?;
    let pitch_rad = config
        .pitch_rad
        .as_deref()
        .ok_or_else(|| CliError::Trajopt {
            summary: format!(
                "DOL verify-sil applied ascent reference `{prefix}` is missing pitch_rad"
            ),
        })?;
    let yaw_rad = config.yaw_rad.as_deref().ok_or_else(|| CliError::Trajopt {
        summary: format!("DOL verify-sil applied ascent reference `{prefix}` is missing yaw_rad"),
    })?;

    for (field, values) in [
        ("schedule_s", schedule_s),
        ("pitch_rad", pitch_rad),
        ("yaw_rad", yaw_rad),
    ] {
        parameter_overrides.push(ParameterOverride {
            path: format!("{prefix}.{field}"),
            value: toml_float_array(values),
        });
    }
    Ok(())
}

fn toml_float_array(values: &[f64]) -> toml::Value {
    toml::Value::Array(values.iter().copied().map(toml::Value::Float).collect())
}

fn is_safe_toml_path_segment(segment: &str) -> bool {
    !segment.is_empty() && !segment.chars().any(|ch| matches!(ch, '.' | '[' | ']'))
}

fn write_telemetry_csv(table: &TelemetryTable, output_csv: &Path) -> Result<(), CliError> {
    ensure_parent_dir(output_csv)?;
    let file = std::fs::File::create(output_csv).map_err(|source| CliError::Io {
        path: output_csv.to_path_buf(),
        source,
    })?;
    table.write_csv(BufWriter::new(file))?;
    Ok(())
}

fn apply_dol_iload_to_verifier_scenario(
    scenario: &mut Scenario,
    payload: &ILoadPayload,
) -> Result<DolIloadApplicationReport, CliError> {
    let pitch_bias = required_dol_gain_table(payload, "dol_pitch_bias_rad")?;
    let yaw_bias = required_dol_gain_table(payload, "dol_yaw_bias_rad")?;
    validate_matching_dol_bias_tables(pitch_bias, yaw_bias)?;
    let pitch_bias_points = pitch_bias.breakpoints.len();
    let yaw_bias_points = yaw_bias.breakpoints.len();

    let Some(fc) = scenario.document.fc.as_mut() else {
        return Ok(DolIloadApplicationReport {
            mode: "no_fc_verifier_scenario".to_owned(),
            updated_ascent_reference_count: 0,
            pitch_bias_points,
            yaw_bias_points,
        });
    };
    if !matches!(fc.guidance, FcGuidanceKind::AscentReference) {
        return Err(CliError::Trajopt {
            summary:
                "DOL verify-run updated I-load application requires guidance = \"ascent_reference\""
                    .to_owned(),
        });
    }

    let mut updated = 0usize;
    if let Some(ascent_reference) = fc.ascent_reference.as_mut() {
        apply_dol_bias_tables_to_ascent_reference(ascent_reference, pitch_bias, yaw_bias)?;
        updated += 1;
    }
    if let Some(by_phase) = fc.ascent_reference_by_phase.as_mut() {
        for ascent_reference in by_phase.values_mut() {
            apply_dol_bias_tables_to_ascent_reference(ascent_reference, pitch_bias, yaw_bias)?;
            updated += 1;
        }
    }
    if updated == 0 {
        return Err(CliError::Trajopt {
            summary:
                "DOL verify-run guidance = \"ascent_reference\" has no ascent-reference configs"
                    .to_owned(),
        });
    }

    scenario.validate_with_registry(&ModelRegistry::full())?;
    Ok(DolIloadApplicationReport {
        mode: "fc_ascent_reference_pitch_yaw_program".to_owned(),
        updated_ascent_reference_count: updated,
        pitch_bias_points,
        yaw_bias_points,
    })
}

fn required_dol_gain_table<'a>(
    payload: &'a ILoadPayload,
    name: &str,
) -> Result<&'a GainTable, CliError> {
    let table = payload
        .gain_tables
        .iter()
        .find(|table| table.name == name)
        .ok_or_else(|| CliError::Trajopt {
            summary: format!("DOL verify-run updated I-load missing `{name}` gain table"),
        })?;
    if table.axis != GainAxis::Time {
        return Err(CliError::Trajopt {
            summary: format!("DOL verify-run `{name}` gain table must use the time axis"),
        });
    }
    Ok(table)
}

fn validate_matching_dol_bias_tables(
    pitch_bias: &GainTable,
    yaw_bias: &GainTable,
) -> Result<(), CliError> {
    if pitch_bias.breakpoints.len() != yaw_bias.breakpoints.len()
        || pitch_bias.breakpoints.len() != pitch_bias.values.len()
        || yaw_bias.breakpoints.len() != yaw_bias.values.len()
    {
        return Err(CliError::Trajopt {
            summary: "DOL verify-run pitch/yaw I-load bias tables must have matching dimensions"
                .to_owned(),
        });
    }
    if pitch_bias
        .breakpoints
        .iter()
        .zip(&yaw_bias.breakpoints)
        .any(|(pitch, yaw)| pitch.to_bits() != yaw.to_bits())
    {
        return Err(CliError::Trajopt {
            summary: "DOL verify-run pitch/yaw I-load bias table breakpoints differ".to_owned(),
        });
    }
    Ok(())
}

fn apply_dol_bias_tables_to_ascent_reference(
    config: &mut FcAscentReferenceConfig,
    pitch_bias: &GainTable,
    yaw_bias: &GainTable,
) -> Result<(), CliError> {
    if config.method != FcAscentReferenceMethod::PitchProgram {
        return Err(CliError::Trajopt {
            summary:
                "DOL verify-run updated I-load application currently requires pitch_program ascent reference"
                    .to_owned(),
        });
    }
    let schedule = config
        .schedule_s
        .as_ref()
        .ok_or_else(|| CliError::Trajopt {
            summary: "DOL verify-run pitch_program verifier is missing schedule_s".to_owned(),
        })?;
    let pitch = config.pitch_rad.as_ref().ok_or_else(|| CliError::Trajopt {
        summary: "DOL verify-run pitch_program verifier is missing pitch_rad".to_owned(),
    })?;
    if schedule.len() != pitch.len() {
        return Err(CliError::Trajopt {
            summary: "DOL verify-run pitch_program verifier schedule/pitch dimensions differ"
                .to_owned(),
        });
    }
    let yaw = config.yaw_rad.as_deref();
    if let Some(yaw_values) = yaw
        && schedule.len() != yaw_values.len()
    {
        return Err(CliError::Trajopt {
            summary: "DOL verify-run pitch_program verifier schedule/yaw dimensions differ"
                .to_owned(),
        });
    }

    let mut applied_pitch = Vec::with_capacity(pitch_bias.breakpoints.len());
    let mut applied_yaw = Vec::with_capacity(yaw_bias.breakpoints.len());
    for ((&t, &pitch_delta), &yaw_delta) in pitch_bias
        .breakpoints
        .iter()
        .zip(&pitch_bias.values)
        .zip(&yaw_bias.values)
    {
        let base_pitch = interpolate_schedule(schedule, pitch, t, "pitch_rad")?;
        let base_yaw = match yaw {
            Some(yaw_values) => interpolate_schedule(schedule, yaw_values, t, "yaw_rad")?,
            None => 0.0,
        };
        applied_pitch.push(base_pitch + pitch_delta);
        applied_yaw.push(base_yaw + yaw_delta);
    }
    config.schedule_s = Some(pitch_bias.breakpoints.clone());
    config.pitch_rad = Some(applied_pitch);
    config.yaw_rad = Some(applied_yaw);
    Ok(())
}

fn interpolate_schedule(
    schedule: &[f64],
    values: &[f64],
    time_s: f64,
    field: &str,
) -> Result<f64, CliError> {
    if schedule.len() != values.len() || schedule.len() < 2 {
        return Err(CliError::Trajopt {
            summary: format!("DOL verify-run verifier {field} schedule is malformed"),
        });
    }
    if !time_s.is_finite() {
        return Err(CliError::Trajopt {
            summary: "DOL verify-run I-load schedule breakpoint must be finite".to_owned(),
        });
    }
    if time_s <= schedule[0] {
        return Ok(values[0]);
    }
    let last = schedule.len() - 1;
    if time_s >= schedule[last] {
        return Ok(values[last]);
    }
    for index in 0..last {
        let t0 = schedule[index];
        let t1 = schedule[index + 1];
        if time_s >= t0 && time_s <= t1 {
            let alpha = (time_s - t0) / (t1 - t0);
            return Ok(values[index] + alpha * (values[index + 1] - values[index]));
        }
    }
    Err(CliError::Trajopt {
        summary: format!("DOL verify-run verifier {field} interpolation failed"),
    })
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum DolTelemetryReducer {
    First,
    Last,
    Mean,
    Max,
    Min,
    MaxAbs,
}

impl DolTelemetryReducer {
    fn parse(value: &str) -> Result<Self, CliError> {
        match value.trim() {
            "first" => Ok(Self::First),
            "last" => Ok(Self::Last),
            "mean" => Ok(Self::Mean),
            "max" => Ok(Self::Max),
            "min" => Ok(Self::Min),
            "max_abs" => Ok(Self::MaxAbs),
            other => Err(CliError::Trajopt {
                summary: format!(
                    "invalid DOL telemetry reducer `{other}`; expected first, last, mean, max, min, or max_abs"
                ),
            }),
        }
    }
}

#[derive(Debug, Deserialize)]
struct DolTelemetryIndicatorSpecCsvRow {
    name: String,
    design_value: f64,
    telemetry_column: String,
    reducer: String,
    allowed_abs_error: f64,
    #[serde(default)]
    time_min_s: Option<f64>,
    #[serde(default)]
    time_max_s: Option<f64>,
}

#[derive(Clone, Debug, PartialEq)]
struct DolTelemetryIndicatorSpec {
    name: String,
    design_value: f64,
    telemetry_column: String,
    reducer: DolTelemetryReducer,
    allowed_abs_error: f64,
    time_min_s: Option<f64>,
    time_max_s: Option<f64>,
}

fn read_telemetry_indicator_specs_csv(
    path: &Path,
) -> Result<Vec<DolTelemetryIndicatorSpec>, CliError> {
    let mut reader = csv::Reader::from_path(path).map_err(|source| CliError::Csv {
        path: path.to_path_buf(),
        source,
    })?;
    let mut specs = Vec::new();
    for row in reader.deserialize() {
        let row: DolTelemetryIndicatorSpecCsvRow = row.map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
        if row.name.trim().is_empty() || row.telemetry_column.trim().is_empty() {
            return Err(CliError::Trajopt {
                summary: "DOL telemetry indicator specs require nonempty name and telemetry_column"
                    .to_owned(),
            });
        }
        if !row.design_value.is_finite() || !row.allowed_abs_error.is_finite() {
            return Err(CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` has non-finite scalar fields",
                    row.name
                ),
            });
        }
        if row.allowed_abs_error < 0.0 {
            return Err(CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` has negative allowed_abs_error",
                    row.name
                ),
            });
        }
        if let Some(time_min_s) = row.time_min_s
            && !time_min_s.is_finite()
        {
            return Err(CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` has non-finite time_min_s",
                    row.name
                ),
            });
        }
        if let Some(time_max_s) = row.time_max_s
            && !time_max_s.is_finite()
        {
            return Err(CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` has non-finite time_max_s",
                    row.name
                ),
            });
        }
        if let (Some(time_min_s), Some(time_max_s)) = (row.time_min_s, row.time_max_s)
            && time_min_s > time_max_s
        {
            return Err(CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` has time_min_s greater than time_max_s",
                    row.name
                ),
            });
        }
        specs.push(DolTelemetryIndicatorSpec {
            name: row.name,
            design_value: row.design_value,
            telemetry_column: row.telemetry_column,
            reducer: DolTelemetryReducer::parse(&row.reducer)?,
            allowed_abs_error: row.allowed_abs_error,
            time_min_s: row.time_min_s,
            time_max_s: row.time_max_s,
        });
    }
    if specs.is_empty() {
        return Err(CliError::Trajopt {
            summary: "DOL telemetry indicator spec CSV must contain at least one row".to_owned(),
        });
    }
    Ok(specs)
}

fn materialize_indicators_from_telemetry(
    telemetry_csv: &Path,
    specs: &[DolTelemetryIndicatorSpec],
) -> Result<Vec<DolVerificationIndicator>, CliError> {
    let mut reader = csv::Reader::from_path(telemetry_csv).map_err(|source| CliError::Csv {
        path: telemetry_csv.to_path_buf(),
        source,
    })?;
    let headers = reader
        .headers()
        .map_err(|source| CliError::Csv {
            path: telemetry_csv.to_path_buf(),
            source,
        })?
        .clone();
    let time_index = headers.iter().position(|name| name == "time_s");
    let mut columns = Vec::with_capacity(specs.len());
    for spec in specs {
        let value_index = headers
            .iter()
            .position(|name| name == spec.telemetry_column)
            .ok_or_else(|| CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` references missing telemetry column `{}`",
                    spec.name, spec.telemetry_column
                ),
            })?;
        if (spec.time_min_s.is_some() || spec.time_max_s.is_some()) && time_index.is_none() {
            return Err(CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` declares a time window but telemetry CSV has no time_s column",
                    spec.name
                ),
            });
        }
        columns.push(value_index);
    }

    let mut sample_sets = vec![Vec::<f64>::new(); specs.len()];
    for record in reader.records() {
        let record = record.map_err(|source| CliError::Csv {
            path: telemetry_csv.to_path_buf(),
            source,
        })?;
        let time_s = if let Some(index) = time_index {
            let value = record.get(index).unwrap_or("").trim();
            if value.is_empty() {
                None
            } else {
                let parsed = value.parse::<f64>().map_err(|_| CliError::Trajopt {
                    summary: format!("telemetry CSV has non-numeric time_s value `{value}`"),
                })?;
                if !parsed.is_finite() {
                    return Err(CliError::Trajopt {
                        summary: "telemetry CSV has non-finite time_s value".to_owned(),
                    });
                }
                Some(parsed)
            }
        } else {
            None
        };

        for (index, spec) in specs.iter().enumerate() {
            if let Some(time_min_s) = spec.time_min_s
                && time_s.is_none_or(|time| time < time_min_s)
            {
                continue;
            }
            if let Some(time_max_s) = spec.time_max_s
                && time_s.is_none_or(|time| time > time_max_s)
            {
                continue;
            }
            let raw = record.get(columns[index]).unwrap_or("").trim();
            if raw.is_empty() {
                continue;
            }
            let value = raw.parse::<f64>().map_err(|_| CliError::Trajopt {
                summary: format!(
                    "DOL telemetry indicator `{}` saw non-numeric value `{raw}` in column `{}`",
                    spec.name, spec.telemetry_column
                ),
            })?;
            if !value.is_finite() {
                return Err(CliError::Trajopt {
                    summary: format!(
                        "DOL telemetry indicator `{}` saw non-finite value in column `{}`",
                        spec.name, spec.telemetry_column
                    ),
                });
            }
            sample_sets[index].push(value);
        }
    }

    specs
        .iter()
        .zip(sample_sets)
        .map(|(spec, samples)| {
            if samples.is_empty() {
                return Err(CliError::Trajopt {
                    summary: format!(
                        "DOL telemetry indicator `{}` has no finite samples in column `{}`",
                        spec.name, spec.telemetry_column
                    ),
                });
            }
            Ok(DolVerificationIndicator {
                name: spec.name.clone(),
                design_value: spec.design_value,
                verified_value: reduce_telemetry_samples(&samples, spec.reducer),
                allowed_abs_error: spec.allowed_abs_error,
            })
        })
        .collect()
}

fn reduce_telemetry_samples(samples: &[f64], reducer: DolTelemetryReducer) -> f64 {
    match reducer {
        DolTelemetryReducer::First => samples[0],
        DolTelemetryReducer::Last => samples[samples.len() - 1],
        DolTelemetryReducer::Mean => samples.iter().sum::<f64>() / samples.len() as f64,
        DolTelemetryReducer::Max => samples.iter().copied().fold(f64::NEG_INFINITY, f64::max),
        DolTelemetryReducer::Min => samples.iter().copied().fold(f64::INFINITY, f64::min),
        DolTelemetryReducer::MaxAbs => samples.iter().copied().map(f64::abs).fold(0.0, f64::max),
    }
}

fn write_verification_indicators_csv(
    path: &Path,
    indicators: &[DolVerificationIndicator],
) -> Result<(), CliError> {
    let mut writer = csv::Writer::from_path(path).map_err(|source| CliError::Csv {
        path: path.to_path_buf(),
        source,
    })?;
    writer
        .write_record([
            "name",
            "design_value",
            "verified_value",
            "allowed_abs_error",
        ])
        .map_err(|source| CliError::Csv {
            path: path.to_path_buf(),
            source,
        })?;
    for indicator in indicators {
        writer
            .write_record([
                indicator.name.as_str(),
                &format!("{:.17e}", indicator.design_value),
                &format!("{:.17e}", indicator.verified_value),
                &format!("{:.17e}", indicator.allowed_abs_error),
            ])
            .map_err(|source| CliError::Csv {
                path: path.to_path_buf(),
                source,
            })?;
    }
    writer.flush().map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn spliced_day_of_launch_wind(scenario: &Scenario) -> Result<SplicedWindProfile, CliError> {
    let wind = scenario
        .document
        .day_of_launch
        .as_ref()
        .and_then(|dol| dol.wind.as_ref())
        .ok_or_else(|| CliError::Trajopt {
            summary: "scenario must declare [day_of_launch.wind]".to_owned(),
        })?;
    if wind.kind != "spliced" {
        return Err(CliError::Trajopt {
            summary: "DOL redesign currently requires day_of_launch.wind.kind = \"spliced\""
                .to_owned(),
        });
    }
    let splice = wind.splice.as_ref().ok_or_else(|| CliError::Trajopt {
        summary: "spliced day-of-launch wind requires [day_of_launch.wind.splice]".to_owned(),
    })?;
    let lower = wind
        .profiles
        .iter()
        .find(|profile| profile.id == splice.lower_profile_id)
        .ok_or_else(|| CliError::Trajopt {
            summary: format!(
                "missing lower day-of-launch wind profile `{}`",
                splice.lower_profile_id
            ),
        })?;
    let upper = wind
        .profiles
        .iter()
        .find(|profile| profile.id == splice.upper_profile_id)
        .ok_or_else(|| CliError::Trajopt {
            summary: format!(
                "missing upper day-of-launch wind profile `{}`",
                splice.upper_profile_id
            ),
        })?;
    let lower_profile = measured_profile_from_config(lower)?;
    let upper_profile = measured_profile_from_config(upper)?;
    let rule = SpliceRule::new(
        splice.overlap_lower_altitude_m,
        splice.overlap_upper_altitude_m,
    )
    .map_err(|source| CliError::Trajopt {
        summary: source.to_string(),
    })?;
    SplicedWindProfile::from_profiles(&lower_profile, &upper_profile, rule).map_err(|source| {
        CliError::Trajopt {
            summary: source.to_string(),
        }
    })
}

fn measured_profile_from_config(
    profile: &openbmp_scenario::DayOfLaunchMeasuredWindProfileConfig,
) -> Result<MeasuredWindProfile, CliError> {
    let source = match profile.source.as_str() {
        "balloon" => WindSource::Balloon,
        "doppler_profiler" => WindSource::DopplerProfiler,
        "lidar" => WindSource::Lidar,
        "climatology" => WindSource::Climatology,
        "synthetic" => WindSource::Synthetic,
        other => {
            return Err(CliError::Trajopt {
                summary: format!("unsupported day-of-launch wind source `{other}`"),
            });
        }
    };
    let provenance = WindProfileProvenance::new(
        profile
            .provenance_id
            .clone()
            .unwrap_or_else(|| profile.id.clone()),
        profile
            .provenance_uri
            .clone()
            .unwrap_or_else(|| "scenario-inline".to_owned()),
        profile.sha256.clone(),
        profile.local_only,
    )
    .map_err(|source| CliError::Trajopt {
        summary: source.to_string(),
    })?;
    let samples: Vec<_> = profile
        .samples
        .iter()
        .map(|sample| {
            MeasuredWindSample::new(
                sample.altitude_m,
                sample.wind_ned_m_s[0],
                sample.wind_ned_m_s[1],
                sample.wind_ned_m_s[2],
                sample.quality_unit,
            )
        })
        .collect();
    MeasuredWindProfile::new(source, profile.released_at_s, samples, provenance).map_err(|source| {
        CliError::Trajopt {
            summary: source.to_string(),
        }
    })
}

fn digest_file(path: &Path) -> Result<String, CliError> {
    let bytes = std::fs::read(path).map_err(|source| CliError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    Ok(sha256_hex(&bytes))
}

fn sha256_hex(bytes: &[u8]) -> String {
    format!("sha256:{:x}", Sha256::digest(bytes))
}

fn scenario_with_speed(
    base_scenario: &Scenario,
    velocity_direction: [f64; 3],
    speed_m_s: f64,
) -> Scenario {
    let mut scenario = base_scenario.clone();
    scenario.document.vehicle.initial_velocity_eci_m_s = scale3(velocity_direction, speed_m_s);
    scenario
}

fn runner_trajectory(
    scenario: &Scenario,
) -> Result<(BallisticState, Vec<ReferenceProfileSample>), String> {
    let outcome = openbmp_runner::run(scenario).map_err(|source| source.to_string())?;
    trajectory_from_outcome(&outcome)
}

fn trajectory_from_outcome(
    outcome: &RunOutcome,
) -> Result<(BallisticState, Vec<ReferenceProfileSample>), String> {
    let channels = StateChannelIds::from_table(&outcome.table)?;
    let rows = outcome.table.rows();
    if rows.is_empty() {
        return Err("runner produced no telemetry rows".to_owned());
    }
    let mut reference_profile = Vec::with_capacity(rows.len());
    for row in rows {
        let sample = channels.state_sample(row)?;
        reference_profile.push(reference_sample(
            row.time.as_seconds(),
            sample.position,
            sample.velocity,
        )?);
    }
    let Some(last_row) = rows.last() else {
        return Err("runner produced no telemetry rows".to_owned());
    };
    let terminal_sample = channels.state_sample(last_row)?;
    let provenance = ForwardSimulationProvenance::for_state(
        terminal_sample.position,
        terminal_sample.velocity,
        0.0,
        last_row.time,
        ForwardSimulationSource::RunnerTelemetryState,
    );
    let terminal_state = BallisticState::from_forward_simulation(
        terminal_sample.position,
        terminal_sample.velocity,
        0.0,
        last_row.time,
        provenance,
    )
    .map_err(|source| source.to_string())?;
    Ok((terminal_state, reference_profile))
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct StateChannelIds {
    position_x: ChannelId,
    position_y: ChannelId,
    position_z: ChannelId,
    velocity_x: ChannelId,
    velocity_y: ChannelId,
    velocity_z: ChannelId,
}

impl StateChannelIds {
    fn from_table(table: &TelemetryTable) -> Result<Self, String> {
        Ok(Self {
            position_x: channel_id(table, "position_x_m")?,
            position_y: channel_id(table, "position_y_m")?,
            position_z: channel_id(table, "position_z_m")?,
            velocity_x: channel_id(table, "velocity_x_m_s")?,
            velocity_y: channel_id(table, "velocity_y_m_s")?,
            velocity_z: channel_id(table, "velocity_z_m_s")?,
        })
    }

    fn state_sample(self, row: &TelemetryRow) -> Result<StateSample, String> {
        Ok(StateSample {
            position: [
                row_f64(row, self.position_x, "position_x_m")?,
                row_f64(row, self.position_y, "position_y_m")?,
                row_f64(row, self.position_z, "position_z_m")?,
            ],
            velocity: [
                row_f64(row, self.velocity_x, "velocity_x_m_s")?,
                row_f64(row, self.velocity_y, "velocity_y_m_s")?,
                row_f64(row, self.velocity_z, "velocity_z_m_s")?,
            ],
        })
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct StateSample {
    position: [f64; 3],
    velocity: [f64; 3],
}

fn channel_id(table: &TelemetryTable, name: &str) -> Result<ChannelId, String> {
    table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == name)
        .map(|channel| channel.id)
        .ok_or_else(|| format!("runner telemetry is missing channel `{name}`"))
}

fn row_f64(row: &TelemetryRow, channel: ChannelId, name: &str) -> Result<f64, String> {
    match row.get(channel) {
        Some(TelemetryValue::Float64(value)) => Ok(*value),
        other => Err(format!(
            "runner telemetry channel `{name}` has unexpected value {other:?}"
        )),
    }
}

fn reference_sample(
    time_s: f64,
    position: [f64; 3],
    velocity: [f64; 3],
) -> Result<ReferenceProfileSample, String> {
    let radius_m = norm3(position);
    let speed_m_s = norm3(velocity);
    if radius_m <= f64::EPSILON || speed_m_s <= f64::EPSILON {
        return Err("runner reference sample is degenerate".to_owned());
    }
    let radial_speed_m_s = dot3(position, velocity) / radius_m;
    Ok(ReferenceProfileSample {
        time_s,
        radius_m,
        speed_m_s,
        flight_path_angle_rad: (radial_speed_m_s / speed_m_s).clamp(-1.0, 1.0).asin(),
    })
}

fn first_speed_variable(free: &[f64]) -> Result<f64, TrajoptError> {
    let Some(&speed) = free.first() else {
        return Err(TrajoptError::InvalidPayload {
            reason: "scenario apogee correction requires one speed variable",
        });
    };
    if !speed.is_finite() || speed <= 0.0 {
        return Err(TrajoptError::InvalidPayload {
            reason: "scenario apogee speed variable must be finite and positive",
        });
    }
    Ok(speed)
}

fn dot3(a: [f64; 3], b: [f64; 3]) -> f64 {
    a[0] * b[0] + a[1] * b[1] + a[2] * b[2]
}

fn norm3(value: [f64; 3]) -> f64 {
    dot3(value, value).sqrt()
}

fn scale3(value: [f64; 3], scale: f64) -> [f64; 3] {
    [value[0] * scale, value[1] * scale, value[2] * scale]
}
