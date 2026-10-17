//! `openbmp` binary entry point.

use std::io::Write;
use std::process::ExitCode;

use clap::Parser;

use openbmp_cli::cli::{
    CampaignCommand, Cli, Command, DictCommand, McAcousticsAscentTraceDesign, McCommand,
    McWilksSide, MigrateCommand, PackageCommand, ReconstructCommand, SilCommand, SysidCommand,
    TrajoptCommand,
};
use openbmp_cli::commands::{
    campaign, check, compare_telemetry, conform, dict, diff, footprint_mc, mc, migrate, package,
    provenance, reconstruct, run, sil, sysid, trajopt, verify_order,
};
use openbmp_cli::tracing;

fn main() -> ExitCode {
    let cli = Cli::parse();
    tracing::init(cli.trace);

    match dispatch(cli.command) {
        Ok(()) => ExitCode::SUCCESS,
        Err(err) => {
            let exit = err.exit_code();
            // Print the chain so the user sees the underlying cause.
            let mut stderr = std::io::stderr().lock();
            let _ = writeln!(stderr, "openbmp: {err}");
            let mut source = std::error::Error::source(&err);
            while let Some(cause) = source {
                let _ = writeln!(stderr, "  caused by: {cause}");
                source = cause.source();
            }
            ExitCode::from(exit)
        }
    }
}

#[allow(clippy::too_many_lines)]
fn dispatch(command: Command) -> Result<(), openbmp_cli::CliError> {
    match command {
        Command::Run {
            scenario,
            output_csv,
            output_json,
            output_parquet,
            run_index_jsonl,
            run_index_parquet,
            run_code_id,
            run_verdict_rules,
            run_verdict_json,
            run_verdict_baseline_jsonl,
        } => {
            let overrides = run::OutputOverrides::new(output_csv, output_json, output_parquet);
            let run_data = run::RunDataOptions::new(
                run_index_jsonl,
                run_index_parquet,
                run_code_id,
                run_verdict_rules,
                run_verdict_json,
                run_verdict_baseline_jsonl,
            );
            let report = run::run_with_options(&scenario, &overrides, &run_data)?;
            println!(
                "openbmp run: ok — {} steps, t = {:.6} s, stop = {}",
                report.final_step, report.final_time_s, report.stop_label,
            );
            if let Some(realtime) = &report.realtime {
                let rtf = realtime
                    .real_time_factor
                    .map_or_else(|| "n/a".to_owned(), |value| format!("{value:.6}"));
                let jitter = realtime.jitter.as_ref().map_or_else(
                    || "jitter=n/a".to_owned(),
                    |summary| {
                        format!(
                            "jitter_ns[p50={}, p99={}, p99.9={}, max={}]",
                            summary.p50_ns, summary.p99_ns, summary.p999_ns, summary.max_ns
                        )
                    },
                );
                println!(
                    "  realtime: mode={}, frames={}, rtf={}, overruns={}, {}",
                    realtime.mode, realtime.frame_count, rtf, realtime.overrun_count, jitter,
                );
                if let Some(execution) = &realtime.frame_execution {
                    let budget = execution
                        .wall_budget_ns
                        .map_or_else(|| "n/a".to_owned(), |value| value.to_string());
                    println!(
                        "  realtime execution: frame_ns[p50={}, p99={}, p99.9={}, max={}], budget={}, budget_overruns={}",
                        execution.p50_ns,
                        execution.p99_ns,
                        execution.p999_ns,
                        execution.max_ns,
                        budget,
                        execution.over_budget_count,
                    );
                }
            }
            if let Some(actuator_stream) = &report.actuator_stream {
                println!(
                    "  actuator stream: packets={}, sha256={}",
                    actuator_stream.packet_count, actuator_stream.sha256_hex,
                );
            }
            if let Some(afts) = &report.afts {
                let rule = afts.rule_id.as_deref().unwrap_or("none");
                println!(
                    "  AFTS: samples={}, terminate={}, rule={}",
                    afts.samples, afts.terminate, rule,
                );
            }
            for path in &report.written {
                println!("  wrote {}", path.display());
            }
            if let Some(record) = &report.run_record {
                println!(
                    "  run record: id={}, sha256={}",
                    record.run_id, record.record_sha256_hex
                );
            }
            if let Some(verdict) = &report.verdict_report {
                println!(
                    "  verdict report: passed={}, count={}",
                    verdict.passed, verdict.verdict_count
                );
            }
            Ok(())
        }
        Command::Diff {
            golden,
            actual,
            report_json,
        } => {
            let report = diff::run(&golden, &actual)?;
            if let Some(path) = report_json {
                let text = serde_json::to_string_pretty(&report).map_err(|source| {
                    openbmp_cli::CliError::DiffReportJson {
                        path: path.clone(),
                        source,
                    }
                })?;
                std::fs::write(&path, text)
                    .map_err(|source| openbmp_cli::CliError::Io { path, source })?;
            }
            if report.identical {
                println!(
                    "openbmp diff: identical ({} rows, {} columns matched)",
                    report.golden_rows, report.columns_matched,
                );
                Ok(())
            } else {
                let divergence =
                    report
                        .first_divergence
                        .ok_or_else(|| openbmp_cli::CliError::Diff {
                            summary: "row counts differ but no first-divergence cell was recorded"
                                .to_owned(),
                        })?;
                let summary = format!(
                    "first divergence at row {} column {}: golden={} actual={} (golden_rows={}, actual_rows={})",
                    divergence.row,
                    divergence.column,
                    divergence.golden,
                    divergence.actual,
                    report.golden_rows,
                    report.actual_rows,
                );
                Err(openbmp_cli::CliError::Diff { summary })
            }
        }
        Command::Check { scenario } => {
            let report = check::run(&scenario)?;
            println!(
                "openbmp check: ok — {} ({})",
                report.scenario_name, report.validation_label,
            );
            for (key, path) in report.resolved_paths {
                println!("  {key} -> {path}");
            }
            for entry in report.resolved_files {
                println!(
                    "  {} -> {} (sha256:{})",
                    entry.field, entry.path, entry.sha256_hex,
                );
            }
            Ok(())
        }
        Command::Conform { scenarios } => {
            let report = conform::run(&scenarios)?;
            println!(
                "openbmp conform: ok — {} scenario(s)",
                report.scenarios.len()
            );
            for scenario in report.scenarios {
                println!(
                    "  {}: {} ({}) — {} steps, t = {:.6} s, stop = {}",
                    scenario.path.display(),
                    scenario.scenario_name,
                    scenario.validation_label,
                    scenario.final_step,
                    scenario.final_time_s,
                    scenario.stop_label,
                );
            }
            Ok(())
        }
        Command::VerifyOrder {
            method,
            campaign_cases,
            output_toml,
        } => {
            let report =
                verify_order::run_campaign(method, campaign_cases, output_toml.as_deref())?;
            println!(
                "openbmp verify-order: ok - {} method-case(s), {} campaign case(s)",
                report.methods.len(),
                report.campaign_cases,
            );
            for method in &report.methods {
                println!(
                    "  {} {}: p(coarse/medium)={:.6}, p(medium/fine)={:.6}, gci_fine={:.12e}, passed={}",
                    method.case_id,
                    method.report.method,
                    method.report.observed_order_coarse_medium,
                    method.report.observed_order_medium_fine,
                    method.report.gci_fine,
                    method.passed,
                );
            }
            if let Some(path) = report.output_toml {
                println!("  wrote verify-order report {}", path.display());
            }
            Ok(())
        }
        Command::Reconstruct { command } => match command {
            ReconstructCommand::SyntheticLinear { output_toml } => {
                let report = reconstruct::run_synthetic_linear(output_toml.as_deref())?;
                println!(
                    "openbmp reconstruct synthetic-linear: ok - NEES {:.6}, NIS {:.6}, batch_error={:.12e}, passed={}",
                    report.reconstruction.nees.in_bounds_fraction,
                    report.reconstruction.nis.in_bounds_fraction,
                    report.reconstruction.batch_update_error_norm,
                    report.reconstruction.passed,
                );
                if let Some(path) = report.output_toml {
                    println!("  wrote reconstruction report {}", path.display());
                }
                Ok(())
            }
            ReconstructCommand::ObserveFc {
                scenario,
                output_toml,
            } => {
                let report = reconstruct::run_observe_fc(&scenario, output_toml.as_deref())?;
                println!(
                    "openbmp reconstruct observe-fc: ok - ticks={}, estimator_samples={}, nis_reports={}, nees_reports={}",
                    report.ticks_observed,
                    report.estimator_samples,
                    report.nis_reports.len(),
                    report.nees_reports.len(),
                );
                for nis in &report.nis_reports {
                    println!(
                        "  {}: samples={}, in_bounds_fraction={:.6}, mean_chi2={:.6e}, max_chi2={:.6e}",
                        nis.report.label,
                        nis.report.samples,
                        nis.report.in_bounds_fraction,
                        nis.mean_chi2,
                        nis.max_chi2,
                    );
                }
                for nees in &report.nees_reports {
                    println!(
                        "  {}: samples={}, in_bounds_fraction={:.6}, mean_chi2={:.6e}, max_chi2={:.6e}",
                        nees.report.label,
                        nees.report.samples,
                        nees.report.in_bounds_fraction,
                        nees.mean_chi2,
                        nees.max_chi2,
                    );
                }
                if let Some(path) = report.output_toml {
                    println!("  wrote FC observation report {}", path.display());
                }
                Ok(())
            }
            ReconstructCommand::ExportTrajectory {
                scenario,
                output_csv,
            } => {
                let report = reconstruct::run_export_trajectory(&scenario, &output_csv)?;
                println!(
                    "openbmp reconstruct export-trajectory: ok - samples={}, final_time_s={:.6}, stop={}",
                    report.samples, report.final_time_s, report.stop_label,
                );
                println!("  wrote trajectory CSV {}", report.output_csv.display());
                Ok(())
            }
            ReconstructCommand::ExportTrajectoryMapping { output_toml, pack } => {
                let report = reconstruct::run_export_trajectory_mapping(&output_toml, pack)?;
                println!(
                    "openbmp reconstruct export-trajectory-mapping: ok - pack={:?}, metrics={}, time_tol={:.6e}, pos_tol={:.6e}, vel_tol={:.6e}",
                    report.pack,
                    report.metrics,
                    report.time_tolerance_s,
                    report.position_tolerance_m,
                    report.velocity_tolerance_m_s,
                );
                println!(
                    "  wrote trajectory mapping {}",
                    report.output_toml.display()
                );
                Ok(())
            }
            ReconstructCommand::LocalWorkflow {
                scenario,
                output_toml,
                trajectory_csv,
                mapping_toml,
                external_reference_csv,
                pack,
            } => {
                let report = reconstruct::run_local_workflow(
                    &scenario,
                    &output_toml,
                    &trajectory_csv,
                    &mapping_toml,
                    &external_reference_csv,
                    pack,
                )?;
                println!(
                    "openbmp reconstruct local-workflow: ok - pack={:?}, commands={}, local_only=true",
                    report.pack,
                    report.commands.len(),
                );
                println!("  wrote local workflow {}", report.output_toml.display());
                Ok(())
            }
        },
        Command::Trajopt { command } => match command {
            TrajoptCommand::CorrectApogee {
                initial_radius_m,
                target_apogee_radius_m,
                initial_speed_m_s,
                coast_duration_s,
                step_s,
                mu_m3_s2,
                residual_tolerance_m,
                max_iterations,
                synthesis_seed,
                scenario_digest,
                source_revision,
                producer,
                output_iload,
            } => {
                let report = trajopt::run_correct_apogee(
                    trajopt::CorrectApogeeArgs {
                        initial_radius_m,
                        target_apogee_radius_m,
                        initial_speed_m_s,
                        coast_duration_s,
                        step_s,
                        mu_m3_s2,
                        residual_tolerance_m,
                        max_iterations,
                        synthesis_seed,
                        scenario_digest,
                        source_revision,
                        producer,
                    },
                    &output_iload,
                )?;
                println!(
                    "openbmp trajopt correct-apogee: ok - corrected_speed_m_s={:.12}, residual_norm_m={:.12e}, iterations={}, reference_samples={}, iload_bytes={}",
                    report.corrected_speed_m_s,
                    report.residual_norm_m,
                    report.iterations,
                    report.reference_samples,
                    report.iload_bytes,
                );
                println!("  wrote I-load {}", report.output_iload.display());
                Ok(())
            }
            TrajoptCommand::CorrectApogeeScenario {
                scenario,
                target_apogee_radius_m,
                initial_speed_m_s,
                mu_m3_s2,
                residual_tolerance_m,
                max_iterations,
                synthesis_seed,
                scenario_digest,
                source_revision,
                producer,
                output_iload,
            } => {
                let report = trajopt::run_correct_apogee_scenario(
                    trajopt::CorrectApogeeScenarioArgs {
                        scenario,
                        target_apogee_radius_m,
                        initial_speed_m_s,
                        mu_m3_s2,
                        residual_tolerance_m,
                        max_iterations,
                        synthesis_seed,
                        scenario_digest,
                        source_revision,
                        producer,
                    },
                    &output_iload,
                )?;
                println!(
                    "openbmp trajopt correct-apogee-scenario: ok - corrected_speed_m_s={:.12}, residual_norm_m={:.12e}, iterations={}, reference_samples={}, iload_bytes={}",
                    report.corrected_speed_m_s,
                    report.residual_norm_m,
                    report.iterations,
                    report.reference_samples,
                    report.iload_bytes,
                );
                println!("  wrote I-load {}", report.output_iload.display());
                Ok(())
            }
            TrajoptCommand::DolRedesign {
                scenario,
                target_apogee_radius_m,
                load_window_min_altitude_m,
                load_window_max_altitude_m,
                airspeed_m_s,
                alpha_reference_rad,
                beta_reference_rad,
                stale_pitch_bias_rad,
                stale_yaw_bias_rad,
                load_limit_angle_rad,
                insertion_residual_norm_m,
                insertion_residual_limit_m,
                synthesis_seed,
                scenario_digest,
                design_profile_digest,
                source_revision,
                producer,
                solver_provenance,
                output_iload,
                report_json,
            } => {
                let report = trajopt::run_dol_redesign(
                    trajopt::DolRedesignArgs {
                        scenario,
                        target_apogee_radius_m,
                        load_window_min_altitude_m,
                        load_window_max_altitude_m,
                        airspeed_m_s,
                        alpha_reference_rad,
                        beta_reference_rad,
                        stale_pitch_bias_rad,
                        stale_yaw_bias_rad,
                        load_limit_angle_rad,
                        insertion_residual_norm_m,
                        insertion_residual_limit_m,
                        synthesis_seed,
                        scenario_digest,
                        design_profile_digest,
                        source_revision,
                        producer,
                        solver_provenance,
                    },
                    &output_iload,
                    report_json.as_deref(),
                )?;
                println!(
                    "openbmp trajopt dol-redesign: ok - stale_min_margin_rad={:.12e}, redesigned_min_margin_rad={:.12e}, not_worse_fraction={:.6}, load_window_samples={}, iload_bytes={}",
                    report.artifact.stale_min_margin_rad,
                    report.artifact.redesigned_min_margin_rad,
                    report.artifact.redesigned_not_worse_fraction,
                    report.artifact.load_window_sample_count,
                    report.iload_bytes,
                );
                println!("  wrote I-load {}", report.output_iload.display());
                if let Some(path) = report.report_json {
                    println!("  wrote DOL report {}", path.display());
                }
                Ok(())
            }
            TrajoptCommand::DolReportCard {
                update_report_json,
                indicators_csv,
                report_id,
                verification_provenance,
                min_required_margin_rad,
                min_not_worse_fraction,
                output_csv,
                report_json,
            } => {
                let report = trajopt::run_dol_report_card(
                    trajopt::DolReportCardArgs {
                        update_report_json,
                        indicators_csv,
                        report_id,
                        verification_provenance,
                        min_required_margin_rad,
                        min_not_worse_fraction,
                    },
                    &output_csv,
                    report_json.as_deref(),
                )?;
                println!(
                    "openbmp trajopt dol-report-card: {} - redesigned_min_margin_rad={:.12e}, not_worse_fraction={:.6}, indicators={}, binding={}",
                    match report.report_card.verdict {
                        openbmp_trajopt::DolReportVerdict::Go => "go",
                        openbmp_trajopt::DolReportVerdict::NoGo => "no_go",
                    },
                    report.report_card.redesigned_min_margin_rad,
                    report.report_card.redesigned_not_worse_fraction,
                    report.report_card.indicators.len(),
                    report
                        .report_card
                        .binding_indicator
                        .as_deref()
                        .unwrap_or(""),
                );
                println!(
                    "  wrote DOL report card CSV {}",
                    report.output_csv.display()
                );
                if let Some(path) = report.report_json {
                    println!("  wrote DOL report card JSON {}", path.display());
                }
                if report.report_card.verdict == openbmp_trajopt::DolReportVerdict::NoGo {
                    return Err(openbmp_cli::CliError::Trajopt {
                        summary: format!(
                            "DOL report card verdict is no_go: binding={}",
                            report
                                .report_card
                                .binding_indicator
                                .as_deref()
                                .unwrap_or("unknown")
                        ),
                    });
                }
                Ok(())
            }
            TrajoptCommand::DolCommitReadiness {
                report_card_json,
                persistence_json,
                cola_json,
                readiness_id,
                output_csv,
                report_json,
            } => {
                let report = trajopt::run_dol_commit_readiness(
                    trajopt::DolCommitReadinessArgs {
                        report_card_json,
                        persistence_json,
                        cola_json,
                        readiness_id,
                    },
                    &output_csv,
                    report_json.as_deref(),
                )?;
                println!(
                    "openbmp trajopt dol-commit-readiness: {} - gates={}, binding_gate={}, binding_detail={}",
                    match report.readiness.verdict {
                        openbmp_trajopt::DolReportVerdict::Go => "go",
                        openbmp_trajopt::DolReportVerdict::NoGo => "no_go",
                    },
                    report.readiness.gates.len(),
                    report.readiness.binding_gate.as_deref().unwrap_or(""),
                    report.readiness.binding_detail.as_deref().unwrap_or(""),
                );
                println!(
                    "  wrote DOL commit-readiness CSV {}",
                    report.output_csv.display()
                );
                if let Some(path) = report.report_json {
                    println!("  wrote DOL commit-readiness JSON {}", path.display());
                }
                if report.readiness.verdict == openbmp_trajopt::DolReportVerdict::NoGo {
                    return Err(openbmp_cli::CliError::Trajopt {
                        summary: format!(
                            "DOL commit readiness verdict is no_go: gate={}, binding={}",
                            report
                                .readiness
                                .binding_gate
                                .as_deref()
                                .unwrap_or("unknown"),
                            report
                                .readiness
                                .binding_detail
                                .as_deref()
                                .unwrap_or("unknown")
                        ),
                    });
                }
                Ok(())
            }
            TrajoptCommand::DolMaterializeIndicators {
                telemetry_csv,
                spec_csv,
                output_csv,
            } => {
                let report = trajopt::run_dol_materialize_indicators(
                    trajopt::DolIndicatorMaterializationArgs {
                        telemetry_csv,
                        spec_csv,
                    },
                    &output_csv,
                )?;
                println!(
                    "openbmp trajopt dol-materialize-indicators: ok - indicators={}",
                    report.indicators.len()
                );
                println!("  wrote DOL indicator CSV {}", report.output_csv.display());
                Ok(())
            }
            TrajoptCommand::DolVerifyRun {
                scenario,
                update_report_json,
                updated_iload,
                indicator_spec_csv,
                verifier_design_profile_digest,
                report_id,
                verification_provenance,
                min_required_margin_rad,
                min_not_worse_fraction,
                output_telemetry_csv,
                output_indicators_csv,
                output_report_csv,
                report_json,
            } => {
                let report = trajopt::run_dol_verify_run(
                    trajopt::DolVerifyRunArgs {
                        scenario,
                        update_report_json,
                        updated_iload,
                        verifier_design_profile_digest,
                        indicator_spec_csv,
                        report_id,
                        verification_provenance,
                        min_required_margin_rad,
                        min_not_worse_fraction,
                    },
                    &output_telemetry_csv,
                    &output_indicators_csv,
                    &output_report_csv,
                    report_json.as_deref(),
                )?;
                println!(
                    "openbmp trajopt dol-verify-run: {} - steps={}, t={:.6} s, stop={}, indicators={}, binding={}, design_profile_digest={}, iload_sha256={}, iload_application={}, applied_refs={}, pitch_points={}, yaw_points={}",
                    match report.report_card.verdict {
                        openbmp_trajopt::DolReportVerdict::Go => "go",
                        openbmp_trajopt::DolReportVerdict::NoGo => "no_go",
                    },
                    report.run_final_step,
                    report.run_final_time_s,
                    report.run_stop_label,
                    report.indicators.len(),
                    report
                        .report_card
                        .binding_indicator
                        .as_deref()
                        .unwrap_or(""),
                    report.verifier_design_profile_digest,
                    report.updated_iload_sha256_hex,
                    report.iload_application.mode,
                    report.iload_application.updated_ascent_reference_count,
                    report.iload_application.pitch_bias_points,
                    report.iload_application.yaw_bias_points,
                );
                println!(
                    "  wrote verifier telemetry {}",
                    report.telemetry_csv.display()
                );
                println!(
                    "  wrote DOL indicator CSV {}",
                    report.indicators_csv.display()
                );
                println!(
                    "  wrote DOL report card CSV {}",
                    report.report_csv.display()
                );
                if let Some(path) = report.report_json {
                    println!("  wrote DOL report card JSON {}", path.display());
                }
                if report.report_card.verdict == openbmp_trajopt::DolReportVerdict::NoGo {
                    return Err(openbmp_cli::CliError::Trajopt {
                        summary: format!(
                            "DOL verify-run report card verdict is no_go: binding={}",
                            report
                                .report_card
                                .binding_indicator
                                .as_deref()
                                .unwrap_or("unknown")
                        ),
                    });
                }
                Ok(())
            }
            TrajoptCommand::DolVerifySil {
                package,
                case,
                update_report_json,
                updated_iload,
                indicator_spec_csv,
                verifier_design_profile_digest,
                report_id,
                verification_provenance,
                min_required_margin_rad,
                min_not_worse_fraction,
                output_telemetry_csv,
                output_indicators_csv,
                output_report_csv,
                output_evidence_dir,
                report_json,
            } => {
                let report = trajopt::run_dol_verify_sil(
                    trajopt::DolVerifySilArgs {
                        package,
                        case,
                        update_report_json,
                        updated_iload,
                        verifier_design_profile_digest,
                        indicator_spec_csv,
                        report_id,
                        verification_provenance,
                        min_required_margin_rad,
                        min_not_worse_fraction,
                    },
                    &output_telemetry_csv,
                    &output_indicators_csv,
                    &output_report_csv,
                    &output_evidence_dir,
                    report_json.as_deref(),
                )?;
                println!(
                    "openbmp trajopt dol-verify-sil: {} - package={}, case={}, steps={}, t={:.6} s, stop={}, indicators={}, binding={}, design_profile_digest={}, iload_sha256={}, iload_application={}, applied_refs={}, pitch_points={}, yaw_points={}, sil_stimuli={}",
                    match report.report_card.verdict {
                        openbmp_trajopt::DolReportVerdict::Go => "go",
                        openbmp_trajopt::DolReportVerdict::NoGo => "no_go",
                    },
                    report.package_id,
                    report.case_id,
                    report.run_final_step,
                    report.run_final_time_s,
                    report.run_stop_label,
                    report.indicators.len(),
                    report
                        .report_card
                        .binding_indicator
                        .as_deref()
                        .unwrap_or(""),
                    report.verifier_design_profile_digest,
                    report.updated_iload_sha256_hex,
                    report.iload_application.mode,
                    report.iload_application.updated_ascent_reference_count,
                    report.iload_application.pitch_bias_points,
                    report.iload_application.yaw_bias_points,
                    report.sil_stimuli,
                );
                println!(
                    "  wrote SIL verifier telemetry {}",
                    report.telemetry_csv.display()
                );
                println!(
                    "  wrote DOL indicator CSV {}",
                    report.indicators_csv.display()
                );
                println!(
                    "  wrote DOL report card CSV {}",
                    report.report_csv.display()
                );
                println!(
                    "  wrote SIL evidence manifest {}",
                    report.evidence_manifest.display()
                );
                if let Some(path) = report.report_json {
                    println!("  wrote DOL report card JSON {}", path.display());
                }
                if report.report_card.verdict == openbmp_trajopt::DolReportVerdict::NoGo {
                    return Err(openbmp_cli::CliError::Trajopt {
                        summary: format!(
                            "DOL verify-sil report card verdict is no_go: binding={}",
                            report
                                .report_card
                                .binding_indicator
                                .as_deref()
                                .unwrap_or("unknown")
                        ),
                    });
                }
                Ok(())
            }
        },
        Command::CompareTelemetry {
            scenario,
            reference_csv,
            mapping,
            report_json,
        } => {
            let report = compare_telemetry::run(&scenario, &reference_csv, &mapping)?;
            println!(
                "openbmp compare-telemetry: {} — {} metrics, {} reference rows, stop = {} at t = {:.6} s",
                if report.passed() { "ok" } else { "FAILED" },
                report.metrics.len(),
                report.reference_rows,
                report.stop_label,
                report.final_time_s,
            );
            for metric in &report.metrics {
                let max_error_at = metric
                    .max_abs_error_time_s
                    .map_or_else(|| "n/a".to_owned(), |time_s| format!("{time_s:.3} s"));
                let max_error_values =
                    match (metric.max_abs_error_reference, metric.max_abs_error_actual) {
                        (Some(reference), Some(actual)) => format!(
                            ", reference={reference:.6e}, actual={actual:.6e}, signed_error={:.6e}",
                            actual - reference
                        ),
                        _ => String::new(),
                    };
                println!(
                    "  {}: compared={}, skipped={}, max_abs_error={:.6e} at {}{}, rms_abs_error={:.6e}, exceedances={}",
                    metric.id,
                    metric.compared_samples,
                    metric.skipped_samples,
                    metric.max_abs_error,
                    max_error_at,
                    max_error_values,
                    metric.rms_abs_error,
                    metric.exceedances,
                );
            }
            if let Some(path) = report_json {
                let text = serde_json::to_string_pretty(&report).map_err(|source| {
                    openbmp_cli::CliError::TelemetryCompareReportJson {
                        path: path.clone(),
                        source,
                    }
                })?;
                std::fs::write(&path, text).map_err(|source| openbmp_cli::CliError::Io {
                    path: path.clone(),
                    source,
                })?;
                println!("  wrote report {}", path.display());
            }
            if report.passed() {
                Ok(())
            } else {
                Err(openbmp_cli::CliError::TelemetryCompare {
                    summary: report.failure_summary(),
                })
            }
        }
        Command::Sysid { command } => match command {
            SysidCommand::Residuals {
                reference_csv,
                actual_csv,
                mapping,
                report_json,
            } => {
                let report =
                    sysid::run_residuals(&reference_csv, &actual_csv, &mapping, &report_json)?;
                println!(
                    "openbmp sysid residuals: {} - {} channel(s), out_of_family={}",
                    if report.any_out_of_family {
                        "FAILED"
                    } else {
                        "ok"
                    },
                    report.channel_count,
                    report.any_out_of_family,
                );
                for channel in &report.channels {
                    println!(
                        "  {}: compared={}, skipped={}, max_abs_error={:.6e}, rms_abs_error={:.6e}, exceedances={}",
                        channel.id,
                        channel.total.compared_samples,
                        channel.total.skipped_samples,
                        channel.total.max_abs_error,
                        channel.total.rms_abs_error,
                        channel.total.exceedances,
                    );
                }
                println!("  wrote report {}", report_json.display());
                if report.any_out_of_family {
                    Err(openbmp_cli::CliError::Sysid {
                        summary: report.failure_summary(),
                    })
                } else {
                    Ok(())
                }
            }
        },
        Command::FootprintMc {
            scenario,
            checkpoint_json,
            max_new_samples,
            toolchain_profile,
            uq_toml,
            credibility_floor,
            credibility_report_md,
        } => {
            let credibility_options = build_mc_credibility_options(
                uq_toml.as_ref(),
                &credibility_floor,
                credibility_report_md.as_ref(),
            )?;
            let report = footprint_mc::run(
                &scenario,
                checkpoint_json.as_deref(),
                max_new_samples,
                &toolchain_profile,
                credibility_options,
            )?;
            println!(
                "openbmp footprint-mc: ok - {} requested, {} completed, {} succeeded, {} failed",
                report.samples_requested,
                report.samples_completed,
                report.samples_succeeded,
                report.samples_failed,
            );
            if !report.complete {
                println!("  checkpoint incomplete; final outputs were not written");
            }
            if let Some(credibility) = &report.credibility {
                println!(
                    "  credibility: binding={}, floor={}, label={}, aggregate_1sigma={:.12e}, accepted={}",
                    credibility.binding_level.as_label(),
                    credibility.floor.as_label(),
                    credibility.legacy_label.as_label(),
                    credibility.aggregate_one_sigma,
                    credibility.accepted,
                );
            }
            if let Some(path) = &report.credibility_report_md {
                println!("  wrote credibility report {}", path.display());
            }
            for path in report.written {
                println!("  wrote {}", path.display());
            }
            Ok(())
        }
        Command::Mc { command } => match command {
            McCommand::Summarize {
                samples_csv,
                value_column,
                success_column,
                confidence,
                uq_toml,
                credibility_floor,
                credibility_report_md,
            } => {
                let credibility_options = build_mc_credibility_options(
                    uq_toml.as_ref(),
                    &credibility_floor,
                    credibility_report_md.as_ref(),
                )?;
                let report = mc::summarize_with_credibility(
                    &samples_csv,
                    &value_column,
                    success_column.as_deref(),
                    confidence,
                    credibility_options,
                )?;
                let summary = report.summary;
                println!(
                    "openbmp mc summarize: ok — {} samples, mean = {:.12e}",
                    summary.samples, summary.mean,
                );
                if let Some(variance) = summary.sample_variance {
                    println!("  sample_variance = {variance:.12e}");
                }
                if let Some(standard_error) = summary.standard_error {
                    println!("  standard_error = {standard_error:.12e}");
                }
                if let Some(success) = summary.success {
                    println!(
                        "  success = {}/{} ({:.6}), {:.3} CI [{:.6}, {:.6}]",
                        success.successes,
                        success.trials,
                        success.fraction,
                        success.confidence,
                        success.lower,
                        success.upper,
                    );
                }
                if let Some(credibility) = report.credibility {
                    println!(
                        "  credibility: binding={}, floor={}, label={}, aggregate_1sigma={:.12e}, accepted={}",
                        credibility.binding_level.as_label(),
                        credibility.floor.as_label(),
                        credibility.legacy_label.as_label(),
                        credibility.aggregate_one_sigma,
                        credibility.accepted,
                    );
                    if let Some(path) = report.credibility_report_md {
                        println!("  wrote credibility report {}", path.display());
                    }
                }
                Ok(())
            }
            McCommand::NestedSummarize {
                samples_csv,
                epistemic_column,
                value_column,
                threshold,
                minimum_probability,
                pbox_csv,
            } => {
                let report = mc::summarize_nested(
                    &samples_csv,
                    &epistemic_column,
                    &value_column,
                    threshold,
                    minimum_probability,
                    pbox_csv.as_deref(),
                )?;
                let analysis = report.analysis;
                println!(
                    "openbmp mc nested-summarize: ok - {} epistemic conditions, aleatory samples min={}, max={}",
                    analysis.epistemic_samples,
                    analysis.min_aleatory_samples,
                    analysis.max_aleatory_samples,
                );
                println!(
                    "  variance: aleatory={:.12e}, epistemic={:.12e}, total={:.12e}",
                    analysis.variance_split.aleatory,
                    analysis.variance_split.epistemic,
                    analysis.variance_split.total,
                );
                if let (Some(probability), Some(passed)) = (
                    analysis.lower_bound_probability,
                    analysis.requirement_passed,
                ) {
                    println!(
                        "  lower_pbox P(y <= {:.12e}) = {:.12e}; required >= {:.12e}; passed={}",
                        threshold, probability, minimum_probability, passed,
                    );
                }
                if let Some(path) = report.pbox_csv {
                    println!("  wrote p-box {}", path.display());
                }
                Ok(())
            }
            McCommand::RareEvent {
                scenario,
                output_toml,
            } => {
                let report = mc::run_rare_event_scenario(mc::McRareEventScenarioOptions {
                    scenario_path: &scenario,
                    output_toml: output_toml.as_deref(),
                })?;
                println!(
                    "openbmp mc rare-event: ok - {} estimate(s), label={}, dimension={}, seed={}",
                    report.estimates.len(),
                    report.limit_state_label,
                    report.dimension,
                    report.seed,
                );
                for estimate in &report.estimates {
                    println!(
                        "  {}: p_fail={:.12e}, cov={:.6}, evaluations={}",
                        estimate.method.as_str(),
                        estimate.failure_probability,
                        estimate.coefficient_of_variation,
                        estimate.evaluations,
                    );
                    if let Some(error) = estimate.abs_log10_error {
                        println!("    abs_log10_error = {error:.6}");
                    }
                }
                if let Some(path) = report.output_toml {
                    println!("  wrote rare-event report {}", path.display());
                }
                Ok(())
            }
            McCommand::Wilks {
                coverage,
                confidence,
                side,
            } => {
                let side = match side {
                    McWilksSide::OneSided => mc::WilksSide::OneSided,
                    McWilksSide::TwoSided => mc::WilksSide::TwoSided,
                };
                let report = mc::wilks_sample_size(side, coverage, confidence)?;
                println!(
                    "openbmp mc wilks: ok — {} N = {}, coverage = {:.8}, confidence = {:.8}, attained = {:.8}",
                    report.side.label(),
                    report.samples,
                    report.coverage,
                    report.confidence,
                    report.attained_confidence,
                );
                Ok(())
            }
            McCommand::Lhs {
                samples,
                dimensions,
                seed,
                output_csv,
            } => {
                let report = mc::write_latin_hypercube(samples, dimensions, seed, &output_csv)?;
                println!(
                    "openbmp mc lhs: ok — wrote {} samples x {} dimensions to {}",
                    report.samples,
                    report.dimensions,
                    report.output_csv.display(),
                );
                Ok(())
            }
            McCommand::Sobol {
                samples,
                dimensions,
                scramble_seed,
                output_csv,
            } => {
                let report = mc::write_sobol(samples, dimensions, scramble_seed, &output_csv)?;
                if let Some(seed) = report.scramble_seed {
                    println!(
                        "openbmp mc sobol: ok — wrote {} samples x {} dimensions to {} (owen scramble seed {seed})",
                        report.samples,
                        report.dimensions,
                        report.output_csv.display(),
                    );
                } else {
                    println!(
                        "openbmp mc sobol: ok — wrote {} samples x {} dimensions to {}",
                        report.samples,
                        report.dimensions,
                        report.output_csv.display(),
                    );
                }
                Ok(())
            }
            McCommand::ImanConover {
                design_csv,
                correlation_csv,
                output_csv,
            } => {
                let report = mc::write_iman_conover(&design_csv, &correlation_csv, &output_csv)?;
                println!(
                    "openbmp mc iman-conover: ok — wrote {} samples x {} dimensions to {}",
                    report.samples,
                    report.dimensions,
                    report.output_csv.display(),
                );
                Ok(())
            }
            McCommand::AeroDeckDrawOverlays {
                samples,
                campaign_seed,
                first_sample_index,
                dimension_id_base,
                mach_correlation_length,
                alpha_correlation_length_deg,
                beta_correlation_length_deg,
                overlay_prefix,
                output_dir,
                manifest_toml,
            } => {
                let report =
                    mc::write_aero_deck_draw_overlays(mc::McAeroDeckDrawOverlaysOptions {
                        samples,
                        campaign_seed,
                        first_sample_index,
                        dimension_id_base,
                        mach_correlation_length,
                        alpha_correlation_length_deg,
                        beta_correlation_length_deg,
                        overlay_prefix: &overlay_prefix,
                        output_dir: &output_dir,
                        manifest_toml: &manifest_toml,
                    })?;
                println!(
                    "openbmp mc aero-deck-draw-overlays: ok - wrote {} overlays under {}",
                    report.samples,
                    report.output_dir.display(),
                );
                println!("  manifest fragment: {}", report.manifest_toml.display());
                Ok(())
            }
            McCommand::AeroDeckDrawCampaign {
                scenario,
                samples,
                campaign_seed,
                first_sample_index,
                dimension_id_base,
                mach_correlation_length,
                alpha_correlation_length_deg,
                beta_correlation_length_deg,
                overlay_prefix,
                campaign_id,
                scenario_id,
                run_seed,
                output_dir,
                run_code_id,
                code_ordinal,
                write_index_parquet,
                workers,
                uq_toml,
                credibility_floor,
                credibility_report_md,
            } => {
                let credibility_options = build_mc_credibility_options(
                    uq_toml.as_ref(),
                    &credibility_floor,
                    credibility_report_md.as_ref(),
                )?;
                let report = mc::run_aero_deck_draw_campaign(mc::McAeroDeckDrawCampaignOptions {
                    scenario_path: &scenario,
                    samples,
                    campaign_seed,
                    first_sample_index,
                    dimension_id_base,
                    mach_correlation_length,
                    alpha_correlation_length_deg,
                    beta_correlation_length_deg,
                    overlay_prefix: &overlay_prefix,
                    campaign_id: &campaign_id,
                    scenario_id: &scenario_id,
                    run_seed,
                    output_dir: &output_dir,
                    run_code_id: &run_code_id,
                    code_ordinal,
                    write_index_parquet,
                    workers,
                    credibility: credibility_options,
                })?;
                println!(
                    "openbmp mc aero-deck-draw-campaign: ok - ran {} samples, wrote {}",
                    report.samples,
                    report.campaign.summary_json.display(),
                );
                println!(
                    "  generated campaign manifest: {}",
                    report.campaign_manifest_toml.display(),
                );
                println!(
                    "  generated overlays: {}",
                    report.overlays.output_dir.display(),
                );
                if let Some(credibility) = &report.credibility {
                    println!(
                        "  credibility: binding={}, floor={}, label={}, aggregate_1sigma={:.12e}, accepted={}",
                        credibility.binding_level.as_label(),
                        credibility.floor.as_label(),
                        credibility.legacy_label.as_label(),
                        credibility.aggregate_one_sigma,
                        credibility.accepted,
                    );
                }
                if let Some(path) = &report.credibility_report_md {
                    println!("  wrote credibility report {}", path.display());
                }
                Ok(())
            }
            McCommand::AcousticsAscentTrace {
                samples,
                campaign_seed,
                design,
                dynamic_pressure_min_pa,
                dynamic_pressure_max_pa,
                mach_min,
                mach_max,
                sample_prefix,
                class_id,
                output_toml,
            } => {
                let design = match design {
                    McAcousticsAscentTraceDesign::LatinHypercube => {
                        mc::McAcousticsAscentTraceDesign::LatinHypercube
                    }
                    McAcousticsAscentTraceDesign::Sobol => mc::McAcousticsAscentTraceDesign::Sobol,
                    McAcousticsAscentTraceDesign::OwenSobol => {
                        mc::McAcousticsAscentTraceDesign::OwenSobol
                    }
                };
                let report = mc::write_acoustics_ascent_trace(mc::McAcousticsAscentTraceOptions {
                    samples,
                    campaign_seed,
                    design,
                    dynamic_pressure_min_pa,
                    dynamic_pressure_max_pa,
                    mach_min,
                    mach_max,
                    sample_prefix: &sample_prefix,
                    class_id: class_id.as_deref(),
                    output_toml: &output_toml,
                })?;
                println!(
                    "openbmp mc acoustics-ascent-trace: ok - wrote {} samples to {}",
                    report.samples,
                    report.output_toml.display(),
                );
                Ok(())
            }
            McCommand::AcousticsResponseSamples {
                samples,
                campaign_seed,
                design,
                surface_mass_min_kg_m2,
                surface_mass_max_kg_m2,
                area_min_m2,
                area_max_m2,
                mode_shape_min,
                mode_shape_max,
                sample_prefix,
                class_id,
                output_toml,
            } => {
                let design = match design {
                    McAcousticsAscentTraceDesign::LatinHypercube => {
                        mc::McAcousticsAscentTraceDesign::LatinHypercube
                    }
                    McAcousticsAscentTraceDesign::Sobol => mc::McAcousticsAscentTraceDesign::Sobol,
                    McAcousticsAscentTraceDesign::OwenSobol => {
                        mc::McAcousticsAscentTraceDesign::OwenSobol
                    }
                };
                let report =
                    mc::write_acoustics_response_samples(mc::McAcousticsResponseSamplesOptions {
                        samples,
                        campaign_seed,
                        design,
                        surface_mass_min_kg_m2,
                        surface_mass_max_kg_m2,
                        area_min_m2,
                        area_max_m2,
                        mode_shape_min,
                        mode_shape_max,
                        sample_prefix: &sample_prefix,
                        class_id: class_id.as_deref(),
                        output_toml: &output_toml,
                    })?;
                println!(
                    "openbmp mc acoustics-response-samples: ok - wrote {} samples to {}",
                    report.samples,
                    report.output_toml.display(),
                );
                Ok(())
            }
            McCommand::AcousticsAscentCampaign {
                scenario,
                samples,
                campaign_seed,
                design,
                dynamic_pressure_min_pa,
                dynamic_pressure_max_pa,
                mach_min,
                mach_max,
                sample_prefix,
                class_id,
                output_csv,
                output_artifact_dir,
                output_scenario_toml,
                checkpoint_json,
                max_new_samples,
                workers,
                toolchain_profile,
            } => {
                let design = match design {
                    McAcousticsAscentTraceDesign::LatinHypercube => {
                        mc::McAcousticsAscentTraceDesign::LatinHypercube
                    }
                    McAcousticsAscentTraceDesign::Sobol => mc::McAcousticsAscentTraceDesign::Sobol,
                    McAcousticsAscentTraceDesign::OwenSobol => {
                        mc::McAcousticsAscentTraceDesign::OwenSobol
                    }
                };
                let report =
                    mc::run_acoustics_ascent_campaign(mc::McAcousticsAscentCampaignOptions {
                        scenario_path: &scenario,
                        samples,
                        campaign_seed,
                        design,
                        dynamic_pressure_min_pa,
                        dynamic_pressure_max_pa,
                        mach_min,
                        mach_max,
                        sample_prefix: &sample_prefix,
                        class_id: class_id.as_deref(),
                        output_csv: &output_csv,
                        output_artifact_dir: output_artifact_dir.as_deref(),
                        output_scenario_toml: output_scenario_toml.as_deref(),
                        checkpoint_json: checkpoint_json.as_deref(),
                        max_new_samples,
                        workers,
                        toolchain_profile: &toolchain_profile,
                    })?;
                if !report.complete {
                    println!(
                        "openbmp mc acoustics-ascent-campaign: ok - checkpointed {}/{} samples at {}",
                        report.completed_samples,
                        report.samples,
                        report
                            .checkpoint_json
                            .as_ref()
                            .map_or_else(|| "<none>".to_owned(), |path| path.display().to_string()),
                    );
                    println!("  checkpoint incomplete; final outputs were not written");
                    return Ok(());
                }
                println!(
                    "openbmp mc acoustics-ascent-campaign: ok - ran {} samples, peak {:.12e} dB, wrote {}",
                    report.samples,
                    report.peak_sample_oaspl_db,
                    report.output_csv.display(),
                );
                if let Some(path) = &report.checkpoint_json {
                    println!(
                        "  checkpoint complete: {}/{} samples at {}",
                        report.completed_samples,
                        report.samples,
                        path.display(),
                    );
                }
                if let Some(p95_50_oaspl_db) = report.p95_50_oaspl_db {
                    println!("  p95/50 OASPL = {p95_50_oaspl_db:.12e} dB");
                }
                if let Some(required) = report.p95_50_wilks_required_sample_count {
                    println!(
                        "  Wilks required samples = {}, sufficient = {}",
                        required,
                        report.p95_50_wilks_sample_count_sufficient.unwrap_or(false),
                    );
                }
                if let Some(p95_50_oaspl_db) = report.fairing_internal_p95_50_oaspl_db {
                    println!("  fairing p95/50 OASPL = {p95_50_oaspl_db:.12e} dB");
                }
                if let Some(required) = report.fairing_internal_p95_50_wilks_required_sample_count {
                    println!(
                        "  fairing Wilks required samples = {}, sufficient = {}",
                        required,
                        report
                            .fairing_internal_p95_50_wilks_sample_count_sufficient
                            .unwrap_or(false),
                    );
                }
                if let Some(value) = report.response_p95_50_total_mean_square_acceleration_m2_s4 {
                    println!("  response p95/50 acceleration mean-square = {value:.12e} m^2/s^4");
                }
                if let Some(value) = report.response_p95_50_total_mean_square_force_n2 {
                    println!("  response p95/50 modal-force mean-square = {value:.12e} N^2");
                }
                if let Some(required) = report.response_p95_50_wilks_required_sample_count {
                    println!(
                        "  response Wilks required samples = {}, sufficient = {}",
                        required,
                        report
                            .response_p95_50_wilks_sample_count_sufficient
                            .unwrap_or(false),
                    );
                }
                if let Some(path) = &report.output_scenario_toml {
                    println!("  wrote generated scenario {}", path.display());
                }
                if let Some(path) = &report.output_artifact_dir {
                    println!(
                        "  wrote {} campaign artifacts under {}",
                        report.artifact_paths.len(),
                        path.display(),
                    );
                }
                if let Some(path) = &report.artifact_manifest_json {
                    println!("  wrote campaign artifact manifest {}", path.display());
                }
                Ok(())
            }
            McCommand::AcousticsResponseCampaign {
                scenario,
                samples,
                campaign_seed,
                design,
                dynamic_pressure_min_pa,
                dynamic_pressure_max_pa,
                mach_min,
                mach_max,
                sample_prefix,
                class_id,
                response_surface_mass_min_kg_m2,
                response_surface_mass_max_kg_m2,
                response_area_min_m2,
                response_area_max_m2,
                response_mode_shape_min,
                response_mode_shape_max,
                response_sample_prefix,
                response_class_id,
                output_csv,
                output_artifact_dir,
                output_scenario_toml,
                checkpoint_json,
                max_new_samples,
                workers,
                toolchain_profile,
            } => {
                let design = match design {
                    McAcousticsAscentTraceDesign::LatinHypercube => {
                        mc::McAcousticsAscentTraceDesign::LatinHypercube
                    }
                    McAcousticsAscentTraceDesign::Sobol => mc::McAcousticsAscentTraceDesign::Sobol,
                    McAcousticsAscentTraceDesign::OwenSobol => {
                        mc::McAcousticsAscentTraceDesign::OwenSobol
                    }
                };
                let response_samples = match (
                    response_surface_mass_min_kg_m2,
                    response_surface_mass_max_kg_m2,
                    response_area_min_m2,
                    response_area_max_m2,
                    response_mode_shape_min,
                    response_mode_shape_max,
                ) {
                    (
                        Some(surface_mass_min_kg_m2),
                        Some(surface_mass_max_kg_m2),
                        Some(area_min_m2),
                        Some(area_max_m2),
                        Some(mode_shape_min),
                        Some(mode_shape_max),
                    ) => Some(mc::McAcousticsResponseCampaignSamplesOptions {
                        surface_mass_min_kg_m2,
                        surface_mass_max_kg_m2,
                        area_min_m2,
                        area_max_m2,
                        mode_shape_min,
                        mode_shape_max,
                        sample_prefix: &response_sample_prefix,
                        class_id: response_class_id.as_deref(),
                    }),
                    (None, None, None, None, None, None) => {
                        if response_class_id.is_some() {
                            return Err(openbmp_cli::CliError::MonteCarlo {
                                summary: "acoustics response campaign --response-class-id requires response sample range flags".to_owned(),
                            });
                        }
                        None
                    }
                    _ => {
                        return Err(openbmp_cli::CliError::MonteCarlo {
                            summary: "acoustics response campaign response sample generation requires all response range flags".to_owned(),
                        });
                    }
                };
                let report =
                    mc::run_acoustics_response_campaign(mc::McAcousticsResponseCampaignOptions {
                        scenario_path: &scenario,
                        samples,
                        campaign_seed,
                        design,
                        dynamic_pressure_min_pa,
                        dynamic_pressure_max_pa,
                        mach_min,
                        mach_max,
                        sample_prefix: &sample_prefix,
                        class_id: class_id.as_deref(),
                        response_samples,
                        output_csv: &output_csv,
                        output_artifact_dir: output_artifact_dir.as_deref(),
                        output_scenario_toml: output_scenario_toml.as_deref(),
                        checkpoint_json: checkpoint_json.as_deref(),
                        max_new_samples,
                        workers,
                        toolchain_profile: &toolchain_profile,
                    })?;
                if !report.complete {
                    println!(
                        "openbmp mc acoustics-response-campaign: ok - checkpointed {}/{} samples at {}",
                        report.completed_samples,
                        report.samples,
                        report
                            .checkpoint_json
                            .as_ref()
                            .map_or_else(|| "<none>".to_owned(), |path| path.display().to_string()),
                    );
                    println!("  checkpoint incomplete; final response outputs were not written");
                    return Ok(());
                }
                println!(
                    "openbmp mc acoustics-response-campaign: ok - ran {} samples, wrote {}",
                    report.samples,
                    report.output_csv.display(),
                );
                if let Some(value) = report.response_p95_50_total_mean_square_acceleration_m2_s4 {
                    println!("  response p95/50 acceleration mean-square = {value:.12e} m^2/s^4");
                }
                if let Some(value) = report.response_p95_50_total_mean_square_force_n2 {
                    println!("  response p95/50 modal-force mean-square = {value:.12e} N^2");
                }
                if let Some(required) = report.response_p95_50_wilks_required_sample_count {
                    println!(
                        "  response Wilks required samples = {}, sufficient = {}",
                        required,
                        report
                            .response_p95_50_wilks_sample_count_sufficient
                            .unwrap_or(false),
                    );
                }
                if let Some(path) = &report.checkpoint_json {
                    println!(
                        "  checkpoint complete: {}/{} samples at {}",
                        report.completed_samples,
                        report.samples,
                        path.display(),
                    );
                }
                if let Some(path) = &report.output_scenario_toml {
                    println!("  wrote generated scenario {}", path.display());
                }
                if let Some(path) = &report.output_artifact_dir {
                    println!(
                        "  wrote {} campaign artifacts under {}",
                        report.artifact_paths.len(),
                        path.display(),
                    );
                }
                if let Some(path) = &report.artifact_manifest_json {
                    println!("  wrote campaign artifact manifest {}", path.display());
                }
                Ok(())
            }
            McCommand::PropulsionFaults {
                scenario,
                library_toml,
                samples,
                campaign_seed,
                dimension_id,
                metric_channel,
                success_min,
                success_max,
                output_csv,
                confidence,
                uq_toml,
                credibility_floor,
                credibility_report_md,
            } => {
                let credibility_options = build_mc_credibility_options(
                    uq_toml.as_ref(),
                    &credibility_floor,
                    credibility_report_md.as_ref(),
                )?;
                let report =
                    mc::run_propulsion_fault_campaign(mc::McPropulsionFaultCampaignOptions {
                        scenario_path: &scenario,
                        library_toml: &library_toml,
                        samples,
                        campaign_seed,
                        dimension_id,
                        metric_channel: &metric_channel,
                        success_min,
                        success_max,
                        output_csv: &output_csv,
                        confidence,
                        credibility: credibility_options,
                    })?;
                println!(
                    "openbmp mc propulsion-faults: ok - {} samples, {} activated, mean = {:.12e}, wrote {}",
                    report.samples,
                    report.activated_samples,
                    report.summary.mean,
                    report.output_csv.display(),
                );
                if let Some(success) = report.summary.success {
                    println!(
                        "  success = {}/{} ({:.6}), {:.3} CI [{:.6}, {:.6}]",
                        success.successes,
                        success.trials,
                        success.fraction,
                        success.confidence,
                        success.lower,
                        success.upper,
                    );
                }
                if let Some(credibility) = &report.credibility {
                    println!(
                        "  credibility: binding={}, floor={}, label={}, aggregate_1sigma={:.12e}, accepted={}",
                        credibility.binding_level.as_label(),
                        credibility.floor.as_label(),
                        credibility.legacy_label.as_label(),
                        credibility.aggregate_one_sigma,
                        credibility.accepted,
                    );
                }
                if let Some(path) = &report.credibility_report_md {
                    println!("  wrote credibility report {}", path.display());
                }
                Ok(())
            }
            McCommand::ResumeScalar {
                samples_csv,
                checkpoint_json,
                value_column,
                success_column,
                sample_count,
                campaign_seed,
                dimension_id,
                scenario_sha256,
                toolchain_profile,
                max_new_samples,
                workers,
                confidence,
            } => {
                let report = mc::resume_scalar_checkpoint(
                    &samples_csv,
                    &checkpoint_json,
                    &value_column,
                    success_column.as_deref(),
                    sample_count,
                    campaign_seed,
                    dimension_id,
                    &scenario_sha256,
                    &toolchain_profile,
                    max_new_samples,
                    workers,
                    confidence,
                )?;
                println!(
                    "openbmp mc resume-scalar: ok - checkpointed {}/{} samples at {}",
                    report.completed,
                    report.sample_count,
                    report.checkpoint_json.display(),
                );
                if let Some(summary) = report.summary {
                    println!(
                        "  complete: mean = {:.12e}, samples = {}",
                        summary.mean, summary.samples,
                    );
                    if let Some(success) = summary.success {
                        println!(
                            "  success = {}/{} ({:.6}), {:.3} CI [{:.6}, {:.6}]",
                            success.successes,
                            success.trials,
                            success.fraction,
                            success.confidence,
                            success.lower,
                            success.upper,
                        );
                    }
                }
                Ok(())
            }
        },
        Command::CheckProvenance { root } => {
            let report = provenance::run(&root)?;
            println!(
                "openbmp check-provenance: {} files seen, {} without provenance.md",
                report.files_seen,
                report.missing_provenance.len(),
            );
            for path in report.missing_provenance {
                println!("  missing: {}", path.display());
            }
            Ok(())
        }
        Command::Dict { command } => match command {
            DictCommand::Export {
                format,
                output,
                experimental,
            } => {
                let report = dict::export(format, output.as_deref(), experimental)?;
                if report.output.is_none() {
                    print!("{}", report.content);
                } else if let Some(path) = report.output {
                    println!(
                        "openbmp dict export: ok — {} topics as {} -> {}",
                        report.topics,
                        report.format,
                        path.display(),
                    );
                }
                Ok(())
            }
        },
        Command::Package { command } => match command {
            PackageCommand::Check { package: manifest } => {
                let report = package::check(&manifest)?;
                println!(
                    "openbmp package check: ok — {} {}",
                    report.package_id, report.package_version,
                );
                println!("  scenario {}", report.scenario_path.display());
                for (key, digest) in report.hashes {
                    println!("  sha256 {key} {digest}");
                }
                Ok(())
            }
            PackageCommand::MaterializeSidecars { package: manifest } => {
                let report = package::materialize_sidecars(&manifest)?;
                println!(
                    "openbmp package materialize-sidecars: ok — {} {}",
                    report.package_id, report.package_version,
                );
                for sidecar in report.sidecars {
                    println!(
                        "  wrote {} {} sha256 {}",
                        sidecar.field,
                        sidecar.path.display(),
                        sidecar.sha256,
                    );
                }
                Ok(())
            }
        },
        Command::Campaign { command } => match command {
            CampaignCommand::Plan {
                manifest,
                output_plan_json,
                output_shards_json,
                workers,
            } => {
                let report = campaign::plan(
                    &manifest,
                    &output_plan_json,
                    output_shards_json.as_deref(),
                    workers,
                )?;
                println!(
                    "openbmp campaign plan: ok - campaign={}, runs={}, wrote {}",
                    report.campaign_id,
                    report.run_count,
                    report.plan_json.display(),
                );
                if let Some(path) = &report.shards_json {
                    let worker_count = report.worker_count.unwrap_or(0);
                    println!("  wrote {} shard(s) to {}", worker_count, path.display());
                }
                Ok(())
            }
            CampaignCommand::Run {
                manifest,
                output_dir,
                run_code_id,
                code_ordinal,
                write_index_parquet,
                workers,
                fail_on_ci_gates,
            } => {
                let report = campaign::run(
                    &manifest,
                    &output_dir,
                    &run_code_id,
                    code_ordinal,
                    write_index_parquet,
                    workers,
                )?;
                if fail_on_ci_gates && !report.ci_failures.is_empty() {
                    return Err(openbmp_cli::CliError::RunDataConfig {
                        summary: format!(
                            "campaign CI gates failed ({} failure(s)); summary={}; {}",
                            report.ci_failures.len(),
                            report.summary_json.display(),
                            report.ci_failures.join("; "),
                        ),
                    });
                }
                println!(
                    "openbmp campaign run: ok - campaign={}, runs={}, wrote {}",
                    report.campaign_id,
                    report.run_count,
                    report.summary_json.display(),
                );
                println!("  run index {}", report.run_index_jsonl.display());
                if let Some(path) = &report.run_index_parquet {
                    println!("  run index parquet {}", path.display());
                }
                if let Some(summary) = &report.verdict_summary {
                    println!(
                        "  verdict summary: status={:?}, reports={}, failed={}",
                        summary.status, summary.report_count, summary.failed_report_count,
                    );
                }
                if let Some(trend) = &report.trend_summary {
                    println!(
                        "  trend summary: store={}, drift={}, scoreboard={}, ci_failures={}",
                        trend.trend_store_json.display(),
                        trend.drift_report_json.display(),
                        trend.scoreboard_md.display(),
                        trend.ci_failures.len(),
                    );
                }
                if !report.ci_failures.is_empty() {
                    println!("  campaign ci failures: {}", report.ci_failures.len());
                }
                Ok(())
            }
        },
        Command::RunPackage {
            package,
            case,
            evidence,
        } => {
            let (report, evidence_path) = sil::run(&package, case.as_deref(), evidence.as_deref())?;
            println!(
                "openbmp run-package: ok — package={}, case={}, {} steps, t = {:.6} s, stop = {}",
                report.package_id,
                report.case_id,
                report.final_step,
                report.final_time_s,
                report.stop_label,
            );
            if let Some(path) = evidence_path {
                println!("  wrote evidence {}", path.display());
            }
            Ok(())
        }
        Command::Sil { command } => match command {
            SilCommand::Run {
                package,
                case,
                evidence,
            } => {
                let (report, evidence_path) =
                    sil::run(&package, case.as_deref(), evidence.as_deref())?;
                println!(
                    "openbmp sil run: ok — package={}, case={}, {} steps, t = {:.6} s, stop = {}",
                    report.package_id,
                    report.case_id,
                    report.final_step,
                    report.final_time_s,
                    report.stop_label,
                );
                if let Some(path) = evidence_path {
                    println!("  wrote evidence {}", path.display());
                }
                Ok(())
            }
            SilCommand::Observe {
                package,
                case,
                decimation,
                evidence,
            } => {
                let (report, evidence_path) =
                    sil::observe(&package, case.as_deref(), decimation, evidence.as_deref())?;
                let observed_ticks = report
                    .evidence
                    .observations
                    .as_ref()
                    .map_or(0, |observations| observations.summary.ticks);
                println!(
                    "openbmp sil observe: {} — package={}, case={}, observed_ticks={}, t = {:.6} s, stop = {}",
                    report.evidence.verdict,
                    report.package_id,
                    report.case_id,
                    observed_ticks,
                    report.final_time_s,
                    report.stop_label,
                );
                if let Some(path) = evidence_path {
                    println!("  wrote evidence {}", path.display());
                }
                Ok(())
            }
            SilCommand::Step {
                package,
                ticks,
                case,
                evidence,
            } => {
                let (report, evidence_path) =
                    sil::step(&package, case.as_deref(), ticks, evidence.as_deref())?;
                println!(
                    "openbmp sil step: ok — package={}, case={}, requested_ticks={}, final_step={}, t = {:.6} s, stop = {}",
                    report.package_id,
                    report.case_id,
                    ticks,
                    report.final_step,
                    report.final_time_s,
                    report.stop_label,
                );
                if let Some(path) = evidence_path {
                    println!("  wrote evidence {}", path.display());
                }
                Ok(())
            }
            SilCommand::RunUntil {
                package,
                case,
                time_s,
                event,
                phase,
                evidence,
            } => {
                let target = sil::run_until_target(time_s, event, phase)?;
                let (report, evidence_path) =
                    sil::run_until(&package, case.as_deref(), target, evidence.as_deref())?;
                println!(
                    "openbmp sil run-until: ok — package={}, case={}, {} steps, t = {:.6} s, stop = {}",
                    report.package_id,
                    report.case_id,
                    report.final_step,
                    report.final_time_s,
                    report.stop_label,
                );
                if let Some(path) = evidence_path {
                    println!("  wrote evidence {}", path.display());
                }
                Ok(())
            }
            SilCommand::ReadChannel {
                package,
                channel,
                case,
                ticks,
                max_samples,
            } => {
                let report =
                    sil::read_channel(&package, case.as_deref(), ticks, &channel, max_samples)?;
                println!(
                    "openbmp sil read-channel: ok — channel={}, kind={}, samples={}",
                    report.channel, report.kind, report.samples,
                );
                if let Some(first) = &report.first {
                    println!(
                        "  first step={} t={:.6} value={}",
                        first.step, first.time_s, first.value
                    );
                }
                if let Some(last) = &report.last {
                    println!(
                        "  last  step={} t={:.6} value={}",
                        last.step, last.time_s, last.value
                    );
                }
                if let (Some(min), Some(max)) = (&report.min, &report.max) {
                    println!("  range min={min} max={max}");
                }
                for sample in report.preview {
                    println!(
                        "  sample step={} t={:.6} value={}",
                        sample.step, sample.time_s, sample.value
                    );
                }
                Ok(())
            }
            SilCommand::CaptureBus {
                package,
                case,
                ticks,
                output,
            } => {
                let frames = sil::capture_bus(&package, case.as_deref(), ticks, output.as_deref())?;
                println!("openbmp sil capture-bus: ok — frames={}", frames.len());
                if let Some(path) = output {
                    println!("  wrote frames {}", path.display());
                } else {
                    for frame in frames.iter().take(16) {
                        println!(
                            "  frame step={} t={:.6} stream={} subject={} value={}",
                            frame.step, frame.time_s, frame.stream, frame.subject, frame.value
                        );
                    }
                }
                Ok(())
            }
            SilCommand::Evidence {
                package,
                case,
                output,
            } => {
                let path = sil::evidence(&package, case.as_deref(), &output)?;
                println!("openbmp sil evidence: ok — wrote {}", path.display());
                Ok(())
            }
            SilCommand::Verdict { evidence } => {
                let report = sil::verdict(&evidence)?;
                println!(
                    "openbmp sil verdict: {} — package={}, case={}, scenario={}, stop={}, events={}",
                    report.verdict,
                    report.package_id,
                    report.case_id,
                    report.scenario_name,
                    report.stop_label,
                    report.event_trace.len(),
                );
                Ok(())
            }
        },
        Command::Migrate { command } => match command {
            MigrateCommand::MissionScriptSplit { scenario } => {
                let report = migrate::mission_script_split(&scenario)?;
                println!(
                    "openbmp migrate mission-script-split: ok — moved {} event(s){} in {}",
                    report.moved_events,
                    if report.written { "" } else { " (no write)" },
                    report.path.display(),
                );
                Ok(())
            }
        },
    }
}

fn build_mc_credibility_options<'a>(
    uq_toml: Option<&'a std::path::PathBuf>,
    credibility_floor: &str,
    credibility_report_md: Option<&'a std::path::PathBuf>,
) -> Result<Option<mc::McCredibilityOptions<'a>>, openbmp_cli::CliError> {
    let floor = mc::parse_credibility_floor(credibility_floor)?;
    if uq_toml.is_none() && credibility_report_md.is_some() {
        return Err(openbmp_cli::CliError::MonteCarlo {
            summary: "--credibility-report-md requires --uq-toml".to_owned(),
        });
    }
    Ok(uq_toml.map(|path| mc::McCredibilityOptions {
        uq_toml: path.as_path(),
        floor,
        report_md: credibility_report_md.map(|path| path.as_path()),
    }))
}
