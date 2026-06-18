//! `openbmp` binary entry point.

use std::io::Write;
use std::process::ExitCode;

use clap::Parser;

use openbmp_cli::cli::{
    Cli, Command, DictCommand, McCommand, McWilksSide, MigrateCommand, PackageCommand,
    ReconstructCommand, SilCommand, TrajoptCommand,
};
use openbmp_cli::commands::{
    check, compare_telemetry, conform, dict, diff, footprint_mc, mc, migrate, package, provenance,
    reconstruct, run, sil, trajopt, verify_order,
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
        } => {
            let overrides = run::OutputOverrides::new(output_csv, output_json, output_parquet);
            let report = run::run_with_overrides(&scenario, &overrides)?;
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
