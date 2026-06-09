//! `openbmp` binary entry point.

use std::io::Write;
use std::process::ExitCode;

use clap::Parser;

use openbmp_cli::cli::{Cli, Command, DictCommand, MigrateCommand, PackageCommand, SilCommand};
use openbmp_cli::commands::{
    check, compare_telemetry, conform, dict, diff, footprint_mc, migrate, package, provenance, run,
    sil,
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
            for path in report.written {
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
        Command::FootprintMc { scenario } => {
            let report = footprint_mc::run(&scenario)?;
            println!(
                "openbmp footprint-mc: ok — {} requested, {} succeeded, {} failed",
                report.samples_requested, report.samples_succeeded, report.samples_failed,
            );
            for path in report.written {
                println!("  wrote {}", path.display());
            }
            Ok(())
        }
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
