//! `openbmp` binary entry point.

use std::io::Write;
use std::process::ExitCode;

use clap::Parser;

use openbmp_cli::cli::{Cli, Command};
use openbmp_cli::commands::{check, compare_telemetry, diff, footprint_mc, provenance, run};
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
        Command::Diff { golden, actual } => {
            let report = diff::run(&golden, &actual)?;
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
        Command::CompareTelemetry {
            scenario,
            reference_csv,
            mapping,
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
    }
}
