//! `openbmp` binary entry point.

use std::io::Write;
use std::process::ExitCode;

use clap::Parser;

use openbmp_cli::cli::{Cli, Command};
use openbmp_cli::commands::{check, diff, provenance, run};
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
