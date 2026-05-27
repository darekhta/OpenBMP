//! Clap argument schema for the `openbmp` binary.

use std::path::PathBuf;

use clap::{Parser, Subcommand};

/// OpenBMP command-line entry point.
#[derive(Debug, Parser)]
#[command(
    name = "openbmp",
    about = "OpenBMP — Open Body Motion Platform CLI",
    long_about = None,
    version
)]
pub struct Cli {
    /// Increase tracing verbosity. May be passed multiple times
    /// (`-t`, `-tt`, `-ttt` ⇒ INFO / DEBUG / TRACE).
    #[arg(short = 't', long = "trace", action = clap::ArgAction::Count, global = true)]
    pub trace: u8,
    /// Subcommand.
    #[command(subcommand)]
    pub command: Command,
}

/// Top-level subcommands.
#[derive(Debug, Subcommand)]
pub enum Command {
    /// Run a scenario through the kernel and write telemetry.
    Run {
        /// Scenario TOML file.
        scenario: PathBuf,
        /// Override the scenario's declared CSV output path.
        #[arg(long = "output-csv")]
        output_csv: Option<PathBuf>,
        /// Override the scenario's declared JSON output path.
        #[arg(long = "output-json")]
        output_json: Option<PathBuf>,
        /// Override the scenario's declared Parquet output path.
        #[arg(long = "output-parquet")]
        output_parquet: Option<PathBuf>,
    },
    /// Compare two telemetry archives byte-for-byte.
    Diff {
        /// Reference / golden archive.
        golden: PathBuf,
        /// Archive produced by the run under test.
        actual: PathBuf,
    },
    /// Lint a scenario without running it.
    Check {
        /// Scenario TOML file.
        scenario: PathBuf,
    },
    /// Run offline landing-footprint Monte Carlo post-processing.
    FootprintMc {
        /// Scenario TOML file.
        scenario: PathBuf,
    },
    /// Walk a data tree and verify provenance records.
    ///
    /// Lists files without a sibling `provenance.md` so the project can
    /// audit a fresh data tree.
    CheckProvenance {
        /// Root directory to walk.
        root: PathBuf,
    },
}
