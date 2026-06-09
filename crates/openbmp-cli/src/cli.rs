//! Clap argument schema for the `openbmp` binary.

use std::path::PathBuf;

use clap::{Parser, Subcommand, ValueEnum};

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
        /// Write a machine-readable determinism/diff report.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
    },
    /// Lint a scenario without running it.
    Check {
        /// Scenario TOML file.
        scenario: PathBuf,
    },
    /// Run one or more scenarios as a fork-runnable conformance smoke suite.
    Conform {
        /// Scenario TOML files to validate and run without writing telemetry.
        #[arg(required = true)]
        scenarios: Vec<PathBuf>,
    },
    /// Compare a scenario run against external local telemetry CSV / JSON.
    CompareTelemetry {
        /// Scenario TOML file.
        scenario: PathBuf,
        /// External reference CSV or JSON. The file is read locally and is not
        /// copied into OpenBMP.
        #[arg(value_name = "REFERENCE")]
        reference_csv: PathBuf,
        /// TOML mapping from reference columns to OpenBMP telemetry
        /// observables and tolerances.
        #[arg(long = "mapping")]
        mapping: PathBuf,
        /// Write a machine-readable comparison report to this JSON file.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
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
    /// Export command/telemetry dictionaries.
    Dict {
        /// Dictionary command to run.
        #[command(subcommand)]
        command: DictCommand,
    },
    /// Validate mission package manifests.
    Package {
        /// Package command to run.
        #[command(subcommand)]
        command: PackageCommand,
    },
    /// Run a mission package test case through the native SIL path.
    RunPackage {
        /// Mission package manifest.
        package: PathBuf,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Optional evidence bundle output directory.
        #[arg(long = "evidence")]
        evidence: Option<PathBuf>,
    },
    /// Native SIL testbench operations.
    Sil {
        /// SIL command to run.
        #[command(subcommand)]
        command: SilCommand,
    },
    /// Rewrite scenario documents to newer schema shapes.
    Migrate {
        /// Migration to apply.
        #[command(subcommand)]
        command: MigrateCommand,
    },
}

/// Dictionary subcommands.
#[derive(Debug, Subcommand)]
pub enum DictCommand {
    /// Export the canonical command/telemetry dictionary.
    Export {
        /// Export format.
        #[arg(long = "format", value_enum, default_value_t = DictFormat::Json)]
        format: DictFormat,
        /// Output file. Omit to write to stdout.
        #[arg(long = "output")]
        output: Option<PathBuf>,
        /// Allow experimental interchange formats such as XTCE.
        #[arg(long = "experimental")]
        experimental: bool,
    },
}

/// Dictionary export format.
#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum DictFormat {
    /// Stable OpenBMP JSON dictionary.
    Json,
    /// Experimental XTCE-like XML export.
    Xtce,
}

/// Mission package subcommands.
#[derive(Debug, Subcommand)]
pub enum PackageCommand {
    /// Validate package references, schema versions, and hashes.
    Check {
        /// Mission package manifest.
        package: PathBuf,
    },
    /// Generate declared sidecars from the package's default scenario.
    MaterializeSidecars {
        /// Mission package manifest.
        package: PathBuf,
    },
}

/// Native SIL testbench subcommands.
#[derive(Debug, Subcommand)]
pub enum SilCommand {
    /// Run a package test case to its declared stop condition.
    Run {
        /// Mission package manifest.
        package: PathBuf,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Evidence bundle output directory.
        #[arg(long = "evidence")]
        evidence: Option<PathBuf>,
    },
    /// Step a package test case for a fixed number of ticks.
    Step {
        /// Mission package manifest.
        package: PathBuf,
        /// Number of scheduler ticks to execute.
        #[arg(long = "ticks")]
        ticks: u64,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Evidence bundle output directory.
        #[arg(long = "evidence")]
        evidence: Option<PathBuf>,
    },
    /// Run until a time, mission event, or mission phase is reached.
    RunUntil {
        /// Mission package manifest.
        package: PathBuf,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Stop at this absolute simulation time in seconds.
        #[arg(long = "time-s")]
        time_s: Option<f64>,
        /// Stop when this declared mission event fires.
        #[arg(long = "event")]
        event: Option<String>,
        /// Stop when a transition into this mission phase/state fires.
        #[arg(long = "phase")]
        phase: Option<String>,
        /// Evidence bundle output directory.
        #[arg(long = "evidence")]
        evidence: Option<PathBuf>,
    },
    /// Run a package case and summarize one telemetry channel.
    ReadChannel {
        /// Mission package manifest.
        package: PathBuf,
        /// Telemetry channel name.
        #[arg(long = "channel")]
        channel: String,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Optional fixed number of ticks to execute instead of a full run.
        #[arg(long = "ticks")]
        ticks: Option<u64>,
        /// Maximum prefix samples to print.
        #[arg(long = "max-samples", default_value_t = 8)]
        max_samples: usize,
    },
    /// Run a package case and capture host-SIL bus/evidence frames.
    CaptureBus {
        /// Mission package manifest.
        package: PathBuf,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Optional fixed number of ticks to execute instead of a full run.
        #[arg(long = "ticks")]
        ticks: Option<u64>,
        /// Optional JSON output path.
        #[arg(long = "output")]
        output: Option<PathBuf>,
    },
    /// Generate an evidence bundle by running a package test case.
    Evidence {
        /// Mission package manifest.
        package: PathBuf,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Evidence bundle output directory.
        #[arg(long = "output")]
        output: PathBuf,
    },
    /// Read an evidence bundle and print its verdict.
    Verdict {
        /// Evidence `manifest.json` path.
        evidence: PathBuf,
    },
}

/// Scenario migration subcommands.
#[derive(Debug, Subcommand)]
pub enum MigrateCommand {
    /// Move simulator-only mission events into `[scenario_script]`.
    MissionScriptSplit {
        /// Scenario TOML file to rewrite in place.
        scenario: PathBuf,
    },
}
