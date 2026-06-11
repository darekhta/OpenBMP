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
    /// Verify integrator observed order and Richardson/GCI behavior.
    VerifyOrder {
        /// Integrator method to verify.
        #[arg(long = "method", value_enum, default_value_t = VerifyOrderMethod::All)]
        method: VerifyOrderMethod,
        /// Optional deterministic TOML evidence output path.
        #[arg(long = "output-toml")]
        output_toml: Option<PathBuf>,
    },
    /// Run offline reconstruction and filter-consistency evidence commands.
    Reconstruct {
        /// Reconstruction command to run.
        #[command(subcommand)]
        command: ReconstructCommand,
    },
    /// Run offline trajectory optimization and I-load synthesis commands.
    Trajopt {
        /// Trajectory optimization command to run.
        #[command(subcommand)]
        command: TrajoptCommand,
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
        /// Optional checkpoint JSON path for resumable footprint MC.
        #[arg(long = "checkpoint-json")]
        checkpoint_json: Option<PathBuf>,
        /// Maximum new samples to propagate when checkpointing is enabled.
        #[arg(long = "max-new-samples")]
        max_new_samples: Option<u32>,
        /// Toolchain / determinism profile recorded in checkpoint metadata.
        #[arg(long = "toolchain-profile", default_value = "host-clean")]
        toolchain_profile: String,
        /// Optional UQ budget TOML to attach to this campaign report.
        #[arg(long = "uq-toml")]
        uq_toml: Option<PathBuf>,
        /// Required binding credibility floor for the UQ budget.
        #[arg(long = "credibility-floor", default_value = "l0")]
        credibility_floor: String,
        /// Optional Markdown output path for the UQ credibility report.
        #[arg(long = "credibility-report-md")]
        credibility_report_md: Option<PathBuf>,
    },
    /// Monte Carlo utility commands.
    Mc {
        /// Monte Carlo command to run.
        #[command(subcommand)]
        command: McCommand,
    },
    /// Walk a data/scenario tree and verify provenance records.
    ///
    /// Lists source files without a sibling `provenance.md` so the project can
    /// audit a fresh data or scenario tree.
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

/// Integrator selection for `openbmp verify-order`.
#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum VerifyOrderMethod {
    /// Verify every method covered by the code-verification gate.
    All,
    /// Verify the fixed-step classical fourth-order Runge-Kutta integrator.
    Rk4,
    /// Verify the fixed-step Dormand-Prince 8(5,3) integrator.
    Dop853,
}

/// Offline reconstruction evidence commands.
#[derive(Debug, Subcommand)]
pub enum ReconstructCommand {
    /// Run the synthetic linear-Gaussian RTS/GN/NEES-NIS fixture.
    SyntheticLinear {
        /// Optional deterministic TOML evidence output path.
        #[arg(long = "output-toml")]
        output_toml: Option<PathBuf>,
    },
    /// Reduce estimator innovation histories from a monitored FC scenario run.
    ObserveFc {
        /// Scenario TOML file with an `[fc]` block.
        scenario: PathBuf,
        /// Optional deterministic TOML evidence output path.
        #[arg(long = "output-toml")]
        output_toml: Option<PathBuf>,
    },
    /// Export an OpenBMP run trajectory CSV for code-to-code comparison tools.
    ExportTrajectory {
        /// Scenario TOML file to run.
        scenario: PathBuf,
        /// Deterministic trajectory CSV output path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
    },
    /// Write a compare-telemetry mapping for exported trajectory CSVs.
    ExportTrajectoryMapping {
        /// Deterministic mapping TOML output path.
        #[arg(long = "output-toml")]
        output_toml: PathBuf,
        /// Tolerance pack to write.
        #[arg(long = "pack", value_enum, default_value_t = TrajectoryTolerancePack::Strict)]
        pack: TrajectoryTolerancePack,
    },
}

/// Offline trajectory-optimization commands.
#[derive(Debug, Subcommand)]
pub enum TrajoptCommand {
    /// Correct tangential cutoff speed to hit a two-body apogee radius.
    CorrectApogee {
        /// Initial inertial radius on the x-axis, in m.
        #[arg(long = "initial-radius-m")]
        initial_radius_m: f64,
        /// Target apogee radius from the central body, in m.
        #[arg(long = "target-apogee-radius-m")]
        target_apogee_radius_m: f64,
        /// Initial tangential speed guess on the y-axis, in m/s.
        ///
        /// Omit to use the circular speed at `initial-radius-m`.
        #[arg(long = "initial-speed-m-s")]
        initial_speed_m_s: Option<f64>,
        /// Coast duration before residual evaluation, in s.
        #[arg(long = "coast-duration-s", default_value_t = 0.0)]
        coast_duration_s: f64,
        /// Fixed RK4 propagation step, in s.
        #[arg(long = "step-s", default_value_t = 10.0)]
        step_s: f64,
        /// Central-body gravitational parameter, in m^3/s^2.
        #[arg(long = "mu-m3-s2", default_value_t = 398_600_441_800_000.0)]
        mu_m3_s2: f64,
        /// Residual convergence tolerance, in m.
        #[arg(long = "residual-tolerance-m", default_value_t = 1.0e-2)]
        residual_tolerance_m: f64,
        /// Maximum Gauss-Newton iterations.
        #[arg(long = "max-iterations", default_value_t = 100)]
        max_iterations: usize,
        /// Deterministic synthesis seed recorded in the I-load header.
        #[arg(long = "synthesis-seed", default_value_t = 0)]
        synthesis_seed: u64,
        /// Opaque scenario or driver digest recorded in the I-load metadata.
        #[arg(long = "scenario-digest", default_value = "two-body-apogee-cli")]
        scenario_digest: String,
        /// Source revision recorded in the I-load metadata.
        #[arg(long = "source-revision", default_value = "working-tree")]
        source_revision: String,
        /// Producer name recorded in the I-load header.
        #[arg(long = "producer", default_value = "openbmp-trajopt-cli")]
        producer: String,
        /// Postcard I-load output path. Written only when the correction converges.
        #[arg(long = "output-iload")]
        output_iload: PathBuf,
    },
    /// Correct a scenario initial-speed magnitude to hit an apogee radius.
    CorrectApogeeScenario {
        /// Scenario TOML file used as the runner forward map.
        scenario: PathBuf,
        /// Target apogee radius from the central body, in m.
        #[arg(long = "target-apogee-radius-m")]
        target_apogee_radius_m: f64,
        /// Optional initial speed guess, in m/s.
        ///
        /// Omit to use the magnitude of the scenario's initial ECI
        /// velocity. The scenario velocity direction is preserved.
        #[arg(long = "initial-speed-m-s")]
        initial_speed_m_s: Option<f64>,
        /// Central-body gravitational parameter for terminal residuals, in m^3/s^2.
        ///
        /// Omit to use the scenario gravity parameter when present, or WGS84 µ
        /// for built-in Earth gravity models.
        #[arg(long = "mu-m3-s2")]
        mu_m3_s2: Option<f64>,
        /// Residual convergence tolerance, in m.
        #[arg(long = "residual-tolerance-m", default_value_t = 1.0e-2)]
        residual_tolerance_m: f64,
        /// Maximum Gauss-Newton iterations.
        #[arg(long = "max-iterations", default_value_t = 100)]
        max_iterations: usize,
        /// Deterministic synthesis seed recorded in the I-load header.
        #[arg(long = "synthesis-seed", default_value_t = 0)]
        synthesis_seed: u64,
        /// Opaque scenario digest recorded in the I-load metadata.
        ///
        /// Omit to record the SHA-256 digest of the scenario file.
        #[arg(long = "scenario-digest")]
        scenario_digest: Option<String>,
        /// Source revision recorded in the I-load metadata.
        #[arg(long = "source-revision", default_value = "working-tree")]
        source_revision: String,
        /// Producer name recorded in the I-load header.
        #[arg(long = "producer", default_value = "openbmp-trajopt-cli")]
        producer: String,
        /// Postcard I-load output path. Written only when the correction converges.
        #[arg(long = "output-iload")]
        output_iload: PathBuf,
    },
}

/// Built-in trajectory code-to-code comparison tolerance packs.
#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum TrajectoryTolerancePack {
    /// Round-trip tolerance for OpenBMP-generated trajectory CSVs.
    Strict,
    /// Starter LEO code-to-code tolerance for external Orekit/GMAT-style exchanges.
    LeoResearch,
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

/// Monte Carlo utility subcommands.
#[derive(Debug, Subcommand)]
pub enum McCommand {
    /// Summarize a scalar sample CSV with Welford and Clopper-Pearson statistics.
    Summarize {
        /// CSV file containing one row per sample.
        samples_csv: PathBuf,
        /// Column containing the scalar quantity of interest.
        #[arg(long = "value-column")]
        value_column: String,
        /// Optional column containing a boolean success outcome.
        #[arg(long = "success-column")]
        success_column: Option<String>,
        /// Confidence level for the Clopper-Pearson success interval.
        #[arg(long = "confidence", default_value_t = 0.95)]
        confidence: f64,
        /// Optional UQ budget TOML to attach to this campaign summary.
        #[arg(long = "uq-toml")]
        uq_toml: Option<PathBuf>,
        /// Required binding credibility floor for the UQ budget.
        #[arg(long = "credibility-floor", default_value = "l0")]
        credibility_floor: String,
        /// Optional Markdown output path for the UQ credibility report.
        #[arg(long = "credibility-report-md")]
        credibility_report_md: Option<PathBuf>,
    },
    /// Summarize nested aleatory/epistemic scalar samples into a p-box.
    NestedSummarize {
        /// CSV file containing one row per nested sample.
        samples_csv: PathBuf,
        /// Column identifying the epistemic outer-loop condition.
        #[arg(long = "epistemic-column")]
        epistemic_column: String,
        /// Column containing the scalar quantity of interest.
        #[arg(long = "value-column")]
        value_column: String,
        /// Scalar threshold for the lower-tail requirement `P(y <= threshold)`.
        #[arg(long = "threshold")]
        threshold: f64,
        /// Required lower-bound probability for the lower-tail requirement.
        #[arg(long = "minimum-probability")]
        minimum_probability: f64,
        /// Optional deterministic p-box CSV output path.
        #[arg(long = "pbox-csv")]
        pbox_csv: Option<PathBuf>,
    },
    /// Run synthetic rare-event estimators declared in a scenario.
    RareEvent {
        /// Scenario TOML file with a v3 `[monte_carlo]` block.
        scenario: PathBuf,
        /// Optional deterministic TOML evidence output path.
        #[arg(long = "output-toml")]
        output_toml: Option<PathBuf>,
    },
    /// Compute a Wilks tolerance-bound sample size.
    Wilks {
        /// Population coverage proportion, for example 0.99865 for a 3-sigma
        /// normal-equivalent content target.
        #[arg(long = "coverage")]
        coverage: f64,
        /// Confidence level for the tolerance bound.
        #[arg(long = "confidence")]
        confidence: f64,
        /// One-sided or two-sided Wilks bound.
        #[arg(long = "side", value_enum, default_value_t = McWilksSide::TwoSided)]
        side: McWilksSide,
    },
    /// Generate a deterministic Latin hypercube unit-cube design CSV.
    Lhs {
        /// Number of sample rows.
        #[arg(long = "samples")]
        samples: u64,
        /// Number of dispersion dimensions.
        #[arg(long = "dimensions")]
        dimensions: u32,
        /// Campaign seed for deterministic design generation.
        #[arg(long = "seed")]
        seed: u64,
        /// Output CSV path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
    },
    /// Generate an unscrambled Sobol unit-cube design CSV.
    Sobol {
        /// Number of sample rows.
        #[arg(long = "samples")]
        samples: u64,
        /// Number of dispersion dimensions.
        #[arg(long = "dimensions")]
        dimensions: u32,
        /// Optional Owen scramble seed.
        #[arg(long = "scramble-seed")]
        scramble_seed: Option<u64>,
        /// Output CSV path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
    },
    /// Reorder a unit-cube design CSV with Iman-Conover rank correlation.
    ImanConover {
        /// Input design CSV with sample_index,u0,u1,... columns.
        #[arg(long = "design-csv")]
        design_csv: PathBuf,
        /// Headerless numeric square correlation matrix CSV.
        #[arg(long = "correlation-csv")]
        correlation_csv: PathBuf,
        /// Output CSV path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
    },
    /// Run sampled scheduled propulsion faults through scenario execution.
    PropulsionFaults {
        /// Base scenario TOML file.
        scenario: PathBuf,
        /// Fault-library TOML with `[[faults]]` templates.
        #[arg(long = "library-toml")]
        library_toml: PathBuf,
        /// Number of samples to execute.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed for deterministic fault activation.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// Dimension id used for deterministic per-sample activation streams.
        #[arg(long = "dimension-id", default_value_t = 0)]
        dimension_id: u32,
        /// Telemetry channel to read from the terminal row.
        #[arg(long = "metric-channel")]
        metric_channel: String,
        /// Inclusive lower success bound for the terminal metric.
        #[arg(long = "success-min")]
        success_min: Option<f64>,
        /// Inclusive upper success bound for the terminal metric.
        #[arg(long = "success-max")]
        success_max: Option<f64>,
        /// Output CSV path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
        /// Confidence level for the Clopper-Pearson success interval.
        #[arg(long = "confidence", default_value_t = 0.95)]
        confidence: f64,
        /// Optional UQ budget TOML to attach to this campaign report.
        #[arg(long = "uq-toml")]
        uq_toml: Option<PathBuf>,
        /// Required binding credibility floor for the UQ budget.
        #[arg(long = "credibility-floor", default_value = "l0")]
        credibility_floor: String,
        /// Optional Markdown output path for the UQ credibility report.
        #[arg(long = "credibility-report-md")]
        credibility_report_md: Option<PathBuf>,
    },
    /// Resume a scalar checkpoint from a precomputed sample CSV.
    ResumeScalar {
        /// CSV file containing one row per sample.
        samples_csv: PathBuf,
        /// Checkpoint JSON path to load or create.
        #[arg(long = "checkpoint-json")]
        checkpoint_json: PathBuf,
        /// Column containing the scalar quantity of interest.
        #[arg(long = "value-column")]
        value_column: String,
        /// Optional column containing a boolean success outcome.
        #[arg(long = "success-column")]
        success_column: Option<String>,
        /// Total campaign sample count.
        #[arg(long = "sample-count")]
        sample_count: u64,
        /// Campaign seed bound into checkpoint metadata.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// Dimension id used for deterministic per-sample streams.
        #[arg(long = "dimension-id", default_value_t = 0)]
        dimension_id: u32,
        /// Scenario SHA-256 digest bound into checkpoint metadata.
        #[arg(long = "scenario-sha256")]
        scenario_sha256: String,
        /// Toolchain / determinism profile bound into checkpoint metadata.
        #[arg(long = "toolchain-profile")]
        toolchain_profile: String,
        /// Maximum missing samples to record this invocation.
        #[arg(long = "max-new-samples")]
        max_new_samples: u64,
        /// Number of worker threads used while recording missing samples.
        #[arg(long = "workers", default_value_t = 1)]
        workers: usize,
        /// Confidence level for the final Clopper-Pearson success interval.
        #[arg(long = "confidence", default_value_t = 0.95)]
        confidence: f64,
    },
}

/// Wilks tolerance-bound side.
#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum McWilksSide {
    /// One-sided first-order tolerance bound.
    OneSided,
    /// Two-sided min/max tolerance interval.
    TwoSided,
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
    /// Run a case with the estimate-vs-truth SIL monitor, evaluating the
    /// case's declared SIL acceptance checks against the in-loop flight
    /// controller's response.
    Observe {
        /// Mission package manifest.
        package: PathBuf,
        /// Optional package test-case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// Record every Nth tick in the observation sample series
        /// (the summary always covers every tick).
        #[arg(long = "decimation", default_value_t = 1)]
        decimation: u64,
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
