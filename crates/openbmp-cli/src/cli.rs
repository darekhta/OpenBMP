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
        /// Append a deterministic run record to this JSONL index.
        #[arg(long = "run-index-jsonl")]
        run_index_jsonl: Option<PathBuf>,
        /// Write a deterministic Parquet mirror of the run index.
        #[arg(long = "run-index-parquet")]
        run_index_parquet: Option<PathBuf>,
        /// Stable code/config id recorded in run-data records.
        #[arg(long = "run-code-id", default_value = "openbmp-cli")]
        run_code_id: String,
        /// Evaluate a deterministic TOML verdict-rule document after the run.
        #[arg(long = "run-verdict-rules")]
        run_verdict_rules: Option<PathBuf>,
        /// Write the deterministic verdict report JSON here.
        #[arg(long = "run-verdict-json")]
        run_verdict_json: Option<PathBuf>,
        /// Optional JSONL run index whose last record is used as baseline.
        #[arg(long = "run-verdict-baseline-jsonl")]
        run_verdict_baseline_jsonl: Option<PathBuf>,
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
        /// Number of deterministic MMS campaign cases to run.
        #[arg(long = "campaign-cases", default_value_t = 1)]
        campaign_cases: u32,
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
    /// Run offline system-identification and model-correlation commands.
    Sysid {
        /// System-identification command to run.
        #[command(subcommand)]
        command: SysidCommand,
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
    /// Plan deterministic run-data campaigns.
    Campaign {
        /// Campaign command to run.
        #[command(subcommand)]
        command: CampaignCommand,
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
    /// Write a LOCAL-only BET/code-to-code comparison workflow manifest.
    LocalWorkflow {
        /// Scenario TOML file to export from OpenBMP.
        scenario: PathBuf,
        /// Deterministic workflow TOML output path.
        #[arg(long = "output-toml")]
        output_toml: PathBuf,
        /// Local OpenBMP trajectory CSV path to produce with export-trajectory.
        #[arg(long = "trajectory-csv")]
        trajectory_csv: PathBuf,
        /// Local compare-telemetry mapping TOML path to produce.
        #[arg(long = "mapping-toml")]
        mapping_toml: PathBuf,
        /// Local external reference CSV path. This file is not read or copied.
        #[arg(long = "external-reference-csv")]
        external_reference_csv: PathBuf,
        /// Tolerance pack to reference in the workflow.
        #[arg(long = "pack", value_enum, default_value_t = TrajectoryTolerancePack::LeoResearch)]
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
    /// Recenter first-stage I-load pitch/yaw bias tables through spliced DOL wind.
    DolRedesign {
        /// Scenario TOML file containing `[day_of_launch.wind] kind = "spliced"`.
        scenario: PathBuf,
        /// Target apogee radius from the central body, in m.
        #[arg(long = "target-apogee-radius-m")]
        target_apogee_radius_m: f64,
        /// Load-window lower altitude, m.
        #[arg(long = "load-window-min-altitude-m")]
        load_window_min_altitude_m: f64,
        /// Load-window upper altitude, m.
        #[arg(long = "load-window-max-altitude-m")]
        load_window_max_altitude_m: f64,
        /// Reference design airspeed through the load window, m/s.
        #[arg(long = "airspeed-m-s")]
        airspeed_m_s: f64,
        /// Desired mean angle of attack, rad.
        #[arg(
            long = "alpha-reference-rad",
            default_value_t = 0.0,
            allow_hyphen_values = true
        )]
        alpha_reference_rad: f64,
        /// Desired mean sideslip, rad.
        #[arg(
            long = "beta-reference-rad",
            default_value_t = 0.0,
            allow_hyphen_values = true
        )]
        beta_reference_rad: f64,
        /// Existing pitch steering bias, rad.
        #[arg(
            long = "stale-pitch-bias-rad",
            default_value_t = 0.0,
            allow_hyphen_values = true
        )]
        stale_pitch_bias_rad: f64,
        /// Existing yaw steering bias, rad.
        #[arg(
            long = "stale-yaw-bias-rad",
            default_value_t = 0.0,
            allow_hyphen_values = true
        )]
        stale_yaw_bias_rad: f64,
        /// Scalar load-angle limit for the toy q-alpha/q-beta surrogate, rad.
        #[arg(long = "load-limit-angle-rad")]
        load_limit_angle_rad: f64,
        /// Terminal/insertion residual norm supplied by the design pass, m.
        #[arg(long = "insertion-residual-norm-m", default_value_t = 0.0)]
        insertion_residual_norm_m: f64,
        /// Accepted terminal/insertion residual limit, m.
        #[arg(long = "insertion-residual-limit-m")]
        insertion_residual_limit_m: f64,
        /// Deterministic synthesis seed recorded in the I-load header.
        #[arg(long = "synthesis-seed", default_value_t = 0)]
        synthesis_seed: u64,
        /// Opaque scenario digest recorded in the I-load metadata.
        ///
        /// Omit to record the SHA-256 digest of the scenario file.
        #[arg(long = "scenario-digest")]
        scenario_digest: Option<String>,
        /// Design-profile digest recorded in the DOL update artifact.
        ///
        /// Omit to record the SHA-256 digest of the scenario file.
        #[arg(long = "design-profile-digest")]
        design_profile_digest: Option<String>,
        /// Source revision recorded in the I-load metadata.
        #[arg(long = "source-revision", default_value = "working-tree")]
        source_revision: String,
        /// Producer name recorded in the I-load header.
        #[arg(long = "producer", default_value = "openbmp-trajopt-cli")]
        producer: String,
        /// Solver provenance recorded in the DOL update artifact.
        #[arg(
            long = "solver-provenance",
            default_value = "dol_spliced_wind_recenter"
        )]
        solver_provenance: String,
        /// Postcard I-load output path.
        #[arg(long = "output-iload")]
        output_iload: PathBuf,
        /// Optional deterministic JSON DOL update report output path.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
    },
    /// Build a DOL go/no-go report card from verifier indicator CSV.
    DolReportCard {
        /// DOL update artifact JSON produced by `trajopt dol-redesign`.
        update_report_json: PathBuf,
        /// Runner/SIL verification indicator CSV.
        indicators_csv: PathBuf,
        /// Stable report identifier.
        #[arg(long = "report-id", default_value = "dol-report-card")]
        report_id: String,
        /// Verification provenance label recorded in the report card.
        #[arg(
            long = "verification-provenance",
            default_value = "file_fed_runner_indicators"
        )]
        verification_provenance: String,
        /// Minimum accepted redesigned load margin, rad.
        #[arg(long = "min-required-margin-rad", allow_hyphen_values = true)]
        min_required_margin_rad: f64,
        /// Minimum accepted redesigned not-worse sample fraction.
        #[arg(long = "min-not-worse-fraction")]
        min_not_worse_fraction: f64,
        /// Deterministic CSV report-card output path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
        /// Optional deterministic JSON report-card output path.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
    },
    /// Latch DOL report-card, persistence, and COLA artifacts into commit readiness.
    DolCommitReadiness {
        /// DOL report-card JSON produced by `trajopt dol-report-card` or verifier commands.
        report_card_json: PathBuf,
        /// DOL persistence knockdown report JSON.
        persistence_json: PathBuf,
        /// DOL launch-COLA screening report JSON.
        cola_json: PathBuf,
        /// Stable readiness identifier.
        #[arg(long = "readiness-id", default_value = "dol-commit-readiness")]
        readiness_id: String,
        /// Deterministic CSV commit-readiness output path.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
        /// Optional deterministic JSON commit-readiness output path.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
    },
    /// Materialize DOL report-card indicators from runner/SIL telemetry CSV.
    DolMaterializeIndicators {
        /// Native runner or SIL telemetry CSV.
        telemetry_csv: PathBuf,
        /// Indicator spec CSV with name, design_value, telemetry_column, reducer, and allowed_abs_error.
        spec_csv: PathBuf,
        /// Deterministic indicator CSV output path for `trajopt dol-report-card`.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
    },
    /// Run a verifier scenario and build a DOL go/no-go report card.
    DolVerifyRun {
        /// Verification scenario run through the native runner.
        scenario: PathBuf,
        /// DOL update artifact JSON produced by `trajopt dol-redesign`.
        update_report_json: PathBuf,
        /// Updated I-load payload produced by `trajopt dol-redesign`.
        updated_iload: PathBuf,
        /// Indicator spec CSV with name, design_value, telemetry_column, reducer, and allowed_abs_error.
        indicator_spec_csv: PathBuf,
        /// Verifier design-profile digest; omit to use the scenario file SHA-256.
        #[arg(long = "verifier-design-profile-digest")]
        verifier_design_profile_digest: Option<String>,
        /// Stable report identifier.
        #[arg(long = "report-id", default_value = "dol-verify-run")]
        report_id: String,
        /// Verification provenance label recorded in the report card.
        #[arg(
            long = "verification-provenance",
            default_value = "native_runner_telemetry"
        )]
        verification_provenance: String,
        /// Minimum accepted redesigned load margin, rad.
        #[arg(long = "min-required-margin-rad", allow_hyphen_values = true)]
        min_required_margin_rad: f64,
        /// Minimum accepted redesigned not-worse sample fraction.
        #[arg(long = "min-not-worse-fraction")]
        min_not_worse_fraction: f64,
        /// Deterministic verifier telemetry CSV output path.
        #[arg(long = "output-telemetry-csv")]
        output_telemetry_csv: PathBuf,
        /// Deterministic materialized indicator CSV output path.
        #[arg(long = "output-indicators-csv")]
        output_indicators_csv: PathBuf,
        /// Deterministic report-card CSV output path.
        #[arg(long = "output-report-csv")]
        output_report_csv: PathBuf,
        /// Optional deterministic JSON report-card output path.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
    },
    /// Run a SIL verifier package and build a DOL go/no-go report card.
    DolVerifySil {
        /// SIL mission package manifest run through the native SIL testbench.
        package: PathBuf,
        /// Optional SIL package case id.
        #[arg(long = "case")]
        case: Option<String>,
        /// DOL update artifact JSON produced by `trajopt dol-redesign`.
        update_report_json: PathBuf,
        /// Updated I-load payload produced by `trajopt dol-redesign`.
        updated_iload: PathBuf,
        /// Indicator spec CSV with name, design_value, telemetry_column, reducer, and allowed_abs_error.
        indicator_spec_csv: PathBuf,
        /// Verifier design-profile digest; omit to use the package manifest SHA-256.
        #[arg(long = "verifier-design-profile-digest")]
        verifier_design_profile_digest: Option<String>,
        /// Stable report identifier.
        #[arg(long = "report-id", default_value = "dol-verify-sil")]
        report_id: String,
        /// Verification provenance label recorded in the report card.
        #[arg(
            long = "verification-provenance",
            default_value = "native_sil_stimulated_telemetry"
        )]
        verification_provenance: String,
        /// Minimum accepted redesigned load margin, rad.
        #[arg(long = "min-required-margin-rad", allow_hyphen_values = true)]
        min_required_margin_rad: f64,
        /// Minimum accepted redesigned not-worse sample fraction.
        #[arg(long = "min-not-worse-fraction")]
        min_not_worse_fraction: f64,
        /// Deterministic SIL telemetry CSV output path.
        #[arg(long = "output-telemetry-csv")]
        output_telemetry_csv: PathBuf,
        /// Deterministic materialized indicator CSV output path.
        #[arg(long = "output-indicators-csv")]
        output_indicators_csv: PathBuf,
        /// Deterministic report-card CSV output path.
        #[arg(long = "output-report-csv")]
        output_report_csv: PathBuf,
        /// Deterministic SIL evidence bundle directory.
        #[arg(long = "output-evidence-dir")]
        output_evidence_dir: PathBuf,
        /// Optional deterministic JSON report-card output path.
        #[arg(long = "report-json")]
        report_json: Option<PathBuf>,
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

/// Run-data campaign subcommands.
#[derive(Debug, Subcommand)]
pub enum CampaignCommand {
    /// Expand a campaign manifest into a deterministic plan JSON.
    Plan {
        /// Campaign manifest TOML file.
        manifest: PathBuf,
        /// Deterministic campaign plan JSON output path.
        #[arg(long = "output-plan-json")]
        output_plan_json: PathBuf,
        /// Optional deterministic worker-shard JSON output path.
        #[arg(long = "output-shards-json")]
        output_shards_json: Option<PathBuf>,
        /// Worker count used when writing shard JSON.
        #[arg(long = "workers", default_value_t = 1)]
        workers: u64,
    },
    /// Execute a campaign manifest through the native runner.
    Run {
        /// Campaign manifest TOML file.
        manifest: PathBuf,
        /// Output directory for plan, telemetry, verdict, index, and summary artifacts.
        #[arg(long = "output-dir")]
        output_dir: PathBuf,
        /// Stable code/config id recorded in run-data records.
        #[arg(long = "run-code-id", default_value = "openbmp-campaign-cli")]
        run_code_id: String,
        /// Code ordinal label recorded for trend-store consumers.
        #[arg(long = "code-ordinal", default_value_t = 0)]
        code_ordinal: u64,
        /// Also write a deterministic Parquet mirror of the campaign run index.
        #[arg(long = "write-index-parquet", default_value_t = false)]
        write_index_parquet: bool,
        /// Maximum worker threads used for campaign run execution.
        #[arg(long = "workers", default_value_t = 1)]
        workers: u64,
        /// Return a non-zero exit after writing artifacts when verdict or trend gates fail.
        #[arg(long = "fail-on-ci-gates", default_value_t = false)]
        fail_on_ci_gates: bool,
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
    /// Generate campaign overlay files for deterministic aero deck draws.
    AeroDeckDrawOverlays {
        /// Number of deck-draw overlays to materialize.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed bound into each deterministic deck draw.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// First sample index to materialize.
        #[arg(long = "first-sample-index", default_value_t = 0)]
        first_sample_index: u64,
        /// First RNG dimension id used by CN, CD, and CM draw streams.
        #[arg(long = "dimension-id-base")]
        dimension_id_base: u32,
        /// Exponential Mach correlation length in Mach units. When omitted,
        /// deck-declared uncertainty correlation metadata is used at runtime.
        #[arg(long = "mach-correlation-length")]
        mach_correlation_length: Option<f64>,
        /// Optional exponential angle-of-attack correlation length in degrees.
        #[arg(long = "alpha-correlation-length-deg")]
        alpha_correlation_length_deg: Option<f64>,
        /// Optional exponential sideslip correlation length in degrees.
        #[arg(long = "beta-correlation-length-deg")]
        beta_correlation_length_deg: Option<f64>,
        /// Prefix for deterministic campaign overlay ids and filenames.
        #[arg(long = "overlay-prefix", default_value = "aero-deck-draw")]
        overlay_prefix: String,
        /// Directory receiving one overlay TOML file per sample.
        #[arg(long = "output-dir")]
        output_dir: PathBuf,
        /// Campaign-manifest TOML fragment listing generated overlays.
        #[arg(long = "manifest-toml")]
        manifest_toml: PathBuf,
    },
    /// Generate and execute an aero deck-draw campaign through the native runner.
    AeroDeckDrawCampaign {
        /// Base scenario TOML file.
        scenario: PathBuf,
        /// Number of deck-draw samples to execute.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed bound into each deterministic deck draw.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// First sample index to execute.
        #[arg(long = "first-sample-index", default_value_t = 0)]
        first_sample_index: u64,
        /// First RNG dimension id used by CN, CD, and CM draw streams.
        #[arg(long = "dimension-id-base")]
        dimension_id_base: u32,
        /// Exponential Mach correlation length in Mach units. When omitted,
        /// deck-declared uncertainty correlation metadata is used at runtime.
        #[arg(long = "mach-correlation-length")]
        mach_correlation_length: Option<f64>,
        /// Optional exponential angle-of-attack correlation length in degrees.
        #[arg(long = "alpha-correlation-length-deg")]
        alpha_correlation_length_deg: Option<f64>,
        /// Optional exponential sideslip correlation length in degrees.
        #[arg(long = "beta-correlation-length-deg")]
        beta_correlation_length_deg: Option<f64>,
        /// Prefix for deterministic campaign overlay ids and filenames.
        #[arg(long = "overlay-prefix", default_value = "aero-deck-draw")]
        overlay_prefix: String,
        /// Stable generated campaign id.
        #[arg(long = "campaign-id", default_value = "aero-deck-draw")]
        campaign_id: String,
        /// Stable generated scenario id.
        #[arg(long = "scenario-id", default_value = "base")]
        scenario_id: String,
        /// Scenario seed used for each generated campaign run.
        #[arg(long = "run-seed", default_value_t = 0)]
        run_seed: u64,
        /// Output root for generated overlays, manifest, and campaign artifacts.
        #[arg(long = "output-dir")]
        output_dir: PathBuf,
        /// Stable code/config id recorded in run-data records.
        #[arg(
            long = "run-code-id",
            default_value = "openbmp-cli-aero-deck-draw-campaign-v1"
        )]
        run_code_id: String,
        /// Code ordinal label recorded for trend-store consumers.
        #[arg(long = "code-ordinal", default_value_t = 0)]
        code_ordinal: u64,
        /// Also write a deterministic Parquet mirror of the campaign run index.
        #[arg(long = "write-index-parquet", default_value_t = false)]
        write_index_parquet: bool,
        /// Maximum worker threads used for generated campaign execution.
        #[arg(long = "workers", default_value_t = 1)]
        workers: u64,
        /// Optional UQ budget TOML to attach to this generated campaign.
        #[arg(long = "uq-toml")]
        uq_toml: Option<PathBuf>,
        /// Required binding credibility floor for the UQ budget.
        #[arg(long = "credibility-floor", default_value = "l0")]
        credibility_floor: String,
        /// Optional Markdown output path for the UQ credibility report.
        #[arg(long = "credibility-report-md")]
        credibility_report_md: Option<PathBuf>,
    },
    /// Generate schema-v3 ascent acoustic trace samples from an MC design.
    AcousticsAscentTrace {
        /// Number of trace samples to materialize.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed for LHS or Owen-scrambled Sobol designs.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// Unit-cube design family.
        #[arg(long = "design", value_enum, default_value_t = McAcousticsAscentTraceDesign::LatinHypercube)]
        design: McAcousticsAscentTraceDesign,
        /// Lower dynamic pressure bound, Pa.
        #[arg(long = "dynamic-pressure-min-pa")]
        dynamic_pressure_min_pa: f64,
        /// Upper dynamic pressure bound, Pa.
        #[arg(long = "dynamic-pressure-max-pa")]
        dynamic_pressure_max_pa: f64,
        /// Lower Mach bound.
        #[arg(long = "mach-min")]
        mach_min: f64,
        /// Upper Mach bound.
        #[arg(long = "mach-max")]
        mach_max: f64,
        /// Prefix for deterministic sample ids.
        #[arg(long = "sample-prefix", default_value = "mc")]
        sample_prefix: String,
        /// Optional per-row ascent class override.
        #[arg(long = "class-id")]
        class_id: Option<String>,
        /// Output TOML snippet containing [[acoustics.ascent.samples]] rows.
        #[arg(long = "output-toml")]
        output_toml: PathBuf,
    },
    /// Generate schema-v3 vibroacoustic response samples from an MC design.
    AcousticsResponseSamples {
        /// Number of response samples to materialize.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed for LHS or Owen-scrambled Sobol designs.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// Unit-cube design family.
        #[arg(long = "design", value_enum, default_value_t = McAcousticsAscentTraceDesign::LatinHypercube)]
        design: McAcousticsAscentTraceDesign,
        /// Lower panel surface mass bound, kg/m^2.
        #[arg(long = "surface-mass-min-kg-m2")]
        surface_mass_min_kg_m2: f64,
        /// Upper panel surface mass bound, kg/m^2.
        #[arg(long = "surface-mass-max-kg-m2")]
        surface_mass_max_kg_m2: f64,
        /// Lower responding area bound, m^2.
        #[arg(long = "area-min-m2")]
        area_min_m2: f64,
        /// Upper responding area bound, m^2.
        #[arg(long = "area-max-m2")]
        area_max_m2: f64,
        /// Lower modal shape participation factor bound.
        #[arg(long = "mode-shape-min")]
        mode_shape_min: f64,
        /// Upper modal shape participation factor bound.
        #[arg(long = "mode-shape-max")]
        mode_shape_max: f64,
        /// Prefix for deterministic sample ids.
        #[arg(long = "sample-prefix", default_value = "response-mc")]
        sample_prefix: String,
        /// Optional per-row response class override.
        #[arg(long = "class-id")]
        class_id: Option<String>,
        /// Output TOML snippet containing [[acoustics.response.samples]] rows.
        #[arg(long = "output-toml")]
        output_toml: PathBuf,
    },
    /// Generate and run an MC-designed ascent acoustic trace campaign.
    AcousticsAscentCampaign {
        /// Base scenario TOML file with an [acoustics.ascent] block.
        scenario: PathBuf,
        /// Number of trace samples to evaluate.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed for LHS or Owen-scrambled Sobol designs.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// Unit-cube design family.
        #[arg(long = "design", value_enum, default_value_t = McAcousticsAscentTraceDesign::LatinHypercube)]
        design: McAcousticsAscentTraceDesign,
        /// Lower dynamic pressure bound, Pa.
        #[arg(long = "dynamic-pressure-min-pa")]
        dynamic_pressure_min_pa: f64,
        /// Upper dynamic pressure bound, Pa.
        #[arg(long = "dynamic-pressure-max-pa")]
        dynamic_pressure_max_pa: f64,
        /// Lower Mach bound.
        #[arg(long = "mach-min")]
        mach_min: f64,
        /// Upper Mach bound.
        #[arg(long = "mach-max")]
        mach_max: f64,
        /// Prefix for deterministic sample ids.
        #[arg(long = "sample-prefix", default_value = "mc")]
        sample_prefix: String,
        /// Optional per-row ascent class override.
        #[arg(long = "class-id")]
        class_id: Option<String>,
        /// Output CSV path for evaluated trace samples.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
        /// Optional directory for evaluated campaign CSV artifacts.
        #[arg(long = "output-artifact-dir")]
        output_artifact_dir: Option<PathBuf>,
        /// Optional generated full scenario TOML path.
        #[arg(long = "output-scenario-toml")]
        output_scenario_toml: Option<PathBuf>,
        /// Optional checkpoint JSON for resumable per-sample campaign evaluation.
        #[arg(long = "checkpoint-json")]
        checkpoint_json: Option<PathBuf>,
        /// Maximum missing samples to evaluate into the checkpoint this invocation.
        #[arg(long = "max-new-samples")]
        max_new_samples: Option<u64>,
        /// Worker threads for checkpointed sample evaluation.
        #[arg(long = "workers", default_value_t = 1)]
        workers: usize,
        /// Toolchain / determinism profile bound into checkpoint metadata.
        #[arg(
            long = "toolchain-profile",
            default_value = "openbmp-cli-acoustics-ascent-campaign-v1"
        )]
        toolchain_profile: String,
    },
    /// Generate and run an MC-designed vibroacoustic response campaign.
    AcousticsResponseCampaign {
        /// Base scenario TOML file with [acoustics.ascent] and [acoustics.response].
        scenario: PathBuf,
        /// Number of trace samples to evaluate.
        #[arg(long = "samples")]
        samples: u64,
        /// Campaign seed for LHS or Owen-scrambled Sobol designs.
        #[arg(long = "campaign-seed")]
        campaign_seed: u64,
        /// Unit-cube design family.
        #[arg(long = "design", value_enum, default_value_t = McAcousticsAscentTraceDesign::LatinHypercube)]
        design: McAcousticsAscentTraceDesign,
        /// Lower dynamic pressure bound, Pa.
        #[arg(long = "dynamic-pressure-min-pa")]
        dynamic_pressure_min_pa: f64,
        /// Upper dynamic pressure bound, Pa.
        #[arg(long = "dynamic-pressure-max-pa")]
        dynamic_pressure_max_pa: f64,
        /// Lower Mach bound.
        #[arg(long = "mach-min")]
        mach_min: f64,
        /// Upper Mach bound.
        #[arg(long = "mach-max")]
        mach_max: f64,
        /// Prefix for deterministic sample ids.
        #[arg(long = "sample-prefix", default_value = "mc")]
        sample_prefix: String,
        /// Optional per-row ascent class override.
        #[arg(long = "class-id")]
        class_id: Option<String>,
        /// Optional lower response panel surface mass bound, kg/m^2.
        #[arg(long = "response-surface-mass-min-kg-m2")]
        response_surface_mass_min_kg_m2: Option<f64>,
        /// Optional upper response panel surface mass bound, kg/m^2.
        #[arg(long = "response-surface-mass-max-kg-m2")]
        response_surface_mass_max_kg_m2: Option<f64>,
        /// Optional lower response area bound, m^2.
        #[arg(long = "response-area-min-m2")]
        response_area_min_m2: Option<f64>,
        /// Optional upper response area bound, m^2.
        #[arg(long = "response-area-max-m2")]
        response_area_max_m2: Option<f64>,
        /// Optional lower response modal shape participation factor bound.
        #[arg(long = "response-mode-shape-min")]
        response_mode_shape_min: Option<f64>,
        /// Optional upper response modal shape participation factor bound.
        #[arg(long = "response-mode-shape-max")]
        response_mode_shape_max: Option<f64>,
        /// Prefix for deterministic response sample ids when generated.
        #[arg(long = "response-sample-prefix", default_value = "response-mc")]
        response_sample_prefix: String,
        /// Optional per-row response class override when generated.
        #[arg(long = "response-class-id")]
        response_class_id: Option<String>,
        /// Output CSV path for evaluated ascent trace samples.
        #[arg(long = "output-csv")]
        output_csv: PathBuf,
        /// Optional directory for evaluated campaign CSV artifacts.
        #[arg(long = "output-artifact-dir")]
        output_artifact_dir: Option<PathBuf>,
        /// Optional generated full scenario TOML path.
        #[arg(long = "output-scenario-toml")]
        output_scenario_toml: Option<PathBuf>,
        /// Optional checkpoint JSON for resumable per-sample campaign evaluation.
        #[arg(long = "checkpoint-json")]
        checkpoint_json: Option<PathBuf>,
        /// Maximum missing samples to evaluate into the checkpoint this invocation.
        #[arg(long = "max-new-samples")]
        max_new_samples: Option<u64>,
        /// Worker threads for checkpointed sample evaluation.
        #[arg(long = "workers", default_value_t = 1)]
        workers: usize,
        /// Toolchain / determinism profile bound into checkpoint metadata.
        #[arg(
            long = "toolchain-profile",
            default_value = "openbmp-cli-acoustics-response-campaign-v1"
        )]
        toolchain_profile: String,
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

/// Design family for acoustics ascent trace materialization.
#[derive(Clone, Copy, Debug, Eq, PartialEq, ValueEnum)]
pub enum McAcousticsAscentTraceDesign {
    /// Latin hypercube design keyed by campaign seed.
    LatinHypercube,
    /// Unscrambled Sobol base design.
    Sobol,
    /// Owen-scrambled Sobol design keyed by campaign seed.
    OwenSobol,
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

/// System-identification and model-correlation subcommands.
#[derive(Debug, Subcommand)]
pub enum SysidCommand {
    /// Compare declared scalar channels from two local CSV time series.
    Residuals {
        /// Reference or truth CSV.
        #[arg(long = "reference")]
        reference_csv: PathBuf,
        /// Actual or candidate CSV.
        #[arg(long = "actual")]
        actual_csv: PathBuf,
        /// TOML mapping declaring columns, units, frames, tolerances, and phases.
        #[arg(long = "mapping")]
        mapping: PathBuf,
        /// Deterministic JSON residual report path.
        #[arg(long = "report-json")]
        report_json: PathBuf,
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
