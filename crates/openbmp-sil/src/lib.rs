//! `openbmp-sil` — host software-in-the-loop package and evidence API.
//!
//! This crate is the native OpenBMP SIL control surface. It deliberately
//! stays above the simulation kernels: callers load a mission package,
//! select a test case, run the normal runner path, and receive a
//! reviewable evidence bundle. The API is shaped after common SIL/HIL
//! bench operations (load, reset, step, run-until, fault/parameter/command
//! stimulation, signal readout, bus-frame capture, evidence export) without
//! claiming ASAM XIL conformance.

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;
use std::time::{SystemTime, UNIX_EPOCH};

use openbmp_hal::{decode_iload_envelope, encode_iload_envelope};
use openbmp_runner::{RunOutcome, RunnerError};
use openbmp_scenario::{Scenario, ScenarioError};
use openbmp_telemetry::{TelemetryTable, TelemetryValue, TelemetryValueKind};
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

/// Payload schema used for package-generated flight-controller I-loads.
///
/// The payload is a canonical TOML document containing the scenario's
/// validated `[fc]` table. It is wrapped in the HAL `OBIL` envelope so
/// package checks exercise the same CRC/version path as hardware I-load
/// storage.
pub const PACKAGE_ILOAD_PAYLOAD_SCHEMA_VERSION: u16 = 1;

/// Errors surfaced by the native SIL package/testbench API.
#[derive(Debug, Error)]
pub enum SilError {
    /// File IO failed.
    #[error("io error: {path}")]
    Io {
        /// Offending path.
        path: PathBuf,
        /// Source IO error.
        #[source]
        source: std::io::Error,
    },
    /// Mission package TOML parsing failed.
    #[error("mission package TOML error: {path}")]
    PackageToml {
        /// Offending manifest path.
        path: PathBuf,
        /// Source TOML parse error.
        #[source]
        source: toml::de::Error,
    },
    /// Scenario TOML serialization failed while creating a step-limited run.
    #[error("scenario TOML serialization error")]
    ScenarioTomlSerialize(#[from] toml::ser::Error),
    /// Scenario parsing or validation failed.
    #[error("scenario error")]
    Scenario(#[from] ScenarioError),
    /// Runner execution failed.
    #[error("runner error")]
    Runner(#[from] RunnerError),
    /// JSON evidence serialization failed.
    #[error("evidence JSON error: {path}")]
    EvidenceJson {
        /// Offending evidence path.
        path: PathBuf,
        /// Source JSON error.
        #[source]
        source: serde_json::Error,
    },
    /// Manifest structure is invalid.
    #[error("mission package error: {summary}")]
    Package {
        /// Human-readable failure summary.
        summary: String,
    },
    /// A declared package hash did not match the file content.
    #[error("hash mismatch for {path}: expected {expected}, actual {actual}")]
    HashMismatch {
        /// File whose digest was checked.
        path: PathBuf,
        /// Expected hex digest.
        expected: String,
        /// Actual hex digest.
        actual: String,
    },
    /// A package sidecar does not match its scenario source.
    #[error("package sidecar mismatch: {summary}")]
    SidecarMismatch {
        /// Human-readable mismatch summary.
        summary: String,
    },
    /// An I-load sidecar could not be encoded or decoded.
    #[error("package I-load error: {summary}")]
    Iload {
        /// Human-readable I-load failure summary.
        summary: String,
    },
    /// A run-until target could not be resolved.
    #[error("SIL run-until target error: {summary}")]
    RunUntil {
        /// Human-readable target failure summary.
        summary: String,
    },
    /// A requested telemetry channel is absent.
    #[error("telemetry channel `{channel}` not found")]
    TelemetryChannel {
        /// Requested channel name.
        channel: String,
    },
    /// A SIL stimulation request could not be applied.
    #[error("SIL stimulation error: {summary}")]
    Stimulation {
        /// Human-readable stimulation failure summary.
        summary: String,
    },
}

/// Top-level mission package wrapper.
#[derive(Clone, Debug)]
pub struct MissionPackage {
    /// Path to the package manifest.
    pub manifest_path: PathBuf,
    /// Directory used to resolve relative package files.
    pub root: PathBuf,
    /// Parsed package manifest.
    pub manifest: MissionPackageManifest,
}

/// Versioned OpenBMP mission package manifest.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct MissionPackageManifest {
    /// Package identity and version.
    pub package: PackageInfo,
    /// Files that make up the package.
    pub files: PackageFiles,
    /// Optional SIL test cases.
    #[serde(default)]
    pub test_cases: Vec<PackageTestCase>,
    /// Optional expected SHA-256 digests keyed by package-relative path.
    #[serde(default)]
    pub hashes: BTreeMap<String, String>,
    /// Optional free-form provenance note.
    #[serde(default)]
    pub provenance: Option<String>,
}

/// Mission package identity.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct PackageInfo {
    /// Stable package id.
    pub id: String,
    /// Package version string.
    pub version: String,
}

/// File references inside a mission package.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct PackageFiles {
    /// Default scenario run file.
    pub scenario: PathBuf,
    /// Optional plant configuration sidecar.
    #[serde(default)]
    pub plant_config: Option<PathBuf>,
    /// Optional flight-software I-load sidecar.
    #[serde(default)]
    pub iload: Option<PathBuf>,
    /// Optional mission graph sidecar.
    #[serde(default)]
    pub mission_graph: Option<PathBuf>,
    /// Optional scenario-script sidecar.
    #[serde(default)]
    pub scenario_script: Option<PathBuf>,
    /// Optional command/telemetry dictionary sidecar.
    #[serde(default)]
    pub dictionary: Option<PathBuf>,
    /// Optional package provenance file.
    #[serde(default)]
    pub provenance: Option<PathBuf>,
}

/// One SIL test case declared by a mission package.
#[derive(Clone, Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct PackageTestCase {
    /// Case id selected from the CLI/API.
    pub id: String,
    /// Optional scenario override for this case.
    #[serde(default)]
    pub scenario: Option<PathBuf>,
    /// Optional human-readable objective.
    #[serde(default)]
    pub objective: Option<String>,
}

/// Result of validating a package manifest.
#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct PackageCheckReport {
    /// Package id.
    pub package_id: String,
    /// Package version.
    pub package_version: String,
    /// Canonical package manifest path.
    pub manifest_path: PathBuf,
    /// Scenario path selected by the package default.
    pub scenario_path: PathBuf,
    /// SHA-256 digests recorded during the check.
    pub hashes: BTreeMap<String, String>,
}

/// One sidecar generated from a package scenario.
#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct MaterializedSidecar {
    /// Manifest field that selected the sidecar.
    pub field: String,
    /// Sidecar path written.
    pub path: PathBuf,
    /// SHA-256 digest of the generated sidecar.
    pub sha256: String,
}

/// Report returned after materializing package sidecars.
#[derive(Clone, Debug, Serialize, PartialEq, Eq)]
pub struct MaterializeSidecarsReport {
    /// Package id.
    pub package_id: String,
    /// Package version.
    pub package_version: String,
    /// Sidecars written.
    pub sidecars: Vec<MaterializedSidecar>,
}

/// Result of a package run.
#[derive(Debug)]
pub struct SilRunReport {
    /// Selected package id.
    pub package_id: String,
    /// Selected case id.
    pub case_id: String,
    /// Runner stop label.
    pub stop_label: String,
    /// Final simulation step.
    pub final_step: u64,
    /// Final simulation time.
    pub final_time_s: f64,
    /// Evidence bundle for this run.
    pub evidence: EvidenceBundle,
    /// Telemetry captured during this run.
    pub telemetry: TelemetryTable,
}

/// Target condition for native `run until ...` SIL operations.
#[derive(Clone, Debug, PartialEq)]
pub enum RunUntilTarget {
    /// Run until simulation time reaches this absolute time in seconds.
    TimeS(f64),
    /// Run until the declared mission event's trigger fires.
    Event(String),
    /// Run until a transition into this mission phase/state fires.
    Phase(String),
}

/// In-memory SIL stimulation applied to one selected package run.
///
/// The stimulation is projected into the scenario TOML before parsing and
/// validation. Package source files and sidecars are not modified.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct SilStimulation {
    /// Load-time fault injections.
    pub faults: Vec<FaultInjection>,
    /// Parameter or I-load-style scenario overrides.
    pub parameter_overrides: Vec<ParameterOverride>,
    /// One-shot command writes.
    pub command_writes: Vec<CommandWrite>,
}

impl SilStimulation {
    /// `true` when this request carries no stimulation operations.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.faults.is_empty()
            && self.parameter_overrides.is_empty()
            && self.command_writes.is_empty()
    }
}

/// Target of a SIL fault injection.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum FaultTarget {
    /// Engine id from `[[vehicle.assembly.engines]]`.
    Engine(String),
    /// Effector id from `[[vehicle.assembly.effectors]]`.
    Effector(String),
}

/// One load-time SIL fault injection.
#[derive(Clone, Debug, PartialEq)]
pub struct FaultInjection {
    /// Engine or effector target.
    pub target: FaultTarget,
    /// Fault payload using the scenario fault table shape.
    ///
    /// Examples: `{ kind = "hard_off" }` for engines or
    /// `{ kind = "reduced_rate", factor = 0.7 }` for effectors.
    pub fault: toml::Value,
}

/// One scenario parameter override applied before parsing.
#[derive(Clone, Debug, PartialEq)]
pub struct ParameterOverride {
    /// Dotted TOML path. Array indexes use `name[index]`, for example
    /// `vehicle.assembly.engines[0].limits.max_thrust_n`.
    pub path: String,
    /// Replacement TOML value.
    pub value: toml::Value,
}

/// One command write projected into `[[scenario_script.events]]`.
#[derive(Clone, Debug, PartialEq)]
pub enum CommandWrite {
    /// One-shot engine command.
    Engine {
        /// Simulation time at which the command fires.
        time_s: f64,
        /// Engine id.
        id: String,
        /// Throttle in `[0, 1]`.
        throttle_unit: f64,
        /// Gimbal pitch command in radians.
        gimbal_pitch_rad: f64,
        /// Gimbal yaw command in radians.
        gimbal_yaw_rad: f64,
        /// Ignition request.
        ignite: bool,
        /// Shutdown request.
        shutdown: bool,
    },
    /// One-shot effector command override.
    Effector {
        /// Simulation time at which the command fires.
        time_s: f64,
        /// Effector id.
        id: String,
        /// Effector command value.
        command: f64,
    },
}

/// One stimulation operation recorded in SIL evidence.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct StimulusRecord {
    /// Stimulation kind.
    pub kind: String,
    /// Target path or id.
    pub target: String,
    /// Human-readable summary.
    pub summary: String,
}

/// One sampled telemetry value returned by the SIL signal-read API.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SignalSample {
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: u64,
    /// Canonical string rendering of the value.
    pub value: String,
}

/// Summary returned by the SIL signal-read API.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct SignalReadReport {
    /// Channel name.
    pub channel: String,
    /// Value kind.
    pub kind: String,
    /// Unit label.
    pub unit: String,
    /// Optional frame label.
    pub frame: Option<String>,
    /// Number of populated samples.
    pub samples: usize,
    /// First populated sample.
    pub first: Option<SignalSample>,
    /// Last populated sample.
    pub last: Option<SignalSample>,
    /// Minimum value for numeric channels.
    pub min: Option<String>,
    /// Maximum value for numeric channels.
    pub max: Option<String>,
    /// Prefix sample list for human inspection.
    pub preview: Vec<SignalSample>,
}

/// One captured host-SIL bus/evidence frame.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct BusFrameEntry {
    /// Frame stream kind.
    pub stream: String,
    /// Subject within the stream.
    pub subject: String,
    /// State/value label.
    pub value: String,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: u64,
}

/// Reviewable SIL evidence emitted for one run.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EvidenceBundle {
    /// Evidence schema version.
    pub schema: String,
    /// Package id.
    pub package_id: String,
    /// Package version.
    pub package_version: String,
    /// Test case id.
    pub case_id: String,
    /// Scenario name from `[meta]`.
    pub scenario_name: String,
    /// Scenario validation label.
    pub validation_label: String,
    /// Runner stop label.
    pub stop_label: String,
    /// Final simulation step.
    pub final_step: u64,
    /// Final simulation time.
    pub final_time_s: f64,
    /// Run verdict: `pass` when every requirement verdict passed (or was
    /// skipped), otherwise `fail`. Derived from [`Self::failures`].
    pub verdict: String,
    /// Unix timestamp in seconds when the evidence was generated.
    pub generated_unix_s: u64,
    /// Git commit if available.
    pub git_commit: Option<String>,
    /// Rust toolchain summary if available.
    pub toolchain: Option<String>,
    /// Host target triple reported by `rustc -vV`, if available.
    pub target_triple: Option<String>,
    /// File and artifact hashes.
    pub hashes: BTreeMap<String, String>,
    /// In-memory SIL stimulation operations applied before the run.
    #[serde(default)]
    pub stimuli: Vec<StimulusRecord>,
    /// Mission marker event trace extracted from telemetry.
    pub event_trace: Vec<EventTraceEntry>,
    /// Active mission phase changes extracted from telemetry.
    pub phase_trace: Vec<PhaseTraceEntry>,
    /// Orthogonal mission-region changes extracted from telemetry.
    pub region_trace: Vec<RegionTraceEntry>,
    /// Command/actuation trace extracted from engine-thrust channels.
    pub command_trace: Vec<CommandTraceEntry>,
    /// ASAM-XIL-inspired host bus-frame capture synthesized from mission,
    /// region, and command traces.
    pub bus_frames: Vec<BusFrameEntry>,
    /// Requirement-style verdict records for this SIL run.
    pub requirement_verdicts: Vec<RequirementVerdict>,
    /// Requirement failures for this run, one per failed
    /// [`RequirementVerdict`]; empty exactly when [`Self::verdict`] is
    /// `pass`.
    pub failures: Vec<EvidenceFailure>,
    /// Telemetry inventory and bounds.
    pub telemetry_summary: TelemetrySummary,
    /// Telemetry row count.
    pub telemetry_rows: usize,
    /// Telemetry channel count.
    pub telemetry_channels: usize,
}

/// One fired mission marker in evidence.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EventTraceEntry {
    /// Marker tag.
    pub tag: String,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: u64,
}

/// One active mission-phase trace entry.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct PhaseTraceEntry {
    /// Phase label or canonical id.
    pub phase: String,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: u64,
}

/// One active mission-region trace entry.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct RegionTraceEntry {
    /// Region channel suffix.
    pub region: String,
    /// Region-state label or canonical id.
    pub state: String,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: u64,
}

/// One command/actuation trace entry.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct CommandTraceEntry {
    /// Telemetry channel that changed state.
    pub channel: String,
    /// Command state label.
    pub state: String,
    /// Simulation time in seconds.
    pub time_s: f64,
    /// Simulation step.
    pub step: u64,
    /// Channel value at the trace point.
    pub value: f64,
}

/// One requirement-style verdict for the run.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq, Eq)]
pub struct RequirementVerdict {
    /// Requirement or evidence-check id.
    pub id: String,
    /// `pass`, `fail`, or `skip`.
    pub verdict: String,
    /// Human-readable summary.
    pub summary: String,
}

/// Failure/divergence summary for a SIL run.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct EvidenceFailure {
    /// Failure label.
    pub kind: String,
    /// Human-readable summary.
    pub summary: String,
    /// First divergent channel, if known.
    pub channel: Option<String>,
    /// First divergent time, if known.
    pub time_s: Option<f64>,
    /// First divergent step, if known.
    pub step: Option<u64>,
}

/// Telemetry inventory and bounds for a SIL run.
#[derive(Clone, Debug, Serialize, Deserialize, PartialEq)]
pub struct TelemetrySummary {
    /// Number of telemetry rows.
    pub rows: usize,
    /// Number of telemetry channels.
    pub channels: usize,
    /// First row timestamp.
    pub first_time_s: Option<f64>,
    /// Last row timestamp.
    pub last_time_s: Option<f64>,
    /// Channel names in archive order.
    pub channel_names: Vec<String>,
}

/// Native SIL testbench object.
#[derive(Debug)]
pub struct SilTestbench {
    package: MissionPackage,
}

impl MissionPackage {
    /// Load a mission package manifest from disk.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] when the file cannot be read or parsed.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, SilError> {
        let path = path.as_ref();
        let text = fs::read_to_string(path).map_err(|source| SilError::Io {
            path: path.to_path_buf(),
            source,
        })?;
        let manifest: MissionPackageManifest =
            toml::from_str(&text).map_err(|source| SilError::PackageToml {
                path: path.to_path_buf(),
                source,
            })?;
        let root = path
            .parent()
            .map_or_else(|| PathBuf::from("."), Path::to_path_buf);
        let package = Self {
            manifest_path: path.to_path_buf(),
            root,
            manifest,
        };
        package.validate_shape()?;
        Ok(package)
    }

    /// Validate package references and declared hashes.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for invalid references or hash mismatches.
    pub fn check(&self) -> Result<PackageCheckReport, SilError> {
        self.validate_shape()?;
        let scenario_path = self.resolve(&self.manifest.files.scenario);
        Scenario::from_file(&scenario_path)?;
        self.validate_sidecars(&scenario_path)?;

        let mut hashes = BTreeMap::new();
        hashes.insert(
            "package.manifest".to_owned(),
            sha256_file(&self.manifest_path)?,
        );
        hashes.insert("files.scenario".to_owned(), sha256_file(&scenario_path)?);
        for (field, path) in self.optional_files() {
            hashes.insert(field.to_owned(), sha256_file(&path)?);
        }

        for (relative, expected) in &self.manifest.hashes {
            let path = self.resolve(Path::new(relative));
            let actual = sha256_file(&path)?;
            if !actual.eq_ignore_ascii_case(expected) {
                return Err(SilError::HashMismatch {
                    path,
                    expected: expected.clone(),
                    actual,
                });
            }
            hashes.insert(format!("hashes.{relative}"), actual);
        }

        Ok(PackageCheckReport {
            package_id: self.manifest.package.id.clone(),
            package_version: self.manifest.package.version.clone(),
            manifest_path: self.manifest_path.clone(),
            scenario_path,
            hashes,
        })
    }

    /// Materialize declared package sidecars from the default scenario.
    ///
    /// Existing sidecar files are replaced. Only sidecars named in
    /// `[files]` are written; absent optional paths are left alone.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for scenario, TOML, I-load, or IO failures.
    pub fn materialize_sidecars(&self) -> Result<MaterializeSidecarsReport, SilError> {
        self.validate_shape()?;
        let scenario_path = self.resolve(&self.manifest.files.scenario);
        Scenario::from_file(&scenario_path)?;
        let scenario = read_toml_value(&scenario_path)?;
        let mut sidecars = Vec::new();

        if let Some(path) = &self.manifest.files.plant_config {
            let output_path = self.resolve(path);
            let value = sidecar_from_keys(&scenario, PLANT_CONFIG_KEYS);
            write_toml_sidecar(&output_path, &value)?;
            sidecars.push(materialized("files.plant_config", &output_path)?);
        }
        if let Some(path) = &self.manifest.files.mission_graph {
            let output_path = self.resolve(path);
            let value = sidecar_from_keys(&scenario, &["mission"]);
            write_toml_sidecar(&output_path, &value)?;
            sidecars.push(materialized("files.mission_graph", &output_path)?);
        }
        if let Some(path) = &self.manifest.files.scenario_script {
            let output_path = self.resolve(path);
            let value = sidecar_from_keys(&scenario, &["scenario_script"]);
            write_toml_sidecar(&output_path, &value)?;
            sidecars.push(materialized("files.scenario_script", &output_path)?);
        }
        if let Some(path) = &self.manifest.files.iload {
            let output_path = self.resolve(path);
            let payload = iload_payload_from_scenario(&scenario)?;
            let mut envelope = vec![0_u8; payload.len() + 32];
            let len = encode_iload_envelope(
                PACKAGE_ILOAD_PAYLOAD_SCHEMA_VERSION,
                payload.as_bytes(),
                &mut envelope,
            )
            .map_err(|source| SilError::Iload {
                summary: source.to_string(),
            })?;
            envelope.truncate(len);
            write_bytes_sidecar(&output_path, &envelope)?;
            sidecars.push(materialized("files.iload", &output_path)?);
        }

        Ok(MaterializeSidecarsReport {
            package_id: self.manifest.package.id.clone(),
            package_version: self.manifest.package.version.clone(),
            sidecars,
        })
    }

    /// Run a package test case through the normal OpenBMP runner.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for package, scenario, or runner failures.
    pub fn run_case(&self, case_id: Option<&str>) -> Result<SilRunReport, SilError> {
        let case = self.resolve_case(case_id)?;
        let scenario_path = self.resolve(&case.scenario_path);
        let scenario = Scenario::from_file(&scenario_path)?;
        let outcome = openbmp_runner::run(&scenario)?;
        self.report_from_outcome(&case.id, &scenario, &outcome)
    }

    /// Run a package test case after shortening the scenario stop time
    /// to `ticks * dt_s` after the scenario start.
    ///
    /// This is the native SIL "step N ticks" operation. It runs the
    /// same runner path as a full test but clamps the case duration.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for package, scenario, TOML, or runner failures.
    pub fn step_case(&self, case_id: Option<&str>, ticks: u64) -> Result<SilRunReport, SilError> {
        let case = self.resolve_case(case_id)?;
        let scenario_path = self.resolve(&case.scenario_path);
        let content = fs::read_to_string(&scenario_path).map_err(|source| SilError::Io {
            path: scenario_path.clone(),
            source,
        })?;
        let base = Scenario::from_file(&scenario_path)?;
        let target_stop_s = (base.document.time.start_s + base.document.time.dt_s * ticks as f64)
            .min(base.document.time.stop_s);
        let mut value: toml::Value =
            toml::from_str(&content).map_err(|source| SilError::PackageToml {
                path: scenario_path.clone(),
                source,
            })?;
        value["time"]["stop_s"] = toml::Value::Float(target_stop_s);
        let text = toml::to_string(&value)?;
        let source_dir = scenario_path.parent().map(Path::to_path_buf);
        let scenario = Scenario::from_toml_str_with_source_dir(&text, source_dir)?;
        let outcome = openbmp_runner::run(&scenario)?;
        self.report_from_outcome(&case.id, &scenario, &outcome)
    }

    /// Run a package test case until a time, mission event, or phase
    /// transition target is reached.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for unknown targets, package, scenario, TOML,
    /// or runner failures.
    pub fn run_case_until(
        &self,
        case_id: Option<&str>,
        target: RunUntilTarget,
    ) -> Result<SilRunReport, SilError> {
        let case = self.resolve_case(case_id)?;
        let scenario_path = self.resolve(&case.scenario_path);
        let content = fs::read_to_string(&scenario_path).map_err(|source| SilError::Io {
            path: scenario_path.clone(),
            source,
        })?;
        let base = Scenario::from_file(&scenario_path)?;
        let mut value: toml::Value =
            toml::from_str(&content).map_err(|source| SilError::PackageToml {
                path: scenario_path.clone(),
                source,
            })?;
        apply_until_target(&mut value, &base, &target)?;
        let text = toml::to_string(&value)?;
        let source_dir = scenario_path.parent().map(Path::to_path_buf);
        let scenario = Scenario::from_toml_str_with_source_dir(&text, source_dir)?;
        let outcome = openbmp_runner::run(&scenario)?;
        self.report_from_outcome(&case.id, &scenario, &outcome)
    }

    /// Run a package test case with in-memory SIL stimulation.
    ///
    /// Faults, parameter overrides, and command writes are applied to a
    /// transient scenario document before the normal parser/runner path.
    /// Package files and sidecars are not modified.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for invalid stimulation, package, scenario,
    /// TOML, or runner failures.
    pub fn run_case_with_stimulation(
        &self,
        case_id: Option<&str>,
        stimulation: &SilStimulation,
    ) -> Result<SilRunReport, SilError> {
        self.run_mutated_case(case_id, |scenario, _base| {
            apply_stimulation(scenario, stimulation)
        })
    }

    /// Step a package test case for `ticks` scheduler ticks with
    /// in-memory SIL stimulation.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for invalid stimulation, package, scenario,
    /// TOML, or runner failures.
    pub fn step_case_with_stimulation(
        &self,
        case_id: Option<&str>,
        ticks: u64,
        stimulation: &SilStimulation,
    ) -> Result<SilRunReport, SilError> {
        self.run_mutated_case(case_id, |scenario, base| {
            let target_stop_s = (base.document.time.start_s
                + base.document.time.dt_s * ticks as f64)
                .min(base.document.time.stop_s);
            scenario["time"]["stop_s"] = toml::Value::Float(target_stop_s);
            apply_stimulation(scenario, stimulation)
        })
    }

    fn validate_shape(&self) -> Result<(), SilError> {
        require_non_empty("package.id", &self.manifest.package.id)?;
        require_non_empty("package.version", &self.manifest.package.version)?;
        if self.manifest.files.scenario.as_os_str().is_empty() {
            return Err(SilError::Package {
                summary: "files.scenario is empty".to_owned(),
            });
        }
        let mut case_ids = std::collections::BTreeSet::new();
        for case in &self.manifest.test_cases {
            require_non_empty("test_cases.id", &case.id)?;
            if !case_ids.insert(case.id.as_str()) {
                return Err(SilError::Package {
                    summary: format!("duplicate test case id `{}`", case.id),
                });
            }
        }
        Ok(())
    }

    fn optional_files(&self) -> Vec<(&'static str, PathBuf)> {
        let files = &self.manifest.files;
        [
            ("files.plant_config", files.plant_config.as_ref()),
            ("files.iload", files.iload.as_ref()),
            ("files.mission_graph", files.mission_graph.as_ref()),
            ("files.scenario_script", files.scenario_script.as_ref()),
            ("files.dictionary", files.dictionary.as_ref()),
            ("files.provenance", files.provenance.as_ref()),
        ]
        .into_iter()
        .filter_map(|(field, path)| path.map(|path| (field, self.resolve(path))))
        .collect()
    }

    fn resolve(&self, path: &Path) -> PathBuf {
        if path.is_absolute() {
            path.to_path_buf()
        } else {
            self.root.join(path)
        }
    }

    fn validate_sidecars(&self, scenario_path: &Path) -> Result<(), SilError> {
        let scenario = read_toml_value(scenario_path)?;
        if let Some(path) = &self.manifest.files.plant_config {
            let sidecar_path = self.resolve(path);
            let expected = sidecar_from_keys(&scenario, PLANT_CONFIG_KEYS);
            validate_toml_sidecar("files.plant_config", &sidecar_path, &expected)?;
        }
        if let Some(path) = &self.manifest.files.mission_graph {
            let sidecar_path = self.resolve(path);
            let expected = sidecar_from_keys(&scenario, &["mission"]);
            validate_toml_sidecar("files.mission_graph", &sidecar_path, &expected)?;
        }
        if let Some(path) = &self.manifest.files.scenario_script {
            let sidecar_path = self.resolve(path);
            let expected = sidecar_from_keys(&scenario, &["scenario_script"]);
            validate_toml_sidecar("files.scenario_script", &sidecar_path, &expected)?;
        }
        if let Some(path) = &self.manifest.files.iload {
            let sidecar_path = self.resolve(path);
            let expected_payload = iload_payload_from_scenario(&scenario)?;
            let bytes = fs::read(&sidecar_path).map_err(|source| SilError::Io {
                path: sidecar_path.clone(),
                source,
            })?;
            let iload = decode_iload_envelope(&bytes, &[PACKAGE_ILOAD_PAYLOAD_SCHEMA_VERSION])
                .map_err(|source| SilError::Iload {
                    summary: source.to_string(),
                })?;
            let payload = std::str::from_utf8(iload.payload).map_err(|source| SilError::Iload {
                summary: format!("files.iload payload is not UTF-8 TOML: {source}"),
            })?;
            if payload != expected_payload {
                return Err(SilError::SidecarMismatch {
                    summary: format!(
                        "files.iload payload does not match scenario `[fc]` table in {}",
                        scenario_path.display()
                    ),
                });
            }
        }
        Ok(())
    }

    fn resolve_case(&self, case_id: Option<&str>) -> Result<ResolvedCase, SilError> {
        if let Some(case_id) = case_id {
            let Some(case) = self
                .manifest
                .test_cases
                .iter()
                .find(|case| case.id == case_id)
            else {
                return Err(SilError::Package {
                    summary: format!("unknown test case `{case_id}`"),
                });
            };
            return Ok(ResolvedCase {
                id: case.id.clone(),
                scenario_path: case
                    .scenario
                    .clone()
                    .unwrap_or_else(|| self.manifest.files.scenario.clone()),
            });
        }
        if self.manifest.test_cases.len() == 1 {
            let case = &self.manifest.test_cases[0];
            return Ok(ResolvedCase {
                id: case.id.clone(),
                scenario_path: case
                    .scenario
                    .clone()
                    .unwrap_or_else(|| self.manifest.files.scenario.clone()),
            });
        }
        Ok(ResolvedCase {
            id: "default".to_owned(),
            scenario_path: self.manifest.files.scenario.clone(),
        })
    }

    fn report_from_outcome(
        &self,
        case_id: &str,
        scenario: &Scenario,
        outcome: &RunOutcome,
    ) -> Result<SilRunReport, SilError> {
        self.report_from_outcome_with_stimuli(case_id, scenario, outcome, Vec::new())
    }

    fn report_from_outcome_with_stimuli(
        &self,
        case_id: &str,
        scenario: &Scenario,
        outcome: &RunOutcome,
        stimuli: Vec<StimulusRecord>,
    ) -> Result<SilRunReport, SilError> {
        let check = self.check()?;
        let telemetry_summary = telemetry_summary(&outcome.table);
        let stop_label = outcome.stop_reason.label().to_owned();
        let requirement_verdicts = requirement_verdicts(&outcome.table, &stop_label);
        let failures =
            evidence_failures(&requirement_verdicts, outcome.final_time_s, outcome.final_step);
        let verdict = aggregate_verdict(&failures).to_owned();
        let evidence = EvidenceBundle {
            schema: "openbmp.sil.evidence.v1".to_owned(),
            package_id: self.manifest.package.id.clone(),
            package_version: self.manifest.package.version.clone(),
            case_id: case_id.to_owned(),
            scenario_name: scenario.document.meta.name.clone(),
            validation_label: scenario.document.meta.validation.as_label().to_owned(),
            stop_label: stop_label.clone(),
            final_step: outcome.final_step,
            final_time_s: outcome.final_time_s,
            verdict,
            generated_unix_s: generated_unix_s(),
            git_commit: git_commit(),
            toolchain: rustc_toolchain(),
            target_triple: rustc_host_triple(),
            hashes: check.hashes,
            stimuli,
            event_trace: event_trace(&outcome.table),
            phase_trace: phase_trace(&outcome.table),
            region_trace: region_trace(&outcome.table),
            command_trace: command_trace(&outcome.table),
            bus_frames: bus_frames(&outcome.table),
            requirement_verdicts,
            failures,
            telemetry_rows: telemetry_summary.rows,
            telemetry_channels: telemetry_summary.channels,
            telemetry_summary,
        };
        Ok(SilRunReport {
            package_id: self.manifest.package.id.clone(),
            case_id: case_id.to_owned(),
            stop_label,
            final_step: outcome.final_step,
            final_time_s: outcome.final_time_s,
            evidence,
            telemetry: outcome.table.clone(),
        })
    }

    fn run_mutated_case(
        &self,
        case_id: Option<&str>,
        mutator: impl FnOnce(&mut toml::Value, &Scenario) -> Result<Vec<StimulusRecord>, SilError>,
    ) -> Result<SilRunReport, SilError> {
        let case = self.resolve_case(case_id)?;
        let scenario_path = self.resolve(&case.scenario_path);
        let content = fs::read_to_string(&scenario_path).map_err(|source| SilError::Io {
            path: scenario_path.clone(),
            source,
        })?;
        let base = Scenario::from_file(&scenario_path)?;
        let mut value: toml::Value =
            toml::from_str(&content).map_err(|source| SilError::PackageToml {
                path: scenario_path.clone(),
                source,
            })?;
        let stimuli = mutator(&mut value, &base)?;
        let text = toml::to_string(&value)?;
        let source_dir = scenario_path.parent().map(Path::to_path_buf);
        let scenario = Scenario::from_toml_str_with_source_dir(&text, source_dir)?;
        let outcome = openbmp_runner::run(&scenario)?;
        self.report_from_outcome_with_stimuli(&case.id, &scenario, &outcome, stimuli)
    }
}

impl SilTestbench {
    /// Load a testbench from a mission package manifest.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] when package loading fails.
    pub fn load(path: impl AsRef<Path>) -> Result<Self, SilError> {
        Ok(Self {
            package: MissionPackage::load(path)?,
        })
    }

    /// Reset the testbench state.
    ///
    /// The current native backend is stateless between runs, so reset
    /// validates the package and returns.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] when package validation fails.
    pub fn reset(&mut self) -> Result<(), SilError> {
        self.package.check()?;
        Ok(())
    }

    /// Step the selected case for `ticks` scheduler ticks.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for package, scenario, or runner failures.
    pub fn step(&self, case_id: Option<&str>, ticks: u64) -> Result<SilRunReport, SilError> {
        self.package.step_case(case_id, ticks)
    }

    /// Run the selected case to its declared stop condition.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for package, scenario, or runner failures.
    pub fn run(&self, case_id: Option<&str>) -> Result<SilRunReport, SilError> {
        self.package.run_case(case_id)
    }

    /// Run the selected case until a target condition is reached.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for package, scenario, target, or runner failures.
    pub fn run_until(
        &self,
        case_id: Option<&str>,
        target: RunUntilTarget,
    ) -> Result<SilRunReport, SilError> {
        self.package.run_case_until(case_id, target)
    }

    /// Run the selected case with in-memory SIL stimulation.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for stimulation, package, scenario, or runner
    /// failures.
    pub fn run_with_stimulation(
        &self,
        case_id: Option<&str>,
        stimulation: &SilStimulation,
    ) -> Result<SilRunReport, SilError> {
        self.package.run_case_with_stimulation(case_id, stimulation)
    }

    /// Step the selected case with in-memory SIL stimulation.
    ///
    /// # Errors
    ///
    /// Returns [`SilError`] for stimulation, package, scenario, or runner
    /// failures.
    pub fn step_with_stimulation(
        &self,
        case_id: Option<&str>,
        ticks: u64,
        stimulation: &SilStimulation,
    ) -> Result<SilRunReport, SilError> {
        self.package
            .step_case_with_stimulation(case_id, ticks, stimulation)
    }

    /// Borrow the loaded package.
    #[must_use]
    pub const fn package(&self) -> &MissionPackage {
        &self.package
    }
}

/// Write an evidence bundle directory containing `manifest.json`.
///
/// # Errors
///
/// Returns [`SilError`] for directory creation or JSON failures.
pub fn write_evidence_bundle(
    directory: impl AsRef<Path>,
    evidence: &EvidenceBundle,
) -> Result<PathBuf, SilError> {
    let directory = directory.as_ref();
    fs::create_dir_all(directory).map_err(|source| SilError::Io {
        path: directory.to_path_buf(),
        source,
    })?;
    let path = directory.join("manifest.json");
    let json = serde_json::to_string_pretty(evidence).map_err(|source| SilError::EvidenceJson {
        path: path.clone(),
        source,
    })?;
    fs::write(&path, json).map_err(|source| SilError::Io {
        path: path.clone(),
        source,
    })?;
    Ok(path)
}

/// Read one telemetry channel from a completed SIL run.
///
/// # Errors
///
/// Returns [`SilError::TelemetryChannel`] when the channel is absent.
pub fn read_signal(
    report: &SilRunReport,
    channel: &str,
    max_preview: usize,
) -> Result<SignalReadReport, SilError> {
    read_signal_from_table(&report.telemetry, channel, max_preview)
}

/// Capture host-SIL bus/evidence frames from a completed SIL run.
#[must_use]
pub fn capture_bus_frames(report: &SilRunReport) -> Vec<BusFrameEntry> {
    report.evidence.bus_frames.clone()
}

/// Compute a lowercase SHA-256 digest for a file.
///
/// # Errors
///
/// Returns [`SilError::Io`] when the file cannot be read.
pub fn sha256_file(path: impl AsRef<Path>) -> Result<String, SilError> {
    let path = path.as_ref();
    let bytes = fs::read(path).map_err(|source| SilError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    Ok(format!("{:x}", hasher.finalize()))
}

#[derive(Clone, Debug)]
struct ResolvedCase {
    id: String,
    scenario_path: PathBuf,
}

const PLANT_CONFIG_KEYS: &[&str] = &[
    "vehicle",
    "environment",
    "forces",
    "epoch",
    "frames",
    "aero",
    "propulsion",
    "wind",
    "atmosphere",
    "solver",
    "data_packages",
    "sensors",
    "faults",
    "batch",
    "landing_footprint",
    "staging_analysis",
    "entry_profile",
    "aerothermal",
    "schedule",
    "multi_body",
];

fn require_non_empty(field: &str, value: &str) -> Result<(), SilError> {
    if value.trim().is_empty() {
        return Err(SilError::Package {
            summary: format!("{field} is empty"),
        });
    }
    Ok(())
}

fn read_toml_value(path: &Path) -> Result<toml::Value, SilError> {
    let text = fs::read_to_string(path).map_err(|source| SilError::Io {
        path: path.to_path_buf(),
        source,
    })?;
    toml::from_str(&text).map_err(|source| SilError::PackageToml {
        path: path.to_path_buf(),
        source,
    })
}

fn sidecar_from_keys(source: &toml::Value, keys: &[&str]) -> toml::Value {
    let mut table = toml::map::Map::new();
    let Some(source_table) = source.as_table() else {
        return toml::Value::Table(table);
    };
    for key in keys {
        if let Some(value) = source_table.get(*key) {
            table.insert((*key).to_owned(), value.clone());
        }
    }
    toml::Value::Table(table)
}

fn iload_payload_from_scenario(scenario: &toml::Value) -> Result<String, SilError> {
    let value = sidecar_from_keys(scenario, &["fc"]);
    toml::to_string_pretty(&value).map_err(SilError::ScenarioTomlSerialize)
}

fn write_toml_sidecar(path: &Path, value: &toml::Value) -> Result<(), SilError> {
    let text = toml::to_string_pretty(value).map_err(SilError::ScenarioTomlSerialize)?;
    write_bytes_sidecar(path, text.as_bytes())
}

fn write_bytes_sidecar(path: &Path, bytes: &[u8]) -> Result<(), SilError> {
    if let Some(parent) = path.parent() {
        fs::create_dir_all(parent).map_err(|source| SilError::Io {
            path: parent.to_path_buf(),
            source,
        })?;
    }
    fs::write(path, bytes).map_err(|source| SilError::Io {
        path: path.to_path_buf(),
        source,
    })
}

fn validate_toml_sidecar(
    field: &str,
    sidecar_path: &Path,
    expected: &toml::Value,
) -> Result<(), SilError> {
    let actual = read_toml_value(sidecar_path)?;
    if &actual != expected {
        return Err(SilError::SidecarMismatch {
            summary: format!(
                "{field} ({}) does not match the scenario source table",
                sidecar_path.display()
            ),
        });
    }
    Ok(())
}

fn apply_stimulation(
    scenario: &mut toml::Value,
    stimulation: &SilStimulation,
) -> Result<Vec<StimulusRecord>, SilError> {
    let mut records = Vec::new();
    for fault in &stimulation.faults {
        records.push(apply_fault_injection(scenario, fault)?);
    }
    for override_ in &stimulation.parameter_overrides {
        set_toml_path(scenario, &override_.path, override_.value.clone())?;
        records.push(StimulusRecord {
            kind: "parameter_override".to_owned(),
            target: override_.path.clone(),
            summary: format!(
                "set {} = {}",
                override_.path,
                telemetry_value_safe_toml(&override_.value)
            ),
        });
    }
    for (index, command) in stimulation.command_writes.iter().enumerate() {
        records.push(append_command_write(scenario, index, command)?);
    }
    Ok(records)
}

fn apply_fault_injection(
    scenario: &mut toml::Value,
    fault: &FaultInjection,
) -> Result<StimulusRecord, SilError> {
    let (array_key, id, target) = match &fault.target {
        FaultTarget::Engine(id) => ("engines", id.as_str(), format!("engine.{id}")),
        FaultTarget::Effector(id) => ("effectors", id.as_str(), format!("effector.{id}")),
    };
    let entries = scenario
        .get_mut("vehicle")
        .and_then(|vehicle| vehicle.get_mut("assembly"))
        .and_then(|assembly| assembly.get_mut(array_key))
        .and_then(toml::Value::as_array_mut)
        .ok_or_else(|| SilError::Stimulation {
            summary: format!("vehicle.assembly.{array_key} is absent"),
        })?;
    let Some(entry) = entries
        .iter_mut()
        .find(|entry| entry.get("id").and_then(toml::Value::as_str) == Some(id))
    else {
        return Err(SilError::Stimulation {
            summary: format!("fault target `{target}` was not found"),
        });
    };
    let Some(table) = entry.as_table_mut() else {
        return Err(SilError::Stimulation {
            summary: format!("fault target `{target}` is not a table"),
        });
    };
    table.insert("fault".to_owned(), fault.fault.clone());
    Ok(StimulusRecord {
        kind: "fault_injection".to_owned(),
        target,
        summary: format!("fault {}", telemetry_value_safe_toml(&fault.fault)),
    })
}

fn append_command_write(
    scenario: &mut toml::Value,
    index: usize,
    command: &CommandWrite,
) -> Result<StimulusRecord, SilError> {
    let mut trigger = toml::map::Map::new();
    trigger.insert("kind".to_owned(), toml::Value::String("at_time".to_owned()));
    trigger.insert(
        "time_s".to_owned(),
        toml::Value::Float(command_time(command)?),
    );

    let (id, target, action, summary) = match command {
        CommandWrite::Engine {
            id,
            throttle_unit,
            gimbal_pitch_rad,
            gimbal_yaw_rad,
            ignite,
            shutdown,
            ..
        } => {
            let mut payload = toml::map::Map::new();
            payload.insert(
                "throttle_unit".to_owned(),
                toml::Value::Float(*throttle_unit),
            );
            payload.insert(
                "gimbal_pitch_rad".to_owned(),
                toml::Value::Float(*gimbal_pitch_rad),
            );
            payload.insert(
                "gimbal_yaw_rad".to_owned(),
                toml::Value::Float(*gimbal_yaw_rad),
            );
            payload.insert("ignite".to_owned(), toml::Value::Boolean(*ignite));
            payload.insert("shutdown".to_owned(), toml::Value::Boolean(*shutdown));

            let mut action = toml::map::Map::new();
            action.insert(
                "kind".to_owned(),
                toml::Value::String("engine_command".to_owned()),
            );
            action.insert("id".to_owned(), toml::Value::String(id.clone()));
            action.insert("command".to_owned(), toml::Value::Table(payload));
            (
                format!("sil_command_engine_{}_{}", sanitize_id(id), index),
                format!("engine.{id}"),
                action,
                format!(
                    "engine command throttle={:.6} ignite={} shutdown={}",
                    throttle_unit, ignite, shutdown
                ),
            )
        }
        CommandWrite::Effector { id, command, .. } => {
            let mut action = toml::map::Map::new();
            action.insert(
                "kind".to_owned(),
                toml::Value::String("effector_override".to_owned()),
            );
            action.insert("id".to_owned(), toml::Value::String(id.clone()));
            action.insert("command".to_owned(), toml::Value::Float(*command));
            (
                format!("sil_command_effector_{}_{}", sanitize_id(id), index),
                format!("effector.{id}"),
                action,
                format!("effector command {command:.6}"),
            )
        }
    };

    let mut event = toml::map::Map::new();
    event.insert("id".to_owned(), toml::Value::String(id));
    event.insert("once".to_owned(), toml::Value::Boolean(true));
    event.insert("trigger".to_owned(), toml::Value::Table(trigger));
    event.insert("action".to_owned(), toml::Value::Table(action));
    scenario_script_events_mut(scenario)?.push(toml::Value::Table(event));
    Ok(StimulusRecord {
        kind: "command_write".to_owned(),
        target,
        summary,
    })
}

fn command_time(command: &CommandWrite) -> Result<f64, SilError> {
    let time_s = match command {
        CommandWrite::Engine { time_s, .. } | CommandWrite::Effector { time_s, .. } => *time_s,
    };
    if !time_s.is_finite() {
        return Err(SilError::Stimulation {
            summary: "command write time_s must be finite".to_owned(),
        });
    }
    Ok(time_s)
}

fn scenario_script_events_mut(
    scenario: &mut toml::Value,
) -> Result<&mut Vec<toml::Value>, SilError> {
    let Some(root) = scenario.as_table_mut() else {
        return Err(SilError::Stimulation {
            summary: "scenario root is not a TOML table".to_owned(),
        });
    };
    if !root.contains_key("scenario_script") {
        root.insert(
            "scenario_script".to_owned(),
            toml::Value::Table(toml::map::Map::new()),
        );
    }
    let script = root
        .get_mut("scenario_script")
        .and_then(toml::Value::as_table_mut)
        .ok_or_else(|| SilError::Stimulation {
            summary: "scenario_script is not a table".to_owned(),
        })?;
    if !script.contains_key("events") {
        script.insert("events".to_owned(), toml::Value::Array(Vec::new()));
    }
    script
        .get_mut("events")
        .and_then(toml::Value::as_array_mut)
        .ok_or_else(|| SilError::Stimulation {
            summary: "scenario_script.events is not an array".to_owned(),
        })
}

fn set_toml_path(root: &mut toml::Value, path: &str, value: toml::Value) -> Result<(), SilError> {
    let segments = parse_toml_path(path)?;
    let Some((last, parents)) = segments.split_last() else {
        return Err(SilError::Stimulation {
            summary: "override path is empty".to_owned(),
        });
    };
    let mut current = root;
    for segment in parents {
        current = descend_toml_path(current, segment)?;
    }
    set_toml_segment(current, last, value)
}

#[derive(Clone, Debug)]
struct TomlPathSegment {
    key: String,
    index: Option<usize>,
}

fn parse_toml_path(path: &str) -> Result<Vec<TomlPathSegment>, SilError> {
    let mut segments = Vec::new();
    for raw in path.split('.') {
        if raw.is_empty() {
            return Err(SilError::Stimulation {
                summary: format!("invalid empty segment in override path `{path}`"),
            });
        }
        if let Some(start) = raw.find('[') {
            let Some(end) = raw.strip_suffix(']').and_then(|_| raw.rfind(']')) else {
                return Err(SilError::Stimulation {
                    summary: format!("invalid array segment `{raw}` in override path `{path}`"),
                });
            };
            if end <= start + 1 {
                return Err(SilError::Stimulation {
                    summary: format!("empty array index in override path `{path}`"),
                });
            }
            let key = &raw[..start];
            if key.is_empty() {
                return Err(SilError::Stimulation {
                    summary: format!("empty array key in override path `{path}`"),
                });
            }
            let index =
                raw[start + 1..end]
                    .parse::<usize>()
                    .map_err(|source| SilError::Stimulation {
                        summary: format!("invalid array index in override path `{path}`: {source}"),
                    })?;
            segments.push(TomlPathSegment {
                key: key.to_owned(),
                index: Some(index),
            });
        } else {
            segments.push(TomlPathSegment {
                key: raw.to_owned(),
                index: None,
            });
        }
    }
    Ok(segments)
}

fn descend_toml_path<'a>(
    value: &'a mut toml::Value,
    segment: &TomlPathSegment,
) -> Result<&'a mut toml::Value, SilError> {
    let table = value.as_table_mut().ok_or_else(|| SilError::Stimulation {
        summary: format!("path segment `{}` does not address a table", segment.key),
    })?;
    let child = table
        .get_mut(&segment.key)
        .ok_or_else(|| SilError::Stimulation {
            summary: format!("path segment `{}` was not found", segment.key),
        })?;
    if let Some(index) = segment.index {
        child
            .as_array_mut()
            .and_then(|array| array.get_mut(index))
            .ok_or_else(|| SilError::Stimulation {
                summary: format!("path segment `{}` index {index} was not found", segment.key),
            })
    } else {
        Ok(child)
    }
}

fn set_toml_segment(
    value: &mut toml::Value,
    segment: &TomlPathSegment,
    replacement: toml::Value,
) -> Result<(), SilError> {
    let table = value.as_table_mut().ok_or_else(|| SilError::Stimulation {
        summary: format!("path segment `{}` does not address a table", segment.key),
    })?;
    if let Some(index) = segment.index {
        let child = table
            .get_mut(&segment.key)
            .ok_or_else(|| SilError::Stimulation {
                summary: format!("path segment `{}` was not found", segment.key),
            })?;
        let array = child.as_array_mut().ok_or_else(|| SilError::Stimulation {
            summary: format!("path segment `{}` is not an array", segment.key),
        })?;
        let Some(slot) = array.get_mut(index) else {
            return Err(SilError::Stimulation {
                summary: format!("path segment `{}` index {index} was not found", segment.key),
            });
        };
        *slot = replacement;
    } else {
        table.insert(segment.key.clone(), replacement);
    }
    Ok(())
}

fn telemetry_value_safe_toml(value: &toml::Value) -> String {
    toml::to_string(value)
        .unwrap_or_else(|_| format!("{value:?}"))
        .trim()
        .to_owned()
}

fn materialized(field: &str, path: &Path) -> Result<MaterializedSidecar, SilError> {
    Ok(MaterializedSidecar {
        field: field.to_owned(),
        path: path.to_path_buf(),
        sha256: sha256_file(path)?,
    })
}

fn apply_until_target(
    scenario: &mut toml::Value,
    base: &Scenario,
    target: &RunUntilTarget,
) -> Result<(), SilError> {
    match target {
        RunUntilTarget::TimeS(time_s) => {
            if !time_s.is_finite() {
                return Err(SilError::RunUntil {
                    summary: "time target must be finite".to_owned(),
                });
            }
            let target_stop_s = time_s
                .max(base.document.time.start_s)
                .min(base.document.time.stop_s);
            scenario["time"]["stop_s"] = toml::Value::Float(target_stop_s);
            Ok(())
        }
        RunUntilTarget::Event(event_id) => append_stop_event_for_event(scenario, event_id),
        RunUntilTarget::Phase(phase) => append_stop_event_for_phase(scenario, phase, base),
    }
}

fn append_stop_event_for_event(scenario: &mut toml::Value, event_id: &str) -> Result<(), SilError> {
    let Some(event) = find_mission_event(scenario, event_id).cloned() else {
        return Err(SilError::RunUntil {
            summary: format!("mission event `{event_id}` was not found"),
        });
    };
    let trigger = event
        .get("trigger")
        .cloned()
        .ok_or_else(|| SilError::RunUntil {
            summary: format!("mission event `{event_id}` has no trigger"),
        })?;
    append_stop_event(
        scenario,
        &format!("sil_until_event_{}", sanitize_id(event_id)),
        &format!("sil.until.event.{event_id}"),
        trigger,
    )
}

fn append_stop_event_for_phase(
    scenario: &mut toml::Value,
    phase: &str,
    base: &Scenario,
) -> Result<(), SilError> {
    let target = phase_path(phase);
    if base
        .document
        .mission
        .as_ref()
        .is_some_and(|mission| mission.initial_phase == target || mission.initial_phase == phase)
    {
        scenario["time"]["stop_s"] = toml::Value::Float(base.document.time.start_s);
        return Ok(());
    }
    let event_id = transition_event_for_phase(scenario, &target, phase)?;
    let Some(event) = find_mission_event(scenario, &event_id).cloned() else {
        return Err(SilError::RunUntil {
            summary: format!(
                "transition into phase `{phase}` uses event `{event_id}`, but that event was not found"
            ),
        });
    };
    let trigger = event
        .get("trigger")
        .cloned()
        .ok_or_else(|| SilError::RunUntil {
            summary: format!("mission event `{event_id}` has no trigger"),
        })?;
    append_stop_event(
        scenario,
        &format!("sil_until_phase_{}", sanitize_id(phase)),
        &format!("sil.until.phase.{phase}"),
        trigger,
    )
}

fn find_mission_event<'a>(scenario: &'a toml::Value, event_id: &str) -> Option<&'a toml::Value> {
    mission_array(scenario, "events")?
        .iter()
        .find(|event| event.get("id").and_then(toml::Value::as_str) == Some(event_id))
}

fn transition_event_for_phase(
    scenario: &toml::Value,
    target: &str,
    original: &str,
) -> Result<String, SilError> {
    let Some(transitions) = mission_array(scenario, "transitions") else {
        return Err(SilError::RunUntil {
            summary: "mission has no transitions".to_owned(),
        });
    };
    for transition in transitions {
        let Some(to) = transition.get("to").and_then(toml::Value::as_str) else {
            continue;
        };
        if to != target && to != original {
            continue;
        }
        let Some(event) = transition.get("event").and_then(toml::Value::as_str) else {
            return Err(SilError::RunUntil {
                summary: format!("transition into `{to}` has no event field"),
            });
        };
        return Ok(event.to_owned());
    }
    Err(SilError::RunUntil {
        summary: format!("no transition into mission phase `{original}`"),
    })
}

fn mission_array<'a>(scenario: &'a toml::Value, key: &str) -> Option<&'a Vec<toml::Value>> {
    scenario.get("mission")?.get(key)?.as_array()
}

fn mission_array_mut<'a>(
    scenario: &'a mut toml::Value,
    key: &str,
) -> Result<&'a mut Vec<toml::Value>, SilError> {
    scenario
        .get_mut("mission")
        .and_then(|mission| mission.get_mut(key))
        .and_then(toml::Value::as_array_mut)
        .ok_or_else(|| SilError::RunUntil {
            summary: format!("mission has no `{key}` array"),
        })
}

fn append_stop_event(
    scenario: &mut toml::Value,
    id: &str,
    label: &str,
    trigger: toml::Value,
) -> Result<(), SilError> {
    let mut action = toml::map::Map::new();
    action.insert("kind".to_owned(), toml::Value::String("stop".to_owned()));
    action.insert("label".to_owned(), toml::Value::String(label.to_owned()));

    let mut event = toml::map::Map::new();
    event.insert("id".to_owned(), toml::Value::String(id.to_owned()));
    event.insert("once".to_owned(), toml::Value::Boolean(true));
    event.insert("trigger".to_owned(), trigger);
    event.insert("action".to_owned(), toml::Value::Table(action));

    mission_array_mut(scenario, "events")?.push(toml::Value::Table(event));
    Ok(())
}

fn phase_path(phase: &str) -> String {
    if phase.starts_with("mission.phases.") || phase.starts_with("mission.states.") {
        phase.to_owned()
    } else {
        format!("mission.phases.{phase}")
    }
}

fn sanitize_id(value: &str) -> String {
    value
        .chars()
        .map(|ch| {
            if ch.is_ascii_alphanumeric() || ch == '_' || ch == '-' {
                ch
            } else {
                '_'
            }
        })
        .collect()
}

fn event_trace(table: &TelemetryTable) -> Vec<EventTraceEntry> {
    let marker_channels: Vec<_> = table
        .schema()
        .channels()
        .iter()
        .filter_map(|channel| {
            channel
                .name
                .strip_prefix("mission.marker.")
                .map(|tag| (channel.id, tag.to_owned()))
        })
        .collect();
    let mut trace = Vec::new();
    for row in table.rows() {
        for (channel_id, tag) in &marker_channels {
            if matches!(row.get(*channel_id), Some(TelemetryValue::Bool(true))) {
                trace.push(EventTraceEntry {
                    tag: tag.clone(),
                    time_s: row.time.as_seconds(),
                    step: row.step.value(),
                });
            }
        }
    }
    trace
}

fn phase_trace(table: &TelemetryTable) -> Vec<PhaseTraceEntry> {
    let Some(channel) = table
        .schema()
        .channels()
        .iter()
        .find(|channel| channel.name == "mission.phase")
    else {
        return Vec::new();
    };
    let mut trace = Vec::new();
    let mut last_phase: Option<String> = None;
    for row in table.rows() {
        let Some(TelemetryValue::Text(phase)) = row.get(channel.id) else {
            continue;
        };
        if last_phase.as_deref() == Some(phase.as_str()) {
            continue;
        }
        last_phase = Some(phase.clone());
        trace.push(PhaseTraceEntry {
            phase: phase.clone(),
            time_s: row.time.as_seconds(),
            step: row.step.value(),
        });
    }
    trace
}

fn region_trace(table: &TelemetryTable) -> Vec<RegionTraceEntry> {
    let region_channels: Vec<_> = table
        .schema()
        .channels()
        .iter()
        .filter_map(|channel| {
            channel
                .name
                .strip_prefix("mission.region.")
                .map(|region| (channel.id, region.to_owned()))
        })
        .collect();
    let mut trace = Vec::new();
    let mut last_states = BTreeMap::new();
    for row in table.rows() {
        for (channel_id, region) in &region_channels {
            let Some(TelemetryValue::Text(state)) = row.get(*channel_id) else {
                continue;
            };
            let key = *channel_id;
            if last_states.get(&key).is_some_and(|last| last == state) {
                continue;
            }
            last_states.insert(key, state.clone());
            trace.push(RegionTraceEntry {
                region: region.clone(),
                state: state.clone(),
                time_s: row.time.as_seconds(),
                step: row.step.value(),
            });
        }
    }
    trace
}

fn command_trace(table: &TelemetryTable) -> Vec<CommandTraceEntry> {
    let thrust_channels: Vec<_> = table
        .schema()
        .channels()
        .iter()
        .filter(|channel| {
            channel.name.starts_with("engine.") && channel.name.ends_with(".thrust_n")
        })
        .map(|channel| (channel.id, channel.name.clone()))
        .collect();
    let mut trace = Vec::new();
    let mut active = BTreeMap::new();
    for row in table.rows() {
        for (channel_id, channel_name) in &thrust_channels {
            let Some(TelemetryValue::Float64(value)) = row.get(*channel_id) else {
                continue;
            };
            let now_active = *value > 1.0;
            let previous = active.insert(*channel_id, now_active);
            if previous == Some(now_active) || (previous.is_none() && !now_active) {
                continue;
            }
            trace.push(CommandTraceEntry {
                channel: channel_name.clone(),
                state: if now_active { "active" } else { "inactive" }.to_owned(),
                time_s: row.time.as_seconds(),
                step: row.step.value(),
                value: *value,
            });
        }
    }
    trace
}

fn bus_frames(table: &TelemetryTable) -> Vec<BusFrameEntry> {
    let mut frames = Vec::new();
    frames.extend(event_trace(table).into_iter().map(|entry| BusFrameEntry {
        stream: "mission.event".to_owned(),
        subject: entry.tag,
        value: "fired".to_owned(),
        time_s: entry.time_s,
        step: entry.step,
    }));
    frames.extend(phase_trace(table).into_iter().map(|entry| BusFrameEntry {
        stream: "mission.phase".to_owned(),
        subject: "mission".to_owned(),
        value: entry.phase,
        time_s: entry.time_s,
        step: entry.step,
    }));
    frames.extend(region_trace(table).into_iter().map(|entry| BusFrameEntry {
        stream: "mission.region".to_owned(),
        subject: entry.region,
        value: entry.state,
        time_s: entry.time_s,
        step: entry.step,
    }));
    frames.extend(command_trace(table).into_iter().map(|entry| BusFrameEntry {
        stream: "actuator.command".to_owned(),
        subject: entry.channel,
        value: entry.state,
        time_s: entry.time_s,
        step: entry.step,
    }));
    frames.sort_by(|left, right| {
        left.step
            .cmp(&right.step)
            .then_with(|| left.time_s.total_cmp(&right.time_s))
            .then_with(|| left.stream.cmp(&right.stream))
            .then_with(|| left.subject.cmp(&right.subject))
            .then_with(|| left.value.cmp(&right.value))
    });
    frames
}

fn read_signal_from_table(
    table: &TelemetryTable,
    channel: &str,
    max_preview: usize,
) -> Result<SignalReadReport, SilError> {
    let Some(metadata) = table
        .schema()
        .channels()
        .iter()
        .find(|metadata| metadata.name == channel)
    else {
        return Err(SilError::TelemetryChannel {
            channel: channel.to_owned(),
        });
    };
    let mut first = None;
    let mut last = None;
    let mut preview = Vec::new();
    let mut samples = 0_usize;
    let mut min_float: Option<f64> = None;
    let mut max_float: Option<f64> = None;
    let mut min_int: Option<i64> = None;
    let mut max_int: Option<i64> = None;

    for row in table.rows() {
        let Some(value) = row.get(metadata.id) else {
            continue;
        };
        samples += 1;
        match value {
            TelemetryValue::Float64(value) => {
                min_float = Some(min_float.map_or(*value, |current| current.min(*value)));
                max_float = Some(max_float.map_or(*value, |current| current.max(*value)));
            }
            TelemetryValue::Int64(value) => {
                min_int = Some(min_int.map_or(*value, |current| current.min(*value)));
                max_int = Some(max_int.map_or(*value, |current| current.max(*value)));
            }
            TelemetryValue::Bool(_) | TelemetryValue::Text(_) => {}
        }
        let sample = SignalSample {
            time_s: row.time.as_seconds(),
            step: row.step.value(),
            value: telemetry_value_string(value),
        };
        if first.is_none() {
            first = Some(sample.clone());
        }
        last = Some(sample.clone());
        if preview.len() < max_preview {
            preview.push(sample);
        }
    }

    let (min, max) = match metadata.value_kind {
        TelemetryValueKind::Float64 => (min_float.map(format_f64), max_float.map(format_f64)),
        TelemetryValueKind::Int64 => (
            min_int.map(|v| v.to_string()),
            max_int.map(|v| v.to_string()),
        ),
        TelemetryValueKind::Bool | TelemetryValueKind::Text => (None, None),
    };

    Ok(SignalReadReport {
        channel: metadata.name.clone(),
        kind: metadata.value_kind.to_string(),
        unit: metadata.unit.clone(),
        frame: metadata.frame.clone(),
        samples,
        first,
        last,
        min,
        max,
        preview,
    })
}

fn telemetry_value_string(value: &TelemetryValue) -> String {
    match value {
        TelemetryValue::Float64(value) => format_f64(*value),
        TelemetryValue::Int64(value) => value.to_string(),
        TelemetryValue::Bool(value) => value.to_string(),
        TelemetryValue::Text(value) => value.clone(),
    }
}

fn format_f64(value: f64) -> String {
    format!("{value:.17e}")
}

fn telemetry_summary(table: &TelemetryTable) -> TelemetrySummary {
    TelemetrySummary {
        rows: table.rows().len(),
        channels: table.schema().channels().len(),
        first_time_s: table.rows().first().map(|row| row.time.as_seconds()),
        last_time_s: table.rows().last().map(|row| row.time.as_seconds()),
        channel_names: table
            .schema()
            .channels()
            .iter()
            .map(|channel| channel.name.clone())
            .collect(),
    }
}

/// Runner stop labels that denote a non-nominal / abnormal termination.
///
/// A SIL run that ends on one of these did NOT reach a clean,
/// scenario-declared terminal: the vehicle hit the ground while a stop
/// guard was armed, the integrator diverged, the step counter overflowed,
/// or the run was truncated by the max-step guard. Every other label —
/// `end-time`, a scenario `[mission]` stop label, or a SIL `run-until`
/// stop — is treated as a nominal completion. New abnormal stop reasons
/// must be listed here so the SIL verdict stays honest; an unrecognised
/// label is treated as nominal.
const NON_NOMINAL_STOP_LABELS: [&str; 4] = [
    "ground-impact",
    "non-finite-state",
    "step-overflow",
    "max-steps",
];

/// Returns `true` when a runner stop label denotes a clean, expected
/// termination rather than a divergence, ground impact, or guard-rail
/// truncation.
fn stop_label_is_nominal(label: &str) -> bool {
    !NON_NOMINAL_STOP_LABELS.contains(&label)
}

/// Builds the failure list for an evidence bundle: every requirement
/// verdict with a `fail` status becomes one [`EvidenceFailure`]. The
/// list is empty exactly when the run passed, so it drives the top-level
/// [`EvidenceBundle::verdict`] via [`aggregate_verdict`].
fn evidence_failures(
    requirement_verdicts: &[RequirementVerdict],
    final_time_s: f64,
    final_step: u64,
) -> Vec<EvidenceFailure> {
    requirement_verdicts
        .iter()
        .filter(|verdict| verdict.verdict == "fail")
        .map(|verdict| EvidenceFailure {
            kind: "requirement-failed".to_owned(),
            summary: format!("{}: {}", verdict.id, verdict.summary),
            channel: None,
            time_s: Some(final_time_s),
            step: Some(final_step),
        })
        .collect()
}

/// Aggregates the top-level run verdict: `pass` only when there are no
/// failures, otherwise `fail`.
fn aggregate_verdict(failures: &[EvidenceFailure]) -> &'static str {
    if failures.is_empty() { "pass" } else { "fail" }
}

fn requirement_verdicts(table: &TelemetryTable, stop_label: &str) -> Vec<RequirementVerdict> {
    let events = event_trace(table);
    let phases = phase_trace(table);
    let regions = region_trace(table);
    let commands = command_trace(table);
    let frames = bus_frames(table);
    vec![
        RequirementVerdict {
            id: "SIL-RUN-COMPLETE".to_owned(),
            verdict: if table.rows().is_empty() {
                "fail"
            } else {
                "pass"
            }
            .to_owned(),
            summary: format!("telemetry rows recorded: {}", table.rows().len()),
        },
        RequirementVerdict {
            id: "SIL-NOMINAL-TERMINATION".to_owned(),
            verdict: if stop_label_is_nominal(stop_label) {
                "pass"
            } else {
                "fail"
            }
            .to_owned(),
            summary: format!("run stop reason: {stop_label}"),
        },
        RequirementVerdict {
            id: "SIL-EVENT-TRACE".to_owned(),
            verdict: if events.is_empty() { "skip" } else { "pass" }.to_owned(),
            summary: format!("mission marker events recorded: {}", events.len()),
        },
        RequirementVerdict {
            id: "SIL-PHASE-TRACE".to_owned(),
            verdict: if phases.is_empty() { "skip" } else { "pass" }.to_owned(),
            summary: format!("mission phase changes recorded: {}", phases.len()),
        },
        RequirementVerdict {
            id: "SIL-REGION-TRACE".to_owned(),
            verdict: if regions.is_empty() { "skip" } else { "pass" }.to_owned(),
            summary: format!("mission region changes recorded: {}", regions.len()),
        },
        RequirementVerdict {
            id: "SIL-COMMAND-TRACE".to_owned(),
            verdict: if commands.is_empty() { "skip" } else { "pass" }.to_owned(),
            summary: format!("engine command-state changes recorded: {}", commands.len()),
        },
        RequirementVerdict {
            id: "SIL-BUS-CAPTURE".to_owned(),
            verdict: if frames.is_empty() { "skip" } else { "pass" }.to_owned(),
            summary: format!("host SIL bus frames captured: {}", frames.len()),
        },
    ]
}

fn generated_unix_s() -> u64 {
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map_or(0, |duration| duration.as_secs())
}

fn git_commit() -> Option<String> {
    let text = command_stdout("git", &["rev-parse", "--verify", "HEAD"])?;
    let commit = text.trim();
    if commit.is_empty() {
        None
    } else {
        Some(commit.to_owned())
    }
}

fn rustc_toolchain() -> Option<String> {
    command_stdout("rustc", &["--version"]).map(|text| text.trim().to_owned())
}

fn rustc_host_triple() -> Option<String> {
    let text = command_stdout("rustc", &["-vV"])?;
    text.lines()
        .find_map(|line| line.strip_prefix("host: "))
        .map(str::to_owned)
}

fn command_stdout(program: &str, args: &[&str]) -> Option<String> {
    let output = Command::new(program).args(args).output().ok()?;
    if !output.status.success() {
        return None;
    }
    String::from_utf8(output.stdout).ok()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    #[test]
    fn package_check_validates_scenario_and_hashes() {
        let dir = tempfile::tempdir().expect("tempdir");
        let scenario = dir.path().join("scenario.toml");
        fs::write(
            &scenario,
            r#"
openbmp.scenario = 3

[meta]
name = "sil-package-fixture"
description = "fixture"
validation = "experimental"

[time]
start_s = 0.0
stop_s = 0.03
dt_s = 0.01
seed = 1

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "body"

[[vehicle.assembly.bodies]]
id = "body"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0

[[vehicle.assembly.effectors]]
id = "delta"
kind = { kind = "linear_actuator" }
limits = { min = -1.0, max = 1.0, max_rate_per_s = 100.0, deadband = 0.0, latency_s = 0.0 }
initial_position = 0.0
unit = "rad"

[environment]
frame_profile = "toy-fixed-earth"
gravity = "constant"
gravity_m_s2 = 9.80665
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/sil.csv"

[validation]
require_finite_state = true
require_monotonic_time = true

[mission]
initial_phase = "mission.phases.pad"

[[mission.phases]]
id = "mission.phases.pad"
label = "pad"

[[mission.phases]]
id = "mission.phases.coast"
label = "coast"

[[mission.events]]
id = "mission.events.marker"
trigger = { kind = "at_time", time_s = 0.01 }
action = { kind = "emit_telemetry_marker", tag = "sil.fixture" }

[[mission.events]]
id = "mission.events.coast"
trigger = { kind = "at_time", time_s = 0.02 }
action = { kind = "emit_telemetry_marker", tag = "sil.coast" }

[[mission.transitions]]
from = "mission.phases.pad"
to = "mission.phases.coast"
event = "mission.events.coast"
"#,
        )
        .expect("write scenario");
        let manifest = dir.path().join("mission-package.toml");
        fs::write(
            &manifest,
            r#"
[package]
id = "pkg"
version = "0.1.0"

[files]
scenario = "scenario.toml"
plant_config = "plant-config.toml"
iload = "fc.iload"
mission_graph = "mission-graph.toml"
scenario_script = "scenario-script.toml"

[[test_cases]]
id = "smoke"
"#,
        )
        .expect("write manifest");

        let package = MissionPackage::load(&manifest).expect("load package");
        let materialized = package
            .materialize_sidecars()
            .expect("materialize sidecars");
        assert_eq!(materialized.sidecars.len(), 4);
        let report = package.check().expect("check package");
        assert_eq!(report.package_id, "pkg");
        assert!(report.hashes.contains_key("files.scenario"));
        assert!(report.hashes.contains_key("files.mission_graph"));
        assert!(report.hashes.contains_key("files.iload"));

        let run = package.run_case(Some("smoke")).expect("run package");
        assert_eq!(
            run.evidence.telemetry_rows,
            run.evidence.telemetry_summary.rows
        );
        assert!(!run.evidence.telemetry_summary.channel_names.is_empty());
        assert!(
            run.evidence
                .requirement_verdicts
                .iter()
                .any(|verdict| { verdict.id == "SIL-RUN-COMPLETE" && verdict.verdict == "pass" })
        );
        // A clean run that reaches its configured end time must report a
        // computed `pass` verdict with no failures and a passing nominal
        // termination check — not the old hardcoded "pass".
        assert_eq!(run.stop_label, "end-time");
        assert_eq!(run.evidence.verdict, "pass");
        assert!(run.evidence.failures.is_empty());
        assert!(
            run.evidence
                .requirement_verdicts
                .iter()
                .any(|verdict| {
                    verdict.id == "SIL-NOMINAL-TERMINATION" && verdict.verdict == "pass"
                })
        );
        assert!(
            run.evidence
                .bus_frames
                .iter()
                .any(|frame| frame.stream == "mission.event" && frame.subject == "sil.fixture")
        );
        let phase_signal = read_signal(&run, "mission.phase", 4).expect("read mission phase");
        assert_eq!(phase_signal.kind, "text");
        assert!(phase_signal.samples > 0);
        assert_eq!(
            phase_signal.preview.len().min(4),
            phase_signal.preview.len()
        );
        assert!(!capture_bus_frames(&run).is_empty());

        let until_event = package
            .run_case_until(
                Some("smoke"),
                RunUntilTarget::Event("mission.events.marker".to_owned()),
            )
            .expect("run until marker");
        assert_eq!(
            until_event.stop_label,
            "sil.until.event.mission.events.marker"
        );
        assert!(
            until_event
                .evidence
                .bus_frames
                .iter()
                .any(|frame| frame.stream == "mission.event" && frame.subject == "sil.fixture")
        );

        let until_phase = package
            .run_case_until(Some("smoke"), RunUntilTarget::Phase("coast".to_owned()))
            .expect("run until coast phase");
        assert_eq!(until_phase.stop_label, "sil.until.phase.coast");
        assert!(
            until_phase
                .evidence
                .phase_trace
                .iter()
                .any(|entry| entry.phase == "mission.phases.coast")
        );

        let command_stimulus = SilStimulation {
            parameter_overrides: vec![ParameterOverride {
                path: "time.stop_s".to_owned(),
                value: toml::Value::Float(0.03),
            }],
            command_writes: vec![CommandWrite::Effector {
                time_s: 0.01,
                id: "delta".to_owned(),
                command: 0.2,
            }],
            ..SilStimulation::default()
        };
        let commanded = package
            .run_case_with_stimulation(Some("smoke"), &command_stimulus)
            .expect("run with command write");
        assert_eq!(commanded.evidence.stimuli.len(), 2);
        let commanded_effector =
            read_signal(&commanded, "effector.delta.actual", 4).expect("read commanded effector");
        let commanded_max = commanded_effector
            .max
            .as_deref()
            .expect("numeric commanded max")
            .parse::<f64>()
            .expect("parse commanded max effector");
        assert!(
            commanded_max > 0.1,
            "command write must move effector, got max {commanded_max}"
        );

        let faulted_stimulus = SilStimulation {
            faults: vec![FaultInjection {
                target: FaultTarget::Effector("delta".to_owned()),
                fault: toml_value(r#"fault = { kind = "jam", at = 0.0 }"#, "fault"),
            }],
            parameter_overrides: vec![ParameterOverride {
                path: "time.stop_s".to_owned(),
                value: toml::Value::Float(0.03),
            }],
            command_writes: vec![CommandWrite::Effector {
                time_s: 0.01,
                id: "delta".to_owned(),
                command: 0.2,
            }],
        };
        let faulted = package
            .run_case_with_stimulation(Some("smoke"), &faulted_stimulus)
            .expect("run with jam fault");
        assert_eq!(faulted.evidence.stimuli.len(), 3);
        let faulted_effector =
            read_signal(&faulted, "effector.delta.actual", 4).expect("read stimulated effector");
        let max_effector = faulted_effector
            .max
            .as_deref()
            .expect("numeric max")
            .parse::<f64>()
            .expect("parse max effector");
        assert!(
            max_effector.abs() < 1.0e-12,
            "jammed effector must reject command, got max {max_effector}"
        );

        fs::write(dir.path().join("mission-graph.toml"), "[mission]\n")
            .expect("drift mission graph sidecar");
        assert!(matches!(
            package.check(),
            Err(SilError::SidecarMismatch { .. })
        ));
    }

    #[test]
    fn stop_label_classifier_flags_abnormal_terminations() {
        // Clean terminals — including scenario mission ends and SIL
        // run-until stops — are nominal.
        assert!(stop_label_is_nominal("end-time"));
        assert!(stop_label_is_nominal("mission-ended"));
        assert!(stop_label_is_nominal("sil.until.event.mission.events.cutoff"));
        // Divergences, ground impact, and guard-rail truncation are not.
        assert!(!stop_label_is_nominal("ground-impact"));
        assert!(!stop_label_is_nominal("non-finite-state"));
        assert!(!stop_label_is_nominal("step-overflow"));
        assert!(!stop_label_is_nominal("max-steps"));
    }

    #[test]
    fn aggregate_verdict_passes_only_without_failures() {
        assert_eq!(aggregate_verdict(&[]), "pass");
        let failures = vec![EvidenceFailure {
            kind: "requirement-failed".to_owned(),
            summary: "SIL-NOMINAL-TERMINATION: run stop reason: ground-impact".to_owned(),
            channel: None,
            time_s: Some(1.0),
            step: Some(2),
        }];
        assert_eq!(aggregate_verdict(&failures), "fail");
    }

    #[test]
    fn non_nominal_stop_fails_verdict_and_records_failure() {
        use openbmp_telemetry::TelemetrySchema;

        // The verdict must be a function of the actual run outcome, not a
        // hardcoded "pass": an abnormal stop reason fails the nominal
        // termination requirement and produces a failure entry.
        let table = TelemetryTable::new(TelemetrySchema::new(Vec::new()).expect("empty schema"));
        let verdicts = requirement_verdicts(&table, "ground-impact");
        let termination = verdicts
            .iter()
            .find(|verdict| verdict.id == "SIL-NOMINAL-TERMINATION")
            .expect("nominal-termination verdict present");
        assert_eq!(termination.verdict, "fail");

        let failures = evidence_failures(&verdicts, 2.5, 7);
        assert_eq!(aggregate_verdict(&failures), "fail");
        assert!(
            failures
                .iter()
                .any(|failure| failure.summary.contains("SIL-NOMINAL-TERMINATION")),
            "ground-impact run must record a nominal-termination failure"
        );
    }

    fn toml_value(text: &str, key: &str) -> toml::Value {
        let value: toml::Value = toml::from_str(text).expect("parse toml value");
        value
            .get(key)
            .cloned()
            .unwrap_or_else(|| panic!("{key} present"))
    }
}
