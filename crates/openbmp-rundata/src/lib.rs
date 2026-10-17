//! Deterministic run-data records and append-only indexes.
//!
//! This crate is the WP-21.1 substrate for file-based run records. It consumes
//! already-produced telemetry tables and emits canonical JSONL record bytes with
//! per-channel digests. It intentionally contains no database, service, wall
//! clock, or runner dependency.

pub mod bands;
pub mod campaign;
pub mod export3d;
pub mod openmct;
#[cfg(feature = "openmct-host")]
pub mod openmct_host;
pub mod render;
pub mod trend;
pub mod verdict;

use std::collections::HashMap;
use std::io::Write;
use std::sync::Arc;

use arrow::array::{ArrayRef, StringBuilder, UInt64Builder};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use openbmp_telemetry::{TelemetryTable, TelemetryValue, TelemetryValueKind};
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;
use serde::{Deserialize, Serialize};
use sha2::{Digest, Sha256};
use thiserror::Error;

/// Run-data schema version written into artifacts.
pub const RUNDATA_SCHEMA_VERSION: &str = "openbmp.rundata.v1";

/// Errors returned by run-data record helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum RundataError {
    /// A required string field was empty after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// A SHA-256 digest field was not 64 lowercase hexadecimal characters.
    #[error("{field} must be a lowercase SHA-256 hex digest")]
    InvalidSha256 {
        /// Field name.
        field: &'static str,
    },
    /// JSON serialization or parsing failed.
    #[error("run-data JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// Arrow record construction failed.
    #[error("run-data Arrow error: {message}")]
    Arrow {
        /// Error message.
        message: String,
    },
    /// Parquet writing failed.
    #[error("run-data Parquet error: {message}")]
    Parquet {
        /// Error message.
        message: String,
    },
    /// Existing JSONL index bytes were not append-safe.
    #[error("existing run index JSONL must be empty or newline-terminated")]
    UnterminatedIndex,
    /// The JSONL index contains the same run id more than once.
    #[error("run index contains duplicate run id {run_id}")]
    DuplicateRunId {
        /// Duplicate run id.
        run_id: String,
    },
}

impl From<serde_json::Error> for RundataError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

impl From<arrow::error::ArrowError> for RundataError {
    fn from(error: arrow::error::ArrowError) -> Self {
        Self::Arrow {
            message: error.to_string(),
        }
    }
}

impl From<parquet::errors::ParquetError> for RundataError {
    fn from(error: parquet::errors::ParquetError) -> Self {
        Self::Parquet {
            message: error.to_string(),
        }
    }
}

/// Reference to a run artifact stored beside a run record.
#[derive(Clone, Debug, Deserialize, Eq, PartialEq, Serialize)]
pub struct ArtifactRef {
    /// Stable role label, for example `telemetry_csv` or `scenario`.
    pub role: String,
    /// Repository- or bundle-relative artifact path.
    pub path: String,
    /// SHA-256 digest of the artifact bytes.
    pub sha256_hex: String,
}

impl ArtifactRef {
    /// Construct a validated artifact reference.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError`] when any field is empty or the digest is not a
    /// lowercase SHA-256 hex string.
    pub fn new(
        role: impl Into<String>,
        path: impl Into<String>,
        sha256_hex: impl Into<String>,
    ) -> Result<Self, RundataError> {
        let role = require_non_empty("artifact.role", role.into())?;
        let path = require_non_empty("artifact.path", path.into())?;
        let sha256_hex = require_sha256("artifact.sha256_hex", sha256_hex.into())?;
        Ok(Self {
            role,
            path,
            sha256_hex,
        })
    }
}

/// Stable label attached to a run record.
#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
pub struct RunLabel {
    /// Label key.
    pub key: String,
    /// Label value.
    pub value: String,
}

impl RunLabel {
    /// Construct a validated run label.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError::EmptyField`] when key or value is empty.
    pub fn new(key: impl Into<String>, value: impl Into<String>) -> Result<Self, RundataError> {
        Ok(Self {
            key: require_non_empty("label.key", key.into())?,
            value: require_non_empty("label.value", value.into())?,
        })
    }
}

/// Gate result included in a run record.
#[derive(Clone, Debug, Deserialize, Eq, Ord, PartialEq, PartialOrd, Serialize)]
pub struct GateResult {
    /// Gate name.
    pub name: String,
    /// Whether the gate passed.
    pub passed: bool,
    /// Stable detail string.
    pub detail: String,
}

impl GateResult {
    /// Construct a validated gate result.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError::EmptyField`] when name or detail is empty.
    pub fn new(
        name: impl Into<String>,
        passed: bool,
        detail: impl Into<String>,
    ) -> Result<Self, RundataError> {
        Ok(Self {
            name: require_non_empty("gate.name", name.into())?,
            passed,
            detail: require_non_empty("gate.detail", detail.into())?,
        })
    }
}

/// Numeric statistics for a telemetry channel.
#[derive(Clone, Copy, Debug, Deserialize, PartialEq, Serialize)]
pub struct NumericChannelDigest {
    /// Minimum observed value.
    pub min: f64,
    /// Maximum observed value.
    pub max: f64,
    /// Arithmetic mean in row order.
    pub mean: f64,
    /// Root-mean-square value in row order.
    pub rms: f64,
    /// Last observed value.
    pub final_value: f64,
}

#[derive(Clone, Copy, Debug)]
struct NumericAccumulator {
    count: u64,
    min: f64,
    max: f64,
    sum: f64,
    sum_squares: f64,
    final_value: f64,
}

impl NumericAccumulator {
    fn new(value: f64) -> Self {
        Self {
            count: 1,
            min: value,
            max: value,
            sum: value,
            sum_squares: value * value,
            final_value: value,
        }
    }

    fn observe(&mut self, value: f64) {
        self.count += 1;
        self.min = self.min.min(value);
        self.max = self.max.max(value);
        self.sum += value;
        self.sum_squares += value * value;
        self.final_value = value;
    }

    fn finish(self) -> NumericChannelDigest {
        let count = self.count as f64;
        NumericChannelDigest {
            min: self.min,
            max: self.max,
            mean: self.sum / count,
            rms: (self.sum_squares / count).sqrt(),
            final_value: self.final_value,
        }
    }
}

/// Deterministic digest for one telemetry channel.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct ChannelDigest {
    /// Channel name from the telemetry schema.
    pub channel: String,
    /// Primitive telemetry value kind.
    pub value_kind: String,
    /// Number of rows in the table.
    pub sample_count: u64,
    /// Number of rows where this channel had a value.
    pub present_count: u64,
    /// Numeric statistics for float64 and int64 channels.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub numeric: Option<NumericChannelDigest>,
    /// Number of true samples for bool channels.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub bool_true_count: Option<u64>,
    /// Last observed bool value for bool channels.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_bool: Option<bool>,
    /// SHA-256 digest of the last observed text value for text channels.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub final_text_sha256_hex: Option<String>,
}

/// Compute channel digests from a telemetry table in schema order.
#[must_use]
pub fn channel_digests(table: &TelemetryTable) -> Vec<ChannelDigest> {
    let sample_count = table.rows().len() as u64;
    table
        .schema()
        .channels()
        .iter()
        .map(|channel| {
            let mut numeric = None::<NumericAccumulator>;
            let mut bool_true_count = 0_u64;
            let mut final_bool = None;
            let mut final_text_sha256_hex = None;
            let mut present_count = 0_u64;
            for row in table.rows() {
                let Some(value) = row.get(channel.id) else {
                    continue;
                };
                present_count += 1;
                match value {
                    TelemetryValue::Float64(value) => observe_numeric(&mut numeric, *value),
                    TelemetryValue::Int64(value) => observe_numeric(&mut numeric, *value as f64),
                    TelemetryValue::Bool(value) => {
                        if *value {
                            bool_true_count += 1;
                        }
                        final_bool = Some(*value);
                    }
                    TelemetryValue::Text(value) => {
                        final_text_sha256_hex = Some(sha256_hex(value.as_bytes()));
                    }
                }
            }
            ChannelDigest {
                channel: channel.name.clone(),
                value_kind: channel.value_kind.as_str().to_owned(),
                sample_count,
                present_count,
                numeric: numeric.map(NumericAccumulator::finish),
                bool_true_count: (channel.value_kind == TelemetryValueKind::Bool)
                    .then_some(bool_true_count),
                final_bool,
                final_text_sha256_hex,
            }
        })
        .collect()
}

fn observe_numeric(accumulator: &mut Option<NumericAccumulator>, value: f64) {
    if let Some(accumulator) = accumulator {
        accumulator.observe(value);
    } else {
        *accumulator = Some(NumericAccumulator::new(value));
    }
}

/// Canonical record for one simulation run.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
pub struct RunRecord {
    /// Stable run id: SHA-256 over scenario digest, seed, and code id.
    pub run_id: String,
    /// Scenario content SHA-256 digest.
    pub scenario_sha256_hex: String,
    /// Scenario seed.
    pub seed: u64,
    /// Stable code/config id, such as a commit hash plus feature set.
    pub code_id: String,
    /// Gate results attached to this run.
    pub gates: Vec<GateResult>,
    /// Per-channel telemetry digests.
    pub channel_digests: Vec<ChannelDigest>,
    /// Referenced artifacts.
    pub artifacts: Vec<ArtifactRef>,
    /// Stable labels.
    pub labels: Vec<RunLabel>,
}

impl RunRecord {
    /// Construct a deterministic run record.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError`] for malformed required fields.
    pub fn new(
        scenario_sha256_hex: impl Into<String>,
        seed: u64,
        code_id: impl Into<String>,
        channel_digests: Vec<ChannelDigest>,
        mut artifacts: Vec<ArtifactRef>,
        mut labels: Vec<RunLabel>,
        mut gates: Vec<GateResult>,
    ) -> Result<Self, RundataError> {
        let scenario_sha256_hex =
            require_sha256("scenario_sha256_hex", scenario_sha256_hex.into())?;
        let code_id = require_non_empty("code_id", code_id.into())?;
        artifacts.sort_by(|a, b| {
            a.role
                .cmp(&b.role)
                .then_with(|| a.path.cmp(&b.path))
                .then_with(|| a.sha256_hex.cmp(&b.sha256_hex))
        });
        labels.sort();
        gates.sort();
        let run_id = compute_run_id(&scenario_sha256_hex, seed, &code_id);
        Ok(Self {
            run_id,
            scenario_sha256_hex,
            seed,
            code_id,
            gates,
            channel_digests,
            artifacts,
            labels,
        })
    }

    /// Serialize as one canonical JSONL record line.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError::Json`] if serialization fails.
    pub fn to_jsonl_line(&self) -> Result<Vec<u8>, RundataError> {
        let mut bytes = serde_json::to_vec(self)?;
        bytes.push(b'\n');
        Ok(bytes)
    }

    /// SHA-256 digest of the canonical JSONL record line.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError::Json`] if serialization fails.
    pub fn record_sha256_hex(&self) -> Result<String, RundataError> {
        Ok(sha256_hex(&self.to_jsonl_line()?))
    }
}

/// Append-only JSONL run index.
#[derive(Clone, Debug, Default, Deserialize, PartialEq, Serialize)]
pub struct RunIndex {
    records: Vec<RunRecord>,
}

impl RunIndex {
    /// Build an empty run index.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            records: Vec::new(),
        }
    }

    /// Records in append order.
    #[must_use]
    pub fn records(&self) -> &[RunRecord] {
        &self.records
    }

    /// Append a record after rejecting duplicate run ids.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError::DuplicateRunId`] when the run id already exists.
    pub fn append(&mut self, record: RunRecord) -> Result<(), RundataError> {
        if self
            .records
            .iter()
            .any(|existing| existing.run_id == record.run_id)
        {
            return Err(RundataError::DuplicateRunId {
                run_id: record.run_id,
            });
        }
        self.records.push(record);
        Ok(())
    }

    /// Parse an index from canonical JSONL bytes.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError`] for JSON errors or duplicate run ids.
    pub fn from_jsonl_bytes(bytes: &[u8]) -> Result<Self, RundataError> {
        let mut index = Self::new();
        for line in bytes.split(|byte| *byte == b'\n') {
            if line.is_empty() {
                continue;
            }
            let record: RunRecord = serde_json::from_slice(line)?;
            index.append(record)?;
        }
        Ok(index)
    }

    /// Serialize the full index as canonical JSONL bytes.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError::Json`] if serialization fails.
    pub fn to_jsonl_bytes(&self) -> Result<Vec<u8>, RundataError> {
        let mut bytes = Vec::new();
        for record in &self.records {
            bytes.extend(record.to_jsonl_line()?);
        }
        Ok(bytes)
    }

    /// Write a deterministic Parquet mirror of the run index.
    ///
    /// The Parquet mirror intentionally stores one summary row per run; the
    /// canonical nested record remains the JSONL line.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError`] if Arrow batch construction or Parquet writing
    /// fails.
    pub fn write_parquet<W: Write + Send>(&self, writer: W) -> Result<(), RundataError> {
        let schema = Arc::new(run_index_arrow_schema());
        let batch = run_index_record_batch(self, schema.clone())?;
        let properties = WriterProperties::builder()
            .set_created_by("openbmp-rundata".to_owned())
            .set_compression(Compression::SNAPPY)
            .build();
        let mut writer = ArrowWriter::try_new(writer, schema, Some(properties))?;
        writer.write(&batch)?;
        writer.close()?;
        Ok(())
    }

    /// Materialize a deterministic Parquet mirror as bytes.
    ///
    /// # Errors
    ///
    /// Returns [`RundataError`] if Parquet writing fails.
    pub fn to_parquet_bytes(&self) -> Result<Vec<u8>, RundataError> {
        let mut bytes = Vec::new();
        self.write_parquet(&mut bytes)?;
        Ok(bytes)
    }
}

fn run_index_arrow_schema() -> Schema {
    let mut metadata = HashMap::new();
    metadata.insert(
        "openbmp.schema_version".to_owned(),
        RUNDATA_SCHEMA_VERSION.to_owned(),
    );
    Schema::new_with_metadata(
        vec![
            Field::new("run_id", DataType::Utf8, false),
            Field::new("scenario_sha256_hex", DataType::Utf8, false),
            Field::new("seed", DataType::UInt64, false),
            Field::new("code_id", DataType::Utf8, false),
            Field::new("record_sha256_hex", DataType::Utf8, false),
            Field::new("channel_count", DataType::UInt64, false),
            Field::new("artifact_count", DataType::UInt64, false),
            Field::new("label_count", DataType::UInt64, false),
            Field::new("gate_count", DataType::UInt64, false),
        ],
        metadata,
    )
}

fn run_index_record_batch(
    index: &RunIndex,
    schema: Arc<Schema>,
) -> Result<RecordBatch, RundataError> {
    let count = index.records.len();
    let mut run_id = StringBuilder::with_capacity(count, count * 64);
    let mut scenario_sha256_hex = StringBuilder::with_capacity(count, count * 64);
    let mut seed = UInt64Builder::with_capacity(count);
    let mut code_id = StringBuilder::with_capacity(count, 0);
    let mut record_sha256_hex = StringBuilder::with_capacity(count, count * 64);
    let mut channel_count = UInt64Builder::with_capacity(count);
    let mut artifact_count = UInt64Builder::with_capacity(count);
    let mut label_count = UInt64Builder::with_capacity(count);
    let mut gate_count = UInt64Builder::with_capacity(count);

    for record in &index.records {
        run_id.append_value(&record.run_id);
        scenario_sha256_hex.append_value(&record.scenario_sha256_hex);
        seed.append_value(record.seed);
        code_id.append_value(&record.code_id);
        record_sha256_hex.append_value(record.record_sha256_hex()?);
        channel_count.append_value(record.channel_digests.len() as u64);
        artifact_count.append_value(record.artifacts.len() as u64);
        label_count.append_value(record.labels.len() as u64);
        gate_count.append_value(record.gates.len() as u64);
    }

    let columns: Vec<ArrayRef> = vec![
        Arc::new(run_id.finish()),
        Arc::new(scenario_sha256_hex.finish()),
        Arc::new(seed.finish()),
        Arc::new(code_id.finish()),
        Arc::new(record_sha256_hex.finish()),
        Arc::new(channel_count.finish()),
        Arc::new(artifact_count.finish()),
        Arc::new(label_count.finish()),
        Arc::new(gate_count.finish()),
    ];
    Ok(RecordBatch::try_new(schema, columns)?)
}

/// Append a canonical record line to existing JSONL index bytes.
///
/// # Errors
///
/// Returns [`RundataError`] if `existing` is not append-safe, serialization
/// fails, or the resulting index contains duplicate run ids.
pub fn append_record_jsonl(existing: &[u8], record: &RunRecord) -> Result<Vec<u8>, RundataError> {
    if !existing.is_empty() && !existing.ends_with(b"\n") {
        return Err(RundataError::UnterminatedIndex);
    }
    let mut bytes = existing.to_vec();
    bytes.extend(record.to_jsonl_line()?);
    RunIndex::from_jsonl_bytes(&bytes)?;
    Ok(bytes)
}

fn compute_run_id(scenario_sha256_hex: &str, seed: u64, code_id: &str) -> String {
    let mut hasher = Sha256::new();
    hasher.update(b"openbmp.rundata.run-id.v1\0");
    hasher.update(scenario_sha256_hex.as_bytes());
    hasher.update(b"\0");
    hasher.update(seed.to_le_bytes());
    hasher.update(b"\0");
    hasher.update(code_id.as_bytes());
    let digest = hasher.finalize();
    hex_digest(&digest)
}

// Keep table-cell escaping identical across Markdown report outputs.
fn markdown_cell(value: &str) -> String {
    value.replace('|', "\\|").replace('\n', " ")
}

pub(crate) fn require_non_empty(
    field: &'static str,
    value: String,
) -> Result<String, RundataError> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return Err(RundataError::EmptyField { field });
    }
    Ok(trimmed.to_owned())
}

fn require_sha256(field: &'static str, value: String) -> Result<String, RundataError> {
    let value = require_non_empty(field, value)?;
    if value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        Ok(value)
    } else {
        Err(RundataError::InvalidSha256 { field })
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let mut hasher = Sha256::new();
    hasher.update(bytes);
    let digest = hasher.finalize();
    hex_digest(&digest)
}

fn hex_digest(bytes: &[u8]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut out = String::with_capacity(bytes.len() * 2);
    for byte in bytes {
        out.push(HEX[(byte >> 4) as usize] as char);
        out.push(HEX[(byte & 0x0f) as usize] as char);
    }
    out
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use openbmp_core::{ChannelId, SimTime, StepIndex};
    use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema};

    use super::*;

    #[test]
    fn markdown_cell_escapes_pipes_and_flattens_line_feeds() {
        assert_eq!(markdown_cell("|first|\nsecond\n"), "\\|first\\| second ");
        assert_eq!(markdown_cell("\\|"), "\\\\|");
    }

    #[test]
    fn markdown_cell_preserves_other_text() {
        for value in ["", "plain text", "Δοκιμή", "a\\b\r\tc"] {
            assert_eq!(markdown_cell(value), value);
        }
    }

    fn fixture_table() -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let mode =
            TelemetryChannel::<i64>::new(ChannelId::new(2), "mode", "1", None::<String>).unwrap();
        let valid =
            TelemetryChannel::<bool>::new(ChannelId::new(3), "valid", "1", None::<String>).unwrap();
        let phase =
            TelemetryChannel::<String>::new(ChannelId::new(4), "phase", "1", None::<String>)
                .unwrap();
        let schema = TelemetrySchema::new(vec![
            altitude.metadata().clone(),
            mode.metadata().clone(),
            valid.metadata().clone(),
            phase.metadata().clone(),
        ])
        .unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut row0 = TelemetryRow::new(SimTime::from_seconds(0.0), StepIndex::new(0)).unwrap();
        row0.insert(&altitude, 3.0).unwrap();
        row0.insert(&mode, 2_i64).unwrap();
        row0.insert(&valid, true).unwrap();
        row0.insert(&phase, "boost".to_owned()).unwrap();
        table.push_row(row0).unwrap();
        let mut row1 = TelemetryRow::new(SimTime::from_seconds(1.0), StepIndex::new(1)).unwrap();
        row1.insert(&altitude, 4.0).unwrap();
        row1.insert(&mode, 6_i64).unwrap();
        row1.insert(&valid, false).unwrap();
        row1.insert(&phase, "coast".to_owned()).unwrap();
        table.push_row(row1).unwrap();
        table
    }

    fn record(seed: u64) -> RunRecord {
        RunRecord::new(
            "1111111111111111111111111111111111111111111111111111111111111111",
            seed,
            "test-code",
            channel_digests(&fixture_table()),
            vec![
                ArtifactRef::new(
                    "telemetry_csv",
                    "out/run.csv",
                    "2222222222222222222222222222222222222222222222222222222222222222",
                )
                .unwrap(),
            ],
            vec![RunLabel::new("family", "unit").unwrap()],
            vec![GateResult::new("finite-state", true, "all finite").unwrap()],
        )
        .unwrap()
    }

    #[test]
    fn channel_digest_matches_direct_statistics() {
        let digests = channel_digests(&fixture_table());
        assert_eq!(digests.len(), 4);
        let altitude = &digests[0];
        assert_eq!(altitude.channel, "altitude_m");
        assert_eq!(altitude.present_count, 2);
        let numeric = altitude.numeric.expect("altitude numeric digest");
        assert_eq!(numeric.min.to_bits(), 3.0_f64.to_bits());
        assert_eq!(numeric.max.to_bits(), 4.0_f64.to_bits());
        assert_eq!(numeric.mean.to_bits(), 3.5_f64.to_bits());
        assert_eq!(numeric.rms.to_bits(), (12.5_f64).sqrt().to_bits());
        assert_eq!(numeric.final_value.to_bits(), 4.0_f64.to_bits());

        assert_eq!(digests[2].bool_true_count, Some(1));
        assert_eq!(digests[2].final_bool, Some(false));
        let coast_sha256 = sha256_hex(b"coast");
        assert_eq!(
            digests[3].final_text_sha256_hex.as_deref(),
            Some(coast_sha256.as_str())
        );
    }

    #[test]
    fn identical_run_record_bytes_are_stable() {
        let a = record(7);
        let b = record(7);
        assert_eq!(a.run_id, b.run_id);
        assert_eq!(a.to_jsonl_line().unwrap(), b.to_jsonl_line().unwrap());
        assert_eq!(
            a.record_sha256_hex().unwrap(),
            b.record_sha256_hex().unwrap()
        );
    }

    #[test]
    fn append_record_jsonl_preserves_existing_prefix() {
        let first = record(1);
        let second = record(2);
        let first_bytes = first.to_jsonl_line().unwrap();
        let appended = append_record_jsonl(&first_bytes, &second).unwrap();
        assert!(appended.starts_with(&first_bytes));
        let index = RunIndex::from_jsonl_bytes(&appended).unwrap();
        assert_eq!(index.records().len(), 2);
        assert_eq!(index.to_jsonl_bytes().unwrap(), appended);
    }

    #[test]
    fn run_index_parquet_mirror_is_byte_stable() {
        let mut index = RunIndex::new();
        index.append(record(1)).unwrap();
        index.append(record(2)).unwrap();

        let a = index.to_parquet_bytes().unwrap();
        let b = index.to_parquet_bytes().unwrap();
        assert_eq!(a, b);
        assert!(!a.is_empty());
    }

    #[test]
    fn append_record_jsonl_rejects_unterminated_existing_index() {
        let err = append_record_jsonl(b"{\"run_id\":\"partial\"}", &record(3)).unwrap_err();
        assert_eq!(err, RundataError::UnterminatedIndex);
    }

    #[test]
    fn run_index_rejects_duplicate_run_ids() {
        let first = record(4);
        let duplicate = record(4);
        let mut index = RunIndex::new();
        index.append(first).unwrap();
        let err = index.append(duplicate).unwrap_err();
        assert!(matches!(err, RundataError::DuplicateRunId { .. }));
    }

    #[test]
    fn record_validation_rejects_bad_sha() {
        let err = RunRecord::new(
            "not-a-sha",
            1,
            "code",
            Vec::new(),
            Vec::new(),
            Vec::new(),
            Vec::new(),
        )
        .unwrap_err();
        assert_eq!(
            err,
            RundataError::InvalidSha256 {
                field: "scenario_sha256_hex"
            }
        );
    }
}
