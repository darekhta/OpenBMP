//! Append-only JSONL indexes and their deterministic Parquet mirror.

use std::collections::HashMap;
use std::io::Write;
use std::sync::Arc;

use arrow::array::{ArrayRef, StringBuilder, UInt64Builder};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;
use serde::{Deserialize, Serialize};

use crate::{RUNDATA_SCHEMA_VERSION, RunRecord, RundataError};

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
            record.append_jsonl_line(&mut bytes)?;
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
    record.append_jsonl_line(&mut bytes)?;
    RunIndex::from_jsonl_bytes(&bytes)?;
    Ok(bytes)
}
