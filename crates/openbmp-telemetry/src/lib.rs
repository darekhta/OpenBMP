//! `openbmp-telemetry` — OpenBMP telemetry channels and exporters.
//!
//! This crate owns the Phase-1.4 telemetry surface:
//!
//! - typed channel metadata;
//! - bounded in-memory ring buffers for recent samples;
//! - deterministic CSV, JSON, and Parquet exporters.
//!
//! Exporters preserve the caller-supplied schema order. Floating-point
//! values are rejected when non-finite and are formatted with `"{:.17e}"`
//! in text archives.

use std::collections::{BTreeMap, BTreeSet, HashMap, VecDeque};
use std::io::Write;
use std::marker::PhantomData;
use std::sync::Arc;

use arrow::array::{
    ArrayRef, BooleanBuilder, Float64Builder, Int64Builder, StringBuilder, UInt64Builder,
};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use openbmp_core::{ChannelId, SimTime, StepIndex};
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;
use thiserror::Error;

/// Telemetry schema version written into archive metadata.
pub const TELEMETRY_SCHEMA_VERSION: &str = "openbmp.telemetry.v1";

/// Error type for telemetry schema, sample, and exporter operations.
#[derive(Debug, Error)]
pub enum TelemetryError {
    /// A channel name, unit, or frame string was empty.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Name of the invalid field.
        field: &'static str,
    },
    /// Ring-buffer capacity was zero.
    #[error("ring-buffer capacity must be greater than zero")]
    InvalidCapacity,
    /// A simulation time value was not finite or was negative.
    #[error("invalid simulation time: {seconds} s")]
    InvalidTime {
        /// Invalid time in seconds.
        seconds: f64,
    },
    /// A floating-point telemetry value was not finite.
    #[error("non-finite telemetry value for channel {channel}: {value}")]
    NonFiniteValue {
        /// Channel carrying the invalid value.
        channel: u64,
        /// Invalid floating-point value.
        value: f64,
    },
    /// The same channel id appeared more than once in a schema.
    #[error("duplicate telemetry channel id {id}")]
    DuplicateChannelId {
        /// Duplicate channel id.
        id: u64,
    },
    /// The same channel name appeared more than once in a schema.
    #[error("duplicate telemetry channel name {name}")]
    DuplicateChannelName {
        /// Duplicate channel name.
        name: String,
    },
    /// A row referenced a channel that is absent from the table schema.
    #[error("unknown telemetry channel id {id}")]
    UnknownChannel {
        /// Unknown channel id.
        id: u64,
    },
    /// A row value's type did not match its schema channel type.
    #[error("type mismatch for channel {channel}: expected {expected}, got {actual}")]
    TypeMismatch {
        /// Channel with the mismatch.
        channel: u64,
        /// Type declared by the schema.
        expected: TelemetryValueKind,
        /// Type supplied by the row.
        actual: TelemetryValueKind,
    },
    /// A row attempted to set the same channel twice.
    #[error("duplicate value for channel id {id} in row")]
    DuplicateRowValue {
        /// Duplicate channel id.
        id: u64,
    },
    /// IO failed while writing an archive.
    #[error("telemetry IO error")]
    Io {
        /// Source IO error.
        #[from]
        source: std::io::Error,
    },
    /// Arrow failed while building a record batch.
    #[error("arrow error")]
    Arrow {
        /// Source Arrow error.
        #[from]
        source: arrow::error::ArrowError,
    },
    /// Parquet failed while writing an archive.
    #[error("parquet error")]
    Parquet {
        /// Source Parquet error.
        #[from]
        source: parquet::errors::ParquetError,
    },
}

/// Primitive telemetry value type carried by a channel.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub enum TelemetryValueKind {
    /// IEEE-754 binary64 value.
    Float64,
    /// Signed 64-bit integer value.
    Int64,
    /// Boolean value.
    Bool,
    /// UTF-8 text value.
    Text,
}

impl TelemetryValueKind {
    /// Returns the canonical schema label.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::Float64 => "float64",
            Self::Int64 => "int64",
            Self::Bool => "bool",
            Self::Text => "text",
        }
    }
}

impl std::fmt::Display for TelemetryValueKind {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str(self.as_str())
    }
}

/// Runtime telemetry value.
#[derive(Clone, Debug, PartialEq)]
pub enum TelemetryValue {
    /// IEEE-754 binary64 value.
    Float64(f64),
    /// Signed 64-bit integer value.
    Int64(i64),
    /// Boolean value.
    Bool(bool),
    /// UTF-8 text value.
    Text(String),
}

impl TelemetryValue {
    /// Returns the value kind.
    #[must_use]
    pub const fn kind(&self) -> TelemetryValueKind {
        match self {
            Self::Float64(_) => TelemetryValueKind::Float64,
            Self::Int64(_) => TelemetryValueKind::Int64,
            Self::Bool(_) => TelemetryValueKind::Bool,
            Self::Text(_) => TelemetryValueKind::Text,
        }
    }

    /// Validates value-level invariants for a channel.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::NonFiniteValue`] for non-finite floats.
    pub fn require_valid_for(self, channel: ChannelId) -> Result<Self, TelemetryError> {
        if let Self::Float64(value) = self
            && !value.is_finite()
        {
            return Err(TelemetryError::NonFiniteValue {
                channel: channel.value(),
                value,
            });
        }
        Ok(self)
    }
}

/// Trait implemented by primitive types that can be recorded on typed channels.
pub trait TelemetryDatum: Sized {
    /// Runtime telemetry value kind for this Rust type.
    const KIND: TelemetryValueKind;

    /// Convert into a runtime [`TelemetryValue`].
    fn into_value(self) -> TelemetryValue;
}

impl TelemetryDatum for f64 {
    const KIND: TelemetryValueKind = TelemetryValueKind::Float64;

    fn into_value(self) -> TelemetryValue {
        TelemetryValue::Float64(self)
    }
}

impl TelemetryDatum for i64 {
    const KIND: TelemetryValueKind = TelemetryValueKind::Int64;

    fn into_value(self) -> TelemetryValue {
        TelemetryValue::Int64(self)
    }
}

impl TelemetryDatum for bool {
    const KIND: TelemetryValueKind = TelemetryValueKind::Bool;

    fn into_value(self) -> TelemetryValue {
        TelemetryValue::Bool(self)
    }
}

impl TelemetryDatum for String {
    const KIND: TelemetryValueKind = TelemetryValueKind::Text;

    fn into_value(self) -> TelemetryValue {
        TelemetryValue::Text(self)
    }
}

/// Metadata for one telemetry channel.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct ChannelMetadata {
    /// Stable channel identifier.
    pub id: ChannelId,
    /// Stable archive column name.
    pub name: String,
    /// Physical unit label, for example `m`, `m/s`, or `kg`.
    pub unit: String,
    /// Optional coordinate-frame label, for example `ECI` or `Body`.
    pub frame: Option<String>,
    /// Primitive value kind.
    pub value_kind: TelemetryValueKind,
}

impl ChannelMetadata {
    /// Create channel metadata.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::EmptyField`] when `name`, `unit`, or
    /// `frame` is empty after trimming.
    pub fn new(
        id: ChannelId,
        name: impl Into<String>,
        unit: impl Into<String>,
        frame: Option<impl Into<String>>,
        value_kind: TelemetryValueKind,
    ) -> Result<Self, TelemetryError> {
        let name = non_empty("name", name.into())?;
        let unit = non_empty("unit", unit.into())?;
        let frame = match frame {
            Some(value) => Some(non_empty("frame", value.into())?),
            None => None,
        };
        Ok(Self {
            id,
            name,
            unit,
            frame,
            value_kind,
        })
    }
}

/// Typed telemetry channel handle.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TelemetryChannel<T> {
    metadata: ChannelMetadata,
    _type: PhantomData<T>,
}

impl<T: TelemetryDatum> TelemetryChannel<T> {
    /// Create a typed channel.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::EmptyField`] for empty metadata fields.
    pub fn new(
        id: ChannelId,
        name: impl Into<String>,
        unit: impl Into<String>,
        frame: Option<impl Into<String>>,
    ) -> Result<Self, TelemetryError> {
        Ok(Self {
            metadata: ChannelMetadata::new(id, name, unit, frame, T::KIND)?,
            _type: PhantomData,
        })
    }

    /// Returns this channel's metadata.
    #[must_use]
    pub const fn metadata(&self) -> &ChannelMetadata {
        &self.metadata
    }

    /// Returns this channel's id.
    #[must_use]
    pub const fn id(&self) -> ChannelId {
        self.metadata.id
    }

    /// Create a validated sample for this channel.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidTime`] for invalid time and
    /// [`TelemetryError::NonFiniteValue`] for non-finite float values.
    pub fn sample(
        &self,
        time: SimTime,
        step: StepIndex,
        value: T,
    ) -> Result<TelemetrySample, TelemetryError> {
        TelemetrySample::new(time, step, self.id(), value.into_value())
    }
}

/// One typed channel sample at one simulation step.
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetrySample {
    /// Simulation time.
    pub time: SimTime,
    /// Simulation step index.
    pub step: StepIndex,
    /// Channel id.
    pub channel: ChannelId,
    /// Sample value.
    pub value: TelemetryValue,
}

impl TelemetrySample {
    /// Construct a validated telemetry sample.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidTime`] for invalid time and
    /// [`TelemetryError::NonFiniteValue`] for non-finite float values.
    pub fn new(
        time: SimTime,
        step: StepIndex,
        channel: ChannelId,
        value: TelemetryValue,
    ) -> Result<Self, TelemetryError> {
        if !time.is_valid() {
            return Err(TelemetryError::InvalidTime {
                seconds: time.as_seconds(),
            });
        }
        Ok(Self {
            time,
            step,
            channel,
            value: value.require_valid_for(channel)?,
        })
    }
}

/// Sink interface for receiving telemetry samples.
pub trait TelemetrySink {
    /// Record one telemetry sample.
    ///
    /// # Errors
    ///
    /// Implementations return [`TelemetryError`] when the sample cannot
    /// be accepted.
    fn push_sample(&mut self, sample: TelemetrySample) -> Result<(), TelemetryError>;
}

/// Bounded ring buffer storing the most recent telemetry samples.
#[derive(Clone, Debug)]
pub struct TelemetryRingBuffer {
    capacity: usize,
    samples: VecDeque<TelemetrySample>,
}

impl TelemetryRingBuffer {
    /// Create a ring buffer with a fixed sample capacity.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidCapacity`] when capacity is zero.
    pub fn new(capacity: usize) -> Result<Self, TelemetryError> {
        if capacity == 0 {
            return Err(TelemetryError::InvalidCapacity);
        }
        Ok(Self {
            capacity,
            samples: VecDeque::with_capacity(capacity),
        })
    }

    /// Returns the configured capacity.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    /// Returns the current number of retained samples.
    #[must_use]
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Returns `true` when no samples are retained.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Iterate over samples from oldest to newest.
    pub fn iter(&self) -> impl Iterator<Item = &TelemetrySample> {
        self.samples.iter()
    }

    /// Return the latest retained sample for a channel.
    #[must_use]
    pub fn latest_for(&self, channel: ChannelId) -> Option<&TelemetrySample> {
        self.samples
            .iter()
            .rev()
            .find(|sample| sample.channel == channel)
    }
}

impl TelemetrySink for TelemetryRingBuffer {
    fn push_sample(&mut self, sample: TelemetrySample) -> Result<(), TelemetryError> {
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
        Ok(())
    }
}

/// Deterministic telemetry archive schema.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TelemetrySchema {
    channels: Vec<ChannelMetadata>,
}

impl TelemetrySchema {
    /// Create a telemetry schema from channels in archive-column order.
    ///
    /// # Errors
    ///
    /// Returns duplicate-id or duplicate-name errors when the schema is
    /// not deterministic.
    pub fn new(channels: Vec<ChannelMetadata>) -> Result<Self, TelemetryError> {
        let mut ids = BTreeSet::new();
        let mut names = BTreeSet::new();
        for channel in &channels {
            if !ids.insert(channel.id) {
                return Err(TelemetryError::DuplicateChannelId {
                    id: channel.id.value(),
                });
            }
            if !names.insert(channel.name.clone()) {
                return Err(TelemetryError::DuplicateChannelName {
                    name: channel.name.clone(),
                });
            }
        }
        Ok(Self { channels })
    }

    /// Returns the channels in archive-column order.
    #[must_use]
    pub fn channels(&self) -> &[ChannelMetadata] {
        &self.channels
    }

    /// Resolve a channel by id.
    #[must_use]
    pub fn channel(&self, id: ChannelId) -> Option<&ChannelMetadata> {
        self.channels.iter().find(|channel| channel.id == id)
    }
}

/// One telemetry archive row.
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetryRow {
    /// Simulation time for the row.
    pub time: SimTime,
    /// Simulation step for the row.
    pub step: StepIndex,
    values: BTreeMap<ChannelId, TelemetryValue>,
}

impl TelemetryRow {
    /// Create an empty row.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidTime`] when `time` is invalid.
    pub fn new(time: SimTime, step: StepIndex) -> Result<Self, TelemetryError> {
        if !time.is_valid() {
            return Err(TelemetryError::InvalidTime {
                seconds: time.as_seconds(),
            });
        }
        Ok(Self {
            time,
            step,
            values: BTreeMap::new(),
        })
    }

    /// Insert a typed channel value.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::DuplicateRowValue`] for duplicate row
    /// values and [`TelemetryError::NonFiniteValue`] for non-finite
    /// floats.
    pub fn insert<T: TelemetryDatum>(
        &mut self,
        channel: &TelemetryChannel<T>,
        value: T,
    ) -> Result<(), TelemetryError> {
        let channel_id = channel.id();
        if self.values.contains_key(&channel_id) {
            return Err(TelemetryError::DuplicateRowValue {
                id: channel_id.value(),
            });
        }
        self.values.insert(
            channel_id,
            value.into_value().require_valid_for(channel_id)?,
        );
        Ok(())
    }

    /// Returns a value by channel id.
    #[must_use]
    pub fn get(&self, channel: ChannelId) -> Option<&TelemetryValue> {
        self.values.get(&channel)
    }
}

/// Telemetry table ready for deterministic archive export.
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetryTable {
    schema: TelemetrySchema,
    rows: Vec<TelemetryRow>,
}

impl TelemetryTable {
    /// Create an empty table.
    #[must_use]
    pub const fn new(schema: TelemetrySchema) -> Self {
        Self {
            schema,
            rows: Vec::new(),
        }
    }

    /// Returns the schema.
    #[must_use]
    pub const fn schema(&self) -> &TelemetrySchema {
        &self.schema
    }

    /// Returns all rows in insertion order.
    #[must_use]
    pub fn rows(&self) -> &[TelemetryRow] {
        &self.rows
    }

    /// Add a row after checking it against the schema.
    ///
    /// Missing channel values are allowed and exported as empty CSV
    /// fields, JSON `null`, and nullable Parquet values.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError`] when a row references an unknown
    /// channel or has a value whose type differs from the schema.
    pub fn push_row(&mut self, row: TelemetryRow) -> Result<(), TelemetryError> {
        for (channel_id, value) in &row.values {
            let Some(channel) = self.schema.channel(*channel_id) else {
                return Err(TelemetryError::UnknownChannel {
                    id: channel_id.value(),
                });
            };
            let actual = value.kind();
            if actual != channel.value_kind {
                return Err(TelemetryError::TypeMismatch {
                    channel: channel_id.value(),
                    expected: channel.value_kind,
                    actual,
                });
            }
        }
        self.rows.push(row);
        Ok(())
    }

    /// Export this table as deterministic CSV.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::Io`] when writing fails.
    pub fn write_csv<W: Write>(&self, mut writer: W) -> Result<(), TelemetryError> {
        writer.write_all(b"time_s,step")?;
        for channel in self.schema.channels() {
            writer.write_all(b",")?;
            write_csv_field(&mut writer, &channel.name)?;
        }
        writer.write_all(b"\n")?;

        for row in &self.rows {
            write!(
                writer,
                "{:.17e},{}",
                row.time.as_seconds(),
                row.step.value()
            )?;
            for channel in self.schema.channels() {
                writer.write_all(b",")?;
                if let Some(value) = row.get(channel.id) {
                    write_csv_value(&mut writer, value)?;
                }
            }
            writer.write_all(b"\n")?;
        }
        Ok(())
    }

    /// Export this table as deterministic JSON.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::Io`] when writing fails.
    pub fn write_json<W: Write>(&self, mut writer: W) -> Result<(), TelemetryError> {
        writer.write_all(b"{\"rows\":[")?;
        for (row_index, row) in self.rows.iter().enumerate() {
            if row_index > 0 {
                writer.write_all(b",")?;
            }
            writer.write_all(b"{\"step\":")?;
            write!(writer, "{}", row.step.value())?;
            writer.write_all(b",\"time_s\":")?;
            write!(writer, "{:.17e}", row.time.as_seconds())?;
            writer.write_all(b",\"values\":{")?;
            for (channel_index, channel) in self.schema.channels().iter().enumerate() {
                if channel_index > 0 {
                    writer.write_all(b",")?;
                }
                write_json_string(&mut writer, &channel.name)?;
                writer.write_all(b":")?;
                match row.get(channel.id) {
                    Some(value) => write_json_value(&mut writer, value)?,
                    None => writer.write_all(b"null")?,
                }
            }
            writer.write_all(b"}}")?;
        }
        writer.write_all(b"],\"schema\":{")?;
        writer.write_all(b"\"channels\":[")?;
        for (index, channel) in self.schema.channels().iter().enumerate() {
            if index > 0 {
                writer.write_all(b",")?;
            }
            writer.write_all(b"{\"frame\":")?;
            match &channel.frame {
                Some(frame) => write_json_string(&mut writer, frame)?,
                None => writer.write_all(b"null")?,
            }
            writer.write_all(b",\"id\":")?;
            write!(writer, "{}", channel.id.value())?;
            writer.write_all(b",\"name\":")?;
            write_json_string(&mut writer, &channel.name)?;
            writer.write_all(b",\"type\":")?;
            write_json_string(&mut writer, channel.value_kind.as_str())?;
            writer.write_all(b",\"unit\":")?;
            write_json_string(&mut writer, &channel.unit)?;
            writer.write_all(b"}")?;
        }
        writer.write_all(b"],\"version\":")?;
        write_json_string(&mut writer, TELEMETRY_SCHEMA_VERSION)?;
        writer.write_all(b"}}")?;
        Ok(())
    }

    /// Export this table as canonical Parquet bytes.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError`] when Arrow batch construction or
    /// Parquet writing fails.
    pub fn to_parquet_bytes(&self) -> Result<Vec<u8>, TelemetryError> {
        let mut bytes = Vec::new();
        self.write_parquet(&mut bytes)?;
        Ok(bytes)
    }

    /// Export this table as Parquet.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError`] when Arrow batch construction or
    /// Parquet writing fails.
    pub fn write_parquet<W: Write + Send>(&self, writer: W) -> Result<(), TelemetryError> {
        let schema = Arc::new(self.arrow_schema());
        let batch = self.arrow_record_batch(schema.clone())?;
        let properties = WriterProperties::builder()
            .set_created_by("openbmp-telemetry".to_owned())
            .set_compression(Compression::SNAPPY)
            .build();
        let mut writer = ArrowWriter::try_new(writer, schema, Some(properties))?;
        writer.write(&batch)?;
        writer.close()?;
        Ok(())
    }

    fn arrow_schema(&self) -> Schema {
        let mut fields = Vec::with_capacity(self.schema.channels().len() + 2);
        fields.push(Field::new("time_s", DataType::Float64, false));
        fields.push(Field::new("step", DataType::UInt64, false));
        for channel in self.schema.channels() {
            let data_type = match channel.value_kind {
                TelemetryValueKind::Float64 => DataType::Float64,
                TelemetryValueKind::Int64 => DataType::Int64,
                TelemetryValueKind::Bool => DataType::Boolean,
                TelemetryValueKind::Text => DataType::Utf8,
            };
            let mut metadata = HashMap::new();
            metadata.insert(
                "openbmp.channel_id".to_owned(),
                channel.id.value().to_string(),
            );
            metadata.insert("openbmp.unit".to_owned(), channel.unit.clone());
            metadata.insert(
                "openbmp.value_type".to_owned(),
                channel.value_kind.as_str().to_owned(),
            );
            metadata.insert(
                "openbmp.schema_version".to_owned(),
                TELEMETRY_SCHEMA_VERSION.to_owned(),
            );
            if let Some(frame) = &channel.frame {
                metadata.insert("openbmp.frame".to_owned(), frame.clone());
            }
            fields.push(Field::new(&channel.name, data_type, true).with_metadata(metadata));
        }
        let mut metadata = HashMap::new();
        metadata.insert(
            "openbmp.schema_version".to_owned(),
            TELEMETRY_SCHEMA_VERSION.to_owned(),
        );
        Schema::new_with_metadata(fields, metadata)
    }

    fn arrow_record_batch(&self, schema: Arc<Schema>) -> Result<RecordBatch, TelemetryError> {
        let mut columns: Vec<ArrayRef> = Vec::with_capacity(self.schema.channels().len() + 2);
        let mut time = Float64Builder::with_capacity(self.rows.len());
        let mut step = UInt64Builder::with_capacity(self.rows.len());
        for row in &self.rows {
            time.append_value(row.time.as_seconds());
            step.append_value(row.step.value());
        }
        columns.push(Arc::new(time.finish()));
        columns.push(Arc::new(step.finish()));

        for channel in self.schema.channels() {
            match channel.value_kind {
                TelemetryValueKind::Float64 => {
                    let mut builder = Float64Builder::with_capacity(self.rows.len());
                    for row in &self.rows {
                        match row.get(channel.id) {
                            Some(TelemetryValue::Float64(value)) => builder.append_value(*value),
                            None => builder.append_null(),
                            _ => unreachable!("row type checked before insertion"),
                        }
                    }
                    columns.push(Arc::new(builder.finish()));
                }
                TelemetryValueKind::Int64 => {
                    let mut builder = Int64Builder::with_capacity(self.rows.len());
                    for row in &self.rows {
                        match row.get(channel.id) {
                            Some(TelemetryValue::Int64(value)) => builder.append_value(*value),
                            None => builder.append_null(),
                            _ => unreachable!("row type checked before insertion"),
                        }
                    }
                    columns.push(Arc::new(builder.finish()));
                }
                TelemetryValueKind::Bool => {
                    let mut builder = BooleanBuilder::with_capacity(self.rows.len());
                    for row in &self.rows {
                        match row.get(channel.id) {
                            Some(TelemetryValue::Bool(value)) => builder.append_value(*value),
                            None => builder.append_null(),
                            _ => unreachable!("row type checked before insertion"),
                        }
                    }
                    columns.push(Arc::new(builder.finish()));
                }
                TelemetryValueKind::Text => {
                    let mut builder = StringBuilder::with_capacity(self.rows.len(), 0);
                    for row in &self.rows {
                        match row.get(channel.id) {
                            Some(TelemetryValue::Text(value)) => builder.append_value(value),
                            None => builder.append_null(),
                            _ => unreachable!("row type checked before insertion"),
                        }
                    }
                    columns.push(Arc::new(builder.finish()));
                }
            }
        }

        Ok(RecordBatch::try_new(schema, columns)?)
    }
}

fn non_empty(field: &'static str, value: String) -> Result<String, TelemetryError> {
    if value.trim().is_empty() {
        Err(TelemetryError::EmptyField { field })
    } else {
        Ok(value)
    }
}

fn write_csv_value<W: Write>(writer: &mut W, value: &TelemetryValue) -> Result<(), TelemetryError> {
    match value {
        TelemetryValue::Float64(value) => write!(writer, "{value:.17e}")?,
        TelemetryValue::Int64(value) => write!(writer, "{value}")?,
        TelemetryValue::Bool(value) => write!(writer, "{value}")?,
        TelemetryValue::Text(value) => write_csv_field(writer, value)?,
    }
    Ok(())
}

fn write_csv_field<W: Write>(writer: &mut W, value: &str) -> Result<(), TelemetryError> {
    let must_quote = value
        .as_bytes()
        .iter()
        .any(|b| matches!(b, b',' | b'"' | b'\n' | b'\r'));
    if must_quote {
        writer.write_all(b"\"")?;
        for byte in value.as_bytes() {
            if *byte == b'"' {
                writer.write_all(b"\"\"")?;
            } else {
                writer.write_all(&[*byte])?;
            }
        }
        writer.write_all(b"\"")?;
    } else {
        writer.write_all(value.as_bytes())?;
    }
    Ok(())
}

fn write_json_string<W: Write>(writer: &mut W, value: &str) -> Result<(), TelemetryError> {
    serde_json::to_writer(writer, value).map_err(std::io::Error::other)?;
    Ok(())
}

fn write_json_value<W: Write>(
    writer: &mut W,
    value: &TelemetryValue,
) -> Result<(), TelemetryError> {
    match value {
        TelemetryValue::Float64(value) => write!(writer, "{value:.17e}")?,
        TelemetryValue::Int64(value) => write!(writer, "{value}")?,
        TelemetryValue::Bool(value) => write!(writer, "{value}")?,
        TelemetryValue::Text(value) => write_json_string(writer, value)?,
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;

    fn channels() -> (
        TelemetryChannel<f64>,
        TelemetryChannel<i64>,
        TelemetryChannel<bool>,
        TelemetryChannel<String>,
    ) {
        (
            TelemetryChannel::new(ChannelId::new(1), "altitude_m", "m", Some("ECI")).unwrap(),
            TelemetryChannel::new(ChannelId::new(2), "mode", "1", None::<String>).unwrap(),
            TelemetryChannel::new(ChannelId::new(3), "valid", "1", None::<String>).unwrap(),
            TelemetryChannel::new(ChannelId::new(4), "label", "1", None::<String>).unwrap(),
        )
    }

    fn sample_table() -> TelemetryTable {
        let (altitude, mode, valid, label) = channels();
        let schema = TelemetrySchema::new(vec![
            altitude.metadata().clone(),
            mode.metadata().clone(),
            valid.metadata().clone(),
            label.metadata().clone(),
        ])
        .unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut row = TelemetryRow::new(SimTime::from_seconds(1.0), StepIndex::new(2)).unwrap();
        row.insert(&altitude, 123.25).unwrap();
        row.insert(&mode, 7_i64).unwrap();
        row.insert(&valid, true).unwrap();
        row.insert(&label, "ascent".to_owned()).unwrap();
        table.push_row(row).unwrap();
        table
    }

    #[test]
    fn ring_buffer_retains_latest_samples() {
        let (altitude, _, _, _) = channels();
        let mut buffer = TelemetryRingBuffer::new(2).unwrap();
        buffer
            .push_sample(
                altitude
                    .sample(SimTime::ZERO, StepIndex::ZERO, 1.0)
                    .unwrap(),
            )
            .unwrap();
        buffer
            .push_sample(
                altitude
                    .sample(SimTime::from_seconds(1.0), StepIndex::new(1), 2.0)
                    .unwrap(),
            )
            .unwrap();
        buffer
            .push_sample(
                altitude
                    .sample(SimTime::from_seconds(2.0), StepIndex::new(2), 3.0)
                    .unwrap(),
            )
            .unwrap();

        assert_eq!(buffer.len(), 2);
        assert_eq!(
            buffer.latest_for(altitude.id()).unwrap().value,
            TelemetryValue::Float64(3.0)
        );
        assert_eq!(buffer.iter().next().unwrap().step, StepIndex::new(1));
    }

    #[test]
    fn rejects_non_finite_float_values() {
        let (altitude, _, _, _) = channels();
        let err = altitude
            .sample(SimTime::ZERO, StepIndex::ZERO, f64::NAN)
            .unwrap_err();
        assert!(matches!(err, TelemetryError::NonFiniteValue { .. }));
    }

    #[test]
    fn schema_rejects_duplicate_names() {
        let a = ChannelMetadata::new(
            ChannelId::new(1),
            "same",
            "m",
            None::<String>,
            TelemetryValueKind::Float64,
        )
        .unwrap();
        let b = ChannelMetadata::new(
            ChannelId::new(2),
            "same",
            "m",
            None::<String>,
            TelemetryValueKind::Float64,
        )
        .unwrap();
        let err = TelemetrySchema::new(vec![a, b]).unwrap_err();
        assert!(matches!(err, TelemetryError::DuplicateChannelName { .. }));
    }

    #[test]
    fn csv_export_is_stable_and_ordered() {
        let table = sample_table();
        let mut csv = Vec::new();
        table.write_csv(&mut csv).unwrap();
        let csv = String::from_utf8(csv).unwrap();
        assert_eq!(
            csv,
            "time_s,step,altitude_m,mode,valid,label\n\
             1.00000000000000000e0,2,1.23250000000000000e2,7,true,ascent\n"
        );
    }

    #[test]
    fn json_export_is_stable_and_ordered() {
        let table = sample_table();
        let mut a = Vec::new();
        let mut b = Vec::new();
        table.write_json(&mut a).unwrap();
        table.write_json(&mut b).unwrap();
        assert_eq!(a, b);
        let json = String::from_utf8(a).unwrap();
        assert!(json.starts_with("{\"rows\":["));
        assert!(json.contains("\"version\":\"openbmp.telemetry.v1\""));
        assert!(json.contains("\"altitude_m\":1.23250000000000000e2"));
    }

    #[test]
    fn parquet_export_is_byte_stable() {
        let table = sample_table();
        let first = table.to_parquet_bytes().unwrap();
        let second = table.to_parquet_bytes().unwrap();
        assert_eq!(first, second);
        assert!(!first.is_empty());
    }
}
