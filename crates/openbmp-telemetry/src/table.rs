//! Telemetry table — schema plus rows — and the entry points to
//! deterministic CSV / JSON / Parquet exporters.

use std::io::Write;

use crate::error::TelemetryError;
use crate::row::TelemetryRow;
use crate::schema::TelemetrySchema;
use crate::{csv_writer, json_writer, parquet_writer};

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

    /// Schema reference.
    #[must_use]
    pub const fn schema(&self) -> &TelemetrySchema {
        &self.schema
    }

    /// Rows in insertion order.
    #[must_use]
    pub fn rows(&self) -> &[TelemetryRow] {
        &self.rows
    }

    /// Take every recorded row out of the table, leaving it empty with
    /// the schema intact.
    ///
    /// For streaming consumers (an interactive host draining state as a
    /// run advances) that must bound table memory: rows accumulate
    /// per step, the consumer drains them each frame, and subsequent
    /// [`Self::push_row`] calls keep type-checking against the unchanged
    /// schema. A drained table exports an empty body — callers that want
    /// a complete archive must not drain.
    #[must_use]
    pub fn take_rows(&mut self) -> Vec<TelemetryRow> {
        core::mem::take(&mut self.rows)
    }

    /// Add a row after type-checking it against the schema.
    ///
    /// Missing channel values are allowed and exported as empty CSV
    /// fields, JSON `null`, and nullable Parquet values.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::UnknownChannel`] when a row references
    /// a channel absent from the schema, or
    /// [`TelemetryError::TypeMismatch`] when a value's type differs
    /// from the schema's declared kind.
    pub fn push_row(&mut self, row: TelemetryRow) -> Result<(), TelemetryError> {
        for (channel_id, value) in row.iter() {
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
    pub fn write_csv<W: Write>(&self, writer: W) -> Result<(), TelemetryError> {
        csv_writer::write(self, writer)
    }

    /// Export this table as deterministic JSON.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::Io`] when writing fails.
    pub fn write_json<W: Write>(&self, writer: W) -> Result<(), TelemetryError> {
        json_writer::write(self, writer)
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
        parquet_writer::write(self, writer)
    }
}
