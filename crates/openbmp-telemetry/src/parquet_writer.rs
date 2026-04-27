//! Parquet writer using Arrow column builders.
//!
//! Per-row schema metadata is attached to each Arrow field so a Parquet
//! reader can recover units, frames, schema version, and the channel id
//! without consulting an external sidecar. The whole-table schema also
//! carries the schema-version string.

use std::collections::HashMap;
use std::io::Write;
use std::sync::Arc;

use arrow::array::{
    ArrayRef, BooleanBuilder, Float64Builder, Int64Builder, StringBuilder, UInt64Builder,
};
use arrow::datatypes::{DataType, Field, Schema};
use arrow::record_batch::RecordBatch;
use parquet::arrow::ArrowWriter;
use parquet::basic::Compression;
use parquet::file::properties::WriterProperties;

use crate::TELEMETRY_SCHEMA_VERSION;
use crate::error::TelemetryError;
use crate::row::TelemetryRow;
use crate::table::TelemetryTable;
use crate::value::{TelemetryValue, TelemetryValueKind};

pub(crate) fn write<W: Write + Send>(
    table: &TelemetryTable,
    writer: W,
) -> Result<(), TelemetryError> {
    let schema = Arc::new(arrow_schema(table));
    let batch = arrow_record_batch(table, schema.clone())?;
    let properties = WriterProperties::builder()
        .set_created_by("openbmp-telemetry".to_owned())
        .set_compression(Compression::SNAPPY)
        .build();
    let mut writer = ArrowWriter::try_new(writer, schema, Some(properties))?;
    writer.write(&batch)?;
    writer.close()?;
    Ok(())
}

fn arrow_schema(table: &TelemetryTable) -> Schema {
    let channels = table.schema().channels();
    let mut fields = Vec::with_capacity(channels.len() + 2);
    fields.push(Field::new("time_s", DataType::Float64, false));
    fields.push(Field::new("step", DataType::UInt64, false));

    for channel in channels {
        let data_type = arrow_data_type(channel.value_kind);
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
    // Copy the schema's table-level metadata into the Arrow schema.
    // Iteration is BTreeMap-ordered (the source) so the output is
    // deterministic.
    for (key, value) in table.schema().metadata() {
        metadata.insert(key.clone(), value.clone());
    }
    Schema::new_with_metadata(fields, metadata)
}

const fn arrow_data_type(kind: TelemetryValueKind) -> DataType {
    match kind {
        TelemetryValueKind::Float64 => DataType::Float64,
        TelemetryValueKind::Int64 => DataType::Int64,
        TelemetryValueKind::Bool => DataType::Boolean,
        TelemetryValueKind::Text => DataType::Utf8,
    }
}

fn arrow_record_batch(
    table: &TelemetryTable,
    schema: Arc<Schema>,
) -> Result<RecordBatch, TelemetryError> {
    let rows = table.rows();
    let channels = table.schema().channels();
    let mut columns: Vec<ArrayRef> = Vec::with_capacity(channels.len() + 2);

    let mut time = Float64Builder::with_capacity(rows.len());
    let mut step = UInt64Builder::with_capacity(rows.len());
    for row in rows {
        time.append_value(row.time.as_seconds());
        step.append_value(row.step.value());
    }
    columns.push(Arc::new(time.finish()));
    columns.push(Arc::new(step.finish()));

    for channel in channels {
        columns.push(build_column(channel.value_kind, channel.id, rows));
    }

    Ok(RecordBatch::try_new(schema, columns)?)
}

/// Build one Arrow column for a channel by walking the rows once.
///
/// A row that has no value for `channel_id` produces an Arrow null;
/// a row whose stored value has the wrong kind also produces null
/// (this can only happen if the schema and table are out of sync,
/// which `TelemetryTable::push_row` rejects, so the path is
/// defence-in-depth rather than expected).
fn build_column(
    kind: TelemetryValueKind,
    channel_id: openbmp_core::ChannelId,
    rows: &[TelemetryRow],
) -> ArrayRef {
    match kind {
        TelemetryValueKind::Float64 => {
            let mut builder = Float64Builder::with_capacity(rows.len());
            for row in rows {
                match row.get(channel_id) {
                    Some(TelemetryValue::Float64(value)) => builder.append_value(*value),
                    _ => builder.append_null(),
                }
            }
            Arc::new(builder.finish()) as ArrayRef
        }
        TelemetryValueKind::Int64 => {
            let mut builder = Int64Builder::with_capacity(rows.len());
            for row in rows {
                match row.get(channel_id) {
                    Some(TelemetryValue::Int64(value)) => builder.append_value(*value),
                    _ => builder.append_null(),
                }
            }
            Arc::new(builder.finish()) as ArrayRef
        }
        TelemetryValueKind::Bool => {
            let mut builder = BooleanBuilder::with_capacity(rows.len());
            for row in rows {
                match row.get(channel_id) {
                    Some(TelemetryValue::Bool(value)) => builder.append_value(*value),
                    _ => builder.append_null(),
                }
            }
            Arc::new(builder.finish()) as ArrayRef
        }
        TelemetryValueKind::Text => {
            let mut builder = StringBuilder::with_capacity(rows.len(), 0);
            for row in rows {
                match row.get(channel_id) {
                    Some(TelemetryValue::Text(value)) => builder.append_value(value),
                    _ => builder.append_null(),
                }
            }
            Arc::new(builder.finish()) as ArrayRef
        }
    }
}
