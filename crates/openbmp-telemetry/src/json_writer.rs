//! Deterministic JSON writer.
//!
//! Layout:
//!
//! ```json
//! {
//!   "rows": [
//!     {"step": 2, "time_s": 1.0e0, "values": {"name": value, ...}},
//!     ...
//!   ],
//!   "schema": {
//!     "channels": [
//!       {"frame": ..., "id": ..., "name": ..., "type": ..., "unit": ...},
//!       ...
//!     ],
//!     "version": "openbmp.telemetry.v1"
//!   }
//! }
//! ```
//!
//! Object keys inside each object are written in fixed order to keep
//! byte-stable across reruns. Float values use the same `"{:.17e}"`
//! formatting as the CSV writer to round-trip through `f64::from_str`
//! losslessly.

use std::io::Write;

use crate::TELEMETRY_SCHEMA_VERSION;
use crate::error::TelemetryError;
use crate::table::TelemetryTable;
use crate::value::TelemetryValue;

pub(crate) fn write<W: Write>(table: &TelemetryTable, mut writer: W) -> Result<(), TelemetryError> {
    writer.write_all(b"{\"rows\":[")?;
    for (index, row) in table.rows().iter().enumerate() {
        if index > 0 {
            writer.write_all(b",")?;
        }
        write!(writer, "{{\"step\":{}", row.step.value())?;
        write!(writer, ",\"time_s\":{:.17e}", row.time.as_seconds())?;
        writer.write_all(b",\"values\":{")?;
        for (channel_index, channel) in table.schema().channels().iter().enumerate() {
            if channel_index > 0 {
                writer.write_all(b",")?;
            }
            write_string(&mut writer, &channel.name)?;
            writer.write_all(b":")?;
            match row.get(channel.id) {
                Some(value) => write_value(&mut writer, value)?,
                None => writer.write_all(b"null")?,
            }
        }
        writer.write_all(b"}}")?;
    }
    writer.write_all(b"],\"schema\":{\"channels\":[")?;
    for (index, channel) in table.schema().channels().iter().enumerate() {
        if index > 0 {
            writer.write_all(b",")?;
        }
        writer.write_all(b"{\"frame\":")?;
        match &channel.frame {
            Some(frame) => write_string(&mut writer, frame)?,
            None => writer.write_all(b"null")?,
        }
        write!(writer, ",\"id\":{}", channel.id.value())?;
        writer.write_all(b",\"name\":")?;
        write_string(&mut writer, &channel.name)?;
        writer.write_all(b",\"type\":")?;
        write_string(&mut writer, channel.value_kind.as_str())?;
        writer.write_all(b",\"unit\":")?;
        write_string(&mut writer, &channel.unit)?;
        writer.write_all(b"}")?;
    }
    writer.write_all(b"],\"version\":")?;
    write_string(&mut writer, TELEMETRY_SCHEMA_VERSION)?;
    writer.write_all(b"}}")?;
    Ok(())
}

fn write_value<W: Write>(writer: &mut W, value: &TelemetryValue) -> Result<(), TelemetryError> {
    match value {
        TelemetryValue::Float64(value) => write!(writer, "{value:.17e}")?,
        TelemetryValue::Int64(value) => write!(writer, "{value}")?,
        TelemetryValue::Bool(value) => write!(writer, "{value}")?,
        TelemetryValue::Text(value) => write_string(writer, value)?,
    }
    Ok(())
}

/// JSON-escape a string. Delegates to `serde_json` for correctness on
/// surrogate pairs, control chars, and the `\uXXXX` escape form.
fn write_string<W: Write>(writer: &mut W, value: &str) -> Result<(), TelemetryError> {
    serde_json::to_writer(writer, value).map_err(std::io::Error::other)?;
    Ok(())
}
