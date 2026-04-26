//! Deterministic CSV writer.
//!
//! Floats are formatted with `"{:.17e}"`, integer step indexes verbatim,
//! booleans as `true` / `false`, and text fields RFC-4180-quoted only
//! when they contain a delimiter, quote, or newline.

use std::io::Write;

use crate::error::TelemetryError;
use crate::table::TelemetryTable;
use crate::value::TelemetryValue;

pub(crate) fn write<W: Write>(table: &TelemetryTable, mut writer: W) -> Result<(), TelemetryError> {
    writer.write_all(b"time_s,step")?;
    for channel in table.schema().channels() {
        writer.write_all(b",")?;
        write_field(&mut writer, &channel.name)?;
    }
    writer.write_all(b"\n")?;

    for row in table.rows() {
        write!(
            writer,
            "{:.17e},{}",
            row.time.as_seconds(),
            row.step.value()
        )?;
        for channel in table.schema().channels() {
            writer.write_all(b",")?;
            if let Some(value) = row.get(channel.id) {
                write_value(&mut writer, value)?;
            }
        }
        writer.write_all(b"\n")?;
    }
    Ok(())
}

fn write_value<W: Write>(writer: &mut W, value: &TelemetryValue) -> Result<(), TelemetryError> {
    match value {
        TelemetryValue::Float64(value) => write!(writer, "{value:.17e}")?,
        TelemetryValue::Int64(value) => write!(writer, "{value}")?,
        TelemetryValue::Bool(value) => write!(writer, "{value}")?,
        TelemetryValue::Text(value) => write_field(writer, value)?,
    }
    Ok(())
}

fn write_field<W: Write>(writer: &mut W, value: &str) -> Result<(), TelemetryError> {
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
                writer.write_all(std::slice::from_ref(byte))?;
            }
        }
        writer.write_all(b"\"")?;
    } else {
        writer.write_all(value.as_bytes())?;
    }
    Ok(())
}
