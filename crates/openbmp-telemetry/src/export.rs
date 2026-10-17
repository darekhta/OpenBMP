//! Experimental telemetry interchange exports.
//!
//! These artifacts are shaped for host-side ground-tool experiments, but they
//! are not CCSDS, XTCE, PUS, or ground-segment conformance claims. The label is
//! embedded into every binary frame stream and dictionary export.

use std::io::Write;

use openbmp_core::{ChannelId, SimTime, StepIndex};

use crate::TELEMETRY_SCHEMA_VERSION;
use crate::channel::ChannelMetadata;
use crate::error::TelemetryError;
use crate::row::TelemetryRow;
use crate::schema::TelemetrySchema;
use crate::table::TelemetryTable;
use crate::value::{TelemetryValue, TelemetryValueKind};

/// File-level magic for OpenBMP experimental frame streams.
pub const FRAME_STREAM_MAGIC: &[u8; 8] = b"OBMPFS1\0";
/// Per-frame sync marker borrowed as shape vocabulary from CCSDS AOS/TM.
pub const CCSDS_SHAPED_FRAME_SYNC: [u8; 4] = [0x1a, 0xcf, 0xfc, 0x1d];
/// Required non-conformance label embedded in frame streams.
pub const FRAME_STREAM_LABEL: &str = "openbmp.telemetry.ccsds-shaped.nonconformant.frame-stream.v1";
/// Required non-conformance label embedded in dictionary exports.
pub const TELEMETRY_DICTIONARY_LABEL: &str =
    "openbmp.telemetry.xtce-shaped.nonconformant.dictionary.v1";

const KIND_FLOAT64: u8 = 1;
const KIND_INT64: u8 = 2;
const KIND_BOOL: u8 = 3;
const KIND_TEXT: u8 = 4;
const ABSENT: u8 = 0;
const PRESENT: u8 = 1;
const NO_FRAME: u16 = u16::MAX;
const MAX_VIRTUAL_CHANNEL_ID: u8 = 63;

/// Write a deterministic, self-describing, CCSDS-shaped binary frame stream.
///
/// # Errors
///
/// Returns [`TelemetryError`] when writing fails, when schema/text fields are
/// too large for the fixed prefixes, or when the virtual channel id exceeds
/// the shaped-frame range used by this experimental adapter.
pub fn write_ccsds_shaped_frame_stream<W: Write>(
    table: &TelemetryTable,
    virtual_channel_id: u8,
    mut writer: W,
) -> Result<(), TelemetryError> {
    if virtual_channel_id > MAX_VIRTUAL_CHANNEL_ID {
        return Err(TelemetryError::InvalidFrameStream {
            reason: "virtual channel id must be in 0..=63",
        });
    }

    writer.write_all(FRAME_STREAM_MAGIC)?;
    write_len16_bytes(
        &mut writer,
        "frame_stream_label",
        FRAME_STREAM_LABEL.as_bytes(),
    )?;
    write_len16_bytes(
        &mut writer,
        "telemetry_schema_version",
        TELEMETRY_SCHEMA_VERSION.as_bytes(),
    )?;
    write_u32(
        &mut writer,
        table.schema().channels().len(),
        "channel_count",
    )?;
    for channel in table.schema().channels() {
        writer.write_all(&channel.id.value().to_be_bytes())?;
        writer.write_all(&[kind_to_byte(channel.value_kind)])?;
        write_len16_bytes(&mut writer, "channel.name", channel.name.as_bytes())?;
        write_len16_bytes(&mut writer, "channel.unit", channel.unit.as_bytes())?;
        match &channel.frame {
            Some(frame) => write_len16_bytes(&mut writer, "channel.frame", frame.as_bytes())?,
            None => writer.write_all(&NO_FRAME.to_be_bytes())?,
        }
    }

    writer.write_all(&(table.rows().len() as u64).to_be_bytes())?;
    for (sequence, row) in table.rows().iter().enumerate() {
        writer.write_all(&CCSDS_SHAPED_FRAME_SYNC)?;
        writer.write_all(&[virtual_channel_id])?;
        writer.write_all(&(sequence as u64).to_be_bytes())?;
        writer.write_all(&row.time.as_seconds().to_bits().to_be_bytes())?;
        writer.write_all(&row.step.value().to_be_bytes())?;
        write_u32(
            &mut writer,
            table.schema().channels().len(),
            "frame_channel_count",
        )?;
        for channel in table.schema().channels() {
            writer.write_all(&channel.id.value().to_be_bytes())?;
            writer.write_all(&[kind_to_byte(channel.value_kind)])?;
            match row.get(channel.id) {
                Some(value) => {
                    writer.write_all(&[PRESENT])?;
                    write_value(&mut writer, value)?;
                }
                None => writer.write_all(&[ABSENT])?,
            }
        }
    }
    Ok(())
}

/// Decode a deterministic CCSDS-shaped OpenBMP frame stream into a telemetry
/// table.
///
/// # Errors
///
/// Returns [`TelemetryError`] when the stream is malformed, truncated, has
/// trailing bytes, or contains invalid telemetry values.
pub fn read_ccsds_shaped_frame_stream(bytes: &[u8]) -> Result<TelemetryTable, TelemetryError> {
    let mut reader = FrameReader::new(bytes);
    if reader.read_array::<8>("magic")? != *FRAME_STREAM_MAGIC {
        return Err(TelemetryError::InvalidFrameStream {
            reason: "bad frame stream magic",
        });
    }
    let label = reader.read_string16("frame_stream_label")?;
    if label != FRAME_STREAM_LABEL {
        return Err(TelemetryError::InvalidFrameStream {
            reason: "bad frame stream label",
        });
    }
    let version = reader.read_string16("telemetry_schema_version")?;
    if version != TELEMETRY_SCHEMA_VERSION {
        return Err(TelemetryError::InvalidFrameStream {
            reason: "unsupported telemetry schema version",
        });
    }

    let channel_count = reader.read_u32("channel_count")? as usize;
    let mut channels = Vec::with_capacity(channel_count);
    for _ in 0..channel_count {
        let id = ChannelId::new(reader.read_u64("channel.id")?);
        let value_kind = byte_to_kind(reader.read_u8("channel.kind")?)?;
        let name = reader.read_string16("channel.name")?;
        let unit = reader.read_string16("channel.unit")?;
        let frame = reader.read_optional_string16("channel.frame")?;
        channels.push(ChannelMetadata::new(id, name, unit, frame, value_kind)?);
    }

    let schema = TelemetrySchema::new(channels)?;
    let channel_order = schema.channels().to_vec();
    let mut table = TelemetryTable::new(schema);
    let row_count = reader.read_u64("row_count")?;
    for expected_sequence in 0..row_count {
        if reader.read_array::<4>("frame_sync")? != CCSDS_SHAPED_FRAME_SYNC {
            return Err(TelemetryError::InvalidFrameStream {
                reason: "bad frame sync marker",
            });
        }
        if reader.read_u8("virtual_channel_id")? > MAX_VIRTUAL_CHANNEL_ID {
            return Err(TelemetryError::InvalidFrameStream {
                reason: "virtual channel id must be in 0..=63",
            });
        }
        let sequence = reader.read_u64("sequence_count")?;
        if sequence != expected_sequence {
            return Err(TelemetryError::InvalidFrameStream {
                reason: "non-contiguous frame sequence count",
            });
        }
        let time = SimTime::from_seconds(f64::from_bits(reader.read_u64("time_s")?));
        let step = StepIndex::new(reader.read_u64("step")?);
        let mut row = TelemetryRow::new(time, step)?;
        let frame_channel_count = reader.read_u32("frame_channel_count")? as usize;
        if frame_channel_count != channel_order.len() {
            return Err(TelemetryError::InvalidFrameStream {
                reason: "frame channel count differs from stream schema",
            });
        }
        for channel in &channel_order {
            let id = ChannelId::new(reader.read_u64("frame.channel.id")?);
            if id != channel.id {
                return Err(TelemetryError::InvalidFrameStream {
                    reason: "frame channel id differs from stream schema",
                });
            }
            let value_kind = byte_to_kind(reader.read_u8("frame.channel.kind")?)?;
            if value_kind != channel.value_kind {
                return Err(TelemetryError::InvalidFrameStream {
                    reason: "frame channel kind differs from stream schema",
                });
            }
            match reader.read_u8("frame.channel.presence")? {
                ABSENT => {}
                PRESENT => {
                    let value = reader.read_value(channel.value_kind)?;
                    row.insert_value(channel.id, value)?;
                }
                _ => {
                    return Err(TelemetryError::InvalidFrameStream {
                        reason: "invalid channel presence flag",
                    });
                }
            }
        }
        table.push_row(row)?;
    }
    reader.finish()?;
    Ok(table)
}

/// Write deterministic OpenBMP channel-dictionary JSON from a telemetry schema.
///
/// # Errors
///
/// Returns [`TelemetryError::Io`] when writing fails.
pub fn write_channel_dictionary_json<W: Write>(
    schema: &TelemetrySchema,
    mut writer: W,
) -> Result<(), TelemetryError> {
    writer.write_all(b"{\"channels\":[")?;
    for (index, channel) in schema.channels().iter().enumerate() {
        if index > 0 {
            writer.write_all(b",")?;
        }
        writer.write_all(b"{\"calibration\":\"identity\",\"frame\":")?;
        match &channel.frame {
            Some(frame) => write_json_string(&mut writer, frame)?,
            None => writer.write_all(b"null")?,
        }
        write!(writer, ",\"id\":{}", channel.id.value())?;
        writer.write_all(b",\"name\":")?;
        write_json_string(&mut writer, &channel.name)?;
        writer.write_all(b",\"type\":")?;
        write_json_string(&mut writer, channel.value_kind.as_str())?;
        writer.write_all(b",\"unit\":")?;
        write_json_string(&mut writer, &channel.unit)?;
        writer.write_all(b"}")?;
    }
    writer.write_all(b"],\"schema\":")?;
    write_json_string(&mut writer, TELEMETRY_DICTIONARY_LABEL)?;
    writer.write_all(b",\"standards\":{\"ccsds\":\"not claimed\",\"pus\":\"not claimed\",\"xtce\":\"not claimed\"},\"telemetry_schema_version\":")?;
    write_json_string(&mut writer, TELEMETRY_SCHEMA_VERSION)?;
    writer.write_all(b"}")?;
    Ok(())
}

/// Write deterministic XTCE-shaped XML from a telemetry schema.
///
/// # Errors
///
/// Returns [`TelemetryError::Io`] when writing fails.
pub fn write_xtce_shaped_dictionary_xml<W: Write>(
    schema: &TelemetrySchema,
    mut writer: W,
) -> Result<(), TelemetryError> {
    writer.write_all(
        br#"<?xml version="1.0" encoding="UTF-8"?>
<!-- OpenBMP telemetry dictionary export: XTCE-shaped metadata, non-conformant; no XTCE/CCSDS/PUS claim. -->
"#,
    )?;
    writeln!(
        writer,
        "<SpaceSystem name=\"OpenBMP\" schema=\"{}\" telemetrySchemaVersion=\"{}\">",
        TELEMETRY_DICTIONARY_LABEL, TELEMETRY_SCHEMA_VERSION
    )?;
    writer.write_all(
        b"  <Header validationStatus=\"non-conformant\" ccsds=\"not claimed\" pus=\"not claimed\" xtce=\"not claimed\"/>\n",
    )?;
    writer.write_all(b"  <TelemetryMetaData>\n    <ParameterSet>\n")?;
    for channel in schema.channels() {
        write!(
            writer,
            "      <Parameter name=\"{}\" id=\"{}\" parameterTypeRef=\"{}\" unit=\"{}\" calibration=\"identity\"",
            xml_escape(&channel.name),
            channel.id.value(),
            channel.value_kind.as_str(),
            xml_escape(&channel.unit)
        )?;
        if let Some(frame) = &channel.frame {
            write!(writer, " frame=\"{}\"", xml_escape(frame))?;
        }
        writer.write_all(b"/>\n")?;
    }
    writer.write_all(b"    </ParameterSet>\n  </TelemetryMetaData>\n</SpaceSystem>\n")?;
    Ok(())
}

fn write_value<W: Write>(writer: &mut W, value: &TelemetryValue) -> Result<(), TelemetryError> {
    match value {
        TelemetryValue::Float64(value) => writer.write_all(&value.to_bits().to_be_bytes())?,
        TelemetryValue::Int64(value) => writer.write_all(&value.to_be_bytes())?,
        TelemetryValue::Bool(value) => writer.write_all(&[u8::from(*value)])?,
        TelemetryValue::Text(value) => {
            write_len32_bytes(writer, "text_value", value.as_bytes())?;
        }
    }
    Ok(())
}

fn write_len16_bytes<W: Write>(
    writer: &mut W,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), TelemetryError> {
    if bytes.len() >= usize::from(NO_FRAME) {
        return Err(TelemetryError::ExportFieldTooLarge {
            field,
            length: bytes.len(),
        });
    }
    writer.write_all(&(bytes.len() as u16).to_be_bytes())?;
    writer.write_all(bytes)?;
    Ok(())
}

fn write_len32_bytes<W: Write>(
    writer: &mut W,
    field: &'static str,
    bytes: &[u8],
) -> Result<(), TelemetryError> {
    let length = u32::try_from(bytes.len()).map_err(|_| TelemetryError::ExportFieldTooLarge {
        field,
        length: bytes.len(),
    })?;
    writer.write_all(&length.to_be_bytes())?;
    writer.write_all(bytes)?;
    Ok(())
}

fn write_u32<W: Write>(
    writer: &mut W,
    value: usize,
    field: &'static str,
) -> Result<(), TelemetryError> {
    let value = u32::try_from(value).map_err(|_| TelemetryError::ExportFieldTooLarge {
        field,
        length: value,
    })?;
    writer.write_all(&value.to_be_bytes())?;
    Ok(())
}

fn kind_to_byte(kind: TelemetryValueKind) -> u8 {
    match kind {
        TelemetryValueKind::Float64 => KIND_FLOAT64,
        TelemetryValueKind::Int64 => KIND_INT64,
        TelemetryValueKind::Bool => KIND_BOOL,
        TelemetryValueKind::Text => KIND_TEXT,
    }
}

fn byte_to_kind(byte: u8) -> Result<TelemetryValueKind, TelemetryError> {
    match byte {
        KIND_FLOAT64 => Ok(TelemetryValueKind::Float64),
        KIND_INT64 => Ok(TelemetryValueKind::Int64),
        KIND_BOOL => Ok(TelemetryValueKind::Bool),
        KIND_TEXT => Ok(TelemetryValueKind::Text),
        _ => Err(TelemetryError::InvalidFrameStream {
            reason: "unknown telemetry value kind",
        }),
    }
}

fn write_json_string<W: Write>(writer: &mut W, value: &str) -> Result<(), TelemetryError> {
    serde_json::to_writer(writer, value).map_err(std::io::Error::other)?;
    Ok(())
}

fn xml_escape(value: &str) -> String {
    let mut escaped = String::with_capacity(value.len());
    for ch in value.chars() {
        match ch {
            '&' => escaped.push_str("&amp;"),
            '<' => escaped.push_str("&lt;"),
            '>' => escaped.push_str("&gt;"),
            '"' => escaped.push_str("&quot;"),
            '\'' => escaped.push_str("&apos;"),
            _ => escaped.push(ch),
        }
    }
    escaped
}

struct FrameReader<'a> {
    bytes: &'a [u8],
    offset: usize,
}

impl<'a> FrameReader<'a> {
    const fn new(bytes: &'a [u8]) -> Self {
        Self { bytes, offset: 0 }
    }

    fn read_array<const N: usize>(
        &mut self,
        field: &'static str,
    ) -> Result<[u8; N], TelemetryError> {
        let bytes = self.read_bytes(N, field)?;
        let mut array = [0_u8; N];
        array.copy_from_slice(bytes);
        Ok(array)
    }

    fn read_u8(&mut self, field: &'static str) -> Result<u8, TelemetryError> {
        Ok(self.read_array::<1>(field)?[0])
    }

    fn read_u16(&mut self, field: &'static str) -> Result<u16, TelemetryError> {
        Ok(u16::from_be_bytes(self.read_array::<2>(field)?))
    }

    fn read_u32(&mut self, field: &'static str) -> Result<u32, TelemetryError> {
        Ok(u32::from_be_bytes(self.read_array::<4>(field)?))
    }

    fn read_u64(&mut self, field: &'static str) -> Result<u64, TelemetryError> {
        Ok(u64::from_be_bytes(self.read_array::<8>(field)?))
    }

    fn read_string16(&mut self, field: &'static str) -> Result<String, TelemetryError> {
        let len = self.read_u16(field)?;
        if len == NO_FRAME {
            return Err(TelemetryError::InvalidFrameStream {
                reason: "unexpected absent optional string marker",
            });
        }
        self.read_string_bytes(usize::from(len), field)
    }

    fn read_optional_string16(
        &mut self,
        field: &'static str,
    ) -> Result<Option<String>, TelemetryError> {
        let len = self.read_u16(field)?;
        if len == NO_FRAME {
            Ok(None)
        } else {
            self.read_string_bytes(usize::from(len), field).map(Some)
        }
    }

    fn read_value(&mut self, kind: TelemetryValueKind) -> Result<TelemetryValue, TelemetryError> {
        match kind {
            TelemetryValueKind::Float64 => Ok(TelemetryValue::Float64(f64::from_bits(
                self.read_u64("float64")?,
            ))),
            TelemetryValueKind::Int64 => Ok(TelemetryValue::Int64(i64::from_be_bytes(
                self.read_array::<8>("int64")?,
            ))),
            TelemetryValueKind::Bool => match self.read_u8("bool")? {
                0 => Ok(TelemetryValue::Bool(false)),
                1 => Ok(TelemetryValue::Bool(true)),
                _ => Err(TelemetryError::InvalidFrameStream {
                    reason: "invalid boolean value",
                }),
            },
            TelemetryValueKind::Text => {
                let len = self.read_u32("text_len")? as usize;
                let value = self.read_string_bytes(len, "text_value")?;
                Ok(TelemetryValue::Text(value))
            }
        }
    }

    fn read_string_bytes(
        &mut self,
        len: usize,
        field: &'static str,
    ) -> Result<String, TelemetryError> {
        let bytes = self.read_bytes(len, field)?;
        let value = std::str::from_utf8(bytes)
            .map_err(|source| TelemetryError::InvalidFrameStreamUtf8 { field, source })?;
        Ok(value.to_owned())
    }

    fn read_bytes(&mut self, len: usize, field: &'static str) -> Result<&'a [u8], TelemetryError> {
        let end = self
            .offset
            .checked_add(len)
            .ok_or(TelemetryError::InvalidFrameStream {
                reason: "frame stream offset overflow",
            })?;
        if end > self.bytes.len() {
            return Err(TelemetryError::InvalidFrameStream { reason: field });
        }
        let bytes = &self.bytes[self.offset..end];
        self.offset = end;
        Ok(bytes)
    }

    fn finish(self) -> Result<(), TelemetryError> {
        if self.offset == self.bytes.len() {
            Ok(())
        } else {
            Err(TelemetryError::InvalidFrameStream {
                reason: "trailing bytes after frame stream",
            })
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::{TelemetryChannel, TelemetryRow};

    fn fixture_table() -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", Some("ECI"))
                .unwrap();
        let mode =
            TelemetryChannel::<i64>::new(ChannelId::new(2), "mode", "1", None::<String>).unwrap();
        let valid =
            TelemetryChannel::<bool>::new(ChannelId::new(3), "valid", "1", None::<String>).unwrap();
        let label =
            TelemetryChannel::<String>::new(ChannelId::new(4), "label", "1", Some("ops & review"))
                .unwrap();
        let schema = TelemetrySchema::new(vec![
            altitude.metadata().clone(),
            mode.metadata().clone(),
            valid.metadata().clone(),
            label.metadata().clone(),
        ])
        .unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut first = TelemetryRow::new(SimTime::from_seconds(1.0), StepIndex::new(2)).unwrap();
        first.insert(&altitude, 123.25).unwrap();
        first.insert(&mode, 7_i64).unwrap();
        first.insert(&valid, true).unwrap();
        first
            .insert(&label, "ascent,\"quoted\"".to_owned())
            .unwrap();
        table.push_row(first).unwrap();
        let mut second = TelemetryRow::new(SimTime::from_seconds(1.5), StepIndex::new(3)).unwrap();
        second.insert(&altitude, -4.5).unwrap();
        second.insert(&valid, false).unwrap();
        table.push_row(second).unwrap();
        table
    }

    #[test]
    fn ccsds_shaped_frame_stream_round_trips_declared_channels() {
        let table = fixture_table();
        let bytes = table.to_ccsds_shaped_frame_bytes(5).unwrap();
        let decoded = read_ccsds_shaped_frame_stream(&bytes).unwrap();
        assert_eq!(decoded, table);
        assert_eq!(
            decoded.to_parquet_bytes().unwrap(),
            table.to_parquet_bytes().unwrap()
        );
        assert!(bytes.starts_with(FRAME_STREAM_MAGIC));
        assert!(
            bytes
                .windows(FRAME_STREAM_LABEL.len())
                .any(|window| window == FRAME_STREAM_LABEL.as_bytes())
        );
    }

    #[test]
    fn ccsds_shaped_frame_stream_is_byte_stable_and_rejects_bad_inputs() {
        let table = fixture_table();
        let first = table.to_ccsds_shaped_frame_bytes(1).unwrap();
        let second = table.to_ccsds_shaped_frame_bytes(1).unwrap();
        assert_eq!(first, second);

        let err = table.to_ccsds_shaped_frame_bytes(64).unwrap_err();
        assert!(matches!(err, TelemetryError::InvalidFrameStream { .. }));

        let truncated = &first[..first.len() - 1];
        let err = read_ccsds_shaped_frame_stream(truncated).unwrap_err();
        assert!(matches!(err, TelemetryError::InvalidFrameStream { .. }));
    }

    #[test]
    fn channel_dictionary_exports_are_stable_and_non_conformant_labeled() {
        let table = fixture_table();
        let mut json = Vec::new();
        table.write_channel_dictionary_json(&mut json).unwrap();
        let json = String::from_utf8(json).unwrap();
        assert!(
            json.contains(
                "\"schema\":\"openbmp.telemetry.xtce-shaped.nonconformant.dictionary.v1\""
            )
        );
        assert!(json.contains("\"ccsds\":\"not claimed\""));
        assert!(json.find("\"altitude_m\"").unwrap() < json.find("\"mode\"").unwrap());

        let mut xml = Vec::new();
        table.write_xtce_shaped_dictionary_xml(&mut xml).unwrap();
        let xml = String::from_utf8(xml).unwrap();
        assert!(xml.contains("non-conformant"));
        assert!(xml.contains("xtce=\"not claimed\""));
        assert!(xml.contains("frame=\"ops &amp; review\""));
        assert!(xml.find("name=\"altitude_m\"").unwrap() < xml.find("name=\"mode\"").unwrap());
    }
}
