//! `openbmp-telemetry` — OpenBMP telemetry channels and exporters.
//!
//! Phase 1.4 ships:
//!
//! - [`TelemetryValue`] / [`TelemetryValueKind`] / [`TelemetryDatum`] —
//!   the four primitive value kinds (`f64`, `i64`, `bool`, `String`)
//!   the rest of the workspace can record.
//! - [`ChannelMetadata`] / [`TelemetryChannel<T>`] — typed channel
//!   handles carrying name, unit, optional frame, and value kind.
//! - [`TelemetrySample`] / [`TelemetrySink`] / [`TelemetryRingBuffer`] —
//!   in-memory current-value access for the kernel's sensor / FC layers.
//! - [`TelemetrySchema`] / [`TelemetryRow`] / [`TelemetryTable`] —
//!   archive layout used by all exporters.
//! - Deterministic CSV, JSON, and Parquet exporters via
//!   [`TelemetryTable::write_csv`], [`TelemetryTable::write_json`], and
//!   [`TelemetryTable::write_parquet`].
//!
//! # Determinism contract
//!
//! Floats are formatted as `"{:.17e}"` everywhere they touch a textual
//! archive so the round-trip through `f64::from_str` is lossless.
//! Channel iteration order, JSON object key order, and Parquet column
//! order are all fixed by the schema. Parquet output uses Snappy
//! compression which produces the same bytes for the same input.
//!
//! See `docs/software-architecture.md § Telemetry` and
//! `docs/verification.md § Golden Telemetry`.

pub mod channel;
mod csv_writer;
pub mod error;
mod json_writer;
mod parquet_writer;
pub mod row;
pub mod sample;
pub mod schema;
pub mod table;
pub mod value;

pub use channel::{ChannelMetadata, TelemetryChannel};
pub use error::TelemetryError;
pub use row::TelemetryRow;
pub use sample::{TelemetryRingBuffer, TelemetrySample, TelemetrySink};
pub use schema::TelemetrySchema;
pub use table::TelemetryTable;
pub use value::{TelemetryDatum, TelemetryValue, TelemetryValueKind};

/// Telemetry schema version written into archive metadata.
pub const TELEMETRY_SCHEMA_VERSION: &str = "openbmp.telemetry.v1";

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::{ChannelId, SimTime, StepIndex};

    fn fixture() -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", Some("ECI"))
                .unwrap();
        let mode =
            TelemetryChannel::<i64>::new(ChannelId::new(2), "mode", "1", None::<String>).unwrap();
        let valid =
            TelemetryChannel::<bool>::new(ChannelId::new(3), "valid", "1", None::<String>).unwrap();
        let label =
            TelemetryChannel::<String>::new(ChannelId::new(4), "label", "1", None::<String>)
                .unwrap();

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
    fn csv_export_is_stable_and_ordered() {
        let table = fixture();
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
    fn json_export_is_stable_across_reruns() {
        let table = fixture();
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
    fn json_escapes_text_values() {
        let label =
            TelemetryChannel::<String>::new(ChannelId::new(1), "label", "1", None::<String>)
                .unwrap();
        let schema = TelemetrySchema::new(vec![label.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut row = TelemetryRow::new(SimTime::ZERO, StepIndex::ZERO).unwrap();
        row.insert(&label, "with \"quotes\" and \n newline".to_owned())
            .unwrap();
        table.push_row(row).unwrap();

        let mut bytes = Vec::new();
        table.write_json(&mut bytes).unwrap();
        let json = String::from_utf8(bytes).unwrap();
        assert!(json.contains(r#""label":"with \"quotes\" and \n newline""#));
    }

    #[test]
    fn parquet_export_is_byte_stable() {
        let table = fixture();
        let first = table.to_parquet_bytes().unwrap();
        let second = table.to_parquet_bytes().unwrap();
        assert_eq!(first, second);
        assert!(!first.is_empty());
    }

    #[test]
    fn table_rejects_type_mismatch() {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let same_id_int =
            TelemetryChannel::<i64>::new(ChannelId::new(1), "mode", "1", None::<String>).unwrap();

        let schema = TelemetrySchema::new(vec![altitude.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut row = TelemetryRow::new(SimTime::ZERO, StepIndex::ZERO).unwrap();
        row.insert(&same_id_int, 7_i64).unwrap();

        let err = table.push_row(row).unwrap_err();
        assert!(matches!(err, TelemetryError::TypeMismatch { .. }));
    }

    #[test]
    fn table_rejects_unknown_channel() {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let other =
            TelemetryChannel::<f64>::new(ChannelId::new(99), "other_m", "m", None::<String>)
                .unwrap();

        let schema = TelemetrySchema::new(vec![altitude.metadata().clone()]).unwrap();
        let mut table = TelemetryTable::new(schema);
        let mut row = TelemetryRow::new(SimTime::ZERO, StepIndex::ZERO).unwrap();
        row.insert(&other, 1.0).unwrap();

        let err = table.push_row(row).unwrap_err();
        assert!(matches!(err, TelemetryError::UnknownChannel { .. }));
    }

    #[test]
    fn row_rejects_duplicate_value() {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let mut row = TelemetryRow::new(SimTime::ZERO, StepIndex::ZERO).unwrap();
        row.insert(&altitude, 1.0).unwrap();
        let err = row.insert(&altitude, 2.0).unwrap_err();
        assert!(matches!(err, TelemetryError::DuplicateRowValue { .. }));
    }
}
