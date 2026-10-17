//! Deterministic Open MCT catalog and replay-message shapes.
//!
//! This module contains the file/message substrate for the host-side Open MCT
//! example. It does not start a WebSocket server; the service boundary remains
//! an explicit example-layer concern.

use std::collections::BTreeMap;

use openbmp_core::ChannelId;
use openbmp_telemetry::{TelemetryTable, TelemetryValue};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::require_non_empty;

/// Open MCT shim artifact schema version.
pub const OPENMCT_SCHEMA_VERSION: &str = "openbmp.rundata.openmct.v1";

/// Errors returned by Open MCT shim helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum OpenMctError {
    /// JSON serialization or parsing failed.
    #[error("Open MCT JSON error: {message}")]
    Json {
        /// Error message.
        message: String,
    },
    /// A required string field was empty after trimming.
    #[error("{field} must not be empty")]
    EmptyField {
        /// Field name.
        field: &'static str,
    },
    /// The requested telemetry channel was absent.
    #[error("unknown telemetry channel {channel}")]
    UnknownChannel {
        /// Channel name.
        channel: String,
    },
    /// A row did not contain the requested telemetry channel.
    #[error("missing sample for channel {channel} at row {row}")]
    MissingSample {
        /// Channel name.
        channel: String,
        /// Row index.
        row: usize,
    },
    /// A sample was not numeric.
    #[error("channel {channel} has non-numeric sample at row {row}")]
    NonNumericSample {
        /// Channel name.
        channel: String,
        /// Row index.
        row: usize,
    },
    /// At least one replay channel is required.
    #[error("Open MCT replay requires at least one channel")]
    EmptyChannels,
    /// Replay speed must be finite and positive.
    #[error("replay speed must be finite and positive")]
    InvalidReplaySpeed,
}

impl From<serde_json::Error> for OpenMctError {
    fn from(error: serde_json::Error) -> Self {
        Self::Json {
            message: error.to_string(),
        }
    }
}

/// Static Open MCT catalog document.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OpenMctCatalog {
    /// Artifact schema version.
    pub schema_version: String,
    /// Namespace used for Open MCT identifiers.
    pub namespace: String,
    /// Catalog objects in telemetry schema order.
    pub objects: Vec<OpenMctCatalogObject>,
}

impl OpenMctCatalog {
    /// Serialize as deterministic JSON bytes.
    ///
    /// # Errors
    ///
    /// Returns [`OpenMctError::Json`] if serialization fails.
    pub fn to_json_bytes(&self) -> Result<Vec<u8>, OpenMctError> {
        Ok(serde_json::to_vec(self)?)
    }

    /// Parse a JSON catalog.
    ///
    /// # Errors
    ///
    /// Returns [`OpenMctError::Json`] if parsing fails.
    pub fn from_json_bytes(bytes: &[u8]) -> Result<Self, OpenMctError> {
        Ok(serde_json::from_slice(bytes)?)
    }
}

/// One Open MCT catalog object.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OpenMctCatalogObject {
    /// Fully qualified object key.
    pub key: String,
    /// Display name.
    pub name: String,
    /// Open MCT object type.
    pub object_type: String,
    /// Telemetry metadata.
    pub telemetry: OpenMctTelemetryMetadata,
}

/// Open MCT telemetry metadata.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OpenMctTelemetryMetadata {
    /// Value descriptors.
    pub values: Vec<OpenMctValueMetadata>,
}

/// Open MCT value metadata.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OpenMctValueMetadata {
    /// Value key.
    pub key: String,
    /// Display name.
    pub name: String,
    /// Engineering units.
    pub units: String,
    /// Value format hint.
    pub format: String,
}

/// One deterministic replay frame.
#[derive(Clone, Debug, Deserialize, PartialEq, Serialize)]
#[serde(deny_unknown_fields)]
pub struct OpenMctReplayFrame {
    /// Simulation timestamp in seconds.
    pub timestamp_s: f64,
    /// Replay timestamp after applying the declared replay speed.
    pub replay_time_s: f64,
    /// Channel values keyed by channel name.
    pub values: BTreeMap<String, f64>,
}

/// Generate a static Open MCT catalog for every numeric telemetry channel.
///
/// # Errors
///
/// Returns [`OpenMctError`] when `namespace` is empty.
pub fn generate_openmct_catalog(
    table: &TelemetryTable,
    namespace: impl Into<String>,
) -> Result<OpenMctCatalog, OpenMctError> {
    let namespace = require_string("openmct.namespace", namespace.into())?;
    let objects = table
        .schema()
        .channels()
        .iter()
        .filter(|channel| channel.value_kind.is_numeric())
        .map(|channel| OpenMctCatalogObject {
            key: format!("{namespace}.{}", channel.name),
            name: channel.name.clone(),
            object_type: "telemetry".to_owned(),
            telemetry: OpenMctTelemetryMetadata {
                values: vec![OpenMctValueMetadata {
                    key: "value".to_owned(),
                    name: channel.name.clone(),
                    units: channel.unit.clone(),
                    format: "float".to_owned(),
                }],
            },
        })
        .collect();
    Ok(OpenMctCatalog {
        schema_version: OPENMCT_SCHEMA_VERSION.to_owned(),
        namespace,
        objects,
    })
}

/// Generate deterministic replay frames for declared numeric channels.
///
/// # Errors
///
/// Returns [`OpenMctError`] when no channels are declared, replay speed is not
/// finite and positive, a channel is unknown, a sample is missing, or a sample
/// is non-numeric.
pub fn generate_openmct_replay_frames(
    table: &TelemetryTable,
    channels: &[&str],
    replay_speed: f64,
) -> Result<Vec<OpenMctReplayFrame>, OpenMctError> {
    if channels.is_empty() {
        return Err(OpenMctError::EmptyChannels);
    }
    if !replay_speed.is_finite() || replay_speed <= 0.0 {
        return Err(OpenMctError::InvalidReplaySpeed);
    }
    let channel_ids = channels
        .iter()
        .map(|channel| Ok(((*channel).to_owned(), channel_id(table, channel)?)))
        .collect::<Result<Vec<_>, OpenMctError>>()?;
    let mut frames = Vec::with_capacity(table.rows().len());
    for (row_index, row) in table.rows().iter().enumerate() {
        let mut values = BTreeMap::new();
        for (channel, channel_id) in &channel_ids {
            let Some(value) = row.get(*channel_id) else {
                return Err(OpenMctError::MissingSample {
                    channel: channel.clone(),
                    row: row_index,
                });
            };
            let Some(value) = numeric_value(value) else {
                return Err(OpenMctError::NonNumericSample {
                    channel: channel.clone(),
                    row: row_index,
                });
            };
            values.insert(channel.clone(), value);
        }
        let timestamp_s = row.time.as_seconds();
        frames.push(OpenMctReplayFrame {
            timestamp_s,
            replay_time_s: timestamp_s / replay_speed,
            values,
        });
    }
    Ok(frames)
}

/// Serialize replay frames as deterministic JSON bytes.
///
/// # Errors
///
/// Returns [`OpenMctError::Json`] if serialization fails.
pub fn openmct_replay_frames_to_json_bytes(
    frames: &[OpenMctReplayFrame],
) -> Result<Vec<u8>, OpenMctError> {
    Ok(serde_json::to_vec(frames)?)
}

fn channel_id(table: &TelemetryTable, channel: &str) -> Result<ChannelId, OpenMctError> {
    table
        .schema()
        .channels()
        .iter()
        .find(|candidate| candidate.name == channel)
        .map(|candidate| candidate.id)
        .ok_or_else(|| OpenMctError::UnknownChannel {
            channel: channel.to_owned(),
        })
}

fn numeric_value(value: &TelemetryValue) -> Option<f64> {
    match value {
        TelemetryValue::Float64(value) => Some(*value),
        TelemetryValue::Int64(value) => Some(*value as f64),
        TelemetryValue::Bool(_) | TelemetryValue::Text(_) => None,
    }
}

fn require_string(field: &'static str, value: String) -> Result<String, OpenMctError> {
    require_non_empty(field, value).map_err(|_| OpenMctError::EmptyField { field })
}

trait NumericKind {
    fn is_numeric(&self) -> bool;
}

impl NumericKind for openbmp_telemetry::TelemetryValueKind {
    fn is_numeric(&self) -> bool {
        matches!(self, Self::Float64 | Self::Int64)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_core::{ChannelId, SimTime, StepIndex};
    use openbmp_telemetry::{TelemetryChannel, TelemetryRow, TelemetrySchema, TelemetryTable};

    fn table() -> TelemetryTable {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", None::<String>)
                .unwrap();
        let mode =
            TelemetryChannel::<i64>::new(ChannelId::new(2), "mode", "1", None::<String>).unwrap();
        let valid =
            TelemetryChannel::<bool>::new(ChannelId::new(3), "valid", "1", None::<String>).unwrap();
        let schema = TelemetrySchema::new(vec![
            altitude.metadata().clone(),
            mode.metadata().clone(),
            valid.metadata().clone(),
        ])
        .unwrap();
        let mut table = TelemetryTable::new(schema);
        for index in 0..2 {
            let mut row = TelemetryRow::new(
                SimTime::from_seconds(index as f64),
                StepIndex::new(index as u64),
            )
            .unwrap();
            row.insert(&altitude, index as f64 + 10.0).unwrap();
            row.insert(&mode, index as i64).unwrap();
            row.insert(&valid, index == 0).unwrap();
            table.push_row(row).unwrap();
        }
        table
    }

    #[test]
    fn openmct_catalog_json_is_byte_stable() {
        let catalog = generate_openmct_catalog(&table(), "openbmp").unwrap();
        let decoded = OpenMctCatalog::from_json_bytes(&catalog.to_json_bytes().unwrap()).unwrap();
        let golden = include_str!("../tests/fixtures/openmct-catalog.json");
        let golden = golden.strip_suffix('\n').unwrap_or(golden);

        assert_eq!(decoded, catalog);
        assert_eq!(
            std::str::from_utf8(&catalog.to_json_bytes().unwrap()).unwrap(),
            golden
        );
    }

    #[test]
    fn openmct_replay_frames_are_byte_stable() {
        let frames =
            generate_openmct_replay_frames(&table(), &["altitude_m", "mode"], 2.0).unwrap();
        let bytes = openmct_replay_frames_to_json_bytes(&frames).unwrap();
        let golden = include_str!("../tests/fixtures/openmct-replay.json");
        let golden = golden.strip_suffix('\n').unwrap_or(golden);

        assert_eq!(frames.len(), 2);
        assert_eq!(frames[1].timestamp_s, 1.0);
        assert_eq!(frames[1].replay_time_s, 0.5);
        assert_eq!(std::str::from_utf8(&bytes).unwrap(), golden);
    }

    #[test]
    fn openmct_helpers_fail_closed_on_bad_inputs() {
        let unknown = generate_openmct_replay_frames(&table(), &["missing"], 1.0).unwrap_err();
        assert!(matches!(unknown, OpenMctError::UnknownChannel { .. }));

        let nonnumeric = generate_openmct_replay_frames(&table(), &["valid"], 1.0).unwrap_err();
        assert!(matches!(nonnumeric, OpenMctError::NonNumericSample { .. }));

        let empty = generate_openmct_replay_frames(&table(), &[], 1.0).unwrap_err();
        assert_eq!(empty, OpenMctError::EmptyChannels);

        let bad_speed = generate_openmct_replay_frames(&table(), &["altitude_m"], 0.0).unwrap_err();
        assert_eq!(bad_speed, OpenMctError::InvalidReplaySpeed);
    }
}
