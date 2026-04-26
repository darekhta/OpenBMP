//! Error type for the telemetry crate.

use thiserror::Error;

use crate::value::TelemetryValueKind;

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
    Io(#[from] std::io::Error),
    /// Arrow failed while building a record batch.
    #[error("arrow error")]
    Arrow(#[from] arrow::error::ArrowError),
    /// Parquet failed while writing an archive.
    #[error("parquet error")]
    Parquet(#[from] parquet::errors::ParquetError),
}
