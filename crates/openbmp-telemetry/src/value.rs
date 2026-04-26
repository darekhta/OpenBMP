//! Primitive telemetry value type and the [`TelemetryDatum`] trait that
//! lifts native Rust scalars into typed channel samples.

use std::fmt;

use openbmp_core::ChannelId;

use crate::error::TelemetryError;

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
    /// Canonical schema label.
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

impl fmt::Display for TelemetryValueKind {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
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

/// Trait implemented by primitive types that can be recorded on typed
/// channels.
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
