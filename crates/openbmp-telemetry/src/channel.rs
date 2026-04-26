//! Typed channels and channel metadata.

use std::marker::PhantomData;

use openbmp_core::{ChannelId, SimTime, StepIndex};

use crate::error::TelemetryError;
use crate::sample::TelemetrySample;
use crate::value::{TelemetryDatum, TelemetryValueKind};

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
        let name = require_non_empty("name", name.into())?;
        let unit = require_non_empty("unit", unit.into())?;
        let frame = match frame {
            Some(value) => Some(require_non_empty("frame", value.into())?),
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

fn require_non_empty(field: &'static str, value: String) -> Result<String, TelemetryError> {
    if value.trim().is_empty() {
        Err(TelemetryError::EmptyField { field })
    } else {
        Ok(value)
    }
}
