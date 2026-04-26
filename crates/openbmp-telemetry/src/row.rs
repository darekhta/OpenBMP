//! One archive row indexed by channel id.

use std::collections::BTreeMap;

use openbmp_core::{ChannelId, SimTime, StepIndex};

use crate::channel::TelemetryChannel;
use crate::error::TelemetryError;
use crate::value::{TelemetryDatum, TelemetryValue};

/// One telemetry archive row.
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetryRow {
    /// Simulation time for the row.
    pub time: SimTime,
    /// Simulation step for the row.
    pub step: StepIndex,
    values: BTreeMap<ChannelId, TelemetryValue>,
}

impl TelemetryRow {
    /// Create an empty row.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidTime`] when `time` is invalid.
    pub fn new(time: SimTime, step: StepIndex) -> Result<Self, TelemetryError> {
        if !time.is_valid() {
            return Err(TelemetryError::InvalidTime {
                seconds: time.as_seconds(),
            });
        }
        Ok(Self {
            time,
            step,
            values: BTreeMap::new(),
        })
    }

    /// Insert a typed channel value.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::DuplicateRowValue`] when the channel
    /// already has a value in this row, or
    /// [`TelemetryError::NonFiniteValue`] for non-finite floats.
    pub fn insert<T: TelemetryDatum>(
        &mut self,
        channel: &TelemetryChannel<T>,
        value: T,
    ) -> Result<(), TelemetryError> {
        let id = channel.id();
        if self.values.contains_key(&id) {
            return Err(TelemetryError::DuplicateRowValue { id: id.value() });
        }
        self.values
            .insert(id, value.into_value().require_valid_for(id)?);
        Ok(())
    }

    /// Value for a channel id, if present.
    #[must_use]
    pub fn get(&self, channel: ChannelId) -> Option<&TelemetryValue> {
        self.values.get(&channel)
    }

    /// Iterate (channel id, value) pairs in id order.
    pub(crate) fn iter(&self) -> impl Iterator<Item = (&ChannelId, &TelemetryValue)> {
        self.values.iter()
    }
}
