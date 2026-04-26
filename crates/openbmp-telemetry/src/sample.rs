//! Telemetry samples, the [`TelemetrySink`] trait, and the bounded
//! [`TelemetryRingBuffer`] sink used for current-value access during a
//! simulation run.

use std::collections::VecDeque;

use openbmp_core::{ChannelId, SimTime, StepIndex};

use crate::error::TelemetryError;
use crate::value::TelemetryValue;

/// One typed channel sample at one simulation step.
#[derive(Clone, Debug, PartialEq)]
pub struct TelemetrySample {
    /// Simulation time.
    pub time: SimTime,
    /// Simulation step index.
    pub step: StepIndex,
    /// Channel id.
    pub channel: ChannelId,
    /// Sample value.
    pub value: TelemetryValue,
}

impl TelemetrySample {
    /// Construct a validated telemetry sample.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidTime`] for invalid time and
    /// [`TelemetryError::NonFiniteValue`] for non-finite float values.
    pub fn new(
        time: SimTime,
        step: StepIndex,
        channel: ChannelId,
        value: TelemetryValue,
    ) -> Result<Self, TelemetryError> {
        if !time.is_valid() {
            return Err(TelemetryError::InvalidTime {
                seconds: time.as_seconds(),
            });
        }
        Ok(Self {
            time,
            step,
            channel,
            value: value.require_valid_for(channel)?,
        })
    }
}

/// Sink interface for receiving telemetry samples.
pub trait TelemetrySink {
    /// Record one telemetry sample.
    ///
    /// # Errors
    ///
    /// Implementations return [`TelemetryError`] when the sample cannot
    /// be accepted.
    fn push_sample(&mut self, sample: TelemetrySample) -> Result<(), TelemetryError>;
}

/// Bounded ring buffer storing the most recent telemetry samples.
#[derive(Clone, Debug)]
pub struct TelemetryRingBuffer {
    capacity: usize,
    samples: VecDeque<TelemetrySample>,
}

impl TelemetryRingBuffer {
    /// Create a ring buffer with a fixed sample capacity.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::InvalidCapacity`] when `capacity` is
    /// zero.
    pub fn new(capacity: usize) -> Result<Self, TelemetryError> {
        if capacity == 0 {
            return Err(TelemetryError::InvalidCapacity);
        }
        Ok(Self {
            capacity,
            samples: VecDeque::with_capacity(capacity),
        })
    }

    /// Configured capacity.
    #[must_use]
    pub const fn capacity(&self) -> usize {
        self.capacity
    }

    /// Number of currently retained samples.
    #[must_use]
    pub fn len(&self) -> usize {
        self.samples.len()
    }

    /// Whether the buffer is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.samples.is_empty()
    }

    /// Iterate samples from oldest to newest.
    pub fn iter(&self) -> impl Iterator<Item = &TelemetrySample> {
        self.samples.iter()
    }

    /// Latest retained sample for a channel.
    #[must_use]
    pub fn latest_for(&self, channel: ChannelId) -> Option<&TelemetrySample> {
        self.samples
            .iter()
            .rev()
            .find(|sample| sample.channel == channel)
    }
}

impl TelemetrySink for TelemetryRingBuffer {
    fn push_sample(&mut self, sample: TelemetrySample) -> Result<(), TelemetryError> {
        if self.samples.len() == self.capacity {
            self.samples.pop_front();
        }
        self.samples.push_back(sample);
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::channel::TelemetryChannel;

    #[test]
    fn ring_buffer_retains_latest_samples() {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", Some("ECI"))
                .unwrap();
        let mut buffer = TelemetryRingBuffer::new(2).unwrap();
        for (step, time_s, value) in [(0_u64, 0.0_f64, 1.0_f64), (1, 1.0, 2.0), (2, 2.0, 3.0)] {
            buffer
                .push_sample(
                    altitude
                        .sample(SimTime::from_seconds(time_s), StepIndex::new(step), value)
                        .unwrap(),
                )
                .unwrap();
        }

        assert_eq!(buffer.len(), 2);
        assert_eq!(
            buffer.latest_for(altitude.id()).unwrap().value,
            TelemetryValue::Float64(3.0)
        );
        assert_eq!(buffer.iter().next().unwrap().step, StepIndex::new(1));
    }

    #[test]
    fn rejects_non_finite_float_values() {
        let altitude =
            TelemetryChannel::<f64>::new(ChannelId::new(1), "altitude_m", "m", Some("ECI"))
                .unwrap();
        let err = altitude
            .sample(SimTime::ZERO, StepIndex::ZERO, f64::NAN)
            .unwrap_err();
        assert!(matches!(err, TelemetryError::NonFiniteValue { .. }));
    }

    #[test]
    fn ring_buffer_rejects_zero_capacity() {
        let err = TelemetryRingBuffer::new(0).unwrap_err();
        assert!(matches!(err, TelemetryError::InvalidCapacity));
    }
}
