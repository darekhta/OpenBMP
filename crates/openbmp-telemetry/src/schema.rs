//! Telemetry archive schema: an ordered list of [`ChannelMetadata`]
//! whose ids and names are unique. The schema fixes archive column
//! order, which is part of the byte-stable archive contract.
//!
//! The schema also carries optional table-level metadata as an
//! ordered `BTreeMap<String, String>`. Telemetry consumers (Parquet
//! writer in particular) merge this map into the archive's
//! schema-level metadata. The runner uses it to record
//! resolved-file SHA-256 digests in the Parquet header — the
//! determinism gate can then verify pin matches at replay time
//! without re-loading the scenario.

use std::collections::{BTreeMap, BTreeSet};

use openbmp_core::ChannelId;

use crate::channel::ChannelMetadata;
use crate::error::TelemetryError;

/// Deterministic telemetry archive schema.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct TelemetrySchema {
    channels: Vec<ChannelMetadata>,
    metadata: BTreeMap<String, String>,
}

impl TelemetrySchema {
    /// Create a telemetry schema from channels in archive-column order.
    ///
    /// The schema starts with no table-level metadata; use
    /// [`Self::with_metadata`] to attach SHA-256 pins or other
    /// header keys.
    ///
    /// # Errors
    ///
    /// Returns [`TelemetryError::DuplicateChannelId`] or
    /// [`TelemetryError::DuplicateChannelName`] when the schema is not
    /// deterministic.
    pub fn new(channels: Vec<ChannelMetadata>) -> Result<Self, TelemetryError> {
        let mut ids = BTreeSet::new();
        let mut names = BTreeSet::new();
        for channel in &channels {
            if !ids.insert(channel.id) {
                return Err(TelemetryError::DuplicateChannelId {
                    id: channel.id.value(),
                });
            }
            if !names.insert(channel.name.clone()) {
                return Err(TelemetryError::DuplicateChannelName {
                    name: channel.name.clone(),
                });
            }
        }
        Ok(Self {
            channels,
            metadata: BTreeMap::new(),
        })
    }

    /// Replace the table-level metadata. The map is persisted in
    /// archive iteration order via [`BTreeMap`], so two schemas with
    /// the same key-value pairs serialise identically.
    #[must_use]
    pub fn with_metadata(mut self, metadata: BTreeMap<String, String>) -> Self {
        self.metadata = metadata;
        self
    }

    /// Channels in archive-column order.
    #[must_use]
    pub fn channels(&self) -> &[ChannelMetadata] {
        &self.channels
    }

    /// Table-level metadata in deterministic key order.
    #[must_use]
    pub fn metadata(&self) -> &BTreeMap<String, String> {
        &self.metadata
    }

    /// Resolve a channel by id.
    #[must_use]
    pub fn channel(&self, id: ChannelId) -> Option<&ChannelMetadata> {
        self.channels.iter().find(|channel| channel.id == id)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::value::TelemetryValueKind;

    fn meta(id: u64, name: &str) -> ChannelMetadata {
        ChannelMetadata::new(
            ChannelId::new(id),
            name,
            "m",
            None::<String>,
            TelemetryValueKind::Float64,
        )
        .unwrap()
    }

    #[test]
    fn rejects_duplicate_names() {
        let err = TelemetrySchema::new(vec![meta(1, "same"), meta(2, "same")]).unwrap_err();
        assert!(matches!(err, TelemetryError::DuplicateChannelName { .. }));
    }

    #[test]
    fn rejects_duplicate_ids() {
        let err = TelemetrySchema::new(vec![meta(1, "a"), meta(1, "b")]).unwrap_err();
        assert!(matches!(err, TelemetryError::DuplicateChannelId { .. }));
    }

    #[test]
    fn preserves_channel_order() {
        let schema = TelemetrySchema::new(vec![meta(2, "second"), meta(1, "first")]).unwrap();
        assert_eq!(schema.channels()[0].name, "second");
        assert_eq!(schema.channels()[1].name, "first");
    }
}
