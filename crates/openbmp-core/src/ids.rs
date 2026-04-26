//! Stable identifiers for telemetry channels, models, and scenarios.
//!
//! Identifiers are integers carried as opaque newtypes. Higher-level
//! crates derive identifiers from stable names; this crate only owns
//! the type.

/// Stable identifier for a telemetry channel.
///
/// Used by [`crate::DeterministicRng::for_channel`] as part of the
/// per-step seed mix.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct ChannelId(u64);

impl ChannelId {
    /// Construct a [`ChannelId`] from an integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for a model instance.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct ModelId(u64);

impl ModelId {
    /// Construct a [`ModelId`] from an integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for a scenario instance.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct ScenarioId(u64);

impl ScenarioId {
    /// Construct a [`ScenarioId`] from an integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn id_round_trip() {
        assert_eq!(ChannelId::new(42).value(), 42);
        assert_eq!(ModelId::new(99).value(), 99);
        assert_eq!(ScenarioId::new(7).value(), 7);
    }

    #[test]
    fn ids_default_to_zero() {
        assert_eq!(ChannelId::default().value(), 0);
        assert_eq!(ModelId::default().value(), 0);
        assert_eq!(ScenarioId::default().value(), 0);
    }

    #[test]
    fn distinct_id_types_are_not_interchangeable() {
        // Compile-time guard: this would not compile if the types
        // collapsed. We only assert that constructing them works.
        let _c: ChannelId = ChannelId::new(1);
        let _m: ModelId = ModelId::new(1);
        let _s: ScenarioId = ScenarioId::new(1);
    }
}
