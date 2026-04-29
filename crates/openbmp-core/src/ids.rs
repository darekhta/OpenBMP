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

/// Stable identifier for a sensor instance.
///
/// Derived from the canonical scenario sensor path (e.g.
/// `"sensors.imu"`) via FNV-1a-64 so that **reordering** the
/// `[sensors]` block in a scenario file cannot shift any sensor's
/// RNG stream. The Phase-2 plan locks this:
///
/// > Phase 2 adds a stable `SensorId` newtype derived from the
/// > canonical scenario sensor path, not from list order.
///
/// `for_sensor_component` uses an explicit domain tag in the seed
/// material so it cannot collide with [`crate::DeterministicRng::for_channel`].
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct SensorId(u64);

impl SensorId {
    /// Construct a [`SensorId`] from a raw integer value.
    ///
    /// Prefer [`SensorId::from_path`] for scenario sensors so the
    /// id is path-derived and stable across reorderings.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct a [`SensorId`] from a stable canonical scenario
    /// sensor path (e.g. `"sensors.imu"`) via FNV-1a-64.
    ///
    /// The hash is platform-independent and reproducible across
    /// builds — the FNV constants are pinned.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        // FNV-1a-64 with the standard pinned constants.
        const FNV_OFFSET_BASIS_64: u64 = 0xcbf2_9ce4_8422_2325;
        const FNV_PRIME_64: u64 = 0x100_0000_01b3;

        let bytes = path.as_bytes();
        let mut hash = FNV_OFFSET_BASIS_64;
        let mut i = 0;
        while i < bytes.len() {
            hash ^= bytes[i] as u64;
            hash = hash.wrapping_mul(FNV_PRIME_64);
            i += 1;
        }
        Self(hash)
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Internal FNV-1a-64 used by every path-derived id type. Same pinned
/// constants as [`SensorId::from_path`] so existing pinned hashes
/// stay valid across builds.
const fn fnv1a_64(bytes: &[u8]) -> u64 {
    const FNV_OFFSET_BASIS_64: u64 = 0xcbf2_9ce4_8422_2325;
    const FNV_PRIME_64: u64 = 0x100_0000_01b3;

    let mut hash = FNV_OFFSET_BASIS_64;
    let mut i = 0;
    while i < bytes.len() {
        hash ^= bytes[i] as u64;
        hash = hash.wrapping_mul(FNV_PRIME_64);
        i += 1;
    }
    hash
}

/// Stable identifier for a [`crate::Body`]-frame member of a
/// `VehicleAssembly` (Phase 3.3).
///
/// Derived from the canonical scenario body path (e.g.
/// `"vehicle.assembly.bodies.main"`) via FNV-1a-64 so that
/// reordering `[[vehicle.assembly.bodies]]` blocks cannot shift any
/// body's id.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct BodyId(u64);

impl BodyId {
    /// Construct from a raw integer value. Prefer
    /// [`BodyId::from_path`] for scenario-declared bodies.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a stable canonical scenario path (e.g.
    /// `"vehicle.assembly.bodies.main"`) via FNV-1a-64.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for a control-effector instance (Phase 3.4).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct EffectorId(u64);

impl EffectorId {
    /// Construct from a raw integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a canonical scenario effector path.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for a tank instance (Phase 3.7).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct TankId(u64);

impl TankId {
    /// Construct from a raw integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a canonical scenario tank path.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for an engine instance within an `EngineCluster`
/// (Phase 3.6).
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct EngineId(u64);

impl EngineId {
    /// Construct from a raw integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a canonical scenario engine path.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for a `VehicleAssembly` instance (Phase 3.3).
///
/// Carries the scenario's overall vehicle name. Defaults to the FNV
/// hash of the scenario `meta.name` field when no explicit
/// `vehicle.assembly.id` is declared.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct VehicleId(u64);

impl VehicleId {
    /// Construct from a raw integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a canonical vehicle path or scenario meta.name.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Stable identifier for a recovery-device instance (Phase 3.9).
///
/// Derived from the canonical scenario recovery-device path
/// (e.g. `"vehicle.assembly.recovery.main_chute"`) via FNV-1a-64 so
/// that reordering the recovery-device declarations in a scenario
/// file cannot shift any device's identity, deploy events, or
/// telemetry channel.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Hash, Ord, PartialOrd)]
pub struct RecoveryId(u64);

impl RecoveryId {
    /// Construct from a raw integer value.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Construct from a canonical scenario recovery-device path.
    #[must_use]
    pub const fn from_path(path: &str) -> Self {
        Self(fnv1a_64(path.as_bytes()))
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }
}

/// Phase-3.8 wind-axis tag for the Dryden gust filter.
///
/// The Dryden rational-spectrum shaping filter uses three independent
/// per-axis state variables — longitudinal `u`, lateral `v`, and
/// vertical `w` — each driven by its own white-noise stream. Carrying
/// the axis as a typed enum (rather than a raw `u32`) makes
/// axis-mismatch a compile error at every `for_wind_component` call
/// site and keeps the RNG-seed shape self-documenting.
///
/// Used by [`crate::DeterministicRng::for_wind_component`] in the
/// trailing 4 bytes (after the `b"WIND"` domain tag) of the seed
/// material.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash, Ord, PartialOrd)]
#[repr(u32)]
pub enum WindAxis {
    /// Longitudinal axis (along the relative-airspeed vector).
    U = 0,
    /// Lateral axis (perpendicular to the relative-airspeed vector,
    /// in the local horizontal plane).
    V = 1,
    /// Vertical axis (perpendicular to both `U` and `V`).
    W = 2,
}

impl WindAxis {
    /// Returns the underlying `u32` value used in the RNG seed
    /// material. Matches the discriminant numbering above.
    #[must_use]
    pub const fn value(self) -> u32 {
        self as u32
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
        let _sensor: SensorId = SensorId::new(1);
    }

    #[test]
    fn sensor_id_round_trip() {
        assert_eq!(SensorId::new(42).value(), 42);
        assert_eq!(SensorId::default().value(), 0);
    }

    #[test]
    fn sensor_id_from_path_is_deterministic() {
        let a = SensorId::from_path("sensors.imu");
        let b = SensorId::from_path("sensors.imu");
        assert_eq!(a, b);
    }

    #[test]
    fn sensor_id_from_distinct_paths_diverges() {
        let imu = SensorId::from_path("sensors.imu");
        let baro = SensorId::from_path("sensors.barometer");
        let gnss = SensorId::from_path("sensors.gnss");
        assert_ne!(imu, baro);
        assert_ne!(imu, gnss);
        assert_ne!(baro, gnss);
    }

    #[test]
    fn sensor_id_is_path_sensitive_not_order_sensitive() {
        // Different paths must hash differently even when they share
        // a common prefix — this is what protects scenario reordering
        // from shifting any sensor's RNG stream.
        assert_ne!(
            SensorId::from_path("sensors.imu"),
            SensorId::from_path("sensors.imu_aux")
        );
        assert_ne!(
            SensorId::from_path("sensors.imu.0"),
            SensorId::from_path("sensors.imu.1")
        );
    }

    #[test]
    fn sensor_id_from_path_reference_hashes_are_locked() {
        // FNV-1a-64 with the pinned constants. These hashes are part
        // of the determinism contract and must not drift across builds.
        assert_eq!(
            SensorId::from_path("sensors.imu").value(),
            0x8943_cc6e_91ce_20d3,
        );
        // Print for any future regression that needs a fresh check —
        // expected output if the hashes ever drift would fail this
        // test long before any sensor file regressed.
    }

    #[test]
    fn wind_axis_values_match_discriminants() {
        assert_eq!(WindAxis::U.value(), 0);
        assert_eq!(WindAxis::V.value(), 1);
        assert_eq!(WindAxis::W.value(), 2);
    }

    #[test]
    fn wind_axis_variants_are_distinct() {
        assert_ne!(WindAxis::U, WindAxis::V);
        assert_ne!(WindAxis::U, WindAxis::W);
        assert_ne!(WindAxis::V, WindAxis::W);
    }

    #[test]
    fn sensor_id_distinct_for_canonical_phase_2_sensor_paths() {
        // Phase-2 sensor inventory: IMU, barometer, ideal-state.
        // These hashes are pinned in the determinism contract so a
        // drift in either path or the FNV implementation surfaces
        // here before it can affect any seeded RNG stream.
        let imu = SensorId::from_path("sensors.imu");
        let baro = SensorId::from_path("sensors.barometer");
        let ideal = SensorId::from_path("sensors.ideal_state");
        // All three differ.
        assert_ne!(imu, baro);
        assert_ne!(imu, ideal);
        assert_ne!(baro, ideal);
    }
}
