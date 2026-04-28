//! Deterministic pseudo-random number generation.
//!
//! [`DeterministicRng`] wraps `rand_chacha::ChaCha8Rng`, the
//! workspace-canonical bit-stable RNG. Constructors derive the seed
//! from explicit `(scenario_seed, step_index, channel_id)` triples so
//! the kernel can produce the same byte stream across reruns.
//!
//! No type in this module reads from system entropy.

use rand::rand_core::Infallible;
use rand::{Rng as _, SeedableRng, TryRng};
use rand_chacha::ChaCha8Rng;

use crate::ids::{ChannelId, EffectorId, SensorId, WindAxis};
use crate::time::StepIndex;

/// Domain tag bytes placed in the trailing 4 bytes of the RNG seed
/// for [`DeterministicRng::for_sensor_component`]. Guarantees that a
/// sensor-component stream can never collide with a `for_channel`
/// stream — `for_channel` always leaves bytes [24..32] zero, while
/// `for_sensor_component` writes a non-zero domain tag in [28..32].
const SENSOR_COMPONENT_DOMAIN_TAG: [u8; 4] = *b"SENS";

/// Domain tag for [`DeterministicRng::for_effector_component`].
/// Distinct from [`SENSOR_COMPONENT_DOMAIN_TAG`] so effector and
/// sensor RNG streams never collide even if their integer payloads
/// happen to overlap.
const EFFECTOR_COMPONENT_DOMAIN_TAG: [u8; 4] = *b"EFFC";

/// Domain tag for [`DeterministicRng::for_wind_component`].
/// Distinct from the other domain tags so the Dryden gust filter's
/// per-axis noise streams never collide with channel / sensor /
/// effector streams even if integer payloads happen to overlap.
const WIND_COMPONENT_DOMAIN_TAG: [u8; 4] = *b"WIND";

/// Deterministic pseudo-random number generator.
///
/// Implements [`rand::Rng`] and is therefore usable anywhere the
/// `rand` ecosystem expects an RNG. The same seed always produces the
/// same byte stream.
#[derive(Debug, Clone)]
pub struct DeterministicRng {
    inner: ChaCha8Rng,
}

impl DeterministicRng {
    /// Construct from a 32-byte raw seed.
    #[must_use]
    pub fn from_raw_seed(seed: [u8; 32]) -> Self {
        Self {
            inner: ChaCha8Rng::from_seed(seed),
        }
    }

    /// Construct from a single 64-bit seed.
    ///
    /// The 64-bit seed is little-endian-packed into the first 8 bytes
    /// of the ChaCha8 seed; remaining bytes are zero.
    #[must_use]
    pub fn from_seed(seed: u64) -> Self {
        let mut bytes = [0u8; 32];
        bytes[0..8].copy_from_slice(&seed.to_le_bytes());
        Self::from_raw_seed(bytes)
    }

    /// Construct a per-channel deterministic stream.
    ///
    /// The seed is derived from the canonical
    /// `(scenario_seed, step_index, channel_id)` triple. Identical
    /// triples produce identical RNG streams; this is the contract that
    /// lets the kernel generate per-sensor noise byte-stably across
    /// reruns.
    #[must_use]
    pub fn for_channel(scenario_seed: u64, step: StepIndex, channel: ChannelId) -> Self {
        let mut bytes = [0u8; 32];
        bytes[0..8].copy_from_slice(&scenario_seed.to_le_bytes());
        bytes[8..16].copy_from_slice(&step.value().to_le_bytes());
        bytes[16..24].copy_from_slice(&channel.value().to_le_bytes());
        // bytes[24..32] reserved for a future stream-id field; kept
        // zero so Phase-1 outputs are stable.
        Self::from_raw_seed(bytes)
    }

    /// Construct a per-sensor-component deterministic stream.
    ///
    /// Phase 2.7 adds this constructor for synthetic sensors. The
    /// seed is derived from
    /// `(scenario_seed, step_index, sensor_id, component_id)` plus
    /// the `SENSOR_COMPONENT_DOMAIN_TAG` in the trailing 4 bytes.
    /// The domain tag guarantees no collision with [`Self::for_channel`]
    /// even if the integer payloads happen to overlap — `for_channel`
    /// leaves bytes [24..32] zero and this constructor writes a
    /// non-zero `b"SENS"` magic into bytes [28..32].
    ///
    /// `component_id` distinguishes the per-sensor noise channels
    /// (e.g. quantisation, ARW, bias instability, RRW, scale-factor)
    /// so adding or removing a sibling component does not shift any
    /// existing component's RNG stream.
    #[must_use]
    pub fn for_sensor_component(
        scenario_seed: u64,
        step: StepIndex,
        sensor_id: SensorId,
        component_id: u32,
    ) -> Self {
        let mut bytes = [0u8; 32];
        bytes[0..8].copy_from_slice(&scenario_seed.to_le_bytes());
        bytes[8..16].copy_from_slice(&step.value().to_le_bytes());
        bytes[16..24].copy_from_slice(&sensor_id.value().to_le_bytes());
        bytes[24..28].copy_from_slice(&component_id.to_le_bytes());
        bytes[28..32].copy_from_slice(&SENSOR_COMPONENT_DOMAIN_TAG);
        Self::from_raw_seed(bytes)
    }

    /// Construct a per-effector-component deterministic stream.
    ///
    /// Phase 3.4 adds this constructor for control effectors. The seed
    /// is derived from
    /// `(scenario_seed, step_index, effector_id, component_id)` plus
    /// the `EFFECTOR_COMPONENT_DOMAIN_TAG` in the trailing 4 bytes.
    ///
    /// The domain tag guarantees no collision with
    /// [`Self::for_channel`] (zero in [24..32]) or
    /// [`Self::for_sensor_component`] (`b"SENS"` in [28..32]).
    ///
    /// Phase-3.4 `LinearActuator` is fully deterministic and does not
    /// draw from this stream; the constructor exists so future
    /// stochastic-fault models can wire byte-stable per-effector noise.
    #[must_use]
    pub fn for_effector_component(
        scenario_seed: u64,
        step: StepIndex,
        effector_id: EffectorId,
        component_id: u32,
    ) -> Self {
        let mut bytes = [0u8; 32];
        bytes[0..8].copy_from_slice(&scenario_seed.to_le_bytes());
        bytes[8..16].copy_from_slice(&step.value().to_le_bytes());
        bytes[16..24].copy_from_slice(&effector_id.value().to_le_bytes());
        bytes[24..28].copy_from_slice(&component_id.to_le_bytes());
        bytes[28..32].copy_from_slice(&EFFECTOR_COMPONENT_DOMAIN_TAG);
        Self::from_raw_seed(bytes)
    }

    /// Construct a per-wind-axis deterministic stream.
    ///
    /// Phase 3.8 adds this constructor for the Dryden rational-spectrum
    /// shaping filter. The seed is derived from
    /// `(scenario_seed, step_index, axis)` plus the
    /// `WIND_COMPONENT_DOMAIN_TAG` in the trailing 4 bytes.
    ///
    /// Seed layout (locked, mirrors [`Self::for_effector_component`]):
    ///
    /// | bytes  | content                                |
    /// |--------|----------------------------------------|
    /// | 0..8   | `scenario_seed.to_le_bytes()`          |
    /// | 8..16  | `step.value().to_le_bytes()`           |
    /// | 16..24 | reserved zero (future `WindModelId`)   |
    /// | 24..28 | `axis.value().to_le_bytes()` (u32)     |
    /// | 28..32 | `b"WIND"`                              |
    ///
    /// Bytes [16..24] are reserved zero so a future Phase-3.X
    /// `WindModelId` can be introduced (multiple wind sources
    /// composing) without re-pinning the stream. The domain tag
    /// guarantees no collision with [`Self::for_channel`] (zero in
    /// [24..32]), [`Self::for_sensor_component`] (`b"SENS"` in
    /// [28..32]), or [`Self::for_effector_component`] (`b"EFFC"` in
    /// [28..32]).
    #[must_use]
    pub fn for_wind_component(scenario_seed: u64, step: StepIndex, axis: WindAxis) -> Self {
        let mut bytes = [0u8; 32];
        bytes[0..8].copy_from_slice(&scenario_seed.to_le_bytes());
        bytes[8..16].copy_from_slice(&step.value().to_le_bytes());
        // bytes[16..24] left zero — reserved for a future
        // `WindModelId` if multi-source wind composition lands.
        bytes[24..28].copy_from_slice(&axis.value().to_le_bytes());
        bytes[28..32].copy_from_slice(&WIND_COMPONENT_DOMAIN_TAG);
        Self::from_raw_seed(bytes)
    }

    /// Return the next `u32` from the deterministic stream.
    #[must_use]
    pub fn next_u32(&mut self) -> u32 {
        self.inner.next_u32()
    }

    /// Return the next `u64` from the deterministic stream.
    #[must_use]
    pub fn next_u64(&mut self) -> u64 {
        self.inner.next_u64()
    }

    /// Fill a byte slice from the deterministic stream.
    pub fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.inner.fill_bytes(dest);
    }
}

impl TryRng for DeterministicRng {
    type Error = Infallible;

    fn try_next_u32(&mut self) -> Result<u32, Self::Error> {
        Ok(Self::next_u32(self))
    }

    fn try_next_u64(&mut self) -> Result<u64, Self::Error> {
        Ok(Self::next_u64(self))
    }

    fn try_fill_bytes(&mut self, dest: &mut [u8]) -> Result<(), Self::Error> {
        Self::fill_bytes(self, dest);
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use proptest::prelude::*;

    #[test]
    fn same_seed_same_stream() {
        let mut a = DeterministicRng::from_seed(42);
        let mut b = DeterministicRng::from_seed(42);
        for _ in 0..1024 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn different_seeds_diverge() {
        let mut a = DeterministicRng::from_seed(42);
        let mut b = DeterministicRng::from_seed(43);
        // Statistical: at least one of the first 8 outputs should differ.
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(
            any_diff,
            "different seeds produced identical first 8 outputs"
        );
    }

    #[test]
    fn for_channel_is_deterministic() {
        let scenario = 0xdead_beef;
        let step = StepIndex::new(123);
        let chan = ChannelId::new(7);
        let mut a = DeterministicRng::for_channel(scenario, step, chan);
        let mut b = DeterministicRng::for_channel(scenario, step, chan);
        for _ in 0..1024 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn for_channel_distinct_inputs_diverge() {
        let s = 1u64;
        let step = StepIndex::new(0);
        let c0 = ChannelId::new(0);
        let c1 = ChannelId::new(1);
        let mut a = DeterministicRng::for_channel(s, step, c0);
        let mut b = DeterministicRng::for_channel(s, step, c1);
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(any_diff);
    }

    #[test]
    fn fill_bytes_matches_byte_stream() {
        let mut a = DeterministicRng::from_seed(99);
        let mut b = DeterministicRng::from_seed(99);
        let mut buf_a = [0u8; 64];
        let mut buf_b = [0u8; 64];
        a.fill_bytes(&mut buf_a);
        b.fill_bytes(&mut buf_b);
        assert_eq!(buf_a, buf_b);
    }

    // -----------------------------------------------------------------
    // for_sensor_component (Phase 2.7)
    // -----------------------------------------------------------------

    #[test]
    fn for_sensor_component_is_deterministic() {
        let scenario = 0xdead_beef;
        let step = StepIndex::new(123);
        let sensor = SensorId::new(0xABCD);
        let component = 7u32;
        let mut a = DeterministicRng::for_sensor_component(scenario, step, sensor, component);
        let mut b = DeterministicRng::for_sensor_component(scenario, step, sensor, component);
        for _ in 0..1024 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn for_sensor_component_distinct_component_ids_diverge() {
        let scenario = 1u64;
        let step = StepIndex::new(0);
        let sensor = SensorId::new(42);
        let mut a = DeterministicRng::for_sensor_component(scenario, step, sensor, 0);
        let mut b = DeterministicRng::for_sensor_component(scenario, step, sensor, 1);
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(
            any_diff,
            "distinct component ids must produce distinct streams"
        );
    }

    #[test]
    fn for_sensor_component_distinct_sensor_ids_diverge() {
        let scenario = 1u64;
        let step = StepIndex::new(0);
        let s_a = SensorId::new(42);
        let s_b = SensorId::new(43);
        let mut a = DeterministicRng::for_sensor_component(scenario, step, s_a, 0);
        let mut b = DeterministicRng::for_sensor_component(scenario, step, s_b, 0);
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(
            any_diff,
            "distinct sensor ids must produce distinct streams"
        );
    }

    #[test]
    fn for_sensor_component_never_collides_with_for_channel() {
        // Pick (scenario, step, integer_payload) such that
        // `for_channel(scenario, step, ChannelId(payload))` and
        // `for_sensor_component(scenario, step, SensorId(payload), 0)`
        // would collide if the domain tag were not present. The
        // `b"SENS"` tag in bytes [28..32] guarantees they differ.
        let scenario = 0x1234_5678u64;
        let step = StepIndex::new(99);
        let payload = 0xCAFEu64;
        let mut chan_rng = DeterministicRng::for_channel(scenario, step, ChannelId::new(payload));
        let mut sensor_rng =
            DeterministicRng::for_sensor_component(scenario, step, SensorId::new(payload), 0);
        let any_diff = (0..8).any(|_| chan_rng.next_u64() != sensor_rng.next_u64());
        assert!(
            any_diff,
            "for_channel and for_sensor_component must produce distinct streams \
             even with overlapping integer payloads",
        );
    }

    #[test]
    fn for_sensor_component_reference_stream_is_locked() {
        let mut rng = DeterministicRng::for_sensor_component(
            0x0123_4567_89ab_cdef,
            StepIndex::new(0x1020_3040_5060_7080),
            SensorId::new(0x1122_3344_5566_7788),
            0x0AABB,
        );
        let mut actual = [0u8; 32];
        rng.fill_bytes(&mut actual);
        // Pinned reference stream — drift would fail this test long
        // before any sensor regression test could pick it up.
        let expected = [
            0x64, 0x52, 0x3a, 0x96, 0xa7, 0xfa, 0xa9, 0xee, 0xa0, 0x66, 0x44, 0x17, 0xfa, 0x85,
            0xf6, 0xed, 0x4b, 0x90, 0xc9, 0xe5, 0xe0, 0x1a, 0x99, 0x1a, 0x44, 0x82, 0x05, 0x05,
            0x25, 0x65, 0x48, 0xf4,
        ];
        assert_eq!(actual, expected);
    }

    #[test]
    fn for_channel_reference_stream_is_locked() {
        let mut rng = DeterministicRng::for_channel(
            0x0123_4567_89ab_cdef,
            StepIndex::new(0x1020_3040_5060_7080),
            ChannelId::new(0x1122_3344_5566_7788),
        );
        let mut actual = [0u8; 32];
        rng.fill_bytes(&mut actual);
        let expected = [
            0xfc, 0x90, 0x2a, 0x70, 0x4d, 0x27, 0xb6, 0x7e, 0x01, 0xff, 0x31, 0xb7, 0x74, 0x36,
            0x57, 0x9b, 0xbc, 0x81, 0x14, 0xad, 0x98, 0x06, 0x4c, 0x4d, 0xa1, 0x11, 0x3a, 0x37,
            0x5f, 0x15, 0x52, 0xd8,
        ];
        assert_eq!(actual, expected);
    }

    #[test]
    fn for_effector_component_is_deterministic() {
        let scenario = 0xdead_beef;
        let step = StepIndex::new(123);
        let effector = EffectorId::new(0xABCD);
        let component = 7u32;
        let mut a = DeterministicRng::for_effector_component(scenario, step, effector, component);
        let mut b = DeterministicRng::for_effector_component(scenario, step, effector, component);
        for _ in 0..1024 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    fn for_effector_component_distinct_component_ids_diverge() {
        let scenario = 1u64;
        let step = StepIndex::new(0);
        let effector = EffectorId::new(42);
        let mut a = DeterministicRng::for_effector_component(scenario, step, effector, 0);
        let mut b = DeterministicRng::for_effector_component(scenario, step, effector, 1);
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(
            any_diff,
            "distinct component ids must produce distinct streams"
        );
    }

    #[test]
    fn for_effector_component_distinct_effector_ids_diverge() {
        let scenario = 1u64;
        let step = StepIndex::new(0);
        let mut a =
            DeterministicRng::for_effector_component(scenario, step, EffectorId::new(42), 0);
        let mut b =
            DeterministicRng::for_effector_component(scenario, step, EffectorId::new(43), 0);
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(
            any_diff,
            "distinct effector ids must produce distinct streams"
        );
    }

    #[test]
    fn for_effector_component_never_collides_with_for_channel() {
        // The `b"EFFC"` tag in bytes [28..32] guarantees no collision
        // with `for_channel`, which leaves bytes [24..32] zero.
        let scenario = 0x1234_5678u64;
        let step = StepIndex::new(99);
        let payload = 0xCAFEu64;
        let mut chan_rng = DeterministicRng::for_channel(scenario, step, ChannelId::new(payload));
        let mut eff_rng =
            DeterministicRng::for_effector_component(scenario, step, EffectorId::new(payload), 0);
        let any_diff = (0..8).any(|_| chan_rng.next_u64() != eff_rng.next_u64());
        assert!(
            any_diff,
            "for_channel and for_effector_component must produce distinct streams \
             even with overlapping integer payloads",
        );
    }

    #[test]
    fn for_effector_component_never_collides_with_for_sensor_component() {
        // `b"SENS"` vs `b"EFFC"` in bytes [28..32] guarantees the
        // sensor and effector RNG families never share a stream even
        // if the integer payloads (id + component) overlap.
        let scenario = 0x9999_aaaau64;
        let step = StepIndex::new(7);
        let payload = 0xBABEu64;
        let mut sensor_rng =
            DeterministicRng::for_sensor_component(scenario, step, SensorId::new(payload), 3);
        let mut eff_rng =
            DeterministicRng::for_effector_component(scenario, step, EffectorId::new(payload), 3);
        let any_diff = (0..8).any(|_| sensor_rng.next_u64() != eff_rng.next_u64());
        assert!(
            any_diff,
            "for_sensor_component and for_effector_component must produce distinct streams",
        );
    }

    #[test]
    fn for_effector_component_reference_stream_is_locked() {
        let mut rng = DeterministicRng::for_effector_component(
            0x0123_4567_89ab_cdef,
            StepIndex::new(0x1020_3040_5060_7080),
            EffectorId::new(0x1122_3344_5566_7788),
            0x0AABB,
        );
        let mut actual = [0u8; 32];
        rng.fill_bytes(&mut actual);
        // Pinned reference stream — any drift fails this test before
        // it can perturb downstream effector telemetry.
        let expected = PINNED_REFERENCE_STREAM;
        assert_eq!(actual, expected);
    }

    /// Pinned reference stream for `for_effector_component_reference_stream_is_locked`.
    /// Values captured at Phase-3.4.A; drift fails the locked test.
    const PINNED_REFERENCE_STREAM: [u8; 32] = [
        0x72, 0x33, 0xae, 0x9e, 0x83, 0xfd, 0xb1, 0x2b, 0x86, 0xee, 0xf2, 0x75, 0xd0, 0x51, 0xbf,
        0x54, 0x63, 0xcb, 0x7b, 0xbd, 0x57, 0x4c, 0x51, 0x42, 0xa4, 0x96, 0x3c, 0x25, 0x94, 0xac,
        0x99, 0xee,
    ];

    // -----------------------------------------------------------------
    // for_wind_component (Phase 3.8.A)
    // -----------------------------------------------------------------

    #[test]
    fn for_wind_component_is_deterministic() {
        let scenario = 0xdead_beef;
        let step = StepIndex::new(123);
        let mut a = DeterministicRng::for_wind_component(scenario, step, WindAxis::U);
        let mut b = DeterministicRng::for_wind_component(scenario, step, WindAxis::U);
        for _ in 0..1024 {
            assert_eq!(a.next_u64(), b.next_u64());
        }
    }

    #[test]
    #[allow(clippy::similar_names)]
    fn for_wind_component_distinct_axes_diverge() {
        let scenario = 1u64;
        let step = StepIndex::new(0);
        let mut rng_u = DeterministicRng::for_wind_component(scenario, step, WindAxis::U);
        let mut rng_v = DeterministicRng::for_wind_component(scenario, step, WindAxis::V);
        let mut rng_w = DeterministicRng::for_wind_component(scenario, step, WindAxis::W);
        let any_uv = (0..8).any(|_| rng_u.next_u64() != rng_v.next_u64());
        let any_uw = (0..8).any(|_| rng_u.next_u64() != rng_w.next_u64());
        let any_vw = (0..8).any(|_| rng_v.next_u64() != rng_w.next_u64());
        assert!(any_uv, "U and V axes must diverge");
        assert!(any_uw, "U and W axes must diverge");
        assert!(any_vw, "V and W axes must diverge");
    }

    #[test]
    fn for_wind_component_distinct_steps_diverge() {
        let scenario = 1u64;
        let mut a =
            DeterministicRng::for_wind_component(scenario, StepIndex::new(0), WindAxis::U);
        let mut b =
            DeterministicRng::for_wind_component(scenario, StepIndex::new(1), WindAxis::U);
        let any_diff = (0..8).any(|_| a.next_u64() != b.next_u64());
        assert!(any_diff, "distinct steps must produce distinct streams");
    }

    #[test]
    fn for_wind_component_never_collides_with_for_channel() {
        // `b"WIND"` in bytes [28..32] vs `for_channel`'s zero-filled
        // [24..32]. Distinct payloads guarantee no stream sharing
        // even when the integer-position fields happen to overlap.
        let scenario = 0x1234_5678u64;
        let step = StepIndex::new(99);
        let payload = 0xCAFEu64;
        let mut chan_rng = DeterministicRng::for_channel(scenario, step, ChannelId::new(payload));
        let mut wind_rng = DeterministicRng::for_wind_component(scenario, step, WindAxis::U);
        let any_diff = (0..8).any(|_| chan_rng.next_u64() != wind_rng.next_u64());
        assert!(
            any_diff,
            "for_channel and for_wind_component must produce distinct streams",
        );
    }

    #[test]
    fn for_wind_component_never_collides_with_for_sensor_component() {
        let scenario = 0x9999_aaaau64;
        let step = StepIndex::new(7);
        let payload = 0xBABEu64;
        let mut sensor_rng =
            DeterministicRng::for_sensor_component(scenario, step, SensorId::new(payload), 0);
        let mut wind_rng = DeterministicRng::for_wind_component(scenario, step, WindAxis::U);
        let any_diff = (0..8).any(|_| sensor_rng.next_u64() != wind_rng.next_u64());
        assert!(
            any_diff,
            "for_sensor_component and for_wind_component must produce distinct streams",
        );
    }

    #[test]
    fn for_wind_component_never_collides_with_for_effector_component() {
        let scenario = 0x1111_2222u64;
        let step = StepIndex::new(13);
        let payload = 0xFEEDu64;
        let mut eff_rng =
            DeterministicRng::for_effector_component(scenario, step, EffectorId::new(payload), 0);
        let mut wind_rng = DeterministicRng::for_wind_component(scenario, step, WindAxis::U);
        let any_diff = (0..8).any(|_| eff_rng.next_u64() != wind_rng.next_u64());
        assert!(
            any_diff,
            "for_effector_component and for_wind_component must produce distinct streams",
        );
    }

    #[test]
    fn for_wind_component_reference_stream_is_locked() {
        let mut rng = DeterministicRng::for_wind_component(
            0x0123_4567_89ab_cdef,
            StepIndex::new(0x1020_3040_5060_7080),
            WindAxis::U,
        );
        let mut actual = [0u8; 32];
        rng.fill_bytes(&mut actual);
        let expected = PINNED_WIND_REFERENCE_STREAM_U;
        assert_eq!(actual, expected);
    }

    /// Pinned reference stream for `for_wind_component_reference_stream_is_locked`.
    /// Values captured at Phase-3.8.A; drift fails the locked test before
    /// it can perturb downstream Dryden gust filter telemetry.
    const PINNED_WIND_REFERENCE_STREAM_U: [u8; 32] = [
        0x7c, 0xcd, 0x0e, 0xe2, 0x1e, 0x12, 0xfe, 0x5d, 0x3b, 0xec, 0x18, 0x5f, 0x05, 0x30, 0xb9,
        0xbd, 0xc2, 0x1d, 0x36, 0x02, 0xdd, 0xd1, 0x9d, 0x15, 0x4b, 0x3d, 0xb5, 0x84, 0xea, 0x7f,
        0xa6, 0x79,
    ];

    proptest! {
        #[test]
        fn property_byte_stable_replay_across_seeds(seed in any::<u64>()) {
            let mut a = DeterministicRng::from_seed(seed);
            let mut b = DeterministicRng::from_seed(seed);
            for _ in 0..32 {
                prop_assert_eq!(a.next_u64(), b.next_u64());
            }
        }
    }
}
