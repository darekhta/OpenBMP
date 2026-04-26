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

use crate::ids::{ChannelId, SensorId};
use crate::time::StepIndex;

/// Domain tag bytes placed in the trailing 4 bytes of the RNG seed
/// for [`DeterministicRng::for_sensor_component`]. Guarantees that a
/// sensor-component stream can never collide with a `for_channel`
/// stream — `for_channel` always leaves bytes [24..32] zero, while
/// `for_sensor_component` writes a non-zero domain tag in [28..32].
const SENSOR_COMPONENT_DOMAIN_TAG: [u8; 4] = *b"SENS";

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
    /// the [`SENSOR_COMPONENT_DOMAIN_TAG`] in the trailing 4 bytes.
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
