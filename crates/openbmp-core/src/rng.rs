//! Deterministic pseudo-random number generation.
//!
//! [`DeterministicRng`] wraps `rand_chacha::ChaCha8Rng`, the
//! workspace-canonical bit-stable RNG. Constructors derive the seed
//! from explicit `(scenario_seed, step_index, channel_id)` triples so
//! the kernel can produce the same byte stream across reruns.
//!
//! No type in this module reads from system entropy.

use rand::{RngCore, SeedableRng};
use rand_chacha::ChaCha8Rng;

use crate::ids::ChannelId;
use crate::time::StepIndex;

/// Deterministic pseudo-random number generator.
///
/// Implements [`rand::RngCore`] and is therefore usable anywhere the
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
}

impl RngCore for DeterministicRng {
    fn next_u32(&mut self) -> u32 {
        self.inner.next_u32()
    }

    fn next_u64(&mut self) -> u64 {
        self.inner.next_u64()
    }

    fn fill_bytes(&mut self, dest: &mut [u8]) {
        self.inner.fill_bytes(dest);
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
