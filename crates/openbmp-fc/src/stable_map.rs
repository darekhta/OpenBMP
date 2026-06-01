//! Deterministic hasher for `indexmap` in `alloc`-only builds.
//!
//! `indexmap::IndexMap` only provides its `RandomState` default hasher
//! with `std`. The flight-controller registries remain dynamic during
//! the current portability slice, so they need an explicit no-std
//! hasher until generated static registries replace these maps.

use core::hash::{BuildHasherDefault, Hasher};

use indexmap::IndexMap;

pub(crate) type StableIndexMap<K, V> = IndexMap<K, V, BuildHasherDefault<StableHasher>>;

#[derive(Clone, Debug)]
pub(crate) struct StableHasher {
    state: u64,
}

impl Default for StableHasher {
    fn default() -> Self {
        Self {
            state: 0xcbf2_9ce4_8422_2325,
        }
    }
}

impl Hasher for StableHasher {
    fn finish(&self) -> u64 {
        self.state
    }

    fn write(&mut self, bytes: &[u8]) {
        for byte in bytes {
            self.state ^= u64::from(*byte);
            self.state = self.state.wrapping_mul(0x0000_0100_0000_01b3);
        }
    }
}
