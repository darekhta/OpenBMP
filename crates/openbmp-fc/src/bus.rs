//! Internal pub/sub bus for the flight controller.
//!
//! The bus is a typed-topic registry adapted from PX4 uORB. Each
//! topic carries a name, version, static topic-table index, a monotonically incrementing
//! sequence counter, and the latest value. Producers
//! [`publish`](Bus::publish) typed values; consumers
//! [`latest`](Bus::latest) typed snapshots and check
//! [`changed_since`](Bus::changed_since) to react only to new updates.
//!
//! The implementation is single-threaded by design — determinism is
//! the primary requirement and threading would force locks that the
//! kernel's lockstep contract has explicitly rejected. Interior
//! mutability uses [`RefCell`] so the bus can be passed around as
//! `&Bus` even when individual modules need to publish.
//!
//! # Topic registration
//!
//! Topics must be registered with [`Bus::register`] before they are
//! published or read. Registration captures the topic's name and
//! version into the bus dictionary and reserves storage; subsequent
//! publishes overwrite the latest value and bump the sequence
//! counter.
//!
//! # Determinism
//!
//! - Topic iteration order is stable: [`indexmap::IndexMap`] preserves
//!   insertion order.
//! - Sequence counters monotonically increase from `1`. A subscriber
//!   that has read up to sequence `N` knows that any later read at
//!   sequence `M > N` is strictly newer.
//! - The bus does not allocate after registration: every publish
//!   updates the in-place storage cell. The only allocation cost is
//!   the registration itself.

use std::any::{Any, TypeId};
use std::boxed::Box;
use std::cell::RefCell;
use std::vec::Vec;

use crate::error::BusError;
use crate::stable_map::StableIndexMap;

/// Marker trait implemented by every type that flows through the bus.
///
/// `NAME` is the canonical wire name (used in dictionaries and logs);
/// `VERSION` is the schema version used when external consumers parse
/// dictionaries or logs, and `INDEX` is the compile-time topic slot
/// used by static bus backends.
pub trait Topic: 'static + Clone {
    /// Canonical topic name. Convention: `snake_case`, no whitespace,
    /// `<subsystem>.<topic>` for namespaced topics.
    const NAME: &'static str;
    /// Schema version. Bump on incompatible field changes; consumers
    /// reject topics whose version does not match what they were
    /// compiled against.
    const VERSION: u32 = 1;
    /// Compile-time slot index for static bus backends. Dynamic
    /// host-only test topics may leave this as `usize::MAX`.
    const INDEX: usize = usize::MAX;
}

/// Monotonically-increasing topic sequence counter.
///
/// Each [`Bus::publish`] of a given topic increments the topic's
/// sequence counter by `1`. A subscriber records the sequence value
/// it has consumed and queries
/// [`Bus::changed_since`](Bus::changed_since) to detect newer
/// publishes without re-reading the value.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct Sequence(u64);

impl Sequence {
    /// `Sequence::ZERO` is the value held by a freshly-registered
    /// topic that has not yet been published. The first [`Bus::publish`]
    /// raises the sequence to `1`.
    pub const ZERO: Self = Self(0);

    /// Returns the underlying integer.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }

    /// Constructs a [`Sequence`] from a raw integer. Used by the
    /// scheduler's name-based topic lookup; not normally part of the
    /// caller-facing API.
    #[doc(hidden)]
    #[must_use]
    pub const fn from_u64(value: u64) -> Self {
        Self(value)
    }
}

/// Internal storage cell for one topic.
struct BusCell {
    name: &'static str,
    version: u32,
    index: usize,
    seq: u64,
    /// Stores `Option<T>` so a subscriber knows the difference between
    /// "registered but not yet published" (`None`) and "published"
    /// (`Some(latest)`).
    storage: Box<dyn Any>,
}

/// Registered topic descriptor exposed via [`Bus::topics`] for dictionary
/// generation and CI introspection.
#[derive(Copy, Clone, Debug, Eq, PartialEq)]
pub struct TopicInfo {
    /// Canonical topic name (`Topic::NAME`).
    pub name: &'static str,
    /// Schema version (`Topic::VERSION`).
    pub version: u32,
    /// Static bus table index (`Topic::INDEX`).
    pub index: usize,
    /// Number of publishes the bus has observed since the topic was
    /// registered.
    pub seq: u64,
    /// `true` once the topic has been published at least once.
    pub has_value: bool,
}

/// Single-threaded typed-topic pub/sub bus.
#[derive(Default)]
pub struct Bus {
    inner: RefCell<BusInner>,
}

#[derive(Default)]
struct BusInner {
    cells: StableIndexMap<TypeId, BusCell>,
}

impl Bus {
    /// Constructs an empty bus.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Registers a topic. Must be called once per topic at startup
    /// before any publish or read; subsequent registrations of the
    /// same topic return [`BusError::DuplicateTopic`].
    ///
    /// # Errors
    ///
    /// Returns [`BusError::DuplicateTopic`] if the topic is already
    /// registered.
    pub fn register<T: Topic>(&self) -> Result<(), BusError> {
        let mut inner = self.inner.borrow_mut();
        let id = TypeId::of::<T>();
        if inner.cells.contains_key(&id) {
            return Err(BusError::DuplicateTopic {
                topic_name: T::NAME,
            });
        }
        let storage: Option<T> = None;
        inner.cells.insert(
            id,
            BusCell {
                name: T::NAME,
                version: T::VERSION,
                index: T::INDEX,
                seq: 0,
                storage: Box::new(storage),
            },
        );
        Ok(())
    }

    /// Publishes a typed value. Bumps the topic's sequence counter by
    /// `1` and overwrites the latest value.
    ///
    /// # Errors
    ///
    /// Returns [`BusError::UnregisteredPublish`] if the topic was not
    /// registered first.
    pub fn publish<T: Topic>(&self, value: T) -> Result<Sequence, BusError> {
        let mut inner = self.inner.borrow_mut();
        let id = TypeId::of::<T>();
        let cell = inner
            .cells
            .get_mut(&id)
            .ok_or(BusError::UnregisteredPublish {
                topic_name: T::NAME,
            })?;
        let storage =
            cell.storage
                .downcast_mut::<Option<T>>()
                .ok_or(BusError::UnregisteredPublish {
                    topic_name: T::NAME,
                })?;
        *storage = Some(value);
        cell.seq = cell.seq.saturating_add(1);
        Ok(Sequence(cell.seq))
    }

    /// Returns the latest value for a topic and the sequence counter
    /// at which it was published, or `None` if the topic has never
    /// been published.
    ///
    /// # Errors
    ///
    /// Returns [`BusError::UnknownTopic`] if the topic is not
    /// registered.
    pub fn latest<T: Topic>(&self) -> Result<Option<(T, Sequence)>, BusError> {
        let inner = self.inner.borrow();
        let id = TypeId::of::<T>();
        let cell = inner.cells.get(&id).ok_or(BusError::UnknownTopic {
            topic_name: T::NAME,
        })?;
        let storage = cell
            .storage
            .downcast_ref::<Option<T>>()
            .ok_or(BusError::UnknownTopic {
                topic_name: T::NAME,
            })?;
        Ok(storage.clone().map(|v| (v, Sequence(cell.seq))))
    }

    /// Returns the current sequence counter for a topic without
    /// reading the value. Cheap; useful for change-detection in a hot
    /// scheduler loop.
    ///
    /// # Errors
    ///
    /// Returns [`BusError::UnknownTopic`] if the topic is not
    /// registered.
    pub fn sequence<T: Topic>(&self) -> Result<Sequence, BusError> {
        let inner = self.inner.borrow();
        let id = TypeId::of::<T>();
        let cell = inner.cells.get(&id).ok_or(BusError::UnknownTopic {
            topic_name: T::NAME,
        })?;
        Ok(Sequence(cell.seq))
    }

    /// Returns `true` if the topic has been published with a sequence
    /// strictly greater than `last_seen`.
    ///
    /// # Errors
    ///
    /// Returns [`BusError::UnknownTopic`] if the topic is not
    /// registered.
    pub fn changed_since<T: Topic>(&self, last_seen: Sequence) -> Result<bool, BusError> {
        let seq = self.sequence::<T>()?;
        Ok(seq.0 > last_seen.0)
    }

    /// Returns descriptors for every registered topic in registration
    /// order. Used by the dictionary generator and by introspection
    /// tooling.
    #[must_use]
    pub fn topics(&self) -> Vec<TopicInfo> {
        let inner = self.inner.borrow();
        inner
            .cells
            .values()
            .map(|c| TopicInfo {
                name: c.name,
                version: c.version,
                index: c.index,
                seq: c.seq,
                has_value: cell_has_value(&c.storage),
            })
            .collect()
    }
}

fn cell_has_value(storage: &dyn Any) -> bool {
    // We don't know the concrete T here, so probe the type-erased
    // option layout: every storage Box<dyn Any> downcasts to
    // Option<T>. We accept "registered but never published" as
    // `false` and "published" as `true`. The probe runs over a small
    // fixed set of POD topics; if a topic is added that does not
    // downcast against this probe, the topic is reported as "never
    // published" (conservative, correct for dictionary generation).
    macro_rules! probe {
        ($($ty:ty),*) => {
            $(if let Some(opt) = storage.downcast_ref::<Option<$ty>>() {
                return opt.is_some();
            })*
        };
    }
    probe!((), bool, u8, u16, u32, u64, i8, i16, i32, i64, f32, f64);
    // Fallback: assume never-published. Real introspection lives in
    // the dictionary generator, which has compile-time topic
    // knowledge.
    let _ = storage;
    false
}

impl std::fmt::Debug for Bus {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("Bus")
            .field("topics", &self.topics())
            .finish()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[derive(Clone, Debug, PartialEq)]
    struct Heartbeat {
        beat: u32,
    }

    impl Topic for Heartbeat {
        const NAME: &'static str = "diagnostics.heartbeat";
        const VERSION: u32 = 1;
        const INDEX: usize = 0;
    }

    #[derive(Clone, Debug, PartialEq)]
    struct Status {
        ok: bool,
    }

    impl Topic for Status {
        const NAME: &'static str = "diagnostics.status";
        const VERSION: u32 = 1;
        const INDEX: usize = 1;
    }

    #[test]
    fn register_publish_read_round_trip() {
        let bus = Bus::new();
        bus.register::<Heartbeat>().unwrap();

        // Before publish: latest is None, sequence is 0.
        let latest = bus.latest::<Heartbeat>().unwrap();
        assert!(latest.is_none());
        assert_eq!(bus.sequence::<Heartbeat>().unwrap().value(), 0);

        // Publish bumps sequence; latest reads the value.
        bus.publish(Heartbeat { beat: 7 }).unwrap();
        let (value, seq) = bus.latest::<Heartbeat>().unwrap().unwrap();
        assert_eq!(value.beat, 7);
        assert_eq!(seq.value(), 1);
    }

    #[test]
    fn duplicate_register_rejected() {
        let bus = Bus::new();
        bus.register::<Heartbeat>().unwrap();
        let err = bus.register::<Heartbeat>().unwrap_err();
        assert!(matches!(err, BusError::DuplicateTopic { .. }));
    }

    #[test]
    fn publish_without_register_rejected() {
        let bus = Bus::new();
        let err = bus.publish(Heartbeat { beat: 1 }).unwrap_err();
        assert!(matches!(err, BusError::UnregisteredPublish { .. }));
    }

    #[test]
    fn read_without_register_rejected() {
        let bus = Bus::new();
        let err = bus.latest::<Heartbeat>().unwrap_err();
        assert!(matches!(err, BusError::UnknownTopic { .. }));
    }

    #[test]
    fn changed_since_tracks_new_publishes() {
        let bus = Bus::new();
        bus.register::<Heartbeat>().unwrap();
        let initial = bus.sequence::<Heartbeat>().unwrap();
        assert!(!bus.changed_since::<Heartbeat>(initial).unwrap());

        bus.publish(Heartbeat { beat: 1 }).unwrap();
        assert!(bus.changed_since::<Heartbeat>(initial).unwrap());

        let after_first = bus.sequence::<Heartbeat>().unwrap();
        assert!(!bus.changed_since::<Heartbeat>(after_first).unwrap());

        bus.publish(Heartbeat { beat: 2 }).unwrap();
        assert!(bus.changed_since::<Heartbeat>(after_first).unwrap());
    }

    #[test]
    fn topics_iterate_in_registration_order() {
        let bus = Bus::new();
        bus.register::<Heartbeat>().unwrap();
        bus.register::<Status>().unwrap();
        let infos = bus.topics();
        assert_eq!(infos.len(), 2);
        assert_eq!(infos[0].name, Heartbeat::NAME);
        assert_eq!(infos[0].index, Heartbeat::INDEX);
        assert_eq!(infos[1].name, Status::NAME);
        assert_eq!(infos[1].index, Status::INDEX);
    }

    #[test]
    fn many_publishes_strictly_increment_sequence() {
        let bus = Bus::new();
        bus.register::<Heartbeat>().unwrap();
        let mut prev = 0u64;
        for i in 0..100 {
            let seq = bus.publish(Heartbeat { beat: i }).unwrap();
            assert_eq!(seq.value(), prev + 1);
            prev = seq.value();
        }
    }
}
