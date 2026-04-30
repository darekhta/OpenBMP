//! Log-replay tooling.
//!
//! Phase 4.9: minimal scaffold for state-stable replay. The
//! [`BusRecorder`] subscribes to a configured set of topics and
//! collects time-stamped samples; the [`BusReplayer`] re-injects a
//! recorded log into a fresh bus for regression replay.
//!
//! Bit-stable replay is a stretch goal that lands once the
//! integration tests validate the academic-tolerance state-stable
//! contract.

use std::collections::VecDeque;

use openbmp_core::SimTime;

use crate::bus::{Bus, Topic};

/// One recorded bus entry.
#[derive(Clone, Debug)]
pub struct ReplayEntry<T: Topic> {
    /// Sample timestamp (driven by the producer's clock).
    pub time: SimTime,
    /// Bus sequence at record time.
    pub seq: u64,
    /// Recorded value.
    pub value: T,
}

/// Records a single topic's publishes for later replay.
#[derive(Debug)]
pub struct BusRecorder<T: Topic> {
    entries: VecDeque<ReplayEntry<T>>,
    last_seq: u64,
}

impl<T: Topic> Default for BusRecorder<T> {
    fn default() -> Self {
        Self {
            entries: VecDeque::new(),
            last_seq: 0,
        }
    }
}

impl<T: Topic> BusRecorder<T> {
    /// Constructs an empty recorder.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Polls the bus once. If a fresh publish is observed, records
    /// it with the current `time`. The caller is responsible for
    /// driving this method at the desired cadence.
    pub fn poll(&mut self, bus: &Bus, time: SimTime) {
        let seq = match bus.sequence::<T>() {
            Ok(seq) => seq.value(),
            Err(_) => return,
        };
        if seq <= self.last_seq {
            return;
        }
        if let Ok(Some((value, sequence))) = bus.latest::<T>() {
            self.entries.push_back(ReplayEntry {
                time,
                seq: sequence.value(),
                value,
            });
            self.last_seq = sequence.value();
        }
    }

    /// Returns the recorded entries in publish order.
    #[must_use]
    pub fn entries(&self) -> &VecDeque<ReplayEntry<T>> {
        &self.entries
    }

    /// Returns the number of recorded entries.
    #[must_use]
    pub fn len(&self) -> usize {
        self.entries.len()
    }

    /// Returns `true` if no entries have been recorded.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.entries.is_empty()
    }
}

/// Replays a previously-recorded log onto a fresh bus.
#[derive(Debug)]
pub struct BusReplayer<T: Topic> {
    entries: VecDeque<ReplayEntry<T>>,
}

impl<T: Topic> BusReplayer<T> {
    /// Constructs the replayer over a recorded sequence.
    #[must_use]
    pub fn new(entries: Vec<ReplayEntry<T>>) -> Self {
        Self {
            entries: entries.into(),
        }
    }

    /// Pops and re-publishes any entry whose recorded `time` is
    /// strictly less than or equal to `time`. Returns the number of
    /// entries replayed this call.
    pub fn replay_until(&mut self, bus: &Bus, time: SimTime) -> usize {
        let mut count = 0;
        while let Some(entry) = self.entries.front()
            && entry.time.as_seconds() <= time.as_seconds()
        {
            // We just observed `front` is `Some`, so `pop_front` returns `Some`.
            if let Some(entry) = self.entries.pop_front() {
                let _ = bus.publish(entry.value);
                count += 1;
            }
        }
        count
    }

    /// Returns the number of entries remaining.
    #[must_use]
    pub fn remaining(&self) -> usize {
        self.entries.len()
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::bus::Bus;

    #[derive(Clone, Debug, PartialEq)]
    struct Beat {
        n: u32,
    }

    impl Topic for Beat {
        const NAME: &'static str = "test.beat";
    }

    #[test]
    fn record_then_replay_round_trips() {
        let bus = Bus::new();
        bus.register::<Beat>().unwrap();
        let mut recorder = BusRecorder::<Beat>::new();
        for i in 0..5 {
            bus.publish(Beat { n: i }).unwrap();
            recorder.poll(&bus, SimTime::from_seconds(f64::from(i) * 0.01));
        }
        assert_eq!(recorder.len(), 5);

        let entries: Vec<_> = recorder.entries().iter().cloned().collect();

        // Replay onto a new bus.
        let new_bus = Bus::new();
        new_bus.register::<Beat>().unwrap();
        let mut replayer = BusReplayer::<Beat>::new(entries);
        let n = replayer.replay_until(&new_bus, SimTime::from_seconds(1.0));
        assert_eq!(n, 5);
        assert_eq!(replayer.remaining(), 0);
        let (latest, _) = new_bus.latest::<Beat>().unwrap().unwrap();
        assert_eq!(latest.n, 4);
    }
}
