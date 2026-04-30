//! Lockstep clock contract.
//!
//! The flight controller does **not** read the system clock. Every
//! module that needs a timestamp acquires it from the [`Clock`] trait,
//! which is injected by the embedder (the simulator's runner in SITL,
//! a downstream HAL crate on real hardware).
//!
//! This is the load-bearing invariant that lets the same controller
//! binary run lockstepped against the simulator's `SimTime` and
//! against a hardware monotonic counter without source-level changes.
//! A workspace-level lint forbids `std::time::Instant::now()` /
//! `std::time::SystemTime::now()` inside `openbmp-fc`; this module
//! exists to make that ban actionable.
//!
//! # SITL pattern
//!
//! ```ignore
//! use openbmp_fc::clock::{Clock, SimulatedClock};
//! use openbmp_core::{SimTime, StepIndex};
//!
//! let clock = SimulatedClock::new();
//! // The runner advances time as the kernel ticks.
//! clock.set(SimTime::from_seconds(0.001), StepIndex::ZERO.next());
//! // Controller modules read time / tick via &dyn Clock.
//! assert_eq!(clock.now().as_seconds(), 0.001);
//! ```

use std::cell::Cell;

use openbmp_core::{SimTime, StepIndex};

/// Source of monotonic timestamps for the flight controller.
///
/// All controller modules that need a timestamp do so through a
/// `&dyn Clock`. The simulator and a downstream HAL each provide
/// their own implementation; the controller crate itself ships only
/// the trait + simulator-friendly implementations.
pub trait Clock {
    /// Returns the current monotonic timestamp.
    fn now(&self) -> SimTime;

    /// Returns the current monotonic tick index. Tick semantics are
    /// embedder-defined: the simulator binds this to its
    /// post-integration tick counter; a HAL adopter typically binds
    /// it to a hardware-timer-driven cycle counter.
    fn tick(&self) -> StepIndex;
}

/// In-process clock backed by interior mutability. The runner advances
/// time and tick counter via [`SimulatedClock::set`]; controller
/// modules read via the [`Clock`] trait.
#[derive(Debug)]
pub struct SimulatedClock {
    time: Cell<SimTime>,
    tick: Cell<StepIndex>,
}

impl SimulatedClock {
    /// Constructs a clock anchored at `SimTime::ZERO` and `StepIndex::ZERO`.
    #[must_use]
    pub fn new() -> Self {
        Self {
            time: Cell::new(SimTime::ZERO),
            tick: Cell::new(StepIndex::ZERO),
        }
    }

    /// Constructs a clock anchored at the given timestamp and tick.
    #[must_use]
    pub fn at(time: SimTime, tick: StepIndex) -> Self {
        Self {
            time: Cell::new(time),
            tick: Cell::new(tick),
        }
    }

    /// Advances both `time` and `tick` to the supplied values. The
    /// runner calls this once per step, after `SimTime` has been
    /// advanced kernel-side.
    pub fn set(&self, time: SimTime, tick: StepIndex) {
        self.time.set(time);
        self.tick.set(tick);
    }
}

impl Default for SimulatedClock {
    fn default() -> Self {
        Self::new()
    }
}

impl Clock for SimulatedClock {
    fn now(&self) -> SimTime {
        self.time.get()
    }

    fn tick(&self) -> StepIndex {
        self.tick.get()
    }
}

/// Read-only clock fixed at construction time. Useful for unit tests
/// and for synchronously sampling a controller against a known
/// timestamp.
#[derive(Copy, Clone, Debug)]
pub struct FixedClock {
    time: SimTime,
    tick: StepIndex,
}

impl FixedClock {
    /// Returns a clock anchored at the given values.
    #[must_use]
    pub const fn new(time: SimTime, tick: StepIndex) -> Self {
        Self { time, tick }
    }
}

impl Clock for FixedClock {
    fn now(&self) -> SimTime {
        self.time
    }

    fn tick(&self) -> StepIndex {
        self.tick
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    #[test]
    fn simulated_clock_round_trips_set_and_read() {
        let clock = SimulatedClock::new();
        assert_eq!(clock.now().as_seconds(), 0.0);
        assert_eq!(clock.tick().value(), 0);

        clock.set(SimTime::from_seconds(0.123), StepIndex::new(42));
        assert!((clock.now().as_seconds() - 0.123).abs() < f64::EPSILON);
        assert_eq!(clock.tick().value(), 42);
    }

    #[test]
    fn fixed_clock_returns_construction_values() {
        let clock = FixedClock::new(SimTime::from_seconds(1.0), StepIndex::new(7));
        assert!((clock.now().as_seconds() - 1.0).abs() < f64::EPSILON);
        assert_eq!(clock.tick().value(), 7);
    }
}
