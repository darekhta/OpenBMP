//! Monotonic time and tick primitives.
//!
//! [`SimTime`] is seconds on a caller-owned monotonic timeline. The
//! simulator binds it to elapsed scenario time; a HAL adopter may bind
//! it to a hardware monotonic counter normalised to controller start.
//! It is not UTC / civil wall-clock time. [`Duration`] is a span
//! between two [`SimTime`] values. [`StepIndex`] is a monotonic tick
//! counter used for deterministic scheduling and seeding.
//!
//! No type in this module accesses `std::time` or any system clock.

use std::ops::{Add, AddAssign, Neg, Sub, SubAssign};

use crate::error::TimeError;

/// Monotonic elapsed time, in seconds since a caller-defined origin.
///
/// In simulator code the origin is scenario start. In HAL-backed code
/// it can be a controller boot epoch or another hardware monotonic
/// reference. The value is not UTC / civil wall-clock time.
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct SimTime(f64);

impl SimTime {
    /// `t = 0` at the caller-defined timeline origin.
    pub const ZERO: Self = Self(0.0);

    /// Construct a [`SimTime`] from seconds.
    #[must_use]
    pub const fn from_seconds(seconds: f64) -> Self {
        Self(seconds)
    }

    /// Returns the underlying value in seconds.
    #[must_use]
    pub const fn as_seconds(self) -> f64 {
        self.0
    }

    /// Returns `true` if the value is finite (not NaN, not infinite).
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.0.is_finite()
    }

    /// Returns `true` if the value is finite and not before the
    /// timeline origin.
    #[must_use]
    pub fn is_valid(self) -> bool {
        self.is_finite() && self.0 >= 0.0
    }

    /// Validate that this value is a usable monotonic timestamp.
    ///
    /// # Errors
    ///
    /// Returns [`TimeError::NotFinite`] if the value is `NaN` or
    /// infinite; returns [`TimeError::NegativeTime`] if the value is
    /// before the timeline origin.
    pub fn require_valid(self) -> Result<Self, TimeError> {
        if !self.is_finite() {
            return Err(TimeError::NotFinite { value: self.0 });
        }
        if self.0 < 0.0 {
            return Err(TimeError::NegativeTime { seconds: self.0 });
        }
        Ok(self)
    }

    /// Validate that `next` is a valid timestamp and not earlier
    /// than `self`.
    ///
    /// # Errors
    ///
    /// Returns [`TimeError::NotFinite`] if either `self` or `next` is
    /// not finite, [`TimeError::NegativeTime`] if either value is before
    /// the timeline origin, and [`TimeError::NotMonotonic`] if
    /// `next < self`.
    pub fn check_advance_to(self, next: Self) -> Result<(), TimeError> {
        self.require_valid()?;
        next.require_valid()?;
        if next.0 < self.0 {
            return Err(TimeError::NotMonotonic {
                prior_s: self.0,
                next_s: next.0,
            });
        }
        Ok(())
    }
}

impl Add<Duration> for SimTime {
    type Output = Self;
    fn add(self, rhs: Duration) -> Self {
        Self(self.0 + rhs.as_seconds())
    }
}

impl AddAssign<Duration> for SimTime {
    fn add_assign(&mut self, rhs: Duration) {
        self.0 += rhs.as_seconds();
    }
}

impl Sub for SimTime {
    type Output = Duration;
    fn sub(self, rhs: Self) -> Duration {
        Duration::from_seconds(self.0 - rhs.0)
    }
}

impl Sub<Duration> for SimTime {
    type Output = Self;
    fn sub(self, rhs: Duration) -> Self {
        Self(self.0 - rhs.as_seconds())
    }
}

/// A duration in seconds. May be positive, zero, or negative.
#[derive(Copy, Clone, Debug, Default, PartialEq, PartialOrd)]
pub struct Duration(f64);

impl Duration {
    /// Zero duration.
    pub const ZERO: Self = Self(0.0);

    /// Construct a [`Duration`] from seconds.
    #[must_use]
    pub const fn from_seconds(seconds: f64) -> Self {
        Self(seconds)
    }

    /// Construct a [`Duration`] from milliseconds.
    #[must_use]
    pub fn from_millis(milliseconds: f64) -> Self {
        Self(milliseconds * 1.0e-3)
    }

    /// Construct a [`Duration`] from milliseconds.
    #[must_use]
    pub fn from_milliseconds(milliseconds: f64) -> Self {
        Self::from_millis(milliseconds)
    }

    /// Returns the underlying value in seconds.
    #[must_use]
    pub const fn as_seconds(self) -> f64 {
        self.0
    }

    /// Returns `true` if the value is finite.
    #[must_use]
    pub fn is_finite(self) -> bool {
        self.0.is_finite()
    }

    /// Returns `true` if the duration is strictly positive and finite.
    #[must_use]
    pub fn is_positive(self) -> bool {
        self.0 > 0.0 && self.0.is_finite()
    }

    /// Validate that the duration is strictly positive and finite.
    ///
    /// # Errors
    ///
    /// Returns [`TimeError::NotFinite`] if the value is `NaN` or
    /// infinite; returns [`TimeError::NotPositive`] if the value is
    /// less than or equal to zero.
    pub fn require_positive(self) -> Result<Self, TimeError> {
        if !self.is_finite() {
            return Err(TimeError::NotFinite { value: self.0 });
        }
        if self.0 <= 0.0 {
            return Err(TimeError::NotPositive { seconds: self.0 });
        }
        Ok(self)
    }
}

impl Add for Duration {
    type Output = Self;
    fn add(self, rhs: Self) -> Self {
        Self(self.0 + rhs.0)
    }
}

impl AddAssign for Duration {
    fn add_assign(&mut self, rhs: Self) {
        self.0 += rhs.0;
    }
}

impl Sub for Duration {
    type Output = Self;
    fn sub(self, rhs: Self) -> Self {
        Self(self.0 - rhs.0)
    }
}

impl SubAssign for Duration {
    fn sub_assign(&mut self, rhs: Self) {
        self.0 -= rhs.0;
    }
}

impl Neg for Duration {
    type Output = Self;
    fn neg(self) -> Self {
        Self(-self.0)
    }
}

/// Monotonic tick counter.
#[derive(Copy, Clone, Debug, Default, PartialEq, Eq, Hash, PartialOrd, Ord)]
pub struct StepIndex(u64);

impl StepIndex {
    /// Tick zero.
    pub const ZERO: Self = Self(0);

    /// Construct a [`StepIndex`] from an integer.
    #[must_use]
    pub const fn new(value: u64) -> Self {
        Self(value)
    }

    /// Returns the underlying integer value.
    #[must_use]
    pub const fn value(self) -> u64 {
        self.0
    }

    /// Returns the next tick in sequence, or `None` at `u64::MAX`.
    ///
    /// Consumers should use [`StepIndex::checked_next`] when they need
    /// a structured overflow error.
    #[must_use]
    pub const fn next(self) -> Option<Self> {
        match self.0.checked_add(1) {
            Some(value) => Some(Self(value)),
            None => None,
        }
    }

    /// Returns the next tick in sequence, failing instead of saturating.
    ///
    /// # Errors
    ///
    /// Returns [`TimeError::StepOverflow`] when called at `u64::MAX`.
    pub const fn checked_next(self) -> Result<Self, TimeError> {
        match self.next() {
            Some(value) => Ok(value),
            None => Err(TimeError::StepOverflow { value: self.0 }),
        }
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use proptest::prelude::*;

    #[test]
    fn sim_time_zero_is_zero() {
        assert_abs_diff_eq!(SimTime::ZERO.as_seconds(), 0.0);
    }

    #[test]
    fn sim_time_round_trip() {
        let t = SimTime::from_seconds(1.5);
        assert_abs_diff_eq!(t.as_seconds(), 1.5);
    }

    #[test]
    fn sim_time_plus_duration() {
        let t0 = SimTime::from_seconds(2.0);
        let dt = Duration::from_seconds(0.25);
        assert_abs_diff_eq!((t0 + dt).as_seconds(), 2.25);
    }

    #[test]
    fn sim_time_difference_is_duration() {
        let t1 = SimTime::from_seconds(3.0);
        let t0 = SimTime::from_seconds(1.0);
        assert_abs_diff_eq!((t1 - t0).as_seconds(), 2.0);
    }

    #[test]
    fn duration_positive_classifier() {
        assert!(Duration::from_seconds(0.001).is_positive());
        assert!(!Duration::from_seconds(0.0).is_positive());
        assert!(!Duration::from_seconds(-1.0).is_positive());
        assert!(!Duration::from_seconds(f64::NAN).is_positive());
        assert!(!Duration::from_seconds(f64::INFINITY).is_positive());
    }

    #[test]
    fn duration_require_positive_fails_on_nonfinite() {
        assert!(Duration::from_seconds(f64::NAN).require_positive().is_err());
        assert!(
            Duration::from_seconds(f64::INFINITY)
                .require_positive()
                .is_err()
        );
        assert!(Duration::from_seconds(0.0).require_positive().is_err());
        assert!(Duration::from_seconds(-1.0).require_positive().is_err());
        assert!(Duration::from_seconds(1.0).require_positive().is_ok());
    }

    #[test]
    fn sim_time_require_valid_rejects_negative() {
        let err = SimTime::from_seconds(-1.0).require_valid().unwrap_err();
        assert!(matches!(err, TimeError::NegativeTime { .. }));
    }

    #[test]
    fn step_index_next_is_monotonic() {
        let s0 = StepIndex::ZERO;
        let s1 = s0.next().unwrap();
        let s2 = s1.next().unwrap();
        assert!(s0 < s1);
        assert!(s1 < s2);
        assert_eq!(s1.value(), 1);
        assert_eq!(s2.value(), 2);
    }

    #[test]
    fn step_index_next_reports_none_at_u64_max() {
        let s = StepIndex::new(u64::MAX);
        assert!(s.next().is_none());
    }

    #[test]
    fn check_advance_rejects_backward_time() {
        let prior = SimTime::from_seconds(10.0);
        let next = SimTime::from_seconds(5.0);
        let err = prior.check_advance_to(next).unwrap_err();
        assert!(matches!(err, TimeError::NotMonotonic { .. }));
    }

    #[test]
    fn check_advance_rejects_nan() {
        let prior = SimTime::from_seconds(10.0);
        let next = SimTime::from_seconds(f64::NAN);
        let err = prior.check_advance_to(next).unwrap_err();
        assert!(matches!(err, TimeError::NotFinite { .. }));
    }

    #[test]
    fn check_advance_rejects_negative_prior() {
        let prior = SimTime::from_seconds(-1.0);
        let next = SimTime::ZERO;
        let err = prior.check_advance_to(next).unwrap_err();
        assert!(matches!(err, TimeError::NegativeTime { .. }));
    }

    #[test]
    fn check_advance_accepts_equal_time() {
        let t = SimTime::from_seconds(7.0);
        assert!(t.check_advance_to(t).is_ok());
    }

    #[test]
    fn checked_step_index_next_reports_overflow() {
        let err = StepIndex::new(u64::MAX).checked_next().unwrap_err();
        assert!(matches!(err, TimeError::StepOverflow { .. }));
    }

    proptest! {
        #[test]
        fn property_sim_time_difference_round_trip(
            a in -1.0e9_f64..1.0e9,
            b in -1.0e9_f64..1.0e9,
        ) {
            let ta = SimTime::from_seconds(a);
            let tb = SimTime::from_seconds(b);
            let dt = tb - ta;
            // Round trip via addition.
            let tb_back = ta + dt;
            // Allow a relative tolerance for floating-point round-off.
            prop_assert!((tb_back.as_seconds() - tb.as_seconds()).abs() < 1.0e-6);
        }

        #[test]
        fn property_check_advance_when_next_ge_prior(
            a in 0.0_f64..1.0e9,
            d in 0.0_f64..1.0e6,
        ) {
            let prior = SimTime::from_seconds(a);
            let next = SimTime::from_seconds(a + d);
            prop_assert!(prior.check_advance_to(next).is_ok());
        }

        #[test]
        fn property_step_index_next_increments(v in 0_u64..u64::MAX - 1) {
            let s = StepIndex::new(v);
            let n = s.next().unwrap();
            prop_assert_eq!(n.value(), v + 1);
            prop_assert!(s < n);
        }
    }
}
