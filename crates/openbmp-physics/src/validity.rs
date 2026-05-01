//! Validity-envelope helpers shared by physics models.
//!
//! The model modules still expose their domain-specific errors, but
//! these small value types give callers and tests a common vocabulary
//! for finite scalar ranges.

/// Inclusive lower / exclusive upper finite scalar range.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct HalfOpenRange {
    /// Inclusive lower bound.
    pub start: f64,
    /// Exclusive upper bound.
    pub end: f64,
}

impl HalfOpenRange {
    /// Constructs a range after checking that both bounds are finite
    /// and ordered.
    #[must_use]
    pub fn new(start: f64, end: f64) -> Option<Self> {
        if start.is_finite() && end.is_finite() && start < end {
            Some(Self { start, end })
        } else {
            None
        }
    }

    /// Returns `true` when `value` lies in `[start, end)`.
    #[must_use]
    pub fn contains(self, value: f64) -> bool {
        value.is_finite() && self.start <= value && value < self.end
    }
}

#[cfg(test)]
mod tests {
    use super::HalfOpenRange;

    #[test]
    fn half_open_range_accepts_only_finite_ordered_bounds() {
        assert!(HalfOpenRange::new(0.0, 1.0).is_some());
        assert!(HalfOpenRange::new(1.0, 1.0).is_none());
        assert!(HalfOpenRange::new(f64::NAN, 1.0).is_none());
    }

    #[test]
    fn half_open_range_contains_start_but_not_end() {
        let range = HalfOpenRange {
            start: 2025.0,
            end: 2030.0,
        };
        assert!(range.contains(2025.0));
        assert!(range.contains(2029.999));
        assert!(!range.contains(2030.0));
    }
}
