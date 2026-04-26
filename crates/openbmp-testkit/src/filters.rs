//! Phase-4 placeholder for the `compare_filters` harness.
//!
//! The full filter-comparison harness (NorthStarUAS pattern) requires
//! the `openbmp-fc` estimators to land first. This module currently
//! exposes only the type sketches that Phase 4 will fill in.

use openbmp_core::ValidationStatus;

/// Placeholder report describing a side-by-side estimator comparison.
///
/// Phase 4 will populate this with per-channel residual statistics
/// (mean, RMS, peak), divergence indicators, and a structured diff.
#[derive(Clone, Debug, Default)]
pub struct CompareFiltersReport {
    /// Validation label of the comparison itself.
    pub validation: ValidationStatus,
    /// Optional human-readable summary.
    pub summary: Option<String>,
}

impl CompareFiltersReport {
    /// Construct a placeholder report with `experimental` validation
    /// status.
    #[must_use]
    pub fn placeholder() -> Self {
        Self::default()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn placeholder_is_experimental() {
        let r = CompareFiltersReport::placeholder();
        assert_eq!(r.validation, ValidationStatus::Experimental);
        assert!(r.summary.is_none());
    }
}
