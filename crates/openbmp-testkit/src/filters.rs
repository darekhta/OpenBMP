//! Placeholder for the `compare_filters` harness.
//!
//! The full filter-comparison harness (NorthStarUAS pattern) builds on
//! the `openbmp-fc` estimators. This module currently
//! exposes only the type sketches that the harness fills in.

use openbmp_core::ValidationStatus;

/// Residual summary for one estimator output channel.
#[derive(Clone, Debug, PartialEq)]
pub struct FilterMetricSummary {
    /// Metric or channel name.
    pub name: String,
    /// Mean residual.
    pub mean_residual: f64,
    /// Root-mean-square residual.
    pub rms_residual: f64,
    /// Peak absolute residual.
    pub peak_abs_residual: f64,
}

/// Placeholder report describing a side-by-side estimator comparison.
///
/// The harness populates this with per-channel residual statistics
/// (mean, RMS, peak), divergence indicators, and a structured diff.
#[derive(Clone, Debug, Default)]
pub struct CompareFiltersReport {
    /// Validation label of the comparison itself.
    pub validation: ValidationStatus,
    /// Whether the compared filters diverged under the selected
    /// tolerance policy.
    pub diverged: bool,
    /// Per-channel residual summaries.
    pub metrics: Vec<FilterMetricSummary>,
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

    /// Returns `true` if this is still only an empty placeholder.
    #[must_use]
    pub fn is_placeholder(&self) -> bool {
        !self.diverged && self.metrics.is_empty() && self.summary.is_none()
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
        assert!(r.metrics.is_empty());
        assert!(!r.diverged);
        assert!(r.is_placeholder());
    }
}
