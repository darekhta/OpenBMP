//! Errors produced by the testkit helpers.

use thiserror::Error;

/// Errors produced by [`crate::tolerance`] and other testkit helpers.
#[derive(Error, Debug)]
pub enum TestkitError {
    /// Failed to read a tolerance-table file.
    #[error("failed to read tolerance file '{path}': {source}")]
    ReadFile {
        /// Path that was attempted.
        path: String,
        /// Underlying IO error.
        #[source]
        source: std::io::Error,
    },
    /// Failed to parse a tolerance-table TOML document.
    #[error("failed to parse tolerance table: {source}")]
    ParseToml {
        /// Underlying parser error.
        #[source]
        source: toml::de::Error,
    },
    /// A tolerance table had no metrics.
    #[error("tolerance table must declare at least one metric")]
    EmptyToleranceTable,
    /// A metric entry had an empty name.
    #[error("tolerance table metric names must not be empty")]
    EmptyMetricName,
    /// A metric name appeared more than once.
    #[error("duplicate tolerance metric '{name}'")]
    DuplicateMetric {
        /// Duplicated metric name.
        name: String,
    },
    /// A metric entry contained a non-finite value or a negative
    /// tolerance.
    #[error("metric '{name}' has invalid {field}: {value}")]
    InvalidMetricValue {
        /// Metric name.
        name: String,
        /// Invalid field name.
        field: &'static str,
        /// Invalid value.
        value: f64,
    },
    /// A code-verification input was non-finite, non-positive, or
    /// structurally invalid.
    #[error("invalid verification input '{field}'={value}: {rule}")]
    InvalidVerificationInput {
        /// Invalid field name.
        field: &'static str,
        /// Invalid value.
        value: f64,
        /// Required rule.
        rule: &'static str,
    },
    /// A reconstruction / filter-consistency input had invalid shape,
    /// dimension, or numeric content.
    #[error("invalid reconstruction input '{field}': {rule}")]
    InvalidReconstructionInput {
        /// Invalid field name.
        field: &'static str,
        /// Required rule.
        rule: &'static str,
    },
    /// A reconstruction / filter-consistency linear solve failed.
    #[error("reconstruction solve failed for '{field}'")]
    ReconstructionSolveFailed {
        /// Failed solve context.
        field: &'static str,
    },
    /// A named metric was not declared in the tolerance table.
    #[error("metric '{name}' is not present in the tolerance table")]
    UnknownMetric {
        /// Metric name that was looked up.
        name: String,
    },
    /// The actual value supplied for a metric check was not finite.
    #[error("metric '{name}' actual value is not finite: {actual}")]
    MetricActualNotFinite {
        /// Metric name.
        name: String,
        /// Actual measured value.
        actual: f64,
    },
    /// A metric value was outside its declared tolerance envelope.
    #[error(
        "metric '{name}' out of tolerance: expected={expected}, actual={actual}, \
         |delta|={abs_diff}, rel={rel_diff} \
         (tol abs={absolute_tolerance}, rel={relative_tolerance})"
    )]
    MetricOutOfTolerance {
        /// Metric name.
        name: String,
        /// Expected reference value.
        expected: f64,
        /// Actual measured value.
        actual: f64,
        /// `|expected - actual|`.
        abs_diff: f64,
        /// Relative difference.
        rel_diff: f64,
        /// Declared absolute tolerance.
        absolute_tolerance: f64,
        /// Declared relative tolerance.
        relative_tolerance: f64,
    },
}
