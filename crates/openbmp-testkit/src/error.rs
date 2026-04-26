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
    /// A named metric was not declared in the tolerance table.
    #[error("metric '{name}' is not present in the tolerance table")]
    UnknownMetric {
        /// Metric name that was looked up.
        name: String,
    },
    /// A metric value was outside its declared tolerance envelope.
    #[error(
        "metric '{name}' out of tolerance: expected={expected}, actual={actual}, \
         |Δ|={abs_diff} (tol abs={absolute_tolerance}, rel={relative_tolerance})"
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
        /// Declared absolute tolerance.
        absolute_tolerance: f64,
        /// Declared relative tolerance.
        relative_tolerance: f64,
    },
}
