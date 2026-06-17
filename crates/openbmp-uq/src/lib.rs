//! `openbmp-uq` — source-tagged uncertainty budgets and credibility records.
//!
//! This crate owns OpenBMP's low-layer uncertainty accounting primitives:
//! correlated one-sigma aggregation, aleatory/epistemic classification, and
//! NASA-STD-7009B-shaped credibility records. It is pure data and math: no
//! scenario parser, runner, telemetry writer, wall-clock access, system RNG, or
//! simulator dependency lives here.

#![cfg_attr(not(feature = "std"), no_std)]
#![deny(unsafe_code)]

extern crate alloc;

use alloc::collections::BTreeMap;
use alloc::format;
use alloc::string::String;
use alloc::vec;
use alloc::vec::Vec;
use core::fmt::Write;

pub use openbmp_core::ValidationStatus;
use thiserror::Error;

const CORRELATION_TOLERANCE: f64 = 1.0e-12;
const PSD_TOLERANCE: f64 = 1.0e-10;

/// Error returned by UQ budget and credibility helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum UqError {
    /// A source, matrix, or credibility field is malformed.
    #[error("invalid UQ input: {field}")]
    InvalidInput {
        /// Field that failed validation.
        field: &'static str,
    },
    /// A matrix shape does not match the declared dimension or source count.
    #[error("UQ matrix dimension mismatch: {field}")]
    DimensionMismatch {
        /// Field that failed validation.
        field: &'static str,
    },
    /// A correlation matrix is not symmetric within the deterministic tolerance.
    #[error("correlation matrix is not symmetric at ({row}, {column})")]
    NonSymmetricCorrelation {
        /// Row index.
        row: usize,
        /// Column index.
        column: usize,
    },
    /// A correlation matrix is not positive semidefinite.
    #[error("correlation matrix is not positive semidefinite; min eigenvalue {min_eigenvalue}")]
    NonPositiveSemidefinite {
        /// Smallest estimated eigenvalue.
        min_eigenvalue: f64,
    },
    /// A calculation overflowed to NaN or infinity.
    #[error("UQ calculation produced non-finite output: {field}")]
    NonFinite {
        /// Field that failed validation.
        field: &'static str,
    },
}

/// Aleatory versus epistemic classification of an uncertainty source.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Hash)]
pub enum UncertaintyClass {
    /// Irreducible run-to-run variability.
    Aleatory,
    /// Reducible lack-of-knowledge or model-form uncertainty.
    Epistemic,
}

impl UncertaintyClass {
    /// Canonical label for reports.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::Aleatory => "aleatory",
            Self::Epistemic => "epistemic",
        }
    }
}

/// NASA-STD-7009B-style credibility level.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Hash)]
pub enum CredibilityLevel {
    /// Insufficient evidence or absent factor.
    L0,
    /// Minimal evidence.
    L1,
    /// Basic evidence suitable for toy validation claims.
    L2,
    /// Research-grade public benchmark or equivalent evidence.
    L3,
    /// Full program-specific evidence; retained for matrix completeness.
    L4,
}

impl CredibilityLevel {
    /// Convert a numeric 7009B-style level to the enum.
    #[must_use]
    pub const fn from_value(value: u8) -> Option<Self> {
        match value {
            0 => Some(Self::L0),
            1 => Some(Self::L1),
            2 => Some(Self::L2),
            3 => Some(Self::L3),
            4 => Some(Self::L4),
            _ => None,
        }
    }

    /// Numeric level value.
    #[must_use]
    pub const fn value(self) -> u8 {
        match self {
            Self::L0 => 0,
            Self::L1 => 1,
            Self::L2 => 2,
            Self::L3 => 3,
            Self::L4 => 4,
        }
    }

    /// Canonical label for reports.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::L0 => "L0",
            Self::L1 => "L1",
            Self::L2 => "L2",
            Self::L3 => "L3",
            Self::L4 => "L4",
        }
    }
}

/// NASA-STD-7009B-shaped credibility factor.
#[derive(Copy, Clone, Debug, Eq, Ord, PartialEq, PartialOrd, Hash)]
pub enum CredibilityFactor {
    /// Code and solution verification evidence.
    Verification,
    /// Validation or comparison against reference data.
    Validation,
    /// Provenance and quality of inputs.
    InputPedigree,
    /// Uncertainty quantification completeness.
    ResultsUncertainty,
    /// Sensitivity and robustness evidence.
    ResultsRobustness,
    /// Prior use history.
    UseHistory,
    /// Modeling-and-simulation management process evidence.
    MsManagement,
    /// People or reviewer qualification evidence.
    PeopleQualification,
}

impl CredibilityFactor {
    /// All factors in deterministic report order.
    pub const ALL: [Self; 8] = [
        Self::Verification,
        Self::Validation,
        Self::InputPedigree,
        Self::ResultsUncertainty,
        Self::ResultsRobustness,
        Self::UseHistory,
        Self::MsManagement,
        Self::PeopleQualification,
    ];

    /// Parse a stable lower-snake-case manifest key.
    #[must_use]
    pub fn from_key(value: &str) -> Option<Self> {
        match value {
            "verification" => Some(Self::Verification),
            "validation" => Some(Self::Validation),
            "input_pedigree" => Some(Self::InputPedigree),
            "results_uncertainty" => Some(Self::ResultsUncertainty),
            "results_robustness" => Some(Self::ResultsRobustness),
            "use_history" => Some(Self::UseHistory),
            "ms_management" => Some(Self::MsManagement),
            "people_qualification" => Some(Self::PeopleQualification),
            _ => None,
        }
    }

    /// Stable lower-snake-case manifest key.
    #[must_use]
    pub const fn as_key(self) -> &'static str {
        match self {
            Self::Verification => "verification",
            Self::Validation => "validation",
            Self::InputPedigree => "input_pedigree",
            Self::ResultsUncertainty => "results_uncertainty",
            Self::ResultsRobustness => "results_robustness",
            Self::UseHistory => "use_history",
            Self::MsManagement => "ms_management",
            Self::PeopleQualification => "people_qualification",
        }
    }

    /// Canonical label for reports.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::Verification => "Verification",
            Self::Validation => "Validation",
            Self::InputPedigree => "InputPedigree",
            Self::ResultsUncertainty => "ResultsUncertainty",
            Self::ResultsRobustness => "ResultsRobustness",
            Self::UseHistory => "UseHistory",
            Self::MsManagement => "MsManagement",
            Self::PeopleQualification => "PeopleQualification",
        }
    }
}

/// Credibility evidence scored across all 7009B-shaped factors.
#[derive(Clone, Debug, Default, Eq, PartialEq)]
pub struct CredibilityRecord {
    /// Level per factor; absent factors bind as [`CredibilityLevel::L0`].
    pub scores: BTreeMap<CredibilityFactor, CredibilityLevel>,
    /// Evidence pointer per scored factor.
    pub evidence: BTreeMap<CredibilityFactor, String>,
}

impl CredibilityRecord {
    /// Construct an empty record. Every absent factor binds at L0.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            scores: BTreeMap::new(),
            evidence: BTreeMap::new(),
        }
    }

    /// Add or replace one scored factor.
    #[must_use]
    pub fn with_score(
        mut self,
        factor: CredibilityFactor,
        level: CredibilityLevel,
        evidence: impl Into<String>,
    ) -> Self {
        self.scores.insert(factor, level);
        self.evidence.insert(factor, evidence.into());
        self
    }

    /// Return one factor score; absent factors bind as L0.
    #[must_use]
    pub fn score(&self, factor: CredibilityFactor) -> CredibilityLevel {
        self.scores
            .get(&factor)
            .copied()
            .unwrap_or(CredibilityLevel::L0)
    }

    /// Binding credibility: the minimum level across all factors.
    #[must_use]
    pub fn binding_level(&self) -> CredibilityLevel {
        CredibilityFactor::ALL
            .iter()
            .map(|factor| self.score(*factor))
            .min()
            .unwrap_or(CredibilityLevel::L0)
    }

    /// Map the binding level onto OpenBMP's four public validation labels.
    #[must_use]
    pub fn legacy_label(&self) -> ValidationStatus {
        match self.binding_level() {
            CredibilityLevel::L0 => ValidationStatus::Experimental,
            CredibilityLevel::L1 => ValidationStatus::Checked,
            CredibilityLevel::L2 => ValidationStatus::ValidatedToy,
            CredibilityLevel::L3 | CredibilityLevel::L4 => ValidationStatus::Research,
        }
    }

    /// Validate evidence consistency.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when a non-zero score has no evidence pointer.
    pub fn validate(&self) -> Result<(), UqError> {
        for factor in CredibilityFactor::ALL {
            if self.score(factor) > CredibilityLevel::L0 {
                let Some(evidence) = self.evidence.get(&factor) else {
                    return Err(UqError::InvalidInput {
                        field: "credibility.evidence",
                    });
                };
                if evidence.trim().is_empty() {
                    return Err(UqError::InvalidInput {
                        field: "credibility.evidence",
                    });
                }
            }
        }
        Ok(())
    }

    /// Render a deterministic Markdown credibility matrix.
    #[must_use]
    pub fn render_markdown(&self) -> String {
        let mut out = String::new();
        let _ = writeln!(out, "| Factor | Level | Evidence |");
        let _ = writeln!(out, "|---|---:|---|");
        for factor in CredibilityFactor::ALL {
            let evidence = self.evidence.get(&factor).map_or("", String::as_str);
            let _ = writeln!(
                out,
                "| {} | {} | {} |",
                factor.as_label(),
                self.score(factor).as_label(),
                evidence
            );
        }
        let _ = writeln!(out);
        let _ = writeln!(
            out,
            "**Binding credibility:** {} ({})",
            self.binding_level().as_label(),
            self.legacy_label().as_label()
        );
        out
    }
}

/// One source's contribution in the source-tagged correlated error budget.
#[derive(Clone, Debug, PartialEq)]
pub struct UncertaintySource {
    /// Stable source identifier.
    pub source_id: String,
    /// One-sigma contribution in the quantity of interest's units.
    pub one_sigma: f64,
    /// Aleatory or epistemic classification.
    pub class: UncertaintyClass,
    /// Credibility evidence for this source.
    pub credibility: CredibilityRecord,
    /// Human-readable evidence pointer or justification.
    pub justification: String,
}

impl UncertaintySource {
    /// Validate source fields.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the source id, one-sigma value, justification,
    /// or credibility evidence is malformed.
    pub fn validate(&self) -> Result<(), UqError> {
        if self.source_id.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "source.source_id",
            });
        }
        if !self.one_sigma.is_finite() || self.one_sigma < 0.0 {
            return Err(UqError::InvalidInput {
                field: "source.one_sigma",
            });
        }
        if self.justification.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "source.justification",
            });
        }
        self.credibility.validate()
    }
}

/// Bounded uncertainty band emitted by an upstream discipline deck.
#[derive(Clone, Debug, PartialEq)]
pub struct UpstreamMargin {
    /// Stable source identifier for the contributing deck entry.
    pub source_id: String,
    /// Quantity represented by the band, such as `c_star_efficiency`.
    pub quantity_id: String,
    /// Nominal value used by the deterministic model.
    pub nominal: f64,
    /// Lower bound for the quantity.
    pub lower: f64,
    /// Upper bound for the quantity.
    pub upper: f64,
    /// Aleatory or epistemic classification.
    pub class: UncertaintyClass,
    /// Credibility evidence inherited from the upstream deck.
    pub credibility: CredibilityRecord,
    /// Human-readable upstream evidence pointer.
    pub justification: String,
}

impl UpstreamMargin {
    /// Validate the bounded upstream margin.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when identifiers are empty, values are non-finite,
    /// the band is not ordered as `lower <= nominal <= upper`, or credibility
    /// evidence is incomplete.
    pub fn validate(&self) -> Result<(), UqError> {
        if self.source_id.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "upstream_margin.source_id",
            });
        }
        if self.quantity_id.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "upstream_margin.quantity_id",
            });
        }
        if !self.nominal.is_finite() || !self.lower.is_finite() || !self.upper.is_finite() {
            return Err(UqError::InvalidInput {
                field: "upstream_margin.value",
            });
        }
        if self.lower > self.nominal || self.nominal > self.upper {
            return Err(UqError::InvalidInput {
                field: "upstream_margin.bounds",
            });
        }
        if self.justification.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "upstream_margin.justification",
            });
        }
        self.credibility.validate()
    }

    /// Conservative one-sigma surrogate for the bounded margin.
    ///
    /// This intentionally preserves the full larger side of the bound as the
    /// source contribution instead of assuming a distribution inside the band.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the margin is malformed or the width is
    /// non-finite.
    pub fn conservative_one_sigma(&self) -> Result<f64, UqError> {
        self.validate()?;
        let one_sigma = (self.nominal - self.lower)
            .abs()
            .max((self.upper - self.nominal).abs());
        if one_sigma.is_finite() {
            Ok(one_sigma)
        } else {
            Err(UqError::NonFinite {
                field: "upstream_margin.one_sigma",
            })
        }
    }

    /// Convert the upstream band into a source-tagged UQ contribution.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the margin or produced source is malformed.
    pub fn into_uncertainty_source(self) -> Result<UncertaintySource, UqError> {
        let one_sigma = self.conservative_one_sigma()?;
        let source = UncertaintySource {
            source_id: self.source_id,
            one_sigma,
            class: self.class,
            credibility: self.credibility,
            justification: self.justification,
        };
        source.validate()?;
        Ok(source)
    }
}

/// Build a credibility record that assigns the same score to every factor.
#[must_use]
pub fn uniform_credibility_record(
    level: CredibilityLevel,
    evidence: impl Into<String>,
) -> CredibilityRecord {
    let evidence = evidence.into();
    let mut record = CredibilityRecord::new();
    for factor in CredibilityFactor::ALL {
        record = record.with_score(factor, level, evidence.clone());
    }
    record
}

/// Convert a propulsion thermochemistry `c*` efficiency band into UQ source form.
///
/// The source is epistemic because the band represents model-form /
/// calibration uncertainty in the upstream propulsion deck, not run-to-run
/// variability.
///
/// # Errors
///
/// Returns [`UqError`] when the deck id, evidence, efficiency band, or
/// resulting source is malformed.
pub fn propulsion_c_star_efficiency_margin_source(
    deck_id: impl Into<String>,
    min_efficiency: f64,
    nominal_efficiency: f64,
    max_efficiency: f64,
    credibility_level: CredibilityLevel,
    evidence: impl Into<String>,
) -> Result<UncertaintySource, UqError> {
    let deck_id = deck_id.into();
    if deck_id.trim().is_empty() {
        return Err(UqError::InvalidInput {
            field: "propulsion_c_star.deck_id",
        });
    }
    let evidence = evidence.into();
    let source_id = format!("05.propulsion.{deck_id}.c_star_efficiency");
    UpstreamMargin {
        source_id,
        quantity_id: "c_star_efficiency".into(),
        nominal: nominal_efficiency,
        lower: min_efficiency,
        upper: max_efficiency,
        class: UncertaintyClass::Epistemic,
        credibility: uniform_credibility_record(credibility_level, evidence.clone()),
        justification: evidence,
    }
    .into_uncertainty_source()
}

/// Symmetric positive-semidefinite correlation matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct CorrelationMatrix {
    dimension: usize,
    values: Vec<f64>,
}

impl CorrelationMatrix {
    /// Construct and validate a row-major correlation matrix.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the matrix shape, entries, symmetry, diagonal,
    /// bounds, or positive-semidefinite check fails.
    pub fn new(dimension: usize, values: Vec<f64>) -> Result<Self, UqError> {
        if dimension == 0 {
            return Err(UqError::InvalidInput {
                field: "correlation.dimension",
            });
        }
        if values.len()
            != dimension
                .checked_mul(dimension)
                .ok_or(UqError::InvalidInput {
                    field: "correlation.dimension",
                })?
        {
            return Err(UqError::DimensionMismatch {
                field: "correlation.values",
            });
        }
        let matrix = Self { dimension, values };
        matrix.validate_entries()?;
        matrix.validate_positive_semidefinite()?;
        Ok(matrix)
    }

    /// Construct an identity correlation matrix.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when `dimension` is zero.
    pub fn identity(dimension: usize) -> Result<Self, UqError> {
        let mut values = vec![
            0.0;
            dimension
                .checked_mul(dimension)
                .ok_or(UqError::InvalidInput {
                    field: "correlation.dimension",
                },)?
        ];
        for index in 0..dimension {
            values[index * dimension + index] = 1.0;
        }
        Self::new(dimension, values)
    }

    /// Matrix dimension.
    #[must_use]
    pub const fn dimension(&self) -> usize {
        self.dimension
    }

    /// Row-major values.
    #[must_use]
    pub fn values(&self) -> &[f64] {
        &self.values
    }

    /// Return a matrix entry.
    #[must_use]
    pub fn get(&self, row: usize, column: usize) -> Option<f64> {
        if row >= self.dimension || column >= self.dimension {
            return None;
        }
        self.values.get(row * self.dimension + column).copied()
    }

    fn entry(&self, row: usize, column: usize) -> f64 {
        self.values[row * self.dimension + column]
    }

    fn validate_entries(&self) -> Result<(), UqError> {
        for row in 0..self.dimension {
            for column in 0..self.dimension {
                let value = self.entry(row, column);
                if !value.is_finite() {
                    return Err(UqError::InvalidInput {
                        field: "correlation.value",
                    });
                }
                if value.abs() > 1.0 + CORRELATION_TOLERANCE {
                    return Err(UqError::InvalidInput {
                        field: "correlation.value",
                    });
                }
                if row == column && (value - 1.0).abs() > CORRELATION_TOLERANCE {
                    return Err(UqError::InvalidInput {
                        field: "correlation.diagonal",
                    });
                }
                let transpose = self.entry(column, row);
                if (value - transpose).abs() > CORRELATION_TOLERANCE {
                    return Err(UqError::NonSymmetricCorrelation { row, column });
                }
            }
        }
        Ok(())
    }

    fn validate_positive_semidefinite(&self) -> Result<(), UqError> {
        let min_eigenvalue = jacobi_min_eigenvalue(self.dimension, self.values.clone())?;
        if min_eigenvalue < -PSD_TOLERANCE {
            Err(UqError::NonPositiveSemidefinite { min_eigenvalue })
        } else {
            Ok(())
        }
    }
}

/// Source-tagged correlated error budget.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct CorrelatedErrorBudget {
    /// Uncertainty sources in matrix row/column order.
    pub sources: Vec<UncertaintySource>,
    /// Optional correlation matrix. `None` means identity.
    pub correlation: Option<CorrelationMatrix>,
}

impl CorrelatedErrorBudget {
    /// Aggregate one-sigma value over every source.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when sources or correlation dimensions are invalid,
    /// or when the quadratic form becomes non-finite.
    pub fn aggregate_one_sigma(&self) -> Result<f64, UqError> {
        self.aggregate_filtered(|_| true)
    }

    /// Aggregate the aleatory source subset.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when sources or correlation dimensions are invalid.
    pub fn aleatory_one_sigma(&self) -> Result<f64, UqError> {
        self.aggregate_filtered(|source| source.class == UncertaintyClass::Aleatory)
    }

    /// Aggregate the epistemic source subset.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when sources or correlation dimensions are invalid.
    pub fn epistemic_one_sigma(&self) -> Result<f64, UqError> {
        self.aggregate_filtered(|source| source.class == UncertaintyClass::Epistemic)
    }

    /// Binding credibility across all sources, by factor-wise minimum.
    #[must_use]
    pub fn binding_credibility(&self) -> Option<CredibilityRecord> {
        if self.sources.is_empty() {
            return None;
        }
        let mut record = CredibilityRecord::new();
        for factor in CredibilityFactor::ALL {
            let mut min_level = CredibilityLevel::L4;
            let mut evidence = String::new();
            for source in &self.sources {
                let level = source.credibility.score(factor);
                if level <= min_level {
                    min_level = level;
                    evidence = source.credibility.evidence.get(&factor).map_or_else(
                        || format!("{}: no evidence", source.source_id),
                        |item| format!("{}: {item}", source.source_id),
                    );
                }
            }
            record = record.with_score(factor, min_level, evidence);
        }
        Some(record)
    }

    fn validate(&self) -> Result<(), UqError> {
        let mut seen = alloc::collections::BTreeSet::new();
        for source in &self.sources {
            source.validate()?;
            if !seen.insert(source.source_id.as_str()) {
                return Err(UqError::InvalidInput {
                    field: "source.source_id",
                });
            }
        }
        if let Some(correlation) = &self.correlation
            && correlation.dimension() != self.sources.len()
        {
            return Err(UqError::DimensionMismatch {
                field: "correlation.dimension",
            });
        }
        Ok(())
    }

    fn aggregate_filtered(
        &self,
        include: impl Fn(&UncertaintySource) -> bool,
    ) -> Result<f64, UqError> {
        self.validate()?;
        if self.sources.is_empty() {
            return Ok(0.0);
        }
        let mut quadratic = 0.0;
        for (row, row_source) in self.sources.iter().enumerate() {
            if !include(row_source) {
                continue;
            }
            for (column, column_source) in self.sources.iter().enumerate() {
                if !include(column_source) {
                    continue;
                }
                let rho = self
                    .correlation
                    .as_ref()
                    .map_or(if row == column { 1.0 } else { 0.0 }, |matrix| {
                        matrix.entry(row, column)
                    });
                quadratic += row_source.one_sigma * rho * column_source.one_sigma;
            }
        }
        if !quadratic.is_finite() {
            return Err(UqError::NonFinite {
                field: "budget.quadratic_form",
            });
        }
        if quadratic < -PSD_TOLERANCE {
            return Err(UqError::NonPositiveSemidefinite {
                min_eigenvalue: quadratic,
            });
        }
        Ok(sqrt_f64(quadratic.max(0.0)))
    }
}

/// Probability-box envelope over nested conditional scalar CDFs.
#[derive(Clone, Debug, PartialEq)]
pub struct ProbabilityBox {
    /// Sorted scalar support grid.
    pub support: Vec<f64>,
    /// Lower empirical CDF envelope, `min_theta F(y|theta)`.
    pub lower_cdf: Vec<f64>,
    /// Upper empirical CDF envelope, `max_theta F(y|theta)`.
    pub upper_cdf: Vec<f64>,
}

impl ProbabilityBox {
    /// Build a p-box from one inner aleatory sample set per epistemic sample.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when no epistemic condition is supplied, any
    /// conditional sample set is empty, or any sample is non-finite.
    pub fn from_conditional_samples(conditional_samples: &[Vec<f64>]) -> Result<Self, UqError> {
        validate_conditional_samples(conditional_samples)?;
        let mut support = Vec::new();
        for samples in conditional_samples {
            support.extend(samples.iter().copied());
        }
        support.sort_by(f64::total_cmp);
        support.dedup_by(|left, right| left.total_cmp(right).is_eq());

        let mut lower_cdf = Vec::with_capacity(support.len());
        let mut upper_cdf = Vec::with_capacity(support.len());
        for threshold in &support {
            let mut lower = 1.0;
            let mut upper = 0.0;
            for samples in conditional_samples {
                let cdf = empirical_cdf(samples, *threshold);
                if cdf < lower {
                    lower = cdf;
                }
                if cdf > upper {
                    upper = cdf;
                }
            }
            lower_cdf.push(lower);
            upper_cdf.push(upper);
        }

        Ok(Self {
            support,
            lower_cdf,
            upper_cdf,
        })
    }

    /// Lower p-box bound at an arbitrary scalar threshold.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the p-box is malformed or the threshold is
    /// non-finite.
    pub fn lower_cdf_at(&self, threshold: f64) -> Result<f64, UqError> {
        self.cdf_at(&self.lower_cdf, threshold)
    }

    /// Upper p-box bound at an arbitrary scalar threshold.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the p-box is malformed or the threshold is
    /// non-finite.
    pub fn upper_cdf_at(&self, threshold: f64) -> Result<f64, UqError> {
        self.cdf_at(&self.upper_cdf, threshold)
    }

    /// Verify a lower-tail probability requirement on the lower p-box bound.
    ///
    /// This intentionally uses `F_lo(threshold)`, not the mean conditional CDF.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when the p-box, threshold, or probability target is
    /// malformed.
    pub fn verify_lower_tail_probability(
        &self,
        threshold: f64,
        minimum_probability: f64,
    ) -> Result<bool, UqError> {
        validate_probability(minimum_probability)?;
        Ok(self.lower_cdf_at(threshold)? >= minimum_probability)
    }

    fn validate(&self) -> Result<(), UqError> {
        if self.support.is_empty()
            || self.support.len() != self.lower_cdf.len()
            || self.support.len() != self.upper_cdf.len()
        {
            return Err(UqError::DimensionMismatch {
                field: "pbox.support",
            });
        }
        for (index, support) in self.support.iter().copied().enumerate() {
            if !support.is_finite()
                || !self.lower_cdf[index].is_finite()
                || !self.upper_cdf[index].is_finite()
            {
                return Err(UqError::InvalidInput {
                    field: "pbox.value",
                });
            }
            validate_probability(self.lower_cdf[index])?;
            validate_probability(self.upper_cdf[index])?;
            if self.lower_cdf[index] > self.upper_cdf[index] {
                return Err(UqError::InvalidInput {
                    field: "pbox.envelope",
                });
            }
            if index > 0 {
                if self.support[index - 1] >= support {
                    return Err(UqError::InvalidInput {
                        field: "pbox.support",
                    });
                }
                if self.lower_cdf[index] < self.lower_cdf[index - 1]
                    || self.upper_cdf[index] < self.upper_cdf[index - 1]
                {
                    return Err(UqError::InvalidInput {
                        field: "pbox.monotonicity",
                    });
                }
            }
        }
        Ok(())
    }

    fn cdf_at(&self, cdf: &[f64], threshold: f64) -> Result<f64, UqError> {
        self.validate()?;
        if !threshold.is_finite() {
            return Err(UqError::InvalidInput {
                field: "pbox.threshold",
            });
        }
        match self
            .support
            .binary_search_by(|support| support.total_cmp(&threshold))
        {
            Ok(index) => Ok(cdf[index]),
            Err(0) => Ok(0.0),
            Err(index) => Ok(cdf[index - 1]),
        }
    }
}

/// Aleatory/epistemic variance split from a nested scalar ensemble.
#[derive(Clone, Debug, PartialEq)]
pub struct VarianceSplit {
    /// `E_theta[Var_a(y|theta)]`.
    pub aleatory: f64,
    /// `Var_theta[E_a(y|theta)]`.
    pub epistemic: f64,
    /// Direct nested total variance.
    pub total: f64,
}

impl VarianceSplit {
    /// Compute the law-of-total-variance split from nested scalar samples.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when no epistemic condition is supplied, any
    /// conditional sample set is empty, or any sample is non-finite.
    pub fn from_conditional_samples(conditional_samples: &[Vec<f64>]) -> Result<Self, UqError> {
        validate_conditional_samples(conditional_samples)?;

        let mut means = Vec::with_capacity(conditional_samples.len());
        let mut variances = Vec::with_capacity(conditional_samples.len());
        for samples in conditional_samples {
            let (mean, variance) = mean_and_population_variance(samples);
            means.push(mean);
            variances.push(variance);
        }
        let epistemic_count = means.len() as f64;
        let global_mean = means.iter().sum::<f64>() / epistemic_count;
        let aleatory = variances.iter().sum::<f64>() / epistemic_count;
        let epistemic = means
            .iter()
            .map(|mean| {
                let delta = mean - global_mean;
                delta * delta
            })
            .sum::<f64>()
            / epistemic_count;

        let mut total = 0.0;
        for samples in conditional_samples {
            let inner_count = samples.len() as f64;
            total += samples
                .iter()
                .map(|sample| {
                    let delta = sample - global_mean;
                    delta * delta
                })
                .sum::<f64>()
                / inner_count;
        }
        total /= epistemic_count;

        if !aleatory.is_finite() || !epistemic.is_finite() || !total.is_finite() {
            return Err(UqError::NonFinite {
                field: "variance_split",
            });
        }
        Ok(Self {
            aleatory,
            epistemic,
            total,
        })
    }

    /// Absolute residual in `total == aleatory + epistemic`.
    #[must_use]
    pub fn identity_residual(&self) -> f64 {
        (self.total - (self.aleatory + self.epistemic)).abs()
    }
}

fn validate_conditional_samples(conditional_samples: &[Vec<f64>]) -> Result<(), UqError> {
    if conditional_samples.is_empty() {
        return Err(UqError::InvalidInput {
            field: "nested.epistemic_samples",
        });
    }
    for samples in conditional_samples {
        if samples.is_empty() {
            return Err(UqError::InvalidInput {
                field: "nested.aleatory_samples",
            });
        }
        for sample in samples {
            if !sample.is_finite() {
                return Err(UqError::InvalidInput {
                    field: "nested.sample",
                });
            }
        }
    }
    Ok(())
}

fn validate_probability(value: f64) -> Result<(), UqError> {
    if value.is_finite() && (0.0..=1.0).contains(&value) {
        Ok(())
    } else {
        Err(UqError::InvalidInput {
            field: "probability",
        })
    }
}

fn empirical_cdf(samples: &[f64], threshold: f64) -> f64 {
    samples
        .iter()
        .filter(|sample| **sample <= threshold)
        .count() as f64
        / samples.len() as f64
}

fn mean_and_population_variance(samples: &[f64]) -> (f64, f64) {
    let count = samples.len() as f64;
    let mean = samples.iter().sum::<f64>() / count;
    let variance = samples
        .iter()
        .map(|sample| {
            let delta = sample - mean;
            delta * delta
        })
        .sum::<f64>()
        / count;
    (mean, variance)
}

/// Legacy one-source uncertainty contribution shape retained for compatibility.
#[derive(Clone, Debug, PartialEq)]
pub struct UncertaintyContribution {
    /// Model identifier.
    pub model_id: String,
    /// One-sigma contribution.
    pub one_sigma: f64,
    /// Public validation label.
    pub status: ValidationStatus,
    /// Evidence pointer or justification.
    pub justification: String,
}

impl UncertaintyContribution {
    /// Validate the contribution.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when fields are malformed.
    pub fn validate(&self) -> Result<(), UqError> {
        if self.model_id.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "contribution.model_id",
            });
        }
        if !self.one_sigma.is_finite() || self.one_sigma < 0.0 {
            return Err(UqError::InvalidInput {
                field: "contribution.one_sigma",
            });
        }
        if self.justification.trim().is_empty() {
            return Err(UqError::InvalidInput {
                field: "contribution.justification",
            });
        }
        Ok(())
    }
}

/// Legacy flat RSS plus correlated-bias error budget.
#[derive(Clone, Debug, Default, PartialEq)]
pub struct ErrorBudget {
    /// Per-model independent contributions.
    pub contributions: Vec<UncertaintyContribution>,
    /// Monolithic correlated additive bias.
    pub correlated_bias: f64,
}

impl ErrorBudget {
    /// Aggregate uncertainty using the legacy `sqrt(sum sigma^2) + |bias|`.
    ///
    /// # Errors
    ///
    /// Returns [`UqError`] when contributions or bias are malformed.
    pub fn aggregate_one_sigma(&self) -> Result<f64, UqError> {
        for contribution in &self.contributions {
            contribution.validate()?;
        }
        if !self.correlated_bias.is_finite() {
            return Err(UqError::InvalidInput {
                field: "budget.correlated_bias",
            });
        }
        let sum_sq: f64 = self
            .contributions
            .iter()
            .map(|contribution| contribution.one_sigma * contribution.one_sigma)
            .sum();
        let aggregate = sqrt_f64(sum_sq) + self.correlated_bias.abs();
        if aggregate.is_finite() {
            Ok(aggregate)
        } else {
            Err(UqError::NonFinite {
                field: "budget.aggregate_one_sigma",
            })
        }
    }

    /// Minimum public validation status across all contributions.
    #[must_use]
    pub fn minimum_status(&self) -> Option<ValidationStatus> {
        self.contributions
            .iter()
            .map(|contribution| contribution.status)
            .min_by_key(|status| validation_rank(*status))
    }

    /// Render a deterministic Markdown credibility report.
    #[must_use]
    pub fn render_markdown(&self, scenario_id: &str) -> String {
        let mut out = String::new();
        let _ = writeln!(out, "# Credibility report - {scenario_id}");
        let _ = writeln!(out);
        let _ = writeln!(
            out,
            "> Academic simulation; not validated for operational flight."
        );
        let _ = writeln!(out);
        let _ = writeln!(out, "| Model | 1-sigma | Status | Justification |");
        let _ = writeln!(out, "|---|---:|---|---|");
        for contribution in &self.contributions {
            let _ = writeln!(
                out,
                "| {} | {:.4e} | {} | {} |",
                contribution.model_id,
                contribution.one_sigma,
                contribution.status.as_label(),
                contribution.justification
            );
        }
        let _ = writeln!(out);
        if let Ok(aggregate) = self.aggregate_one_sigma() {
            let _ = writeln!(
                out,
                "**Aggregate 1-sigma (RSS + correlated bias):** {aggregate:.4e}"
            );
        }
        if let Some(status) = self.minimum_status() {
            let _ = writeln!(out, "**Minimum validation status:** {}", status.as_label());
        }
        out
    }
}

fn validation_rank(status: ValidationStatus) -> u8 {
    match status {
        ValidationStatus::Experimental => 0,
        ValidationStatus::Checked => 1,
        ValidationStatus::ValidatedToy => 2,
        ValidationStatus::Research => 3,
    }
}

fn sqrt_f64(value: f64) -> f64 {
    #[cfg(feature = "std")]
    {
        value.sqrt()
    }
    #[cfg(not(feature = "std"))]
    {
        num_traits::Float::sqrt(value)
    }
}

fn jacobi_min_eigenvalue(dimension: usize, mut matrix: Vec<f64>) -> Result<f64, UqError> {
    if dimension == 1 {
        return matrix
            .first()
            .copied()
            .filter(|value| value.is_finite())
            .ok_or(UqError::InvalidInput {
                field: "correlation.value",
            });
    }
    let max_sweeps = dimension
        .checked_mul(dimension)
        .and_then(|value| value.checked_mul(32))
        .ok_or(UqError::InvalidInput {
            field: "correlation.dimension",
        })?;
    for _ in 0..max_sweeps {
        let mut pivot_row = 0;
        let mut pivot_column = 1;
        let mut max_offdiag = 0.0;
        for row in 0..dimension {
            for column in (row + 1)..dimension {
                let value = matrix[row * dimension + column].abs();
                if value > max_offdiag {
                    max_offdiag = value;
                    pivot_row = row;
                    pivot_column = column;
                }
            }
        }
        if max_offdiag <= CORRELATION_TOLERANCE {
            break;
        }

        let pp = matrix[pivot_row * dimension + pivot_row];
        let qq = matrix[pivot_column * dimension + pivot_column];
        let pq = matrix[pivot_row * dimension + pivot_column];
        let tau = (qq - pp) / (2.0 * pq);
        let sign = if tau >= 0.0 { 1.0 } else { -1.0 };
        let t = sign / (tau.abs() + sqrt_f64(1.0 + tau * tau));
        let cosine = 1.0 / sqrt_f64(1.0 + t * t);
        let sine = t * cosine;

        for index in 0..dimension {
            if index != pivot_row && index != pivot_column {
                let ip = matrix[index * dimension + pivot_row];
                let iq = matrix[index * dimension + pivot_column];
                let new_ip = cosine * ip - sine * iq;
                let new_iq = sine * ip + cosine * iq;
                matrix[index * dimension + pivot_row] = new_ip;
                matrix[pivot_row * dimension + index] = new_ip;
                matrix[index * dimension + pivot_column] = new_iq;
                matrix[pivot_column * dimension + index] = new_iq;
            }
        }
        let new_pp = cosine * cosine * pp - 2.0 * sine * cosine * pq + sine * sine * qq;
        let new_qq = sine * sine * pp + 2.0 * sine * cosine * pq + cosine * cosine * qq;
        matrix[pivot_row * dimension + pivot_row] = new_pp;
        matrix[pivot_column * dimension + pivot_column] = new_qq;
        matrix[pivot_row * dimension + pivot_column] = 0.0;
        matrix[pivot_column * dimension + pivot_row] = 0.0;
    }

    let mut min_eigenvalue = matrix[0];
    for index in 1..dimension {
        let value = matrix[index * dimension + index];
        if !value.is_finite() {
            return Err(UqError::NonFinite {
                field: "correlation.eigenvalue",
            });
        }
        if value < min_eigenvalue {
            min_eigenvalue = value;
        }
    }
    Ok(min_eigenvalue)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    fn credibility(level: CredibilityLevel) -> CredibilityRecord {
        let mut record = CredibilityRecord::new();
        for factor in CredibilityFactor::ALL {
            record = record.with_score(factor, level, format!("V-{factor:?}"));
        }
        record
    }

    fn source(id: &str, sigma: f64, class: UncertaintyClass) -> UncertaintySource {
        UncertaintySource {
            source_id: String::from(id),
            one_sigma: sigma,
            class,
            credibility: credibility(CredibilityLevel::L2),
            justification: String::from("synthetic evidence"),
        }
    }

    fn contribution(id: &str, sigma: f64, status: ValidationStatus) -> UncertaintyContribution {
        UncertaintyContribution {
            model_id: String::from(id),
            one_sigma: sigma,
            status,
            justification: String::from("textbook reference"),
        }
    }

    #[test]
    fn legacy_budget_preserves_rss_plus_bias() {
        let budget = ErrorBudget {
            contributions: vec![
                contribution("a", 3.0, ValidationStatus::Research),
                contribution("b", 4.0, ValidationStatus::Research),
            ],
            correlated_bias: 1.0,
        };
        let aggregate = budget.aggregate_one_sigma().unwrap();
        assert!((aggregate - 6.0).abs() < 1.0e-12);
    }

    #[test]
    fn correlated_budget_identity_matches_legacy_rss_without_bias() {
        let budget = CorrelatedErrorBudget {
            sources: vec![
                source("a", 3.0, UncertaintyClass::Aleatory),
                source("b", 4.0, UncertaintyClass::Epistemic),
            ],
            correlation: None,
        };
        let aggregate = budget.aggregate_one_sigma().unwrap();
        assert!((aggregate - 5.0).abs() < 1.0e-12);
    }

    #[test]
    fn correlated_budget_applies_quadratic_form() {
        let matrix = CorrelationMatrix::new(2, vec![1.0, 0.5, 0.5, 1.0]).unwrap();
        let budget = CorrelatedErrorBudget {
            sources: vec![
                source("a", 3.0, UncertaintyClass::Aleatory),
                source("b", 4.0, UncertaintyClass::Aleatory),
            ],
            correlation: Some(matrix),
        };
        let aggregate = budget.aggregate_one_sigma().unwrap();
        assert!((aggregate - 37.0_f64.sqrt()).abs() < 1.0e-12);
    }

    #[test]
    fn aleatory_and_epistemic_sub_aggregates_mask_sources() {
        let matrix = CorrelationMatrix::new(
            3,
            vec![
                1.0, 0.25, 0.5, //
                0.25, 1.0, 0.1, //
                0.5, 0.1, 1.0,
            ],
        )
        .unwrap();
        let budget = CorrelatedErrorBudget {
            sources: vec![
                source("wind", 2.0, UncertaintyClass::Aleatory),
                source("sensor", 3.0, UncertaintyClass::Aleatory),
                source("aero", 5.0, UncertaintyClass::Epistemic),
            ],
            correlation: Some(matrix),
        };
        let aleatory = budget.aleatory_one_sigma().unwrap();
        let epistemic = budget.epistemic_one_sigma().unwrap();
        assert!((aleatory - 16.0_f64.sqrt()).abs() < 1.0e-12);
        assert!((epistemic - 5.0).abs() < 1.0e-12);
    }

    #[test]
    fn propulsion_c_star_efficiency_band_flows_into_epistemic_budget() {
        let source = propulsion_c_star_efficiency_margin_source(
            "lox_lch4_cantera_gri30.v1",
            0.96,
            0.98,
            1.0,
            CredibilityLevel::L3,
            "data/thermochem/lox-lch4-cantera-gri30-schema-1.toml",
        )
        .unwrap();
        assert_eq!(
            source.source_id,
            "05.propulsion.lox_lch4_cantera_gri30.v1.c_star_efficiency"
        );
        assert_eq!(source.class, UncertaintyClass::Epistemic);
        assert!((source.one_sigma - 0.02).abs() < 1.0e-15);

        let budget = CorrelatedErrorBudget {
            sources: vec![source],
            correlation: None,
        };
        assert!((budget.epistemic_one_sigma().unwrap() - 0.02).abs() < 1.0e-15);
        assert_eq!(
            budget
                .binding_credibility()
                .expect("binding record")
                .binding_level(),
            CredibilityLevel::L3
        );
    }

    #[test]
    fn upstream_margin_rejects_unordered_or_missing_evidence() {
        let unordered = UpstreamMargin {
            source_id: String::from("05.propulsion.bad.c_star_efficiency"),
            quantity_id: String::from("c_star_efficiency"),
            nominal: 0.98,
            lower: 1.0,
            upper: 0.96,
            class: UncertaintyClass::Epistemic,
            credibility: credibility(CredibilityLevel::L2),
            justification: String::from("synthetic bad band"),
        };
        assert!(matches!(
            unordered.conservative_one_sigma(),
            Err(UqError::InvalidInput {
                field: "upstream_margin.bounds"
            })
        ));

        assert!(matches!(
            propulsion_c_star_efficiency_margin_source(
                " ",
                0.96,
                0.98,
                1.0,
                CredibilityLevel::L2,
                "evidence",
            ),
            Err(UqError::InvalidInput {
                field: "propulsion_c_star.deck_id"
            })
        ));
    }

    #[test]
    fn probability_box_envelopes_conditional_cdfs_and_uses_lower_bound() {
        let conditional = vec![vec![0.0, 0.0], vec![10.0, 10.0]];
        let pbox = ProbabilityBox::from_conditional_samples(&conditional).unwrap();
        assert_eq!(pbox.support, vec![0.0, 10.0]);
        assert_eq!(pbox.lower_cdf, vec![0.0, 1.0]);
        assert_eq!(pbox.upper_cdf, vec![1.0, 1.0]);
        assert_eq!(pbox.lower_cdf_at(0.0).unwrap().to_bits(), 0.0_f64.to_bits());
        assert_eq!(pbox.upper_cdf_at(0.0).unwrap().to_bits(), 1.0_f64.to_bits());
        assert!(
            !pbox
                .verify_lower_tail_probability(0.0, 0.5)
                .expect("lower p-box requirement")
        );
        assert!(
            pbox.verify_lower_tail_probability(10.0, 1.0)
                .expect("lower p-box requirement")
        );
    }

    #[test]
    fn variance_split_matches_law_of_total_variance_for_separable_samples() {
        let conditional = vec![vec![-3.0, 1.0], vec![-1.0, 3.0]];
        let split = VarianceSplit::from_conditional_samples(&conditional).unwrap();
        assert!((split.aleatory - 4.0).abs() < 1.0e-12);
        assert!((split.epistemic - 1.0).abs() < 1.0e-12);
        assert!((split.total - 5.0).abs() < 1.0e-12);
        assert!(split.identity_residual() <= 1.0e-12);
    }

    #[test]
    fn nested_uq_rejects_empty_or_non_finite_samples() {
        assert!(matches!(
            ProbabilityBox::from_conditional_samples(&[]),
            Err(UqError::InvalidInput { .. })
        ));
        assert!(matches!(
            VarianceSplit::from_conditional_samples(&[vec![1.0], vec![f64::NAN]]),
            Err(UqError::InvalidInput {
                field: "nested.sample"
            })
        ));
    }

    #[test]
    fn non_psd_correlation_matrix_rejected() {
        let err = CorrelationMatrix::new(
            3,
            vec![
                1.0, 0.9, 0.9, //
                0.9, 1.0, -0.9, //
                0.9, -0.9, 1.0,
            ],
        )
        .expect_err("matrix is not PSD");
        assert!(matches!(err, UqError::NonPositiveSemidefinite { .. }));
    }

    #[test]
    fn credibility_missing_factor_binds_to_l0() {
        let record = CredibilityRecord::new().with_score(
            CredibilityFactor::Verification,
            CredibilityLevel::L4,
            "V-001",
        );
        assert_eq!(
            record.score(CredibilityFactor::Verification),
            CredibilityLevel::L4
        );
        assert_eq!(record.binding_level(), CredibilityLevel::L0);
        assert_eq!(record.legacy_label(), ValidationStatus::Experimental);
    }

    #[test]
    fn credibility_full_l2_maps_to_validated_toy() {
        let record = credibility(CredibilityLevel::L2);
        assert_eq!(record.binding_level(), CredibilityLevel::L2);
        assert_eq!(record.legacy_label(), ValidationStatus::ValidatedToy);
        assert!(record.render_markdown().contains("Binding credibility"));
    }

    #[test]
    fn credibility_level_and_factor_manifest_helpers_are_stable() {
        assert_eq!(CredibilityLevel::from_value(3), Some(CredibilityLevel::L3));
        assert_eq!(CredibilityLevel::from_value(5), None);
        assert_eq!(
            CredibilityFactor::from_key("input_pedigree"),
            Some(CredibilityFactor::InputPedigree)
        );
        assert_eq!(
            CredibilityFactor::ResultsUncertainty.as_key(),
            "results_uncertainty"
        );
        assert_eq!(CredibilityFactor::from_key("unknown"), None);
    }

    #[test]
    fn binding_credibility_reduces_sources_factorwise() {
        let weaker = source("aero", 1.0, UncertaintyClass::Epistemic);
        let mut stronger = source("wind", 1.0, UncertaintyClass::Aleatory);
        stronger.credibility = credibility(CredibilityLevel::L3);
        let budget = CorrelatedErrorBudget {
            sources: vec![stronger, weaker],
            correlation: None,
        };
        let binding = budget.binding_credibility().expect("record");
        assert_eq!(binding.binding_level(), CredibilityLevel::L2);
        assert_eq!(binding.legacy_label(), ValidationStatus::ValidatedToy);
    }

    #[test]
    fn legacy_markdown_uses_core_validation_label() {
        let budget = ErrorBudget {
            contributions: vec![contribution(
                "atmos",
                1.5e-2,
                ValidationStatus::ValidatedToy,
            )],
            correlated_bias: 0.0,
        };
        let markdown = budget.render_markdown("test-scenario");
        assert!(markdown.contains("test-scenario"));
        assert!(markdown.contains("validated-toy"));
        assert!(markdown.contains("Academic simulation"));
    }
}
