//! Acoustic environment statistics helpers.
//!
//! This module provides the first WP-22.6 maximum-expected-environment
//! substrate: deterministic per-band normal P95/50 limits over acoustic SPL
//! spectra. It is method evidence for downstream MC/run-data wiring, not a
//! campaign-size selector or qualification environment generator.

use openbmp_physics::inverse_standard_normal_cdf;
use thiserror::Error;

use crate::{AcousticBand, AcousticsError, Spl, SplSpectrum};

/// Default one-sided content probability for P95/50 environments.
pub const P95_50_COVERAGE_PROBABILITY: f64 = 0.95;

/// Default confidence probability for P95/50 environments.
pub const P95_50_CONFIDENCE_PROBABILITY: f64 = 0.50;

/// Errors returned by acoustic environment statistics helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AcousticStatsError {
    /// A lower-level acoustic value was invalid.
    #[error(transparent)]
    Acoustics(#[from] AcousticsError),
    /// At least two samples are required for a dispersion estimate.
    #[error("at least two spectra are required")]
    InsufficientSamples,
    /// A probability was outside `(0, 1)`.
    #[error("{field} must be in (0, 1)")]
    InvalidProbability {
        /// Field name.
        field: &'static str,
    },
    /// A scalar calculation produced NaN or infinity.
    #[error("{field} must be finite")]
    NonFinite {
        /// Field name.
        field: &'static str,
    },
    /// Input spectra used different band grids.
    #[error("spectra use different band grids")]
    BandGridMismatch,
}

/// Per-band normal P95/50 statistics in dB space.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SplP95_50BandStatistic {
    /// Acoustic band.
    pub band: AcousticBand,
    /// Arithmetic sample mean of SPL levels, dB re 20 uPa.
    pub mean_spl_db: f64,
    /// Unbiased sample standard deviation of SPL levels, dB.
    pub sigma_spl_db: f64,
    /// P95/50 normal tolerance limit, dB re 20 uPa.
    pub p95_50_spl_db: f64,
    /// Minimum sample SPL in this band, dB re 20 uPa.
    pub min_sample_spl_db: f64,
    /// Maximum sample SPL in this band, dB re 20 uPa.
    pub max_sample_spl_db: f64,
}

/// Evaluated P95/50 SPL environment over a common band grid.
#[derive(Clone, Debug, PartialEq)]
pub struct SplP95_50Environment {
    sample_count: usize,
    coverage_probability: f64,
    confidence_probability: f64,
    tolerance_factor: f64,
    statistics: Vec<SplP95_50BandStatistic>,
    spectrum: SplSpectrum,
    oaspl: Spl,
}

impl SplP95_50Environment {
    /// Number of spectra used in the estimate.
    #[must_use]
    pub const fn sample_count(&self) -> usize {
        self.sample_count
    }

    /// One-sided content probability.
    #[must_use]
    pub const fn coverage_probability(&self) -> f64 {
        self.coverage_probability
    }

    /// Confidence probability.
    #[must_use]
    pub const fn confidence_probability(&self) -> f64 {
        self.confidence_probability
    }

    /// Normal tolerance factor applied to per-band sample sigma.
    #[must_use]
    pub const fn tolerance_factor(&self) -> f64 {
        self.tolerance_factor
    }

    /// Per-band statistics.
    #[must_use]
    pub fn statistics(&self) -> &[SplP95_50BandStatistic] {
        &self.statistics
    }

    /// P95/50 SPL spectrum.
    #[must_use]
    pub fn spectrum(&self) -> &SplSpectrum {
        &self.spectrum
    }

    /// OASPL of the P95/50 spectrum.
    #[must_use]
    pub const fn oaspl(&self) -> Spl {
        self.oaspl
    }

    /// Deterministic CSV table for downstream run-data storage.
    #[must_use]
    pub fn to_csv(&self) -> Vec<u8> {
        let mut csv = String::from(
            "center_hz,lower_hz,upper_hz,mean_spl_db,sigma_spl_db,p95_50_spl_db,min_sample_spl_db,max_sample_spl_db,sample_count,coverage_probability,confidence_probability,tolerance_factor\n",
        );
        for statistic in &self.statistics {
            csv.push_str(&format!(
                "{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{},{:.17e},{:.17e},{:.17e}\n",
                statistic.band.center_hz,
                statistic.band.lower_hz,
                statistic.band.upper_hz,
                statistic.mean_spl_db,
                statistic.sigma_spl_db,
                statistic.p95_50_spl_db,
                statistic.min_sample_spl_db,
                statistic.max_sample_spl_db,
                self.sample_count,
                self.coverage_probability,
                self.confidence_probability,
                self.tolerance_factor,
            ));
        }
        csv.into_bytes()
    }
}

/// Compute the first-order one-sided normal tolerance factor.
///
/// For P95/50, the confidence term is zero because `Z(0.50) = 0`, so the
/// factor is the standard-normal 95th percentile. The sample-count argument is
/// kept in the general helper because later campaign/reporting layers can use
/// the same surface for non-median confidence sensitivity without changing the
/// deterministic operand order.
///
/// # Errors
///
/// Returns [`AcousticStatsError`] when probabilities are outside `(0, 1)`, fewer
/// than two samples are available, or the normal inverse calculation is
/// non-finite.
pub fn one_sided_normal_tolerance_factor(
    coverage_probability: f64,
    confidence_probability: f64,
    sample_count: usize,
) -> Result<f64, AcousticStatsError> {
    validate_probability("coverage_probability", coverage_probability)?;
    validate_probability("confidence_probability", confidence_probability)?;
    if sample_count < 2 {
        return Err(AcousticStatsError::InsufficientSamples);
    }
    let z_coverage = inverse_standard_normal_cdf(coverage_probability);
    let z_confidence = inverse_standard_normal_cdf(confidence_probability);
    let factor = z_coverage + z_confidence / (sample_count as f64).sqrt();
    require_finite("tolerance_factor", factor)?;
    Ok(factor)
}

/// Evaluate a normal P95/50 SPL environment over spectra sharing one band grid.
///
/// # Errors
///
/// Returns [`AcousticStatsError`] when fewer than two spectra are supplied, the
/// probability inputs are invalid, the spectra use different band grids, or a
/// derived statistic is non-finite.
pub fn evaluate_spl_p95_50_environment(
    spectra: &[SplSpectrum],
) -> Result<SplP95_50Environment, AcousticStatsError> {
    evaluate_spl_normal_tolerance_environment(
        spectra,
        P95_50_COVERAGE_PROBABILITY,
        P95_50_CONFIDENCE_PROBABILITY,
    )
}

/// Evaluate a one-sided normal-tolerance SPL environment over common-grid spectra.
///
/// # Errors
///
/// Returns [`AcousticStatsError`] when fewer than two spectra are supplied, the
/// probability inputs are invalid, the spectra use different band grids, or a
/// derived statistic is non-finite.
pub fn evaluate_spl_normal_tolerance_environment(
    spectra: &[SplSpectrum],
    coverage_probability: f64,
    confidence_probability: f64,
) -> Result<SplP95_50Environment, AcousticStatsError> {
    if spectra.len() < 2 {
        return Err(AcousticStatsError::InsufficientSamples);
    }
    let reference_bands = spectra[0].bands();
    for spectrum in &spectra[1..] {
        if spectrum.bands() != reference_bands {
            return Err(AcousticStatsError::BandGridMismatch);
        }
    }
    let tolerance_factor = one_sided_normal_tolerance_factor(
        coverage_probability,
        confidence_probability,
        spectra.len(),
    )?;
    let mut statistics = Vec::with_capacity(reference_bands.len());
    let mut p95_50_levels = Vec::with_capacity(reference_bands.len());
    for (band_index, band) in reference_bands.iter().enumerate() {
        let mut values = Vec::with_capacity(spectra.len());
        for spectrum in spectra {
            values.push(spectrum.levels()[band_index].db_re_20upa());
        }
        let statistic = band_statistic(*band, &values, tolerance_factor)?;
        p95_50_levels.push(Spl::new(statistic.p95_50_spl_db)?);
        statistics.push(statistic);
    }
    let spectrum = SplSpectrum::new(reference_bands.to_vec(), p95_50_levels)?;
    let oaspl = spectrum.oaspl()?;
    Ok(SplP95_50Environment {
        sample_count: spectra.len(),
        coverage_probability,
        confidence_probability,
        tolerance_factor,
        statistics,
        spectrum,
        oaspl,
    })
}

fn band_statistic(
    band: AcousticBand,
    values: &[f64],
    tolerance_factor: f64,
) -> Result<SplP95_50BandStatistic, AcousticStatsError> {
    debug_assert!(values.len() >= 2);
    let mut sum = 0.0;
    let mut min_sample_spl_db = f64::INFINITY;
    let mut max_sample_spl_db = f64::NEG_INFINITY;
    for value in values {
        require_finite("sample_spl_db", *value)?;
        sum += *value;
        min_sample_spl_db = min_sample_spl_db.min(*value);
        max_sample_spl_db = max_sample_spl_db.max(*value);
    }
    let mean_spl_db = sum / values.len() as f64;
    require_finite("mean_spl_db", mean_spl_db)?;
    let mut variance_sum = 0.0;
    for value in values {
        let delta = *value - mean_spl_db;
        variance_sum += delta * delta;
    }
    let sigma_spl_db = (variance_sum / (values.len() - 1) as f64).sqrt();
    require_finite("sigma_spl_db", sigma_spl_db)?;
    let p95_50_spl_db = mean_spl_db + tolerance_factor * sigma_spl_db;
    require_finite("p95_50_spl_db", p95_50_spl_db)?;
    Ok(SplP95_50BandStatistic {
        band,
        mean_spl_db,
        sigma_spl_db,
        p95_50_spl_db,
        min_sample_spl_db,
        max_sample_spl_db,
    })
}

fn validate_probability(field: &'static str, value: f64) -> Result<(), AcousticStatsError> {
    if value.is_finite() && value > 0.0 && value < 1.0 {
        Ok(())
    } else {
        Err(AcousticStatsError::InvalidProbability { field })
    }
}

fn require_finite(field: &'static str, value: f64) -> Result<(), AcousticStatsError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(AcousticStatsError::NonFinite { field })
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_abs_diff_eq;

    use super::*;
    use crate::STANDARD_THIRD_OCTAVE_CENTER_HZ;

    #[test]
    fn spl_p95_50_environment_matches_closed_form_normal_limit() {
        let spectra = [
            flat_standard_spl(100.0),
            flat_standard_spl(102.0),
            flat_standard_spl(104.0),
        ];

        let environment = evaluate_spl_p95_50_environment(&spectra).expect("environment");
        let expected_factor = inverse_standard_normal_cdf(P95_50_COVERAGE_PROBABILITY);
        let expected_band_level = 102.0 + expected_factor * 2.0;

        assert_eq!(environment.sample_count(), 3);
        assert_abs_diff_eq!(
            environment.coverage_probability(),
            P95_50_COVERAGE_PROBABILITY,
            epsilon = 0.0
        );
        assert_abs_diff_eq!(
            environment.confidence_probability(),
            P95_50_CONFIDENCE_PROBABILITY,
            epsilon = 0.0
        );
        assert_abs_diff_eq!(
            environment.tolerance_factor(),
            expected_factor,
            epsilon = 1.0e-12
        );
        assert_eq!(
            environment.statistics().len(),
            STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );
        let first = environment.statistics()[0];
        assert_abs_diff_eq!(first.mean_spl_db, 102.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(first.sigma_spl_db, 2.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(first.p95_50_spl_db, expected_band_level, epsilon = 1.0e-12);
        assert_abs_diff_eq!(first.min_sample_spl_db, 100.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(first.max_sample_spl_db, 104.0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            environment.spectrum().levels()[0].db_re_20upa(),
            expected_band_level,
            epsilon = 1.0e-12
        );
        assert_abs_diff_eq!(
            environment.oaspl().db_re_20upa(),
            expected_band_level + 10.0 * (STANDARD_THIRD_OCTAVE_CENTER_HZ.len() as f64).log10(),
            epsilon = 1.0e-12
        );
        assert_eq!(
            std::str::from_utf8(&environment.to_csv())
                .expect("CSV is UTF-8")
                .lines()
                .count(),
            STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
    }

    #[test]
    fn normal_tolerance_factor_uses_confidence_term() {
        let factor =
            one_sided_normal_tolerance_factor(0.95, 0.841_344_746_068_543, 4).expect("factor");
        assert_abs_diff_eq!(
            factor,
            inverse_standard_normal_cdf(0.95) + 0.5,
            epsilon = 1.0e-9
        );
    }

    #[test]
    fn spl_p95_50_environment_rejects_bad_inputs() {
        assert!(matches!(
            evaluate_spl_p95_50_environment(&[flat_standard_spl(100.0)]),
            Err(AcousticStatsError::InsufficientSamples)
        ));
        assert!(matches!(
            one_sided_normal_tolerance_factor(1.0, 0.5, 2),
            Err(AcousticStatsError::InvalidProbability {
                field: "coverage_probability"
            })
        ));
        assert!(matches!(
            one_sided_normal_tolerance_factor(0.95, 0.0, 2),
            Err(AcousticStatsError::InvalidProbability {
                field: "confidence_probability"
            })
        ));
        let mismatched = [
            flat_standard_spl(100.0),
            SplSpectrum::new(
                vec![AcousticBand::one_third_octave(1000.0).expect("band")],
                vec![Spl::new(100.0).expect("level")],
            )
            .expect("mismatched spectrum"),
        ];
        assert!(matches!(
            evaluate_spl_p95_50_environment(&mismatched),
            Err(AcousticStatsError::BandGridMismatch)
        ));
    }

    fn flat_standard_spl(level_db: f64) -> SplSpectrum {
        SplSpectrum::standard_third_octave(
            STANDARD_THIRD_OCTAVE_CENTER_HZ
                .iter()
                .map(|_| Spl::new(level_db).expect("level"))
                .collect(),
        )
        .expect("spectrum")
    }
}
