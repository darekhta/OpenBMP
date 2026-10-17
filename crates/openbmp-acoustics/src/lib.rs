//! `openbmp-acoustics` — deterministic acoustics spectral primitives.
//!
//! This crate is the L2 acoustics/vibroacoustics substrate. It provides the
//! one-third-octave band grid, sound-pressure-level and pressure-spectral-
//! density value types, locked-order band algebra, OASPL reduction, transfer
//! application, and enveloping operations that later DSM/IOP/ascent acoustic
//! models will consume.
//!
//! # Crate Layering
//!
//! `openbmp-acoustics` intentionally depends only on small parsing, hashing,
//! error, and shared numerical-statistics primitives. It has no dependency on `openbmp-runner`,
//! `openbmp-sim`, or `openbmp-fc`; acoustic environments are post-run analysis
//! products and structural-load inputs, not flight-controller inputs.

use std::fmt::Write as _;

use thiserror::Error;

pub mod ascent;
pub mod dsm;
pub mod iop;
pub mod pad;
pub mod response;
pub mod stats;

/// Acoustic reference pressure in Pa for SPL.
pub const REFERENCE_PRESSURE_PA: f64 = 20.0e-6;

/// Preferred one-third-octave center frequencies in Hz.
pub const STANDARD_THIRD_OCTAVE_CENTER_HZ: &[f64] = &[
    20.0, 25.0, 31.5, 40.0, 50.0, 63.0, 80.0, 100.0, 125.0, 160.0, 200.0, 250.0, 315.0, 400.0,
    500.0, 630.0, 800.0, 1000.0, 1250.0, 1600.0, 2000.0, 2500.0, 3150.0, 4000.0, 5000.0, 6300.0,
    8000.0, 10000.0, 12500.0, 16000.0, 20000.0,
];

/// Errors returned by acoustic spectral helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum AcousticsError {
    /// A scalar was non-finite.
    #[error("{field} must be finite")]
    NonFinite {
        /// Field name.
        field: &'static str,
    },
    /// A scalar was not positive.
    #[error("{field} must be positive")]
    NonPositive {
        /// Field name.
        field: &'static str,
    },
    /// A scalar was negative.
    #[error("{field} must be non-negative")]
    Negative {
        /// Field name.
        field: &'static str,
    },
    /// Spectra must contain at least one band.
    #[error("spectrum must contain at least one band")]
    EmptySpectrum,
    /// Band/value lengths did not match.
    #[error("band count {bands} does not match value count {values}")]
    LengthMismatch {
        /// Band count.
        bands: usize,
        /// Value count.
        values: usize,
    },
    /// Band sets did not match exactly.
    #[error("spectra use different band grids")]
    BandGridMismatch,
    /// Bands were not strictly increasing.
    #[error("bands must be strictly increasing")]
    NonIncreasingBands,
}

/// One frequency band.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AcousticBand {
    /// Center frequency in Hz.
    pub center_hz: f64,
    /// Lower band edge in Hz.
    pub lower_hz: f64,
    /// Upper band edge in Hz.
    pub upper_hz: f64,
}

impl AcousticBand {
    /// Construct a one-third-octave band from a center frequency.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] if the center frequency is non-finite or not
    /// positive.
    pub fn one_third_octave(center_hz: f64) -> Result<Self, AcousticsError> {
        require_positive("center_hz", center_hz)?;
        Ok(one_third_octave_unchecked(center_hz))
    }

    /// Bandwidth in Hz.
    #[must_use]
    pub fn bandwidth_hz(self) -> f64 {
        self.upper_hz - self.lower_hz
    }
}

/// Sound pressure level in dB re 20 uPa.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Spl {
    db_re_20upa: f64,
}

impl Spl {
    /// Construct an SPL value.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError::NonFinite`] if `db_re_20upa` is not finite.
    pub fn new(db_re_20upa: f64) -> Result<Self, AcousticsError> {
        require_finite("db_re_20upa", db_re_20upa)?;
        Ok(Self { db_re_20upa })
    }

    /// SPL value in dB re 20 uPa.
    #[must_use]
    pub const fn db_re_20upa(self) -> f64 {
        self.db_re_20upa
    }

    /// Mean-square pressure in Pa^2.
    #[must_use]
    pub fn mean_square_pressure_pa2(self) -> f64 {
        REFERENCE_PRESSURE_PA * REFERENCE_PRESSURE_PA * 10.0_f64.powf(self.db_re_20upa / 10.0)
    }

    /// Construct SPL from mean-square pressure.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] when the pressure power is non-finite or not
    /// positive.
    pub fn from_mean_square_pressure_pa2(value: f64) -> Result<Self, AcousticsError> {
        require_positive("mean_square_pressure_pa2", value)?;
        Self::new(10.0 * (value / (REFERENCE_PRESSURE_PA * REFERENCE_PRESSURE_PA)).log10())
    }
}

/// Pressure spectral density in Pa^2/Hz.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Psd {
    pa2_per_hz: f64,
}

impl Psd {
    /// Construct a pressure spectral-density value.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] if the value is non-finite or negative.
    pub fn new(pa2_per_hz: f64) -> Result<Self, AcousticsError> {
        require_non_negative("pa2_per_hz", pa2_per_hz)?;
        Ok(Self { pa2_per_hz })
    }

    /// PSD value in Pa^2/Hz.
    #[must_use]
    pub const fn pa2_per_hz(self) -> f64 {
        self.pa2_per_hz
    }

    /// Integrated band pressure power in Pa^2.
    #[must_use]
    pub fn band_power_pa2(self, band: AcousticBand) -> f64 {
        self.pa2_per_hz * band.bandwidth_hz()
    }
}

/// SPL spectrum over an acoustic band grid.
#[derive(Clone, Debug, PartialEq)]
pub struct SplSpectrum {
    bands: Vec<AcousticBand>,
    levels: Vec<Spl>,
}

impl SplSpectrum {
    /// Construct a validated SPL spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] for empty spectra, length mismatches, or
    /// non-increasing band grids.
    pub fn new(bands: Vec<AcousticBand>, levels: Vec<Spl>) -> Result<Self, AcousticsError> {
        validate_grid_and_len(&bands, levels.len())?;
        Ok(Self { bands, levels })
    }

    /// Construct an SPL spectrum on the standard one-third-octave band grid.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] if `levels` length does not match the grid.
    pub fn standard_third_octave(levels: Vec<Spl>) -> Result<Self, AcousticsError> {
        Self::new(standard_third_octave_bands(), levels)
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// SPL levels.
    #[must_use]
    pub fn levels(&self) -> &[Spl] {
        &self.levels
    }

    /// Overall SPL by locked-order energy summation.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] if the integrated pressure power is zero.
    pub fn oaspl(&self) -> Result<Spl, AcousticsError> {
        Spl::from_mean_square_pressure_pa2(self.total_mean_square_pressure_pa2())
    }

    /// Total mean-square pressure in Pa^2 by locked-order band summation.
    #[must_use]
    pub fn total_mean_square_pressure_pa2(&self) -> f64 {
        let mut total = 0.0;
        for level in &self.levels {
            total += level.mean_square_pressure_pa2();
        }
        total
    }

    /// Add two SPL spectra by pressure-energy summation per band.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError::BandGridMismatch`] if the grids differ.
    pub fn add_levels(&self, other: &Self) -> Result<Self, AcousticsError> {
        self.require_same_grid(other)?;
        let levels = self
            .levels
            .iter()
            .zip(other.levels.iter())
            .map(|(lhs, rhs)| {
                Spl::from_mean_square_pressure_pa2(
                    lhs.mean_square_pressure_pa2() + rhs.mean_square_pressure_pa2(),
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        Self::new(self.bands.clone(), levels)
    }

    /// Apply a per-band transfer curve in dB.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] when the transfer length is wrong or a
    /// transfer value is non-finite.
    pub fn apply_transfer_db(&self, transfer_db: &[f64]) -> Result<Self, AcousticsError> {
        if transfer_db.len() != self.levels.len() {
            return Err(AcousticsError::LengthMismatch {
                bands: self.levels.len(),
                values: transfer_db.len(),
            });
        }
        let levels = self
            .levels
            .iter()
            .zip(transfer_db.iter())
            .map(|(level, transfer)| {
                require_finite("transfer_db", *transfer)?;
                Spl::new(level.db_re_20upa + transfer)
            })
            .collect::<Result<Vec<_>, AcousticsError>>()?;
        Self::new(self.bands.clone(), levels)
    }

    /// Envelope two spectra by taking the maximum SPL in each band.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError::BandGridMismatch`] if the grids differ.
    pub fn envelope(&self, other: &Self) -> Result<Self, AcousticsError> {
        self.require_same_grid(other)?;
        let levels = self
            .levels
            .iter()
            .zip(other.levels.iter())
            .map(|(lhs, rhs)| Spl::new(lhs.db_re_20upa.max(rhs.db_re_20upa)))
            .collect::<Result<Vec<_>, _>>()?;
        Self::new(self.bands.clone(), levels)
    }

    /// Convert band levels to equivalent flat-in-band PSD values.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] if any derived PSD is invalid.
    pub fn to_psd_spectrum(&self) -> Result<PsdSpectrum, AcousticsError> {
        let values = self
            .bands
            .iter()
            .zip(self.levels.iter())
            .map(|(band, level)| Psd::new(level.mean_square_pressure_pa2() / band.bandwidth_hz()))
            .collect::<Result<Vec<_>, _>>()?;
        PsdSpectrum::new(self.bands.clone(), values)
    }

    fn require_same_grid(&self, other: &Self) -> Result<(), AcousticsError> {
        if same_band_grid(&self.bands, &other.bands) {
            Ok(())
        } else {
            Err(AcousticsError::BandGridMismatch)
        }
    }
}

/// Pressure spectral-density spectrum over an acoustic band grid.
#[derive(Clone, Debug, PartialEq)]
pub struct PsdSpectrum {
    bands: Vec<AcousticBand>,
    values: Vec<Psd>,
}

impl PsdSpectrum {
    /// Construct a validated PSD spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] for empty spectra, length mismatches, or
    /// non-increasing band grids.
    pub fn new(bands: Vec<AcousticBand>, values: Vec<Psd>) -> Result<Self, AcousticsError> {
        validate_grid_and_len(&bands, values.len())?;
        Ok(Self { bands, values })
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// PSD values.
    #[must_use]
    pub fn values(&self) -> &[Psd] {
        &self.values
    }

    /// Total integrated mean-square pressure in Pa^2.
    #[must_use]
    pub fn total_mean_square_pressure_pa2(&self) -> f64 {
        let mut total = 0.0;
        for (band, value) in self.bands.iter().zip(self.values.iter()) {
            total += value.band_power_pa2(*band);
        }
        total
    }

    /// Convert flat-in-band PSD values to SPL band levels.
    ///
    /// # Errors
    ///
    /// Returns [`AcousticsError`] if any band has zero pressure power.
    pub fn to_spl_spectrum(&self) -> Result<SplSpectrum, AcousticsError> {
        let levels = self
            .bands
            .iter()
            .zip(self.values.iter())
            .map(|(band, value)| Spl::from_mean_square_pressure_pa2(value.band_power_pa2(*band)))
            .collect::<Result<Vec<_>, _>>()?;
        SplSpectrum::new(self.bands.clone(), levels)
    }
}

/// Standard one-third-octave band grid.
#[must_use]
pub fn standard_third_octave_bands() -> Vec<AcousticBand> {
    STANDARD_THIRD_OCTAVE_CENTER_HZ
        .iter()
        .map(|center| one_third_octave_unchecked(*center))
        .collect()
}

/// Render the standard band grid as deterministic CSV bytes.
#[must_use]
pub fn standard_band_grid_csv() -> String {
    let mut out = String::from("center_hz,lower_hz,upper_hz\n");
    for band in standard_third_octave_bands() {
        let _ = writeln!(
            out,
            "{},{},{}",
            format_f64(band.center_hz),
            format_f64(band.lower_hz),
            format_f64(band.upper_hz)
        );
    }
    out
}

fn validate_grid_and_len(bands: &[AcousticBand], value_count: usize) -> Result<(), AcousticsError> {
    if bands.is_empty() {
        return Err(AcousticsError::EmptySpectrum);
    }
    if bands.len() != value_count {
        return Err(AcousticsError::LengthMismatch {
            bands: bands.len(),
            values: value_count,
        });
    }
    let mut previous = None;
    for band in bands {
        require_positive("band.center_hz", band.center_hz)?;
        require_positive("band.lower_hz", band.lower_hz)?;
        require_positive("band.upper_hz", band.upper_hz)?;
        if band.lower_hz >= band.upper_hz {
            return Err(AcousticsError::NonIncreasingBands);
        }
        if previous.is_some_and(|previous| band.center_hz <= previous) {
            return Err(AcousticsError::NonIncreasingBands);
        }
        previous = Some(band.center_hz);
    }
    Ok(())
}

fn one_third_octave_unchecked(center_hz: f64) -> AcousticBand {
    let edge_ratio = 2.0_f64.powf(1.0 / 6.0);
    AcousticBand {
        center_hz,
        lower_hz: center_hz / edge_ratio,
        upper_hz: center_hz * edge_ratio,
    }
}

fn same_band_grid(lhs: &[AcousticBand], rhs: &[AcousticBand]) -> bool {
    lhs.len() == rhs.len()
        && lhs.iter().zip(rhs.iter()).all(|(lhs, rhs)| {
            lhs.center_hz.to_bits() == rhs.center_hz.to_bits()
                && lhs.lower_hz.to_bits() == rhs.lower_hz.to_bits()
                && lhs.upper_hz.to_bits() == rhs.upper_hz.to_bits()
        })
}

fn require_finite(field: &'static str, value: f64) -> Result<(), AcousticsError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(AcousticsError::NonFinite { field })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<(), AcousticsError> {
    require_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(AcousticsError::NonPositive { field })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), AcousticsError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(AcousticsError::Negative { field })
    }
}

fn format_f64(value: f64) -> String {
    format!("{value:.17e}")
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use approx::assert_relative_eq;
    use proptest::prelude::*;

    use super::*;

    fn levels(values: &[f64]) -> Vec<Spl> {
        values
            .iter()
            .map(|value| Spl::new(*value).unwrap())
            .collect()
    }

    #[test]
    fn standard_one_third_octave_grid_is_byte_stable() {
        assert_eq!(
            standard_band_grid_csv(),
            include_str!("../tests/fixtures/standard-third-octave-bands.csv")
        );
        let bands = standard_third_octave_bands();
        assert_eq!(bands.len(), STANDARD_THIRD_OCTAVE_CENTER_HZ.len());
        assert_eq!(bands[0].center_hz, 20.0);
        assert_eq!(bands[bands.len() - 1].center_hz, 20000.0);
    }

    #[test]
    fn oaspl_matches_locked_energy_sum() {
        let spectrum = SplSpectrum::standard_third_octave(levels(&vec![
                100.0;
                STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
            ]))
        .unwrap();
        let expected = 100.0 + 10.0 * (STANDARD_THIRD_OCTAVE_CENTER_HZ.len() as f64).log10();

        assert_relative_eq!(
            spectrum.oaspl().unwrap().db_re_20upa(),
            expected,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn transfer_and_envelope_ops_are_locked_order() {
        let bands = standard_third_octave_bands()[..3].to_vec();
        let a = SplSpectrum::new(bands.clone(), levels(&[80.0, 90.0, 100.0])).unwrap();
        let b = SplSpectrum::new(bands, levels(&[85.0, 85.0, 105.0])).unwrap();

        let transferred = a.apply_transfer_db(&[1.0, -2.0, 3.0]).unwrap();
        assert_eq!(transferred.levels()[0].db_re_20upa(), 81.0);
        assert_eq!(transferred.levels()[1].db_re_20upa(), 88.0);
        assert_eq!(transferred.levels()[2].db_re_20upa(), 103.0);

        let envelope = transferred.envelope(&b).unwrap();
        assert_eq!(envelope.levels()[0].db_re_20upa(), 85.0);
        assert_eq!(envelope.levels()[1].db_re_20upa(), 88.0);
        assert_eq!(envelope.levels()[2].db_re_20upa(), 105.0);
    }

    #[test]
    fn psd_parseval_closure_on_synthetic_spectrum() {
        let bands = standard_third_octave_bands()[..4].to_vec();
        let psd = PsdSpectrum::new(
            bands,
            vec![
                Psd::new(1.0e-4).unwrap(),
                Psd::new(2.0e-4).unwrap(),
                Psd::new(3.0e-4).unwrap(),
                Psd::new(4.0e-4).unwrap(),
            ],
        )
        .unwrap();

        let total = psd.total_mean_square_pressure_pa2();
        let spl = psd.to_spl_spectrum().unwrap();
        let round_tripped = spl.to_psd_spectrum().unwrap();

        assert_relative_eq!(
            spl.total_mean_square_pressure_pa2(),
            total,
            epsilon = 1.0e-15
        );
        assert_relative_eq!(
            round_tripped.total_mean_square_pressure_pa2(),
            total,
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn validation_rejects_bad_inputs() {
        assert_eq!(
            Spl::new(f64::NAN).unwrap_err(),
            AcousticsError::NonFinite {
                field: "db_re_20upa"
            }
        );
        assert!(matches!(
            Psd::new(-1.0).unwrap_err(),
            AcousticsError::Negative { .. }
        ));
        assert_eq!(
            SplSpectrum::new(Vec::new(), Vec::new()).unwrap_err(),
            AcousticsError::EmptySpectrum
        );
        assert!(matches!(
            SplSpectrum::new(standard_third_octave_bands()[..2].to_vec(), levels(&[80.0]))
                .unwrap_err(),
            AcousticsError::LengthMismatch { .. }
        ));
        assert!(matches!(
            AcousticBand::one_third_octave(0.0).unwrap_err(),
            AcousticsError::NonPositive { .. }
        ));
    }

    proptest! {
        #[test]
        fn band_level_addition_matches_energy_sum(a in -20.0_f64..180.0, b in -20.0_f64..180.0) {
            let band = AcousticBand::one_third_octave(1000.0).unwrap();
            let lhs = SplSpectrum::new(vec![band], vec![Spl::new(a).unwrap()]).unwrap();
            let rhs = SplSpectrum::new(vec![band], vec![Spl::new(b).unwrap()]).unwrap();

            let added = lhs.add_levels(&rhs).unwrap();
            let expected = lhs.total_mean_square_pressure_pa2() + rhs.total_mean_square_pressure_pa2();

            prop_assert!((added.total_mean_square_pressure_pa2() - expected).abs() <= expected * 1.0e-12);
        }

        #[test]
        fn psd_spl_round_trip_preserves_band_power(values in prop::collection::vec(1.0e-12_f64..1.0e2, 1..16)) {
            let bands = standard_third_octave_bands()[..values.len()].to_vec();
            let psd_values = values
                .iter()
                .map(|value| Psd::new(*value).unwrap())
                .collect::<Vec<_>>();
            let psd = PsdSpectrum::new(bands, psd_values).unwrap();
            let total = psd.total_mean_square_pressure_pa2();
            let round_trip = psd.to_spl_spectrum().unwrap().to_psd_spectrum().unwrap();

            prop_assert!((round_trip.total_mean_square_pressure_pa2() - total).abs() <= total * 1.0e-12);
        }
    }
}
