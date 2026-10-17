//! Vibroacoustic response-transfer substrates.
//!
//! This module is the first WP-22.6 layer. It maps acoustic SPL spectra into
//! zone acceleration PSDs through provenance-pinned empirical transfer decks
//! and computes a deterministic modal-force PSD handoff term from pressure PSD,
//! zone area, and mode-shape participation. The checked-in fixture is
//! synthetic method evidence, not a qualification environment.

use std::collections::BTreeSet;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{AcousticBand, AcousticsError, SplSpectrum};

/// Schema version for provenance-pinned vibroacoustic transfer decks.
pub const VIBROACOUSTIC_TRANSFER_DECK_SCHEMA_VERSION: u32 = 1;

/// Errors returned by vibroacoustic response helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum VibroacousticResponseError {
    /// A lower-level acoustic value was invalid.
    #[error(transparent)]
    Acoustics(#[from] AcousticsError),
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
    /// A list or text field was empty.
    #[error("{field} must not be empty")]
    Empty {
        /// Field name.
        field: &'static str,
    },
    /// A list contained duplicate values.
    #[error("{field} must not contain duplicate values")]
    Duplicate {
        /// Field name.
        field: &'static str,
    },
    /// Bands were not strictly increasing.
    #[error("{field} must be strictly increasing")]
    NonIncreasing {
        /// Field name.
        field: &'static str,
    },
    /// Band/value lengths did not match.
    #[error("band count {bands} does not match value count {values}")]
    LengthMismatch {
        /// Band count.
        bands: usize,
        /// Value count.
        values: usize,
    },
    /// A digest was not a lowercase SHA-256 hex string.
    #[error("{field} must be a lowercase SHA-256 hex digest")]
    InvalidSha256 {
        /// Field name.
        field: &'static str,
    },
    /// A requested class does not exist in the deck.
    #[error("unknown vibroacoustic transfer class {class_id}")]
    UnknownClass {
        /// Requested class id.
        class_id: String,
    },
    /// A deck schema version is unsupported.
    #[error("unsupported vibroacoustic transfer deck schema version {schema_version}")]
    UnsupportedDeckSchema {
        /// Parsed schema version.
        schema_version: u32,
    },
    /// A deck could not be parsed.
    #[error("malformed vibroacoustic transfer deck: {reason}")]
    MalformedDeck {
        /// Parse or structural failure.
        reason: String,
    },
}

/// Provenance metadata for a vibroacoustic transfer deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct VibroacousticTransferDeckMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source class, for example `synthetic-openbmp`.
    pub source_class: String,
    /// Human-readable source title.
    pub source_title: String,
    /// Short provenance statement.
    pub provenance: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label.
    pub validation: String,
}

impl VibroacousticTransferDeckMeta {
    fn from_row(row: VibroacousticTransferDeckMetaRow) -> Result<Self, VibroacousticResponseError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_class", &row.source_class)?;
        require_text("meta.source_title", &row.source_title)?;
        require_text("meta.provenance", &row.provenance)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_class: row.source_class.trim().to_owned(),
            source_title: row.source_title.trim().to_owned(),
            provenance: row.provenance.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// One empirical vibroacoustic transfer class.
#[derive(Clone, Debug, PartialEq)]
pub struct VibroacousticTransferClass {
    /// Stable class id referenced by future scenarios.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Reference surface mass used by the transfer deck, kg/m^2.
    pub reference_surface_mass_kg_m2: f64,
    /// Reference zone area for the declared transfer evidence, m^2.
    pub reference_area_m2: f64,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Per-band acceleration-PSD gain, `(m/s^2)^2 / Pa^2`.
    pub acceleration_psd_per_pressure_psd: Vec<f64>,
}

impl VibroacousticTransferClass {
    fn from_row(
        row: VibroacousticTransferClassRow,
        band_count: usize,
    ) -> Result<Self, VibroacousticResponseError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        require_positive(
            "classes.reference_surface_mass_kg_m2",
            row.reference_surface_mass_kg_m2,
        )?;
        require_positive("classes.reference_area_m2", row.reference_area_m2)?;
        require_non_negative("classes.uncertainty_db", row.uncertainty_db)?;
        if row.acceleration_psd_per_pressure_psd.len() != band_count {
            return Err(VibroacousticResponseError::LengthMismatch {
                bands: band_count,
                values: row.acceleration_psd_per_pressure_psd.len(),
            });
        }
        for value in &row.acceleration_psd_per_pressure_psd {
            require_non_negative("classes.acceleration_psd_per_pressure_psd", *value)?;
        }
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            reference_surface_mass_kg_m2: row.reference_surface_mass_kg_m2,
            reference_area_m2: row.reference_area_m2,
            uncertainty_db: row.uncertainty_db,
            acceleration_psd_per_pressure_psd: row.acceleration_psd_per_pressure_psd,
        })
    }
}

/// Provenance-pinned empirical vibroacoustic transfer deck.
#[derive(Clone, Debug, PartialEq)]
pub struct VibroacousticTransferDeck {
    meta: VibroacousticTransferDeckMeta,
    bands: Vec<AcousticBand>,
    classes: Vec<VibroacousticTransferClass>,
    sha256_hex: String,
}

impl VibroacousticTransferDeck {
    /// Parse a Schema-1 vibroacoustic transfer deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] when TOML parsing fails, the
    /// schema marker is unsupported, provenance fields are incomplete, or
    /// band/class rows are malformed.
    pub fn load_from_str(input: &str) -> Result<Self, VibroacousticResponseError> {
        let parsed: VibroacousticTransferDeckFile =
            toml::from_str(input).map_err(|error| VibroacousticResponseError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_vibroacoustic_transfer_deck
            != VIBROACOUSTIC_TRANSFER_DECK_SCHEMA_VERSION
        {
            return Err(VibroacousticResponseError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_vibroacoustic_transfer_deck,
            });
        }
        let bands = bands_from_center_hz(parsed.environment.center_hz)?;
        let classes = classes_from_rows(parsed.classes, bands.len())?;
        let sha256_hex = sha256_hex(input.as_bytes());
        require_sha256_hex("deck.sha256_hex", &sha256_hex)?;
        Ok(Self {
            meta: VibroacousticTransferDeckMeta::from_row(parsed.meta)?,
            bands,
            classes,
            sha256_hex,
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &VibroacousticTransferDeckMeta {
        &self.meta
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Declared transfer classes.
    #[must_use]
    pub fn classes(&self) -> &[VibroacousticTransferClass] {
        &self.classes
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }

    /// Look up a transfer class by id.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError::UnknownClass`] when `class_id` is absent.
    pub fn class(
        &self,
        class_id: &str,
    ) -> Result<&VibroacousticTransferClass, VibroacousticResponseError> {
        let class_id = class_id.trim();
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| VibroacousticResponseError::UnknownClass {
                class_id: class_id.to_owned(),
            })
    }

    /// Evaluate one transfer class against an acoustic SPL spectrum.
    ///
    /// Acceleration PSD scales with the square of
    /// `reference_surface_mass_kg_m2 / surface_mass_kg_m2`. The modal-force
    /// PSD handoff is computed directly from acoustic pressure PSD as
    /// `pressure_psd * (area_m2 * mode_shape_factor)^2`.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] when the class id is unknown,
    /// the input is invalid, the transfer deck and SPL spectrum use different
    /// band grids, or a resulting PSD value is invalid.
    pub fn evaluate_class(
        &self,
        class_id: &str,
        acoustic_spectrum: &SplSpectrum,
        input: VibroacousticResponseInput,
    ) -> Result<VibroacousticResponsePrediction, VibroacousticResponseError> {
        input.validate()?;
        let class = self.class(class_id)?;
        if self.bands.as_slice() != acoustic_spectrum.bands() {
            return Err(VibroacousticResponseError::Acoustics(
                AcousticsError::BandGridMismatch,
            ));
        }
        let pressure_psd = acoustic_spectrum.to_psd_spectrum()?;
        let surface_mass_gain =
            (class.reference_surface_mass_kg_m2 / input.surface_mass_kg_m2).powi(2);
        let modal_force_participation_m2 = input.area_m2 * input.mode_shape_factor;
        let acceleration_values = pressure_psd
            .values()
            .iter()
            .zip(class.acceleration_psd_per_pressure_psd.iter())
            .map(|(pressure, gain)| {
                AccelerationPsd::new(pressure.pa2_per_hz() * *gain * surface_mass_gain)
            })
            .collect::<Result<Vec<_>, _>>()?;
        let modal_force_values = pressure_psd
            .values()
            .iter()
            .map(|pressure| {
                ForcePsd::new(
                    pressure.pa2_per_hz()
                        * modal_force_participation_m2
                        * modal_force_participation_m2,
                )
            })
            .collect::<Result<Vec<_>, _>>()?;
        Ok(VibroacousticResponsePrediction {
            class_id: class.class_id.clone(),
            acceleration_psd: AccelerationPsdSpectrum::new(
                self.bands.clone(),
                acceleration_values,
            )?,
            modal_force_psd: ForcePsdSpectrum::new(self.bands.clone(), modal_force_values)?,
            uncertainty_db: class.uncertainty_db,
            surface_mass_gain,
            modal_force_participation_m2,
        })
    }
}

/// Vibroacoustic response input for one zone/mode.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct VibroacousticResponseInput {
    /// Effective surface mass, kg/m^2.
    pub surface_mass_kg_m2: f64,
    /// Acoustic zone area, m^2.
    pub area_m2: f64,
    /// Dimensionless modal participation over the zone.
    pub mode_shape_factor: f64,
}

impl VibroacousticResponseInput {
    /// Construct a validated response input.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] if surface mass/area are not
    /// positive or if the mode-shape factor is non-finite.
    pub fn new(
        surface_mass_kg_m2: f64,
        area_m2: f64,
        mode_shape_factor: f64,
    ) -> Result<Self, VibroacousticResponseError> {
        let input = Self {
            surface_mass_kg_m2,
            area_m2,
            mode_shape_factor,
        };
        input.validate()?;
        Ok(input)
    }

    fn validate(self) -> Result<(), VibroacousticResponseError> {
        require_positive("surface_mass_kg_m2", self.surface_mass_kg_m2)?;
        require_positive("area_m2", self.area_m2)?;
        require_finite("mode_shape_factor", self.mode_shape_factor)
    }
}

/// Acceleration PSD in `(m/s^2)^2/Hz`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AccelerationPsd {
    m2_s4_per_hz: f64,
}

impl AccelerationPsd {
    /// Construct an acceleration PSD value.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] if the value is non-finite or
    /// negative.
    pub fn new(m2_s4_per_hz: f64) -> Result<Self, VibroacousticResponseError> {
        require_non_negative("m2_s4_per_hz", m2_s4_per_hz)?;
        Ok(Self { m2_s4_per_hz })
    }

    /// PSD value in `(m/s^2)^2/Hz`.
    #[must_use]
    pub const fn m2_s4_per_hz(self) -> f64 {
        self.m2_s4_per_hz
    }

    /// Integrated band acceleration power, `(m/s^2)^2`.
    #[must_use]
    pub fn band_power_m2_s4(self, band: AcousticBand) -> f64 {
        self.m2_s4_per_hz * band.bandwidth_hz()
    }
}

/// Force PSD in `N^2/Hz`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ForcePsd {
    n2_per_hz: f64,
}

impl ForcePsd {
    /// Construct a force PSD value.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] if the value is non-finite or
    /// negative.
    pub fn new(n2_per_hz: f64) -> Result<Self, VibroacousticResponseError> {
        require_non_negative("n2_per_hz", n2_per_hz)?;
        Ok(Self { n2_per_hz })
    }

    /// PSD value in `N^2/Hz`.
    #[must_use]
    pub const fn n2_per_hz(self) -> f64 {
        self.n2_per_hz
    }

    /// Integrated band force power, `N^2`.
    #[must_use]
    pub fn band_power_n2(self, band: AcousticBand) -> f64 {
        self.n2_per_hz * band.bandwidth_hz()
    }
}

/// Acceleration PSD spectrum over an acoustic band grid.
#[derive(Clone, Debug, PartialEq)]
pub struct AccelerationPsdSpectrum {
    bands: Vec<AcousticBand>,
    values: Vec<AccelerationPsd>,
}

impl AccelerationPsdSpectrum {
    /// Construct a validated acceleration PSD spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] for empty spectra, length
    /// mismatches, or non-increasing band grids.
    pub fn new(
        bands: Vec<AcousticBand>,
        values: Vec<AccelerationPsd>,
    ) -> Result<Self, VibroacousticResponseError> {
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
    pub fn values(&self) -> &[AccelerationPsd] {
        &self.values
    }

    /// Total integrated mean-square acceleration, `(m/s^2)^2`.
    #[must_use]
    pub fn total_mean_square_acceleration_m2_s4(&self) -> f64 {
        self.bands
            .iter()
            .zip(self.values.iter())
            .map(|(band, value)| value.band_power_m2_s4(*band))
            .sum()
    }
}

/// Modal-force PSD spectrum over an acoustic band grid.
#[derive(Clone, Debug, PartialEq)]
pub struct ForcePsdSpectrum {
    bands: Vec<AcousticBand>,
    values: Vec<ForcePsd>,
}

impl ForcePsdSpectrum {
    /// Construct a validated force PSD spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] for empty spectra, length
    /// mismatches, or non-increasing band grids.
    pub fn new(
        bands: Vec<AcousticBand>,
        values: Vec<ForcePsd>,
    ) -> Result<Self, VibroacousticResponseError> {
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
    pub fn values(&self) -> &[ForcePsd] {
        &self.values
    }

    /// Total integrated modal-force power, `N^2`.
    #[must_use]
    pub fn total_mean_square_force_n2(&self) -> f64 {
        self.bands
            .iter()
            .zip(self.values.iter())
            .map(|(band, value)| value.band_power_n2(*band))
            .sum()
    }
}

/// Evaluated vibroacoustic response for one transfer class/input pair.
#[derive(Clone, Debug, PartialEq)]
pub struct VibroacousticResponsePrediction {
    /// Selected class id.
    pub class_id: String,
    /// Zone acceleration PSD spectrum.
    pub acceleration_psd: AccelerationPsdSpectrum,
    /// Modal-force PSD handoff spectrum.
    pub modal_force_psd: ForcePsdSpectrum,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Applied surface-mass response gain.
    pub surface_mass_gain: f64,
    /// Applied modal force participation, `area_m2 * mode_shape_factor`.
    pub modal_force_participation_m2: f64,
}

impl VibroacousticResponsePrediction {
    /// Build the structural-loads handoff view for this response prediction.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] if the integrated force power is
    /// non-finite.
    pub fn structural_loads_handoff(
        &self,
    ) -> Result<VibroacousticLoadsHandoff, VibroacousticResponseError> {
        VibroacousticLoadsHandoff::from_modal_force_psd(
            self.modal_force_psd.clone(),
            self.modal_force_participation_m2,
        )
    }
}

/// Typed vibroacoustic loads handoff for structural consumers.
#[derive(Clone, Debug, PartialEq)]
pub struct VibroacousticLoadsHandoff {
    modal_force_psd: ForcePsdSpectrum,
    modal_force_participation_m2: f64,
    total_mean_square_force_n2: f64,
}

impl VibroacousticLoadsHandoff {
    /// Construct from a modal-force PSD spectrum and signed modal
    /// participation over the acoustic zone.
    ///
    /// # Errors
    ///
    /// Returns [`VibroacousticResponseError`] if the participation or
    /// integrated force power is non-finite.
    pub fn from_modal_force_psd(
        modal_force_psd: ForcePsdSpectrum,
        modal_force_participation_m2: f64,
    ) -> Result<Self, VibroacousticResponseError> {
        require_finite(
            "loads_handoff.modal_force_participation_m2",
            modal_force_participation_m2,
        )?;
        let total_mean_square_force_n2 = modal_force_psd.total_mean_square_force_n2();
        require_finite(
            "loads_handoff.total_mean_square_force_n2",
            total_mean_square_force_n2,
        )?;
        Ok(Self {
            modal_force_psd,
            modal_force_participation_m2,
            total_mean_square_force_n2,
        })
    }

    /// Modal-force PSD spectrum.
    #[must_use]
    pub fn modal_force_psd(&self) -> &ForcePsdSpectrum {
        &self.modal_force_psd
    }

    /// Signed modal force participation, `area_m2 * mode_shape_factor`.
    #[must_use]
    pub const fn modal_force_participation_m2(&self) -> f64 {
        self.modal_force_participation_m2
    }

    /// Integrated mean-square modal force, `N^2`.
    #[must_use]
    pub const fn total_mean_square_force_n2(&self) -> f64 {
        self.total_mean_square_force_n2
    }

    /// Integrated force contribution per band, `N^2`.
    #[must_use]
    pub fn band_mean_square_force_n2(&self) -> Vec<f64> {
        self.modal_force_psd
            .bands()
            .iter()
            .zip(self.modal_force_psd.values().iter())
            .map(|(band, value)| value.band_power_n2(*band))
            .collect()
    }

    /// Deterministic structural loads handoff CSV.
    #[must_use]
    pub fn to_csv(&self) -> Vec<u8> {
        let mut csv = String::from(
            "center_hz,lower_hz,upper_hz,modal_force_psd_n2_per_hz,band_mean_square_force_n2,modal_force_participation_m2,total_mean_square_force_n2\n",
        );
        for (band, value) in self
            .modal_force_psd
            .bands()
            .iter()
            .zip(self.modal_force_psd.values().iter())
        {
            csv.push_str(&format!(
                "{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{:.17e}\n",
                band.center_hz,
                band.lower_hz,
                band.upper_hz,
                value.n2_per_hz(),
                value.band_power_n2(*band),
                self.modal_force_participation_m2,
                self.total_mean_square_force_n2,
            ));
        }
        csv.into_bytes()
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct VibroacousticTransferDeckFile {
    openbmp: VibroacousticTransferSchemaMarker,
    meta: VibroacousticTransferDeckMetaRow,
    environment: VibroacousticTransferEnvironmentSection,
    classes: Vec<VibroacousticTransferClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct VibroacousticTransferSchemaMarker {
    acoustics_vibroacoustic_transfer_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct VibroacousticTransferDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct VibroacousticTransferEnvironmentSection {
    center_hz: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct VibroacousticTransferClassRow {
    class_id: String,
    description: String,
    reference_surface_mass_kg_m2: f64,
    reference_area_m2: f64,
    uncertainty_db: f64,
    acceleration_psd_per_pressure_psd: Vec<f64>,
}

fn classes_from_rows(
    rows: Vec<VibroacousticTransferClassRow>,
    band_count: usize,
) -> Result<Vec<VibroacousticTransferClass>, VibroacousticResponseError> {
    if rows.is_empty() {
        return Err(VibroacousticResponseError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = VibroacousticTransferClass::from_row(row, band_count)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(VibroacousticResponseError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn bands_from_center_hz(values: Vec<f64>) -> Result<Vec<AcousticBand>, VibroacousticResponseError> {
    if values.is_empty() {
        return Err(VibroacousticResponseError::Empty {
            field: "environment.center_hz",
        });
    }
    let mut previous = None;
    let mut bands = Vec::with_capacity(values.len());
    for value in values {
        if previous.is_some_and(|previous| value <= previous) {
            return Err(VibroacousticResponseError::NonIncreasing {
                field: "environment.center_hz",
            });
        }
        previous = Some(value);
        bands.push(AcousticBand::one_third_octave(value)?);
    }
    Ok(bands)
}

fn validate_grid_and_len(
    bands: &[AcousticBand],
    value_count: usize,
) -> Result<(), VibroacousticResponseError> {
    if bands.is_empty() {
        return Err(VibroacousticResponseError::Empty { field: "bands" });
    }
    if bands.len() != value_count {
        return Err(VibroacousticResponseError::LengthMismatch {
            bands: bands.len(),
            values: value_count,
        });
    }
    let mut previous = None;
    for band in bands {
        require_positive("bands.center_hz", band.center_hz)?;
        require_positive("bands.lower_hz", band.lower_hz)?;
        require_positive("bands.upper_hz", band.upper_hz)?;
        if band.lower_hz >= band.upper_hz
            || previous.is_some_and(|previous| band.center_hz <= previous)
        {
            return Err(VibroacousticResponseError::NonIncreasing { field: "bands" });
        }
        previous = Some(band.center_hz);
    }
    Ok(())
}

fn require_text(field: &'static str, value: &str) -> Result<(), VibroacousticResponseError> {
    if value.trim().is_empty() {
        Err(VibroacousticResponseError::Empty { field })
    } else {
        Ok(())
    }
}

fn require_finite(field: &'static str, value: f64) -> Result<(), VibroacousticResponseError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(VibroacousticResponseError::NonFinite { field })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<(), VibroacousticResponseError> {
    require_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(VibroacousticResponseError::NonPositive { field })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), VibroacousticResponseError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(VibroacousticResponseError::Negative { field })
    }
}

fn require_sha256_hex(field: &'static str, value: &str) -> Result<(), VibroacousticResponseError> {
    if value.len() == 64
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
    {
        Ok(())
    } else {
        Err(VibroacousticResponseError::InvalidSha256 { field })
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut output = String::with_capacity(64);
    const HEX: &[u8; 16] = b"0123456789abcdef";
    for byte in digest {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::{STANDARD_THIRD_OCTAVE_CENTER_HZ, Spl};

    #[test]
    fn vibroacoustic_transfer_deck_parses_fixture_and_evaluates_response() {
        let fixture = include_str!("../../../data/acoustics/vibroacoustic-transfer-v1.toml");
        let deck = VibroacousticTransferDeck::load_from_str(fixture).expect("fixture parses");
        require_sha256_hex("deck.sha256_hex", deck.sha256_hex()).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.vibro_transfer.v1"
        );
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(deck.classes().len(), 2);
        assert_eq!(deck.bands().len(), STANDARD_THIRD_OCTAVE_CENTER_HZ.len());

        let acoustic = flat_standard_spl(100.0);
        let pressure = acoustic.to_psd_spectrum().unwrap();
        let prediction = deck
            .evaluate_class(
                "synthetic_panel_light",
                &acoustic,
                VibroacousticResponseInput::new(12.0, 3.0, 0.5).unwrap(),
            )
            .expect("response evaluates");

        assert_eq!(prediction.class_id, "synthetic_panel_light");
        assert_eq!(prediction.uncertainty_db.to_bits(), 5.0_f64.to_bits());
        assert_eq!(prediction.surface_mass_gain.to_bits(), 1.0_f64.to_bits());
        assert_eq!(
            prediction.modal_force_participation_m2.to_bits(),
            1.5_f64.to_bits()
        );
        let first_pressure_psd = pressure.values()[0].pa2_per_hz();
        assert_relative_eq!(
            prediction.acceleration_psd.values()[0].m2_s4_per_hz(),
            first_pressure_psd * 2.0,
            epsilon = 1.0e-18
        );
        assert_relative_eq!(
            prediction.modal_force_psd.values()[0].n2_per_hz(),
            first_pressure_psd * 1.5_f64 * 1.5,
            epsilon = 1.0e-18
        );
        let handoff = prediction
            .structural_loads_handoff()
            .expect("loads handoff builds");
        assert_eq!(
            handoff.modal_force_participation_m2().to_bits(),
            prediction.modal_force_participation_m2.to_bits()
        );
        assert_relative_eq!(
            handoff.total_mean_square_force_n2(),
            prediction.modal_force_psd.total_mean_square_force_n2(),
            epsilon = 1.0e-18
        );
        assert_relative_eq!(
            handoff.band_mean_square_force_n2().iter().sum::<f64>(),
            handoff.total_mean_square_force_n2(),
            max_relative = 1.0e-12
        );
        let handoff_csv_bytes = handoff.to_csv();
        let handoff_csv = std::str::from_utf8(&handoff_csv_bytes).expect("handoff CSV is UTF-8");
        assert!(handoff_csv.contains("band_mean_square_force_n2"));
        assert_eq!(
            handoff_csv.lines().count(),
            STANDARD_THIRD_OCTAVE_CENTER_HZ.len() + 1
        );
        assert_relative_eq!(
            prediction
                .acceleration_psd
                .total_mean_square_acceleration_m2_s4(),
            prediction
                .acceleration_psd
                .bands()
                .iter()
                .zip(prediction.acceleration_psd.values().iter())
                .map(|(band, value)| value.band_power_m2_s4(*band))
                .sum::<f64>(),
            epsilon = 1.0e-18
        );

        let heavier = deck
            .evaluate_class(
                "synthetic_panel_light",
                &acoustic,
                VibroacousticResponseInput::new(24.0, 3.0, 0.5).unwrap(),
            )
            .expect("heavier response evaluates");
        assert_relative_eq!(
            heavier.acceleration_psd.values()[0].m2_s4_per_hz(),
            prediction.acceleration_psd.values()[0].m2_s4_per_hz() * 0.25,
            epsilon = 1.0e-18
        );
        assert_relative_eq!(
            heavier.modal_force_psd.values()[0].n2_per_hz(),
            prediction.modal_force_psd.values()[0].n2_per_hz(),
            epsilon = 1.0e-18
        );
    }

    #[test]
    fn vibroacoustic_transfer_deck_helpers_fail_closed_on_bad_inputs() {
        let fixture = include_str!("../../../data/acoustics/vibroacoustic-transfer-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_vibroacoustic_transfer_deck = 1",
            "openbmp.acoustics_vibroacoustic_transfer_deck = 2",
        );
        assert!(matches!(
            VibroacousticTransferDeck::load_from_str(&bad_schema),
            Err(VibroacousticResponseError::UnsupportedDeckSchema { .. })
        ));

        let duplicate_class = fixture.replacen(
            "class_id = \"synthetic_panel_heavy\"",
            "class_id = \"synthetic_panel_light\"",
            1,
        );
        assert!(matches!(
            VibroacousticTransferDeck::load_from_str(&duplicate_class),
            Err(VibroacousticResponseError::Duplicate {
                field: "classes.class_id"
            })
        ));

        let short_transfer = fixture.replacen("  2.0,\n]\n\n[[classes]]", "]\n\n[[classes]]", 1);
        assert!(matches!(
            VibroacousticTransferDeck::load_from_str(&short_transfer),
            Err(VibroacousticResponseError::LengthMismatch { .. })
        ));

        let negative_gain = fixture.replacen("  2.0,", "  -2.0,", 1);
        assert!(matches!(
            VibroacousticTransferDeck::load_from_str(&negative_gain),
            Err(VibroacousticResponseError::Negative {
                field: "classes.acceleration_psd_per_pressure_psd"
            })
        ));

        let deck = VibroacousticTransferDeck::load_from_str(fixture).expect("fixture parses");
        assert!(matches!(
            VibroacousticResponseInput::new(0.0, 1.0, 1.0),
            Err(VibroacousticResponseError::NonPositive {
                field: "surface_mass_kg_m2"
            })
        ));
        assert!(matches!(
            deck.evaluate_class(
                "missing",
                &flat_standard_spl(100.0),
                VibroacousticResponseInput::new(12.0, 3.0, 0.5).unwrap(),
            ),
            Err(VibroacousticResponseError::UnknownClass { .. })
        ));
        let mismatched = SplSpectrum::new(
            vec![AcousticBand::one_third_octave(1000.0).unwrap()],
            vec![Spl::new(100.0).unwrap()],
        )
        .unwrap();
        assert!(matches!(
            deck.evaluate_class(
                "synthetic_panel_light",
                &mismatched,
                VibroacousticResponseInput::new(12.0, 3.0, 0.5).unwrap(),
            ),
            Err(VibroacousticResponseError::Acoustics(
                AcousticsError::BandGridMismatch
            ))
        ));
    }

    fn flat_standard_spl(level_db: f64) -> SplSpectrum {
        SplSpectrum::standard_third_octave(
            STANDARD_THIRD_OCTAVE_CENTER_HZ
                .iter()
                .map(|_| Spl::new(level_db).unwrap())
                .collect(),
        )
        .unwrap()
    }
}
