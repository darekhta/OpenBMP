//! Ascent acoustic environment class decks.
//!
//! This module is a deterministic substrate for WP-22.5. It evaluates
//! class-based external ascent acoustic spectra from dynamic pressure and Mach
//! gates, and applies fairing-internal insertion-loss decks to those external
//! spectra. The checked-in fixtures are synthetic and are not launch-vehicle
//! user guides or flight-correlated acoustic environments.

use std::collections::BTreeSet;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{AcousticBand, AcousticsError, Spl, SplSpectrum};

/// Schema version for provenance-pinned ascent acoustic environment decks.
pub const ASCENT_ACOUSTIC_DECK_SCHEMA_VERSION: u32 = 1;

/// Schema version for provenance-pinned fairing-internal acoustic decks.
pub const FAIRING_INTERNAL_DECK_SCHEMA_VERSION: u32 = 1;

/// Errors returned by ascent and fairing acoustic helpers.
#[derive(Clone, Debug, Error, PartialEq)]
pub enum AscentAcousticsError {
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
    /// A lower/upper range was invalid.
    #[error("{field} minimum must not exceed maximum")]
    InvalidRange {
        /// Field name.
        field: &'static str,
    },
    /// Bands were not strictly increasing.
    #[error("{field} must be strictly increasing")]
    NonIncreasing {
        /// Field name.
        field: &'static str,
    },
    /// A value was outside a class gate.
    #[error("{field}={value} is outside [{min}, {max}]")]
    OutOfRange {
        /// Field name.
        field: &'static str,
        /// Supplied value.
        value: f64,
        /// Inclusive lower bound.
        min: f64,
        /// Inclusive upper bound.
        max: f64,
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
    #[error("unknown ascent acoustic class {class_id}")]
    UnknownClass {
        /// Requested class id.
        class_id: String,
    },
    /// No ascent class gate matched an automatic selector input.
    #[error("no ascent acoustic class gate matches {field}={value}")]
    NoMatchingClassGate {
        /// Selector field name.
        field: &'static str,
        /// Selector value.
        value: f64,
    },
    /// More than one ascent class gate matched an automatic selector input.
    #[error("ambiguous ascent acoustic class gate matches for {field}={value}: {class_ids:?}")]
    AmbiguousClassGate {
        /// Selector field name.
        field: &'static str,
        /// Selector value.
        value: f64,
        /// Matching class ids.
        class_ids: Vec<String>,
    },
    /// A requested fairing class does not exist in the deck.
    #[error("unknown fairing internal acoustic class {class_id}")]
    UnknownFairingClass {
        /// Requested class id.
        class_id: String,
    },
    /// A deck schema version is unsupported.
    #[error("unsupported ascent acoustic deck schema version {schema_version}")]
    UnsupportedDeckSchema {
        /// Parsed schema version.
        schema_version: u32,
    },
    /// A fairing-internal deck schema version is unsupported.
    #[error("unsupported fairing internal acoustic deck schema version {schema_version}")]
    UnsupportedFairingDeckSchema {
        /// Parsed schema version.
        schema_version: u32,
    },
    /// A deck could not be parsed.
    #[error("malformed acoustic deck: {reason}")]
    MalformedDeck {
        /// Parse or structural failure.
        reason: String,
    },
}

/// Provenance metadata for an ascent acoustic deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct AscentAcousticDeckMeta {
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

impl AscentAcousticDeckMeta {
    fn from_row(row: AscentAcousticDeckMetaRow) -> Result<Self, AscentAcousticsError> {
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

/// One ascent acoustic class in a deck.
#[derive(Clone, Debug, PartialEq)]
pub struct AscentAcousticClass {
    /// Stable class id referenced by future scenario files.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Reference dynamic pressure for the declared OASPL, Pa.
    pub reference_dynamic_pressure_pa: f64,
    /// Exponent used by `10 * exponent * log10(q / q_ref)`.
    pub dynamic_pressure_exponent: f64,
    /// Inclusive minimum Mach gate.
    pub mach_min: f64,
    /// Inclusive maximum Mach gate.
    pub mach_max: f64,
    /// OASPL at the reference dynamic pressure, dB re 20 uPa.
    pub reference_oaspl_db: f64,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Relative per-band levels. The evaluator normalizes these to the target OASPL.
    pub relative_band_level_db: Vec<f64>,
}

impl AscentAcousticClass {
    fn from_row(
        row: AscentAcousticClassRow,
        band_count: usize,
    ) -> Result<Self, AscentAcousticsError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        require_positive(
            "classes.reference_dynamic_pressure_pa",
            row.reference_dynamic_pressure_pa,
        )?;
        require_non_negative(
            "classes.dynamic_pressure_exponent",
            row.dynamic_pressure_exponent,
        )?;
        require_non_negative("classes.mach_min", row.mach_min)?;
        require_non_negative("classes.mach_max", row.mach_max)?;
        if row.mach_min > row.mach_max {
            return Err(AscentAcousticsError::InvalidRange {
                field: "classes.mach",
            });
        }
        require_finite("classes.reference_oaspl_db", row.reference_oaspl_db)?;
        require_non_negative("classes.uncertainty_db", row.uncertainty_db)?;
        if row.relative_band_level_db.len() != band_count {
            return Err(AscentAcousticsError::LengthMismatch {
                bands: band_count,
                values: row.relative_band_level_db.len(),
            });
        }
        for value in &row.relative_band_level_db {
            require_finite("classes.relative_band_level_db", *value)?;
        }
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            reference_dynamic_pressure_pa: row.reference_dynamic_pressure_pa,
            dynamic_pressure_exponent: row.dynamic_pressure_exponent,
            mach_min: row.mach_min,
            mach_max: row.mach_max,
            reference_oaspl_db: row.reference_oaspl_db,
            uncertainty_db: row.uncertainty_db,
            relative_band_level_db: row.relative_band_level_db,
        })
    }
}

/// Provenance-pinned ascent acoustic environment deck.
#[derive(Clone, Debug, PartialEq)]
pub struct AscentAcousticDeck {
    meta: AscentAcousticDeckMeta,
    bands: Vec<AcousticBand>,
    classes: Vec<AscentAcousticClass>,
    sha256_hex: String,
}

impl AscentAcousticDeck {
    /// Parse a Schema-1 ascent acoustic environment deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when TOML parsing fails, the schema
    /// marker is unsupported, provenance fields are incomplete, or band/class
    /// rows are malformed.
    pub fn load_from_str(input: &str) -> Result<Self, AscentAcousticsError> {
        let parsed: AscentAcousticDeckFile =
            toml::from_str(input).map_err(|error| AscentAcousticsError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_ascent_environment_deck != ASCENT_ACOUSTIC_DECK_SCHEMA_VERSION {
            return Err(AscentAcousticsError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_ascent_environment_deck,
            });
        }
        let bands = bands_from_center_hz(parsed.environment.center_hz)?;
        let classes = classes_from_rows(parsed.classes, bands.len())?;
        let sha256_hex = sha256_hex(input.as_bytes());
        require_sha256_hex("deck.sha256_hex", &sha256_hex)?;
        Ok(Self {
            meta: AscentAcousticDeckMeta::from_row(parsed.meta)?,
            bands,
            classes,
            sha256_hex,
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &AscentAcousticDeckMeta {
        &self.meta
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Declared ascent acoustic classes.
    #[must_use]
    pub fn classes(&self) -> &[AscentAcousticClass] {
        &self.classes
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }

    /// Look up an ascent acoustic class by id.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError::UnknownClass`] when `class_id` is absent.
    pub fn class(&self, class_id: &str) -> Result<&AscentAcousticClass, AscentAcousticsError> {
        let class_id = class_id.trim();
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| AscentAcousticsError::UnknownClass {
                class_id: class_id.to_owned(),
            })
    }

    /// Select exactly one ascent acoustic class from the deck Mach gates.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when the input is invalid, no class
    /// Mach gate contains the input Mach number, or more than one class gate
    /// matches the input Mach number.
    pub fn select_class_for_input(
        &self,
        input: AscentAcousticInput,
    ) -> Result<&AscentAcousticClass, AscentAcousticsError> {
        input.validate()?;
        let matches = self
            .classes
            .iter()
            .filter(|class| (class.mach_min..=class.mach_max).contains(&input.mach))
            .collect::<Vec<_>>();
        match matches.as_slice() {
            [] => Err(AscentAcousticsError::NoMatchingClassGate {
                field: "mach",
                value: input.mach,
            }),
            [class] => Ok(class),
            classes => Err(AscentAcousticsError::AmbiguousClassGate {
                field: "mach",
                value: input.mach,
                class_ids: classes.iter().map(|class| class.class_id.clone()).collect(),
            }),
        }
    }

    /// Evaluate the class selected by this deck's Mach gates.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when automatic class selection or
    /// acoustic evaluation fails.
    pub fn evaluate_mach_gate(
        &self,
        input: AscentAcousticInput,
    ) -> Result<AscentAcousticPrediction, AscentAcousticsError> {
        let class_id = self.select_class_for_input(input)?.class_id.clone();
        self.evaluate_class(&class_id, input)
    }

    /// Evaluate one ascent acoustic class at a trajectory sample.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when the class id is unknown, the input
    /// is invalid, the Mach sample is outside the class gate, or the resulting
    /// SPL spectrum cannot be constructed.
    pub fn evaluate_class(
        &self,
        class_id: &str,
        input: AscentAcousticInput,
    ) -> Result<AscentAcousticPrediction, AscentAcousticsError> {
        input.validate()?;
        let class = self.class(class_id)?;
        if !(class.mach_min..=class.mach_max).contains(&input.mach) {
            return Err(AscentAcousticsError::OutOfRange {
                field: "mach",
                value: input.mach,
                min: class.mach_min,
                max: class.mach_max,
            });
        }
        let q_ratio = input.dynamic_pressure_pa / class.reference_dynamic_pressure_pa;
        let oaspl_db =
            class.reference_oaspl_db + 10.0 * class.dynamic_pressure_exponent * q_ratio.log10();
        let spectrum =
            spectrum_from_relative_shape(&self.bands, oaspl_db, &class.relative_band_level_db)?;
        let oaspl = spectrum.oaspl()?;
        Ok(AscentAcousticPrediction {
            class_id: class.class_id.clone(),
            spectrum,
            oaspl,
            uncertainty_db: class.uncertainty_db,
        })
    }

    /// Evaluate and envelope an ascent acoustic trace.
    ///
    /// The envelope is a per-band maximum over the sample spectra. The
    /// `peak_sample_index` records the largest individual sample OASPL, while
    /// `envelope_oaspl` records the energy sum of the enveloped spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when the trace is empty or any sample
    /// fails [`Self::evaluate_class`].
    pub fn evaluate_trace(
        &self,
        samples: &[AscentAcousticTraceSample],
    ) -> Result<AscentAcousticTracePrediction, AscentAcousticsError> {
        let first = samples
            .first()
            .ok_or(AscentAcousticsError::Empty { field: "samples" })?;
        let first_prediction = self.evaluate_class(&first.class_id, first.input)?;
        let mut peak_oaspl_db = first_prediction.oaspl.db_re_20upa();
        let mut uncertainty_db = first_prediction.uncertainty_db;
        let mut points = vec![AscentAcousticTracePoint {
            class_id: first_prediction.class_id.clone(),
            spectrum: first_prediction.spectrum.clone(),
            dynamic_pressure_pa: first.input.dynamic_pressure_pa,
            mach: first.input.mach,
            oaspl: first_prediction.oaspl,
            uncertainty_db: first_prediction.uncertainty_db,
        }];
        let mut envelope_spectrum = first_prediction.spectrum;
        let mut peak_sample_index = 0usize;

        for (sample_index, sample) in samples.iter().enumerate().skip(1) {
            let prediction = self.evaluate_class(&sample.class_id, sample.input)?;
            let oaspl_db = prediction.oaspl.db_re_20upa();
            if oaspl_db > peak_oaspl_db {
                peak_oaspl_db = oaspl_db;
                peak_sample_index = sample_index;
            }
            uncertainty_db = uncertainty_db.max(prediction.uncertainty_db);
            envelope_spectrum = envelope_spectrum.envelope(&prediction.spectrum)?;
            points.push(AscentAcousticTracePoint {
                class_id: prediction.class_id,
                spectrum: prediction.spectrum,
                dynamic_pressure_pa: sample.input.dynamic_pressure_pa,
                mach: sample.input.mach,
                oaspl: prediction.oaspl,
                uncertainty_db: prediction.uncertainty_db,
            });
        }
        let envelope_oaspl = envelope_spectrum.oaspl()?;
        Ok(AscentAcousticTracePrediction {
            points,
            envelope_spectrum,
            envelope_oaspl,
            peak_sample_index,
            uncertainty_db,
        })
    }
}

/// Trajectory sample used by ascent acoustic class evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct AscentAcousticInput {
    /// Dynamic pressure, Pa.
    pub dynamic_pressure_pa: f64,
    /// Mach number.
    pub mach: f64,
}

impl AscentAcousticInput {
    /// Construct a validated ascent acoustic sample.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] if dynamic pressure is not positive or
    /// Mach is non-finite/negative.
    pub fn new(dynamic_pressure_pa: f64, mach: f64) -> Result<Self, AscentAcousticsError> {
        let input = Self {
            dynamic_pressure_pa,
            mach,
        };
        input.validate()?;
        Ok(input)
    }

    fn validate(self) -> Result<(), AscentAcousticsError> {
        require_positive("dynamic_pressure_pa", self.dynamic_pressure_pa)?;
        require_non_negative("mach", self.mach)?;
        Ok(())
    }
}

/// Evaluated ascent acoustic environment for one class/sample pair.
#[derive(Clone, Debug, PartialEq)]
pub struct AscentAcousticPrediction {
    /// Selected class id.
    pub class_id: String,
    /// One-third-octave SPL spectrum normalized to the predicted OASPL.
    pub spectrum: SplSpectrum,
    /// Overall sound-pressure level, dB re 20 uPa.
    pub oaspl: Spl,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
}

/// One sample in an ascent acoustic trace.
#[derive(Clone, Debug, PartialEq)]
pub struct AscentAcousticTraceSample {
    /// Class id selected for this sample.
    pub class_id: String,
    /// Dynamic-pressure/Mach sample.
    pub input: AscentAcousticInput,
}

impl AscentAcousticTraceSample {
    /// Construct a validated trace sample.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] if the class id is empty or the input
    /// is invalid.
    pub fn new(
        class_id: impl Into<String>,
        input: AscentAcousticInput,
    ) -> Result<Self, AscentAcousticsError> {
        let class_id = class_id.into();
        require_text("samples.class_id", &class_id)?;
        input.validate()?;
        Ok(Self {
            class_id: class_id.trim().to_owned(),
            input,
        })
    }
}

/// Evaluated sample in an ascent acoustic trace.
#[derive(Clone, Debug, PartialEq)]
pub struct AscentAcousticTracePoint {
    /// Selected class id.
    pub class_id: String,
    /// One-third-octave SPL spectrum for this sample.
    pub spectrum: SplSpectrum,
    /// Dynamic pressure, Pa.
    pub dynamic_pressure_pa: f64,
    /// Mach number.
    pub mach: f64,
    /// Sample OASPL, dB re 20 uPa.
    pub oaspl: Spl,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
}

/// Envelope over one or more ascent acoustic samples.
#[derive(Clone, Debug, PartialEq)]
pub struct AscentAcousticTracePrediction {
    /// Per-sample predictions in input order.
    pub points: Vec<AscentAcousticTracePoint>,
    /// One-third-octave envelope spectrum.
    pub envelope_spectrum: SplSpectrum,
    /// OASPL of the one-third-octave envelope spectrum, dB re 20 uPa.
    pub envelope_oaspl: Spl,
    /// Index of the sample with the largest individual OASPL.
    pub peak_sample_index: usize,
    /// Maximum declared one-sigma uncertainty across samples, dB.
    pub uncertainty_db: f64,
}

/// Provenance metadata for a fairing-internal acoustic deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct FairingInternalDeckMeta {
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

impl FairingInternalDeckMeta {
    fn from_row(row: FairingInternalDeckMetaRow) -> Result<Self, AscentAcousticsError> {
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

/// One fairing-internal transfer class in a deck.
#[derive(Clone, Debug, PartialEq)]
pub struct FairingInternalClass {
    /// Stable class id referenced by scenario files.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Per-band fairing insertion loss, dB.
    pub insertion_loss_db: Vec<f64>,
}

impl FairingInternalClass {
    fn from_row(
        row: FairingInternalClassRow,
        band_count: usize,
    ) -> Result<Self, AscentAcousticsError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        require_non_negative("classes.uncertainty_db", row.uncertainty_db)?;
        if row.insertion_loss_db.len() != band_count {
            return Err(AscentAcousticsError::LengthMismatch {
                bands: band_count,
                values: row.insertion_loss_db.len(),
            });
        }
        for value in &row.insertion_loss_db {
            require_non_negative("classes.insertion_loss_db", *value)?;
        }
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            uncertainty_db: row.uncertainty_db,
            insertion_loss_db: row.insertion_loss_db,
        })
    }
}

/// Provenance-pinned fairing-internal acoustic transfer deck.
#[derive(Clone, Debug, PartialEq)]
pub struct FairingInternalDeck {
    meta: FairingInternalDeckMeta,
    bands: Vec<AcousticBand>,
    classes: Vec<FairingInternalClass>,
    sha256_hex: String,
}

impl FairingInternalDeck {
    /// Parse a Schema-1 fairing-internal acoustic transfer deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when TOML parsing fails, the schema
    /// marker is unsupported, provenance fields are incomplete, or band/class
    /// rows are malformed.
    pub fn load_from_str(input: &str) -> Result<Self, AscentAcousticsError> {
        let parsed: FairingInternalDeckFile =
            toml::from_str(input).map_err(|error| AscentAcousticsError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_fairing_internal_deck != FAIRING_INTERNAL_DECK_SCHEMA_VERSION {
            return Err(AscentAcousticsError::UnsupportedFairingDeckSchema {
                schema_version: parsed.openbmp.acoustics_fairing_internal_deck,
            });
        }
        let bands = bands_from_center_hz(parsed.environment.center_hz)?;
        let classes = fairing_classes_from_rows(parsed.classes, bands.len())?;
        let sha256_hex = sha256_hex(input.as_bytes());
        require_sha256_hex("deck.sha256_hex", &sha256_hex)?;
        Ok(Self {
            meta: FairingInternalDeckMeta::from_row(parsed.meta)?,
            bands,
            classes,
            sha256_hex,
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &FairingInternalDeckMeta {
        &self.meta
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Declared fairing-internal classes.
    #[must_use]
    pub fn classes(&self) -> &[FairingInternalClass] {
        &self.classes
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }

    /// Look up a fairing-internal acoustic class by id.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError::UnknownFairingClass`] when `class_id` is
    /// absent.
    pub fn class(&self, class_id: &str) -> Result<&FairingInternalClass, AscentAcousticsError> {
        let class_id = class_id.trim();
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| AscentAcousticsError::UnknownFairingClass {
                class_id: class_id.to_owned(),
            })
    }

    /// Evaluate a fairing-internal class against an external ascent spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] when the class id is unknown, the input
    /// is invalid, the fairing and external spectra use different band grids,
    /// or the resulting SPL spectrum cannot be constructed.
    pub fn evaluate_class(
        &self,
        class_id: &str,
        external_spectrum: &SplSpectrum,
        input: FairingInternalInput,
    ) -> Result<FairingInternalPrediction, AscentAcousticsError> {
        input.validate()?;
        let class = self.class(class_id)?;
        if self.bands.as_slice() != external_spectrum.bands() {
            return Err(AscentAcousticsError::Acoustics(
                AcousticsError::BandGridMismatch,
            ));
        }
        let fill_factor_gain_db = input.fill_factor_gain_db();
        let transfer_db = class
            .insertion_loss_db
            .iter()
            .map(|loss_db| fill_factor_gain_db - *loss_db)
            .collect::<Vec<_>>();
        let spectrum = external_spectrum.apply_transfer_db(&transfer_db)?;
        let oaspl = spectrum.oaspl()?;
        Ok(FairingInternalPrediction {
            class_id: class.class_id.clone(),
            spectrum,
            oaspl,
            uncertainty_db: class.uncertainty_db,
            fill_factor_gain_db,
        })
    }
}

/// Payload fill-factor sample used by fairing-internal acoustic evaluation.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FairingInternalInput {
    /// Dimensionless fill factor in `[0, 1]`.
    pub fill_factor: f64,
}

impl FairingInternalInput {
    /// Construct a validated fairing-internal acoustic sample.
    ///
    /// # Errors
    ///
    /// Returns [`AscentAcousticsError`] if fill factor is non-finite or outside
    /// `[0, 1]`.
    pub fn new(fill_factor: f64) -> Result<Self, AscentAcousticsError> {
        let input = Self { fill_factor };
        input.validate()?;
        Ok(input)
    }

    fn validate(self) -> Result<(), AscentAcousticsError> {
        require_finite("fill_factor", self.fill_factor)?;
        if (0.0..=1.0).contains(&self.fill_factor) {
            Ok(())
        } else {
            Err(AscentAcousticsError::OutOfRange {
                field: "fill_factor",
                value: self.fill_factor,
                min: 0.0,
                max: 1.0,
            })
        }
    }

    fn fill_factor_gain_db(self) -> f64 {
        10.0 * (1.0 + self.fill_factor).log10()
    }
}

/// Evaluated fairing-internal acoustic environment for one class/sample pair.
#[derive(Clone, Debug, PartialEq)]
pub struct FairingInternalPrediction {
    /// Selected class id.
    pub class_id: String,
    /// Internal one-third-octave SPL spectrum after fairing transfer.
    pub spectrum: SplSpectrum,
    /// Internal overall sound-pressure level, dB re 20 uPa.
    pub oaspl: Spl,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Fill-factor gain applied to all bands, dB.
    pub fill_factor_gain_db: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AscentAcousticDeckFile {
    openbmp: AscentAcousticSchemaMarker,
    meta: AscentAcousticDeckMetaRow,
    environment: AscentAcousticEnvironmentSection,
    classes: Vec<AscentAcousticClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AscentAcousticSchemaMarker {
    acoustics_ascent_environment_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AscentAcousticDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AscentAcousticEnvironmentSection {
    center_hz: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct AscentAcousticClassRow {
    class_id: String,
    description: String,
    reference_dynamic_pressure_pa: f64,
    dynamic_pressure_exponent: f64,
    mach_min: f64,
    mach_max: f64,
    reference_oaspl_db: f64,
    uncertainty_db: f64,
    relative_band_level_db: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FairingInternalDeckFile {
    openbmp: FairingInternalSchemaMarker,
    meta: FairingInternalDeckMetaRow,
    environment: FairingInternalEnvironmentSection,
    classes: Vec<FairingInternalClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FairingInternalSchemaMarker {
    acoustics_fairing_internal_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FairingInternalDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FairingInternalEnvironmentSection {
    center_hz: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct FairingInternalClassRow {
    class_id: String,
    description: String,
    uncertainty_db: f64,
    insertion_loss_db: Vec<f64>,
}

fn spectrum_from_relative_shape(
    bands: &[AcousticBand],
    oaspl_db: f64,
    relative_band_level_db: &[f64],
) -> Result<SplSpectrum, AscentAcousticsError> {
    require_finite("oaspl_db", oaspl_db)?;
    if bands.len() != relative_band_level_db.len() {
        return Err(AscentAcousticsError::LengthMismatch {
            bands: bands.len(),
            values: relative_band_level_db.len(),
        });
    }
    let mut relative_power_sum = 0.0;
    for relative_db in relative_band_level_db {
        require_finite("relative_band_level_db", *relative_db)?;
        relative_power_sum += 10.0_f64.powf(*relative_db / 10.0);
    }
    require_positive("relative_band_level_power_sum", relative_power_sum)?;
    let normalizer_db = 10.0 * relative_power_sum.log10();
    let levels = relative_band_level_db
        .iter()
        .map(|relative_db| Spl::new(oaspl_db + *relative_db - normalizer_db))
        .collect::<Result<Vec<_>, _>>()?;
    Ok(SplSpectrum::new(bands.to_vec(), levels)?)
}

fn bands_from_center_hz(values: Vec<f64>) -> Result<Vec<AcousticBand>, AscentAcousticsError> {
    if values.is_empty() {
        return Err(AscentAcousticsError::Empty {
            field: "environment.center_hz",
        });
    }
    let mut previous = None;
    let mut bands = Vec::with_capacity(values.len());
    for value in values {
        if previous.is_some_and(|previous| value <= previous) {
            return Err(AscentAcousticsError::NonIncreasing {
                field: "environment.center_hz",
            });
        }
        previous = Some(value);
        bands.push(AcousticBand::one_third_octave(value)?);
    }
    Ok(bands)
}

fn classes_from_rows(
    rows: Vec<AscentAcousticClassRow>,
    band_count: usize,
) -> Result<Vec<AscentAcousticClass>, AscentAcousticsError> {
    if rows.is_empty() {
        return Err(AscentAcousticsError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = AscentAcousticClass::from_row(row, band_count)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(AscentAcousticsError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn fairing_classes_from_rows(
    rows: Vec<FairingInternalClassRow>,
    band_count: usize,
) -> Result<Vec<FairingInternalClass>, AscentAcousticsError> {
    if rows.is_empty() {
        return Err(AscentAcousticsError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = FairingInternalClass::from_row(row, band_count)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(AscentAcousticsError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn require_text(field: &'static str, value: &str) -> Result<(), AscentAcousticsError> {
    if value.trim().is_empty() {
        Err(AscentAcousticsError::Empty { field })
    } else {
        Ok(())
    }
}

fn require_finite(field: &'static str, value: f64) -> Result<(), AscentAcousticsError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(AscentAcousticsError::NonFinite { field })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<(), AscentAcousticsError> {
    require_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(AscentAcousticsError::NonPositive { field })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), AscentAcousticsError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(AscentAcousticsError::Negative { field })
    }
}

fn require_sha256_hex(field: &'static str, value: &str) -> Result<(), AscentAcousticsError> {
    if value.len() == 64
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
    {
        Ok(())
    } else {
        Err(AscentAcousticsError::InvalidSha256 { field })
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

    #[test]
    fn ascent_acoustic_deck_parses_fixture_and_evaluates_q_scaling() {
        let fixture = include_str!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let deck = AscentAcousticDeck::load_from_str(fixture).expect("fixture parses");
        require_sha256_hex("deck.sha256_hex", deck.sha256_hex()).unwrap();

        assert_eq!(deck.meta().dataset_id, "openbmp.acoustics.ascent_env.v1");
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(deck.classes().len(), 2);
        assert_eq!(
            deck.bands().len(),
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );

        let reference = deck
            .evaluate_class(
                "attached_turbulent",
                AscentAcousticInput::new(50_000.0, 0.8).unwrap(),
            )
            .expect("reference class evaluates");
        assert_eq!(reference.class_id, "attached_turbulent");
        assert_eq!(reference.uncertainty_db.to_bits(), 3.0_f64.to_bits());
        assert_relative_eq!(reference.oaspl.db_re_20upa(), 132.0, epsilon = 1.0e-12);
        assert_relative_eq!(
            reference.spectrum.oaspl().unwrap().db_re_20upa(),
            reference.oaspl.db_re_20upa(),
            epsilon = 1.0e-12
        );

        let high_q = deck
            .evaluate_class(
                "attached_turbulent",
                AscentAcousticInput::new(200_000.0, 0.8).unwrap(),
            )
            .expect("high-q class evaluates");
        let expected_delta = 10.0 * 0.6 * (4.0_f64).log10();
        assert_relative_eq!(
            high_q.oaspl.db_re_20upa() - reference.oaspl.db_re_20upa(),
            expected_delta,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn ascent_acoustic_deck_evaluates_trace_envelope() {
        let fixture = include_str!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let deck = AscentAcousticDeck::load_from_str(fixture).expect("fixture parses");
        let samples = vec![
            AscentAcousticTraceSample::new(
                "attached_turbulent",
                AscentAcousticInput::new(50_000.0, 0.8).unwrap(),
            )
            .unwrap(),
            AscentAcousticTraceSample::new(
                "attached_turbulent",
                AscentAcousticInput::new(200_000.0, 0.8).unwrap(),
            )
            .unwrap(),
        ];
        let trace = deck.evaluate_trace(&samples).expect("trace evaluates");
        let expected_high_q_oaspl = 132.0 + 10.0 * 0.6 * 4.0_f64.log10();

        assert_eq!(trace.points.len(), 2);
        assert_eq!(trace.peak_sample_index, 1);
        assert_eq!(trace.uncertainty_db.to_bits(), 3.0_f64.to_bits());
        assert_relative_eq!(
            trace.points[0].oaspl.db_re_20upa(),
            132.0,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            trace.points[1].oaspl.db_re_20upa(),
            expected_high_q_oaspl,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            trace.envelope_oaspl.db_re_20upa(),
            expected_high_q_oaspl,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            trace.envelope_spectrum.oaspl().unwrap().db_re_20upa(),
            trace.envelope_oaspl.db_re_20upa(),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn ascent_acoustic_deck_selects_classes_by_mach_gate() {
        let fixture = include_str!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let deck = AscentAcousticDeck::load_from_str(fixture).expect("fixture parses");

        let attached_input = AscentAcousticInput::new(50_000.0, 0.5).unwrap();
        let attached = deck
            .select_class_for_input(attached_input)
            .expect("attached class selected");
        assert_eq!(attached.class_id, "attached_turbulent");
        let attached_prediction = deck
            .evaluate_mach_gate(attached_input)
            .expect("attached selected evaluation succeeds");
        assert_eq!(attached_prediction.class_id, "attached_turbulent");

        let transonic_input = AscentAcousticInput::new(65_000.0, 1.1).unwrap();
        let transonic = deck
            .select_class_for_input(transonic_input)
            .expect("transonic class selected");
        assert_eq!(transonic.class_id, "transonic_buffet");
        let transonic_prediction = deck
            .evaluate_mach_gate(transonic_input)
            .expect("transonic selected evaluation succeeds");
        assert_eq!(transonic_prediction.class_id, "transonic_buffet");

        assert!(matches!(
            deck.select_class_for_input(AscentAcousticInput::new(50_000.0, 0.1).unwrap()),
            Err(AscentAcousticsError::NoMatchingClassGate { field: "mach", .. })
        ));
        match deck
            .select_class_for_input(AscentAcousticInput::new(50_000.0, 0.85).unwrap())
            .unwrap_err()
        {
            AscentAcousticsError::AmbiguousClassGate {
                field, class_ids, ..
            } => {
                assert_eq!(field, "mach");
                assert_eq!(
                    class_ids,
                    vec![
                        "attached_turbulent".to_owned(),
                        "transonic_buffet".to_owned()
                    ]
                );
            }
            other => panic!("unexpected selection error: {other:?}"),
        }
    }

    #[test]
    fn ascent_acoustic_deck_helpers_fail_closed_on_bad_inputs() {
        let fixture = include_str!("../../../data/acoustics/ascent-acoustic-env-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_ascent_environment_deck = 1",
            "openbmp.acoustics_ascent_environment_deck = 2",
        );
        assert!(matches!(
            AscentAcousticDeck::load_from_str(&bad_schema),
            Err(AscentAcousticsError::UnsupportedDeckSchema { .. })
        ));

        let duplicate_class = fixture.replacen(
            "class_id = \"transonic_buffet\"",
            "class_id = \"attached_turbulent\"",
            1,
        );
        assert!(matches!(
            AscentAcousticDeck::load_from_str(&duplicate_class),
            Err(AscentAcousticsError::Duplicate {
                field: "classes.class_id"
            })
        ));

        let short_shape = fixture.replacen("  0.0,\n]\n", "]\n", 1);
        assert!(matches!(
            AscentAcousticDeck::load_from_str(&short_shape),
            Err(AscentAcousticsError::LengthMismatch { .. })
        ));

        let deck = AscentAcousticDeck::load_from_str(fixture).expect("fixture parses");
        assert!(matches!(
            AscentAcousticInput::new(0.0, 0.8),
            Err(AscentAcousticsError::NonPositive {
                field: "dynamic_pressure_pa"
            })
        ));
        assert!(matches!(
            deck.evaluate_class(
                "attached_turbulent",
                AscentAcousticInput::new(50_000.0, 1.4).unwrap(),
            ),
            Err(AscentAcousticsError::OutOfRange { field: "mach", .. })
        ));
        assert!(matches!(
            deck.evaluate_class("missing", AscentAcousticInput::new(50_000.0, 0.8).unwrap(),),
            Err(AscentAcousticsError::UnknownClass { .. })
        ));
        assert!(matches!(
            deck.evaluate_trace(&[]),
            Err(AscentAcousticsError::Empty { field: "samples" })
        ));
    }

    #[test]
    fn fairing_internal_deck_parses_fixture_and_applies_insertion_loss() {
        let ascent_fixture = include_str!("../../../data/acoustics/ascent-acoustic-env-v1.toml");
        let fairing_fixture = include_str!("../../../data/acoustics/fairing-internal-v1.toml");
        let ascent_deck = AscentAcousticDeck::load_from_str(ascent_fixture).expect("ascent parses");
        let external = ascent_deck
            .evaluate_class(
                "attached_turbulent",
                AscentAcousticInput::new(50_000.0, 0.8).unwrap(),
            )
            .expect("external spectrum evaluates");
        let fairing_deck =
            FairingInternalDeck::load_from_str(fairing_fixture).expect("fairing fixture parses");
        require_sha256_hex("deck.sha256_hex", fairing_deck.sha256_hex()).unwrap();

        assert_eq!(
            fairing_deck.meta().dataset_id,
            "openbmp.acoustics.fairing_internal.v1"
        );
        assert_eq!(fairing_deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(fairing_deck.classes().len(), 2);
        assert_eq!(
            fairing_deck.bands().len(),
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );

        let prediction = fairing_deck
            .evaluate_class(
                "synthetic_light_fairing",
                &external.spectrum,
                FairingInternalInput::new(1.0).unwrap(),
            )
            .expect("fairing class evaluates");
        let expected_gain_db = 10.0 * 2.0_f64.log10();
        let expected_oaspl_db = 132.0 - 8.0 + expected_gain_db;
        assert_eq!(prediction.class_id, "synthetic_light_fairing");
        assert_eq!(prediction.uncertainty_db.to_bits(), 2.5_f64.to_bits());
        assert_relative_eq!(
            prediction.fill_factor_gain_db,
            expected_gain_db,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            prediction.oaspl.db_re_20upa(),
            expected_oaspl_db,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            prediction.spectrum.oaspl().unwrap().db_re_20upa(),
            prediction.oaspl.db_re_20upa(),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn fairing_internal_deck_helpers_fail_closed_on_bad_inputs() {
        let fixture = include_str!("../../../data/acoustics/fairing-internal-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_fairing_internal_deck = 1",
            "openbmp.acoustics_fairing_internal_deck = 2",
        );
        assert!(matches!(
            FairingInternalDeck::load_from_str(&bad_schema),
            Err(AscentAcousticsError::UnsupportedFairingDeckSchema { .. })
        ));

        let duplicate_class = fixture.replacen(
            "class_id = \"synthetic_heavy_fairing\"",
            "class_id = \"synthetic_light_fairing\"",
            1,
        );
        assert!(matches!(
            FairingInternalDeck::load_from_str(&duplicate_class),
            Err(AscentAcousticsError::Duplicate {
                field: "classes.class_id"
            })
        ));

        let short_transfer = fixture.replacen("  8.0,\n]\n\n[[classes]]", "]\n\n[[classes]]", 1);
        assert!(matches!(
            FairingInternalDeck::load_from_str(&short_transfer),
            Err(AscentAcousticsError::LengthMismatch { .. })
        ));

        let deck = FairingInternalDeck::load_from_str(fixture).expect("fixture parses");
        let external = SplSpectrum::standard_third_octave(
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ
                .iter()
                .map(|_| Spl::new(100.0).unwrap())
                .collect(),
        )
        .unwrap();
        assert!(matches!(
            FairingInternalInput::new(1.1),
            Err(AscentAcousticsError::OutOfRange {
                field: "fill_factor",
                ..
            })
        ));
        assert!(matches!(
            deck.evaluate_class(
                "missing",
                &external,
                FairingInternalInput::new(0.5).unwrap(),
            ),
            Err(AscentAcousticsError::UnknownFairingClass { .. })
        ));

        let mismatched_external = SplSpectrum::new(
            vec![AcousticBand::one_third_octave(1000.0).unwrap()],
            vec![Spl::new(100.0).unwrap()],
        )
        .unwrap();
        assert!(matches!(
            deck.evaluate_class(
                "synthetic_light_fairing",
                &mismatched_external,
                FairingInternalInput::new(0.5).unwrap(),
            ),
            Err(AscentAcousticsError::Acoustics(
                AcousticsError::BandGridMismatch
            ))
        ));
    }
}
