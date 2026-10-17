//! Distributed-source-method liftoff acoustics substrate.
//!
//! This is a deterministic free-field single-engine slice: acoustic power from
//! exit conditions, source-line allocation, Strouhal-shaped band fractions,
//! directivity interpolation, spherical spreading, air absorption, and station
//! spectrum synthesis. Pad geometry, water suppression, and digitized SP-8072
//! production decks are later layers.

use std::collections::BTreeSet;
use std::f64::consts::PI;
use std::fmt::Write as _;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{AcousticBand, AcousticsError, Spl, SplSpectrum, standard_third_octave_bands};

/// Reference acoustic power in watts for sound-power level.
pub const REFERENCE_ACOUSTIC_POWER_W: f64 = 1.0e-12;
/// Schema version for provenance-pinned DSM source/directivity decks.
pub const DSM_DECK_SCHEMA_VERSION: u32 = 1;
/// Schema version for provenance-pinned DSM air-absorption decks.
pub const DSM_ABSORPTION_DECK_SCHEMA_VERSION: u32 = 1;
/// Schema version for the NASA SP-8072 source-manifest inventory.
pub const SP8072_SOURCE_MANIFEST_SCHEMA_VERSION: u32 = 1;
/// Schema version for digitized SP-8072 axial source-power curves.
pub const SP8072_AXIAL_POWER_CURVE_SCHEMA_VERSION: u32 = 1;
/// Schema version for digitized SP-8072 axial spectrum curves.
pub const SP8072_AXIAL_SPECTRUM_CURVE_SCHEMA_VERSION: u32 = 1;
/// Schema version for digitized SP-8072 spectrum curves.
pub const SP8072_SPECTRUM_CURVE_SCHEMA_VERSION: u32 = 1;
/// Schema version for digitized SP-8072 directivity-index curves.
pub const SP8072_DIRECTIVITY_CURVE_SCHEMA_VERSION: u32 = 1;
/// Schema version for digitized SP-8072 apparent-source-location curves.
pub const SP8072_APPARENT_SOURCE_CURVE_SCHEMA_VERSION: u32 = 1;

/// Errors returned by DSM helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum DsmError {
    /// A lower-level acoustic spectral value was invalid.
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
    /// A fraction was outside `[0, 1]`.
    #[error("{field} must lie in [0, 1]")]
    InvalidFraction {
        /// Field name.
        field: &'static str,
    },
    /// A digest was not a lowercase SHA-256 hex string.
    #[error("{field} must be a lowercase SHA-256 hex digest")]
    InvalidSha256 {
        /// Field name.
        field: &'static str,
    },
    /// A list was empty.
    #[error("{field} must not be empty")]
    Empty {
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
    /// Angles or source positions were not strictly increasing.
    #[error("{field} must be strictly increasing")]
    NonIncreasing {
        /// Field name.
        field: &'static str,
    },
    /// A list contained duplicate values.
    #[error("{field} must not contain duplicate values")]
    Duplicate {
        /// Field name.
        field: &'static str,
    },
    /// A value was outside a validated curve domain.
    #[error("{field} is outside the curve domain")]
    OutOfRange {
        /// Field name.
        field: &'static str,
    },
    /// A DSM deck schema version is unsupported.
    #[error("unsupported DSM deck schema version {schema_version}")]
    UnsupportedDeckSchema {
        /// Parsed schema version.
        schema_version: u32,
    },
    /// A DSM deck could not be parsed.
    #[error("malformed DSM deck: {reason}")]
    MalformedDeck {
        /// Parse or structural failure.
        reason: String,
    },
}

/// Provenance metadata carried by a DSM source/directivity deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DsmDeckMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source class, for example `synthetic-openbmp` or `published-public`.
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

impl DsmDeckMeta {
    fn from_row(row: DsmDeckMetaRow) -> Result<Self, DsmError> {
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

/// Provenance-pinned DSM source-allocation and directivity deck.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmPredictionDeck {
    meta: DsmDeckMeta,
    source_allocation: DsmSourceAllocation,
    directivity: DsmDirectivityDeck,
    sha256_hex: String,
}

impl DsmPredictionDeck {
    /// Parse a Schema-1 DSM deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are empty, source allocation is not
    /// strictly increasing and normalized, or directivity dimensions are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: DsmDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_dsm_deck != DSM_DECK_SCHEMA_VERSION {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_dsm_deck,
            });
        }
        let meta = DsmDeckMeta::from_row(parsed.meta)?;
        let source_allocation = source_allocation_from_deck_rows(parsed.source_allocation)?;
        let bands = parsed
            .directivity
            .center_hz
            .into_iter()
            .map(AcousticBand::one_third_octave)
            .collect::<Result<Vec<_>, _>>()?;
        let directivity = DsmDirectivityDeck::new(
            bands,
            parsed.directivity.angles_deg,
            parsed.directivity.rows_db,
        )?;
        Ok(Self {
            meta,
            source_allocation,
            directivity,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &DsmDeckMeta {
        &self.meta
    }

    /// Parsed source allocation.
    #[must_use]
    pub fn source_allocation(&self) -> &DsmSourceAllocation {
        &self.source_allocation
    }

    /// Parsed directivity deck.
    #[must_use]
    pub fn directivity(&self) -> &DsmDirectivityDeck {
        &self.directivity
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance-pinned DSM air-absorption deck.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmAirAbsorptionDeck {
    meta: DsmDeckMeta,
    profiles: Vec<DsmAirAbsorptionProfile>,
    sha256_hex: String,
}

/// One named atmospheric air-absorption profile.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmAirAbsorptionProfile {
    /// Stable profile id.
    pub profile_id: String,
    /// Human-readable profile description.
    pub description: String,
    /// Representative static air temperature, K, when declared.
    pub temperature_k: Option<f64>,
    /// Representative static pressure, Pa, when declared.
    pub static_pressure_pa: Option<f64>,
    /// Representative relative humidity in `[0, 1]`, when declared.
    pub relative_humidity: Option<f64>,
    absorption: DsmAbsorptionDeck,
}

impl DsmAirAbsorptionDeck {
    /// Parse a Schema-1 DSM air-absorption deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance metadata is incomplete, or the absorption band
    /// grid and values are malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: DsmAirAbsorptionDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_dsm_absorption_deck != DSM_ABSORPTION_DECK_SCHEMA_VERSION {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_dsm_absorption_deck,
            });
        }
        let profiles = absorption_profiles_from_deck(parsed.absorption, parsed.profiles)?;
        Ok(Self {
            meta: DsmDeckMeta::from_row(parsed.meta)?,
            profiles,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &DsmDeckMeta {
        &self.meta
    }

    /// Parsed per-band absorption deck.
    #[must_use]
    pub fn absorption(&self) -> &DsmAbsorptionDeck {
        &self.profiles[0].absorption
    }

    /// Parsed atmospheric absorption profiles.
    #[must_use]
    pub fn profiles(&self) -> &[DsmAirAbsorptionProfile] {
        &self.profiles
    }

    /// Select the default or a named atmospheric absorption profile.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `profile_id` is declared but no matching
    /// profile exists in the deck.
    pub fn absorption_profile(
        &self,
        profile_id: Option<&str>,
    ) -> Result<&DsmAirAbsorptionProfile, DsmError> {
        let Some(profile_id) = profile_id else {
            return Ok(&self.profiles[0]);
        };
        require_text("absorption_profile_id", profile_id)?;
        self.profiles
            .iter()
            .find(|profile| profile.profile_id == profile_id)
            .ok_or_else(|| DsmError::MalformedDeck {
                reason: format!("unknown absorption profile id `{profile_id}`"),
            })
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

impl DsmAirAbsorptionProfile {
    /// Parsed per-band absorption deck for this profile.
    #[must_use]
    pub fn absorption(&self) -> &DsmAbsorptionDeck {
        &self.absorption
    }
}

/// Provenance metadata for the public NASA SP-8072 source PDF.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072SourceMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Human-readable source title.
    pub source_title: String,
    /// NASA Technical Reports Server citation id.
    pub ntrs_citation_id: String,
    /// Public source URL used for retrieval.
    pub source_url: String,
    /// SHA-256 digest of the retrieved source PDF.
    pub pdf_sha256: String,
    /// Retrieval date in UTC.
    pub retrieved_utc: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label for this source inventory.
    pub validation: String,
}

impl Sp8072SourceMeta {
    fn from_row(row: Sp8072SourceMetaRow) -> Result<Self, DsmError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_title", &row.source_title)?;
        require_text("meta.ntrs_citation_id", &row.ntrs_citation_id)?;
        require_text("meta.source_url", &row.source_url)?;
        require_sha256_hex("meta.pdf_sha256", &row.pdf_sha256)?;
        require_text("meta.retrieved_utc", &row.retrieved_utc)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_title: row.source_title.trim().to_owned(),
            ntrs_citation_id: row.ntrs_citation_id.trim().to_owned(),
            source_url: row.source_url.trim().to_owned(),
            pdf_sha256: row.pdf_sha256.trim().to_owned(),
            retrieved_utc: row.retrieved_utc.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// One SP-8072 figure that future digitized DSM decks may cite.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072FigureReference {
    /// Stable figure id, for example `figure_12`.
    pub figure_id: String,
    /// One-based PDF page number in the pinned source PDF.
    pub pdf_page: u32,
    /// Figure title from the source.
    pub title: String,
    /// Intended OpenBMP use of the figure.
    pub use_for: String,
    /// Digitization status for this source figure.
    pub digitization_status: String,
}

impl Sp8072FigureReference {
    fn from_row(row: Sp8072FigureRow) -> Result<Self, DsmError> {
        require_text("figures.figure_id", &row.figure_id)?;
        if row.pdf_page == 0 {
            return Err(DsmError::MalformedDeck {
                reason: "figures.pdf_page must be positive".to_owned(),
            });
        }
        require_text("figures.title", &row.title)?;
        require_text("figures.use_for", &row.use_for)?;
        require_text("figures.digitization_status", &row.digitization_status)?;
        Ok(Self {
            figure_id: row.figure_id.trim().to_owned(),
            pdf_page: row.pdf_page,
            title: row.title.trim().to_owned(),
            use_for: row.use_for.trim().to_owned(),
            digitization_status: row.digitization_status.trim().to_owned(),
        })
    }
}

/// Provenance-pinned inventory for the public NASA SP-8072 source PDF.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072SourceManifest {
    meta: Sp8072SourceMeta,
    figures: Vec<Sp8072FigureReference>,
    sha256_hex: String,
}

impl Sp8072SourceManifest {
    /// Parse a Schema-1 NASA SP-8072 source manifest from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance metadata is incomplete, the PDF digest is not a
    /// lowercase SHA-256 hex string, or the figure inventory is empty or
    /// contains duplicate ids.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: Sp8072SourceManifestFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_sp8072_source_manifest != SP8072_SOURCE_MANIFEST_SCHEMA_VERSION
        {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_sp8072_source_manifest,
            });
        }
        let meta = Sp8072SourceMeta::from_row(parsed.meta)?;
        let figures = sp8072_figures_from_rows(parsed.figures)?;
        Ok(Self {
            meta,
            figures,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Source provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &Sp8072SourceMeta {
        &self.meta
    }

    /// Referenced SP-8072 figures.
    #[must_use]
    pub fn figures(&self) -> &[Sp8072FigureReference] {
        &self.figures
    }

    /// Find a figure by id.
    #[must_use]
    pub fn figure(&self, figure_id: &str) -> Option<&Sp8072FigureReference> {
        self.figures
            .iter()
            .find(|figure| figure.figure_id == figure_id)
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance metadata for a digitized NASA SP-8072 axial power curve.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072AxialPowerCurveMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source manifest dataset id.
    pub source_manifest_id: String,
    /// Repository-relative source manifest path.
    pub source_manifest_file: String,
    /// Source figure id from the SP-8072 source manifest.
    pub source_figure_id: String,
    /// SHA-256 digest of the retrieved source PDF.
    pub source_pdf_sha256: String,
    /// Human-readable digitization method statement.
    pub digitization_method: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label for this digitized curve.
    pub validation: String,
}

impl Sp8072AxialPowerCurveMeta {
    fn from_row(row: Sp8072AxialPowerCurveMetaRow) -> Result<Self, DsmError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_manifest_id", &row.source_manifest_id)?;
        require_text("meta.source_manifest_file", &row.source_manifest_file)?;
        require_text("meta.source_figure_id", &row.source_figure_id)?;
        require_sha256_hex("meta.source_pdf_sha256", &row.source_pdf_sha256)?;
        require_text("meta.digitization_method", &row.digitization_method)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_manifest_id: row.source_manifest_id.trim().to_owned(),
            source_manifest_file: row.source_manifest_file.trim().to_owned(),
            source_figure_id: row.source_figure_id.trim().to_owned(),
            source_pdf_sha256: row.source_pdf_sha256.trim().to_owned(),
            digitization_method: row.digitization_method.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// One digitized point from SP-8072 Figure 12.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmAxialPowerPoint {
    /// Axial coordinate normalized by core length, `x / x_t`.
    pub x_over_core_length: f64,
    /// Relative sound power per unit core length in dB.
    pub relative_power_db: f64,
}

/// Dimensionless axial source-power density curve.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmAxialPowerCurve {
    points: Vec<DsmAxialPowerPoint>,
}

impl DsmAxialPowerCurve {
    /// Construct a validated axial power-density curve.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when fewer than two points are supplied, positions
    /// are not finite positive and strictly increasing, or relative dB values
    /// are not finite.
    pub fn new(points: Vec<DsmAxialPowerPoint>) -> Result<Self, DsmError> {
        if points.len() < 2 {
            return Err(DsmError::MalformedDeck {
                reason: "axial power curve requires at least two points".to_owned(),
            });
        }
        let mut previous = None;
        for point in &points {
            require_positive("axial_power.x_over_core_length", point.x_over_core_length)?;
            require_finite("axial_power.relative_power_db", point.relative_power_db)?;
            if previous.is_some_and(|previous| point.x_over_core_length <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "axial_power.x_over_core_length",
                });
            }
            previous = Some(point.x_over_core_length);
        }
        Ok(Self { points })
    }

    /// Digitized points in source order.
    #[must_use]
    pub fn points(&self) -> &[DsmAxialPowerPoint] {
        &self.points
    }

    /// Interpolate relative power-density dB in log-x space.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `x_over_core_length` is not finite and
    /// positive or lies outside the digitized curve domain.
    pub fn relative_power_db_at(&self, x_over_core_length: f64) -> Result<f64, DsmError> {
        require_positive("x_over_core_length", x_over_core_length)?;
        let first = self.points[0];
        let last = self.points[self.points.len() - 1];
        if x_over_core_length < first.x_over_core_length
            || x_over_core_length > last.x_over_core_length
        {
            return Err(DsmError::OutOfRange {
                field: "x_over_core_length",
            });
        }
        for point in &self.points {
            if point.x_over_core_length.to_bits() == x_over_core_length.to_bits() {
                return Ok(point.relative_power_db);
            }
        }
        let upper = self
            .points
            .iter()
            .position(|point| point.x_over_core_length >= x_over_core_length)
            .ok_or(DsmError::OutOfRange {
                field: "x_over_core_length",
            })?;
        let lower = upper - 1;
        let lower_point = self.points[lower];
        let upper_point = self.points[upper];
        let lower_log = lower_point.x_over_core_length.log10();
        let span_log = upper_point.x_over_core_length.log10() - lower_log;
        let fraction = (x_over_core_length.log10() - lower_log) / span_log;
        Ok(lower_point.relative_power_db * (1.0 - fraction)
            + upper_point.relative_power_db * fraction)
    }

    /// Convert the dimensionless power-density samples into a normalized DSM
    /// point-source allocation using trapezoidal tributary widths.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `core_length_m` is not finite and positive or
    /// derived weights are invalid.
    pub fn to_source_allocation(
        &self,
        core_length_m: f64,
    ) -> Result<DsmSourceAllocation, DsmError> {
        require_positive("core_length_m", core_length_m)?;
        let mut weighted_sources = Vec::with_capacity(self.points.len());
        for (index, point) in self.points.iter().enumerate() {
            let width = if index == 0 {
                0.5 * (self.points[1].x_over_core_length - point.x_over_core_length)
            } else if index == self.points.len() - 1 {
                0.5 * (point.x_over_core_length - self.points[index - 1].x_over_core_length)
            } else {
                0.5 * (self.points[index + 1].x_over_core_length
                    - self.points[index - 1].x_over_core_length)
            };
            let power_density = 10.0_f64.powf(point.relative_power_db / 10.0);
            weighted_sources.push((
                point.x_over_core_length * core_length_m,
                power_density * width,
            ));
        }
        DsmSourceAllocation::normalized_from_weights(&weighted_sources)
    }
}

/// Provenance-pinned SP-8072 axial source-power curve deck.
#[derive(Clone, Debug, PartialEq)]
pub struct Sp8072AxialPowerCurveDeck {
    meta: Sp8072AxialPowerCurveMeta,
    curve: DsmAxialPowerCurve,
    sha256_hex: String,
}

impl Sp8072AxialPowerCurveDeck {
    /// Parse a Schema-1 SP-8072 axial power curve deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are incomplete, or the curve points are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: Sp8072AxialPowerCurveDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_sp8072_axial_power_curve
            != SP8072_AXIAL_POWER_CURVE_SCHEMA_VERSION
        {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_sp8072_axial_power_curve,
            });
        }
        Ok(Self {
            meta: Sp8072AxialPowerCurveMeta::from_row(parsed.meta)?,
            curve: axial_power_curve_from_rows(parsed.points)?,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Curve provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &Sp8072AxialPowerCurveMeta {
        &self.meta
    }

    /// Verify this curve deck cites the supplied SP-8072 source manifest.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the deck's manifest dataset id, pinned source
    /// PDF SHA-256, or source figure id does not match the supplied manifest.
    pub fn validate_source_manifest(
        &self,
        manifest: &Sp8072SourceManifest,
    ) -> Result<(), DsmError> {
        validate_sp8072_curve_source_manifest(
            &self.meta.source_manifest_id,
            &self.meta.source_figure_id,
            &self.meta.source_pdf_sha256,
            manifest,
        )
    }

    /// Parsed axial power curve.
    #[must_use]
    pub fn curve(&self) -> &DsmAxialPowerCurve {
        &self.curve
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance metadata for a digitized NASA SP-8072 axial spectrum curve.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072AxialSpectrumCurveMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source manifest dataset id.
    pub source_manifest_id: String,
    /// Repository-relative source manifest path.
    pub source_manifest_file: String,
    /// Source figure id from the SP-8072 source manifest.
    pub source_figure_id: String,
    /// SHA-256 digest of the retrieved source PDF.
    pub source_pdf_sha256: String,
    /// Human-readable digitization method statement.
    pub digitization_method: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label for this digitized curve.
    pub validation: String,
}

impl Sp8072AxialSpectrumCurveMeta {
    fn from_row(row: Sp8072AxialSpectrumCurveMetaRow) -> Result<Self, DsmError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_manifest_id", &row.source_manifest_id)?;
        require_text("meta.source_manifest_file", &row.source_manifest_file)?;
        require_text("meta.source_figure_id", &row.source_figure_id)?;
        require_sha256_hex("meta.source_pdf_sha256", &row.source_pdf_sha256)?;
        require_text("meta.digitization_method", &row.digitization_method)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_manifest_id: row.source_manifest_id.trim().to_owned(),
            source_manifest_file: row.source_manifest_file.trim().to_owned(),
            source_figure_id: row.source_figure_id.trim().to_owned(),
            source_pdf_sha256: row.source_pdf_sha256.trim().to_owned(),
            digitization_method: row.digitization_method.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// One digitized point from SP-8072 Figure 13.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmAxialSpectrumPoint {
    /// Modified axial Strouhal number used by Figure 13.
    pub modified_axial_strouhal: f64,
    /// Normalized relative sound-power spectrum level in dB.
    pub relative_power_db: f64,
}

/// Dimensionless source-position-dependent spectrum curve.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmAxialSpectrumCurve {
    points: Vec<DsmAxialSpectrumPoint>,
}

impl DsmAxialSpectrumCurve {
    /// Construct a validated axial spectrum curve.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when fewer than two points are supplied, modified
    /// Strouhal values are not finite positive and strictly increasing, or
    /// relative dB values are not finite.
    pub fn new(points: Vec<DsmAxialSpectrumPoint>) -> Result<Self, DsmError> {
        if points.len() < 2 {
            return Err(DsmError::MalformedDeck {
                reason: "axial spectrum curve requires at least two points".to_owned(),
            });
        }
        let mut previous = None;
        for point in &points {
            require_positive(
                "axial_spectrum.modified_axial_strouhal",
                point.modified_axial_strouhal,
            )?;
            require_finite("axial_spectrum.relative_power_db", point.relative_power_db)?;
            if previous.is_some_and(|previous| point.modified_axial_strouhal <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "axial_spectrum.modified_axial_strouhal",
                });
            }
            previous = Some(point.modified_axial_strouhal);
        }
        Ok(Self { points })
    }

    /// Digitized points in source order.
    #[must_use]
    pub fn points(&self) -> &[DsmAxialSpectrumPoint] {
        &self.points
    }

    /// Interpolate relative spectrum dB in log modified-axial-Strouhal space,
    /// clamping outside the digitized curve domain.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `modified_axial_strouhal` is non-finite or
    /// negative.
    pub fn relative_power_db_at(&self, modified_axial_strouhal: f64) -> Result<f64, DsmError> {
        require_non_negative("modified_axial_strouhal", modified_axial_strouhal)?;
        if modified_axial_strouhal <= self.points[0].modified_axial_strouhal {
            return Ok(self.points[0].relative_power_db);
        }
        let last = self.points.len() - 1;
        if modified_axial_strouhal >= self.points[last].modified_axial_strouhal {
            return Ok(self.points[last].relative_power_db);
        }
        let axis = self
            .points
            .iter()
            .map(|point| point.modified_axial_strouhal)
            .collect::<Vec<_>>();
        let lower = lower_axis_index_log(&axis, modified_axial_strouhal);
        let upper = (lower + 1).min(axis.len() - 1);
        let fraction = log_axis_fraction(axis[lower], axis[upper], modified_axial_strouhal);
        Ok(interpolate_linear(
            self.points[lower].relative_power_db,
            self.points[upper].relative_power_db,
            fraction,
        ))
    }

    /// Combine the axial spectrum curve with a source allocation and a base
    /// band spectrum into normalized source-by-band power fractions.
    ///
    /// The current coarse implementation uses `f * x / U_e` as the modified
    /// axial Strouhal coordinate so the source-position correction can be
    /// applied deterministically before SP-8072 production digitization lands.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when exit velocity is invalid or generated weights
    /// are malformed.
    pub fn to_source_band_weights(
        &self,
        allocation: &DsmSourceAllocation,
        shape: &DsmSpectrumShape,
        exit_velocity_m_s: f64,
    ) -> Result<DsmSourceBandWeights, DsmError> {
        require_positive("exit_velocity_m_s", exit_velocity_m_s)?;
        let mut weights = Vec::with_capacity(allocation.sources().len());
        for source in allocation.sources() {
            let mut row = Vec::with_capacity(shape.bands().len());
            for (band, band_fraction) in shape.bands().iter().zip(shape.band_power_fractions()) {
                let modified_axial_strouhal = band.center_hz * source.axial_m / exit_velocity_m_s;
                let axial_level_db = self.relative_power_db_at(modified_axial_strouhal)?;
                let axial_weight = 10.0_f64.powf(axial_level_db / 10.0);
                row.push(source.power_fraction * band_fraction * axial_weight);
            }
            weights.push(row);
        }
        DsmSourceBandWeights::normalized_from_weights(shape.bands().to_vec(), weights)
    }
}

/// Provenance-pinned SP-8072 axial spectrum curve deck.
#[derive(Clone, Debug, PartialEq)]
pub struct Sp8072AxialSpectrumCurveDeck {
    meta: Sp8072AxialSpectrumCurveMeta,
    curve: DsmAxialSpectrumCurve,
    sha256_hex: String,
}

impl Sp8072AxialSpectrumCurveDeck {
    /// Parse a Schema-1 SP-8072 axial spectrum curve deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are incomplete, or the curve points are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: Sp8072AxialSpectrumCurveDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_sp8072_axial_spectrum_curve
            != SP8072_AXIAL_SPECTRUM_CURVE_SCHEMA_VERSION
        {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_sp8072_axial_spectrum_curve,
            });
        }
        Ok(Self {
            meta: Sp8072AxialSpectrumCurveMeta::from_row(parsed.meta)?,
            curve: axial_spectrum_curve_from_rows(parsed.points)?,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Curve provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &Sp8072AxialSpectrumCurveMeta {
        &self.meta
    }

    /// Verify this curve deck cites the supplied SP-8072 source manifest.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the deck's manifest dataset id, pinned source
    /// PDF SHA-256, or source figure id does not match the supplied manifest.
    pub fn validate_source_manifest(
        &self,
        manifest: &Sp8072SourceManifest,
    ) -> Result<(), DsmError> {
        validate_sp8072_curve_source_manifest(
            &self.meta.source_manifest_id,
            &self.meta.source_figure_id,
            &self.meta.source_pdf_sha256,
            manifest,
        )
    }

    /// Parsed axial spectrum curve.
    #[must_use]
    pub fn curve(&self) -> &DsmAxialSpectrumCurve {
        &self.curve
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance metadata for a digitized NASA SP-8072 spectrum curve.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072SpectrumCurveMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source manifest dataset id.
    pub source_manifest_id: String,
    /// Repository-relative source manifest path.
    pub source_manifest_file: String,
    /// Source figure id from the SP-8072 source manifest.
    pub source_figure_id: String,
    /// SHA-256 digest of the retrieved source PDF.
    pub source_pdf_sha256: String,
    /// Human-readable digitization method statement.
    pub digitization_method: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label for this digitized curve.
    pub validation: String,
}

impl Sp8072SpectrumCurveMeta {
    fn from_row(row: Sp8072SpectrumCurveMetaRow) -> Result<Self, DsmError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_manifest_id", &row.source_manifest_id)?;
        require_text("meta.source_manifest_file", &row.source_manifest_file)?;
        require_text("meta.source_figure_id", &row.source_figure_id)?;
        require_sha256_hex("meta.source_pdf_sha256", &row.source_pdf_sha256)?;
        require_text("meta.digitization_method", &row.digitization_method)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_manifest_id: row.source_manifest_id.trim().to_owned(),
            source_manifest_file: row.source_manifest_file.trim().to_owned(),
            source_figure_id: row.source_figure_id.trim().to_owned(),
            source_pdf_sha256: row.source_pdf_sha256.trim().to_owned(),
            digitization_method: row.digitization_method.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// One digitized point from SP-8072 Figure 5.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmSpectrumPoint {
    /// Strouhal number `f * d_e / U_e`.
    pub strouhal: f64,
    /// Relative sound-power density level in dB.
    pub relative_power_db: f64,
}

/// Dimensionless sound-power spectrum density curve.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmSpectrumCurve {
    points: Vec<DsmSpectrumPoint>,
}

impl DsmSpectrumCurve {
    /// Construct a validated spectrum-density curve.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when fewer than two points are supplied, Strouhal
    /// values are not finite positive and strictly increasing, or relative dB
    /// values are not finite.
    pub fn new(points: Vec<DsmSpectrumPoint>) -> Result<Self, DsmError> {
        if points.len() < 2 {
            return Err(DsmError::MalformedDeck {
                reason: "spectrum curve requires at least two points".to_owned(),
            });
        }
        let mut previous = None;
        for point in &points {
            require_positive("spectrum.strouhal", point.strouhal)?;
            require_finite("spectrum.relative_power_db", point.relative_power_db)?;
            if previous.is_some_and(|previous| point.strouhal <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "spectrum.strouhal",
                });
            }
            previous = Some(point.strouhal);
        }
        Ok(Self { points })
    }

    /// Digitized points in source order.
    #[must_use]
    pub fn points(&self) -> &[DsmSpectrumPoint] {
        &self.points
    }

    /// Interpolate relative spectrum-density dB in log-Strouhal space,
    /// clamping outside the digitized curve domain.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `strouhal` is not finite and positive.
    pub fn relative_power_db_at(&self, strouhal: f64) -> Result<f64, DsmError> {
        require_positive("strouhal", strouhal)?;
        let axis = self
            .points
            .iter()
            .map(|point| point.strouhal)
            .collect::<Vec<_>>();
        let lower = lower_axis_index_log(&axis, strouhal);
        let upper = (lower + 1).min(axis.len() - 1);
        let fraction = log_axis_fraction(axis[lower], axis[upper], strouhal);
        Ok(interpolate_linear(
            self.points[lower].relative_power_db,
            self.points[upper].relative_power_db,
            fraction,
        ))
    }

    /// Convert the SP-8072 spectrum-density curve into normalized
    /// one-third-octave band-power fractions.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when exit geometry is invalid or generated band
    /// weights are invalid.
    pub fn to_spectrum_shape(
        &self,
        exit_diameter_m: f64,
        exit_velocity_m_s: f64,
    ) -> Result<DsmSpectrumShape, DsmError> {
        require_positive("exit_diameter_m", exit_diameter_m)?;
        require_positive("exit_velocity_m_s", exit_velocity_m_s)?;
        let bands = standard_third_octave_bands();
        let mut weights = Vec::with_capacity(bands.len());
        for band in &bands {
            let strouhal = band.center_hz * exit_diameter_m / exit_velocity_m_s;
            let strouhal_width =
                (band.upper_hz - band.lower_hz) * exit_diameter_m / exit_velocity_m_s;
            let density = 10.0_f64.powf(self.relative_power_db_at(strouhal)? / 10.0);
            weights.push(density * strouhal_width);
        }
        normalize_band_weights(bands, weights)
    }
}

/// Provenance-pinned SP-8072 spectrum curve deck.
#[derive(Clone, Debug, PartialEq)]
pub struct Sp8072SpectrumCurveDeck {
    meta: Sp8072SpectrumCurveMeta,
    curve: DsmSpectrumCurve,
    sha256_hex: String,
}

impl Sp8072SpectrumCurveDeck {
    /// Parse a Schema-1 SP-8072 spectrum curve deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are incomplete, or the curve points are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: Sp8072SpectrumCurveDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_sp8072_spectrum_curve != SP8072_SPECTRUM_CURVE_SCHEMA_VERSION {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_sp8072_spectrum_curve,
            });
        }
        Ok(Self {
            meta: Sp8072SpectrumCurveMeta::from_row(parsed.meta)?,
            curve: spectrum_curve_from_rows(parsed.points)?,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Curve provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &Sp8072SpectrumCurveMeta {
        &self.meta
    }

    /// Verify this curve deck cites the supplied SP-8072 source manifest.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the deck's manifest dataset id, pinned source
    /// PDF SHA-256, or source figure id does not match the supplied manifest.
    pub fn validate_source_manifest(
        &self,
        manifest: &Sp8072SourceManifest,
    ) -> Result<(), DsmError> {
        validate_sp8072_curve_source_manifest(
            &self.meta.source_manifest_id,
            &self.meta.source_figure_id,
            &self.meta.source_pdf_sha256,
            manifest,
        )
    }

    /// Parsed spectrum curve.
    #[must_use]
    pub fn curve(&self) -> &DsmSpectrumCurve {
        &self.curve
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance metadata for a digitized NASA SP-8072 directivity curve.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072DirectivityCurveMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source manifest dataset id.
    pub source_manifest_id: String,
    /// Repository-relative source manifest path.
    pub source_manifest_file: String,
    /// Source figure id from the SP-8072 source manifest.
    pub source_figure_id: String,
    /// SHA-256 digest of the retrieved source PDF.
    pub source_pdf_sha256: String,
    /// Human-readable digitization method statement.
    pub digitization_method: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label for this digitized curve.
    pub validation: String,
}

impl Sp8072DirectivityCurveMeta {
    fn from_row(row: Sp8072DirectivityCurveMetaRow) -> Result<Self, DsmError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_manifest_id", &row.source_manifest_id)?;
        require_text("meta.source_manifest_file", &row.source_manifest_file)?;
        require_text("meta.source_figure_id", &row.source_figure_id)?;
        require_sha256_hex("meta.source_pdf_sha256", &row.source_pdf_sha256)?;
        require_text("meta.digitization_method", &row.digitization_method)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_manifest_id: row.source_manifest_id.trim().to_owned(),
            source_manifest_file: row.source_manifest_file.trim().to_owned(),
            source_figure_id: row.source_figure_id.trim().to_owned(),
            source_pdf_sha256: row.source_pdf_sha256.trim().to_owned(),
            digitization_method: row.digitization_method.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// Dimensionless SP-8072 directivity-index curve over Strouhal and angle.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmDirectivityCurve {
    strouhal_values: Vec<f64>,
    angles_deg: Vec<f64>,
    directivity_db: Vec<Vec<f64>>,
}

impl DsmDirectivityCurve {
    /// Construct a validated directivity-index curve.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when axes are empty, not finite positive/increasing
    /// where required, or directivity rows do not match the axis dimensions.
    pub fn new(
        strouhal_values: Vec<f64>,
        angles_deg: Vec<f64>,
        directivity_db: Vec<Vec<f64>>,
    ) -> Result<Self, DsmError> {
        if strouhal_values.is_empty() {
            return Err(DsmError::Empty {
                field: "strouhal_values",
            });
        }
        if angles_deg.is_empty() {
            return Err(DsmError::Empty {
                field: "angles_deg",
            });
        }
        if strouhal_values.len() != directivity_db.len() {
            return Err(DsmError::LengthMismatch {
                bands: strouhal_values.len(),
                values: directivity_db.len(),
            });
        }
        let mut previous = None;
        for value in &strouhal_values {
            require_positive("directivity.strouhal", *value)?;
            if previous.is_some_and(|previous| *value <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "directivity.strouhal",
                });
            }
            previous = Some(*value);
        }
        previous = None;
        for angle in &angles_deg {
            require_non_negative("directivity.angle_deg", *angle)?;
            if previous.is_some_and(|previous| *angle <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "directivity.angle_deg",
                });
            }
            previous = Some(*angle);
        }
        for row in &directivity_db {
            if row.len() != angles_deg.len() {
                return Err(DsmError::LengthMismatch {
                    bands: angles_deg.len(),
                    values: row.len(),
                });
            }
            for value in row {
                require_finite("directivity.db", *value)?;
            }
        }
        Ok(Self {
            strouhal_values,
            angles_deg,
            directivity_db,
        })
    }

    /// Digitized Strouhal axis values.
    #[must_use]
    pub fn strouhal_values(&self) -> &[f64] {
        &self.strouhal_values
    }

    /// Digitized angle axis values.
    #[must_use]
    pub fn angles_deg(&self) -> &[f64] {
        &self.angles_deg
    }

    /// Interpolate directivity index in dB, clamping outside the digitized
    /// Strouhal and angle domains.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if either input is non-finite or non-positive
    /// where required.
    pub fn directivity_db_at(&self, strouhal: f64, angle_deg: f64) -> Result<f64, DsmError> {
        require_positive("strouhal", strouhal)?;
        require_non_negative("angle_deg", angle_deg)?;
        let lower_s = lower_axis_index_log(&self.strouhal_values, strouhal);
        let upper_s = (lower_s + 1).min(self.strouhal_values.len() - 1);
        let lower_a = lower_axis_index_linear(&self.angles_deg, angle_deg);
        let upper_a = (lower_a + 1).min(self.angles_deg.len() - 1);

        let s_fraction = log_axis_fraction(
            self.strouhal_values[lower_s],
            self.strouhal_values[upper_s],
            strouhal,
        );
        let a_fraction = linear_axis_fraction(
            self.angles_deg[lower_a],
            self.angles_deg[upper_a],
            angle_deg,
        );
        let lower_angle_lower_s = interpolate_linear(
            self.directivity_db[lower_s][lower_a],
            self.directivity_db[lower_s][upper_a],
            a_fraction,
        );
        let lower_angle_upper_s = interpolate_linear(
            self.directivity_db[upper_s][lower_a],
            self.directivity_db[upper_s][upper_a],
            a_fraction,
        );
        Ok(interpolate_linear(
            lower_angle_lower_s,
            lower_angle_upper_s,
            s_fraction,
        ))
    }

    /// Convert the SP-8072 Strouhal/angle curve into a band directivity deck.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when exit geometry is invalid or the generated deck
    /// fails validation.
    pub fn to_directivity_deck(
        &self,
        bands: Vec<AcousticBand>,
        exit_diameter_m: f64,
        exit_velocity_m_s: f64,
    ) -> Result<DsmDirectivityDeck, DsmError> {
        require_positive("exit_diameter_m", exit_diameter_m)?;
        require_positive("exit_velocity_m_s", exit_velocity_m_s)?;
        let mut rows = Vec::with_capacity(self.angles_deg.len());
        for angle in &self.angles_deg {
            let mut row = Vec::with_capacity(bands.len());
            for band in &bands {
                let strouhal = band.center_hz * exit_diameter_m / exit_velocity_m_s;
                row.push(self.directivity_db_at(strouhal, *angle)?);
            }
            rows.push(row);
        }
        DsmDirectivityDeck::new(bands, self.angles_deg.clone(), rows)
    }
}

/// Provenance-pinned SP-8072 directivity curve deck.
#[derive(Clone, Debug, PartialEq)]
pub struct Sp8072DirectivityCurveDeck {
    meta: Sp8072DirectivityCurveMeta,
    curve: DsmDirectivityCurve,
    sha256_hex: String,
}

impl Sp8072DirectivityCurveDeck {
    /// Parse a Schema-1 SP-8072 directivity curve deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are incomplete, or the curve matrix is
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: Sp8072DirectivityCurveDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_sp8072_directivity_curve
            != SP8072_DIRECTIVITY_CURVE_SCHEMA_VERSION
        {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_sp8072_directivity_curve,
            });
        }
        Ok(Self {
            meta: Sp8072DirectivityCurveMeta::from_row(parsed.meta)?,
            curve: DsmDirectivityCurve::new(
                parsed.curve.strouhal,
                parsed.curve.angles_deg,
                parsed.curve.rows_db,
            )?,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Curve provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &Sp8072DirectivityCurveMeta {
        &self.meta
    }

    /// Verify this curve deck cites the supplied SP-8072 source manifest.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the deck's manifest dataset id, pinned source
    /// PDF SHA-256, or source figure id does not match the supplied manifest.
    pub fn validate_source_manifest(
        &self,
        manifest: &Sp8072SourceManifest,
    ) -> Result<(), DsmError> {
        validate_sp8072_curve_source_manifest(
            &self.meta.source_manifest_id,
            &self.meta.source_figure_id,
            &self.meta.source_pdf_sha256,
            manifest,
        )
    }

    /// Parsed directivity curve.
    #[must_use]
    pub fn curve(&self) -> &DsmDirectivityCurve {
        &self.curve
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance metadata for a digitized NASA SP-8072 apparent-source curve.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Sp8072ApparentSourceCurveMeta {
    /// Stable dataset id.
    pub dataset_id: String,
    /// Source manifest dataset id.
    pub source_manifest_id: String,
    /// Repository-relative source manifest path.
    pub source_manifest_file: String,
    /// Source figure id from the SP-8072 source manifest.
    pub source_figure_id: String,
    /// SHA-256 digest of the retrieved source PDF.
    pub source_pdf_sha256: String,
    /// Human-readable digitization method statement.
    pub digitization_method: String,
    /// License or access terms.
    pub license_or_terms: String,
    /// Validation label for this digitized curve.
    pub validation: String,
}

impl Sp8072ApparentSourceCurveMeta {
    fn from_row(row: Sp8072ApparentSourceCurveMetaRow) -> Result<Self, DsmError> {
        require_text("meta.dataset_id", &row.dataset_id)?;
        require_text("meta.source_manifest_id", &row.source_manifest_id)?;
        require_text("meta.source_manifest_file", &row.source_manifest_file)?;
        require_text("meta.source_figure_id", &row.source_figure_id)?;
        require_sha256_hex("meta.source_pdf_sha256", &row.source_pdf_sha256)?;
        require_text("meta.digitization_method", &row.digitization_method)?;
        require_text("meta.license_or_terms", &row.license_or_terms)?;
        require_text("meta.validation", &row.validation)?;
        Ok(Self {
            dataset_id: row.dataset_id.trim().to_owned(),
            source_manifest_id: row.source_manifest_id.trim().to_owned(),
            source_manifest_file: row.source_manifest_file.trim().to_owned(),
            source_figure_id: row.source_figure_id.trim().to_owned(),
            source_pdf_sha256: row.source_pdf_sha256.trim().to_owned(),
            digitization_method: row.digitization_method.trim().to_owned(),
            license_or_terms: row.license_or_terms.trim().to_owned(),
            validation: row.validation.trim().to_owned(),
        })
    }
}

/// One digitized point from SP-8072 Figure 14.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmApparentSourcePoint {
    /// Strouhal number `f * d_e / U_e`.
    pub strouhal: f64,
    /// Apparent source axial position normalized by exit diameter.
    pub x_over_exit_diameter: f64,
}

/// Dimensionless apparent-source-location curve.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmApparentSourceCurve {
    points: Vec<DsmApparentSourcePoint>,
}

impl DsmApparentSourceCurve {
    /// Construct a validated apparent-source-location curve.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when fewer than two points are supplied, Strouhal
    /// values are not finite positive and strictly increasing, or normalized
    /// source locations are not finite positive.
    pub fn new(points: Vec<DsmApparentSourcePoint>) -> Result<Self, DsmError> {
        if points.len() < 2 {
            return Err(DsmError::MalformedDeck {
                reason: "apparent source curve requires at least two points".to_owned(),
            });
        }
        let mut previous = None;
        for point in &points {
            require_positive("apparent_source.strouhal", point.strouhal)?;
            require_positive(
                "apparent_source.x_over_exit_diameter",
                point.x_over_exit_diameter,
            )?;
            if previous.is_some_and(|previous| point.strouhal <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "apparent_source.strouhal",
                });
            }
            previous = Some(point.strouhal);
        }
        Ok(Self { points })
    }

    /// Digitized points in source order.
    #[must_use]
    pub fn points(&self) -> &[DsmApparentSourcePoint] {
        &self.points
    }

    /// Interpolate apparent axial source position in log-Strouhal space,
    /// clamping outside the digitized curve domain.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `strouhal` is not finite and positive.
    pub fn x_over_exit_diameter_at(&self, strouhal: f64) -> Result<f64, DsmError> {
        require_positive("strouhal", strouhal)?;
        let axis = self
            .points
            .iter()
            .map(|point| point.strouhal)
            .collect::<Vec<_>>();
        let lower = lower_axis_index_log(&axis, strouhal);
        let upper = (lower + 1).min(axis.len() - 1);
        let fraction = log_axis_fraction(axis[lower], axis[upper], strouhal);
        Ok(interpolate_linear(
            self.points[lower].x_over_exit_diameter,
            self.points[upper].x_over_exit_diameter,
            fraction,
        ))
    }

    /// Convert a Strouhal value into an axial source location in metres.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when exit diameter or Strouhal is invalid.
    pub fn axial_m_at(&self, strouhal: f64, exit_diameter_m: f64) -> Result<f64, DsmError> {
        require_positive("exit_diameter_m", exit_diameter_m)?;
        Ok(self.x_over_exit_diameter_at(strouhal)? * exit_diameter_m)
    }
}

/// Provenance-pinned SP-8072 apparent-source-location curve deck.
#[derive(Clone, Debug, PartialEq)]
pub struct Sp8072ApparentSourceCurveDeck {
    meta: Sp8072ApparentSourceCurveMeta,
    curve: DsmApparentSourceCurve,
    sha256_hex: String,
}

impl Sp8072ApparentSourceCurveDeck {
    /// Parse a Schema-1 SP-8072 apparent-source curve deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are incomplete, or the curve points are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, DsmError> {
        let parsed: Sp8072ApparentSourceCurveDeckFile =
            toml::from_str(input).map_err(|error| DsmError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_sp8072_apparent_source_curve
            != SP8072_APPARENT_SOURCE_CURVE_SCHEMA_VERSION
        {
            return Err(DsmError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_sp8072_apparent_source_curve,
            });
        }
        Ok(Self {
            meta: Sp8072ApparentSourceCurveMeta::from_row(parsed.meta)?,
            curve: apparent_source_curve_from_rows(parsed.points)?,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Curve provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &Sp8072ApparentSourceCurveMeta {
        &self.meta
    }

    /// Verify this curve deck cites the supplied SP-8072 source manifest.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the deck's manifest dataset id, pinned source
    /// PDF SHA-256, or source figure id does not match the supplied manifest.
    pub fn validate_source_manifest(
        &self,
        manifest: &Sp8072SourceManifest,
    ) -> Result<(), DsmError> {
        validate_sp8072_curve_source_manifest(
            &self.meta.source_manifest_id,
            &self.meta.source_figure_id,
            &self.meta.source_pdf_sha256,
            manifest,
        )
    }

    /// Parsed apparent-source curve.
    #[must_use]
    pub fn curve(&self) -> &DsmApparentSourceCurve {
        &self.curve
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Single-engine exit condition used by the free-field DSM slice.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmExitCondition {
    /// Exhaust mass flow in kg/s.
    pub mass_flow_kg_s: f64,
    /// Effective exhaust velocity in m/s.
    pub exit_velocity_m_s: f64,
    /// Exit diameter in m, used for Strouhal scaling.
    pub exit_diameter_m: f64,
    /// Acoustic efficiency `eta`.
    pub acoustic_efficiency: f64,
}

impl DsmExitCondition {
    /// Validate and return this exit condition.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] for non-finite, non-positive, or out-of-range
    /// values.
    pub fn validate(self) -> Result<Self, DsmError> {
        require_positive("mass_flow_kg_s", self.mass_flow_kg_s)?;
        require_positive("exit_velocity_m_s", self.exit_velocity_m_s)?;
        require_positive("exit_diameter_m", self.exit_diameter_m)?;
        require_fraction("acoustic_efficiency", self.acoustic_efficiency)?;
        if self.acoustic_efficiency == 0.0 {
            return Err(DsmError::NonPositive {
                field: "acoustic_efficiency",
            });
        }
        Ok(self)
    }

    /// Acoustic power in watts: `eta * 0.5 * mdot * Ve^2`.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if the exit condition is invalid.
    pub fn acoustic_power_w(self) -> Result<f64, DsmError> {
        self.validate()?;
        Ok(self.acoustic_efficiency
            * 0.5
            * self.mass_flow_kg_s
            * self.exit_velocity_m_s
            * self.exit_velocity_m_s)
    }

    /// Sound power level in dB re 1 pW.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if the exit condition is invalid.
    pub fn sound_power_level_db(self) -> Result<f64, DsmError> {
        Ok(10.0 * (self.acoustic_power_w()? / REFERENCE_ACOUSTIC_POWER_W).log10())
    }
}

/// One source location and acoustic-power fraction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmSource {
    /// Axial coordinate along the plume centerline in m.
    pub axial_m: f64,
    /// Fraction of acoustic power assigned to this source.
    pub power_fraction: f64,
}

/// Source allocation in increasing axial order.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmSourceAllocation {
    sources: Vec<DsmSource>,
}

impl DsmSourceAllocation {
    /// Construct a validated source allocation.
    ///
    /// Fractions are normalized by callers through
    /// [`Self::normalized_from_weights`] when needed; this constructor expects
    /// fractions to sum to one.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] for empty, non-finite, non-increasing, or
    /// non-normalized inputs.
    pub fn new(mut sources: Vec<DsmSource>) -> Result<Self, DsmError> {
        if sources.is_empty() {
            return Err(DsmError::Empty { field: "sources" });
        }
        sources.sort_by(|a, b| a.axial_m.total_cmp(&b.axial_m));
        let mut previous = None;
        let mut total = 0.0;
        for source in &sources {
            require_non_negative("source.axial_m", source.axial_m)?;
            require_fraction("source.power_fraction", source.power_fraction)?;
            if previous.is_some_and(|previous| source.axial_m <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "source.axial_m",
                });
            }
            previous = Some(source.axial_m);
            total += source.power_fraction;
        }
        if (total - 1.0).abs() > 1.0e-12 {
            return Err(DsmError::InvalidFraction {
                field: "source.power_fraction_sum",
            });
        }
        Ok(Self { sources })
    }

    /// Construct a source allocation from non-negative weights.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] for invalid source positions or all-zero weights.
    pub fn normalized_from_weights(sources: &[(f64, f64)]) -> Result<Self, DsmError> {
        if sources.is_empty() {
            return Err(DsmError::Empty { field: "sources" });
        }
        let mut total = 0.0;
        for (_, weight) in sources {
            require_non_negative("source.weight", *weight)?;
            total += *weight;
        }
        require_positive("source.weight_sum", total)?;
        Self::new(
            sources
                .iter()
                .map(|(axial_m, weight)| DsmSource {
                    axial_m: *axial_m,
                    power_fraction: *weight / total,
                })
                .collect(),
        )
    }

    /// Sources in deterministic order.
    #[must_use]
    pub fn sources(&self) -> &[DsmSource] {
        &self.sources
    }
}

/// Receiver station in plume-axis coordinates.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmReceiver {
    /// Stable station id.
    pub station_id: String,
    /// Axial coordinate in m.
    pub axial_m: f64,
    /// Radial offset from the plume centerline in m.
    pub radial_m: f64,
}

impl DsmReceiver {
    /// Construct a validated receiver.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if text or geometry inputs are invalid.
    pub fn new(
        station_id: impl Into<String>,
        axial_m: f64,
        radial_m: f64,
    ) -> Result<Self, DsmError> {
        let station_id = station_id.into();
        if station_id.trim().is_empty() {
            return Err(DsmError::Empty {
                field: "station_id",
            });
        }
        require_non_negative("receiver.axial_m", axial_m)?;
        require_positive("receiver.radial_m", radial_m)?;
        Ok(Self {
            station_id: station_id.trim().to_owned(),
            axial_m,
            radial_m,
        })
    }

    fn validate(&self) -> Result<(), DsmError> {
        if self.station_id.trim().is_empty() {
            return Err(DsmError::Empty {
                field: "station_id",
            });
        }
        require_non_negative("receiver.axial_m", self.axial_m)?;
        require_positive("receiver.radial_m", self.radial_m)
    }

    fn distance_to_source_m(&self, source: DsmSource) -> f64 {
        let dx = self.axial_m - source.axial_m;
        (dx * dx + self.radial_m * self.radial_m).sqrt()
    }

    fn angle_from_plume_axis_deg(&self, source: DsmSource) -> f64 {
        let dx = (self.axial_m - source.axial_m).abs();
        self.radial_m.atan2(dx).to_degrees()
    }
}

/// Deterministic single-bend plume-axis approximation for pad deflectors.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmPlumeDeflection {
    /// Axial coordinate where the plume axis begins bending, metres.
    pub origin_axial_m: f64,
    /// Bend angle from the original plume axis toward positive receiver
    /// radial direction, degrees.
    pub angle_deg: f64,
}

impl DsmPlumeDeflection {
    /// Construct a validated plume-axis deflection.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the origin is negative or the angle is outside
    /// `[0, 90]` degrees.
    pub fn new(origin_axial_m: f64, angle_deg: f64) -> Result<Self, DsmError> {
        let deflection = Self {
            origin_axial_m,
            angle_deg,
        };
        deflection.validate()?;
        Ok(deflection)
    }

    fn validate(self) -> Result<(), DsmError> {
        require_non_negative("plume_deflection.origin_axial_m", self.origin_axial_m)?;
        require_finite("plume_deflection.angle_deg", self.angle_deg)?;
        if !(0.0..=90.0).contains(&self.angle_deg) {
            return Err(DsmError::OutOfRange {
                field: "plume_deflection.angle_deg",
            });
        }
        Ok(())
    }

    fn source_point(self, source: DsmSource) -> DsmSourcePoint {
        if source.axial_m <= self.origin_axial_m {
            return DsmSourcePoint {
                axial_m: source.axial_m,
                radial_m: 0.0,
                tangent_angle_rad: 0.0,
            };
        }
        let downstream_m = source.axial_m - self.origin_axial_m;
        let angle_rad = self.angle_deg.to_radians();
        DsmSourcePoint {
            axial_m: self.origin_axial_m + downstream_m * angle_rad.cos(),
            radial_m: downstream_m * angle_rad.sin(),
            tangent_angle_rad: angle_rad,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq)]
struct DsmSourcePoint {
    axial_m: f64,
    radial_m: f64,
    tangent_angle_rad: f64,
}

fn deflected_source_metrics(
    receiver: &DsmReceiver,
    source: DsmSource,
    deflection: DsmPlumeDeflection,
) -> Result<(f64, f64), DsmError> {
    let point = deflection.source_point(source);
    let dx = receiver.axial_m - point.axial_m;
    let dy = receiver.radial_m - point.radial_m;
    let distance_m = dx.hypot(dy);
    require_positive("source_receiver_distance_m", distance_m)?;
    let tangent_x = point.tangent_angle_rad.cos();
    let tangent_y = point.tangent_angle_rad.sin();
    let cos_angle = ((dx * tangent_x + dy * tangent_y) / distance_m).clamp(-1.0, 1.0);
    Ok((distance_m, cos_angle.acos().to_degrees()))
}

/// Normalized per-band acoustic power fractions.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmSpectrumShape {
    bands: Vec<AcousticBand>,
    band_power_fractions: Vec<f64>,
}

impl DsmSpectrumShape {
    /// Construct a validated normalized spectrum shape.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] for invalid grids, lengths, or fractions.
    pub fn new(bands: Vec<AcousticBand>, band_power_fractions: Vec<f64>) -> Result<Self, DsmError> {
        validate_band_values(&bands, band_power_fractions.len())?;
        let mut total = 0.0;
        for fraction in &band_power_fractions {
            require_fraction("band_power_fraction", *fraction)?;
            total += *fraction;
        }
        if (total - 1.0).abs() > 1.0e-12 {
            return Err(DsmError::InvalidFraction {
                field: "band_power_fraction_sum",
            });
        }
        Ok(Self {
            bands,
            band_power_fractions,
        })
    }

    /// Build a deterministic Strouhal-shaped spectrum over the standard band
    /// grid.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when inputs are invalid.
    pub fn strouhal_gaussian(
        exit_diameter_m: f64,
        exit_velocity_m_s: f64,
        peak_strouhal: f64,
        width_decades: f64,
    ) -> Result<Self, DsmError> {
        require_positive("exit_diameter_m", exit_diameter_m)?;
        require_positive("exit_velocity_m_s", exit_velocity_m_s)?;
        require_positive("peak_strouhal", peak_strouhal)?;
        require_positive("width_decades", width_decades)?;
        let bands = standard_third_octave_bands();
        let mut weights = Vec::with_capacity(bands.len());
        for band in &bands {
            let strouhal = band.center_hz * exit_diameter_m / exit_velocity_m_s;
            let distance = (strouhal / peak_strouhal).log10();
            weights.push((-0.5 * (distance / width_decades).powi(2)).exp());
        }
        normalize_band_weights(bands, weights)
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Normalized band power fractions.
    #[must_use]
    pub fn band_power_fractions(&self) -> &[f64] {
        &self.band_power_fractions
    }
}

/// Normalized source-by-band acoustic power fractions.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmSourceBandWeights {
    bands: Vec<AcousticBand>,
    source_band_power_fractions: Vec<Vec<f64>>,
}

impl DsmSourceBandWeights {
    /// Construct normalized source-by-band power fractions from non-negative
    /// weights.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the band grid or matrix dimensions are invalid,
    /// any weight is negative/non-finite, or the total weight is zero.
    pub fn normalized_from_weights(
        bands: Vec<AcousticBand>,
        source_band_weights: Vec<Vec<f64>>,
    ) -> Result<Self, DsmError> {
        if source_band_weights.is_empty() {
            return Err(DsmError::Empty {
                field: "source_band_weights",
            });
        }
        validate_band_values(&bands, bands.len())?;
        let mut total = 0.0;
        for row in &source_band_weights {
            if row.len() != bands.len() {
                return Err(DsmError::LengthMismatch {
                    bands: bands.len(),
                    values: row.len(),
                });
            }
            for weight in row {
                require_non_negative("source_band_weight", *weight)?;
                total += *weight;
            }
        }
        require_positive("source_band_weight_sum", total)?;
        Ok(Self {
            bands,
            source_band_power_fractions: source_band_weights
                .into_iter()
                .map(|row| row.into_iter().map(|weight| weight / total).collect())
                .collect(),
        })
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Number of source rows.
    #[must_use]
    pub fn source_count(&self) -> usize {
        self.source_band_power_fractions.len()
    }

    /// Normalized source-by-band fractions.
    #[must_use]
    pub fn source_band_power_fractions(&self) -> &[Vec<f64>] {
        &self.source_band_power_fractions
    }
}

/// Angle-indexed directivity deck, one value per band at each angle.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmDirectivityDeck {
    bands: Vec<AcousticBand>,
    angles_deg: Vec<f64>,
    directivity_db: Vec<Vec<f64>>,
}

impl DsmDirectivityDeck {
    /// Construct a validated directivity deck.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the deck is empty, not increasing, or has
    /// mismatched dimensions.
    pub fn new(
        bands: Vec<AcousticBand>,
        angles_deg: Vec<f64>,
        directivity_db: Vec<Vec<f64>>,
    ) -> Result<Self, DsmError> {
        validate_band_values(&bands, bands.len())?;
        if angles_deg.is_empty() {
            return Err(DsmError::Empty {
                field: "angles_deg",
            });
        }
        if angles_deg.len() != directivity_db.len() {
            return Err(DsmError::LengthMismatch {
                bands: angles_deg.len(),
                values: directivity_db.len(),
            });
        }
        let mut previous = None;
        for angle in &angles_deg {
            require_non_negative("angle_deg", *angle)?;
            if previous.is_some_and(|previous| *angle <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "angles_deg",
                });
            }
            previous = Some(*angle);
        }
        for row in &directivity_db {
            if row.len() != bands.len() {
                return Err(DsmError::LengthMismatch {
                    bands: bands.len(),
                    values: row.len(),
                });
            }
            for value in row {
                require_finite("directivity_db", *value)?;
            }
        }
        Ok(Self {
            bands,
            angles_deg,
            directivity_db,
        })
    }

    /// Zero-directivity deck over the given band grid.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if the band grid is invalid.
    pub fn omnidirectional(bands: Vec<AcousticBand>) -> Result<Self, DsmError> {
        Self::new(
            bands.clone(),
            vec![0.0, 180.0],
            vec![vec![0.0; bands.len()]; 2],
        )
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Directivity sample angles in degrees.
    #[must_use]
    pub fn angles_deg(&self) -> &[f64] {
        &self.angles_deg
    }

    /// Interpolate directivity in dB for an angle.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `angle_deg` is non-finite.
    pub fn directivity_db_at(&self, angle_deg: f64) -> Result<Vec<f64>, DsmError> {
        require_finite("angle_deg", angle_deg)?;
        if angle_deg <= self.angles_deg[0] {
            return Ok(self.directivity_db[0].clone());
        }
        let last = self.angles_deg.len() - 1;
        if angle_deg >= self.angles_deg[last] {
            return Ok(self.directivity_db[last].clone());
        }
        let upper = self
            .angles_deg
            .iter()
            .position(|candidate| *candidate >= angle_deg)
            .ok_or(DsmError::NonIncreasing {
                field: "angles_deg",
            })?;
        let lower = upper - 1;
        let span = self.angles_deg[upper] - self.angles_deg[lower];
        let fraction = (angle_deg - self.angles_deg[lower]) / span;
        Ok(self.directivity_db[lower]
            .iter()
            .zip(self.directivity_db[upper].iter())
            .map(|(lhs, rhs)| lhs * (1.0 - fraction) + rhs * fraction)
            .collect())
    }

    /// Apply a per-band dB transfer to every directivity angle row.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the transfer length does not match the band
    /// grid or any value is non-finite.
    pub fn with_band_transfer_db(&self, transfer_db: &[f64]) -> Result<Self, DsmError> {
        if transfer_db.len() != self.bands.len() {
            return Err(DsmError::LengthMismatch {
                bands: self.bands.len(),
                values: transfer_db.len(),
            });
        }
        for value in transfer_db {
            require_finite("transfer_db", *value)?;
        }
        let rows = self
            .directivity_db
            .iter()
            .map(|row| {
                row.iter()
                    .zip(transfer_db.iter())
                    .map(|(directivity, transfer)| directivity + transfer)
                    .collect()
            })
            .collect();
        Self::new(self.bands.clone(), self.angles_deg.clone(), rows)
    }
}

/// Per-band air absorption loss coefficients.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmAbsorptionDeck {
    bands: Vec<AcousticBand>,
    db_per_m: Vec<f64>,
}

impl DsmAbsorptionDeck {
    /// Construct a validated per-band absorption deck.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if the band grid is invalid, the value count does
    /// not match the grid, or any value is negative/non-finite.
    pub fn new(bands: Vec<AcousticBand>, db_per_m: Vec<f64>) -> Result<Self, DsmError> {
        validate_band_values(&bands, db_per_m.len())?;
        for value in &db_per_m {
            require_non_negative("absorption.db_per_m", *value)?;
        }
        Ok(Self { bands, db_per_m })
    }

    /// Build a uniform absorption deck over a caller-supplied band grid.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the band grid is invalid or the scalar
    /// coefficient is negative/non-finite.
    pub fn uniform(bands: Vec<AcousticBand>, db_per_m: f64) -> Result<Self, DsmError> {
        require_non_negative("absorption_db_per_m", db_per_m)?;
        let value_count = bands.len();
        Self::new(bands, vec![db_per_m; value_count])
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Per-band loss coefficient values in dB/m.
    #[must_use]
    pub fn db_per_m(&self) -> &[f64] {
        &self.db_per_m
    }

    fn loss_db(&self, band_index: usize, distance_m: f64) -> f64 {
        self.db_per_m[band_index] * distance_m
    }
}

/// Propagation environment for the free-field DSM slice.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmPropagation {
    /// Ambient acoustic impedance `rho*c` in Pa*s/m.
    pub ambient_impedance_pa_s_m: f64,
    /// Per-band air absorption loss coefficients.
    pub absorption: DsmAbsorptionDeck,
    /// Extra propagation path length for pad duct/trench/deflector geometry, metres.
    pub path_length_extra_m: f64,
}

impl DsmPropagation {
    /// Construct propagation with a uniform absorption scalar over a band grid.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if the impedance, absorption scalar, or band grid
    /// is invalid.
    pub fn uniform(
        ambient_impedance_pa_s_m: f64,
        absorption_db_per_m: f64,
        bands: Vec<AcousticBand>,
    ) -> Result<Self, DsmError> {
        require_positive("ambient_impedance_pa_s_m", ambient_impedance_pa_s_m)?;
        Ok(Self {
            ambient_impedance_pa_s_m,
            absorption: DsmAbsorptionDeck::uniform(bands, absorption_db_per_m)?,
            path_length_extra_m: 0.0,
        })
    }

    /// Return a copy with an additional propagation path length, metres.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when `path_length_extra_m` is negative or non-finite.
    pub fn with_path_length_extra_m(mut self, path_length_extra_m: f64) -> Result<Self, DsmError> {
        require_non_negative("path_length_extra_m", path_length_extra_m)?;
        self.path_length_extra_m = path_length_extra_m;
        Ok(self)
    }

    /// Validate propagation inputs.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if inputs are invalid.
    pub fn validate(&self) -> Result<(), DsmError> {
        require_positive("ambient_impedance_pa_s_m", self.ambient_impedance_pa_s_m)?;
        require_non_negative("path_length_extra_m", self.path_length_extra_m)?;
        validate_band_values(&self.absorption.bands, self.absorption.db_per_m.len())?;
        for value in &self.absorption.db_per_m {
            require_non_negative("absorption.db_per_m", *value)?;
        }
        Ok(())
    }

    fn propagation_distance_m(&self, geometric_distance_m: f64) -> f64 {
        geometric_distance_m + self.path_length_extra_m
    }
}

/// DSM station result.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmStationSpectrum {
    /// Station id.
    pub station_id: String,
    /// Evaluated SPL spectrum.
    pub spectrum: SplSpectrum,
    /// Source count included in the evaluation.
    pub source_count: usize,
}

impl DsmStationSpectrum {
    /// Construct a station spectrum from an already evaluated SPL spectrum.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the station id is empty or the source count is
    /// zero.
    pub fn new(
        station_id: impl Into<String>,
        spectrum: SplSpectrum,
        source_count: usize,
    ) -> Result<Self, DsmError> {
        let station_id = station_id.into();
        if station_id.trim().is_empty() {
            return Err(DsmError::Empty {
                field: "station_id",
            });
        }
        if source_count == 0 {
            return Err(DsmError::Empty {
                field: "source_count",
            });
        }
        Ok(Self {
            station_id: station_id.trim().to_owned(),
            spectrum,
            source_count,
        })
    }

    /// Render band levels as deterministic CSV.
    #[must_use]
    pub fn to_csv(&self) -> String {
        let mut out = String::from("station_id,center_hz,spl_db\n");
        for (band, level) in self.spectrum.bands().iter().zip(self.spectrum.levels()) {
            let _ = writeln!(
                out,
                "{},{:.17e},{:.17e}",
                self.station_id,
                band.center_hz,
                level.db_re_20upa()
            );
        }
        out
    }
}

/// One band row in a DSM worked-example spectrum tolerance report.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmSpectrumToleranceRow {
    /// One-third-octave center frequency, Hz.
    pub center_hz: f64,
    /// Evaluated band SPL, dB re 20 uPa.
    pub evaluated_spl_db: f64,
    /// Reference band SPL, dB re 20 uPa.
    pub reference_spl_db: f64,
    /// Evaluated-minus-reference residual, dB.
    pub residual_db: f64,
    /// Absolute residual tolerance for this band, dB.
    pub tolerance_db: f64,
    /// Whether the absolute residual is within tolerance.
    pub within_tolerance: bool,
}

/// DSM spectrum tolerance report for a worked-example comparison.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmSpectrumToleranceReport {
    /// Evaluated OASPL, dB re 20 uPa.
    pub oaspl_evaluated_db: f64,
    /// Reference OASPL, dB re 20 uPa.
    pub oaspl_reference_db: f64,
    /// Evaluated-minus-reference OASPL residual, dB.
    pub oaspl_residual_db: f64,
    /// Absolute OASPL residual tolerance, dB.
    pub oaspl_tolerance_db: f64,
    /// Whether the absolute OASPL residual is within tolerance.
    pub oaspl_within_tolerance: bool,
    /// Per-band absolute SPL residual tolerance, dB.
    pub band_tolerance_db: f64,
    rows: Vec<DsmSpectrumToleranceRow>,
}

impl DsmSpectrumToleranceReport {
    /// Band rows in deterministic band-grid order.
    #[must_use]
    pub fn rows(&self) -> &[DsmSpectrumToleranceRow] {
        &self.rows
    }

    /// Whether OASPL and every band row are within tolerance.
    #[must_use]
    pub fn within_tolerance(&self) -> bool {
        self.oaspl_within_tolerance && self.rows.iter().all(|row| row.within_tolerance)
    }

    /// Render the tolerance report as deterministic CSV.
    #[must_use]
    pub fn to_csv(&self) -> String {
        let mut out = String::from(
            "metric,center_hz,evaluated_spl_db,reference_spl_db,residual_db,tolerance_db,within_tolerance\n",
        );
        let _ = writeln!(
            out,
            "oaspl,all,{:.17e},{:.17e},{:.17e},{:.17e},{}",
            self.oaspl_evaluated_db,
            self.oaspl_reference_db,
            self.oaspl_residual_db,
            self.oaspl_tolerance_db,
            self.oaspl_within_tolerance
        );
        for row in &self.rows {
            let _ = writeln!(
                out,
                "band,{:.17e},{:.17e},{:.17e},{:.17e},{:.17e},{}",
                row.center_hz,
                row.evaluated_spl_db,
                row.reference_spl_db,
                row.residual_db,
                row.tolerance_db,
                row.within_tolerance
            );
        }
        out
    }
}

/// Compare an evaluated DSM spectrum with a reference worked-example spectrum.
///
/// # Errors
///
/// Returns [`DsmError`] when tolerances are negative or non-finite, the spectra
/// use different band grids, or either OASPL cannot be computed.
pub fn dsm_spectrum_tolerance_report(
    evaluated: &SplSpectrum,
    reference: &SplSpectrum,
    oaspl_tolerance_db: f64,
    band_tolerance_db: f64,
) -> Result<DsmSpectrumToleranceReport, DsmError> {
    require_non_negative("oaspl_tolerance_db", oaspl_tolerance_db)?;
    require_non_negative("band_tolerance_db", band_tolerance_db)?;
    if !same_bands(evaluated.bands(), reference.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }

    let rows = evaluated
        .bands()
        .iter()
        .zip(evaluated.levels().iter().zip(reference.levels()))
        .map(|(band, (evaluated, reference))| {
            let evaluated_spl_db = evaluated.db_re_20upa();
            let reference_spl_db = reference.db_re_20upa();
            let residual_db = evaluated_spl_db - reference_spl_db;
            DsmSpectrumToleranceRow {
                center_hz: band.center_hz,
                evaluated_spl_db,
                reference_spl_db,
                residual_db,
                tolerance_db: band_tolerance_db,
                within_tolerance: residual_db.abs() <= band_tolerance_db,
            }
        })
        .collect::<Vec<_>>();
    let oaspl_evaluated_db = evaluated.oaspl()?.db_re_20upa();
    let oaspl_reference_db = reference.oaspl()?.db_re_20upa();
    let oaspl_residual_db = oaspl_evaluated_db - oaspl_reference_db;
    Ok(DsmSpectrumToleranceReport {
        oaspl_evaluated_db,
        oaspl_reference_db,
        oaspl_residual_db,
        oaspl_tolerance_db,
        oaspl_within_tolerance: oaspl_residual_db.abs() <= oaspl_tolerance_db,
        band_tolerance_db,
        rows,
    })
}

/// One vehicle-height sample for a height-stepped liftoff DSM trace.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct DsmLiftoffStep {
    /// Sample time in seconds.
    pub time_s: f64,
    /// Vehicle station axial offset from the initial station in m.
    pub vehicle_height_m: f64,
}

impl DsmLiftoffStep {
    /// Construct a validated liftoff sample.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when time or height is non-finite or negative.
    pub fn new(time_s: f64, vehicle_height_m: f64) -> Result<Self, DsmError> {
        require_non_negative("liftoff_step.time_s", time_s)?;
        require_non_negative("liftoff_step.vehicle_height_m", vehicle_height_m)?;
        Ok(Self {
            time_s,
            vehicle_height_m,
        })
    }

    fn validate(self) -> Result<Self, DsmError> {
        Self::new(self.time_s, self.vehicle_height_m)
    }
}

/// One evaluated height-step point for a station.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmLiftoffTracePoint {
    /// Height/time sample.
    pub step: DsmLiftoffStep,
    /// Station spectrum at this sample.
    pub station: DsmStationSpectrum,
}

/// Height-stepped free-field DSM trace for one station.
#[derive(Clone, Debug, PartialEq)]
pub struct DsmLiftoffTrace {
    /// Stable station id.
    pub station_id: String,
    points: Vec<DsmLiftoffTracePoint>,
}

impl DsmLiftoffTrace {
    /// Construct a height-stepped trace from evaluated points.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] when the station id is empty, points are empty, or
    /// point times are not strictly increasing.
    pub fn new(
        station_id: impl Into<String>,
        points: Vec<DsmLiftoffTracePoint>,
    ) -> Result<Self, DsmError> {
        let station_id = station_id.into();
        if station_id.trim().is_empty() {
            return Err(DsmError::Empty {
                field: "station_id",
            });
        }
        if points.is_empty() {
            return Err(DsmError::Empty {
                field: "liftoff_steps",
            });
        }
        let mut previous_time = None;
        for point in &points {
            if previous_time.is_some_and(|previous| point.step.time_s <= previous) {
                return Err(DsmError::NonIncreasing {
                    field: "liftoff_step.time_s",
                });
            }
            previous_time = Some(point.step.time_s);
        }
        Ok(Self {
            station_id: station_id.trim().to_owned(),
            points,
        })
    }

    /// Evaluated trace points in caller step order.
    #[must_use]
    pub fn points(&self) -> &[DsmLiftoffTracePoint] {
        &self.points
    }

    /// Render a compact deterministic OASPL-vs-height CSV.
    ///
    /// # Errors
    ///
    /// Returns [`DsmError`] if any point's OASPL cannot be computed.
    pub fn to_oaspl_csv(&self) -> Result<String, DsmError> {
        let mut out = String::from("station_id,time_s,vehicle_height_m,oaspl_db\n");
        for point in &self.points {
            let _ = writeln!(
                out,
                "{},{:.17e},{:.17e},{:.17e}",
                self.station_id,
                point.step.time_s,
                point.step.vehicle_height_m,
                point.station.spectrum.oaspl()?.db_re_20upa()
            );
        }
        Ok(out)
    }
}

/// Evaluate the free-field single-engine DSM slice at one receiver.
///
/// # Errors
///
/// Returns [`DsmError`] if inputs are invalid or band grids are inconsistent.
pub fn evaluate_free_field_station(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    shape: &DsmSpectrumShape,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
) -> Result<DsmStationSpectrum, DsmError> {
    exit.validate()?;
    receiver.validate()?;
    propagation.validate()?;
    if !same_bands(shape.bands(), &directivity.bands) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    if !same_bands(shape.bands(), propagation.absorption.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    let acoustic_power_w = exit.acoustic_power_w()?;
    let mut band_pressure_pa2 = vec![0.0; shape.bands.len()];
    for source in allocation.sources() {
        let distance_m = receiver.distance_to_source_m(*source);
        let propagation_distance_m = propagation.propagation_distance_m(distance_m);
        let angle_deg = receiver.angle_from_plume_axis_deg(*source);
        let directivity_db = directivity.directivity_db_at(angle_deg)?;
        for (band_index, band_fraction) in shape.band_power_fractions.iter().enumerate() {
            let source_band_power_w = acoustic_power_w * source.power_fraction * band_fraction;
            let spreading = source_band_power_w * propagation.ambient_impedance_pa_s_m
                / (4.0 * PI * propagation_distance_m * propagation_distance_m);
            let loss_db = propagation
                .absorption
                .loss_db(band_index, propagation_distance_m);
            let gain = 10.0_f64.powf((directivity_db[band_index] - loss_db) / 10.0);
            band_pressure_pa2[band_index] += spreading * gain;
        }
    }
    let levels = band_pressure_pa2
        .into_iter()
        .map(Spl::from_mean_square_pressure_pa2)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(DsmStationSpectrum {
        station_id: receiver.station_id.clone(),
        spectrum: SplSpectrum::new(shape.bands.clone(), levels)?,
        source_count: allocation.sources().len(),
    })
}

/// Evaluate a DSM station with source coordinates mapped through a declared
/// single-bend plume deflection.
///
/// # Errors
///
/// Returns [`DsmError`] if inputs are invalid, band grids are inconsistent, or
/// a deflected source lands exactly on the receiver.
pub fn evaluate_deflected_free_field_station(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    shape: &DsmSpectrumShape,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    deflection: DsmPlumeDeflection,
) -> Result<DsmStationSpectrum, DsmError> {
    exit.validate()?;
    receiver.validate()?;
    propagation.validate()?;
    deflection.validate()?;
    if !same_bands(shape.bands(), &directivity.bands) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    if !same_bands(shape.bands(), propagation.absorption.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    let acoustic_power_w = exit.acoustic_power_w()?;
    let mut band_pressure_pa2 = vec![0.0; shape.bands.len()];
    for source in allocation.sources() {
        let (distance_m, angle_deg) = deflected_source_metrics(receiver, *source, deflection)?;
        let propagation_distance_m = propagation.propagation_distance_m(distance_m);
        let directivity_db = directivity.directivity_db_at(angle_deg)?;
        for (band_index, band_fraction) in shape.band_power_fractions.iter().enumerate() {
            let source_band_power_w = acoustic_power_w * source.power_fraction * band_fraction;
            let spreading = source_band_power_w * propagation.ambient_impedance_pa_s_m
                / (4.0 * PI * propagation_distance_m * propagation_distance_m);
            let loss_db = propagation
                .absorption
                .loss_db(band_index, propagation_distance_m);
            let gain = 10.0_f64.powf((directivity_db[band_index] - loss_db) / 10.0);
            band_pressure_pa2[band_index] += spreading * gain;
        }
    }
    let levels = band_pressure_pa2
        .into_iter()
        .map(Spl::from_mean_square_pressure_pa2)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(DsmStationSpectrum {
        station_id: receiver.station_id.clone(),
        spectrum: SplSpectrum::new(shape.bands.clone(), levels)?,
        source_count: allocation.sources().len(),
    })
}

/// Evaluate a distributed source-line DSM slice with source-position-dependent
/// band weights.
///
/// # Errors
///
/// Returns [`DsmError`] if inputs are invalid, source counts differ, or band
/// grids are inconsistent.
pub fn evaluate_source_band_weighted_station(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    source_band_weights: &DsmSourceBandWeights,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
) -> Result<DsmStationSpectrum, DsmError> {
    exit.validate()?;
    receiver.validate()?;
    propagation.validate()?;
    if allocation.sources().len() != source_band_weights.source_count() {
        return Err(DsmError::LengthMismatch {
            bands: allocation.sources().len(),
            values: source_band_weights.source_count(),
        });
    }
    if !same_bands(source_band_weights.bands(), &directivity.bands) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    if !same_bands(source_band_weights.bands(), propagation.absorption.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    let acoustic_power_w = exit.acoustic_power_w()?;
    let mut band_pressure_pa2 = vec![0.0; source_band_weights.bands.len()];
    for (source, band_fractions) in allocation
        .sources()
        .iter()
        .zip(source_band_weights.source_band_power_fractions())
    {
        let distance_m = receiver.distance_to_source_m(*source);
        let propagation_distance_m = propagation.propagation_distance_m(distance_m);
        let angle_deg = receiver.angle_from_plume_axis_deg(*source);
        let directivity_db = directivity.directivity_db_at(angle_deg)?;
        for (band_index, source_band_fraction) in band_fractions.iter().enumerate() {
            let source_band_power_w = acoustic_power_w * source_band_fraction;
            let spreading = source_band_power_w * propagation.ambient_impedance_pa_s_m
                / (4.0 * PI * propagation_distance_m * propagation_distance_m);
            let loss_db = propagation
                .absorption
                .loss_db(band_index, propagation_distance_m);
            let gain = 10.0_f64.powf((directivity_db[band_index] - loss_db) / 10.0);
            band_pressure_pa2[band_index] += spreading * gain;
        }
    }
    let levels = band_pressure_pa2
        .into_iter()
        .map(Spl::from_mean_square_pressure_pa2)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(DsmStationSpectrum {
        station_id: receiver.station_id.clone(),
        spectrum: SplSpectrum::new(source_band_weights.bands.clone(), levels)?,
        source_count: allocation.sources().len(),
    })
}

/// Evaluate a source-band-weighted DSM station through a declared single-bend
/// plume deflection.
///
/// # Errors
///
/// Returns [`DsmError`] if inputs are invalid, source counts differ, band grids
/// are inconsistent, or a deflected source lands exactly on the receiver.
pub fn evaluate_deflected_source_band_weighted_station(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    source_band_weights: &DsmSourceBandWeights,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    deflection: DsmPlumeDeflection,
) -> Result<DsmStationSpectrum, DsmError> {
    exit.validate()?;
    receiver.validate()?;
    propagation.validate()?;
    deflection.validate()?;
    if allocation.sources().len() != source_band_weights.source_count() {
        return Err(DsmError::LengthMismatch {
            bands: allocation.sources().len(),
            values: source_band_weights.source_count(),
        });
    }
    if !same_bands(source_band_weights.bands(), &directivity.bands) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    if !same_bands(source_band_weights.bands(), propagation.absorption.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    let acoustic_power_w = exit.acoustic_power_w()?;
    let mut band_pressure_pa2 = vec![0.0; source_band_weights.bands.len()];
    for (source, band_fractions) in allocation
        .sources()
        .iter()
        .zip(source_band_weights.source_band_power_fractions())
    {
        let (distance_m, angle_deg) = deflected_source_metrics(receiver, *source, deflection)?;
        let propagation_distance_m = propagation.propagation_distance_m(distance_m);
        let directivity_db = directivity.directivity_db_at(angle_deg)?;
        for (band_index, source_band_fraction) in band_fractions.iter().enumerate() {
            let source_band_power_w = acoustic_power_w * source_band_fraction;
            let spreading = source_band_power_w * propagation.ambient_impedance_pa_s_m
                / (4.0 * PI * propagation_distance_m * propagation_distance_m);
            let loss_db = propagation
                .absorption
                .loss_db(band_index, propagation_distance_m);
            let gain = 10.0_f64.powf((directivity_db[band_index] - loss_db) / 10.0);
            band_pressure_pa2[band_index] += spreading * gain;
        }
    }
    let levels = band_pressure_pa2
        .into_iter()
        .map(Spl::from_mean_square_pressure_pa2)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(DsmStationSpectrum {
        station_id: receiver.station_id.clone(),
        spectrum: SplSpectrum::new(source_band_weights.bands.clone(), levels)?,
        source_count: allocation.sources().len(),
    })
}

/// Evaluate the SP-8072 Figure 14 apparent-source free-field slice.
///
/// This alternate single-engine path places one effective source location per
/// frequency band from a Strouhal-indexed apparent-source curve, then applies
/// the same spreading, directivity, and absorption math as the distributed
/// source-line evaluator.
///
/// # Errors
///
/// Returns [`DsmError`] if inputs are invalid or band grids are inconsistent.
pub fn evaluate_apparent_source_station(
    exit: DsmExitCondition,
    shape: &DsmSpectrumShape,
    apparent_sources: &DsmApparentSourceCurve,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
) -> Result<DsmStationSpectrum, DsmError> {
    exit.validate()?;
    receiver.validate()?;
    propagation.validate()?;
    if !same_bands(shape.bands(), &directivity.bands) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    if !same_bands(shape.bands(), propagation.absorption.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    let acoustic_power_w = exit.acoustic_power_w()?;
    let mut band_pressure_pa2 = vec![0.0; shape.bands.len()];
    for (band_index, (band, band_fraction)) in shape
        .bands
        .iter()
        .zip(shape.band_power_fractions.iter())
        .enumerate()
    {
        let strouhal = band.center_hz * exit.exit_diameter_m / exit.exit_velocity_m_s;
        let source = DsmSource {
            axial_m: apparent_sources.axial_m_at(strouhal, exit.exit_diameter_m)?,
            power_fraction: 1.0,
        };
        let distance_m = receiver.distance_to_source_m(source);
        let propagation_distance_m = propagation.propagation_distance_m(distance_m);
        let angle_deg = receiver.angle_from_plume_axis_deg(source);
        let directivity_db = directivity.directivity_db_at(angle_deg)?;
        let source_band_power_w = acoustic_power_w * band_fraction;
        let spreading = source_band_power_w * propagation.ambient_impedance_pa_s_m
            / (4.0 * PI * propagation_distance_m * propagation_distance_m);
        let loss_db = propagation
            .absorption
            .loss_db(band_index, propagation_distance_m);
        let gain = 10.0_f64.powf((directivity_db[band_index] - loss_db) / 10.0);
        band_pressure_pa2[band_index] += spreading * gain;
    }
    let levels = band_pressure_pa2
        .into_iter()
        .map(Spl::from_mean_square_pressure_pa2)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(DsmStationSpectrum {
        station_id: receiver.station_id.clone(),
        spectrum: SplSpectrum::new(shape.bands.clone(), levels)?,
        source_count: shape.bands.len(),
    })
}

/// Evaluate the SP-8072 Figure 14 apparent-source slice through a declared
/// single-bend plume deflection.
///
/// # Errors
///
/// Returns [`DsmError`] if inputs are invalid, band grids are inconsistent, or
/// a deflected apparent source lands exactly on the receiver.
pub fn evaluate_deflected_apparent_source_station(
    exit: DsmExitCondition,
    shape: &DsmSpectrumShape,
    apparent_sources: &DsmApparentSourceCurve,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    deflection: DsmPlumeDeflection,
) -> Result<DsmStationSpectrum, DsmError> {
    exit.validate()?;
    receiver.validate()?;
    propagation.validate()?;
    deflection.validate()?;
    if !same_bands(shape.bands(), &directivity.bands) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    if !same_bands(shape.bands(), propagation.absorption.bands()) {
        return Err(DsmError::Acoustics(AcousticsError::BandGridMismatch));
    }
    let acoustic_power_w = exit.acoustic_power_w()?;
    let mut band_pressure_pa2 = vec![0.0; shape.bands.len()];
    for (band_index, (band, band_fraction)) in shape
        .bands
        .iter()
        .zip(shape.band_power_fractions.iter())
        .enumerate()
    {
        let strouhal = band.center_hz * exit.exit_diameter_m / exit.exit_velocity_m_s;
        let source = DsmSource {
            axial_m: apparent_sources.axial_m_at(strouhal, exit.exit_diameter_m)?,
            power_fraction: 1.0,
        };
        let (distance_m, angle_deg) = deflected_source_metrics(receiver, source, deflection)?;
        let propagation_distance_m = propagation.propagation_distance_m(distance_m);
        let directivity_db = directivity.directivity_db_at(angle_deg)?;
        let source_band_power_w = acoustic_power_w * band_fraction;
        let spreading = source_band_power_w * propagation.ambient_impedance_pa_s_m
            / (4.0 * PI * propagation_distance_m * propagation_distance_m);
        let loss_db = propagation
            .absorption
            .loss_db(band_index, propagation_distance_m);
        let gain = 10.0_f64.powf((directivity_db[band_index] - loss_db) / 10.0);
        band_pressure_pa2[band_index] += spreading * gain;
    }
    let levels = band_pressure_pa2
        .into_iter()
        .map(Spl::from_mean_square_pressure_pa2)
        .collect::<Result<Vec<_>, _>>()?;
    Ok(DsmStationSpectrum {
        station_id: receiver.station_id.clone(),
        spectrum: SplSpectrum::new(shape.bands.clone(), levels)?,
        source_count: shape.bands.len(),
    })
}

/// Evaluate a height-stepped free-field DSM trace at one receiver station.
///
/// Each step shifts the receiver's axial coordinate by `vehicle_height_m`,
/// representing a station fixed to the rising vehicle over a fixed pad source
/// line.
///
/// # Errors
///
/// Returns [`DsmError`] if the steps are empty, sample times are not strictly
/// increasing, shifted receiver geometry is invalid, or station evaluation
/// fails.
pub fn evaluate_free_field_liftoff_trace(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    shape: &DsmSpectrumShape,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    steps: &[DsmLiftoffStep],
) -> Result<DsmLiftoffTrace, DsmError> {
    if steps.is_empty() {
        return Err(DsmError::Empty {
            field: "liftoff_steps",
        });
    }
    receiver.validate()?;
    let mut previous_time = None;
    let mut points = Vec::with_capacity(steps.len());
    for step in steps {
        let step = step.validate()?;
        if previous_time.is_some_and(|previous| step.time_s <= previous) {
            return Err(DsmError::NonIncreasing {
                field: "liftoff_step.time_s",
            });
        }
        previous_time = Some(step.time_s);
        let moved_receiver = DsmReceiver::new(
            receiver.station_id.clone(),
            receiver.axial_m + step.vehicle_height_m,
            receiver.radial_m,
        )?;
        let station = evaluate_free_field_station(
            exit,
            allocation,
            shape,
            directivity,
            &moved_receiver,
            propagation.clone(),
        )?;
        points.push(DsmLiftoffTracePoint { step, station });
    }
    Ok(DsmLiftoffTrace {
        station_id: receiver.station_id.clone(),
        points,
    })
}

/// Evaluate a height-stepped source-band-weighted DSM trace.
///
/// # Errors
///
/// Returns [`DsmError`] if the steps are empty, sample times are not strictly
/// increasing, shifted receiver geometry is invalid, or station evaluation
/// fails.
pub fn evaluate_source_band_weighted_liftoff_trace(
    exit: DsmExitCondition,
    allocation: &DsmSourceAllocation,
    source_band_weights: &DsmSourceBandWeights,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    steps: &[DsmLiftoffStep],
) -> Result<DsmLiftoffTrace, DsmError> {
    if steps.is_empty() {
        return Err(DsmError::Empty {
            field: "liftoff_steps",
        });
    }
    receiver.validate()?;
    let mut previous_time = None;
    let mut points = Vec::with_capacity(steps.len());
    for step in steps {
        let step = step.validate()?;
        if previous_time.is_some_and(|previous| step.time_s <= previous) {
            return Err(DsmError::NonIncreasing {
                field: "liftoff_step.time_s",
            });
        }
        previous_time = Some(step.time_s);
        let moved_receiver = DsmReceiver::new(
            receiver.station_id.clone(),
            receiver.axial_m + step.vehicle_height_m,
            receiver.radial_m,
        )?;
        let station = evaluate_source_band_weighted_station(
            exit,
            allocation,
            source_band_weights,
            directivity,
            &moved_receiver,
            propagation.clone(),
        )?;
        points.push(DsmLiftoffTracePoint { step, station });
    }
    Ok(DsmLiftoffTrace {
        station_id: receiver.station_id.clone(),
        points,
    })
}

/// Evaluate a height-stepped Figure 14 apparent-source DSM trace.
///
/// # Errors
///
/// Returns [`DsmError`] if the steps are empty, sample times are not strictly
/// increasing, shifted receiver geometry is invalid, or station evaluation
/// fails.
pub fn evaluate_apparent_source_liftoff_trace(
    exit: DsmExitCondition,
    shape: &DsmSpectrumShape,
    apparent_sources: &DsmApparentSourceCurve,
    directivity: &DsmDirectivityDeck,
    receiver: &DsmReceiver,
    propagation: DsmPropagation,
    steps: &[DsmLiftoffStep],
) -> Result<DsmLiftoffTrace, DsmError> {
    if steps.is_empty() {
        return Err(DsmError::Empty {
            field: "liftoff_steps",
        });
    }
    receiver.validate()?;
    let mut previous_time = None;
    let mut points = Vec::with_capacity(steps.len());
    for step in steps {
        let step = step.validate()?;
        if previous_time.is_some_and(|previous| step.time_s <= previous) {
            return Err(DsmError::NonIncreasing {
                field: "liftoff_step.time_s",
            });
        }
        previous_time = Some(step.time_s);
        let moved_receiver = DsmReceiver::new(
            receiver.station_id.clone(),
            receiver.axial_m + step.vehicle_height_m,
            receiver.radial_m,
        )?;
        let station = evaluate_apparent_source_station(
            exit,
            shape,
            apparent_sources,
            directivity,
            &moved_receiver,
            propagation.clone(),
        )?;
        points.push(DsmLiftoffTracePoint { step, station });
    }
    Ok(DsmLiftoffTrace {
        station_id: receiver.station_id.clone(),
        points,
    })
}

/// Spherical spreading loss in dB between two distances.
///
/// # Errors
///
/// Returns [`DsmError`] when either distance is invalid.
pub fn spreading_loss_delta_db(
    reference_distance_m: f64,
    distance_m: f64,
) -> Result<f64, DsmError> {
    require_positive("reference_distance_m", reference_distance_m)?;
    require_positive("distance_m", distance_m)?;
    Ok(20.0 * (distance_m / reference_distance_m).log10())
}

/// Air absorption loss in dB for a path length.
///
/// # Errors
///
/// Returns [`DsmError`] for invalid inputs.
pub fn absorption_loss_db(absorption_db_per_m: f64, distance_m: f64) -> Result<f64, DsmError> {
    require_non_negative("absorption_db_per_m", absorption_db_per_m)?;
    require_non_negative("distance_m", distance_m)?;
    Ok(absorption_db_per_m * distance_m)
}

/// NASA SP-8072 Figure 11 core-length fit, returning `x_t / d_e`.
///
/// # Errors
///
/// Returns [`DsmError`] when `exit_mach` is not finite and positive.
pub fn sp8072_core_length_over_exit_diameter(exit_mach: f64) -> Result<f64, DsmError> {
    require_positive("exit_mach", exit_mach)?;
    Ok(3.45 * (1.0 + 0.38 * exit_mach).powi(2))
}

/// NASA SP-8072 Figure 11 core length in meters.
///
/// # Errors
///
/// Returns [`DsmError`] when `exit_diameter_m` or `exit_mach` is not finite
/// and positive.
pub fn sp8072_core_length_m(exit_diameter_m: f64, exit_mach: f64) -> Result<f64, DsmError> {
    require_positive("exit_diameter_m", exit_diameter_m)?;
    Ok(exit_diameter_m * sp8072_core_length_over_exit_diameter(exit_mach)?)
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmDeckFile {
    openbmp: DsmDeckSchemaMarker,
    meta: DsmDeckMetaRow,
    source_allocation: Vec<DsmDeckSourceRow>,
    directivity: DsmDeckDirectivitySection,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmAirAbsorptionDeckFile {
    openbmp: DsmAirAbsorptionSchemaMarker,
    meta: DsmDeckMetaRow,
    absorption: Option<DsmAirAbsorptionSection>,
    #[serde(default)]
    profiles: Vec<DsmAirAbsorptionProfileRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmDeckSchemaMarker {
    acoustics_dsm_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmAirAbsorptionSchemaMarker {
    acoustics_dsm_absorption_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmDeckSourceRow {
    axial_m: f64,
    power_fraction: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmDeckDirectivitySection {
    center_hz: Vec<f64>,
    angles_deg: Vec<f64>,
    rows_db: Vec<Vec<f64>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmAirAbsorptionSection {
    center_hz: Vec<f64>,
    db_per_m: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct DsmAirAbsorptionProfileRow {
    profile_id: String,
    description: String,
    temperature_k: Option<f64>,
    static_pressure_pa: Option<f64>,
    relative_humidity: Option<f64>,
    center_hz: Vec<f64>,
    db_per_m: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SourceManifestFile {
    openbmp: Sp8072SourceSchemaMarker,
    meta: Sp8072SourceMetaRow,
    figures: Vec<Sp8072FigureRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SourceSchemaMarker {
    acoustics_sp8072_source_manifest: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SourceMetaRow {
    dataset_id: String,
    source_title: String,
    ntrs_citation_id: String,
    source_url: String,
    pdf_sha256: String,
    retrieved_utc: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072FigureRow {
    figure_id: String,
    pdf_page: u32,
    title: String,
    use_for: String,
    digitization_status: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialPowerCurveDeckFile {
    openbmp: Sp8072AxialPowerCurveSchemaMarker,
    meta: Sp8072AxialPowerCurveMetaRow,
    points: Vec<Sp8072AxialPowerPointRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialPowerCurveSchemaMarker {
    acoustics_sp8072_axial_power_curve: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialPowerCurveMetaRow {
    dataset_id: String,
    source_manifest_id: String,
    source_manifest_file: String,
    source_figure_id: String,
    source_pdf_sha256: String,
    digitization_method: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialPowerPointRow {
    x_over_core_length: f64,
    relative_power_db: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialSpectrumCurveDeckFile {
    openbmp: Sp8072AxialSpectrumCurveSchemaMarker,
    meta: Sp8072AxialSpectrumCurveMetaRow,
    points: Vec<Sp8072AxialSpectrumPointRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialSpectrumCurveSchemaMarker {
    acoustics_sp8072_axial_spectrum_curve: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialSpectrumCurveMetaRow {
    dataset_id: String,
    source_manifest_id: String,
    source_manifest_file: String,
    source_figure_id: String,
    source_pdf_sha256: String,
    digitization_method: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072AxialSpectrumPointRow {
    modified_axial_strouhal: f64,
    relative_power_db: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SpectrumCurveDeckFile {
    openbmp: Sp8072SpectrumCurveSchemaMarker,
    meta: Sp8072SpectrumCurveMetaRow,
    points: Vec<Sp8072SpectrumPointRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SpectrumCurveSchemaMarker {
    acoustics_sp8072_spectrum_curve: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SpectrumCurveMetaRow {
    dataset_id: String,
    source_manifest_id: String,
    source_manifest_file: String,
    source_figure_id: String,
    source_pdf_sha256: String,
    digitization_method: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072SpectrumPointRow {
    strouhal: f64,
    relative_power_db: f64,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072DirectivityCurveDeckFile {
    openbmp: Sp8072DirectivityCurveSchemaMarker,
    meta: Sp8072DirectivityCurveMetaRow,
    curve: Sp8072DirectivityCurveSection,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072DirectivityCurveSchemaMarker {
    acoustics_sp8072_directivity_curve: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072DirectivityCurveMetaRow {
    dataset_id: String,
    source_manifest_id: String,
    source_manifest_file: String,
    source_figure_id: String,
    source_pdf_sha256: String,
    digitization_method: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072DirectivityCurveSection {
    strouhal: Vec<f64>,
    angles_deg: Vec<f64>,
    rows_db: Vec<Vec<f64>>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072ApparentSourceCurveDeckFile {
    openbmp: Sp8072ApparentSourceCurveSchemaMarker,
    meta: Sp8072ApparentSourceCurveMetaRow,
    points: Vec<Sp8072ApparentSourcePointRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072ApparentSourceCurveSchemaMarker {
    acoustics_sp8072_apparent_source_curve: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072ApparentSourceCurveMetaRow {
    dataset_id: String,
    source_manifest_id: String,
    source_manifest_file: String,
    source_figure_id: String,
    source_pdf_sha256: String,
    digitization_method: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Clone, Copy, Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct Sp8072ApparentSourcePointRow {
    strouhal: f64,
    x_over_exit_diameter: f64,
}

fn source_allocation_from_deck_rows(
    rows: Vec<DsmDeckSourceRow>,
) -> Result<DsmSourceAllocation, DsmError> {
    if rows.is_empty() {
        return Err(DsmError::Empty {
            field: "source_allocation",
        });
    }
    let mut previous = None;
    let mut sources = Vec::with_capacity(rows.len());
    for row in rows {
        require_non_negative("source_allocation.axial_m", row.axial_m)?;
        require_fraction("source_allocation.power_fraction", row.power_fraction)?;
        if previous.is_some_and(|previous| row.axial_m <= previous) {
            return Err(DsmError::NonIncreasing {
                field: "source_allocation.axial_m",
            });
        }
        previous = Some(row.axial_m);
        sources.push(DsmSource {
            axial_m: row.axial_m,
            power_fraction: row.power_fraction,
        });
    }
    DsmSourceAllocation::new(sources)
}

fn sp8072_figures_from_rows(
    rows: Vec<Sp8072FigureRow>,
) -> Result<Vec<Sp8072FigureReference>, DsmError> {
    if rows.is_empty() {
        return Err(DsmError::Empty { field: "figures" });
    }
    let mut seen = BTreeSet::new();
    let mut figures = Vec::with_capacity(rows.len());
    for row in rows {
        let figure = Sp8072FigureReference::from_row(row)?;
        if !seen.insert(figure.figure_id.clone()) {
            return Err(DsmError::Duplicate {
                field: "figures.figure_id",
            });
        }
        figures.push(figure);
    }
    Ok(figures)
}

fn validate_sp8072_curve_source_manifest(
    source_manifest_id: &str,
    source_figure_id: &str,
    source_pdf_sha256: &str,
    manifest: &Sp8072SourceManifest,
) -> Result<(), DsmError> {
    if source_manifest_id != manifest.meta.dataset_id {
        return Err(DsmError::MalformedDeck {
            reason: format!(
                "SP-8072 curve source_manifest_id `{source_manifest_id}` does not match manifest dataset_id `{}`",
                manifest.meta.dataset_id
            ),
        });
    }
    if source_pdf_sha256 != manifest.meta.pdf_sha256 {
        return Err(DsmError::MalformedDeck {
            reason: format!(
                "SP-8072 curve source_pdf_sha256 `{source_pdf_sha256}` does not match manifest pdf_sha256 `{}`",
                manifest.meta.pdf_sha256
            ),
        });
    }
    if manifest.figure(source_figure_id).is_none() {
        return Err(DsmError::MalformedDeck {
            reason: format!(
                "SP-8072 curve source_figure_id `{source_figure_id}` is not present in source manifest"
            ),
        });
    }
    Ok(())
}

fn axial_power_curve_from_rows(
    rows: Vec<Sp8072AxialPowerPointRow>,
) -> Result<DsmAxialPowerCurve, DsmError> {
    DsmAxialPowerCurve::new(
        rows.into_iter()
            .map(|row| DsmAxialPowerPoint {
                x_over_core_length: row.x_over_core_length,
                relative_power_db: row.relative_power_db,
            })
            .collect(),
    )
}

fn axial_spectrum_curve_from_rows(
    rows: Vec<Sp8072AxialSpectrumPointRow>,
) -> Result<DsmAxialSpectrumCurve, DsmError> {
    DsmAxialSpectrumCurve::new(
        rows.into_iter()
            .map(|row| DsmAxialSpectrumPoint {
                modified_axial_strouhal: row.modified_axial_strouhal,
                relative_power_db: row.relative_power_db,
            })
            .collect(),
    )
}

fn spectrum_curve_from_rows(
    rows: Vec<Sp8072SpectrumPointRow>,
) -> Result<DsmSpectrumCurve, DsmError> {
    DsmSpectrumCurve::new(
        rows.into_iter()
            .map(|row| DsmSpectrumPoint {
                strouhal: row.strouhal,
                relative_power_db: row.relative_power_db,
            })
            .collect(),
    )
}

fn apparent_source_curve_from_rows(
    rows: Vec<Sp8072ApparentSourcePointRow>,
) -> Result<DsmApparentSourceCurve, DsmError> {
    DsmApparentSourceCurve::new(
        rows.into_iter()
            .map(|row| DsmApparentSourcePoint {
                strouhal: row.strouhal,
                x_over_exit_diameter: row.x_over_exit_diameter,
            })
            .collect(),
    )
}

fn absorption_profiles_from_deck(
    absorption: Option<DsmAirAbsorptionSection>,
    profile_rows: Vec<DsmAirAbsorptionProfileRow>,
) -> Result<Vec<DsmAirAbsorptionProfile>, DsmError> {
    if absorption.is_some() && !profile_rows.is_empty() {
        return Err(DsmError::MalformedDeck {
            reason:
                "air-absorption deck must declare either [absorption] or [[profiles]], not both"
                    .to_owned(),
        });
    }

    if let Some(absorption) = absorption {
        let deck = absorption_deck_from_section(absorption)?;
        return Ok(vec![DsmAirAbsorptionProfile {
            profile_id: "default".to_owned(),
            description: "single-profile air absorption section".to_owned(),
            temperature_k: None,
            static_pressure_pa: None,
            relative_humidity: None,
            absorption: deck,
        }]);
    }

    if profile_rows.is_empty() {
        return Err(DsmError::Empty { field: "profiles" });
    }

    let mut seen = BTreeSet::new();
    let mut profiles = Vec::with_capacity(profile_rows.len());
    for row in profile_rows {
        require_text("profiles.profile_id", &row.profile_id)?;
        require_text("profiles.description", &row.description)?;
        let profile_id = row.profile_id.trim().to_owned();
        if !seen.insert(profile_id.clone()) {
            return Err(DsmError::Duplicate {
                field: "profiles.profile_id",
            });
        }
        if let Some(temperature_k) = row.temperature_k {
            require_positive("profiles.temperature_k", temperature_k)?;
        }
        if let Some(static_pressure_pa) = row.static_pressure_pa {
            require_positive("profiles.static_pressure_pa", static_pressure_pa)?;
        }
        if let Some(relative_humidity) = row.relative_humidity {
            require_fraction("profiles.relative_humidity", relative_humidity)?;
        }
        profiles.push(DsmAirAbsorptionProfile {
            profile_id,
            description: row.description.trim().to_owned(),
            temperature_k: row.temperature_k,
            static_pressure_pa: row.static_pressure_pa,
            relative_humidity: row.relative_humidity,
            absorption: absorption_deck_from_section(DsmAirAbsorptionSection {
                center_hz: row.center_hz,
                db_per_m: row.db_per_m,
            })?,
        });
    }
    Ok(profiles)
}

fn absorption_deck_from_section(
    section: DsmAirAbsorptionSection,
) -> Result<DsmAbsorptionDeck, DsmError> {
    let bands = section
        .center_hz
        .into_iter()
        .map(AcousticBand::one_third_octave)
        .collect::<Result<Vec<_>, _>>()?;
    DsmAbsorptionDeck::new(bands, section.db_per_m)
}

fn lower_axis_index_linear(axis: &[f64], value: f64) -> usize {
    if axis.len() == 1 || value <= axis[0] {
        return 0;
    }
    let last = axis.len() - 1;
    if value >= axis[last] {
        return last;
    }
    axis.windows(2)
        .position(|window| value >= window[0] && value <= window[1])
        .unwrap_or(0)
}

fn lower_axis_index_log(axis: &[f64], value: f64) -> usize {
    lower_axis_index_linear(axis, value)
}

fn linear_axis_fraction(lower: f64, upper: f64, value: f64) -> f64 {
    if upper.to_bits() == lower.to_bits() {
        0.0
    } else {
        ((value - lower) / (upper - lower)).clamp(0.0, 1.0)
    }
}

fn log_axis_fraction(lower: f64, upper: f64, value: f64) -> f64 {
    if upper.to_bits() == lower.to_bits() {
        0.0
    } else {
        let lower_log = lower.log10();
        ((value.log10() - lower_log) / (upper.log10() - lower_log)).clamp(0.0, 1.0)
    }
}

fn interpolate_linear(lower: f64, upper: f64, fraction: f64) -> f64 {
    lower * (1.0 - fraction) + upper * fraction
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in digest {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

fn normalize_band_weights(
    bands: Vec<AcousticBand>,
    weights: Vec<f64>,
) -> Result<DsmSpectrumShape, DsmError> {
    let mut total = 0.0;
    for weight in &weights {
        require_non_negative("band_weight", *weight)?;
        total += *weight;
    }
    require_positive("band_weight_sum", total)?;
    DsmSpectrumShape::new(
        bands,
        weights.into_iter().map(|weight| weight / total).collect(),
    )
}

fn validate_band_values(bands: &[AcousticBand], value_count: usize) -> Result<(), DsmError> {
    if bands.is_empty() {
        return Err(DsmError::Empty { field: "bands" });
    }
    if bands.len() != value_count {
        return Err(DsmError::LengthMismatch {
            bands: bands.len(),
            values: value_count,
        });
    }
    let mut previous = None;
    for band in bands {
        require_positive("band.center_hz", band.center_hz)?;
        require_positive("band.lower_hz", band.lower_hz)?;
        require_positive("band.upper_hz", band.upper_hz)?;
        if previous.is_some_and(|previous| band.center_hz <= previous) {
            return Err(DsmError::NonIncreasing {
                field: "band.center_hz",
            });
        }
        previous = Some(band.center_hz);
    }
    Ok(())
}

fn same_bands(lhs: &[AcousticBand], rhs: &[AcousticBand]) -> bool {
    lhs.len() == rhs.len()
        && lhs.iter().zip(rhs.iter()).all(|(lhs, rhs)| {
            lhs.center_hz.to_bits() == rhs.center_hz.to_bits()
                && lhs.lower_hz.to_bits() == rhs.lower_hz.to_bits()
                && lhs.upper_hz.to_bits() == rhs.upper_hz.to_bits()
        })
}

fn require_finite(field: &'static str, value: f64) -> Result<(), DsmError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(DsmError::NonFinite { field })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<(), DsmError> {
    require_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(DsmError::NonPositive { field })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), DsmError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(DsmError::Negative { field })
    }
}

fn require_fraction(field: &'static str, value: f64) -> Result<(), DsmError> {
    require_finite(field, value)?;
    if (0.0..=1.0).contains(&value) {
        Ok(())
    } else {
        Err(DsmError::InvalidFraction { field })
    }
}

fn require_sha256_hex(field: &'static str, value: &str) -> Result<(), DsmError> {
    let value = value.trim();
    if value.len() == 64
        && value
            .bytes()
            .all(|byte| matches!(byte, b'0'..=b'9' | b'a'..=b'f'))
    {
        Ok(())
    } else {
        Err(DsmError::InvalidSha256 { field })
    }
}

fn require_text(field: &'static str, value: &str) -> Result<(), DsmError> {
    if value.trim().is_empty() {
        Err(DsmError::Empty { field })
    } else {
        Ok(())
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    fn toy_exit() -> DsmExitCondition {
        DsmExitCondition {
            mass_flow_kg_s: 10.0,
            exit_velocity_m_s: 2000.0,
            exit_diameter_m: 1.0,
            acoustic_efficiency: 0.01,
        }
    }

    fn toy_shape() -> DsmSpectrumShape {
        DsmSpectrumShape::strouhal_gaussian(1.0, 2000.0, 0.2, 0.5).unwrap()
    }

    fn toy_allocation() -> DsmSourceAllocation {
        DsmSourceAllocation::normalized_from_weights(&[(0.0, 1.0), (10.0, 2.0), (20.0, 1.0)])
            .unwrap()
    }

    fn toy_receiver() -> DsmReceiver {
        DsmReceiver::new("station-a", 5.0, 30.0).unwrap()
    }

    fn toy_propagation() -> DsmPropagation {
        let bands = standard_third_octave_bands();
        DsmPropagation::uniform(408.0, 0.001, bands).unwrap()
    }

    #[test]
    fn dsm_acoustic_power_matches_exit_condition_closed_form() {
        let exit = toy_exit();

        assert_eq!(exit.acoustic_power_w().unwrap(), 200_000.0);
        assert_relative_eq!(
            exit.sound_power_level_db().unwrap(),
            10.0 * (200_000.0_f64 / REFERENCE_ACOUSTIC_POWER_W).log10(),
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn strouhal_shape_is_normalized_and_deterministic() {
        let shape = toy_shape();
        let sum = shape.band_power_fractions().iter().sum::<f64>();

        assert_relative_eq!(sum, 1.0, epsilon = 1.0e-15);
        assert_eq!(
            shape.bands().len(),
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );
        assert!(
            shape
                .band_power_fractions()
                .windows(2)
                .any(|window| window[0].to_bits() != window[1].to_bits())
        );
    }

    #[test]
    fn directivity_deck_interpolates_by_angle() {
        let bands = standard_third_octave_bands()[..2].to_vec();
        let deck = DsmDirectivityDeck::new(
            bands,
            vec![0.0, 90.0],
            vec![vec![0.0, 10.0], vec![20.0, 30.0]],
        )
        .unwrap();

        let values = deck.directivity_db_at(45.0).unwrap();

        assert_eq!(values, vec![10.0, 20.0]);
    }

    #[test]
    fn absorption_deck_validates_band_coefficients() {
        let bands = standard_third_octave_bands()[..2].to_vec();
        let deck = DsmAbsorptionDeck::new(bands.clone(), vec![0.001, 0.002]).unwrap();

        assert_eq!(deck.bands(), bands.as_slice());
        assert_eq!(deck.db_per_m(), &[0.001, 0.002]);

        assert!(matches!(
            DsmAbsorptionDeck::new(bands.clone(), vec![0.001]).unwrap_err(),
            DsmError::LengthMismatch { .. }
        ));
        assert!(matches!(
            DsmAbsorptionDeck::new(bands, vec![0.001, -0.002]).unwrap_err(),
            DsmError::Negative {
                field: "absorption.db_per_m"
            }
        ));
    }

    #[test]
    fn spreading_and_absorption_closed_forms_match_reference() {
        assert_relative_eq!(
            spreading_loss_delta_db(10.0, 20.0).unwrap(),
            6.020_599_913_279_624,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            absorption_loss_db(0.1, 10.0).unwrap(),
            1.0,
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn sp8072_core_length_fit_matches_figure_11_formula() {
        assert_relative_eq!(
            sp8072_core_length_over_exit_diameter(0.5).unwrap(),
            4.885_545,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            sp8072_core_length_over_exit_diameter(1.0).unwrap(),
            6.570_18,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            sp8072_core_length_over_exit_diameter(2.0).unwrap(),
            10.686_72,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            sp8072_core_length_m(2.0, 1.0).unwrap(),
            13.140_36,
            epsilon = 1.0e-12
        );
    }

    #[test]
    fn sp8072_core_length_fit_rejects_bad_inputs() {
        assert!(matches!(
            sp8072_core_length_over_exit_diameter(0.0).unwrap_err(),
            DsmError::NonPositive { field: "exit_mach" }
        ));
        assert!(matches!(
            sp8072_core_length_m(-1.0, 1.0).unwrap_err(),
            DsmError::NonPositive {
                field: "exit_diameter_m"
            }
        ));
        assert!(matches!(
            sp8072_core_length_m(1.0, f64::NAN).unwrap_err(),
            DsmError::NonFinite { field: "exit_mach" }
        ));
    }

    #[test]
    fn free_field_dsm_synthetic_worked_example_is_byte_stable() {
        let shape = toy_shape();
        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let result = evaluate_free_field_station(
            toy_exit(),
            &toy_allocation(),
            &shape,
            &directivity,
            &toy_receiver(),
            toy_propagation(),
        )
        .unwrap();

        assert_eq!(result.source_count, 3);
        assert_relative_eq!(
            result.spectrum.oaspl().unwrap().db_re_20upa(),
            132.214_258_788_974_58,
            epsilon = 1.0e-12
        );
        assert_eq!(
            result.to_csv(),
            include_str!("../tests/fixtures/dsm-free-field-station.csv")
        );
    }

    #[test]
    fn plume_deflection_changes_station_range_by_single_bend_geometry() {
        let shape = toy_shape();
        let allocation = DsmSourceAllocation::normalized_from_weights(&[(10.0, 1.0)]).unwrap();
        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let receiver = DsmReceiver::new("station-a", 0.0, 20.0).unwrap();
        let propagation = DsmPropagation::uniform(408.0, 0.0, shape.bands().to_vec()).unwrap();
        let deflection = DsmPlumeDeflection::new(0.0, 90.0).unwrap();

        let straight = evaluate_free_field_station(
            toy_exit(),
            &allocation,
            &shape,
            &directivity,
            &receiver,
            propagation.clone(),
        )
        .unwrap();
        let deflected = evaluate_deflected_free_field_station(
            toy_exit(),
            &allocation,
            &shape,
            &directivity,
            &receiver,
            propagation,
            deflection,
        )
        .unwrap();
        let straight_distance_m = 10.0_f64.hypot(20.0);
        let deflected_distance_m = 10.0_f64;
        let expected_delta_db = 20.0 * (straight_distance_m / deflected_distance_m).log10();

        assert_relative_eq!(
            deflected.spectrum.oaspl().unwrap().db_re_20upa()
                - straight.spectrum.oaspl().unwrap().db_re_20upa(),
            expected_delta_db,
            epsilon = 1.0e-12
        );
        assert!(matches!(
            DsmPlumeDeflection::new(0.0, 91.0).unwrap_err(),
            DsmError::OutOfRange {
                field: "plume_deflection.angle_deg"
            }
        ));
    }

    #[test]
    fn free_field_liftoff_trace_is_byte_stable_and_decays_with_height() {
        let shape = toy_shape();
        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let steps = [
            DsmLiftoffStep::new(0.0, 0.0).unwrap(),
            DsmLiftoffStep::new(1.0, 50.0).unwrap(),
            DsmLiftoffStep::new(2.0, 100.0).unwrap(),
        ];

        let trace = evaluate_free_field_liftoff_trace(
            toy_exit(),
            &toy_allocation(),
            &shape,
            &directivity,
            &toy_receiver(),
            toy_propagation(),
            &steps,
        )
        .unwrap();

        let oaspl = trace
            .points()
            .iter()
            .map(|point| point.station.spectrum.oaspl().unwrap().db_re_20upa())
            .collect::<Vec<_>>();
        assert_eq!(trace.points().len(), 3);
        assert!(oaspl.windows(2).all(|window| window[0] > window[1]));
        assert_eq!(
            trace.to_oaspl_csv().unwrap(),
            include_str!("../tests/fixtures/dsm-liftoff-oaspl.csv")
        );
    }

    #[test]
    fn dsm_deck_parses_fixture_and_feeds_station_evaluation() {
        let fixture = include_str!("../../../data/acoustics/dsm-source-directivity-v1.toml");
        let deck = DsmPredictionDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.dsm_source_directivity.v1"
        );
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(
            deck.sha256_hex(),
            "169a6c35e513aafd8f111e6ba7e0b8d707cd0f1c8b1904eabc056d5d7536fe00"
        );
        assert_eq!(deck.source_allocation().sources().len(), 3);
        assert_eq!(
            deck.directivity().bands().len(),
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );
        assert_eq!(deck.directivity().angles_deg(), &[0.0, 90.0, 180.0]);
        let interpolated = deck.directivity().directivity_db_at(45.0).unwrap();
        assert!(
            interpolated
                .iter()
                .all(|value| (*value + 1.5).abs() <= 1.0e-12)
        );

        let station = evaluate_free_field_station(
            toy_exit(),
            deck.source_allocation(),
            &toy_shape(),
            deck.directivity(),
            &toy_receiver(),
            toy_propagation(),
        )
        .unwrap();

        assert_eq!(station.source_count, 3);
        assert!(station.spectrum.oaspl().unwrap().db_re_20upa().is_finite());
    }

    #[test]
    fn dsm_deck_helpers_fail_closed_on_bad_inputs() {
        let fixture = include_str!("../../../data/acoustics/dsm-source-directivity-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_dsm_deck = 1",
            "openbmp.acoustics_dsm_deck = 2",
        );
        assert!(matches!(
            DsmPredictionDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_allocation_sum =
            fixture.replacen("power_fraction = 0.3", "power_fraction = 0.4", 1);
        assert!(matches!(
            DsmPredictionDeck::load_from_str(&bad_allocation_sum).unwrap_err(),
            DsmError::InvalidFraction {
                field: "source.power_fraction_sum"
            }
        ));

        let bad_allocation_order = fixture.replacen("axial_m = 10.0", "axial_m = 0.0", 1);
        assert!(matches!(
            DsmPredictionDeck::load_from_str(&bad_allocation_order).unwrap_err(),
            DsmError::NonIncreasing {
                field: "source_allocation.axial_m"
            }
        ));

        let bad_directivity = fixture.replacen("    -3.0,\n", "", 1);
        assert!(matches!(
            DsmPredictionDeck::load_from_str(&bad_directivity).unwrap_err(),
            DsmError::LengthMismatch { .. }
        ));
    }

    #[test]
    fn dsm_air_absorption_deck_parses_fixture_and_feeds_station_evaluation() {
        let fixture = include_str!("../../../data/acoustics/dsm-air-absorption-v1.toml");
        let deck = DsmAirAbsorptionDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.dsm_air_absorption.v1"
        );
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(
            deck.sha256_hex(),
            "cda1136c9a9f935ac576fbe06671f00d8e0d1ed56bbb5d9c2cd42dc34847af48"
        );
        assert_eq!(deck.profiles().len(), 1);
        assert_eq!(deck.profiles()[0].profile_id, "default");
        assert_eq!(
            deck.absorption().bands().len(),
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );
        assert_relative_eq!(deck.absorption().db_per_m()[13], 0.00030, epsilon = 1.0e-15);

        let shape = toy_shape();
        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let propagation = DsmPropagation {
            ambient_impedance_pa_s_m: 408.0,
            absorption: deck.absorption().clone(),
            path_length_extra_m: 0.0,
        };
        let station = evaluate_free_field_station(
            toy_exit(),
            &toy_allocation(),
            &shape,
            &directivity,
            &toy_receiver(),
            propagation,
        )
        .unwrap();

        assert_eq!(station.source_count, 3);
        assert!(station.spectrum.oaspl().unwrap().db_re_20upa().is_finite());
    }

    #[test]
    fn dsm_air_absorption_deck_selects_atmospheric_profiles() {
        let fixture =
            include_str!("../../../data/acoustics/dsm-air-absorption-atmospheres-v1.toml");
        let deck = DsmAirAbsorptionDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.dsm_air_absorption_atmospheres.v1"
        );
        assert_eq!(
            deck.sha256_hex(),
            "c4f425557a8a99204b1edb05383abf7a07abcf846345679544ac2894ba841fca"
        );
        assert_eq!(deck.profiles().len(), 2);
        assert_eq!(deck.absorption().db_per_m()[13], 0.00030);
        let humid = deck
            .absorption_profile(Some("humid_pad_303k_70rh"))
            .expect("humid profile present");
        assert_eq!(humid.profile_id, "humid_pad_303k_70rh");
        assert_eq!(humid.temperature_k, Some(303.0));
        assert_eq!(humid.relative_humidity, Some(0.70));
        assert_relative_eq!(
            humid.absorption().db_per_m()[13],
            0.00044,
            epsilon = 1.0e-15
        );
        assert!(matches!(
            deck.absorption_profile(Some("missing_profile"))
                .unwrap_err(),
            DsmError::MalformedDeck { .. }
        ));
    }

    #[test]
    fn dsm_air_absorption_deck_helpers_fail_closed_on_bad_inputs() {
        let fixture = include_str!("../../../data/acoustics/dsm-air-absorption-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_dsm_absorption_deck = 1",
            "openbmp.acoustics_dsm_absorption_deck = 2",
        );
        assert!(matches!(
            DsmAirAbsorptionDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_length = fixture.replacen("  0.08500,\n", "", 1);
        assert!(matches!(
            DsmAirAbsorptionDeck::load_from_str(&bad_length).unwrap_err(),
            DsmError::LengthMismatch { .. }
        ));

        let bad_value = fixture.replacen("  0.00030,", "  -0.00030,", 1);
        assert!(matches!(
            DsmAirAbsorptionDeck::load_from_str(&bad_value).unwrap_err(),
            DsmError::Negative {
                field: "absorption.db_per_m"
            }
        ));

        let profiles =
            include_str!("../../../data/acoustics/dsm-air-absorption-atmospheres-v1.toml");
        let duplicate_profile = profiles.replace("humid_pad_303k_70rh", "standard_sea_level");
        assert!(matches!(
            DsmAirAbsorptionDeck::load_from_str(&duplicate_profile).unwrap_err(),
            DsmError::Duplicate {
                field: "profiles.profile_id"
            }
        ));

        let bad_humidity =
            profiles.replacen("relative_humidity = 0.70", "relative_humidity = 1.70", 1);
        assert!(matches!(
            DsmAirAbsorptionDeck::load_from_str(&bad_humidity).unwrap_err(),
            DsmError::InvalidFraction {
                field: "profiles.relative_humidity"
            }
        ));
    }

    #[test]
    fn sp8072_source_manifest_parses_public_ntrs_inventory() {
        let fixture = include_str!("../../../data/acoustics/sp8072-source-v1.toml");
        let manifest = Sp8072SourceManifest::load_from_str(fixture).unwrap();

        assert_eq!(
            manifest.meta().dataset_id,
            "openbmp.acoustics.sp8072_source.v1"
        );
        assert_eq!(manifest.meta().ntrs_citation_id, "19710023719");
        assert_eq!(
            manifest.meta().source_url,
            "https://ntrs.nasa.gov/api/citations/19710023719/downloads/19710023719.pdf"
        );
        assert_eq!(
            manifest.meta().pdf_sha256,
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
        );
        assert_eq!(
            manifest.sha256_hex(),
            "4b47a0436aa59ac47793979e7f2373f4ee44f3d60ece011cb26e677571b4b3ea"
        );
        assert_eq!(manifest.figures().len(), 9);

        let figure_5 = manifest.figure("figure_5").unwrap();
        assert_eq!(figure_5.pdf_page, 16);
        assert_eq!(figure_5.use_for, "DSM normalized spectrum digitization");
        assert_eq!(figure_5.digitization_status, "coarse-manual-digitized-v1");

        let figure_12 = manifest.figure("figure_12").unwrap();
        assert_eq!(figure_12.pdf_page, 21);
        assert_eq!(
            figure_12.use_for,
            "DSM source-power distribution digitization"
        );
        assert_eq!(figure_12.digitization_status, "coarse-manual-digitized-v1");
        assert_eq!(
            manifest.figure("figure_11").unwrap().digitization_status,
            "formula-implemented"
        );
        assert!(
            manifest
                .figure("figure_10")
                .unwrap()
                .title
                .contains("Directivity")
        );
        assert_eq!(
            manifest.figure("figure_10").unwrap().digitization_status,
            "coarse-manual-digitized-v1"
        );
        assert_eq!(
            manifest.figure("figure_13").unwrap().digitization_status,
            "coarse-manual-digitized-v1"
        );
        assert_eq!(
            manifest.figure("figure_14").unwrap().digitization_status,
            "coarse-manual-digitized-v1"
        );
        assert_eq!(manifest.figure("figure_20").unwrap().pdf_page, 30);
        assert!(
            manifest
                .figures()
                .iter()
                .filter(|figure| figure.digitization_status == "pending-public-digitization")
                .count()
                >= 3
        );
    }

    #[test]
    fn sp8072_source_manifest_helpers_fail_closed_on_bad_inputs() {
        let fixture = include_str!("../../../data/acoustics/sp8072-source-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_sp8072_source_manifest = 1",
            "openbmp.acoustics_sp8072_source_manifest = 2",
        );
        assert!(matches!(
            Sp8072SourceManifest::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_sha = fixture.replace(
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8",
            "84F1483C2C8477810186E98765EA99E4B7F7CC71E0A47EB8F86977D3B02AABF8",
        );
        assert!(matches!(
            Sp8072SourceManifest::load_from_str(&bad_sha).unwrap_err(),
            DsmError::InvalidSha256 {
                field: "meta.pdf_sha256"
            }
        ));

        let duplicate_figure =
            fixture.replacen("figure_id = \"figure_13\"", "figure_id = \"figure_12\"", 1);
        assert!(matches!(
            Sp8072SourceManifest::load_from_str(&duplicate_figure).unwrap_err(),
            DsmError::Duplicate {
                field: "figures.figure_id"
            }
        ));

        let empty_title = fixture.replacen(
            "title = \"Directivity of far-field noise for standard chemical rockets\"",
            "title = \"\"",
            1,
        );
        assert!(matches!(
            Sp8072SourceManifest::load_from_str(&empty_title).unwrap_err(),
            DsmError::Empty {
                field: "figures.title"
            }
        ));
    }

    #[test]
    fn sp8072_curve_decks_validate_public_source_manifest_bindings() {
        let manifest = Sp8072SourceManifest::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-source-v1.toml"
        ))
        .unwrap();
        let spectrum = Sp8072SpectrumCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure5-spectrum-v1.toml"
        ))
        .unwrap();
        let source_power = Sp8072AxialPowerCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure12-source-power-v1.toml"
        ))
        .unwrap();
        let axial_spectrum = Sp8072AxialSpectrumCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure13-axial-spectrum-v1.toml"
        ))
        .unwrap();
        let directivity = Sp8072DirectivityCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure10-directivity-v1.toml"
        ))
        .unwrap();
        let apparent_source = Sp8072ApparentSourceCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure14-apparent-source-v1.toml"
        ))
        .unwrap();

        spectrum.validate_source_manifest(&manifest).unwrap();
        source_power.validate_source_manifest(&manifest).unwrap();
        axial_spectrum.validate_source_manifest(&manifest).unwrap();
        directivity.validate_source_manifest(&manifest).unwrap();
        apparent_source.validate_source_manifest(&manifest).unwrap();

        let fixture = include_str!("../../../data/acoustics/sp8072-figure5-spectrum-v1.toml");
        let bad_manifest_id = fixture.replace(
            "source_manifest_id = \"openbmp.acoustics.sp8072_source.v1\"",
            "source_manifest_id = \"openbmp.acoustics.sp8072_source.bad\"",
        );
        let bad_manifest_id_deck =
            Sp8072SpectrumCurveDeck::load_from_str(&bad_manifest_id).unwrap();
        match bad_manifest_id_deck
            .validate_source_manifest(&manifest)
            .unwrap_err()
        {
            DsmError::MalformedDeck { reason } => {
                assert!(reason.contains("source_manifest_id"));
            }
            other => panic!("unexpected error: {other:?}"),
        }

        let bad_pdf_hash = fixture.replace(
            "source_pdf_sha256 = \"84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8\"",
            "source_pdf_sha256 = \"0000000000000000000000000000000000000000000000000000000000000000\"",
        );
        let bad_pdf_hash_deck = Sp8072SpectrumCurveDeck::load_from_str(&bad_pdf_hash).unwrap();
        match bad_pdf_hash_deck
            .validate_source_manifest(&manifest)
            .unwrap_err()
        {
            DsmError::MalformedDeck { reason } => {
                assert!(reason.contains("source_pdf_sha256"));
            }
            other => panic!("unexpected error: {other:?}"),
        }

        let bad_figure = fixture.replace(
            "source_figure_id = \"figure_5\"",
            "source_figure_id = \"figure_999\"",
        );
        let bad_figure_deck = Sp8072SpectrumCurveDeck::load_from_str(&bad_figure).unwrap();
        match bad_figure_deck
            .validate_source_manifest(&manifest)
            .unwrap_err()
        {
            DsmError::MalformedDeck { reason } => {
                assert!(reason.contains("source_figure_id"));
            }
            other => panic!("unexpected error: {other:?}"),
        }
    }

    #[test]
    fn sp8072_coarse_curve_worked_example_tolerance_table_is_deterministic() {
        let spectrum_deck = Sp8072SpectrumCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure5-spectrum-v1.toml"
        ))
        .unwrap();
        let source_power_deck = Sp8072AxialPowerCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure12-source-power-v1.toml"
        ))
        .unwrap();
        let axial_spectrum_deck = Sp8072AxialSpectrumCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure13-axial-spectrum-v1.toml"
        ))
        .unwrap();
        let directivity_deck = Sp8072DirectivityCurveDeck::load_from_str(include_str!(
            "../../../data/acoustics/sp8072-figure10-directivity-v1.toml"
        ))
        .unwrap();
        let shape = spectrum_deck
            .curve()
            .to_spectrum_shape(toy_exit().exit_diameter_m, toy_exit().exit_velocity_m_s)
            .unwrap();
        let allocation = source_power_deck
            .curve()
            .to_source_allocation(sp8072_core_length_m(toy_exit().exit_diameter_m, 1.0).unwrap())
            .unwrap();
        let source_band_weights = axial_spectrum_deck
            .curve()
            .to_source_band_weights(&allocation, &shape, toy_exit().exit_velocity_m_s)
            .unwrap();
        let directivity = directivity_deck
            .curve()
            .to_directivity_deck(
                shape.bands().to_vec(),
                toy_exit().exit_diameter_m,
                toy_exit().exit_velocity_m_s,
            )
            .unwrap();
        let station = evaluate_source_band_weighted_station(
            toy_exit(),
            &allocation,
            &source_band_weights,
            &directivity,
            &toy_receiver(),
            toy_propagation(),
        )
        .unwrap();
        let reference = station
            .spectrum
            .apply_transfer_db(&vec![0.25; station.spectrum.levels().len()])
            .unwrap();
        let report =
            dsm_spectrum_tolerance_report(&station.spectrum, &reference, 1.0, 2.0).unwrap();

        assert!(report.within_tolerance());
        assert_eq!(
            report.rows().len(),
            crate::STANDARD_THIRD_OCTAVE_CENTER_HZ.len()
        );
        assert_relative_eq!(report.oaspl_residual_db, -0.25, epsilon = 1.0e-12);
        assert!(
            report
                .rows()
                .iter()
                .all(|row| (row.residual_db + 0.25).abs() <= 1.0e-12)
        );
        assert!(report.to_csv().starts_with(
            "metric,center_hz,evaluated_spl_db,reference_spl_db,residual_db,tolerance_db,within_tolerance\n\
             oaspl,all,"
        ));

        let failing_reference = station
            .spectrum
            .apply_transfer_db(&vec![3.0; station.spectrum.levels().len()])
            .unwrap();
        let failing_report =
            dsm_spectrum_tolerance_report(&station.spectrum, &failing_reference, 1.0, 2.0).unwrap();
        assert!(!failing_report.within_tolerance());
        assert!(!failing_report.oaspl_within_tolerance);
        assert!(
            failing_report
                .rows()
                .iter()
                .all(|row| !row.within_tolerance)
        );

        assert!(matches!(
            dsm_spectrum_tolerance_report(&station.spectrum, &reference, -1.0, 2.0).unwrap_err(),
            DsmError::Negative {
                field: "oaspl_tolerance_db"
            }
        ));
    }

    #[test]
    fn sp8072_figure5_spectrum_curve_derives_band_shape() {
        let fixture = include_str!("../../../data/acoustics/sp8072-figure5-spectrum-v1.toml");
        let deck = Sp8072SpectrumCurveDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.sp8072_figure5_spectrum.v1"
        );
        assert_eq!(deck.meta().source_figure_id, "figure_5");
        assert_eq!(
            deck.meta().source_pdf_sha256,
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
        );
        assert_eq!(
            deck.sha256_hex(),
            "0d5c9ed47b94e835562f13e71d3ace4207b5a56b0991bfdbf0b9ebed11bb8b19"
        );
        assert_eq!(deck.curve().points().len(), 18);
        assert_relative_eq!(
            deck.curve().relative_power_db_at(0.01).unwrap(),
            9.9,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().relative_power_db_at(0.04).unwrap(),
            10.073_658_410_747_349,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().relative_power_db_at(0.001).unwrap(),
            0.0,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().relative_power_db_at(10.0).unwrap(),
            -27.0,
            epsilon = 1.0e-12
        );

        let shape = deck.curve().to_spectrum_shape(1.0, 2000.0).unwrap();
        let sum = shape.band_power_fractions().iter().sum::<f64>();
        let max_index = shape
            .band_power_fractions()
            .iter()
            .enumerate()
            .max_by(|lhs, rhs| lhs.1.total_cmp(rhs.1))
            .map(|(index, _)| index)
            .unwrap();

        assert_relative_eq!(sum, 1.0, epsilon = 1.0e-15);
        assert_eq!(shape.bands()[max_index].center_hz, 100.0);
        assert_relative_eq!(
            shape.band_power_fractions()[max_index],
            0.079_685_807_970_345_68,
            epsilon = 1.0e-15
        );
    }

    #[test]
    fn sp8072_figure5_spectrum_curve_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/sp8072-figure5-spectrum-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_sp8072_spectrum_curve = 1",
            "openbmp.acoustics_sp8072_spectrum_curve = 2",
        );
        assert!(matches!(
            Sp8072SpectrumCurveDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_hash = fixture.replace(
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8",
            "bad-hash",
        );
        assert!(matches!(
            Sp8072SpectrumCurveDeck::load_from_str(&bad_hash).unwrap_err(),
            DsmError::InvalidSha256 {
                field: "meta.source_pdf_sha256"
            }
        ));

        let bad_order = fixture.replacen("strouhal = 0.003", "strouhal = 0.002", 1);
        assert!(matches!(
            Sp8072SpectrumCurveDeck::load_from_str(&bad_order).unwrap_err(),
            DsmError::NonIncreasing {
                field: "spectrum.strouhal"
            }
        ));

        let deck = Sp8072SpectrumCurveDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.curve().relative_power_db_at(0.0).unwrap_err(),
            DsmError::NonPositive { field: "strouhal" }
        ));
        assert!(matches!(
            deck.curve().to_spectrum_shape(1.0, 0.0).unwrap_err(),
            DsmError::NonPositive {
                field: "exit_velocity_m_s"
            }
        ));
    }

    #[test]
    fn sp8072_figure12_axial_power_curve_derives_source_allocation() {
        let fixture = include_str!("../../../data/acoustics/sp8072-figure12-source-power-v1.toml");
        let deck = Sp8072AxialPowerCurveDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.sp8072_figure12_source_power.v1"
        );
        assert_eq!(deck.meta().source_figure_id, "figure_12");
        assert_eq!(
            deck.meta().source_pdf_sha256,
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
        );
        assert_eq!(
            deck.sha256_hex(),
            "d199bebbb4bcd217f03f7a8ad717f875aa33419f7dbb2532e3b04649fb848b62"
        );
        assert_eq!(deck.curve().points().len(), 15);
        assert_relative_eq!(
            deck.curve().relative_power_db_at(0.4).unwrap(),
            -10.272_585_139_345_23,
            epsilon = 1.0e-12
        );

        let allocation = deck.curve().to_source_allocation(10.0).unwrap();
        let sources = allocation.sources();
        assert_eq!(sources.len(), 15);
        assert_eq!(sources[0].axial_m, 1.0);
        assert_eq!(sources[14].axial_m, 50.0);
        assert_relative_eq!(
            sources
                .iter()
                .map(|source| source.power_fraction)
                .sum::<f64>(),
            1.0,
            epsilon = 1.0e-15
        );
        assert_relative_eq!(
            sources[8].power_fraction,
            0.199_329_004_722_259_12,
            epsilon = 1.0e-15
        );
        assert_eq!(
            sources
                .iter()
                .max_by(|lhs, rhs| lhs.power_fraction.total_cmp(&rhs.power_fraction))
                .unwrap()
                .axial_m,
            18.0
        );
    }

    #[test]
    fn sp8072_figure12_axial_power_curve_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/sp8072-figure12-source-power-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_sp8072_axial_power_curve = 1",
            "openbmp.acoustics_sp8072_axial_power_curve = 2",
        );
        assert!(matches!(
            Sp8072AxialPowerCurveDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_hash = fixture.replace(
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8",
            "bad-hash",
        );
        assert!(matches!(
            Sp8072AxialPowerCurveDeck::load_from_str(&bad_hash).unwrap_err(),
            DsmError::InvalidSha256 {
                field: "meta.source_pdf_sha256"
            }
        ));

        let bad_order =
            fixture.replacen("x_over_core_length = 0.15", "x_over_core_length = 0.10", 1);
        assert!(matches!(
            Sp8072AxialPowerCurveDeck::load_from_str(&bad_order).unwrap_err(),
            DsmError::NonIncreasing {
                field: "axial_power.x_over_core_length"
            }
        ));

        let deck = Sp8072AxialPowerCurveDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.curve().relative_power_db_at(0.05).unwrap_err(),
            DsmError::OutOfRange {
                field: "x_over_core_length"
            }
        ));
        assert!(matches!(
            deck.curve().to_source_allocation(0.0).unwrap_err(),
            DsmError::NonPositive {
                field: "core_length_m"
            }
        ));
    }

    #[test]
    fn sp8072_figure13_axial_spectrum_curve_feeds_weighted_station_evaluation() {
        let fixture =
            include_str!("../../../data/acoustics/sp8072-figure13-axial-spectrum-v1.toml");
        let deck = Sp8072AxialSpectrumCurveDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.sp8072_figure13_axial_spectrum.v1"
        );
        assert_eq!(deck.meta().source_figure_id, "figure_13");
        assert_eq!(
            deck.meta().source_pdf_sha256,
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
        );
        assert_eq!(
            deck.sha256_hex(),
            "41d90c99e53afe77d1d4503d7bf17df627341d4ac26323ec2a32f601faa318da"
        );
        assert_eq!(deck.curve().points().len(), 12);
        assert_relative_eq!(
            deck.curve().relative_power_db_at(0.2).unwrap(),
            -14.0,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().relative_power_db_at(0.0).unwrap(),
            -20.0,
            epsilon = 1.0e-12
        );

        let shape = toy_shape();
        let allocation = toy_allocation();
        let source_band_weights = deck
            .curve()
            .to_source_band_weights(&allocation, &shape, toy_exit().exit_velocity_m_s)
            .unwrap();
        let total = source_band_weights
            .source_band_power_fractions()
            .iter()
            .flat_map(|row| row.iter())
            .sum::<f64>();

        assert_eq!(
            source_band_weights.source_count(),
            allocation.sources().len()
        );
        assert_eq!(source_band_weights.bands(), shape.bands());
        assert_relative_eq!(total, 1.0, epsilon = 1.0e-15);

        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let weighted = evaluate_source_band_weighted_station(
            toy_exit(),
            &allocation,
            &source_band_weights,
            &directivity,
            &toy_receiver(),
            toy_propagation(),
        )
        .unwrap();
        let baseline = evaluate_free_field_station(
            toy_exit(),
            &allocation,
            &shape,
            &directivity,
            &toy_receiver(),
            toy_propagation(),
        )
        .unwrap();

        assert_eq!(weighted.source_count, allocation.sources().len());
        assert!(weighted.spectrum.oaspl().unwrap().db_re_20upa().is_finite());
        assert!(
            (weighted.spectrum.oaspl().unwrap().db_re_20upa()
                - baseline.spectrum.oaspl().unwrap().db_re_20upa())
            .abs()
                > 1.0e-9
        );
    }

    #[test]
    fn sp8072_figure13_axial_spectrum_curve_helpers_fail_closed() {
        let fixture =
            include_str!("../../../data/acoustics/sp8072-figure13-axial-spectrum-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_sp8072_axial_spectrum_curve = 1",
            "openbmp.acoustics_sp8072_axial_spectrum_curve = 2",
        );
        assert!(matches!(
            Sp8072AxialSpectrumCurveDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_hash = fixture.replace(
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8",
            "bad-hash",
        );
        assert!(matches!(
            Sp8072AxialSpectrumCurveDeck::load_from_str(&bad_hash).unwrap_err(),
            DsmError::InvalidSha256 {
                field: "meta.source_pdf_sha256"
            }
        ));

        let bad_order = fixture.replacen(
            "modified_axial_strouhal = 0.10",
            "modified_axial_strouhal = 0.04",
            1,
        );
        assert!(matches!(
            Sp8072AxialSpectrumCurveDeck::load_from_str(&bad_order).unwrap_err(),
            DsmError::NonIncreasing {
                field: "axial_spectrum.modified_axial_strouhal"
            }
        ));

        let deck = Sp8072AxialSpectrumCurveDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.curve().relative_power_db_at(-0.1).unwrap_err(),
            DsmError::Negative {
                field: "modified_axial_strouhal"
            }
        ));
        assert!(matches!(
            deck.curve()
                .to_source_band_weights(&toy_allocation(), &toy_shape(), 0.0)
                .unwrap_err(),
            DsmError::NonPositive {
                field: "exit_velocity_m_s"
            }
        ));
    }

    #[test]
    fn sp8072_figure10_directivity_curve_derives_band_deck() {
        let fixture = include_str!("../../../data/acoustics/sp8072-figure10-directivity-v1.toml");
        let deck = Sp8072DirectivityCurveDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.sp8072_figure10_directivity.v1"
        );
        assert_eq!(deck.meta().source_figure_id, "figure_10");
        assert_eq!(
            deck.meta().source_pdf_sha256,
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
        );
        assert_eq!(
            deck.sha256_hex(),
            "b374017177dae2cb5028ae7d096d253e9fd300665d05f33dd82d1ce2aaad7894"
        );
        assert_eq!(deck.curve().strouhal_values().len(), 5);
        assert_eq!(deck.curve().angles_deg().len(), 11);
        assert_relative_eq!(
            deck.curve().directivity_db_at(0.04, 90.0).unwrap(),
            -1.75,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().directivity_db_at(0.02, 50.0).unwrap(),
            5.438_368_808_143_029,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().directivity_db_at(1.0, 30.0).unwrap(),
            8.0,
            epsilon = 1.0e-12
        );

        let directivity = deck
            .curve()
            .to_directivity_deck(standard_third_octave_bands(), 1.0, 2000.0)
            .unwrap();
        let band_index = directivity
            .bands()
            .iter()
            .position(|band| band.center_hz.to_bits() == 80.0_f64.to_bits())
            .unwrap();
        let angle_100 = directivity.directivity_db_at(100.0).unwrap();

        assert_eq!(directivity.angles_deg(), deck.curve().angles_deg());
        assert_relative_eq!(angle_100[band_index], -4.5, epsilon = 1.0e-12);
    }

    #[test]
    fn sp8072_figure10_directivity_curve_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/sp8072-figure10-directivity-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_sp8072_directivity_curve = 1",
            "openbmp.acoustics_sp8072_directivity_curve = 2",
        );
        assert!(matches!(
            Sp8072DirectivityCurveDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_hash = fixture.replace(
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8",
            "bad-hash",
        );
        assert!(matches!(
            Sp8072DirectivityCurveDeck::load_from_str(&bad_hash).unwrap_err(),
            DsmError::InvalidSha256 {
                field: "meta.source_pdf_sha256"
            }
        ));

        let bad_strouhal_order = fixture.replacen("0.0125", "0.004", 1);
        assert!(matches!(
            Sp8072DirectivityCurveDeck::load_from_str(&bad_strouhal_order).unwrap_err(),
            DsmError::NonIncreasing {
                field: "directivity.strouhal"
            }
        ));

        let bad_row = fixture.replacen("-17.0", "", 1);
        assert!(matches!(
            Sp8072DirectivityCurveDeck::load_from_str(&bad_row).unwrap_err(),
            DsmError::LengthMismatch { .. }
        ));

        let deck = Sp8072DirectivityCurveDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.curve().directivity_db_at(0.0, 20.0).unwrap_err(),
            DsmError::NonPositive { field: "strouhal" }
        ));
        assert!(matches!(
            deck.curve()
                .to_directivity_deck(standard_third_octave_bands(), 0.0, 2000.0)
                .unwrap_err(),
            DsmError::NonPositive {
                field: "exit_diameter_m"
            }
        ));
    }

    #[test]
    fn sp8072_figure14_apparent_source_curve_feeds_station_evaluation() {
        let fixture =
            include_str!("../../../data/acoustics/sp8072-figure14-apparent-source-v1.toml");
        let deck = Sp8072ApparentSourceCurveDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.sp8072_figure14_apparent_source.v1"
        );
        assert_eq!(deck.meta().source_figure_id, "figure_14");
        assert_eq!(
            deck.meta().source_pdf_sha256,
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8"
        );
        assert_eq!(
            deck.sha256_hex(),
            "5c768b2ce628c07ad2cc8e2a1a6df99e893318ce79757c0675e77bc79f1b6000"
        );
        assert_eq!(deck.curve().points().len(), 7);
        assert_relative_eq!(
            deck.curve().x_over_exit_diameter_at(0.01).unwrap(),
            28.0,
            epsilon = 1.0e-12
        );
        assert_relative_eq!(
            deck.curve().axial_m_at(0.02, 2.0).unwrap(),
            50.0,
            epsilon = 1.0e-12
        );

        let shape = toy_shape();
        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let station = evaluate_apparent_source_station(
            toy_exit(),
            &shape,
            deck.curve(),
            &directivity,
            &toy_receiver(),
            toy_propagation(),
        )
        .unwrap();

        assert_eq!(station.source_count, shape.bands().len());
        assert!(station.spectrum.oaspl().unwrap().db_re_20upa().is_finite());
    }

    #[test]
    fn sp8072_figure14_apparent_source_curve_helpers_fail_closed() {
        let fixture =
            include_str!("../../../data/acoustics/sp8072-figure14-apparent-source-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_sp8072_apparent_source_curve = 1",
            "openbmp.acoustics_sp8072_apparent_source_curve = 2",
        );
        assert!(matches!(
            Sp8072ApparentSourceCurveDeck::load_from_str(&bad_schema).unwrap_err(),
            DsmError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_hash = fixture.replace(
            "84f1483c2c8477810186e98765ea99e4b7f7cc71e0a47eb8f86977d3b02aabf8",
            "bad-hash",
        );
        assert!(matches!(
            Sp8072ApparentSourceCurveDeck::load_from_str(&bad_hash).unwrap_err(),
            DsmError::InvalidSha256 {
                field: "meta.source_pdf_sha256"
            }
        ));

        let bad_order = fixture.replacen("strouhal = 0.005", "strouhal = 0.002", 1);
        assert!(matches!(
            Sp8072ApparentSourceCurveDeck::load_from_str(&bad_order).unwrap_err(),
            DsmError::NonIncreasing {
                field: "apparent_source.strouhal"
            }
        ));

        let bad_location = fixture.replacen(
            "x_over_exit_diameter = 30.0",
            "x_over_exit_diameter = 0.0",
            1,
        );
        assert!(matches!(
            Sp8072ApparentSourceCurveDeck::load_from_str(&bad_location).unwrap_err(),
            DsmError::NonPositive {
                field: "apparent_source.x_over_exit_diameter"
            }
        ));

        let deck = Sp8072ApparentSourceCurveDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.curve().x_over_exit_diameter_at(0.0).unwrap_err(),
            DsmError::NonPositive { field: "strouhal" }
        ));
        assert!(matches!(
            deck.curve().axial_m_at(0.01, 0.0).unwrap_err(),
            DsmError::NonPositive {
                field: "exit_diameter_m"
            }
        ));
    }

    #[test]
    fn dsm_helpers_fail_closed_on_bad_inputs() {
        let bad_exit = DsmExitCondition {
            acoustic_efficiency: 2.0,
            ..toy_exit()
        }
        .validate()
        .unwrap_err();
        assert!(matches!(bad_exit, DsmError::InvalidFraction { .. }));

        let bad_sources = DsmSourceAllocation::normalized_from_weights(&[(0.0, 0.0)]).unwrap_err();
        assert!(matches!(bad_sources, DsmError::NonPositive { .. }));

        let bad_receiver = DsmReceiver::new("", 0.0, 1.0).unwrap_err();
        assert!(matches!(bad_receiver, DsmError::Empty { .. }));

        let bad_absorption = absorption_loss_db(-1.0, 1.0).unwrap_err();
        assert!(matches!(bad_absorption, DsmError::Negative { .. }));

        let shape = toy_shape();
        let directivity = DsmDirectivityDeck::omnidirectional(shape.bands().to_vec()).unwrap();
        let bad_trace = evaluate_free_field_liftoff_trace(
            toy_exit(),
            &toy_allocation(),
            &shape,
            &directivity,
            &toy_receiver(),
            toy_propagation(),
            &[
                DsmLiftoffStep::new(1.0, 0.0).unwrap(),
                DsmLiftoffStep::new(1.0, 1.0).unwrap(),
            ],
        )
        .unwrap_err();
        assert!(matches!(bad_trace, DsmError::NonIncreasing { .. }));
    }
}
