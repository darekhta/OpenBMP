//! Pad acoustic environment modifiers.
//!
//! This module holds deterministic, provenance-pinned pad acoustic decks used
//! by runner-side acoustics reports. It intentionally stays independent from
//! the simulation force path.

use std::collections::BTreeSet;
use std::fmt::Write as _;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

use crate::{AcousticBand, AcousticsError};

/// Schema version for pad acoustic suppression decks.
pub const PAD_SUPPRESSION_DECK_SCHEMA_VERSION: u32 = 1;
/// Schema version for pad cluster acoustic interaction decks.
pub const PAD_CLUSTER_INTERACTION_DECK_SCHEMA_VERSION: u32 = 1;

/// Errors returned by pad acoustic helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum PadAcousticsError {
    /// A lower-level acoustic value was invalid.
    #[error(transparent)]
    Acoustics(#[from] AcousticsError),
    /// A scalar was non-finite.
    #[error("{field} must be finite")]
    NonFinite {
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
    #[error("unknown pad acoustics class {class_id}")]
    UnknownClass {
        /// Requested class id.
        class_id: String,
    },
    /// A deck schema version is unsupported.
    #[error("unsupported pad acoustics deck schema version {schema_version}")]
    UnsupportedDeckSchema {
        /// Parsed schema version.
        schema_version: u32,
    },
    /// A deck could not be parsed.
    #[error("malformed pad acoustics deck: {reason}")]
    MalformedDeck {
        /// Parse or structural failure.
        reason: String,
    },
}

/// Provenance metadata for a pad suppression deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct PadSuppressionDeckMeta {
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

impl PadSuppressionDeckMeta {
    fn from_row(row: PadSuppressionDeckMetaRow) -> Result<Self, PadAcousticsError> {
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

/// One water/pad suppression class in a deck.
#[derive(Clone, Debug, PartialEq)]
pub struct PadSuppressionClass {
    /// Stable class id referenced by scenario files.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Per-band positive knockdown in dB.
    pub knockdown_db: Vec<f64>,
}

impl PadSuppressionClass {
    fn from_row(row: PadSuppressionClassRow, band_count: usize) -> Result<Self, PadAcousticsError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        require_non_negative("classes.uncertainty_db", row.uncertainty_db)?;
        if row.knockdown_db.len() != band_count {
            return Err(PadAcousticsError::LengthMismatch {
                bands: band_count,
                values: row.knockdown_db.len(),
            });
        }
        for value in &row.knockdown_db {
            require_non_negative("classes.knockdown_db", *value)?;
        }
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            uncertainty_db: row.uncertainty_db,
            knockdown_db: row.knockdown_db,
        })
    }
}

/// Provenance-pinned pad acoustic suppression deck.
#[derive(Clone, Debug, PartialEq)]
pub struct PadSuppressionDeck {
    meta: PadSuppressionDeckMeta,
    bands: Vec<AcousticBand>,
    classes: Vec<PadSuppressionClass>,
    sha256_hex: String,
}

impl PadSuppressionDeck {
    /// Parse a Schema-1 pad acoustic suppression deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`PadAcousticsError`] when TOML parsing fails, the schema marker
    /// is unsupported, provenance fields are incomplete, or band/class rows are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, PadAcousticsError> {
        let parsed: PadSuppressionDeckFile =
            toml::from_str(input).map_err(|error| PadAcousticsError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_pad_suppression_deck != PAD_SUPPRESSION_DECK_SCHEMA_VERSION {
            return Err(PadAcousticsError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_pad_suppression_deck,
            });
        }
        let bands = bands_from_center_hz(parsed.suppression.center_hz, "suppression.center_hz")?;
        let classes = suppression_classes_from_rows(parsed.classes, bands.len())?;
        Ok(Self {
            meta: PadSuppressionDeckMeta::from_row(parsed.meta)?,
            bands,
            classes,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &PadSuppressionDeckMeta {
        &self.meta
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Suppression classes.
    #[must_use]
    pub fn classes(&self) -> &[PadSuppressionClass] {
        &self.classes
    }

    /// Lookup a suppression class by id.
    ///
    /// # Errors
    ///
    /// Returns [`PadAcousticsError::UnknownClass`] when the class id is absent.
    pub fn class(&self, class_id: &str) -> Result<&PadSuppressionClass, PadAcousticsError> {
        let class_id = class_id.trim();
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| PadAcousticsError::UnknownClass {
                class_id: class_id.to_owned(),
            })
    }

    /// Per-band negative transfer values for a suppression class.
    ///
    /// # Errors
    ///
    /// Returns [`PadAcousticsError::UnknownClass`] when the class id is absent.
    pub fn transfer_db_for_class(&self, class_id: &str) -> Result<Vec<f64>, PadAcousticsError> {
        Ok(self
            .class(class_id)?
            .knockdown_db
            .iter()
            .map(|value| -*value)
            .collect())
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

/// Provenance metadata for a pad cluster interaction deck.
pub type PadClusterInteractionDeckMeta = PadSuppressionDeckMeta;

/// One cluster interaction class in a deck.
#[derive(Clone, Debug, PartialEq)]
pub struct PadClusterInteractionClass {
    /// Stable class id referenced by scenario files.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Declared one-sigma uncertainty, dB.
    pub uncertainty_db: f64,
    /// Per-band cross-engine pressure correlation coefficient in `[0, 1]`.
    pub correlation_coefficient: Vec<f64>,
    /// Per-band broadband-equivalent shielding loss in dB.
    pub shielding_loss_db: Vec<f64>,
}

impl PadClusterInteractionClass {
    fn from_row(
        row: PadClusterInteractionClassRow,
        band_count: usize,
    ) -> Result<Self, PadAcousticsError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        require_non_negative("classes.uncertainty_db", row.uncertainty_db)?;
        if row.correlation_coefficient.len() != band_count {
            return Err(PadAcousticsError::LengthMismatch {
                bands: band_count,
                values: row.correlation_coefficient.len(),
            });
        }
        if row.shielding_loss_db.len() != band_count {
            return Err(PadAcousticsError::LengthMismatch {
                bands: band_count,
                values: row.shielding_loss_db.len(),
            });
        }
        for value in &row.correlation_coefficient {
            require_fraction("classes.correlation_coefficient", *value)?;
        }
        for value in &row.shielding_loss_db {
            require_non_negative("classes.shielding_loss_db", *value)?;
        }
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            uncertainty_db: row.uncertainty_db,
            correlation_coefficient: row.correlation_coefficient,
            shielding_loss_db: row.shielding_loss_db,
        })
    }
}

/// Provenance-pinned pad cluster acoustic interaction deck.
#[derive(Clone, Debug, PartialEq)]
pub struct PadClusterInteractionDeck {
    meta: PadClusterInteractionDeckMeta,
    bands: Vec<AcousticBand>,
    classes: Vec<PadClusterInteractionClass>,
    sha256_hex: String,
}

impl PadClusterInteractionDeck {
    /// Parse a Schema-1 pad cluster interaction deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`PadAcousticsError`] when TOML parsing fails, the schema marker
    /// is unsupported, provenance fields are incomplete, or band/class rows are
    /// malformed.
    pub fn load_from_str(input: &str) -> Result<Self, PadAcousticsError> {
        let parsed: PadClusterInteractionDeckFile =
            toml::from_str(input).map_err(|error| PadAcousticsError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_pad_cluster_interaction_deck
            != PAD_CLUSTER_INTERACTION_DECK_SCHEMA_VERSION
        {
            return Err(PadAcousticsError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_pad_cluster_interaction_deck,
            });
        }
        let bands = bands_from_center_hz(
            parsed.cluster_interaction.center_hz,
            "cluster_interaction.center_hz",
        )?;
        let classes = cluster_interaction_classes_from_rows(parsed.classes, bands.len())?;
        Ok(Self {
            meta: PadClusterInteractionDeckMeta::from_row(parsed.meta)?,
            bands,
            classes,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &PadClusterInteractionDeckMeta {
        &self.meta
    }

    /// Band grid.
    #[must_use]
    pub fn bands(&self) -> &[AcousticBand] {
        &self.bands
    }

    /// Cluster interaction classes.
    #[must_use]
    pub fn classes(&self) -> &[PadClusterInteractionClass] {
        &self.classes
    }

    /// Lookup a cluster interaction class by id.
    ///
    /// # Errors
    ///
    /// Returns [`PadAcousticsError::UnknownClass`] when the class id is absent.
    pub fn class(&self, class_id: &str) -> Result<&PadClusterInteractionClass, PadAcousticsError> {
        let class_id = class_id.trim();
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| PadAcousticsError::UnknownClass {
                class_id: class_id.to_owned(),
            })
    }

    /// SHA-256 digest of the exact TOML bytes passed to [`Self::load_from_str`].
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadSuppressionDeckFile {
    openbmp: PadSuppressionSchemaMarker,
    meta: PadSuppressionDeckMetaRow,
    suppression: PadSuppressionSection,
    classes: Vec<PadSuppressionClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadSuppressionSchemaMarker {
    acoustics_pad_suppression_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadSuppressionDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadSuppressionSection {
    center_hz: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadSuppressionClassRow {
    class_id: String,
    description: String,
    uncertainty_db: f64,
    knockdown_db: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadClusterInteractionDeckFile {
    openbmp: PadClusterInteractionSchemaMarker,
    meta: PadSuppressionDeckMetaRow,
    cluster_interaction: PadClusterInteractionSection,
    classes: Vec<PadClusterInteractionClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadClusterInteractionSchemaMarker {
    acoustics_pad_cluster_interaction_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadClusterInteractionSection {
    center_hz: Vec<f64>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct PadClusterInteractionClassRow {
    class_id: String,
    description: String,
    uncertainty_db: f64,
    correlation_coefficient: Vec<f64>,
    shielding_loss_db: Vec<f64>,
}

fn bands_from_center_hz(
    values: Vec<f64>,
    field: &'static str,
) -> Result<Vec<AcousticBand>, PadAcousticsError> {
    if values.is_empty() {
        return Err(PadAcousticsError::Empty { field });
    }
    let mut previous = None;
    let mut bands = Vec::with_capacity(values.len());
    for value in values {
        if previous.is_some_and(|previous| value <= previous) {
            return Err(PadAcousticsError::NonIncreasing { field });
        }
        previous = Some(value);
        bands.push(AcousticBand::one_third_octave(value)?);
    }
    Ok(bands)
}

fn suppression_classes_from_rows(
    rows: Vec<PadSuppressionClassRow>,
    band_count: usize,
) -> Result<Vec<PadSuppressionClass>, PadAcousticsError> {
    if rows.is_empty() {
        return Err(PadAcousticsError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = PadSuppressionClass::from_row(row, band_count)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(PadAcousticsError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn cluster_interaction_classes_from_rows(
    rows: Vec<PadClusterInteractionClassRow>,
    band_count: usize,
) -> Result<Vec<PadClusterInteractionClass>, PadAcousticsError> {
    if rows.is_empty() {
        return Err(PadAcousticsError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = PadClusterInteractionClass::from_row(row, band_count)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(PadAcousticsError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn require_finite(field: &'static str, value: f64) -> Result<(), PadAcousticsError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(PadAcousticsError::NonFinite { field })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), PadAcousticsError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(PadAcousticsError::Negative { field })
    }
}

fn require_fraction(field: &'static str, value: f64) -> Result<(), PadAcousticsError> {
    require_finite(field, value)?;
    if (0.0..=1.0).contains(&value) {
        Ok(())
    } else {
        Err(PadAcousticsError::InvalidFraction { field })
    }
}

fn require_text(field: &'static str, value: &str) -> Result<(), PadAcousticsError> {
    if value.trim().is_empty() {
        Err(PadAcousticsError::Empty { field })
    } else {
        Ok(())
    }
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in digest {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;

    #[test]
    fn pad_suppression_deck_parses_fixture_and_derives_transfer() {
        let fixture = include_str!("../../../data/acoustics/pad-suppression-v1.toml");
        let deck = PadSuppressionDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.pad_suppression.v1"
        );
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(deck.bands().len(), 31);
        assert_eq!(deck.classes().len(), 2);
        assert_eq!(
            deck.sha256_hex(),
            "4888aeb8dde08333638d7d1127d864ea544a1a2ab78e4eabd95ad3d0b42cf932"
        );

        let dry_transfer = deck.transfer_db_for_class("dry").unwrap();
        assert_eq!(dry_transfer[0], -0.0);
        let water = deck.class("water_nominal").unwrap();
        let water_transfer = deck.transfer_db_for_class("water_nominal").unwrap();

        assert_relative_eq!(water.uncertainty_db, 3.0, epsilon = 1.0e-12);
        assert_relative_eq!(water.knockdown_db[0], 1.0, epsilon = 1.0e-12);
        assert_relative_eq!(water.knockdown_db[30], 8.0, epsilon = 1.0e-12);
        assert_relative_eq!(water_transfer[30], -8.0, epsilon = 1.0e-12);
    }

    #[test]
    fn pad_suppression_deck_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/pad-suppression-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_pad_suppression_deck = 1",
            "openbmp.acoustics_pad_suppression_deck = 2",
        );
        assert!(matches!(
            PadSuppressionDeck::load_from_str(&bad_schema).unwrap_err(),
            PadAcousticsError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let bad_source_class = fixture.replace(
            "source_class = \"synthetic-openbmp\"",
            "source_class = \"\"",
        );
        assert!(matches!(
            PadSuppressionDeck::load_from_str(&bad_source_class).unwrap_err(),
            PadAcousticsError::Empty {
                field: "meta.source_class"
            }
        ));

        let duplicate_class =
            fixture.replacen("class_id = \"water_nominal\"", "class_id = \"dry\"", 1);
        assert!(matches!(
            PadSuppressionDeck::load_from_str(&duplicate_class).unwrap_err(),
            PadAcousticsError::Duplicate {
                field: "classes.class_id"
            }
        ));

        let negative_knockdown = fixture.replacen("  1.0,", "  -1.0,", 1);
        assert!(matches!(
            PadSuppressionDeck::load_from_str(&negative_knockdown).unwrap_err(),
            PadAcousticsError::Negative {
                field: "classes.knockdown_db"
            }
        ));

        let deck = PadSuppressionDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.transfer_db_for_class("unknown").unwrap_err(),
            PadAcousticsError::UnknownClass { .. }
        ));
    }

    #[test]
    fn pad_cluster_interaction_deck_parses_fixture() {
        let fixture = include_str!("../../../data/acoustics/pad-cluster-interaction-v1.toml");
        let deck = PadClusterInteractionDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.pad_cluster_interaction.v1"
        );
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(deck.bands().len(), 31);
        assert_eq!(deck.classes().len(), 2);
        assert_eq!(
            deck.sha256_hex(),
            "301aaf420d9231834a89962184b8e57e9a3117a704f556f61a4eb1ab0c836b37"
        );

        let coherent = deck.class("coherent_shielded").unwrap();
        assert_relative_eq!(coherent.uncertainty_db, 2.0, epsilon = 1.0e-12);
        assert_relative_eq!(coherent.correlation_coefficient[0], 1.0, epsilon = 1.0e-12);
        assert_relative_eq!(coherent.shielding_loss_db[30], 1.5, epsilon = 1.0e-12);
    }

    #[test]
    fn pad_cluster_interaction_deck_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/pad-cluster-interaction-v1.toml");

        let bad_schema = fixture.replace(
            "openbmp.acoustics_pad_cluster_interaction_deck = 1",
            "openbmp.acoustics_pad_cluster_interaction_deck = 2",
        );
        assert!(matches!(
            PadClusterInteractionDeck::load_from_str(&bad_schema).unwrap_err(),
            PadAcousticsError::UnsupportedDeckSchema { schema_version: 2 }
        ));

        let duplicate_class = fixture.replacen(
            "class_id = \"coherent_shielded\"",
            "class_id = \"incoherent_open\"",
            1,
        );
        assert!(matches!(
            PadClusterInteractionDeck::load_from_str(&duplicate_class).unwrap_err(),
            PadAcousticsError::Duplicate {
                field: "classes.class_id"
            }
        ));

        let bad_correlation = fixture.replacen("  1.0,", "  1.1,", 1);
        assert!(matches!(
            PadClusterInteractionDeck::load_from_str(&bad_correlation).unwrap_err(),
            PadAcousticsError::InvalidFraction {
                field: "classes.correlation_coefficient"
            }
        ));

        let negative_shielding = fixture.replacen("  1.5,", "  -1.5,", 1);
        assert!(matches!(
            PadClusterInteractionDeck::load_from_str(&negative_shielding).unwrap_err(),
            PadAcousticsError::Negative {
                field: "classes.shielding_loss_db"
            }
        ));

        let deck = PadClusterInteractionDeck::load_from_str(fixture).unwrap();
        assert!(matches!(
            deck.class("unknown").unwrap_err(),
            PadAcousticsError::UnknownClass { .. }
        ));
    }
}
