//! Ignition-overpressure engineering pulse primitives.
//!
//! This module is a deterministic, low-order substrate for WP-22.4. It models
//! source strength from the start-transient rate of combustion-gas generation,
//! duct/trench residence-time coupling, inverse-square receiver decay,
//! optional pressure-amplitude suppression, and declared uncertainty bounds.
//! It is not a facility-calibrated IOP reconstruction.

use std::collections::BTreeSet;
use std::f64::consts::PI;
use std::fmt::Write as _;

use serde::Deserialize;
use sha2::{Digest, Sha256};
use thiserror::Error;

/// Schema version for provenance-pinned IOP duct/trench decks.
pub const IOP_DUCT_DECK_SCHEMA_VERSION: u32 = 1;
/// Schema version for provenance-pinned IOP suppression decks.
pub const IOP_SUPPRESSION_DECK_SCHEMA_VERSION: u32 = 1;

/// Errors returned by ignition-overpressure helpers.
#[derive(Clone, Debug, Error, PartialEq, Eq)]
pub enum IopError {
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
    /// A digest was not a lowercase SHA-256 hex string.
    #[error("{field} must be a lowercase SHA-256 hex digest")]
    InvalidSha256 {
        /// Field name.
        field: &'static str,
    },
    /// A requested class does not exist in the deck.
    #[error("unknown IOP class {class_id}")]
    UnknownClass {
        /// Requested class id.
        class_id: String,
    },
    /// A deck schema version is unsupported.
    #[error("unsupported IOP deck schema version {schema_version}")]
    UnsupportedDeckSchema {
        /// Parsed schema version.
        schema_version: u32,
    },
    /// A deck could not be parsed.
    #[error("malformed IOP deck: {reason}")]
    MalformedDeck {
        /// Parse or structural failure.
        reason: String,
    },
}

/// Provenance metadata for an IOP duct/trench deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IopDuctDeckMeta {
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

impl IopDuctDeckMeta {
    fn from_row(row: IopDuctDeckMetaRow) -> Result<Self, IopError> {
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

/// One duct/trench geometry class in an IOP duct deck.
#[derive(Clone, Debug, PartialEq)]
pub struct IopDuctClass {
    /// Stable class id referenced by scenario files.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Duct/trench coupling values.
    pub duct: IopDuctCoupling,
}

impl IopDuctClass {
    fn from_row(row: IopDuctClassRow) -> Result<Self, IopError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            duct: IopDuctCoupling::new(
                row.volume_m3,
                row.exit_area_m2,
                row.discharge_velocity_m_s,
                row.geometry_gain,
            )?,
        })
    }
}

/// Provenance-pinned IOP duct/trench geometry deck.
#[derive(Clone, Debug, PartialEq)]
pub struct IopDuctDeck {
    meta: IopDuctDeckMeta,
    classes: Vec<IopDuctClass>,
    sha256_hex: String,
}

impl IopDuctDeck {
    /// Parse a Schema-1 IOP duct/trench deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`IopError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are empty, class ids are duplicated, or
    /// duct coupling fields are invalid.
    pub fn load_from_str(input: &str) -> Result<Self, IopError> {
        let parsed: IopDuctDeckFile =
            toml::from_str(input).map_err(|error| IopError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_iop_duct_deck != IOP_DUCT_DECK_SCHEMA_VERSION {
            return Err(IopError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_iop_duct_deck,
            });
        }
        let classes = duct_classes_from_rows(parsed.classes)?;
        Ok(Self {
            meta: IopDuctDeckMeta::from_row(parsed.meta)?,
            classes,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &IopDuctDeckMeta {
        &self.meta
    }

    /// Declared duct/trench classes.
    #[must_use]
    pub fn classes(&self) -> &[IopDuctClass] {
        &self.classes
    }

    /// Exact SHA-256 digest of the input TOML text.
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }

    /// Look up a duct/trench class by id.
    ///
    /// # Errors
    ///
    /// Returns [`IopError::UnknownClass`] when `class_id` is not present.
    pub fn class(&self, class_id: &str) -> Result<&IopDuctClass, IopError> {
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| IopError::UnknownClass {
                class_id: class_id.to_owned(),
            })
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopDuctDeckFile {
    openbmp: IopDuctSchemaMarker,
    meta: IopDuctDeckMetaRow,
    classes: Vec<IopDuctClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopDuctSchemaMarker {
    acoustics_iop_duct_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopDuctDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopDuctClassRow {
    class_id: String,
    description: String,
    volume_m3: f64,
    exit_area_m2: f64,
    discharge_velocity_m_s: f64,
    geometry_gain: f64,
}

/// Provenance metadata for an IOP suppression deck.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct IopSuppressionDeckMeta {
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

impl IopSuppressionDeckMeta {
    fn from_row(row: IopSuppressionDeckMetaRow) -> Result<Self, IopError> {
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

/// One pressure-amplitude suppression class in an IOP suppression deck.
#[derive(Clone, Debug, PartialEq)]
pub struct IopSuppressionClass {
    /// Stable class id referenced by scenario files.
    pub class_id: String,
    /// Human-readable class description.
    pub description: String,
    /// Pressure-amplitude knockdown applied by this class, dB.
    pub suppression_loss_db: f64,
    /// Declared class uncertainty, dB.
    pub uncertainty_db: f64,
}

impl IopSuppressionClass {
    fn from_row(row: IopSuppressionClassRow) -> Result<Self, IopError> {
        require_text("classes.class_id", &row.class_id)?;
        require_text("classes.description", &row.description)?;
        require_non_negative("classes.suppression_loss_db", row.suppression_loss_db)?;
        require_non_negative("classes.uncertainty_db", row.uncertainty_db)?;
        Ok(Self {
            class_id: row.class_id.trim().to_owned(),
            description: row.description.trim().to_owned(),
            suppression_loss_db: row.suppression_loss_db,
            uncertainty_db: row.uncertainty_db,
        })
    }
}

/// Provenance-pinned IOP suppression deck.
#[derive(Clone, Debug, PartialEq)]
pub struct IopSuppressionDeck {
    meta: IopSuppressionDeckMeta,
    classes: Vec<IopSuppressionClass>,
    sha256_hex: String,
}

impl IopSuppressionDeck {
    /// Parse a Schema-1 IOP suppression deck from TOML text.
    ///
    /// # Errors
    ///
    /// Returns [`IopError`] when TOML parsing fails, the schema marker is
    /// unsupported, provenance fields are empty, class ids are duplicated, or
    /// suppression fields are invalid.
    pub fn load_from_str(input: &str) -> Result<Self, IopError> {
        let parsed: IopSuppressionDeckFile =
            toml::from_str(input).map_err(|error| IopError::MalformedDeck {
                reason: error.to_string(),
            })?;
        if parsed.openbmp.acoustics_iop_suppression_deck != IOP_SUPPRESSION_DECK_SCHEMA_VERSION {
            return Err(IopError::UnsupportedDeckSchema {
                schema_version: parsed.openbmp.acoustics_iop_suppression_deck,
            });
        }
        let classes = suppression_classes_from_rows(parsed.classes)?;
        Ok(Self {
            meta: IopSuppressionDeckMeta::from_row(parsed.meta)?,
            classes,
            sha256_hex: sha256_hex(input.as_bytes()),
        })
    }

    /// Deck provenance metadata.
    #[must_use]
    pub fn meta(&self) -> &IopSuppressionDeckMeta {
        &self.meta
    }

    /// Declared suppression classes.
    #[must_use]
    pub fn classes(&self) -> &[IopSuppressionClass] {
        &self.classes
    }

    /// Exact SHA-256 digest of the input TOML text.
    #[must_use]
    pub fn sha256_hex(&self) -> &str {
        &self.sha256_hex
    }

    /// Look up a suppression class by id.
    ///
    /// # Errors
    ///
    /// Returns [`IopError::UnknownClass`] when `class_id` is not present.
    pub fn class(&self, class_id: &str) -> Result<&IopSuppressionClass, IopError> {
        self.classes
            .iter()
            .find(|class| class.class_id == class_id)
            .ok_or_else(|| IopError::UnknownClass {
                class_id: class_id.to_owned(),
            })
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopSuppressionDeckFile {
    openbmp: IopSuppressionSchemaMarker,
    meta: IopSuppressionDeckMetaRow,
    classes: Vec<IopSuppressionClassRow>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopSuppressionSchemaMarker {
    acoustics_iop_suppression_deck: u32,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopSuppressionDeckMetaRow {
    dataset_id: String,
    source_class: String,
    source_title: String,
    provenance: String,
    license_or_terms: String,
    validation: String,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct IopSuppressionClassRow {
    class_id: String,
    description: String,
    suppression_loss_db: f64,
    uncertainty_db: f64,
}

/// Start-transient source term for the IOP pulse.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IopStartTransient {
    gas_generation_rate_n_per_s: f64,
    ramp_duration_s: f64,
}

impl IopStartTransient {
    /// Construct a start-transient source.
    ///
    /// `gas_generation_rate_n_per_s` represents `d(mdot * c*) / dt`; it is the
    /// source-rate scaling term used by this low-order pulse model.
    ///
    /// # Errors
    ///
    /// Returns [`IopError`] if the source rate is not positive or the ramp
    /// duration is negative/non-finite.
    pub fn new(gas_generation_rate_n_per_s: f64, ramp_duration_s: f64) -> Result<Self, IopError> {
        require_positive("gas_generation_rate_n_per_s", gas_generation_rate_n_per_s)?;
        require_non_negative("ramp_duration_s", ramp_duration_s)?;
        Ok(Self {
            gas_generation_rate_n_per_s,
            ramp_duration_s,
        })
    }

    /// Source-rate scaling term, N/s.
    #[must_use]
    pub const fn gas_generation_rate_n_per_s(self) -> f64 {
        self.gas_generation_rate_n_per_s
    }

    /// Start-ramp duration, s.
    #[must_use]
    pub const fn ramp_duration_s(self) -> f64 {
        self.ramp_duration_s
    }
}

/// Duct/trench coupling term for the IOP pulse.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IopDuctCoupling {
    volume_m3: f64,
    exit_area_m2: f64,
    discharge_velocity_m_s: f64,
    geometry_gain: f64,
}

impl IopDuctCoupling {
    /// Construct a duct/trench coupling term.
    ///
    /// The residence time is `volume / (exit_area * discharge_velocity)`;
    /// `geometry_gain` is a non-negative empirical multiplier for declared
    /// duct/trench geometry.
    ///
    /// # Errors
    ///
    /// Returns [`IopError`] if the geometric terms are not positive or the
    /// gain is negative/non-finite.
    pub fn new(
        volume_m3: f64,
        exit_area_m2: f64,
        discharge_velocity_m_s: f64,
        geometry_gain: f64,
    ) -> Result<Self, IopError> {
        require_positive("volume_m3", volume_m3)?;
        require_positive("exit_area_m2", exit_area_m2)?;
        require_positive("discharge_velocity_m_s", discharge_velocity_m_s)?;
        require_non_negative("geometry_gain", geometry_gain)?;
        Ok(Self {
            volume_m3,
            exit_area_m2,
            discharge_velocity_m_s,
            geometry_gain,
        })
    }

    /// Duct/trench volume, m^3.
    #[must_use]
    pub const fn volume_m3(self) -> f64 {
        self.volume_m3
    }

    /// Effective vent/exit area, m^2.
    #[must_use]
    pub const fn exit_area_m2(self) -> f64 {
        self.exit_area_m2
    }

    /// Effective discharge velocity, m/s.
    #[must_use]
    pub const fn discharge_velocity_m_s(self) -> f64 {
        self.discharge_velocity_m_s
    }

    /// Empirical geometry gain.
    #[must_use]
    pub const fn geometry_gain(self) -> f64 {
        self.geometry_gain
    }

    /// Residence-time coupling term, s.
    #[must_use]
    pub fn residence_time_s(self) -> f64 {
        self.volume_m3 / (self.exit_area_m2 * self.discharge_velocity_m_s)
    }
}

/// Declared one-sided dB uncertainty on an IOP peak-pressure prediction.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IopUncertainty {
    minus_db: f64,
    plus_db: f64,
}

impl IopUncertainty {
    /// Construct a dB uncertainty band.
    ///
    /// # Errors
    ///
    /// Returns [`IopError`] if either uncertainty side is negative/non-finite.
    pub fn new(minus_db: f64, plus_db: f64) -> Result<Self, IopError> {
        require_non_negative("minus_db", minus_db)?;
        require_non_negative("plus_db", plus_db)?;
        Ok(Self { minus_db, plus_db })
    }

    /// Lower-side uncertainty in dB.
    #[must_use]
    pub const fn minus_db(self) -> f64 {
        self.minus_db
    }

    /// Upper-side uncertainty in dB.
    #[must_use]
    pub const fn plus_db(self) -> f64 {
        self.plus_db
    }
}

/// Inputs to the low-order IOP pulse model.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IopPulseInput {
    source: IopStartTransient,
    duct: IopDuctCoupling,
    receiver_distance_m: f64,
    suppression_loss_db: f64,
    uncertainty: IopUncertainty,
}

impl IopPulseInput {
    /// Construct pulse-model inputs.
    ///
    /// # Errors
    ///
    /// Returns [`IopError`] if receiver distance is not positive or suppression
    /// loss is negative/non-finite.
    pub fn new(
        source: IopStartTransient,
        duct: IopDuctCoupling,
        receiver_distance_m: f64,
        suppression_loss_db: f64,
        uncertainty: IopUncertainty,
    ) -> Result<Self, IopError> {
        require_positive("receiver_distance_m", receiver_distance_m)?;
        require_non_negative("suppression_loss_db", suppression_loss_db)?;
        Ok(Self {
            source,
            duct,
            receiver_distance_m,
            suppression_loss_db,
            uncertainty,
        })
    }

    /// Start-transient source.
    #[must_use]
    pub const fn source(self) -> IopStartTransient {
        self.source
    }

    /// Duct/trench coupling.
    #[must_use]
    pub const fn duct(self) -> IopDuctCoupling {
        self.duct
    }

    /// Receiver distance, m.
    #[must_use]
    pub const fn receiver_distance_m(self) -> f64 {
        self.receiver_distance_m
    }

    /// Pressure-amplitude suppression loss, dB.
    #[must_use]
    pub const fn suppression_loss_db(self) -> f64 {
        self.suppression_loss_db
    }

    /// Declared uncertainty.
    #[must_use]
    pub const fn uncertainty(self) -> IopUncertainty {
        self.uncertainty
    }
}

/// IOP peak-pressure and duration envelope.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct IopPulseEnvelope {
    peak_overpressure_pa: f64,
    lower_peak_overpressure_pa: f64,
    upper_peak_overpressure_pa: f64,
    duration_s: f64,
}

impl IopPulseEnvelope {
    /// Nominal peak overpressure, Pa.
    #[must_use]
    pub const fn peak_overpressure_pa(self) -> f64 {
        self.peak_overpressure_pa
    }

    /// Lower uncertainty-bound peak overpressure, Pa.
    #[must_use]
    pub const fn lower_peak_overpressure_pa(self) -> f64 {
        self.lower_peak_overpressure_pa
    }

    /// Upper uncertainty-bound peak overpressure, Pa.
    #[must_use]
    pub const fn upper_peak_overpressure_pa(self) -> f64 {
        self.upper_peak_overpressure_pa
    }

    /// Pulse-duration envelope, s.
    #[must_use]
    pub const fn duration_s(self) -> f64 {
        self.duration_s
    }
}

/// Evaluate the low-order IOP pulse model.
#[must_use]
pub fn evaluate_iop_pulse(input: IopPulseInput) -> IopPulseEnvelope {
    let residence_time_s = input.duct.residence_time_s();
    let spherical_area_m2 = 4.0 * PI * input.receiver_distance_m * input.receiver_distance_m;
    let unsuppressed_peak_pa =
        input.source.gas_generation_rate_n_per_s() * residence_time_s * input.duct.geometry_gain()
            / spherical_area_m2;
    let peak_overpressure_pa =
        unsuppressed_peak_pa * pressure_ratio_from_db(-input.suppression_loss_db);
    let lower_peak_overpressure_pa =
        peak_overpressure_pa * pressure_ratio_from_db(-input.uncertainty.minus_db);
    let upper_peak_overpressure_pa =
        peak_overpressure_pa * pressure_ratio_from_db(input.uncertainty.plus_db);
    IopPulseEnvelope {
        peak_overpressure_pa,
        lower_peak_overpressure_pa,
        upper_peak_overpressure_pa,
        duration_s: input.source.ramp_duration_s() + residence_time_s,
    }
}

#[must_use]
fn pressure_ratio_from_db(db: f64) -> f64 {
    10.0_f64.powf(db / 20.0)
}

fn require_finite(field: &'static str, value: f64) -> Result<(), IopError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(IopError::NonFinite { field })
    }
}

fn require_positive(field: &'static str, value: f64) -> Result<(), IopError> {
    require_finite(field, value)?;
    if value > 0.0 {
        Ok(())
    } else {
        Err(IopError::NonPositive { field })
    }
}

fn require_non_negative(field: &'static str, value: f64) -> Result<(), IopError> {
    require_finite(field, value)?;
    if value >= 0.0 {
        Ok(())
    } else {
        Err(IopError::Negative { field })
    }
}

fn require_text(field: &'static str, value: &str) -> Result<(), IopError> {
    if value.trim().is_empty() {
        Err(IopError::Empty { field })
    } else {
        Ok(())
    }
}

fn duct_classes_from_rows(rows: Vec<IopDuctClassRow>) -> Result<Vec<IopDuctClass>, IopError> {
    if rows.is_empty() {
        return Err(IopError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = IopDuctClass::from_row(row)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(IopError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn suppression_classes_from_rows(
    rows: Vec<IopSuppressionClassRow>,
) -> Result<Vec<IopSuppressionClass>, IopError> {
    if rows.is_empty() {
        return Err(IopError::Empty { field: "classes" });
    }
    let mut seen = BTreeSet::new();
    let mut classes = Vec::with_capacity(rows.len());
    for row in rows {
        let class = IopSuppressionClass::from_row(row)?;
        if !seen.insert(class.class_id.clone()) {
            return Err(IopError::Duplicate {
                field: "classes.class_id",
            });
        }
        classes.push(class);
    }
    Ok(classes)
}

fn sha256_hex(bytes: &[u8]) -> String {
    let digest = Sha256::digest(bytes);
    let mut out = String::with_capacity(digest.len() * 2);
    for byte in digest {
        let _ = write!(out, "{byte:02x}");
    }
    out
}

#[allow(dead_code)]
fn require_sha256(field: &'static str, value: &str) -> Result<(), IopError> {
    let valid = value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte));
    if valid {
        Ok(())
    } else {
        Err(IopError::InvalidSha256 { field })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn nominal_input(rate: f64, suppression_loss_db: f64) -> IopPulseInput {
        IopPulseInput::new(
            IopStartTransient::new(rate, 0.25).unwrap(),
            IopDuctCoupling::new(120.0, 6.0, 40.0, 1.5).unwrap(),
            30.0,
            suppression_loss_db,
            IopUncertainty::new(3.0, 6.0).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn iop_pulse_matches_closed_form_scaling() {
        let input = nominal_input(2.4e6, 0.0);
        let envelope = evaluate_iop_pulse(input);
        let residence_time_s = 120.0 / (6.0 * 40.0);
        let expected_peak_pa = 2.4e6 * residence_time_s * 1.5 / (4.0 * PI * 30.0 * 30.0);

        assert_eq!(
            input.duct().residence_time_s().to_bits(),
            residence_time_s.to_bits()
        );
        assert!((envelope.peak_overpressure_pa() - expected_peak_pa).abs() <= 1.0e-12);
        assert_eq!(
            envelope.duration_s().to_bits(),
            (0.25 + residence_time_s).to_bits()
        );
        assert!(
            (envelope.lower_peak_overpressure_pa() - expected_peak_pa * 10.0_f64.powf(-3.0 / 20.0))
                .abs()
                <= 1.0e-12
        );
        assert!(
            (envelope.upper_peak_overpressure_pa() - expected_peak_pa * 10.0_f64.powf(6.0 / 20.0))
                .abs()
                <= 1.0e-12
        );
    }

    #[test]
    fn iop_pulse_scales_with_start_rate_distance_and_suppression() {
        let baseline = evaluate_iop_pulse(nominal_input(2.4e6, 0.0));
        let doubled_rate = evaluate_iop_pulse(nominal_input(4.8e6, 0.0));
        let suppressed = evaluate_iop_pulse(nominal_input(2.4e6, 6.0));
        let farther = evaluate_iop_pulse(
            IopPulseInput::new(
                nominal_input(2.4e6, 0.0).source(),
                nominal_input(2.4e6, 0.0).duct(),
                60.0,
                0.0,
                nominal_input(2.4e6, 0.0).uncertainty(),
            )
            .unwrap(),
        );

        assert!(
            (doubled_rate.peak_overpressure_pa() / baseline.peak_overpressure_pa() - 2.0).abs()
                <= 1.0e-12
        );
        assert!(
            (suppressed.peak_overpressure_pa() / baseline.peak_overpressure_pa()
                - 10.0_f64.powf(-6.0 / 20.0))
            .abs()
                <= 1.0e-12
        );
        assert!(
            (farther.peak_overpressure_pa() / baseline.peak_overpressure_pa() - 0.25).abs()
                <= 1.0e-12
        );
    }

    #[test]
    fn iop_helpers_fail_closed_on_bad_inputs() {
        assert_eq!(
            IopStartTransient::new(0.0, 0.1).unwrap_err(),
            IopError::NonPositive {
                field: "gas_generation_rate_n_per_s"
            }
        );
        assert_eq!(
            IopStartTransient::new(1.0, -0.1).unwrap_err(),
            IopError::Negative {
                field: "ramp_duration_s"
            }
        );
        assert_eq!(
            IopDuctCoupling::new(1.0, 0.0, 1.0, 1.0).unwrap_err(),
            IopError::NonPositive {
                field: "exit_area_m2"
            }
        );
        assert_eq!(
            IopDuctCoupling::new(1.0, 1.0, 1.0, -1.0).unwrap_err(),
            IopError::Negative {
                field: "geometry_gain"
            }
        );
        assert_eq!(
            IopUncertainty::new(-1.0, 0.0).unwrap_err(),
            IopError::Negative { field: "minus_db" }
        );
        assert_eq!(
            IopPulseInput::new(
                IopStartTransient::new(1.0, 0.0).unwrap(),
                IopDuctCoupling::new(1.0, 1.0, 1.0, 1.0).unwrap(),
                f64::INFINITY,
                0.0,
                IopUncertainty::new(0.0, 0.0).unwrap(),
            )
            .unwrap_err(),
            IopError::NonFinite {
                field: "receiver_distance_m"
            }
        );
    }

    #[test]
    fn iop_duct_deck_parses_fixture() {
        let fixture = include_str!("../../../data/acoustics/iop-duct-v1.toml");
        let deck = IopDuctDeck::load_from_str(fixture).unwrap();

        assert_eq!(deck.meta().dataset_id, "openbmp.acoustics.iop_duct.v1");
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(deck.classes().len(), 2);
        assert_eq!(
            deck.sha256_hex(),
            "7711a6c472fc5343fda1452626ad7ff41323c8510b6d82d8de6d7b707e807c7e"
        );

        let open = deck.class("open_pad").unwrap();
        assert_eq!(open.description, "Open pad synthetic IOP duct baseline");
        assert_eq!(open.duct.volume_m3().to_bits(), 120.0_f64.to_bits());
        assert_eq!(open.duct.exit_area_m2().to_bits(), 6.0_f64.to_bits());
        assert_eq!(
            open.duct.discharge_velocity_m_s().to_bits(),
            40.0_f64.to_bits()
        );
        assert_eq!(open.duct.geometry_gain().to_bits(), 1.5_f64.to_bits());

        let trench = deck.class("long_trench").unwrap();
        assert_eq!(
            trench.duct.residence_time_s().to_bits(),
            (300.0_f64 / (9.0 * 35.0)).to_bits()
        );
    }

    #[test]
    fn iop_duct_deck_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/iop-duct-v1.toml");
        let bad_schema =
            fixture.replace("acoustics_iop_duct_deck = 1", "acoustics_iop_duct_deck = 2");
        assert_eq!(
            IopDuctDeck::load_from_str(&bad_schema).unwrap_err(),
            IopError::UnsupportedDeckSchema { schema_version: 2 }
        );

        let duplicate_class =
            fixture.replace("class_id = \"long_trench\"", "class_id = \"open_pad\"");
        assert_eq!(
            IopDuctDeck::load_from_str(&duplicate_class).unwrap_err(),
            IopError::Duplicate {
                field: "classes.class_id"
            }
        );

        let bad_area = fixture.replace("exit_area_m2 = 6.0", "exit_area_m2 = 0.0");
        assert_eq!(
            IopDuctDeck::load_from_str(&bad_area).unwrap_err(),
            IopError::NonPositive {
                field: "exit_area_m2"
            }
        );

        let bad_gain = fixture.replace("geometry_gain = 1.5", "geometry_gain = -0.1");
        assert_eq!(
            IopDuctDeck::load_from_str(&bad_gain).unwrap_err(),
            IopError::Negative {
                field: "geometry_gain"
            }
        );

        let deck = IopDuctDeck::load_from_str(fixture).unwrap();
        assert_eq!(
            deck.class("missing").unwrap_err(),
            IopError::UnknownClass {
                class_id: "missing".to_owned()
            }
        );
    }

    #[test]
    fn iop_suppression_deck_parses_fixture() {
        let fixture = include_str!("../../../data/acoustics/iop-suppression-v1.toml");
        let deck = IopSuppressionDeck::load_from_str(fixture).unwrap();

        assert_eq!(
            deck.meta().dataset_id,
            "openbmp.acoustics.iop_suppression.v1"
        );
        assert_eq!(deck.meta().source_class, "synthetic-openbmp");
        assert_eq!(deck.classes().len(), 2);
        assert_eq!(
            deck.sha256_hex(),
            "641894d3cd5ac3dbc8529219b47e39deb1c0974121724fb5023f879f98e9ecc7"
        );

        let dry = deck.class("dry").unwrap();
        assert_eq!(dry.description, "No IOP suppression synthetic baseline");
        assert_eq!(dry.suppression_loss_db.to_bits(), 0.0_f64.to_bits());
        assert_eq!(dry.uncertainty_db.to_bits(), 0.0_f64.to_bits());

        let water = deck.class("water_deluge_nominal").unwrap();
        assert_eq!(water.suppression_loss_db.to_bits(), 3.0_f64.to_bits());
        assert_eq!(water.uncertainty_db.to_bits(), 1.5_f64.to_bits());
    }

    #[test]
    fn iop_suppression_deck_helpers_fail_closed() {
        let fixture = include_str!("../../../data/acoustics/iop-suppression-v1.toml");
        let bad_schema = fixture.replace(
            "acoustics_iop_suppression_deck = 1",
            "acoustics_iop_suppression_deck = 2",
        );
        assert_eq!(
            IopSuppressionDeck::load_from_str(&bad_schema).unwrap_err(),
            IopError::UnsupportedDeckSchema { schema_version: 2 }
        );

        let duplicate_class =
            fixture.replace("class_id = \"water_deluge_nominal\"", "class_id = \"dry\"");
        assert_eq!(
            IopSuppressionDeck::load_from_str(&duplicate_class).unwrap_err(),
            IopError::Duplicate {
                field: "classes.class_id"
            }
        );

        let bad_loss = fixture.replace("suppression_loss_db = 3.0", "suppression_loss_db = -0.1");
        assert_eq!(
            IopSuppressionDeck::load_from_str(&bad_loss).unwrap_err(),
            IopError::Negative {
                field: "classes.suppression_loss_db"
            }
        );

        let bad_uncertainty = fixture.replace("uncertainty_db = 1.5", "uncertainty_db = -0.1");
        assert_eq!(
            IopSuppressionDeck::load_from_str(&bad_uncertainty).unwrap_err(),
            IopError::Negative {
                field: "classes.uncertainty_db"
            }
        );

        let deck = IopSuppressionDeck::load_from_str(fixture).unwrap();
        assert_eq!(
            deck.class("missing").unwrap_err(),
            IopError::UnknownClass {
                class_id: "missing".to_owned()
            }
        );
    }
}
