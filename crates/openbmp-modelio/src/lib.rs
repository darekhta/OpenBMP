//! External model-ingestion schemas for OpenBMP.
//!
//! `openbmp-modelio` is an L1 boundary crate for deterministic, provenance-aware
//! ingestion of reduced structural model assets. It deliberately sits below the
//! runner and scenario layers and does not depend on `openbmp-sim`,
//! `openbmp-runner`, or `openbmp-vehicle`; higher layers adapt parsed assets into
//! their own runtime types.
//!
//! The first implemented asset is [`ModalAsset`], a CSV v1 representation of a
//! reduced lateral modal model exported from an external FEM workflow. OP2/OP4
//! binary readers can land behind this crate later while preserving the same
//! validated asset surface.

#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeSet;

use serde::Deserialize;
use thiserror::Error;

/// Canonical schema string for modal CSV v1 assets.
pub const MODAL_ASSET_CSV_V1_SCHEMA: &str = "openbmp.modal_asset.csv.v1";

/// A validated external modal asset.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalAsset {
    modes: Vec<ModalModeAsset>,
}

impl ModalAsset {
    /// Parse and validate a modal CSV v1 asset.
    ///
    /// # Errors
    ///
    /// Returns [`ModelIoError`] when the CSV cannot be decoded, has no modes,
    /// declares a wrong schema string, duplicates a mode id, carries non-finite
    /// or out-of-range values, or has a zero mode-shape vector that would make a
    /// MAC comparison undefined.
    pub fn from_csv_str(input: &str) -> Result<Self, ModelIoError> {
        let mut reader = csv::ReaderBuilder::new()
            .trim(csv::Trim::All)
            .from_reader(input.as_bytes());
        let mut modes = Vec::new();
        for (index, row) in reader.deserialize::<ModalCsvRow>().enumerate() {
            let row = row?;
            modes.push(row.into_mode(index + 2)?);
        }
        Self::new(modes)
    }

    /// Construct a modal asset from already-decoded mode rows.
    ///
    /// # Errors
    ///
    /// Returns [`ModelIoError`] when the mode list is empty, duplicate ids are
    /// present, or any row fails numeric/schema validation.
    pub fn new(modes: Vec<ModalModeAsset>) -> Result<Self, ModelIoError> {
        if modes.is_empty() {
            return Err(ModelIoError::EmptyAsset);
        }
        let mut ids = BTreeSet::new();
        for (index, mode) in modes.iter().enumerate() {
            mode.validate(index + 1)?;
            if !ids.insert(mode.id.as_str()) {
                return Err(ModelIoError::DuplicateModeId {
                    id: mode.id.clone(),
                });
            }
        }
        Ok(Self { modes })
    }

    /// Declaration-ordered modes from the asset.
    #[must_use]
    pub fn modes(&self) -> &[ModalModeAsset] {
        &self.modes
    }

    /// Validate ingested frequencies and shape vectors against the source
    /// reference columns carried by the asset.
    ///
    /// # Errors
    ///
    /// Returns [`ModelIoError::ReferenceMismatch`] when a mode exceeds the
    /// allowed relative frequency error or falls below the minimum MAC.
    pub fn validate_references(
        &self,
        frequency_tolerance_fraction: f64,
        mac_min: f64,
    ) -> Result<Vec<ModalReferenceCheck>, ModelIoError> {
        if !frequency_tolerance_fraction.is_finite() || frequency_tolerance_fraction < 0.0 {
            return Err(ModelIoError::InvalidNumber {
                row: 0,
                field: "frequency_tolerance_fraction",
                value: frequency_tolerance_fraction,
                rule: "must be finite and non-negative",
            });
        }
        if !mac_min.is_finite() || !(0.0..=1.0).contains(&mac_min) {
            return Err(ModelIoError::InvalidNumber {
                row: 0,
                field: "mac_min",
                value: mac_min,
                rule: "must be finite and in [0, 1]",
            });
        }

        let mut checks = Vec::with_capacity(self.modes.len());
        for mode in &self.modes {
            let frequency_error_fraction = ((mode.frequency_hz - mode.source_frequency_hz).abs()
                / mode.source_frequency_hz)
                .abs();
            let mac = shape_mac(mode.shape_vector(), mode.source_shape_vector())?;
            if frequency_error_fraction > frequency_tolerance_fraction || mac < mac_min {
                return Err(ModelIoError::ReferenceMismatch {
                    id: mode.id.clone(),
                    frequency_error_fraction,
                    frequency_tolerance_fraction,
                    mac,
                    mac_min,
                });
            }
            checks.push(ModalReferenceCheck {
                id: mode.id.clone(),
                frequency_error_fraction,
                mac,
            });
        }
        Ok(checks)
    }
}

/// One lateral modal row from an external reduced model asset.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalModeAsset {
    /// Stable mode id.
    pub id: String,
    /// Modal natural frequency, Hz.
    pub frequency_hz: f64,
    /// Modal damping ratio.
    pub damping_ratio: f64,
    /// Generalized modal mass, kg.
    pub modal_mass_kg: f64,
    /// Mode-shape slope at the engine station.
    pub slope_at_engine: f64,
    /// Mode-shape slope at the rate-gyro station.
    pub slope_at_gyro: f64,
    /// Mode-shape value at the accelerometer station.
    pub shape_at_accelerometer: f64,
    /// Source FEM modal frequency, Hz.
    pub source_frequency_hz: f64,
    /// Source shape component at the engine station.
    pub source_slope_at_engine: f64,
    /// Source shape component at the rate-gyro station.
    pub source_slope_at_gyro: f64,
    /// Source shape component at the accelerometer station.
    pub source_shape_at_accelerometer: f64,
}

impl ModalModeAsset {
    /// Ingested shape vector used by the MAC check.
    #[must_use]
    pub fn shape_vector(&self) -> [f64; 3] {
        [
            self.slope_at_engine,
            self.slope_at_gyro,
            self.shape_at_accelerometer,
        ]
    }

    /// Source-reference shape vector used by the MAC check.
    #[must_use]
    pub fn source_shape_vector(&self) -> [f64; 3] {
        [
            self.source_slope_at_engine,
            self.source_slope_at_gyro,
            self.source_shape_at_accelerometer,
        ]
    }

    fn validate(&self, row: usize) -> Result<(), ModelIoError> {
        require_schema(row, &self.id, "id")?;
        require_positive(row, "frequency_hz", self.frequency_hz)?;
        require_non_negative(row, "damping_ratio", self.damping_ratio)?;
        require_positive(row, "modal_mass_kg", self.modal_mass_kg)?;
        require_finite(row, "slope_at_engine", self.slope_at_engine)?;
        require_finite(row, "slope_at_gyro", self.slope_at_gyro)?;
        require_finite(row, "shape_at_accelerometer", self.shape_at_accelerometer)?;
        require_positive(row, "source_frequency_hz", self.source_frequency_hz)?;
        require_finite(row, "source_slope_at_engine", self.source_slope_at_engine)?;
        require_finite(row, "source_slope_at_gyro", self.source_slope_at_gyro)?;
        require_finite(
            row,
            "source_shape_at_accelerometer",
            self.source_shape_at_accelerometer,
        )?;
        require_non_zero_vector(row, self.shape_vector(), "ingested shape")?;
        require_non_zero_vector(row, self.source_shape_vector(), "source shape")?;
        Ok(())
    }
}

#[derive(Clone, Debug, Deserialize)]
struct ModalCsvRow {
    schema: String,
    id: String,
    frequency_hz: f64,
    damping_ratio: f64,
    modal_mass_kg: f64,
    slope_at_engine: f64,
    slope_at_gyro: f64,
    shape_at_accelerometer: f64,
    source_frequency_hz: f64,
    source_slope_at_engine: f64,
    source_slope_at_gyro: f64,
    source_shape_at_accelerometer: f64,
}

impl ModalCsvRow {
    fn into_mode(self, row: usize) -> Result<ModalModeAsset, ModelIoError> {
        if self.schema != MODAL_ASSET_CSV_V1_SCHEMA {
            return Err(ModelIoError::SchemaMismatch {
                row,
                expected: MODAL_ASSET_CSV_V1_SCHEMA,
                found: self.schema,
            });
        }
        let mode = ModalModeAsset {
            id: self.id,
            frequency_hz: self.frequency_hz,
            damping_ratio: self.damping_ratio,
            modal_mass_kg: self.modal_mass_kg,
            slope_at_engine: self.slope_at_engine,
            slope_at_gyro: self.slope_at_gyro,
            shape_at_accelerometer: self.shape_at_accelerometer,
            source_frequency_hz: self.source_frequency_hz,
            source_slope_at_engine: self.source_slope_at_engine,
            source_slope_at_gyro: self.source_slope_at_gyro,
            source_shape_at_accelerometer: self.source_shape_at_accelerometer,
        };
        mode.validate(row)?;
        Ok(mode)
    }
}

/// Code-to-code validation result for one modal row.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalReferenceCheck {
    /// Stable mode id.
    pub id: String,
    /// Relative frequency error against the source-reference column.
    pub frequency_error_fraction: f64,
    /// Modal assurance criterion between ingested and source shape vectors.
    pub mac: f64,
}

/// Errors raised while parsing or validating external model assets.
#[derive(Debug, Error)]
pub enum ModelIoError {
    /// CSV decoding failed.
    #[error("modal CSV parse error: {0}")]
    Csv(#[from] csv::Error),
    /// The asset carries no mode rows.
    #[error("modal asset must contain at least one mode")]
    EmptyAsset,
    /// A row declared the wrong schema string.
    #[error("modal asset schema mismatch at CSV row {row}: expected {expected}, found {found}")]
    SchemaMismatch {
        /// One-based CSV row number, including the header row.
        row: usize,
        /// Expected schema string.
        expected: &'static str,
        /// Found schema string.
        found: String,
    },
    /// A mode id is duplicated.
    #[error("modal asset duplicate mode id `{id}`")]
    DuplicateModeId {
        /// Duplicated id.
        id: String,
    },
    /// A numeric field was not finite or outside its allowed range.
    #[error("modal asset invalid number at row {row} field {field}: {value} {rule}")]
    InvalidNumber {
        /// One-based row number, or zero for API parameters.
        row: usize,
        /// Field name.
        field: &'static str,
        /// Offending value.
        value: f64,
        /// Human-readable validation rule.
        rule: &'static str,
    },
    /// A string field was empty.
    #[error("modal asset invalid string at row {row} field {field}: must be non-empty")]
    InvalidString {
        /// One-based row number.
        row: usize,
        /// Field name.
        field: &'static str,
    },
    /// A mode-shape vector had zero norm.
    #[error("modal asset invalid shape vector at row {row}: {which} vector norm must be positive")]
    ZeroShapeVector {
        /// One-based row number.
        row: usize,
        /// Vector label.
        which: &'static str,
    },
    /// A source-reference comparison exceeded its tolerance.
    #[error(
        "modal asset reference mismatch for `{id}`: frequency error {frequency_error_fraction} > {frequency_tolerance_fraction} or MAC {mac} < {mac_min}"
    )]
    ReferenceMismatch {
        /// Mode id.
        id: String,
        /// Relative frequency error.
        frequency_error_fraction: f64,
        /// Allowed relative frequency error.
        frequency_tolerance_fraction: f64,
        /// Modal assurance criterion.
        mac: f64,
        /// Minimum allowed MAC.
        mac_min: f64,
    },
}

fn shape_mac(a: [f64; 3], b: [f64; 3]) -> Result<f64, ModelIoError> {
    let dot = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    let norm_a = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    let norm_b = b[0] * b[0] + b[1] * b[1] + b[2] * b[2];
    if norm_a <= 0.0 {
        return Err(ModelIoError::ZeroShapeVector {
            row: 0,
            which: "ingested shape",
        });
    }
    if norm_b <= 0.0 {
        return Err(ModelIoError::ZeroShapeVector {
            row: 0,
            which: "source shape",
        });
    }
    Ok((dot * dot) / (norm_a * norm_b))
}

fn require_schema(row: usize, value: &str, field: &'static str) -> Result<(), ModelIoError> {
    if value.trim().is_empty() {
        return Err(ModelIoError::InvalidString { row, field });
    }
    Ok(())
}

fn require_finite(row: usize, field: &'static str, value: f64) -> Result<(), ModelIoError> {
    if !value.is_finite() {
        return Err(ModelIoError::InvalidNumber {
            row,
            field,
            value,
            rule: "must be finite",
        });
    }
    Ok(())
}

fn require_positive(row: usize, field: &'static str, value: f64) -> Result<(), ModelIoError> {
    require_finite(row, field, value)?;
    if value <= 0.0 {
        return Err(ModelIoError::InvalidNumber {
            row,
            field,
            value,
            rule: "must be positive",
        });
    }
    Ok(())
}

fn require_non_negative(row: usize, field: &'static str, value: f64) -> Result<(), ModelIoError> {
    require_finite(row, field, value)?;
    if value < 0.0 {
        return Err(ModelIoError::InvalidNumber {
            row,
            field,
            value,
            rule: "must be non-negative",
        });
    }
    Ok(())
}

fn require_non_zero_vector(
    row: usize,
    value: [f64; 3],
    which: &'static str,
) -> Result<(), ModelIoError> {
    let norm = value[0] * value[0] + value[1] * value[1] + value[2] * value[2];
    if !norm.is_finite() || norm <= 0.0 {
        return Err(ModelIoError::ZeroShapeVector { row, which });
    }
    Ok(())
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
mod tests {
    use super::*;
    use sha2::{Digest, Sha256};

    const MODAL_CSV: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/modal/synthetic-sol103-modal-asset-v1.csv"
    ));
    const MODAL_PROVENANCE: &str = include_str!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../data/modal/provenance.md"
    ));
    const MODAL_CSV_SHA256: &str =
        "6d3f3c621a9340d509cca35f111826ad06cf45bc2ded92027a98b4af3c5f3171";

    #[test]
    fn parses_modal_csv_in_declaration_order_and_validates_references() {
        let digest = Sha256::digest(MODAL_CSV.as_bytes());
        assert_eq!(format!("{digest:x}"), MODAL_CSV_SHA256);
        assert!(MODAL_PROVENANCE.contains(MODAL_CSV_SHA256));

        let asset = ModalAsset::from_csv_str(MODAL_CSV).unwrap();
        assert_eq!(asset.modes().len(), 2);
        assert_eq!(asset.modes()[0].id, "first_bending");
        assert_eq!(asset.modes()[1].id, "second_bending");
        let checks = asset.validate_references(0.01, 0.9).unwrap();
        assert_eq!(checks.len(), 2);
        for check in checks {
            assert!(check.frequency_error_fraction < 0.01);
            assert!(check.mac > 0.9);
        }
    }

    #[test]
    fn rejects_schema_mismatch_duplicate_ids_and_bad_reference_mac() {
        let bad_schema = MODAL_CSV.replace(MODAL_ASSET_CSV_V1_SCHEMA, "wrong.schema");
        assert!(matches!(
            ModalAsset::from_csv_str(&bad_schema).unwrap_err(),
            ModelIoError::SchemaMismatch { .. }
        ));

        let duplicate = MODAL_CSV.replace("second_bending", "first_bending");
        assert!(matches!(
            ModalAsset::from_csv_str(&duplicate).unwrap_err(),
            ModelIoError::DuplicateModeId { .. }
        ));

        let bad_mac = MODAL_CSV.replace("-0.2020,0.0490,0.0010", "0.0500,0.2000,0.0000");
        let asset = ModalAsset::from_csv_str(&bad_mac).unwrap();
        assert!(matches!(
            asset.validate_references(0.01, 0.9).unwrap_err(),
            ModelIoError::ReferenceMismatch { .. }
        ));
    }
}
