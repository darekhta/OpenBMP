//! Schema-1 IMU noise-budget TOML parser.
//!
//! Parses the architecture-locked schema:
//!
//! ```toml
//! openbmp.imu_noise_budget = 1
//!
//! [meta]
//! name       = "imu-tactical"
//! provenance = "..."
//! validation = "validated-toy"
//!
//! [sample]
//! dt_s = 0.005
//!
//! [gyro]
//! arw_per_sqrt_s        = 0.001
//! bias_ou_theta         = 0.01
//! bias_ou_sigma         = 0.0001
//! rrw_sigma_per_sqrt_s  = 0.00001
//! scale_factor_ppm      = 50.0
//! quantization_lsb      = 0.0
//!
//! [accel]
//! arw_per_sqrt_s        = 0.0005
//! bias_ou_theta         = 0.005
//! bias_ou_sigma         = 0.00005
//! rrw_sigma_per_sqrt_s  = 0.000005
//! scale_factor_ppm      = 50.0
//! quantization_lsb      = 0.0
//! ```

use std::path::Path;

use openbmp_core::ValidationStatus;
use serde::Deserialize;

use crate::error::SensorError;
use crate::imu::{ImuNoiseBudget, TriaxialNoiseBudget};

const SCHEMA_VERSION: u32 = 1;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ImuBudgetFile {
    openbmp: SchemaMarker,
    #[allow(dead_code)] // surfaced through provenance.md, not the runtime budget.
    meta: MetaSection,
    sample: SampleSection,
    gyro: TriaxialSection,
    accel: TriaxialSection,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SchemaMarker {
    imu_noise_budget: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MetaSection {
    #[allow(dead_code)]
    name: String,
    #[allow(dead_code)]
    provenance: String,
    #[allow(dead_code)]
    validation: ValidationStatus,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SampleSection {
    dt_s: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct TriaxialSection {
    arw_per_sqrt_s: f64,
    bias_ou_theta: f64,
    bias_ou_sigma: f64,
    rrw_sigma_per_sqrt_s: f64,
    scale_factor_ppm: f64,
    quantization_lsb: f64,
}

impl ImuNoiseBudget {
    /// Parse a Schema-1 IMU noise-budget file from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::MalformedBudget`] for parser failures
    /// (syntax error, missing field, unknown field, wrong schema
    /// version) or [`SensorError::InvalidParameter`] /
    /// [`SensorError::NonFinite`] for value-level failures from
    /// [`TriaxialNoiseBudget::new`] / [`Self::new`].
    pub fn load_from_str(s: &str) -> Result<Self, SensorError> {
        let parsed: ImuBudgetFile =
            toml::from_str(s).map_err(|_e| SensorError::MalformedBudget {
                reason: "IMU noise-budget TOML did not parse against Schema-1",
            })?;
        if parsed.openbmp.imu_noise_budget != SCHEMA_VERSION {
            return Err(SensorError::MalformedBudget {
                reason: "openbmp.imu_noise_budget schema version is not 1",
            });
        }
        let gyro = TriaxialNoiseBudget::new(
            parsed.gyro.arw_per_sqrt_s,
            parsed.gyro.bias_ou_theta,
            parsed.gyro.bias_ou_sigma,
            parsed.gyro.rrw_sigma_per_sqrt_s,
            parsed.gyro.scale_factor_ppm,
            parsed.gyro.quantization_lsb,
        )?;
        let accel = TriaxialNoiseBudget::new(
            parsed.accel.arw_per_sqrt_s,
            parsed.accel.bias_ou_theta,
            parsed.accel.bias_ou_sigma,
            parsed.accel.rrw_sigma_per_sqrt_s,
            parsed.accel.scale_factor_ppm,
            parsed.accel.quantization_lsb,
        )?;
        ImuNoiseBudget::new(gyro, accel, parsed.sample.dt_s)
    }

    /// Parse a Schema-1 IMU noise-budget file from disk.
    ///
    /// # Errors
    ///
    /// Returns [`SensorError::Io`] if the file cannot be read.
    /// Otherwise returns the same errors as [`Self::load_from_str`].
    pub fn load_from_toml(path: &Path) -> Result<Self, SensorError> {
        let text = std::fs::read_to_string(path).map_err(|e| SensorError::Io {
            reason: format!(
                "could not read IMU noise-budget file {}: {e}",
                path.display()
            ),
        })?;
        Self::load_from_str(&text)
    }
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]
mod tests {
    use super::*;

    /// Minimal IMU noise-budget parser fixture loaded from
    /// `crates/openbmp-sensors/tests/fixtures/minimal-imu-budget.toml`.
    /// Per the inline-data tripwire (`docs/data-provenance.md §
    /// Inline Data Tripwires`), parser test fixtures live in sibling
    /// files, not inline raw strings.
    fn minimal_budget_toml() -> &'static str {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/minimal-imu-budget.toml"
        ))
    }

    #[test]
    fn parser_round_trips_minimal_budget() {
        let b = ImuNoiseBudget::load_from_str(minimal_budget_toml()).unwrap();
        assert_eq!(b.dt_s.to_bits(), 0.001_f64.to_bits());
        assert_eq!(b.gyro.arw_per_sqrt_s.to_bits(), 0.001_f64.to_bits());
        assert_eq!(b.accel.arw_per_sqrt_s.to_bits(), 0.01_f64.to_bits());
    }

    #[test]
    fn parser_rejects_wrong_schema_version() {
        let s = minimal_budget_toml().replace(
            "openbmp.imu_noise_budget = 1",
            "openbmp.imu_noise_budget = 2",
        );
        assert!(matches!(
            ImuNoiseBudget::load_from_str(&s),
            Err(SensorError::MalformedBudget { .. }),
        ));
    }

    #[test]
    fn parser_rejects_unknown_top_level_field() {
        let s = format!("{}\n[unexpected]\nfoo = 1\n", minimal_budget_toml());
        assert!(matches!(
            ImuNoiseBudget::load_from_str(&s),
            Err(SensorError::MalformedBudget { .. }),
        ));
    }

    #[test]
    fn parser_rejects_negative_arw() {
        let s = minimal_budget_toml().replace(
            "arw_per_sqrt_s        = 0.001",
            "arw_per_sqrt_s        = -0.001",
        );
        assert!(matches!(
            ImuNoiseBudget::load_from_str(&s),
            Err(SensorError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn parser_rejects_unknown_validation_label() {
        let s = minimal_budget_toml().replace(
            "validation = \"validated-toy\"",
            "validation = \"validatd-toy\"",
        );
        assert!(matches!(
            ImuNoiseBudget::load_from_str(&s),
            Err(SensorError::MalformedBudget { .. }),
        ));
    }
}
