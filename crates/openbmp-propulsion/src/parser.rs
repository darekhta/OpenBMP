//! Schema-1 motor-file TOML parser.
//!
//! Parses the architecture-locked schema:
//!
//! ```toml
//! openbmp.motor = 1
//!
//! [meta]
//! name       = "synthetic-solid-A"
//! provenance = "synthetic, designed for OpenBMP analytic-toy validation"
//! validation = "validated-toy"   # experimental | checked | validated-toy | research
//!
//! [burn]
//! duration_s         = 4.0
//! total_impulse_n_s  = 2444.5
//! specific_impulse_s = 226.60875296586778
//! propellant_mass_kg = 1.10
//! dry_mass_kg        = 0.40
//!
//! [thrust_curve]
//! points = [[0.00, 0.0], [0.05, 750.0], [0.50, 720.0], [3.50, 580.0], [4.00, 0.0]]
//!
//! [geometry]
//! exit_area_m2                 = 0.0019
//! throat_area_m2               = 0.0002375 # optional; required for pressure_thrust
//! gamma                        = 1.2       # optional; required for pressure_thrust
//! ambient_pressure_correction  = "constant"
//! separation                   = "off"     # off | summerfield | schmucker
//! ```
//!
//! `serde(deny_unknown_fields)` is enforced everywhere so a typo'd
//! field can't smuggle past the parser. The `validation` and
//! `ambient_pressure_correction` fields are typed enums; unknown
//! string values reject with `MalformedMotor`.

use std::path::Path;

use serde::Deserialize;

use crate::error::MotorError;
use crate::motor::{
    AmbientPressureCorrection, BurnSpec, MotorGeometry, MotorMeta, NozzleSeparationCriterion,
    SolidMotor, ThrustCurve, Validation,
};

const SCHEMA_VERSION: u32 = 1;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MotorFile {
    openbmp: SchemaMarker,
    meta: MetaSection,
    burn: BurnSection,
    thrust_curve: ThrustCurveSection,
    geometry: GeometrySection,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SchemaMarker {
    motor: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct MetaSection {
    name: String,
    provenance: String,
    validation: String,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct BurnSection {
    duration_s: f64,
    total_impulse_n_s: f64,
    specific_impulse_s: f64,
    propellant_mass_kg: f64,
    dry_mass_kg: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct ThrustCurveSection {
    points: Vec<[f64; 2]>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct GeometrySection {
    exit_area_m2: f64,
    #[serde(default)]
    throat_area_m2: Option<f64>,
    #[serde(default)]
    gamma: Option<f64>,
    ambient_pressure_correction: String,
    #[serde(default)]
    separation: Option<String>,
}

impl SolidMotor {
    /// Parse a Schema-1 solid-motor file from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::MalformedMotor`] for parser failures
    /// (syntax error, missing required field, unknown field, wrong
    /// schema version, unrecognised `validation` or
    /// `ambient_pressure_correction` value) or for any structural
    /// invariant rejected by [`SolidMotor::new`] /
    /// [`ThrustCurve::new`]. Returns [`MotorError::NonFinite`] /
    /// [`MotorError::InvalidParameter`] for value-level issues.
    pub fn load_from_str(s: &str) -> Result<Self, MotorError> {
        let parsed: MotorFile = toml::from_str(s).map_err(|_e| MotorError::MalformedMotor {
            reason: "motor TOML did not parse against Schema-1",
        })?;
        if parsed.openbmp.motor != SCHEMA_VERSION {
            return Err(MotorError::MalformedMotor {
                reason: "openbmp.motor schema version is not 1",
            });
        }
        let validation = match parsed.meta.validation.as_str() {
            "experimental" => Validation::Experimental,
            "checked" => Validation::Checked,
            "validated-toy" => Validation::ValidatedToy,
            "research" => Validation::Research,
            _ => {
                return Err(MotorError::MalformedMotor {
                    reason: "meta.validation must be one of experimental, checked, validated-toy, research",
                });
            }
        };
        let ambient_pressure_correction = match parsed.geometry.ambient_pressure_correction.as_str()
        {
            "constant" => AmbientPressureCorrection::Constant,
            "pressure_thrust" => AmbientPressureCorrection::PressureThrust,
            _ => {
                return Err(MotorError::MalformedMotor {
                    reason: "geometry.ambient_pressure_correction must be \"constant\" or \"pressure_thrust\"",
                });
            }
        };
        let separation = match parsed.geometry.separation.as_deref().unwrap_or("off") {
            "off" => NozzleSeparationCriterion::Off,
            "summerfield" => NozzleSeparationCriterion::Summerfield,
            "schmucker" => NozzleSeparationCriterion::Schmucker,
            _ => {
                return Err(MotorError::MalformedMotor {
                    reason: "geometry.separation must be \"off\", \"summerfield\", or \"schmucker\"",
                });
            }
        };
        let meta = MotorMeta {
            name: parsed.meta.name,
            provenance: parsed.meta.provenance,
            validation,
        };
        let burn = BurnSpec {
            duration_s: parsed.burn.duration_s,
            total_impulse_n_s: parsed.burn.total_impulse_n_s,
            specific_impulse_s: parsed.burn.specific_impulse_s,
            propellant_mass_kg: parsed.burn.propellant_mass_kg,
            dry_mass_kg: parsed.burn.dry_mass_kg,
        };
        let curve = ThrustCurve::new(parsed.thrust_curve.points)?;
        let geometry = MotorGeometry {
            exit_area_m2: parsed.geometry.exit_area_m2,
            throat_area_m2: parsed.geometry.throat_area_m2,
            gamma: parsed.geometry.gamma,
            ambient_pressure_correction,
            separation,
        };
        SolidMotor::new(meta, burn, curve, geometry)
    }

    /// Parse a Schema-1 solid-motor file from a TOML file on disk.
    ///
    /// # Errors
    ///
    /// Returns [`MotorError::Io`] if the file cannot be read.
    /// Otherwise returns the same errors as [`SolidMotor::load_from_str`].
    pub fn load_from_toml(path: &Path) -> Result<Self, MotorError> {
        let text = std::fs::read_to_string(path).map_err(|e| MotorError::Io {
            reason: format!("could not read motor file {}: {e}", path.display()),
        })?;
        Self::load_from_str(&text)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::motor::Motor;

    /// Minimal solid-motor parser fixture loaded from
    /// `crates/openbmp-propulsion/tests/fixtures/minimal-motor.toml`.
    /// Per the inline-data tripwire (`docs/data-provenance.md §
    /// Inline Data Tripwires`), parser test fixtures live in sibling
    /// files, not inline raw strings.
    fn minimal_motor_toml() -> String {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/minimal-motor.toml"
        ))
        .to_string()
    }

    #[test]
    fn parser_round_trips_minimal_motor() {
        let m = SolidMotor::load_from_str(&minimal_motor_toml()).unwrap();
        assert_eq!(m.meta().name, "test-trapezoidal");
        assert_eq!(m.meta().validation, Validation::ValidatedToy);
        assert_eq!(m.burn_duration_s().to_bits(), 4.0_f64.to_bits());
        assert_eq!(m.total_impulse_n_s().to_bits(), 3500.0_f64.to_bits());
        // Curve corner values come back through the lookup.
        assert_eq!(m.thrust_n_at(0.5).unwrap().to_bits(), 1000.0_f64.to_bits());
        assert_eq!(m.thrust_n_at(3.5).unwrap().to_bits(), 1000.0_f64.to_bits());
    }

    #[test]
    fn parser_recognises_each_validation_label() {
        for (label, expected) in [
            ("experimental", Validation::Experimental),
            ("checked", Validation::Checked),
            ("validated-toy", Validation::ValidatedToy),
            ("research", Validation::Research),
        ] {
            let toml_str = minimal_motor_toml().replace(
                "validation = \"validated-toy\"",
                &format!("validation = \"{label}\""),
            );
            let m = SolidMotor::load_from_str(&toml_str).unwrap();
            assert_eq!(m.meta().validation, expected);
        }
    }

    #[test]
    fn parser_rejects_unknown_validation_label() {
        let toml_str = minimal_motor_toml().replace(
            "validation = \"validated-toy\"",
            "validation = \"invalid-validation-label\"",
        );
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }

    #[test]
    fn parser_rejects_unknown_ambient_pressure_correction() {
        let toml_str = minimal_motor_toml().replace(
            "ambient_pressure_correction  = \"constant\"",
            "ambient_pressure_correction  = \"linear\"",
        );
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }

    #[test]
    fn parser_accepts_pressure_thrust_with_nozzle_state() {
        let toml_str = minimal_motor_toml().replace(
            "exit_area_m2                 = 0.0019\nambient_pressure_correction  = \"constant\"",
            "exit_area_m2                 = 0.0019\nthroat_area_m2               = 0.0002375\ngamma                        = 1.2\nambient_pressure_correction  = \"pressure_thrust\"\nseparation                   = \"summerfield\"",
        );
        let m = SolidMotor::load_from_str(&toml_str).unwrap();
        assert_eq!(
            m.geometry().ambient_pressure_correction,
            AmbientPressureCorrection::PressureThrust
        );
        assert_eq!(
            m.geometry().throat_area_m2.unwrap().to_bits(),
            0.0002375_f64.to_bits()
        );
        assert_eq!(m.geometry().gamma.unwrap().to_bits(), 1.2_f64.to_bits());
        assert_eq!(
            m.geometry().separation,
            NozzleSeparationCriterion::Summerfield
        );
    }

    #[test]
    fn parser_rejects_unknown_separation() {
        let toml_str = minimal_motor_toml().replace(
            "ambient_pressure_correction  = \"constant\"",
            "ambient_pressure_correction  = \"constant\"\nseparation                   = \"side_load\"",
        );
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }

    #[test]
    fn parser_rejects_pressure_thrust_without_nozzle_state() {
        let toml_str = minimal_motor_toml().replace(
            "ambient_pressure_correction  = \"constant\"",
            "ambient_pressure_correction  = \"pressure_thrust\"",
        );
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn parser_rejects_wrong_schema_version() {
        let toml_str = minimal_motor_toml().replace("openbmp.motor = 1", "openbmp.motor = 2");
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }

    #[test]
    fn parser_rejects_unknown_top_level_field() {
        let toml_str = format!("{}\n[unexpected]\nfoo = 1\n", minimal_motor_toml());
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }

    #[test]
    fn parser_rejects_burn_duration_curve_mismatch() {
        let toml_str =
            minimal_motor_toml().replace("duration_s         = 4.0", "duration_s         = 5.0");
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }

    #[test]
    fn parser_rejects_negative_propellant_mass() {
        let toml_str =
            minimal_motor_toml().replace("propellant_mass_kg = 1.0", "propellant_mass_kg = -1.0");
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn parser_rejects_thrust_curve_starting_at_nonzero_t() {
        let toml_str = minimal_motor_toml().replace(
            "points = [[0.0, 0.0], [0.5, 1000.0], [3.5, 1000.0], [4.0, 0.0]]",
            "points = [[0.1, 0.0], [0.5, 1000.0], [3.5, 1000.0], [4.0, 0.0]]",
        );
        assert!(matches!(
            SolidMotor::load_from_str(&toml_str),
            Err(MotorError::MalformedMotor { .. }),
        ));
    }
}
