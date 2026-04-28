//! Schema-1 deck-file TOML parser.
//!
//! Parses the architecture-locked deck schema:
//!
//! ```toml
//! openbmp.aero_deck = 1
//!
//! reference.area_m2  = 0.196
//! reference.length_m = 0.5
//! provenance         = "synthetic textbook example"
//! validation         = "validated-toy"  # project validation label
//! extrapolation      = "fail-closed"   # optional; default fail-closed
//!
//! [grid]
//! mach      = [0.0, 0.5, 0.8, 1.2, 2.0, 3.0]
//! alpha_deg = [-10.0, -5.0, 0.0, 5.0, 10.0]
//! beta_deg  = [0.0]
//!
//! [coefficients.cn]
//! data = [/* row-major over (mach, alpha, beta) */]
//!
//! [coefficients.cd]
//! data = [/* ... */]
//!
//! [coefficients.cm]
//! data = [/* ... */]
//! ```
//!
//! `serde(deny_unknown_fields)` is enforced everywhere so a deck
//! file can't smuggle a typo'd field through unnoticed.

use std::path::Path;

use serde::Deserialize;

use crate::deck::{AeroDeck, ExtrapolationPolicy};
use crate::error::AeroError;

const SCHEMA_VERSION: u32 = 1;

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DeckFile {
    openbmp: SchemaMarker,
    reference: Reference,
    #[allow(dead_code)] // surfaced through provenance.md, not the runtime deck.
    provenance: String,
    #[allow(dead_code)] // ditto — used by the audit walk, not the lookup path.
    validation: DeckValidationStatus,
    grid: Grid,
    coefficients: Coefficients,
    #[serde(default)]
    extrapolation: Option<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct SchemaMarker {
    aero_deck: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Reference {
    area_m2: f64,
    length_m: f64,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Grid {
    mach: Vec<f64>,
    alpha_deg: Vec<f64>,
    beta_deg: Vec<f64>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct Coefficients {
    cn: CoefficientTable,
    cd: CoefficientTable,
    cm: CoefficientTable,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct CoefficientTable {
    data: Vec<f64>,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
enum DeckValidationStatus {
    Experimental,
    Checked,
    ValidatedToy,
    Research,
}

impl AeroDeck {
    /// Parse a Schema-1 deck from a TOML string.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for parser failures
    /// (syntax error, missing required field, unknown field, wrong
    /// schema version, unrecognised `validation` / `extrapolation`
    /// value) or for any structural invariant rejected by
    /// [`AeroDeck::new`].
    /// Returns [`AeroError::NonFinite`] / [`AeroError::InvalidParameter`]
    /// for value-level issues from [`AeroDeck::new`].
    pub fn load_from_str(s: &str) -> Result<Self, AeroError> {
        let parsed: DeckFile = toml::from_str(s).map_err(|_e| AeroError::MalformedDeck {
            reason: "deck TOML did not parse against Schema-1",
        })?;
        if parsed.openbmp.aero_deck != SCHEMA_VERSION {
            return Err(AeroError::MalformedDeck {
                reason: "openbmp.aero_deck schema version is not 1",
            });
        }
        let extrapolation = match parsed.extrapolation.as_deref() {
            None | Some("fail-closed") => ExtrapolationPolicy::FailClosed,
            Some("clamp") => ExtrapolationPolicy::Clamp,
            Some(_) => {
                return Err(AeroError::MalformedDeck {
                    reason: "deck `extrapolation` must be \"fail-closed\" or \"clamp\"",
                });
            }
        };
        let deck = AeroDeck::new(
            parsed.grid.mach,
            parsed.grid.alpha_deg,
            parsed.grid.beta_deg,
            parsed.coefficients.cn.data,
            parsed.coefficients.cd.data,
            parsed.coefficients.cm.data,
            parsed.reference.area_m2,
            parsed.reference.length_m,
        )?;
        Ok(deck.with_extrapolation_policy(extrapolation))
    }

    /// Parse a Schema-1 deck from a TOML file on disk.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::Io`] if the file cannot be read.
    /// Otherwise returns the same errors as [`AeroDeck::load_from_str`].
    pub fn load_from_toml(path: &Path) -> Result<Self, AeroError> {
        let text = std::fs::read_to_string(path).map_err(|e| AeroError::Io {
            reason: format!("could not read deck file {}: {e}", path.display()),
        })?;
        Self::load_from_str(&text)
    }
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::AeroCoefficients;

    /// Minimal-deck parser fixture loaded from
    /// `crates/openbmp-aero/tests/fixtures/minimal-deck.toml`. Per the
    /// inline-data tripwire (`docs/data-provenance.md § Inline Data
    /// Tripwires`), parser test fixtures live in sibling files, not
    /// inline raw strings.
    fn minimal_deck_toml() -> String {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/minimal-deck.toml"
        ))
        .to_string()
    }

    #[test]
    fn round_trip_minimal_deck_lookup_at_corner() {
        let deck = AeroDeck::load_from_str(&minimal_deck_toml()).unwrap();
        // Corner (mach=1, alpha=1, beta=0) should be the last
        // row-major entry: index 1·2·1 + 1·1 + 0 = 3 → 3.0.
        let r = deck
            .lookup(1.0, 1.0, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        assert_eq!(r.cn.to_bits(), 3.0_f64.to_bits());
        assert_eq!(r.cd.to_bits(), 3.0_f64.to_bits());
        assert_eq!(r.cm.to_bits(), 3.0_f64.to_bits());
    }

    #[test]
    fn parser_carries_reference_geometry() {
        let toml_str = minimal_deck_toml().replace(
            "reference.area_m2  = 1.0\nreference.length_m = 1.0",
            "reference.area_m2  = 0.196\nreference.length_m = 0.5",
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        assert_eq!(deck.reference_area_m2(), 0.196);
        assert_eq!(deck.reference_length_m(), 0.5);
    }

    #[test]
    fn parser_default_extrapolation_is_fail_closed() {
        let deck = AeroDeck::load_from_str(&minimal_deck_toml()).unwrap();
        assert_eq!(deck.extrapolation_policy(), ExtrapolationPolicy::FailClosed);
    }

    #[test]
    fn parser_recognises_clamp_extrapolation() {
        let toml_str = format!("extrapolation = \"clamp\"\n{}", minimal_deck_toml());
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        assert_eq!(deck.extrapolation_policy(), ExtrapolationPolicy::Clamp);
    }

    #[test]
    fn parser_recognises_explicit_fail_closed_extrapolation() {
        let toml_str = format!("extrapolation = \"fail-closed\"\n{}", minimal_deck_toml());
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        assert_eq!(deck.extrapolation_policy(), ExtrapolationPolicy::FailClosed);
    }

    #[test]
    fn parser_rejects_unknown_extrapolation_value() {
        let toml_str = format!("extrapolation = \"linear\"\n{}", minimal_deck_toml());
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_unknown_validation_label() {
        let toml_str = minimal_deck_toml().replace(
            "validation         = \"validated-toy\"",
            "validation         = \"validatd-toy\"",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_wrong_schema_version() {
        let toml_str =
            minimal_deck_toml().replace("openbmp.aero_deck = 1", "openbmp.aero_deck = 2");
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_unknown_top_level_field() {
        let toml_str = format!("{}\n[unexpected]\nfoo = 1\n", minimal_deck_toml());
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_missing_reference() {
        let toml_str =
            minimal_deck_toml().replace("reference.area_m2  = 1.0\nreference.length_m = 1.0\n", "");
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_inconsistent_coefficient_table_size() {
        let toml_str = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/inconsistent-coefficient-table.toml"
        ));
        assert!(matches!(
            AeroDeck::load_from_str(toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn round_trip_minimal_deck_centre_lookup() {
        let deck = AeroDeck::load_from_str(&minimal_deck_toml()).unwrap();
        // (mach=0.5, alpha=0.5, beta=0): centroid of the 4 corners
        // 0,1,2,3 → 1.5 (the 4-corner average for a 2x2 face when beta
        // is single-valued).
        let r = deck
            .lookup(0.5, 0.5, 0.0, &std::collections::BTreeMap::new())
            .unwrap();
        let expected = AeroCoefficients {
            cn: 1.5,
            cd: 1.5,
            cm: 1.5,
        };
        assert_eq!(r.cn.to_bits(), expected.cn.to_bits());
        assert_eq!(r.cd.to_bits(), expected.cd.to_bits());
        assert_eq!(r.cm.to_bits(), expected.cm.to_bits());
    }
}
