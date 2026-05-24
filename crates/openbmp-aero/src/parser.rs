//! Aero deck-file TOML parser. Coefficient decks use Schema 1
//! and Schema 2. A separate strict panel-mesh deck handles
//! [`crate::hypersonic::LocalInclinationPanels`].
//!
//! # Schema 1
//!
//! Three-axis tabulated deck:
//!
//! ```toml
//! openbmp.aero_deck = 1
//!
//! reference.area_m2  = 0.196
//! reference.length_m = 0.5
//! provenance         = "synthetic textbook example"
//! validation         = "validated-toy"  # project validation label
//! extrapolation      = "fail-closed"    # optional; default fail-closed
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
//! # Schema 2 — control-effector axes
//!
//! Schema 2 extends the deck with optional effector-axis dimensions
//! (e.g. `delta_e_deg`). The wire format adds a required
//! `[axis_order]` table declaring the locked reduction order, a
//! required `[interpolation]` table fixing the method, and per-axis
//! grid keys for the new dimensions:
//!
//! ```toml
//! openbmp.aero_deck = 2
//!
//! reference.area_m2  = 1.227
//! reference.length_m = 1.250
//! provenance         = "..."
//! validation         = "experimental"
//!
//! [grid]
//! mach        = [0.3, 0.6, 0.9, 1.2, 1.6, 2.0, 3.0, 5.0]
//! alpha_deg   = [-5.0, 0.0, 5.0, 10.0, 15.0, 20.0]
//! beta_deg    = [-4.0, 0.0, 4.0]
//! delta_e_deg = [-20.0, -10.0, 0.0, 10.0, 20.0]
//!
//! [axis_order]
//! order = ["mach", "alpha", "beta", "delta_e_deg"]
//!
//! [coefficients.cn]
//! data = [/* row-major over the 4-axis cartesian product */]
//!
//! [coefficients.cd]
//! data = [/* ... */]
//!
//! [coefficients.cm]
//! data = [/* ... */]
//!
//! [interpolation]
//! method        = "multilinear"
//! extrapolation = "error"
//! ```
//!
//! Schema-2 rules:
//!
//! - `axis_order.order` must start with `["mach", "alpha", "beta"]`
//!   (the three base axes are always present and always first).
//! - `axis_order.order[3..]` are the effector axes; each name must
//!   end with `_deg` or `_rad` so the runner can match the deck axis
//!   to a scenario effector's `unit` field at runtime.
//! - The `[grid]` table must carry exactly the axes declared by
//!   `axis_order` — extra keys are rejected, missing keys are
//!   rejected. The three base axes use the existing Schema-1 grid
//!   keys (`mach`, `alpha_deg`, `beta_deg`); effector axes use their
//!   `axis_order` name verbatim as the grid key.
//! - `interpolation.method` must be `"multilinear"`. Other methods
//!   are reserved for hypersonic work.
//! - `interpolation.extrapolation` must be `"error"`. Schema-2
//!   decks fail closed on out-of-grid effector deflections by
//!   contract: control surfaces saturate via the `ControlEffector`
//!   layer, not via the deck.
//!
//! `serde(deny_unknown_fields)` is enforced everywhere so a deck
//! file can't smuggle a typo'd field through unnoticed.
//!
//! # Panel-mesh hypersonic deck
//!
//! Panel meshes intentionally use their own marker so coefficient
//! decks retain the Schema-1 / Schema-2 semantics:
//!
//! ```toml
//! openbmp.panel_mesh_aero = 1
//! provenance = "academic mesh fixture"
//! validation = "checked"
//! method = "modified-newtonian"
//! cp_max = 2.0
//! shadowing = true # optional; default true
//!
//! [mesh]
//! vertices_m = [
//!   [1.0, -0.5, -0.5],
//!   [1.0,  0.5, -0.5],
//!   [1.0,  0.5,  0.5],
//!   [1.0, -0.5,  0.5],
//! ]
//! triangles = [[0, 1, 2], [0, 2, 3]]
//! ```

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use nalgebra::Vector3;
use serde::Deserialize;

use crate::deck::{AeroDeck, ExtrapolationPolicy};
use crate::error::AeroError;
use crate::hypersonic::{LocalInclinationPanels, PanelMesh};

const SCHEMA_VERSION_1: u32 = 1;
const SCHEMA_VERSION_2: u32 = 2;
const PANEL_MESH_AERO_SCHEMA_VERSION_1: u32 = 1;

// ---------------------------------------------------------------------
// Schema-version peek
// ---------------------------------------------------------------------

#[derive(Deserialize)]
struct SchemaPeek {
    openbmp: SchemaPeekMarker,
}

#[derive(Deserialize)]
struct SchemaPeekMarker {
    aero_deck: u32,
}

// ---------------------------------------------------------------------
// Schema-1 parser shape
// ---------------------------------------------------------------------

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DeckFileV1 {
    openbmp: SchemaMarker,
    reference: Reference,
    provenance: String,
    validation: DeckValidationStatus,
    grid: GridV1,
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
struct GridV1 {
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

fn validate_deck_metadata(
    provenance: &str,
    _validation: DeckValidationStatus,
) -> Result<(), AeroError> {
    if provenance.trim().is_empty() {
        return Err(AeroError::MalformedDeck {
            reason: "deck `provenance` must not be blank",
        });
    }
    Ok(())
}

// ---------------------------------------------------------------------
// Schema-2 parser shape
// ---------------------------------------------------------------------

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DeckFileV2 {
    openbmp: SchemaMarker,
    reference: Reference,
    provenance: String,
    validation: DeckValidationStatus,
    /// Axis grids keyed by wire-name. The Schema-2 parser does its own
    /// structural validation against `[axis_order]` so the parser
    /// can reject extra / missing axis keys with a typed error.
    grid: BTreeMap<String, Vec<f64>>,
    axis_order: AxisOrderConfig,
    coefficients: Coefficients,
    interpolation: InterpolationConfig,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct AxisOrderConfig {
    order: Vec<String>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct InterpolationConfig {
    method: String,
    extrapolation: String,
}

// ---------------------------------------------------------------------
// Panel-mesh parser shape
// ---------------------------------------------------------------------

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PanelMeshDeckFile {
    openbmp: PanelMeshSchemaMarker,
    provenance: String,
    validation: DeckValidationStatus,
    method: PanelDeckMethod,
    cp_max: f64,
    #[serde(default = "default_panel_shadowing")]
    shadowing: bool,
    mesh: PanelMeshToml,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PanelMeshSchemaMarker {
    panel_mesh_aero: u32,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PanelMeshToml {
    vertices_m: Vec<[f64; 3]>,
    triangles: Vec<[u32; 3]>,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
enum PanelDeckMethod {
    ModifiedNewtonian,
}

const fn default_panel_shadowing() -> bool {
    true
}

impl AeroDeck {
    /// Parse a deck from a TOML string. Auto-detects Schema 1 vs
    /// Schema 2 from the `openbmp.aero_deck` integer marker.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for parser failures
    /// (syntax error, missing required field, unknown field, wrong
    /// schema version, unrecognised `validation` / `extrapolation`
    /// value) or for any structural invariant rejected by
    /// [`AeroDeck::new`] / [`AeroDeck::new_n_d`].
    /// Returns [`AeroError::NonFinite`] / [`AeroError::InvalidParameter`]
    /// for value-level issues.
    pub fn load_from_str(s: &str) -> Result<Self, AeroError> {
        let peek: SchemaPeek = toml::from_str(s).map_err(|_e| AeroError::MalformedDeck {
            reason: "deck TOML did not parse: missing or malformed openbmp.aero_deck",
        })?;
        match peek.openbmp.aero_deck {
            SCHEMA_VERSION_1 => Self::load_schema1(s),
            SCHEMA_VERSION_2 => Self::load_schema2(s),
            _ => Err(AeroError::MalformedDeck {
                reason: "openbmp.aero_deck schema version must be 1 or 2",
            }),
        }
    }

    /// Parse a deck from a TOML file on disk. Auto-detects Schema 1
    /// vs Schema 2.
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

    fn load_schema1(s: &str) -> Result<Self, AeroError> {
        let parsed: DeckFileV1 = toml::from_str(s).map_err(|_e| AeroError::MalformedDeck {
            reason: "deck TOML did not parse against Schema-1",
        })?;
        if parsed.openbmp.aero_deck != SCHEMA_VERSION_1 {
            return Err(AeroError::MalformedDeck {
                reason: "openbmp.aero_deck schema version is not 1",
            });
        }
        validate_deck_metadata(&parsed.provenance, parsed.validation)?;
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

    fn load_schema2(s: &str) -> Result<Self, AeroError> {
        let parsed: DeckFileV2 = toml::from_str(s).map_err(|_e| AeroError::MalformedDeck {
            reason: "deck TOML did not parse against Schema-2",
        })?;
        if parsed.openbmp.aero_deck != SCHEMA_VERSION_2 {
            return Err(AeroError::MalformedDeck {
                reason: "openbmp.aero_deck schema version is not 2",
            });
        }
        validate_deck_metadata(&parsed.provenance, parsed.validation)?;
        validate_schema2_axis_order(&parsed.axis_order.order)?;
        validate_schema2_interpolation(&parsed.interpolation)?;
        let internal_axis_order = canonicalise_axis_order(&parsed.axis_order.order);
        let axes = build_schema2_axes(&parsed.axis_order.order, &parsed.grid)?;
        validate_schema2_grid_keys_match_axis_order(&parsed.grid, &parsed.axis_order.order)?;
        let deck = AeroDeck::new_n_d(
            internal_axis_order,
            axes,
            parsed.coefficients.cn.data,
            parsed.coefficients.cd.data,
            parsed.coefficients.cm.data,
            parsed.reference.area_m2,
            parsed.reference.length_m,
        )?;
        // Schema-2 contract: extrapolation is fail-closed for control
        // surfaces. The `interpolation.extrapolation = "error"` literal
        // wire setting is validated above; the runtime policy is
        // therefore always FailClosed. Clamp is a Schema-1-only feature.
        Ok(deck.with_extrapolation_policy(ExtrapolationPolicy::FailClosed))
    }
}

impl LocalInclinationPanels {
    /// Parse a strict panel-mesh hypersonic aero deck from
    /// TOML text.
    ///
    /// This parser is intentionally separate from [`AeroDeck`]
    /// coefficient schemas. It constructs an in-memory
    /// [`LocalInclinationPanels`] method and routes all geometry
    /// validation through [`PanelMesh::new`].
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for TOML syntax /
    /// structural failures, unknown fields, unsupported schema
    /// versions, or mesh topology rejected by [`PanelMesh::new`].
    /// Returns [`AeroError::InvalidParameter`] for invalid method
    /// parameters and [`AeroError::NonFinite`] for non-finite mesh
    /// coordinates.
    pub fn load_from_str(s: &str) -> Result<Self, AeroError> {
        let parsed: PanelMeshDeckFile =
            toml::from_str(s).map_err(|_e| AeroError::MalformedDeck {
                reason: "panel-mesh aero TOML did not parse against schema",
            })?;
        if parsed.openbmp.panel_mesh_aero != PANEL_MESH_AERO_SCHEMA_VERSION_1 {
            return Err(AeroError::MalformedDeck {
                reason: "openbmp.panel_mesh_aero schema version must be 1",
            });
        }
        validate_deck_metadata(&parsed.provenance, parsed.validation)?;
        if !(parsed.cp_max.is_finite() && parsed.cp_max >= 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "panel-mesh aero cp_max must be finite and non-negative",
            });
        }
        let vertices = parsed
            .mesh
            .vertices_m
            .into_iter()
            .map(|[x, y, z]| Vector3::new(x, y, z))
            .collect();
        let geometry = PanelMesh::new(vertices, parsed.mesh.triangles)?;
        let mut panels = match parsed.method {
            PanelDeckMethod::ModifiedNewtonian => Self::modified_newtonian(geometry, parsed.cp_max),
        };
        panels.shadowing = parsed.shadowing;
        Ok(panels)
    }

    /// Parse a strict panel-mesh hypersonic aero deck from a
    /// TOML file on disk.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::Io`] if the file cannot be read.
    /// Otherwise returns the same errors as [`Self::load_from_str`].
    pub fn load_from_toml(path: &Path) -> Result<Self, AeroError> {
        let text = std::fs::read_to_string(path).map_err(|e| AeroError::Io {
            reason: format!(
                "could not read panel-mesh aero file {}: {e}",
                path.display()
            ),
        })?;
        Self::load_from_str(&text)
    }
}

// ---------------------------------------------------------------------
// Schema-2 helpers
// ---------------------------------------------------------------------

const SCHEMA2_AXIS_PREFIX: [&str; 3] = ["mach", "alpha", "beta"];

fn validate_schema2_axis_order(order: &[String]) -> Result<(), AeroError> {
    if order.len() < 3 {
        return Err(AeroError::MalformedDeck {
            reason: "axis_order.order must declare at least mach/alpha/beta",
        });
    }
    for (k, expected) in SCHEMA2_AXIS_PREFIX.iter().enumerate() {
        if order.get(k).map(String::as_str) != Some(*expected) {
            return Err(AeroError::MalformedDeck {
                reason: "axis_order.order must start with [mach, alpha, beta]",
            });
        }
    }
    let mut seen: BTreeSet<&str> = BTreeSet::new();
    for name in order {
        if !seen.insert(name.as_str()) {
            return Err(AeroError::MalformedDeck {
                reason: "axis_order.order contains a duplicate name",
            });
        }
    }
    if order.len() > 6 {
        // 3 base axes + at most 3 effector axes (`delta_e`, `delta_a`,
        // `delta_r` is the canonical example). The cap is 3
        // effector axes; larger decks are not supported.
        return Err(AeroError::MalformedDeck {
            reason: "schema-2 deck supports at most 3 effector axes (6 axes total)",
        });
    }
    let mut seen_bare_effector_names: BTreeSet<&str> = BTreeSet::new();
    for name in &order[3..] {
        let (bare, _unit) = schema2_effector_unit_suffix(name).ok_or(AeroError::MalformedDeck {
            reason: "schema-2 effector axis names must end with `_deg` or `_rad`",
        })?;
        if !seen_bare_effector_names.insert(bare) {
            return Err(AeroError::MalformedDeck {
                reason: "schema-2 effector axis names must be unique after stripping unit suffix",
            });
        }
    }
    Ok(())
}

fn schema2_effector_unit_suffix(name: &str) -> Option<(&str, &str)> {
    if let Some(bare) = name.strip_suffix("_deg") {
        Some((bare, "deg"))
    } else {
        name.strip_suffix("_rad").map(|bare| (bare, "rad"))
    }
}

fn validate_schema2_interpolation(interp: &InterpolationConfig) -> Result<(), AeroError> {
    if interp.method != "multilinear" {
        return Err(AeroError::MalformedDeck {
            reason: "schema-2 interpolation.method must be \"multilinear\"",
        });
    }
    if interp.extrapolation != "error" {
        return Err(AeroError::MalformedDeck {
            reason: "schema-2 interpolation.extrapolation must be \"error\"",
        });
    }
    Ok(())
}

/// Translate an `axis_order` name to the corresponding `[grid]` key.
/// The three base axes use the Schema-1 wire convention
/// (`mach` → `mach`, `alpha` → `alpha_deg`, `beta` → `beta_deg`);
/// effector axes use their `axis_order` name verbatim.
fn grid_key_for_axis_name(name: &str) -> &str {
    match name {
        "mach" => "mach",
        "alpha" => "alpha_deg",
        "beta" => "beta_deg",
        other => other,
    }
}

fn build_schema2_axes(
    order: &[String],
    grid: &BTreeMap<String, Vec<f64>>,
) -> Result<Vec<Vec<f64>>, AeroError> {
    let mut grids = Vec::with_capacity(order.len());
    for name in order {
        let key = grid_key_for_axis_name(name);
        let values = grid.get(key).ok_or(AeroError::MalformedDeck {
            reason: "schema-2 grid is missing an axis declared in axis_order",
        })?;
        grids.push(values.clone());
    }
    Ok(grids)
}

fn validate_schema2_grid_keys_match_axis_order(
    grid: &BTreeMap<String, Vec<f64>>,
    order: &[String],
) -> Result<(), AeroError> {
    let mut allowed: BTreeSet<&str> = BTreeSet::new();
    for name in order {
        allowed.insert(grid_key_for_axis_name(name));
    }
    for key in grid.keys() {
        if !allowed.contains(key.as_str()) {
            return Err(AeroError::MalformedDeck {
                reason: "schema-2 grid contains an axis not declared in axis_order",
            });
        }
    }
    Ok(())
}

/// Convert a wire-form `axis_order` list to the canonical internal
/// representation expected by [`AeroDeck::new_n_d`]: the first three
/// names are always `["mach", "alpha", "beta"]` (already the case for
/// well-formed schema-2 decks), and effector axis names retain their
/// unit suffix verbatim so the runner can match them against
/// scenario effector `unit` fields.
fn canonicalise_axis_order(order: &[String]) -> Vec<String> {
    order.to_vec()
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::{AeroCoefficients, AeroMethod};

    /// Minimal Schema-1 deck parser fixture (sibling .toml).
    fn minimal_deck_toml() -> String {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/minimal-deck.toml"
        ))
        .to_string()
    }

    /// Minimal Schema-2 deck parser fixture (4 axes, 1 effector axis).
    fn minimal_schema2_deck_toml() -> String {
        include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/minimal-schema2-deck.toml"
        ))
        .to_string()
    }

    fn minimal_panel_mesh_toml() -> &'static str {
        r#"
openbmp.panel_mesh_aero = 1
provenance = "unit-test academic square plate"
validation = "checked"
method = "modified-newtonian"
cp_max = 2.0

[mesh]
vertices_m = [
  [1.0, -0.5, -0.5],
  [1.0,  0.5, -0.5],
  [1.0,  0.5,  0.5],
  [1.0, -0.5,  0.5],
]
triangles = [[0, 1, 2], [0, 2, 3]]
"#
    }

    // ---------------------------------------------------------------
    // Schema-1 parser tests — preserved verbatim
    // ---------------------------------------------------------------

    #[test]
    fn round_trip_minimal_deck_lookup_at_corner() {
        let deck = AeroDeck::load_from_str(&minimal_deck_toml()).unwrap();
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
    fn parser_rejects_blank_provenance() {
        let toml_str = minimal_deck_toml().replace(
            "provenance         = \"test fixture; no real source\"",
            "provenance         = \"   \"",
        );
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

    #[test]
    fn parser_rejects_unknown_schema_version_above_two() {
        let toml_str =
            minimal_deck_toml().replace("openbmp.aero_deck = 1", "openbmp.aero_deck = 3");
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    // ---------------------------------------------------------------
    // Schema-2 parser tests
    // ---------------------------------------------------------------

    #[test]
    fn round_trips_minimal_schema2_deck() {
        let deck = AeroDeck::load_from_str(&minimal_schema2_deck_toml()).unwrap();
        assert_eq!(
            deck.axis_order(),
            &[
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "delta_e_deg".to_string(),
            ]
        );
        assert_eq!(deck.effector_axis_names(), &["delta_e_deg".to_string()]);
        assert_eq!(deck.reference_area_m2(), 1.0);
        assert_eq!(deck.reference_length_m(), 1.0);
    }

    #[test]
    fn schema2_default_extrapolation_is_fail_closed() {
        let deck = AeroDeck::load_from_str(&minimal_schema2_deck_toml()).unwrap();
        assert_eq!(deck.extrapolation_policy(), ExtrapolationPolicy::FailClosed);
    }

    #[test]
    fn schema2_lookup_at_zero_deflection_matches_schema1_companion() {
        // The schema-2 fixture is constructed so its delta_e_deg = 0
        // slice equals an equivalent schema-1 deck (CN = m + a, CD = 0,
        // CM = 0) at the same (mach, alpha, beta) grid points.
        let s2 = AeroDeck::load_from_str(&minimal_schema2_deck_toml()).unwrap();
        let mut def = std::collections::BTreeMap::new();
        def.insert("delta_e_deg", 0.0);
        for (m, a) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0)] {
            let r = s2.lookup(m, a, 0.0, &def).unwrap();
            assert_eq!(r.cn.to_bits(), (m + a).to_bits());
            assert_eq!(r.cd.to_bits(), 0.0_f64.to_bits());
            assert_eq!(r.cm.to_bits(), 0.0_f64.to_bits());
        }
    }

    #[test]
    fn schema2_corner_lookup_returns_stored_value_exactly() {
        let deck = AeroDeck::load_from_str(&minimal_schema2_deck_toml()).unwrap();
        let mut def = std::collections::BTreeMap::new();
        def.insert("delta_e_deg", 20.0);
        // (m=1, a=1, b=0, delta=20) should hit a stored corner.
        // Fixture CN at that corner = 1 + 1 + 0.1 * 20 = 4.0.
        let r = deck.lookup(1.0, 1.0, 0.0, &def).unwrap();
        assert_eq!(r.cn.to_bits(), 4.0_f64.to_bits());
    }

    #[test]
    fn schema2_rejects_missing_axis_order_block() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "[axis_order]\norder = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\"]\n",
            "",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_axis_order_not_starting_mach_alpha_beta() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\"]",
            "order = [\"alpha\", \"mach\", \"beta\", \"delta_e_deg\"]",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_effector_axis_name_without_unit_suffix() {
        // Strip the `_deg` suffix from the effector axis name in
        // both `[axis_order]` and `[grid]`. The schema-2 contract
        // requires every effector axis name to carry a unit suffix.
        let toml_str = minimal_schema2_deck_toml().replace("delta_e_deg", "delta_e");
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_interpolation_method_not_multilinear() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "method        = \"multilinear\"",
            "method        = \"trilinear\"",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_extrapolation_not_error() {
        let toml_str = minimal_schema2_deck_toml()
            .replace("extrapolation = \"error\"", "extrapolation = \"clamp\"");
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_grid_axis_not_declared_in_axis_order() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "delta_e_deg = [-20.0, 0.0, 20.0]",
            "delta_e_deg = [-20.0, 0.0, 20.0]\ndelta_a_deg = [-15.0, 0.0, 15.0]",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_axis_order_axis_missing_from_grid() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\"]",
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\", \"delta_a_deg\"]",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_inconsistent_coefficient_table_size() {
        // Per the inline-data tripwire, the malformed deck lives in a
        // sibling fixture file rather than as a raw string here.
        let toml_str = include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/tests/fixtures/schema2-inconsistent-coefficient-table.toml"
        ));
        assert!(matches!(
            AeroDeck::load_from_str(toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_duplicate_effector_axis_name() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\"]",
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\", \"delta_e_deg\"]",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn schema2_rejects_duplicate_effector_axis_name_after_unit_strip() {
        let toml_str = minimal_schema2_deck_toml().replace(
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\"]",
            "order = [\"mach\", \"alpha\", \"beta\", \"delta_e_deg\", \"delta_e_rad\"]",
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    // ---------------------------------------------------------------
    // Panel-mesh parser tests
    // ---------------------------------------------------------------

    fn panel_ctx(q: f64) -> crate::AeroContext {
        crate::AeroContext {
            mach: 12.0,
            alpha_deg: 0.0,
            beta_deg: 0.0,
            dynamic_pressure_pa: q,
        }
    }

    #[test]
    fn panel_mesh_parser_builds_local_inclination_method() {
        let panels = LocalInclinationPanels::load_from_str(minimal_panel_mesh_toml()).unwrap();
        assert_eq!(panels.method, crate::PanelMethod::ModifiedNewtonian);
        assert!(panels.shadowing);
        assert_eq!(panels.geometry.vertices().len(), 4);
        assert_eq!(panels.geometry.triangles().len(), 2);

        let fmt = panels.aero_force_moment_body(&panel_ctx(100.0)).unwrap();
        assert_eq!(fmt.force_n_body.x.to_bits(), (-200.0_f64).to_bits());
        assert_eq!(fmt.force_n_body.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(fmt.force_n_body.z.to_bits(), 0.0_f64.to_bits());
        assert_eq!(fmt.moment_n_m_body.norm().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn panel_mesh_parser_accepts_explicit_shadowing_false() {
        let toml_str =
            minimal_panel_mesh_toml().replace("cp_max = 2.0", "cp_max = 2.0\nshadowing = false");
        let panels = LocalInclinationPanels::load_from_str(&toml_str).unwrap();
        assert!(!panels.shadowing);
    }

    #[test]
    fn panel_mesh_parser_rejects_unknown_method() {
        let toml_str = minimal_panel_mesh_toml().replace(
            "method = \"modified-newtonian\"",
            "method = \"taylor-maccoll\"",
        );
        assert!(matches!(
            LocalInclinationPanels::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn panel_mesh_parser_rejects_unknown_field() {
        let toml_str = format!("{}\nunexpected = true\n", minimal_panel_mesh_toml());
        assert!(matches!(
            LocalInclinationPanels::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn panel_mesh_parser_rejects_bad_schema_version() {
        let toml_str = minimal_panel_mesh_toml()
            .replace("openbmp.panel_mesh_aero = 1", "openbmp.panel_mesh_aero = 2");
        assert!(matches!(
            LocalInclinationPanels::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn panel_mesh_parser_rejects_invalid_cp_max() {
        let toml_str = minimal_panel_mesh_toml().replace("cp_max = 2.0", "cp_max = -1.0");
        assert!(matches!(
            LocalInclinationPanels::load_from_str(&toml_str),
            Err(AeroError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn panel_mesh_parser_rejects_degenerate_mesh() {
        let toml_str = minimal_panel_mesh_toml().replace(
            "triangles = [[0, 1, 2], [0, 2, 3]]",
            "triangles = [[0, 1, 1]]",
        );
        assert!(matches!(
            LocalInclinationPanels::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }
}
