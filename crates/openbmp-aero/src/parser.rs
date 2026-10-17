//! Aero deck-file TOML parser. Coefficient decks use Schema 1
//! and Schema 2. A separate strict panel-mesh deck handles
//! [`crate::hypersonic::LocalInclinationPanels`], and strict ASCII / binary
//! STL ingestion builds [`crate::hypersonic::PanelMesh`] directly.
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
//!
//! [six_coefficients.cx] # optional explicit body-axis six-channel block
//! data = [/* row-major body-x force coefficient */]
//!
//! [six_coefficients.cy]
//! data = [/* body-y force coefficient */]
//!
//! [six_coefficients.cz]
//! data = [/* body-z force coefficient */]
//!
//! [six_coefficients.cl]
//! data = [/* roll-moment coefficient */]
//!
//! [six_coefficients.cm]
//! data = [/* pitch-moment coefficient */]
//!
//! [six_coefficients.cn]
//! data = [/* yaw-moment coefficient */]
//!
//! [dynamic_derivatives.cm_q] # optional dynamic derivative block
//! data = [/* row-major Cm_q values */]
//!
//! [dynamic_derivatives.cn_r]
//! data = [/* row-major Cn_r values */]
//!
//! [dynamic_derivatives.cl_p]
//! data = [/* row-major Cl_p values */]
//!
//! # Optional cross-coupling tables use the same pattern:
//! # cl_q, cl_r, cm_p, cm_r, cn_p, cn_q
//! ```
//!
//! # Optional one-sigma uncertainty tables
//!
//! Schema 1 and Schema 2 may include per-node coefficient one-sigma
//! uncertainty tables aligned exactly to the nominal coefficient grid:
//!
//! ```toml
//! [uncertainty.cn]
//! data = [/* row-major one-sigma CN values */]
//!
//! [uncertainty.cd]
//! data = [/* row-major one-sigma CD values */]
//!
//! [uncertainty.cm]
//! data = [/* row-major one-sigma CM values */]
//!
//! [uncertainty.correlation]
//! mach_length = 1.5
//! coefficient = [
//!   [1.0, 0.25, 0.0],
//!   [0.25, 1.0, 0.0],
//!   [0.0, 0.0, 1.0],
//! ]
//! six_coefficient = [
//!   [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
//!   [0.0, 1.0, 0.0, 0.0, 0.0, 0.0],
//!   [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
//!   [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
//!   [0.0, 0.0, 0.0, 0.0, 1.0, 0.0],
//!   [0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
//! ]
//!
//! [uncertainty.six_coefficients.cx] # optional, requires nominal six tables
//! data = [/* row-major one-sigma CX values */]
//! ```
//!
//! The tables are optional as a block. When present, all three channels
//! are required, finite, non-negative, and length-matched to the deck
//! node count. The optional `[uncertainty.correlation]` block records
//! deck-level exponential Mach-correlation metadata plus optional reduced
//! and six-channel coefficient-correlation matrices for higher-layer
//! sampling. Explicit six-channel uncertainty tables are optional as a block
//! under `[uncertainty.six_coefficients.*]` and require nominal
//! `[six_coefficients.*]` tables. When absent, [`AeroDeck::lookup_sigma`] and
//! [`AeroDeck::lookup_six_sigma`] return `Ok(None)`.
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

use crate::deck::{
    AeroDeck, AeroDynamicDerivativeTableSet, AeroUncertaintyCorrelation, ExtrapolationPolicy,
};
use crate::error::AeroError;
use crate::hypersonic::{LocalInclinationPanels, PanelMesh, PanelSkinFriction};

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
    six_coefficients: Option<SixCoefficients>,
    #[serde(default)]
    dynamic_derivatives: Option<DynamicDerivatives>,
    #[serde(default)]
    uncertainty: Option<UncertaintyTables>,
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
struct SixCoefficients {
    cx: CoefficientTable,
    cy: CoefficientTable,
    cz: CoefficientTable,
    cl: CoefficientTable,
    cm: CoefficientTable,
    cn: CoefficientTable,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct DynamicDerivatives {
    #[serde(default)]
    cl_p: Option<CoefficientTable>,
    #[serde(default)]
    cl_q: Option<CoefficientTable>,
    #[serde(default)]
    cl_r: Option<CoefficientTable>,
    #[serde(default)]
    cm_p: Option<CoefficientTable>,
    #[serde(default)]
    cm_q: Option<CoefficientTable>,
    #[serde(default)]
    cm_r: Option<CoefficientTable>,
    #[serde(default)]
    cn_p: Option<CoefficientTable>,
    #[serde(default)]
    cn_q: Option<CoefficientTable>,
    #[serde(default)]
    cn_r: Option<CoefficientTable>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct UncertaintyTables {
    cn: CoefficientTable,
    cd: CoefficientTable,
    cm: CoefficientTable,
    #[serde(default)]
    six_coefficients: Option<SixCoefficients>,
    #[serde(default)]
    correlation: Option<UncertaintyCorrelationConfig>,
}

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct UncertaintyCorrelationConfig {
    mach_length: f64,
    #[serde(default)]
    coefficient: Option<[[f64; 3]; 3]>,
    #[serde(default)]
    six_coefficient: Option<[[f64; 6]; 6]>,
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
    #[serde(default)]
    six_coefficients: Option<SixCoefficients>,
    #[serde(default)]
    dynamic_derivatives: Option<DynamicDerivatives>,
    #[serde(default)]
    uncertainty: Option<UncertaintyTables>,
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
    #[serde(default)]
    skin_friction: Option<PanelSkinFrictionToml>,
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

#[derive(Deserialize)]
#[serde(deny_unknown_fields)]
struct PanelSkinFrictionToml {
    reference_length_m: f64,
    #[serde(default = "default_panel_wall_to_edge_temperature_ratio")]
    wall_to_edge_temperature_ratio: f64,
    #[serde(default = "default_panel_transition_reynolds")]
    transition_reynolds: f64,
    #[serde(default)]
    roughness_over_length: f64,
}

#[derive(Copy, Clone, Debug, Deserialize, Eq, PartialEq)]
#[serde(rename_all = "kebab-case")]
enum PanelDeckMethod {
    ModifiedNewtonian,
}

const fn default_panel_shadowing() -> bool {
    true
}

const fn default_panel_wall_to_edge_temperature_ratio() -> f64 {
    1.0
}

const fn default_panel_transition_reynolds() -> f64 {
    PanelSkinFriction::DEFAULT_TRANSITION_REYNOLDS
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
        )?
        .with_extrapolation_policy(extrapolation);
        let deck = apply_six_coefficient_tables(deck, parsed.six_coefficients)?;
        let deck = apply_dynamic_derivative_tables(deck, parsed.dynamic_derivatives)?;
        apply_uncertainty_tables(deck, parsed.uncertainty)
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
        let deck = deck.with_extrapolation_policy(ExtrapolationPolicy::FailClosed);
        let deck = apply_six_coefficient_tables(deck, parsed.six_coefficients)?;
        let deck = apply_dynamic_derivative_tables(deck, parsed.dynamic_derivatives)?;
        apply_uncertainty_tables(deck, parsed.uncertainty)
    }
}

fn apply_six_coefficient_tables(
    deck: AeroDeck,
    six: Option<SixCoefficients>,
) -> Result<AeroDeck, AeroError> {
    let Some(six) = six else {
        return Ok(deck);
    };
    deck.with_six_coefficient_tables(
        six.cx.data,
        six.cy.data,
        six.cz.data,
        six.cl.data,
        six.cm.data,
        six.cn.data,
    )
}

fn apply_dynamic_derivative_tables(
    deck: AeroDeck,
    dynamic: Option<DynamicDerivatives>,
) -> Result<AeroDeck, AeroError> {
    let Some(dynamic) = dynamic else {
        return Ok(deck);
    };
    if dynamic.cl_p.is_none()
        && dynamic.cl_q.is_none()
        && dynamic.cl_r.is_none()
        && dynamic.cm_p.is_none()
        && dynamic.cm_q.is_none()
        && dynamic.cm_r.is_none()
        && dynamic.cn_p.is_none()
        && dynamic.cn_q.is_none()
        && dynamic.cn_r.is_none()
    {
        return Err(AeroError::MalformedDeck {
            reason: "dynamic derivative block contains no tables",
        });
    }
    let expected = deck.node_count();
    let table_or_zeros = |table: Option<CoefficientTable>| {
        table
            .map(|table| table.data)
            .unwrap_or_else(|| vec![0.0; expected])
    };
    deck.with_dynamic_derivative_table_set(AeroDynamicDerivativeTableSet {
        cl_p: table_or_zeros(dynamic.cl_p),
        cl_q: table_or_zeros(dynamic.cl_q),
        cl_r: table_or_zeros(dynamic.cl_r),
        cm_p: table_or_zeros(dynamic.cm_p),
        cm_q: table_or_zeros(dynamic.cm_q),
        cm_r: table_or_zeros(dynamic.cm_r),
        cn_p: table_or_zeros(dynamic.cn_p),
        cn_q: table_or_zeros(dynamic.cn_q),
        cn_r: table_or_zeros(dynamic.cn_r),
    })
}

fn apply_uncertainty_tables(
    deck: AeroDeck,
    uncertainty: Option<UncertaintyTables>,
) -> Result<AeroDeck, AeroError> {
    let Some(uncertainty) = uncertainty else {
        return Ok(deck);
    };
    let mut deck = deck.with_sigma_tables(
        uncertainty.cn.data,
        uncertainty.cd.data,
        uncertainty.cm.data,
    )?;
    if let Some(six) = uncertainty.six_coefficients {
        deck = deck.with_six_sigma_tables(
            six.cx.data,
            six.cy.data,
            six.cz.data,
            six.cl.data,
            six.cm.data,
            six.cn.data,
        )?;
    }
    if let Some(correlation) = uncertainty.correlation {
        let mut correlation_metadata =
            AeroUncertaintyCorrelation::exponential_mach(correlation.mach_length)?;
        if let Some(coefficient) = correlation.coefficient {
            correlation_metadata =
                correlation_metadata.with_coefficient_correlation(coefficient)?;
        }
        if let Some(coefficient) = correlation.six_coefficient {
            correlation_metadata =
                correlation_metadata.with_six_coefficient_correlation(coefficient)?;
        }
        deck = deck.with_uncertainty_correlation(correlation_metadata);
    }
    Ok(deck)
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
        if let Some(skin_friction) = parsed.skin_friction {
            panels = panels.with_skin_friction(PanelSkinFriction::new(
                skin_friction.reference_length_m,
                skin_friction.wall_to_edge_temperature_ratio,
                skin_friction.transition_reynolds,
                skin_friction.roughness_over_length,
            )?);
        }
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

impl PanelMesh {
    /// Parse a strict ASCII STL mesh from text.
    ///
    /// This accepts the standard `solid` / `facet normal` / `outer loop` /
    /// three-`vertex` / `endloop` / `endfacet` / `endsolid` record sequence.
    /// Vertex coordinates are metres in the body frame. Duplicate vertices are
    /// deduplicated by exact floating-point bit pattern after canonicalising
    /// signed zero.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for record-order, token-count,
    /// numeric parse, or geometry validation failures, and
    /// [`AeroError::NonFinite`] for NaN / Inf normals or coordinates.
    pub fn load_ascii_stl_from_str(s: &str) -> Result<Self, AeroError> {
        let mut lines = s.lines().map(str::trim).filter(|line| !line.is_empty());
        let first = next_ascii_stl_line(&mut lines, "ASCII STL must start with a `solid` record")?;
        let first_fields = ascii_stl_fields(first);
        if first_fields.first().copied() != Some("solid") {
            return Err(AeroError::MalformedDeck {
                reason: "ASCII STL must start with a `solid` record",
            });
        }

        let mut vertices = Vec::new();
        let mut vertex_ids = BTreeMap::new();
        let mut triangles = Vec::new();
        loop {
            let line =
                next_ascii_stl_line(&mut lines, "ASCII STL ended before an `endsolid` record")?;
            let fields = ascii_stl_fields(line);
            match fields.first().copied() {
                Some("endsolid") => {
                    if lines.next().is_some() {
                        return Err(AeroError::MalformedDeck {
                            reason: "ASCII STL contains records after `endsolid`",
                        });
                    }
                    break;
                }
                Some("facet") => {
                    validate_ascii_stl_facet_header(&fields)?;
                    require_ascii_stl_record(
                        &ascii_stl_fields(next_ascii_stl_line(
                            &mut lines,
                            "ASCII STL facet is missing `outer loop`",
                        )?),
                        &["outer", "loop"],
                        "ASCII STL facet must contain `outer loop`",
                    )?;
                    let mut triangle = [0_u32; 3];
                    for index in &mut triangle {
                        let vertex =
                            parse_ascii_stl_vertex(&ascii_stl_fields(next_ascii_stl_line(
                                &mut lines,
                                "ASCII STL facet is missing a `vertex` record",
                            )?))?;
                        *index = intern_ascii_stl_vertex(&mut vertices, &mut vertex_ids, vertex)?;
                    }
                    require_ascii_stl_record(
                        &ascii_stl_fields(next_ascii_stl_line(
                            &mut lines,
                            "ASCII STL facet is missing `endloop`",
                        )?),
                        &["endloop"],
                        "ASCII STL facet must close with `endloop`",
                    )?;
                    require_ascii_stl_record(
                        &ascii_stl_fields(next_ascii_stl_line(
                            &mut lines,
                            "ASCII STL facet is missing `endfacet`",
                        )?),
                        &["endfacet"],
                        "ASCII STL facet must close with `endfacet`",
                    )?;
                    triangles.push(triangle);
                }
                _ => {
                    return Err(AeroError::MalformedDeck {
                        reason: "ASCII STL expected `facet` or `endsolid` record",
                    });
                }
            }
        }
        Self::new(vertices, triangles)
    }

    /// Parse a strict ASCII STL mesh from a file on disk.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::Io`] if the file cannot be read. Otherwise returns
    /// the same errors as [`Self::load_ascii_stl_from_str`].
    pub fn load_ascii_stl(path: &Path) -> Result<Self, AeroError> {
        let text = std::fs::read_to_string(path).map_err(|e| AeroError::Io {
            reason: format!("could not read ASCII STL file {}: {e}", path.display()),
        })?;
        Self::load_ascii_stl_from_str(&text)
    }

    /// Parse a strict binary STL mesh from bytes.
    ///
    /// The parser consumes the standard 80-byte header, little-endian `u32`
    /// triangle count, and fixed 50-byte triangle records. Vertex coordinates are
    /// metres in the body frame. Duplicate vertices are deduplicated by exact
    /// floating-point bit pattern after converting binary `f32` fields to `f64`
    /// and canonicalising signed zero.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for byte-count, triangle-count, or
    /// geometry validation failures, and [`AeroError::NonFinite`] for NaN / Inf
    /// normals or coordinates.
    pub fn load_binary_stl_from_bytes(bytes: &[u8]) -> Result<Self, AeroError> {
        const BINARY_STL_HEADER_BYTES: usize = 80;
        const BINARY_STL_COUNT_BYTES: usize = 4;
        const BINARY_STL_TRIANGLE_BYTES: usize = 50;
        const BINARY_STL_ATTRIBUTE_BYTES: usize = 2;

        if bytes.len() < BINARY_STL_HEADER_BYTES + BINARY_STL_COUNT_BYTES {
            return Err(AeroError::MalformedDeck {
                reason: "binary STL is shorter than the header and triangle count",
            });
        }
        let count_start = BINARY_STL_HEADER_BYTES;
        let count_end = count_start + BINARY_STL_COUNT_BYTES;
        let triangle_count =
            u32::from_le_bytes(bytes[count_start..count_end].try_into().map_err(|_| {
                AeroError::MalformedDeck {
                    reason: "binary STL triangle count did not parse",
                }
            })?) as usize;
        let payload_bytes = triangle_count
            .checked_mul(BINARY_STL_TRIANGLE_BYTES)
            .and_then(|value| value.checked_add(BINARY_STL_HEADER_BYTES + BINARY_STL_COUNT_BYTES))
            .ok_or(AeroError::MalformedDeck {
                reason: "binary STL byte count overflows usize",
            })?;
        if bytes.len() != payload_bytes {
            return Err(AeroError::MalformedDeck {
                reason: "binary STL byte count does not match triangle count",
            });
        }

        let mut vertices = Vec::new();
        let mut vertex_ids = BTreeMap::new();
        let mut triangles = Vec::with_capacity(triangle_count);
        let mut offset = payload_bytes - triangle_count * BINARY_STL_TRIANGLE_BYTES;
        for _ in 0..triangle_count {
            for _ in 0..3 {
                let _ = parse_binary_stl_f32(
                    bytes,
                    &mut offset,
                    "binary STL normal component is NaN or Inf",
                )?;
            }
            let mut triangle = [0_u32; 3];
            for index in &mut triangle {
                let vertex = Vector3::new(
                    parse_binary_stl_f32(
                        bytes,
                        &mut offset,
                        "binary STL vertex coordinate is NaN or Inf",
                    )?,
                    parse_binary_stl_f32(
                        bytes,
                        &mut offset,
                        "binary STL vertex coordinate is NaN or Inf",
                    )?,
                    parse_binary_stl_f32(
                        bytes,
                        &mut offset,
                        "binary STL vertex coordinate is NaN or Inf",
                    )?,
                );
                *index = intern_binary_stl_vertex(&mut vertices, &mut vertex_ids, vertex)?;
            }
            triangles.push(triangle);
            offset += BINARY_STL_ATTRIBUTE_BYTES;
        }
        Self::new(vertices, triangles)
    }

    /// Parse a strict binary STL mesh from a file on disk.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::Io`] if the file cannot be read. Otherwise returns
    /// the same errors as [`Self::load_binary_stl_from_bytes`].
    pub fn load_binary_stl(path: &Path) -> Result<Self, AeroError> {
        let bytes = std::fs::read(path).map_err(|e| AeroError::Io {
            reason: format!("could not read binary STL file {}: {e}", path.display()),
        })?;
        Self::load_binary_stl_from_bytes(&bytes)
    }
}

fn next_ascii_stl_line<'a, I>(lines: &mut I, reason: &'static str) -> Result<&'a str, AeroError>
where
    I: Iterator<Item = &'a str>,
{
    lines.next().ok_or(AeroError::MalformedDeck { reason })
}

fn ascii_stl_fields(line: &str) -> Vec<&str> {
    line.split_whitespace().collect()
}

fn require_ascii_stl_record(
    fields: &[&str],
    expected: &[&str],
    reason: &'static str,
) -> Result<(), AeroError> {
    if fields == expected {
        Ok(())
    } else {
        Err(AeroError::MalformedDeck { reason })
    }
}

fn validate_ascii_stl_facet_header(fields: &[&str]) -> Result<(), AeroError> {
    if fields.len() != 5 || fields[0] != "facet" || fields[1] != "normal" {
        return Err(AeroError::MalformedDeck {
            reason: "ASCII STL facet header must be `facet normal nx ny nz`",
        });
    }
    for value in &fields[2..] {
        let _ = parse_ascii_stl_f64(
            value,
            "ASCII STL normal component did not parse as f64",
            "ASCII STL normal component is NaN or Inf",
        )?;
    }
    Ok(())
}

fn parse_ascii_stl_vertex(fields: &[&str]) -> Result<Vector3<f64>, AeroError> {
    if fields.len() != 4 || fields[0] != "vertex" {
        return Err(AeroError::MalformedDeck {
            reason: "ASCII STL vertex record must be `vertex x y z`",
        });
    }
    Ok(Vector3::new(
        parse_ascii_stl_f64(
            fields[1],
            "ASCII STL vertex coordinate did not parse as f64",
            "ASCII STL vertex coordinate is NaN or Inf",
        )?,
        parse_ascii_stl_f64(
            fields[2],
            "ASCII STL vertex coordinate did not parse as f64",
            "ASCII STL vertex coordinate is NaN or Inf",
        )?,
        parse_ascii_stl_f64(
            fields[3],
            "ASCII STL vertex coordinate did not parse as f64",
            "ASCII STL vertex coordinate is NaN or Inf",
        )?,
    ))
}

fn parse_ascii_stl_f64(
    value: &str,
    parse_reason: &'static str,
    finite_reason: &'static str,
) -> Result<f64, AeroError> {
    let parsed = value.parse::<f64>().map_err(|_| AeroError::MalformedDeck {
        reason: parse_reason,
    })?;
    if parsed.is_finite() {
        Ok(canonical_ascii_stl_coord(parsed))
    } else {
        Err(AeroError::NonFinite {
            reason: finite_reason,
        })
    }
}

fn intern_ascii_stl_vertex(
    vertices: &mut Vec<Vector3<f64>>,
    vertex_ids: &mut BTreeMap<(u64, u64, u64), u32>,
    vertex: Vector3<f64>,
) -> Result<u32, AeroError> {
    let key = (vertex.x.to_bits(), vertex.y.to_bits(), vertex.z.to_bits());
    if let Some(index) = vertex_ids.get(&key) {
        return Ok(*index);
    }
    let index = u32::try_from(vertices.len()).map_err(|_| AeroError::MalformedDeck {
        reason: "ASCII STL unique vertex count exceeds u32 index range",
    })?;
    vertices.push(vertex);
    vertex_ids.insert(key, index);
    Ok(index)
}

fn parse_binary_stl_f32(
    bytes: &[u8],
    offset: &mut usize,
    finite_reason: &'static str,
) -> Result<f64, AeroError> {
    let end = offset.checked_add(4).ok_or(AeroError::MalformedDeck {
        reason: "binary STL field offset overflows usize",
    })?;
    let raw = bytes
        .get(*offset..end)
        .ok_or(AeroError::MalformedDeck {
            reason: "binary STL ended before a required field",
        })?
        .try_into()
        .map_err(|_| AeroError::MalformedDeck {
            reason: "binary STL field did not parse",
        })?;
    *offset = end;
    let parsed = f32::from_le_bytes(raw) as f64;
    if parsed.is_finite() {
        Ok(canonical_ascii_stl_coord(parsed))
    } else {
        Err(AeroError::NonFinite {
            reason: finite_reason,
        })
    }
}

fn intern_binary_stl_vertex(
    vertices: &mut Vec<Vector3<f64>>,
    vertex_ids: &mut BTreeMap<(u64, u64, u64), u32>,
    vertex: Vector3<f64>,
) -> Result<u32, AeroError> {
    let key = (vertex.x.to_bits(), vertex.y.to_bits(), vertex.z.to_bits());
    if let Some(index) = vertex_ids.get(&key) {
        return Ok(*index);
    }
    let index = u32::try_from(vertices.len()).map_err(|_| AeroError::MalformedDeck {
        reason: "binary STL unique vertex count exceeds u32 index range",
    })?;
    vertices.push(vertex);
    vertex_ids.insert(key, index);
    Ok(index)
}

fn canonical_ascii_stl_coord(value: f64) -> f64 {
    if value == 0.0 { 0.0 } else { value }
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

    fn minimal_panel_ascii_stl() -> &'static str {
        r#"
solid square
  facet normal 1 0 0
    outer loop
      vertex 1 -0.5 -0.5
      vertex 1 0.5 -0.5
      vertex 1 0.5 0.5
    endloop
  endfacet
  facet normal 1 0 0
    outer loop
      vertex 1 -0.5 -0.5
      vertex 1 0.5 0.5
      vertex 1 -0.5 0.5
    endloop
  endfacet
endsolid square
"#
    }

    fn minimal_panel_binary_stl() -> Vec<u8> {
        let mut bytes = vec![0_u8; 80];
        bytes.extend_from_slice(&2_u32.to_le_bytes());
        push_binary_stl_triangle(
            &mut bytes,
            [1.0, 0.0, 0.0],
            [[1.0, -0.5, -0.5], [1.0, 0.5, -0.5], [1.0, 0.5, 0.5]],
        );
        push_binary_stl_triangle(
            &mut bytes,
            [1.0, 0.0, 0.0],
            [[1.0, -0.5, -0.5], [1.0, 0.5, 0.5], [1.0, -0.5, 0.5]],
        );
        bytes
    }

    fn push_binary_stl_triangle(bytes: &mut Vec<u8>, normal: [f32; 3], vertices: [[f32; 3]; 3]) {
        for value in normal {
            bytes.extend_from_slice(&value.to_le_bytes());
        }
        for vertex in vertices {
            for value in vertex {
                bytes.extend_from_slice(&value.to_le_bytes());
            }
        }
        bytes.extend_from_slice(&0_u16.to_le_bytes());
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
    fn parser_accepts_optional_uncertainty_tables() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1, 0.2, 0.3, 0.4]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n",
            minimal_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let sigma = deck
            .lookup_sigma(0.5, 0.5, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();

        assert!(deck.has_sigma_tables());
        assert_eq!(sigma.cn.to_bits(), 0.25_f64.to_bits());
        assert_eq!(sigma.cd.to_bits(), 0.35_f64.to_bits());
        assert_eq!(sigma.cm.to_bits(), 0.45_f64.to_bits());
    }

    #[test]
    fn parser_accepts_optional_six_coefficient_tables_and_sigmas() {
        let toml_str = format!(
            "{}\n\
             [six_coefficients.cx]\n\
             data = [1.0, 2.0, 3.0, 4.0]\n\
             \n\
             [six_coefficients.cy]\n\
             data = [2.0, 3.0, 4.0, 5.0]\n\
             \n\
             [six_coefficients.cz]\n\
             data = [3.0, 4.0, 5.0, 6.0]\n\
             \n\
             [six_coefficients.cl]\n\
             data = [4.0, 5.0, 6.0, 7.0]\n\
             \n\
             [six_coefficients.cm]\n\
             data = [5.0, 6.0, 7.0, 8.0]\n\
             \n\
             [six_coefficients.cn]\n\
             data = [6.0, 7.0, 8.0, 9.0]\n\
             \n\
             [uncertainty.cn]\n\
             data = [0.0, 0.0, 0.0, 0.0]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.0, 0.0, 0.0, 0.0]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.0, 0.0, 0.0, 0.0]\n\
             \n\
             [uncertainty.six_coefficients.cx]\n\
             data = [0.10, 0.10, 0.10, 0.10]\n\
             \n\
             [uncertainty.six_coefficients.cy]\n\
             data = [0.20, 0.20, 0.20, 0.20]\n\
             \n\
             [uncertainty.six_coefficients.cz]\n\
             data = [0.30, 0.30, 0.30, 0.30]\n\
             \n\
             [uncertainty.six_coefficients.cl]\n\
             data = [0.40, 0.40, 0.40, 0.40]\n\
             \n\
             [uncertainty.six_coefficients.cm]\n\
             data = [0.50, 0.50, 0.50, 0.50]\n\
             \n\
             [uncertainty.six_coefficients.cn]\n\
             data = [0.60, 0.60, 0.60, 0.60]\n",
            minimal_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let six = deck
            .lookup_six(0.5, 0.5, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();
        let sigma = deck
            .lookup_six_sigma(0.5, 0.5, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();

        assert!(deck.has_six_coefficient_tables());
        assert!(deck.has_six_sigma_tables());
        assert_eq!(six.cx.to_bits(), 2.5_f64.to_bits());
        assert_eq!(six.cy.to_bits(), 3.5_f64.to_bits());
        assert_eq!(six.cz.to_bits(), 4.5_f64.to_bits());
        assert_eq!(six.cl.to_bits(), 5.5_f64.to_bits());
        assert_eq!(six.cm.to_bits(), 6.5_f64.to_bits());
        assert_eq!(six.cn.to_bits(), 7.5_f64.to_bits());
        assert_eq!(sigma.cx.to_bits(), 0.10_f64.to_bits());
        assert_eq!(sigma.cy.to_bits(), 0.20_f64.to_bits());
        assert_eq!(sigma.cz.to_bits(), 0.30_f64.to_bits());
        assert_eq!(sigma.cl.to_bits(), 0.40_f64.to_bits());
        assert_eq!(sigma.cm.to_bits(), 0.50_f64.to_bits());
        assert_eq!(sigma.cn.to_bits(), 0.60_f64.to_bits());
    }

    #[test]
    fn parser_accepts_optional_dynamic_derivative_tables() {
        let toml_str = format!(
            "{}\n\
             [dynamic_derivatives.cm_q]\n\
             data = [0.0, 1.0, 2.0, 3.0]\n\
             \n\
             [dynamic_derivatives.cn_r]\n\
             data = [10.0, 11.0, 12.0, 13.0]\n\
             \n\
             [dynamic_derivatives.cl_p]\n\
             data = [20.0, 21.0, 22.0, 23.0]\n",
            minimal_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let derivatives = deck
            .lookup_dynamic_derivatives(0.5, 0.5, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();

        assert!(deck.has_dynamic_derivative_tables());
        assert_eq!(derivatives.cm_q.to_bits(), 1.5_f64.to_bits());
        assert_eq!(derivatives.cn_r.to_bits(), 11.5_f64.to_bits());
        assert_eq!(derivatives.cl_p.to_bits(), 21.5_f64.to_bits());
        assert_eq!(derivatives.cl_q.to_bits(), 0.0_f64.to_bits());
        assert_eq!(derivatives.cm_p.to_bits(), 0.0_f64.to_bits());
        assert_eq!(derivatives.cn_q.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn parser_accepts_full_dynamic_derivative_matrix_tables() {
        let toml_str = format!(
            "{}\n\
             [dynamic_derivatives.cl_p]\n\
             data = [0.0, 1.0, 2.0, 3.0]\n\
             \n\
             [dynamic_derivatives.cl_q]\n\
             data = [10.0, 11.0, 12.0, 13.0]\n\
             \n\
             [dynamic_derivatives.cl_r]\n\
             data = [20.0, 21.0, 22.0, 23.0]\n\
             \n\
             [dynamic_derivatives.cm_p]\n\
             data = [30.0, 31.0, 32.0, 33.0]\n\
             \n\
             [dynamic_derivatives.cm_q]\n\
             data = [40.0, 41.0, 42.0, 43.0]\n\
             \n\
             [dynamic_derivatives.cm_r]\n\
             data = [50.0, 51.0, 52.0, 53.0]\n\
             \n\
             [dynamic_derivatives.cn_p]\n\
             data = [60.0, 61.0, 62.0, 63.0]\n\
             \n\
             [dynamic_derivatives.cn_q]\n\
             data = [70.0, 71.0, 72.0, 73.0]\n\
             \n\
             [dynamic_derivatives.cn_r]\n\
             data = [80.0, 81.0, 82.0, 83.0]\n",
            minimal_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let derivatives = deck
            .lookup_dynamic_derivatives(0.5, 0.5, 0.0, &std::collections::BTreeMap::new())
            .unwrap()
            .unwrap();

        assert!(deck.has_dynamic_derivative_tables());
        assert_eq!(derivatives.cl_p.to_bits(), 1.5_f64.to_bits());
        assert_eq!(derivatives.cl_q.to_bits(), 11.5_f64.to_bits());
        assert_eq!(derivatives.cl_r.to_bits(), 21.5_f64.to_bits());
        assert_eq!(derivatives.cm_p.to_bits(), 31.5_f64.to_bits());
        assert_eq!(derivatives.cm_q.to_bits(), 41.5_f64.to_bits());
        assert_eq!(derivatives.cm_r.to_bits(), 51.5_f64.to_bits());
        assert_eq!(derivatives.cn_p.to_bits(), 61.5_f64.to_bits());
        assert_eq!(derivatives.cn_q.to_bits(), 71.5_f64.to_bits());
        assert_eq!(derivatives.cn_r.to_bits(), 81.5_f64.to_bits());
    }

    #[test]
    fn parser_rejects_empty_dynamic_derivative_block() {
        let toml_str = format!("{}\n[dynamic_derivatives]\n", minimal_deck_toml());
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_dynamic_derivative_table_length_mismatch() {
        let toml_str = format!(
            "{}\n\
             [dynamic_derivatives.cm_q]\n\
             data = [0.0, 1.0, 2.0]\n\
             \n\
             [dynamic_derivatives.cn_r]\n\
             data = [10.0, 11.0, 12.0, 13.0]\n\
             \n\
             [dynamic_derivatives.cl_p]\n\
             data = [20.0, 21.0, 22.0, 23.0]\n",
            minimal_deck_toml()
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_accepts_optional_uncertainty_correlation_metadata() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1, 0.2, 0.3, 0.4]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n\
             \n\
             [uncertainty.correlation]\n\
             mach_length = 1.5\n\
             coefficient = [[1.0, 0.25, 0.0], [0.25, 1.0, 0.0], [0.0, 0.0, 1.0]]\n\
             six_coefficient = [[1.0, 0.25, 0.0, 0.0, 0.0, 0.0], [0.25, 1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.0, -0.2], [0.0, 0.0, 0.0, 0.0, -0.2, 1.0]]\n",
            minimal_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let correlation = deck.uncertainty_correlation().unwrap();
        let six_coefficient = [
            [1.0, 0.25, 0.0, 0.0, 0.0, 0.0],
            [0.25, 1.0, 0.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 1.0, 0.0, 0.0],
            [0.0, 0.0, 0.0, 0.0, 1.0, -0.2],
            [0.0, 0.0, 0.0, 0.0, -0.2, 1.0],
        ];

        assert_eq!(correlation.mach_length().to_bits(), 1.5_f64.to_bits());
        assert_eq!(
            correlation.coefficient_correlation(),
            Some([[1.0, 0.25, 0.0], [0.25, 1.0, 0.0], [0.0, 0.0, 1.0]])
        );
        assert_eq!(
            correlation.six_coefficient_correlation(),
            Some(six_coefficient)
        );
    }

    #[test]
    fn parser_rejects_invalid_uncertainty_correlation_metadata() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1, 0.2, 0.3, 0.4]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n\
             \n\
             [uncertainty.correlation]\n\
             mach_length = 0.0\n",
            minimal_deck_toml()
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn parser_rejects_invalid_uncertainty_coefficient_correlation_metadata() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1, 0.2, 0.3, 0.4]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n\
             \n\
             [uncertainty.correlation]\n\
             mach_length = 1.5\n\
             coefficient = [[1.0, 0.25, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]]\n",
            minimal_deck_toml()
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn parser_rejects_invalid_uncertainty_six_coefficient_correlation_metadata() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1, 0.2, 0.3, 0.4]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n\
             \n\
             [uncertainty.correlation]\n\
             mach_length = 1.5\n\
             six_coefficient = [[1.0, 0.25, 0.0, 0.0, 0.0, 0.0], [0.0, 1.0, 0.0, 0.0, 0.0, 0.0], [0.0, 0.0, 1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0, 0.0, 0.0], [0.0, 0.0, 0.0, 0.0, 1.0, 0.0], [0.0, 0.0, 0.0, 0.0, 0.0, 1.0]]\n",
            minimal_deck_toml()
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn parser_rejects_uncertainty_table_length_mismatch() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n",
            minimal_deck_toml()
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn parser_rejects_negative_uncertainty_value() {
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [-0.1, 0.2, 0.3, 0.4]\n\
             \n\
             [uncertainty.cd]\n\
             data = [0.2, 0.3, 0.4, 0.5]\n\
             \n\
             [uncertainty.cm]\n\
             data = [0.3, 0.4, 0.5, 0.6]\n",
            minimal_deck_toml()
        );
        assert!(matches!(
            AeroDeck::load_from_str(&toml_str),
            Err(AeroError::InvalidParameter { .. }),
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
    fn schema2_parser_accepts_optional_uncertainty_tables() {
        let zeros = "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0";
        let toml_str = format!(
            "{}\n\
             [uncertainty.cn]\n\
             data = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0, 1.1, 1.2]\n\
             \n\
             [uncertainty.cd]\n\
             data = [{zeros}]\n\
             \n\
             [uncertainty.cm]\n\
             data = [{zeros}]\n",
            minimal_schema2_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let mut def = std::collections::BTreeMap::new();
        def.insert("delta_e_deg", 0.0);
        let sigma = deck.lookup_sigma(0.0, 0.0, 0.0, &def).unwrap().unwrap();

        assert!(deck.has_sigma_tables());
        assert_eq!(sigma.cn.to_bits(), 0.2_f64.to_bits());
        assert_eq!(sigma.cd.to_bits(), 0.0_f64.to_bits());
        assert_eq!(sigma.cm.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn schema2_parser_accepts_optional_six_coefficient_tables() {
        let ones = "1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0";
        let twos = "2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0";
        let threes = "3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0";
        let fours = "4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0";
        let fives = "5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0, 5.0";
        let sixes = "6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0, 6.0";
        let toml_str = format!(
            "{}\n\
             [six_coefficients.cx]\n\
             data = [{ones}]\n\
             \n\
             [six_coefficients.cy]\n\
             data = [{twos}]\n\
             \n\
             [six_coefficients.cz]\n\
             data = [{threes}]\n\
             \n\
             [six_coefficients.cl]\n\
             data = [{fours}]\n\
             \n\
             [six_coefficients.cm]\n\
             data = [{fives}]\n\
             \n\
             [six_coefficients.cn]\n\
             data = [{sixes}]\n",
            minimal_schema2_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let mut def = std::collections::BTreeMap::new();
        def.insert("delta_e_deg", 0.0);
        let six = deck.lookup_six(0.5, 0.5, 0.0, &def).unwrap().unwrap();

        assert!(deck.has_six_coefficient_tables());
        assert_eq!(six.cx.to_bits(), 1.0_f64.to_bits());
        assert_eq!(six.cy.to_bits(), 2.0_f64.to_bits());
        assert_eq!(six.cz.to_bits(), 3.0_f64.to_bits());
        assert_eq!(six.cl.to_bits(), 4.0_f64.to_bits());
        assert_eq!(six.cm.to_bits(), 5.0_f64.to_bits());
        assert_eq!(six.cn.to_bits(), 6.0_f64.to_bits());
    }

    #[test]
    fn schema2_parser_accepts_optional_dynamic_derivative_tables() {
        let ones = "1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0";
        let twos = "2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0";
        let threes = "3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0, 3.0";
        let toml_str = format!(
            "{}\n\
             [dynamic_derivatives.cm_q]\n\
             data = [{ones}]\n\
             \n\
             [dynamic_derivatives.cn_r]\n\
             data = [{twos}]\n\
             \n\
             [dynamic_derivatives.cl_p]\n\
             data = [{threes}]\n",
            minimal_schema2_deck_toml()
        );
        let deck = AeroDeck::load_from_str(&toml_str).unwrap();
        let mut def = std::collections::BTreeMap::new();
        def.insert("delta_e_deg", 0.0);
        let derivatives = deck
            .lookup_dynamic_derivatives(0.5, 0.5, 0.0, &def)
            .unwrap()
            .unwrap();

        assert!(deck.has_dynamic_derivative_tables());
        assert_eq!(derivatives.cm_q.to_bits(), 1.0_f64.to_bits());
        assert_eq!(derivatives.cn_r.to_bits(), 2.0_f64.to_bits());
        assert_eq!(derivatives.cl_p.to_bits(), 3.0_f64.to_bits());
        assert_eq!(derivatives.cl_r.to_bits(), 0.0_f64.to_bits());
        assert_eq!(derivatives.cm_p.to_bits(), 0.0_f64.to_bits());
        assert_eq!(derivatives.cn_q.to_bits(), 0.0_f64.to_bits());
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
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: nalgebra::Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
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
    fn panel_mesh_parser_accepts_skin_friction_block() {
        let toml_str = minimal_panel_mesh_toml().replace(
            "cp_max = 2.0",
            "cp_max = 2.0\n\n[skin_friction]\nreference_length_m = 1.0\nwall_to_edge_temperature_ratio = 0.9\ntransition_reynolds = 600000.0\nroughness_over_length = 0.0001",
        );
        let panels = LocalInclinationPanels::load_from_str(&toml_str).unwrap();
        let skin_friction = panels.skin_friction.unwrap();
        assert_eq!(
            skin_friction.reference_length_m.to_bits(),
            1.0_f64.to_bits()
        );
        assert_eq!(
            skin_friction.wall_to_edge_temperature_ratio.to_bits(),
            0.9_f64.to_bits()
        );
        assert_eq!(
            skin_friction.transition_reynolds.to_bits(),
            600000.0_f64.to_bits()
        );
        assert_eq!(
            skin_friction.roughness_over_length.to_bits(),
            0.0001_f64.to_bits()
        );
    }

    #[test]
    fn panel_mesh_parser_rejects_invalid_skin_friction_block() {
        let toml_str = minimal_panel_mesh_toml().replace(
            "cp_max = 2.0",
            "cp_max = 2.0\n\n[skin_friction]\nreference_length_m = -1.0",
        );
        assert!(matches!(
            LocalInclinationPanels::load_from_str(&toml_str),
            Err(AeroError::InvalidParameter { .. }),
        ));
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

    #[test]
    fn panel_mesh_ascii_stl_parser_builds_mesh_and_preserves_winding() {
        let mesh = PanelMesh::load_ascii_stl_from_str(minimal_panel_ascii_stl()).unwrap();
        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles(), &[[0, 1, 2], [0, 2, 3]]);

        let panels = LocalInclinationPanels::modified_newtonian(mesh, 2.0);
        let fmt = panels.aero_force_moment_body(&panel_ctx(100.0)).unwrap();
        assert_eq!(fmt.force_n_body.x.to_bits(), (-200.0_f64).to_bits());
        assert_eq!(fmt.force_n_body.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(fmt.force_n_body.z.to_bits(), 0.0_f64.to_bits());
        assert_eq!(fmt.moment_n_m_body.norm().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn panel_mesh_ascii_stl_file_loader_reads_file() {
        let path = std::env::temp_dir().join(format!(
            "openbmp-panel-mesh-ascii-stl-{}.stl",
            std::process::id()
        ));
        std::fs::write(&path, minimal_panel_ascii_stl()).unwrap();
        let mesh = PanelMesh::load_ascii_stl(&path).unwrap();
        std::fs::remove_file(&path).unwrap();

        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles().len(), 2);
    }

    #[test]
    fn panel_mesh_ascii_stl_parser_rejects_malformed_records() {
        let cases = [
            "facet normal 1 0 0\nendsolid missing_solid\n",
            "solid bad\ncolor red\nendsolid bad\n",
            "solid bad\nfacet normal 1 0 0 0\nendsolid bad\n",
            "solid bad\nfacet normal 1 0 0\nouter loop\nvertex 0 0 0\nvertex 1 0 0\nvertex 0 1 0\nendloop\nendsolid bad\n",
            "solid bad\nendsolid bad\nfacet normal 1 0 0\n",
        ];
        for case in cases {
            assert!(
                matches!(
                    PanelMesh::load_ascii_stl_from_str(case),
                    Err(AeroError::MalformedDeck { .. }),
                ),
                "{case}"
            );
        }
    }

    #[test]
    fn panel_mesh_ascii_stl_parser_rejects_nonfinite_and_degenerate_geometry() {
        let nonfinite = minimal_panel_ascii_stl().replace("vertex 1 0.5 0.5", "vertex 1 NaN 0.5");
        assert!(matches!(
            PanelMesh::load_ascii_stl_from_str(&nonfinite),
            Err(AeroError::NonFinite { .. }),
        ));

        let degenerate = r#"
solid bad
  facet normal 1 0 0
    outer loop
      vertex 0 0 0
      vertex 1 0 0
      vertex 2 0 0
    endloop
  endfacet
endsolid bad
"#;
        assert!(matches!(
            PanelMesh::load_ascii_stl_from_str(degenerate),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn panel_mesh_binary_stl_parser_builds_mesh_and_preserves_winding() {
        let mesh = PanelMesh::load_binary_stl_from_bytes(&minimal_panel_binary_stl()).unwrap();
        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles(), &[[0, 1, 2], [0, 2, 3]]);

        let panels = LocalInclinationPanels::modified_newtonian(mesh, 2.0);
        let fmt = panels.aero_force_moment_body(&panel_ctx(100.0)).unwrap();
        assert_eq!(fmt.force_n_body.x.to_bits(), (-200.0_f64).to_bits());
        assert_eq!(fmt.force_n_body.y.to_bits(), 0.0_f64.to_bits());
        assert_eq!(fmt.force_n_body.z.to_bits(), 0.0_f64.to_bits());
        assert_eq!(fmt.moment_n_m_body.norm().to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn panel_mesh_binary_stl_file_loader_reads_file() {
        let path = std::env::temp_dir().join(format!(
            "openbmp-panel-mesh-binary-stl-{}.stl",
            std::process::id()
        ));
        std::fs::write(&path, minimal_panel_binary_stl()).unwrap();
        let mesh = PanelMesh::load_binary_stl(&path).unwrap();
        std::fs::remove_file(&path).unwrap();

        assert_eq!(mesh.vertices().len(), 4);
        assert_eq!(mesh.triangles().len(), 2);
    }

    #[test]
    fn panel_mesh_binary_stl_parser_rejects_malformed_byte_counts() {
        assert!(matches!(
            PanelMesh::load_binary_stl_from_bytes(&[0_u8; 83]),
            Err(AeroError::MalformedDeck { .. }),
        ));

        let mut truncated = minimal_panel_binary_stl();
        truncated.pop();
        assert!(matches!(
            PanelMesh::load_binary_stl_from_bytes(&truncated),
            Err(AeroError::MalformedDeck { .. }),
        ));

        let mut trailing = minimal_panel_binary_stl();
        trailing.push(0);
        assert!(matches!(
            PanelMesh::load_binary_stl_from_bytes(&trailing),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }

    #[test]
    fn panel_mesh_binary_stl_parser_rejects_nonfinite_and_degenerate_geometry() {
        let mut nonfinite = vec![0_u8; 80];
        nonfinite.extend_from_slice(&1_u32.to_le_bytes());
        push_binary_stl_triangle(
            &mut nonfinite,
            [1.0, 0.0, 0.0],
            [[0.0, 0.0, 0.0], [f32::NAN, 0.0, 0.0], [0.0, 1.0, 0.0]],
        );
        assert!(matches!(
            PanelMesh::load_binary_stl_from_bytes(&nonfinite),
            Err(AeroError::NonFinite { .. }),
        ));

        let mut degenerate = vec![0_u8; 80];
        degenerate.extend_from_slice(&1_u32.to_le_bytes());
        push_binary_stl_triangle(
            &mut degenerate,
            [1.0, 0.0, 0.0],
            [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0], [2.0, 0.0, 0.0]],
        );
        assert!(matches!(
            PanelMesh::load_binary_stl_from_bytes(&degenerate),
            Err(AeroError::MalformedDeck { .. }),
        ));
    }
}
