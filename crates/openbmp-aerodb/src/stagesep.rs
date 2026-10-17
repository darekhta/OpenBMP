//! Stage-separation proximity-interference deck (WP-03.7).
//!
//! During a separation event two bodies fly in close proximity and each
//! perturbs the other's flow field. This module stores, over a structured
//! grid of relative separation distance and freestream Mach number, the
//! six-coefficient *increment* `dC = C_proximity - C_isolated` for the affected
//! body, and applies it additively (superposition) to the body's isolated-flow
//! coefficients at runtime.
//!
//! Condition-only by construction: the independent variables are the relative
//! body **separation distance** and the freestream **Mach number** -- forward
//! physics geometry and flow state. There is no aimpoint, range, target, or
//! geodetic field anywhere in the deck, consistent with the `openbmp-aerodb`
//! dual-use posture.
//!
//! Interference runs off to zero as the bodies move apart: the deck returns a
//! zero increment beyond the largest tabulated separation, and construction
//! fails closed unless the increments at that largest separation have already
//! decayed below a caller-supplied runoff tolerance (so the runoff is physical,
//! not a discontinuity at the grid edge).
//!
//! Scope (WP-03.7 is partial): this is the deterministic deck representation,
//! runtime interpolation, superposition, and a panel-derived build hook. The
//! scenario/runner staging wiring and published Ares-I-class validation remain
//! open and are tracked in `docs/parity/progress.md`.

use nalgebra::Vector3;
use openbmp_aero::{
    AeroContext, AeroSixCoefficients, LocalInclinationPanels, PanelMesh, PanelMeshBvh,
    PanelSolution,
};
use serde::Deserialize;

use crate::{AerodbError, lint_condition_only_run_matrix_toml};

/// Schema tag for an ingested proximity-interference deck TOML document.
const PROXIMITY_DECK_SCHEMA_V1: &str = "openbmp.stagesep_proximity.v1";

/// Per-body six-coefficient proximity-interference increment deck over a
/// (separation distance x Mach) grid.
#[derive(Clone, Debug, PartialEq)]
pub struct ProximityInterferenceDeck {
    /// Strictly increasing separation grid (metres), length >= 2.
    separations_m: Vec<f64>,
    /// Strictly increasing Mach grid, length >= 1.
    machs: Vec<f64>,
    /// Row-major increments indexed `[sep_index * machs.len() + mach_index]`.
    increments: Vec<AeroSixCoefficients>,
}

impl ProximityInterferenceDeck {
    /// Construct and validate a proximity-interference deck from tabulated
    /// increments.
    ///
    /// `increments` is row-major: increment for separation `i` and Mach `j` is
    /// at `i * machs.len() + j`. `runoff_tolerance` bounds the maximum absolute
    /// coefficient at the largest separation; construction fails closed if the
    /// interference has not decayed below it there (so the zero-beyond-grid
    /// runoff is continuous).
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::InvalidInput`] if an axis is too short, not
    /// strictly increasing, negative, or non-finite; if the increment count does
    /// not match the grid; if any increment is non-finite; or if the
    /// largest-separation increments exceed `runoff_tolerance`.
    pub fn new(
        separations_m: Vec<f64>,
        machs: Vec<f64>,
        increments: Vec<AeroSixCoefficients>,
        runoff_tolerance: f64,
    ) -> Result<Self, AerodbError> {
        require_increasing(&separations_m, 2, "stagesep.separations_m")?;
        require_increasing(&machs, 1, "stagesep.machs")?;
        if !(runoff_tolerance.is_finite() && runoff_tolerance >= 0.0) {
            return Err(AerodbError::InvalidInput {
                field: "stagesep.runoff_tolerance",
            });
        }
        if increments.len() != separations_m.len() * machs.len() {
            return Err(AerodbError::InvalidInput {
                field: "stagesep.increments.len",
            });
        }
        for increment in &increments {
            if !six_is_finite(increment) {
                return Err(AerodbError::InvalidInput {
                    field: "stagesep.increments.value",
                });
            }
        }
        let deck = Self {
            separations_m,
            machs,
            increments,
        };
        // Runoff invariant: interference must have decayed at the far edge.
        let last_sep = deck.separations_m.len() - 1;
        for j in 0..deck.machs.len() {
            if six_max_abs(&deck.increments[last_sep * deck.machs.len() + j]) > runoff_tolerance {
                return Err(AerodbError::InvalidInput {
                    field: "stagesep.runoff_not_decayed",
                });
            }
        }
        Ok(deck)
    }

    /// Ingest a proximity-interference deck from a provenance-pinned TOML
    /// document (schema `openbmp.stagesep_proximity.v1`).
    ///
    /// The raw text is first linted for forbidden target/aimpoint/range
    /// vocabulary (condition-only dual-use guard), then parsed strictly. The
    /// `increment` array is row-major over `[separation][mach]`, each entry a
    /// six-coefficient body-axis set.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError::ForbiddenVocabulary`] for forbidden keys,
    /// [`AerodbError::MalformedToml`] for a parse/schema failure, or any
    /// [`Self::new`] validation error.
    pub fn load_toml_str(s: &str) -> Result<Self, AerodbError> {
        lint_condition_only_run_matrix_toml(s)?;
        let parsed: ProximityDeckToml =
            toml::from_str(s).map_err(|_| AerodbError::MalformedToml {
                reason: "stagesep proximity deck TOML failed strict parsing",
            })?;
        if parsed.schema != PROXIMITY_DECK_SCHEMA_V1 {
            return Err(AerodbError::MalformedToml {
                reason: "stagesep proximity deck schema tag is unsupported",
            });
        }
        let increments = parsed
            .increment
            .iter()
            .map(|row| AeroSixCoefficients {
                cx: row.cx,
                cy: row.cy,
                cz: row.cz,
                cl: row.cl,
                cm: row.cm,
                cn: row.cn,
            })
            .collect();
        Self::new(
            parsed.separations_m,
            parsed.machs,
            increments,
            parsed.runoff_tolerance,
        )
    }

    /// Build a deck by differencing proximity and isolated coefficients produced
    /// by `pair` at each grid node: `dC = C_proximity - C_isolated`.
    ///
    /// `pair(separation_m, mach)` is the panel-solver hook -- it typically runs
    /// the local-inclination panel method with the other body's mesh present
    /// (proximity, e.g. via BVH shadowing) and absent (isolated). It is kept as
    /// a closure so the deck builder is testable without a full mesh.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError`] if `pair` fails for any node, or if the resulting
    /// deck fails [`Self::new`] validation.
    pub fn build_from_pair<F>(
        separations_m: Vec<f64>,
        machs: Vec<f64>,
        runoff_tolerance: f64,
        mut pair: F,
    ) -> Result<Self, AerodbError>
    where
        F: FnMut(f64, f64) -> Result<(AeroSixCoefficients, AeroSixCoefficients), AerodbError>,
    {
        let mut increments = Vec::with_capacity(separations_m.len() * machs.len());
        for &separation_m in &separations_m {
            for &mach in &machs {
                let (proximity, isolated) = pair(separation_m, mach)?;
                increments.push(six_sub(&proximity, &isolated));
            }
        }
        Self::new(separations_m, machs, increments, runoff_tolerance)
    }

    /// Build a proximity-interference deck directly from the local-inclination
    /// panel solver: at each `(separation, Mach)` node the affected body's
    /// six-coefficients are computed both with the other body present (proximity,
    /// via a combined-geometry shadow BVH) and absent (isolated, self-shadow),
    /// and differenced. This is the in-Rust, no-external-tool generator for
    /// WP-03.7.
    ///
    /// The other body is placed at `separation_axis_unit * separation` relative
    /// to the affected body for each node. Coefficients are non-dimensionalised
    /// by `dynamic_pressure_pa`, `reference_area_m2`, and `reference_length_m`
    /// (moments). Flow direction is set by `alpha_deg` / `beta_deg`. This is
    /// condition-only: separation distance + Mach + incidence, no aimpoint.
    ///
    /// # Errors
    ///
    /// Returns [`AerodbError`] for non-finite / non-positive references or a
    /// zero separation axis, for any panel-solver failure, or if the resulting
    /// deck fails [`Self::new`] validation (e.g. interference has not decayed by
    /// the largest separation).
    #[allow(clippy::too_many_arguments)]
    pub fn build_from_panel_meshes(
        affected: &LocalInclinationPanels,
        other_mesh: &PanelMesh,
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
        separation_axis_unit: Vector3<f64>,
        separations_m: Vec<f64>,
        machs: Vec<f64>,
        alpha_deg: f64,
        beta_deg: f64,
        dynamic_pressure_pa: f64,
        reference_area_m2: f64,
        reference_length_m: f64,
        runoff_tolerance: f64,
    ) -> Result<Self, AerodbError> {
        require_positive(dynamic_pressure_pa, "stagesep.dynamic_pressure_pa")?;
        require_positive(reference_area_m2, "stagesep.reference_area_m2")?;
        require_positive(reference_length_m, "stagesep.reference_length_m")?;
        let axis_norm = separation_axis_unit.norm();
        if !axis_norm.is_finite() || axis_norm <= 0.0 {
            return Err(AerodbError::InvalidInput {
                field: "stagesep.separation_axis_unit",
            });
        }
        let axis = separation_axis_unit / axis_norm;
        Self::build_from_pair(
            separations_m,
            machs,
            runoff_tolerance,
            |separation_m, mach| {
                let ctx = flow_context(mach, alpha_deg, beta_deg, dynamic_pressure_pa);
                let combined =
                    combined_shadow_bvh(&affected.geometry, other_mesh, axis * separation_m)?;
                let proximity = affected.solve_about_cg_with_shadow_bvh(
                    &ctx,
                    cg_body_m,
                    axial_stations_m,
                    &combined,
                )?;
                let isolated = affected.solve_about_cg_with_bvh_shadowing(
                    &ctx,
                    cg_body_m,
                    axial_stations_m,
                )?;
                Ok((
                    six_from_solution(
                        &proximity,
                        dynamic_pressure_pa,
                        reference_area_m2,
                        reference_length_m,
                    ),
                    six_from_solution(
                        &isolated,
                        dynamic_pressure_pa,
                        reference_area_m2,
                        reference_length_m,
                    ),
                ))
            },
        )
    }

    /// Interpolated proximity increment at a separation distance and Mach.
    ///
    /// Returns a zero increment for separations at or beyond the largest
    /// tabulated value (runoff). Separations below the smallest value clamp to
    /// the closest-proximity row, and Mach clamps to the tabulated range.
    /// Interpolation is bilinear in (separation, Mach).
    #[must_use]
    pub fn increment_at(&self, separation_m: f64, mach: f64) -> AeroSixCoefficients {
        let n_sep = self.separations_m.len();
        let n_mach = self.machs.len();
        // Runoff: at or beyond the far edge the interference has decayed.
        if !separation_m.is_finite() || separation_m >= self.separations_m[n_sep - 1] {
            return AeroSixCoefficients::default();
        }
        let (i, t) = bracket(&self.separations_m, separation_m);
        let (j, u) = bracket(&self.machs, mach);
        let j_hi = if n_mach == 1 { j } else { j + 1 };
        let row_lo = i * n_mach;
        let row_hi = (i + 1) * n_mach;
        let c00 = self.increments[row_lo + j];
        let c01 = self.increments[row_lo + j_hi];
        let c10 = self.increments[row_hi + j];
        let c11 = self.increments[row_hi + j_hi];
        let lo = six_lerp(&c00, &c01, u);
        let hi = six_lerp(&c10, &c11, u);
        six_lerp(&lo, &hi, t)
    }

    /// Superpose the proximity increment onto an isolated-body coefficient set.
    #[must_use]
    pub fn superpose(
        &self,
        isolated: AeroSixCoefficients,
        separation_m: f64,
        mach: f64,
    ) -> AeroSixCoefficients {
        six_add(&isolated, &self.increment_at(separation_m, mach))
    }

    /// Tabulated separation grid (metres).
    #[must_use]
    pub fn separations_m(&self) -> &[f64] {
        &self.separations_m
    }

    /// Tabulated Mach grid.
    #[must_use]
    pub fn machs(&self) -> &[f64] {
        &self.machs
    }
}

/// Bracket `value` in a strictly increasing grid, returning the lower index and
/// the in-cell fraction `t in [0, 1]`. Values at or below the first node return
/// `(0, 0.0)`; values at or above the last return `(len - 2, 1.0)` (so the
/// caller's `i + 1` stays in range). A single-element grid returns `(0, 0.0)`.
fn bracket(grid: &[f64], value: f64) -> (usize, f64) {
    let n = grid.len();
    if n == 1 || !value.is_finite() || value <= grid[0] {
        return (0, 0.0);
    }
    if value >= grid[n - 1] {
        return (n - 2, 1.0);
    }
    let mut i = 0;
    while i + 1 < n && grid[i + 1] <= value {
        i += 1;
    }
    let span = grid[i + 1] - grid[i];
    let t = if span > 0.0 {
        (value - grid[i]) / span
    } else {
        0.0
    };
    (i, t)
}

fn require_increasing(
    grid: &[f64],
    min_len: usize,
    field: &'static str,
) -> Result<(), AerodbError> {
    if grid.len() < min_len {
        return Err(AerodbError::InvalidInput { field });
    }
    let mut previous = f64::NEG_INFINITY;
    for &value in grid {
        if !value.is_finite() || value < 0.0 || value <= previous {
            return Err(AerodbError::InvalidInput { field });
        }
        previous = value;
    }
    Ok(())
}

fn six_is_finite(c: &AeroSixCoefficients) -> bool {
    c.cx.is_finite()
        && c.cy.is_finite()
        && c.cz.is_finite()
        && c.cl.is_finite()
        && c.cm.is_finite()
        && c.cn.is_finite()
}

fn six_max_abs(c: &AeroSixCoefficients) -> f64 {
    c.cx.abs()
        .max(c.cy.abs())
        .max(c.cz.abs())
        .max(c.cl.abs())
        .max(c.cm.abs())
        .max(c.cn.abs())
}

fn six_sub(a: &AeroSixCoefficients, b: &AeroSixCoefficients) -> AeroSixCoefficients {
    AeroSixCoefficients {
        cx: a.cx - b.cx,
        cy: a.cy - b.cy,
        cz: a.cz - b.cz,
        cl: a.cl - b.cl,
        cm: a.cm - b.cm,
        cn: a.cn - b.cn,
    }
}

fn six_add(a: &AeroSixCoefficients, b: &AeroSixCoefficients) -> AeroSixCoefficients {
    AeroSixCoefficients {
        cx: a.cx + b.cx,
        cy: a.cy + b.cy,
        cz: a.cz + b.cz,
        cl: a.cl + b.cl,
        cm: a.cm + b.cm,
        cn: a.cn + b.cn,
    }
}

fn six_lerp(a: &AeroSixCoefficients, b: &AeroSixCoefficients, t: f64) -> AeroSixCoefficients {
    AeroSixCoefficients {
        cx: a.cx + (b.cx - a.cx) * t,
        cy: a.cy + (b.cy - a.cy) * t,
        cz: a.cz + (b.cz - a.cz) * t,
        cl: a.cl + (b.cl - a.cl) * t,
        cm: a.cm + (b.cm - a.cm) * t,
        cn: a.cn + (b.cn - a.cn) * t,
    }
}

fn require_positive(value: f64, field: &'static str) -> Result<(), AerodbError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(AerodbError::InvalidInput { field });
    }
    Ok(())
}

/// Flight-condition context for a panel solve (no body rates; Reynolds/Knudsen
/// left at zero since the proximity increment is inviscid-panel geometry).
fn flow_context(mach: f64, alpha_deg: f64, beta_deg: f64, dynamic_pressure_pa: f64) -> AeroContext {
    AeroContext {
        mach,
        alpha_deg,
        beta_deg,
        dynamic_pressure_pa,
        airspeed_m_s: 0.0,
        body_angular_rates_rad_s: Vector3::zeros(),
        knudsen: 0.0,
        reynolds_length: 0.0,
    }
}

/// Build a shadow BVH from the affected mesh combined with the other body's
/// mesh translated by `offset`. The two bodies use disjoint vertex indices, so
/// the union remains a watertight two-manifold (each component is closed).
fn combined_shadow_bvh(
    affected: &PanelMesh,
    other: &PanelMesh,
    offset: Vector3<f64>,
) -> Result<PanelMeshBvh, AerodbError> {
    let mut vertices: Vec<Vector3<f64>> = affected.vertices().to_vec();
    let base = u32::try_from(vertices.len()).map_err(|_| AerodbError::InvalidInput {
        field: "stagesep.affected_mesh.vertex_count",
    })?;
    for vertex in other.vertices() {
        vertices.push(vertex + offset);
    }
    let mut triangles: Vec<[u32; 3]> = affected.triangles().to_vec();
    for triangle in other.triangles() {
        triangles.push([triangle[0] + base, triangle[1] + base, triangle[2] + base]);
    }
    let combined = PanelMesh::new(vertices, triangles)?;
    Ok(PanelMeshBvh::from_watertight_mesh(combined)?)
}

/// Non-dimensionalise a panel solution into six body-axis coefficients.
fn six_from_solution(
    solution: &PanelSolution,
    dynamic_pressure_pa: f64,
    reference_area_m2: f64,
    reference_length_m: f64,
) -> AeroSixCoefficients {
    let qs = dynamic_pressure_pa * reference_area_m2;
    let qsl = qs * reference_length_m;
    AeroSixCoefficients {
        cx: solution.force_n_body.x / qs,
        cy: solution.force_n_body.y / qs,
        cz: solution.force_n_body.z / qs,
        cl: solution.moment_n_m_body.x / qsl,
        cm: solution.moment_n_m_body.y / qsl,
        cn: solution.moment_n_m_body.z / qsl,
    }
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct ProximityDeckToml {
    schema: String,
    separations_m: Vec<f64>,
    machs: Vec<f64>,
    runoff_tolerance: f64,
    increment: Vec<SixToml>,
}

#[derive(Debug, Deserialize)]
#[serde(deny_unknown_fields)]
struct SixToml {
    cx: f64,
    cy: f64,
    cz: f64,
    cl: f64,
    cm: f64,
    cn: f64,
}

#[cfg(test)]
#[allow(clippy::float_cmp, clippy::unwrap_used)]
mod tests {
    use super::*;

    /// Watertight axis-aligned box mesh centered at `center` with half-extents
    /// `half` (same corner/triangle topology as the aero crate's box fixture).
    fn box_mesh(center: Vector3<f64>, half: Vector3<f64>) -> PanelMesh {
        let c = center;
        let h = half;
        PanelMesh::new(
            vec![
                Vector3::new(c.x - h.x, c.y - h.y, c.z - h.z),
                Vector3::new(c.x - h.x, c.y + h.y, c.z - h.z),
                Vector3::new(c.x - h.x, c.y + h.y, c.z + h.z),
                Vector3::new(c.x - h.x, c.y - h.y, c.z + h.z),
                Vector3::new(c.x + h.x, c.y - h.y, c.z - h.z),
                Vector3::new(c.x + h.x, c.y + h.y, c.z - h.z),
                Vector3::new(c.x + h.x, c.y + h.y, c.z + h.z),
                Vector3::new(c.x + h.x, c.y - h.y, c.z + h.z),
            ],
            // Outward-facing winding (reversed vs the aero crate's BVH-only box
            // fixture, which uses inward normals) so windward panels are kept.
            vec![
                [0, 2, 1],
                [0, 3, 2],
                [4, 5, 6],
                [4, 6, 7],
                [0, 7, 3],
                [0, 4, 7],
                [1, 6, 5],
                [1, 2, 6],
                [0, 5, 4],
                [0, 1, 5],
                [3, 6, 2],
                [3, 7, 6],
            ],
        )
        .unwrap()
    }

    #[test]
    fn solve_with_external_shadow_bvh_captures_cross_body_shadowing() {
        // Affected box at origin; an identical box placed directly UPSTREAM (+x,
        // the freestream direction at alpha=beta=0) occludes the affected body's
        // windward panels, so its loads with the combined shadow BVH differ from
        // the self-shadow (isolated) loads.
        let affected = LocalInclinationPanels::modified_newtonian(
            box_mesh(Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0)),
            2.0,
        );
        let other = box_mesh(Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0));
        let ctx = flow_context(8.0, 0.0, 0.0, 100.0);
        let isolated = affected
            .solve_about_cg_with_bvh_shadowing(&ctx, Vector3::zeros(), &[])
            .unwrap();
        let combined =
            combined_shadow_bvh(&affected.geometry, &other, Vector3::new(3.0, 0.0, 0.0)).unwrap();
        let proximity = affected
            .solve_about_cg_with_shadow_bvh(&ctx, Vector3::zeros(), &[], &combined)
            .unwrap();
        // The upstream blocker reduces the windward (axial, body-x) load.
        assert!(
            (proximity.force_n_body.x - isolated.force_n_body.x).abs() > 1.0e-6,
            "expected cross-body shadowing to change the axial load: iso={}, prox={}",
            isolated.force_n_body.x,
            proximity.force_n_body.x
        );
    }

    #[test]
    fn build_from_panel_meshes_runs_off_for_lateral_separation() {
        // Other body separates laterally (+z) from the affected box. With the
        // flow along +x, a purely lateral body never occludes the affected body,
        // so every increment is ~0 -- the deck builds (runoff invariant holds)
        // and is deterministic.
        let affected = LocalInclinationPanels::modified_newtonian(
            box_mesh(Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0)),
            2.0,
        );
        let other = box_mesh(Vector3::zeros(), Vector3::new(1.0, 1.0, 1.0));
        let build = || {
            ProximityInterferenceDeck::build_from_panel_meshes(
                &affected,
                &other,
                Vector3::zeros(),
                &[],
                Vector3::new(0.0, 0.0, 1.0), // lateral separation axis
                vec![4.0, 40.0],
                vec![6.0],
                0.0,
                0.0,
                100.0,
                4.0, // reference area
                2.0, // reference length
                1.0e-9,
            )
        };
        let deck = build().unwrap();
        // No occlusion at any separation -> increments are ~0.
        let near = deck.increment_at(4.0, 6.0);
        assert!(
            six_max_abs(&near) < 1.0e-9,
            "lateral separation should not interfere: {near:?}"
        );
        // Deterministic.
        assert_eq!(deck, build().unwrap());
    }

    fn six(cx: f64, cz: f64, cm: f64) -> AeroSixCoefficients {
        AeroSixCoefficients {
            cx,
            cy: 0.0,
            cz,
            cl: 0.0,
            cm,
            cn: 0.0,
        }
    }

    // 2 separations x 2 Mach. Near (sep=1) interference is strong; far (sep=5)
    // it has decayed to zero (runoff edge).
    fn sample_deck() -> ProximityInterferenceDeck {
        let increments = vec![
            six(0.10, -0.20, 0.05), // sep=1, M=2
            six(0.06, -0.12, 0.03), // sep=1, M=4
            six(0.0, 0.0, 0.0),     // sep=5, M=2
            six(0.0, 0.0, 0.0),     // sep=5, M=4
        ];
        ProximityInterferenceDeck::new(vec![1.0, 5.0], vec![2.0, 4.0], increments, 1.0e-9).unwrap()
    }

    const VALID_DECK_TOML: &str = r#"
schema = "openbmp.stagesep_proximity.v1"
separations_m = [1.0, 5.0]
machs = [2.0]
runoff_tolerance = 1.0e-9
[[increment]]
cx = 0.10
cy = 0.0
cz = -0.20
cl = 0.0
cm = 0.05
cn = 0.0
[[increment]]
cx = 0.0
cy = 0.0
cz = 0.0
cl = 0.0
cm = 0.0
cn = 0.0
"#;

    #[test]
    fn load_toml_str_round_trips_a_valid_deck() {
        let deck = ProximityInterferenceDeck::load_toml_str(VALID_DECK_TOML).unwrap();
        assert_eq!(deck.separations_m(), &[1.0, 5.0]);
        assert_eq!(deck.machs(), &[2.0]);
        assert_eq!(deck.increment_at(1.0, 2.0), six(0.10, -0.20, 0.05));
        assert_eq!(deck.increment_at(5.0, 2.0), AeroSixCoefficients::default());
    }

    #[test]
    fn load_toml_str_rejects_forbidden_vocabulary() {
        // A `target`/`aimpoint`/`range` key must fail the dual-use lint.
        let toml = VALID_DECK_TOML.replace("runoff_tolerance", "target_range_m");
        assert!(matches!(
            ProximityInterferenceDeck::load_toml_str(&toml),
            Err(AerodbError::ForbiddenVocabulary { .. })
        ));
    }

    #[test]
    fn load_toml_str_rejects_bad_schema_and_undecayed_runoff() {
        let bad_schema = VALID_DECK_TOML.replace("stagesep_proximity.v1", "stagesep_proximity.v9");
        assert!(matches!(
            ProximityInterferenceDeck::load_toml_str(&bad_schema),
            Err(AerodbError::MalformedToml { .. })
        ));
        // Far-edge increment that has not decayed fails Self::new validation.
        let undecayed = VALID_DECK_TOML.replace("cx = 0.0", "cx = 0.5");
        assert!(ProximityInterferenceDeck::load_toml_str(&undecayed).is_err());
    }

    #[test]
    fn returns_exact_node_values() {
        let deck = sample_deck();
        assert_eq!(deck.increment_at(1.0, 2.0), six(0.10, -0.20, 0.05));
        assert_eq!(deck.increment_at(1.0, 4.0), six(0.06, -0.12, 0.03));
    }

    #[test]
    fn bilinear_interpolates_interior() {
        let deck = sample_deck();
        // Midpoint in separation (1->5) and Mach (2->4): average of the four
        // corners. Far corners are zero, so result is a quarter of the near row.
        let mid = deck.increment_at(3.0, 3.0);
        let expected_cx = (0.10 + 0.06 + 0.0 + 0.0) / 4.0;
        let expected_cz = (-0.20 - 0.12 + 0.0 + 0.0) / 4.0;
        assert!((mid.cx - expected_cx).abs() < 1.0e-12);
        assert!((mid.cz - expected_cz).abs() < 1.0e-12);
    }

    #[test]
    fn runs_off_to_zero_beyond_grid() {
        let deck = sample_deck();
        assert_eq!(deck.increment_at(5.0, 3.0), AeroSixCoefficients::default());
        assert_eq!(deck.increment_at(50.0, 3.0), AeroSixCoefficients::default());
        assert_eq!(
            deck.increment_at(f64::INFINITY, 3.0),
            AeroSixCoefficients::default()
        );
    }

    #[test]
    fn clamps_below_grid_and_off_mach_range() {
        let deck = sample_deck();
        // Separation below the first node clamps to the closest-proximity row.
        assert_eq!(deck.increment_at(0.1, 2.0), six(0.10, -0.20, 0.05));
        // Mach below / above the grid clamps to the edge columns.
        assert_eq!(deck.increment_at(1.0, 0.5), six(0.10, -0.20, 0.05));
        assert_eq!(deck.increment_at(1.0, 99.0), six(0.06, -0.12, 0.03));
    }

    #[test]
    fn superpose_adds_increment() {
        let deck = sample_deck();
        let isolated = six(1.0, -1.0, 0.5);
        let total = deck.superpose(isolated, 1.0, 2.0);
        assert_eq!(total, six(1.10, -1.20, 0.55));
        // Beyond runoff, superposition returns the isolated value unchanged.
        assert_eq!(deck.superpose(isolated, 100.0, 2.0), isolated);
    }

    #[test]
    fn build_from_pair_differences_proximity_and_isolated() {
        // proximity - isolated should recover the prescribed increment.
        let deck = ProximityInterferenceDeck::build_from_pair(
            vec![1.0, 5.0],
            vec![2.0],
            1.0e-9,
            |sep, _mach| {
                let isolated = six(1.0, -1.0, 0.5);
                let delta = if sep < 2.0 {
                    six(0.1, -0.2, 0.05)
                } else {
                    six(0.0, 0.0, 0.0)
                };
                Ok((six_add(&isolated, &delta), isolated))
            },
        )
        .unwrap();
        let near = deck.increment_at(1.0, 2.0);
        assert!((near.cx - 0.1).abs() < 1.0e-12);
        assert!((near.cz + 0.2).abs() < 1.0e-12);
        assert!((near.cm - 0.05).abs() < 1.0e-12);
        assert_eq!(deck.increment_at(5.0, 2.0), AeroSixCoefficients::default());
    }

    #[test]
    fn rejects_invalid_grids_and_undecayed_runoff() {
        let good = vec![six(0.1, -0.2, 0.05), six(0.0, 0.0, 0.0)];
        // Non-increasing separation.
        assert!(
            ProximityInterferenceDeck::new(vec![5.0, 1.0], vec![2.0], good.clone(), 1e-9).is_err()
        );
        // Too few separations.
        assert!(
            ProximityInterferenceDeck::new(vec![1.0], vec![2.0], vec![six(0.0, 0.0, 0.0)], 1e-9)
                .is_err()
        );
        // Increment count mismatch: 2 separations x 2 Mach needs 4 increments.
        assert!(
            ProximityInterferenceDeck::new(vec![1.0, 5.0], vec![2.0, 4.0], good.clone(), 1e-9)
                .is_err()
        );
        // Far edge has NOT decayed -> fail closed.
        let undecayed = vec![
            six(0.1, -0.2, 0.05),
            six(0.05, -0.1, 0.02),
            six(0.04, -0.05, 0.01),
            six(0.0, 0.0, 0.0),
        ];
        assert!(
            ProximityInterferenceDeck::new(vec![1.0, 5.0], vec![2.0, 4.0], undecayed, 1e-9)
                .is_err()
        );
    }

    #[test]
    fn construction_is_deterministic() {
        assert_eq!(sample_deck(), sample_deck());
    }
}
