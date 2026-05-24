//! Phase-6.3 hypersonic aerodynamic methods.
//!
//! Engineering hypersonic methods that do not require a tabulated
//! deck:
//!
//! * [`ModifiedNewtonian`] — `Cp(θ) = Cp_max · cos²(θ)` for the
//!   local-normal inclination convention used by this module, with
//!   `Cp_max` from the normal-shock stagnation pressure
//!   (`Cp_max ≈ 1.838` for `γ = 1.4`, `M_∞ → ∞`).
//! * [`TangentCone`] — local cone half-angle `θ_c` mapped through a
//!   modified-Newtonian tangent-cone approximation. Full Taylor-Maccoll
//!   integration is deferred.
//! * [`TangentWedge`] — 2-D analogue with the oblique-shock pressure
//!   coefficient.
//! * [`LocalInclinationPanels`] — mesh-panel Modified Newtonian
//!   integration with optional back-face shadowing.
//! * [`HypersonicSimilarityParameter`] — convenience helper for
//!   `K = M · θ_b` slenderness scaling.
//!
//! These are scenario-independent academic methods. They consume the
//! shared [`crate::method::AeroContext`] and return
//! [`crate::method::AeroForceMomentBody`] on a single representative
//! station: a body-axis-aligned sphere of given nose radius. Mesh
//! studies can use [`LocalInclinationPanels`] directly.
//!
//! # Determinism
//!
//! Pure `f64` arithmetic with locked operand order; no FMA. The
//! `Cp_max` formula uses `pow()` via `f64::powf`, which is
//! state-stable across platforms — callers that need bit-stable
//! output should cache `Cp_max` at scenario load with the high-Mach
//! limit `Cp_max = 2.0` (classical Newtonian) and avoid the
//! per-Mach correction.

use std::f64::consts::PI;

use nalgebra::Vector3;

use crate::error::AeroError;
use crate::method::{AeroContext, AeroForceMomentBody, AeroMethod};

/// Local inclination angle convention.
///
/// `theta_rad` is measured from the freestream direction to the
/// local outward-pointing surface normal. `θ = 0` is a panel facing
/// directly into the flow (stagnation point); `θ = π/2` is a panel
/// whose normal is perpendicular to the flow (no contribution under
/// Newtonian); `θ > π/2` is shadowed.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PanelInclination {
    /// Inclination angle (rad), measured from freestream to surface
    /// normal.
    pub theta_rad: f64,
}

/// Triangulated body-surface mesh for local-inclination panel methods.
///
/// Vertex coordinates are body-frame metres. Triangle winding must
/// produce outward-pointing normals by the right-hand rule. The
/// constructor validates finite vertices, in-range indices, and
/// non-degenerate triangle area; it does not attempt to repair
/// winding or close open meshes.
#[derive(Clone, Debug, PartialEq)]
pub struct PanelMesh {
    vertices: Vec<Vector3<f64>>,
    triangles: Vec<[u32; 3]>,
}

impl PanelMesh {
    /// Construct a validated panel mesh.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for empty / degenerate
    /// geometry, non-finite coordinates, or out-of-range indices.
    pub fn new(vertices: Vec<Vector3<f64>>, triangles: Vec<[u32; 3]>) -> Result<Self, AeroError> {
        if vertices.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "panel mesh must contain at least one vertex",
            });
        }
        if triangles.is_empty() {
            return Err(AeroError::MalformedDeck {
                reason: "panel mesh must contain at least one triangle",
            });
        }
        for vertex in &vertices {
            if !(vertex.x.is_finite() && vertex.y.is_finite() && vertex.z.is_finite()) {
                return Err(AeroError::NonFinite {
                    reason: "panel mesh vertex coordinate is NaN or Inf",
                });
            }
        }
        let n_vertices = vertices.len();
        for triangle in &triangles {
            let [ia, ib, ic] = *triangle;
            let a = usize::try_from(ia).map_err(|_| AeroError::MalformedDeck {
                reason: "panel mesh triangle index overflows usize",
            })?;
            let b = usize::try_from(ib).map_err(|_| AeroError::MalformedDeck {
                reason: "panel mesh triangle index overflows usize",
            })?;
            let c = usize::try_from(ic).map_err(|_| AeroError::MalformedDeck {
                reason: "panel mesh triangle index overflows usize",
            })?;
            if a >= n_vertices || b >= n_vertices || c >= n_vertices {
                return Err(AeroError::MalformedDeck {
                    reason: "panel mesh triangle index is out of range",
                });
            }
            if a == b || b == c || a == c {
                return Err(AeroError::MalformedDeck {
                    reason: "panel mesh triangle has repeated vertices",
                });
            }
            let area2 = (vertices[b] - vertices[a]).cross(&(vertices[c] - vertices[a]));
            if !(area2.x.is_finite()
                && area2.y.is_finite()
                && area2.z.is_finite()
                && area2.norm() > 0.0)
            {
                return Err(AeroError::MalformedDeck {
                    reason: "panel mesh triangle has zero or non-finite area",
                });
            }
        }
        Ok(Self {
            vertices,
            triangles,
        })
    }

    /// Vertices in body-frame metres.
    #[must_use]
    pub fn vertices(&self) -> &[Vector3<f64>] {
        &self.vertices
    }

    /// Triangle index triplets.
    #[must_use]
    pub fn triangles(&self) -> &[[u32; 3]] {
        &self.triangles
    }
}

/// Per-panel pressure-coefficient model.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PanelMethod {
    /// Modified Newtonian: `Cp = Cp_max cos²(theta)`.
    ModifiedNewtonian,
}

/// Local-inclination panel integration.
///
/// Each triangle contributes `F_i = -Cp_i q A_i n_i`, where `n_i`
/// is the outward unit normal from the mesh winding. Moments use
/// the triangle centroid about the body origin. When `shadowing` is
/// true, panels with `n_i · u_upstream <= 0` are skipped.
#[derive(Clone, Debug, PartialEq)]
pub struct LocalInclinationPanels {
    /// Triangulated body surface.
    pub geometry: PanelMesh,
    /// Per-panel pressure model.
    pub method: PanelMethod,
    /// Whether to zero back-facing panels.
    pub shadowing: bool,
    /// Stagnation pressure coefficient used by Modified Newtonian.
    pub cp_max: f64,
}

impl LocalInclinationPanels {
    /// Build a Modified-Newtonian panel method with shadowing enabled.
    #[must_use]
    pub const fn modified_newtonian(geometry: PanelMesh, cp_max: f64) -> Self {
        Self {
            geometry,
            method: PanelMethod::ModifiedNewtonian,
            shadowing: true,
            cp_max,
        }
    }

    fn upstream_unit(ctx: &AeroContext) -> Vector3<f64> {
        let alpha = ctx.alpha_deg.to_radians();
        let beta = ctx.beta_deg.to_radians();
        let ca = alpha.cos();
        let sa = alpha.sin();
        let cb = beta.cos();
        let sb = beta.sin();
        Vector3::new(ca * cb, sb, sa * cb).normalize()
    }
}

impl AeroMethod for LocalInclinationPanels {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        validate_context(ctx)?;
        if !(self.cp_max.is_finite() && self.cp_max >= 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "local_inclination_panels cp_max must be finite and non-negative",
            });
        }
        let upstream = Self::upstream_unit(ctx);
        let mut force = Vector3::zeros();
        let mut moment = Vector3::zeros();
        let pressure_scale = ctx.dynamic_pressure_pa;
        for &[ia, ib, ic] in self.geometry.triangles() {
            let a = self.geometry.vertices[ia as usize];
            let b = self.geometry.vertices[ib as usize];
            let c = self.geometry.vertices[ic as usize];
            let area_vec = (b - a).cross(&(c - a));
            let area = 0.5 * area_vec.norm();
            let normal = area_vec.normalize();
            let cos_theta = normal.dot(&upstream);
            if self.shadowing && cos_theta <= 0.0 {
                continue;
            }
            let cp = match self.method {
                PanelMethod::ModifiedNewtonian => {
                    if cos_theta <= 0.0 {
                        0.0
                    } else {
                        self.cp_max * cos_theta * cos_theta
                    }
                }
            };
            let panel_force = -normal * (cp * pressure_scale * area);
            let centroid = (a + b + c) / 3.0;
            force += panel_force;
            moment += centroid.cross(&panel_force);
        }
        if !(force.x.is_finite()
            && force.y.is_finite()
            && force.z.is_finite()
            && moment.x.is_finite()
            && moment.y.is_finite()
            && moment.z.is_finite())
        {
            return Err(AeroError::NonFinite {
                reason: "local-inclination force or moment is non-finite",
            });
        }
        Ok(AeroForceMomentBody {
            force_n_body: force,
            moment_n_m_body: moment,
        })
    }
}

/// Modified-Newtonian aerodynamic method.
///
/// `Cp(θ) = Cp_max · cos²(θ)` for `θ ∈ [0, π/2]`; `Cp = 0` for
/// shadowed panels (`θ > π/2`). `Cp_max` is the stagnation-point
/// pressure coefficient behind a normal shock; the "modified" in
/// Modified Newtonian uses the high-Mach real-gas-corrected
/// `Cp_max ≈ 1.838` (perfect-gas, γ = 1.4) rather than the classical
/// Newtonian value of 2.0.
///
/// Phase-6.3 scope: produces aero force as if the vehicle were a
/// **single representative panel** — an axisymmetric blunt body
/// with nose radius `nose_radius_m` and reference area `area_m2`,
/// inclined at angle of attack `alpha_deg`. Mesh-based panel
/// integration ships in the [`LocalInclinationPanels`] slice.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct ModifiedNewtonian {
    /// Effective stagnation `Cp_max` to use. Use
    /// [`Self::cp_max_perfect_gas`] for the perfect-gas correction or
    /// [`Self::CP_MAX_CLASSICAL_NEWTONIAN`] for the high-Mach limit
    /// (`Cp = 2.0`).
    pub cp_max: f64,
    /// Reference area for force composition (m²).
    pub reference_area_m2: f64,
    /// Reference length for moment composition (m). Defaults to
    /// `2 · nose_radius`.
    pub reference_length_m: f64,
}

impl ModifiedNewtonian {
    /// Classical Newtonian (`Cp_max = 2.0`).
    pub const CP_MAX_CLASSICAL_NEWTONIAN: f64 = 2.0;

    /// Perfect-gas (γ = 1.4) stagnation `Cp_max` behind a normal
    /// shock at infinite Mach: `Cp_max → 1.838`.
    pub const CP_MAX_PERFECT_GAS_INFINITE_MACH: f64 = 1.839_166_666_666_666_7;

    /// Perfect-gas (γ = 1.4) `Cp_max` at finite Mach.
    ///
    /// `Cp_max = (2 / (γ M²)) · [ ((γ+1)² M² / (4 γ M² − 2 (γ−1)))^(γ/(γ−1))
    ///             · ((1 − γ + 2 γ M²) / (γ + 1)) − 1 ]`
    ///
    /// Returns the high-Mach limit when `M ≤ 1` (the shock relation
    /// is undefined there). Pure `f64::powf` calls — state-stable.
    #[must_use]
    pub fn cp_max_perfect_gas(mach: f64, gamma: f64) -> f64 {
        if !mach.is_finite() || !gamma.is_finite() || mach <= 1.0 || gamma <= 1.0 {
            return Self::CP_MAX_PERFECT_GAS_INFINITE_MACH;
        }
        let m2 = mach * mach;
        let g = gamma;
        let g_plus = g + 1.0;
        let g_minus = g - 1.0;
        let numer = g_plus * g_plus * m2;
        let denom = 4.0 * g * m2 - 2.0 * g_minus;
        let ratio_t = numer / denom;
        let exponent_pow = g / g_minus;
        let factor_a = ratio_t.powf(exponent_pow);
        let factor_b = (1.0 - g + 2.0 * g * m2) / g_plus;
        let pt2_over_p_inf = factor_a * factor_b;
        (2.0 / (g * m2)) * (pt2_over_p_inf - 1.0)
    }

    /// Pressure coefficient at local normal inclination `θ`.
    ///
    /// This type defines `θ` as the angle between the freestream and
    /// the outward surface normal: `θ = 0` at the stagnation point and
    /// `θ = π/2` at a tangential panel. With that convention the
    /// Newtonian pressure law is `Cp = Cp_max * cos²(θ)`.
    #[must_use]
    pub fn cp(&self, panel: PanelInclination) -> f64 {
        let theta = panel.theta_rad;
        if !theta.is_finite() || theta >= 0.5 * PI {
            return 0.0;
        }
        if theta <= 0.0 {
            return self.cp_max;
        }
        let c = theta.cos();
        self.cp_max * c * c
    }
}

impl AeroMethod for ModifiedNewtonian {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        validate_context(ctx)?;
        validate_non_negative_model_parameter(
            self.cp_max,
            "modified_newtonian cp_max must be finite and non-negative",
        )?;
        validate_non_negative_model_parameter(
            self.reference_area_m2,
            "modified_newtonian reference_area_m2 must be finite and non-negative",
        )?;
        validate_non_negative_model_parameter(
            self.reference_length_m,
            "modified_newtonian reference_length_m must be finite and non-negative",
        )?;
        // Representative-panel approximation:
        //   - Stagnation panel: θ = α (measured from freestream).
        //   - Net force aligned along the inward normal at stagnation.
        //   - The body-frame projection peels off into drag (along
        //     body -x̂ at α = 0) and normal force (along body -ẑ for
        //     positive α).
        let alpha_rad = ctx.alpha_deg.to_radians();
        let s = alpha_rad.sin();
        let cn_stag = self.cp_max * s * s; // pressure coefficient at stagnation
        // Drag at zero α equals the projected stagnation pressure on
        // the cross-section, integrated over the bow → for a single
        // representative panel we use `Cp_max` at α = 0.
        let cd = self.cp_max * alpha_rad.cos().powi(2);
        let q = ctx.dynamic_pressure_pa;
        let s_ref = self.reference_area_m2;
        let drag = cd * q * s_ref;
        let normal = cn_stag * q * s_ref;
        let force = Vector3::new(-drag, 0.0, -normal);
        // Moment about the body origin: for a single-panel
        // representative model, take the moment arm as half the
        // reference length (nose-to-CG distance proxy). The
        // [`LocalInclinationPanels`] slice replaces this with the
        // per-panel moment arm.
        let m_y = normal * self.reference_length_m * 0.5;
        let moment = Vector3::new(0.0, m_y, 0.0);
        Ok(AeroForceMomentBody {
            force_n_body: force,
            moment_n_m_body: moment,
        })
    }
}

/// Tangent-cone hypersonic method.
///
/// Local cone half-angle `θ_c` mapped through a modified-Newtonian
/// tangent-cone approximation. The full Taylor-Maccoll cone shock
/// integration is deferred.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TangentCone {
    /// Local cone half-angle at the representative station (rad).
    pub cone_half_angle_rad: f64,
    /// Reference area for force composition (m²).
    pub reference_area_m2: f64,
    /// Reference length for moment composition (m).
    pub reference_length_m: f64,
    /// Ratio of specific heats (default 1.4 for perfect-gas air).
    pub gamma: f64,
}

impl TangentCone {
    /// Closed-form cone-shock pressure coefficient.
    ///
    /// Uses the **modified-Newtonian tangent-cone** approximation:
    ///
    /// `Cp_cone(M, θ_c) = Cp_max(M, γ) · sin²(θ_c)`
    ///
    /// The cone surface pressure coefficient is approximated by
    /// Modified Newtonian evaluated at the cone half-angle. The
    /// approximation is monotonic in `θ_c` and recovers the cold-flow
    /// vanishing pressure as `θ_c -> 0`. This is checked as an
    /// engineering approximation only; the Taylor-Maccoll validation
    /// table is deferred. Below `M = 1` the relation is undefined and
    /// the function returns 0.
    #[must_use]
    pub fn cp_cone(mach: f64, theta_c_rad: f64, gamma: f64) -> f64 {
        if !mach.is_finite()
            || mach <= 1.0
            || !theta_c_rad.is_finite()
            || theta_c_rad <= 0.0
            || theta_c_rad >= 0.5 * PI
            || !gamma.is_finite()
            || gamma <= 1.0
        {
            return 0.0;
        }
        let s = theta_c_rad.sin();
        let cp_max = ModifiedNewtonian::cp_max_perfect_gas(mach, gamma);
        cp_max * s * s
    }
}

impl AeroMethod for TangentCone {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        validate_context(ctx)?;
        if !(self.gamma.is_finite() && self.gamma > 1.0) {
            return Err(AeroError::InvalidParameter {
                reason: "tangent_cone gamma must be finite and > 1",
            });
        }
        if !(self.cone_half_angle_rad.is_finite() && self.cone_half_angle_rad > 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "tangent_cone cone_half_angle_rad must be positive",
            });
        }
        if self.cone_half_angle_rad >= 0.5 * PI {
            return Err(AeroError::InvalidParameter {
                reason: "tangent_cone cone_half_angle_rad must be less than pi/2",
            });
        }
        validate_non_negative_model_parameter(
            self.reference_area_m2,
            "tangent_cone reference_area_m2 must be finite and non-negative",
        )?;
        validate_non_negative_model_parameter(
            self.reference_length_m,
            "tangent_cone reference_length_m must be finite and non-negative",
        )?;
        let cp = Self::cp_cone(ctx.mach, self.cone_half_angle_rad, self.gamma);
        let q = ctx.dynamic_pressure_pa;
        let s_ref = self.reference_area_m2;
        // Single representative cone station: for a full circular
        // cone referenced to base area, the side-area growth cancels
        // the axial sine projection of pressure, giving
        // D = Cp * q * S_ref.
        let drag = cp * q * s_ref;
        let force = Vector3::new(-drag, 0.0, 0.0);
        let moment = Vector3::zeros();
        Ok(AeroForceMomentBody {
            force_n_body: force,
            moment_n_m_body: moment,
        })
    }
}

/// Tangent-wedge 2-D hypersonic method.
///
/// 2-D analogue of [`TangentCone`]: uses the oblique-shock pressure
/// coefficient for a wedge of half-angle `θ_w`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct TangentWedge {
    /// Wedge half-angle (rad).
    pub wedge_half_angle_rad: f64,
    /// Reference area for force composition (m²).
    pub reference_area_m2: f64,
    /// Ratio of specific heats.
    pub gamma: f64,
}

impl TangentWedge {
    /// Oblique-shock wedge pressure coefficient (closed form,
    /// strong-shock approximation): `Cp_w = 2 · sin²(θ_w)` in the
    /// hypersonic Newtonian limit; higher-order correction
    /// `Cp_w = (γ + 1) · sin²(θ_w) − (1 − 1/M²) · ...` lands in a
    /// later slice. The closed-form Newtonian fit shipped here is
    /// the published engineering reference (Anderson 2019 §14.4).
    #[must_use]
    pub fn cp_wedge(mach: f64, theta_w_rad: f64) -> f64 {
        if !mach.is_finite()
            || mach <= 1.0
            || !theta_w_rad.is_finite()
            || theta_w_rad <= 0.0
            || theta_w_rad >= 0.5 * PI
        {
            return 0.0;
        }
        let s = theta_w_rad.sin();
        2.0 * s * s
    }
}

impl AeroMethod for TangentWedge {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        validate_context(ctx)?;
        if !(self.gamma.is_finite() && self.gamma > 1.0) {
            return Err(AeroError::InvalidParameter {
                reason: "tangent_wedge gamma must be finite and > 1",
            });
        }
        if !(self.wedge_half_angle_rad.is_finite() && self.wedge_half_angle_rad > 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "tangent_wedge wedge_half_angle_rad must be positive",
            });
        }
        if self.wedge_half_angle_rad >= 0.5 * PI {
            return Err(AeroError::InvalidParameter {
                reason: "tangent_wedge wedge_half_angle_rad must be less than pi/2",
            });
        }
        validate_non_negative_model_parameter(
            self.reference_area_m2,
            "tangent_wedge reference_area_m2 must be finite and non-negative",
        )?;
        let cp = Self::cp_wedge(ctx.mach, self.wedge_half_angle_rad);
        let q = ctx.dynamic_pressure_pa;
        let drag = cp * q * self.reference_area_m2;
        Ok(AeroForceMomentBody {
            force_n_body: Vector3::new(-drag, 0.0, 0.0),
            moment_n_m_body: Vector3::zeros(),
        })
    }
}

/// Hypersonic similarity parameter `K = M · θ_b` for slenderness scaling.
#[must_use]
pub fn hypersonic_similarity_parameter(mach: f64, body_slenderness_rad: f64) -> f64 {
    mach * body_slenderness_rad
}

fn validate_context(ctx: &AeroContext) -> Result<(), AeroError> {
    for (name, v) in [
        ("mach", ctx.mach),
        ("alpha_deg", ctx.alpha_deg),
        ("beta_deg", ctx.beta_deg),
        ("dynamic_pressure_pa", ctx.dynamic_pressure_pa),
    ] {
        if !v.is_finite() {
            let _ = name;
            return Err(AeroError::NonFinite {
                reason: "aero context input is NaN or Inf",
            });
        }
    }
    if ctx.dynamic_pressure_pa < 0.0 {
        return Err(AeroError::InvalidParameter {
            reason: "dynamic_pressure_pa must be non-negative",
        });
    }
    Ok(())
}

fn validate_non_negative_model_parameter(
    value: f64,
    reason: &'static str,
) -> Result<(), AeroError> {
    if !(value.is_finite() && value >= 0.0) {
        return Err(AeroError::InvalidParameter { reason });
    }
    Ok(())
}

#[cfg(test)]
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::missing_panics_doc,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn ctx(mach: f64, alpha: f64, q: f64) -> AeroContext {
        AeroContext {
            mach,
            alpha_deg: alpha,
            beta_deg: 0.0,
            dynamic_pressure_pa: q,
        }
    }

    #[test]
    fn modified_newtonian_cp_max_perfect_gas_infinite_mach() {
        // Cp_max → 1.838... for γ = 1.4, M → ∞
        let cp_inf = ModifiedNewtonian::cp_max_perfect_gas(1.0e6, 1.4);
        assert!(
            (cp_inf - ModifiedNewtonian::CP_MAX_PERFECT_GAS_INFINITE_MACH).abs() < 1.0e-2,
            "Cp_max at huge M = {cp_inf}"
        );
    }

    #[test]
    fn modified_newtonian_cp_max_at_mach_10_is_finite() {
        let cp = ModifiedNewtonian::cp_max_perfect_gas(10.0, 1.4);
        assert_relative_eq!(cp, 1.831_670_977_387_537, max_relative = 1e-12);
    }

    #[test]
    fn modified_newtonian_stagnation_cp_equals_cp_max() {
        let m = ModifiedNewtonian {
            cp_max: 1.839,
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };
        // θ = 0 → panel facing flow → Cp = Cp_max
        let cp = m.cp(PanelInclination { theta_rad: 0.0 });
        assert_relative_eq!(cp, 1.839, max_relative = 1e-12);
    }

    #[test]
    fn modified_newtonian_shadowed_panel_returns_zero() {
        let m = ModifiedNewtonian {
            cp_max: 1.839,
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };
        // θ = π/2 → panel perpendicular to flow → Cp = 0
        let cp = m.cp(PanelInclination {
            theta_rad: 0.5 * PI,
        });
        assert_relative_eq!(cp, 0.0, epsilon = 1e-12);
        // θ > π/2 → shadowed → Cp = 0
        let cp_shadow = m.cp(PanelInclination {
            theta_rad: 0.6 * PI,
        });
        assert_relative_eq!(cp_shadow, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn modified_newtonian_force_at_alpha_zero_is_pure_drag() {
        let m = ModifiedNewtonian {
            cp_max: 1.839,
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };
        let fmt = m.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)).unwrap();
        // Force along -x; no side or normal force.
        assert!(fmt.force_n_body.x < 0.0);
        assert_relative_eq!(fmt.force_n_body.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.force_n_body.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn modified_newtonian_rejects_non_finite_inputs() {
        let m = ModifiedNewtonian {
            cp_max: 1.839,
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };
        assert!(matches!(
            m.aero_force_moment_body(&ctx(f64::NAN, 0.0, 1.0e4)),
            Err(AeroError::NonFinite { .. })
        ));
    }

    #[test]
    fn modified_newtonian_rejects_invalid_model_parameters() {
        let mut m = ModifiedNewtonian {
            cp_max: -1.0,
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };
        assert!(matches!(
            m.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)),
            Err(AeroError::InvalidParameter { .. })
        ));
        m.cp_max = 1.0;
        m.reference_area_m2 = f64::NAN;
        assert!(matches!(
            m.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tangent_cone_cp_increases_with_cone_angle() {
        let g = 1.4;
        let small = TangentCone::cp_cone(10.0, 5.0_f64.to_radians(), g);
        let mid = TangentCone::cp_cone(10.0, 15.0_f64.to_radians(), g);
        let large = TangentCone::cp_cone(10.0, 30.0_f64.to_radians(), g);
        assert!(small < mid);
        assert!(mid < large);
    }

    #[test]
    fn tangent_cone_cp_rejects_nonphysical_public_inputs() {
        assert_relative_eq!(
            TangentCone::cp_cone(10.0, f64::NAN, 1.4),
            0.0,
            epsilon = 0.0
        );
        assert_relative_eq!(
            TangentCone::cp_cone(10.0, 10.0_f64.to_radians(), f64::NAN),
            0.0,
            epsilon = 0.0
        );
        assert_relative_eq!(
            TangentCone::cp_cone(10.0, 90.0_f64.to_radians(), 1.4),
            0.0,
            epsilon = 0.0
        );
    }

    #[test]
    fn tangent_cone_force_along_minus_x() {
        let c = TangentCone {
            cone_half_angle_rad: 15.0_f64.to_radians(),
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
            gamma: 1.4,
        };
        let fmt = c.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)).unwrap();
        assert!(fmt.force_n_body.x < 0.0);
    }

    #[test]
    fn tangent_cone_rejects_invalid_model_parameters() {
        let c = TangentCone {
            cone_half_angle_rad: 15.0_f64.to_radians(),
            reference_area_m2: -1.0,
            reference_length_m: 1.0,
            gamma: 1.4,
        };
        assert!(matches!(
            c.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)),
            Err(AeroError::InvalidParameter { .. })
        ));
        let c = TangentCone {
            cone_half_angle_rad: 90.0_f64.to_radians(),
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
            gamma: 1.4,
        };
        assert!(matches!(
            c.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    #[test]
    fn tangent_wedge_cp_matches_newtonian() {
        // 2-D Newtonian: Cp = 2 sin²(θ).
        let theta = 10.0_f64.to_radians();
        let cp = TangentWedge::cp_wedge(8.0, theta);
        let expected = 2.0 * theta.sin() * theta.sin();
        assert_relative_eq!(cp, expected, max_relative = 1e-12);
    }

    #[test]
    fn tangent_wedge_rejects_invalid_model_parameters() {
        let w = TangentWedge {
            wedge_half_angle_rad: 10.0_f64.to_radians(),
            reference_area_m2: 1.0,
            gamma: 1.0,
        };
        assert!(matches!(
            w.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)),
            Err(AeroError::InvalidParameter { .. })
        ));
        let w = TangentWedge {
            wedge_half_angle_rad: 10.0_f64.to_radians(),
            reference_area_m2: -1.0,
            gamma: 1.4,
        };
        assert!(matches!(
            w.aero_force_moment_body(&ctx(10.0, 0.0, 1.0e4)),
            Err(AeroError::InvalidParameter { .. })
        ));
    }

    fn square_plate_mesh() -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(1.0, -0.5, -0.5),
                Vector3::new(1.0, 0.5, -0.5),
                Vector3::new(1.0, 0.5, 0.5),
                Vector3::new(1.0, -0.5, 0.5),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap()
    }

    #[test]
    fn local_inclination_square_plate_matches_newtonian_drag() {
        let mesh = square_plate_mesh();
        let panels = LocalInclinationPanels::modified_newtonian(mesh, 2.0);
        let fmt = panels
            .aero_force_moment_body(&ctx(12.0, 0.0, 100.0))
            .unwrap();
        assert_relative_eq!(fmt.force_n_body.x, -200.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.force_n_body.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.force_n_body.z, 0.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.moment_n_m_body.norm(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn local_inclination_shadowing_skips_back_face() {
        let mesh = PanelMesh::new(
            vec![
                Vector3::new(-1.0, -0.5, -0.5),
                Vector3::new(-1.0, -0.5, 0.5),
                Vector3::new(-1.0, 0.5, 0.5),
                Vector3::new(-1.0, 0.5, -0.5),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap();
        let panels = LocalInclinationPanels::modified_newtonian(mesh, 2.0);
        let fmt = panels
            .aero_force_moment_body(&ctx(12.0, 0.0, 100.0))
            .unwrap();
        assert_relative_eq!(fmt.force_n_body.norm(), 0.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.moment_n_m_body.norm(), 0.0, epsilon = 1e-12);
    }

    #[test]
    fn panel_mesh_rejects_degenerate_triangle() {
        let err = PanelMesh::new(
            vec![Vector3::zeros(), Vector3::x(), Vector3::new(2.0, 0.0, 0.0)],
            vec![[0, 1, 2]],
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn hypersonic_similarity_parameter_is_product() {
        assert_relative_eq!(
            hypersonic_similarity_parameter(10.0, 0.1),
            1.0,
            max_relative = 1e-12
        );
    }

    #[test]
    fn determinism_two_runs_byte_identical() {
        let m = ModifiedNewtonian {
            cp_max: 1.839,
            reference_area_m2: 1.0,
            reference_length_m: 1.0,
        };
        let a = m.aero_force_moment_body(&ctx(15.0, 5.0, 5000.0)).unwrap();
        let b = m.aero_force_moment_body(&ctx(15.0, 5.0, 5000.0)).unwrap();
        assert_eq!(a.force_n_body.x.to_bits(), b.force_n_body.x.to_bits());
        assert_eq!(a.force_n_body.z.to_bits(), b.force_n_body.z.to_bits());
    }
}
