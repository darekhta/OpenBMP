//! Hypersonic aerodynamic methods.
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
//!   integration with optional back-face/BVH shadowing and opt-in
//!   reference-temperature skin friction.
//! * `HypersonicSimilarityParameter` — convenience helper for
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

#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
use core::f64::consts::PI;

use nalgebra::Vector3;
#[cfg(not(feature = "std"))]
use num_traits::Float;

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

/// Deterministic watertight-mesh BVH for panel ray queries.
#[derive(Clone, Debug, PartialEq)]
pub struct PanelMeshBvh {
    mesh: PanelMesh,
    nodes: Vec<PanelMeshBvhNode>,
    triangle_order: Vec<usize>,
}

#[derive(Clone, Debug, PartialEq)]
struct PanelMeshBvhNode {
    aabb: PanelMeshAabb,
    left: Option<usize>,
    right: Option<usize>,
    first: usize,
    count: usize,
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct PanelMeshAabb {
    min: Vector3<f64>,
    max: Vector3<f64>,
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

    /// Validate that every undirected mesh edge is incident to exactly two
    /// triangles.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] when the mesh has a boundary edge,
    /// a non-manifold edge, or an internal edge-count overflow.
    pub fn validate_watertight_manifold(&self) -> Result<(), AeroError> {
        let mut edges = Vec::with_capacity(self.triangles.len().checked_mul(3).ok_or(
            AeroError::MalformedDeck {
                reason: "panel mesh edge count overflows usize",
            },
        )?);
        for &[a, b, c] in &self.triangles {
            edges.push(canonical_edge(a, b));
            edges.push(canonical_edge(b, c));
            edges.push(canonical_edge(c, a));
        }
        edges.sort_unstable();
        let mut index = 0;
        while index < edges.len() {
            let edge = edges[index];
            let mut count = 1;
            index += 1;
            while index < edges.len() && edges[index] == edge {
                count += 1;
                index += 1;
            }
            if count != 2 {
                return Err(AeroError::MalformedDeck {
                    reason: "panel mesh is not a watertight two-manifold",
                });
            }
        }
        Ok(())
    }
}

impl PanelMeshBvh {
    const LEAF_TRIANGLES: usize = 4;

    /// Build a byte-stable median-split BVH for a watertight panel mesh.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] when the mesh is not a watertight
    /// two-manifold.
    pub fn from_watertight_mesh(mesh: PanelMesh) -> Result<Self, AeroError> {
        mesh.validate_watertight_manifold()?;
        let indices = (0..mesh.triangles.len()).collect::<Vec<_>>();
        let mut bvh = Self {
            mesh,
            nodes: Vec::new(),
            triangle_order: Vec::new(),
        };
        build_bvh_node(&bvh.mesh, &mut bvh.nodes, &mut bvh.triangle_order, indices);
        Ok(bvh)
    }

    /// Borrow the source mesh.
    #[must_use]
    pub fn mesh(&self) -> &PanelMesh {
        &self.mesh
    }

    /// Return whether a ray intersects any panel triangle.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::NonFinite`] or [`AeroError::InvalidParameter`] when
    /// the ray origin or direction is invalid.
    pub fn ray_intersects(
        &self,
        origin_body_m: Vector3<f64>,
        direction_body: Vector3<f64>,
    ) -> Result<bool, AeroError> {
        require_finite_vector("panel mesh ray origin", origin_body_m)?;
        require_finite_vector("panel mesh ray direction", direction_body)?;
        let norm = direction_body.norm();
        if !(norm.is_finite() && norm > 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "panel mesh ray direction must be non-zero",
            });
        }
        let direction_unit = direction_body / norm;
        if self.nodes.is_empty() {
            return Ok(false);
        }
        Ok(ray_intersects_node(self, 0, origin_body_m, direction_unit))
    }
}

fn canonical_edge(a: u32, b: u32) -> (u32, u32) {
    if a <= b { (a, b) } else { (b, a) }
}

fn build_bvh_node(
    mesh: &PanelMesh,
    nodes: &mut Vec<PanelMeshBvhNode>,
    triangle_order: &mut Vec<usize>,
    mut indices: Vec<usize>,
) -> usize {
    let aabb = aabb_for_triangles(mesh, &indices);
    let node_index = nodes.len();
    nodes.push(PanelMeshBvhNode {
        aabb,
        left: None,
        right: None,
        first: 0,
        count: 0,
    });
    if indices.len() <= PanelMeshBvh::LEAF_TRIANGLES {
        let first = triangle_order.len();
        let count = indices.len();
        triangle_order.extend(indices);
        nodes[node_index].first = first;
        nodes[node_index].count = count;
        return node_index;
    }
    let axis = longest_axis(aabb);
    indices.sort_unstable_by(|left, right| {
        let left_value = triangle_centroid_axis(mesh, *left, axis);
        let right_value = triangle_centroid_axis(mesh, *right, axis);
        left_value
            .total_cmp(&right_value)
            .then_with(|| left.cmp(right))
    });
    let right_indices = indices.split_off(indices.len() / 2);
    let left = build_bvh_node(mesh, nodes, triangle_order, indices);
    let right = build_bvh_node(mesh, nodes, triangle_order, right_indices);
    nodes[node_index].left = Some(left);
    nodes[node_index].right = Some(right);
    node_index
}

fn aabb_for_triangles(mesh: &PanelMesh, indices: &[usize]) -> PanelMeshAabb {
    let mut aabb = PanelMeshAabb {
        min: Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY),
        max: Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY),
    };
    for &index in indices {
        let [a, b, c] = mesh.triangles[index];
        for vertex in [
            mesh.vertices[a as usize],
            mesh.vertices[b as usize],
            mesh.vertices[c as usize],
        ] {
            aabb.min.x = aabb.min.x.min(vertex.x);
            aabb.min.y = aabb.min.y.min(vertex.y);
            aabb.min.z = aabb.min.z.min(vertex.z);
            aabb.max.x = aabb.max.x.max(vertex.x);
            aabb.max.y = aabb.max.y.max(vertex.y);
            aabb.max.z = aabb.max.z.max(vertex.z);
        }
    }
    aabb
}

fn longest_axis(aabb: PanelMeshAabb) -> usize {
    let dx = aabb.max.x - aabb.min.x;
    let dy = aabb.max.y - aabb.min.y;
    let dz = aabb.max.z - aabb.min.z;
    if dx >= dy && dx >= dz {
        0
    } else if dy >= dz {
        1
    } else {
        2
    }
}

fn triangle_centroid_axis(mesh: &PanelMesh, triangle_index: usize, axis: usize) -> f64 {
    let [a, b, c] = mesh.triangles[triangle_index];
    let centroid =
        (mesh.vertices[a as usize] + mesh.vertices[b as usize] + mesh.vertices[c as usize]) / 3.0;
    match axis {
        0 => centroid.x,
        1 => centroid.y,
        _ => centroid.z,
    }
}

fn ray_intersects_node(
    bvh: &PanelMeshBvh,
    node_index: usize,
    origin: Vector3<f64>,
    direction_unit: Vector3<f64>,
) -> bool {
    let node = &bvh.nodes[node_index];
    if !ray_intersects_aabb(origin, direction_unit, node.aabb) {
        return false;
    }
    if node.count > 0 {
        return bvh.triangle_order[node.first..node.first + node.count]
            .iter()
            .any(|triangle_index| {
                ray_intersects_mesh_triangle(&bvh.mesh, *triangle_index, origin, direction_unit)
            });
    }
    node.left
        .is_some_and(|left| ray_intersects_node(bvh, left, origin, direction_unit))
        || node
            .right
            .is_some_and(|right| ray_intersects_node(bvh, right, origin, direction_unit))
}

fn ray_intersects_aabb(
    origin: Vector3<f64>,
    direction_unit: Vector3<f64>,
    aabb: PanelMeshAabb,
) -> bool {
    let mut t_min = 0.0_f64;
    let mut t_max = f64::INFINITY;
    for (origin_component, direction_component, min_component, max_component) in [
        (origin.x, direction_unit.x, aabb.min.x, aabb.max.x),
        (origin.y, direction_unit.y, aabb.min.y, aabb.max.y),
        (origin.z, direction_unit.z, aabb.min.z, aabb.max.z),
    ] {
        if direction_component.abs() <= 1.0e-12 {
            if origin_component < min_component || origin_component > max_component {
                return false;
            }
            continue;
        }
        let inv_direction = 1.0 / direction_component;
        let mut t0 = (min_component - origin_component) * inv_direction;
        let mut t1 = (max_component - origin_component) * inv_direction;
        if t0 > t1 {
            core::mem::swap(&mut t0, &mut t1);
        }
        t_min = t_min.max(t0);
        t_max = t_max.min(t1);
        if t_max < t_min {
            return false;
        }
    }
    t_max > 1.0e-12
}

fn ray_intersects_mesh_triangle(
    mesh: &PanelMesh,
    triangle_index: usize,
    origin: Vector3<f64>,
    direction_unit: Vector3<f64>,
) -> bool {
    let [ia, ib, ic] = mesh.triangles[triangle_index];
    let a = mesh.vertices[ia as usize];
    let b = mesh.vertices[ib as usize];
    let c = mesh.vertices[ic as usize];
    let edge1 = b - a;
    let edge2 = c - a;
    let pvec = direction_unit.cross(&edge2);
    let det = edge1.dot(&pvec);
    if det.abs() <= 1.0e-12 {
        return false;
    }
    let inv_det = 1.0 / det;
    let tvec = origin - a;
    let u = tvec.dot(&pvec) * inv_det;
    if !(0.0..=1.0).contains(&u) {
        return false;
    }
    let qvec = tvec.cross(&edge1);
    let v = direction_unit.dot(&qvec) * inv_det;
    if v < 0.0 || u + v > 1.0 {
        return false;
    }
    edge2.dot(&qvec) * inv_det > 1.0e-12
}

fn require_finite_vector(field: &'static str, vector: Vector3<f64>) -> Result<(), AeroError> {
    if vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite() {
        Ok(())
    } else {
        Err(AeroError::NonFinite { reason: field })
    }
}

/// Per-panel pressure-coefficient model.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum PanelMethod {
    /// Modified Newtonian: `Cp = Cp_max cos²(theta)`.
    ModifiedNewtonian,
}

/// Reference-temperature skin-friction settings for panel integration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct PanelSkinFriction {
    /// Reynolds-number reference length used by [`AeroContext::reynolds_length`].
    pub reference_length_m: f64,
    /// Wall-to-boundary-layer-edge temperature ratio `T_w / T_e`.
    pub wall_to_edge_temperature_ratio: f64,
    /// Reynolds number where the flat-plate correlation switches to turbulent.
    pub transition_reynolds: f64,
    /// Equivalent roughness divided by `reference_length_m`.
    pub roughness_over_length: f64,
}

impl PanelSkinFriction {
    /// Default smooth-wall transition Reynolds number.
    pub const DEFAULT_TRANSITION_REYNOLDS: f64 = 5.0e5;
    /// Lower Reynolds envelope shared with the continuum buildup model.
    pub const MIN_REYNOLDS: f64 = 1.0e3;

    /// Build a smooth-wall reference-temperature model with default transition.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::InvalidParameter`] when the reference length is not
    /// positive.
    pub fn smooth_reference_temperature(reference_length_m: f64) -> Result<Self, AeroError> {
        Self::new(
            reference_length_m,
            1.0,
            Self::DEFAULT_TRANSITION_REYNOLDS,
            0.0,
        )
    }

    /// Build a reference-temperature skin-friction configuration.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::InvalidParameter`] when a parameter is non-finite or
    /// outside its positive / non-negative envelope.
    pub fn new(
        reference_length_m: f64,
        wall_to_edge_temperature_ratio: f64,
        transition_reynolds: f64,
        roughness_over_length: f64,
    ) -> Result<Self, AeroError> {
        let model = Self {
            reference_length_m,
            wall_to_edge_temperature_ratio,
            transition_reynolds,
            roughness_over_length,
        };
        model.validate()?;
        Ok(model)
    }

    /// Return the local skin-friction coefficient at `mach` and `Re_x`.
    ///
    /// The smooth-wall branch uses Blasius mean laminar `Cf = 1.328/sqrt(Re*)`
    /// below transition and the compressible Schlichting/White turbulent form
    /// above transition. Roughness acts as a lower bound.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] for invalid model, Mach, Reynolds, or reference
    /// temperature values.
    pub fn skin_friction_coefficient(&self, mach: f64, reynolds_x: f64) -> Result<f64, AeroError> {
        self.validate()?;
        if !(mach.is_finite() && reynolds_x.is_finite()) {
            return Err(AeroError::NonFinite {
                reason: "panel skin-friction Mach or Reynolds number is NaN or Inf",
            });
        }
        if mach < 0.0 || reynolds_x < Self::MIN_REYNOLDS {
            return Err(AeroError::InvalidParameter {
                reason: "panel skin-friction Mach/Reynolds input is outside envelope",
            });
        }
        let reference_temperature_ratio =
            1.0 + 0.032 * mach * mach + 0.58 * (self.wall_to_edge_temperature_ratio - 1.0);
        if !(reference_temperature_ratio.is_finite() && reference_temperature_ratio > 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "panel skin-friction reference temperature ratio is outside envelope",
            });
        }
        let reynolds_star =
            (reynolds_x / reference_temperature_ratio.powf(1.5)).max(Self::MIN_REYNOLDS);
        let smooth = if reynolds_star < self.transition_reynolds {
            1.328 / reynolds_star.sqrt()
        } else {
            let compressibility = (1.0 + 0.144 * mach * mach).powf(0.65);
            0.455 / (reynolds_star.log10().powf(2.58) * compressibility)
        };
        let rough = if self.roughness_over_length > 0.0 {
            0.032 * self.roughness_over_length.powf(0.2)
        } else {
            0.0
        };
        Ok(smooth.max(rough))
    }

    fn validate(&self) -> Result<(), AeroError> {
        if !(self.reference_length_m.is_finite()
            && self.wall_to_edge_temperature_ratio.is_finite()
            && self.transition_reynolds.is_finite()
            && self.roughness_over_length.is_finite())
        {
            return Err(AeroError::InvalidParameter {
                reason: "panel skin-friction parameter is NaN or Inf",
            });
        }
        if self.reference_length_m <= 0.0
            || self.wall_to_edge_temperature_ratio <= 0.0
            || self.transition_reynolds <= 0.0
            || self.roughness_over_length < 0.0
        {
            return Err(AeroError::InvalidParameter {
                reason: "panel skin-friction parameter is outside envelope",
            });
        }
        Ok(())
    }
}

/// Integrated aerodynamic load over one axial station bin.
#[derive(Clone, Debug, PartialEq)]
pub struct AxialLineLoad {
    /// Inclusive lower station bound in body-frame x (m).
    pub x_min_m: f64,
    /// Exclusive upper station bound in body-frame x (m), except for
    /// the final bin where the upper bound is inclusive.
    pub x_max_m: f64,
    /// Integrated body-frame force over panels whose centroids fall
    /// inside this axial bin (N).
    pub force_n_body: Vector3<f64>,
    /// Integrated body-frame moment about the solution CG from panels
    /// whose centroids fall inside this axial bin (N m).
    pub moment_n_m_body: Vector3<f64>,
}

/// Local-inclination panel solution with optional axial line-load bins.
#[derive(Clone, Debug, PartialEq)]
pub struct PanelSolution {
    /// Integrated body-frame force over all contributing panels (N).
    pub force_n_body: Vector3<f64>,
    /// Integrated body-frame moment about `cg_body_m` (N m).
    pub moment_n_m_body: Vector3<f64>,
    /// Moment reference point in body-frame metres.
    pub cg_body_m: Vector3<f64>,
    /// Integrated axial station loads, one per adjacent station pair.
    pub axial_line_loads: Vec<AxialLineLoad>,
}

/// Local-inclination panel integration.
///
/// Each triangle contributes `F_i = -Cp_i q A_i n_i`, where `n_i`
/// is the outward unit normal from the mesh winding. Moments use
/// `(centroid - cg_body_m) × F_i`. When `shadowing` is
/// true, panels with `n_i · u_upstream < 0` are skipped. When
/// `shadowing` is false, triangles are treated as two-sided academic
/// panels: away-facing normals are flipped for the pressure
/// direction, so open-mesh fixtures can still produce load without
/// changing vertex winding. Optional skin friction adds an axial
/// tangential drag term without changing the default inviscid behavior.
#[derive(Clone, Debug, PartialEq)]
pub struct LocalInclinationPanels {
    /// Triangulated body surface.
    pub geometry: PanelMesh,
    /// Per-panel pressure model.
    pub method: PanelMethod,
    /// Whether to zero back-facing panels. If false, panels are
    /// evaluated as two-sided by flipping away-facing normals for the
    /// pressure direction.
    pub shadowing: bool,
    /// Stagnation pressure coefficient used by Modified Newtonian.
    pub cp_max: f64,
    /// Optional reference-temperature skin-friction model.
    pub skin_friction: Option<PanelSkinFriction>,
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
            skin_friction: None,
        }
    }

    /// Return a copy with reference-temperature skin friction enabled.
    #[must_use]
    pub const fn with_skin_friction(mut self, skin_friction: PanelSkinFriction) -> Self {
        self.skin_friction = Some(skin_friction);
        self
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

    /// Solve the panel loads about the body-frame origin without
    /// station bins.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when context or model parameters are
    /// malformed, non-finite, or produce non-finite loads.
    pub fn solve(&self, ctx: &AeroContext) -> Result<PanelSolution, AeroError> {
        self.solve_about_cg(ctx, Vector3::zeros(), &[])
    }

    /// Solve the panel loads about the body-frame origin using
    /// BVH-backed ray shadowing.
    ///
    /// This path requires the panel mesh to be a watertight
    /// two-manifold. It preserves the sign-test back-face culling and
    /// additionally rejects otherwise facing panels whose centroid ray
    /// toward the upstream direction intersects another mesh panel.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when the mesh is not watertight, context
    /// or model parameters are malformed, or the solve produces
    /// non-finite loads.
    pub fn solve_with_bvh_shadowing(&self, ctx: &AeroContext) -> Result<PanelSolution, AeroError> {
        self.solve_about_cg_with_bvh_shadowing(ctx, Vector3::zeros(), &[])
    }

    /// Solve panel loads about an arbitrary body-frame CG with
    /// optional axial station bins.
    ///
    /// `axial_stations_m` may be empty. When supplied, it must contain
    /// at least two finite, strictly increasing body-x coordinates;
    /// adjacent pairs define the reported [`AxialLineLoad`] bins.
    /// Panels with centroids outside the supplied station envelope
    /// still contribute to total force and moment but not to any line
    /// bin.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when context, CG, station, or model
    /// inputs are malformed, non-finite, or produce non-finite loads.
    pub fn solve_about_cg(
        &self,
        ctx: &AeroContext,
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<PanelSolution, AeroError> {
        self.solve_about_cg_internal(ctx, cg_body_m, axial_stations_m, None)
    }

    /// Solve panel loads about an arbitrary body-frame CG using
    /// BVH-backed ray shadowing.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when the mesh is not watertight, context,
    /// CG, station, or model inputs are malformed, or the solve
    /// produces non-finite loads.
    pub fn solve_about_cg_with_bvh_shadowing(
        &self,
        ctx: &AeroContext,
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
    ) -> Result<PanelSolution, AeroError> {
        let shadow_bvh = PanelMeshBvh::from_watertight_mesh(self.geometry.clone())?;
        self.solve_about_cg_internal(ctx, cg_body_m, axial_stations_m, Some(&shadow_bvh))
    }

    /// Solve panel loads about a body-frame CG using a caller-supplied external
    /// shadow BVH instead of this body's own self-shadow BVH.
    ///
    /// Forces are still integrated over this body's panels only; the supplied
    /// BVH is used purely for ray-shadow occlusion. Passing a BVH built from
    /// this body combined with a second body (positioned at a relative offset)
    /// yields the proximity-shadowed loads on this body, which is how the
    /// stage-separation proximity-interference deck (WP-03.7) is generated.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError`] when the mesh is not watertight or the context,
    /// CG, station, or model inputs are malformed, or the solve produces
    /// non-finite loads.
    pub fn solve_about_cg_with_shadow_bvh(
        &self,
        ctx: &AeroContext,
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
        shadow_bvh: &PanelMeshBvh,
    ) -> Result<PanelSolution, AeroError> {
        self.solve_about_cg_internal(ctx, cg_body_m, axial_stations_m, Some(shadow_bvh))
    }

    fn solve_about_cg_internal(
        &self,
        ctx: &AeroContext,
        cg_body_m: Vector3<f64>,
        axial_stations_m: &[f64],
        shadow_bvh: Option<&PanelMeshBvh>,
    ) -> Result<PanelSolution, AeroError> {
        validate_context(ctx)?;
        require_finite_vector(
            "local_inclination_panels cg_body_m is NaN or Inf",
            cg_body_m,
        )?;
        validate_axial_stations(axial_stations_m)?;
        if !(self.cp_max.is_finite() && self.cp_max >= 0.0) {
            return Err(AeroError::InvalidParameter {
                reason: "local_inclination_panels cp_max must be finite and non-negative",
            });
        }
        let upstream = Self::upstream_unit(ctx);
        let mesh_x_min = self.skin_friction.map(|_| panel_mesh_min_x(&self.geometry));
        let mut force = Vector3::zeros();
        let mut moment = Vector3::zeros();
        let mut axial_line_loads = axial_stations_m
            .windows(2)
            .map(|window| AxialLineLoad {
                x_min_m: window[0],
                x_max_m: window[1],
                force_n_body: Vector3::zeros(),
                moment_n_m_body: Vector3::zeros(),
            })
            .collect::<Vec<_>>();
        let pressure_scale = ctx.dynamic_pressure_pa;
        for &[ia, ib, ic] in self.geometry.triangles() {
            let a = self.geometry.vertices[ia as usize];
            let b = self.geometry.vertices[ib as usize];
            let c = self.geometry.vertices[ic as usize];
            let area_vec = (b - a).cross(&(c - a));
            let area = 0.5 * area_vec.norm();
            let normal = area_vec.normalize();
            let cos_theta = normal.dot(&upstream);
            if self.shadowing && cos_theta < 0.0 {
                continue;
            }
            let facing_normal = if cos_theta >= 0.0 { normal } else { -normal };
            let cos_incidence = cos_theta.abs();
            let cp = match self.method {
                PanelMethod::ModifiedNewtonian => self.cp_max * cos_incidence * cos_incidence,
            };
            let panel_force = -facing_normal * (cp * pressure_scale * area);
            let centroid = (a + b + c) / 3.0;
            if self.shadowing
                && let Some(bvh) = shadow_bvh
                && panel_centroid_occluded_by_bvh(bvh, centroid, upstream)?
            {
                continue;
            }
            let panel_friction =
                if let (Some(skin_friction), Some(mesh_x_min)) = (self.skin_friction, mesh_x_min) {
                    panel_skin_friction_force(
                        skin_friction,
                        ctx,
                        mesh_x_min,
                        centroid,
                        normal,
                        upstream,
                        area,
                    )?
                } else {
                    Vector3::zeros()
                };
            let panel_force = panel_force + panel_friction;
            let panel_moment = (centroid - cg_body_m).cross(&panel_force);
            force += panel_force;
            moment += panel_moment;
            if let Some(bin) = axial_station_bin(axial_stations_m, centroid.x) {
                axial_line_loads[bin].force_n_body += panel_force;
                axial_line_loads[bin].moment_n_m_body += panel_moment;
            }
        }
        if !is_finite_vector(force)
            || !is_finite_vector(moment)
            || axial_line_loads.iter().any(|bin| {
                !is_finite_vector(bin.force_n_body) || !is_finite_vector(bin.moment_n_m_body)
            })
        {
            return Err(AeroError::NonFinite {
                reason: "local-inclination force or moment is non-finite",
            });
        }
        Ok(PanelSolution {
            force_n_body: force,
            moment_n_m_body: moment,
            cg_body_m,
            axial_line_loads,
        })
    }
}

fn panel_centroid_occluded_by_bvh(
    bvh: &PanelMeshBvh,
    centroid_body_m: Vector3<f64>,
    upstream_unit: Vector3<f64>,
) -> Result<bool, AeroError> {
    let origin = centroid_body_m + upstream_unit * 1.0e-9;
    bvh.ray_intersects(origin, upstream_unit)
}

fn panel_skin_friction_force(
    skin_friction: PanelSkinFriction,
    ctx: &AeroContext,
    mesh_x_min_m: f64,
    centroid_body_m: Vector3<f64>,
    normal: Vector3<f64>,
    upstream: Vector3<f64>,
    area_m2: f64,
) -> Result<Vector3<f64>, AeroError> {
    if !(ctx.reynolds_length.is_finite() && ctx.reynolds_length > 0.0) {
        return Err(AeroError::InvalidParameter {
            reason: "panel skin-friction context Reynolds number must be positive",
        });
    }
    let tangent = upstream - normal * normal.dot(&upstream);
    let tangent_norm = tangent.norm();
    if tangent_norm <= 0.0 {
        return Ok(Vector3::zeros());
    }
    let x_from_le_m = (centroid_body_m.x - mesh_x_min_m).max(0.0);
    let reynolds_x = (ctx.reynolds_length * x_from_le_m / skin_friction.reference_length_m)
        .max(PanelSkinFriction::MIN_REYNOLDS);
    let cf = skin_friction.skin_friction_coefficient(ctx.mach, reynolds_x)?;
    Ok(-(tangent / tangent_norm) * (ctx.dynamic_pressure_pa * cf * area_m2))
}

fn panel_mesh_min_x(mesh: &PanelMesh) -> f64 {
    mesh.vertices()
        .iter()
        .fold(f64::INFINITY, |minimum, vertex| minimum.min(vertex.x))
}

impl AeroMethod for LocalInclinationPanels {
    fn aero_force_moment_body(&self, ctx: &AeroContext) -> Result<AeroForceMomentBody, AeroError> {
        let solution = self.solve(ctx)?;
        Ok(AeroForceMomentBody {
            force_n_body: solution.force_n_body,
            moment_n_m_body: solution.moment_n_m_body,
        })
    }
}

fn validate_axial_stations(axial_stations_m: &[f64]) -> Result<(), AeroError> {
    if axial_stations_m.is_empty() {
        return Ok(());
    }
    if axial_stations_m.len() < 2 {
        return Err(AeroError::InvalidParameter {
            reason: "local_inclination_panels axial stations require at least two coordinates",
        });
    }
    let mut previous = axial_stations_m[0];
    if !previous.is_finite() {
        return Err(AeroError::NonFinite {
            reason: "local_inclination_panels axial station is NaN or Inf",
        });
    }
    for &station in &axial_stations_m[1..] {
        if !station.is_finite() {
            return Err(AeroError::NonFinite {
                reason: "local_inclination_panels axial station is NaN or Inf",
            });
        }
        if station <= previous {
            return Err(AeroError::InvalidParameter {
                reason: "local_inclination_panels axial stations must be strictly increasing",
            });
        }
        previous = station;
    }
    Ok(())
}

fn axial_station_bin(axial_stations_m: &[f64], x_m: f64) -> Option<usize> {
    if axial_stations_m.len() < 2 {
        return None;
    }
    let last = axial_stations_m.len() - 1;
    (0..last).find(|&index| {
        x_m >= axial_stations_m[index]
            && (x_m < axial_stations_m[index + 1]
                || (index + 1 == last && x_m <= axial_stations_m[index + 1]))
    })
}

fn is_finite_vector(vector: Vector3<f64>) -> bool {
    vector.x.is_finite() && vector.y.is_finite() && vector.z.is_finite()
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
/// Scope: produces aero force as if the vehicle were a
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
        let numerator = g_plus * g_plus * m2;
        let denom = 4.0 * g * m2 - 2.0 * g_minus;
        let ratio_t = numerator / denom;
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
        ("knudsen", ctx.knudsen),
        ("reynolds_length", ctx.reynolds_length),
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

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelSphereToleranceFixture {
        case: PanelSphereToleranceCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelSphereToleranceCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        radius_m: f64,
        latitude_segments: usize,
        longitude_segments: usize,
        cp_max: f64,
        mach: f64,
        alpha_deg: f64,
        beta_deg: f64,
        dynamic_pressure_pa: f64,
        expected_cd: f64,
        max_relative_error: f64,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelSharpConeToleranceFixture {
        case: PanelSharpConeToleranceCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelSharpConeToleranceCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        base_radius_m: f64,
        half_angle_deg: f64,
        longitude_segments: usize,
        mach: f64,
        gamma: f64,
        cp_max: f64,
        dynamic_pressure_pa: f64,
        expected_ca: f64,
        max_relative_error: f64,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelHb2ForebodyFigure12Fixture {
        case: PanelHb2ForebodyFigure12Case,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelHb2ForebodyFigure12Case {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        source_id: String,
        figure_id: String,
        reference_diameter_m: f64,
        body_length_over_diameter: f64,
        station_segments: usize,
        longitude_segments: usize,
        gamma: f64,
        dynamic_pressure_pa: f64,
        max_relative_error: f64,
        samples: Vec<PanelHb2ForebodyFigure12Sample>,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelHb2ForebodyFigure12Sample {
        mach: f64,
        reynolds_diameter: f64,
        expected_forebody_ca: f64,
        digitized_note: String,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelHb2ForebodySkinFrictionFigure12Fixture {
        case: PanelHb2ForebodySkinFrictionFigure12Case,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelHb2ForebodySkinFrictionFigure12Case {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        source_id: String,
        figure_id: String,
        reference_diameter_m: f64,
        body_length_over_diameter: f64,
        station_segments: usize,
        longitude_segments: usize,
        gamma: f64,
        dynamic_pressure_pa: f64,
        wall_to_edge_temperature_ratio: f64,
        transition_reynolds: f64,
        roughness_over_length: f64,
        max_relative_error: f64,
        samples: Vec<PanelHb2ForebodySkinFrictionFigure12Sample>,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelHb2ForebodySkinFrictionFigure12Sample {
        mach: f64,
        reynolds_diameter: f64,
        expected_total_forebody_ca: f64,
        digitized_note: String,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelSharpConeSkinFrictionTrendFixture {
        case: PanelSharpConeSkinFrictionTrendCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelSharpConeSkinFrictionTrendCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        base_radius_m: f64,
        half_angle_deg: f64,
        longitude_segments: usize,
        mach: f64,
        gamma: f64,
        dynamic_pressure_pa: f64,
        wall_to_edge_temperature_ratio: f64,
        transition_reynolds: f64,
        roughness_over_length: f64,
        low_reynolds_length: f64,
        high_reynolds_length: f64,
        expected_inviscid_ca: f64,
        expected_low_re_viscous_delta_ca: f64,
        expected_high_re_viscous_delta_ca: f64,
        max_relative_error: f64,
        max_high_re_delta_fraction_of_inviscid: f64,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelWedgeCavityShadowFixture {
        case: PanelWedgeCavityShadowCase,
    }

    #[cfg(feature = "parser")]
    #[derive(Debug, serde::Deserialize)]
    struct PanelWedgeCavityShadowCase {
        dataset_id: String,
        source_class: String,
        validation_label: String,
        mach: f64,
        alpha_deg: f64,
        beta_deg: f64,
        dynamic_pressure_pa: f64,
        cp_max: f64,
        expected_occluded_triangle_indices: Vec<usize>,
        expected_sign_only_force_x_n: f64,
        expected_bvh_shadowed_force_x_n: f64,
    }

    fn ctx(mach: f64, alpha: f64, q: f64) -> AeroContext {
        AeroContext {
            mach,
            alpha_deg: alpha,
            beta_deg: 0.0,
            dynamic_pressure_pa: q,
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
        }
    }

    fn ctx_with_beta(mach: f64, alpha: f64, beta: f64, q: f64) -> AeroContext {
        AeroContext {
            mach,
            alpha_deg: alpha,
            beta_deg: beta,
            dynamic_pressure_pa: q,
            airspeed_m_s: 100.0,
            body_angular_rates_rad_s: Vector3::zeros(),
            knudsen: 0.0,
            reynolds_length: 0.0,
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

    fn flow_aligned_plate_mesh() -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(0.0, -0.5, 0.0),
                Vector3::new(1.0, -0.5, 0.0),
                Vector3::new(1.0, 0.5, 0.0),
                Vector3::new(0.0, 0.5, 0.0),
            ],
            vec![[0, 1, 2], [0, 2, 3]],
        )
        .unwrap()
    }

    fn two_station_plate_mesh() -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(1.0, -0.5, -0.5),
                Vector3::new(1.0, 0.5, -0.5),
                Vector3::new(1.0, 0.5, 0.5),
                Vector3::new(1.0, -0.5, 0.5),
                Vector3::new(3.0, -0.5, -0.5),
                Vector3::new(3.0, 0.5, -0.5),
                Vector3::new(3.0, 0.5, 0.5),
                Vector3::new(3.0, -0.5, 0.5),
            ],
            vec![[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]],
        )
        .unwrap()
    }

    fn latitude_longitude_sphere_mesh(
        radius_m: f64,
        latitude_segments: usize,
        longitude_segments: usize,
    ) -> PanelMesh {
        assert!(latitude_segments >= 4);
        assert!(longitude_segments >= 8);
        let mut vertices = Vec::new();
        vertices.push(Vector3::new(radius_m, 0.0, 0.0));
        for i in 1..latitude_segments {
            let theta = PI * (i as f64) / (latitude_segments as f64);
            let sin_theta = theta.sin();
            let cos_theta = theta.cos();
            for j in 0..longitude_segments {
                let phi = 2.0 * PI * (j as f64) / (longitude_segments as f64);
                vertices.push(Vector3::new(
                    radius_m * cos_theta,
                    radius_m * sin_theta * phi.cos(),
                    radius_m * sin_theta * phi.sin(),
                ));
            }
        }
        let back_pole = vertices.len();
        vertices.push(Vector3::new(-radius_m, 0.0, 0.0));

        let ring_index = |ring: usize, longitude: usize| -> u32 {
            u32::try_from(1 + (ring - 1) * longitude_segments + longitude % longitude_segments)
                .unwrap()
        };
        let front_pole = 0_u32;
        let back_pole = u32::try_from(back_pole).unwrap();
        let mut triangles = Vec::new();
        for j in 0..longitude_segments {
            triangles.push([front_pole, ring_index(1, j), ring_index(1, j + 1)]);
        }
        for ring in 1..(latitude_segments - 1) {
            for j in 0..longitude_segments {
                let v00 = ring_index(ring, j);
                let v10 = ring_index(ring + 1, j);
                let v11 = ring_index(ring + 1, j + 1);
                let v01 = ring_index(ring, j + 1);
                triangles.push([v00, v10, v11]);
                triangles.push([v00, v11, v01]);
            }
        }
        let last_ring = latitude_segments - 1;
        for j in 0..longitude_segments {
            triangles.push([
                ring_index(last_ring, j),
                back_pole,
                ring_index(last_ring, j + 1),
            ]);
        }

        PanelMesh::new(vertices, triangles).unwrap()
    }

    fn sharp_cone_side_mesh(
        base_radius_m: f64,
        half_angle_rad: f64,
        longitude_segments: usize,
    ) -> PanelMesh {
        assert!(base_radius_m > 0.0);
        assert!(half_angle_rad > 0.0 && half_angle_rad < 0.5 * PI);
        assert!(longitude_segments >= 8);
        let length_m = base_radius_m / half_angle_rad.tan();
        let mut vertices = Vec::with_capacity(longitude_segments + 1);
        vertices.push(Vector3::new(length_m, 0.0, 0.0));
        for j in 0..longitude_segments {
            let phi = 2.0 * PI * (j as f64) / (longitude_segments as f64);
            vertices.push(Vector3::new(
                0.0,
                base_radius_m * phi.cos(),
                base_radius_m * phi.sin(),
            ));
        }
        let base_index = |longitude: usize| -> u32 {
            u32::try_from(1 + longitude % longitude_segments).unwrap()
        };
        let mut triangles = Vec::with_capacity(longitude_segments);
        for j in 0..longitude_segments {
            triangles.push([0, base_index(j), base_index(j + 1)]);
        }
        PanelMesh::new(vertices, triangles).unwrap()
    }

    fn hb2_forebody_radius_over_diameter(station_over_diameter: f64) -> f64 {
        const NOSE_RADIUS_D: f64 = 0.3;
        const BODY_LENGTH_D: f64 = 4.5;
        const HEMISPHERE_CONE_D: f64 = 0.0353 * 4.9;
        const CONE_SHOULDER_D: f64 = 0.1064 * 4.9;
        const SHOULDER_CYLINDER_D: f64 = 0.167 * 4.9;
        const CYLINDER_RADIUS_D: f64 = 0.583 * 4.9;
        const RADIUS_FLARE_D: f64 = 0.725 * 4.9;
        const CONE_SHOULDER_RADIUS_D: f64 = 0.435;
        const CYLINDER_RADIUS: f64 = 0.5;
        const RADIUS_FLARE_RADIUS_D: f64 = 0.55;
        const AFT_FLARE_RADIUS_D: f64 = 0.7;

        let s = station_over_diameter.clamp(0.0, BODY_LENGTH_D);
        if s <= HEMISPHERE_CONE_D {
            return (NOSE_RADIUS_D * NOSE_RADIUS_D - (s - NOSE_RADIUS_D).powi(2))
                .max(0.0)
                .sqrt();
        }
        if s <= CONE_SHOULDER_D {
            let t = (s - HEMISPHERE_CONE_D) / (CONE_SHOULDER_D - HEMISPHERE_CONE_D);
            let r0 = hb2_forebody_radius_over_diameter(HEMISPHERE_CONE_D);
            return r0 + t * (CONE_SHOULDER_RADIUS_D - r0);
        }
        if s <= SHOULDER_CYLINDER_D {
            let t = (s - CONE_SHOULDER_D) / (SHOULDER_CYLINDER_D - CONE_SHOULDER_D);
            let smooth = t * t * (3.0 - 2.0 * t);
            return CONE_SHOULDER_RADIUS_D + smooth * (CYLINDER_RADIUS - CONE_SHOULDER_RADIUS_D);
        }
        if s <= CYLINDER_RADIUS_D {
            return CYLINDER_RADIUS;
        }
        if s <= RADIUS_FLARE_D {
            let t = (s - CYLINDER_RADIUS_D) / (RADIUS_FLARE_D - CYLINDER_RADIUS_D);
            let smooth = t * t * (3.0 - 2.0 * t);
            return CYLINDER_RADIUS + smooth * (RADIUS_FLARE_RADIUS_D - CYLINDER_RADIUS);
        }
        let t = (s - RADIUS_FLARE_D) / (BODY_LENGTH_D - RADIUS_FLARE_D);
        RADIUS_FLARE_RADIUS_D + t * (AFT_FLARE_RADIUS_D - RADIUS_FLARE_RADIUS_D)
    }

    fn hb2_forebody_side_mesh(
        reference_diameter_m: f64,
        station_segments: usize,
        longitude_segments: usize,
    ) -> PanelMesh {
        assert!(reference_diameter_m > 0.0);
        assert!(station_segments >= 8);
        assert!(longitude_segments >= 8);
        let body_length_m = 4.5 * reference_diameter_m;
        let mut vertices = Vec::with_capacity(1 + station_segments * longitude_segments);
        vertices.push(Vector3::new(body_length_m, 0.0, 0.0));
        for i in 1..=station_segments {
            let station_d = 4.5 * (i as f64) / (station_segments as f64);
            let x_m = body_length_m - station_d * reference_diameter_m;
            let radius_m = hb2_forebody_radius_over_diameter(station_d) * reference_diameter_m;
            for j in 0..longitude_segments {
                let phi = 2.0 * PI * (j as f64) / (longitude_segments as f64);
                vertices.push(Vector3::new(
                    x_m,
                    radius_m * phi.cos(),
                    radius_m * phi.sin(),
                ));
            }
        }
        let ring_index = |ring: usize, longitude: usize| -> u32 {
            u32::try_from(1 + (ring - 1) * longitude_segments + longitude % longitude_segments)
                .unwrap()
        };
        let mut triangles =
            Vec::with_capacity(longitude_segments * (1 + 2 * (station_segments - 1)));
        for j in 0..longitude_segments {
            triangles.push([0, ring_index(1, j), ring_index(1, j + 1)]);
        }
        for ring in 1..station_segments {
            for j in 0..longitude_segments {
                let upstream = ring_index(ring, j);
                let upstream_next = ring_index(ring, j + 1);
                let downstream = ring_index(ring + 1, j);
                let downstream_next = ring_index(ring + 1, j + 1);
                triangles.push([upstream, downstream, downstream_next]);
                triangles.push([upstream, downstream_next, upstream_next]);
            }
        }
        PanelMesh::new(vertices, triangles).unwrap()
    }

    fn front_facing_cuboid_mesh(
        x_min_m: f64,
        x_max_m: f64,
        y_half_m: f64,
        z_half_m: f64,
    ) -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(x_max_m, -y_half_m, -z_half_m),
                Vector3::new(x_max_m, y_half_m, -z_half_m),
                Vector3::new(x_max_m, y_half_m, z_half_m),
                Vector3::new(x_max_m, -y_half_m, z_half_m),
                Vector3::new(x_min_m, -y_half_m, -z_half_m),
                Vector3::new(x_min_m, -y_half_m, z_half_m),
                Vector3::new(x_min_m, y_half_m, z_half_m),
                Vector3::new(x_min_m, y_half_m, -z_half_m),
            ],
            vec![
                [0, 1, 2],
                [0, 2, 3],
                [4, 5, 6],
                [4, 6, 7],
                [4, 7, 1],
                [4, 1, 0],
                [5, 3, 2],
                [5, 2, 6],
                [4, 0, 3],
                [4, 3, 5],
                [7, 6, 2],
                [7, 2, 1],
            ],
        )
        .unwrap()
    }

    fn triangular_prism_wedge_mesh(
        x_min_m: f64,
        x_max_m: f64,
        y_half_m: f64,
        z_half_m: f64,
    ) -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(x_max_m, -y_half_m, -z_half_m),
                Vector3::new(x_max_m, y_half_m, -z_half_m),
                Vector3::new(x_max_m, 0.0, z_half_m),
                Vector3::new(x_min_m, -y_half_m, -z_half_m),
                Vector3::new(x_min_m, y_half_m, -z_half_m),
                Vector3::new(x_min_m, 0.0, z_half_m),
            ],
            vec![
                [0, 1, 2],
                [3, 5, 4],
                [3, 4, 1],
                [3, 1, 0],
                [3, 0, 2],
                [3, 2, 5],
                [4, 5, 2],
                [4, 2, 1],
            ],
        )
        .unwrap()
    }

    fn merge_meshes(first: PanelMesh, second: PanelMesh) -> PanelMesh {
        let mut vertices = first.vertices().to_vec();
        let offset = u32::try_from(vertices.len()).unwrap();
        vertices.extend(second.vertices().iter().copied());
        let mut triangles = first.triangles().to_vec();
        triangles.extend(
            second
                .triangles()
                .iter()
                .map(|[a, b, c]| [a + offset, b + offset, c + offset]),
        );
        PanelMesh::new(vertices, triangles).unwrap()
    }

    fn nested_shadow_mesh() -> PanelMesh {
        merge_meshes(
            front_facing_cuboid_mesh(1.0, 2.0, 2.0, 2.0),
            front_facing_cuboid_mesh(1.2, 1.4, 0.5, 0.5),
        )
    }

    fn wedge_cavity_shadow_mesh() -> PanelMesh {
        merge_meshes(
            front_facing_cuboid_mesh(1.0, 2.0, 2.0, 2.0),
            triangular_prism_wedge_mesh(1.2, 1.4, 0.5, 0.5),
        )
    }

    fn bvh_occluded_facing_triangle_indices(
        panels: &LocalInclinationPanels,
        ctx: &AeroContext,
    ) -> Vec<usize> {
        let upstream = LocalInclinationPanels::upstream_unit(ctx);
        let bvh = PanelMeshBvh::from_watertight_mesh(panels.geometry.clone()).unwrap();
        panels
            .geometry
            .triangles()
            .iter()
            .enumerate()
            .filter_map(|(index, &[ia, ib, ic])| {
                let a = panels.geometry.vertices[ia as usize];
                let b = panels.geometry.vertices[ib as usize];
                let c = panels.geometry.vertices[ic as usize];
                let area_vec = (b - a).cross(&(c - a));
                let normal = area_vec.normalize();
                if normal.dot(&upstream) <= 0.0 {
                    return None;
                }
                let centroid = (a + b + c) / 3.0;
                panel_centroid_occluded_by_bvh(&bvh, centroid, upstream)
                    .unwrap()
                    .then_some(index)
            })
            .collect()
    }

    fn closed_box_mesh() -> PanelMesh {
        PanelMesh::new(
            vec![
                Vector3::new(1.0, -1.0, -1.0),
                Vector3::new(1.0, 1.0, -1.0),
                Vector3::new(1.0, 1.0, 1.0),
                Vector3::new(1.0, -1.0, 1.0),
                Vector3::new(2.0, -1.0, -1.0),
                Vector3::new(2.0, 1.0, -1.0),
                Vector3::new(2.0, 1.0, 1.0),
                Vector3::new(2.0, -1.0, 1.0),
            ],
            vec![
                [0, 1, 2],
                [0, 2, 3],
                [4, 6, 5],
                [4, 7, 6],
                [0, 3, 7],
                [0, 7, 4],
                [1, 5, 6],
                [1, 6, 2],
                [0, 4, 5],
                [0, 5, 1],
                [3, 2, 6],
                [3, 6, 7],
            ],
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
    fn local_inclination_solution_reports_cg_referenced_moment() {
        let panels = LocalInclinationPanels::modified_newtonian(square_plate_mesh(), 2.0);
        let origin = panels.solve(&ctx(12.0, 0.0, 100.0)).unwrap();
        let shifted = panels
            .solve_about_cg(&ctx(12.0, 0.0, 100.0), Vector3::new(0.0, 0.0, 1.0), &[])
            .unwrap();

        assert_relative_eq!(origin.moment_n_m_body.norm(), 0.0, epsilon = 1e-12);
        assert_relative_eq!(shifted.force_n_body.x, -200.0, epsilon = 1e-12);
        assert_relative_eq!(shifted.moment_n_m_body.x, 0.0, epsilon = 1e-12);
        assert_relative_eq!(shifted.moment_n_m_body.y, 200.0, epsilon = 1e-12);
        assert_relative_eq!(shifted.moment_n_m_body.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn local_inclination_solution_bins_axial_line_loads() {
        let panels = LocalInclinationPanels::modified_newtonian(two_station_plate_mesh(), 2.0);
        let solution = panels
            .solve_about_cg(&ctx(12.0, 0.0, 100.0), Vector3::zeros(), &[0.0, 2.0, 4.0])
            .unwrap();

        assert_eq!(solution.axial_line_loads.len(), 2);
        assert_relative_eq!(solution.force_n_body.x, -400.0, epsilon = 1e-12);
        assert_relative_eq!(
            solution.axial_line_loads[0].force_n_body.x,
            -200.0,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            solution.axial_line_loads[1].force_n_body.x,
            -200.0,
            epsilon = 1e-12
        );
        let binned_force =
            solution.axial_line_loads[0].force_n_body + solution.axial_line_loads[1].force_n_body;
        assert_eq!(binned_force.x.to_bits(), solution.force_n_body.x.to_bits());
        assert_eq!(binned_force.y.to_bits(), solution.force_n_body.y.to_bits());
        assert_eq!(binned_force.z.to_bits(), solution.force_n_body.z.to_bits());
    }

    #[test]
    fn local_inclination_solution_rejects_invalid_cg_and_station_bins() {
        let panels = LocalInclinationPanels::modified_newtonian(square_plate_mesh(), 2.0);
        assert!(matches!(
            panels.solve_about_cg(
                &ctx(12.0, 0.0, 100.0),
                Vector3::new(f64::NAN, 0.0, 0.0),
                &[]
            ),
            Err(AeroError::NonFinite { .. })
        ));
        assert!(matches!(
            panels.solve_about_cg(&ctx(12.0, 0.0, 100.0), Vector3::zeros(), &[0.0]),
            Err(AeroError::InvalidParameter { .. })
        ));
        assert!(matches!(
            panels.solve_about_cg(&ctx(12.0, 0.0, 100.0), Vector3::zeros(), &[0.0, 0.0]),
            Err(AeroError::InvalidParameter { .. })
        ));
        assert!(matches!(
            panels.solve_about_cg(&ctx(12.0, 0.0, 100.0), Vector3::zeros(), &[0.0, f64::NAN]),
            Err(AeroError::NonFinite { .. })
        ));
    }

    #[test]
    fn panel_skin_friction_coefficient_matches_reference_temperature_formulas() {
        let skin_friction = PanelSkinFriction::new(1.0, 1.0, 5.0e5, 0.0).unwrap();
        let laminar = skin_friction.skin_friction_coefficient(0.0, 1.0e5).unwrap();
        assert_relative_eq!(laminar, 1.328 / 1.0e5_f64.sqrt(), max_relative = 1e-12);

        let mach: f64 = 2.0;
        let reynolds_x: f64 = 1.0e7;
        let turbulent = skin_friction
            .skin_friction_coefficient(mach, reynolds_x)
            .unwrap();
        let t_star_over_t_edge = 1.0 + 0.032 * mach * mach;
        let reynolds_star = reynolds_x / t_star_over_t_edge.powf(1.5);
        let expected =
            0.455 / (reynolds_star.log10().powf(2.58) * (1.0 + 0.144 * mach * mach).powf(0.65));
        assert_relative_eq!(turbulent, expected, max_relative = 1e-12);
    }

    #[test]
    fn local_inclination_skin_friction_adds_tangential_drag_when_enabled() {
        let mut context = ctx(0.0, 0.0, 100.0);
        context.reynolds_length = 1.0e6;
        let mesh = flow_aligned_plate_mesh();
        let inviscid = LocalInclinationPanels::modified_newtonian(mesh.clone(), 2.0)
            .solve(&context)
            .unwrap();
        assert_relative_eq!(inviscid.force_n_body.norm(), 0.0, epsilon = 1e-12);

        let skin_friction = PanelSkinFriction::new(1.0, 1.0, 2.0e6, 0.0).unwrap();
        let solution = LocalInclinationPanels::modified_newtonian(mesh, 2.0)
            .with_skin_friction(skin_friction)
            .solve(&context)
            .unwrap();
        let upstream_triangle_re = context.reynolds_length * (2.0 / 3.0);
        let downstream_triangle_re = context.reynolds_length * (1.0 / 3.0);
        let expected_drag = -context.dynamic_pressure_pa
            * 0.5
            * (skin_friction
                .skin_friction_coefficient(context.mach, upstream_triangle_re)
                .unwrap()
                + skin_friction
                    .skin_friction_coefficient(context.mach, downstream_triangle_re)
                    .unwrap());
        assert_relative_eq!(solution.force_n_body.x, expected_drag, max_relative = 1e-12);
        assert_relative_eq!(solution.force_n_body.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(solution.force_n_body.z, 0.0, epsilon = 1e-12);
    }

    #[test]
    fn local_inclination_skin_friction_rejects_invalid_inputs() {
        assert!(matches!(
            PanelSkinFriction::new(-1.0, 1.0, 5.0e5, 0.0),
            Err(AeroError::InvalidParameter { .. }),
        ));

        let skin_friction = PanelSkinFriction::smooth_reference_temperature(1.0).unwrap();
        let panels = LocalInclinationPanels::modified_newtonian(flow_aligned_plate_mesh(), 2.0)
            .with_skin_friction(skin_friction);
        let mut context = ctx(0.0, 0.0, 100.0);
        context.reynolds_length = 0.0;
        assert!(matches!(
            panels.solve(&context),
            Err(AeroError::InvalidParameter { .. }),
        ));
    }

    #[test]
    fn local_inclination_bvh_shadowing_skips_occluded_nested_shell() {
        let panels = LocalInclinationPanels::modified_newtonian(nested_shadow_mesh(), 2.0);
        panels
            .geometry
            .validate_watertight_manifold()
            .expect("nested shells are watertight two-manifolds");
        let sign_only = panels.solve(&ctx(12.0, 0.0, 100.0)).unwrap();
        let bvh_shadowed = panels
            .solve_with_bvh_shadowing(&ctx(12.0, 0.0, 100.0))
            .unwrap();

        assert_relative_eq!(sign_only.force_n_body.x, -3400.0, epsilon = 1e-12);
        assert_relative_eq!(bvh_shadowed.force_n_body.x, -3200.0, epsilon = 1e-12);
        assert!(bvh_shadowed.force_n_body.x > sign_only.force_n_body.x);
    }

    #[test]
    fn local_inclination_bvh_shadowing_rejects_open_meshes() {
        let panels = LocalInclinationPanels::modified_newtonian(square_plate_mesh(), 2.0);
        assert!(matches!(
            panels.solve_with_bvh_shadowing(&ctx(12.0, 0.0, 100.0)),
            Err(AeroError::MalformedDeck { .. })
        ));
    }

    #[cfg(feature = "parser")]
    #[test]
    fn local_inclination_wedge_cavity_bvh_occlusion_set_matches_fixture() {
        let fixture: PanelWedgeCavityShadowFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/panel-wedge-cavity-shadow-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(case.dataset_id, "openbmp.aero.panel_wedge_cavity_shadow.v1");
        assert_eq!(case.source_class, "synthetic-openbmp");
        assert_eq!(case.validation_label, "checked");

        let panels =
            LocalInclinationPanels::modified_newtonian(wedge_cavity_shadow_mesh(), case.cp_max);
        panels
            .geometry
            .validate_watertight_manifold()
            .expect("synthetic wedge-cavity mesh is watertight");
        let context = ctx_with_beta(
            case.mach,
            case.alpha_deg,
            case.beta_deg,
            case.dynamic_pressure_pa,
        );

        assert_eq!(
            bvh_occluded_facing_triangle_indices(&panels, &context),
            case.expected_occluded_triangle_indices
        );
        assert_relative_eq!(
            panels.solve(&context).unwrap().force_n_body.x,
            case.expected_sign_only_force_x_n,
            epsilon = 1e-12
        );
        assert_relative_eq!(
            panels
                .solve_with_bvh_shadowing(&context)
                .unwrap()
                .force_n_body
                .x,
            case.expected_bvh_shadowed_force_x_n,
            epsilon = 1e-12
        );
    }

    #[cfg(feature = "parser")]
    #[test]
    fn local_inclination_sphere_cd_matches_closed_form_tolerance_table() {
        let fixture: PanelSphereToleranceFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/panel-sphere-tolerance-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(case.dataset_id, "openbmp.aero.panel_sphere_tolerance.v1");
        assert_eq!(case.source_class, "synthetic-openbmp");
        assert_eq!(case.validation_label, "checked");

        let mesh = latitude_longitude_sphere_mesh(
            case.radius_m,
            case.latitude_segments,
            case.longitude_segments,
        );
        mesh.validate_watertight_manifold()
            .expect("generated sphere mesh is watertight");
        let panels = LocalInclinationPanels::modified_newtonian(mesh, case.cp_max);
        let solution = panels
            .solve(&ctx_with_beta(
                case.mach,
                case.alpha_deg,
                case.beta_deg,
                case.dynamic_pressure_pa,
            ))
            .unwrap();
        let reference_area_m2 = PI * case.radius_m * case.radius_m;
        let cd = -solution.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
        let relative_error = ((cd - case.expected_cd) / case.expected_cd).abs();

        assert_relative_eq!(case.expected_cd, case.cp_max * 0.5, max_relative = 1e-15);
        assert!(
            relative_error <= case.max_relative_error,
            "sphere CD {cd} differs from closed-form {} by {relative_error}",
            case.expected_cd
        );
    }

    #[cfg(feature = "parser")]
    #[test]
    fn local_inclination_sharp_cone_ca_matches_tangent_cone_tolerance_table() {
        let fixture: PanelSharpConeToleranceFixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/panel-sharp-cone-tolerance-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(
            case.dataset_id,
            "openbmp.aero.panel_sharp_cone_tolerance.v1"
        );
        assert_eq!(case.source_class, "synthetic-openbmp");
        assert_eq!(case.validation_label, "checked");

        let half_angle_rad = case.half_angle_deg.to_radians();
        assert_relative_eq!(
            case.cp_max,
            ModifiedNewtonian::cp_max_perfect_gas(case.mach, case.gamma),
            max_relative = 1e-15
        );
        assert_relative_eq!(
            case.expected_ca,
            TangentCone::cp_cone(case.mach, half_angle_rad, case.gamma),
            max_relative = 1e-15
        );

        let panels = LocalInclinationPanels::modified_newtonian(
            sharp_cone_side_mesh(case.base_radius_m, half_angle_rad, case.longitude_segments),
            case.cp_max,
        );
        let solution = panels
            .solve(&ctx(case.mach, 0.0, case.dynamic_pressure_pa))
            .unwrap();
        let reference_area_m2 = PI * case.base_radius_m * case.base_radius_m;
        let ca = -solution.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
        let relative_error = ((ca - case.expected_ca) / case.expected_ca).abs();

        assert!(
            relative_error <= case.max_relative_error,
            "sharp-cone CA {ca} differs from tangent-cone {} by {relative_error}",
            case.expected_ca
        );
    }

    #[cfg(feature = "parser")]
    #[test]
    fn local_inclination_hb2_forebody_ca_matches_figure12_high_mach_fixture() {
        let fixture: PanelHb2ForebodyFigure12Fixture = toml::from_str(include_str!(concat!(
            env!("CARGO_MANIFEST_DIR"),
            "/../../data/aero/panel-hb2-forebody-figure12-v1.toml"
        )))
        .unwrap();
        let case = fixture.case;
        assert_eq!(
            case.dataset_id,
            "openbmp.aero.panel_hb2_forebody_figure12.v1"
        );
        assert_eq!(case.source_class, "public-domain-us-gov");
        assert_eq!(case.validation_label, "research");
        assert_eq!(case.source_id, "AEDC-TDR-64-137 / AD602769");
        assert_eq!(case.figure_id, "Figure 12b");
        assert_relative_eq!(case.body_length_over_diameter, 4.5, epsilon = 0.0);
        assert!(!case.samples.is_empty());

        let mesh = hb2_forebody_side_mesh(
            case.reference_diameter_m,
            case.station_segments,
            case.longitude_segments,
        );
        let reference_area_m2 =
            PI * (0.5 * case.reference_diameter_m) * (0.5 * case.reference_diameter_m);
        for sample in case.samples {
            assert!(
                sample.digitized_note.contains("Figure 12b"),
                "sample must carry the plot-digitization note"
            );
            let cp_max = ModifiedNewtonian::cp_max_perfect_gas(sample.mach, case.gamma);
            let panels = LocalInclinationPanels::modified_newtonian(mesh.clone(), cp_max);
            let mut context = ctx(sample.mach, 0.0, case.dynamic_pressure_pa);
            context.reynolds_length = sample.reynolds_diameter;
            let solution = panels.solve(&context).unwrap();
            let ca = -solution.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
            let relative_error =
                ((ca - sample.expected_forebody_ca) / sample.expected_forebody_ca).abs();

            assert!(
                relative_error <= case.max_relative_error,
                "HB-2 forebody CA {ca} at Mach {} differs from digitized {} by {relative_error}",
                sample.mach,
                sample.expected_forebody_ca
            );
        }
    }

    #[cfg(feature = "parser")]
    #[test]
    fn local_inclination_hb2_forebody_skin_friction_matches_figure12_high_mach_fixture() {
        let fixture: PanelHb2ForebodySkinFrictionFigure12Fixture =
            toml::from_str(include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../data/aero/panel-hb2-forebody-skin-friction-figure12-v1.toml"
            )))
            .unwrap();
        let case = fixture.case;
        assert_eq!(
            case.dataset_id,
            "openbmp.aero.panel_hb2_forebody_skin_friction_figure12.v1"
        );
        assert_eq!(case.source_class, "public-domain-us-gov");
        assert_eq!(case.validation_label, "research");
        assert_eq!(case.source_id, "AEDC-TDR-64-137 / AD602769");
        assert_eq!(case.figure_id, "Figure 12b");
        assert_relative_eq!(case.body_length_over_diameter, 4.5, epsilon = 0.0);
        assert!(!case.samples.is_empty());

        let mesh = hb2_forebody_side_mesh(
            case.reference_diameter_m,
            case.station_segments,
            case.longitude_segments,
        );
        let skin_friction = PanelSkinFriction::new(
            case.reference_diameter_m,
            case.wall_to_edge_temperature_ratio,
            case.transition_reynolds,
            case.roughness_over_length,
        )
        .unwrap();
        let reference_area_m2 =
            PI * (0.5 * case.reference_diameter_m) * (0.5 * case.reference_diameter_m);
        for sample in case.samples {
            assert!(
                sample.digitized_note.contains("Figure 12b"),
                "sample must carry the plot-digitization note"
            );
            let cp_max = ModifiedNewtonian::cp_max_perfect_gas(sample.mach, case.gamma);
            let inviscid_panels = LocalInclinationPanels::modified_newtonian(mesh.clone(), cp_max);
            let viscous_panels = inviscid_panels.clone().with_skin_friction(skin_friction);
            let mut context = ctx(sample.mach, 0.0, case.dynamic_pressure_pa);
            context.reynolds_length = sample.reynolds_diameter;

            let inviscid = inviscid_panels.solve(&context).unwrap();
            let inviscid_ca =
                -inviscid.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
            let viscous = viscous_panels.solve(&context).unwrap();
            let viscous_ca =
                -viscous.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
            let relative_error = ((viscous_ca - sample.expected_total_forebody_ca)
                / sample.expected_total_forebody_ca)
                .abs();

            assert!(
                viscous_ca > inviscid_ca,
                "HB-2 skin friction should add positive axial CA at Mach {}",
                sample.mach
            );
            assert!(
                relative_error <= case.max_relative_error,
                "HB-2 skin-friction CA {viscous_ca} at Mach {} differs from digitized {} by {relative_error}",
                sample.mach,
                sample.expected_total_forebody_ca
            );
        }
    }

    #[cfg(feature = "parser")]
    #[test]
    fn local_inclination_sharp_cone_skin_friction_matches_trend_fixture() {
        let fixture: PanelSharpConeSkinFrictionTrendFixture =
            toml::from_str(include_str!(concat!(
                env!("CARGO_MANIFEST_DIR"),
                "/../../data/aero/panel-sharp-cone-skin-friction-trend-v1.toml"
            )))
            .unwrap();
        let case = fixture.case;
        assert_eq!(
            case.dataset_id,
            "openbmp.aero.panel_sharp_cone_skin_friction_trend.v1"
        );
        assert_eq!(case.source_class, "synthetic-openbmp");
        assert_eq!(case.validation_label, "validated-toy");

        let half_angle_rad = case.half_angle_deg.to_radians();
        let reference_length_m = case.base_radius_m / half_angle_rad.tan();
        let cp_max = ModifiedNewtonian::cp_max_perfect_gas(case.mach, case.gamma);
        let reference_area_m2 = PI * case.base_radius_m * case.base_radius_m;
        let panels = LocalInclinationPanels::modified_newtonian(
            sharp_cone_side_mesh(case.base_radius_m, half_angle_rad, case.longitude_segments),
            cp_max,
        );
        let skin_friction = PanelSkinFriction::new(
            reference_length_m,
            case.wall_to_edge_temperature_ratio,
            case.transition_reynolds,
            case.roughness_over_length,
        )
        .unwrap();
        let low_re_panels = panels.clone().with_skin_friction(skin_friction);
        let high_re_panels = panels.clone().with_skin_friction(skin_friction);

        let mut inviscid_context = ctx(case.mach, 0.0, case.dynamic_pressure_pa);
        inviscid_context.reynolds_length = case.high_reynolds_length;
        let inviscid = panels.solve(&inviscid_context).unwrap();
        let inviscid_ca = -inviscid.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);

        let mut low_re_context = inviscid_context;
        low_re_context.reynolds_length = case.low_reynolds_length;
        let low_re = low_re_panels.solve(&low_re_context).unwrap();
        let low_re_ca = -low_re.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
        let low_re_delta_ca = low_re_ca - inviscid_ca;

        let high_re = high_re_panels.solve(&inviscid_context).unwrap();
        let high_re_ca = -high_re.force_n_body.x / (case.dynamic_pressure_pa * reference_area_m2);
        let high_re_delta_ca = high_re_ca - inviscid_ca;

        assert_relative_eq!(
            inviscid_ca,
            case.expected_inviscid_ca,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            low_re_delta_ca,
            case.expected_low_re_viscous_delta_ca,
            max_relative = case.max_relative_error
        );
        assert_relative_eq!(
            high_re_delta_ca,
            case.expected_high_re_viscous_delta_ca,
            max_relative = case.max_relative_error
        );
        assert!(low_re_delta_ca > high_re_delta_ca);
        assert!(high_re_delta_ca > 0.0);
        assert!(
            high_re_delta_ca / inviscid_ca <= case.max_high_re_delta_fraction_of_inviscid,
            "high-Re smooth-wall correction {high_re_delta_ca} exceeds fixture fraction"
        );
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
    fn local_inclination_shadowing_false_makes_open_panels_two_sided() {
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
        let mut panels = LocalInclinationPanels::modified_newtonian(mesh, 2.0);
        panels.shadowing = false;
        let fmt = panels
            .aero_force_moment_body(&ctx(12.0, 0.0, 100.0))
            .unwrap();
        assert_relative_eq!(fmt.force_n_body.x, -200.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.force_n_body.y, 0.0, epsilon = 1e-12);
        assert_relative_eq!(fmt.force_n_body.z, 0.0, epsilon = 1e-12);
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
    fn panel_mesh_watertight_validation_rejects_open_edges() {
        let mesh = square_plate_mesh();

        assert!(matches!(
            mesh.validate_watertight_manifold(),
            Err(AeroError::MalformedDeck { .. })
        ));
        assert!(matches!(
            PanelMeshBvh::from_watertight_mesh(mesh),
            Err(AeroError::MalformedDeck { .. })
        ));
    }

    #[test]
    fn panel_mesh_bvh_ray_query_is_deterministic_for_closed_mesh() {
        let mesh = closed_box_mesh();
        mesh.validate_watertight_manifold()
            .expect("closed box is watertight");
        let bvh = PanelMeshBvh::from_watertight_mesh(mesh.clone()).expect("BVH builds");
        let repeat = PanelMeshBvh::from_watertight_mesh(mesh).expect("repeat BVH builds");

        assert_eq!(bvh, repeat);
        assert!(
            bvh.ray_intersects(Vector3::zeros(), Vector3::new(1.0, 0.0, 0.0))
                .expect("ray validates")
        );
        assert!(
            !bvh.ray_intersects(Vector3::zeros(), Vector3::new(1.0, 2.0, 0.0))
                .expect("ray validates")
        );
        assert!(
            !bvh.ray_intersects(Vector3::zeros(), Vector3::new(0.0, 1.0, 0.0))
                .expect("ray validates")
        );
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
