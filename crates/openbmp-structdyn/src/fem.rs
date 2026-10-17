//! Pure-Rust structural finite-element helpers.
//!
//! The shipped FEM substrate includes a two-node Euler-Bernoulli bending beam,
//! a three-node constant-strain plane-stress membrane triangle, a four-node
//! bilinear plane-stress membrane quadrilateral, a three-node
//! Reissner-Mindlin plate-bending triangle, and combined six-DOF triangular and
//! quadrilateral shells with drilling-rotation regularization. They assemble
//! deterministic row-major mass and stiffness matrices that can feed the
//! crate's CB and modal transient helpers.

use std::collections::BTreeSet;

use crate::{DenseMatrix, StructDynError};

/// Dimensionless clamped-free beam beta*L constants from the classical
/// Euler-Bernoulli characteristic equation.
pub const CANTILEVER_BETA_L: [f64; 5] = [
    1.875_104_068_711_961,
    4.694_091_132_974_174,
    7.854_757_438_237_612,
    10.995_540_734_875_466,
    14.137_168_391_046_47,
];

/// Default Reissner-Mindlin transverse shear correction factor for rectangular
/// through-thickness stress distribution.
pub const MINDLIN_DEFAULT_SHEAR_CORRECTION: f64 = 5.0 / 6.0;

/// Default nondimensional drilling-rotation stiffness factor for the combined
/// triangular shell element. This is a small numerical regularization, not a
/// calibrated physical drilling stiffness.
pub const SHELL_DEFAULT_DRILLING_STIFFNESS_FACTOR: f64 = 1.0e-6;

/// One two-node Euler-Bernoulli beam element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct EulerBernoulliBeamElement {
    node_a: usize,
    node_b: usize,
    length_m: f64,
    flexural_rigidity_n_m2: f64,
    mass_per_length_kg_m: f64,
}

/// One structural membrane node in a local planar shell coordinate system.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct MembraneNode {
    x_m: f64,
    y_m: f64,
}

impl MembraneNode {
    /// Construct a validated 2D membrane node.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when either coordinate is non-finite.
    pub fn new(x_m: f64, y_m: f64) -> Result<Self, StructDynError> {
        if !x_m.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "membrane.node.x_m",
            });
        }
        if !y_m.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "membrane.node.y_m",
            });
        }
        Ok(Self { x_m, y_m })
    }

    /// Local x coordinate, m.
    #[must_use]
    pub const fn x_m(&self) -> f64 {
        self.x_m
    }

    /// Local y coordinate, m.
    #[must_use]
    pub const fn y_m(&self) -> f64 {
        self.y_m
    }
}

/// One three-node constant-strain triangle membrane element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ConstantStrainTriangleMembraneElement {
    node_a: usize,
    node_b: usize,
    node_c: usize,
    thickness_m: f64,
    young_modulus_pa: f64,
    poisson_ratio: f64,
    density_kg_m3: f64,
}

/// One four-node bilinear quadrilateral membrane element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BilinearQuadrilateralMembraneElement {
    node_a: usize,
    node_b: usize,
    node_c: usize,
    node_d: usize,
    thickness_m: f64,
    young_modulus_pa: f64,
    poisson_ratio: f64,
    density_kg_m3: f64,
}

/// One three-node shear-deformable Reissner-Mindlin plate-bending element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ReissnerMindlinTrianglePlateElement {
    node_a: usize,
    node_b: usize,
    node_c: usize,
    thickness_m: f64,
    young_modulus_pa: f64,
    poisson_ratio: f64,
    density_kg_m3: f64,
    shear_correction_factor: f64,
}

/// One three-node six-DOF triangular shell element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct TriangularShellElement {
    node_a: usize,
    node_b: usize,
    node_c: usize,
    thickness_m: f64,
    young_modulus_pa: f64,
    poisson_ratio: f64,
    density_kg_m3: f64,
    drilling_stiffness_factor: f64,
}

/// One four-node six-DOF quadrilateral shell element.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct QuadrilateralShellElement {
    node_a: usize,
    node_b: usize,
    node_c: usize,
    node_d: usize,
    thickness_m: f64,
    young_modulus_pa: f64,
    poisson_ratio: f64,
    density_kg_m3: f64,
    drilling_stiffness_factor: f64,
}

impl QuadrilateralShellElement {
    /// Construct a validated combined quadrilateral membrane/plate shell.
    ///
    /// The element uses locked node DOF order
    /// `[u_x, u_y, w, theta_x, theta_y, theta_z]`, where `theta_z` is the
    /// drilling rotation regularized by
    /// [`SHELL_DEFAULT_DRILLING_STIFFNESS_FACTOR`]. The in-plane membrane
    /// block is the four-node bilinear quadrilateral; the plate block is the
    /// deterministic two-triangle split `(a,b,c)` and `(a,c,d)`.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when node ids repeat, thickness, Young's
    /// modulus, or density are not positive finite, or the Poisson ratio is
    /// outside `-1 < nu < 0.5`.
    pub fn new(
        node_ids: [usize; 4],
        thickness_m: f64,
        young_modulus_pa: f64,
        poisson_ratio: f64,
        density_kg_m3: f64,
    ) -> Result<Self, StructDynError> {
        let [node_a, node_b, node_c, node_d] = node_ids;
        if node_a == node_b
            || node_a == node_c
            || node_a == node_d
            || node_b == node_c
            || node_b == node_d
            || node_c == node_d
        {
            return Err(StructDynError::InvalidMatrix {
                field: "quad_shell.element",
                reason: "element nodes must be distinct",
            });
        }
        require_positive("quad_shell.thickness_m", thickness_m)?;
        require_positive("quad_shell.young_modulus_pa", young_modulus_pa)?;
        require_positive("quad_shell.density_kg_m3", density_kg_m3)?;
        if !poisson_ratio.is_finite() || poisson_ratio <= -1.0 || poisson_ratio >= 0.5 {
            return Err(StructDynError::InvalidNumber {
                field: "quad_shell.poisson_ratio",
            });
        }
        Ok(Self {
            node_a,
            node_b,
            node_c,
            node_d,
            thickness_m,
            young_modulus_pa,
            poisson_ratio,
            density_kg_m3,
            drilling_stiffness_factor: SHELL_DEFAULT_DRILLING_STIFFNESS_FACTOR,
        })
    }

    /// First node index.
    #[must_use]
    pub const fn node_a(&self) -> usize {
        self.node_a
    }

    /// Second node index.
    #[must_use]
    pub const fn node_b(&self) -> usize {
        self.node_b
    }

    /// Third node index.
    #[must_use]
    pub const fn node_c(&self) -> usize {
        self.node_c
    }

    /// Fourth node index.
    #[must_use]
    pub const fn node_d(&self) -> usize {
        self.node_d
    }

    /// Element thickness, m.
    #[must_use]
    pub const fn thickness_m(&self) -> f64 {
        self.thickness_m
    }

    /// Young's modulus, Pa.
    #[must_use]
    pub const fn young_modulus_pa(&self) -> f64 {
        self.young_modulus_pa
    }

    /// Poisson ratio.
    #[must_use]
    pub const fn poisson_ratio(&self) -> f64 {
        self.poisson_ratio
    }

    /// Material density, kg/m^3.
    #[must_use]
    pub const fn density_kg_m3(&self) -> f64 {
        self.density_kg_m3
    }

    /// Drilling stiffness regularization factor.
    #[must_use]
    pub const fn drilling_stiffness_factor(&self) -> f64 {
        self.drilling_stiffness_factor
    }
}

impl TriangularShellElement {
    /// Construct a validated combined membrane/plate shell element.
    ///
    /// The element uses locked node DOF order
    /// `[u_x, u_y, w, theta_x, theta_y, theta_z]`, where `theta_z` is the
    /// drilling rotation regularized by
    /// [`SHELL_DEFAULT_DRILLING_STIFFNESS_FACTOR`].
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when node ids repeat, thickness, Young's
    /// modulus, or density are not positive finite, or the Poisson ratio is
    /// outside `-1 < nu < 0.5`.
    pub fn new(
        node_a: usize,
        node_b: usize,
        node_c: usize,
        thickness_m: f64,
        young_modulus_pa: f64,
        poisson_ratio: f64,
        density_kg_m3: f64,
    ) -> Result<Self, StructDynError> {
        if node_a == node_b || node_a == node_c || node_b == node_c {
            return Err(StructDynError::InvalidMatrix {
                field: "shell.element",
                reason: "element nodes must be distinct",
            });
        }
        require_positive("shell.thickness_m", thickness_m)?;
        require_positive("shell.young_modulus_pa", young_modulus_pa)?;
        require_positive("shell.density_kg_m3", density_kg_m3)?;
        if !poisson_ratio.is_finite() || poisson_ratio <= -1.0 || poisson_ratio >= 0.5 {
            return Err(StructDynError::InvalidNumber {
                field: "shell.poisson_ratio",
            });
        }
        Ok(Self {
            node_a,
            node_b,
            node_c,
            thickness_m,
            young_modulus_pa,
            poisson_ratio,
            density_kg_m3,
            drilling_stiffness_factor: SHELL_DEFAULT_DRILLING_STIFFNESS_FACTOR,
        })
    }

    /// First node index.
    #[must_use]
    pub const fn node_a(&self) -> usize {
        self.node_a
    }

    /// Second node index.
    #[must_use]
    pub const fn node_b(&self) -> usize {
        self.node_b
    }

    /// Third node index.
    #[must_use]
    pub const fn node_c(&self) -> usize {
        self.node_c
    }

    /// Element thickness, m.
    #[must_use]
    pub const fn thickness_m(&self) -> f64 {
        self.thickness_m
    }

    /// Young's modulus, Pa.
    #[must_use]
    pub const fn young_modulus_pa(&self) -> f64 {
        self.young_modulus_pa
    }

    /// Poisson ratio.
    #[must_use]
    pub const fn poisson_ratio(&self) -> f64 {
        self.poisson_ratio
    }

    /// Material density, kg/m^3.
    #[must_use]
    pub const fn density_kg_m3(&self) -> f64 {
        self.density_kg_m3
    }

    /// Drilling stiffness regularization factor.
    #[must_use]
    pub const fn drilling_stiffness_factor(&self) -> f64 {
        self.drilling_stiffness_factor
    }
}

impl ReissnerMindlinTrianglePlateElement {
    /// Construct a validated three-node plate-bending element.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when node ids repeat, thickness, Young's
    /// modulus, or density are not positive finite, or the Poisson ratio is
    /// outside `-1 < nu < 0.5`.
    pub fn new(
        node_a: usize,
        node_b: usize,
        node_c: usize,
        thickness_m: f64,
        young_modulus_pa: f64,
        poisson_ratio: f64,
        density_kg_m3: f64,
    ) -> Result<Self, StructDynError> {
        if node_a == node_b || node_a == node_c || node_b == node_c {
            return Err(StructDynError::InvalidMatrix {
                field: "plate.element",
                reason: "element nodes must be distinct",
            });
        }
        require_positive("plate.thickness_m", thickness_m)?;
        require_positive("plate.young_modulus_pa", young_modulus_pa)?;
        require_positive("plate.density_kg_m3", density_kg_m3)?;
        if !poisson_ratio.is_finite() || poisson_ratio <= -1.0 || poisson_ratio >= 0.5 {
            return Err(StructDynError::InvalidNumber {
                field: "plate.poisson_ratio",
            });
        }
        Ok(Self {
            node_a,
            node_b,
            node_c,
            thickness_m,
            young_modulus_pa,
            poisson_ratio,
            density_kg_m3,
            shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
        })
    }

    /// First node index.
    #[must_use]
    pub const fn node_a(&self) -> usize {
        self.node_a
    }

    /// Second node index.
    #[must_use]
    pub const fn node_b(&self) -> usize {
        self.node_b
    }

    /// Third node index.
    #[must_use]
    pub const fn node_c(&self) -> usize {
        self.node_c
    }

    /// Element thickness, m.
    #[must_use]
    pub const fn thickness_m(&self) -> f64 {
        self.thickness_m
    }

    /// Young's modulus, Pa.
    #[must_use]
    pub const fn young_modulus_pa(&self) -> f64 {
        self.young_modulus_pa
    }

    /// Poisson ratio.
    #[must_use]
    pub const fn poisson_ratio(&self) -> f64 {
        self.poisson_ratio
    }

    /// Material density, kg/m^3.
    #[must_use]
    pub const fn density_kg_m3(&self) -> f64 {
        self.density_kg_m3
    }

    /// Shear correction factor.
    #[must_use]
    pub const fn shear_correction_factor(&self) -> f64 {
        self.shear_correction_factor
    }
}

impl ConstantStrainTriangleMembraneElement {
    /// Construct a validated plane-stress CST membrane element.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when node ids repeat, thickness, Young's
    /// modulus, or density are not positive finite, or the Poisson ratio is
    /// outside `-1 < nu < 0.5`.
    pub fn new(
        node_a: usize,
        node_b: usize,
        node_c: usize,
        thickness_m: f64,
        young_modulus_pa: f64,
        poisson_ratio: f64,
        density_kg_m3: f64,
    ) -> Result<Self, StructDynError> {
        if node_a == node_b || node_a == node_c || node_b == node_c {
            return Err(StructDynError::InvalidMatrix {
                field: "membrane.element",
                reason: "element nodes must be distinct",
            });
        }
        require_positive("membrane.thickness_m", thickness_m)?;
        require_positive("membrane.young_modulus_pa", young_modulus_pa)?;
        require_positive("membrane.density_kg_m3", density_kg_m3)?;
        if !poisson_ratio.is_finite() || poisson_ratio <= -1.0 || poisson_ratio >= 0.5 {
            return Err(StructDynError::InvalidNumber {
                field: "membrane.poisson_ratio",
            });
        }
        Ok(Self {
            node_a,
            node_b,
            node_c,
            thickness_m,
            young_modulus_pa,
            poisson_ratio,
            density_kg_m3,
        })
    }

    /// First node index.
    #[must_use]
    pub const fn node_a(&self) -> usize {
        self.node_a
    }

    /// Second node index.
    #[must_use]
    pub const fn node_b(&self) -> usize {
        self.node_b
    }

    /// Third node index.
    #[must_use]
    pub const fn node_c(&self) -> usize {
        self.node_c
    }

    /// Element thickness, m.
    #[must_use]
    pub const fn thickness_m(&self) -> f64 {
        self.thickness_m
    }

    /// Young's modulus, Pa.
    #[must_use]
    pub const fn young_modulus_pa(&self) -> f64 {
        self.young_modulus_pa
    }

    /// Plane-stress Poisson ratio.
    #[must_use]
    pub const fn poisson_ratio(&self) -> f64 {
        self.poisson_ratio
    }

    /// Material density, kg/m^3.
    #[must_use]
    pub const fn density_kg_m3(&self) -> f64 {
        self.density_kg_m3
    }
}

impl BilinearQuadrilateralMembraneElement {
    /// Construct a validated plane-stress bilinear quadrilateral membrane.
    ///
    /// Node order is declaration order around the perimeter. The assembler
    /// checks the isoparametric Jacobian at every Gauss point and rejects
    /// folded or degenerate elements.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when node ids repeat, thickness, Young's
    /// modulus, or density are not positive finite, or the Poisson ratio is
    /// outside `-1 < nu < 0.5`.
    pub fn new(
        node_ids: [usize; 4],
        thickness_m: f64,
        young_modulus_pa: f64,
        poisson_ratio: f64,
        density_kg_m3: f64,
    ) -> Result<Self, StructDynError> {
        let [node_a, node_b, node_c, node_d] = node_ids;
        if node_a == node_b
            || node_a == node_c
            || node_a == node_d
            || node_b == node_c
            || node_b == node_d
            || node_c == node_d
        {
            return Err(StructDynError::InvalidMatrix {
                field: "quad_membrane.element",
                reason: "element nodes must be distinct",
            });
        }
        require_positive("quad_membrane.thickness_m", thickness_m)?;
        require_positive("quad_membrane.young_modulus_pa", young_modulus_pa)?;
        require_positive("quad_membrane.density_kg_m3", density_kg_m3)?;
        if !poisson_ratio.is_finite() || poisson_ratio <= -1.0 || poisson_ratio >= 0.5 {
            return Err(StructDynError::InvalidNumber {
                field: "quad_membrane.poisson_ratio",
            });
        }
        Ok(Self {
            node_a,
            node_b,
            node_c,
            node_d,
            thickness_m,
            young_modulus_pa,
            poisson_ratio,
            density_kg_m3,
        })
    }

    /// First node index.
    #[must_use]
    pub const fn node_a(&self) -> usize {
        self.node_a
    }

    /// Second node index.
    #[must_use]
    pub const fn node_b(&self) -> usize {
        self.node_b
    }

    /// Third node index.
    #[must_use]
    pub const fn node_c(&self) -> usize {
        self.node_c
    }

    /// Fourth node index.
    #[must_use]
    pub const fn node_d(&self) -> usize {
        self.node_d
    }

    /// Element thickness, m.
    #[must_use]
    pub const fn thickness_m(&self) -> f64 {
        self.thickness_m
    }

    /// Young's modulus, Pa.
    #[must_use]
    pub const fn young_modulus_pa(&self) -> f64 {
        self.young_modulus_pa
    }

    /// Plane-stress Poisson ratio.
    #[must_use]
    pub const fn poisson_ratio(&self) -> f64 {
        self.poisson_ratio
    }

    /// Material density, kg/m^3.
    #[must_use]
    pub const fn density_kg_m3(&self) -> f64 {
        self.density_kg_m3
    }
}

impl EulerBernoulliBeamElement {
    /// Construct a validated two-node bending beam element.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the nodes are identical or length,
    /// flexural rigidity, or mass per length is not positive finite.
    pub fn new(
        node_a: usize,
        node_b: usize,
        length_m: f64,
        flexural_rigidity_n_m2: f64,
        mass_per_length_kg_m: f64,
    ) -> Result<Self, StructDynError> {
        if node_a == node_b {
            return Err(StructDynError::InvalidMatrix {
                field: "beam.element",
                reason: "element nodes must differ",
            });
        }
        require_positive("beam.length_m", length_m)?;
        require_positive("beam.flexural_rigidity_n_m2", flexural_rigidity_n_m2)?;
        require_positive("beam.mass_per_length_kg_m", mass_per_length_kg_m)?;
        Ok(Self {
            node_a,
            node_b,
            length_m,
            flexural_rigidity_n_m2,
            mass_per_length_kg_m,
        })
    }

    /// First node index.
    #[must_use]
    pub const fn node_a(&self) -> usize {
        self.node_a
    }

    /// Second node index.
    #[must_use]
    pub const fn node_b(&self) -> usize {
        self.node_b
    }

    /// Element length, m.
    #[must_use]
    pub const fn length_m(&self) -> f64 {
        self.length_m
    }

    /// Bending stiffness `EI`, N*m^2.
    #[must_use]
    pub const fn flexural_rigidity_n_m2(&self) -> f64 {
        self.flexural_rigidity_n_m2
    }

    /// Mass per unit length, kg/m.
    #[must_use]
    pub const fn mass_per_length_kg_m(&self) -> f64 {
        self.mass_per_length_kg_m
    }
}

/// Assembled beam FEM matrices in locked node/DOF order.
#[derive(Clone, Debug, PartialEq)]
pub struct BeamAssembly {
    node_count: usize,
    mass: DenseMatrix,
    stiffness: DenseMatrix,
}

/// Assembled membrane FEM matrices in locked node/DOF order.
#[derive(Clone, Debug, PartialEq)]
pub struct MembraneAssembly {
    node_count: usize,
    mass: DenseMatrix,
    stiffness: DenseMatrix,
}

/// Assembled plate-bending FEM matrices in locked node/DOF order.
#[derive(Clone, Debug, PartialEq)]
pub struct PlateBendingAssembly {
    node_count: usize,
    mass: DenseMatrix,
    stiffness: DenseMatrix,
}

/// Assembled six-DOF shell FEM matrices in locked node/DOF order.
#[derive(Clone, Debug, PartialEq)]
pub struct ShellAssembly {
    node_count: usize,
    mass: DenseMatrix,
    stiffness: DenseMatrix,
}

impl ShellAssembly {
    /// Number of nodes in the assembled model.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        self.node_count
    }

    /// Number of shell DOFs.
    #[must_use]
    pub const fn dof_count(&self) -> usize {
        self.node_count * 6
    }

    /// Combined translational/rotary mass matrix.
    #[must_use]
    pub const fn mass(&self) -> &DenseMatrix {
        &self.mass
    }

    /// Combined membrane/plate/drilling stiffness matrix.
    #[must_use]
    pub const fn stiffness(&self) -> &DenseMatrix {
        &self.stiffness
    }

    /// Extract free-DOF mass and stiffness matrices after clamping fixed DOFs.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when fixed DOFs are duplicated, out of range,
    /// or remove every DOF.
    pub fn free_dof_matrices(
        &self,
        fixed_dofs: &[usize],
    ) -> Result<FreeDofMatrices, StructDynError> {
        let fixed_dofs = validate_fixed_dofs("shell.fixed_dofs", fixed_dofs, self.dof_count())?;
        let free_dofs = (0..self.dof_count())
            .filter(|candidate| !fixed_dofs.contains(candidate))
            .collect::<Vec<_>>();
        if free_dofs.is_empty() {
            return Err(StructDynError::InvalidMatrix {
                field: "shell.fixed_dofs",
                reason: "at least one free DOF is required",
            });
        }
        let mass = submatrix(&self.mass, &free_dofs)?;
        let stiffness = submatrix(&self.stiffness, &free_dofs)?;
        Ok(FreeDofMatrices {
            mass,
            stiffness,
            free_dofs,
        })
    }
}

impl PlateBendingAssembly {
    /// Number of nodes in the assembled model.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        self.node_count
    }

    /// Number of transverse displacement / rotation DOFs.
    #[must_use]
    pub const fn dof_count(&self) -> usize {
        self.node_count * 3
    }

    /// Consistent translational/rotary mass matrix.
    #[must_use]
    pub const fn mass(&self) -> &DenseMatrix {
        &self.mass
    }

    /// Reissner-Mindlin bending plus shear stiffness matrix.
    #[must_use]
    pub const fn stiffness(&self) -> &DenseMatrix {
        &self.stiffness
    }

    /// Extract free-DOF mass and stiffness matrices after clamping fixed DOFs.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when fixed DOFs are duplicated, out of range,
    /// or remove every DOF.
    pub fn free_dof_matrices(
        &self,
        fixed_dofs: &[usize],
    ) -> Result<FreeDofMatrices, StructDynError> {
        let fixed_dofs = validate_fixed_dofs("plate.fixed_dofs", fixed_dofs, self.dof_count())?;
        let free_dofs = (0..self.dof_count())
            .filter(|candidate| !fixed_dofs.contains(candidate))
            .collect::<Vec<_>>();
        if free_dofs.is_empty() {
            return Err(StructDynError::InvalidMatrix {
                field: "plate.fixed_dofs",
                reason: "at least one free DOF is required",
            });
        }
        let mass = submatrix(&self.mass, &free_dofs)?;
        let stiffness = submatrix(&self.stiffness, &free_dofs)?;
        Ok(FreeDofMatrices {
            mass,
            stiffness,
            free_dofs,
        })
    }
}

impl MembraneAssembly {
    /// Number of nodes in the assembled model.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        self.node_count
    }

    /// Number of in-plane translational DOFs.
    #[must_use]
    pub const fn dof_count(&self) -> usize {
        self.node_count * 2
    }

    /// Consistent translational mass matrix.
    #[must_use]
    pub const fn mass(&self) -> &DenseMatrix {
        &self.mass
    }

    /// Plane-stress membrane stiffness matrix.
    #[must_use]
    pub const fn stiffness(&self) -> &DenseMatrix {
        &self.stiffness
    }

    /// Extract free-DOF mass and stiffness matrices after clamping fixed DOFs.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when fixed DOFs are duplicated, out of range,
    /// or remove every DOF.
    pub fn free_dof_matrices(
        &self,
        fixed_dofs: &[usize],
    ) -> Result<FreeDofMatrices, StructDynError> {
        let fixed_dofs = validate_fixed_dofs("membrane.fixed_dofs", fixed_dofs, self.dof_count())?;
        let free_dofs = (0..self.dof_count())
            .filter(|candidate| !fixed_dofs.contains(candidate))
            .collect::<Vec<_>>();
        if free_dofs.is_empty() {
            return Err(StructDynError::InvalidMatrix {
                field: "membrane.fixed_dofs",
                reason: "at least one free DOF is required",
            });
        }
        let mass = submatrix(&self.mass, &free_dofs)?;
        let stiffness = submatrix(&self.stiffness, &free_dofs)?;
        Ok(FreeDofMatrices {
            mass,
            stiffness,
            free_dofs,
        })
    }
}

impl BeamAssembly {
    /// Number of nodes in the assembled model.
    #[must_use]
    pub const fn node_count(&self) -> usize {
        self.node_count
    }

    /// Number of displacement/rotation DOFs.
    #[must_use]
    pub const fn dof_count(&self) -> usize {
        self.node_count * 2
    }

    /// Consistent mass matrix.
    #[must_use]
    pub const fn mass(&self) -> &DenseMatrix {
        &self.mass
    }

    /// Bending stiffness matrix.
    #[must_use]
    pub const fn stiffness(&self) -> &DenseMatrix {
        &self.stiffness
    }

    /// Extract free-DOF mass and stiffness matrices after clamping fixed DOFs.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when fixed DOFs are duplicated, out of range,
    /// or remove every DOF.
    pub fn free_dof_matrices(
        &self,
        fixed_dofs: &[usize],
    ) -> Result<FreeDofMatrices, StructDynError> {
        let fixed_dofs = validate_fixed_dofs("beam.fixed_dofs", fixed_dofs, self.dof_count())?;
        let free_dofs = (0..self.dof_count())
            .filter(|candidate| !fixed_dofs.contains(candidate))
            .collect::<Vec<_>>();
        if free_dofs.is_empty() {
            return Err(StructDynError::InvalidMatrix {
                field: "beam.fixed_dofs",
                reason: "at least one free DOF is required",
            });
        }
        let mass = submatrix(&self.mass, &free_dofs)?;
        let stiffness = submatrix(&self.stiffness, &free_dofs)?;
        Ok(FreeDofMatrices {
            mass,
            stiffness,
            free_dofs,
        })
    }
}

/// Reduced free-DOF matrices for a clamped beam model.
#[derive(Clone, Debug, PartialEq)]
pub struct FreeDofMatrices {
    mass: DenseMatrix,
    stiffness: DenseMatrix,
    free_dofs: Vec<usize>,
}

impl FreeDofMatrices {
    /// Reduced mass matrix.
    #[must_use]
    pub const fn mass(&self) -> &DenseMatrix {
        &self.mass
    }

    /// Reduced stiffness matrix.
    #[must_use]
    pub const fn stiffness(&self) -> &DenseMatrix {
        &self.stiffness
    }

    /// Free DOF indices in the original assembled model.
    #[must_use]
    pub fn free_dofs(&self) -> &[usize] {
        &self.free_dofs
    }
}

/// Assemble deterministic Euler-Bernoulli beam mass and stiffness matrices.
///
/// Each node contributes two DOFs in declaration order: transverse displacement
/// then rotation. Element local DOFs are `[w_a, theta_a, w_b, theta_b]`.
///
/// # Errors
///
/// Returns [`StructDynError`] for fewer than two nodes, no elements, out-of-range
/// element node ids, or invalid element values.
pub fn assemble_euler_bernoulli_beam(
    node_count: usize,
    elements: &[EulerBernoulliBeamElement],
) -> Result<BeamAssembly, StructDynError> {
    if node_count < 2 {
        return Err(StructDynError::DimensionMismatch {
            field: "beam.node_count",
            expected: 2,
            actual: node_count,
        });
    }
    if elements.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "beam.elements",
            reason: "at least one element is required",
        });
    }
    let dof_count = node_count
        .checked_mul(2)
        .ok_or(StructDynError::InvalidMatrix {
            field: "beam.dofs",
            reason: "node count overflow",
        })?;
    let mut mass = DenseMatrix::zeros(dof_count, dof_count)?;
    let mut stiffness = DenseMatrix::zeros(dof_count, dof_count)?;

    for element in elements {
        if element.node_a >= node_count {
            return Err(StructDynError::DimensionMismatch {
                field: "beam.element.node_a",
                expected: node_count,
                actual: element.node_a + 1,
            });
        }
        if element.node_b >= node_count {
            return Err(StructDynError::DimensionMismatch {
                field: "beam.element.node_b",
                expected: node_count,
                actual: element.node_b + 1,
            });
        }
        let dofs = [
            2 * element.node_a,
            (2 * element.node_a) + 1,
            2 * element.node_b,
            (2 * element.node_b) + 1,
        ];
        let local_mass = local_consistent_mass(*element);
        let local_stiffness = local_bending_stiffness(*element);
        scatter_add(&mut mass, &dofs, &local_mass);
        scatter_add(&mut stiffness, &dofs, &local_stiffness);
    }

    Ok(BeamAssembly {
        node_count,
        mass,
        stiffness,
    })
}

/// Assemble deterministic plane-stress CST membrane mass and stiffness matrices.
///
/// Each node contributes two DOFs in declaration order: in-plane `u_x`, then
/// `u_y`. Element local DOFs are `[u_ax, u_ay, u_bx, u_by, u_cx, u_cy]`.
///
/// # Errors
///
/// Returns [`StructDynError`] for fewer than three nodes, no elements,
/// out-of-range element node ids, degenerate triangle area, or invalid element
/// values.
pub fn assemble_constant_strain_triangle_membrane(
    nodes: &[MembraneNode],
    elements: &[ConstantStrainTriangleMembraneElement],
) -> Result<MembraneAssembly, StructDynError> {
    if nodes.len() < 3 {
        return Err(StructDynError::DimensionMismatch {
            field: "membrane.node_count",
            expected: 3,
            actual: nodes.len(),
        });
    }
    if elements.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "membrane.elements",
            reason: "at least one element is required",
        });
    }
    let dof_count = nodes
        .len()
        .checked_mul(2)
        .ok_or(StructDynError::InvalidMatrix {
            field: "membrane.dofs",
            reason: "node count overflow",
        })?;
    let mut mass = DenseMatrix::zeros(dof_count, dof_count)?;
    let mut stiffness = DenseMatrix::zeros(dof_count, dof_count)?;

    for element in elements {
        validate_node_index("membrane.element.node_a", element.node_a, nodes.len())?;
        validate_node_index("membrane.element.node_b", element.node_b, nodes.len())?;
        validate_node_index("membrane.element.node_c", element.node_c, nodes.len())?;
        let element_nodes = [
            nodes[element.node_a],
            nodes[element.node_b],
            nodes[element.node_c],
        ];
        let dofs = [
            2 * element.node_a,
            (2 * element.node_a) + 1,
            2 * element.node_b,
            (2 * element.node_b) + 1,
            2 * element.node_c,
            (2 * element.node_c) + 1,
        ];
        let local_mass = local_membrane_consistent_mass(*element, &element_nodes)?;
        let local_stiffness = local_membrane_stiffness(*element, &element_nodes)?;
        scatter_add_square(&mut mass, &dofs, &local_mass);
        scatter_add_square(&mut stiffness, &dofs, &local_stiffness);
    }

    Ok(MembraneAssembly {
        node_count: nodes.len(),
        mass,
        stiffness,
    })
}

/// Assemble deterministic plane-stress bilinear quadrilateral membrane matrices.
///
/// Each node contributes two DOFs in declaration order: in-plane `u_x`, then
/// `u_y`. Element local DOFs are
/// `[u_ax, u_ay, u_bx, u_by, u_cx, u_cy, u_dx, u_dy]`. Local mass and stiffness
/// are evaluated by 2x2 Gauss integration over the isoparametric element.
///
/// # Errors
///
/// Returns [`StructDynError`] for fewer than four nodes, no elements,
/// out-of-range element node ids, folded/degenerate quadrilaterals, or invalid
/// element values.
pub fn assemble_bilinear_quadrilateral_membrane(
    nodes: &[MembraneNode],
    elements: &[BilinearQuadrilateralMembraneElement],
) -> Result<MembraneAssembly, StructDynError> {
    if nodes.len() < 4 {
        return Err(StructDynError::DimensionMismatch {
            field: "quad_membrane.node_count",
            expected: 4,
            actual: nodes.len(),
        });
    }
    if elements.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "quad_membrane.elements",
            reason: "at least one element is required",
        });
    }
    let dof_count = nodes
        .len()
        .checked_mul(2)
        .ok_or(StructDynError::InvalidMatrix {
            field: "quad_membrane.dofs",
            reason: "node count overflow",
        })?;
    let mut mass = DenseMatrix::zeros(dof_count, dof_count)?;
    let mut stiffness = DenseMatrix::zeros(dof_count, dof_count)?;

    for element in elements {
        validate_node_index("quad_membrane.element.node_a", element.node_a, nodes.len())?;
        validate_node_index("quad_membrane.element.node_b", element.node_b, nodes.len())?;
        validate_node_index("quad_membrane.element.node_c", element.node_c, nodes.len())?;
        validate_node_index("quad_membrane.element.node_d", element.node_d, nodes.len())?;
        let element_nodes = [
            nodes[element.node_a],
            nodes[element.node_b],
            nodes[element.node_c],
            nodes[element.node_d],
        ];
        let dofs = [
            2 * element.node_a,
            (2 * element.node_a) + 1,
            2 * element.node_b,
            (2 * element.node_b) + 1,
            2 * element.node_c,
            (2 * element.node_c) + 1,
            2 * element.node_d,
            (2 * element.node_d) + 1,
        ];
        let local_mass = local_quad_membrane_consistent_mass(*element, &element_nodes)?;
        let local_stiffness = local_quad_membrane_stiffness(*element, &element_nodes)?;
        scatter_add_square(&mut mass, &dofs, &local_mass);
        scatter_add_square(&mut stiffness, &dofs, &local_stiffness);
    }

    Ok(MembraneAssembly {
        node_count: nodes.len(),
        mass,
        stiffness,
    })
}

/// Assemble deterministic Reissner-Mindlin triangular plate-bending matrices.
///
/// Each node contributes three DOFs in declaration order: transverse
/// displacement `w`, rotation `theta_x`, then rotation `theta_y`. Element local
/// DOFs are `[w_a, theta_x_a, theta_y_a, ...]`. The element uses constant
/// bending curvature plus centroid-evaluated transverse shear.
///
/// # Errors
///
/// Returns [`StructDynError`] for fewer than three nodes, no elements,
/// out-of-range element node ids, degenerate triangle area, or invalid element
/// values.
pub fn assemble_reissner_mindlin_triangle_plate(
    nodes: &[MembraneNode],
    elements: &[ReissnerMindlinTrianglePlateElement],
) -> Result<PlateBendingAssembly, StructDynError> {
    if nodes.len() < 3 {
        return Err(StructDynError::DimensionMismatch {
            field: "plate.node_count",
            expected: 3,
            actual: nodes.len(),
        });
    }
    if elements.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "plate.elements",
            reason: "at least one element is required",
        });
    }
    let dof_count = nodes
        .len()
        .checked_mul(3)
        .ok_or(StructDynError::InvalidMatrix {
            field: "plate.dofs",
            reason: "node count overflow",
        })?;
    let mut mass = DenseMatrix::zeros(dof_count, dof_count)?;
    let mut stiffness = DenseMatrix::zeros(dof_count, dof_count)?;

    for element in elements {
        validate_node_index("plate.element.node_a", element.node_a, nodes.len())?;
        validate_node_index("plate.element.node_b", element.node_b, nodes.len())?;
        validate_node_index("plate.element.node_c", element.node_c, nodes.len())?;
        let element_nodes = [
            nodes[element.node_a],
            nodes[element.node_b],
            nodes[element.node_c],
        ];
        let dofs = [
            3 * element.node_a,
            (3 * element.node_a) + 1,
            (3 * element.node_a) + 2,
            3 * element.node_b,
            (3 * element.node_b) + 1,
            (3 * element.node_b) + 2,
            3 * element.node_c,
            (3 * element.node_c) + 1,
            (3 * element.node_c) + 2,
        ];
        let local_mass = local_plate_consistent_mass(*element, &element_nodes)?;
        let local_stiffness = local_plate_stiffness(*element, &element_nodes)?;
        scatter_add_square(&mut mass, &dofs, &local_mass);
        scatter_add_square(&mut stiffness, &dofs, &local_stiffness);
    }

    Ok(PlateBendingAssembly {
        node_count: nodes.len(),
        mass,
        stiffness,
    })
}

/// Assemble deterministic six-DOF triangular shell mass and stiffness matrices.
///
/// Each node contributes six DOFs in declaration order:
/// `[u_x, u_y, w, theta_x, theta_y, theta_z]`. The element combines the CST
/// plane-stress membrane block, the Reissner-Mindlin plate-bending block, and
/// a small diagonal drilling-rotation regularization.
///
/// # Errors
///
/// Returns [`StructDynError`] for fewer than three nodes, no elements,
/// out-of-range element node ids, degenerate triangle area, or invalid element
/// values.
pub fn assemble_triangular_shell(
    nodes: &[MembraneNode],
    elements: &[TriangularShellElement],
) -> Result<ShellAssembly, StructDynError> {
    if nodes.len() < 3 {
        return Err(StructDynError::DimensionMismatch {
            field: "shell.node_count",
            expected: 3,
            actual: nodes.len(),
        });
    }
    if elements.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "shell.elements",
            reason: "at least one element is required",
        });
    }
    let dof_count = nodes
        .len()
        .checked_mul(6)
        .ok_or(StructDynError::InvalidMatrix {
            field: "shell.dofs",
            reason: "node count overflow",
        })?;
    let mut mass = DenseMatrix::zeros(dof_count, dof_count)?;
    let mut stiffness = DenseMatrix::zeros(dof_count, dof_count)?;

    for element in elements {
        validate_node_index("shell.element.node_a", element.node_a, nodes.len())?;
        validate_node_index("shell.element.node_b", element.node_b, nodes.len())?;
        validate_node_index("shell.element.node_c", element.node_c, nodes.len())?;
        let element_nodes = [
            nodes[element.node_a],
            nodes[element.node_b],
            nodes[element.node_c],
        ];
        let dofs = [
            6 * element.node_a,
            (6 * element.node_a) + 1,
            (6 * element.node_a) + 2,
            (6 * element.node_a) + 3,
            (6 * element.node_a) + 4,
            (6 * element.node_a) + 5,
            6 * element.node_b,
            (6 * element.node_b) + 1,
            (6 * element.node_b) + 2,
            (6 * element.node_b) + 3,
            (6 * element.node_b) + 4,
            (6 * element.node_b) + 5,
            6 * element.node_c,
            (6 * element.node_c) + 1,
            (6 * element.node_c) + 2,
            (6 * element.node_c) + 3,
            (6 * element.node_c) + 4,
            (6 * element.node_c) + 5,
        ];
        let local_mass = local_shell_consistent_mass(*element, &element_nodes)?;
        let local_stiffness = local_shell_stiffness(*element, &element_nodes)?;
        scatter_add_square(&mut mass, &dofs, &local_mass);
        scatter_add_square(&mut stiffness, &dofs, &local_stiffness);
    }

    Ok(ShellAssembly {
        node_count: nodes.len(),
        mass,
        stiffness,
    })
}

/// Assemble deterministic six-DOF quadrilateral shell mass and stiffness matrices.
///
/// Each node contributes six DOFs in declaration order:
/// `[u_x, u_y, w, theta_x, theta_y, theta_z]`. The element combines the
/// bilinear quadrilateral plane-stress membrane block, a deterministic
/// two-triangle Reissner-Mindlin plate block, and a small diagonal
/// drilling-rotation regularization.
///
/// # Errors
///
/// Returns [`StructDynError`] for fewer than four nodes, no elements,
/// out-of-range element node ids, folded/degenerate quadrilaterals, degenerate
/// plate triangles, or invalid element values.
pub fn assemble_quadrilateral_shell(
    nodes: &[MembraneNode],
    elements: &[QuadrilateralShellElement],
) -> Result<ShellAssembly, StructDynError> {
    if nodes.len() < 4 {
        return Err(StructDynError::DimensionMismatch {
            field: "quad_shell.node_count",
            expected: 4,
            actual: nodes.len(),
        });
    }
    if elements.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "quad_shell.elements",
            reason: "at least one element is required",
        });
    }
    let dof_count = nodes
        .len()
        .checked_mul(6)
        .ok_or(StructDynError::InvalidMatrix {
            field: "quad_shell.dofs",
            reason: "node count overflow",
        })?;
    let mut mass = DenseMatrix::zeros(dof_count, dof_count)?;
    let mut stiffness = DenseMatrix::zeros(dof_count, dof_count)?;

    for element in elements {
        validate_node_index("quad_shell.element.node_a", element.node_a, nodes.len())?;
        validate_node_index("quad_shell.element.node_b", element.node_b, nodes.len())?;
        validate_node_index("quad_shell.element.node_c", element.node_c, nodes.len())?;
        validate_node_index("quad_shell.element.node_d", element.node_d, nodes.len())?;
        let element_nodes = [
            nodes[element.node_a],
            nodes[element.node_b],
            nodes[element.node_c],
            nodes[element.node_d],
        ];
        let dofs = [
            6 * element.node_a,
            (6 * element.node_a) + 1,
            (6 * element.node_a) + 2,
            (6 * element.node_a) + 3,
            (6 * element.node_a) + 4,
            (6 * element.node_a) + 5,
            6 * element.node_b,
            (6 * element.node_b) + 1,
            (6 * element.node_b) + 2,
            (6 * element.node_b) + 3,
            (6 * element.node_b) + 4,
            (6 * element.node_b) + 5,
            6 * element.node_c,
            (6 * element.node_c) + 1,
            (6 * element.node_c) + 2,
            (6 * element.node_c) + 3,
            (6 * element.node_c) + 4,
            (6 * element.node_c) + 5,
            6 * element.node_d,
            (6 * element.node_d) + 1,
            (6 * element.node_d) + 2,
            (6 * element.node_d) + 3,
            (6 * element.node_d) + 4,
            (6 * element.node_d) + 5,
        ];
        let local_mass = local_quad_shell_consistent_mass(*element, &element_nodes)?;
        let local_stiffness = local_quad_shell_stiffness(*element, &element_nodes)?;
        scatter_add_square(&mut mass, &dofs, &local_mass);
        scatter_add_square(&mut stiffness, &dofs, &local_stiffness);
    }

    Ok(ShellAssembly {
        node_count: nodes.len(),
        mass,
        stiffness,
    })
}

/// Classical clamped-free Euler-Bernoulli bending frequency reference.
///
/// `beta_l` should be one entry from [`CANTILEVER_BETA_L`] or another
/// dimensionless root of `cos(beta_l) cosh(beta_l) = -1`.
///
/// # Errors
///
/// Returns [`StructDynError`] when any scalar is not positive finite.
pub fn cantilever_bending_frequency_hz(
    beta_l: f64,
    length_m: f64,
    flexural_rigidity_n_m2: f64,
    mass_per_length_kg_m: f64,
) -> Result<f64, StructDynError> {
    require_positive("beam.beta_l", beta_l)?;
    require_positive("beam.length_m", length_m)?;
    require_positive("beam.flexural_rigidity_n_m2", flexural_rigidity_n_m2)?;
    require_positive("beam.mass_per_length_kg_m", mass_per_length_kg_m)?;
    let omega_rad_s = beta_l * beta_l * (flexural_rigidity_n_m2 / mass_per_length_kg_m).sqrt()
        / (length_m * length_m);
    Ok(omega_rad_s / std::f64::consts::TAU)
}

fn require_positive(field: &'static str, value: f64) -> Result<(), StructDynError> {
    if value.is_finite() && value > 0.0 {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn validate_fixed_dofs(
    field: &'static str,
    fixed_dofs: &[usize],
    dof_count: usize,
) -> Result<Vec<usize>, StructDynError> {
    if fixed_dofs.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field,
            reason: "at least one fixed DOF is required",
        });
    }
    let mut seen = BTreeSet::new();
    for dof in fixed_dofs {
        if *dof >= dof_count {
            return Err(StructDynError::DimensionMismatch {
                field,
                expected: dof_count,
                actual: *dof + 1,
            });
        }
        if !seen.insert(*dof) {
            return Err(StructDynError::InvalidMatrix {
                field,
                reason: "duplicate fixed DOF",
            });
        }
    }
    Ok(fixed_dofs.to_vec())
}

fn validate_node_index(
    field: &'static str,
    node: usize,
    node_count: usize,
) -> Result<(), StructDynError> {
    if node < node_count {
        Ok(())
    } else {
        Err(StructDynError::DimensionMismatch {
            field,
            expected: node_count,
            actual: node + 1,
        })
    }
}

fn local_bending_stiffness(element: EulerBernoulliBeamElement) -> [[f64; 4]; 4] {
    let length = element.length_m;
    let length2 = length * length;
    let scale = element.flexural_rigidity_n_m2 / (length * length * length);
    [
        [
            12.0 * scale,
            6.0 * length * scale,
            -12.0 * scale,
            6.0 * length * scale,
        ],
        [
            6.0 * length * scale,
            4.0 * length2 * scale,
            -6.0 * length * scale,
            2.0 * length2 * scale,
        ],
        [
            -12.0 * scale,
            -6.0 * length * scale,
            12.0 * scale,
            -6.0 * length * scale,
        ],
        [
            6.0 * length * scale,
            2.0 * length2 * scale,
            -6.0 * length * scale,
            4.0 * length2 * scale,
        ],
    ]
}

fn local_consistent_mass(element: EulerBernoulliBeamElement) -> [[f64; 4]; 4] {
    let length = element.length_m;
    let length2 = length * length;
    let scale = element.mass_per_length_kg_m * length / 420.0;
    [
        [
            156.0 * scale,
            22.0 * length * scale,
            54.0 * scale,
            -13.0 * length * scale,
        ],
        [
            22.0 * length * scale,
            4.0 * length2 * scale,
            13.0 * length * scale,
            -3.0 * length2 * scale,
        ],
        [
            54.0 * scale,
            13.0 * length * scale,
            156.0 * scale,
            -22.0 * length * scale,
        ],
        [
            -13.0 * length * scale,
            -3.0 * length2 * scale,
            -22.0 * length * scale,
            4.0 * length2 * scale,
        ],
    ]
}

fn local_membrane_stiffness(
    element: ConstantStrainTriangleMembraneElement,
    nodes: &[MembraneNode; 3],
) -> Result<Vec<f64>, StructDynError> {
    let signed_area2 = signed_membrane_area2(nodes);
    let area = 0.5 * signed_area2.abs();
    if area <= 0.0 || !area.is_finite() {
        return Err(StructDynError::InvalidMatrix {
            field: "membrane.element.area",
            reason: "triangle area must be positive finite",
        });
    }
    let b = [
        nodes[1].y_m - nodes[2].y_m,
        nodes[2].y_m - nodes[0].y_m,
        nodes[0].y_m - nodes[1].y_m,
    ];
    let c = [
        nodes[2].x_m - nodes[1].x_m,
        nodes[0].x_m - nodes[2].x_m,
        nodes[1].x_m - nodes[0].x_m,
    ];
    let inv_area2 = 1.0 / signed_area2;
    let strain_displacement = [
        [
            b[0] * inv_area2,
            0.0,
            b[1] * inv_area2,
            0.0,
            b[2] * inv_area2,
            0.0,
        ],
        [
            0.0,
            c[0] * inv_area2,
            0.0,
            c[1] * inv_area2,
            0.0,
            c[2] * inv_area2,
        ],
        [
            c[0] * inv_area2,
            b[0] * inv_area2,
            c[1] * inv_area2,
            b[1] * inv_area2,
            c[2] * inv_area2,
            b[2] * inv_area2,
        ],
    ];
    let elastic = plane_stress_elastic_matrix(element.young_modulus_pa, element.poisson_ratio);
    let scale = element.thickness_m * area;
    let mut local = vec![0.0; 36];
    for row in 0..6 {
        for col in 0..6 {
            let mut value = 0.0;
            for strain_row in 0..3 {
                for strain_col in 0..3 {
                    value += strain_displacement[strain_row][row]
                        * elastic[strain_row][strain_col]
                        * strain_displacement[strain_col][col];
                }
            }
            local[(row * 6) + col] = scale * value;
        }
    }
    Ok(local)
}

fn local_membrane_consistent_mass(
    element: ConstantStrainTriangleMembraneElement,
    nodes: &[MembraneNode; 3],
) -> Result<Vec<f64>, StructDynError> {
    let signed_area2 = signed_membrane_area2(nodes);
    let area = 0.5 * signed_area2.abs();
    if area <= 0.0 || !area.is_finite() {
        return Err(StructDynError::InvalidMatrix {
            field: "membrane.element.area",
            reason: "triangle area must be positive finite",
        });
    }
    let scale = element.density_kg_m3 * element.thickness_m * area / 12.0;
    let mut local = vec![0.0; 36];
    for node_row in 0..3 {
        for node_col in 0..3 {
            let coefficient = if node_row == node_col { 2.0 } else { 1.0 };
            for axis in 0..2 {
                let row = (node_row * 2) + axis;
                let col = (node_col * 2) + axis;
                local[(row * 6) + col] = coefficient * scale;
            }
        }
    }
    Ok(local)
}

fn local_quad_membrane_stiffness(
    element: BilinearQuadrilateralMembraneElement,
    nodes: &[MembraneNode; 4],
) -> Result<Vec<f64>, StructDynError> {
    validate_quad_gauss_jacobians(nodes)?;
    let elastic = plane_stress_elastic_matrix(element.young_modulus_pa, element.poisson_ratio);
    let mut local = vec![0.0; 64];
    for (xi, eta) in quad_gauss_points() {
        let evaluation = quad_shape_values_and_gradients(nodes, xi, eta)?;
        let mut strain_displacement = [[0.0_f64; 8]; 3];
        for node in 0..4 {
            let dndx = evaluation.gradients[node][0];
            let dndy = evaluation.gradients[node][1];
            strain_displacement[0][node * 2] = dndx;
            strain_displacement[1][(node * 2) + 1] = dndy;
            strain_displacement[2][node * 2] = dndy;
            strain_displacement[2][(node * 2) + 1] = dndx;
        }
        let scale = element.thickness_m * evaluation.det_j_abs;
        for row in 0..8 {
            for col in 0..8 {
                let mut value = 0.0;
                for strain_row in 0..3 {
                    for strain_col in 0..3 {
                        value += strain_displacement[strain_row][row]
                            * elastic[strain_row][strain_col]
                            * strain_displacement[strain_col][col];
                    }
                }
                local[(row * 8) + col] += scale * value;
            }
        }
    }
    Ok(local)
}

fn local_quad_membrane_consistent_mass(
    element: BilinearQuadrilateralMembraneElement,
    nodes: &[MembraneNode; 4],
) -> Result<Vec<f64>, StructDynError> {
    validate_quad_gauss_jacobians(nodes)?;
    let mut local = vec![0.0; 64];
    for (xi, eta) in quad_gauss_points() {
        let evaluation = quad_shape_values_and_gradients(nodes, xi, eta)?;
        let scale = element.density_kg_m3 * element.thickness_m * evaluation.det_j_abs;
        for node_row in 0..4 {
            for node_col in 0..4 {
                let value = scale * evaluation.shape[node_row] * evaluation.shape[node_col];
                for axis in 0..2 {
                    let row = (node_row * 2) + axis;
                    let col = (node_col * 2) + axis;
                    local[(row * 8) + col] += value;
                }
            }
        }
    }
    Ok(local)
}

fn quad_gauss_points() -> [(f64, f64); 4] {
    let point = 1.0_f64 / 3.0_f64.sqrt();
    [
        (-point, -point),
        (point, -point),
        (point, point),
        (-point, point),
    ]
}

struct QuadShapeEvaluation {
    shape: [f64; 4],
    gradients: [[f64; 2]; 4],
    det_j_abs: f64,
}

fn validate_quad_gauss_jacobians(nodes: &[MembraneNode; 4]) -> Result<(), StructDynError> {
    let mut first_det = None;
    for (xi, eta) in quad_gauss_points() {
        let det_j = quad_jacobian_determinant(nodes, xi, eta);
        if !det_j.is_finite() || det_j == 0.0 {
            return Err(StructDynError::InvalidMatrix {
                field: "quad_membrane.element.jacobian",
                reason: "quadrilateral Jacobian must be nonzero finite at every Gauss point",
            });
        }
        if let Some(reference) = first_det {
            if det_j * reference <= 0.0 {
                return Err(StructDynError::InvalidMatrix {
                    field: "quad_membrane.element.jacobian",
                    reason: "quadrilateral Jacobian sign must be consistent at every Gauss point",
                });
            }
        } else {
            first_det = Some(det_j);
        }
    }
    Ok(())
}

fn quad_shape_values_and_gradients(
    nodes: &[MembraneNode; 4],
    xi: f64,
    eta: f64,
) -> Result<QuadShapeEvaluation, StructDynError> {
    let shape = [
        0.25 * (1.0 - xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 - eta),
        0.25 * (1.0 + xi) * (1.0 + eta),
        0.25 * (1.0 - xi) * (1.0 + eta),
    ];
    let dndxi = [
        -0.25 * (1.0 - eta),
        0.25 * (1.0 - eta),
        0.25 * (1.0 + eta),
        -0.25 * (1.0 + eta),
    ];
    let dndeta = [
        -0.25 * (1.0 - xi),
        -0.25 * (1.0 + xi),
        0.25 * (1.0 + xi),
        0.25 * (1.0 - xi),
    ];
    let [dx_dxi, dx_deta, dy_dxi, dy_deta] = quad_jacobian_terms(nodes, &dndxi, &dndeta);
    let det_j = (dx_dxi * dy_deta) - (dx_deta * dy_dxi);
    if !det_j.is_finite() || det_j == 0.0 {
        return Err(StructDynError::InvalidMatrix {
            field: "quad_membrane.element.jacobian",
            reason: "quadrilateral Jacobian must be nonzero finite at every Gauss point",
        });
    }
    let mut gradients = [[0.0; 2]; 4];
    for node in 0..4 {
        gradients[node][0] = ((dy_deta * dndxi[node]) - (dy_dxi * dndeta[node])) / det_j;
        gradients[node][1] = ((-dx_deta * dndxi[node]) + (dx_dxi * dndeta[node])) / det_j;
    }
    Ok(QuadShapeEvaluation {
        shape,
        gradients,
        det_j_abs: det_j.abs(),
    })
}

fn quad_jacobian_determinant(nodes: &[MembraneNode; 4], xi: f64, eta: f64) -> f64 {
    let dndxi = [
        -0.25 * (1.0 - eta),
        0.25 * (1.0 - eta),
        0.25 * (1.0 + eta),
        -0.25 * (1.0 + eta),
    ];
    let dndeta = [
        -0.25 * (1.0 - xi),
        -0.25 * (1.0 + xi),
        0.25 * (1.0 + xi),
        0.25 * (1.0 - xi),
    ];
    let [dx_dxi, dx_deta, dy_dxi, dy_deta] = quad_jacobian_terms(nodes, &dndxi, &dndeta);
    (dx_dxi * dy_deta) - (dx_deta * dy_dxi)
}

fn quad_jacobian_terms(nodes: &[MembraneNode; 4], dndxi: &[f64; 4], dndeta: &[f64; 4]) -> [f64; 4] {
    let mut dx_dxi = 0.0;
    let mut dx_deta = 0.0;
    let mut dy_dxi = 0.0;
    let mut dy_deta = 0.0;
    for node in 0..4 {
        dx_dxi += dndxi[node] * nodes[node].x_m;
        dx_deta += dndeta[node] * nodes[node].x_m;
        dy_dxi += dndxi[node] * nodes[node].y_m;
        dy_deta += dndeta[node] * nodes[node].y_m;
    }
    [dx_dxi, dx_deta, dy_dxi, dy_deta]
}

fn local_plate_stiffness(
    element: ReissnerMindlinTrianglePlateElement,
    nodes: &[MembraneNode; 3],
) -> Result<Vec<f64>, StructDynError> {
    let signed_area2 = signed_membrane_area2(nodes);
    let area = 0.5 * signed_area2.abs();
    if area <= 0.0 || !area.is_finite() {
        return Err(StructDynError::InvalidMatrix {
            field: "plate.element.area",
            reason: "triangle area must be positive finite",
        });
    }
    let b = [
        nodes[1].y_m - nodes[2].y_m,
        nodes[2].y_m - nodes[0].y_m,
        nodes[0].y_m - nodes[1].y_m,
    ];
    let c = [
        nodes[2].x_m - nodes[1].x_m,
        nodes[0].x_m - nodes[2].x_m,
        nodes[1].x_m - nodes[0].x_m,
    ];
    let inv_area2 = 1.0 / signed_area2;
    let shape_at_centroid = 1.0 / 3.0;
    let bending_strain = [
        [
            0.0,
            b[0] * inv_area2,
            0.0,
            0.0,
            b[1] * inv_area2,
            0.0,
            0.0,
            b[2] * inv_area2,
            0.0,
        ],
        [
            0.0,
            0.0,
            c[0] * inv_area2,
            0.0,
            0.0,
            c[1] * inv_area2,
            0.0,
            0.0,
            c[2] * inv_area2,
        ],
        [
            0.0,
            c[0] * inv_area2,
            b[0] * inv_area2,
            0.0,
            c[1] * inv_area2,
            b[1] * inv_area2,
            0.0,
            c[2] * inv_area2,
            b[2] * inv_area2,
        ],
    ];
    let shear_strain = [
        [
            b[0] * inv_area2,
            shape_at_centroid,
            0.0,
            b[1] * inv_area2,
            shape_at_centroid,
            0.0,
            b[2] * inv_area2,
            shape_at_centroid,
            0.0,
        ],
        [
            c[0] * inv_area2,
            0.0,
            shape_at_centroid,
            c[1] * inv_area2,
            0.0,
            shape_at_centroid,
            c[2] * inv_area2,
            0.0,
            shape_at_centroid,
        ],
    ];
    let bending_elastic = plate_bending_elastic_matrix(
        element.young_modulus_pa,
        element.poisson_ratio,
        element.thickness_m,
    );
    let shear_elastic = plate_shear_elastic_matrix(
        element.young_modulus_pa,
        element.poisson_ratio,
        element.thickness_m,
        element.shear_correction_factor,
    );
    let mut local = vec![0.0; 81];
    for row in 0..9 {
        for col in 0..9 {
            let mut value = 0.0;
            for strain_row in 0..3 {
                for strain_col in 0..3 {
                    value += bending_strain[strain_row][row]
                        * bending_elastic[strain_row][strain_col]
                        * bending_strain[strain_col][col];
                }
            }
            for strain_row in 0..2 {
                for strain_col in 0..2 {
                    value += shear_strain[strain_row][row]
                        * shear_elastic[strain_row][strain_col]
                        * shear_strain[strain_col][col];
                }
            }
            local[(row * 9) + col] = area * value;
        }
    }
    Ok(local)
}

fn local_plate_consistent_mass(
    element: ReissnerMindlinTrianglePlateElement,
    nodes: &[MembraneNode; 3],
) -> Result<Vec<f64>, StructDynError> {
    let signed_area2 = signed_membrane_area2(nodes);
    let area = 0.5 * signed_area2.abs();
    if area <= 0.0 || !area.is_finite() {
        return Err(StructDynError::InvalidMatrix {
            field: "plate.element.area",
            reason: "triangle area must be positive finite",
        });
    }
    let transverse_scale = element.density_kg_m3 * element.thickness_m * area / 12.0;
    let rotary_scale = element.density_kg_m3 * element.thickness_m.powi(3) * area / 144.0;
    let mut local = vec![0.0; 81];
    for node_row in 0..3 {
        for node_col in 0..3 {
            let coefficient = if node_row == node_col { 2.0 } else { 1.0 };
            let w_row = node_row * 3;
            let w_col = node_col * 3;
            local[(w_row * 9) + w_col] = coefficient * transverse_scale;
            for axis in 1..=2 {
                let row = (node_row * 3) + axis;
                let col = (node_col * 3) + axis;
                local[(row * 9) + col] = coefficient * rotary_scale;
            }
        }
    }
    Ok(local)
}

fn local_shell_stiffness(
    element: TriangularShellElement,
    nodes: &[MembraneNode; 3],
) -> Result<Vec<f64>, StructDynError> {
    let membrane = ConstantStrainTriangleMembraneElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
    };
    let plate = ReissnerMindlinTrianglePlateElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
        shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
    };
    let membrane_stiffness = local_membrane_stiffness(membrane, nodes)?;
    let plate_stiffness = local_plate_stiffness(plate, nodes)?;
    let area = checked_triangle_area("shell.element.area", nodes)?;
    let drilling_stiffness =
        element.drilling_stiffness_factor * element.young_modulus_pa * element.thickness_m * area;
    let mut local = vec![0.0; 18 * 18];
    scatter_subblock(
        &mut local,
        18,
        &shell_membrane_local_dofs(),
        6,
        &membrane_stiffness,
    );
    scatter_subblock(
        &mut local,
        18,
        &shell_plate_local_dofs(),
        9,
        &plate_stiffness,
    );
    for node in 0..3 {
        let dof = (node * 6) + 5;
        local[(dof * 18) + dof] += drilling_stiffness / 3.0;
    }
    Ok(local)
}

fn local_shell_consistent_mass(
    element: TriangularShellElement,
    nodes: &[MembraneNode; 3],
) -> Result<Vec<f64>, StructDynError> {
    let membrane = ConstantStrainTriangleMembraneElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
    };
    let plate = ReissnerMindlinTrianglePlateElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
        shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
    };
    let membrane_mass = local_membrane_consistent_mass(membrane, nodes)?;
    let plate_mass = local_plate_consistent_mass(plate, nodes)?;
    let area = checked_triangle_area("shell.element.area", nodes)?;
    let drilling_inertia = element.density_kg_m3 * element.thickness_m.powi(3) * area / 12.0;
    let mut local = vec![0.0; 18 * 18];
    scatter_subblock(
        &mut local,
        18,
        &shell_membrane_local_dofs(),
        6,
        &membrane_mass,
    );
    scatter_subblock(&mut local, 18, &shell_plate_local_dofs(), 9, &plate_mass);
    for node_row in 0..3 {
        for node_col in 0..3 {
            let coefficient = if node_row == node_col { 2.0 } else { 1.0 };
            let row = (node_row * 6) + 5;
            let col = (node_col * 6) + 5;
            local[(row * 18) + col] += coefficient * drilling_inertia / 12.0;
        }
    }
    Ok(local)
}

fn local_quad_shell_stiffness(
    element: QuadrilateralShellElement,
    nodes: &[MembraneNode; 4],
) -> Result<Vec<f64>, StructDynError> {
    let membrane = BilinearQuadrilateralMembraneElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        node_d: element.node_d,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
    };
    let plate_abc = ReissnerMindlinTrianglePlateElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
        shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
    };
    let plate_acd = ReissnerMindlinTrianglePlateElement {
        node_a: element.node_a,
        node_b: element.node_c,
        node_c: element.node_d,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
        shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
    };
    let membrane_stiffness = local_quad_membrane_stiffness(membrane, nodes)?;
    let plate_abc_stiffness = local_plate_stiffness(plate_abc, &[nodes[0], nodes[1], nodes[2]])?;
    let plate_acd_stiffness = local_plate_stiffness(plate_acd, &[nodes[0], nodes[2], nodes[3]])?;
    let area = checked_quad_area("quad_shell.element.area", nodes)?;
    let drilling_stiffness =
        element.drilling_stiffness_factor * element.young_modulus_pa * element.thickness_m * area;
    let mut local = vec![0.0; 24 * 24];
    scatter_subblock(
        &mut local,
        24,
        &quad_shell_membrane_local_dofs(),
        8,
        &membrane_stiffness,
    );
    scatter_subblock(
        &mut local,
        24,
        &quad_shell_plate_local_dofs([0, 1, 2]),
        9,
        &plate_abc_stiffness,
    );
    scatter_subblock(
        &mut local,
        24,
        &quad_shell_plate_local_dofs([0, 2, 3]),
        9,
        &plate_acd_stiffness,
    );
    for node in 0..4 {
        let dof = (node * 6) + 5;
        local[(dof * 24) + dof] += drilling_stiffness / 4.0;
    }
    Ok(local)
}

fn local_quad_shell_consistent_mass(
    element: QuadrilateralShellElement,
    nodes: &[MembraneNode; 4],
) -> Result<Vec<f64>, StructDynError> {
    let membrane = BilinearQuadrilateralMembraneElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        node_d: element.node_d,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
    };
    let plate_abc = ReissnerMindlinTrianglePlateElement {
        node_a: element.node_a,
        node_b: element.node_b,
        node_c: element.node_c,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
        shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
    };
    let plate_acd = ReissnerMindlinTrianglePlateElement {
        node_a: element.node_a,
        node_b: element.node_c,
        node_c: element.node_d,
        thickness_m: element.thickness_m,
        young_modulus_pa: element.young_modulus_pa,
        poisson_ratio: element.poisson_ratio,
        density_kg_m3: element.density_kg_m3,
        shear_correction_factor: MINDLIN_DEFAULT_SHEAR_CORRECTION,
    };
    let membrane_mass = local_quad_membrane_consistent_mass(membrane, nodes)?;
    let plate_abc_mass = local_plate_consistent_mass(plate_abc, &[nodes[0], nodes[1], nodes[2]])?;
    let plate_acd_mass = local_plate_consistent_mass(plate_acd, &[nodes[0], nodes[2], nodes[3]])?;
    let mut local = vec![0.0; 24 * 24];
    scatter_subblock(
        &mut local,
        24,
        &quad_shell_membrane_local_dofs(),
        8,
        &membrane_mass,
    );
    scatter_subblock(
        &mut local,
        24,
        &quad_shell_plate_local_dofs([0, 1, 2]),
        9,
        &plate_abc_mass,
    );
    scatter_subblock(
        &mut local,
        24,
        &quad_shell_plate_local_dofs([0, 2, 3]),
        9,
        &plate_acd_mass,
    );
    for (xi, eta) in quad_gauss_points() {
        let evaluation = quad_shape_values_and_gradients(nodes, xi, eta)?;
        let scale =
            element.density_kg_m3 * element.thickness_m.powi(3) * evaluation.det_j_abs / 12.0;
        for node_row in 0..4 {
            for node_col in 0..4 {
                let row = (node_row * 6) + 5;
                let col = (node_col * 6) + 5;
                local[(row * 24) + col] +=
                    scale * evaluation.shape[node_row] * evaluation.shape[node_col];
            }
        }
    }
    Ok(local)
}

fn checked_triangle_area(
    field: &'static str,
    nodes: &[MembraneNode; 3],
) -> Result<f64, StructDynError> {
    let area = 0.5 * signed_membrane_area2(nodes).abs();
    if area <= 0.0 || !area.is_finite() {
        Err(StructDynError::InvalidMatrix {
            field,
            reason: "triangle area must be positive finite",
        })
    } else {
        Ok(area)
    }
}

fn checked_quad_area(
    field: &'static str,
    nodes: &[MembraneNode; 4],
) -> Result<f64, StructDynError> {
    let first = checked_triangle_area(field, &[nodes[0], nodes[1], nodes[2]])?;
    let second = checked_triangle_area(field, &[nodes[0], nodes[2], nodes[3]])?;
    Ok(first + second)
}

fn shell_membrane_local_dofs() -> [usize; 6] {
    [0, 1, 6, 7, 12, 13]
}

fn shell_plate_local_dofs() -> [usize; 9] {
    [2, 3, 4, 8, 9, 10, 14, 15, 16]
}

fn quad_shell_membrane_local_dofs() -> [usize; 8] {
    [0, 1, 6, 7, 12, 13, 18, 19]
}

fn quad_shell_plate_local_dofs(nodes: [usize; 3]) -> [usize; 9] {
    [
        (nodes[0] * 6) + 2,
        (nodes[0] * 6) + 3,
        (nodes[0] * 6) + 4,
        (nodes[1] * 6) + 2,
        (nodes[1] * 6) + 3,
        (nodes[1] * 6) + 4,
        (nodes[2] * 6) + 2,
        (nodes[2] * 6) + 3,
        (nodes[2] * 6) + 4,
    ]
}

fn signed_membrane_area2(nodes: &[MembraneNode; 3]) -> f64 {
    ((nodes[1].x_m - nodes[0].x_m) * (nodes[2].y_m - nodes[0].y_m))
        - ((nodes[2].x_m - nodes[0].x_m) * (nodes[1].y_m - nodes[0].y_m))
}

fn plane_stress_elastic_matrix(young_modulus_pa: f64, poisson_ratio: f64) -> [[f64; 3]; 3] {
    let scale = young_modulus_pa / (1.0 - (poisson_ratio * poisson_ratio));
    [
        [scale, scale * poisson_ratio, 0.0],
        [scale * poisson_ratio, scale, 0.0],
        [0.0, 0.0, scale * (1.0 - poisson_ratio) * 0.5],
    ]
}

fn plate_bending_elastic_matrix(
    young_modulus_pa: f64,
    poisson_ratio: f64,
    thickness_m: f64,
) -> [[f64; 3]; 3] {
    let scale =
        young_modulus_pa * thickness_m.powi(3) / (12.0 * (1.0 - (poisson_ratio * poisson_ratio)));
    [
        [scale, scale * poisson_ratio, 0.0],
        [scale * poisson_ratio, scale, 0.0],
        [0.0, 0.0, scale * (1.0 - poisson_ratio) * 0.5],
    ]
}

fn plate_shear_elastic_matrix(
    young_modulus_pa: f64,
    poisson_ratio: f64,
    thickness_m: f64,
    shear_correction_factor: f64,
) -> [[f64; 2]; 2] {
    let shear_modulus_pa = young_modulus_pa / (2.0 * (1.0 + poisson_ratio));
    let scale = shear_correction_factor * shear_modulus_pa * thickness_m;
    [[scale, 0.0], [0.0, scale]]
}

fn scatter_add(matrix: &mut DenseMatrix, dofs: &[usize; 4], local: &[[f64; 4]; 4]) {
    for (local_row, global_row) in dofs.iter().enumerate() {
        for (local_col, global_col) in dofs.iter().enumerate() {
            let value = matrix.get(*global_row, *global_col) + local[local_row][local_col];
            matrix.set(*global_row, *global_col, value);
        }
    }
}

fn scatter_add_square(matrix: &mut DenseMatrix, dofs: &[usize], local: &[f64]) {
    let order = dofs.len();
    for (local_row, global_row) in dofs.iter().enumerate() {
        for (local_col, global_col) in dofs.iter().enumerate() {
            let value =
                matrix.get(*global_row, *global_col) + local[(local_row * order) + local_col];
            matrix.set(*global_row, *global_col, value);
        }
    }
}

fn scatter_subblock(
    target: &mut [f64],
    target_order: usize,
    dofs: &[usize],
    order: usize,
    block: &[f64],
) {
    for (block_row, target_row) in dofs.iter().enumerate() {
        for (block_col, target_col) in dofs.iter().enumerate() {
            target[(*target_row * target_order) + *target_col] +=
                block[(block_row * order) + block_col];
        }
    }
}

fn submatrix(matrix: &DenseMatrix, dofs: &[usize]) -> Result<DenseMatrix, StructDynError> {
    let mut out = DenseMatrix::zeros(dofs.len(), dofs.len())?;
    for (row_index, row) in dofs.iter().enumerate() {
        for (col_index, col) in dofs.iter().enumerate() {
            out.set(row_index, col_index, matrix.get(*row, *col));
        }
    }
    Ok(out)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::cb::{
        FixedInterfaceExtractionOptions, SparseMatrixEntry, SparseSymmetricMatrix,
        extract_fixed_interface_modes_shift_invert,
    };

    fn uniform_beam(node_count: usize, length_m: f64) -> Vec<EulerBernoulliBeamElement> {
        let element_length = length_m / ((node_count - 1) as f64);
        (0..(node_count - 1))
            .map(|node| {
                EulerBernoulliBeamElement::new(node, node + 1, element_length, 1.0, 1.0).unwrap()
            })
            .collect()
    }

    fn dense_to_sparse(matrix: &DenseMatrix) -> SparseSymmetricMatrix {
        let mut entries = Vec::new();
        for row in 0..matrix.rows() {
            for col in row..matrix.cols() {
                let value = matrix.get(row, col);
                if value.to_bits() != 0.0_f64.to_bits() {
                    entries.push(SparseMatrixEntry { row, col, value });
                }
            }
        }
        SparseSymmetricMatrix::new(matrix.rows(), entries).unwrap()
    }

    fn unit_triangle_nodes() -> Vec<MembraneNode> {
        vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(0.0, 1.0).unwrap(),
        ]
    }

    fn unit_triangle_element() -> ConstantStrainTriangleMembraneElement {
        ConstantStrainTriangleMembraneElement::new(0, 1, 2, 0.1, 1_000.0, 0.25, 2.0).unwrap()
    }

    fn unit_square_nodes() -> Vec<MembraneNode> {
        vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 1.0).unwrap(),
            MembraneNode::new(0.0, 1.0).unwrap(),
        ]
    }

    fn unit_quad_element() -> BilinearQuadrilateralMembraneElement {
        BilinearQuadrilateralMembraneElement::new([0, 1, 2, 3], 0.1, 1_000.0, 0.25, 2.0).unwrap()
    }

    fn unit_plate_element() -> ReissnerMindlinTrianglePlateElement {
        ReissnerMindlinTrianglePlateElement::new(0, 1, 2, 0.1, 1_000.0, 0.25, 2.0).unwrap()
    }

    fn unit_shell_element() -> TriangularShellElement {
        TriangularShellElement::new(0, 1, 2, 0.1, 1_000.0, 0.25, 2.0).unwrap()
    }

    fn unit_quad_shell_element() -> QuadrilateralShellElement {
        QuadrilateralShellElement::new([0, 1, 2, 3], 0.1, 1_000.0, 0.25, 2.0).unwrap()
    }

    fn matrix_vector_product(matrix: &DenseMatrix, values: &[f64]) -> Vec<f64> {
        (0..matrix.rows())
            .map(|row| {
                values
                    .iter()
                    .enumerate()
                    .map(|(col, value)| matrix.get(row, col) * *value)
                    .sum()
            })
            .collect()
    }

    #[test]
    fn uniform_cantilever_beam_frequencies_match_blevins_reference() {
        let total_length_m = 1.0;
        let mode_count = 5;
        let assembly =
            assemble_euler_bernoulli_beam(33, &uniform_beam(33, total_length_m)).unwrap();
        let mass = dense_to_sparse(assembly.mass());
        let stiffness = dense_to_sparse(assembly.stiffness());
        let modes = extract_fixed_interface_modes_shift_invert(
            &mass,
            &stiffness,
            &[0, 1],
            FixedInterfaceExtractionOptions {
                mode_count,
                shift_rad_s2: 0.0,
                tolerance: 1.0e-7,
                max_iterations: 250,
            },
        )
        .unwrap();

        for (mode_index, beta_l) in CANTILEVER_BETA_L.iter().take(mode_count).enumerate() {
            let actual_hz = modes.eigenvalues_rad_s2()[mode_index].sqrt() / std::f64::consts::TAU;
            let expected_hz =
                cantilever_bending_frequency_hz(*beta_l, total_length_m, 1.0, 1.0).unwrap();
            let relative_error = ((actual_hz - expected_hz) / expected_hz).abs();
            assert!(
                relative_error < 0.01,
                "mode {mode_index} relative error {relative_error} exceeded 1%"
            );
        }
    }

    #[test]
    fn beam_assembly_extracts_free_dof_matrices_in_locked_order() {
        let assembly = assemble_euler_bernoulli_beam(2, &uniform_beam(2, 2.0)).unwrap();
        let free = assembly.free_dof_matrices(&[0, 1]).unwrap();

        assert_eq!(free.free_dofs(), &[2, 3]);
        assert_eq!(free.mass().rows(), 2);
        assert_eq!(free.stiffness().cols(), 2);
        assert!((free.stiffness().get(0, 0) - 1.5).abs() < 1.0e-14);
        assert!((free.stiffness().get(1, 1) - 2.0).abs() < 1.0e-14);
    }

    #[test]
    fn beam_assembly_rejects_invalid_inputs() {
        assert!(EulerBernoulliBeamElement::new(0, 0, 1.0, 1.0, 1.0).is_err());
        assert!(EulerBernoulliBeamElement::new(0, 1, 0.0, 1.0, 1.0).is_err());
        assert!(assemble_euler_bernoulli_beam(1, &[]).is_err());
        let bad_node = EulerBernoulliBeamElement::new(0, 2, 1.0, 1.0, 1.0).unwrap();
        assert!(assemble_euler_bernoulli_beam(2, &[bad_node]).is_err());
        let assembly = assemble_euler_bernoulli_beam(2, &uniform_beam(2, 1.0)).unwrap();
        assert!(assembly.free_dof_matrices(&[0, 0]).is_err());
        assert!(assembly.free_dof_matrices(&[0, 1, 2, 3]).is_err());
        assert!(cantilever_bending_frequency_hz(CANTILEVER_BETA_L[0], 0.0, 1.0, 1.0).is_err());
    }

    #[test]
    fn cst_membrane_has_rigid_translation_null_modes_and_consistent_mass() {
        let assembly = assemble_constant_strain_triangle_membrane(
            &unit_triangle_nodes(),
            &[unit_triangle_element()],
        )
        .unwrap();

        assert_eq!(assembly.node_count(), 3);
        assert_eq!(assembly.dof_count(), 6);
        for row in 0..assembly.dof_count() {
            for col in 0..assembly.dof_count() {
                assert!(
                    (assembly.stiffness().get(row, col) - assembly.stiffness().get(col, row)).abs()
                        < 1.0e-12
                );
                assert!(
                    (assembly.mass().get(row, col) - assembly.mass().get(col, row)).abs() < 1.0e-14
                );
            }
        }

        let x_translation =
            matrix_vector_product(assembly.stiffness(), &[1.0, 0.0, 1.0, 0.0, 1.0, 0.0]);
        let y_translation =
            matrix_vector_product(assembly.stiffness(), &[0.0, 1.0, 0.0, 1.0, 0.0, 1.0]);
        assert!(x_translation.iter().all(|value| value.abs() < 1.0e-12));
        assert!(y_translation.iter().all(|value| value.abs() < 1.0e-12));

        let x_mass_row_sum = [0_usize, 2, 4]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        let y_mass_row_sum = [1_usize, 3, 5]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        assert!((x_mass_row_sum - 0.1).abs() < 1.0e-14);
        assert!((y_mass_row_sum - 0.1).abs() < 1.0e-14);
    }

    #[test]
    fn cst_membrane_extracts_free_dof_matrices_in_locked_order() {
        let assembly = assemble_constant_strain_triangle_membrane(
            &unit_triangle_nodes(),
            &[unit_triangle_element()],
        )
        .unwrap();
        let free = assembly.free_dof_matrices(&[0, 1, 2]).unwrap();

        assert_eq!(free.free_dofs(), &[3, 4, 5]);
        assert_eq!(free.mass().rows(), 3);
        assert_eq!(free.stiffness().cols(), 3);
        assert!((free.mass().get(0, 0) - assembly.mass().get(3, 3)).abs() < 1.0e-14);
        assert!((free.stiffness().get(1, 2) - assembly.stiffness().get(4, 5)).abs() < 1.0e-14);
    }

    #[test]
    fn cst_membrane_rejects_invalid_inputs() {
        assert!(MembraneNode::new(f64::NAN, 0.0).is_err());
        assert!(ConstantStrainTriangleMembraneElement::new(0, 0, 1, 0.1, 1.0, 0.25, 1.0).is_err());
        assert!(ConstantStrainTriangleMembraneElement::new(0, 1, 2, 0.0, 1.0, 0.25, 1.0).is_err());
        assert!(ConstantStrainTriangleMembraneElement::new(0, 1, 2, 0.1, 1.0, 0.5, 1.0).is_err());
        assert!(
            assemble_constant_strain_triangle_membrane(
                &unit_triangle_nodes()[..2],
                &[unit_triangle_element()]
            )
            .is_err()
        );
        let bad_node =
            ConstantStrainTriangleMembraneElement::new(0, 1, 3, 0.1, 1.0, 0.25, 1.0).unwrap();
        assert!(
            assemble_constant_strain_triangle_membrane(&unit_triangle_nodes(), &[bad_node])
                .is_err()
        );
        let collinear = vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(2.0, 0.0).unwrap(),
        ];
        assert!(
            assemble_constant_strain_triangle_membrane(&collinear, &[unit_triangle_element()])
                .is_err()
        );
        let assembly = assemble_constant_strain_triangle_membrane(
            &unit_triangle_nodes(),
            &[unit_triangle_element()],
        )
        .unwrap();
        assert!(assembly.free_dof_matrices(&[0, 0]).is_err());
        assert!(assembly.free_dof_matrices(&[0, 1, 2, 3, 4, 5]).is_err());
    }

    #[test]
    fn bilinear_quad_membrane_has_rigid_translation_null_modes_and_consistent_mass() {
        let assembly =
            assemble_bilinear_quadrilateral_membrane(&unit_square_nodes(), &[unit_quad_element()])
                .unwrap();

        assert_eq!(assembly.node_count(), 4);
        assert_eq!(assembly.dof_count(), 8);
        for row in 0..assembly.dof_count() {
            for col in 0..assembly.dof_count() {
                assert!(
                    (assembly.stiffness().get(row, col) - assembly.stiffness().get(col, row)).abs()
                        < 1.0e-12
                );
                assert!(
                    (assembly.mass().get(row, col) - assembly.mass().get(col, row)).abs() < 1.0e-14
                );
            }
        }

        let x_translation = matrix_vector_product(
            assembly.stiffness(),
            &[1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0],
        );
        let y_translation = matrix_vector_product(
            assembly.stiffness(),
            &[0.0, 1.0, 0.0, 1.0, 0.0, 1.0, 0.0, 1.0],
        );
        assert!(x_translation.iter().all(|value| value.abs() < 1.0e-11));
        assert!(y_translation.iter().all(|value| value.abs() < 1.0e-11));

        let x_mass_row_sum = [0_usize, 2, 4, 6]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        let y_mass_row_sum = [1_usize, 3, 5, 7]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        assert!((x_mass_row_sum - 0.2).abs() < 1.0e-14);
        assert!((y_mass_row_sum - 0.2).abs() < 1.0e-14);
    }

    #[test]
    fn bilinear_quad_membrane_extracts_free_dof_matrices_in_locked_order() {
        let assembly =
            assemble_bilinear_quadrilateral_membrane(&unit_square_nodes(), &[unit_quad_element()])
                .unwrap();
        let free = assembly.free_dof_matrices(&[0, 1, 2]).unwrap();

        assert_eq!(free.free_dofs(), &[3, 4, 5, 6, 7]);
        assert_eq!(free.mass().rows(), 5);
        assert_eq!(free.stiffness().cols(), 5);
        assert!((free.mass().get(0, 0) - assembly.mass().get(3, 3)).abs() < 1.0e-14);
        assert!((free.stiffness().get(2, 4) - assembly.stiffness().get(5, 7)).abs() < 1.0e-14);
    }

    #[test]
    fn bilinear_quad_membrane_rejects_invalid_inputs() {
        assert!(
            BilinearQuadrilateralMembraneElement::new([0, 0, 2, 3], 0.1, 1.0, 0.25, 1.0).is_err()
        );
        assert!(
            BilinearQuadrilateralMembraneElement::new([0, 1, 2, 3], 0.0, 1.0, 0.25, 1.0).is_err()
        );
        assert!(
            BilinearQuadrilateralMembraneElement::new([0, 1, 2, 3], 0.1, 1.0, 0.5, 1.0).is_err()
        );
        assert!(
            assemble_bilinear_quadrilateral_membrane(
                &unit_square_nodes()[..3],
                &[unit_quad_element()]
            )
            .is_err()
        );
        let bad_node =
            BilinearQuadrilateralMembraneElement::new([0, 1, 2, 4], 0.1, 1.0, 0.25, 1.0).unwrap();
        assert!(
            assemble_bilinear_quadrilateral_membrane(&unit_square_nodes(), &[bad_node]).is_err()
        );
        let folded = vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(0.0, 1.0).unwrap(),
            MembraneNode::new(1.0, 1.0).unwrap(),
        ];
        assert!(assemble_bilinear_quadrilateral_membrane(&folded, &[unit_quad_element()]).is_err());
        let assembly =
            assemble_bilinear_quadrilateral_membrane(&unit_square_nodes(), &[unit_quad_element()])
                .unwrap();
        assert!(assembly.free_dof_matrices(&[0, 0]).is_err());
        assert!(
            assembly
                .free_dof_matrices(&[0, 1, 2, 3, 4, 5, 6, 7])
                .is_err()
        );
    }

    #[test]
    fn mindlin_plate_has_rigid_body_null_modes_and_consistent_mass() {
        let assembly = assemble_reissner_mindlin_triangle_plate(
            &unit_triangle_nodes(),
            &[unit_plate_element()],
        )
        .unwrap();

        assert_eq!(assembly.node_count(), 3);
        assert_eq!(assembly.dof_count(), 9);
        for row in 0..assembly.dof_count() {
            for col in 0..assembly.dof_count() {
                assert!(
                    (assembly.stiffness().get(row, col) - assembly.stiffness().get(col, row)).abs()
                        < 1.0e-12
                );
                assert!(
                    (assembly.mass().get(row, col) - assembly.mass().get(col, row)).abs() < 1.0e-14
                );
            }
        }

        let transverse_translation = matrix_vector_product(
            assembly.stiffness(),
            &[1.0, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0, 0.0, 0.0],
        );
        let rigid_x_slope = matrix_vector_product(
            assembly.stiffness(),
            &[0.0, 1.0, 0.0, -1.0, 1.0, 0.0, 0.0, 1.0, 0.0],
        );
        let rigid_y_slope = matrix_vector_product(
            assembly.stiffness(),
            &[0.0, 0.0, 1.0, 0.0, 0.0, 1.0, -1.0, 0.0, 1.0],
        );
        assert!(
            transverse_translation
                .iter()
                .all(|value| value.abs() < 1.0e-12)
        );
        assert!(rigid_x_slope.iter().all(|value| value.abs() < 1.0e-12));
        assert!(rigid_y_slope.iter().all(|value| value.abs() < 1.0e-12));

        let transverse_mass = [0_usize, 3, 6]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        let theta_x_mass = [1_usize, 4, 7]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        let theta_y_mass = [2_usize, 5, 8]
            .into_iter()
            .map(|row| {
                (0..assembly.dof_count())
                    .map(|col| assembly.mass().get(row, col))
                    .sum::<f64>()
            })
            .sum::<f64>();
        assert!((transverse_mass - 0.1).abs() < 1.0e-14);
        assert!((theta_x_mass - (0.1_f64.powi(2) * 0.1 / 12.0)).abs() < 1.0e-14);
        assert!((theta_y_mass - (0.1_f64.powi(2) * 0.1 / 12.0)).abs() < 1.0e-14);
    }

    #[test]
    fn mindlin_plate_extracts_free_dof_matrices_in_locked_order() {
        let assembly = assemble_reissner_mindlin_triangle_plate(
            &unit_triangle_nodes(),
            &[unit_plate_element()],
        )
        .unwrap();
        let free = assembly.free_dof_matrices(&[0, 1, 2, 3]).unwrap();

        assert_eq!(free.free_dofs(), &[4, 5, 6, 7, 8]);
        assert_eq!(free.mass().rows(), 5);
        assert_eq!(free.stiffness().cols(), 5);
        assert!((free.mass().get(0, 0) - assembly.mass().get(4, 4)).abs() < 1.0e-14);
        assert!((free.stiffness().get(1, 3) - assembly.stiffness().get(5, 7)).abs() < 1.0e-14);
    }

    #[test]
    fn mindlin_plate_rejects_invalid_inputs() {
        assert!(ReissnerMindlinTrianglePlateElement::new(0, 0, 1, 0.1, 1.0, 0.25, 1.0).is_err());
        assert!(ReissnerMindlinTrianglePlateElement::new(0, 1, 2, 0.0, 1.0, 0.25, 1.0).is_err());
        assert!(ReissnerMindlinTrianglePlateElement::new(0, 1, 2, 0.1, 1.0, 0.5, 1.0).is_err());
        assert!(
            assemble_reissner_mindlin_triangle_plate(
                &unit_triangle_nodes()[..2],
                &[unit_plate_element()]
            )
            .is_err()
        );
        let bad_node =
            ReissnerMindlinTrianglePlateElement::new(0, 1, 3, 0.1, 1.0, 0.25, 1.0).unwrap();
        assert!(
            assemble_reissner_mindlin_triangle_plate(&unit_triangle_nodes(), &[bad_node]).is_err()
        );
        let collinear = vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(2.0, 0.0).unwrap(),
        ];
        assert!(
            assemble_reissner_mindlin_triangle_plate(&collinear, &[unit_plate_element()]).is_err()
        );
        let assembly = assemble_reissner_mindlin_triangle_plate(
            &unit_triangle_nodes(),
            &[unit_plate_element()],
        )
        .unwrap();
        assert!(assembly.free_dof_matrices(&[0, 0]).is_err());
        assert!(
            assembly
                .free_dof_matrices(&[0, 1, 2, 3, 4, 5, 6, 7, 8])
                .is_err()
        );
    }

    #[test]
    fn triangular_shell_combines_membrane_plate_and_drilling_blocks() {
        let nodes = unit_triangle_nodes();
        let shell = assemble_triangular_shell(&nodes, &[unit_shell_element()]).unwrap();
        let membrane =
            assemble_constant_strain_triangle_membrane(&nodes, &[unit_triangle_element()]).unwrap();
        let plate =
            assemble_reissner_mindlin_triangle_plate(&nodes, &[unit_plate_element()]).unwrap();

        assert_eq!(shell.node_count(), 3);
        assert_eq!(shell.dof_count(), 18);
        let membrane_dofs = shell_membrane_local_dofs();
        for (row, shell_row) in membrane_dofs.iter().enumerate() {
            for (col, shell_col) in membrane_dofs.iter().enumerate() {
                assert!(
                    (shell.stiffness().get(*shell_row, *shell_col)
                        - membrane.stiffness().get(row, col))
                    .abs()
                        < 1.0e-12
                );
                assert!(
                    (shell.mass().get(*shell_row, *shell_col) - membrane.mass().get(row, col))
                        .abs()
                        < 1.0e-14
                );
            }
        }
        let plate_dofs = shell_plate_local_dofs();
        for (row, shell_row) in plate_dofs.iter().enumerate() {
            for (col, shell_col) in plate_dofs.iter().enumerate() {
                assert!(
                    (shell.stiffness().get(*shell_row, *shell_col)
                        - plate.stiffness().get(row, col))
                    .abs()
                        < 1.0e-12
                );
                assert!(
                    (shell.mass().get(*shell_row, *shell_col) - plate.mass().get(row, col)).abs()
                        < 1.0e-14
                );
            }
        }

        for drilling_dof in [5_usize, 11, 17] {
            assert!(shell.stiffness().get(drilling_dof, drilling_dof) > 0.0);
            for col in 0..shell.dof_count() {
                if col != drilling_dof {
                    assert!(shell.stiffness().get(drilling_dof, col).abs() < 1.0e-14);
                    assert!(shell.stiffness().get(col, drilling_dof).abs() < 1.0e-14);
                }
            }
        }
    }

    #[test]
    fn triangular_shell_extracts_free_dof_matrices_in_locked_order() {
        let assembly =
            assemble_triangular_shell(&unit_triangle_nodes(), &[unit_shell_element()]).unwrap();
        let free = assembly.free_dof_matrices(&[0, 1, 2, 3, 4, 5, 6]).unwrap();

        assert_eq!(free.free_dofs(), &[7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17]);
        assert_eq!(free.mass().rows(), 11);
        assert_eq!(free.stiffness().cols(), 11);
        assert!((free.mass().get(0, 0) - assembly.mass().get(7, 7)).abs() < 1.0e-14);
        assert!((free.stiffness().get(3, 8) - assembly.stiffness().get(10, 15)).abs() < 1.0e-14);
    }

    #[test]
    fn triangular_shell_rejects_invalid_inputs() {
        assert!(TriangularShellElement::new(0, 0, 1, 0.1, 1.0, 0.25, 1.0).is_err());
        assert!(TriangularShellElement::new(0, 1, 2, 0.0, 1.0, 0.25, 1.0).is_err());
        assert!(TriangularShellElement::new(0, 1, 2, 0.1, 1.0, 0.5, 1.0).is_err());
        assert!(
            assemble_triangular_shell(&unit_triangle_nodes()[..2], &[unit_shell_element()])
                .is_err()
        );
        let bad_node = TriangularShellElement::new(0, 1, 3, 0.1, 1.0, 0.25, 1.0).unwrap();
        assert!(assemble_triangular_shell(&unit_triangle_nodes(), &[bad_node]).is_err());
        let collinear = vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(2.0, 0.0).unwrap(),
        ];
        assert!(assemble_triangular_shell(&collinear, &[unit_shell_element()]).is_err());
        let assembly =
            assemble_triangular_shell(&unit_triangle_nodes(), &[unit_shell_element()]).unwrap();
        assert!(assembly.free_dof_matrices(&[0, 0]).is_err());
        assert!(
            assembly
                .free_dof_matrices(&(0..assembly.dof_count()).collect::<Vec<_>>())
                .is_err()
        );
    }

    #[test]
    fn quadrilateral_shell_combines_membrane_plate_and_drilling_blocks() {
        let nodes = unit_square_nodes();
        let shell = assemble_quadrilateral_shell(&nodes, &[unit_quad_shell_element()]).unwrap();
        let membrane =
            assemble_bilinear_quadrilateral_membrane(&nodes, &[unit_quad_element()]).unwrap();
        let plate_elements = [
            ReissnerMindlinTrianglePlateElement::new(0, 1, 2, 0.1, 1_000.0, 0.25, 2.0).unwrap(),
            ReissnerMindlinTrianglePlateElement::new(0, 2, 3, 0.1, 1_000.0, 0.25, 2.0).unwrap(),
        ];
        let plate = assemble_reissner_mindlin_triangle_plate(&nodes, &plate_elements).unwrap();

        assert_eq!(shell.node_count(), 4);
        assert_eq!(shell.dof_count(), 24);
        let membrane_dofs = quad_shell_membrane_local_dofs();
        for (row, shell_row) in membrane_dofs.iter().enumerate() {
            for (col, shell_col) in membrane_dofs.iter().enumerate() {
                assert!(
                    (shell.stiffness().get(*shell_row, *shell_col)
                        - membrane.stiffness().get(row, col))
                    .abs()
                        < 1.0e-12
                );
                assert!(
                    (shell.mass().get(*shell_row, *shell_col) - membrane.mass().get(row, col))
                        .abs()
                        < 1.0e-14
                );
            }
        }
        let plate_dofs = [2_usize, 3, 4, 8, 9, 10, 14, 15, 16, 20, 21, 22];
        for (row, shell_row) in plate_dofs.iter().enumerate() {
            for (col, shell_col) in plate_dofs.iter().enumerate() {
                assert!(
                    (shell.stiffness().get(*shell_row, *shell_col)
                        - plate.stiffness().get(row, col))
                    .abs()
                        < 1.0e-12
                );
                assert!(
                    (shell.mass().get(*shell_row, *shell_col) - plate.mass().get(row, col)).abs()
                        < 1.0e-14
                );
            }
        }

        for drilling_dof in [5_usize, 11, 17, 23] {
            assert!(shell.stiffness().get(drilling_dof, drilling_dof) > 0.0);
            for col in 0..shell.dof_count() {
                if col != drilling_dof {
                    assert!(shell.stiffness().get(drilling_dof, col).abs() < 1.0e-14);
                    assert!(shell.stiffness().get(col, drilling_dof).abs() < 1.0e-14);
                }
            }
        }
    }

    #[test]
    fn quadrilateral_shell_extracts_free_dof_matrices_in_locked_order() {
        let assembly =
            assemble_quadrilateral_shell(&unit_square_nodes(), &[unit_quad_shell_element()])
                .unwrap();
        let free = assembly.free_dof_matrices(&[0, 1, 2, 3, 4, 5, 6]).unwrap();
        let expected_free_dofs = (7..24).collect::<Vec<_>>();

        assert_eq!(free.free_dofs(), expected_free_dofs.as_slice());
        assert_eq!(free.mass().rows(), 17);
        assert_eq!(free.stiffness().cols(), 17);
        assert!((free.mass().get(0, 0) - assembly.mass().get(7, 7)).abs() < 1.0e-14);
        assert!((free.stiffness().get(4, 11) - assembly.stiffness().get(11, 18)).abs() < 1.0e-14);
    }

    #[test]
    fn quadrilateral_shell_rejects_invalid_inputs() {
        assert!(QuadrilateralShellElement::new([0, 0, 2, 3], 0.1, 1.0, 0.25, 1.0).is_err());
        assert!(QuadrilateralShellElement::new([0, 1, 2, 3], 0.0, 1.0, 0.25, 1.0).is_err());
        assert!(QuadrilateralShellElement::new([0, 1, 2, 3], 0.1, 1.0, 0.5, 1.0).is_err());
        assert!(
            assemble_quadrilateral_shell(&unit_square_nodes()[..3], &[unit_quad_shell_element()])
                .is_err()
        );
        let bad_node = QuadrilateralShellElement::new([0, 1, 2, 4], 0.1, 1.0, 0.25, 1.0).unwrap();
        assert!(assemble_quadrilateral_shell(&unit_square_nodes(), &[bad_node]).is_err());
        let folded = vec![
            MembraneNode::new(0.0, 0.0).unwrap(),
            MembraneNode::new(1.0, 0.0).unwrap(),
            MembraneNode::new(0.0, 1.0).unwrap(),
            MembraneNode::new(1.0, 1.0).unwrap(),
        ];
        assert!(assemble_quadrilateral_shell(&folded, &[unit_quad_shell_element()]).is_err());
        let assembly =
            assemble_quadrilateral_shell(&unit_square_nodes(), &[unit_quad_shell_element()])
                .unwrap();
        assert!(assembly.free_dof_matrices(&[0, 0]).is_err());
        assert!(
            assembly
                .free_dof_matrices(&(0..assembly.dof_count()).collect::<Vec<_>>())
                .is_err()
        );
    }
}
