//! Craig-Bampton reduction helpers.
//!
//! The routines here operate in deterministic row-major / declaration order.
//! They implement the core CB algebra used by WP-02.3-a: partition the full
//! model into internal and boundary coordinates, solve static constraint modes
//! from the fixed-interface stiffness block, extract sparse fixed-interface
//! modes, assemble the CB transform, and project full mass/stiffness matrices
//! into reduced coordinates.

use std::collections::BTreeSet;

use crate::{DenseCholesky, DenseMatrix, StructDynError};

/// Craig-Bampton reduced model and transform.
#[derive(Clone, Debug, PartialEq)]
pub struct CraigBamptonReduction {
    transform: DenseMatrix,
    reduced_mass: DenseMatrix,
    reduced_stiffness: DenseMatrix,
    internal_dofs: Vec<usize>,
    boundary_dofs: Vec<usize>,
}

/// One entry in a symmetric sparse matrix.
///
/// Entries are stored once; off-diagonal entries are mirrored during matrix
/// operations.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct SparseMatrixEntry {
    /// Row index.
    pub row: usize,
    /// Column index.
    pub col: usize,
    /// Matrix value.
    pub value: f64,
}

/// Symmetric sparse matrix with deterministic declaration-order entries.
#[derive(Clone, Debug, PartialEq)]
pub struct SparseSymmetricMatrix {
    size: usize,
    entries: Vec<SparseMatrixEntry>,
}

impl SparseSymmetricMatrix {
    /// Construct a symmetric sparse matrix from one-sided triplets.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when `size == 0`, a triplet is out of range,
    /// any value is non-finite, or a symmetric entry is duplicated.
    pub fn new(size: usize, entries: Vec<SparseMatrixEntry>) -> Result<Self, StructDynError> {
        if size == 0 {
            return Err(StructDynError::DimensionMismatch {
                field: "sparse.size",
                expected: 1,
                actual: 0,
            });
        }
        let mut seen = BTreeSet::new();
        for entry in &entries {
            if entry.row >= size {
                return Err(StructDynError::DimensionMismatch {
                    field: "sparse.row",
                    expected: size,
                    actual: entry.row + 1,
                });
            }
            if entry.col >= size {
                return Err(StructDynError::DimensionMismatch {
                    field: "sparse.col",
                    expected: size,
                    actual: entry.col + 1,
                });
            }
            if !entry.value.is_finite() {
                return Err(StructDynError::InvalidNumber {
                    field: "sparse.value",
                });
            }
            let key = if entry.row <= entry.col {
                (entry.row, entry.col)
            } else {
                (entry.col, entry.row)
            };
            if !seen.insert(key) {
                return Err(StructDynError::InvalidMatrix {
                    field: "sparse.entries",
                    reason: "duplicate symmetric entry",
                });
            }
        }
        Ok(Self { size, entries })
    }

    /// Matrix dimension.
    #[must_use]
    pub const fn size(&self) -> usize {
        self.size
    }

    /// Declaration-order one-sided entries.
    #[must_use]
    pub fn entries(&self) -> &[SparseMatrixEntry] {
        &self.entries
    }

    /// Sparse symmetric matrix-vector product.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when vector dimensions mismatch or vector
    /// values are non-finite.
    pub fn mul_vec(&self, values: &[f64]) -> Result<Vec<f64>, StructDynError> {
        if values.len() != self.size {
            return Err(StructDynError::DimensionMismatch {
                field: "sparse.vector",
                expected: self.size,
                actual: values.len(),
            });
        }
        if !values.iter().all(|value| value.is_finite()) {
            return Err(StructDynError::InvalidNumber {
                field: "sparse.vector",
            });
        }
        Ok(self.mul_vec_unchecked(values))
    }

    fn mul_vec_unchecked(&self, values: &[f64]) -> Vec<f64> {
        let mut out = vec![0.0; self.size];
        for entry in &self.entries {
            out[entry.row] += entry.value * values[entry.col];
            if entry.row != entry.col {
                out[entry.col] += entry.value * values[entry.row];
            }
        }
        out
    }

    fn submatrix(&self, rows: &[usize]) -> Result<Self, StructDynError> {
        let mut index_map = vec![None; self.size];
        for (local_index, row) in rows.iter().enumerate() {
            index_map[*row] = Some(local_index);
        }
        let mut entries = Vec::new();
        for entry in &self.entries {
            if let (Some(row), Some(col)) = (index_map[entry.row], index_map[entry.col]) {
                entries.push(SparseMatrixEntry {
                    row,
                    col,
                    value: entry.value,
                });
            }
        }
        Self::new(rows.len(), entries)
    }

    fn to_dense(&self) -> Result<DenseMatrix, StructDynError> {
        let mut dense = DenseMatrix::zeros(self.size, self.size)?;
        for entry in &self.entries {
            dense.set(entry.row, entry.col, entry.value);
            if entry.row != entry.col {
                dense.set(entry.col, entry.row, entry.value);
            }
        }
        Ok(dense)
    }
}

/// Shift-invert fixed-interface extraction settings.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FixedInterfaceExtractionOptions {
    /// Number of fixed-interface modes to retain.
    pub mode_count: usize,
    /// Shift in rad²/s². The current Cholesky-backed solver requires
    /// `K_ii - shift_rad_s2 M_ii` to be positive definite.
    pub shift_rad_s2: f64,
    /// Residual norm convergence tolerance.
    pub tolerance: f64,
    /// Maximum inverse-iteration count per mode.
    pub max_iterations: usize,
}

impl FixedInterfaceExtractionOptions {
    fn validate(self, internal_dofs: usize) -> Result<Self, StructDynError> {
        if self.mode_count == 0 {
            return Err(StructDynError::DimensionMismatch {
                field: "fixed_interface.mode_count",
                expected: 1,
                actual: 0,
            });
        }
        if self.mode_count > internal_dofs {
            return Err(StructDynError::DimensionMismatch {
                field: "fixed_interface.mode_count",
                expected: internal_dofs,
                actual: self.mode_count,
            });
        }
        if !self.shift_rad_s2.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "fixed_interface.shift_rad_s2",
            });
        }
        if !self.tolerance.is_finite() || self.tolerance <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "fixed_interface.tolerance",
            });
        }
        if self.max_iterations == 0 {
            return Err(StructDynError::DimensionMismatch {
                field: "fixed_interface.max_iterations",
                expected: 1,
                actual: 0,
            });
        }
        Ok(self)
    }
}

/// Extracted fixed-interface modes.
#[derive(Clone, Debug, PartialEq)]
pub struct FixedInterfaceModes {
    modes: DenseMatrix,
    eigenvalues_rad_s2: Vec<f64>,
    residual_norms: Vec<f64>,
    iterations: Vec<usize>,
    internal_dofs: Vec<usize>,
    boundary_dofs: Vec<usize>,
}

/// One reference mode from a modal benchmark or code-to-code run.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalBenchmarkMode {
    id: String,
    frequency_hz: f64,
    shape: Vec<f64>,
}

impl ModalBenchmarkMode {
    /// Construct a reference mode row.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the id is empty, the frequency is not
    /// positive finite, the shape vector is empty, non-finite, or has zero
    /// norm.
    pub fn new(
        id: impl Into<String>,
        frequency_hz: f64,
        shape: Vec<f64>,
    ) -> Result<Self, StructDynError> {
        let id = id.into();
        if id.trim().is_empty() {
            return Err(StructDynError::InvalidString {
                field: "benchmark.mode.id",
            });
        }
        if !frequency_hz.is_finite() || frequency_hz <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "benchmark.mode.frequency_hz",
            });
        }
        validate_shape("benchmark.mode.shape", &shape)?;
        Ok(Self {
            id,
            frequency_hz,
            shape,
        })
    }

    /// Stable reference mode id.
    #[must_use]
    pub fn id(&self) -> &str {
        &self.id
    }

    /// Reference mode frequency, Hz.
    #[must_use]
    pub const fn frequency_hz(&self) -> f64 {
        self.frequency_hz
    }

    /// Reference shape vector in locked benchmark order.
    #[must_use]
    pub fn shape(&self) -> &[f64] {
        &self.shape
    }
}

/// Frequency/MAC comparison result for one mode.
#[derive(Clone, Debug, PartialEq)]
pub struct ModeCorrelation {
    mode_id: String,
    frequency_error_fraction: f64,
    mac: f64,
}

impl ModeCorrelation {
    /// Reference mode id.
    #[must_use]
    pub fn mode_id(&self) -> &str {
        &self.mode_id
    }

    /// Relative frequency error `|f_actual - f_ref| / |f_ref|`.
    #[must_use]
    pub const fn frequency_error_fraction(&self) -> f64 {
        self.frequency_error_fraction
    }

    /// Modal Assurance Criterion.
    #[must_use]
    pub const fn mac(&self) -> f64 {
        self.mac
    }
}

impl FixedInterfaceModes {
    /// Mass-orthonormal modes, one row per internal DOF and one column per
    /// retained mode.
    #[must_use]
    pub const fn modes(&self) -> &DenseMatrix {
        &self.modes
    }

    /// Fixed-interface eigenvalues in rad²/s².
    #[must_use]
    pub fn eigenvalues_rad_s2(&self) -> &[f64] {
        &self.eigenvalues_rad_s2
    }

    /// Final `Kφ - λMφ` Euclidean residual norms.
    #[must_use]
    pub fn residual_norms(&self) -> &[f64] {
        &self.residual_norms
    }

    /// Iterations used for each retained mode.
    #[must_use]
    pub fn iterations(&self) -> &[usize] {
        &self.iterations
    }

    /// Internal DOF indices in the original full-order model.
    #[must_use]
    pub fn internal_dofs(&self) -> &[usize] {
        &self.internal_dofs
    }

    /// Boundary DOF indices in the original full-order model.
    #[must_use]
    pub fn boundary_dofs(&self) -> &[usize] {
        &self.boundary_dofs
    }
}

/// Compute the Modal Assurance Criterion for two finite nonzero shape vectors.
///
/// # Errors
///
/// Returns [`StructDynError`] when dimensions mismatch, vectors are empty,
/// contain non-finite values, or either vector has zero norm.
pub fn modal_assurance_criterion(left: &[f64], right: &[f64]) -> Result<f64, StructDynError> {
    if left.len() != right.len() {
        return Err(StructDynError::DimensionMismatch {
            field: "mac.shape",
            expected: left.len(),
            actual: right.len(),
        });
    }
    validate_shape("mac.left", left)?;
    validate_shape("mac.right", right)?;
    let dot: f64 = left
        .iter()
        .zip(right.iter())
        .map(|(left_value, right_value)| *left_value * *right_value)
        .sum();
    let left_norm_squared = euclidean_norm_squared(left);
    let right_norm_squared = euclidean_norm_squared(right);
    Ok((dot * dot) / (left_norm_squared * right_norm_squared))
}

/// Build the full MAC matrix between two modal bases.
///
/// Columns are interpreted as mode-shape vectors in locked DOF order. The
/// output row index follows `left_modes` columns and the output column index
/// follows `right_modes` columns.
///
/// # Errors
///
/// Returns [`StructDynError`] when row counts differ, either basis has no modes,
/// or any shape column is invalid.
pub fn modal_assurance_matrix(
    left_modes: &DenseMatrix,
    right_modes: &DenseMatrix,
) -> Result<DenseMatrix, StructDynError> {
    validate_modal_basis_pair("mac.matrix", left_modes, right_modes)?;
    let mut data = Vec::with_capacity(left_modes.cols() * right_modes.cols());
    for left_col in 0..left_modes.cols() {
        let left_shape = matrix_column(left_modes, left_col);
        for right_col in 0..right_modes.cols() {
            let right_shape = matrix_column(right_modes, right_col);
            data.push(modal_assurance_criterion(&left_shape, &right_shape)?);
        }
    }
    DenseMatrix::from_row_major(left_modes.cols(), right_modes.cols(), data)
}

/// Build the mass-weighted cross-orthogonality matrix `left^T M right`.
///
/// Columns are interpreted as mode-shape vectors in locked DOF order. For
/// mass-normalized matching bases, the diagonal should approach one and the
/// off-diagonal terms should approach zero. The output row index follows
/// `left_modes` columns and the output column index follows `right_modes`
/// columns.
///
/// # Errors
///
/// Returns [`StructDynError`] when dimensions are inconsistent, either basis has
/// no modes, or any shape column is invalid.
pub fn modal_cross_orthogonality_matrix(
    left_modes: &DenseMatrix,
    right_modes: &DenseMatrix,
    mass: &DenseMatrix,
) -> Result<DenseMatrix, StructDynError> {
    validate_modal_basis_pair("cross_orthogonality.matrix", left_modes, right_modes)?;
    if mass.rows() != mass.cols() {
        return Err(StructDynError::DimensionMismatch {
            field: "cross_orthogonality.mass",
            expected: mass.rows(),
            actual: mass.cols(),
        });
    }
    if mass.rows() != left_modes.rows() {
        return Err(StructDynError::DimensionMismatch {
            field: "cross_orthogonality.mass",
            expected: left_modes.rows(),
            actual: mass.rows(),
        });
    }
    let mut data = Vec::with_capacity(left_modes.cols() * right_modes.cols());
    for left_col in 0..left_modes.cols() {
        let left_shape = matrix_column(left_modes, left_col);
        validate_shape("cross_orthogonality.left", &left_shape)?;
        for right_col in 0..right_modes.cols() {
            let right_shape = matrix_column(right_modes, right_col);
            validate_shape("cross_orthogonality.right", &right_shape)?;
            data.push(mass_weighted_inner(&left_shape, mass, &right_shape));
        }
    }
    DenseMatrix::from_row_major(left_modes.cols(), right_modes.cols(), data)
}

/// Correlate extracted fixed-interface modes against reference frequency and
/// shape rows.
///
/// Reference rows must be in the same mode order as the extracted modes. The
/// check fails closed if any relative frequency error exceeds
/// `frequency_tolerance_fraction` or any MAC falls below `mac_min`.
///
/// # Errors
///
/// Returns [`StructDynError`] for dimension mismatches, invalid gates, invalid
/// eigenvalues, invalid reference rows, or out-of-tolerance comparisons.
pub fn correlate_fixed_interface_modes(
    extracted: &FixedInterfaceModes,
    reference_modes: &[ModalBenchmarkMode],
    frequency_tolerance_fraction: f64,
    mac_min: f64,
) -> Result<Vec<ModeCorrelation>, StructDynError> {
    if reference_modes.len() != extracted.eigenvalues_rad_s2.len() {
        return Err(StructDynError::DimensionMismatch {
            field: "benchmark.modes",
            expected: extracted.eigenvalues_rad_s2.len(),
            actual: reference_modes.len(),
        });
    }
    if !frequency_tolerance_fraction.is_finite() || frequency_tolerance_fraction < 0.0 {
        return Err(StructDynError::InvalidNumber {
            field: "benchmark.frequency_tolerance_fraction",
        });
    }
    if !mac_min.is_finite() || !(0.0..=1.0).contains(&mac_min) {
        return Err(StructDynError::InvalidNumber {
            field: "benchmark.mac_min",
        });
    }
    let mut correlations = Vec::with_capacity(reference_modes.len());
    for (mode_index, reference) in reference_modes.iter().enumerate() {
        if reference.shape.len() != extracted.modes.rows() {
            return Err(StructDynError::DimensionMismatch {
                field: "benchmark.mode.shape",
                expected: extracted.modes.rows(),
                actual: reference.shape.len(),
            });
        }
        let eigenvalue = extracted.eigenvalues_rad_s2[mode_index];
        if !eigenvalue.is_finite() || eigenvalue <= 0.0 {
            return Err(StructDynError::InvalidNumber {
                field: "fixed_interface.eigenvalue_rad_s2",
            });
        }
        let actual_frequency_hz = eigenvalue.sqrt() / std::f64::consts::TAU;
        let frequency_error_fraction =
            (actual_frequency_hz - reference.frequency_hz).abs() / reference.frequency_hz;
        let actual_shape = (0..extracted.modes.rows())
            .map(|row| extracted.modes.get(row, mode_index))
            .collect::<Vec<_>>();
        let mac = modal_assurance_criterion(&actual_shape, &reference.shape)?;
        if frequency_error_fraction > frequency_tolerance_fraction || mac < mac_min {
            return Err(StructDynError::InvalidMatrix {
                field: "benchmark.mode",
                reason: "frequency error or MAC outside tolerance",
            });
        }
        correlations.push(ModeCorrelation {
            mode_id: reference.id.clone(),
            frequency_error_fraction,
            mac,
        });
    }
    Ok(correlations)
}

impl CraigBamptonReduction {
    /// Full-to-reduced transform `T`.
    #[must_use]
    pub const fn transform(&self) -> &DenseMatrix {
        &self.transform
    }

    /// Reduced mass matrix `T^T M T`.
    #[must_use]
    pub const fn reduced_mass(&self) -> &DenseMatrix {
        &self.reduced_mass
    }

    /// Reduced stiffness matrix `T^T K T`.
    #[must_use]
    pub const fn reduced_stiffness(&self) -> &DenseMatrix {
        &self.reduced_stiffness
    }

    /// Internal DOF indices in the original full-order model.
    #[must_use]
    pub fn internal_dofs(&self) -> &[usize] {
        &self.internal_dofs
    }

    /// Boundary DOF indices in the original full-order model.
    #[must_use]
    pub fn boundary_dofs(&self) -> &[usize] {
        &self.boundary_dofs
    }
}

/// Extract sparse fixed-interface modes for the internal partition.
///
/// The solver performs deterministic shift-invert inverse iteration on the
/// internal generalized eigenproblem `K_ii φ = λ M_ii φ`. The returned dense
/// mode matrix is row-aligned with [`FixedInterfaceModes::internal_dofs`] and
/// can be passed directly to [`reduce_craig_bampton`].
///
/// # Errors
///
/// Returns [`StructDynError`] when sparse matrices are dimensionally invalid,
/// boundary DOFs are invalid, the internal mass or shifted stiffness is not
/// positive definite, or a mode does not converge within `max_iterations`.
pub fn extract_fixed_interface_modes_shift_invert(
    mass: &SparseSymmetricMatrix,
    stiffness: &SparseSymmetricMatrix,
    boundary_dofs: &[usize],
    options: FixedInterfaceExtractionOptions,
) -> Result<FixedInterfaceModes, StructDynError> {
    validate_sparse_pair(mass, stiffness)?;
    let boundary_dofs = validate_boundary_dofs(boundary_dofs, mass.size())?;
    let internal_dofs = internal_dofs(mass.size(), &boundary_dofs);
    if internal_dofs.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "boundary_dofs",
            reason: "at least one internal DOF is required",
        });
    }
    let options = options.validate(internal_dofs.len())?;
    let mass_ii = mass.submatrix(&internal_dofs)?;
    let stiffness_ii = stiffness.submatrix(&internal_dofs)?;
    let mass_dense = mass_ii.to_dense()?;
    DenseCholesky::factor(&mass_dense, "M_ii")?;
    let shifted = shifted_dense(&stiffness_ii, &mass_ii, options.shift_rad_s2)?;
    let shifted_factor = DenseCholesky::factor(&shifted, "K_ii - shift*M_ii")?;

    let mut extracted = Vec::with_capacity(options.mode_count);
    let mut eigenvalues_rad_s2 = Vec::with_capacity(options.mode_count);
    let mut residual_norms = Vec::with_capacity(options.mode_count);
    let mut iterations = Vec::with_capacity(options.mode_count);
    for mode_index in 0..options.mode_count {
        let mut vector = initial_trial(mode_index, mass_ii.size(), &extracted, &mass_ii)?;
        let mut converged = false;
        let mut iteration_count = 0;
        let mut eigenvalue = 0.0;
        let mut residual_norm = f64::INFINITY;
        for iteration in 1..=options.max_iterations {
            let rhs = mass_ii.mul_vec(&vector)?;
            let mut candidate = shifted_factor.solve(&rhs)?;
            mass_orthogonalize(&mut candidate, &extracted, &mass_ii)?;
            mass_normalize(&mut candidate, &mass_ii)?;
            eigenvalue = rayleigh_quotient(&candidate, &stiffness_ii, &mass_ii)?;
            residual_norm =
                fixed_interface_residual_norm(&candidate, eigenvalue, &stiffness_ii, &mass_ii)?;
            vector = candidate;
            iteration_count = iteration;
            if residual_norm <= options.tolerance {
                converged = true;
                break;
            }
        }
        if !converged {
            return Err(StructDynError::InvalidMatrix {
                field: "fixed_interface.iteration",
                reason: "shift-invert mode did not converge",
            });
        }
        extracted.push(vector);
        eigenvalues_rad_s2.push(eigenvalue);
        residual_norms.push(residual_norm);
        iterations.push(iteration_count);
    }

    let mut modes = DenseMatrix::zeros(internal_dofs.len(), options.mode_count)?;
    for (mode_col, vector) in extracted.iter().enumerate() {
        for (row, value) in vector.iter().enumerate() {
            modes.set(row, mode_col, *value);
        }
    }

    Ok(FixedInterfaceModes {
        modes,
        eigenvalues_rad_s2,
        residual_norms,
        iterations,
        internal_dofs,
        boundary_dofs,
    })
}

/// Compute a dense Craig-Bampton reduction.
///
/// `fixed_interface_modes`, when provided, has one row per internal DOF and
/// one column per retained fixed-interface mode. Boundary generalized
/// coordinates are appended after those modal coordinates in the transform.
///
/// # Errors
///
/// Returns [`StructDynError`] when matrices are not conforming square matrices,
/// boundary DOFs are invalid/duplicated, fixed-interface mode dimensions do not
/// match the internal partition, or the fixed-interface stiffness block is not
/// positive definite.
pub fn reduce_craig_bampton(
    mass: &DenseMatrix,
    stiffness: &DenseMatrix,
    boundary_dofs: &[usize],
    fixed_interface_modes: Option<&DenseMatrix>,
) -> Result<CraigBamptonReduction, StructDynError> {
    validate_square_pair(mass, stiffness)?;
    let full_dofs = mass.rows();
    let boundary_dofs = validate_boundary_dofs(boundary_dofs, full_dofs)?;
    let internal_dofs = internal_dofs(full_dofs, &boundary_dofs);
    if internal_dofs.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "boundary_dofs",
            reason: "at least one internal DOF is required",
        });
    }
    if let Some(modes) = fixed_interface_modes
        && modes.rows() != internal_dofs.len()
    {
        return Err(StructDynError::DimensionMismatch {
            field: "fixed_interface_modes.rows",
            expected: internal_dofs.len(),
            actual: modes.rows(),
        });
    }

    let k_ii = submatrix(stiffness, &internal_dofs, &internal_dofs)?;
    let k_ib = submatrix(stiffness, &internal_dofs, &boundary_dofs)?;
    let k_ii_factor = DenseCholesky::factor(&k_ii, "K_ii")?;
    let fixed_mode_count = fixed_interface_modes.map_or(0, DenseMatrix::cols);
    let reduced_dofs = fixed_mode_count + boundary_dofs.len();
    let mut transform = DenseMatrix::zeros(full_dofs, reduced_dofs)?;

    if let Some(modes) = fixed_interface_modes {
        for (internal_row, full_row) in internal_dofs.iter().enumerate() {
            for mode_col in 0..fixed_mode_count {
                transform.set(*full_row, mode_col, modes.get(internal_row, mode_col));
            }
        }
    }

    for (boundary_col, boundary_dof) in boundary_dofs.iter().enumerate() {
        let rhs = (0..internal_dofs.len())
            .map(|row| -k_ib.get(row, boundary_col))
            .collect::<Vec<_>>();
        let constraint = k_ii_factor.solve(&rhs)?;
        for (internal_row, full_row) in internal_dofs.iter().enumerate() {
            transform.set(
                *full_row,
                fixed_mode_count + boundary_col,
                constraint[internal_row],
            );
        }
        transform.set(*boundary_dof, fixed_mode_count + boundary_col, 1.0);
    }

    let mass_times_transform = mass.mul_matrix(&transform)?;
    let stiffness_times_transform = stiffness.mul_matrix(&transform)?;
    let reduced_mass = transform.transpose_mul_matrix(&mass_times_transform)?;
    let reduced_stiffness = transform.transpose_mul_matrix(&stiffness_times_transform)?;

    Ok(CraigBamptonReduction {
        transform,
        reduced_mass,
        reduced_stiffness,
        internal_dofs,
        boundary_dofs,
    })
}

fn validate_square_pair(mass: &DenseMatrix, stiffness: &DenseMatrix) -> Result<(), StructDynError> {
    if mass.rows() != mass.cols() {
        return Err(StructDynError::DimensionMismatch {
            field: "mass.cols",
            expected: mass.rows(),
            actual: mass.cols(),
        });
    }
    if stiffness.rows() != stiffness.cols() {
        return Err(StructDynError::DimensionMismatch {
            field: "stiffness.cols",
            expected: stiffness.rows(),
            actual: stiffness.cols(),
        });
    }
    if mass.rows() != stiffness.rows() {
        return Err(StructDynError::DimensionMismatch {
            field: "stiffness.rows",
            expected: mass.rows(),
            actual: stiffness.rows(),
        });
    }
    Ok(())
}

fn validate_sparse_pair(
    mass: &SparseSymmetricMatrix,
    stiffness: &SparseSymmetricMatrix,
) -> Result<(), StructDynError> {
    if mass.size() != stiffness.size() {
        return Err(StructDynError::DimensionMismatch {
            field: "sparse.stiffness.size",
            expected: mass.size(),
            actual: stiffness.size(),
        });
    }
    Ok(())
}

fn validate_boundary_dofs(
    boundary_dofs: &[usize],
    full_dofs: usize,
) -> Result<Vec<usize>, StructDynError> {
    if boundary_dofs.is_empty() {
        return Err(StructDynError::InvalidMatrix {
            field: "boundary_dofs",
            reason: "at least one boundary DOF is required",
        });
    }
    let mut seen = BTreeSet::new();
    for dof in boundary_dofs {
        if *dof >= full_dofs {
            return Err(StructDynError::DimensionMismatch {
                field: "boundary_dofs",
                expected: full_dofs,
                actual: *dof + 1,
            });
        }
        if !seen.insert(*dof) {
            return Err(StructDynError::InvalidMatrix {
                field: "boundary_dofs",
                reason: "duplicate boundary DOF",
            });
        }
    }
    Ok(boundary_dofs.to_vec())
}

fn internal_dofs(full_dofs: usize, boundary_dofs: &[usize]) -> Vec<usize> {
    (0..full_dofs)
        .filter(|candidate| !boundary_dofs.contains(candidate))
        .collect()
}

fn submatrix(
    matrix: &DenseMatrix,
    rows: &[usize],
    cols: &[usize],
) -> Result<DenseMatrix, StructDynError> {
    let mut out = DenseMatrix::zeros(rows.len(), cols.len())?;
    for (row_index, row) in rows.iter().enumerate() {
        for (col_index, col) in cols.iter().enumerate() {
            out.set(row_index, col_index, matrix.get(*row, *col));
        }
    }
    Ok(out)
}

fn shifted_dense(
    stiffness: &SparseSymmetricMatrix,
    mass: &SparseSymmetricMatrix,
    shift_rad_s2: f64,
) -> Result<DenseMatrix, StructDynError> {
    validate_sparse_pair(mass, stiffness)?;
    let mut dense = stiffness.to_dense()?;
    for entry in mass.entries() {
        dense.set(
            entry.row,
            entry.col,
            dense.get(entry.row, entry.col) - shift_rad_s2 * entry.value,
        );
        if entry.row != entry.col {
            dense.set(
                entry.col,
                entry.row,
                dense.get(entry.col, entry.row) - shift_rad_s2 * entry.value,
            );
        }
    }
    Ok(dense)
}

fn initial_trial(
    mode_index: usize,
    size: usize,
    previous_modes: &[Vec<f64>],
    mass: &SparseSymmetricMatrix,
) -> Result<Vec<f64>, StructDynError> {
    for offset in 0..size {
        let mut trial = vec![0.0; size];
        trial[(mode_index + offset) % size] = 1.0;
        mass_orthogonalize(&mut trial, previous_modes, mass)?;
        if mass_norm_squared(&trial, mass)? > 1.0e-24 {
            mass_normalize(&mut trial, mass)?;
            return Ok(trial);
        }
    }
    Err(StructDynError::InvalidMatrix {
        field: "fixed_interface.trial",
        reason: "could not construct M-orthogonal trial vector",
    })
}

fn mass_orthogonalize(
    vector: &mut [f64],
    previous_modes: &[Vec<f64>],
    mass: &SparseSymmetricMatrix,
) -> Result<(), StructDynError> {
    for previous in previous_modes {
        let alpha = mass_inner(previous, vector, mass)?;
        for (value, previous_value) in vector.iter_mut().zip(previous) {
            *value -= alpha * *previous_value;
        }
    }
    Ok(())
}

fn mass_normalize(vector: &mut [f64], mass: &SparseSymmetricMatrix) -> Result<(), StructDynError> {
    let norm_squared = mass_norm_squared(vector, mass)?;
    if !norm_squared.is_finite() || norm_squared <= 0.0 {
        return Err(StructDynError::NonPositiveDefinite {
            field: "M_ii",
            index: 0,
        });
    }
    let scale = norm_squared.sqrt();
    for value in vector {
        *value /= scale;
    }
    Ok(())
}

fn mass_norm_squared(vector: &[f64], mass: &SparseSymmetricMatrix) -> Result<f64, StructDynError> {
    mass_inner(vector, vector, mass)
}

fn mass_inner(
    left: &[f64],
    right: &[f64],
    mass: &SparseSymmetricMatrix,
) -> Result<f64, StructDynError> {
    if left.len() != right.len() {
        return Err(StructDynError::DimensionMismatch {
            field: "fixed_interface.inner",
            expected: left.len(),
            actual: right.len(),
        });
    }
    let mass_right = mass.mul_vec(right)?;
    Ok(left
        .iter()
        .zip(mass_right.iter())
        .map(|(left_value, right_value)| *left_value * *right_value)
        .sum())
}

fn rayleigh_quotient(
    vector: &[f64],
    stiffness: &SparseSymmetricMatrix,
    mass: &SparseSymmetricMatrix,
) -> Result<f64, StructDynError> {
    let stiffness_vector = stiffness.mul_vec(vector)?;
    let mass_vector = mass.mul_vec(vector)?;
    let numerator: f64 = vector
        .iter()
        .zip(stiffness_vector.iter())
        .map(|(value, stiffness_value)| *value * *stiffness_value)
        .sum();
    let denominator: f64 = vector
        .iter()
        .zip(mass_vector.iter())
        .map(|(value, mass_value)| *value * *mass_value)
        .sum();
    if !denominator.is_finite() || denominator <= 0.0 {
        return Err(StructDynError::NonPositiveDefinite {
            field: "M_ii",
            index: 0,
        });
    }
    Ok(numerator / denominator)
}

fn fixed_interface_residual_norm(
    vector: &[f64],
    eigenvalue: f64,
    stiffness: &SparseSymmetricMatrix,
    mass: &SparseSymmetricMatrix,
) -> Result<f64, StructDynError> {
    let stiffness_vector = stiffness.mul_vec(vector)?;
    let mass_vector = mass.mul_vec(vector)?;
    let residual_squared: f64 = stiffness_vector
        .iter()
        .zip(mass_vector.iter())
        .map(|(stiffness_value, mass_value)| {
            let residual = *stiffness_value - eigenvalue * *mass_value;
            residual * residual
        })
        .sum();
    Ok(residual_squared.sqrt())
}

fn validate_shape(field: &'static str, values: &[f64]) -> Result<(), StructDynError> {
    if values.is_empty() {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected: 1,
            actual: 0,
        });
    }
    if !values.iter().all(|value| value.is_finite()) {
        return Err(StructDynError::InvalidNumber { field });
    }
    if euclidean_norm_squared(values) <= 0.0 {
        return Err(StructDynError::InvalidMatrix {
            field,
            reason: "shape vector must have nonzero norm",
        });
    }
    Ok(())
}

fn validate_modal_basis_pair(
    field: &'static str,
    left_modes: &DenseMatrix,
    right_modes: &DenseMatrix,
) -> Result<(), StructDynError> {
    if left_modes.rows() != right_modes.rows() {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected: left_modes.rows(),
            actual: right_modes.rows(),
        });
    }
    if left_modes.cols() == 0 {
        return Err(StructDynError::DimensionMismatch {
            field: "modal_basis.left_modes",
            expected: 1,
            actual: 0,
        });
    }
    if right_modes.cols() == 0 {
        return Err(StructDynError::DimensionMismatch {
            field: "modal_basis.right_modes",
            expected: 1,
            actual: 0,
        });
    }
    for col in 0..left_modes.cols() {
        validate_shape("modal_basis.left_shape", &matrix_column(left_modes, col))?;
    }
    for col in 0..right_modes.cols() {
        validate_shape("modal_basis.right_shape", &matrix_column(right_modes, col))?;
    }
    Ok(())
}

fn matrix_column(matrix: &DenseMatrix, col: usize) -> Vec<f64> {
    (0..matrix.rows()).map(|row| matrix.get(row, col)).collect()
}

fn mass_weighted_inner(left: &[f64], mass: &DenseMatrix, right: &[f64]) -> f64 {
    let mut sum = 0.0;
    for (row, left_value) in left.iter().enumerate() {
        for (col, right_value) in right.iter().enumerate() {
            sum += *left_value * mass.get(row, col) * *right_value;
        }
    }
    sum
}

fn euclidean_norm_squared(values: &[f64]) -> f64 {
    values.iter().map(|value| *value * *value).sum()
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    const SYNTHETIC_CB_BENCHMARK: &str =
        include_str!("../../../data/modal/synthetic-calculix-cb-benchmark-v1.csv");
    const CB_BENCHMARK_SCHEMA: &str = "openbmp.structural_cb_modal_benchmark.csv.v1";

    fn two_dof_mass() -> DenseMatrix {
        DenseMatrix::identity(2).unwrap()
    }

    fn two_dof_stiffness() -> DenseMatrix {
        DenseMatrix::from_row_major(2, 2, vec![6.0, -2.0, -2.0, 3.0]).unwrap()
    }

    fn entry(row: usize, col: usize, value: f64) -> SparseMatrixEntry {
        SparseMatrixEntry { row, col, value }
    }

    fn sparse_three_dof_mass() -> SparseSymmetricMatrix {
        SparseSymmetricMatrix::new(
            3,
            vec![entry(0, 0, 2.0), entry(1, 1, 1.0), entry(2, 2, 1.0)],
        )
        .unwrap()
    }

    fn sparse_three_dof_stiffness() -> SparseSymmetricMatrix {
        SparseSymmetricMatrix::new(
            3,
            vec![
                entry(0, 0, 8.0),
                entry(1, 1, 9.0),
                entry(2, 2, 10.0),
                entry(0, 2, -2.0),
                entry(1, 2, -3.0),
            ],
        )
        .unwrap()
    }

    fn fixed_options(mode_count: usize) -> FixedInterfaceExtractionOptions {
        FixedInterfaceExtractionOptions {
            mode_count,
            shift_rad_s2: 0.0,
            tolerance: 1.0e-12,
            max_iterations: 8,
        }
    }

    fn parse_synthetic_cb_benchmark() -> Vec<ModalBenchmarkMode> {
        let mut rows = SYNTHETIC_CB_BENCHMARK.lines();
        assert_eq!(
            rows.next().unwrap(),
            "schema,id,frequency_hz,shape_internal_dof_0,shape_internal_dof_1"
        );
        rows.map(|line| {
            let fields = line.split(',').collect::<Vec<_>>();
            assert_eq!(fields.len(), 5);
            assert_eq!(fields[0], CB_BENCHMARK_SCHEMA);
            ModalBenchmarkMode::new(
                fields[1],
                fields[2].parse::<f64>().unwrap(),
                vec![
                    fields[3].parse::<f64>().unwrap(),
                    fields[4].parse::<f64>().unwrap(),
                ],
            )
            .unwrap()
        })
        .collect()
    }

    fn benchmark_shape_matrix(references: &[ModalBenchmarkMode]) -> DenseMatrix {
        let row_count = references[0].shape().len();
        let col_count = references.len();
        let mut data = vec![0.0; row_count * col_count];
        for (col, reference) in references.iter().enumerate() {
            for (row, value) in reference.shape().iter().enumerate() {
                data[row * col_count + col] = *value;
            }
        }
        DenseMatrix::from_row_major(row_count, col_count, data).unwrap()
    }

    #[test]
    fn cb_constraint_mode_recovers_static_boundary_stiffness() {
        let reduction =
            reduce_craig_bampton(&two_dof_mass(), &two_dof_stiffness(), &[1], None).unwrap();
        assert_eq!(reduction.internal_dofs(), &[0]);
        assert_eq!(reduction.boundary_dofs(), &[1]);
        assert_eq!(reduction.transform().rows(), 2);
        assert_eq!(reduction.transform().cols(), 1);
        assert!((reduction.transform().get(0, 0) - (1.0 / 3.0)).abs() < 1.0e-15);
        assert_eq!(reduction.transform().get(1, 0).to_bits(), 1.0_f64.to_bits());
        assert!((reduction.reduced_stiffness().get(0, 0) - (7.0 / 3.0)).abs() < 1.0e-15);
    }

    #[test]
    fn cb_transform_keeps_fixed_interface_mode_and_boundary_coordinate() {
        let fixed_mode = DenseMatrix::from_row_major(1, 1, vec![1.0]).unwrap();
        let reduction = reduce_craig_bampton(
            &two_dof_mass(),
            &two_dof_stiffness(),
            &[1],
            Some(&fixed_mode),
        )
        .unwrap();

        let transform = reduction.transform().as_row_major();
        assert_eq!(transform[0].to_bits(), 1.0_f64.to_bits());
        assert!((transform[1] - (1.0 / 3.0)).abs() < 1.0e-15);
        assert_eq!(transform[2].to_bits(), 0.0_f64.to_bits());
        assert_eq!(transform[3].to_bits(), 1.0_f64.to_bits());
        assert!((reduction.reduced_stiffness().get(0, 0) - 6.0).abs() < 1.0e-15);
        assert!(reduction.reduced_stiffness().get(0, 1).abs() < 1.0e-15);
        assert!(reduction.reduced_stiffness().get(1, 0).abs() < 1.0e-15);
        assert!((reduction.reduced_stiffness().get(1, 1) - (7.0 / 3.0)).abs() < 1.0e-15);
        assert!((reduction.reduced_mass().get(0, 1) - (1.0 / 3.0)).abs() < 1.0e-15);
    }

    #[test]
    fn sparse_shift_invert_extracts_mass_orthonormal_fixed_interface_modes() {
        let modes = extract_fixed_interface_modes_shift_invert(
            &sparse_three_dof_mass(),
            &sparse_three_dof_stiffness(),
            &[2],
            fixed_options(2),
        )
        .unwrap();

        assert_eq!(modes.internal_dofs(), &[0, 1]);
        assert_eq!(modes.boundary_dofs(), &[2]);
        assert_eq!(modes.modes().rows(), 2);
        assert_eq!(modes.modes().cols(), 2);
        assert!((modes.eigenvalues_rad_s2()[0] - 4.0).abs() < 1.0e-12);
        assert!((modes.eigenvalues_rad_s2()[1] - 9.0).abs() < 1.0e-12);
        assert!(
            modes
                .residual_norms()
                .iter()
                .all(|residual| *residual <= 1.0e-12)
        );
        assert_eq!(modes.iterations(), &[1, 1]);

        let phi = modes.modes();
        let mass_ii = sparse_three_dof_mass()
            .submatrix(modes.internal_dofs())
            .unwrap();
        let first = [phi.get(0, 0), phi.get(1, 0)];
        let second = [phi.get(0, 1), phi.get(1, 1)];
        assert!((mass_inner(&first, &first, &mass_ii).unwrap() - 1.0).abs() < 1.0e-12);
        assert!((mass_inner(&second, &second, &mass_ii).unwrap() - 1.0).abs() < 1.0e-12);
        assert!(mass_inner(&first, &second, &mass_ii).unwrap().abs() < 1.0e-12);
    }

    #[test]
    fn sparse_fixed_interface_modes_feed_cb_reduction() {
        let extracted = extract_fixed_interface_modes_shift_invert(
            &sparse_three_dof_mass(),
            &sparse_three_dof_stiffness(),
            &[2],
            fixed_options(2),
        )
        .unwrap();
        let mass =
            DenseMatrix::from_row_major(3, 3, vec![2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0])
                .unwrap();
        let stiffness = DenseMatrix::from_row_major(
            3,
            3,
            vec![8.0, 0.0, -2.0, 0.0, 9.0, -3.0, -2.0, -3.0, 10.0],
        )
        .unwrap();
        let reduction =
            reduce_craig_bampton(&mass, &stiffness, &[2], Some(extracted.modes())).unwrap();

        assert_eq!(reduction.transform().rows(), 3);
        assert_eq!(reduction.transform().cols(), 3);
        assert!((reduction.transform().get(0, 0) - 2.0_f64.sqrt().recip()).abs() < 1.0e-12);
        assert_eq!(reduction.transform().get(1, 0).to_bits(), 0.0_f64.to_bits());
        assert_eq!(reduction.transform().get(0, 1).to_bits(), 0.0_f64.to_bits());
        assert_eq!(reduction.transform().get(1, 1).to_bits(), 1.0_f64.to_bits());
        assert!((reduction.transform().get(0, 2) - 0.25).abs() < 1.0e-12);
        assert!((reduction.transform().get(1, 2) - (1.0 / 3.0)).abs() < 1.0e-12);
        assert_eq!(reduction.transform().get(2, 2).to_bits(), 1.0_f64.to_bits());
        assert!((reduction.reduced_stiffness().get(0, 0) - 4.0).abs() < 1.0e-12);
        assert!((reduction.reduced_stiffness().get(1, 1) - 9.0).abs() < 1.0e-12);
    }

    #[test]
    fn cb_frequency_mac_benchmark_fixture_matches_sparse_extraction() {
        let extracted = extract_fixed_interface_modes_shift_invert(
            &sparse_three_dof_mass(),
            &sparse_three_dof_stiffness(),
            &[2],
            fixed_options(2),
        )
        .unwrap();
        let correlations =
            correlate_fixed_interface_modes(&extracted, &parse_synthetic_cb_benchmark(), 0.01, 0.9)
                .unwrap();

        assert_eq!(correlations.len(), 2);
        assert_eq!(correlations[0].mode_id(), "cb_mode_1");
        assert_eq!(correlations[1].mode_id(), "cb_mode_2");
        assert!(
            correlations
                .iter()
                .all(|correlation| correlation.frequency_error_fraction() < 1.0e-14)
        );
        assert!(
            correlations
                .iter()
                .all(|correlation| correlation.mac() > 0.999_999_999_999)
        );
    }

    #[test]
    fn cb_benchmark_matrix_evidence_reports_mac_and_cross_orthogonality() {
        let extracted = extract_fixed_interface_modes_shift_invert(
            &sparse_three_dof_mass(),
            &sparse_three_dof_stiffness(),
            &[2],
            fixed_options(2),
        )
        .unwrap();
        let references = parse_synthetic_cb_benchmark();
        let reference_shapes = benchmark_shape_matrix(&references);
        let mac = modal_assurance_matrix(extracted.modes(), &reference_shapes).unwrap();

        assert_eq!(mac.rows(), 2);
        assert_eq!(mac.cols(), 2);
        assert!((mac.get(0, 0) - 1.0).abs() < 1.0e-14);
        assert!(mac.get(0, 1) < 1.0e-14);
        assert!(mac.get(1, 0) < 1.0e-14);
        assert!((mac.get(1, 1) - 1.0).abs() < 1.0e-14);

        let mass_ii = sparse_three_dof_mass()
            .submatrix(extracted.internal_dofs())
            .unwrap()
            .to_dense()
            .unwrap();
        let cross =
            modal_cross_orthogonality_matrix(extracted.modes(), &reference_shapes, &mass_ii)
                .unwrap();

        assert_eq!(cross.rows(), 2);
        assert_eq!(cross.cols(), 2);
        assert!((cross.get(0, 0) - 1.0).abs() < 1.0e-14);
        assert!(cross.get(0, 1).abs() < 1.0e-14);
        assert!(cross.get(1, 0).abs() < 1.0e-14);
        assert!((cross.get(1, 1) - 1.0).abs() < 1.0e-14);
    }

    #[test]
    fn cb_benchmark_matrix_evidence_fails_closed_on_bad_shapes() {
        let left = DenseMatrix::from_row_major(2, 1, vec![1.0, 0.0]).unwrap();
        let right = DenseMatrix::from_row_major(1, 1, vec![1.0]).unwrap();
        assert!(modal_assurance_matrix(&left, &right).is_err());

        let empty_modes = DenseMatrix::from_row_major(2, 0, vec![]).unwrap();
        assert!(modal_assurance_matrix(&empty_modes, &left).is_err());

        let mass_wrong = DenseMatrix::identity(3).unwrap();
        assert!(modal_cross_orthogonality_matrix(&left, &left, &mass_wrong).is_err());

        let zero_shape = DenseMatrix::from_row_major(2, 1, vec![0.0, 0.0]).unwrap();
        assert!(modal_assurance_matrix(&left, &zero_shape).is_err());
    }

    #[test]
    fn cb_frequency_mac_benchmark_fails_closed_on_reference_drift() {
        let extracted = extract_fixed_interface_modes_shift_invert(
            &sparse_three_dof_mass(),
            &sparse_three_dof_stiffness(),
            &[2],
            fixed_options(2),
        )
        .unwrap();
        let mut references = parse_synthetic_cb_benchmark();
        references[0] = ModalBenchmarkMode::new("cb_mode_1_drifted", 0.4, vec![0.0, 1.0]).unwrap();

        assert!(correlate_fixed_interface_modes(&extracted, &references, 0.01, 0.9).is_err());
        assert!(modal_assurance_criterion(&[1.0, 0.0], &[0.0]).is_err());
        assert!(ModalBenchmarkMode::new("", 1.0, vec![1.0]).is_err());
    }

    #[test]
    fn cb_rejects_invalid_boundary_and_singular_internal_stiffness() {
        assert!(reduce_craig_bampton(&two_dof_mass(), &two_dof_stiffness(), &[], None).is_err());
        assert!(reduce_craig_bampton(&two_dof_mass(), &two_dof_stiffness(), &[2], None).is_err());
        assert!(
            reduce_craig_bampton(&two_dof_mass(), &two_dof_stiffness(), &[1, 1], None).is_err()
        );

        let singular = DenseMatrix::from_row_major(2, 2, vec![0.0, 0.0, 0.0, 1.0]).unwrap();
        assert!(reduce_craig_bampton(&two_dof_mass(), &singular, &[1], None).is_err());
    }

    #[test]
    fn sparse_fixed_interface_extraction_rejects_invalid_inputs() {
        assert!(SparseSymmetricMatrix::new(2, vec![entry(0, 1, 1.0), entry(1, 0, 1.0)]).is_err());
        assert!(SparseSymmetricMatrix::new(2, vec![entry(0, 0, f64::NAN)]).is_err());

        let mass = sparse_three_dof_mass();
        let stiffness = sparse_three_dof_stiffness();
        assert!(
            extract_fixed_interface_modes_shift_invert(&mass, &stiffness, &[2], fixed_options(0))
                .is_err()
        );
        assert!(
            extract_fixed_interface_modes_shift_invert(&mass, &stiffness, &[2], fixed_options(3))
                .is_err()
        );
        assert!(
            extract_fixed_interface_modes_shift_invert(
                &mass,
                &stiffness,
                &[2],
                FixedInterfaceExtractionOptions {
                    mode_count: 1,
                    shift_rad_s2: 5.0,
                    tolerance: 1.0e-12,
                    max_iterations: 8,
                },
            )
            .is_err()
        );

        let coupled = SparseSymmetricMatrix::new(
            3,
            vec![
                entry(0, 0, 8.0),
                entry(1, 1, 9.0),
                entry(0, 1, 1.0),
                entry(2, 2, 10.0),
            ],
        )
        .unwrap();
        assert!(
            extract_fixed_interface_modes_shift_invert(
                &mass,
                &coupled,
                &[2],
                FixedInterfaceExtractionOptions {
                    mode_count: 1,
                    shift_rad_s2: 0.0,
                    tolerance: 1.0e-30,
                    max_iterations: 1,
                },
            )
            .is_err()
        );
    }
}
