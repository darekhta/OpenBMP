//! Deterministic structural-dynamics numerics.
//!
//! The first shipped module is station line-load recovery for modal structural
//! states. It evaluates declared output rows with a locked-order dense GEMV:
//!
//! ```text
//! y = A_d q + A_a q_ddot + A_ma q_ddot
//! ```
//!
//! where `A_ma` is the optional mode-acceleration residual correction. The
//! Later modules add the same pure-numerics boundary for Craig-Bampton
//! reduction, modal transients, beam/membrane/plate finite elements, declared
//! event forcing with a closed CLA event-class vocabulary, and sampled
//! structural POGO loop margins.
//! The caller owns all scenario/file I/O; this crate only validates dimensions
//! and performs deterministic arithmetic.

#![forbid(unsafe_code)]
#![deny(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::collections::BTreeSet;

pub mod cb;
pub mod events;
pub mod fem;
pub mod pogo;
pub mod transient;

/// Dense row-major matrix used by structural-dynamics kernels.
#[derive(Clone, Debug, PartialEq)]
pub struct DenseMatrix {
    rows: usize,
    cols: usize,
    data: Vec<f64>,
}

impl DenseMatrix {
    /// Construct a dense row-major matrix.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the row count is zero, the data length
    /// does not match `rows * cols`, or any value is non-finite.
    pub fn from_row_major(
        rows: usize,
        cols: usize,
        data: Vec<f64>,
    ) -> Result<Self, StructDynError> {
        if rows == 0 {
            return Err(StructDynError::DimensionMismatch {
                field: "matrix.rows",
                expected: 1,
                actual: 0,
            });
        }
        let expected = rows
            .checked_mul(cols)
            .ok_or(StructDynError::InvalidMatrix {
                field: "matrix",
                reason: "row/column product overflow",
            })?;
        if data.len() != expected {
            return Err(StructDynError::DimensionMismatch {
                field: "matrix.data",
                expected,
                actual: data.len(),
            });
        }
        require_all_finite("matrix.data", &data)?;
        Ok(Self { rows, cols, data })
    }

    /// Construct a zero matrix.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions overflow.
    pub fn zeros(rows: usize, cols: usize) -> Result<Self, StructDynError> {
        let len = rows
            .checked_mul(cols)
            .ok_or(StructDynError::InvalidMatrix {
                field: "matrix",
                reason: "row/column product overflow",
            })?;
        Self::from_row_major(rows, cols, vec![0.0; len])
    }

    /// Construct an identity matrix.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when `size == 0`.
    pub fn identity(size: usize) -> Result<Self, StructDynError> {
        let mut matrix = Self::zeros(size, size)?;
        for index in 0..size {
            matrix.set(index, index, 1.0);
        }
        Ok(matrix)
    }

    /// Row count.
    #[must_use]
    pub const fn rows(&self) -> usize {
        self.rows
    }

    /// Column count.
    #[must_use]
    pub const fn cols(&self) -> usize {
        self.cols
    }

    /// Backing row-major values.
    #[must_use]
    pub fn as_row_major(&self) -> &[f64] {
        &self.data
    }

    /// Read one matrix entry.
    #[must_use]
    pub fn get(&self, row: usize, col: usize) -> f64 {
        self.data[row * self.cols + col]
    }

    /// Set one matrix entry.
    pub fn set(&mut self, row: usize, col: usize, value: f64) {
        self.data[row * self.cols + col] = value;
    }

    /// Matrix-vector product in locked row-major order.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the vector length does not match.
    pub fn mul_vec(&self, values: &[f64]) -> Result<Vec<f64>, StructDynError> {
        if values.len() != self.cols {
            return Err(StructDynError::DimensionMismatch {
                field: "matrix.vector",
                expected: self.cols,
                actual: values.len(),
            });
        }
        require_all_finite("matrix.vector", values)?;
        Ok(mul_vec_unchecked(self, values))
    }

    /// Matrix-matrix product in locked row-major order.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when dimensions do not conform.
    pub fn mul_matrix(&self, rhs: &Self) -> Result<Self, StructDynError> {
        if self.cols != rhs.rows {
            return Err(StructDynError::DimensionMismatch {
                field: "matrix.mul",
                expected: self.cols,
                actual: rhs.rows,
            });
        }
        let mut out = Self::zeros(self.rows, rhs.cols)?;
        for row in 0..self.rows {
            for col in 0..rhs.cols {
                let mut sum = 0.0;
                for k in 0..self.cols {
                    sum += self.get(row, k) * rhs.get(k, col);
                }
                out.set(row, col, sum);
            }
        }
        Ok(out)
    }

    /// Product `self^T * rhs` in locked row-major order.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when row counts do not conform.
    pub fn transpose_mul_matrix(&self, rhs: &Self) -> Result<Self, StructDynError> {
        if self.rows != rhs.rows {
            return Err(StructDynError::DimensionMismatch {
                field: "matrix.transpose_mul",
                expected: self.rows,
                actual: rhs.rows,
            });
        }
        let mut out = Self::zeros(self.cols, rhs.cols)?;
        for row in 0..self.cols {
            for col in 0..rhs.cols {
                let mut sum = 0.0;
                for k in 0..self.rows {
                    sum += self.get(k, row) * rhs.get(k, col);
                }
                out.set(row, col, sum);
            }
        }
        Ok(out)
    }

    fn checked_square(&self, field: &'static str) -> Result<(), StructDynError> {
        if self.rows != self.cols {
            return Err(StructDynError::DimensionMismatch {
                field,
                expected: self.rows,
                actual: self.cols,
            });
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq)]
pub(crate) struct DenseCholesky {
    size: usize,
    lower: Vec<f64>,
}

impl DenseCholesky {
    pub(crate) fn factor(
        matrix: &DenseMatrix,
        field: &'static str,
    ) -> Result<Self, StructDynError> {
        matrix.checked_square(field)?;
        let size = matrix.rows;
        let mut lower = vec![0.0; size * size];
        for row in 0..size {
            for col in 0..=row {
                let mut sum = matrix.get(row, col);
                for k in 0..col {
                    sum -= lower[row * size + k] * lower[col * size + k];
                }
                if row == col {
                    if !sum.is_finite() || sum <= 0.0 {
                        return Err(StructDynError::NonPositiveDefinite { field, index: row });
                    }
                    lower[row * size + col] = sum.sqrt();
                } else {
                    lower[row * size + col] = sum / lower[col * size + col];
                }
            }
        }
        Ok(Self { size, lower })
    }

    pub(crate) fn solve(&self, rhs: &[f64]) -> Result<Vec<f64>, StructDynError> {
        if rhs.len() != self.size {
            return Err(StructDynError::DimensionMismatch {
                field: "cholesky.rhs",
                expected: self.size,
                actual: rhs.len(),
            });
        }
        require_all_finite("cholesky.rhs", rhs)?;
        let mut y = vec![0.0; self.size];
        for row in 0..self.size {
            let mut sum = rhs[row];
            for (col, y_col) in y.iter().take(row).enumerate() {
                sum -= self.lower[row * self.size + col] * *y_col;
            }
            y[row] = sum / self.lower[row * self.size + row];
        }
        let mut x = vec![0.0; self.size];
        for row in (0..self.size).rev() {
            let mut sum = y[row];
            for (col, x_col) in x.iter().enumerate().skip(row + 1) {
                sum -= self.lower[col * self.size + row] * *x_col;
            }
            x[row] = sum / self.lower[row * self.size + row];
        }
        Ok(x)
    }
}

pub(crate) fn add_scaled_matrix(
    out: &mut DenseMatrix,
    matrix: &DenseMatrix,
    scale: f64,
) -> Result<(), StructDynError> {
    if out.rows != matrix.rows || out.cols != matrix.cols {
        return Err(StructDynError::DimensionMismatch {
            field: "matrix.add_scaled",
            expected: out.rows * out.cols,
            actual: matrix.rows * matrix.cols,
        });
    }
    if !scale.is_finite() {
        return Err(StructDynError::InvalidNumber {
            field: "matrix.scale",
        });
    }
    for (out_value, matrix_value) in out.data.iter_mut().zip(&matrix.data) {
        *out_value += scale * *matrix_value;
    }
    Ok(())
}

pub(crate) fn mul_vec_unchecked(matrix: &DenseMatrix, values: &[f64]) -> Vec<f64> {
    let mut out = vec![0.0; matrix.rows];
    for (row, out_value) in out.iter_mut().enumerate() {
        let mut sum = 0.0;
        for (col, value) in values.iter().enumerate().take(matrix.cols) {
            sum += matrix.get(row, col) * *value;
        }
        *out_value = sum;
    }
    out
}

/// Structural line-load output vocabulary.
#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum OutputId {
    /// Axial station force, N.
    AxialForce,
    /// Body-x shear force, N.
    ShearX,
    /// Body-y shear force, N.
    ShearY,
    /// Bending moment about body x, N*m.
    BendingX,
    /// Bending moment about body y, N*m.
    BendingY,
    /// Torsional moment about the vehicle axis, N*m.
    Torsion,
}

impl OutputId {
    /// Stable snake-case telemetry suffix.
    #[must_use]
    pub const fn as_str(self) -> &'static str {
        match self {
            Self::AxialForce => "axial_force_n",
            Self::ShearX => "shear_x_n",
            Self::ShearY => "shear_y_n",
            Self::BendingX => "bending_x_n_m",
            Self::BendingY => "bending_y_n_m",
            Self::Torsion => "torsion_n_m",
        }
    }

    /// Engineering unit string for telemetry metadata.
    #[must_use]
    pub const fn unit(self) -> &'static str {
        match self {
            Self::AxialForce | Self::ShearX | Self::ShearY => "N",
            Self::BendingX | Self::BendingY | Self::Torsion => "N*m",
        }
    }
}

/// One station output sample.
#[derive(Clone, Debug, PartialEq)]
pub struct SectionLoad {
    /// Station id declared by the caller.
    pub station_id: String,
    /// Structural output kind.
    pub output_id: OutputId,
    /// Recovered engineering value.
    pub value: f64,
}

/// Modal coordinates and accelerations in caller-defined locked order.
#[derive(Clone, Debug, PartialEq)]
pub struct ModalRecoveryState {
    /// Modal coordinates.
    pub q: Vec<f64>,
    /// Modal accelerations.
    pub q_ddot: Vec<f64>,
}

impl ModalRecoveryState {
    /// Construct and validate a modal recovery state.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the two vectors have different lengths,
    /// are empty, or contain non-finite values.
    pub fn new(q: Vec<f64>, q_ddot: Vec<f64>) -> Result<Self, StructDynError> {
        if q.is_empty() {
            return Err(StructDynError::DimensionMismatch {
                field: "q",
                expected: 1,
                actual: 0,
            });
        }
        if q.len() != q_ddot.len() {
            return Err(StructDynError::DimensionMismatch {
                field: "q_ddot",
                expected: q.len(),
                actual: q_ddot.len(),
            });
        }
        require_all_finite("q", &q)?;
        require_all_finite("q_ddot", &q_ddot)?;
        Ok(Self { q, q_ddot })
    }

    /// Modal vector length.
    #[must_use]
    pub fn len(&self) -> usize {
        self.q.len()
    }

    /// Whether the state is empty. A constructed state is never empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.q.is_empty()
    }
}

/// One output row at one recovery station.
#[derive(Clone, Debug, PartialEq)]
pub struct RecoveryRow {
    /// Output kind.
    pub output_id: OutputId,
    displacement_coefficients: Vec<f64>,
    acceleration_coefficients: Vec<f64>,
    mode_acceleration_coefficients: Vec<f64>,
}

impl RecoveryRow {
    /// Construct and validate one output row.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when any coefficient vector has the wrong
    /// length or contains non-finite values.
    pub fn new(
        output_id: OutputId,
        displacement_coefficients: Vec<f64>,
        acceleration_coefficients: Vec<f64>,
        mode_acceleration_coefficients: Vec<f64>,
        coordinate_count: usize,
    ) -> Result<Self, StructDynError> {
        validate_coefficients(
            "displacement_coefficients",
            &displacement_coefficients,
            coordinate_count,
        )?;
        validate_coefficients(
            "acceleration_coefficients",
            &acceleration_coefficients,
            coordinate_count,
        )?;
        validate_coefficients(
            "mode_acceleration_coefficients",
            &mode_acceleration_coefficients,
            coordinate_count,
        )?;
        Ok(Self {
            output_id,
            displacement_coefficients,
            acceleration_coefficients,
            mode_acceleration_coefficients,
        })
    }

    fn evaluate(&self, state: &ModalRecoveryState, include_mode_acceleration: bool) -> f64 {
        let displacement = dense_dot(&self.displacement_coefficients, &state.q);
        let acceleration = dense_dot(&self.acceleration_coefficients, &state.q_ddot);
        let correction = if include_mode_acceleration {
            dense_dot(&self.mode_acceleration_coefficients, &state.q_ddot)
        } else {
            0.0
        };
        displacement + acceleration + correction
    }
}

/// One recovery station.
#[derive(Clone, Debug, PartialEq)]
pub struct RecoveryStation {
    /// Station id.
    pub id: String,
    /// Axial station coordinate, m.
    pub axial_m: f64,
    rows: Vec<RecoveryRow>,
}

impl RecoveryStation {
    /// Construct and validate one station.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] for an empty id, non-finite station location,
    /// empty output rows, or duplicate output ids at the station.
    pub fn new(id: String, axial_m: f64, rows: Vec<RecoveryRow>) -> Result<Self, StructDynError> {
        if id.trim().is_empty() {
            return Err(StructDynError::InvalidString {
                field: "station.id",
            });
        }
        if !axial_m.is_finite() {
            return Err(StructDynError::InvalidNumber {
                field: "station.axial_m",
            });
        }
        if rows.is_empty() {
            return Err(StructDynError::EmptyRows { station_id: id });
        }
        let mut seen = BTreeSet::new();
        for row in &rows {
            if !seen.insert(row.output_id) {
                return Err(StructDynError::DuplicateOutput {
                    station_id: id,
                    output_id: row.output_id,
                });
            }
        }
        Ok(Self { id, axial_m, rows })
    }

    /// Output rows in declaration order.
    #[must_use]
    pub fn rows(&self) -> &[RecoveryRow] {
        &self.rows
    }
}

/// Deterministic station line-load recovery matrix.
#[derive(Clone, Debug, PartialEq)]
pub struct LineLoadRecovery {
    coordinate_count: usize,
    include_mode_acceleration: bool,
    stations: Vec<RecoveryStation>,
}

impl LineLoadRecovery {
    /// Construct and validate a line-load recovery evaluator.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] for zero coordinate count, no stations, or
    /// duplicate station ids.
    pub fn new(
        coordinate_count: usize,
        include_mode_acceleration: bool,
        stations: Vec<RecoveryStation>,
    ) -> Result<Self, StructDynError> {
        if coordinate_count == 0 {
            return Err(StructDynError::DimensionMismatch {
                field: "coordinate_count",
                expected: 1,
                actual: 0,
            });
        }
        if stations.is_empty() {
            return Err(StructDynError::EmptyStations);
        }
        let mut seen = BTreeSet::new();
        for station in &stations {
            if !seen.insert(station.id.as_str()) {
                return Err(StructDynError::DuplicateStation {
                    station_id: station.id.clone(),
                });
            }
        }
        Ok(Self {
            coordinate_count,
            include_mode_acceleration,
            stations,
        })
    }

    /// Modal coordinate count expected by this recovery matrix.
    #[must_use]
    pub const fn coordinate_count(&self) -> usize {
        self.coordinate_count
    }

    /// Recovery stations in declaration order.
    #[must_use]
    pub fn stations(&self) -> &[RecoveryStation] {
        &self.stations
    }

    /// Evaluate all station outputs in declaration order.
    ///
    /// # Errors
    ///
    /// Returns [`StructDynError`] when the state dimension does not match or
    /// a recovered value is non-finite.
    pub fn evaluate(&self, state: &ModalRecoveryState) -> Result<Vec<SectionLoad>, StructDynError> {
        if state.len() != self.coordinate_count {
            return Err(StructDynError::DimensionMismatch {
                field: "state",
                expected: self.coordinate_count,
                actual: state.len(),
            });
        }
        let mut loads = Vec::new();
        for station in &self.stations {
            for row in station.rows() {
                let value = row.evaluate(state, self.include_mode_acceleration);
                if !value.is_finite() {
                    return Err(StructDynError::InvalidNumber {
                        field: "section_load.value",
                    });
                }
                loads.push(SectionLoad {
                    station_id: station.id.clone(),
                    output_id: row.output_id,
                    value,
                });
            }
        }
        Ok(loads)
    }
}

/// Error type for structural dynamics numerics.
#[derive(Clone, Debug, PartialEq, thiserror::Error)]
pub enum StructDynError {
    /// A string field was empty.
    #[error("{field} must be non-empty")]
    InvalidString {
        /// Field name.
        field: &'static str,
    },
    /// A numeric field was invalid.
    #[error("{field} must be finite")]
    InvalidNumber {
        /// Field name.
        field: &'static str,
    },
    /// A vector dimension did not match the expected modal coordinate count.
    #[error("{field} dimension mismatch: expected {expected}, got {actual}")]
    DimensionMismatch {
        /// Field name.
        field: &'static str,
        /// Expected length.
        expected: usize,
        /// Actual length.
        actual: usize,
    },
    /// Matrix content was structurally invalid.
    #[error("{field} is invalid: {reason}")]
    InvalidMatrix {
        /// Field name.
        field: &'static str,
        /// Human-readable rule.
        reason: &'static str,
    },
    /// A symmetric matrix was not positive definite.
    #[error("{field} is not positive definite at pivot {index}")]
    NonPositiveDefinite {
        /// Field name.
        field: &'static str,
        /// Failed diagonal pivot index.
        index: usize,
    },
    /// No recovery stations were declared.
    #[error("line-load recovery requires at least one station")]
    EmptyStations,
    /// A station had no output rows.
    #[error("line-load recovery station `{station_id}` requires at least one output row")]
    EmptyRows {
        /// Station id.
        station_id: String,
    },
    /// Duplicate station id.
    #[error("duplicate line-load recovery station `{station_id}`")]
    DuplicateStation {
        /// Station id.
        station_id: String,
    },
    /// Duplicate output kind for one station.
    #[error("duplicate output `{output_id:?}` at station `{station_id}`")]
    DuplicateOutput {
        /// Station id.
        station_id: String,
        /// Output id.
        output_id: OutputId,
    },
}

fn validate_coefficients(
    field: &'static str,
    coefficients: &[f64],
    coordinate_count: usize,
) -> Result<(), StructDynError> {
    if coefficients.len() != coordinate_count {
        return Err(StructDynError::DimensionMismatch {
            field,
            expected: coordinate_count,
            actual: coefficients.len(),
        });
    }
    require_all_finite(field, coefficients)
}

fn require_all_finite(field: &'static str, values: &[f64]) -> Result<(), StructDynError> {
    if values.iter().all(|value| value.is_finite()) {
        Ok(())
    } else {
        Err(StructDynError::InvalidNumber { field })
    }
}

fn dense_dot(coefficients: &[f64], values: &[f64]) -> f64 {
    coefficients
        .iter()
        .zip(values)
        .fold(0.0, |sum, (coefficient, value)| {
            sum + (*coefficient * *value)
        })
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

    fn row(output_id: OutputId, ma: [f64; 2]) -> RecoveryRow {
        RecoveryRow::new(output_id, vec![2.0, -1.0], vec![10.0, 5.0], ma.to_vec(), 2).unwrap()
    }

    #[test]
    fn mode_acceleration_correction_closes_synthetic_static_recovery() {
        let station = RecoveryStation::new(
            "interstage".to_owned(),
            12.0,
            vec![row(OutputId::ShearX, [1.5, -0.5])],
        )
        .unwrap();
        let with_correction = LineLoadRecovery::new(2, true, vec![station.clone()]).unwrap();
        let without_correction = LineLoadRecovery::new(2, false, vec![station]).unwrap();
        let state = ModalRecoveryState::new(vec![0.25, -0.10], vec![3.0, -2.0]).unwrap();

        let corrected = with_correction.evaluate(&state).unwrap()[0].value;
        let uncorrected = without_correction.evaluate(&state).unwrap()[0].value;
        let full_fem_static_reference = 26.10;

        let corrected_error =
            ((corrected - full_fem_static_reference) / full_fem_static_reference).abs();
        let uncorrected_error =
            ((uncorrected - full_fem_static_reference) / full_fem_static_reference).abs();

        assert!(
            corrected_error < 0.02,
            "mode-acceleration correction must land within 2%, got {corrected_error}"
        );
        assert!(
            uncorrected_error > 0.10,
            "uncorrected recovery should document the residual error, got {uncorrected_error}"
        );
    }

    #[test]
    fn evaluates_outputs_in_station_declaration_order() {
        let recovery = LineLoadRecovery::new(
            2,
            true,
            vec![
                RecoveryStation::new(
                    "aft".to_owned(),
                    0.0,
                    vec![
                        row(OutputId::ShearY, [0.0, 1.0]),
                        row(OutputId::BendingX, [1.0, 0.0]),
                    ],
                )
                .unwrap(),
                RecoveryStation::new(
                    "forward".to_owned(),
                    5.0,
                    vec![row(OutputId::Torsion, [0.5, 0.5])],
                )
                .unwrap(),
            ],
        )
        .unwrap();
        let state = ModalRecoveryState::new(vec![1.0, 2.0], vec![0.5, -0.25]).unwrap();
        let loads = recovery.evaluate(&state).unwrap();

        assert_eq!(loads[0].station_id, "aft");
        assert_eq!(loads[0].output_id, OutputId::ShearY);
        assert_eq!(loads[1].station_id, "aft");
        assert_eq!(loads[1].output_id, OutputId::BendingX);
        assert_eq!(loads[2].station_id, "forward");
        assert_eq!(loads[2].output_id, OutputId::Torsion);
    }

    #[test]
    fn rejects_bad_dimensions_and_duplicates() {
        assert!(ModalRecoveryState::new(Vec::new(), Vec::new()).is_err());
        assert!(ModalRecoveryState::new(vec![1.0], vec![1.0, 2.0]).is_err());
        assert!(RecoveryRow::new(OutputId::ShearX, vec![1.0], vec![0.0], vec![0.0], 2).is_err());

        let duplicate = RecoveryStation::new(
            "s".to_owned(),
            0.0,
            vec![
                row(OutputId::ShearX, [0.0, 0.0]),
                row(OutputId::ShearX, [1.0, 1.0]),
            ],
        );
        assert!(duplicate.is_err());

        let station =
            RecoveryStation::new("s".to_owned(), 0.0, vec![row(OutputId::ShearX, [0.0, 0.0])])
                .unwrap();
        assert!(LineLoadRecovery::new(2, true, vec![station.clone(), station]).is_err());
    }
}
