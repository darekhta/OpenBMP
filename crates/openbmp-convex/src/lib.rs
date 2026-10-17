//! Portable deterministic convex-optimization helpers.
//!
//! The crate is a low-layer home for conic-solver settings and small CSC
//! assembly utilities shared by solver-backed flight-controller primitives and
//! host-side trajectory optimization. It deliberately carries no dependency on
//! `openbmp-fc`, `openbmp-trajopt`, the simulator, scenarios, or runner crates.

#![cfg_attr(not(feature = "std"), no_std)]
#![forbid(unsafe_code)]

extern crate alloc;

#[cfg(feature = "conic")]
use alloc::vec;
#[cfg(feature = "conic")]
use alloc::vec::Vec;

#[cfg(feature = "conic")]
use clarabel::{algebra::CscMatrix, solver::DefaultSettings};

/// Clarabel settings type used by deterministic OpenBMP conic/QP solves.
#[cfg(feature = "conic")]
pub type ClarabelSettings = DefaultSettings<f64>;

/// Error raised while assembling a Clarabel CSC matrix.
#[cfg(feature = "conic")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ConvexMatrixError {
    /// A row entry referenced a column outside the configured matrix width.
    ColumnOutOfBounds {
        /// Referenced column index.
        column: usize,
        /// Configured matrix column count.
        cols: usize,
    },
    /// A row entry carried NaN or infinity.
    NonFiniteValue,
}

/// Row-oriented helper for deterministic Clarabel CSC matrix assembly.
#[cfg(feature = "conic")]
#[derive(Clone, Debug, PartialEq)]
pub struct ClarabelCscMatrixBuilder {
    cols: usize,
    rows: usize,
    triplets: Vec<(usize, usize, f64)>,
}

#[cfg(feature = "conic")]
impl ClarabelCscMatrixBuilder {
    /// Create an empty row-oriented CSC builder with `cols` columns.
    #[must_use]
    pub fn new(cols: usize) -> Self {
        Self {
            cols,
            rows: 0,
            triplets: Vec::new(),
        }
    }

    /// Number of rows appended so far.
    #[must_use]
    pub fn row_count(&self) -> usize {
        self.rows
    }

    /// Append one row from `(column, value)` entries.
    ///
    /// Zero-valued entries are skipped. Non-finite values and out-of-range
    /// columns fail closed before the row count is advanced.
    ///
    /// # Errors
    ///
    /// Returns [`ConvexMatrixError`] when an entry is non-finite or references
    /// a column outside the configured matrix width.
    pub fn try_push_row(&mut self, entries: &[(usize, f64)]) -> Result<(), ConvexMatrixError> {
        for &(column, value) in entries {
            if column >= self.cols {
                return Err(ConvexMatrixError::ColumnOutOfBounds {
                    column,
                    cols: self.cols,
                });
            }
            if !value.is_finite() {
                return Err(ConvexMatrixError::NonFiniteValue);
            }
        }
        for &(column, value) in entries {
            if value != 0.0 {
                self.triplets.push((self.rows, column, value));
            }
        }
        self.rows += 1;
        Ok(())
    }

    /// Materialize the accumulated rows as a Clarabel CSC matrix.
    #[must_use]
    pub fn finish(self) -> CscMatrix<f64> {
        let mut columns = vec![Vec::new(); self.cols];
        for (row, column, value) in self.triplets {
            columns[column].push((row, value));
        }
        let mut colptr = Vec::with_capacity(self.cols + 1);
        let mut rowval = Vec::new();
        let mut nzval = Vec::new();
        colptr.push(0);
        for column in columns {
            for (row, value) in column {
                rowval.push(row);
                nzval.push(value);
            }
            colptr.push(nzval.len());
        }
        CscMatrix::new(self.rows, self.cols, colptr, rowval, nzval)
    }
}

/// Returns the deterministic Clarabel settings profile shared by OpenBMP conic
/// and QP solves.
#[cfg(feature = "conic")]
#[must_use]
pub fn deterministic_clarabel_settings() -> ClarabelSettings {
    DefaultSettings::<f64> {
        verbose: false,
        max_iter: 100,
        time_limit: f64::INFINITY,
        tol_gap_abs: 1.0e-9,
        tol_gap_rel: 1.0e-9,
        tol_feas: 1.0e-9,
        tol_infeas_abs: 1.0e-9,
        tol_infeas_rel: 1.0e-9,
        tol_ktratio: 1.0e-9,
        ..DefaultSettings::<f64>::default()
    }
}

#[cfg(all(test, feature = "conic"))]
mod tests {
    use super::*;

    #[test]
    fn deterministic_clarabel_settings_are_pinned() {
        let settings = deterministic_clarabel_settings();
        assert!(!settings.verbose);
        assert_eq!(settings.max_iter, 100);
        assert!(settings.time_limit.is_infinite() && settings.time_limit.is_sign_positive());
        assert_eq!(settings.tol_gap_abs.to_bits(), 1.0e-9_f64.to_bits());
        assert_eq!(settings.tol_gap_rel.to_bits(), 1.0e-9_f64.to_bits());
        assert_eq!(settings.tol_feas.to_bits(), 1.0e-9_f64.to_bits());
        assert_eq!(settings.tol_infeas_abs.to_bits(), 1.0e-9_f64.to_bits());
        assert_eq!(settings.tol_infeas_rel.to_bits(), 1.0e-9_f64.to_bits());
        assert_eq!(settings.tol_ktratio.to_bits(), 1.0e-9_f64.to_bits());
    }

    #[test]
    fn csc_builder_materializes_column_major_matrix() {
        let mut builder = ClarabelCscMatrixBuilder::new(3);
        assert!(builder.try_push_row(&[(0, 1.0), (2, 3.0)]).is_ok());
        assert!(builder.try_push_row(&[(1, 2.0), (2, 0.0)]).is_ok());

        let matrix = builder.finish();
        assert_eq!(matrix.m, 2);
        assert_eq!(matrix.n, 3);
        assert_eq!(matrix.colptr, vec![0, 1, 2, 3]);
        assert_eq!(matrix.rowval, vec![0, 1, 0]);
        assert_eq!(matrix.nzval, vec![1.0, 2.0, 3.0]);
    }

    #[test]
    fn csc_builder_rejects_bad_rows_without_advancing() {
        let mut builder = ClarabelCscMatrixBuilder::new(2);
        assert_eq!(
            builder.try_push_row(&[(2, 1.0)]),
            Err(ConvexMatrixError::ColumnOutOfBounds { column: 2, cols: 2 })
        );
        assert_eq!(builder.row_count(), 0);
        assert_eq!(
            builder.try_push_row(&[(0, f64::NAN)]),
            Err(ConvexMatrixError::NonFiniteValue)
        );
        assert_eq!(builder.row_count(), 0);
    }
}
