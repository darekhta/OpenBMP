//! UD covariance factorization helpers for the `square-root-ekf`
//! feature.
//!
//! Bierman's UD representation stores a symmetric positive-definite
//! covariance as a unit-triangular factor and a diagonal scale. The
//! default EKF path keeps the full Joseph-form covariance; this
//! feature exists for adopters who need the factorized representation
//! for conditioning audits.

use nalgebra::SMatrix;

/// Fixed-size UD factorization.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UdFactor<const N: usize> {
    /// Unit upper-triangular factor stored so `P = Uᵀ D U`.
    pub u: SMatrix<f64, N, N>,
    /// Diagonal entries.
    pub d: [f64; N],
}

impl<const N: usize> UdFactor<N> {
    /// Factorizes a symmetric positive-definite covariance matrix.
    ///
    /// # Errors
    ///
    /// Returns an error if a non-positive or non-finite pivot is
    /// encountered.
    pub fn factorize(p: &SMatrix<f64, N, N>) -> Result<Self, &'static str> {
        let mut l = SMatrix::<f64, N, N>::zeros();
        let mut d = [0.0_f64; N];
        for i in 0..N {
            for j in 0..i {
                let mut sum = p[(i, j)];
                for k in 0..j {
                    sum -= l[(i, k)] * l[(j, k)] * d[k];
                }
                l[(i, j)] = sum / d[j];
            }
            let mut diag = p[(i, i)];
            for k in 0..i {
                diag -= l[(i, k)] * l[(i, k)] * d[k];
            }
            if !diag.is_finite() || diag <= 0.0 {
                return Err("covariance is not positive definite");
            }
            d[i] = diag;
            l[(i, i)] = 1.0;
        }
        Ok(Self {
            u: l.transpose(),
            d,
        })
    }

    /// Reconstructs `P = Uᵀ D U`.
    #[must_use]
    pub fn reconstruct(&self) -> SMatrix<f64, N, N> {
        let d_matrix =
            SMatrix::<f64, N, N>::from_diagonal(&nalgebra::SVector::from_row_slice(&self.d));
        self.u.transpose() * d_matrix * self.u
    }
}

#[cfg(test)]
#[allow(clippy::expect_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn ud_factorization_reconstructs_well_conditioned_covariance() {
        let p = SMatrix::<f64, 3, 3>::new(4.0, 0.2, 0.1, 0.2, 3.0, 0.4, 0.1, 0.4, 2.0);
        let factor = UdFactor::<3>::factorize(&p).expect("positive definite covariance");
        let reconstructed = factor.reconstruct();
        for r in 0..3 {
            for c in 0..3 {
                assert_abs_diff_eq!(reconstructed[(r, c)], p[(r, c)], epsilon = 1.0e-12);
            }
        }
    }
}
