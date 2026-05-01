//! Numerical-method primitives for filter / detector consumers.
//!
//! Pure deterministic functions that compute statistical inverses
//! used to derive innovation gates from a stated false-alarm rate.
//! No physical units, no scenario state, no allocation. All callers
//! across the workspace (FC estimators, FDIR detectors, sim-side
//! property tests, scenario validation gates) consume from here so
//! the operand order and rational-approximation coefficients are not
//! reinvented per consumer.
//!
//! # References
//!
//! * Wilson, E. B., & Hilferty, M. M. (1931). "The distribution of
//!   chi-square." *Proc. Natl. Acad. Sci.* 17(12):684–688. The
//!   transformation used here approximates `χ²_n` as
//!   `n · (1 − 2/9n + Z · √(2/9n))³` for `Z` standard-normal.
//! * Acklam, P. J. (2003). "An algorithm for computing the inverse
//!   normal cumulative distribution function." Used for the
//!   standard-normal inverse step.

/// Returns the inverse of the chi-square cumulative distribution
/// function at probability `p` for `dof` degrees of freedom, using
/// the Wilson-Hilferty (1931) cube-root approximation. Returns
/// `f64::INFINITY` for non-finite or non-positive inputs to keep
/// downstream consumers fail-loud rather than silently using a
/// wrong gate.
#[must_use]
pub fn chi_square_inverse_cdf_wilson_hilferty(probability: f64, dof: f64) -> f64 {
    if !probability.is_finite() || !dof.is_finite() || dof <= 0.0 {
        return f64::INFINITY;
    }
    let z = inverse_standard_normal_cdf(probability);
    let a = 2.0 / (9.0 * dof);
    dof * (1.0 - a + z * a.sqrt()).powi(3)
}

/// Inverse of the standard-normal CDF (Acklam 2003 rational
/// approximation). Coefficients are dimensionless numerical-method
/// constants, not physical data — they are the published Acklam
/// table verbatim.
#[must_use]
pub fn inverse_standard_normal_cdf(probability: f64) -> f64 {
    const A: [f64; 6] = [
        -3.969_683_028_665_376e1,
        2.209_460_984_245_205e2,
        -2.759_285_104_469_687e2,
        1.383_577_518_672_69e2,
        -3.066_479_806_614_716e1,
        2.506_628_277_459_239,
    ];
    const B: [f64; 5] = [
        -5.447_609_879_822_406e1,
        1.615_858_368_580_409e2,
        -1.556_989_798_598_866e2,
        6.680_131_188_771_972e1,
        -1.328_068_155_288_572e1,
    ];
    const C: [f64; 6] = [
        -7.784_894_002_430_293e-3,
        -3.223_964_580_411_365e-1,
        -2.400_758_277_161_838,
        -2.549_732_539_343_734,
        4.374_664_141_464_968,
        2.938_163_982_698_783,
    ];
    const D: [f64; 4] = [
        7.784_695_709_041_462e-3,
        3.224_671_290_700_398e-1,
        2.445_134_137_142_996,
        3.754_408_661_907_416,
    ];
    const P_LOW: f64 = 0.024_25;
    const P_HIGH: f64 = 1.0 - P_LOW;

    let p = probability.clamp(f64::MIN_POSITIVE, 1.0 - f64::EPSILON);
    if p < P_LOW {
        let q = (-2.0 * p.ln()).sqrt();
        return (((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    if p > P_HIGH {
        let q = (-2.0 * (1.0 - p).ln()).sqrt();
        return -(((((C[0] * q + C[1]) * q + C[2]) * q + C[3]) * q + C[4]) * q + C[5])
            / ((((D[0] * q + D[1]) * q + D[2]) * q + D[3]) * q + 1.0);
    }
    let q = p - 0.5;
    let r = q * q;
    (((((A[0] * r + A[1]) * r + A[2]) * r + A[3]) * r + A[4]) * r + A[5]) * q
        / (((((B[0] * r + B[1]) * r + B[2]) * r + B[3]) * r + B[4]) * r + 1.0)
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    #[test]
    fn chi_square_inverse_known_3dof_99pct() {
        // Tabulated value: chi²(0.99, 3) ≈ 11.345. Wilson-Hilferty
        // approximation gets within ~0.2 % at 3 DOF.
        let v = chi_square_inverse_cdf_wilson_hilferty(0.99, 3.0);
        assert_abs_diff_eq!(v, 11.345, epsilon = 0.05);
    }

    #[test]
    fn chi_square_inverse_known_6dof_99pct() {
        // Tabulated: chi²(0.99, 6) ≈ 16.812. Wilson-Hilferty: same
        // tolerance.
        let v = chi_square_inverse_cdf_wilson_hilferty(0.99, 6.0);
        assert_abs_diff_eq!(v, 16.812, epsilon = 0.05);
    }

    #[test]
    fn chi_square_inverse_returns_infinity_on_invalid() {
        assert!(chi_square_inverse_cdf_wilson_hilferty(f64::NAN, 3.0).is_infinite());
        assert!(chi_square_inverse_cdf_wilson_hilferty(0.5, 0.0).is_infinite());
        assert!(chi_square_inverse_cdf_wilson_hilferty(0.5, -1.0).is_infinite());
    }

    #[test]
    fn standard_normal_inverse_at_median_is_zero() {
        let z = inverse_standard_normal_cdf(0.5);
        assert_abs_diff_eq!(z, 0.0, epsilon = 1.0e-9);
    }

    #[test]
    fn standard_normal_inverse_one_sigma_known_value() {
        // Z(0.8413) ≈ 1.0
        let z = inverse_standard_normal_cdf(0.841_344_746_068_543);
        assert_abs_diff_eq!(z, 1.0, epsilon = 1.0e-6);
    }

    #[test]
    fn standard_normal_inverse_is_antisymmetric_around_median() {
        let z_pos = inverse_standard_normal_cdf(0.95);
        let z_neg = inverse_standard_normal_cdf(0.05);
        assert_abs_diff_eq!(z_pos, -z_neg, epsilon = 1.0e-9);
    }
}
