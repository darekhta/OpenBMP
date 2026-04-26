//! Aerodynamic deck (Schema 1, axisymmetric reduced sounding-rocket
//! deck) and locked-order trilinear interpolation.
//!
//! # Schema
//!
//! Phase 2.5 ships **Schema 1**: a three-axis tabulated deck indexed
//! by `(mach, alpha_deg, beta_deg)` and producing the three reduced
//! coefficients `(CN, CD, CM)`. The semantics are pinned by
//! `docs/phase-2-plan.md § Implementation Seams Locked Before Coding`:
//!
//! * `CN` — normal-force coefficient in the wind / body longitudinal
//!   plane.
//! * `CD` — drag coefficient (opposes relative wind).
//! * `CM` — pitching-moment coefficient about the body lateral axis;
//!   roll moment is identically zero by axisymmetry.
//!
//! The full six-coefficient `CX/CY/CZ/Cl/Cm/Cn` deck and
//! control-effector axes are deferred to Phase 3.
//!
//! # Storage layout
//!
//! Each coefficient table is a flat `Vec<f64>` of length
//! `n_mach · n_alpha · n_beta`, stored in **row-major** order over
//! `(mach, alpha, beta)` so that `beta` varies fastest. The index
//! map is:
//!
//! ```text
//! index(im, ia, ib) = im · (n_alpha · n_beta) + ia · n_beta + ib
//! ```
//!
//! This places `beta` in the innermost reduction slot of the
//! trilinear lookup, which the locked operand order below is keyed to.
//!
//! # Trilinear interpolation
//!
//! Locked operand order per Demmel & Nguyen 2020 (*Algorithms for
//! Efficient Reproducible Floating Point Summation*, ACM TOMS 46:3):
//!
//! ```text
//! v00 = (1-c) · c000 + c · c001
//! v01 = (1-c) · c010 + c · c011
//! v10 = (1-c) · c100 + c · c101
//! v11 = (1-c) · c110 + c · c111
//! v0  = (1-b) · v00  + b · v01
//! v1  = (1-b) · v10  + b · v11
//! r   = (1-a) · v0   + a · v1
//! ```
//!
//! where `a = mach-fraction`, `b = alpha-fraction`,
//! `c = beta-fraction` (innermost). FMA is disabled — the formula
//! is written so each term reads as a single `Mul + Add` instruction
//! the compiler cannot collapse to `mul_add`.
//!
//! # Out-of-grid behaviour
//!
//! Default is [`ExtrapolationPolicy::FailClosed`]: a query outside
//! the grid returns [`AeroError::OutOfEnvelope`]. Decks may opt into
//! [`ExtrapolationPolicy::Clamp`] which snaps an off-grid query to
//! the nearest grid face. Linear extrapolation is intentionally not
//! offered — the OpenRocket / Niskanen 2009 corpus shows linear
//! extrapolation produces nonsensical drag for high Mach decks.

use crate::error::AeroError;

// ---------------------------------------------------------------------
// AeroCoefficients
// ---------------------------------------------------------------------

/// The three reduced coefficients returned by an aerodynamic deck
/// lookup. Schema 1.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct AeroCoefficients {
    /// Normal-force coefficient (dimensionless).
    pub cn: f64,
    /// Drag coefficient (dimensionless).
    pub cd: f64,
    /// Pitching-moment coefficient (dimensionless).
    pub cm: f64,
}

// ---------------------------------------------------------------------
// ExtrapolationPolicy
// ---------------------------------------------------------------------

/// Out-of-grid behaviour for an [`AeroDeck`] lookup.
#[derive(Copy, Clone, Debug, Default, Eq, PartialEq)]
pub enum ExtrapolationPolicy {
    /// Default. Returns [`AeroError::OutOfEnvelope`] when the query
    /// is outside the grid envelope on any axis.
    #[default]
    FailClosed,
    /// Snaps an off-grid query to the nearest grid face on the
    /// offending axis (zero gradient outside). Documented as opt-in
    /// only because linear extrapolation of aero coefficients past
    /// the validated grid is a common source of silent error.
    Clamp,
}

// ---------------------------------------------------------------------
// AeroDeck
// ---------------------------------------------------------------------

/// Schema-1 tabulated aerodynamic deck.
///
/// Constructed via [`AeroDeck::new`] with strictly monotone-increasing
/// axes, finite values throughout, and coefficient tables sized to
/// the product of the three axis lengths. The Phase-2.5.B TOML parser
/// builds an `AeroDeck` from a deck file; this constructor is the
/// in-memory entry point used by tests and by the parser internally.
#[derive(Clone, Debug, PartialEq)]
pub struct AeroDeck {
    mach: Vec<f64>,
    alpha_deg: Vec<f64>,
    beta_deg: Vec<f64>,
    cn: Vec<f64>,
    cd: Vec<f64>,
    cm: Vec<f64>,
    reference_area_m2: f64,
    reference_length_m: f64,
    extrapolation: ExtrapolationPolicy,
}

impl AeroDeck {
    /// Construct an in-memory deck and validate its structural and
    /// finiteness invariants.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::MalformedDeck`] for an empty axis, a
    /// non-monotone axis, a duplicated breakpoint, or a coefficient
    /// table whose length does not equal `n_mach · n_alpha · n_beta`.
    /// Returns [`AeroError::NonFinite`] for any `NaN` / `Inf` value
    /// in an axis, in a coefficient table, or in `reference_*`.
    /// Returns [`AeroError::InvalidParameter`] when a `reference_*`
    /// is non-positive.
    #[allow(clippy::too_many_arguments)] // mirrors the deck-file schema; the TOML parser is the primary user.
    pub fn new(
        mach: Vec<f64>,
        alpha_deg: Vec<f64>,
        beta_deg: Vec<f64>,
        cn: Vec<f64>,
        cd: Vec<f64>,
        cm: Vec<f64>,
        reference_area_m2: f64,
        reference_length_m: f64,
    ) -> Result<Self, AeroError> {
        validate_axis(&mach, "mach axis")?;
        validate_axis(&alpha_deg, "alpha axis")?;
        validate_axis(&beta_deg, "beta axis")?;
        let expected = mach.len() * alpha_deg.len() * beta_deg.len();
        validate_table(&cn, expected, "CN table")?;
        validate_table(&cd, expected, "CD table")?;
        validate_table(&cm, expected, "CM table")?;
        validate_reference(reference_area_m2, "reference area")?;
        validate_reference(reference_length_m, "reference length")?;
        Ok(Self {
            mach,
            alpha_deg,
            beta_deg,
            cn,
            cd,
            cm,
            reference_area_m2,
            reference_length_m,
            extrapolation: ExtrapolationPolicy::FailClosed,
        })
    }

    /// Set the deck's extrapolation policy. Builder-style.
    #[must_use]
    pub fn with_extrapolation_policy(mut self, policy: ExtrapolationPolicy) -> Self {
        self.extrapolation = policy;
        self
    }

    /// The active extrapolation policy.
    #[must_use]
    pub const fn extrapolation_policy(&self) -> ExtrapolationPolicy {
        self.extrapolation
    }

    /// Reference aerodynamic area (m²).
    #[must_use]
    pub const fn reference_area_m2(&self) -> f64 {
        self.reference_area_m2
    }

    /// Reference length (m); used for the moment coefficient
    /// non-dimensionalisation.
    #[must_use]
    pub const fn reference_length_m(&self) -> f64 {
        self.reference_length_m
    }

    /// Mach grid (read-only access; ordered low-to-high).
    #[must_use]
    pub fn mach_grid(&self) -> &[f64] {
        &self.mach
    }

    /// Angle-of-attack grid in degrees (read-only; ordered low-to-high).
    #[must_use]
    pub fn alpha_grid_deg(&self) -> &[f64] {
        &self.alpha_deg
    }

    /// Side-slip grid in degrees (read-only; ordered low-to-high).
    #[must_use]
    pub fn beta_grid_deg(&self) -> &[f64] {
        &self.beta_deg
    }

    /// Look up the deck at `(mach, alpha_deg, beta_deg)` using
    /// locked-order trilinear interpolation.
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::NonFinite`] when any input is `NaN` or
    /// `Inf`. Returns [`AeroError::OutOfEnvelope`] when the input is
    /// outside the grid on any axis and the deck is configured
    /// [`ExtrapolationPolicy::FailClosed`] (the default).
    #[allow(clippy::similar_names)] // im/ia/ib + im_next/ia_next/ib_next match the trilinear notation.
    pub fn lookup(
        &self,
        mach: f64,
        alpha_deg: f64,
        beta_deg: f64,
    ) -> Result<AeroCoefficients, AeroError> {
        let (im, fm) = bracket(
            &self.mach,
            mach,
            self.extrapolation,
            "mach outside deck envelope",
        )?;
        let (ia, fa) = bracket(
            &self.alpha_deg,
            alpha_deg,
            self.extrapolation,
            "alpha outside deck envelope",
        )?;
        let (ib, fb) = bracket(
            &self.beta_deg,
            beta_deg,
            self.extrapolation,
            "beta outside deck envelope",
        )?;

        // Single-point axes: i_next = i and the corresponding fraction
        // is 0, so the (1 - f)·c[i] + f·c[i_next] reduction degenerates
        // to c[i] without any out-of-bounds access.
        let im_next = if self.mach.len() == 1 { im } else { im + 1 };
        let ia_next = if self.alpha_deg.len() == 1 {
            ia
        } else {
            ia + 1
        };
        let ib_next = if self.beta_deg.len() == 1 { ib } else { ib + 1 };

        let n_beta = self.beta_deg.len();
        let n_ab = self.alpha_deg.len() * n_beta;

        let i000 = im * n_ab + ia * n_beta + ib;
        let i001 = im * n_ab + ia * n_beta + ib_next;
        let i010 = im * n_ab + ia_next * n_beta + ib;
        let i011 = im * n_ab + ia_next * n_beta + ib_next;
        let i100 = im_next * n_ab + ia * n_beta + ib;
        let i101 = im_next * n_ab + ia * n_beta + ib_next;
        let i110 = im_next * n_ab + ia_next * n_beta + ib;
        let i111 = im_next * n_ab + ia_next * n_beta + ib_next;

        let interp = |table: &[f64]| -> f64 {
            // Locked order: c (innermost = beta), then b (alpha),
            // then a (mach). FMA disabled — each `(1 - f) * x + f * y`
            // is two separate fmul + fadd instructions.
            let v00 = (1.0 - fb) * table[i000] + fb * table[i001];
            let v01 = (1.0 - fb) * table[i010] + fb * table[i011];
            let v10 = (1.0 - fb) * table[i100] + fb * table[i101];
            let v11 = (1.0 - fb) * table[i110] + fb * table[i111];
            let v0 = (1.0 - fa) * v00 + fa * v01;
            let v1 = (1.0 - fa) * v10 + fa * v11;
            (1.0 - fm) * v0 + fm * v1
        };

        Ok(AeroCoefficients {
            cn: interp(&self.cn),
            cd: interp(&self.cd),
            cm: interp(&self.cm),
        })
    }
}

// ---------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------

fn validate_axis(axis: &[f64], label: &'static str) -> Result<(), AeroError> {
    if axis.is_empty() {
        return Err(AeroError::MalformedDeck { reason: label });
    }
    for &v in axis {
        if !v.is_finite() {
            return Err(AeroError::NonFinite { reason: label });
        }
    }
    // Strict monotone-increasing. NaN is already excluded by the
    // finiteness check above, so a direct `>=` is well-defined.
    for w in axis.windows(2) {
        if w[1] <= w[0] {
            return Err(AeroError::MalformedDeck { reason: label });
        }
    }
    Ok(())
}

fn validate_table(
    table: &[f64],
    expected_len: usize,
    label: &'static str,
) -> Result<(), AeroError> {
    if table.len() != expected_len {
        return Err(AeroError::MalformedDeck { reason: label });
    }
    for &v in table {
        if !v.is_finite() {
            return Err(AeroError::NonFinite { reason: label });
        }
    }
    Ok(())
}

fn validate_reference(value: f64, label: &'static str) -> Result<(), AeroError> {
    if !value.is_finite() {
        return Err(AeroError::NonFinite { reason: label });
    }
    if value <= 0.0 {
        return Err(AeroError::InvalidParameter { reason: label });
    }
    Ok(())
}

/// Bracket `q` against `axis` and return `(i, fraction)` such that
/// the lookup uses `(1 - fraction) * axis_value[i] + fraction *
/// axis_value[i + 1]` (or, for a single-point axis, returns
/// `(0, 0.0)`).
///
/// The `policy` controls how out-of-grid queries are handled.
fn bracket(
    axis: &[f64],
    q: f64,
    policy: ExtrapolationPolicy,
    out_of_envelope_reason: &'static str,
) -> Result<(usize, f64), AeroError> {
    if !q.is_finite() {
        return Err(AeroError::NonFinite {
            reason: "aero deck lookup query is NaN or infinite",
        });
    }
    let n = axis.len();
    if n == 1 {
        // Single-point axis: exact-equality with the stored point is
        // the only in-envelope query without `Clamp`. NaN is already
        // ruled out above, so the float `==` here is well-defined.
        #[allow(clippy::float_cmp)]
        let exact_match = q == axis[0];
        if exact_match || policy == ExtrapolationPolicy::Clamp {
            return Ok((0, 0.0));
        }
        return Err(AeroError::OutOfEnvelope {
            reason: out_of_envelope_reason,
        });
    }
    let lo = axis[0];
    let hi = axis[n - 1];
    if q < lo {
        return match policy {
            ExtrapolationPolicy::FailClosed => Err(AeroError::OutOfEnvelope {
                reason: out_of_envelope_reason,
            }),
            ExtrapolationPolicy::Clamp => Ok((0, 0.0)),
        };
    }
    if q > hi {
        return match policy {
            ExtrapolationPolicy::FailClosed => Err(AeroError::OutOfEnvelope {
                reason: out_of_envelope_reason,
            }),
            ExtrapolationPolicy::Clamp => Ok((n - 2, 1.0)),
        };
    }
    // q is in [axis[0], axis[n - 1]]. partition_point returns the
    // first index where the predicate is false; here that's the first
    // axis breakpoint strictly greater than q. The bracketing
    // interval is then [upper - 1, upper], clamped so upper - 1 stays
    // a valid lower index even at q == axis[n - 1].
    let upper = axis.partition_point(|&x| x <= q);
    let i_lo = upper.saturating_sub(1).min(n - 2);
    let denom = axis[i_lo + 1] - axis[i_lo];
    let f = (q - axis[i_lo]) / denom;
    Ok((i_lo, f))
}

#[cfg(test)]
#[allow(clippy::expect_used, clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    // -----------------------------------------------------------------
    // Fixtures
    // -----------------------------------------------------------------

    /// 2 x 2 x 2 deck with corner values 0..7 indexed in row-major
    /// (mach, alpha, beta) order. CN, CD, CM are all the same table
    /// for simplicity; tests that need distinct coefficients build
    /// their own.
    fn cube_deck() -> AeroDeck {
        let table: Vec<f64> = (0..8).map(f64::from).collect();
        AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            vec![0.0, 1.0],
            table.clone(),
            table.clone(),
            table,
            1.0,
            1.0,
        )
        .unwrap()
    }

    /// 3 x 3 x 1 deck (single-point beta axis) with CN = mach + alpha,
    /// CD = mach * alpha, CM = mach - alpha. Used for sanity checks
    /// that hit each coefficient channel distinctly.
    fn small_3x3x1_deck() -> AeroDeck {
        let mach = vec![0.0, 1.0, 2.0];
        let alpha = vec![-1.0, 0.0, 1.0];
        let beta = vec![0.0];
        let mut cn = Vec::new();
        let mut cd = Vec::new();
        let mut cm = Vec::new();
        for m in &mach {
            for a in &alpha {
                for _b in &beta {
                    cn.push(m + a);
                    cd.push(m * a);
                    cm.push(m - a);
                }
            }
        }
        AeroDeck::new(mach, alpha, beta, cn, cd, cm, 1.0, 1.0).unwrap()
    }

    // -----------------------------------------------------------------
    // Corner / boundary / centroid identities
    // -----------------------------------------------------------------

    #[test]
    fn corner_lookups_return_stored_value_exactly() {
        let deck = cube_deck();
        // 8 corners with known stored values.
        let cases: [(f64, f64, f64, f64); 8] = [
            (0.0, 0.0, 0.0, 0.0),
            (0.0, 0.0, 1.0, 1.0),
            (0.0, 1.0, 0.0, 2.0),
            (0.0, 1.0, 1.0, 3.0),
            (1.0, 0.0, 0.0, 4.0),
            (1.0, 0.0, 1.0, 5.0),
            (1.0, 1.0, 0.0, 6.0),
            (1.0, 1.0, 1.0, 7.0),
        ];
        for (m, a, b, expected) in cases {
            let r = deck.lookup(m, a, b).unwrap();
            assert_eq!(r.cn.to_bits(), expected.to_bits(), "CN at ({m},{a},{b})");
            assert_eq!(r.cd.to_bits(), expected.to_bits(), "CD at ({m},{a},{b})");
            assert_eq!(r.cm.to_bits(), expected.to_bits(), "CM at ({m},{a},{b})");
        }
    }

    #[test]
    fn centroid_of_cube_returns_average_of_eight_corners() {
        // For corner values 0..7 the average is 28/8 = 3.5, exactly
        // representable in f64. The locked-order reduction with
        // fm = fa = fb = 0.5 produces 0.125 · sum which equals 3.5
        // bit-exactly under f64.
        let deck = cube_deck();
        let r = deck.lookup(0.5, 0.5, 0.5).unwrap();
        assert_eq!(r.cn.to_bits(), 3.5_f64.to_bits());
        assert_eq!(r.cd.to_bits(), 3.5_f64.to_bits());
        assert_eq!(r.cm.to_bits(), 3.5_f64.to_bits());
    }

    #[test]
    fn boundary_lookup_equals_one_dimensional_sub_grid() {
        let deck = cube_deck();
        // Hold mach and alpha at their corner values; vary only beta.
        // Then the trilinear collapses to the 1-D linear interpolant
        // along beta between the two (mach, alpha) corners.
        let m = 0.0;
        let a = 0.0;
        for b_query in [0.0, 0.25, 0.5, 0.75, 1.0] {
            // Stored corners at (0,0,0) = 0 and (0,0,1) = 1.
            let expected = (1.0 - b_query) * 0.0 + b_query * 1.0;
            let r = deck.lookup(m, a, b_query).unwrap();
            assert_eq!(r.cn.to_bits(), expected.to_bits(), "beta = {b_query}");
        }
    }

    #[test]
    fn lookup_recovers_distinct_coefficients_per_channel() {
        let deck = small_3x3x1_deck();
        // Centre point (mach = 1, alpha = 0).
        let r = deck.lookup(1.0, 0.0, 0.0).unwrap();
        assert_abs_diff_eq!(r.cn, 1.0); // 1 + 0
        assert_abs_diff_eq!(r.cd, 0.0); // 1 * 0
        assert_abs_diff_eq!(r.cm, 1.0); // 1 - 0
    }

    // -----------------------------------------------------------------
    // Determinism / bit-stability
    // -----------------------------------------------------------------

    #[test]
    fn lookup_is_bit_stable_across_two_invocations() {
        let deck = cube_deck();
        let queries = [
            (0.0, 0.0, 0.0),
            (0.5, 0.5, 0.5),
            (0.123_456, 0.789_012, 0.345_678),
            (1.0, 1.0, 1.0),
        ];
        for q in queries {
            let a = deck.lookup(q.0, q.1, q.2).unwrap();
            let b = deck.lookup(q.0, q.1, q.2).unwrap();
            assert_eq!(a.cn.to_bits(), b.cn.to_bits());
            assert_eq!(a.cd.to_bits(), b.cd.to_bits());
            assert_eq!(a.cm.to_bits(), b.cm.to_bits());
        }
    }

    #[test]
    fn lookup_is_bit_stable_across_two_deck_clones() {
        let deck1 = cube_deck();
        let deck2 = deck1.clone();
        let q = (0.123_456, 0.789_012, 0.345_678);
        let a = deck1.lookup(q.0, q.1, q.2).unwrap();
        let b = deck2.lookup(q.0, q.1, q.2).unwrap();
        assert_eq!(a.cn.to_bits(), b.cn.to_bits());
        assert_eq!(a.cd.to_bits(), b.cd.to_bits());
        assert_eq!(a.cm.to_bits(), b.cm.to_bits());
    }

    // -----------------------------------------------------------------
    // Out-of-envelope behaviour
    // -----------------------------------------------------------------

    #[test]
    fn fail_closed_below_grid_returns_out_of_envelope() {
        let deck = cube_deck();
        assert!(matches!(
            deck.lookup(-0.1, 0.5, 0.5),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, -0.1, 0.5),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 0.5, -0.1),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn fail_closed_above_grid_returns_out_of_envelope() {
        let deck = cube_deck();
        assert!(matches!(
            deck.lookup(1.1, 0.5, 0.5),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 1.1, 0.5),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 0.5, 1.1),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn clamp_below_grid_returns_first_corner_value() {
        let deck = cube_deck().with_extrapolation_policy(ExtrapolationPolicy::Clamp);
        // Below all three lower bounds → corner (0,0,0) value.
        let r = deck.lookup(-1.0, -1.0, -1.0).unwrap();
        assert_eq!(r.cn.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn clamp_above_grid_returns_last_corner_value() {
        let deck = cube_deck().with_extrapolation_policy(ExtrapolationPolicy::Clamp);
        // Above all three upper bounds → corner (1,1,1) value = 7.
        let r = deck.lookup(2.0, 2.0, 2.0).unwrap();
        assert_eq!(r.cn.to_bits(), 7.0_f64.to_bits());
    }

    #[test]
    fn lookup_rejects_non_finite_query() {
        let deck = cube_deck();
        assert!(matches!(
            deck.lookup(f64::NAN, 0.5, 0.5),
            Err(AeroError::NonFinite { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, f64::INFINITY, 0.5),
            Err(AeroError::NonFinite { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 0.5, f64::NEG_INFINITY),
            Err(AeroError::NonFinite { .. })
        ));
    }

    // -----------------------------------------------------------------
    // Single-point axis (beta = [0])
    // -----------------------------------------------------------------

    #[test]
    fn single_point_beta_axis_handles_lookup_at_the_point() {
        let deck = small_3x3x1_deck();
        let r = deck.lookup(0.5, 0.5, 0.0).unwrap();
        // At (m=0.5, a=0.5, b=0): CN = m + a = 1.0 (linear in the
        // 4 corners (0,0)=0, (0,1)=1, (1,0)=1, (1,1)=2 → centre 1.0).
        assert_abs_diff_eq!(r.cn, 1.0);
    }

    #[test]
    fn single_point_beta_axis_off_point_fails_closed_by_default() {
        let deck = small_3x3x1_deck();
        assert!(matches!(
            deck.lookup(0.5, 0.5, 0.5),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn single_point_beta_axis_off_point_clamps_when_opted_in() {
        let deck = small_3x3x1_deck().with_extrapolation_policy(ExtrapolationPolicy::Clamp);
        let r_at = deck.lookup(0.5, 0.5, 0.0).unwrap();
        let r_off = deck.lookup(0.5, 0.5, 5.0).unwrap();
        assert_eq!(r_at.cn.to_bits(), r_off.cn.to_bits());
    }

    // -----------------------------------------------------------------
    // Constructor validation
    // -----------------------------------------------------------------

    #[test]
    fn constructor_rejects_empty_axis() {
        let err = AeroDeck::new(
            vec![],
            vec![0.0],
            vec![0.0],
            vec![],
            vec![],
            vec![],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn constructor_rejects_non_monotone_axis() {
        let err = AeroDeck::new(
            vec![0.0, 0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn constructor_rejects_decreasing_axis() {
        let err = AeroDeck::new(
            vec![1.0, 0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn constructor_rejects_inconsistent_table_length() {
        let err = AeroDeck::new(
            vec![0.0, 1.0],
            vec![0.0],
            vec![0.0],
            vec![0.0, 1.0, 2.0], // length 3, expected 2
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn constructor_rejects_non_finite_axis() {
        let err = AeroDeck::new(
            vec![0.0, f64::NAN],
            vec![0.0],
            vec![0.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            vec![0.0, 0.0],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::NonFinite { .. }));
    }

    #[test]
    fn constructor_rejects_non_finite_table() {
        let err = AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![f64::INFINITY],
            vec![0.0],
            vec![0.0],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::NonFinite { .. }));
    }

    #[test]
    fn constructor_rejects_non_positive_reference_area() {
        let err = AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            0.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::InvalidParameter { .. }));
    }

    #[test]
    fn constructor_rejects_non_positive_reference_length() {
        let err = AeroDeck::new(
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            vec![0.0],
            1.0,
            -1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::InvalidParameter { .. }));
    }
}
