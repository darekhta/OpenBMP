//! Aerodynamic deck and locked-order multilinear interpolation.
//!
//! # Schema
//!
//! **Schema 1** is a three-axis tabulated deck indexed
//! by `(mach, alpha_deg, beta_deg)` and producing the three reduced
//! coefficients `(CN, CD, CM)`. **Schema 2** generalises the internal
//! representation to an **N-axis** deck: the same three
//! base axes plus 1–3 optional control-effector axes. Schema-1 decks
//! parse to an N=3 instance with no effector axes — the lookup at
//! N=3 is bit-identical to the original trilinear path.
//!
//! `axis_order[0..3]` is always `["mach", "alpha", "beta"]`;
//! `axis_order[3..]` (when present) names the effector axes (e.g.
//! `delta_e_deg`) declared by the schema-2 deck.
//!
//! The semantics are pinned by `docs/phase-2-plan.md § Implementation
//! Seams Locked Before Coding`:
//!
//! * `CN` — normal-force coefficient in the wind / body longitudinal
//!   plane.
//! * `CD` — drag coefficient (opposes relative wind).
//! * `CM` — pitching-moment coefficient about the body lateral axis;
//!   roll moment is identically zero by axisymmetry.
//!
//! Six-coefficient `CX/CY/CZ/Cl/Cm/Cn` decks are deferred past 3.5.
//!
//! # Storage layout
//!
//! Each coefficient table is a flat `Vec<f64>` of length
//! `axes[0].len() · axes[1].len() · ... · axes[N-1].len()`, stored in
//! **row-major** order over `axis_order` so that the **last axis
//! varies fastest**. At N=3 with `axis_order = ["mach", "alpha", "beta"]`
//! this matches the Schema-1 layout exactly:
//!
//! ```text
//! index(im, ia, ib) = im · (n_alpha · n_beta) + ia · n_beta + ib
//! ```
//!
//! # Multilinear interpolation
//!
//! Locked operand order, **innermost-axis-first** reduction. At N=3
//! this collapses to the Schema-1 trilinear form bit-for-bit — beta
//! reduces first, then alpha, then mach. Per Demmel & Nguyen 2020
//! (*Algorithms for Efficient Reproducible Floating Point Summation*,
//! ACM TOMS 46:3) the operand order is part of the byte-stability
//! contract and is asserted by a property test that compares the
//! N-D multilinear output against the literal Schema-1 closed-form
//! trilinear expression at random query points.
//!
//! ```text
//! Step k (0-indexed, innermost-first):
//!   axis_index = N - 1 - k
//!   f          = fractions[axis_index]
//!   for each adjacent corner pair (buf[2i], buf[2i + 1]):
//!     buf[i] = (1 - f) * buf[2i] + f * buf[2i + 1]
//! ```
//!
//! FMA is disabled — each `(1 - f) * x + f * y` is two separate
//! `fmul + fadd` instructions the compiler cannot fold to `mul_add`.
//!
//! # Out-of-grid behaviour
//!
//! Default is [`ExtrapolationPolicy::FailClosed`]: a query outside
//! the grid returns [`AeroError::OutOfEnvelope`]. Decks may opt into
//! [`ExtrapolationPolicy::Clamp`] which snaps an off-grid query to
//! the nearest grid face. Linear extrapolation is intentionally not
//! offered — the OpenRocket / Niskanen 2009 corpus shows linear
//! extrapolation produces nonsensical drag for high Mach decks.

use std::collections::BTreeMap;

use crate::error::AeroError;

// ---------------------------------------------------------------------
// AeroCoefficients
// ---------------------------------------------------------------------

/// The three reduced coefficients returned by an aerodynamic deck
/// lookup. Schema 1 / Schema 2.
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

/// Tabulated aerodynamic deck. Holds the locked axis names, the
/// per-axis grids, and the row-major coefficient tables.
///
/// At N=3 (Schema 1) `axis_order = ["mach", "alpha", "beta"]` and the
/// lookup is the original trilinear form. At N > 3 (Schema 2)
/// `axis_order[3..]` names the additional effector axes.
#[derive(Clone, Debug, PartialEq)]
pub struct AeroDeck {
    /// Locked axis names. Always starts with
    /// `["mach", "alpha", "beta"]`; Schema 2 appends effector-axis
    /// names (each ending with `_deg` or `_rad`).
    axis_order: Vec<String>,
    /// One axis grid per name. Strictly monotone-increasing, finite,
    /// non-empty. Same length and order as `axis_order`.
    axes: Vec<Vec<f64>>,
    /// Coefficient tables, each row-major over the cartesian product
    /// of `axes`. Length = product of axis lengths. The last axis in
    /// `axis_order` varies fastest in storage.
    cn: Vec<f64>,
    cd: Vec<f64>,
    cm: Vec<f64>,
    reference_area_m2: f64,
    reference_length_m: f64,
    extrapolation: ExtrapolationPolicy,
}

impl AeroDeck {
    /// Construct a Schema-1 (3-axis) in-memory deck.
    ///
    /// Equivalent to [`AeroDeck::new_n_d`] with
    /// `axis_order = ["mach", "alpha", "beta"]`. The Schema-1 parser
    /// uses this entry point.
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
    #[allow(clippy::too_many_arguments)] // mirrors the Schema-1 deck-file shape; the TOML parser is the primary user.
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
        Self::new_n_d(
            vec!["mach".to_string(), "alpha".to_string(), "beta".to_string()],
            vec![mach, alpha_deg, beta_deg],
            cn,
            cd,
            cm,
            reference_area_m2,
            reference_length_m,
        )
    }

    /// Construct an N-axis deck with explicit `axis_order` and per-axis
    /// grids. Schema-2 decks (with effector axes) use this entry point.
    ///
    /// Validates that:
    ///
    /// - `axis_order.len() == axes.len()`
    /// - `axis_order` is non-empty and contains no duplicates
    /// - the first three names are exactly `["mach", "alpha", "beta"]`
    /// - at most 6 axes are declared (3 base axes + 3 effector axes)
    /// - every axis is non-empty, finite, and strictly monotone-increasing
    /// - every coefficient table length equals the product of axis lengths
    ///
    /// # Errors
    ///
    /// See [`AeroDeck::new`] — same error class. Adds
    /// [`AeroError::MalformedDeck`] for `axis_order` violations.
    #[allow(clippy::too_many_arguments)] // matches the deck-file schema; the parser is the primary user.
    pub fn new_n_d(
        axis_order: Vec<String>,
        axes: Vec<Vec<f64>>,
        cn: Vec<f64>,
        cd: Vec<f64>,
        cm: Vec<f64>,
        reference_area_m2: f64,
        reference_length_m: f64,
    ) -> Result<Self, AeroError> {
        validate_axis_order(&axis_order, axes.len())?;
        for (axis, name) in axes.iter().zip(axis_order.iter()) {
            validate_axis(axis, axis_label(name))?;
        }
        let expected = axes.iter().try_fold(1usize, |acc, axis| {
            acc.checked_mul(axis.len()).ok_or(AeroError::MalformedDeck {
                reason: "axis size product overflows usize",
            })
        })?;
        validate_table(&cn, expected, "CN table")?;
        validate_table(&cd, expected, "CD table")?;
        validate_table(&cm, expected, "CM table")?;
        validate_reference(reference_area_m2, "reference area")?;
        validate_reference(reference_length_m, "reference length")?;
        Ok(Self {
            axis_order,
            axes,
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

    /// All declared axis names in canonical order.
    /// Always starts with `["mach", "alpha", "beta"]`.
    #[must_use]
    pub fn axis_order(&self) -> &[String] {
        &self.axis_order
    }

    /// Effector-axis names declared by this deck (Schema 2 only).
    /// Empty for Schema-1 decks.
    #[must_use]
    pub fn effector_axis_names(&self) -> &[String] {
        &self.axis_order[3..]
    }

    /// Mach grid (read-only access; ordered low-to-high).
    #[must_use]
    pub fn mach_grid(&self) -> &[f64] {
        &self.axes[0]
    }

    /// Angle-of-attack grid in degrees (read-only; ordered low-to-high).
    #[must_use]
    pub fn alpha_grid_deg(&self) -> &[f64] {
        &self.axes[1]
    }

    /// Side-slip grid in degrees (read-only; ordered low-to-high).
    #[must_use]
    pub fn beta_grid_deg(&self) -> &[f64] {
        &self.axes[2]
    }

    /// Look up the deck at `(mach, alpha_deg, beta_deg, deflections)`
    /// using locked-order multilinear interpolation.
    ///
    /// Schema-1 decks ignore `deflections` entirely. Schema-2 decks
    /// require a value for every declared effector axis (missing key
    /// returns [`AeroError::InvalidParameter`]). Extra keys in
    /// `deflections` not declared by the deck are silently ignored
    /// (forward-compatible).
    ///
    /// # Errors
    ///
    /// Returns [`AeroError::NonFinite`] when any input is `NaN` or
    /// `Inf`. Returns [`AeroError::OutOfEnvelope`] when the input is
    /// outside the grid on any axis and the deck is configured
    /// [`ExtrapolationPolicy::FailClosed`] (the default). Returns
    /// [`AeroError::InvalidParameter`] when a Schema-2 deck axis has
    /// no entry in `deflections`.
    pub fn lookup(
        &self,
        mach: f64,
        alpha_deg: f64,
        beta_deg: f64,
        deflections: &BTreeMap<&str, f64>,
    ) -> Result<AeroCoefficients, AeroError> {
        let n = self.axis_order.len();
        // Per-axis (i_lo, fraction) pairs in axis_order.
        let mut indices: Vec<(usize, f64)> = Vec::with_capacity(n);
        indices.push(bracket(
            &self.axes[0],
            mach,
            self.extrapolation,
            "mach outside deck envelope",
        )?);
        indices.push(bracket(
            &self.axes[1],
            alpha_deg,
            self.extrapolation,
            "alpha outside deck envelope",
        )?);
        indices.push(bracket(
            &self.axes[2],
            beta_deg,
            self.extrapolation,
            "beta outside deck envelope",
        )?);
        for axis_index in 3..n {
            let name = self.axis_order[axis_index].as_str();
            let value = deflections
                .get(name)
                .copied()
                .ok_or(AeroError::InvalidParameter {
                    reason: "deck lookup missing effector deflection",
                })?;
            indices.push(bracket(
                &self.axes[axis_index],
                value,
                self.extrapolation,
                "effector deflection outside deck envelope",
            )?);
        }

        // Pre-compute storage strides and the corner stencil.
        let strides = compute_strides(&self.axes);
        let corners = collect_corner_indices(&self.axes, &indices, &strides);
        let fractions: Vec<f64> = indices.iter().map(|(_, f)| *f).collect();

        Ok(AeroCoefficients {
            cn: multilinear_reduce(&self.cn, &corners, &fractions),
            cd: multilinear_reduce(&self.cd, &corners, &fractions),
            cm: multilinear_reduce(&self.cm, &corners, &fractions),
        })
    }
}

// ---------------------------------------------------------------------
// Internal helpers — multilinear lookup
// ---------------------------------------------------------------------

/// Per-axis row-major strides. `strides[k]` is the number of
/// `Vec<f64>` entries one step along `axes[k]` advances.
fn compute_strides(axes: &[Vec<f64>]) -> Vec<usize> {
    let n = axes.len();
    let mut strides = vec![1usize; n];
    for k in (0..n - 1).rev() {
        strides[k] = strides[k + 1] * axes[k + 1].len();
    }
    strides
}

/// Collect the `2^N` corner indices around `indices` in row-major
/// order over `axis_order`. The k-th axis varies in the bit at
/// position `(N - 1 - k)` of the corner-array index, so the **last
/// axis varies fastest** in the corner array — matching the storage
/// layout. This is the layout the innermost-first reduction below
/// expects.
fn collect_corner_indices(
    axes: &[Vec<f64>],
    indices: &[(usize, f64)],
    strides: &[usize],
) -> Vec<usize> {
    let n = axes.len();
    let count = 1usize << n;
    let mut out = Vec::with_capacity(count);
    for corner in 0..count {
        let mut idx = 0usize;
        for k in 0..n {
            // Bit at position (N - 1 - k) selects this axis's offset.
            let bit = (corner >> (n - 1 - k)) & 1;
            // Single-point axis: i_next = i and bit-1 maps to the same
            // index as bit-0, so the (1 - f)·c[i] + f·c[i_next]
            // reduction degenerates to c[i].
            let next = if axes[k].len() == 1 {
                indices[k].0
            } else {
                indices[k].0 + bit
            };
            idx += next * strides[k];
        }
        out.push(idx);
    }
    out
}

/// Locked-order multilinear reduction.
///
/// `corners.len() == 2^N`, in row-major order over `axis_order`
/// (last axis fastest). `fractions[k]` is the bracket fraction for
/// `axes[k]` in `axis_order`. Reduction collapses the **innermost
/// axis first** (k = N - 1), then walks outward to k = 0. At N = 3
/// with `axis_order = ["mach", "alpha", "beta"]` this is bit-identical
/// to the Schema-1 trilinear formula.
fn multilinear_reduce(table: &[f64], corners: &[usize], fractions: &[f64]) -> f64 {
    let n = fractions.len();
    let mut buf: Vec<f64> = corners.iter().map(|&i| table[i]).collect();
    // Reduce innermost-first: k = N-1, N-2, ..., 0.
    for k in (0..n).rev() {
        let f = fractions[k];
        let one_minus_f = 1.0 - f;
        let half = buf.len() / 2;
        for i in 0..half {
            let lo = buf[2 * i];
            let hi = buf[2 * i + 1];
            buf[i] = one_minus_f * lo + f * hi;
        }
        buf.truncate(half);
    }
    buf[0]
}

// ---------------------------------------------------------------------
// Internal helpers — validation
// ---------------------------------------------------------------------

const REQUIRED_PREFIX: [&str; 3] = ["mach", "alpha", "beta"];

fn validate_axis_order(axis_order: &[String], axes_len: usize) -> Result<(), AeroError> {
    if axis_order.is_empty() {
        return Err(AeroError::MalformedDeck {
            reason: "axis_order must declare at least mach/alpha/beta",
        });
    }
    if axis_order.len() != axes_len {
        return Err(AeroError::MalformedDeck {
            reason: "axis_order length does not match the number of axis grids",
        });
    }
    if axis_order.len() > 6 {
        return Err(AeroError::MalformedDeck {
            reason: "schema-2 deck supports at most 3 effector axes (6 axes total)",
        });
    }
    for (k, required) in REQUIRED_PREFIX.iter().enumerate() {
        if axis_order.get(k).map(String::as_str) != Some(*required) {
            return Err(AeroError::MalformedDeck {
                reason: "axis_order must start with [mach, alpha, beta]",
            });
        }
    }
    let mut seen: std::collections::BTreeSet<&str> = std::collections::BTreeSet::new();
    for name in axis_order {
        if !seen.insert(name.as_str()) {
            return Err(AeroError::MalformedDeck {
                reason: "axis_order contains a duplicate name",
            });
        }
    }
    Ok(())
}

fn axis_label(name: &str) -> &'static str {
    match name {
        "mach" => "mach axis",
        "alpha" => "alpha axis",
        "beta" => "beta axis",
        _ => "effector axis",
    }
}

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
#[allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::cast_lossless,
    clippy::similar_names
)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use proptest::prelude::*;

    /// Empty deflections map for Schema-1 lookup calls.
    fn nd() -> BTreeMap<&'static str, f64> {
        BTreeMap::new()
    }

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
            let r = deck.lookup(m, a, b, &nd()).unwrap();
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
        let r = deck.lookup(0.5, 0.5, 0.5, &nd()).unwrap();
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
            let r = deck.lookup(m, a, b_query, &nd()).unwrap();
            assert_eq!(r.cn.to_bits(), expected.to_bits(), "beta = {b_query}");
        }
    }

    #[test]
    fn lookup_recovers_distinct_coefficients_per_channel() {
        let deck = small_3x3x1_deck();
        // Centre point (mach = 1, alpha = 0).
        let r = deck.lookup(1.0, 0.0, 0.0, &nd()).unwrap();
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
            let a = deck.lookup(q.0, q.1, q.2, &nd()).unwrap();
            let b = deck.lookup(q.0, q.1, q.2, &nd()).unwrap();
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
        let a = deck1.lookup(q.0, q.1, q.2, &nd()).unwrap();
        let b = deck2.lookup(q.0, q.1, q.2, &nd()).unwrap();
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
            deck.lookup(-0.1, 0.5, 0.5, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, -0.1, 0.5, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 0.5, -0.1, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn fail_closed_above_grid_returns_out_of_envelope() {
        let deck = cube_deck();
        assert!(matches!(
            deck.lookup(1.1, 0.5, 0.5, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 1.1, 0.5, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 0.5, 1.1, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn clamp_below_grid_returns_first_corner_value() {
        let deck = cube_deck().with_extrapolation_policy(ExtrapolationPolicy::Clamp);
        // Below all three lower bounds → corner (0,0,0) value.
        let r = deck.lookup(-1.0, -1.0, -1.0, &nd()).unwrap();
        assert_eq!(r.cn.to_bits(), 0.0_f64.to_bits());
    }

    #[test]
    fn clamp_above_grid_returns_last_corner_value() {
        let deck = cube_deck().with_extrapolation_policy(ExtrapolationPolicy::Clamp);
        // Above all three upper bounds → corner (1,1,1) value = 7.
        let r = deck.lookup(2.0, 2.0, 2.0, &nd()).unwrap();
        assert_eq!(r.cn.to_bits(), 7.0_f64.to_bits());
    }

    #[test]
    fn lookup_rejects_non_finite_query() {
        let deck = cube_deck();
        assert!(matches!(
            deck.lookup(f64::NAN, 0.5, 0.5, &nd()),
            Err(AeroError::NonFinite { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, f64::INFINITY, 0.5, &nd()),
            Err(AeroError::NonFinite { .. })
        ));
        assert!(matches!(
            deck.lookup(0.5, 0.5, f64::NEG_INFINITY, &nd()),
            Err(AeroError::NonFinite { .. })
        ));
    }

    // -----------------------------------------------------------------
    // Single-point axis (beta = [0])
    // -----------------------------------------------------------------

    #[test]
    fn single_point_beta_axis_handles_lookup_at_the_point() {
        let deck = small_3x3x1_deck();
        let r = deck.lookup(0.5, 0.5, 0.0, &nd()).unwrap();
        // At (m=0.5, a=0.5, b=0): CN = m + a = 1.0 (linear in the
        // 4 corners (0,0)=0, (0,1)=1, (1,0)=1, (1,1)=2 → centre 1.0).
        assert_abs_diff_eq!(r.cn, 1.0);
    }

    #[test]
    fn single_point_beta_axis_off_point_fails_closed_by_default() {
        let deck = small_3x3x1_deck();
        assert!(matches!(
            deck.lookup(0.5, 0.5, 0.5, &nd()),
            Err(AeroError::OutOfEnvelope { .. })
        ));
    }

    #[test]
    fn single_point_beta_axis_off_point_clamps_when_opted_in() {
        let deck = small_3x3x1_deck().with_extrapolation_policy(ExtrapolationPolicy::Clamp);
        let r_at = deck.lookup(0.5, 0.5, 0.0, &nd()).unwrap();
        let r_off = deck.lookup(0.5, 0.5, 5.0, &nd()).unwrap();
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

    // -----------------------------------------------------------------
    // N-D axis_order validation
    // -----------------------------------------------------------------

    #[test]
    fn new_n_d_rejects_axis_order_not_starting_mach_alpha_beta() {
        let err = AeroDeck::new_n_d(
            vec!["alpha".to_string(), "mach".to_string(), "beta".to_string()],
            vec![vec![0.0, 1.0], vec![0.0, 1.0], vec![0.0, 1.0]],
            (0..8).map(f64::from).collect(),
            (0..8).map(f64::from).collect(),
            (0..8).map(f64::from).collect(),
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn new_n_d_rejects_axis_order_length_mismatch() {
        let err = AeroDeck::new_n_d(
            vec!["mach".to_string(), "alpha".to_string()],
            vec![vec![0.0, 1.0], vec![0.0, 1.0], vec![0.0, 1.0]],
            (0..8).map(f64::from).collect(),
            (0..8).map(f64::from).collect(),
            (0..8).map(f64::from).collect(),
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn new_n_d_rejects_duplicate_axis_name() {
        let err = AeroDeck::new_n_d(
            vec![
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "delta_e_deg".to_string(),
                "delta_e_deg".to_string(),
            ],
            vec![
                vec![0.0, 1.0],
                vec![0.0, 1.0],
                vec![0.0, 1.0],
                vec![-1.0, 1.0],
                vec![-1.0, 1.0],
            ],
            vec![0.0; 32],
            vec![0.0; 32],
            vec![0.0; 32],
            1.0,
            1.0,
        )
        .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn new_n_d_rejects_more_than_six_axes() {
        let axis_order = vec![
            "mach".to_string(),
            "alpha".to_string(),
            "beta".to_string(),
            "delta_1_deg".to_string(),
            "delta_2_deg".to_string(),
            "delta_3_deg".to_string(),
            "delta_4_deg".to_string(),
        ];
        let axes = vec![vec![0.0]; axis_order.len()];
        let err = AeroDeck::new_n_d(axis_order, axes, vec![0.0], vec![0.0], vec![0.0], 1.0, 1.0)
            .unwrap_err();
        assert!(matches!(err, AeroError::MalformedDeck { .. }));
    }

    #[test]
    fn axis_order_returns_canonical_prefix_for_schema1() {
        let deck = cube_deck();
        assert_eq!(
            deck.axis_order(),
            &["mach".to_string(), "alpha".to_string(), "beta".to_string()]
        );
        assert!(deck.effector_axis_names().is_empty());
    }

    // -----------------------------------------------------------------
    // Multilinear-at-N3 ↔ Schema-1 trilinear bit-equality
    // -----------------------------------------------------------------

    /// Independent reference: the literal Schema-1 trilinear formula
    /// transcribed verbatim from `software-architecture.md` and from
    /// the pre-3.5.A `AeroDeck::lookup` body. Compared against
    /// `AeroDeck::lookup` to anchor byte-stability.
    fn trilinear_reference(
        mach: &[f64],
        alpha: &[f64],
        beta: &[f64],
        table: &[f64],
        m_q: f64,
        a_q: f64,
        b_q: f64,
    ) -> f64 {
        let bracket_local = |axis: &[f64], q: f64| -> (usize, f64) {
            let n = axis.len();
            if n == 1 {
                return (0, 0.0);
            }
            let upper = axis.partition_point(|&x| x <= q);
            let i_lo = upper.saturating_sub(1).min(n - 2);
            let denom = axis[i_lo + 1] - axis[i_lo];
            (i_lo, (q - axis[i_lo]) / denom)
        };
        let (im, fm) = bracket_local(mach, m_q);
        let (ia, fa) = bracket_local(alpha, a_q);
        let (ib, fb) = bracket_local(beta, b_q);
        let im_next = if mach.len() == 1 { im } else { im + 1 };
        let ia_next = if alpha.len() == 1 { ia } else { ia + 1 };
        let ib_next = if beta.len() == 1 { ib } else { ib + 1 };
        let n_beta = beta.len();
        let n_ab = alpha.len() * n_beta;
        let i000 = im * n_ab + ia * n_beta + ib;
        let i001 = im * n_ab + ia * n_beta + ib_next;
        let i010 = im * n_ab + ia_next * n_beta + ib;
        let i011 = im * n_ab + ia_next * n_beta + ib_next;
        let i100 = im_next * n_ab + ia * n_beta + ib;
        let i101 = im_next * n_ab + ia * n_beta + ib_next;
        let i110 = im_next * n_ab + ia_next * n_beta + ib;
        let i111 = im_next * n_ab + ia_next * n_beta + ib_next;
        let v00 = (1.0 - fb) * table[i000] + fb * table[i001];
        let v01 = (1.0 - fb) * table[i010] + fb * table[i011];
        let v10 = (1.0 - fb) * table[i100] + fb * table[i101];
        let v11 = (1.0 - fb) * table[i110] + fb * table[i111];
        let v0 = (1.0 - fa) * v00 + fa * v01;
        let v1 = (1.0 - fa) * v10 + fa * v11;
        (1.0 - fm) * v0 + fm * v1
    }

    /// Independent 4-D reference written as an explicit innermost-first
    /// reduction: axis 3, then axis 2, then axis 1, then axis 0.
    #[allow(clippy::too_many_arguments)]
    fn quadlinear_reference(
        axis0: &[f64],
        axis1: &[f64],
        axis2: &[f64],
        axis3: &[f64],
        table: &[f64],
        q0: f64,
        q1: f64,
        q2: f64,
        q3: f64,
    ) -> f64 {
        let bracket_local = |axis: &[f64], q: f64| -> (usize, f64) {
            let n = axis.len();
            let upper = axis.partition_point(|&x| x <= q);
            let i_lo = upper.saturating_sub(1).min(n - 2);
            let denom = axis[i_lo + 1] - axis[i_lo];
            (i_lo, (q - axis[i_lo]) / denom)
        };
        let (i0, f0) = bracket_local(axis0, q0);
        let (i1, f1) = bracket_local(axis1, q1);
        let (i2, f2) = bracket_local(axis2, q2);
        let (i3, f3) = bracket_local(axis3, q3);
        let i0n = i0 + 1;
        let i1n = i1 + 1;
        let i2n = i2 + 1;
        let i3n = i3 + 1;
        let s0 = axis1.len() * axis2.len() * axis3.len();
        let s1 = axis2.len() * axis3.len();
        let s2 = axis3.len();
        let idx = |a: usize, b: usize, c: usize, d: usize| a * s0 + b * s1 + c * s2 + d;
        let lerp = |f: f64, lo: f64, hi: f64| (1.0 - f) * lo + f * hi;

        let v000 = lerp(f3, table[idx(i0, i1, i2, i3)], table[idx(i0, i1, i2, i3n)]);
        let v001 = lerp(
            f3,
            table[idx(i0, i1, i2n, i3)],
            table[idx(i0, i1, i2n, i3n)],
        );
        let v010 = lerp(
            f3,
            table[idx(i0, i1n, i2, i3)],
            table[idx(i0, i1n, i2, i3n)],
        );
        let v011 = lerp(
            f3,
            table[idx(i0, i1n, i2n, i3)],
            table[idx(i0, i1n, i2n, i3n)],
        );
        let v100 = lerp(
            f3,
            table[idx(i0n, i1, i2, i3)],
            table[idx(i0n, i1, i2, i3n)],
        );
        let v101 = lerp(
            f3,
            table[idx(i0n, i1, i2n, i3)],
            table[idx(i0n, i1, i2n, i3n)],
        );
        let v110 = lerp(
            f3,
            table[idx(i0n, i1n, i2, i3)],
            table[idx(i0n, i1n, i2, i3n)],
        );
        let v111 = lerp(
            f3,
            table[idx(i0n, i1n, i2n, i3)],
            table[idx(i0n, i1n, i2n, i3n)],
        );

        let v00 = lerp(f2, v000, v001);
        let v01 = lerp(f2, v010, v011);
        let v10 = lerp(f2, v100, v101);
        let v11 = lerp(f2, v110, v111);
        let v0 = lerp(f1, v00, v01);
        let v1 = lerp(f1, v10, v11);
        lerp(f0, v0, v1)
    }

    proptest! {
        /// 1000 random (n_mach, n_alpha, n_beta) grids and random
        /// query points; multilinear at N=3 must be bit-identical to
        /// the literal Schema-1 trilinear closed-form. This is the
        /// byte-stability anchor for 3.5.A — every other byte-stable
        /// claim downstream rests on this property holding.
        #[test]
        fn multilinear_at_n3_matches_schema1_trilinear_bit_identical(
            seed in 0u64..1024,
        ) {
            // Deterministic seed → grids and queries.
            let mut s = seed.wrapping_mul(0x9E37_79B9_7F4A_7C15);
            let mut rng = || {
                s = s.wrapping_mul(0x5851_F42D_4C95_7F2D).wrapping_add(0x1405_7B7E_F767_814F);
                ((s >> 33) as u32) as f64 / u32::MAX as f64
            };
            let n_m = 2 + (rng() * 4.0) as usize; // 2..=5
            let n_a = 2 + (rng() * 4.0) as usize;
            let n_b = 2 + (rng() * 4.0) as usize;
            let mut mach: Vec<f64> = (0..n_m).map(|i| i as f64 + 0.5 * rng()).collect();
            let mut alpha: Vec<f64> = (0..n_a).map(|i| i as f64 - (n_a as f64) / 2.0 + 0.5 * rng()).collect();
            let mut beta: Vec<f64> = (0..n_b).map(|i| i as f64 - (n_b as f64) / 2.0 + 0.5 * rng()).collect();
            // Force strict monotone (defeats coincident-point edge cases from rng).
            for k in 1..mach.len() { if mach[k] <= mach[k-1] { mach[k] = mach[k-1] + 1.0; } }
            for k in 1..alpha.len() { if alpha[k] <= alpha[k-1] { alpha[k] = alpha[k-1] + 1.0; } }
            for k in 1..beta.len() { if beta[k] <= beta[k-1] { beta[k] = beta[k-1] + 1.0; } }
            let count = n_m * n_a * n_b;
            let table: Vec<f64> = (0..count).map(|_| rng() * 10.0 - 5.0).collect();
            let deck = AeroDeck::new(
                mach.clone(), alpha.clone(), beta.clone(),
                table.clone(), table.clone(), table.clone(),
                1.0, 1.0,
            ).unwrap();
            // Random query inside the envelope.
            let m_q = mach[0] + rng() * (mach[mach.len() - 1] - mach[0]);
            let a_q = alpha[0] + rng() * (alpha[alpha.len() - 1] - alpha[0]);
            let b_q = beta[0] + rng() * (beta[beta.len() - 1] - beta[0]);
            let ours = deck.lookup(m_q, a_q, b_q, &nd()).unwrap();
            let expected = trilinear_reference(&mach, &alpha, &beta, &table, m_q, a_q, b_q);
            prop_assert_eq!(
                ours.cn.to_bits(), expected.to_bits(),
                "CN bit-mismatch at m={}, a={}, b={}", m_q, a_q, b_q
            );
            prop_assert_eq!(ours.cd.to_bits(), expected.to_bits());
            prop_assert_eq!(ours.cm.to_bits(), expected.to_bits());
        }

        /// 4-D schema-2 lookup must match an independent explicit
        /// quadlinear reference with the same locked operand order.
        #[test]
        fn multilinear_at_n4_matches_hand_rolled_quadlinear_bit_identical(
            seed in 0u64..1024,
        ) {
            let mut s = seed.wrapping_mul(0xA076_1D64_78BD_642F);
            let mut rng = || {
                s = s.wrapping_mul(0xE703_7ED1_A0B4_28DB).wrapping_add(0x8EBC_6AF0_9C88_C6E3);
                ((s >> 33) as u32) as f64 / u32::MAX as f64
            };
            let n0 = 2 + (rng() * 3.0) as usize; // 2..=4
            let n1 = 2 + (rng() * 3.0) as usize;
            let n2 = 2 + (rng() * 3.0) as usize;
            let n3 = 2 + (rng() * 3.0) as usize;
            let mut a0: Vec<f64> = (0..n0).map(|i| i as f64 + 0.25 * rng()).collect();
            let mut a1: Vec<f64> = (0..n1).map(|i| i as f64 - 2.0 + 0.25 * rng()).collect();
            let mut a2: Vec<f64> = (0..n2).map(|i| i as f64 - 1.0 + 0.25 * rng()).collect();
            let mut a3: Vec<f64> = (0..n3).map(|i| -20.0 + 10.0 * i as f64 + rng()).collect();
            for k in 1..a0.len() { if a0[k] <= a0[k-1] { a0[k] = a0[k-1] + 1.0; } }
            for k in 1..a1.len() { if a1[k] <= a1[k-1] { a1[k] = a1[k-1] + 1.0; } }
            for k in 1..a2.len() { if a2[k] <= a2[k-1] { a2[k] = a2[k-1] + 1.0; } }
            for k in 1..a3.len() { if a3[k] <= a3[k-1] { a3[k] = a3[k-1] + 1.0; } }
            let count = n0 * n1 * n2 * n3;
            let table: Vec<f64> = (0..count).map(|_| rng() * 20.0 - 10.0).collect();
            let deck = AeroDeck::new_n_d(
                vec![
                    "mach".to_string(),
                    "alpha".to_string(),
                    "beta".to_string(),
                    "delta_e_deg".to_string(),
                ],
                vec![a0.clone(), a1.clone(), a2.clone(), a3.clone()],
                table.clone(),
                table.clone(),
                table.clone(),
                1.0,
                1.0,
            ).unwrap();
            let q0 = a0[0] + rng() * (a0[a0.len() - 1] - a0[0]);
            let q1 = a1[0] + rng() * (a1[a1.len() - 1] - a1[0]);
            let q2 = a2[0] + rng() * (a2[a2.len() - 1] - a2[0]);
            let q3 = a3[0] + rng() * (a3[a3.len() - 1] - a3[0]);
            let mut def = BTreeMap::new();
            def.insert("delta_e_deg", q3);
            let ours = deck.lookup(q0, q1, q2, &def).unwrap();
            let expected = quadlinear_reference(&a0, &a1, &a2, &a3, &table, q0, q1, q2, q3);
            prop_assert_eq!(ours.cn.to_bits(), expected.to_bits());
            prop_assert_eq!(ours.cd.to_bits(), expected.to_bits());
            prop_assert_eq!(ours.cm.to_bits(), expected.to_bits());
        }
    }

    // -----------------------------------------------------------------
    // Schema-2 path (4-D N=4 deck with one effector axis)
    // -----------------------------------------------------------------

    /// 2x2x1x3 N=4 deck: (mach, alpha, beta, `delta_e_deg`) where
    /// CN = m + a + 0.1 * delta. `delta_e_deg` ∈ [-20, 0, 20].
    fn elevon_4d_deck() -> AeroDeck {
        let mach = vec![0.0, 1.0];
        let alpha = vec![0.0, 1.0];
        let beta = vec![0.0];
        let delta = vec![-20.0, 0.0, 20.0];
        let mut cn = Vec::new();
        let mut cd = Vec::new();
        let mut cm = Vec::new();
        for m in &mach {
            for a in &alpha {
                for _b in &beta {
                    for d in &delta {
                        cn.push(m + a + 0.1 * d);
                        cd.push(0.05 * d);
                        cm.push(*d);
                    }
                }
            }
        }
        AeroDeck::new_n_d(
            vec![
                "mach".to_string(),
                "alpha".to_string(),
                "beta".to_string(),
                "delta_e_deg".to_string(),
            ],
            vec![mach, alpha, beta, delta],
            cn,
            cd,
            cm,
            1.0,
            1.0,
        )
        .unwrap()
    }

    #[test]
    fn schema2_corner_lookup_returns_stored_value_exactly() {
        let deck = elevon_4d_deck();
        let mut def = BTreeMap::new();
        for &m in &[0.0, 1.0] {
            for &a in &[0.0, 1.0] {
                for &d in &[-20.0, 0.0, 20.0] {
                    def.insert("delta_e_deg", d);
                    let r = deck.lookup(m, a, 0.0, &def).unwrap();
                    assert_eq!(r.cn.to_bits(), (m + a + 0.1 * d).to_bits());
                    assert_eq!(r.cd.to_bits(), (0.05 * d).to_bits());
                    assert_eq!(r.cm.to_bits(), d.to_bits());
                }
            }
        }
    }

    #[test]
    fn schema2_lookup_at_zero_deflection_matches_schema1_companion() {
        // Build the schema-1 deck with the same (m, a, b) slice as
        // the 4-D deck at delta_e_deg = 0. CN = m + a.
        let mach = vec![0.0, 1.0];
        let alpha = vec![0.0, 1.0];
        let beta = vec![0.0];
        let mut cn1 = Vec::new();
        let mut cd1 = Vec::new();
        let mut cm1 = Vec::new();
        for m in &mach {
            for a in &alpha {
                for _b in &beta {
                    cn1.push(m + a);
                    cd1.push(0.0);
                    cm1.push(0.0);
                }
            }
        }
        let s1 = AeroDeck::new(mach, alpha, beta, cn1, cd1, cm1, 1.0, 1.0).unwrap();
        let s2 = elevon_4d_deck();
        let mut def = BTreeMap::new();
        def.insert("delta_e_deg", 0.0);
        for (m, a) in [(0.0, 0.0), (0.5, 0.5), (1.0, 1.0), (0.123_456, 0.789_012)] {
            let r1 = s1.lookup(m, a, 0.0, &nd()).unwrap();
            let r2 = s2.lookup(m, a, 0.0, &def).unwrap();
            assert_eq!(
                r1.cn.to_bits(),
                r2.cn.to_bits(),
                "CN diverges at (m={m}, a={a}, delta_e_deg=0)"
            );
            assert_eq!(r1.cd.to_bits(), r2.cd.to_bits());
            assert_eq!(r1.cm.to_bits(), r2.cm.to_bits());
        }
    }

    #[test]
    fn schema2_lookup_rejects_missing_effector_axis() {
        let deck = elevon_4d_deck();
        let err = deck.lookup(0.5, 0.5, 0.0, &nd()).unwrap_err();
        assert!(matches!(err, AeroError::InvalidParameter { .. }));
    }

    #[test]
    fn schema2_lookup_ignores_unknown_extra_keys() {
        let deck = elevon_4d_deck();
        let mut def = BTreeMap::new();
        def.insert("delta_e_deg", 0.0);
        def.insert("delta_a_deg", 99.0); // not declared by this deck
        let r = deck.lookup(0.5, 0.5, 0.0, &def).unwrap();
        // Same as if delta_a_deg were absent (deck doesn't reference it).
        assert!(r.cn.is_finite());
    }

    #[test]
    fn schema2_corner_lookup_is_bit_stable_across_two_invocations() {
        let deck = elevon_4d_deck();
        let mut def = BTreeMap::new();
        def.insert("delta_e_deg", 10.5);
        let r1 = deck.lookup(0.4, 0.7, 0.0, &def).unwrap();
        let r2 = deck.lookup(0.4, 0.7, 0.0, &def).unwrap();
        assert_eq!(r1.cn.to_bits(), r2.cn.to_bits());
        assert_eq!(r1.cd.to_bits(), r2.cd.to_bits());
        assert_eq!(r1.cm.to_bits(), r2.cm.to_bits());
    }
}
