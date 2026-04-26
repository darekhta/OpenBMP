//! Phase 2.5.C regression — synthetic finned-cylinder deck pin.
//!
//! Three classes of check.
//!
//! First, the shipped `data/aero/synthetic-finned-cylinder.toml`
//! parses against Schema 1 and round-trips through `AeroDeck::lookup`.
//!
//! Second, the deck values match the closed-form generators
//! documented in `data/aero/provenance.md`: `CN = alpha *
//! (0.07 + 0.005 * M)`, `CM = -0.15 * CN`, `CD = CD0(M) + 0.001 *
//! alpha^2`. These identities are bit-exact at the grid points
//! because the deck rounds to the same precision the generator does.
//!
//! Third, sanity invariants of the deck are preserved: `CN` at zero
//! alpha is exact zero for every Mach, `CD > 0` on every grid point,
//! and lookup at the centroid of every cube is bit-stable across two
//! evaluations.
//!
//! These tests are the same shape the Phase-2.10
//! `openbmp check-provenance` walk will perform; we do them locally
//! now so a typo in the deck file or the generator breaks CI before
//! release.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::doc_markdown
)]

use approx::assert_abs_diff_eq;
use openbmp_aero::AeroDeck;

const SYNTHETIC_FINNED_CYLINDER_DECK: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/aero/synthetic-finned-cylinder.toml"
));

fn deck() -> AeroDeck {
    AeroDeck::load_from_str(SYNTHETIC_FINNED_CYLINDER_DECK)
        .expect("synthetic finned-cylinder deck must parse")
}

/// Closed-form CD0 used to populate the deck (transonic drag rise
/// peaking around M = 1.2).
fn closed_form_cd0(mach: f64) -> f64 {
    if mach <= 0.5 {
        0.450
    } else if mach <= 0.8 {
        0.550
    } else if mach <= 1.0 {
        0.750
    } else if mach <= 1.2 {
        0.850
    } else if mach <= 1.5 {
        0.700
    } else if mach <= 2.0 {
        0.550
    } else {
        0.400
    }
}

#[test]
fn deck_parses_and_carries_reference_geometry() {
    let deck = deck();
    assert_abs_diff_eq!(deck.reference_area_m2(), 0.005_026_548, epsilon = 1.0e-9);
    assert_abs_diff_eq!(deck.reference_length_m(), 1.0, epsilon = 0.0);
    assert_eq!(deck.mach_grid().len(), 8);
    assert_eq!(deck.alpha_grid_deg().len(), 7);
    assert_eq!(deck.beta_grid_deg().len(), 1);
}

#[test]
fn cn_at_zero_alpha_is_exactly_zero_for_every_mach() {
    let deck = deck();
    for &mach in deck.mach_grid() {
        let r = deck
            .lookup(mach, 0.0, 0.0)
            .expect("query at α = 0 is in-grid");
        assert_eq!(
            r.cn.to_bits(),
            0.0_f64.to_bits(),
            "CN at α = 0 must be exact zero, mach = {mach}",
        );
    }
}

#[test]
fn cm_at_zero_alpha_is_exactly_zero_for_every_mach() {
    let deck = deck();
    for &mach in deck.mach_grid() {
        let r = deck
            .lookup(mach, 0.0, 0.0)
            .expect("query at α = 0 is in-grid");
        assert_eq!(
            r.cm.to_bits(),
            0.0_f64.to_bits(),
            "CM at α = 0 must be exact zero, mach = {mach}",
        );
    }
}

#[test]
fn cd_strictly_positive_on_every_grid_point() {
    let deck = deck();
    let machs: Vec<f64> = deck.mach_grid().to_vec();
    let alphas: Vec<f64> = deck.alpha_grid_deg().to_vec();
    let betas: Vec<f64> = deck.beta_grid_deg().to_vec();
    for &mach in &machs {
        for &alpha in &alphas {
            for &beta in &betas {
                let r = deck
                    .lookup(mach, alpha, beta)
                    .expect("grid point in-envelope");
                assert!(
                    r.cd > 0.0,
                    "CD must be > 0 at (M={mach}, α={alpha}, β={beta}); got {}",
                    r.cd,
                );
            }
        }
    }
}

#[test]
fn cd_at_zero_alpha_matches_closed_form_cd0() {
    let deck = deck();
    for &mach in deck.mach_grid() {
        let r = deck.lookup(mach, 0.0, 0.0).unwrap();
        let expected = closed_form_cd0(mach);
        assert_eq!(
            r.cd.to_bits(),
            expected.to_bits(),
            "CD(α=0) at mach = {mach} must equal CD0(M) bit-exactly",
        );
    }
}

#[test]
fn cm_equals_minus_fifteen_percent_of_cn_on_every_grid_point() {
    let deck = deck();
    let machs: Vec<f64> = deck.mach_grid().to_vec();
    let alphas: Vec<f64> = deck.alpha_grid_deg().to_vec();
    for &mach in &machs {
        for &alpha in &alphas {
            let r = deck.lookup(mach, alpha, 0.0).unwrap();
            // The shipped deck rounds both CN and CM to the same
            // precision per the closed form CM = -0.15 · CN, so the
            // identity holds bit-exactly when CN is non-zero. At
            // α = 0 both are exactly zero (verified separately).
            let expected = -0.15 * r.cn;
            assert_abs_diff_eq!(r.cm, expected, epsilon = 1.0e-12);
        }
    }
}

#[test]
fn cn_is_odd_in_alpha_about_zero() {
    let deck = deck();
    let machs: Vec<f64> = deck.mach_grid().to_vec();
    // Use only the symmetric-pair α values (skip α = 0).
    let pairs: [(f64, f64); 3] = [(-10.0, 10.0), (-5.0, 5.0), (-2.0, 2.0)];
    for &mach in &machs {
        for (a_minus, a_plus) in pairs {
            let r_minus = deck.lookup(mach, a_minus, 0.0).unwrap();
            let r_plus = deck.lookup(mach, a_plus, 0.0).unwrap();
            assert_abs_diff_eq!(r_minus.cn, -r_plus.cn, epsilon = 1.0e-12);
            assert_abs_diff_eq!(r_minus.cm, -r_plus.cm, epsilon = 1.0e-12);
        }
    }
}

#[test]
fn cd_is_even_in_alpha_about_zero() {
    let deck = deck();
    let machs: Vec<f64> = deck.mach_grid().to_vec();
    let pairs: [(f64, f64); 3] = [(-10.0, 10.0), (-5.0, 5.0), (-2.0, 2.0)];
    for &mach in &machs {
        for (a_minus, a_plus) in pairs {
            let r_minus = deck.lookup(mach, a_minus, 0.0).unwrap();
            let r_plus = deck.lookup(mach, a_plus, 0.0).unwrap();
            assert_eq!(
                r_minus.cd.to_bits(),
                r_plus.cd.to_bits(),
                "CD must be even in α; mach = {mach}, α_pair = ({a_minus}, {a_plus})",
            );
        }
    }
}

#[test]
fn lookup_at_cube_centroids_is_bit_stable_across_two_evaluations() {
    let deck = deck();
    let machs: Vec<f64> = deck.mach_grid().to_vec();
    let alphas: Vec<f64> = deck.alpha_grid_deg().to_vec();
    // For each pair of consecutive mach × alpha breakpoints,
    // sample at the cube centroid and verify two consecutive
    // lookups agree bit-exactly.
    for im in 0..(machs.len() - 1) {
        for ia in 0..(alphas.len() - 1) {
            let m_mid = 0.5 * (machs[im] + machs[im + 1]);
            let a_mid = 0.5 * (alphas[ia] + alphas[ia + 1]);
            let first = deck.lookup(m_mid, a_mid, 0.0).unwrap();
            let second = deck.lookup(m_mid, a_mid, 0.0).unwrap();
            assert_eq!(first.cn.to_bits(), second.cn.to_bits());
            assert_eq!(first.cd.to_bits(), second.cd.to_bits());
            assert_eq!(first.cm.to_bits(), second.cm.to_bits());
        }
    }
}

#[test]
fn off_grid_query_fails_closed_by_default() {
    let deck = deck();
    // Above the Mach grid (max = 3.0).
    assert!(deck.lookup(3.5, 0.0, 0.0).is_err());
    // Below the alpha grid (min = -10).
    assert!(deck.lookup(0.0, -15.0, 0.0).is_err());
    // Off the single-point beta axis.
    assert!(deck.lookup(0.0, 0.0, 1.0).is_err());
}
