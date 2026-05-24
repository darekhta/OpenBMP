//! Regression — synthetic finned-cylinder deck pin.
//!
//! Three classes of check.
//!
//! First, the shipped `data/aero/synthetic-finned-cylinder.toml`
//! parses against Schema 1 and round-trips through `AeroDeck::lookup`.
//!
//! Second, the deck values match the closed-form generators
//! documented in `data/aero/provenance.md`: `CN = alpha *
//! (0.07 + 0.005 * M)`, `CM = -0.15 * CN`, `CD = CD0(M) + 0.001 *
//! alpha^2`. The deck values are emitted from integer-scaled rational
//! forms of those expressions, so the grid-point checks below can assert
//! bit equality after TOML parse.
//!
//! Third, sanity invariants of the deck are preserved: `CN` at zero
//! alpha is exact zero for every Mach, `CD > 0` on every grid point,
//! and lookup at the centroid of every cube is bit-stable across two
//! evaluations.
//!
//! These tests are the same shape the
//! `openbmp check-provenance` walk performs; we do them locally
//! so a typo in the deck file or the generator breaks CI before
//! release.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::doc_markdown,
    clippy::similar_names
)]

use std::collections::BTreeMap;

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

/// Schema-1 lookups carry no effector deflections.
fn nd() -> BTreeMap<&'static str, f64> {
    BTreeMap::new()
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

fn mach_tenths(mach: f64) -> i32 {
    [
        (0.0, 0),
        (0.5, 5),
        (0.8, 8),
        (1.0, 10),
        (1.2, 12),
        (1.5, 15),
        (2.0, 20),
        (3.0, 30),
    ]
    .into_iter()
    .find_map(|(grid_value, tenths)| (mach == grid_value).then_some(tenths))
    .expect("synthetic finned-cylinder Mach grid must be locked")
}

fn alpha_degrees(alpha: f64) -> i32 {
    [-10, -5, -2, 0, 2, 5, 10]
        .into_iter()
        .find(|&grid_value| alpha == f64::from(grid_value))
        .expect("synthetic finned-cylinder alpha grid must be locked")
}

fn closed_form_cd0_milli(mach_tenths: i32) -> i32 {
    match mach_tenths {
        0 | 5 => 450,
        8 | 20 => 550,
        10 => 750,
        12 => 850,
        15 => 700,
        30 => 400,
        _ => unreachable!("synthetic finned-cylinder Mach grid is locked"),
    }
}

fn scaled_ratio(numerator: i32, denominator: i32) -> f64 {
    if numerator == 0 {
        0.0
    } else {
        f64::from(numerator) / f64::from(denominator)
    }
}

fn expected_grid_coefficients(mach: f64, alpha: f64) -> (f64, f64, f64) {
    let mach_tenths = mach_tenths(mach);
    let alpha_deg = alpha_degrees(alpha);

    // Integer-scaled generator equivalent to:
    //   CN = alpha_deg * (0.07 + 0.005 * M)
    // where M is represented as tenths and CN has denominator 2000.
    let cn_numerator = alpha_deg * (140 + mach_tenths);
    let cn = scaled_ratio(cn_numerator, 2_000);

    // CD0 and alpha-induced drag are emitted in thousandths.
    let cd_milli = closed_form_cd0_milli(mach_tenths) + alpha_deg * alpha_deg;
    let cd = scaled_ratio(cd_milli, 1_000);

    // CM = -0.15 * CN = -15 * CN / 100. Preserve +0.0 at alpha = 0.
    let cm = scaled_ratio(-15 * cn_numerator, 200_000);

    (cn, cd, cm)
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
fn every_grid_value_matches_locked_integer_scaled_generator() {
    let deck = deck();
    let machs: Vec<f64> = deck.mach_grid().to_vec();
    let alphas: Vec<f64> = deck.alpha_grid_deg().to_vec();
    let betas: Vec<f64> = deck.beta_grid_deg().to_vec();
    for &mach in &machs {
        for &alpha in &alphas {
            for &beta in &betas {
                let r = deck
                    .lookup(mach, alpha, beta, &nd())
                    .expect("grid point in-envelope");
                let (expected_cn, expected_cd, expected_cm) =
                    expected_grid_coefficients(mach, alpha);
                assert_eq!(
                    r.cn.to_bits(),
                    expected_cn.to_bits(),
                    "CN mismatch at (M={mach}, α={alpha}, β={beta})",
                );
                assert_eq!(
                    r.cd.to_bits(),
                    expected_cd.to_bits(),
                    "CD mismatch at (M={mach}, α={alpha}, β={beta})",
                );
                assert_eq!(
                    r.cm.to_bits(),
                    expected_cm.to_bits(),
                    "CM mismatch at (M={mach}, α={alpha}, β={beta})",
                );
            }
        }
    }
}

#[test]
fn cn_at_zero_alpha_is_exactly_zero_for_every_mach() {
    let deck = deck();
    for &mach in deck.mach_grid() {
        let r = deck
            .lookup(mach, 0.0, 0.0, &nd())
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
            .lookup(mach, 0.0, 0.0, &nd())
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
                    .lookup(mach, alpha, beta, &nd())
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
        let r = deck.lookup(mach, 0.0, 0.0, &nd()).unwrap();
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
            let r = deck.lookup(mach, alpha, 0.0, &nd()).unwrap();
            // This exercises the physical identity using ordinary f64
            // arithmetic. The exact emitted deck values are covered by
            // `every_grid_value_matches_locked_integer_scaled_generator`.
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
            let r_minus = deck.lookup(mach, a_minus, 0.0, &nd()).unwrap();
            let r_plus = deck.lookup(mach, a_plus, 0.0, &nd()).unwrap();
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
            let r_minus = deck.lookup(mach, a_minus, 0.0, &nd()).unwrap();
            let r_plus = deck.lookup(mach, a_plus, 0.0, &nd()).unwrap();
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
            let first = deck.lookup(m_mid, a_mid, 0.0, &nd()).unwrap();
            let second = deck.lookup(m_mid, a_mid, 0.0, &nd()).unwrap();
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
    assert!(deck.lookup(3.5, 0.0, 0.0, &nd()).is_err());
    // Below the alpha grid (min = -10).
    assert!(deck.lookup(0.0, -15.0, 0.0, &nd()).is_err());
    // Off the single-point beta axis.
    assert!(deck.lookup(0.0, 0.0, 1.0, &nd()).is_err());
}
