//! Phase 2.6.B regression — synthetic solid-motor data pins.
//!
//! Both shipped motor files
//! (`data/motors/synthetic-solid-textbook.toml` and
//! `data/motors/synthetic-solid-d-class.toml`) parse against
//! Schema 1, integrate to their declared `total_impulse_n_s` to bit
//! precision, satisfy the impulse-weighted mass-flow conservation
//! identity (`m(burn_duration) == dry_mass`), and are bit-stable
//! across two evaluations of the same query.
//!
//! These tests are the same shape the Phase-2.10
//! `openbmp check-provenance` walk will perform; we do them locally
//! now so a typo in either deck file or in the `Motor` mass-model
//! arithmetic breaks CI before release.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::doc_markdown
)]

use approx::assert_abs_diff_eq;
use openbmp_propulsion::{Motor, SolidMotor};

const TEXTBOOK: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/synthetic-solid-textbook.toml"
));

const D_CLASS: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/synthetic-solid-d-class.toml"
));

fn shipped_motors() -> Vec<(&'static str, SolidMotor)> {
    vec![
        (
            "synthetic-solid-textbook",
            SolidMotor::load_from_str(TEXTBOOK).expect("textbook deck must parse"),
        ),
        (
            "synthetic-solid-d-class",
            SolidMotor::load_from_str(D_CLASS).expect("d-class deck must parse"),
        ),
    ]
}

#[test]
fn both_decks_parse_and_carry_expected_metadata() {
    let motors = shipped_motors();
    assert_eq!(motors.len(), 2);
    for (name, m) in &motors {
        assert_eq!(m.meta().name, *name);
        assert!(!m.meta().provenance.is_empty(), "{name}: provenance empty");
    }
}

#[test]
fn integrated_impulse_matches_declared_total_impulse_for_every_deck() {
    for (name, m) in shipped_motors() {
        let integrated = m.thrust_curve().integrated_impulse_n_s();
        let declared = m.total_impulse_n_s();
        let denom = declared.abs();
        let rel = (integrated - declared).abs() / denom;
        assert!(
            rel < 1.0e-12,
            "{name}: integrated impulse {integrated} differs from declared {declared} by {rel} relative",
        );
    }
}

#[test]
fn mass_at_burnout_equals_dry_mass_exactly_for_every_deck() {
    for (name, m) in shipped_motors() {
        let final_mass = m.mass_kg(m.burn_duration_s()).unwrap();
        assert_eq!(
            final_mass.to_bits(),
            m.dry_mass_kg().to_bits(),
            "{name}: mass at burnout must equal dry_mass bit-exactly",
        );
    }
}

#[test]
fn mass_at_zero_equals_initial_mass_for_every_deck() {
    for (name, m) in shipped_motors() {
        let initial_mass = m.mass_kg(0.0).unwrap();
        let expected = m.dry_mass_kg() + m.propellant_mass_kg();
        assert_abs_diff_eq!(initial_mass, expected, epsilon = 0.0);
        let _ = name; // for clarity
    }
}

#[test]
fn mass_loss_at_burnout_equals_propellant_mass_for_every_deck() {
    for (name, m) in shipped_motors() {
        let initial = m.mass_kg(0.0).unwrap();
        let final_mass = m.mass_kg(m.burn_duration_s()).unwrap();
        let consumed = initial - final_mass;
        assert_abs_diff_eq!(consumed, m.propellant_mass_kg(), epsilon = 1.0e-12);
        let _ = name;
    }
}

#[test]
fn mass_rate_is_non_positive_throughout_burn_window_for_every_deck() {
    for (name, m) in shipped_motors() {
        let burn = m.burn_duration_s();
        let mut t = 0.0_f64;
        let step = burn / 200.0;
        while t <= burn {
            let r = m.mass_rate_kg_s(t).unwrap();
            assert!(
                r <= 0.0,
                "{name}: mass rate must be ≤ 0 at t = {t}, got {r}"
            );
            t += step;
        }
    }
}

#[test]
fn mass_is_monotone_decreasing_through_burn_for_every_deck() {
    for (name, m) in shipped_motors() {
        let burn = m.burn_duration_s();
        let step = burn / 200.0;
        let mut last = m.mass_kg(0.0).unwrap();
        let mut t = step;
        while t <= burn {
            let now = m.mass_kg(t).unwrap();
            assert!(
                now <= last,
                "{name}: mass must be monotone-decreasing at t = {t}",
            );
            last = now;
            t += step;
        }
    }
}

#[test]
fn thrust_at_grid_corners_returns_stored_values_for_every_deck() {
    for (name, m) in shipped_motors() {
        for point in m.thrust_curve().points() {
            let stored_thrust = point[1];
            let queried = m.thrust_n_at(point[0]).unwrap();
            assert_eq!(
                queried.to_bits(),
                stored_thrust.to_bits(),
                "{name}: thrust at grid corner t = {} must match stored value",
                point[0],
            );
        }
    }
}

#[test]
fn thrust_outside_burn_window_is_zero_for_every_deck() {
    for (name, m) in shipped_motors() {
        let pre = m.thrust_n_at(-1.0).unwrap();
        let post = m.thrust_n_at(m.burn_duration_s() + 1.0).unwrap();
        assert_eq!(
            pre.to_bits(),
            0.0_f64.to_bits(),
            "{name}: pre-ignition thrust must be zero",
        );
        assert_eq!(
            post.to_bits(),
            0.0_f64.to_bits(),
            "{name}: post-burnout thrust must be zero",
        );
    }
}

#[test]
fn thrust_and_mass_lookups_are_bit_stable_across_two_evaluations() {
    for (name, m) in shipped_motors() {
        let burn = m.burn_duration_s();
        for fraction in [0.0, 0.1, 0.25, 0.5, 0.75, 0.9, 1.0] {
            let t = fraction * burn;
            let t1 = m.thrust_n_at(t).unwrap();
            let t2 = m.thrust_n_at(t).unwrap();
            assert_eq!(
                t1.to_bits(),
                t2.to_bits(),
                "{name}: thrust bit-unstable at t = {t}"
            );
            let m1 = m.mass_kg(t).unwrap();
            let m2 = m.mass_kg(t).unwrap();
            assert_eq!(
                m1.to_bits(),
                m2.to_bits(),
                "{name}: mass bit-unstable at t = {t}"
            );
            let r1 = m.mass_rate_kg_s(t).unwrap();
            let r2 = m.mass_rate_kg_s(t).unwrap();
            assert_eq!(
                r1.to_bits(),
                r2.to_bits(),
                "{name}: mass rate bit-unstable at t = {t}",
            );
        }
    }
}
