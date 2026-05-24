//! Regression test for the Cesaroni Pro75 M1670
//! motor port. Pins:
//!
//!  - The OpenBMP TOML at `data/motors/cesaroni-m1670.toml`
//!    parses and integrates to the declared total impulse to bit
//!    precision.
//!  - The RocketPy-Calisto mass variant at
//!    `data/motors/rocketpy-calisto-m1670.toml` uses RocketPy's
//!    `SolidMotor` dry mass and grain-geometry propellant mass while
//!    retaining the same upstream thrust curve.
//!  - The committed thrust-curve points round-trip against the
//!    upstream RASP `.eng` file shipped at
//!    `data/motors/Cesaroni_M1670.eng`.
//!  - The upstream `.eng` file's SHA-256 matches the pin
//!    recorded in `data/motors/provenance.md`.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]

use openbmp_propulsion::{Motor, SolidMotor};
use sha2::{Digest, Sha256};
use std::fmt::Write;

const M1670_TOML: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/cesaroni-m1670.toml"
));

const M1670_ENG: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/Cesaroni_M1670.eng"
));

const ROCKETPY_CALISTO_M1670_TOML: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/motors/rocketpy-calisto-m1670.toml"
));

const M1670_ENG_SHA256: &str = "c0153d19c999021ad83686040fca5c35d82bb43efeebd0bd94f9e177484d9a3a";

#[test]
fn cesaroni_m1670_toml_parses_with_expected_metadata() {
    let m = SolidMotor::load_from_str(M1670_TOML).expect("M1670 deck must parse");
    assert_eq!(m.meta().name, "cesaroni-m1670");
    assert_eq!(m.burn_duration_s(), 3.9);
    assert_eq!(m.total_impulse_n_s(), 6026.350);
    assert_eq!(m.propellant_mass_kg(), 3.101);
    assert_eq!(m.dry_mass_kg(), 2.130);
}

#[test]
fn rocketpy_calisto_m1670_toml_parses_with_rocketpy_mass_profile() {
    let m = SolidMotor::load_from_str(ROCKETPY_CALISTO_M1670_TOML)
        .expect("RocketPy Calisto M1670 deck must parse");
    assert_eq!(m.meta().name, "rocketpy-calisto-m1670");
    assert_eq!(m.burn_duration_s(), 3.9);
    assert_eq!(m.total_impulse_n_s(), 6026.350);
    assert_eq!(m.propellant_mass_kg(), 2.955_911_961_392_022_4);
    assert_eq!(m.dry_mass_kg(), 1.815);
    assert_eq!(m.initial_mass_kg(), 4.770_911_961_392_022);
}

#[test]
fn cesaroni_m1670_integrated_impulse_matches_declared_to_bit_precision() {
    let m = SolidMotor::load_from_str(M1670_TOML).expect("M1670 deck must parse");
    let integrated = m.thrust_curve().integrated_impulse_n_s();
    let declared = m.total_impulse_n_s();
    let rel = (integrated - declared).abs() / declared.abs();
    assert!(
        rel < 1.0e-12,
        "integrated impulse {integrated} differs from declared {declared} by {rel} relative",
    );
}

#[test]
fn rocketpy_calisto_m1670_integrated_impulse_matches_declared_to_bit_precision() {
    let m = SolidMotor::load_from_str(ROCKETPY_CALISTO_M1670_TOML)
        .expect("RocketPy Calisto M1670 deck must parse");
    let integrated = m.thrust_curve().integrated_impulse_n_s();
    let declared = m.total_impulse_n_s();
    let rel = (integrated - declared).abs() / declared.abs();
    assert!(
        rel < 1.0e-12,
        "integrated impulse {integrated} differs from declared {declared} by {rel} relative",
    );
}

#[test]
fn cesaroni_m1670_eng_sha256_matches_provenance_pin() {
    let mut hasher = Sha256::new();
    hasher.update(M1670_ENG.as_bytes());
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(64);
    for byte in digest {
        write!(hex, "{byte:02x}").expect("hex write");
    }
    assert_eq!(
        hex, M1670_ENG_SHA256,
        "upstream Cesaroni_M1670.eng SHA-256 changed; update provenance.md after confirming with ThrustCurve",
    );
}

#[test]
fn cesaroni_m1670_toml_thrust_curve_round_trips_against_upstream_eng() {
    // Parse the upstream .eng file and assert the (time, thrust)
    // points in our TOML match (modulo the OpenBMP-required
    // (0, 0) prepend at the start).
    assert_toml_thrust_curve_round_trips_against_upstream_eng(M1670_TOML);
}

#[test]
fn rocketpy_calisto_m1670_toml_thrust_curve_round_trips_against_upstream_eng() {
    assert_toml_thrust_curve_round_trips_against_upstream_eng(ROCKETPY_CALISTO_M1670_TOML);
}

fn assert_toml_thrust_curve_round_trips_against_upstream_eng(toml: &str) {
    let m = SolidMotor::load_from_str(toml).expect("M1670 deck must parse");
    let toml_points: Vec<(f64, f64)> = m
        .thrust_curve()
        .points()
        .iter()
        .map(|p| (p[0], p[1]))
        .collect();

    let mut eng_points: Vec<(f64, f64)> = Vec::new();
    for (line_idx, line) in M1670_ENG.lines().enumerate() {
        if line_idx == 0 {
            // RASP header line — skip.
            continue;
        }
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let fields: Vec<&str> = trimmed.split_whitespace().collect();
        assert_eq!(
            fields.len(),
            2,
            ".eng line {line_idx} has unexpected field count: {trimmed}"
        );
        let t: f64 = fields[0].parse().expect(".eng time");
        let th: f64 = fields[1].parse().expect(".eng thrust");
        eng_points.push((t, th));
    }

    // OpenBMP requires (0, 0) at index 0; the upstream .eng omits it.
    assert_eq!(
        toml_points[0],
        (0.0, 0.0),
        "OpenBMP TOML must start at (0,0)"
    );
    assert_eq!(
        toml_points.len() - 1,
        eng_points.len(),
        "TOML has {} points (including (0,0) prepend); upstream .eng has {}",
        toml_points.len(),
        eng_points.len()
    );
    for (i, (eng, toml)) in eng_points
        .iter()
        .zip(toml_points.iter().skip(1))
        .enumerate()
    {
        assert_eq!(
            eng.0.to_bits(),
            toml.0.to_bits(),
            "row {i}: time mismatch: .eng = {} vs TOML = {}",
            eng.0,
            toml.0
        );
        assert_eq!(
            eng.1.to_bits(),
            toml.1.to_bits(),
            "row {i}: thrust mismatch: .eng = {} vs TOML = {}",
            eng.1,
            toml.1
        );
    }
}
