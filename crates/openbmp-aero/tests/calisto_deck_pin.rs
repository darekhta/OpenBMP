//! Phase-3.11.B regression test for the Calisto drag-curve deck.
//!
//! Pins:
//!  - The OpenBMP TOML at `data/aero/calisto-drag.toml` parses
//!    cleanly through `AeroDeck::load_from_str`.
//!  - Every (mach, CD) pair in the TOML round-trips against the
//!    upstream RocketPy CSV at `data/aero/calisto-drag.csv` to bit
//!    precision.
//!  - The upstream CSV's SHA-256 matches the pin recorded in
//!    `data/aero/provenance.md`.
//!  - CN and CM are identically zero on every grid point (the deck
//!    is an axisymmetric reduced point-mass model).
//!  - CD ≥ 0 on every grid point.

#![allow(
    clippy::expect_used,
    clippy::unwrap_used,
    clippy::float_cmp,
    clippy::panic
)]

use openbmp_aero::AeroDeck;
use sha2::{Digest, Sha256};
use std::collections::BTreeMap;
use std::fmt::Write;

const CALISTO_TOML: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/aero/calisto-drag.toml"
));

const CALISTO_CSV: &str = include_str!(concat!(
    env!("CARGO_MANIFEST_DIR"),
    "/../../data/aero/calisto-drag.csv"
));

const CALISTO_CSV_SHA256: &str = "94760a42d5f5fad4fb815f448db72201fafd2b2a3c6ec2d2c83618b2953207e0";

fn parse_csv() -> Vec<(f64, f64)> {
    let mut rows = Vec::new();
    for (line_idx, line) in CALISTO_CSV.lines().enumerate() {
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        let fields: Vec<&str> = trimmed.split(',').collect();
        assert_eq!(
            fields.len(),
            2,
            "calisto-drag.csv line {} has unexpected field count: {trimmed}",
            line_idx + 1
        );
        let m: f64 = fields[0].parse().expect("mach parse");
        let cd: f64 = fields[1].parse().expect("cd parse");
        rows.push((m, cd));
    }
    rows
}

#[test]
fn calisto_drag_csv_sha256_matches_provenance_pin() {
    let mut hasher = Sha256::new();
    hasher.update(CALISTO_CSV.as_bytes());
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(64);
    for byte in digest {
        write!(hex, "{byte:02x}").expect("hex write");
    }
    assert_eq!(
        hex, CALISTO_CSV_SHA256,
        "upstream RocketPy Calisto drag CSV SHA-256 changed; update provenance.md after confirming with RocketPy",
    );
}

#[test]
fn calisto_drag_toml_parses_with_expected_grid() {
    let deck = AeroDeck::load_from_str(CALISTO_TOML).expect("Calisto deck must parse");
    assert_eq!(deck.mach_grid().len(), 200, "expected 200-point Mach grid");
    assert_eq!(deck.mach_grid()[0], 0.01);
    assert_eq!(deck.mach_grid()[199], 2.0);
}

#[test]
fn calisto_cd_round_trips_against_upstream_csv() {
    let csv_rows = parse_csv();
    assert_eq!(csv_rows.len(), 200);
    let deck = AeroDeck::load_from_str(CALISTO_TOML).expect("Calisto deck must parse");
    let no_deflections: BTreeMap<&str, f64> = BTreeMap::new();
    for (i, (mach, expected_cd)) in csv_rows.iter().enumerate() {
        let coeffs = deck
            .lookup(*mach, 0.0, 0.0, &no_deflections)
            .expect("lookup");
        assert_eq!(
            coeffs.cd.to_bits(),
            expected_cd.to_bits(),
            "row {i}: mach {mach}: deck CD {} differs from CSV CD {}",
            coeffs.cd,
            expected_cd
        );
    }
}

#[test]
fn calisto_cn_and_cm_are_zero_everywhere() {
    let deck = AeroDeck::load_from_str(CALISTO_TOML).expect("Calisto deck must parse");
    let no_deflections: BTreeMap<&str, f64> = BTreeMap::new();
    for &mach in deck.mach_grid() {
        let coeffs = deck
            .lookup(mach, 0.0, 0.0, &no_deflections)
            .expect("lookup");
        assert_eq!(
            coeffs.cn.to_bits(),
            0.0_f64.to_bits(),
            "axisymmetric deck must report CN = 0 (mach {mach})"
        );
        assert_eq!(
            coeffs.cm.to_bits(),
            0.0_f64.to_bits(),
            "axisymmetric deck must report CM = 0 (mach {mach})"
        );
    }
}

#[test]
fn calisto_cd_is_non_negative_everywhere() {
    let deck = AeroDeck::load_from_str(CALISTO_TOML).expect("Calisto deck must parse");
    let no_deflections: BTreeMap<&str, f64> = BTreeMap::new();
    for &mach in deck.mach_grid() {
        let coeffs = deck
            .lookup(mach, 0.0, 0.0, &no_deflections)
            .expect("lookup");
        assert!(
            coeffs.cd >= 0.0,
            "CD must be non-negative (got {} at mach {mach})",
            coeffs.cd
        );
    }
}
