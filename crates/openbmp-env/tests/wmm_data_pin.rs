//! Phase-3.10.A regression test: the in-source `COEFFS` table in
//! `openbmp_env::magnetic::wmm2025::coefficients` agrees byte-for-
//! byte with `data/magnetic/WMM.COF`, and the file's SHA-256 digest
//! matches the `provenance.md` pin.
//!
//! The test ships under `tests/` (integration tests) so it only
//! runs at `cargo test` time and the data files don't need to be
//! readable on the kernel hot path.

#![allow(clippy::expect_used, clippy::float_cmp, clippy::unwrap_used)]

use openbmp_env::Wmm2025;

const WMM_COF: &str = include_str!("../../../data/magnetic/WMM.COF");

/// SHA-256 pin recorded in `data/magnetic/provenance.md`.
const WMM_COF_SHA256_HEX: &str = "dfa8597825af4e0b87ff4198a5b4fb661b3c49f4cd090cd0164e0259b075582f";

#[test]
fn wmm_cof_sha256_matches_provenance_pin() {
    use std::fmt::Write;
    // Hand-rolled SHA-256 to avoid pulling a hash crate just for the
    // test. The standard library doesn't ship SHA-256 but the
    // workspace's `sha2` is already wired (see Phase-2.10
    // SHA-256 pin contract). Use it through a minimal local path.
    use sha2::{Digest, Sha256};
    let mut hasher = Sha256::new();
    hasher.update(WMM_COF.as_bytes());
    let digest = hasher.finalize();
    let mut hex = String::with_capacity(64);
    for byte in digest {
        write!(hex, "{byte:02x}").expect("hex write");
    }
    assert_eq!(
        hex, WMM_COF_SHA256_HEX,
        "WMM.COF SHA-256 changed; update provenance.md (and confirm with NOAA)"
    );
}

#[test]
fn wmm_cof_round_trips_into_coefficients_table() {
    // Parse the WMM.COF file line-by-line and verify each entry
    // matches the in-source `COEFFS` table.
    let mut file_rows: Vec<(u8, u8, f64, f64, f64, f64)> = Vec::new();
    let mut lines = WMM_COF.lines();
    let header = lines.next().expect("WMM.COF must have header line");
    assert!(
        header.contains("WMM-2025"),
        "header is not WMM-2025: {header}"
    );
    for line in lines {
        let line = line.trim();
        if line.is_empty() || line.starts_with("999999") {
            break;
        }
        let parts: Vec<&str> = line.split_whitespace().collect();
        assert_eq!(parts.len(), 6, "WMM row has unexpected field count: {line}");
        let n: u8 = parts[0].parse().expect("n");
        let m: u8 = parts[1].parse().expect("m");
        let g: f64 = parts[2].parse().expect("g");
        let h: f64 = parts[3].parse().expect("h");
        let g_dot: f64 = parts[4].parse().expect("g_dot");
        let h_dot: f64 = parts[5].parse().expect("h_dot");
        file_rows.push((n, m, g, h, g_dot, h_dot));
    }
    let coeffs = Wmm2025::coefficients();
    assert_eq!(
        file_rows.len(),
        coeffs.len(),
        "row count mismatch between WMM.COF ({}) and COEFFS ({})",
        file_rows.len(),
        coeffs.len()
    );
    for (file_row, source_row) in file_rows.iter().zip(coeffs.iter()) {
        assert_eq!(file_row.0, source_row.n, "n mismatch");
        assert_eq!(file_row.1, source_row.m, "m mismatch");
        assert_eq!(
            file_row.2.to_bits(),
            source_row.g_nt.to_bits(),
            "g mismatch at (n={}, m={})",
            file_row.0,
            file_row.1
        );
        assert_eq!(
            file_row.3.to_bits(),
            source_row.h_nt.to_bits(),
            "h mismatch at (n={}, m={})",
            file_row.0,
            file_row.1
        );
        assert_eq!(
            file_row.4.to_bits(),
            source_row.g_dot_nt_per_year.to_bits(),
            "g_dot mismatch at (n={}, m={})",
            file_row.0,
            file_row.1
        );
        assert_eq!(
            file_row.5.to_bits(),
            source_row.h_dot_nt_per_year.to_bits(),
            "h_dot mismatch at (n={}, m={})",
            file_row.0,
            file_row.1
        );
    }
}
