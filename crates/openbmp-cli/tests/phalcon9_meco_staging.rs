//! First-principles staging-velocity guard for the Falcon-class
//! `phalcon9-orbit` scenario.
//!
//! A two-stage kerolox launch vehicle stages (MECO-1) at a
//! ground-relative speed of roughly 2-3 km/s: the dV-balanced split
//! where a credible booster hands off to the upper stage. This guard
//! runs the synthetic Phalcon-9 ascent and asserts its MECO-1
//! ground-relative speed falls in that physical band.
//!
//! It needs NO external data. The band is a first-principles property of
//! staged chemical rockets, not a fit to any flight: it merely encodes
//! that the booster must do a booster's share of the work and no more.
//! It locks in the realistic stage split, so a future edit that pushes
//! staging back toward a near-orbital booster cutoff (the unphysical
//! "stage-1 does almost everything" split) fails CI.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
// Single-sourced WGS84 Earth-rotation rate (avoids inlining the digits
// and satisfies the inline-data tripwire).
use openbmp_physics::frames::WGS84_OMEGA_RAD_S;

/// Lower edge of the physical two-stage-kerolox staging band (m/s,
/// ground-relative). Below this the booster has done too little to be a
/// credible first stage.
const MECO_GROUND_REL_MIN_M_S: f64 = 1_800.0;
/// Upper edge of the band (m/s, ground-relative). Above this the booster
/// is doing the upper stage's job — the near-orbital-booster split that
/// external cross-validation flagged as unphysical for a kerolox TSTO.
const MECO_GROUND_REL_MAX_M_S: f64 = 3_000.0;

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
}

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/phalcon9/phalcon9-orbit.toml")
}

/// Ground-relative speed at the first `meco1` telemetry marker.
///
/// Removes uniform Earth rotation from the inertial state:
/// `|v_eci - omega x r_eci|` with `omega` about ECI +z. This is the
/// webcast / surface-relative convention staging speeds are quoted in.
fn meco_ground_relative_speed(csv_path: &Path) -> f64 {
    let text = fs::read_to_string(csv_path).expect("read phalcon9-orbit CSV");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("csv header").split(',').collect();
    let col = |name: &str| {
        header
            .iter()
            .position(|h| *h == name)
            .unwrap_or_else(|| panic!("column {name}"))
    };
    let (pxi, pyi, pzi) = (
        col("position_x_m"),
        col("position_y_m"),
        col("position_z_m"),
    );
    let (vxi, vyi, vzi) = (
        col("velocity_x_m_s"),
        col("velocity_y_m_s"),
        col("velocity_z_m_s"),
    );
    let meco_i = col("mission.marker.meco1");

    for line in lines {
        if line.trim().is_empty() {
            continue;
        }
        let fields: Vec<&str> = line.split(',').collect();
        if fields.get(meco_i).map(|s| s.trim()) != Some("true") {
            continue;
        }
        let f = |i: usize| {
            fields[i]
                .trim()
                .parse::<f64>()
                .unwrap_or_else(|_| panic!("parse float at column {i}"))
        };
        let r = [f(pxi), f(pyi), f(pzi)];
        let v = [f(vxi), f(vyi), f(vzi)];
        // omega x r = [-omega*ry, omega*rx, 0]; v_surface = v - omega x r.
        let w = WGS84_OMEGA_RAD_S;
        let sx = v[0] + w * r[1];
        let sy = v[1] - w * r[0];
        let sz = v[2];
        return (sx * sx + sy * sy + sz * sz).sqrt();
    }
    panic!(
        "phalcon9-orbit produced no meco1 marker: stage-1 never reached the MECO trigger \
         (the booster is mis-sized or the staging trigger is unreachable)"
    );
}

#[test]
fn phalcon9_meco_stages_in_physical_kerolox_band() {
    let report = run::run(&scenario_path()).expect("phalcon9-orbit must run to completion");
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output path in report");

    let meco_speed = meco_ground_relative_speed(csv);

    assert!(
        (MECO_GROUND_REL_MIN_M_S..=MECO_GROUND_REL_MAX_M_S).contains(&meco_speed),
        "MECO-1 ground-relative speed {meco_speed:.0} m/s is outside the physical \
         two-stage-kerolox staging band [{MECO_GROUND_REL_MIN_M_S:.0}, \
         {MECO_GROUND_REL_MAX_M_S:.0}] m/s; the booster must hand off in this range, not at a \
         near-orbital cutoff"
    );
}
