//! End-to-end check for `phalcon9-orbit-staged`: the full discrete mission
//! sequence (booster separation, payload-fairing jettison in coast, payload
//! deploy in orbit) on the synthetic Phalcon-9.
//!
//! This is the regression guard for the multi-body continuing-stack mass
//! model. The lumped upper body is split into stage-2 dry + fairing +
//! payload (inert bodies), and the fairing/payload are jettisoned at
//! distinct times. The test asserts:
//!   1. a bound, near-circular, near-equatorial LEO insertion is still
//!      reached;
//!   2. each jettison genuinely SHEDS its body's mass from the continuing
//!      stack (the active vehicle mass drops by the body's dry mass) — the
//!      bug this fix closes was a cosmetic jettison that shed nothing;
//!   3. TOTAL mass (continuing stack + every separated lane) is CONSERVED
//!      across each separation (no mass leak), changing only through
//!      propellant burn.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::collections::BTreeMap;
use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
use openbmp_physics::frames::WGS84_MU_M3_S2;

const MU_EARTH: f64 = WGS84_MU_M3_S2;
const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

fn scenario_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
        .join("scenarios/phalcon9/phalcon9-orbit-staged.toml")
}

/// One CSV row reduced to the fields we need: time, ECI state, and every
/// `*.mass_kg` column (primary `mass_kg` plus each separated body lane).
struct Row {
    t: f64,
    r: [f64; 3],
    v: [f64; 3],
    primary_mass: f64,
    total_mass: f64,
}

fn parse_rows(csv: &Path) -> Vec<Row> {
    let text = fs::read_to_string(csv).expect("read CSV");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("header").split(',').collect();
    let idx: BTreeMap<&str, usize> = header.iter().enumerate().map(|(i, h)| (*h, i)).collect();
    let col = |n: &str| *idx.get(n).unwrap_or_else(|| panic!("column {n}"));
    let (ti, pxi, pyi, pzi, vxi, vyi, vzi, mi) = (
        col("time_s"),
        col("position_x_m"),
        col("position_y_m"),
        col("position_z_m"),
        col("velocity_x_m_s"),
        col("velocity_y_m_s"),
        col("velocity_z_m_s"),
        col("mass_kg"),
    );
    // Every separated-lane mass column (body.<id>.mass_kg).
    let lane_cols: Vec<usize> = header
        .iter()
        .enumerate()
        .filter(|(_, h)| h.starts_with("body.") && h.ends_with(".mass_kg"))
        .map(|(i, _)| i)
        .collect();

    let mut out = Vec::new();
    for line in lines.filter(|l| !l.trim().is_empty()) {
        let f: Vec<f64> = line
            .split(',')
            .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
            .collect();
        let primary_mass = f[mi];
        let total_mass = primary_mass + lane_cols.iter().map(|&c| f[c]).sum::<f64>();
        out.push(Row {
            t: f[ti],
            r: [f[pxi], f[pyi], f[pzi]],
            v: [f[vxi], f[vyi], f[vzi]],
            primary_mass,
            total_mass,
        });
    }
    out
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn mass_at(rows: &[Row], t: f64, primary: bool) -> f64 {
    let row = rows
        .iter()
        .min_by(|a, b| (a.t - t).abs().total_cmp(&(b.t - t).abs()))
        .expect("row near t");
    if primary {
        row.primary_mass
    } else {
        row.total_mass
    }
}

#[test]
fn phalcon9_orbit_staged_sheds_mass_and_conserves_total() {
    let report = run::run(&scenario_path()).expect("phalcon9-orbit-staged must run");
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output");
    let rows = parse_rows(csv);
    assert!(rows.len() > 100);

    // (1) Bound, near-circular, near-equatorial LEO insertion.
    let last = rows.last().expect("final row");
    let r = norm(last.r);
    let speed_sq = last.v.iter().map(|c| c * c).sum::<f64>();
    let eps = 0.5 * speed_sq - MU_EARTH / r;
    let a = -MU_EARTH / (2.0 * eps);
    let h = [
        last.r[1] * last.v[2] - last.r[2] * last.v[1],
        last.r[2] * last.v[0] - last.r[0] * last.v[2],
        last.r[0] * last.v[1] - last.r[1] * last.v[0],
    ];
    let h_mag = norm(h);
    let e = (1.0 + 2.0 * eps * h_mag * h_mag / (MU_EARTH * MU_EARTH))
        .max(0.0)
        .sqrt();
    let perigee_km = (a * (1.0 - e) - EARTH_MEAN_RADIUS_M) / 1000.0;
    let incl_deg = (h[2] / h_mag).clamp(-1.0, 1.0).acos().to_degrees();
    assert!(eps < 0.0, "insertion must be bound");
    assert!(e < 0.02, "insertion must be near-circular, e={e:.4}");
    assert!(
        (150.0..600.0).contains(&perigee_km),
        "perigee {perigee_km:.1} km must be LEO"
    );
    assert!(
        incl_deg < 1.0,
        "inclination {incl_deg:.3} deg must stay near-equatorial"
    );

    // (2) Each jettison sheds its body's dry mass from the continuing stack.
    // Fairing (600 kg) jettisons at t=320; payload (1500 kg) at t=900.
    let primary_before_fairing = mass_at(&rows, 318.0, true);
    let primary_after_fairing = mass_at(&rows, 322.0, true);
    let fairing_shed = primary_before_fairing - primary_after_fairing;
    assert!(
        (fairing_shed - 600.0).abs() < 1.0,
        "fairing jettison must shed ~600 kg from the stack; shed {fairing_shed:.1} kg"
    );
    let primary_before_payload = mass_at(&rows, 898.0, true);
    let primary_after_payload = mass_at(&rows, 902.0, true);
    let payload_shed = primary_before_payload - primary_after_payload;
    assert!(
        (payload_shed - 1500.0).abs() < 1.0,
        "payload deploy must shed ~1500 kg from the stack; shed {payload_shed:.1} kg"
    );

    // (3) Total mass (stack + every separated lane) is CONSERVED across each
    // separation — the shed mass moves to a lane, it does not vanish. (Away
    // from the stage-2 burn the only total-mass change is propellant; both
    // jettisons happen with engines off, so total is flat across them.)
    let total_before_fairing = mass_at(&rows, 318.0, false);
    let total_after_fairing = mass_at(&rows, 322.0, false);
    assert!(
        (total_before_fairing - total_after_fairing).abs() < 1.0e-3,
        "total mass must be conserved across fairing jettison: {total_before_fairing:.3} -> {total_after_fairing:.3}"
    );
    let total_before_payload = mass_at(&rows, 898.0, false);
    let total_after_payload = mass_at(&rows, 902.0, false);
    assert!(
        (total_before_payload - total_after_payload).abs() < 1.0e-3,
        "total mass must be conserved across payload deploy: {total_before_payload:.3} -> {total_after_payload:.3}"
    );

    println!(
        "phalcon9-orbit-staged: perigee {perigee_km:.1} km, e {e:.4}, incl {incl_deg:.3} deg; \
         fairing shed {fairing_shed:.1} kg, payload shed {payload_shed:.1} kg, total conserved"
    );
}
