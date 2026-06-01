//! End-to-end check that a RESTARTABLE liquid engine does two distinct
//! burns separated by a coast — the runner/kernel exercise of the
//! `limits.restartable` engine policy (a shut-down engine re-arms and
//! re-ignites, which the default one-shot lifecycle forbids).
//!
//! Asserts on the `engine-restart` scenario (vertical point-mass, constant
//! gravity, no atmosphere): propellant is consumed during the first burn,
//! the mass is FLAT during the mid-coast (engine genuinely off), and
//! propellant is consumed AGAIN during the restart burn.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;

fn scenario_path() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
        .join("scenarios/multi-engine-octaweb/engine-restart.toml")
}

fn mass_series(csv: &Path) -> Vec<(f64, f64)> {
    let text = fs::read_to_string(csv).expect("read CSV");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("header").split(',').collect();
    let col = |n: &str| {
        header
            .iter()
            .position(|h| *h == n)
            .unwrap_or_else(|| panic!("col {n}"))
    };
    let (ti, mi) = (col("time_s"), col("mass_kg"));
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<f64> = l
                .split(',')
                .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
                .collect();
            (f[ti], f[mi])
        })
        .collect()
}

fn mass_at(series: &[(f64, f64)], t: f64) -> f64 {
    series
        .iter()
        .min_by(|a, b| (a.0 - t).abs().total_cmp(&(b.0 - t).abs()))
        .expect("sample")
        .1
}

#[test]
fn engine_restart_does_two_burns_separated_by_coast() {
    let report = run::run(&scenario_path()).expect("engine-restart must run");
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output");
    let series = mass_series(csv);
    assert!(series.len() > 1000);

    // First burn: 0.5 -> 2.9 s consumes propellant.
    let m_burn1_start = mass_at(&series, 0.5);
    let m_burn1_end = mass_at(&series, 2.9);
    let burn1 = m_burn1_start - m_burn1_end;
    assert!(
        burn1 > 1.0,
        "first burn must consume propellant; consumed {burn1:.3} kg"
    );

    // Mid-coast: after the shutdown transient completes (~3.1 s) the engine
    // is off — mass is flat through to the restart at 6.0 s.
    let m_coast_a = mass_at(&series, 3.5);
    let m_coast_b = mass_at(&series, 5.9);
    let coast = m_coast_a - m_coast_b;
    assert!(
        coast.abs() < 1.0e-3,
        "mass must be flat during the coast (engine off): {m_coast_a:.4} -> {m_coast_b:.4} kg"
    );

    // RESTART: second burn 6.0 -> 8.9 s consumes propellant again — only
    // possible because the engine re-armed after shutdown.
    let m_burn2_start = mass_at(&series, 6.1);
    let m_burn2_end = mass_at(&series, 8.9);
    let burn2 = m_burn2_start - m_burn2_end;
    assert!(
        burn2 > 1.0,
        "restart burn must consume propellant; consumed {burn2:.3} kg"
    );

    println!(
        "engine-restart: burn1 {burn1:.2} kg, coast |Δm| {:.4} kg, restart burn2 {burn2:.2} kg",
        coast.abs()
    );
}
