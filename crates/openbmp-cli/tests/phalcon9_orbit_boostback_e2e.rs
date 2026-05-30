//! End-to-end check for `phalcon9-orbit-boostback`: an INTEGRATED booster
//! boostback — in the same run as the ascent, the separated booster flips
//! retrograde (separation attitude offset) and fires a dedicated boostback
//! engine (scripted ignite→cut) to decelerate.
//!
//! Asserts:
//!   1. the ascent (upper stage) still reaches a bound near-circular LEO;
//!   2. the booster lane BURNS during the scripted boostback window (its
//!      dedicated propellant depletes — a real engine burn, not an impulse);
//!   3. the burn DECELERATES the booster (its speed drops materially across
//!      the burn) — the boostback maneuver, observed live on the separated
//!      lane in the same run as the ascent.
//!
//! This is a DECELERATION-ONLY boostback: a scripted retrograde Δv that sheds
//! velocity, with the booster's landing point an emergent ballistic
//! consequence — never an input. A *guided* boostback flown to a landing
//! site/pad would be a ground aimpoint (the same math as terminal targeting)
//! and is out of scope by doctrine, not an unfinished feature — see
//! docs/dual-use-assessment.md §4 and docs/launch-vehicle-fidelity-frontier.md.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

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
        .join("scenarios/phalcon9/phalcon9-orbit-boostback.toml")
}

struct Row {
    t: f64,
    r: [f64; 3],
    v: [f64; 3],
    booster_mass: f64,
    booster_speed: f64,
}

fn parse(csv: &Path) -> Vec<Row> {
    let text = fs::read_to_string(csv).expect("read CSV");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("header").split(',').collect();
    let col = |n: &str| {
        header
            .iter()
            .position(|h| *h == n)
            .unwrap_or_else(|| panic!("col {n}"))
    };
    let (ti, px, py, pz, vx, vy, vz) = (
        col("time_s"),
        col("position_x_m"),
        col("position_y_m"),
        col("position_z_m"),
        col("velocity_x_m_s"),
        col("velocity_y_m_s"),
        col("velocity_z_m_s"),
    );
    let (bm, bvx, bvy, bvz) = (
        col("body.lower.mass_kg"),
        col("body.lower.velocity_x_m_s"),
        col("body.lower.velocity_y_m_s"),
        col("body.lower.velocity_z_m_s"),
    );
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<f64> = l
                .split(',')
                .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
                .collect();
            Row {
                t: f[ti],
                r: [f[px], f[py], f[pz]],
                v: [f[vx], f[vy], f[vz]],
                booster_mass: f[bm],
                booster_speed: (f[bvx] * f[bvx] + f[bvy] * f[bvy] + f[bvz] * f[bvz]).sqrt(),
            }
        })
        .collect()
}

fn at<'a>(rows: &'a [Row], t: f64) -> &'a Row {
    rows.iter()
        .min_by(|a, b| (a.t - t).abs().partial_cmp(&(b.t - t).abs()).unwrap())
        .expect("row")
}

#[test]
fn phalcon9_boostback_ascends_to_orbit_and_booster_burns_retrograde() {
    let report = run::run(&scenario_path()).expect("phalcon9-orbit-boostback must run");
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output");
    let rows = parse(csv);
    assert!(rows.len() > 100);

    // (1) Ascent reaches a bound near-circular LEO.
    let last = rows.last().expect("final");
    let r = (last.r[0] * last.r[0] + last.r[1] * last.r[1] + last.r[2] * last.r[2]).sqrt();
    let speed_sq = last.v.iter().map(|c| c * c).sum::<f64>();
    let eps = 0.5 * speed_sq - MU_EARTH / r;
    let a = -MU_EARTH / (2.0 * eps);
    let h = [
        last.r[1] * last.v[2] - last.r[2] * last.v[1],
        last.r[2] * last.v[0] - last.r[0] * last.v[2],
        last.r[0] * last.v[1] - last.r[1] * last.v[0],
    ];
    let h_mag = (h[0] * h[0] + h[1] * h[1] + h[2] * h[2]).sqrt();
    let e = (1.0 + 2.0 * eps * h_mag * h_mag / (MU_EARTH * MU_EARTH))
        .max(0.0)
        .sqrt();
    let perigee_km = (a * (1.0 - e) - EARTH_MEAN_RADIUS_M) / 1000.0;
    assert!(eps < 0.0, "ascent must reach a bound orbit");
    assert!(e < 0.05, "ascent must be near-circular, e={e:.4}");
    assert!(
        (150.0..500.0).contains(&perigee_km),
        "ascent perigee {perigee_km:.1} km must be LEO"
    );

    // (2) The booster burns its dedicated boostback propellant across the
    //     scripted window (ignite 250 s → cut 290 s).
    let m_before = at(&rows, 248.0).booster_mass;
    let m_after = at(&rows, 292.0).booster_mass;
    let burned = m_before - m_after;
    assert!(
        burned > 3000.0,
        "boostback engine must burn its reserve; burned {burned:.0} kg"
    );
    // ...and the burn ends (mass flat after the cut).
    let m_coast = at(&rows, 350.0).booster_mass;
    assert!(
        (m_after - m_coast).abs() < 1.0,
        "boostback engine must cut (mass flat after 290 s)"
    );

    // (3) The burn DECELERATES the booster — the boostback maneuver.
    let v_before = at(&rows, 248.0).booster_speed;
    let v_after = at(&rows, 292.0).booster_speed;
    let decel = v_before - v_after;
    assert!(
        decel > 150.0,
        "retrograde boostback burn must materially decelerate the booster; Δ|v| {decel:.0} m/s"
    );

    println!(
        "phalcon9-orbit-boostback: ascent perigee {perigee_km:.1} km e {e:.4}; \
         booster boostback burned {burned:.0} kg, decelerated {decel:.0} m/s"
    );
}
