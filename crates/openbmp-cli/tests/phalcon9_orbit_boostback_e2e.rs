//! End-to-end check for `phalcon9-orbit-boostback`: an INTEGRATED booster
//! boostback — in the same run as the ascent, the separated booster flips
//! retrograde (separation attitude offset) and fires a dedicated boostback
//! engine from separated-lane range/relative-speed triggers to decelerate.
//!
//! Asserts:
//!   1. the ascent (upper stage) still reaches a bound near-circular LEO;
//!   2. the booster lane BURNS after the lane-local range trigger and cuts
//!      at the relative-speed residual (its dedicated propellant depletes —
//!      a real engine burn, not an impulse);
//!   3. the burn DECELERATES the booster (its speed drops materially across
//!      the burn) — the boostback maneuver, observed live on the separated
//!      lane in the same run as the ascent.
//!
//! This is a DECELERATION-ONLY boostback: a retrograde Δv that sheds
//! velocity, with the booster's terminal descent an emergent consequence.
//! The SIL package records entry/terminal/ground-crossing recovery regions
//! from the separated booster lane, but a guided landing-site controller
//! remains future work; see docs/launch-vehicle-fidelity-frontier.md.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
use openbmp_physics::frames::WGS84_MU_M3_S2;
use openbmp_sil::{MissionPackage, capture_bus_frames, read_signal};

const MU_EARTH: f64 = WGS84_MU_M3_S2;
const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

fn scenario_path() -> PathBuf {
    workspace_root().join("scenarios/phalcon9/phalcon9-orbit-boostback.toml")
}

fn package_path() -> PathBuf {
    workspace_root().join("scenarios/phalcon9/mission-package-boostback.toml")
}

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
}

struct Row {
    r: [f64; 3],
    v: [f64; 3],
    booster_mass: f64,
    booster_speed: f64,
    boost_thrust: f64,
    booster_upper_range: f64,
    booster_upper_relative_speed: f64,
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
    let (px, py, pz, vx, vy, vz) = (
        col("position_x_m"),
        col("position_y_m"),
        col("position_z_m"),
        col("velocity_x_m_s"),
        col("velocity_y_m_s"),
        col("velocity_z_m_s"),
    );
    let (bm, bpx, bpy, bpz, bvx, bvy, bvz, bt) = (
        col("body.lower.mass_kg"),
        col("body.lower.position_x_m"),
        col("body.lower.position_y_m"),
        col("body.lower.position_z_m"),
        col("body.lower.velocity_x_m_s"),
        col("body.lower.velocity_y_m_s"),
        col("body.lower.velocity_z_m_s"),
        col("engine.eng_boost.thrust_n"),
    );
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<f64> = l
                .split(',')
                .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
                .collect();
            let r = [f[px], f[py], f[pz]];
            let v = [f[vx], f[vy], f[vz]];
            let booster_r = [f[bpx], f[bpy], f[bpz]];
            let booster_v = [f[bvx], f[bvy], f[bvz]];
            let booster_upper_range = ((booster_r[0] - r[0]).powi(2)
                + (booster_r[1] - r[1]).powi(2)
                + (booster_r[2] - r[2]).powi(2))
            .sqrt();
            let booster_upper_relative_speed = ((booster_v[0] - v[0]).powi(2)
                + (booster_v[1] - v[1]).powi(2)
                + (booster_v[2] - v[2]).powi(2))
            .sqrt();
            Row {
                r,
                v,
                booster_mass: f[bm],
                booster_speed: (booster_v[0] * booster_v[0]
                    + booster_v[1] * booster_v[1]
                    + booster_v[2] * booster_v[2])
                    .sqrt(),
                boost_thrust: f[bt],
                booster_upper_range,
                booster_upper_relative_speed,
            }
        })
        .collect()
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

    let active_indices: Vec<usize> = rows
        .iter()
        .enumerate()
        .filter_map(|(index, row)| (row.boost_thrust > 1.0).then_some(index))
        .collect();
    assert!(!active_indices.is_empty(), "boostback engine must light");
    let first_active = *active_indices.first().expect("first active");
    let last_active = *active_indices.last().expect("last active");
    let burn_start = &rows[first_active];
    let burn_end = &rows[last_active];
    assert!(
        burn_start.booster_upper_range >= 600.0,
        "boostback ignition must wait for separated-lane range trigger; range {:.0} m",
        burn_start.booster_upper_range
    );
    assert!(
        burn_end.booster_upper_relative_speed >= 1200.0,
        "boostback cut must wait for relative-speed residual trigger; relative speed {:.0} m/s",
        burn_end.booster_upper_relative_speed
    );

    // (2) The booster burns its dedicated boostback propellant across the
    //     lane-local trigger window.
    let m_before = rows[first_active.saturating_sub(1)].booster_mass;
    let m_after = rows[(last_active + 1).min(rows.len() - 1)].booster_mass;
    let burned = m_before - m_after;
    assert!(
        burned > 3000.0,
        "boostback engine must burn its reserve; burned {burned:.0} kg"
    );
    // ...and the burn ends (mass flat after the cut).
    let m_coast = rows[(last_active + 500).min(rows.len() - 1)].booster_mass;
    assert!(
        (m_after - m_coast).abs() < 1.0,
        "boostback engine must cut (mass flat after relative-speed trigger)"
    );

    // (3) The burn DECELERATES the booster — the boostback maneuver.
    let v_before = rows[first_active.saturating_sub(1)].booster_speed;
    let v_after = rows[(last_active + 1).min(rows.len() - 1)].booster_speed;
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

#[test]
fn phalcon9_boostback_sil_evidence_tracks_lane_regions_and_bus_frames() {
    let package = MissionPackage::load(package_path()).expect("load boostback package");
    package.check().expect("boostback package check");
    let report = package
        .step_case(Some("orbit-boostback"), 25_000)
        .expect("step boostback package through booster ground crossing");

    let booster_states: Vec<&str> = report
        .evidence
        .region_trace
        .iter()
        .filter_map(|entry| (entry.region == "booster").then_some(entry.state.as_str()))
        .collect();
    assert!(
        booster_states.contains(&"mission.regions.booster.post_sep_reorient"),
        "booster region should record post-separation reorientation: {booster_states:?}"
    );
    assert!(
        booster_states.contains(&"mission.regions.booster.boostback_burn"),
        "booster region should record boostback burn: {booster_states:?}"
    );
    assert!(
        booster_states.contains(&"mission.regions.booster.coast_entry"),
        "booster region should record boostback cut/coast-entry: {booster_states:?}"
    );
    assert!(
        booster_states.contains(&"mission.regions.booster.entry_attitude"),
        "booster region should record body-lane entry interface: {booster_states:?}"
    );
    assert!(
        booster_states.contains(&"mission.regions.booster.landing_burn"),
        "booster region should record terminal recovery window: {booster_states:?}"
    );
    assert!(
        booster_states.contains(&"mission.regions.booster.touchdown_or_safed"),
        "booster region should record lower-lane ground crossing: {booster_states:?}"
    );

    let upper_states: Vec<&str> = report
        .evidence
        .region_trace
        .iter()
        .filter_map(|entry| (entry.region == "upper_stage").then_some(entry.state.as_str()))
        .collect();
    assert!(
        upper_states.contains(&"mission.regions.upper_stage.coast"),
        "upper-stage region should record independent coast state: {upper_states:?}"
    );

    let booster_signal =
        read_signal(&report, "mission.region.booster", 8).expect("read booster region signal");
    assert!(booster_signal.samples > 0);
    assert!(
        booster_signal
            .preview
            .iter()
            .any(|sample| sample.value == "mission.regions.booster.attached")
    );

    let frames = capture_bus_frames(&report);
    assert!(
        frames.iter().any(|frame| {
            frame.stream == "mission.region"
                && frame.subject == "booster"
                && frame.value == "mission.regions.booster.boostback_burn"
        }),
        "bus frames should include booster boostback region transition: {frames:?}"
    );
    assert!(
        frames.iter().any(|frame| {
            frame.stream == "mission.region"
                && frame.subject == "booster"
                && frame.value == "mission.regions.booster.touchdown_or_safed"
        }),
        "bus frames should include booster ground-crossing region transition: {frames:?}"
    );
    assert!(
        frames.iter().any(|frame| {
            frame.stream == "actuator.command"
                && frame.subject == "engine.eng_boost.thrust_n"
                && frame.value == "active"
        }),
        "bus frames should include boostback engine activation: {frames:?}"
    );
    assert!(
        report
            .evidence
            .requirement_verdicts
            .iter()
            .any(|verdict| verdict.id == "SIL-BUS-CAPTURE" && verdict.verdict == "pass")
    );
}
