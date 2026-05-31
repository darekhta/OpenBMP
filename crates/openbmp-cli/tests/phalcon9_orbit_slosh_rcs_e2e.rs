//! End-to-end check for `phalcon9-orbit-slosh-rcs`: closed-loop orbital
//! insertion with upper-stage propellant SLOSH damped by an RCS
//! (reaction-control) coast attitude hold.
//!
//! With an EquivalentPendulum slosh model on the stage-2 tank, the
//! engines-off coast has no gimbal authority and no aerodynamic moment in
//! vacuum. The upper stage's three body-axis `direct_torque` RCS effectors
//! provide the active coast attitude-hold path; with the corrected
//! specific-force/capillary slosh model the residual rates are small, but
//! the RCS still commands measurable damping and the circularisation burn
//! stays well pointed.
//!
//! This test asserts, against first-principles orbital-mechanics invariants
//! only (vis-viva; no fielded-vehicle data):
//!   1. the run integrates to completion (slosh + RCS + engines + tanks all
//!      coexisting in the kernel is a stable, finite simulation);
//!   2. the RCS is actively engaged in coast and the slosh-driven body rate
//!      stays small;
//!   3. at PEG burn cutoff the stage is inserted to a BOUND, near-circular
//!      LEO (negative specific energy, small eccentricity, LEO perigee);
//!   4. that orbit is STABLE — the end-of-run elements still match the
//!      insertion to within a tight tolerance (no secular drift). This is the
//!      payoff of the freefall slosh well-posedness fix: the model is fed the
//!      SPECIFIC force (not the gravity-laden kinematic accel) and given a
//!      capillary restoring + critical-damping floor in freefall, so the
//!      coast slosh decays to rest and its reaction force stops perturbing the
//!      orbit. (An earlier revision left the freefall slosh undamped and the
//!      orbit drifted to reentry over minutes; that is now fixed, so insertion
//!      is a genuine stable orbit rather than a burn-cutoff snapshot.)

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
        .join("scenarios/phalcon9/phalcon9-orbit-slosh-rcs.toml")
}

struct Row {
    t: f64,
    r: [f64; 3],
    v: [f64; 3],
    omega_mag: f64,
    rcs_actual_mag: f64,
    thrust_mag: f64,
    apogee: bool,
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
    let (wx, wy, wz) = (
        col("angular_velocity.x_rad_s"),
        col("angular_velocity.y_rad_s"),
        col("angular_velocity.z_rad_s"),
    );
    let (fx, fy, fz) = (
        col("force.thrust.x_n"),
        col("force.thrust.y_n"),
        col("force.thrust.z_n"),
    );
    let (rr, rp, ry) = (
        col("effector.rcs_roll.actual"),
        col("effector.rcs_pitch.actual"),
        col("effector.rcs_yaw.actual"),
    );
    let apo = col("mission.marker.apogee");
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<&str> = l.split(',').collect();
            let num = |i: usize| f[i].trim().parse::<f64>().unwrap_or(f64::NAN);
            Row {
                t: num(ti),
                r: [num(px), num(py), num(pz)],
                v: [num(vx), num(vy), num(vz)],
                omega_mag: (num(wx).powi(2) + num(wy).powi(2) + num(wz).powi(2)).sqrt(),
                rcs_actual_mag: (num(rr).powi(2) + num(rp).powi(2) + num(ry).powi(2)).sqrt(),
                thrust_mag: (num(fx).powi(2) + num(fy).powi(2) + num(fz).powi(2)).sqrt(),
                apogee: f[apo].trim() == "true",
            }
        })
        .collect()
}

struct Orbit {
    eps: f64,
    e: f64,
    perigee_km: f64,
    apogee_km: f64,
}

fn orbit(row: &Row) -> Orbit {
    let r = (row.r[0].powi(2) + row.r[1].powi(2) + row.r[2].powi(2)).sqrt();
    let speed_sq = row.v.iter().map(|c| c * c).sum::<f64>();
    let eps = 0.5 * speed_sq - MU_EARTH / r;
    let a = -MU_EARTH / (2.0 * eps);
    let h = [
        row.r[1] * row.v[2] - row.r[2] * row.v[1],
        row.r[2] * row.v[0] - row.r[0] * row.v[2],
        row.r[0] * row.v[1] - row.r[1] * row.v[0],
    ];
    let h_mag = (h[0] * h[0] + h[1] * h[1] + h[2] * h[2]).sqrt();
    let e = (1.0 + 2.0 * eps * h_mag * h_mag / (MU_EARTH * MU_EARTH))
        .max(0.0)
        .sqrt();
    Orbit {
        eps,
        e,
        perigee_km: (a * (1.0 - e) - EARTH_MEAN_RADIUS_M) / 1000.0,
        apogee_km: (a * (1.0 + e) - EARTH_MEAN_RADIUS_M) / 1000.0,
    }
}

fn at(rows: &[Row], t: f64) -> &Row {
    rows.iter()
        .min_by(|a, b| (a.t - t).abs().partial_cmp(&(b.t - t).abs()).unwrap())
        .expect("row")
}

#[test]
fn phalcon9_slosh_rcs_damps_coast_rates_and_inserts_to_leo() {
    let report = run::run(&scenario_path()).expect("phalcon9-orbit-slosh-rcs must run");
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output");
    let rows = parse(csv);
    assert!(rows.len() > 1000, "expected a full-length run");

    // (1) The run integrates to completion with finite state — slosh +
    //     RCS + engine cluster + tank rack all coexist in the kernel.
    let last = rows.last().expect("final row");
    assert!(last.t > 950.0, "run must reach the configured stop time");
    assert!(
        last.r.iter().chain(last.v.iter()).all(|x| x.is_finite()),
        "final state must be finite (no slosh-driven blow-up)"
    );

    // Apogee marks the coast → circularise boundary.
    let apogee_t = rows
        .iter()
        .find(|r| r.apogee)
        .map(|r| r.t)
        .expect("apogee event must fire");

    // Closed-loop circularisation starts from the controller's estimated
    // apogee transition. The truth apogee marker can occur a few seconds
    // later because the burn immediately pushes radial velocity positive, so
    // use thrust onset as the physical end of the unpowered coast window.
    let burn_start_t = rows
        .iter()
        .filter(|r| r.t > 245.0)
        .find(|r| r.thrust_mag > 1000.0)
        .map(|r| r.t)
        .expect("upper-stage circularisation burn must start");
    assert!(
        burn_start_t > 700.0 && burn_start_t < 760.0,
        "circularisation burn must start near apogee; t = {burn_start_t:.1} s"
    );

    // (2) RCS is actively engaged and the slosh-driven coast rate stays
    //     small. The window is the unpowered coast: post-separation up to the
    //     first upper-stage thrust.
    let peak_coast_omega = rows
        .iter()
        .filter(|r| r.t > 245.0 && r.t < burn_start_t)
        .map(|r| r.omega_mag)
        .fold(0.0_f64, f64::max);
    let peak_coast_rcs = rows
        .iter()
        .filter(|r| r.t > 245.0 && r.t < burn_start_t)
        .map(|r| r.rcs_actual_mag)
        .fold(0.0_f64, f64::max);
    assert!(
        peak_coast_rcs > 0.01,
        "RCS must command measurable coast damping; peak command magnitude = {peak_coast_rcs:.4}"
    );
    assert!(
        peak_coast_omega < 0.02,
        "RCS coast hold must keep the slosh-driven body rate small; peak |omega| = {peak_coast_omega:.3} rad/s"
    );

    // (3) At PEG burn cutoff the stage is inserted to a bound near-circular
    //     LEO. Burn cutoff = first time after upper-stage thrust onset that
    //     thrust returns to ~0; assess the orbit a short settle after.
    let mut burning = false;
    let cutoff_t = rows
        .iter()
        .filter(|r| r.t >= burn_start_t)
        .find_map(|r| {
            if r.thrust_mag > 1000.0 {
                burning = true;
                None
            } else if burning {
                Some(r.t)
            } else {
                None
            }
        })
        .expect("circularise burn must fire and cut off");

    let insertion = orbit(at(&rows, cutoff_t + 30.0));
    assert!(
        insertion.eps < 0.0,
        "insertion must be a BOUND orbit; eps = {:.0} J/kg",
        insertion.eps
    );
    assert!(
        insertion.e < 0.02,
        "insertion must be near-circular; e = {:.4}",
        insertion.e
    );
    assert!(
        (250.0..600.0).contains(&insertion.perigee_km),
        "insertion perigee {:.1} km must be a LEO",
        insertion.perigee_km
    );

    // (4) The inserted orbit is STABLE: the end-of-run elements still match
    //     the insertion (no secular slosh drift). The freefall slosh decays
    //     to rest, so its reaction force stops perturbing the orbit.
    let final_orbit = orbit(last);
    assert!(
        final_orbit.eps < 0.0 && final_orbit.e < 0.02,
        "inserted orbit must stay bound and near-circular to end of run (no slosh drift); \
         end e = {:.4}, eps = {:.0}",
        final_orbit.e,
        final_orbit.eps
    );
    assert!(
        (final_orbit.perigee_km - insertion.perigee_km).abs() < 25.0
            && (final_orbit.apogee_km - insertion.apogee_km).abs() < 25.0,
        "orbit must not drift between insertion and end of run; \
         perigee {:.1} -> {:.1} km, apogee {:.1} -> {:.1} km",
        insertion.perigee_km,
        final_orbit.perigee_km,
        insertion.apogee_km,
        final_orbit.apogee_km
    );

    println!(
        "phalcon9-orbit-slosh-rcs: coast ends at t={burn_start_t:.1}s; peak coast |omega| {peak_coast_omega:.3} rad/s, peak RCS command {peak_coast_rcs:.3}; \
         truth apogee marker t={apogee_t:.1}s; insertion at cutoff+30s (t={:.0}s): e {:.4}, perigee {:.1} km, apogee {:.1} km; \
         end-of-run (t={:.0}s): e {:.4}, perigee {:.1} km, apogee {:.1} km (stable)",
        cutoff_t + 30.0,
        insertion.e,
        insertion.perigee_km,
        insertion.apogee_km,
        last.t,
        final_orbit.e,
        final_orbit.perigee_km,
        final_orbit.apogee_km
    );
}
