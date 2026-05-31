//! End-to-end check for `phalcon9-orbit-flex`: the launch vehicle carries a
//! first lateral structural BENDING MODE that the rate gyro senses, and the
//! closed-loop GNC inserts to orbit robustly through that flex coupling.
//!
//! Asserts:
//!   1. the flex vehicle still reaches a bound, near-circular, near-equatorial
//!      LEO (the autopilot is robust to a first bending mode at its tuned
//!      gains);
//!   2. the bending mode genuinely COUPLES — the flex insertion differs
//!      measurably from the rigid `phalcon9-orbit`. The controller rejects
//!      most of the perturbation, so this is a nonzero-coupling/no-op guard,
//!      not a demand for a large final-orbit error.
//!
//! Together these show the structural-flex subsystem is wired end-to-end
//! (model → rack → gyro sensor truth) and is not cosmetic. The gyro-notch
//! flex gain-stabilisation tool is available (`[fc.autopilot_params].gyro_notch`)
//! but is not needed at this vehicle's gains, so none is applied here.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
use openbmp_physics::frames::WGS84_MU_M3_S2;

const MU_EARTH: f64 = WGS84_MU_M3_S2;
const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

fn workspace_root() -> PathBuf {
    PathBuf::from(env!("CARGO_MANIFEST_DIR"))
        .parent()
        .and_then(Path::parent)
        .map(Path::to_path_buf)
        .expect("workspace root")
}

struct Orbit {
    bound: bool,
    eccentricity: f64,
    perigee_km: f64,
    inclination_deg: f64,
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn final_orbit(scenario_rel: &str) -> Orbit {
    let path = workspace_root().join(scenario_rel);
    let report = run::run(&path).unwrap_or_else(|e| panic!("{scenario_rel} must run: {e:?}"));
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output");
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
    let last = lines
        .filter(|l| !l.trim().is_empty())
        .next_back()
        .expect("final row");
    let f: Vec<f64> = last
        .split(',')
        .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
        .collect();
    let r = [f[px], f[py], f[pz]];
    let v = [f[vx], f[vy], f[vz]];
    let r_mag = norm(r);
    let speed_sq = v.iter().map(|c| c * c).sum::<f64>();
    let eps = 0.5 * speed_sq - MU_EARTH / r_mag;
    let a = -MU_EARTH / (2.0 * eps);
    let h = [
        r[1] * v[2] - r[2] * v[1],
        r[2] * v[0] - r[0] * v[2],
        r[0] * v[1] - r[1] * v[0],
    ];
    let h_mag = norm(h);
    let e = (1.0 + 2.0 * eps * h_mag * h_mag / (MU_EARTH * MU_EARTH))
        .max(0.0)
        .sqrt();
    Orbit {
        bound: eps < 0.0,
        eccentricity: e,
        perigee_km: (a * (1.0 - e) - EARTH_MEAN_RADIUS_M) / 1000.0,
        inclination_deg: (h[2] / h_mag).clamp(-1.0, 1.0).acos().to_degrees(),
    }
}

#[test]
fn phalcon9_orbit_flex_inserts_and_bending_couples() {
    let flex = final_orbit("scenarios/phalcon9/phalcon9-orbit-flex.toml");
    let rigid = final_orbit("scenarios/phalcon9/phalcon9-orbit.toml");

    // (1) The flex vehicle still reaches a bound, near-circular,
    //     near-equatorial LEO.
    assert!(flex.bound, "flex insertion must be bound");
    assert!(
        flex.eccentricity < 0.05,
        "flex insertion must be near-circular, e={:.4}",
        flex.eccentricity
    );
    assert!(
        (150.0..500.0).contains(&flex.perigee_km),
        "flex perigee {:.1} km must be a sustainable LEO",
        flex.perigee_km
    );
    assert!(
        flex.inclination_deg < 2.0,
        "flex inclination {:.3} deg must stay near-equatorial",
        flex.inclination_deg
    );

    // (2) The bending mode genuinely couples — the flex insertion differs
    //     measurably from the rigid run. A no-op flex model would be
    //     identical, but the tuned controller should reject almost all of the
    //     disturbance, so the threshold is deliberately small.
    let perigee_diff = (flex.perigee_km - rigid.perigee_km).abs();
    let ecc_diff = (flex.eccentricity - rigid.eccentricity).abs();
    assert!(
        perigee_diff > 0.05 || ecc_diff > 1.0e-5,
        "bending mode must measurably couple into the trajectory; \
         flex perigee {:.3} km vs rigid {:.3} km (Δ {perigee_diff:.3}), e {:.6} vs {:.6}",
        flex.perigee_km,
        rigid.perigee_km,
        flex.eccentricity,
        rigid.eccentricity
    );

    println!(
        "phalcon9-orbit-flex: perigee {:.1} km, e {:.4}, incl {:.3} deg; \
         coupling vs rigid: Δperigee {perigee_diff:.1} km, Δe {ecc_diff:.4}",
        flex.perigee_km, flex.eccentricity, flex.inclination_deg
    );
}
