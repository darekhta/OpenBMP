//! End-to-end check for the `phalcon9-orbit-iers` scenario: the same
//! synthetic launch vehicle flown on the higher-fidelity IERS-tabulated
//! Earth frame (IAU precession + nutation + tabulated polar motion /
//! UT1-UTC / LOD) instead of uniform rotation.
//!
//! Two things are asserted:
//!   1. The launch still delivers a bound, near-circular, near-equatorial
//!      LEO insertion through the FC bridge on the IERS frame — the
//!      high-fidelity frame does not break the closed-loop ascent.
//!   2. The IERS frame is genuinely ENGAGED, not a silent no-op: the
//!      trajectory measurably differs from the uniform-rotation
//!      `phalcon9-orbit` run (the precession/nutation/polar-motion/LOD
//!      terms perturb the air-relative velocity), yet the final orbit and
//!      its inclination match the uniform-rotation case closely.
//!
//! No real-vehicle and no real Earth-orientation data: the EOP table is
//! synthetic/illustrative (see scenarios/phalcon9/eop-synthetic.toml).

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

#[derive(Clone, Copy)]
struct Sample {
    r: [f64; 3],
    v: [f64; 3],
}

fn run_and_parse(scenario_rel: &str) -> Vec<Sample> {
    let path = workspace_root().join(scenario_rel);
    let report = run::run(&path).unwrap_or_else(|e| panic!("{scenario_rel} must run: {e:?}"));
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output path");
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
    lines
        .filter(|l| !l.trim().is_empty())
        .map(|l| {
            let f: Vec<f64> = l
                .split(',')
                .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
                .collect();
            Sample {
                r: [f[px], f[py], f[pz]],
                v: [f[vx], f[vy], f[vz]],
            }
        })
        .collect()
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

struct Orbit {
    specific_energy: f64,
    eccentricity: f64,
    perigee_km: f64,
    inclination_deg: f64,
}

fn orbit_of(s: Sample) -> Orbit {
    let r = norm(s.r);
    let speed_sq = s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2];
    let eps = 0.5 * speed_sq - MU_EARTH / r;
    let a = -MU_EARTH / (2.0 * eps);
    let h = [
        s.r[1] * s.v[2] - s.r[2] * s.v[1],
        s.r[2] * s.v[0] - s.r[0] * s.v[2],
        s.r[0] * s.v[1] - s.r[1] * s.v[0],
    ];
    let h_mag = norm(h);
    let e = (1.0 + 2.0 * eps * h_mag * h_mag / (MU_EARTH * MU_EARTH))
        .max(0.0)
        .sqrt();
    Orbit {
        specific_energy: eps,
        eccentricity: e,
        perigee_km: (a * (1.0 - e) - EARTH_MEAN_RADIUS_M) / 1000.0,
        inclination_deg: (h[2] / h_mag).clamp(-1.0, 1.0).acos().to_degrees(),
    }
}

#[test]
fn phalcon9_orbit_iers_inserts_and_frame_is_engaged() {
    let uniform = run_and_parse("scenarios/phalcon9/phalcon9-orbit.toml");
    let iers = run_and_parse("scenarios/phalcon9/phalcon9-orbit-iers.toml");
    assert!(iers.len() > 100 && uniform.len() == iers.len());

    // (1) The IERS-frame launch reaches a bound, near-circular,
    //     near-equatorial LEO.
    let o = orbit_of(*iers.last().expect("final IERS sample"));
    assert!(
        o.specific_energy < 0.0,
        "IERS insertion must be bound, eps={:.3e}",
        o.specific_energy
    );
    assert!(
        o.eccentricity < 0.02,
        "IERS insertion eccentricity {:.4} must be near-circular",
        o.eccentricity
    );
    assert!(
        (150.0..600.0).contains(&o.perigee_km),
        "IERS perigee {:.1} km must be a sustainable LEO",
        o.perigee_km
    );
    assert!(
        o.inclination_deg < 1.0,
        "IERS inclination {:.3} deg must stay near-equatorial",
        o.inclination_deg
    );

    // (2) The IERS frame is genuinely engaged: the trajectory diverges from
    //     the uniform-rotation run by a non-trivial amount (precession /
    //     nutation / polar motion / LOD acting through the air-relative
    //     velocity), proving it is not a silent fall-through to uniform.
    let max_dpos = uniform
        .iter()
        .zip(iers.iter())
        .map(|(a, b)| norm([a.r[0] - b.r[0], a.r[1] - b.r[1], a.r[2] - b.r[2]]))
        .fold(0.0_f64, f64::max);
    assert!(
        max_dpos > 0.1,
        "IERS frame must measurably change the trajectory vs uniform rotation; max |Δpos| {max_dpos:.4} m"
    );

    // ...yet the high-fidelity frame does not change the achieved orbit
    //    meaningfully: same insertion to within tight tolerances.
    let u = orbit_of(*uniform.last().expect("final uniform sample"));
    assert!(
        (o.inclination_deg - u.inclination_deg).abs() < 0.05,
        "IERS inclination {:.3} vs uniform {:.3} deg should match closely",
        o.inclination_deg,
        u.inclination_deg
    );
    assert!(
        (o.perigee_km - u.perigee_km).abs() < 10.0,
        "IERS perigee {:.1} vs uniform {:.1} km should match closely",
        o.perigee_km,
        u.perigee_km
    );

    println!(
        "phalcon9-orbit-iers: perigee {:.1} km, e {:.4}, inclination {:.3} deg; \
         max |Δpos| vs uniform = {max_dpos:.3} m",
        o.perigee_km, o.eccentricity, o.inclination_deg
    );
}
