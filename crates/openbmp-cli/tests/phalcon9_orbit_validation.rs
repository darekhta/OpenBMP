//! Physics-benchmark validation for the `phalcon9-orbit` scenario.
//!
//! This is the doctrine-compatible answer to "is the result credible?":
//! the synthetic Phalcon-9 is NOT benchmarked against any real fielded
//! vehicle or trajectory (that would require fielded parameters the
//! project forbids). Instead its insertion is validated against
//! first-principles ORBITAL MECHANICS invariants that hold for any
//! body in a central + axisymmetric gravity field:
//!
//! 1. **Vis-viva / bound insertion.** From the final ECI state the
//!    specific orbital energy `ε = v²/2 − μ/r` is negative (bound), and
//!    the derived orbit is near-circular and sustainable (low
//!    eccentricity, perigee well above the atmosphere). This validates
//!    that the closed-loop ascent + PEG cutoff actually achieved a LEO
//!    insertion, not merely "went up".
//!
//! 2. **Axial angular momentum is conserved on the coast.** EGM2008
//!    gravity is axisymmetric about the ECI z-axis, so once the engines
//!    are off (post-SECO) the z-component of specific angular momentum
//!    `(r × v)_z` is an EXACT invariant. Asserting it holds to tight
//!    tolerance over the orbital coast validates that the gravity model
//!    and the integrator conserve it — no spurious torque / energy leak.
//!
//! 3. **Specific energy is conserved on the coast** (to a looser J2-level
//!    tolerance, since the two-body `μ/r` energy oscillates slightly
//!    under the oblateness perturbation).
//!
//! All of these are open, analytic, physics-based references — no
//! real-vehicle data. The vehicle remains a synthetic class anchor.

#![allow(clippy::expect_used, clippy::panic, clippy::float_cmp)]

use std::fs;
use std::path::{Path, PathBuf};

use openbmp_cli::commands::run;
// Reference the canonical WGS84 GM (source-of-truth in openbmp-physics)
// rather than inlining the digits — keeps the value single-sourced and
// satisfies the inline-data tripwire.
use openbmp_physics::frames::WGS84_MU_M3_S2;

/// WGS84/EGM2008 gravitational parameter (m³/s²).
const MU_EARTH: f64 = WGS84_MU_M3_S2;
/// Mean spherical Earth radius (m) — matches the scenario launch radius
/// convention used elsewhere in the tree.
const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

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

#[derive(Clone, Copy)]
struct Sample {
    t: f64,
    r: [f64; 3],
    v: [f64; 3],
}

fn parse_samples(csv_path: &Path) -> Vec<Sample> {
    let text = fs::read_to_string(csv_path).expect("read phalcon9-orbit CSV");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("csv header").split(',').collect();
    let col = |name: &str| {
        header
            .iter()
            .position(|h| *h == name)
            .unwrap_or_else(|| panic!("column {name}"))
    };
    let (ti, pxi, pyi, pzi, vxi, vyi, vzi) = (
        col("time_s"),
        col("position_x_m"),
        col("position_y_m"),
        col("position_z_m"),
        col("velocity_x_m_s"),
        col("velocity_y_m_s"),
        col("velocity_z_m_s"),
    );
    let mut out = Vec::new();
    for line in lines {
        if line.trim().is_empty() {
            continue;
        }
        let f: Vec<f64> = line
            .split(',')
            .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
            .collect();
        out.push(Sample {
            t: f[ti],
            r: [f[pxi], f[pyi], f[pzi]],
            v: [f[vxi], f[vyi], f[vzi]],
        });
    }
    out
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

fn cross_z(r: [f64; 3], v: [f64; 3]) -> f64 {
    r[0] * v[1] - r[1] * v[0]
}

fn ang_mom(r: [f64; 3], v: [f64; 3]) -> f64 {
    let hx = r[1] * v[2] - r[2] * v[1];
    let hy = r[2] * v[0] - r[0] * v[2];
    let hz = r[0] * v[1] - r[1] * v[0];
    (hx * hx + hy * hy + hz * hz).sqrt()
}

/// Two-body specific orbital energy (J/kg).
fn specific_energy(s: Sample) -> f64 {
    let speed_sq = s.v[0] * s.v[0] + s.v[1] * s.v[1] + s.v[2] * s.v[2];
    0.5 * speed_sq - MU_EARTH / norm(s.r)
}

#[test]
fn phalcon9_orbit_meets_orbital_mechanics_invariants() {
    let report = run::run(&scenario_path()).expect("phalcon9-orbit must run to completion");
    let csv = report
        .written
        .iter()
        .find(|p| p.extension().and_then(|e| e.to_str()) == Some("csv"))
        .expect("CSV output path in report");
    let samples = parse_samples(csv);
    assert!(
        samples.len() > 100,
        "expected a full trajectory, got {} rows",
        samples.len()
    );

    // --- (1) Vis-viva: bound, near-circular, sustainable insertion. ---
    let last = *samples.last().expect("final sample");
    let v_f = norm(last.v);
    let eps = specific_energy(last);
    assert!(
        eps < 0.0,
        "final specific energy {eps:.3e} J/kg must be negative (bound orbit)"
    );
    let a = -MU_EARTH / (2.0 * eps); // semi-major axis
    let h = ang_mom(last.r, last.v);
    let e = (1.0 + 2.0 * eps * h * h / (MU_EARTH * MU_EARTH))
        .max(0.0)
        .sqrt();
    let perigee_alt = a * (1.0 - e) - EARTH_MEAN_RADIUS_M;
    let apogee_alt = a * (1.0 + e) - EARTH_MEAN_RADIUS_M;
    assert!(
        e < 0.02,
        "insertion eccentricity {e:.4} must be near-circular (<0.02); perigee {:.0} km apogee {:.0} km",
        perigee_alt / 1000.0,
        apogee_alt / 1000.0
    );
    assert!(
        (150_000.0..500_000.0).contains(&perigee_alt),
        "perigee altitude {:.1} km must be a bound low-LEO parking orbit (150-500 km); \
         the synthetic vehicle inserts to a ~170-210 km parking orbit after a realistic \
         ~2.3 km/s ground-relative staging",
        perigee_alt / 1000.0
    );
    assert!(
        (7_000.0..8_500.0).contains(&v_f),
        "final orbital speed {v_f:.0} m/s must be LEO-class (7-8.5 km/s)"
    );

    // Inclination: this is an equatorial due-east launch, so the orbital
    // plane should be near-equatorial. The guidance resolves its roll
    // reference against the orbital-plane normal, which keeps the commanded
    // attitude continuous through the horizontal pitch-over and so holds the
    // residual inclination low (the fixed-axis reference used to flip there,
    // injecting ~2.4 deg). h_z/|h| = cos(inclination).
    let h_z = last.r[0] * last.v[1] - last.r[1] * last.v[0];
    let inclination_deg = (h_z / h).clamp(-1.0, 1.0).acos().to_degrees();
    assert!(
        inclination_deg < 1.0,
        "equatorial insertion inclination {inclination_deg:.3} deg must stay below 1 deg \
         (roll-reference continuity holds the plane)"
    );

    // --- (2)/(3) Conservation on the post-SECO orbital coast. ---
    // Engines are off well before t = 820 s (SECO ~760 s); sample the
    // coast to the end of the run.
    let coast: Vec<Sample> = samples.iter().copied().filter(|s| s.t >= 820.0).collect();
    assert!(
        coast.len() > 50,
        "expected a coast window, got {} samples",
        coast.len()
    );

    let hz: Vec<f64> = coast.iter().map(|s| cross_z(s.r, s.v)).collect();
    let hz_min = hz.iter().cloned().fold(f64::INFINITY, f64::min);
    let hz_max = hz.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let hz_mean = hz.iter().sum::<f64>() / hz.len() as f64;
    let hz_drift = (hz_max - hz_min) / hz_mean.abs();
    assert!(
        hz_drift < 1.0e-3,
        "axial angular momentum (r×v)_z must be conserved on the coast \
         (axisymmetric gravity); relative drift {hz_drift:.2e} exceeds 1e-3"
    );

    let energy: Vec<f64> = coast.iter().map(|s| specific_energy(*s)).collect();
    let e_min = energy.iter().cloned().fold(f64::INFINITY, f64::min);
    let e_max = energy.iter().cloned().fold(f64::NEG_INFINITY, f64::max);
    let e_mean = energy.iter().sum::<f64>() / energy.len() as f64;
    let e_drift = (e_max - e_min) / e_mean.abs();
    assert!(
        e_drift < 2.0e-2,
        "two-body specific energy must be ~conserved on the coast (J2-level \
         oscillation only); relative drift {e_drift:.2e} exceeds 2e-2"
    );

    println!(
        "phalcon9-orbit validated: perigee {:.0} km, apogee {:.0} km, e {:.4}, \
         |v|_f {:.0} m/s; coast (r×v)_z drift {:.2e}, energy drift {:.2e}",
        perigee_alt / 1000.0,
        apogee_alt / 1000.0,
        e,
        v_f,
        hz_drift,
        e_drift
    );
}
