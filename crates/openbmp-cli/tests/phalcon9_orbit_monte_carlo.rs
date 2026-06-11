//! Monte-Carlo robustness envelope for the `phalcon9-orbit` ascent.
//!
//! Addresses the credibility gap "only one nominal trajectory". The
//! closed-loop GNC (guidance setpoints, gains, PEG cutoff) is held FIXED
//! while the PLANT and the navigation noise are dispersed sample-to-sample
//! — the textbook Monte-Carlo philosophy: a fixed controller flown against
//! a randomised vehicle/environment. Each sample re-runs the full 6-DOF
//! ascent and we assert the controller still delivers a bound, sustainable
//! LEO insertion across the ensemble.
//!
//! Dispersions (all WHOLLY SYNTHETIC, modest 1-sigma class figures — this
//! is a robustness demonstration of the synthetic vehicle, NOT a tuned
//! reproduction of any real flight-dispersion deck):
//!   * navigation/process seed — full reseed per sample, so every sensor
//!     and process-noise realisation differs;
//!   * stage performance — common-mode Isp and thrust scale per stage;
//!   * structural dry mass — per body;
//!   * propellant load — per tank (underfill only, stays physical);
//!   * initial state — small ECI position/velocity offsets;
//!   * winds — a per-sample horizontal wind (the dominant lower-atmosphere
//!     ascent dispersion), perturbing the air-relative velocity / drag.
//!
//! Guidance is NOT dispersed: the MECO/PEG/SECO setpoints are the same in
//! every sample, so a successful ensemble shows the controller — not a
//! hand-tuned trajectory — is what reaches orbit.
//!
//! No real-vehicle data is used. The orbital-mechanics references are the
//! same first-principles invariants asserted by the companion test
//! `phalcon9_orbit_validation.rs`.

#![allow(
    clippy::cast_possible_truncation,
    clippy::cast_precision_loss,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::many_single_char_names,
    clippy::panic
)]

use std::path::{Path, PathBuf};

use openbmp_mc::{SampleRng, ScalarOutcome, run_scalar_campaign};
use openbmp_physics::frames::WGS84_MU_M3_S2;
use openbmp_runner as runner;
use openbmp_scenario::Scenario;
use openbmp_scenario::document::WindConfig;

/// WGS84/EGM2008 gravitational parameter (m^3/s^2) — canonical
/// source-of-truth in openbmp-physics (not inlined; satisfies the
/// inline-data tripwire).
const MU_EARTH: f64 = WGS84_MU_M3_S2;
/// Mean spherical Earth radius (m).
const EARTH_MEAN_RADIUS_M: f64 = 6_371_000.0;

/// Number of dispersed samples. Sample 0 is the undispersed nominal.
const SAMPLES: u64 = 16;
/// Campaign seed for the synthetic Phalcon-9 robustness Monte Carlo.
const CAMPAIGN_SEED: u64 = 0x0B19_C900_0000_0000;

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

/// Insertion metrics derived from a single dispersed run's final state.
#[derive(Clone, Copy, Debug)]
struct Insertion {
    specific_energy: f64,
    eccentricity: f64,
    perigee_km: f64,
    apogee_km: f64,
    speed_m_s: f64,
    inclination_deg: f64,
}

impl Insertion {
    fn bound(&self) -> bool {
        self.specific_energy < 0.0
    }
    /// A sustainable LEO insertion: bound, near-circular, perigee well
    /// clear of the sensible atmosphere.
    fn sustainable(&self) -> bool {
        self.bound() && self.eccentricity < 0.05 && self.perigee_km > 150.0
    }
}

fn norm(a: [f64; 3]) -> f64 {
    (a[0] * a[0] + a[1] * a[1] + a[2] * a[2]).sqrt()
}

/// Read the final ECI state from a run's telemetry table via an in-memory
/// CSV round-trip (name-based column lookup — robust to channel ordering).
fn final_state(outcome: &runner::RunOutcome) -> ([f64; 3], [f64; 3]) {
    let mut buf: Vec<u8> = Vec::new();
    outcome
        .table
        .write_csv(&mut buf)
        .expect("serialise telemetry to CSV");
    let text = String::from_utf8(buf).expect("utf8 telemetry");
    let mut lines = text.lines();
    let header: Vec<&str> = lines.next().expect("csv header").split(',').collect();
    let col = |name: &str| {
        header
            .iter()
            .position(|h| *h == name)
            .unwrap_or_else(|| panic!("column {name}"))
    };
    let (pxi, pyi, pzi, vxi, vyi, vzi) = (
        col("position_x_m"),
        col("position_y_m"),
        col("position_z_m"),
        col("velocity_x_m_s"),
        col("velocity_y_m_s"),
        col("velocity_z_m_s"),
    );
    let last = lines
        .rfind(|l| !l.trim().is_empty())
        .expect("at least one telemetry row");
    let f: Vec<f64> = last
        .split(',')
        .map(|s| s.trim().parse::<f64>().unwrap_or(f64::NAN))
        .collect();
    ([f[pxi], f[pyi], f[pzi]], [f[vxi], f[vyi], f[vzi]])
}

fn insertion_from_state(r: [f64; 3], v: [f64; 3]) -> Insertion {
    let r_mag = norm(r);
    let speed = norm(v);
    let eps = 0.5 * speed * speed - MU_EARTH / r_mag;
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
    let inclination_deg = (h[2] / h_mag).clamp(-1.0, 1.0).acos().to_degrees();
    Insertion {
        specific_energy: eps,
        eccentricity: e,
        perigee_km: (a * (1.0 - e) - EARTH_MEAN_RADIUS_M) / 1000.0,
        apogee_km: (a * (1.0 + e) - EARTH_MEAN_RADIUS_M) / 1000.0,
        speed_m_s: speed,
        inclination_deg,
    }
}

/// A constant-NED-wind [`WindConfig`] (only the `kind` + `wind_ned_m_s`
/// fields apply; everything else is unset).
fn constant_wind(wind_ned_m_s: [f64; 3]) -> WindConfig {
    WindConfig {
        kind: "constant".to_owned(),
        wind_ned_m_s: Some(wind_ned_m_s),
        layers: None,
        intensity_m_s: None,
        length_scale_m: None,
        airspeed_m_s: None,
        mean_wind_ned_m_s: None,
        year: None,
        day_of_year: None,
        utc_s: None,
        latitude_deg: None,
        longitude_deg: None,
        ap_current_3h: None,
    }
}

/// Run sample `idx`. Sample 0 is the undispersed nominal; samples >= 1 are
/// dispersed deterministically from `idx`.
fn run_sample(idx: u64, rng: &mut SampleRng) -> Insertion {
    let mut scenario = Scenario::from_file(scenario_path()).expect("load phalcon9-orbit");
    let doc = &mut scenario.document;

    if idx > 0 {
        // (1) Full reseed → fresh sensor + process-noise realisation.
        doc.time.seed ^= rng.next_u64() | 1;

        // (2) Stage common-mode performance (Isp ~0.4%, thrust ~1.2% 1σ).
        let s1_isp = 1.0 + 0.004 * rng.standard_normal();
        let s1_thrust = 1.0 + 0.012 * rng.standard_normal();
        let s2_isp = 1.0 + 0.004 * rng.standard_normal();
        let s2_thrust = 1.0 + 0.012 * rng.standard_normal();
        for eng in &mut doc.vehicle.assembly.engines {
            let (kisp, kthr) = if eng.id.starts_with("eng_s1_") {
                (s1_isp, s1_thrust)
            } else {
                (s2_isp, s2_thrust)
            };
            eng.limits.isp_s *= kisp;
            eng.limits.max_thrust_n *= kthr;
        }

        // (3) Structural dry mass (~1.5% 1σ per body).
        for body in &mut doc.vehicle.assembly.bodies {
            body.dry_mass_kg *= 1.0 + 0.015 * rng.standard_normal();
        }

        // (4) Propellant load — underfill only so fill stays in [0, 1].
        for tank in &mut doc.vehicle.assembly.tanks {
            let underfill = 0.004 * rng.standard_normal().abs();
            tank.initial_fill_fraction = (tank.initial_fill_fraction - underfill).clamp(0.0, 1.0);
        }

        // (5) Initial-state offsets (position ~30 m, velocity ~0.5 m/s 1σ).
        for k in 0..3 {
            doc.vehicle.initial_position_eci_m[k] += 30.0 * rng.standard_normal();
            doc.vehicle.initial_velocity_eci_m_s[k] += 0.5 * rng.standard_normal();
        }

        // (6) WINDS — a per-sample horizontal wind (~12 m/s 1σ each axis, no
        //     vertical; jet-stream-class tails ~25-35 m/s), the dominant
        //     lower-atmosphere ascent dispersion.
        //     A constant NED wind avoids the flat-earth altitude proxy that
        //     the layered/HWM14 models use (wrong for this geocentric
        //     equatorial launch); drag only acts in the lower atmosphere
        //     anyway, where a constant wind is a fair approximation. The
        //     scenario carries [frames.local_origin], required to rotate the
        //     NED wind into ECI for the air-relative aerodynamics.
        let wind_ned = [
            12.0 * rng.standard_normal(),
            12.0 * rng.standard_normal(),
            0.0,
        ];
        "constant".clone_into(&mut doc.environment.wind);
        doc.wind = Some(constant_wind(wind_ned));
    }

    let outcome = runner::run(&scenario).expect("dispersed phalcon9-orbit must run to completion");
    let (r, v) = final_state(&outcome);
    insertion_from_state(r, v)
}

fn run_indexed_sample(idx: u64) -> Insertion {
    let mut rng = SampleRng::new(CAMPAIGN_SEED, idx, 0);
    run_sample(idx, &mut rng)
}

#[test]
fn phalcon9_orbit_monte_carlo_robustness() {
    let mut results = Vec::with_capacity(SAMPLES as usize);
    let report = run_scalar_campaign(CAMPAIGN_SEED, SAMPLES, 0, |sample_index, rng| {
        let insertion = run_sample(sample_index, rng);
        let outcome = ScalarOutcome::new(insertion.perigee_km, insertion.sustainable())?;
        results.push(insertion);
        Ok(outcome)
    })
    .expect("run MC campaign");

    let bound = results.iter().filter(|r| r.bound()).count();
    let sustainable = report.success_stats.successes();

    let perigees: Vec<f64> = results.iter().map(|r| r.perigee_km).collect();
    let peri_min = perigees.iter().copied().fold(f64::INFINITY, f64::min);
    let peri_max = perigees.iter().copied().fold(f64::NEG_INFINITY, f64::max);
    let peri_mean = report.value_stats.mean();
    let sustainable_ci = report
        .success_interval(0.95)
        .expect("valid sustainable Clopper-Pearson interval");
    let ecc_max = results.iter().map(|r| r.eccentricity).fold(0.0, f64::max);
    let incl_min = results
        .iter()
        .map(|r| r.inclination_deg)
        .fold(f64::INFINITY, f64::min);
    let incl_max = results
        .iter()
        .map(|r| r.inclination_deg)
        .fold(f64::NEG_INFINITY, f64::max);

    println!(
        "phalcon9-orbit Monte-Carlo: {SAMPLES} samples\n  \
         bound: {bound}/{SAMPLES}, sustainable LEO: {sustainable}/{SAMPLES}\n  \
         sustainable 95% Clopper-Pearson [{:.3}, {:.3}]\n  \
         perigee km [min {peri_min:.1}, mean {peri_mean:.1}, max {peri_max:.1}]\n  \
         max eccentricity {ecc_max:.4}, inclination deg [{incl_min:.2}, {incl_max:.2}]",
        sustainable_ci.lower, sustainable_ci.upper,
    );
    for (i, r) in results.iter().enumerate() {
        println!(
            "  sample {i:2}: perigee {:7.1} km, apogee {:7.1} km, e {:.4}, |v| {:.0} m/s, i {:.2} deg",
            r.perigee_km, r.apogee_km, r.eccentricity, r.speed_m_s, r.inclination_deg
        );
    }

    // The controller must deliver a bound orbit in EVERY dispersed case —
    // it never fails to reach orbit under these dispersions.
    assert_eq!(
        bound, SAMPLES as usize,
        "every dispersed sample must reach a bound orbit (specific energy < 0)"
    );
    // The great majority reach a sustainable (near-circular, perigee well
    // clear of the sensible atmosphere) LEO. We do NOT claim 100%: the low
    // tail of the dispersion can graze a short-lived perigee, and the test
    // reports that envelope honestly rather than tuning it away.
    let sustainable_fraction = sustainable as f64 / SAMPLES as f64;
    assert!(
        sustainable_fraction >= 0.90,
        "at least 90% of samples must reach a sustainable LEO (e<0.05, perigee>150 km); \
         got {sustainable}/{SAMPLES}"
    );
    // Even the worst-case sample stays a genuine orbit (not a sub-orbital
    // lob): every perigee clears a 120 km floor.
    assert!(
        peri_min > 120.0,
        "worst-case perigee {peri_min:.1} km must remain a genuine orbit (>120 km)"
    );
    // Envelope sanity: the whole perigee cloud stays within LEO.
    assert!(
        (120.0..700.0).contains(&peri_min) && (120.0..700.0).contains(&peri_max),
        "perigee envelope [{peri_min:.1}, {peri_max:.1}] km must stay within LEO"
    );
    // Inclination is tightly held near-equatorial across the whole ensemble:
    // the guidance resolves its roll reference against the orbital-plane
    // normal, so the residual inclination stays sub-degree even under the
    // dispersions (it was ~1.9-3.6 deg before the roll-reference fix).
    assert!(
        incl_max < 1.5,
        "every dispersed sample must stay near-equatorial; max inclination {incl_max:.2} deg",
    );
    let _ = incl_min;
}

#[test]
fn phalcon9_orbit_monte_carlo_is_deterministic() {
    // The whole ensemble is a pure function of the sample index, so a
    // given sample reproduces bit-for-bit. Guards against any hidden
    // nondeterminism leaking into the dispersed runs.
    let a = run_indexed_sample(3);
    let b = run_indexed_sample(3);
    assert_eq!(
        a.perigee_km.to_bits(),
        b.perigee_km.to_bits(),
        "perigee must be reproducible"
    );
    assert_eq!(
        a.eccentricity.to_bits(),
        b.eccentricity.to_bits(),
        "eccentricity must be reproducible"
    );
    assert_eq!(
        a.speed_m_s.to_bits(),
        b.speed_m_s.to_bits(),
        "speed must be reproducible"
    );
}
