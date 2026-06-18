//! Native tests for the web host: embedded-asset fidelity, snapshot
//! layout integrity, stimulus-timeline determinism, and fail-closed
//! command validation. Everything here runs on the host target — the
//! WebAssembly shell adds only marshalling on top of the same code.

#![allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]

use std::path::Path;

use openbmp_scenario::Scenario;
use openbmp_web::assets;
use openbmp_web::session::WebSession;

/// Every embedded scenario must resolve to exactly the same digests and
/// bytes as the canonical repository files resolved from disk — the
/// browser flies the audited scenario, not a copy that can drift.
#[test]
fn embedded_assets_match_filesystem_resolution() {
    for embedded in assets::SCENARIOS {
        let (_, embedded_files) = embedded.load().expect("embedded load");

        let scenario_path = Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../scenarios/phalcon9")
            .join(format!("{}.toml", embedded.key));
        let toml = std::fs::read_to_string(&scenario_path).expect("read scenario from disk");
        assert_eq!(
            toml, embedded.toml,
            "embedded scenario TOML for `{}` must match the repository file",
            embedded.key
        );
        let scenario = Scenario::from_toml_str_with_source_dir(
            &toml,
            scenario_path.parent().map(Path::to_path_buf),
        )
        .expect("scenario parses");
        let disk_files = scenario.resolved_files().expect("disk resolution");

        let embedded_keys: Vec<&String> = embedded_files.keys().collect();
        let disk_keys: Vec<&String> = disk_files.keys().collect();
        assert_eq!(
            embedded_keys, disk_keys,
            "embedded resolution for `{}` must cover the same file set",
            embedded.key
        );
        for (key, disk) in &disk_files {
            let resolved = &embedded_files[key];
            assert_eq!(
                resolved.sha256_hex, disk.sha256_hex,
                "embedded `{key}` digest must match disk for `{}`",
                embedded.key
            );
            assert_eq!(
                resolved.bytes, disk.bytes,
                "embedded `{key}` bytes must match disk for `{}`",
                embedded.key
            );
        }
    }
}

/// The scenario list is non-empty, JSON, and constructible.
#[test]
fn scenario_list_parses_and_constructs() {
    let list: serde_json::Value =
        serde_json::from_str(&WebSession::scenario_list_json()).expect("list is JSON");
    let entries = list.as_array().expect("list is an array");
    assert!(!entries.is_empty());
    for entry in entries {
        let key = entry["key"].as_str().expect("key");
        let session = WebSession::new(key, None, None, None).expect("construct session");
        assert!(!session.is_finished());
    }
}

/// Snapshot length matches the published stride; header fields carry
/// sane values after stepping through ignition.
#[test]
fn snapshot_matches_layout() {
    let mut session = WebSession::new("phalcon9-orbit", None, None, None).expect("construct");
    let layout: serde_json::Value =
        serde_json::from_str(&session.layout_json()).expect("layout is JSON");
    let stride = usize::try_from(layout["stride"].as_u64().expect("stride")).expect("usize");
    let engine_count = layout["engines"].as_array().expect("engines").len();
    assert_eq!(engine_count, 10, "phalcon9 declares 9 + 1 engines");
    assert_eq!(layout["validation"], "validated-toy");
    assert!(
        layout["tanks"][0]["initial_propellant_kg"]
            .as_f64()
            .expect("tank mass")
            > 100_000.0
    );

    // Step through ignition (0.05 s) plus a little settling.
    let finished = session.step_many(100).expect("steps");
    assert!(!finished);
    let snapshot = session.snapshot();
    assert_eq!(snapshot.len(), stride);

    let field = |name: &str| -> usize {
        usize::try_from(layout["fields"][name].as_u64().expect("field index")).expect("usize")
    };
    assert!(
        (snapshot[field("time_s")] - 2.0).abs() < 1e-9,
        "100 steps at 20 ms"
    );
    assert!(snapshot[field("mass_kg")] > 100_000.0, "stack mass is sane");
    assert!(
        snapshot[field("atmosphere_density_kg_m3")] > 1.0,
        "sea-level density at the pad"
    );
    assert_eq!(
        snapshot[field("estimate_valid")].to_bits(),
        1.0_f64.to_bits(),
        "EKF published"
    );

    // All nine first-stage engines burning: lifecycle index 2, thrust > 0.
    let engines_base = field("engines_base");
    let engine_width =
        usize::try_from(layout["engine_slot_width"].as_u64().expect("width")).expect("usize");
    let thrust_norm = |slot: usize| -> f64 {
        let base = engines_base + slot * engine_width;
        (snapshot[base].powi(2) + snapshot[base + 1].powi(2) + snapshot[base + 2].powi(2)).sqrt()
    };
    for slot in 0..9 {
        assert!(thrust_norm(slot) > 0.0, "stage-1 engine {slot} burning");
    }
    assert!(!session.mission_phase().is_empty());
}

/// A recorded stimulus timeline replays to the identical state, and an
/// undisturbed pair of sessions stays identical (sanity for the
/// determinism story the share-links rely on).
#[test]
fn stimulus_timeline_replays_identically() {
    let mut live = WebSession::new("phalcon9-orbit", None, None, None).expect("construct");
    live.step_many(100).expect("steps");
    live.inject_engine_fault("eng_s1_4", "hard_off", 0.0)
        .expect("inject");
    live.set_wind([25.0, 5.0, 0.0], true).expect("wind");
    live.step_many(150).expect("steps");
    live.inject_sensor_outage("gnss", 1.5).expect("outage");
    live.step_many(250).expect("steps");
    let timeline = live.timeline_json();
    let live_snapshot = live.snapshot();

    let mut replayed =
        WebSession::new("phalcon9-orbit", None, Some(&timeline), None).expect("construct replay");
    replayed.step_many(500).expect("steps");
    let replayed_snapshot = replayed.snapshot();

    assert_eq!(replayed.step_index(), live.step_index());
    let bits = |values: &[f64]| -> Vec<u64> { values.iter().map(|v| v.to_bits()).collect() };
    assert_eq!(
        bits(&live_snapshot),
        bits(&replayed_snapshot),
        "replayed timeline must reproduce the live run bit-for-bit"
    );
}

/// Stimulus commands validate fail-closed.
#[test]
fn stimulus_commands_fail_closed() {
    let mut session = WebSession::new("phalcon9-orbit", None, None, None).expect("construct");
    assert!(session.inject_engine_fault("eng_s1_0", "rud", 0.0).is_err());
    assert!(
        session
            .inject_engine_fault("eng_s1_0", "thrust_loss", 1.5)
            .is_err()
    );
    assert!(
        session
            .inject_engine_fault("eng_nope", "hard_off", 0.0)
            .is_err()
    );
    assert!(session.inject_sensor_outage("gnss", -1.0).is_err());
    assert!(session.inject_sensor_outage("lidar", 1.0).is_err());
    assert!(session.set_wind([f64::NAN, 0.0, 0.0], true).is_err());
    assert!(WebSession::new("not-a-scenario", None, None, None).is_err());

    let timeline = r#"{"version":1,"scenario":"phalcon9-orbit","seed":1,"commands":[]}"#;
    assert!(WebSession::new("phalcon9-orbit-boostback", None, Some(timeline), None).is_err());
}

/// Each selectable flight-software config must FLY the mission, not merely
/// construct: both onboard navigation filters (EKF and SR-UKF) have to keep
/// the estimate-vs-truth nav error bounded the whole way to orbit. A bare
/// construct+step check is not enough — the SR-UKF once built and stepped
/// fine for 20 s, then its position/velocity covariance collapsed and it
/// dead-reckoned the vehicle into the ground. This catches that natively,
/// far faster than the headless browser run.
#[test]
fn fc_modes_fly_to_orbit_with_bounded_nav_error() {
    const R_EARTH_M: f64 = 6_371_000.0;
    for mode in ["ekf", "sr_ukf"] {
        let mut session = WebSession::new("phalcon9-orbit-boostback", None, None, Some(mode))
            .unwrap_or_else(|err| panic!("fc mode `{mode}` failed to construct: {err}"));
        let layout: serde_json::Value =
            serde_json::from_str(&session.layout_json()).expect("layout is JSON");
        let field = |name: &str| -> usize {
            usize::try_from(layout["fields"][name].as_u64().expect("field index")).expect("usize")
        };
        let (i_t, i_pos, i_vel, i_est) = (
            field("time_s"),
            field("position_eci_m"),
            field("velocity_eci_m_s"),
            field("estimate_position_eci_m"),
        );

        let mut max_alt_m = 0.0_f64;
        let mut max_speed_m_s = 0.0_f64;
        let mut max_nav_err_m = 0.0_f64;
        let mut last_t = 0.0_f64;
        // ~600 s of flight at 20 ms ticks, 1 s per outer step: enough to
        // clear apogee (>150 km) AND build to orbital velocity (~T+500),
        // running far past the SR-UKF divergence window (which opened by
        // T+10 s), without paying for the full boostback descent (the
        // browser e2e covers that).
        for _ in 0..600 {
            let finished = session
                .step_many(50)
                .unwrap_or_else(|err| panic!("fc mode `{mode}` failed to step: {err}"));
            let s = session.snapshot();
            let alt =
                (s[i_pos].powi(2) + s[i_pos + 1].powi(2) + s[i_pos + 2].powi(2)).sqrt() - R_EARTH_M;
            max_alt_m = max_alt_m.max(alt);
            let speed = (s[i_vel].powi(2) + s[i_vel + 1].powi(2) + s[i_vel + 2].powi(2)).sqrt();
            max_speed_m_s = max_speed_m_s.max(speed);
            let nav_err = ((s[i_pos] - s[i_est]).powi(2)
                + (s[i_pos + 1] - s[i_est + 1]).powi(2)
                + (s[i_pos + 2] - s[i_est + 2]).powi(2))
            .sqrt();
            // Skip the first 5 s of EKF/UKF convergence transient.
            if s[i_t] > 5.0 {
                max_nav_err_m = max_nav_err_m.max(nav_err);
            }
            last_t = s[i_t];
            if finished {
                break;
            }
        }

        assert!(
            max_alt_m > 150_000.0,
            "fc mode `{mode}`: reached only {:.0} km — expected orbital altitude (>150 km)",
            max_alt_m / 1000.0
        );
        // Orbital velocity, not just altitude: a vehicle whose attitude
        // estimate has diverged coasts THROUGH 150 km on a suborbital arc
        // and falls back (this is exactly how the SR-UKF failed before the
        // attitude fix — nav error stayed bounded but the vehicle never
        // gained orbital energy). Require it to actually reach orbital speed.
        assert!(
            max_speed_m_s > 6_500.0,
            "fc mode `{mode}`: reached only {:.2} km/s — expected orbital velocity (>6.5 km/s); \
             the vehicle flew a suborbital arc (attitude estimate likely diverged)",
            max_speed_m_s / 1000.0
        );
        assert!(
            max_nav_err_m < 5_000.0,
            "fc mode `{mode}`: nav error diverged to {max_nav_err_m:.0} m (expected < 5 km) — \
             the onboard estimate lost the vehicle",
        );
        assert!(
            last_t > 200.0,
            "fc mode `{mode}`: mission only advanced to T+{last_t:.0} s",
        );
    }
    assert!(WebSession::new("phalcon9-orbit", None, None, Some("nonsense")).is_err());
}
