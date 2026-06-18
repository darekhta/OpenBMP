//! SIL stimulation coverage for in-memory AFTS rule injection.

#![allow(clippy::expect_used)]

use std::fs;
use std::path::Path;

use openbmp_scenario::AftsZoneColorConfig;
use openbmp_sil::{AftsZoneStimulus, AftsZoneStimulusVertex, MissionPackage, SilStimulation};

const POINT_MASS_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "sil-afts-stimulus-test"
description = "Synthetic point-mass SIL AFTS stimulation regression."
validation = "validated-toy"

[time]
start_s = 0.0
stop_s = 0.1
dt_s = 0.1
seed = 21

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6379137.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "sil-afts-stimulus-test"

[[vehicle.assembly.bodies]]
id = "body"
geometry = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0
dry_cg_body_m = [0.0, 0.0, 0.0]

[environment]
frame_profile = "toy-fixed-earth"
gravity = "point_mass"
mu_m3_s2 = 3.986004418e14
atmosphere = "none"
wind = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/sil-afts-stimulus-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

fn write_package(dir: &Path) -> (MissionPackage, std::path::PathBuf) {
    let scenario_path = dir.join("scenario.toml");
    fs::write(&scenario_path, POINT_MASS_SCENARIO).expect("write scenario");
    let manifest_path = dir.join("mission-package.toml");
    fs::write(
        &manifest_path,
        "[package]\nid = \"test.afts-stimulus\"\nversion = \"0.0.1\"\n\n[files]\nscenario = \"scenario.toml\"\n",
    )
    .expect("write manifest");
    (
        MissionPackage::load(&manifest_path).expect("load package"),
        scenario_path,
    )
}

#[test]
fn afts_zone_stimulation_latches_afts_evidence() {
    let dir = tempfile::tempdir().expect("tempdir");
    let (package, scenario_path) = write_package(dir.path());

    let nominal = package.run_case(None).expect("nominal run");
    assert!(
        nominal.evidence.afts.is_none(),
        "base package should not declare AFTS evidence"
    );

    let stimulus = SilStimulation {
        afts_zones: vec![AftsZoneStimulus {
            id: "sil-red-zone".to_owned(),
            evidence: "tests/fixtures/afts/sil-red-zone.md#synthetic".to_owned(),
            color: AftsZoneColorConfig::Red,
            vertices: vec![
                AftsZoneStimulusVertex {
                    latitude_deg: -1.0,
                    longitude_deg: -1.0,
                },
                AftsZoneStimulusVertex {
                    latitude_deg: -1.0,
                    longitude_deg: 1.0,
                },
                AftsZoneStimulusVertex {
                    latitude_deg: 1.0,
                    longitude_deg: 1.0,
                },
                AftsZoneStimulusVertex {
                    latitude_deg: 1.0,
                    longitude_deg: -1.0,
                },
            ],
        }],
        ..SilStimulation::default()
    };
    let faulted = package
        .run_case_with_stimulation(None, &stimulus)
        .expect("AFTS-stimulated run");

    let afts = faulted
        .evidence
        .afts
        .as_ref()
        .expect("stimulated run should attach AFTS evidence");
    assert!(afts.terminate);
    assert_eq!(afts.rule_id.as_deref(), Some("sil-red-zone"));
    assert_eq!(afts.rule_table.len(), 1);
    assert_eq!(afts.rule_table[0].kind, "zone");
    assert_eq!(afts.rule_table[0].zone_color.as_deref(), Some("red"));
    assert!(
        faulted
            .evidence
            .stimuli
            .iter()
            .any(|record| record.kind == "afts_zone_stimulus"
                && record.target == "afts.zone.sil-red-zone"),
        "AFTS zone stimulus must be recorded as SIL evidence"
    );
    assert!(
        !fs::read_to_string(scenario_path)
            .expect("read original scenario")
            .contains("[afts]"),
        "SIL stimulation must not mutate the package scenario file"
    );
}
