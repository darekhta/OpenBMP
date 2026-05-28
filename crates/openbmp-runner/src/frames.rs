//! Runner-side frame-context construction.

use std::collections::BTreeMap;

use openbmp_physics::{
    ARCSECOND_TO_RAD, EarthOrientationSample, EarthOrientationTable, FrameContext,
    LocalGeodeticOrigin,
};
use openbmp_scenario::{ResolvedFile, ScenarioDocument};

use crate::error::RunnerError;

/// Build the frame context declared by a scenario.
///
/// # Errors
///
/// Returns [`RunnerError`] when the selected frame profile is not
/// wired by the runner, an IERS table is missing or malformed, or the
/// pinned table does not cover the configured simulation interval.
pub(crate) fn build_frame_context(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<FrameContext, RunnerError> {
    let local_origin = local_origin(document)?;
    match document.environment.frame_profile.as_str() {
        "toy-fixed-earth" => Ok(FrameContext::toy_fixed_earth()),
        "wgs84-uniform-rotation" => Ok(FrameContext::wgs84_uniform_rotation(local_origin)),
        "iers-tabulated" => build_iers_tabulated_frame(document, resolved_files, local_origin),
        "spice-reference" => Err(RunnerError::UnsupportedScenario {
            what: "frame_profile = \"spice-reference\" is reserved for validation and is not wired"
                .to_owned(),
        }),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("frame_profile = \"{other}\" is not wired"),
        }),
    }
}

fn local_origin(document: &ScenarioDocument) -> Result<Option<LocalGeodeticOrigin>, RunnerError> {
    let Some(local_origin) = document
        .frames
        .as_ref()
        .and_then(|frames| frames.local_origin.as_ref())
    else {
        return Ok(None);
    };
    let origin = LocalGeodeticOrigin::new_degrees(
        local_origin.latitude_deg,
        local_origin.longitude_deg,
        local_origin.height_m,
    )
    .map_err(|err| RunnerError::Env(err.into()))?;
    Ok(Some(origin))
}

fn build_iers_tabulated_frame(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    local_origin: Option<LocalGeodeticOrigin>,
) -> Result<FrameContext, RunnerError> {
    let epoch = document
        .epoch
        .as_ref()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "frame_profile = \"iers-tabulated\" requires [epoch]".to_owned(),
        })?;
    if epoch.scale.to_ascii_uppercase() != "UTC" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "frame_profile = \"iers-tabulated\" requires epoch.scale = \"UTC\"; got \"{}\"",
                epoch.scale
            ),
        });
    }
    let epoch_utc_julian_date = crate::celestial::parse_iso8601_julian_date(&epoch.iso8601)?;
    let resolved =
        resolved_files
            .get("epoch.eop")
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "frame_profile = \"iers-tabulated\" requires resolved epoch.eop".to_owned(),
            })?;
    let table = parse_earth_orientation_table(resolved)?;
    if !table.covers_interval(document.time.start_s, document.time.stop_s) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop table does not cover simulation interval [{}, {}] s",
                document.time.start_s, document.time.stop_s
            ),
        });
    }
    FrameContext::iers_tabulated(epoch_utc_julian_date, local_origin, table)
        .map_err(|err| RunnerError::Env(err.into()))
}

fn parse_earth_orientation_table(
    resolved: &ResolvedFile,
) -> Result<EarthOrientationTable, RunnerError> {
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.eop is not valid UTF-8: {err}"),
        })?;
    let value: toml::Value =
        toml::from_str(text).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.eop TOML parse failed: {err}"),
        })?;
    if let Some(format) = value.get("format") {
        let format = format
            .as_str()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "epoch.eop format must be a string".to_owned(),
            })?;
        if format != "openbmp-eop-v1" {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("epoch.eop format \"{format}\" is not supported"),
            });
        }
    }
    let samples = value
        .get("samples")
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "epoch.eop requires [[samples]] entries".to_owned(),
        })?;
    let mut parsed = Vec::with_capacity(samples.len());
    for (index, sample) in samples.iter().enumerate() {
        parsed.push(parse_eop_sample(index, sample)?);
    }
    EarthOrientationTable::new(parsed).map_err(|err| RunnerError::Env(err.into()))
}

fn parse_eop_sample(
    index: usize,
    sample: &toml::Value,
) -> Result<EarthOrientationSample, RunnerError> {
    let table = sample
        .as_table()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.eop samples[{index}] must be a table"),
        })?;
    let time_s = number_at(table, "time_s", index)?;
    let ut1_minus_utc_s = number_at(table, "ut1_minus_utc_s", index)?;
    let x_pole_arcsec = number_at(table, "x_pole_arcsec", index)?;
    let y_pole_arcsec = number_at(table, "y_pole_arcsec", index)?;
    EarthOrientationSample::new(
        time_s,
        ut1_minus_utc_s,
        x_pole_arcsec * ARCSECOND_TO_RAD,
        y_pole_arcsec * ARCSECOND_TO_RAD,
    )
    .map_err(|err| RunnerError::Env(err.into()))
}

#[allow(clippy::cast_precision_loss)]
fn number_at(
    table: &toml::map::Map<String, toml::Value>,
    key: &str,
    index: usize,
) -> Result<f64, RunnerError> {
    let value = table
        .get(key)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.eop samples[{index}].{key} is required"),
        })?;
    match value {
        toml::Value::Float(v) => Ok(*v),
        toml::Value::Integer(v) => Ok(*v as f64),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.eop samples[{index}].{key} must be numeric"),
        }),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use openbmp_scenario::Scenario;
    use std::path::PathBuf;

    const IERS_SCENARIO: &str = r#"
openbmp.scenario = 2

[meta]
name = "iers-frame-test"
description = "Frame context construction fixture."
validation = "validated-toy"
provenance = "synthetic frame-context test"

[time]
start_s = 0.0
stop_s = 10.0
dt_s = 1.0
seed = 1

[epoch]
scale = "UTC"
iso8601 = "2000-01-01T12:00:00Z"
eop = "eop.toml"

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "frame-test"

[[vehicle.assembly.bodies]]
id          = "main"
geometry    = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0

[environment]
frame_profile = "iers-tabulated"
gravity       = "constant"
gravity_m_s2  = 9.80665
atmosphere    = "none"
wind          = "none"

[frames]
profile = "iers-tabulated"

[telemetry]
output.csv = "out/frame.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const EOP: &str = r#"
format = "openbmp-eop-v1"

[[samples]]
time_s = 0.0
ut1_minus_utc_s = 0.0
x_pole_arcsec = 0.1
y_pole_arcsec = -0.2

[[samples]]
time_s = 10.0
ut1_minus_utc_s = 1.0
x_pole_arcsec = 0.2
y_pole_arcsec = -0.1
"#;

    fn resolved_eop(bytes: &[u8]) -> BTreeMap<String, ResolvedFile> {
        let mut files = BTreeMap::new();
        files.insert(
            "epoch.eop".to_owned(),
            ResolvedFile {
                path: PathBuf::from("eop.toml"),
                sha256_hex: "not-used-in-unit-test".to_owned(),
                bytes: bytes.to_vec(),
            },
        );
        files
    }

    #[test]
    fn parses_openbmp_eop_table() {
        let file = ResolvedFile {
            path: PathBuf::from("eop.toml"),
            sha256_hex: "not-used-in-unit-test".to_owned(),
            bytes: EOP.as_bytes().to_vec(),
        };
        let table = parse_earth_orientation_table(&file).unwrap();
        assert_eq!(table.samples().len(), 2);
        assert!(table.covers_interval(0.0, 10.0));
    }

    #[test]
    fn builds_iers_tabulated_frame_context() {
        let scenario = Scenario::from_toml_str(IERS_SCENARIO).unwrap();
        let frame = build_frame_context(&scenario.document, &resolved_eop(EOP.as_bytes())).unwrap();
        assert_eq!(frame.profile().as_label(), "iers-tabulated");
        assert_eq!(frame.earth_orientation_table().unwrap().samples().len(), 2);
    }

    #[test]
    fn rejects_iers_table_that_does_not_cover_run() {
        let scenario = Scenario::from_toml_str(IERS_SCENARIO).unwrap();
        let short_eop = EOP.replace("time_s = 10.0", "time_s = 5.0");
        let err = build_frame_context(&scenario.document, &resolved_eop(short_eop.as_bytes()))
            .unwrap_err();
        assert!(matches!(err, RunnerError::UnsupportedScenario { .. }));
    }
}
