//! Runner-side frame-context construction.

use std::collections::BTreeMap;

use openbmp_physics::{
    ARCSECOND_TO_RAD, CioXysSample, CioXysTable, EarthOrientationSample, EarthOrientationTable,
    FrameContext, LocalGeodeticOrigin,
};
use openbmp_scenario::{ResolvedFile, ScenarioDocument};

use crate::error::RunnerError;

const SECONDS_PER_DAY: f64 = 86_400.0;
const MJD_ZERO_JULIAN_DATE: f64 = 2_400_000.5;
const MILLIARCSECOND_TO_ARCSECOND: f64 = 0.001;
const MILLISECOND_TO_SECOND: f64 = 0.001;

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
        "iers-cio" => build_iers_cio_frame(document, resolved_files, local_origin),
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
    if !epoch.scale.eq_ignore_ascii_case("UTC") {
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
    let table = parse_earth_orientation_table(resolved, epoch_utc_julian_date)?;
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

fn build_iers_cio_frame(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    local_origin: Option<LocalGeodeticOrigin>,
) -> Result<FrameContext, RunnerError> {
    let epoch = document
        .epoch
        .as_ref()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "frame_profile = \"iers-cio\" requires [epoch]".to_owned(),
        })?;
    if !epoch.scale.eq_ignore_ascii_case("UTC") {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "frame_profile = \"iers-cio\" requires epoch.scale = \"UTC\"; got \"{}\"",
                epoch.scale
            ),
        });
    }
    let epoch_utc_julian_date = crate::celestial::parse_iso8601_julian_date(&epoch.iso8601)?;
    let eop_resolved =
        resolved_files
            .get("epoch.eop")
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "frame_profile = \"iers-cio\" requires resolved epoch.eop".to_owned(),
            })?;
    let eop = parse_earth_orientation_table(eop_resolved, epoch_utc_julian_date)?;
    if !eop.covers_interval(document.time.start_s, document.time.stop_s) {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop table does not cover simulation interval [{}, {}] s",
                document.time.start_s, document.time.stop_s
            ),
        });
    }
    let leap_seconds =
        crate::celestial::resolved_leap_second_table(document, resolved_files)?.ok_or_else(
            || RunnerError::UnsupportedScenario {
                what: "frame_profile = \"iers-cio\" requires resolved epoch.leap_second_table or a NAIF LSK in environment.ephemeris_meta_kernel"
                    .to_owned(),
            },
        )?;
    let tai_minus_utc_s = leap_seconds.tai_minus_utc_s(epoch_utc_julian_date)?;
    if let Some(xys_resolved) = resolved_files.get("epoch.cio_xys") {
        let xys = parse_cio_xys_table(xys_resolved)?;
        if !xys.covers_interval(document.time.start_s, document.time.stop_s) {
            return Err(RunnerError::UnsupportedScenario {
                what: format!(
                    "epoch.cio_xys table does not cover simulation interval [{}, {}] s",
                    document.time.start_s, document.time.stop_s
                ),
            });
        }
        FrameContext::iers_cio(
            epoch_utc_julian_date,
            tai_minus_utc_s,
            local_origin,
            eop,
            xys,
        )
        .map_err(|err| RunnerError::Env(err.into()))
    } else {
        FrameContext::iers_cio_iau2006a(epoch_utc_julian_date, tai_minus_utc_s, local_origin, eop)
            .map_err(|err| RunnerError::Env(err.into()))
    }
}

pub(crate) fn parse_earth_orientation_table(
    resolved: &ResolvedFile,
    epoch_utc_julian_date: f64,
) -> Result<EarthOrientationTable, RunnerError> {
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.eop is not valid UTF-8: {err}"),
        })?;
    if text
        .lines()
        .find(|line| !line.trim().is_empty() && !line.trim_start().starts_with('#'))
        .is_some_and(looks_like_finals2000a_line)
    {
        return parse_finals2000a_earth_orientation_table(text, epoch_utc_julian_date);
    }
    if text.lines().any(looks_like_eop14_c04_line) {
        return parse_eop14_c04_earth_orientation_table(text, epoch_utc_julian_date);
    }
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

pub(crate) fn parse_cio_xys_table(resolved: &ResolvedFile) -> Result<CioXysTable, RunnerError> {
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.cio_xys is not valid UTF-8: {err}"),
        })?;
    let value: toml::Value =
        toml::from_str(text).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.cio_xys TOML parse failed: {err}"),
        })?;
    if let Some(format) = value.get("format") {
        let format = format
            .as_str()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "epoch.cio_xys format must be a string".to_owned(),
            })?;
        if format != "openbmp-cio-xys-v1" {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("epoch.cio_xys format \"{format}\" is not supported"),
            });
        }
    }
    let samples = value
        .get("samples")
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "epoch.cio_xys requires [[samples]] entries".to_owned(),
        })?;
    let mut parsed = Vec::with_capacity(samples.len());
    for (index, sample) in samples.iter().enumerate() {
        parsed.push(parse_cio_xys_sample(index, sample)?);
    }
    CioXysTable::new(parsed).map_err(|err| RunnerError::Env(err.into()))
}

fn parse_finals2000a_earth_orientation_table(
    text: &str,
    epoch_utc_julian_date: f64,
) -> Result<EarthOrientationTable, RunnerError> {
    if !epoch_utc_julian_date.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: "epoch.eop finals2000A conversion requires a finite UTC epoch".to_owned(),
        });
    }
    let epoch_mjd_utc = epoch_utc_julian_date - MJD_ZERO_JULIAN_DATE;
    let mut parsed = Vec::new();
    for (line_index, line) in text.lines().enumerate() {
        if line.trim().is_empty() || line.trim_start().starts_with('#') {
            continue;
        }
        parsed.push(parse_finals2000a_sample(
            line_index + 1,
            line,
            epoch_mjd_utc,
        )?);
    }
    EarthOrientationTable::new(parsed).map_err(|err| RunnerError::Env(err.into()))
}

fn parse_eop14_c04_earth_orientation_table(
    text: &str,
    epoch_utc_julian_date: f64,
) -> Result<EarthOrientationTable, RunnerError> {
    if !epoch_utc_julian_date.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: "epoch.eop EOP 14 C04 conversion requires a finite UTC epoch".to_owned(),
        });
    }
    let epoch_mjd_utc = epoch_utc_julian_date - MJD_ZERO_JULIAN_DATE;
    let mut parsed = Vec::new();
    for (line_index, line) in text.lines().enumerate() {
        if !looks_like_eop14_c04_line(line) {
            continue;
        }
        parsed.push(parse_eop14_c04_sample(line_index + 1, line, epoch_mjd_utc)?);
    }
    EarthOrientationTable::new(parsed).map_err(|err| RunnerError::Env(err.into()))
}

fn looks_like_eop14_c04_line(line: &str) -> bool {
    let fields = line.split_whitespace().collect::<Vec<_>>();
    if fields.len() < 14 {
        return false;
    }
    let Ok(year) = fields[0].parse::<i32>() else {
        return false;
    };
    let Ok(month) = fields[1].parse::<u32>() else {
        return false;
    };
    let Ok(day) = fields[2].parse::<u32>() else {
        return false;
    };
    (1900..=2200).contains(&year)
        && (1..=12).contains(&month)
        && (1..=31).contains(&day)
        && fields[3].parse::<f64>().is_ok()
        && fields[4..14]
            .iter()
            .all(|field| field.parse::<f64>().is_ok())
}

fn parse_eop14_c04_sample(
    line_number: usize,
    line: &str,
    epoch_mjd_utc: f64,
) -> Result<EarthOrientationSample, RunnerError> {
    let fields = line.split_whitespace().collect::<Vec<_>>();
    if fields.len() < 14 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop EOP 14 C04 line {line_number} must contain at least 14 fields"
            ),
        });
    }
    let mjd_utc = parse_eop14_c04_field(&fields, line_number, 3, "MJD UTC")?;
    let time_s = (mjd_utc - epoch_mjd_utc) * SECONDS_PER_DAY;
    let x_pole_arcsec = parse_eop14_c04_field(&fields, line_number, 4, "x pole")?;
    let y_pole_arcsec = parse_eop14_c04_field(&fields, line_number, 5, "y pole")?;
    let ut1_minus_utc_s = parse_eop14_c04_field(&fields, line_number, 6, "UT1-UTC")?;
    let lod_s = parse_eop14_c04_field(&fields, line_number, 7, "LOD")?;
    let cip_offset_x_arcsec = parse_eop14_c04_field(&fields, line_number, 8, "dX")?;
    let cip_offset_y_arcsec = parse_eop14_c04_field(&fields, line_number, 9, "dY")?;

    EarthOrientationSample::new_with_lod(
        time_s,
        ut1_minus_utc_s,
        x_pole_arcsec * ARCSECOND_TO_RAD,
        y_pole_arcsec * ARCSECOND_TO_RAD,
        lod_s,
    )
    .map_err(|err| RunnerError::Env(err.into()))?
    .with_cip_offsets(
        cip_offset_x_arcsec * ARCSECOND_TO_RAD,
        cip_offset_y_arcsec * ARCSECOND_TO_RAD,
    )
    .map_err(|err| RunnerError::Env(err.into()))
}

fn parse_eop14_c04_field(
    fields: &[&str],
    line_number: usize,
    index: usize,
    label: &str,
) -> Result<f64, RunnerError> {
    let field = fields
        .get(index)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.eop EOP 14 C04 line {line_number} missing {label}"),
        })?;
    field
        .parse::<f64>()
        .map_err(|_| RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop EOP 14 C04 line {line_number} field {label} value `{field}` is not numeric"
            ),
        })
}

fn looks_like_finals2000a_line(line: &str) -> bool {
    line.len() >= 15
        && line
            .get(7..15)
            .is_some_and(|field| field.trim().parse::<f64>().is_ok())
}

fn parse_finals2000a_sample(
    line_number: usize,
    line: &str,
    epoch_mjd_utc: f64,
) -> Result<EarthOrientationSample, RunnerError> {
    let mjd_utc = parse_finals2000a_required_field(line, line_number, 8, 15, "MJD UTC")?;
    let time_s = (mjd_utc - epoch_mjd_utc) * SECONDS_PER_DAY;
    let x_pole_arcsec =
        parse_finals2000a_optional_field(line, line_number, 135, 144, "Bulletin B PM-x")?
            .unwrap_or(parse_finals2000a_required_field(
                line,
                line_number,
                19,
                27,
                "Bulletin A PM-x",
            )?);
    let y_pole_arcsec =
        parse_finals2000a_optional_field(line, line_number, 145, 154, "Bulletin B PM-y")?
            .unwrap_or(parse_finals2000a_required_field(
                line,
                line_number,
                38,
                46,
                "Bulletin A PM-y",
            )?);
    let ut1_minus_utc_s =
        parse_finals2000a_optional_field(line, line_number, 155, 165, "Bulletin B UT1-UTC")?
            .unwrap_or(parse_finals2000a_required_field(
                line,
                line_number,
                59,
                68,
                "Bulletin A UT1-UTC",
            )?);
    let cip_offset_x_arcsec =
        parse_finals2000a_optional_field(line, line_number, 166, 175, "Bulletin B dX")?.unwrap_or(
            parse_finals2000a_required_field(line, line_number, 98, 106, "Bulletin A dX")?,
        ) * MILLIARCSECOND_TO_ARCSECOND;
    let cip_offset_y_arcsec =
        parse_finals2000a_optional_field(line, line_number, 176, 185, "Bulletin B dY")?.unwrap_or(
            parse_finals2000a_required_field(line, line_number, 117, 125, "Bulletin A dY")?,
        ) * MILLIARCSECOND_TO_ARCSECOND;
    let lod_s = parse_finals2000a_optional_field(line, line_number, 80, 86, "Bulletin A LOD")?
        .map(|lod_ms| lod_ms * MILLISECOND_TO_SECOND);

    let sample = if let Some(lod_s) = lod_s {
        EarthOrientationSample::new_with_lod(
            time_s,
            ut1_minus_utc_s,
            x_pole_arcsec * ARCSECOND_TO_RAD,
            y_pole_arcsec * ARCSECOND_TO_RAD,
            lod_s,
        )
    } else {
        EarthOrientationSample::new(
            time_s,
            ut1_minus_utc_s,
            x_pole_arcsec * ARCSECOND_TO_RAD,
            y_pole_arcsec * ARCSECOND_TO_RAD,
        )
    };
    sample
        .map_err(|err| RunnerError::Env(err.into()))?
        .with_cip_offsets(
            cip_offset_x_arcsec * ARCSECOND_TO_RAD,
            cip_offset_y_arcsec * ARCSECOND_TO_RAD,
        )
        .map_err(|err| RunnerError::Env(err.into()))
}

fn parse_finals2000a_required_field(
    line: &str,
    line_number: usize,
    start_col: usize,
    end_col: usize,
    label: &str,
) -> Result<f64, RunnerError> {
    parse_finals2000a_optional_field(line, line_number, start_col, end_col, label)?.ok_or_else(
        || RunnerError::UnsupportedScenario {
            what: format!("epoch.eop finals2000A line {line_number} missing {label}"),
        },
    )
}

fn parse_finals2000a_optional_field(
    line: &str,
    line_number: usize,
    start_col: usize,
    end_col: usize,
    label: &str,
) -> Result<Option<f64>, RunnerError> {
    let field = line
        .get(start_col - 1..end_col)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop finals2000A line {line_number} too short for {label} columns {start_col}-{end_col}"
            ),
        })?
        .trim();
    if field.is_empty() {
        return Ok(None);
    }
    field
        .parse::<f64>()
        .map(Some)
        .map_err(|_| RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop finals2000A line {line_number} field {label} value `{field}` is not numeric"
            ),
        })
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
    let cip_offsets = optional_cip_offsets_arcsec(table, index)?;
    let lod_s = optional_number_at(table, "lod_s", index)?;
    let sample = if let Some(lod_s) = lod_s {
        EarthOrientationSample::new_with_lod(
            time_s,
            ut1_minus_utc_s,
            x_pole_arcsec * ARCSECOND_TO_RAD,
            y_pole_arcsec * ARCSECOND_TO_RAD,
            lod_s,
        )
    } else {
        EarthOrientationSample::new(
            time_s,
            ut1_minus_utc_s,
            x_pole_arcsec * ARCSECOND_TO_RAD,
            y_pole_arcsec * ARCSECOND_TO_RAD,
        )
    };
    let sample = sample.map_err(|err| RunnerError::Env(err.into()))?;
    if let Some((cip_offset_x_arcsec, cip_offset_y_arcsec)) = cip_offsets {
        sample
            .with_cip_offsets(
                cip_offset_x_arcsec * ARCSECOND_TO_RAD,
                cip_offset_y_arcsec * ARCSECOND_TO_RAD,
            )
            .map_err(|err| RunnerError::Env(err.into()))
    } else {
        Ok(sample)
    }
}

fn parse_cio_xys_sample(index: usize, sample: &toml::Value) -> Result<CioXysSample, RunnerError> {
    let table = sample
        .as_table()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.cio_xys samples[{index}] must be a table"),
        })?;
    CioXysSample::new(
        cio_xys_number_at(table, "time_s", index)?,
        cio_xys_number_at(table, "x_rad", index)?,
        cio_xys_number_at(table, "y_rad", index)?,
        cio_xys_number_at(table, "s_rad", index)?,
    )
    .map_err(|err| RunnerError::Env(err.into()))
}

fn optional_cip_offsets_arcsec(
    table: &toml::map::Map<String, toml::Value>,
    index: usize,
) -> Result<Option<(f64, f64)>, RunnerError> {
    let x = optional_number_at(table, "cip_offset_x_arcsec", index)?;
    let y = optional_number_at(table, "cip_offset_y_arcsec", index)?;
    match (x, y) {
        (None, None) => Ok(None),
        (Some(x), Some(y)) => Ok(Some((x, y))),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.eop samples[{index}] must declare both cip_offset_x_arcsec and cip_offset_y_arcsec"
            ),
        }),
    }
}

#[allow(clippy::cast_precision_loss)]
fn optional_number_at(
    table: &toml::map::Map<String, toml::Value>,
    key: &str,
    index: usize,
) -> Result<Option<f64>, RunnerError> {
    let Some(value) = table.get(key) else {
        return Ok(None);
    };
    match value {
        toml::Value::Float(v) => Ok(Some(*v)),
        toml::Value::Integer(v) => Ok(Some(*v as f64)),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.eop samples[{index}].{key} must be numeric"),
        }),
    }
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

#[allow(clippy::cast_precision_loss)]
fn cio_xys_number_at(
    table: &toml::map::Map<String, toml::Value>,
    key: &str,
    index: usize,
) -> Result<f64, RunnerError> {
    let value = table
        .get(key)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.cio_xys samples[{index}].{key} is required"),
        })?;
    match value {
        toml::Value::Float(v) => Ok(*v),
        toml::Value::Integer(v) => Ok(*v as f64),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.cio_xys samples[{index}].{key} must be numeric"),
        }),
    }
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use openbmp_scenario::Scenario;
    use sha2::{Digest, Sha256};
    use std::path::PathBuf;

    const FINALS2000A_2017_EOP: &[u8] =
        include_bytes!("../../../data/eop/finals2000a-2017-001-004-openbmp-eop-v1.toml");
    const FINALS2000A_2017_EOP_SHA256: &str =
        "b26049e519db72a585118186ca2edb74e6441a86a632a5816ad4b7ab6ce8110f";
    const FINALS2000A_2017_RAW: &[u8] =
        include_bytes!("../../../data/eop/finals2000a-2017-001-004.raw");
    const FINALS2000A_2017_RAW_SHA256: &str =
        "c28e05cb04563bfc3b99adf3ea4a1400e49bbfca2742c3429035ba78c8cab88d";
    const EOP14_C04_2017_RAW: &[u8] =
        include_bytes!("../../../data/eop/eopc04-14-iau2000a-2017-001-004.raw");
    const EOP14_C04_2017_RAW_SHA256: &str =
        "8b51d2fafe4b474fe6af68c6e93698f5ddd70b03a9c619d16aa5237b7bc4b389";
    const FINALS2000A_2017_EPOCH_JD: f64 = 2_457_754.5;

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
cip_offset_x_arcsec = 0.010
cip_offset_y_arcsec = -0.020
lod_s = 0.001

[[samples]]
time_s = 10.0
ut1_minus_utc_s = 1.0
x_pole_arcsec = 0.2
y_pole_arcsec = -0.1
cip_offset_x_arcsec = 0.030
cip_offset_y_arcsec = 0.040
lod_s = 0.003
"#;

    const CIO_XYS: &str = r#"
format = "openbmp-cio-xys-v1"

[[samples]]
time_s = 0.0
x_rad = 0.0010
y_rad = -0.000002
s_rad = 0.000000001

[[samples]]
time_s = 10.0
x_rad = 0.0012
y_rad = -0.000004
s_rad = 0.000000005
"#;

    const LEAP_SECONDS: &str = r#"
format = "openbmp-leap-seconds-v1"

[[entries]]
effective_utc = "1999-01-01T00:00:00Z"
tai_minus_utc_s = 32
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

    fn iers_cio_scenario() -> String {
        IERS_SCENARIO.replace("iers-tabulated", "iers-cio").replace(
            "eop = \"eop.toml\"",
            "eop = \"eop.toml\"\ncio_xys = \"cio-xys.toml\"\nleap_second_table = \"leaps.toml\"",
        )
    }

    fn iers_cio_generated_scenario() -> String {
        IERS_SCENARIO.replace("iers-tabulated", "iers-cio").replace(
            "eop = \"eop.toml\"",
            "eop = \"eop.toml\"\nleap_second_table = \"leaps.toml\"",
        )
    }

    fn resolved_iers_cio(cio_xys: &str) -> BTreeMap<String, ResolvedFile> {
        let mut files = resolved_eop(EOP.as_bytes());
        files.insert(
            "epoch.cio_xys".to_owned(),
            ResolvedFile {
                path: PathBuf::from("cio-xys.toml"),
                sha256_hex: "not-used-in-unit-test".to_owned(),
                bytes: cio_xys.as_bytes().to_vec(),
            },
        );
        files.insert(
            "epoch.leap_second_table".to_owned(),
            ResolvedFile {
                path: PathBuf::from("leaps.toml"),
                sha256_hex: "not-used-in-unit-test".to_owned(),
                bytes: LEAP_SECONDS.as_bytes().to_vec(),
            },
        );
        files
    }

    fn resolved_iers_cio_generated() -> BTreeMap<String, ResolvedFile> {
        let mut files = resolved_eop(EOP.as_bytes());
        files.insert(
            "epoch.leap_second_table".to_owned(),
            ResolvedFile {
                path: PathBuf::from("leaps.toml"),
                sha256_hex: "not-used-in-unit-test".to_owned(),
                bytes: LEAP_SECONDS.as_bytes().to_vec(),
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
        let table = parse_earth_orientation_table(&file, FINALS2000A_2017_EPOCH_JD).unwrap();
        assert_eq!(table.samples().len(), 2);
        assert!(table.covers_interval(0.0, 10.0));
        assert_eq!(table.samples()[0].lod_s, Some(0.001));
        assert_abs_diff_eq!(
            table.samples()[0].cip_offset_x_rad,
            0.010 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-20
        );
        assert_abs_diff_eq!(
            table
                .sample(openbmp_core::SimTime::from_seconds(5.0))
                .cip_offset_y_rad,
            0.010 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-20
        );
    }

    #[test]
    fn parses_openbmp_cio_xys_table() {
        let file = ResolvedFile {
            path: PathBuf::from("cio-xys.toml"),
            sha256_hex: "not-used-in-unit-test".to_owned(),
            bytes: CIO_XYS.as_bytes().to_vec(),
        };
        let table = parse_cio_xys_table(&file).unwrap();
        assert_eq!(table.samples().len(), 2);
        assert!(table.covers_interval(0.0, 10.0));
        let sample = table
            .sample(openbmp_core::SimTime::from_seconds(5.0))
            .unwrap();
        assert_abs_diff_eq!(sample.x_rad, 0.0011, epsilon = 1.0e-18);
        assert_abs_diff_eq!(sample.y_rad, -0.000003, epsilon = 1.0e-21);
        assert_abs_diff_eq!(sample.s_rad, 0.000000003, epsilon = 1.0e-24);
    }

    #[test]
    fn parses_provenance_pinned_finals2000a_eop_subset() {
        let digest = Sha256::digest(FINALS2000A_2017_EOP);
        assert_eq!(format!("{digest:x}"), FINALS2000A_2017_EOP_SHA256);

        let file = ResolvedFile {
            path: PathBuf::from("data/eop/finals2000a-2017-001-004-openbmp-eop-v1.toml"),
            sha256_hex: FINALS2000A_2017_EOP_SHA256.to_owned(),
            bytes: FINALS2000A_2017_EOP.to_vec(),
        };
        let table = parse_earth_orientation_table(&file, FINALS2000A_2017_EPOCH_JD).unwrap();

        assert_eq!(table.samples().len(), 4);
        assert!(table.covers_interval(0.0, 259_200.0));
        let first = table.samples()[0];
        assert_abs_diff_eq!(first.ut1_minus_utc_s, 0.591_297_5);
        assert_abs_diff_eq!(
            first.polar_motion_x_rad,
            0.080_450 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-20
        );
        assert_abs_diff_eq!(
            first.cip_offset_y_rad,
            -0.000_057 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-24
        );
        assert_abs_diff_eq!(first.lod_s.unwrap(), 0.001_034_2);

        let midpoint = table.sample(openbmp_core::SimTime::from_seconds(43_200.0));
        assert_abs_diff_eq!(
            midpoint.polar_motion_y_rad,
            0.263_334_5 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-20
        );
        assert_abs_diff_eq!(
            midpoint.cip_offset_x_rad,
            -0.000_023 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-24
        );
        assert_abs_diff_eq!(midpoint.lod_s.unwrap(), 0.001_103_6);
    }

    #[test]
    fn parses_raw_finals2000a_eop_subset() {
        let digest = Sha256::digest(FINALS2000A_2017_RAW);
        assert_eq!(format!("{digest:x}"), FINALS2000A_2017_RAW_SHA256);

        let raw_file = ResolvedFile {
            path: PathBuf::from("data/eop/finals2000a-2017-001-004.raw"),
            sha256_hex: FINALS2000A_2017_RAW_SHA256.to_owned(),
            bytes: FINALS2000A_2017_RAW.to_vec(),
        };
        let raw_table =
            parse_earth_orientation_table(&raw_file, FINALS2000A_2017_EPOCH_JD).unwrap();
        let toml_file = ResolvedFile {
            path: PathBuf::from("data/eop/finals2000a-2017-001-004-openbmp-eop-v1.toml"),
            sha256_hex: FINALS2000A_2017_EOP_SHA256.to_owned(),
            bytes: FINALS2000A_2017_EOP.to_vec(),
        };
        let toml_table =
            parse_earth_orientation_table(&toml_file, FINALS2000A_2017_EPOCH_JD).unwrap();

        assert_eq!(raw_table.samples().len(), toml_table.samples().len());
        assert!(raw_table.covers_interval(0.0, 259_200.0));
        for (raw, converted) in raw_table
            .samples()
            .iter()
            .copied()
            .zip(toml_table.samples().iter().copied())
        {
            assert_abs_diff_eq!(raw.time_s, converted.time_s, epsilon = 1.0e-9);
            assert_abs_diff_eq!(
                raw.ut1_minus_utc_s,
                converted.ut1_minus_utc_s,
                epsilon = 1.0e-12
            );
            assert_abs_diff_eq!(
                raw.polar_motion_x_rad,
                converted.polar_motion_x_rad,
                epsilon = 1.0e-20
            );
            assert_abs_diff_eq!(
                raw.polar_motion_y_rad,
                converted.polar_motion_y_rad,
                epsilon = 1.0e-20
            );
            assert_abs_diff_eq!(
                raw.cip_offset_x_rad,
                converted.cip_offset_x_rad,
                epsilon = 1.0e-24
            );
            assert_abs_diff_eq!(
                raw.cip_offset_y_rad,
                converted.cip_offset_y_rad,
                epsilon = 1.0e-24
            );
            assert_abs_diff_eq!(
                raw.lod_s.unwrap(),
                converted.lod_s.unwrap(),
                epsilon = 1.0e-12
            );
        }
    }

    #[test]
    fn parses_provenance_pinned_eop14_c04_subset() {
        let digest = Sha256::digest(EOP14_C04_2017_RAW);
        assert_eq!(format!("{digest:x}"), EOP14_C04_2017_RAW_SHA256);

        let raw_file = ResolvedFile {
            path: PathBuf::from("data/eop/eopc04-14-iau2000a-2017-001-004.raw"),
            sha256_hex: EOP14_C04_2017_RAW_SHA256.to_owned(),
            bytes: EOP14_C04_2017_RAW.to_vec(),
        };
        let table = parse_earth_orientation_table(&raw_file, FINALS2000A_2017_EPOCH_JD).unwrap();

        assert_eq!(table.samples().len(), 4);
        assert!(table.covers_interval(0.0, 259_200.0));
        let first = table.samples()[0];
        assert_abs_diff_eq!(first.time_s, 0.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(first.ut1_minus_utc_s, 0.591_297_7, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            first.polar_motion_x_rad,
            0.080_406 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-20
        );
        assert_abs_diff_eq!(
            first.polar_motion_y_rad,
            0.263_110 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-20
        );
        assert_abs_diff_eq!(
            first.cip_offset_x_rad,
            -0.000_041 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-24
        );
        assert_abs_diff_eq!(
            first.cip_offset_y_rad,
            -0.000_127 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-24
        );
        assert_abs_diff_eq!(first.lod_s.unwrap(), 0.001_016_0, epsilon = 1.0e-12);

        let last = table.samples()[3];
        assert_abs_diff_eq!(last.time_s, 259_200.0, epsilon = 1.0e-9);
        assert_abs_diff_eq!(last.ut1_minus_utc_s, 0.587_556_0, epsilon = 1.0e-12);
        assert_abs_diff_eq!(
            last.cip_offset_y_rad,
            -0.000_128 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-24
        );
    }

    #[test]
    fn builds_iers_tabulated_frame_context() {
        let scenario = Scenario::from_toml_str(IERS_SCENARIO).unwrap();
        let frame = build_frame_context(&scenario.document, &resolved_eop(EOP.as_bytes())).unwrap();
        assert_eq!(frame.profile().as_label(), "iers-tabulated");
        assert_eq!(frame.earth_orientation_table().unwrap().samples().len(), 2);
    }

    #[test]
    fn builds_iers_cio_frame_context() {
        let scenario = Scenario::from_toml_str(&iers_cio_scenario()).unwrap();
        let frame = build_frame_context(&scenario.document, &resolved_iers_cio(CIO_XYS)).unwrap();
        assert_eq!(frame.profile().as_label(), "iers-cio");
        assert_eq!(frame.earth_orientation_table().unwrap().samples().len(), 2);
        let model = frame.cio_frame_model().unwrap();
        let sample = model
            .sample(openbmp_core::SimTime::from_seconds(5.0))
            .unwrap();
        assert_abs_diff_eq!(
            sample.x_rad,
            0.0011 + 0.020 * ARCSECOND_TO_RAD,
            epsilon = 1.0e-18
        );
        let matrix = model
            .gcrs_to_itrs_matrix(openbmp_core::SimTime::from_seconds(5.0))
            .unwrap();
        assert!(matrix.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn builds_iers_cio_frame_context_with_generated_xys() {
        let scenario = Scenario::from_toml_str(&iers_cio_generated_scenario()).unwrap();
        let frame =
            build_frame_context(&scenario.document, &resolved_iers_cio_generated()).unwrap();
        assert_eq!(frame.profile().as_label(), "iers-cio");
        assert_eq!(frame.earth_orientation_table().unwrap().samples().len(), 2);
        let model = frame.cio_frame_model().unwrap();
        assert!(model.xys_table().is_none());
        assert!(model.uses_generated_iau2006a_xys());
        let sample = model
            .sample(openbmp_core::SimTime::from_seconds(5.0))
            .unwrap();
        assert!(sample.x_rad.is_finite());
        assert!(sample.y_rad.is_finite());
        assert!(sample.s_rad.is_finite());
        let matrix = model
            .gcrs_to_itrs_matrix(openbmp_core::SimTime::from_seconds(5.0))
            .unwrap();
        assert!(matrix.iter().all(|value| value.is_finite()));
    }

    #[test]
    fn rejects_iers_cio_xys_table_that_does_not_cover_run() {
        let scenario = Scenario::from_toml_str(&iers_cio_scenario()).unwrap();
        let short_xys = CIO_XYS.replace("time_s = 10.0", "time_s = 5.0");
        let err =
            build_frame_context(&scenario.document, &resolved_iers_cio(&short_xys)).unwrap_err();
        assert!(matches!(err, RunnerError::UnsupportedScenario { .. }));
    }

    #[test]
    fn rejects_iers_table_that_does_not_cover_run() {
        let scenario = Scenario::from_toml_str(IERS_SCENARIO).unwrap();
        let short_eop = EOP.replace("time_s = 10.0", "time_s = 5.0");
        let err = build_frame_context(&scenario.document, &resolved_eop(short_eop.as_bytes()))
            .unwrap_err();
        assert!(matches!(err, RunnerError::UnsupportedScenario { .. }));
    }

    #[test]
    fn rejects_partial_cip_offset_eop_sample() {
        let partial = EOP.replace("cip_offset_y_arcsec = -0.020\n", "");
        let file = ResolvedFile {
            path: PathBuf::from("eop.toml"),
            sha256_hex: "not-used-in-unit-test".to_owned(),
            bytes: partial.as_bytes().to_vec(),
        };
        let err = parse_earth_orientation_table(&file, FINALS2000A_2017_EPOCH_JD).unwrap_err();
        assert!(matches!(err, RunnerError::UnsupportedScenario { .. }));
    }
}
