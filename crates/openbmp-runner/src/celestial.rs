//! Runner-side celestial ephemeris and third-body gravity wiring.

use std::collections::BTreeMap;

use openbmp_core::{Eci, Position3, SimTime};
use openbmp_physics::{
    CelestialBody, Egm2008ZonalGravity, EphemerisModel, EphemerisState, GravityModel, J2Gravity,
    J2000_JULIAN_DATE, LowPrecisionSunMoonEphemeris, PointMassGravity, SpkEphemeris, ThirdBody,
    ThirdBodyGravity, WGS84_J2,
};
use openbmp_scenario::{ResolvedFile, ScenarioDocument};

use crate::error::RunnerError;

const SECONDS_PER_DAY: f64 = 86_400.0;
const TT_MINUS_TAI_S: f64 = 32.184;
const DEG_TO_RAD: f64 = std::f64::consts::PI / 180.0;

/// Concrete central-gravity variants available under
/// `environment.gravity = "third_body"`.
#[derive(Copy, Clone, Debug)]
pub(crate) enum RuntimeCentralGravity {
    /// Point-mass Earth central gravity.
    PointMass(PointMassGravity),
    /// J2 Earth central gravity.
    J2(J2Gravity),
    /// Pinned zonal-only EGM2008 central gravity.
    Egm2008(Egm2008ZonalGravity),
}

impl GravityModel for RuntimeCentralGravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<nalgebra::Vector3<f64>, openbmp_physics::PhysicsError> {
        match self {
            Self::PointMass(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::J2(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::Egm2008(model) => model.gravity_eci_m_s2(position_eci, time),
        }
    }
}

/// Concrete ephemeris variants available under
/// `environment.gravity = "third_body"`.
#[derive(Clone, Debug, PartialEq)]
pub(crate) enum RuntimeEphemeris {
    /// Built-in deterministic analytical Sun/Moon approximation.
    LowPrecisionSunMoon(LowPrecisionSunMoonEphemeris),
    /// Pinned binary SPK/BSP kernel.
    Spk(SpkEphemeris),
}

impl EphemerisModel for RuntimeEphemeris {
    fn body_position_eci_m(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<nalgebra::Vector3<f64>, openbmp_physics::PhysicsError> {
        match self {
            Self::LowPrecisionSunMoon(ephemeris) => ephemeris.body_position_eci_m(body, time),
            Self::Spk(ephemeris) => ephemeris.body_position_eci_m(body, time),
        }
    }

    fn body_state_eci_m_s(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<EphemerisState, openbmp_physics::PhysicsError> {
        match self {
            Self::LowPrecisionSunMoon(ephemeris) => ephemeris.body_state_eci_m_s(body, time),
            Self::Spk(ephemeris) => ephemeris.body_state_eci_m_s(body, time),
        }
    }
}

/// Concrete third-body gravity type used by both runner paths.
pub(crate) type RuntimeThirdBodyGravity = ThirdBodyGravity<RuntimeCentralGravity, RuntimeEphemeris>;

/// Build the configured third-body gravity model.
///
/// # Errors
///
/// Returns [`RunnerError`] if the scenario's validated third-body
/// gravity fields are internally inconsistent or the epoch cannot be
/// parsed by the runner's low-precision ephemeris path.
pub(crate) fn build_third_body_gravity(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RuntimeThirdBodyGravity, RunnerError> {
    let central = build_central_gravity(document)?;
    let ephemeris = build_ephemeris(document, resolved_files)?;
    let mut bodies = Vec::with_capacity(document.environment.third_bodies.len());
    for label in &document.environment.third_bodies {
        bodies.push(ThirdBody::canonical(parse_celestial_body(label)?));
    }
    Ok(ThirdBodyGravity::new(central, ephemeris, bodies)?)
}

fn build_ephemeris(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RuntimeEphemeris, RunnerError> {
    let ephemeris_kind = document
        .environment
        .ephemeris
        .as_deref()
        .unwrap_or("low_precision_sun_moon");
    match ephemeris_kind {
        "low_precision_sun_moon" => Ok(RuntimeEphemeris::LowPrecisionSunMoon(
            LowPrecisionSunMoonEphemeris::new(epoch_tdb_julian_date(
                document,
                resolved_files,
                false,
            )?)?,
        )),
        "spk" => {
            if document.epoch.is_none() {
                return Err(RunnerError::UnsupportedScenario {
                    what: "environment.ephemeris = \"spk\" requires [epoch]".to_owned(),
                });
            }
            let kernels = resolved_spk_kernels(document, resolved_files)?;
            Ok(RuntimeEphemeris::Spk(SpkEphemeris::from_kernels(
                epoch_tdb_julian_date(document, resolved_files, true)?,
                kernels,
            )?))
        }
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("environment.ephemeris = {other} is not wired"),
        }),
    }
}

fn resolved_spk_kernels<'a>(
    document: &ScenarioDocument,
    resolved_files: &'a BTreeMap<String, ResolvedFile>,
) -> Result<Vec<&'a [u8]>, RunnerError> {
    if document.environment.ephemeris_file.is_some() {
        let resolved = resolved_files
            .get("environment.ephemeris_file")
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what:
                    "environment.ephemeris = \"spk\" requires resolved environment.ephemeris_file"
                        .to_owned(),
            })?;
        return Ok(vec![resolved.bytes.as_slice()]);
    }
    if document.environment.ephemeris_meta_kernel.is_some() {
        let mut kernels = Vec::new();
        for index in 0.. {
            let key = format!("environment.ephemeris_meta_kernel.files[{index}]");
            let Some(resolved) = resolved_files.get(&key) else {
                break;
            };
            if resolved.bytes.starts_with(b"DAF/SPK ") {
                kernels.push(resolved.bytes.as_slice());
            }
        }
        if kernels.is_empty() {
            return Err(RunnerError::UnsupportedScenario {
                what: "environment.ephemeris_meta_kernel did not resolve any binary SPK kernels"
                    .to_owned(),
            });
        }
        return Ok(kernels);
    }
    let mut kernels = Vec::with_capacity(document.environment.ephemeris_files.len());
    for index in 0..document.environment.ephemeris_files.len() {
        let key = format!("environment.ephemeris_files[{index}]");
        let resolved =
            resolved_files
                .get(&key)
                .ok_or_else(|| RunnerError::UnsupportedScenario {
                    what: format!("environment.ephemeris = \"spk\" requires resolved {key}"),
                })?;
        kernels.push(resolved.bytes.as_slice());
    }
    if kernels.is_empty() {
        return Err(RunnerError::UnsupportedScenario {
            what: "environment.ephemeris = \"spk\" requires at least one resolved SPK kernel"
                .to_owned(),
        });
    }
    Ok(kernels)
}

fn build_central_gravity(
    document: &ScenarioDocument,
) -> Result<RuntimeCentralGravity, RunnerError> {
    let base = document
        .environment
        .gravity_base
        .as_deref()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.gravity_base missing for third_body gravity".to_owned(),
        })?;
    match base {
        "point_mass" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for third_body point_mass base"
                            .to_owned(),
                    })?;
            Ok(RuntimeCentralGravity::PointMass(PointMassGravity::new(mu)?))
        }
        "j2" => {
            let mu =
                document
                    .environment
                    .mu_m3_s2
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.mu_m3_s2 missing for third_body j2 base".to_owned(),
                    })?;
            let r_e =
                document
                    .environment
                    .r_e_m
                    .ok_or_else(|| RunnerError::UnsupportedScenario {
                        what: "environment.r_e_m missing for third_body j2 base".to_owned(),
                    })?;
            let j2 = document.environment.j2.unwrap_or(WGS84_J2);
            Ok(RuntimeCentralGravity::J2(J2Gravity::new(mu, r_e, j2)?))
        }
        "egm2008" => Ok(RuntimeCentralGravity::Egm2008(
            Egm2008ZonalGravity::wgs84_egm2008_zonal(),
        )),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("environment.gravity_base = {other} is not wired"),
        }),
    }
}

fn parse_celestial_body(label: &str) -> Result<CelestialBody, RunnerError> {
    match label {
        "sun" => Ok(CelestialBody::Sun),
        "moon" => Ok(CelestialBody::Moon),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("environment.third_bodies contains unsupported body `{other}`"),
        }),
    }
}

fn epoch_tdb_julian_date(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
    require_leap_seconds_for_utc: bool,
) -> Result<f64, RunnerError> {
    let Some(epoch) = &document.epoch else {
        return Ok(J2000_JULIAN_DATE);
    };
    let scale = epoch.scale.to_ascii_uppercase();
    let jd = parse_iso8601_julian_date(&epoch.iso8601)?;
    match scale.as_str() {
        "TDB" => Ok(jd),
        "TT" => Ok(tt_to_tdb_julian_date(jd)),
        "UTC" => {
            let Some(table) = resolved_leap_second_table(document, resolved_files)? else {
                if require_leap_seconds_for_utc {
                    return Err(RunnerError::UnsupportedScenario {
                        what: "epoch.scale = \"UTC\" requires resolved epoch.leap_second_table or a NAIF LSK in environment.ephemeris_meta_kernel for TDB ephemeris conversion".to_owned(),
                    });
                }
                return Ok(jd);
            };
            Ok(utc_to_tdb_julian_date(jd, &table)?)
        }
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.scale = {} is not supported by ephemeris conversion",
                epoch.scale
            ),
        }),
    }
}

pub(crate) fn parse_iso8601_julian_date(value: &str) -> Result<f64, RunnerError> {
    let trimmed = value.trim();
    if trimmed.is_empty() {
        return Err(RunnerError::UnsupportedScenario {
            what: "epoch.iso8601 must be non-empty for third_body gravity".to_owned(),
        });
    }
    let without_z = trimmed.strip_suffix('Z').unwrap_or(trimmed);
    let normalized = without_z.replace('T', " ");
    let mut parts = normalized.split_whitespace();
    let date = parts
        .next()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.iso8601 `{value}` is missing date"),
        })?;
    let time = parts.next().unwrap_or("00:00:00");
    if parts.next().is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.iso8601 `{value}` has extra fields"),
        });
    }

    let mut date_parts = date.split('-');
    let year = parse_i32(date_parts.next(), value, "year")?;
    let month = parse_u32(date_parts.next(), value, "month")?;
    let day = parse_u32(date_parts.next(), value, "day")?;
    if date_parts.next().is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.iso8601 `{value}` has invalid date"),
        });
    }

    let mut time_parts = time.split(':');
    let hour = parse_u32(time_parts.next(), value, "hour")?;
    let minute = parse_u32(time_parts.next(), value, "minute")?;
    let second = parse_f64(time_parts.next(), value, "second")?;
    if time_parts.next().is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.iso8601 `{value}` has invalid time"),
        });
    }
    julian_date_from_gregorian(year, month, day, hour, minute, second, value)
}

#[derive(Clone, Debug, PartialEq)]
struct LeapSecondEntry {
    effective_utc_julian_date: f64,
    tai_minus_utc_s: f64,
}

#[derive(Clone, Debug, PartialEq)]
struct LeapSecondTable {
    entries: Vec<LeapSecondEntry>,
}

impl LeapSecondTable {
    fn new(entries: Vec<LeapSecondEntry>) -> Result<Self, RunnerError> {
        if entries.is_empty() {
            return Err(RunnerError::UnsupportedScenario {
                what: "epoch.leap_second_table requires at least one [[entries]] row".to_owned(),
            });
        }
        for entry in &entries {
            if !entry.effective_utc_julian_date.is_finite() || !entry.tai_minus_utc_s.is_finite() {
                return Err(RunnerError::UnsupportedScenario {
                    what: "epoch.leap_second_table entries must be finite".to_owned(),
                });
            }
        }
        for pair in entries.windows(2) {
            if pair[0].effective_utc_julian_date >= pair[1].effective_utc_julian_date {
                return Err(RunnerError::UnsupportedScenario {
                    what: "epoch.leap_second_table entries must be strictly time-ordered"
                        .to_owned(),
                });
            }
        }
        Ok(Self { entries })
    }

    fn tai_minus_utc_s(&self, utc_julian_date: f64) -> Result<f64, RunnerError> {
        let index = self
            .entries
            .partition_point(|entry| entry.effective_utc_julian_date <= utc_julian_date);
        if index == 0 {
            return Err(RunnerError::UnsupportedScenario {
                what: "epoch.leap_second_table does not cover epoch.iso8601".to_owned(),
            });
        }
        Ok(self.entries[index - 1].tai_minus_utc_s)
    }
}

fn resolved_leap_second_table(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<Option<LeapSecondTable>, RunnerError> {
    if let Some(resolved) = resolved_files.get("epoch.leap_second_table") {
        return parse_leap_second_table(resolved).map(Some);
    }
    if document.environment.ephemeris_meta_kernel.is_none() {
        return Ok(None);
    }
    let mut table = None;
    for index in 0.. {
        let key = format!("environment.ephemeris_meta_kernel.files[{index}]");
        let Some(resolved) = resolved_files.get(&key) else {
            break;
        };
        if is_naif_leap_second_kernel(resolved) {
            table = Some(parse_leap_second_table(resolved)?);
        }
    }
    Ok(table)
}

fn parse_leap_second_table(resolved: &ResolvedFile) -> Result<LeapSecondTable, RunnerError> {
    let text =
        std::str::from_utf8(&resolved.bytes).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table is not valid UTF-8: {err}"),
        })?;
    if text.trim_start().starts_with("KPL/LSK") || text.contains("DELTET/DELTA_AT") {
        return parse_naif_leap_second_kernel(text);
    }
    parse_openbmp_leap_second_table(text)
}

fn is_naif_leap_second_kernel(resolved: &ResolvedFile) -> bool {
    let Ok(text) = std::str::from_utf8(&resolved.bytes) else {
        return false;
    };
    text.trim_start().starts_with("KPL/LSK") || text.contains("DELTET/DELTA_AT")
}

fn parse_openbmp_leap_second_table(text: &str) -> Result<LeapSecondTable, RunnerError> {
    let value: toml::Value =
        toml::from_str(text).map_err(|err| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table TOML parse failed: {err}"),
        })?;
    if let Some(format) = value.get("format") {
        let format = format
            .as_str()
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "epoch.leap_second_table format must be a string".to_owned(),
            })?;
        if format != "openbmp-leap-seconds-v1" {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("epoch.leap_second_table format \"{format}\" is not supported"),
            });
        }
    }
    let entries = value
        .get("entries")
        .and_then(toml::Value::as_array)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "epoch.leap_second_table requires [[entries]] rows".to_owned(),
        })?;
    let mut parsed = Vec::with_capacity(entries.len());
    for (index, entry) in entries.iter().enumerate() {
        parsed.push(parse_leap_second_entry(index, entry)?);
    }
    LeapSecondTable::new(parsed)
}

fn parse_naif_leap_second_kernel(text: &str) -> Result<LeapSecondTable, RunnerError> {
    let data = if let Some((_, after_begin)) = text.split_once("\\begindata") {
        after_begin
            .split_once("\\begintext")
            .map_or(after_begin, |(before_end, _)| before_end)
    } else {
        text
    };
    let assignment = data
        .split_once("DELTET/DELTA_AT")
        .map(|(_, after)| after)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "epoch.leap_second_table NAIF LSK missing DELTET/DELTA_AT".to_owned(),
        })?;
    let after_equals = assignment
        .split_once('=')
        .map(|(_, after)| after)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "epoch.leap_second_table NAIF LSK DELTET/DELTA_AT missing `=`".to_owned(),
        })?;
    let body = after_equals
        .split_once('(')
        .and_then(|(_, after_open)| after_open.split_once(')').map(|(inner, _)| inner))
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "epoch.leap_second_table NAIF LSK DELTET/DELTA_AT missing parenthesized entries"
                .to_owned(),
        })?;
    let normalized = body.replace(',', " ");
    let mut tokens = normalized.split_whitespace();
    let mut entries = Vec::new();
    while let Some(delta_at) = tokens.next() {
        let Some(effective_utc) = tokens.next() else {
            return Err(RunnerError::UnsupportedScenario {
                what: "epoch.leap_second_table NAIF LSK DELTET/DELTA_AT has an unmatched value"
                    .to_owned(),
            });
        };
        let tai_minus_utc_s =
            delta_at
                .parse::<f64>()
                .map_err(|_| RunnerError::UnsupportedScenario {
                    what: format!(
                        "epoch.leap_second_table NAIF LSK DELTET/DELTA_AT value `{delta_at}` is not numeric"
                    ),
                })?;
        entries.push(LeapSecondEntry {
            effective_utc_julian_date: parse_naif_lsk_epoch(effective_utc)?,
            tai_minus_utc_s,
        });
    }
    LeapSecondTable::new(entries)
}

fn parse_leap_second_entry(
    index: usize,
    entry: &toml::Value,
) -> Result<LeapSecondEntry, RunnerError> {
    let table = entry
        .as_table()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table entries[{index}] must be a table"),
        })?;
    let effective_utc = table
        .get("effective_utc")
        .and_then(toml::Value::as_str)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table entries[{index}].effective_utc is required"),
        })?;
    let tai_minus_utc_s = leap_number_at(table, "tai_minus_utc_s", index)?;
    Ok(LeapSecondEntry {
        effective_utc_julian_date: parse_iso8601_julian_date(effective_utc)?,
        tai_minus_utc_s,
    })
}

#[allow(clippy::cast_precision_loss)]
fn leap_number_at(
    table: &toml::map::Map<String, toml::Value>,
    key: &str,
    index: usize,
) -> Result<f64, RunnerError> {
    let value = table
        .get(key)
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table entries[{index}].{key} is required"),
        })?;
    match value {
        toml::Value::Float(v) => Ok(*v),
        toml::Value::Integer(v) => Ok(*v as f64),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table entries[{index}].{key} must be numeric"),
        }),
    }
}

fn parse_naif_lsk_epoch(value: &str) -> Result<f64, RunnerError> {
    let trimmed =
        value
            .trim()
            .strip_prefix('@')
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!(
                    "epoch.leap_second_table NAIF LSK date `{value}` must start with `@`"
                ),
            })?;
    let mut parts = trimmed.split('-');
    let year = parts
        .next()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK date `{value}` is missing year"),
        })?;
    let month = parts
        .next()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK date `{value}` is missing month"),
        })?;
    let day = parts
        .next()
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK date `{value}` is missing day"),
        })?;
    if parts.next().is_some() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK date `{value}` has extra fields"),
        });
    }
    let year = year
        .parse::<i32>()
        .map_err(|_| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK date `{value}` has invalid year"),
        })?;
    let month = naif_lsk_month_number(month)?;
    let day = day
        .parse::<u32>()
        .map_err(|_| RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK date `{value}` has invalid day"),
        })?;
    julian_date_from_gregorian(year, month, day, 0, 0, 0.0, value)
}

fn naif_lsk_month_number(month: &str) -> Result<u32, RunnerError> {
    match month.to_ascii_uppercase().as_str() {
        "JAN" => Ok(1),
        "FEB" => Ok(2),
        "MAR" => Ok(3),
        "APR" => Ok(4),
        "MAY" => Ok(5),
        "JUN" => Ok(6),
        "JUL" => Ok(7),
        "AUG" => Ok(8),
        "SEP" => Ok(9),
        "OCT" => Ok(10),
        "NOV" => Ok(11),
        "DEC" => Ok(12),
        _ => Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.leap_second_table NAIF LSK month `{month}` is not supported"),
        }),
    }
}

fn utc_to_tdb_julian_date(
    utc_julian_date: f64,
    leap_seconds: &LeapSecondTable,
) -> Result<f64, RunnerError> {
    let tai_minus_utc_s = leap_seconds.tai_minus_utc_s(utc_julian_date)?;
    let tt_julian_date = utc_julian_date + (tai_minus_utc_s + TT_MINUS_TAI_S) / SECONDS_PER_DAY;
    Ok(tt_to_tdb_julian_date(tt_julian_date))
}

fn tt_to_tdb_julian_date(tt_julian_date: f64) -> f64 {
    let days_since_j2000 = tt_julian_date - J2000_JULIAN_DATE;
    let mean_anomaly_rad = (357.53 + 0.985_600_3 * days_since_j2000) * DEG_TO_RAD;
    let tdb_minus_tt_s =
        0.001_657 * mean_anomaly_rad.sin() + 0.000_013_85 * (2.0 * mean_anomaly_rad).sin();
    tt_julian_date + tdb_minus_tt_s / SECONDS_PER_DAY
}

fn parse_i32(part: Option<&str>, original: &str, field: &str) -> Result<i32, RunnerError> {
    part.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("epoch.iso8601 `{original}` is missing {field}"),
    })?
    .parse::<i32>()
    .map_err(|_| RunnerError::UnsupportedScenario {
        what: format!("epoch.iso8601 `{original}` has invalid {field}"),
    })
}

fn parse_u32(part: Option<&str>, original: &str, field: &str) -> Result<u32, RunnerError> {
    part.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("epoch.iso8601 `{original}` is missing {field}"),
    })?
    .parse::<u32>()
    .map_err(|_| RunnerError::UnsupportedScenario {
        what: format!("epoch.iso8601 `{original}` has invalid {field}"),
    })
}

fn parse_f64(part: Option<&str>, original: &str, field: &str) -> Result<f64, RunnerError> {
    part.ok_or_else(|| RunnerError::UnsupportedScenario {
        what: format!("epoch.iso8601 `{original}` is missing {field}"),
    })?
    .parse::<f64>()
    .map_err(|_| RunnerError::UnsupportedScenario {
        what: format!("epoch.iso8601 `{original}` has invalid {field}"),
    })
}

#[allow(clippy::cast_precision_loss)]
fn julian_date_from_gregorian(
    year: i32,
    month: u32,
    day: u32,
    hour: u32,
    minute: u32,
    second: f64,
    original: &str,
) -> Result<f64, RunnerError> {
    if !(1..=12).contains(&month)
        || !(1..=31).contains(&day)
        || hour > 23
        || minute > 59
        || !(0.0..60.0).contains(&second)
    {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.iso8601 `{original}` is outside supported calendar bounds"),
        });
    }
    let mut y = year;
    let mut m = month as i32;
    if m <= 2 {
        y -= 1;
        m += 12;
    }
    let a = (f64::from(y) / 100.0).floor();
    let b = 2.0 - a + (a / 4.0).floor();
    let jd0 = (365.25 * f64::from(y + 4_716)).floor()
        + (30.600_1 * f64::from(m + 1)).floor()
        + f64::from(day)
        + b
        - 1_524.5;
    let day_fraction = (f64::from(hour) + f64::from(minute) / 60.0 + second / 3_600.0) / 24.0;
    let jd = jd0 + day_fraction;
    if !jd.is_finite() {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("epoch.iso8601 `{original}` produced non-finite Julian Date"),
        });
    }
    Ok(jd)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::panic)]
mod tests {
    use super::*;
    use openbmp_scenario::Scenario;
    use std::path::PathBuf;

    #[test]
    fn parses_j2000_noon_epoch() {
        let jd = parse_iso8601_julian_date("2000-01-01T12:00:00Z").unwrap();
        assert!((jd - J2000_JULIAN_DATE).abs() < 1.0e-9);
    }

    #[test]
    fn parses_midnight_before_j2000() {
        let jd = parse_iso8601_julian_date("2000-01-01T00:00:00Z").unwrap();
        assert!((jd - (J2000_JULIAN_DATE - 0.5)).abs() < 1.0e-9);
    }

    #[test]
    fn leap_second_table_converts_utc_epoch_to_tdb_axis() {
        let table = parse_leap_second_table(&resolved_file("leaps.toml", LEAP_SECOND_TABLE))
            .expect("parse leap-second table");
        let utc = parse_iso8601_julian_date("2017-01-01T00:00:00Z").unwrap();
        let tdb = utc_to_tdb_julian_date(utc, &table).unwrap();
        let offset_s = (tdb - utc) * SECONDS_PER_DAY;
        assert!((69.18..69.19).contains(&offset_s));
    }

    #[test]
    fn naif_lsk_converts_utc_epoch_to_tdb_axis() {
        let table = parse_leap_second_table(&resolved_file("naif0012.tls", NAIF_LSK))
            .expect("parse NAIF leap-second kernel");
        let utc = parse_iso8601_julian_date("2017-01-01T00:00:00Z").unwrap();
        let tdb = utc_to_tdb_julian_date(utc, &table).unwrap();
        let offset_s = (tdb - utc) * SECONDS_PER_DAY;
        assert!((69.18..69.19).contains(&offset_s));
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_resolved_file() {
        let scenario = Scenario::from_toml_str(SPK_THIRD_BODY_SCENARIO).unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "environment.ephemeris_file".to_owned(),
            resolved_bytes("synthetic.bsp", synthetic_spk()),
        );
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => assert_eq!(spk.segment_count(), 4),
            other => panic!("expected SPK ephemeris, got {other:?}"),
        }
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_resolved_file_list() {
        let toml = SPK_THIRD_BODY_SCENARIO.replace(
            "ephemeris_file = \"synthetic.bsp\"",
            "ephemeris_files = [\"base.bsp\", \"override.bsp\"]",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut files = BTreeMap::new();
        for (index, name) in ["base.bsp", "override.bsp"].iter().enumerate() {
            files.insert(
                format!("environment.ephemeris_files[{index}]"),
                resolved_bytes(name, synthetic_spk()),
            );
        }
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => assert_eq!(spk.segment_count(), 8),
            other => panic!("expected SPK ephemeris, got {other:?}"),
        }
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_meta_kernel_files() {
        let toml = SPK_THIRD_BODY_SCENARIO.replace(
            "ephemeris_file = \"synthetic.bsp\"",
            "ephemeris_meta_kernel = \"mission.tm\"",
        );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "environment.ephemeris_meta_kernel".to_owned(),
            resolved_file("mission.tm", "KPL/MK\n"),
        );
        files.insert(
            "environment.ephemeris_meta_kernel.files[0]".to_owned(),
            resolved_file("naif0012.tls", "KPL/LSK\n"),
        );
        files.insert(
            "environment.ephemeris_meta_kernel.files[1]".to_owned(),
            resolved_bytes("de440s.bsp", synthetic_spk()),
        );
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => assert_eq!(spk.segment_count(), 4),
            other => panic!("expected SPK ephemeris, got {other:?}"),
        }
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_utc_epoch_with_leap_seconds() {
        let toml = SPK_THIRD_BODY_SCENARIO
            .replace("scale = \"TDB\"", "scale = \"UTC\"")
            .replace(
                "iso8601 = \"2000-01-01T12:00:00Z\"",
                "iso8601 = \"2017-01-01T00:00:00Z\"\nleap_second_table = \"leaps.toml\"",
            );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "environment.ephemeris_file".to_owned(),
            resolved_bytes("synthetic.bsp", synthetic_spk()),
        );
        files.insert(
            "epoch.leap_second_table".to_owned(),
            resolved_file("leaps.toml", LEAP_SECOND_TABLE),
        );
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => assert_eq!(spk.segment_count(), 4),
            other => panic!("expected SPK ephemeris, got {other:?}"),
        }
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_utc_epoch_with_naif_lsk() {
        let toml = SPK_THIRD_BODY_SCENARIO
            .replace("scale = \"TDB\"", "scale = \"UTC\"")
            .replace(
                "iso8601 = \"2000-01-01T12:00:00Z\"",
                "iso8601 = \"2017-01-01T00:00:00Z\"\nleap_second_table = \"naif0012.tls\"",
            );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "environment.ephemeris_file".to_owned(),
            resolved_bytes("synthetic.bsp", synthetic_spk()),
        );
        files.insert(
            "epoch.leap_second_table".to_owned(),
            resolved_file("naif0012.tls", NAIF_LSK),
        );
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => assert_eq!(spk.segment_count(), 4),
            other => panic!("expected SPK ephemeris, got {other:?}"),
        }
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_utc_epoch_with_meta_kernel_lsk() {
        let toml = SPK_THIRD_BODY_SCENARIO
            .replace("scale = \"TDB\"", "scale = \"UTC\"")
            .replace(
                "iso8601 = \"2000-01-01T12:00:00Z\"",
                "iso8601 = \"2017-01-01T00:00:00Z\"",
            )
            .replace(
                "ephemeris_file = \"synthetic.bsp\"",
                "ephemeris_meta_kernel = \"mission.tm\"",
            );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "environment.ephemeris_meta_kernel".to_owned(),
            resolved_file("mission.tm", "KPL/MK\n"),
        );
        files.insert(
            "environment.ephemeris_meta_kernel.files[0]".to_owned(),
            resolved_file("naif0012.tls", NAIF_LSK),
        );
        files.insert(
            "environment.ephemeris_meta_kernel.files[1]".to_owned(),
            resolved_bytes("de440s.bsp", synthetic_spk()),
        );
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => assert_eq!(spk.segment_count(), 4),
            other => panic!("expected SPK ephemeris, got {other:?}"),
        }
    }

    const LEAP_SECOND_TABLE: &str = r#"
format = "openbmp-leap-seconds-v1"

[[entries]]
effective_utc = "2015-07-01T00:00:00Z"
tai_minus_utc_s = 36

[[entries]]
effective_utc = "2017-01-01T00:00:00Z"
tai_minus_utc_s = 37
"#;

    const NAIF_LSK: &str = r#"
KPL/LSK

\begindata

DELTET/DELTA_T_A = 32.184
DELTET/K         = 1.657D-3
DELTET/EB        = 1.671D-2
DELTET/M         = ( 6.239996D0 1.99096871D-7 )
DELTET/DELTA_AT  = ( 10, @1972-JAN-1
                     36, @2015-JUL-1
                     37,
                     @2017-JAN-1 )

\begintext
"#;

    const SPK_THIRD_BODY_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "spk-third-body-test"
description = "SPK third-body wiring test."
validation = "validated-toy"
provenance = "synthetic runner unit test"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 1.0
seed = 1

[epoch]
scale = "TDB"
iso8601 = "2000-01-01T12:00:00Z"

[vehicle]
kind = "point_mass"
initial_position_eci_m = [0.0, 0.0, 0.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "spk-test"

[[vehicle.assembly.bodies]]
id          = "main"
geometry    = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0

[environment]
frame_profile = "toy-fixed-earth"
gravity       = "third_body"
gravity_base  = "point_mass"
mu_m3_s2      = 3.986004418e14
third_bodies  = ["sun", "moon"]
ephemeris     = "spk"
ephemeris_file = "synthetic.bsp"
atmosphere    = "none"
wind          = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/spk-third-body-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    #[derive(Copy, Clone)]
    struct SyntheticSegment {
        target: i32,
        center: i32,
        position_km: [f64; 3],
    }

    fn synthetic_spk() -> Vec<u8> {
        const RECORD_BYTES: usize = 1_024;
        const WORDS_PER_RECORD: usize = 128;
        const SUMMARY_CONTROL_WORDS: usize = 3;
        const SUMMARY_WORDS: usize = 5;
        const J2000_FRAME: i32 = 1;
        let segments = [
            SyntheticSegment {
                target: 3,
                center: 0,
                position_km: [4_700.0, 1_200.0, -300.0],
            },
            SyntheticSegment {
                target: 399,
                center: 3,
                position_km: [0.0, 0.0, 0.0],
            },
            SyntheticSegment {
                target: 10,
                center: 0,
                position_km: [149_597_870.0, 0.0, 0.0],
            },
            SyntheticSegment {
                target: 301,
                center: 3,
                position_km: [384_400.0, 0.0, 0.0],
            },
        ];
        let mut bytes = vec![0_u8; (4 + segments.len()) * RECORD_BYTES];
        bytes[0..8].copy_from_slice(b"DAF/SPK ");
        write_i32(&mut bytes, 8, 2);
        write_i32(&mut bytes, 12, 6);
        write_i32(&mut bytes, 76, 2);
        write_i32(&mut bytes, 80, 2);
        write_i32(&mut bytes, 84, 1);
        bytes[88..96].copy_from_slice(b"LTL-IEEE");

        let summary_offset = RECORD_BYTES;
        write_f64(&mut bytes, summary_offset, 0.0);
        write_f64(&mut bytes, summary_offset + 8, 0.0);
        write_f64(&mut bytes, summary_offset + 16, segments.len() as f64);
        for (index, segment) in segments.iter().enumerate() {
            let offset = summary_offset + (SUMMARY_CONTROL_WORDS + index * SUMMARY_WORDS) * 8;
            write_f64(&mut bytes, offset, -10.0);
            write_f64(&mut bytes, offset + 8, 10.0);
            write_i32(&mut bytes, offset + 16, segment.target);
            write_i32(&mut bytes, offset + 20, segment.center);
            write_i32(&mut bytes, offset + 24, J2000_FRAME);
            write_i32(&mut bytes, offset + 28, 2);
            let address = (WORDS_PER_RECORD * (3 + index) + 1) as i32;
            write_i32(&mut bytes, offset + 32, address);
            write_i32(&mut bytes, offset + 36, address + 8);
            let data = type2_constant_segment(segment.position_km);
            for (data_index, value) in data.iter().enumerate() {
                write_f64(
                    &mut bytes,
                    ((address as usize - 1) + data_index) * 8,
                    *value,
                );
            }
        }
        bytes
    }

    fn resolved_file(path: &str, text: &str) -> ResolvedFile {
        resolved_bytes(path, text.as_bytes().to_vec())
    }

    fn resolved_bytes(path: &str, bytes: Vec<u8>) -> ResolvedFile {
        ResolvedFile {
            path: PathBuf::from(path),
            sha256_hex: "not-used-in-unit-test".to_owned(),
            bytes,
        }
    }

    fn type2_constant_segment(position_km: [f64; 3]) -> [f64; 9] {
        [
            0.0,
            10.0,
            position_km[0],
            position_km[1],
            position_km[2],
            -10.0,
            20.0,
            5.0,
            1.0,
        ]
    }

    fn write_i32(bytes: &mut [u8], offset: usize, value: i32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn write_f64(bytes: &mut [u8], offset: usize, value: f64) {
        bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}
