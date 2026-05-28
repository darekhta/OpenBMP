//! Runner-side celestial ephemeris and third-body gravity wiring.

use openbmp_core::{Eci, Position3, SimTime};
use openbmp_physics::{
    CelestialBody, Egm2008ZonalGravity, GravityModel, J2Gravity, J2000_JULIAN_DATE,
    LowPrecisionSunMoonEphemeris, PointMassGravity, ThirdBody, ThirdBodyGravity, WGS84_J2,
};
use openbmp_scenario::ScenarioDocument;

use crate::error::RunnerError;

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

/// Concrete third-body gravity type used by both runner paths.
pub(crate) type RuntimeThirdBodyGravity =
    ThirdBodyGravity<RuntimeCentralGravity, LowPrecisionSunMoonEphemeris>;

/// Build the configured third-body gravity model.
///
/// # Errors
///
/// Returns [`RunnerError`] if the scenario's validated third-body
/// gravity fields are internally inconsistent or the epoch cannot be
/// parsed by the runner's low-precision ephemeris path.
pub(crate) fn build_third_body_gravity(
    document: &ScenarioDocument,
) -> Result<RuntimeThirdBodyGravity, RunnerError> {
    let central = build_central_gravity(document)?;
    let ephemeris_kind = document
        .environment
        .ephemeris
        .as_deref()
        .unwrap_or("low_precision_sun_moon");
    if ephemeris_kind != "low_precision_sun_moon" {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("environment.ephemeris = {ephemeris_kind} is not wired"),
        });
    }
    let ephemeris = LowPrecisionSunMoonEphemeris::new(epoch_julian_date(document)?)?;
    let mut bodies = Vec::with_capacity(document.environment.third_bodies.len());
    for label in &document.environment.third_bodies {
        bodies.push(ThirdBody::canonical(parse_celestial_body(label)?));
    }
    Ok(ThirdBodyGravity::new(central, ephemeris, bodies)?)
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

fn epoch_julian_date(document: &ScenarioDocument) -> Result<f64, RunnerError> {
    let Some(epoch) = &document.epoch else {
        return Ok(J2000_JULIAN_DATE);
    };
    let scale = epoch.scale.to_ascii_uppercase();
    if !matches!(scale.as_str(), "UTC" | "TT" | "TDB") {
        return Err(RunnerError::UnsupportedScenario {
            what: format!(
                "epoch.scale = {} is not supported by low_precision_sun_moon ephemeris",
                epoch.scale
            ),
        });
    }
    parse_iso8601_julian_date(&epoch.iso8601)
}

fn parse_iso8601_julian_date(value: &str) -> Result<f64, RunnerError> {
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
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

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
}
