//! Runner-side celestial ephemeris and third-body gravity wiring.

use std::collections::BTreeMap;

use nalgebra::Matrix3;
use openbmp_core::{Eci, FrameError, Position3, SimTime};
use openbmp_physics::{
    CelestialBody, DegreeTwoTesseralCoefficients, EarthFixedGravity, Egm2008ZonalGravity,
    EphemerisModel, EphemerisState, FiniteDifferencePinesGravity, GravityModel,
    HarmonicSynthesisTier, J2Gravity, J2000_JULIAN_DATE, LocalGeodeticOrigin,
    LowPrecisionSunMoonEphemeris, PointMassGravity, SpkEphemeris, SpkFixedFrame, TesseralGravity,
    ThirdBody, ThirdBodyGravity, TideSystem, TimeScaleBridge, WGS84_J2,
};
use openbmp_scenario::{ResolvedFile, ScenarioDocument};

use crate::error::RunnerError;

#[cfg(test)]
const SECONDS_PER_DAY: f64 = 86_400.0;

const DEFAULT_PINES_FINITE_DIFFERENCE_STEP_M: f64 = 10.0;

/// EGM2008 central-gravity runtime selected by the scenario.
#[derive(Clone, Debug)]
pub(crate) enum RuntimeEgm2008Gravity {
    /// Legacy pinned degree-2 through degree-6 zonal-only model.
    Zonal(Egm2008ZonalGravity),
    /// Opt-in coefficient-file transition model evaluated through the
    /// selected Earth-fixed frame.
    Pines(EarthFixedGravity<FiniteDifferencePinesGravity>),
}

impl GravityModel for RuntimeEgm2008Gravity {
    fn gravity_eci_m_s2(
        &self,
        position_eci: Position3<Eci>,
        time: SimTime,
    ) -> Result<nalgebra::Vector3<f64>, openbmp_physics::PhysicsError> {
        match self {
            Self::Zonal(model) => model.gravity_eci_m_s2(position_eci, time),
            Self::Pines(model) => model.gravity_eci_m_s2(position_eci, time),
        }
    }
}

/// Concrete central-gravity variants available under
/// `environment.gravity = "third_body"`.
#[derive(Clone, Debug)]
pub(crate) enum RuntimeCentralGravity {
    /// Point-mass Earth central gravity.
    PointMass(PointMassGravity),
    /// J2 Earth central gravity.
    J2(J2Gravity),
    /// Pinned or coefficient-file EGM2008 central gravity.
    Egm2008(RuntimeEgm2008Gravity),
    /// Frame-coupled degree-2 tesseral/sectoral Earth central gravity.
    Tesseral(EarthFixedGravity<TesseralGravity>),
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
            Self::Tesseral(model) => model.gravity_eci_m_s2(position_eci, time),
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
    let central = build_central_gravity(document, resolved_files)?;
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
            let fixed_frames = resolved_spk_fixed_frames(document, resolved_files)?;
            Ok(RuntimeEphemeris::Spk(
                SpkEphemeris::from_kernels_with_fixed_frames(
                    epoch_tdb_julian_date(document, resolved_files, true)?,
                    kernels,
                    fixed_frames,
                )?,
            ))
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

#[derive(Clone, Debug, PartialEq)]
struct SpiceKernelAssignment {
    key: String,
    value: String,
}

fn resolved_spk_fixed_frames(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<BTreeMap<i32, SpkFixedFrame>, RunnerError> {
    let mut fixed_frames = BTreeMap::new();
    if document.environment.ephemeris_meta_kernel.is_none() {
        return Ok(fixed_frames);
    }
    for index in 0.. {
        let key = format!("environment.ephemeris_meta_kernel.files[{index}]");
        let Some(resolved) = resolved_files.get(&key) else {
            break;
        };
        if resolved.bytes.starts_with(b"DAF/SPK ") {
            continue;
        }
        let Ok(text) = std::str::from_utf8(&resolved.bytes) else {
            continue;
        };
        if !text.to_ascii_uppercase().contains("TKFRAME_") {
            continue;
        }
        fixed_frames.extend(parse_spk_fixed_frames_text_kernel(text)?);
    }
    Ok(fixed_frames)
}

fn parse_spk_fixed_frames_text_kernel(
    text: &str,
) -> Result<BTreeMap<i32, SpkFixedFrame>, RunnerError> {
    let assignments = spice_text_kernel_assignments(text);
    let mut values = BTreeMap::new();
    for assignment in assignments {
        values.insert(assignment.key, assignment.value);
    }
    let frame_names = spice_frame_name_assignments(&values);
    let mut fixed_frames = BTreeMap::new();
    for (key, spec_value) in &values {
        let Some(frame_token) = key
            .strip_prefix("TKFRAME_")
            .and_then(|suffix| suffix.strip_suffix("_SPEC"))
        else {
            continue;
        };
        let relative_key = format!("TKFRAME_{frame_token}_RELATIVE");
        let Some(spec) = spice_scalar(spec_value) else {
            continue;
        };
        let Some(frame_id) = spice_frame_reference_to_id(frame_token, &frame_names) else {
            continue;
        };
        let Some(relative) = values
            .get(&relative_key)
            .and_then(|value| spice_scalar(value))
        else {
            continue;
        };
        let Some(relative_frame) = spice_frame_reference_to_id(&relative, &frame_names) else {
            continue;
        };
        let Some(frame_to_relative) = spice_tkframe_matrix(frame_token, &spec, &values)? else {
            continue;
        };
        fixed_frames.insert(
            frame_id,
            SpkFixedFrame::new(relative_frame, frame_to_relative)?,
        );
    }
    Ok(fixed_frames)
}

fn spice_tkframe_matrix(
    frame_token: &str,
    spec: &str,
    values: &BTreeMap<String, String>,
) -> Result<Option<Matrix3<f64>>, RunnerError> {
    match spec.to_ascii_uppercase().as_str() {
        "MATRIX" => {
            let matrix_key = format!("TKFRAME_{frame_token}_MATRIX");
            let Some(matrix_value) = values.get(&matrix_key) else {
                return Ok(None);
            };
            let entries = spice_numeric_values(matrix_value)?;
            if entries.len() != 9 {
                return Err(RunnerError::UnsupportedScenario {
                    what: format!(
                        "SPICE TKFRAME_{frame_token}_MATRIX must contain exactly 9 numeric values"
                    ),
                });
            }
            Ok(Some(Matrix3::new(
                entries[0], entries[3], entries[6], entries[1], entries[4], entries[7], entries[2],
                entries[5], entries[8],
            )))
        }
        "ANGLES" => spice_tkframe_angles_matrix(frame_token, values).map(Some),
        "QUATERNION" => spice_tkframe_quaternion_matrix(frame_token, values).map(Some),
        _ => Ok(None),
    }
}

fn spice_tkframe_angles_matrix(
    frame_token: &str,
    values: &BTreeMap<String, String>,
) -> Result<Matrix3<f64>, RunnerError> {
    let angles_key = format!("TKFRAME_{frame_token}_ANGLES");
    let axes_key = format!("TKFRAME_{frame_token}_AXES");
    let units_key = format!("TKFRAME_{frame_token}_UNITS");
    let angles = required_spice_numeric_values(values, &angles_key)?;
    let axes = required_spice_numeric_values(values, &axes_key)?;
    if angles.len() != 3 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("SPICE {angles_key} must contain exactly 3 numeric values"),
        });
    }
    if axes.len() != 3 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("SPICE {axes_key} must contain exactly 3 numeric values"),
        });
    }
    let units = values
        .get(&units_key)
        .and_then(|value| spice_scalar(value))
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("SPICE {units_key} is required for ANGLES TK frames"),
        })?;
    let scale =
        spice_angle_unit_to_radians(&units).ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("SPICE {units_key} value `{units}` is not supported"),
        })?;
    let mut axis_values = [0_i32; 3];
    for (index, value) in axes.iter().enumerate() {
        axis_values[index] =
            spice_axis_number(*value).ok_or_else(|| RunnerError::UnsupportedScenario {
                what: format!("SPICE {axes_key}[{index}] must be one of 1, 2, or 3"),
            })?;
    }
    Ok(spice_euler_matrix(
        [angles[0] * scale, angles[1] * scale, angles[2] * scale],
        axis_values,
    ))
}

fn spice_tkframe_quaternion_matrix(
    frame_token: &str,
    values: &BTreeMap<String, String>,
) -> Result<Matrix3<f64>, RunnerError> {
    let q_key = format!("TKFRAME_{frame_token}_Q");
    let q = required_spice_numeric_values(values, &q_key)?;
    if q.len() != 4 {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("SPICE {q_key} must contain exactly 4 numeric values"),
        });
    }
    Ok(spice_quaternion_to_matrix([q[0], q[1], q[2], q[3]]))
}

fn spice_text_kernel_assignments(text: &str) -> Vec<SpiceKernelAssignment> {
    let parse_all = !text
        .lines()
        .any(|line| line.trim().eq_ignore_ascii_case("\\begindata"));
    let mut assignments = Vec::new();
    let mut in_data = parse_all;
    let mut current_key: Option<String> = None;
    let mut current_value = String::new();
    let mut paren_balance = 0_i32;

    for line in text.lines() {
        let trimmed = line.trim();
        if trimmed.eq_ignore_ascii_case("\\begindata") {
            in_data = true;
            continue;
        }
        if trimmed.eq_ignore_ascii_case("\\begintext") {
            in_data = parse_all;
            current_key = None;
            current_value.clear();
            paren_balance = 0;
            continue;
        }
        if !in_data || trimmed.is_empty() {
            continue;
        }
        if let Some(key) = &current_key {
            current_value.push(' ');
            current_value.push_str(trimmed);
            paren_balance += spice_paren_balance_delta(trimmed);
            if paren_balance <= 0 {
                assignments.push(SpiceKernelAssignment {
                    key: key.clone(),
                    value: current_value.trim().to_owned(),
                });
                current_key = None;
                current_value.clear();
                paren_balance = 0;
            }
            continue;
        }
        let Some((key, value)) = trimmed.split_once('=') else {
            continue;
        };
        let key = key.trim().to_ascii_uppercase();
        let value = value.trim().to_owned();
        paren_balance = spice_paren_balance_delta(&value);
        if paren_balance > 0 {
            current_key = Some(key);
            current_value = value;
        } else {
            assignments.push(SpiceKernelAssignment { key, value });
        }
    }
    assignments
}

fn spice_paren_balance_delta(value: &str) -> i32 {
    value.chars().fold(0_i32, |balance, character| {
        balance
            + match character {
                '(' => 1,
                ')' => -1,
                _ => 0,
            }
    })
}

fn spice_frame_name_assignments(values: &BTreeMap<String, String>) -> BTreeMap<String, i32> {
    let mut names = BTreeMap::new();
    for (key, value) in values {
        let Some(frame_key) = key.strip_prefix("FRAME_") else {
            continue;
        };
        if let Some(id_token) = frame_key.strip_suffix("_NAME") {
            if let Ok(frame_id) = id_token.parse::<i32>()
                && let Some(name) = spice_scalar(value)
            {
                names.insert(name.to_ascii_uppercase(), frame_id);
            }
            continue;
        }
        if spice_direct_frame_id_key(frame_key)
            && let Ok(frame_id) = spice_scalar(value)
                .unwrap_or_else(|| value.trim().to_owned())
                .parse::<i32>()
        {
            names.insert(frame_key.to_ascii_uppercase(), frame_id);
        }
    }
    names
}

fn spice_direct_frame_id_key(frame_key: &str) -> bool {
    !frame_key.starts_with('-')
        && !frame_key
            .chars()
            .next()
            .is_some_and(|character| character.is_ascii_digit())
        && !frame_key.ends_with("_NAME")
        && !frame_key.ends_with("_CLASS")
        && !frame_key.ends_with("_CLASS_ID")
        && !frame_key.ends_with("_CENTER")
}

fn spice_frame_reference_to_id(
    reference: &str,
    frame_names: &BTreeMap<String, i32>,
) -> Option<i32> {
    let trimmed = reference.trim();
    if let Ok(frame_id) = trimmed.parse::<i32>() {
        return Some(frame_id);
    }
    let normalized = trimmed.to_ascii_uppercase();
    if let Some(frame_id) = frame_names.get(&normalized) {
        return Some(*frame_id);
    }
    spice_builtin_frame_name_to_id(&normalized)
}

fn spice_builtin_frame_name_to_id(name: &str) -> Option<i32> {
    let compact: String = name
        .chars()
        .filter(|character| !matches!(character, '-' | '_' | ' '))
        .collect();
    match compact.as_str() {
        "J2000" => Some(1),
        "B1950" => Some(2),
        "FK4" => Some(3),
        "DE118" => Some(4),
        "DE96" => Some(5),
        "DE102" => Some(6),
        "DE108" => Some(7),
        "DE111" => Some(8),
        "DE114" => Some(9),
        "DE122" => Some(10),
        "DE125" => Some(11),
        "DE130" => Some(12),
        "GALACTIC" => Some(13),
        "DE200" => Some(14),
        "DE202" => Some(15),
        "MARSIAU" => Some(16),
        "ECLIPJ2000" => Some(17),
        "ECLIPB1950" => Some(18),
        "DE140" => Some(19),
        "DE142" => Some(20),
        "DE143" => Some(21),
        _ => None,
    }
}

fn spice_scalar(value: &str) -> Option<String> {
    let trimmed = value.trim().trim_end_matches(',').trim();
    for quote in ['\'', '"'] {
        if let Some(rest) = trimmed.strip_prefix(quote)
            && let Some((scalar, _)) = rest.split_once(quote)
        {
            return Some(scalar.trim().to_owned());
        }
    }
    trimmed
        .trim_matches(|character| matches!(character, '(' | ')' | ','))
        .split_whitespace()
        .next()
        .map(str::to_owned)
}

fn spice_numeric_values(value: &str) -> Result<Vec<f64>, RunnerError> {
    let normalized = value.replace(['(', ')', ','], " ");
    let mut parsed = Vec::new();
    for token in normalized.split_whitespace() {
        let token = token
            .chars()
            .map(|character| match character {
                'D' => 'E',
                'd' => 'e',
                other => other,
            })
            .collect::<String>();
        let value = token
            .parse::<f64>()
            .map_err(|_| RunnerError::UnsupportedScenario {
                what: format!("SPICE TKFRAME matrix value `{token}` is not numeric"),
            })?;
        if !value.is_finite() {
            return Err(RunnerError::UnsupportedScenario {
                what: format!("SPICE TKFRAME matrix value `{token}` is not finite"),
            });
        }
        parsed.push(value);
    }
    Ok(parsed)
}

fn required_spice_numeric_values(
    values: &BTreeMap<String, String>,
    key: &str,
) -> Result<Vec<f64>, RunnerError> {
    let Some(value) = values.get(key) else {
        return Err(RunnerError::UnsupportedScenario {
            what: format!("SPICE {key} is required for TK frame definition"),
        });
    };
    spice_numeric_values(value)
}

fn spice_angle_unit_to_radians(units: &str) -> Option<f64> {
    match units.to_ascii_uppercase().as_str() {
        "RADIANS" | "RADIAN" => Some(1.0),
        "DEGREES" | "DEGREE" => Some(std::f64::consts::PI / 180.0),
        "ARCSECONDS" | "ARCSECOND" => Some(std::f64::consts::PI / (180.0 * 3_600.0)),
        _ => None,
    }
}

fn spice_axis_number(value: f64) -> Option<i32> {
    if !value.is_finite() {
        return None;
    }
    let rounded = value.round();
    if (value - rounded).abs() > 0.0 {
        return None;
    }
    match rounded as i32 {
        axis @ 1..=3 => Some(axis),
        _ => None,
    }
}

fn spice_euler_matrix(angles_rad: [f64; 3], axes: [i32; 3]) -> Matrix3<f64> {
    spice_coordinate_rotation_matrix(angles_rad[2], axes[2])
        * spice_coordinate_rotation_matrix(angles_rad[1], axes[1])
        * spice_coordinate_rotation_matrix(angles_rad[0], axes[0])
}

fn spice_coordinate_rotation_matrix(angle_rad: f64, axis: i32) -> Matrix3<f64> {
    let (sin, cos) = angle_rad.sin_cos();
    match axis {
        1 => Matrix3::new(1.0, 0.0, 0.0, 0.0, cos, sin, 0.0, -sin, cos),
        2 => Matrix3::new(cos, 0.0, -sin, 0.0, 1.0, 0.0, sin, 0.0, cos),
        3 => Matrix3::new(cos, sin, 0.0, -sin, cos, 0.0, 0.0, 0.0, 1.0),
        _ => Matrix3::identity(),
    }
}

fn spice_quaternion_to_matrix(q: [f64; 4]) -> Matrix3<f64> {
    let norm = q.iter().map(|value| value * value).sum::<f64>().sqrt();
    if norm == 0.0 || !norm.is_finite() {
        return Matrix3::identity();
    }
    let q0 = q[0] / norm;
    let q1 = q[1] / norm;
    let q2 = q[2] / norm;
    let q3 = q[3] / norm;
    Matrix3::new(
        1.0 - 2.0 * (q2 * q2 + q3 * q3),
        2.0 * (q1 * q2 - q0 * q3),
        2.0 * (q1 * q3 + q0 * q2),
        2.0 * (q1 * q2 + q0 * q3),
        1.0 - 2.0 * (q1 * q1 + q3 * q3),
        2.0 * (q2 * q3 - q0 * q1),
        2.0 * (q1 * q3 - q0 * q2),
        2.0 * (q2 * q3 + q0 * q1),
        1.0 - 2.0 * (q1 * q1 + q2 * q2),
    )
}

fn build_central_gravity(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
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
        "egm2008" => Ok(RuntimeCentralGravity::Egm2008(build_egm2008_gravity(
            document,
            resolved_files,
        )?)),
        "tesseral" => Ok(RuntimeCentralGravity::Tesseral(build_tesseral_gravity(
            document,
            resolved_files,
        )?)),
        other => Err(RunnerError::UnsupportedScenario {
            what: format!("environment.gravity_base = {other} is not wired"),
        }),
    }
}

/// Build the EGM2008 runtime gravity selected by the scenario.
///
/// Without `environment.egm2008_coefficients_file`, this preserves the legacy
/// pinned zonal-only degree-2 through degree-6 model. With a resolved ICGEM GFC
/// file and explicit degree/order, it builds the finite-difference Pines
/// transition model and evaluates it through the selected Earth-fixed frame.
pub(crate) fn build_egm2008_gravity(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<RuntimeEgm2008Gravity, RunnerError> {
    if document.environment.egm2008_coefficients_file.is_none() {
        return Ok(RuntimeEgm2008Gravity::Zonal(
            Egm2008ZonalGravity::wgs84_egm2008_zonal(),
        ));
    }
    let resolved = resolved_files
        .get("environment.egm2008_coefficients_file")
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.egm2008_coefficients_file was not resolved".to_owned(),
        })?;
    let gfc = std::str::from_utf8(resolved.bytes.as_slice()).map_err(|err| {
        RunnerError::UnsupportedScenario {
            what: format!("environment.egm2008_coefficients_file must be UTF-8 GFC text: {err}"),
        }
    })?;
    let finite_difference_step_m = document
        .environment
        .egm2008_finite_difference_step_m
        .unwrap_or(DEFAULT_PINES_FINITE_DIFFERENCE_STEP_M);
    let body_fixed =
        if let Some(tier_tag) = &document.environment.egm2008_tier {
            let tier = HarmonicSynthesisTier::from_egm2008_tag(tier_tag)?;
            FiniteDifferencePinesGravity::new_from_icgem_gfc_str_with_synthesis_tier(
                gfc,
                tier,
                finite_difference_step_m,
            )?
        } else {
            let degree = document.environment.egm2008_degree.ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: "environment.egm2008_degree missing for EGM2008 coefficient-file gravity"
                        .to_owned(),
                }
            })?;
            let order = document.environment.egm2008_order.ok_or_else(|| {
                RunnerError::UnsupportedScenario {
                    what: "environment.egm2008_order missing for EGM2008 coefficient-file gravity"
                        .to_owned(),
                }
            })?;
            FiniteDifferencePinesGravity::new_from_icgem_gfc_str(
                gfc,
                degree,
                order,
                finite_difference_step_m,
            )?
        };
    let frame = crate::frames::build_frame_context(document, resolved_files)?;
    Ok(RuntimeEgm2008Gravity::Pines(EarthFixedGravity::new(
        body_fixed, frame,
    )))
}

/// Build the current bounded degree-2 Earth-fixed tesseral gravity model.
///
/// The coefficient evaluator is intentionally still the low-degree transition
/// path from `openbmp-physics`; this runner hook only wires the existing model
/// through the selected frame context.
pub(crate) fn build_tesseral_gravity(
    document: &ScenarioDocument,
    resolved_files: &BTreeMap<String, ResolvedFile>,
) -> Result<EarthFixedGravity<TesseralGravity>, RunnerError> {
    let mu = document
        .environment
        .mu_m3_s2
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.mu_m3_s2 missing for tesseral gravity".to_owned(),
        })?;
    let r_e = document
        .environment
        .r_e_m
        .ok_or_else(|| RunnerError::UnsupportedScenario {
            what: "environment.r_e_m missing for tesseral gravity".to_owned(),
        })?;
    let scalar = |name: &'static str, value: Option<f64>| -> Result<f64, RunnerError> {
        value.ok_or_else(|| RunnerError::UnsupportedScenario {
            what: format!("environment.{name} missing for tesseral gravity"),
        })
    };
    let tide_system = TideSystem::from_tag(
        document
            .environment
            .tesseral_tide_system
            .as_deref()
            .unwrap_or("tide_free"),
    )?;
    let coefficients = DegreeTwoTesseralCoefficients::new(
        scalar("tesseral_c20", document.environment.tesseral_c20)?,
        scalar("tesseral_c21", document.environment.tesseral_c21)?,
        scalar("tesseral_s21", document.environment.tesseral_s21)?,
        scalar("tesseral_c22", document.environment.tesseral_c22)?,
        scalar("tesseral_s22", document.environment.tesseral_s22)?,
        tide_system,
    )?;
    let degree =
        document
            .environment
            .tesseral_degree
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "environment.tesseral_degree missing for tesseral gravity".to_owned(),
            })?;
    let order =
        document
            .environment
            .tesseral_order
            .ok_or_else(|| RunnerError::UnsupportedScenario {
                what: "environment.tesseral_order missing for tesseral gravity".to_owned(),
            })?;
    let body_fixed = TesseralGravity::new(mu, r_e, coefficients, degree, order)?;
    let frame = crate::frames::build_frame_context(document, resolved_files)?;
    Ok(EarthFixedGravity::new(body_fixed, frame))
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
    let observer = dtdb_observer_geometry(document)?;
    match scale.as_str() {
        "TDB" => Ok(jd),
        "TT" => TimeScaleBridge::tt_julian_date_to_tdb_erfa_approx(
            jd,
            0.0,
            observer.longitude_rad,
            observer.distance_spin_axis_km,
            observer.distance_north_equator_km,
        )
        .map_err(time_scale_bridge_error),
        "UTC" => {
            let Some(table) = resolved_leap_second_table(document, resolved_files)? else {
                if require_leap_seconds_for_utc {
                    return Err(RunnerError::UnsupportedScenario {
                        what: "epoch.scale = \"UTC\" requires resolved epoch.leap_second_table or a NAIF LSK in environment.ephemeris_meta_kernel for TDB ephemeris conversion".to_owned(),
                    });
                }
                return Ok(jd);
            };
            Ok(utc_to_tdb_julian_date(
                jd,
                &table,
                resolved_files,
                observer,
            )?)
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
pub(crate) struct LeapSecondTable {
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

    pub(crate) fn tai_minus_utc_s(&self, utc_julian_date: f64) -> Result<f64, RunnerError> {
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

pub(crate) fn resolved_leap_second_table(
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
    resolved_files: &BTreeMap<String, ResolvedFile>,
    observer: DtdbObserverGeometry,
) -> Result<f64, RunnerError> {
    let tai_minus_utc_s = leap_seconds.tai_minus_utc_s(utc_julian_date)?;
    let ut1_minus_utc_s = epoch_ut1_minus_utc_s(resolved_files, utc_julian_date)?.unwrap_or(0.0);
    let bridge =
        TimeScaleBridge::new(tai_minus_utc_s, ut1_minus_utc_s).map_err(time_scale_bridge_error)?;
    bridge
        .utc_julian_date_to_tdb_erfa_approx(
            utc_julian_date,
            observer.longitude_rad,
            observer.distance_spin_axis_km,
            observer.distance_north_equator_km,
        )
        .map_err(time_scale_bridge_error)
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct DtdbObserverGeometry {
    longitude_rad: f64,
    distance_spin_axis_km: f64,
    distance_north_equator_km: f64,
}

impl DtdbObserverGeometry {
    const GEOCENTRIC: Self = Self {
        longitude_rad: 0.0,
        distance_spin_axis_km: 0.0,
        distance_north_equator_km: 0.0,
    };
}

fn dtdb_observer_geometry(
    document: &ScenarioDocument,
) -> Result<DtdbObserverGeometry, RunnerError> {
    let Some(local_origin) = document
        .frames
        .as_ref()
        .and_then(|frames| frames.local_origin.as_ref())
    else {
        return Ok(DtdbObserverGeometry::GEOCENTRIC);
    };
    let origin = LocalGeodeticOrigin::new_degrees(
        local_origin.latitude_deg,
        local_origin.longitude_deg,
        local_origin.height_m,
    )
    .map_err(|err| RunnerError::Env(err.into()))?;
    let ecef = origin.to_ecef_position().vector;
    Ok(DtdbObserverGeometry {
        longitude_rad: origin.longitude_rad,
        distance_spin_axis_km: (ecef.x * ecef.x + ecef.y * ecef.y).sqrt() * 1.0e-3,
        distance_north_equator_km: ecef.z * 1.0e-3,
    })
}

fn epoch_ut1_minus_utc_s(
    resolved_files: &BTreeMap<String, ResolvedFile>,
    epoch_utc_julian_date: f64,
) -> Result<Option<f64>, RunnerError> {
    let Some(resolved) = resolved_files.get("epoch.eop") else {
        return Ok(None);
    };
    let table = crate::frames::parse_earth_orientation_table(resolved, epoch_utc_julian_date)?;
    Ok(Some(table.sample(SimTime::ZERO).ut1_minus_utc_s))
}

fn time_scale_bridge_error(err: FrameError) -> RunnerError {
    RunnerError::UnsupportedScenario {
        what: format!("epoch time-scale conversion failed: {err}"),
    }
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
#[allow(clippy::expect_used, clippy::panic, clippy::unwrap_used)]
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
        let tdb = utc_to_tdb_julian_date(
            utc,
            &table,
            &BTreeMap::new(),
            DtdbObserverGeometry::GEOCENTRIC,
        )
        .unwrap();
        let offset_s = (tdb - utc) * SECONDS_PER_DAY;
        assert!((offset_s - 69.183_950_5).abs() < 4.0e-5);
    }

    #[test]
    fn naif_lsk_converts_utc_epoch_to_tdb_axis() {
        let table = parse_leap_second_table(&resolved_file("naif0012.tls", NAIF_LSK))
            .expect("parse NAIF leap-second kernel");
        let utc = parse_iso8601_julian_date("2017-01-01T00:00:00Z").unwrap();
        let tdb = utc_to_tdb_julian_date(
            utc,
            &table,
            &BTreeMap::new(),
            DtdbObserverGeometry::GEOCENTRIC,
        )
        .unwrap();
        let offset_s = (tdb - utc) * SECONDS_PER_DAY;
        assert!((offset_s - 69.183_950_5).abs() < 4.0e-5);
    }

    #[test]
    fn local_origin_supplies_topocentric_dtdb_geometry() {
        let table = parse_leap_second_table(&resolved_file("leaps.toml", LEAP_SECOND_TABLE))
            .expect("parse leap-second table");
        let utc = parse_iso8601_julian_date("2017-01-01T00:00:00Z").unwrap();
        let scenario = Scenario::from_toml_str(&format!(
            "{SPK_THIRD_BODY_SCENARIO}\n\
             [frames]\n\
             profile = \"toy-fixed-earth\"\n\
             [frames.local_origin]\n\
             latitude_deg = 52.3\n\
             longitude_deg = -80.661\n\
             height_m = 0.0\n\
             source = \"synthetic dtdb unit-test origin\"\n"
        ))
        .unwrap();
        let observer = dtdb_observer_geometry(&scenario.document).unwrap();

        assert!(observer.longitude_rad.is_sign_negative());
        assert!(observer.distance_spin_axis_km > 3_800.0);
        assert!(observer.distance_north_equator_km > 5_000.0);

        let bridge = TimeScaleBridge::new(table.tai_minus_utc_s(utc).unwrap(), 0.0).unwrap();
        let tt = bridge.utc_julian_date_to_tt(utc).unwrap();
        let ut1_day_fraction = bridge.utc_julian_date_to_ut1(utc).unwrap().rem_euclid(1.0);
        let geocentric = TimeScaleBridge::tdb_minus_tt_erfa_approx_s(
            tt,
            ut1_day_fraction,
            DtdbObserverGeometry::GEOCENTRIC.longitude_rad,
            DtdbObserverGeometry::GEOCENTRIC.distance_spin_axis_km,
            DtdbObserverGeometry::GEOCENTRIC.distance_north_equator_km,
        )
        .unwrap();
        let topocentric = TimeScaleBridge::tdb_minus_tt_erfa_approx_s(
            tt,
            ut1_day_fraction,
            observer.longitude_rad,
            observer.distance_spin_axis_km,
            observer.distance_north_equator_km,
        )
        .unwrap();
        let delta_s = topocentric - geocentric;

        assert!(delta_s.abs() > 1.0e-10);
        assert!(delta_s.abs() < 3.0e-6);
    }

    #[test]
    fn epoch_eop_supplies_ut1_offset_for_dtdb_phase() {
        let scenario = Scenario::from_toml_str(SPK_THIRD_BODY_SCENARIO).unwrap();
        let epoch = parse_iso8601_julian_date("2017-01-01T00:00:00Z").unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "epoch.eop".to_owned(),
            resolved_file(
                "eop.toml",
                r#"
format = "openbmp-eop-v1"

[[samples]]
time_s = 0.0
ut1_minus_utc_s = 0.5912975
x_pole_arcsec = 0.080450
y_pole_arcsec = 0.263074
cip_offset_x_arcsec = -0.000019
cip_offset_y_arcsec = -0.000057
lod_s = 0.0010342
"#,
            ),
        );

        assert_eq!(
            epoch_ut1_minus_utc_s(&files, epoch).unwrap(),
            Some(0.591_297_5)
        );
        let geocentric_observer = dtdb_observer_geometry(&scenario.document).unwrap();
        assert!(geocentric_observer.distance_spin_axis_km.abs() <= f64::EPSILON);
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
    fn builds_third_body_gravity_with_tesseral_central_model() {
        let toml = SPK_THIRD_BODY_SCENARIO
            .replace(
                "gravity_base  = \"point_mass\"\nmu_m3_s2      = 3.986004418e14",
                "gravity_base  = \"tesseral\"\nmu_m3_s2      = 3.986004418e14\nr_e_m         = 6378137.0\ntesseral_degree = 2\ntesseral_order = 2\ntesseral_c20 = -1.082626683e-3\ntesseral_c21 = 2.0e-7\ntesseral_s21 = -3.0e-7\ntesseral_c22 = 1.0e-7\ntesseral_s22 = -2.0e-7",
            )
            .replace(
                "ephemeris     = \"spk\"\nephemeris_file = \"synthetic.bsp\"",
                "ephemeris     = \"low_precision_sun_moon\"",
            );
        let scenario = Scenario::from_toml_str(&toml).unwrap();
        let gravity = build_third_body_gravity(&scenario.document, &BTreeMap::new()).unwrap();

        match gravity.central() {
            RuntimeCentralGravity::Tesseral(model) => {
                let acceleration = model
                    .gravity_eci_m_s2(
                        Position3::new(6_900_000.0, 400_000.0, 300_000.0),
                        SimTime::ZERO,
                    )
                    .unwrap();
                assert!(acceleration.y.is_finite());
                assert!(acceleration.y.abs() > 1.0e-6);
            }
            other => panic!("expected tesseral central gravity, got {other:?}"),
        }
        match gravity.ephemeris() {
            RuntimeEphemeris::LowPrecisionSunMoon(_) => {}
            other => panic!("expected low-precision ephemeris, got {other:?}"),
        }
    }

    #[test]
    fn builds_file_backed_egm2008_named_tier_gravity() {
        let scenario = Scenario::from_toml_str(EGM2008_TIER_SCENARIO).unwrap();
        let mut files = BTreeMap::new();
        files.insert(
            "environment.egm2008_coefficients_file".to_owned(),
            resolved_file("egm70.gfc", SYNTHETIC_EGM2008_DEGREE70_GFC),
        );

        let gravity = build_egm2008_gravity(&scenario.document, &files).unwrap();
        match gravity {
            RuntimeEgm2008Gravity::Pines(model) => {
                let acceleration = model
                    .gravity_eci_m_s2(
                        Position3::new(6_900_000.0, 400_000.0, 300_000.0),
                        SimTime::ZERO,
                    )
                    .unwrap();
                assert!(acceleration.iter().all(|value| value.is_finite()));
            }
            other => panic!("expected file-backed Pines EGM2008 gravity, got {other:?}"),
        }

        files.insert(
            "environment.egm2008_coefficients_file".to_owned(),
            resolved_file(
                "egm10.gfc",
                &SYNTHETIC_EGM2008_DEGREE70_GFC.replace("max_degree 70", "max_degree 10"),
            ),
        );
        assert!(matches!(
            build_egm2008_gravity(&scenario.document, &files),
            Err(RunnerError::Env(
                openbmp_physics::PhysicsError::InvalidParameter { .. }
            ))
        ));
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
    fn builds_spk_third_body_ephemeris_from_meta_kernel_fixed_frame() {
        const MISSION_FRAME: i32 = 123_001;
        let sun = spk_meta_kernel_sun_position_for_frame(
            MISSION_FRAME,
            r#"
KPL/FK

\begindata
FRAME_MISSION_FRAME = 123001
TKFRAME_MISSION_FRAME_SPEC = 'MATRIX'
TKFRAME_MISSION_FRAME_RELATIVE = 'J2000'
TKFRAME_MISSION_FRAME_MATRIX = (  0
                                  1
                                  0
                                 -1
                                  0
                                  0
                                  0
                                  0
                                  1 )
\begintext
"#,
        );
        assert_rotated_fixed_frame_sun_position(sun);
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_meta_kernel_angle_frame() {
        const ANGLE_FRAME: i32 = 123_002;
        let sun = spk_meta_kernel_sun_position_for_frame(
            ANGLE_FRAME,
            r#"
KPL/FK

\begindata
FRAME_ANGLE_FRAME = 123002
TKFRAME_ANGLE_FRAME_SPEC = 'ANGLES'
TKFRAME_ANGLE_FRAME_RELATIVE = 'J2000'
TKFRAME_ANGLE_FRAME_ANGLES = ( -90.0, 0.0, 0.0 )
TKFRAME_ANGLE_FRAME_AXES = ( 3, 2, 1 )
TKFRAME_ANGLE_FRAME_UNITS = 'DEGREES'
\begintext
"#,
        );
        assert_rotated_fixed_frame_sun_position(sun);
    }

    #[test]
    fn builds_spk_third_body_ephemeris_from_meta_kernel_quaternion_frame() {
        const QUATERNION_FRAME: i32 = 123_003;
        let root_half = 0.5_f64.sqrt();
        let frame_kernel = format!(
            r#"
KPL/FK

\begindata
FRAME_QUATERNION_FRAME = 123003
TKFRAME_QUATERNION_FRAME_SPEC = 'QUATERNION'
TKFRAME_QUATERNION_FRAME_RELATIVE = 'J2000'
TKFRAME_QUATERNION_FRAME_Q = ( {root_half}, 0.0, 0.0, {root_half} )
\begintext
"#,
        );
        let sun = spk_meta_kernel_sun_position_for_frame(QUATERNION_FRAME, &frame_kernel);
        assert_rotated_fixed_frame_sun_position(sun);
    }

    fn assert_rotated_fixed_frame_sun_position(sun: nalgebra::Vector3<f64>) {
        let expected = nalgebra::Vector3::new(-4_700.0e3, 149_596_670.0e3, 300.0e3);
        assert!(
            (sun - expected).norm() < 1.0e-3,
            "rotated fixed-frame Sun position {sun:?} differs from {expected:?}"
        );
    }

    fn spk_meta_kernel_sun_position_for_frame(
        frame: i32,
        frame_kernel: &str,
    ) -> nalgebra::Vector3<f64> {
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
            resolved_file("mission.tf", frame_kernel),
        );
        files.insert(
            "environment.ephemeris_meta_kernel.files[1]".to_owned(),
            resolved_bytes("custom.bsp", synthetic_spk_with_sun_frame(frame)),
        );
        let gravity = build_third_body_gravity(&scenario.document, &files).unwrap();
        match gravity.ephemeris() {
            RuntimeEphemeris::Spk(spk) => {
                assert_eq!(spk.segment_count(), 4);
                spk.body_position_eci_m(CelestialBody::Sun, SimTime::ZERO)
                    .unwrap()
            }
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

    const EGM2008_TIER_SCENARIO: &str = r#"
openbmp.scenario = 3

[meta]
name = "egm2008-tier-test"
description = "File-backed EGM2008 named-tier wiring test."
validation = "validated-toy"
provenance = "synthetic runner unit test"

[time]
start_s = 0.0
stop_s = 1.0
dt_s = 1.0
seed = 1

[vehicle]
kind = "point_mass"
initial_position_eci_m = [6900000.0, 400000.0, 300000.0]
initial_velocity_eci_m_s = [0.0, 0.0, 0.0]

[vehicle.assembly]
id = "egm2008-tier-test"

[[vehicle.assembly.bodies]]
id          = "main"
geometry    = { kind = "reference", length_m = 1.0, area_m2 = 1.0 }
dry_mass_kg = 1.0

[environment]
frame_profile = "toy-fixed-earth"
gravity       = "egm2008"
egm2008_coefficients_file = "egm70.gfc"
egm2008_tier = "degree70"
atmosphere    = "none"
wind          = "none"

[forces]
models = ["gravity"]

[telemetry]
output.csv = "out/egm2008-tier-test.csv"

[validation]
require_finite_state = true
require_monotonic_time = true
"#;

    const SYNTHETIC_EGM2008_DEGREE70_GFC: &str = r#"
modelname synthetic_egm2008_degree70
earth_gravity_constant 398600441800000.0
radius 6378137.0
max_degree 70
norm fully_normalized
tide_system tide_free
end_of_head
gfc 0 0 1.0 0.0
gfc 2 0 -4.84165143790815e-4 0.0
gfc 2 2 1.0e-7 -2.0e-7
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
        frame: i32,
        position_km: [f64; 3],
    }

    fn synthetic_spk() -> Vec<u8> {
        synthetic_spk_with_sun_frame(1)
    }

    fn synthetic_spk_with_sun_frame(sun_frame: i32) -> Vec<u8> {
        const RECORD_BYTES: usize = 1_024;
        const WORDS_PER_RECORD: usize = 128;
        const SUMMARY_CONTROL_WORDS: usize = 3;
        const SUMMARY_WORDS: usize = 5;
        const J2000_FRAME: i32 = 1;
        let segments = [
            SyntheticSegment {
                target: 3,
                center: 0,
                frame: J2000_FRAME,
                position_km: [4_700.0, 1_200.0, -300.0],
            },
            SyntheticSegment {
                target: 399,
                center: 3,
                frame: J2000_FRAME,
                position_km: [0.0, 0.0, 0.0],
            },
            SyntheticSegment {
                target: 10,
                center: 0,
                frame: sun_frame,
                position_km: [149_597_870.0, 0.0, 0.0],
            },
            SyntheticSegment {
                target: 301,
                center: 3,
                frame: J2000_FRAME,
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
            write_i32(&mut bytes, offset + 24, segment.frame);
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
