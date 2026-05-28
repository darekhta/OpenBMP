//! Deterministic celestial ephemeris helpers.
//!
//! This module keeps file I/O out of `openbmp-physics`. It provides a
//! HAL-portable ephemeris trait, a deterministic built-in Sun/Moon
//! approximation, and a small SPK/BSP byte parser for runner-supplied
//! pinned JPL DE kernels.

use nalgebra::Vector3;
use openbmp_core::SimTime;

use crate::error::PhysicsError;

/// Astronomical unit in metres (IAU 2012 exact definition).
pub const ASTRONOMICAL_UNIT_M: f64 = 149_597_870_700.0;

/// Solar gravitational parameter, m^3/s^2.
///
/// Source: IAU 2015 nominal solar GM.
pub const SUN_MU_M3_S2: f64 = 1.327_124_4e20;

/// Lunar gravitational parameter, m^3/s^2.
///
/// Source: NASA/JPL DE-series canonical lunar GM, rounded to the
/// precision needed by this deterministic low-order model.
pub const MOON_MU_M3_S2: f64 = 4.904_869_5e12;

/// J2000 epoch as a Julian Date.
pub const J2000_JULIAN_DATE: f64 = 2_451_545.0;

const SECONDS_PER_DAY: f64 = 86_400.0;
const METRES_PER_KILOMETRE: f64 = 1_000.0;
const DAF_RECORD_BYTES: usize = 1_024;
const SPK_ND: i32 = 2;
const SPK_NI: i32 = 6;
const SPK_SUMMARY_WORDS: usize = 5;
const SPK_SUMMARY_CONTROL_WORDS: usize = 3;
const SPK_J2000_FRAME_ID: i32 = 1;
const NAIF_SOLAR_SYSTEM_BARYCENTER: i32 = 0;
const NAIF_EARTH: i32 = 399;
const NAIF_MOON: i32 = 301;
const NAIF_SUN: i32 = 10;

const DEG_TO_RAD: f64 = core::f64::consts::PI / 180.0;

/// Celestial body supported by built-in ephemeris and third-body
/// gravity models.
#[derive(Copy, Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum CelestialBody {
    /// The Sun.
    Sun,
    /// Earth's Moon.
    Moon,
}

impl CelestialBody {
    /// Canonical scenario label.
    #[must_use]
    pub const fn as_label(self) -> &'static str {
        match self {
            Self::Sun => "sun",
            Self::Moon => "moon",
        }
    }

    /// Gravitational parameter in m^3/s^2.
    #[must_use]
    pub const fn mu_m3_s2(self) -> f64 {
        match self {
            Self::Sun => SUN_MU_M3_S2,
            Self::Moon => MOON_MU_M3_S2,
        }
    }
}

/// Position provider for named celestial bodies.
///
/// Returned vectors are Earth-centered inertial, in metres, expressed
/// in the same ECI axes used by the scenario state.
pub trait EphemerisModel {
    /// Earth-centered inertial body position in metres.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the requested body is outside the
    /// model envelope or the computed state is non-finite.
    fn body_position_eci_m(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError>;
}

/// Low-precision deterministic Sun/Moon ephemeris.
///
/// The formulas are compact analytical approximations around J2000:
/// the Sun path follows the standard low-precision apparent solar
/// longitude / distance approximation, and the Moon path follows a
/// first-order longitude, latitude, and distance approximation. The
/// model is appropriate for deterministic perturbation studies; it is
/// not a substitute for JPL DE / SPICE navigation products.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct LowPrecisionSunMoonEphemeris {
    epoch_julian_date: f64,
}

impl LowPrecisionSunMoonEphemeris {
    /// Construct from the scenario epoch as Julian Date.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError::InvalidParameter`] if the epoch is not
    /// finite.
    pub fn new(epoch_julian_date: f64) -> Result<Self, PhysicsError> {
        if !epoch_julian_date.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ephemeris epoch Julian Date must be finite",
            });
        }
        Ok(Self { epoch_julian_date })
    }

    /// J2000 epoch.
    #[must_use]
    pub const fn j2000() -> Self {
        Self {
            epoch_julian_date: J2000_JULIAN_DATE,
        }
    }

    /// Configured epoch as Julian Date.
    #[must_use]
    pub const fn epoch_julian_date(&self) -> f64 {
        self.epoch_julian_date
    }

    fn days_since_j2000(self, time: SimTime) -> f64 {
        self.epoch_julian_date - J2000_JULIAN_DATE + time.as_seconds() / 86_400.0
    }
}

impl Default for LowPrecisionSunMoonEphemeris {
    fn default() -> Self {
        Self::j2000()
    }
}

impl EphemerisModel for LowPrecisionSunMoonEphemeris {
    fn body_position_eci_m(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let days = self.days_since_j2000(time);
        if !days.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ephemeris query time produced non-finite Julian offset",
            });
        }
        let position = match body {
            CelestialBody::Sun => low_precision_sun_eci_m(days),
            CelestialBody::Moon => low_precision_moon_eci_m(days),
        };
        if !position.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "low-precision ephemeris produced non-finite position",
            });
        }
        Ok(position)
    }
}

/// Deterministic SPK/BSP ephemeris backed by parsed DAF bytes.
///
/// This reader implements the subset required for JPL DE-style
/// planetary kernels used by third-body perturbations: SPK type 2
/// (Chebyshev position) and type 3 (Chebyshev position and velocity)
/// segments in the J2000 inertial frame. It computes geometric
/// positions only and does not implement light-time, aberration,
/// non-inertial frame transforms, or text-kernel loading.
#[derive(Clone, Debug, PartialEq)]
pub struct SpkEphemeris {
    epoch_tdb_julian_date: f64,
    segments: Vec<SpkSegment>,
}

impl SpkEphemeris {
    /// Parse a binary SPK/BSP kernel from bytes.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when the bytes are not a supported
    /// DAF/SPK file or no supported type 2/3 J2000 segments are found.
    pub fn from_bytes(epoch_tdb_julian_date: f64, bytes: &[u8]) -> Result<Self, PhysicsError> {
        Self::from_kernels(epoch_tdb_julian_date, [bytes])
    }

    /// Parse one or more binary SPK/BSP kernels from bytes.
    ///
    /// Segment precedence follows SPICE's practical load-order rule:
    /// later kernels in the iterator take priority over earlier
    /// kernels when overlapping target/coverage segments exist.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] when any byte slice is not a supported
    /// DAF/SPK file, no kernels are supplied, or no supported type 2/3
    /// J2000 segments are found across all kernels.
    pub fn from_kernels<'a, I>(epoch_tdb_julian_date: f64, kernels: I) -> Result<Self, PhysicsError>
    where
        I: IntoIterator<Item = &'a [u8]>,
    {
        if !epoch_tdb_julian_date.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK epoch Julian Date must be finite",
            });
        }
        let mut kernel_count = 0_usize;
        let mut segments = Vec::new();
        for bytes in kernels {
            kernel_count += 1;
            let daf = DafView::new(bytes)?;
            segments.extend(daf.spk_segments()?);
        }
        if kernel_count == 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK ephemeris requires at least one kernel",
            });
        }
        if segments.is_empty() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK kernel contains no supported type 2/3 J2000 segments",
            });
        }
        Ok(Self {
            epoch_tdb_julian_date,
            segments,
        })
    }

    /// Parsed segment count.
    #[must_use]
    pub fn segment_count(&self) -> usize {
        self.segments.len()
    }

    fn ephemeris_seconds(&self, time: SimTime) -> f64 {
        (self.epoch_tdb_julian_date - J2000_JULIAN_DATE) * SECONDS_PER_DAY + time.as_seconds()
    }

    fn body_position_relative_to_earth_km(
        &self,
        target: i32,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let et_s = self.ephemeris_seconds(time);
        if !et_s.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK query produced non-finite ephemeris seconds",
            });
        }
        self.position_between_km(target, NAIF_EARTH, et_s)
    }

    fn position_between_km(
        &self,
        target: i32,
        observer: i32,
        et_s: f64,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let (target_root, target_position) = self.position_to_root_km(target, et_s)?;
        let (observer_root, observer_position) = self.position_to_root_km(observer, et_s)?;
        if target_root != observer_root {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK target and observer do not share a common center",
            });
        }
        Ok(target_position - observer_position)
    }

    fn position_to_root_km(
        &self,
        body: i32,
        et_s: f64,
    ) -> Result<(i32, Vector3<f64>), PhysicsError> {
        let mut current = body;
        let mut position = Vector3::zeros();
        for _ in 0..16 {
            if current == NAIF_SOLAR_SYSTEM_BARYCENTER {
                return Ok((current, position));
            }
            let Some(segment) = self.select_segment(current, et_s) else {
                return Ok((current, position));
            };
            position += segment.position_km(et_s)?;
            current = segment.center;
        }
        Err(PhysicsError::InvalidParameter {
            reason: "SPK segment center chain is too deep",
        })
    }

    fn select_segment(&self, target: i32, et_s: f64) -> Option<&SpkSegment> {
        self.segments.iter().rev().find(|segment| {
            segment.target == target && segment.start_et_s <= et_s && et_s <= segment.stop_et_s
        })
    }
}

impl EphemerisModel for SpkEphemeris {
    fn body_position_eci_m(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<Vector3<f64>, PhysicsError> {
        let target = match body {
            CelestialBody::Sun => NAIF_SUN,
            CelestialBody::Moon => NAIF_MOON,
        };
        let position_km = self.body_position_relative_to_earth_km(target, time)?;
        let position_m = position_km * METRES_PER_KILOMETRE;
        if !position_m.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "SPK ephemeris produced non-finite position",
            });
        }
        Ok(position_m)
    }
}

#[derive(Copy, Clone, Debug, Eq, PartialEq)]
enum DafEndian {
    Little,
    Big,
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct SpkDescriptor {
    start_et_s: f64,
    stop_et_s: f64,
    target: i32,
    center: i32,
    frame: i32,
    data_type: i32,
    initial_address: i32,
    final_address: i32,
}

#[derive(Clone, Debug, PartialEq)]
struct SpkSegment {
    start_et_s: f64,
    stop_et_s: f64,
    target: i32,
    center: i32,
    data_type: i32,
    data: Vec<f64>,
}

impl SpkSegment {
    fn position_km(&self, et_s: f64) -> Result<Vector3<f64>, PhysicsError> {
        match self.data_type {
            2 => self.chebyshev_position_km(et_s, 3),
            3 => self.chebyshev_position_km(et_s, 6),
            _ => Err(PhysicsError::InvalidParameter {
                reason: "unsupported SPK data type",
            }),
        }
    }

    fn chebyshev_position_km(
        &self,
        et_s: f64,
        component_count: usize,
    ) -> Result<Vector3<f64>, PhysicsError> {
        if self.data.len() < 4 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK Chebyshev segment is too short",
            });
        }
        let directory = self.data.len() - 4;
        let init = self.data[directory];
        let intlen = self.data[directory + 1];
        let rsize_f = self.data[directory + 2];
        let n_f = self.data[directory + 3];
        if !init.is_finite()
            || !intlen.is_finite()
            || !rsize_f.is_finite()
            || !n_f.is_finite()
            || intlen <= 0.0
            || rsize_f < 5.0
            || n_f < 1.0
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK Chebyshev directory",
            });
        }
        let rsize = f64_to_usize(rsize_f)?;
        let record_count = f64_to_usize(n_f)?;
        if rsize * record_count + 4 != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK Chebyshev directory does not match segment length",
            });
        }
        let mut record_index = ((et_s - init) / intlen).floor();
        if record_index < 0.0 {
            record_index = 0.0;
        }
        let max_index = (record_count - 1) as f64;
        if record_index > max_index {
            record_index = max_index;
        }
        let record_index = record_index as usize;
        let record_start = record_index * rsize;
        let record = &self.data[record_start..record_start + rsize];
        let midpoint = record[0];
        let radius = record[1];
        if !midpoint.is_finite() || !radius.is_finite() || radius <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK Chebyshev record interval",
            });
        }
        let coeff_count = (rsize - 2) / component_count;
        if coeff_count == 0 || 2 + component_count * coeff_count != rsize {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK Chebyshev coefficient count",
            });
        }
        let tau = (et_s - midpoint) / radius;
        let x = evaluate_chebyshev(tau, &record[2..2 + coeff_count]);
        let y = evaluate_chebyshev(tau, &record[2 + coeff_count..2 + 2 * coeff_count]);
        let z = evaluate_chebyshev(tau, &record[2 + 2 * coeff_count..2 + 3 * coeff_count]);
        Ok(Vector3::new(x, y, z))
    }
}

struct DafView<'a> {
    bytes: &'a [u8],
    endian: DafEndian,
    forward_record: i32,
}

impl<'a> DafView<'a> {
    fn new(bytes: &'a [u8]) -> Result<Self, PhysicsError> {
        if bytes.len() < DAF_RECORD_BYTES {
            return Err(PhysicsError::InvalidParameter {
                reason: "DAF/SPK file is shorter than one record",
            });
        }
        if &bytes[0..8] != b"DAF/SPK " {
            return Err(PhysicsError::InvalidParameter {
                reason: "DAF/SPK file has invalid identification word",
            });
        }
        let format = ascii_trim(&bytes[88..96]);
        let endian = match format {
            "LTL-IEEE" => DafEndian::Little,
            "BIG-IEEE" => DafEndian::Big,
            _ => {
                return Err(PhysicsError::InvalidParameter {
                    reason: "DAF/SPK file has unsupported binary format",
                });
            }
        };
        let nd = read_i32_at(bytes, 8, endian)?;
        let ni = read_i32_at(bytes, 12, endian)?;
        if nd != SPK_ND || ni != SPK_NI {
            return Err(PhysicsError::InvalidParameter {
                reason: "DAF/SPK summary format is not ND=2, NI=6",
            });
        }
        let forward_record = read_i32_at(bytes, 76, endian)?;
        if forward_record <= 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "DAF/SPK initial summary record is invalid",
            });
        }
        Ok(Self {
            bytes,
            endian,
            forward_record,
        })
    }

    fn spk_segments(&self) -> Result<Vec<SpkSegment>, PhysicsError> {
        let mut segments = Vec::new();
        let mut record = self.forward_record;
        for _ in 0..1_024 {
            if record == 0 {
                break;
            }
            let record_offset = self.record_offset(record)?;
            let next = f64_to_i32(read_f64_at(self.bytes, record_offset, self.endian)?)?;
            let summary_count =
                f64_to_usize(read_f64_at(self.bytes, record_offset + 16, self.endian)?)?;
            for index in 0..summary_count {
                let summary_offset =
                    record_offset + (SPK_SUMMARY_CONTROL_WORDS + index * SPK_SUMMARY_WORDS) * 8;
                let descriptor = self.spk_descriptor(summary_offset)?;
                if descriptor.frame == SPK_J2000_FRAME_ID
                    && matches!(descriptor.data_type, 2 | 3)
                    && let Some(segment) = self.segment_from_descriptor(descriptor)?
                {
                    segments.push(segment);
                }
            }
            record = next;
        }
        Ok(segments)
    }

    fn spk_descriptor(&self, offset: usize) -> Result<SpkDescriptor, PhysicsError> {
        let start_et_s = read_f64_at(self.bytes, offset, self.endian)?;
        let stop_et_s = read_f64_at(self.bytes, offset + 8, self.endian)?;
        let target = read_i32_at(self.bytes, offset + 16, self.endian)?;
        let center = read_i32_at(self.bytes, offset + 20, self.endian)?;
        let frame = read_i32_at(self.bytes, offset + 24, self.endian)?;
        let data_type = read_i32_at(self.bytes, offset + 28, self.endian)?;
        let initial_address = read_i32_at(self.bytes, offset + 32, self.endian)?;
        let final_address = read_i32_at(self.bytes, offset + 36, self.endian)?;
        Ok(SpkDescriptor {
            start_et_s,
            stop_et_s,
            target,
            center,
            frame,
            data_type,
            initial_address,
            final_address,
        })
    }

    fn segment_from_descriptor(
        &self,
        descriptor: SpkDescriptor,
    ) -> Result<Option<SpkSegment>, PhysicsError> {
        if !descriptor.start_et_s.is_finite()
            || !descriptor.stop_et_s.is_finite()
            || descriptor.start_et_s > descriptor.stop_et_s
            || descriptor.initial_address <= 0
            || descriptor.final_address < descriptor.initial_address
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK segment descriptor",
            });
        }
        let start_word = usize::try_from(descriptor.initial_address - 1).map_err(|_| {
            PhysicsError::InvalidParameter {
                reason: "SPK segment initial address is invalid",
            }
        })?;
        let end_word = usize::try_from(descriptor.final_address).map_err(|_| {
            PhysicsError::InvalidParameter {
                reason: "SPK segment final address is invalid",
            }
        })?;
        let start = start_word
            .checked_mul(8)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK segment byte address overflow",
            })?;
        let end = end_word
            .checked_mul(8)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK segment byte address overflow",
            })?;
        if end > self.bytes.len() || start >= end {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK segment addresses are outside the file",
            });
        }
        let mut data = Vec::with_capacity(end_word - start_word);
        for offset in (start..end).step_by(8) {
            data.push(read_f64_at(self.bytes, offset, self.endian)?);
        }
        Ok(Some(SpkSegment {
            start_et_s: descriptor.start_et_s,
            stop_et_s: descriptor.stop_et_s,
            target: descriptor.target,
            center: descriptor.center,
            data_type: descriptor.data_type,
            data,
        }))
    }

    fn record_offset(&self, record: i32) -> Result<usize, PhysicsError> {
        let record_index =
            usize::try_from(record - 1).map_err(|_| PhysicsError::InvalidParameter {
                reason: "DAF record number is invalid",
            })?;
        let offset =
            record_index
                .checked_mul(DAF_RECORD_BYTES)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "DAF record offset overflow",
                })?;
        if offset + DAF_RECORD_BYTES > self.bytes.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "DAF record is outside the file",
            });
        }
        Ok(offset)
    }
}

fn wrap_degrees(degrees: f64) -> f64 {
    degrees.rem_euclid(360.0)
}

fn sin_deg(degrees: f64) -> f64 {
    (wrap_degrees(degrees) * DEG_TO_RAD).sin()
}

fn cos_deg(degrees: f64) -> f64 {
    (wrap_degrees(degrees) * DEG_TO_RAD).cos()
}

fn low_precision_sun_eci_m(days_since_j2000: f64) -> Vector3<f64> {
    let mean_longitude_deg = wrap_degrees(280.460 + 0.985_647_4 * days_since_j2000);
    let mean_anomaly_deg = wrap_degrees(357.528 + 0.985_600_3 * days_since_j2000);
    let ecliptic_longitude_deg = mean_longitude_deg
        + 1.915 * sin_deg(mean_anomaly_deg)
        + 0.020 * sin_deg(2.0 * mean_anomaly_deg);
    let obliquity_deg = 23.439 - 0.000_000_4 * days_since_j2000;
    let radius_au = 1.000_14
        - 0.016_71 * cos_deg(mean_anomaly_deg)
        - 0.000_14 * cos_deg(2.0 * mean_anomaly_deg);
    let radius_m = radius_au * ASTRONOMICAL_UNIT_M;
    let cos_lambda = cos_deg(ecliptic_longitude_deg);
    let sin_lambda = sin_deg(ecliptic_longitude_deg);
    let cos_eps = cos_deg(obliquity_deg);
    let sin_eps = sin_deg(obliquity_deg);
    Vector3::new(
        radius_m * cos_lambda,
        radius_m * cos_eps * sin_lambda,
        radius_m * sin_eps * sin_lambda,
    )
}

fn low_precision_moon_eci_m(days_since_j2000: f64) -> Vector3<f64> {
    let mean_longitude_deg = wrap_degrees(218.316 + 13.176_396 * days_since_j2000);
    let mean_anomaly_deg = wrap_degrees(134.963 + 13.064_993 * days_since_j2000);
    let argument_of_latitude_deg = wrap_degrees(93.272 + 13.229_350 * days_since_j2000);
    let ecliptic_longitude_deg = mean_longitude_deg + 6.289 * sin_deg(mean_anomaly_deg);
    let ecliptic_latitude_deg = 5.128 * sin_deg(argument_of_latitude_deg);
    let radius_m = (385_001.0 - 20_905.0 * cos_deg(mean_anomaly_deg)) * 1_000.0;
    let obliquity_deg = 23.439 - 0.000_000_4 * days_since_j2000;

    let cos_lambda = cos_deg(ecliptic_longitude_deg);
    let sin_lambda = sin_deg(ecliptic_longitude_deg);
    let cos_beta = cos_deg(ecliptic_latitude_deg);
    let sin_beta = sin_deg(ecliptic_latitude_deg);
    let cos_eps = cos_deg(obliquity_deg);
    let sin_eps = sin_deg(obliquity_deg);

    Vector3::new(
        radius_m * cos_beta * cos_lambda,
        radius_m * (cos_beta * sin_lambda * cos_eps - sin_beta * sin_eps),
        radius_m * (cos_beta * sin_lambda * sin_eps + sin_beta * cos_eps),
    )
}

fn evaluate_chebyshev(tau: f64, coefficients: &[f64]) -> f64 {
    if coefficients.is_empty() {
        return 0.0;
    }
    let mut sum = coefficients[0];
    if coefficients.len() == 1 {
        return sum;
    }
    let mut t_prev = 1.0;
    let mut t_curr = tau;
    sum += coefficients[1] * t_curr;
    for coefficient in &coefficients[2..] {
        let t_next = 2.0 * tau * t_curr - t_prev;
        sum += coefficient * t_next;
        t_prev = t_curr;
        t_curr = t_next;
    }
    sum
}

fn read_i32_at(bytes: &[u8], offset: usize, endian: DafEndian) -> Result<i32, PhysicsError> {
    let slice = bytes
        .get(offset..offset + 4)
        .ok_or(PhysicsError::InvalidParameter {
            reason: "DAF integer read is outside the file",
        })?;
    let raw = [slice[0], slice[1], slice[2], slice[3]];
    Ok(match endian {
        DafEndian::Little => i32::from_le_bytes(raw),
        DafEndian::Big => i32::from_be_bytes(raw),
    })
}

fn read_f64_at(bytes: &[u8], offset: usize, endian: DafEndian) -> Result<f64, PhysicsError> {
    let slice = bytes
        .get(offset..offset + 8)
        .ok_or(PhysicsError::InvalidParameter {
            reason: "DAF double read is outside the file",
        })?;
    let raw = [
        slice[0], slice[1], slice[2], slice[3], slice[4], slice[5], slice[6], slice[7],
    ];
    let value = match endian {
        DafEndian::Little => f64::from_le_bytes(raw),
        DafEndian::Big => f64::from_be_bytes(raw),
    };
    if !value.is_finite() {
        return Err(PhysicsError::InvalidParameter {
            reason: "DAF double value is not finite",
        });
    }
    Ok(value)
}

fn ascii_trim(bytes: &[u8]) -> &str {
    std::str::from_utf8(bytes)
        .unwrap_or("")
        .trim_matches(char::from(0))
        .trim()
}

fn f64_to_i32(value: f64) -> Result<i32, PhysicsError> {
    if !value.is_finite()
        || value.fract() != 0.0
        || value < f64::from(i32::MIN)
        || value > f64::from(i32::MAX)
    {
        return Err(PhysicsError::InvalidParameter {
            reason: "DAF numeric integer value is invalid",
        });
    }
    Ok(value as i32)
}

fn f64_to_usize(value: f64) -> Result<usize, PhysicsError> {
    if !value.is_finite() || value.fract() != 0.0 || value < 0.0 || value > usize::MAX as f64 {
        return Err(PhysicsError::InvalidParameter {
            reason: "DAF numeric usize value is invalid",
        });
    }
    Ok(value as usize)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;

    const DAF_DOUBLE_WORDS_PER_RECORD: usize = 128;

    #[test]
    fn low_precision_sun_distance_is_near_one_au() {
        let ephemeris = LowPrecisionSunMoonEphemeris::j2000();
        let sun = ephemeris
            .body_position_eci_m(CelestialBody::Sun, SimTime::ZERO)
            .unwrap();
        let radius_au = sun.norm() / ASTRONOMICAL_UNIT_M;
        assert!((0.98..1.02).contains(&radius_au));
    }

    #[test]
    fn low_precision_moon_distance_is_lunar_scale() {
        let ephemeris = LowPrecisionSunMoonEphemeris::j2000();
        let moon = ephemeris
            .body_position_eci_m(CelestialBody::Moon, SimTime::ZERO)
            .unwrap();
        let radius_km = moon.norm() / 1_000.0;
        assert!((350_000.0..410_000.0).contains(&radius_km));
    }

    #[test]
    fn spk_ephemeris_reads_synthetic_type2_sun_and_moon() {
        let bytes = synthetic_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        assert_eq!(ephemeris.segment_count(), 4);
        let sun = ephemeris
            .body_position_eci_m(CelestialBody::Sun, SimTime::ZERO)
            .unwrap();
        let moon = ephemeris
            .body_position_eci_m(CelestialBody::Moon, SimTime::ZERO)
            .unwrap();
        assert_eq!(
            sun,
            Vector3::new(149_597_870.0e3 - 4_700.0e3, -1_200.0e3, 300.0e3)
        );
        assert_eq!(moon, Vector3::new(384_400.0e3, 0.0, 0.0));
    }

    #[test]
    fn spk_ephemeris_combines_kernels_with_later_precedence() {
        let first = synthetic_spk_with_sun_x_km(149_597_870.0);
        let second = synthetic_spk_with_sun_x_km(149_597_880.0);
        let ephemeris =
            SpkEphemeris::from_kernels(J2000_JULIAN_DATE, [&first[..], &second[..]]).unwrap();
        assert_eq!(ephemeris.segment_count(), 8);
        let sun = ephemeris
            .body_position_eci_m(CelestialBody::Sun, SimTime::ZERO)
            .unwrap();
        assert_eq!(
            sun,
            Vector3::new(149_597_880.0e3 - 4_700.0e3, -1_200.0e3, 300.0e3)
        );
    }

    #[test]
    fn spk_ephemeris_rejects_invalid_file_id() {
        let mut bytes = synthetic_spk();
        bytes[0] = b'X';
        let err = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap_err();
        assert!(matches!(err, PhysicsError::InvalidParameter { .. }));
    }

    #[test]
    fn spk_ephemeris_reports_out_of_coverage_query() {
        let bytes = synthetic_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let err = ephemeris
            .body_position_eci_m(CelestialBody::Sun, SimTime::from_seconds(20.0))
            .unwrap_err();
        assert!(matches!(err, PhysicsError::OutOfEnvelope { .. }));
    }

    #[derive(Copy, Clone)]
    struct SyntheticSegment {
        target: i32,
        center: i32,
        position_km: [f64; 3],
    }

    fn synthetic_spk() -> Vec<u8> {
        synthetic_spk_with_sun_x_km(149_597_870.0)
    }

    fn synthetic_spk_with_sun_x_km(sun_x_km: f64) -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: 3,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                position_km: [4_700.0, 1_200.0, -300.0],
            },
            SyntheticSegment {
                target: NAIF_EARTH,
                center: 3,
                position_km: [0.0, 0.0, 0.0],
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                position_km: [sun_x_km, 0.0, 0.0],
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: 3,
                position_km: [384_400.0, 0.0, 0.0],
            },
        ];
        let record_count = 4 + segments.len();
        let mut bytes = vec![0_u8; record_count * DAF_RECORD_BYTES];
        write_file_record(&mut bytes);
        write_summary_record(&mut bytes, &segments);
        for (index, segment) in segments.iter().enumerate() {
            let address = data_address(index);
            let data = type2_constant_segment(segment.position_km);
            write_f64_data(&mut bytes, address, &data);
        }
        bytes
    }

    fn write_file_record(bytes: &mut [u8]) {
        bytes[0..8].copy_from_slice(b"DAF/SPK ");
        write_i32(bytes, 8, SPK_ND);
        write_i32(bytes, 12, SPK_NI);
        bytes[16..28].copy_from_slice(b"OPENBMP SPK ");
        write_i32(bytes, 76, 2);
        write_i32(bytes, 80, 2);
        write_i32(bytes, 84, 1);
        bytes[88..96].copy_from_slice(b"LTL-IEEE");
    }

    fn write_summary_record(bytes: &mut [u8], segments: &[SyntheticSegment]) {
        let record_offset = DAF_RECORD_BYTES;
        write_f64(bytes, record_offset, 0.0);
        write_f64(bytes, record_offset + 8, 0.0);
        write_f64(bytes, record_offset + 16, segments.len() as f64);
        for (index, segment) in segments.iter().enumerate() {
            let offset =
                record_offset + (SPK_SUMMARY_CONTROL_WORDS + index * SPK_SUMMARY_WORDS) * 8;
            write_f64(bytes, offset, -10.0);
            write_f64(bytes, offset + 8, 10.0);
            write_i32(bytes, offset + 16, segment.target);
            write_i32(bytes, offset + 20, segment.center);
            write_i32(bytes, offset + 24, SPK_J2000_FRAME_ID);
            write_i32(bytes, offset + 28, 2);
            write_i32(bytes, offset + 32, data_address(index));
            write_i32(bytes, offset + 36, data_address(index) + 8);
        }
    }

    fn data_address(index: usize) -> i32 {
        (DAF_DOUBLE_WORDS_PER_RECORD * (3 + index) + 1) as i32
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

    fn write_f64_data(bytes: &mut [u8], start_address: i32, data: &[f64]) {
        for (index, value) in data.iter().enumerate() {
            let offset = ((start_address as usize - 1) + index) * 8;
            write_f64(bytes, offset, *value);
        }
    }

    fn write_i32(bytes: &mut [u8], offset: usize, value: i32) {
        bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
    }

    fn write_f64(bytes: &mut [u8], offset: usize, value: f64) {
        bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
    }
}
