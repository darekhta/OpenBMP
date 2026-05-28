//! Deterministic celestial ephemeris helpers.
//!
//! This module keeps file I/O out of `openbmp-physics`. It provides a
//! HAL-portable ephemeris trait, a deterministic built-in Sun/Moon
//! approximation, and a small SPK/BSP byte parser for runner-supplied
//! pinned JPL DE and mission kernels.

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
const SPK_ECLIPJ2000_FRAME_ID: i32 = 17;
const NAIF_SOLAR_SYSTEM_BARYCENTER: i32 = 0;
const NAIF_EARTH: i32 = 399;
const NAIF_MOON: i32 = 301;
const NAIF_SUN: i32 = 10;
const J2000_ECLIPTIC_OBLIQUITY_RAD: f64 = 84_381.448 * DEG_TO_RAD / 3_600.0;

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

/// Earth-centered inertial celestial body state.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct EphemerisState {
    /// Earth-centered inertial position in metres.
    pub position_eci_m: Vector3<f64>,
    /// Earth-centered inertial velocity in metres per second.
    pub velocity_eci_m_s: Vector3<f64>,
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

    /// Earth-centered inertial body state in metres and metres per
    /// second.
    ///
    /// The default implementation uses a one-second finite difference
    /// around [`Self::body_position_eci_m`]. Higher-fidelity ephemeris
    /// backends should override this when the source supplies velocity
    /// directly or supports an analytic derivative.
    ///
    /// # Errors
    ///
    /// Returns [`PhysicsError`] if the position query or derived state
    /// is outside the model envelope or non-finite.
    fn body_state_eci_m_s(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<EphemerisState, PhysicsError> {
        let position = self.body_position_eci_m(body, time)?;
        let t_s = time.as_seconds();
        if !t_s.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "ephemeris state query time must be finite",
            });
        }
        let velocity = if t_s >= 1.0 {
            let before = self.body_position_eci_m(body, SimTime::from_seconds(t_s - 1.0))?;
            let after = self.body_position_eci_m(body, SimTime::from_seconds(t_s + 1.0))?;
            (after - before) * 0.5
        } else {
            let after = self.body_position_eci_m(body, SimTime::from_seconds(t_s + 1.0))?;
            after - position
        };
        let state = EphemerisState {
            position_eci_m: position,
            velocity_eci_m_s: velocity,
        };
        if !state.position_eci_m.iter().all(|v| v.is_finite())
            || !state.velocity_eci_m_s.iter().all(|v| v.is_finite())
        {
            return Err(PhysicsError::NonFinite {
                reason: "ephemeris state produced non-finite components",
            });
        }
        Ok(state)
    }
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
/// planetary and mission kernels used by third-body perturbations:
/// SPK type 2 (Chebyshev position), type 3 (Chebyshev position and
/// velocity), type 5 (two-body propagation between discrete states),
/// type 8/9 (equal/unequal-time Lagrange state
/// interpolation), type 12/13 (equal/unequal-time Hermite state
/// interpolation), type 14 (generic non-uniform Chebyshev position and
/// velocity), type 15 (precessing conic propagation), type 18
/// (ESOC/DDID Hermite/Lagrange interpolation), type 19 (ESOC/DDID
/// piecewise interpolation), and type 20 (Chebyshev velocity) segments
/// in the J2000 inertial frame. It also accepts the
/// built-in SPICE
/// `ECLIPJ2000` inertial frame and rotates those segment states into
/// J2000. It computes geometric states and does not implement
/// light-time, aberration, non-inertial frame chains, or generic
/// text-kernel loading.
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
    /// DAF/SPK file or no supported type 2/3/5/8/9/12/13/14/15/18/19/20 J2000
    /// segments are found.
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
    /// DAF/SPK file, no kernels are supplied, or no supported type
    /// 2/3/5/8/9/12/13/14/15/18/19/20 J2000 segments are found across all kernels.
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
                reason: "SPK kernel contains no supported type 2/3/5/8/9/12/13/14/15/18/19/20 J2000 segments",
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

    fn body_state_relative_to_earth_km_s(
        &self,
        target: i32,
        time: SimTime,
    ) -> Result<SpkStateKmS, PhysicsError> {
        let et_s = self.ephemeris_seconds(time);
        if !et_s.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK query produced non-finite ephemeris seconds",
            });
        }
        self.state_between_km_s(target, NAIF_EARTH, et_s)
    }

    fn state_between_km_s(
        &self,
        target: i32,
        observer: i32,
        et_s: f64,
    ) -> Result<SpkStateKmS, PhysicsError> {
        let (target_root, target_state) = self.state_to_root_km_s(target, et_s)?;
        let (observer_root, observer_state) = self.state_to_root_km_s(observer, et_s)?;
        if target_root != observer_root {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK target and observer do not share a common center",
            });
        }
        Ok(SpkStateKmS {
            position_km: target_state.position_km - observer_state.position_km,
            velocity_km_s: target_state.velocity_km_s - observer_state.velocity_km_s,
        })
    }

    fn state_to_root_km_s(&self, body: i32, et_s: f64) -> Result<(i32, SpkStateKmS), PhysicsError> {
        let mut current = body;
        let mut state = SpkStateKmS::zero();
        for _ in 0..16 {
            if current == NAIF_SOLAR_SYSTEM_BARYCENTER {
                return Ok((current, state));
            }
            let Some(segment) = self.select_segment(current, et_s) else {
                return Ok((current, state));
            };
            state += segment.state_km_s(et_s)?;
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
        let position_km = self
            .body_state_relative_to_earth_km_s(target, time)?
            .position_km;
        let position_m = position_km * METRES_PER_KILOMETRE;
        if !position_m.iter().all(|v| v.is_finite()) {
            return Err(PhysicsError::NonFinite {
                reason: "SPK ephemeris produced non-finite position",
            });
        }
        Ok(position_m)
    }

    fn body_state_eci_m_s(
        &self,
        body: CelestialBody,
        time: SimTime,
    ) -> Result<EphemerisState, PhysicsError> {
        let target = match body {
            CelestialBody::Sun => NAIF_SUN,
            CelestialBody::Moon => NAIF_MOON,
        };
        let state_km_s = self.body_state_relative_to_earth_km_s(target, time)?;
        let state = EphemerisState {
            position_eci_m: state_km_s.position_km * METRES_PER_KILOMETRE,
            velocity_eci_m_s: state_km_s.velocity_km_s * METRES_PER_KILOMETRE,
        };
        if !state.position_eci_m.iter().all(|v| v.is_finite())
            || !state.velocity_eci_m_s.iter().all(|v| v.is_finite())
        {
            return Err(PhysicsError::NonFinite {
                reason: "SPK ephemeris produced non-finite state",
            });
        }
        Ok(state)
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
    frame: i32,
    data_type: i32,
    data: Vec<f64>,
}

#[derive(Copy, Clone, Debug, PartialEq)]
struct SpkStateKmS {
    position_km: Vector3<f64>,
    velocity_km_s: Vector3<f64>,
}

impl SpkStateKmS {
    fn zero() -> Self {
        Self {
            position_km: Vector3::zeros(),
            velocity_km_s: Vector3::zeros(),
        }
    }
}

impl std::ops::AddAssign for SpkStateKmS {
    fn add_assign(&mut self, rhs: Self) {
        self.position_km += rhs.position_km;
        self.velocity_km_s += rhs.velocity_km_s;
    }
}

struct ChebyshevRecordView<'a> {
    record: &'a [f64],
    tau: f64,
    radius_s: f64,
    coeff_count: usize,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
struct GenericSegmentMetadata {
    conbas: usize,
    ncon: usize,
    rdrbas: usize,
    nrdr: usize,
    rdrtyp: usize,
    refbas: usize,
    nref: usize,
    pdrbas: usize,
    npdr: usize,
    pdrtyp: usize,
    pktbas: usize,
    npkt: usize,
    rsvbas: usize,
    nrsv: usize,
    pktsz: usize,
    pktoff: usize,
    nmeta: usize,
}

impl GenericSegmentMetadata {
    fn from_data(data: &[f64]) -> Result<Self, PhysicsError> {
        if data.len() < 18 {
            return Err(PhysicsError::InvalidParameter {
                reason: "generic SPK segment is too short",
            });
        }
        let nmeta = f64_to_usize(*data.last().ok_or(PhysicsError::InvalidParameter {
            reason: "generic SPK segment metadata is missing",
        })?)?;
        if nmeta < 17 || nmeta > data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "generic SPK segment metadata size is invalid",
            });
        }
        let metadata_start = data.len() - nmeta;
        let mut metadata = [0_usize; 17];
        for (index, value) in data[metadata_start..metadata_start + 17].iter().enumerate() {
            metadata[index] = f64_to_usize(*value)?;
        }
        if metadata[16] < 17 {
            return Err(PhysicsError::InvalidParameter {
                reason: "generic SPK segment metadata count is invalid",
            });
        }
        Ok(Self {
            conbas: metadata[0],
            ncon: metadata[1],
            rdrbas: metadata[2],
            nrdr: metadata[3],
            rdrtyp: metadata[4],
            refbas: metadata[5],
            nref: metadata[6],
            pdrbas: metadata[7],
            npdr: metadata[8],
            pdrtyp: metadata[9],
            pktbas: metadata[10],
            npkt: metadata[11],
            rsvbas: metadata[12],
            nrsv: metadata[13],
            pktsz: metadata[14],
            pktoff: metadata[15],
            nmeta,
        })
    }

    fn partition_range(
        &self,
        base: usize,
        count: usize,
        data_len: usize,
        reason: &'static str,
    ) -> Result<std::ops::Range<usize>, PhysicsError> {
        let metadata_start = data_len
            .checked_sub(self.nmeta)
            .ok_or(PhysicsError::InvalidParameter { reason })?;
        let end = base
            .checked_add(count)
            .ok_or(PhysicsError::InvalidParameter { reason })?;
        if end > metadata_start {
            return Err(PhysicsError::InvalidParameter { reason });
        }
        Ok(base..end)
    }
}

impl SpkSegment {
    fn state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        let state = self.raw_state_km_s(et_s)?;
        spk_frame_state_to_j2000_km_s(self.frame, state)
    }

    fn raw_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        match self.data_type {
            2 => {
                let record = self.chebyshev_record(et_s, 3)?;
                Ok(SpkStateKmS {
                    position_km: chebyshev_vector(record.record, record.coeff_count, 0, record.tau),
                    velocity_km_s: chebyshev_derivative_vector(
                        record.record,
                        record.coeff_count,
                        0,
                        record.radius_s,
                        record.tau,
                    ),
                })
            }
            3 => {
                let record = self.chebyshev_record(et_s, 6)?;
                Ok(SpkStateKmS {
                    position_km: chebyshev_vector(record.record, record.coeff_count, 0, record.tau),
                    velocity_km_s: chebyshev_vector(
                        record.record,
                        record.coeff_count,
                        3,
                        record.tau,
                    ),
                })
            }
            5 => self.two_body_discrete_state_km_s(et_s),
            8 => self.equal_step_lagrange_state_km_s(et_s),
            9 => self.unequal_step_lagrange_state_km_s(et_s),
            12 => self.equal_step_hermite_state_km_s(et_s),
            13 => self.unequal_step_hermite_state_km_s(et_s),
            14 => self.generic_chebyshev_state_km_s(et_s),
            15 => self.precessing_conic_state_km_s(et_s),
            18 => self.esoc_ddid_state_km_s(et_s),
            19 => self.esoc_ddid_piecewise_state_km_s(et_s),
            20 => self.chebyshev_velocity_state_km_s(et_s),
            _ => Err(PhysicsError::InvalidParameter {
                reason: "unsupported SPK data type",
            }),
        }
    }

    fn two_body_discrete_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 9 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 5 segment is too short",
            });
        }
        let n = f64_to_usize(self.data[self.data.len() - 1])?;
        let gm_km3_s2 = self.data[self.data.len() - 2];
        if n == 0 || !gm_km3_s2.is_finite() || gm_km3_s2 <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 5 directory",
            });
        }
        let directory_count = n / 100;
        let expected_len = 6_usize
            .checked_mul(n)
            .and_then(|state_len| state_len.checked_add(n))
            .and_then(|base| base.checked_add(directory_count))
            .and_then(|base| base.checked_add(2))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 5 segment length overflow",
            })?;
        if expected_len != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 5 segment length does not match directory",
            });
        }

        let states = &self.data[..6 * n];
        let epochs = &self.data[6 * n..7 * n];
        validate_strictly_increasing_epochs(epochs, "SPK type 5 epochs are invalid")?;

        let insertion = epochs.partition_point(|epoch| *epoch < et_s);
        let (first_index, second_index) = if insertion == 0 {
            (0, 0)
        } else if insertion >= n {
            (n - 1, n - 1)
        } else {
            (insertion - 1, insertion)
        };
        type5_two_body_blend_state(
            &states[first_index * 6..first_index * 6 + 6],
            epochs[first_index],
            &states[second_index * 6..second_index * 6 + 6],
            epochs[second_index],
            gm_km3_s2,
            et_s,
        )
    }

    fn equal_step_lagrange_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 10 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 8 segment is too short",
            });
        }
        let n = f64_to_usize(self.data[self.data.len() - 1])?;
        let degree = f64_to_usize(self.data[self.data.len() - 2])?;
        let step_s = self.data[self.data.len() - 3];
        let first_epoch_s = self.data[self.data.len() - 4];
        if !first_epoch_s.is_finite() || !step_s.is_finite() || step_s <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 8 epoch directory",
            });
        }
        if degree == 0 || n <= degree {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 8 interpolation degree",
            });
        }
        let expected_len = 6_usize
            .checked_mul(n)
            .and_then(|state_len| state_len.checked_add(4))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 8 segment length overflow",
            })?;
        if expected_len != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 8 segment length does not match state count",
            });
        }
        let last_epoch_s = first_epoch_s + step_s * (n - 1) as f64;
        if !last_epoch_s.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 8 epoch coverage is invalid",
            });
        }
        if et_s < first_epoch_s || et_s > last_epoch_s {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 8 query is outside state epoch coverage",
            });
        }

        let window_size = degree + 1;
        let start = equal_step_window_start(first_epoch_s, step_s, n, et_s, window_size);
        let mut state = [0.0_f64; 6];
        for offset in 0..window_size {
            let source_index = start + offset;
            let basis =
                equal_step_lagrange_basis(first_epoch_s, step_s, start, window_size, offset, et_s)?;
            for component in 0..6 {
                state[component] += basis * self.data[source_index * 6 + component];
            }
        }
        finite_state_from_components(state, "SPK type 8 Lagrange interpolation")
    }

    fn unequal_step_lagrange_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 9 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 9 segment is too short",
            });
        }
        let n = f64_to_usize(self.data[self.data.len() - 1])?;
        let degree = f64_to_usize(self.data[self.data.len() - 2])?;
        if n == 0 || n <= degree {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 9 interpolation degree",
            });
        }
        let window_size = degree + 1;
        let directory_count = (n - 1) / 100;
        let expected_len = 6_usize
            .checked_mul(n)
            .and_then(|state_len| state_len.checked_add(n))
            .and_then(|base| base.checked_add(directory_count))
            .and_then(|base| base.checked_add(2))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 9 segment length overflow",
            })?;
        if expected_len != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 9 segment length does not match directory",
            });
        }

        let states = &self.data[..6 * n];
        let epochs = &self.data[6 * n..7 * n];
        validate_strictly_increasing_epochs(epochs, "SPK type 9 epochs are invalid")?;
        if et_s < epochs[0] || et_s > epochs[n - 1] {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 9 query is outside state epoch coverage",
            });
        }

        let start = lagrange_window_start(epochs, et_s, window_size);
        let mut state = [0.0_f64; 6];
        for offset in 0..window_size {
            let source_index = start + offset;
            let basis = unequal_step_lagrange_basis(epochs, start, window_size, offset, et_s)?;
            for component in 0..6 {
                state[component] += basis * states[source_index * 6 + component];
            }
        }
        finite_state_from_components(state, "SPK type 9 Lagrange interpolation")
    }

    fn equal_step_hermite_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 10 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 12 segment is too short",
            });
        }
        let n = f64_to_usize(self.data[self.data.len() - 1])?;
        let window_size_minus_one = f64_to_usize(self.data[self.data.len() - 2])?;
        let step_s = self.data[self.data.len() - 3];
        let first_epoch_s = self.data[self.data.len() - 4];
        if !first_epoch_s.is_finite() || !step_s.is_finite() || step_s <= 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 12 epoch directory",
            });
        }
        let window_size =
            window_size_minus_one
                .checked_add(1)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "SPK type 12 window size overflow",
                })?;
        if n == 0 || window_size > n {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 12 interpolation window",
            });
        }
        let expected_len = 6_usize
            .checked_mul(n)
            .and_then(|state_len| state_len.checked_add(4))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 12 segment length overflow",
            })?;
        if expected_len != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 12 segment length does not match state count",
            });
        }
        let last_epoch_s = first_epoch_s + step_s * (n - 1) as f64;
        if !last_epoch_s.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 12 epoch coverage is invalid",
            });
        }
        if et_s < first_epoch_s || et_s > last_epoch_s {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 12 query is outside state epoch coverage",
            });
        }

        let start = equal_step_window_start(first_epoch_s, step_s, n, et_s, window_size);
        hermite_state_from_equal_step_window(
            &self.data[..6 * n],
            first_epoch_s,
            step_s,
            start,
            window_size,
            et_s,
        )
    }

    fn unequal_step_hermite_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 9 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 13 segment is too short",
            });
        }
        let n = f64_to_usize(self.data[self.data.len() - 1])?;
        let window_size_minus_one = f64_to_usize(self.data[self.data.len() - 2])?;
        let window_size =
            window_size_minus_one
                .checked_add(1)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "SPK type 13 window size overflow",
                })?;
        if n == 0 || window_size > n {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 13 interpolation window",
            });
        }
        let directory_count = (n - 1) / 100;
        let expected_len = 6_usize
            .checked_mul(n)
            .and_then(|state_len| state_len.checked_add(n))
            .and_then(|base| base.checked_add(directory_count))
            .and_then(|base| base.checked_add(2))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 13 segment length overflow",
            })?;
        if expected_len != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 13 segment length does not match directory",
            });
        }

        let states = &self.data[..6 * n];
        let epochs = &self.data[6 * n..7 * n];
        validate_strictly_increasing_epochs(epochs, "SPK type 13 epochs are invalid")?;
        if et_s < epochs[0] || et_s > epochs[n - 1] {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 13 query is outside state epoch coverage",
            });
        }

        let start = lagrange_window_start(epochs, et_s, window_size);
        hermite_state_from_unequal_step_window(states, epochs, start, window_size, et_s)
    }

    fn generic_chebyshev_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        let metadata = GenericSegmentMetadata::from_data(&self.data)?;
        if metadata.ncon != 1
            || metadata.rdrtyp != 3
            || metadata.pdrtyp != 0
            || metadata.npdr != 0
            || metadata.nref != metadata.npkt
            || metadata.npkt == 0
            || metadata.pktoff != 1
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "unsupported SPK type 14 generic segment layout",
            });
        }
        let expected_reference_directory_count = (metadata.nref - 1) / 100;
        if metadata.nrdr != expected_reference_directory_count || metadata.nrsv != 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 14 generic segment directory is invalid",
            });
        }
        let constants = metadata.partition_range(
            metadata.conbas,
            metadata.ncon,
            self.data.len(),
            "SPK type 14 constants are outside the segment",
        )?;
        let ncoeff = f64_to_usize(self.data[constants.start])?;
        if ncoeff == 0 || ncoeff > 19 {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 14 Chebyshev coefficient count",
            });
        }
        let expected_packet_size = 2_usize
            .checked_add(
                6_usize
                    .checked_mul(ncoeff)
                    .ok_or(PhysicsError::InvalidParameter {
                        reason: "SPK type 14 packet size overflow",
                    })?,
            )
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 14 packet size overflow",
            })?;
        if metadata.pktsz != expected_packet_size {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 14 packet size does not match coefficient count",
            });
        }
        let packet_record_size =
            metadata
                .pktsz
                .checked_add(metadata.pktoff)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "SPK type 14 packet record size overflow",
                })?;
        metadata.partition_range(
            metadata.pktbas,
            metadata.npkt.checked_mul(packet_record_size).ok_or(
                PhysicsError::InvalidParameter {
                    reason: "SPK type 14 packet partition size overflow",
                },
            )?,
            self.data.len(),
            "SPK type 14 packets are outside the segment",
        )?;
        let references = metadata.partition_range(
            metadata.refbas,
            metadata.nref,
            self.data.len(),
            "SPK type 14 reference epochs are outside the segment",
        )?;
        metadata.partition_range(
            metadata.rdrbas,
            metadata.nrdr,
            self.data.len(),
            "SPK type 14 reference directory is outside the segment",
        )?;
        metadata.partition_range(
            metadata.pdrbas,
            metadata.npdr,
            self.data.len(),
            "SPK type 14 packet directory is outside the segment",
        )?;
        metadata.partition_range(
            metadata.rsvbas,
            metadata.nrsv,
            self.data.len(),
            "SPK type 14 reserved partition is outside the segment",
        )?;
        let epochs = &self.data[references];
        validate_strictly_increasing_epochs(epochs, "SPK type 14 reference epochs are invalid")?;
        if et_s < epochs[0] || et_s > self.stop_et_s {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 14 query is outside reference epoch coverage",
            });
        }

        let insertion = epochs.partition_point(|epoch| *epoch <= et_s);
        let packet_index = insertion.saturating_sub(1).min(metadata.npkt - 1);
        let packet_start = metadata
            .pktbas
            .checked_add(packet_index.checked_mul(packet_record_size).ok_or(
                PhysicsError::InvalidParameter {
                    reason: "SPK type 14 packet address overflow",
                },
            )?)
            .and_then(|start| start.checked_add(metadata.pktoff))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 14 packet address overflow",
            })?;
        let packet = self
            .data
            .get(packet_start..packet_start + metadata.pktsz)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 14 packet is outside the segment",
            })?;
        type14_chebyshev_state_from_packet(packet, ncoeff, et_s)
    }

    fn precessing_conic_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() != 16 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 15 segment must contain exactly 16 values",
            });
        }
        type15_precessing_conic_state(&self.data, et_s)
    }

    fn esoc_ddid_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 10 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 18 segment is too short",
            });
        }
        let n = f64_to_usize(self.data[self.data.len() - 1])?;
        let window_size = f64_to_usize(self.data[self.data.len() - 2])?;
        let subtype = f64_to_i32(self.data[self.data.len() - 3])?;
        if n == 0 || window_size == 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 18 interpolation window",
            });
        }
        let packet_size: usize = match subtype {
            0 => 12,
            1 => 6,
            _ => {
                return Err(PhysicsError::InvalidParameter {
                    reason: "unsupported SPK type 18 subtype",
                });
            }
        };
        let directory_count = (n - 1) / 100;
        let expected_len = packet_size
            .checked_mul(n)
            .and_then(|packet_len| packet_len.checked_add(n))
            .and_then(|base| base.checked_add(directory_count))
            .and_then(|base| base.checked_add(3))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 18 segment length overflow",
            })?;
        if expected_len != self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 18 segment length does not match directory",
            });
        }

        let packets = &self.data[..packet_size * n];
        let epochs = &self.data[packet_size * n..packet_size * n + n];
        validate_strictly_increasing_epochs(epochs, "SPK type 18 epochs are invalid")?;
        if et_s < epochs[0] || et_s > epochs[n - 1] {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 18 query is outside packet epoch coverage",
            });
        }

        let actual_window = window_size.min(n);
        let start = lagrange_window_start(epochs, et_s, actual_window);
        match subtype {
            0 => esoc_ddid_hermite_state_from_window(packets, epochs, start, actual_window, et_s),
            1 => esoc_ddid_lagrange_state_from_window(packets, epochs, start, actual_window, et_s),
            _ => unreachable!(),
        }
    }

    fn esoc_ddid_piecewise_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 12 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 19 segment is too short",
            });
        }
        let interval_count = f64_to_usize(self.data[self.data.len() - 1])?;
        if interval_count == 0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 19 interval count is invalid",
            });
        }
        let use_later_boundary = match self.data[self.data.len() - 2] {
            0.0 => false,
            1.0 => true,
            _ => {
                return Err(PhysicsError::InvalidParameter {
                    reason: "SPK type 19 boundary choice flag is invalid",
                });
            }
        };
        let boundary_count =
            interval_count
                .checked_add(1)
                .ok_or(PhysicsError::InvalidParameter {
                    reason: "SPK type 19 interval count overflow",
                })?;
        let interval_directory_count = interval_count / 100;
        let pointer_count = boundary_count;
        let trailer_len = boundary_count
            .checked_add(interval_directory_count)
            .and_then(|len| len.checked_add(pointer_count))
            .and_then(|len| len.checked_add(2))
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 19 segment length overflow",
            })?;
        if trailer_len >= self.data.len() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 19 segment length does not match directory",
            });
        }

        let boundary_block_len = boundary_count.checked_add(interval_directory_count).ok_or(
            PhysicsError::InvalidParameter {
                reason: "SPK type 19 segment length overflow",
            },
        )?;
        let pointer_start = self.data.len() - 2 - pointer_count;
        let boundary_start = pointer_start.checked_sub(boundary_block_len).ok_or(
            PhysicsError::InvalidParameter {
                reason: "SPK type 19 segment length does not match directory",
            },
        )?;
        let boundaries = &self.data[boundary_start..boundary_start + boundary_count];
        validate_strictly_increasing_epochs(
            boundaries,
            "SPK type 19 interval boundaries are invalid",
        )?;
        if et_s < boundaries[0] || et_s > boundaries[boundary_count - 1] {
            return Err(PhysicsError::OutOfEnvelope {
                reason: "SPK type 19 query is outside interval coverage",
            });
        }

        let mut pointer_indices = Vec::with_capacity(pointer_count);
        for pointer in &self.data[pointer_start..pointer_start + pointer_count] {
            let pointer = f64_to_usize(*pointer)?;
            if pointer == 0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "SPK type 19 mini-segment pointer is invalid",
                });
            }
            pointer_indices.push(pointer - 1);
        }
        if pointer_indices[0] != 0 || pointer_indices[pointer_count - 1] != boundary_start {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 19 mini-segment pointers do not match segment layout",
            });
        }
        for pair in pointer_indices.windows(2) {
            if pair[1] <= pair[0] || pair[1] > boundary_start {
                return Err(PhysicsError::InvalidParameter {
                    reason: "SPK type 19 mini-segment pointers are invalid",
                });
            }
        }

        let interval_index = type19_interval_index(boundaries, et_s, use_later_boundary).ok_or(
            PhysicsError::OutOfEnvelope {
                reason: "SPK type 19 query is outside interval coverage",
            },
        )?;
        let mini_start = pointer_indices[interval_index];
        let mini_end = pointer_indices[interval_index + 1];
        esoc_ddid_mini_segment_state_km_s(&self.data[mini_start..mini_end], et_s)
    }

    fn chebyshev_velocity_state_km_s(&self, et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
        if self.data.len() < 13 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 20 segment is too short",
            });
        }
        let directory = self.data.len() - 7;
        let distance_scale_km = self.data[directory];
        let time_scale_s = self.data[directory + 1];
        let init_julian_date = self.data[directory + 2] + self.data[directory + 3];
        let interval_len_days = self.data[directory + 4];
        let rsize = f64_to_usize(self.data[directory + 5])?;
        let record_count = f64_to_usize(self.data[directory + 6])?;
        if !distance_scale_km.is_finite()
            || !time_scale_s.is_finite()
            || !init_julian_date.is_finite()
            || !interval_len_days.is_finite()
            || distance_scale_km <= 0.0
            || time_scale_s <= 0.0
            || interval_len_days <= 0.0
            || rsize < 6
            || record_count == 0
            || (rsize - 3) % 3 != 0
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "invalid SPK type 20 directory",
            });
        }
        if rsize
            .checked_mul(record_count)
            .and_then(|records| records.checked_add(7))
            != Some(self.data.len())
        {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 20 directory does not match segment length",
            });
        }

        let interval_len_s = interval_len_days * SECONDS_PER_DAY;
        let init_et_s = (init_julian_date - J2000_JULIAN_DATE) * SECONDS_PER_DAY;
        if !interval_len_s.is_finite() || !init_et_s.is_finite() {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 20 epoch directory is invalid",
            });
        }
        let mut record_index = ((et_s - init_et_s) / interval_len_s).floor();
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
        let coeff_count = (rsize - 3) / 3;
        let midpoint_s = init_et_s + (record_index as f64 + 0.5) * interval_len_s;
        let radius_s = 0.5 * interval_len_s;
        let tau = (et_s - midpoint_s) / radius_s;
        let velocity_scale = distance_scale_km / time_scale_s;

        let mut state = [0.0_f64; 6];
        for component in 0..3 {
            let base = component * (coeff_count + 1);
            let coefficients = &record[base..base + coeff_count];
            let midpoint_position = record[base + coeff_count] * distance_scale_km;
            let velocity = evaluate_chebyshev(tau, coefficients) * velocity_scale;
            let displacement = radius_s
                * velocity_scale
                * evaluate_chebyshev_integral_from_zero(tau, coefficients);
            state[component] = midpoint_position + displacement;
            state[component + 3] = velocity;
        }
        finite_state_from_components(state, "SPK type 20 Chebyshev velocity interpolation")
    }

    fn chebyshev_record(
        &self,
        et_s: f64,
        component_count: usize,
    ) -> Result<ChebyshevRecordView<'_>, PhysicsError> {
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
        Ok(ChebyshevRecordView {
            record,
            tau: (et_s - midpoint) / radius,
            radius_s: radius,
            coeff_count,
        })
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
                if supported_spk_inertial_frame(descriptor.frame)
                    && matches!(
                        descriptor.data_type,
                        2 | 3 | 5 | 8 | 9 | 12 | 13 | 14 | 15 | 18 | 19 | 20
                    )
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
            frame: descriptor.frame,
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

fn supported_spk_inertial_frame(frame: i32) -> bool {
    matches!(frame, SPK_J2000_FRAME_ID | SPK_ECLIPJ2000_FRAME_ID)
}

fn spk_frame_state_to_j2000_km_s(
    frame: i32,
    state: SpkStateKmS,
) -> Result<SpkStateKmS, PhysicsError> {
    match frame {
        SPK_J2000_FRAME_ID => Ok(state),
        SPK_ECLIPJ2000_FRAME_ID => Ok(SpkStateKmS {
            position_km: rotate_x(state.position_km, J2000_ECLIPTIC_OBLIQUITY_RAD),
            velocity_km_s: rotate_x(state.velocity_km_s, J2000_ECLIPTIC_OBLIQUITY_RAD),
        }),
        _ => Err(PhysicsError::InvalidParameter {
            reason: "unsupported SPK inertial frame",
        }),
    }
}

fn rotate_x(v: Vector3<f64>, theta: f64) -> Vector3<f64> {
    let (s, c) = theta.sin_cos();
    Vector3::new(v.x, c * v.y - s * v.z, s * v.y + c * v.z)
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

fn evaluate_chebyshev_derivative(tau: f64, coefficients: &[f64]) -> f64 {
    if coefficients.len() <= 1 {
        return 0.0;
    }
    let mut sum = coefficients[1];
    let mut t_prev = 1.0;
    let mut t_curr = tau;
    let mut dt_prev = 0.0;
    let mut dt_curr = 1.0;
    for coefficient in &coefficients[2..] {
        let t_next = 2.0 * tau * t_curr - t_prev;
        let dt_next = 2.0 * t_curr + 2.0 * tau * dt_curr - dt_prev;
        sum += coefficient * dt_next;
        t_prev = t_curr;
        t_curr = t_next;
        dt_prev = dt_curr;
        dt_curr = dt_next;
    }
    sum
}

fn evaluate_chebyshev_integral_from_zero(tau: f64, coefficients: &[f64]) -> f64 {
    if coefficients.is_empty() {
        return 0.0;
    }
    let mut values = Vec::with_capacity(coefficients.len() + 1);
    values.push(1.0);
    values.push(tau);
    for degree in 2..=coefficients.len() {
        values.push(2.0 * tau * values[degree - 1] - values[degree - 2]);
    }

    let mut zero_values = Vec::with_capacity(coefficients.len() + 1);
    zero_values.push(1.0);
    zero_values.push(0.0);
    for degree in 2..=coefficients.len() {
        zero_values.push(-zero_values[degree - 2]);
    }

    let mut sum = coefficients[0] * tau;
    for (degree, coefficient) in coefficients.iter().enumerate().skip(1) {
        let integral = if degree == 1 {
            0.5 * tau * tau
        } else {
            let degree_f = degree as f64;
            0.5 * ((values[degree + 1] - zero_values[degree + 1]) / (degree_f + 1.0)
                - (values[degree - 1] - zero_values[degree - 1]) / (degree_f - 1.0))
        };
        sum += coefficient * integral;
    }
    sum
}

fn chebyshev_vector(
    record: &[f64],
    coeff_count: usize,
    component_offset: usize,
    tau: f64,
) -> Vector3<f64> {
    let base = 2 + component_offset * coeff_count;
    Vector3::new(
        evaluate_chebyshev(tau, &record[base..base + coeff_count]),
        evaluate_chebyshev(tau, &record[base + coeff_count..base + 2 * coeff_count]),
        evaluate_chebyshev(tau, &record[base + 2 * coeff_count..base + 3 * coeff_count]),
    )
}

fn chebyshev_derivative_vector(
    record: &[f64],
    coeff_count: usize,
    component_offset: usize,
    radius_s: f64,
    tau: f64,
) -> Vector3<f64> {
    let base = 2 + component_offset * coeff_count;
    Vector3::new(
        evaluate_chebyshev_derivative(tau, &record[base..base + coeff_count]) / radius_s,
        evaluate_chebyshev_derivative(tau, &record[base + coeff_count..base + 2 * coeff_count])
            / radius_s,
        evaluate_chebyshev_derivative(tau, &record[base + 2 * coeff_count..base + 3 * coeff_count])
            / radius_s,
    )
}

fn type5_two_body_blend_state(
    first_state: &[f64],
    first_epoch_s: f64,
    second_state: &[f64],
    second_epoch_s: f64,
    gm_km3_s2: f64,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    if first_state.len() != 6 || second_state.len() != 6 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 record state size is invalid",
        });
    }
    if !first_epoch_s.is_finite() || !second_epoch_s.is_finite() || second_epoch_s < first_epoch_s {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 record epochs are invalid",
        });
    }
    let first = finite_state_from_components(
        [
            first_state[0],
            first_state[1],
            first_state[2],
            first_state[3],
            first_state[4],
            first_state[5],
        ],
        "SPK type 5 first record",
    )?;
    if first_epoch_s == second_epoch_s {
        return propagate_two_body_state_km_s(&first, gm_km3_s2, et_s - first_epoch_s);
    }
    let second = finite_state_from_components(
        [
            second_state[0],
            second_state[1],
            second_state[2],
            second_state[3],
            second_state[4],
            second_state[5],
        ],
        "SPK type 5 second record",
    )?;
    let first_propagated = propagate_two_body_state_km_s(&first, gm_km3_s2, et_s - first_epoch_s)?;
    let second_propagated =
        propagate_two_body_state_km_s(&second, gm_km3_s2, et_s - second_epoch_s)?;
    let denominator_s = second_epoch_s - first_epoch_s;
    if denominator_s <= 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 bracketing interval is invalid",
        });
    }
    let arg = core::f64::consts::PI * (et_s - first_epoch_s) / denominator_s;
    let weight = 0.5 + 0.5 * arg.cos();
    let weight_dot = -0.5 * arg.sin() * core::f64::consts::PI / denominator_s;
    let position =
        first_propagated.position_km * weight + second_propagated.position_km * (1.0 - weight);
    let velocity = first_propagated.velocity_km_s * weight
        + second_propagated.velocity_km_s * (1.0 - weight)
        + (first_propagated.position_km - second_propagated.position_km) * weight_dot;
    finite_state_from_components(
        [
            position.x, position.y, position.z, velocity.x, velocity.y, velocity.z,
        ],
        "SPK type 5 two-body interpolation",
    )
}

fn propagate_two_body_state_km_s(
    state: &SpkStateKmS,
    gm_km3_s2: f64,
    dt_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    if !gm_km3_s2.is_finite() || gm_km3_s2 <= 0.0 || !dt_s.is_finite() {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 two-body propagation inputs are invalid",
        });
    }
    let r0 = state.position_km;
    let v0 = state.velocity_km_s;
    let r0_norm = r0.norm();
    let v0_norm = v0.norm();
    if !r0_norm.is_finite() || !v0_norm.is_finite() || r0_norm == 0.0 || v0_norm == 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 two-body propagation state is invalid",
        });
    }
    if r0.cross(&v0).norm_squared() == 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 two-body propagation has rectilinear motion",
        });
    }
    if dt_s == 0.0 {
        return Ok(*state);
    }

    let sqrt_mu = gm_km3_s2.sqrt();
    let rv0 = r0.dot(&v0);
    let alpha = 2.0 / r0_norm - v0.norm_squared() / gm_km3_s2;
    let mut chi = initial_universal_anomaly(alpha, gm_km3_s2, sqrt_mu, r0_norm, rv0, dt_s);
    if !chi.is_finite() {
        chi = sqrt_mu * dt_s / r0_norm;
    }

    let mut converged = false;
    for _ in 0..64 {
        let z = alpha * chi * chi;
        let (c2, c3) = stumpff_c2_c3(z)?;
        let value = r0_norm * rv0 / sqrt_mu * chi * chi * c2
            + (1.0 - alpha * r0_norm) * chi * chi * chi * c3
            + r0_norm * chi
            - sqrt_mu * dt_s;
        let derivative = r0_norm * rv0 / sqrt_mu * chi * (1.0 - z * c3)
            + (1.0 - alpha * r0_norm) * chi * chi * c2
            + r0_norm;
        if !value.is_finite() || !derivative.is_finite() || derivative == 0.0 {
            return Err(PhysicsError::NonFinite {
                reason: "SPK type 5 two-body propagation failed to converge",
            });
        }
        let delta = value / derivative;
        chi -= delta;
        if !chi.is_finite() {
            return Err(PhysicsError::NonFinite {
                reason: "SPK type 5 two-body propagation failed to converge",
            });
        }
        if delta.abs() <= 1.0e-12 * chi.abs().max(1.0) {
            converged = true;
            break;
        }
    }
    if !converged {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 5 two-body propagation did not converge",
        });
    }

    let z = alpha * chi * chi;
    let (c2, c3) = stumpff_c2_c3(z)?;
    let f = 1.0 - chi * chi * c2 / r0_norm;
    let g = dt_s - chi * chi * chi * c3 / sqrt_mu;
    let position = r0 * f + v0 * g;
    let radius = position.norm();
    if !radius.is_finite() || radius == 0.0 {
        return Err(PhysicsError::NonFinite {
            reason: "SPK type 5 two-body propagation produced invalid radius",
        });
    }
    let f_dot = sqrt_mu * chi * (z * c3 - 1.0) / (radius * r0_norm);
    let g_dot = 1.0 - chi * chi * c2 / radius;
    let velocity = r0 * f_dot + v0 * g_dot;
    finite_state_from_components(
        [
            position.x, position.y, position.z, velocity.x, velocity.y, velocity.z,
        ],
        "SPK type 5 two-body propagation",
    )
}

fn initial_universal_anomaly(
    alpha: f64,
    gm_km3_s2: f64,
    sqrt_mu: f64,
    r0_norm: f64,
    rv0: f64,
    dt_s: f64,
) -> f64 {
    if alpha > 1.0e-12 {
        sqrt_mu * alpha * dt_s
    } else if alpha < -1.0e-12 {
        let semi_major_axis_km = 1.0 / alpha;
        let denominator = rv0
            + dt_s.signum() * (-gm_km3_s2 * semi_major_axis_km).sqrt() * (1.0 - r0_norm * alpha);
        let ratio = (-2.0 * gm_km3_s2 * alpha * dt_s) / denominator;
        if ratio.is_finite() && ratio > 0.0 {
            dt_s.signum() * (-semi_major_axis_km).sqrt() * ratio.ln()
        } else {
            dt_s.signum() * (-semi_major_axis_km).sqrt()
        }
    } else {
        sqrt_mu * dt_s / r0_norm
    }
}

fn stumpff_c2_c3(z: f64) -> Result<(f64, f64), PhysicsError> {
    if !z.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK type 5 Stumpff argument is non-finite",
        });
    }
    let (c2, c3) = if z > 1.0e-8 {
        let root = z.sqrt();
        (
            (1.0 - root.cos()) / z,
            (root - root.sin()) / (root * root * root),
        )
    } else if z < -1.0e-8 {
        let root = (-z).sqrt();
        if root > 700.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK type 5 Stumpff argument is outside the numeric envelope",
            });
        }
        (
            (root.cosh() - 1.0) / (-z),
            (root.sinh() - root) / (root * root * root),
        )
    } else {
        (
            0.5 - z / 24.0 + z * z / 720.0 - z * z * z / 40_320.0,
            1.0 / 6.0 - z / 120.0 + z * z / 5_040.0 - z * z * z / 362_880.0,
        )
    };
    if !c2.is_finite() || !c3.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK type 5 Stumpff functions are non-finite",
        });
    }
    Ok((c2, c3))
}

fn type15_precessing_conic_state(record: &[f64], et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
    if record.len() != 16 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 15 record size is invalid",
        });
    }
    let epoch_s = record[0];
    let trajectory_pole = unit_vector(
        Vector3::new(record[1], record[2], record[3]),
        "SPK type 15 trajectory pole vector is invalid",
    )?;
    let periapsis_direction = unit_vector(
        Vector3::new(record[4], record[5], record[6]),
        "SPK type 15 periapsis vector is invalid",
    )?;
    let semi_latus_rectum_km = record[7];
    let eccentricity = record[8];
    let j2_flag = f64_to_i32(record[9])?;
    let central_pole = unit_vector(
        Vector3::new(record[10], record[11], record[12]),
        "SPK type 15 central body pole vector is invalid",
    )?;
    let gm_km3_s2 = record[13];
    let j2 = record[14];
    let central_radius_km = record[15];

    if !epoch_s.is_finite()
        || !semi_latus_rectum_km.is_finite()
        || !eccentricity.is_finite()
        || !gm_km3_s2.is_finite()
        || !j2.is_finite()
        || !central_radius_km.is_finite()
        || semi_latus_rectum_km <= 0.0
        || eccentricity < 0.0
        || gm_km3_s2 <= 0.0
        || central_radius_km < 0.0
    {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 15 conic parameters are invalid",
        });
    }
    if periapsis_direction.dot(&trajectory_pole).abs() > 1.0e-5 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 15 trajectory pole and periapsis vectors are not orthogonal",
        });
    }

    let periapsis_radius_km = semi_latus_rectum_km / (1.0 + eccentricity);
    let periapsis_speed_km_s = (gm_km3_s2 / semi_latus_rectum_km).sqrt() * (1.0 + eccentricity);
    let periapsis_state = SpkStateKmS {
        position_km: periapsis_direction * periapsis_radius_km,
        velocity_km_s: trajectory_pole.cross(&periapsis_direction) * periapsis_speed_km_s,
    };
    let dt_s = et_s - epoch_s;
    let mut state = propagate_two_body_state_km_s(&periapsis_state, gm_km3_s2, dt_s)?;

    if j2_flag != 3 && j2 != 0.0 && eccentricity < 1.0 && periapsis_radius_km > central_radius_km {
        let one_minus_e2 = 1.0 - eccentricity * eccentricity;
        let mean_anomaly_rate = one_minus_e2 / semi_latus_rectum_km
            * (gm_km3_s2 * one_minus_e2 / semi_latus_rectum_km).sqrt();
        let mean_anomaly = mean_anomaly_rate * dt_s;
        let mut theta = mean_anomaly % core::f64::consts::TAU;
        if theta.abs() > core::f64::consts::PI {
            theta -= theta.signum() * core::f64::consts::TAU;
        }
        let completed_revolutions_angle = mean_anomaly - theta;
        let mut true_anomaly = vector_separation_rad(periapsis_direction, state.position_km)?;
        true_anomaly = true_anomaly.copysign(theta) + completed_revolutions_angle;

        let cos_inclination = central_pole.dot(&trajectory_pole);
        let scaled_j2_angle =
            true_anomaly * 1.5 * j2 * (central_radius_km / semi_latus_rectum_km).powi(2);
        let node_regression_rad = -scaled_j2_angle * cos_inclination;
        let apsis_precession_rad =
            scaled_j2_angle * (2.5 * cos_inclination * cos_inclination - 0.5);

        if j2_flag != 1 {
            state = rotate_state_about_axis(state, trajectory_pole, apsis_precession_rad);
        }
        if j2_flag != 2 {
            state = rotate_state_about_axis(state, central_pole, node_regression_rad);
        }
    }

    finite_state_from_components(
        [
            state.position_km.x,
            state.position_km.y,
            state.position_km.z,
            state.velocity_km_s.x,
            state.velocity_km_s.y,
            state.velocity_km_s.z,
        ],
        "SPK type 15 precessing conic propagation",
    )
}

fn unit_vector(v: Vector3<f64>, reason: &'static str) -> Result<Vector3<f64>, PhysicsError> {
    if !v.iter().all(|value| value.is_finite()) {
        return Err(PhysicsError::InvalidParameter { reason });
    }
    let norm = v.norm();
    if !norm.is_finite() || norm == 0.0 {
        return Err(PhysicsError::InvalidParameter { reason });
    }
    Ok(v / norm)
}

fn vector_separation_rad(a: Vector3<f64>, b: Vector3<f64>) -> Result<f64, PhysicsError> {
    let a_norm = a.norm();
    let b_norm = b.norm();
    if !a_norm.is_finite() || !b_norm.is_finite() || a_norm == 0.0 || b_norm == 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 15 vector separation is invalid",
        });
    }
    let cos_angle = (a.dot(&b) / (a_norm * b_norm)).clamp(-1.0, 1.0);
    let angle = cos_angle.acos();
    if !angle.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK type 15 vector separation produced non-finite angle",
        });
    }
    Ok(angle)
}

fn rotate_state_about_axis(state: SpkStateKmS, axis: Vector3<f64>, angle_rad: f64) -> SpkStateKmS {
    SpkStateKmS {
        position_km: rotate_about_axis(state.position_km, axis, angle_rad),
        velocity_km_s: rotate_about_axis(state.velocity_km_s, axis, angle_rad),
    }
}

fn rotate_about_axis(v: Vector3<f64>, axis: Vector3<f64>, angle_rad: f64) -> Vector3<f64> {
    let (sin_angle, cos_angle) = angle_rad.sin_cos();
    v * cos_angle + axis.cross(&v) * sin_angle + axis * axis.dot(&v) * (1.0 - cos_angle)
}

fn type14_chebyshev_state_from_packet(
    packet: &[f64],
    coeff_count: usize,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    let expected_len =
        2_usize
            .checked_add(6_usize.checked_mul(coeff_count).ok_or(
                PhysicsError::InvalidParameter {
                    reason: "SPK type 14 packet size overflow",
                },
            )?)
            .ok_or(PhysicsError::InvalidParameter {
                reason: "SPK type 14 packet size overflow",
            })?;
    if packet.len() != expected_len {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 14 packet size is invalid",
        });
    }
    let midpoint_s = packet[0];
    let radius_s = packet[1];
    if !midpoint_s.is_finite() || !radius_s.is_finite() || radius_s <= 0.0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 14 packet interval is invalid",
        });
    }
    let tau = (et_s - midpoint_s) / radius_s;
    if !tau.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK type 14 Chebyshev interpolation produced non-finite state",
        });
    }
    let mut state = [0.0_f64; 6];
    for component in 0..6 {
        let base = 2 + component * coeff_count;
        state[component] = evaluate_chebyshev(tau, &packet[base..base + coeff_count]);
    }
    finite_state_from_components(state, "SPK type 14 Chebyshev interpolation")
}

fn validate_strictly_increasing_epochs(
    epochs: &[f64],
    reason: &'static str,
) -> Result<(), PhysicsError> {
    if epochs.is_empty() || !epochs[0].is_finite() {
        return Err(PhysicsError::InvalidParameter { reason });
    }
    for pair in epochs.windows(2) {
        if !pair[1].is_finite() || pair[1] <= pair[0] {
            return Err(PhysicsError::InvalidParameter { reason });
        }
    }
    Ok(())
}

fn lagrange_window_start(epochs: &[f64], et_s: f64, window_size: usize) -> usize {
    debug_assert!(window_size <= epochs.len());
    let last_start = epochs.len() - window_size;
    let half = window_size / 2;
    let candidate = if window_size % 2 == 0 {
        let insertion = epochs.partition_point(|epoch| *epoch < et_s);
        insertion.saturating_sub(half)
    } else {
        nearest_epoch_index(epochs, et_s).saturating_sub(half)
    };
    candidate.min(last_start)
}

fn equal_step_window_start(
    first_epoch_s: f64,
    step_s: f64,
    count: usize,
    et_s: f64,
    window_size: usize,
) -> usize {
    debug_assert!(window_size <= count);
    let last_start = count - window_size;
    let half = window_size / 2;
    let candidate = if window_size % 2 == 0 {
        equal_step_insertion_index(first_epoch_s, step_s, count, et_s).saturating_sub(half)
    } else {
        nearest_equal_step_epoch_index(first_epoch_s, step_s, count, et_s).saturating_sub(half)
    };
    candidate.min(last_start)
}

fn equal_step_insertion_index(first_epoch_s: f64, step_s: f64, count: usize, et_s: f64) -> usize {
    if et_s <= first_epoch_s {
        return 0;
    }
    let scaled = ((et_s - first_epoch_s) / step_s).ceil();
    if scaled >= count as f64 {
        count
    } else {
        scaled as usize
    }
}

fn nearest_equal_step_epoch_index(
    first_epoch_s: f64,
    step_s: f64,
    count: usize,
    et_s: f64,
) -> usize {
    if et_s <= first_epoch_s {
        return 0;
    }
    let scaled = (et_s - first_epoch_s) / step_s;
    let before = scaled.floor() as usize;
    if before + 1 >= count {
        return count - 1;
    }
    let before_epoch = first_epoch_s + step_s * before as f64;
    let after_epoch = before_epoch + step_s;
    if (et_s - before_epoch).abs() <= (after_epoch - et_s).abs() {
        before
    } else {
        before + 1
    }
}

fn nearest_epoch_index(epochs: &[f64], et_s: f64) -> usize {
    let insertion = epochs.partition_point(|epoch| *epoch < et_s);
    if insertion == 0 {
        return 0;
    }
    if insertion >= epochs.len() {
        return epochs.len() - 1;
    }
    let before = insertion - 1;
    if (et_s - epochs[before]).abs() <= (epochs[insertion] - et_s).abs() {
        before
    } else {
        insertion
    }
}

fn equal_step_lagrange_basis(
    first_epoch_s: f64,
    step_s: f64,
    start: usize,
    window_size: usize,
    offset: usize,
    et_s: f64,
) -> Result<f64, PhysicsError> {
    let source_epoch = first_epoch_s + step_s * (start + offset) as f64;
    let mut basis = 1.0;
    for other_offset in 0..window_size {
        if other_offset == offset {
            continue;
        }
        let other_epoch = first_epoch_s + step_s * (start + other_offset) as f64;
        let denominator = source_epoch - other_epoch;
        if denominator == 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK equal-step Lagrange epochs contain duplicates",
            });
        }
        basis *= (et_s - other_epoch) / denominator;
    }
    if !basis.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK equal-step Lagrange interpolation produced non-finite basis",
        });
    }
    Ok(basis)
}

fn unequal_step_lagrange_basis(
    epochs: &[f64],
    start: usize,
    window_size: usize,
    offset: usize,
    et_s: f64,
) -> Result<f64, PhysicsError> {
    let source_epoch = epochs[start + offset];
    let mut basis = 1.0;
    for other_offset in 0..window_size {
        if other_offset == offset {
            continue;
        }
        let other_epoch = epochs[start + other_offset];
        let denominator = source_epoch - other_epoch;
        if denominator == 0.0 {
            return Err(PhysicsError::InvalidParameter {
                reason: "SPK unequal-step Lagrange epochs contain duplicates",
            });
        }
        basis *= (et_s - other_epoch) / denominator;
    }
    if !basis.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK unequal-step Lagrange interpolation produced non-finite basis",
        });
    }
    Ok(basis)
}

fn hermite_state_from_equal_step_window(
    states: &[f64],
    first_epoch_s: f64,
    step_s: f64,
    start: usize,
    window_size: usize,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    let epochs: Vec<f64> = (0..window_size)
        .map(|offset| first_epoch_s + step_s * (start + offset) as f64)
        .collect();
    hermite_state_from_window(states, &epochs, start, window_size, et_s)
}

fn hermite_state_from_unequal_step_window(
    states: &[f64],
    epochs: &[f64],
    start: usize,
    window_size: usize,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    hermite_state_from_window(
        states,
        &epochs[start..start + window_size],
        start,
        window_size,
        et_s,
    )
}

fn hermite_state_from_window(
    states: &[f64],
    epochs: &[f64],
    state_start: usize,
    window_size: usize,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    let mut state = [0.0_f64; 6];
    for component in 0..3 {
        let mut positions = Vec::with_capacity(window_size);
        let mut velocities = Vec::with_capacity(window_size);
        for offset in 0..window_size {
            let state_index = state_start + offset;
            positions.push(states[state_index * 6 + component]);
            velocities.push(states[state_index * 6 + 3 + component]);
        }
        let (position, velocity) =
            hermite_interpolate_value_derivative(epochs, &positions, &velocities, et_s)?;
        state[component] = position;
        state[component + 3] = velocity;
    }
    finite_state_from_components(state, "SPK Hermite interpolation")
}

fn esoc_ddid_mini_segment_state_km_s(data: &[f64], et_s: f64) -> Result<SpkStateKmS, PhysicsError> {
    if data.len() < 10 {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 19 mini-segment is too short",
        });
    }
    let n = f64_to_usize(data[data.len() - 1])?;
    let window_size = f64_to_usize(data[data.len() - 2])?;
    let subtype = f64_to_i32(data[data.len() - 3])?;
    if n == 0 || window_size == 0 {
        return Err(PhysicsError::InvalidParameter {
            reason: "invalid SPK type 19 mini-segment interpolation window",
        });
    }
    let packet_size: usize = match subtype {
        0 => 12,
        1 | 2 => 6,
        _ => {
            return Err(PhysicsError::InvalidParameter {
                reason: "unsupported SPK type 19 mini-segment subtype",
            });
        }
    };
    let directory_count = (n - 1) / 100;
    let expected_len = packet_size
        .checked_mul(n)
        .and_then(|packet_len| packet_len.checked_add(n))
        .and_then(|base| base.checked_add(directory_count))
        .and_then(|base| base.checked_add(3))
        .ok_or(PhysicsError::InvalidParameter {
            reason: "SPK type 19 mini-segment length overflow",
        })?;
    if expected_len != data.len() {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK type 19 mini-segment length does not match directory",
        });
    }

    let packets = &data[..packet_size * n];
    let epochs = &data[packet_size * n..packet_size * n + n];
    validate_strictly_increasing_epochs(epochs, "SPK type 19 mini-segment epochs are invalid")?;
    if et_s < epochs[0] || et_s > epochs[n - 1] {
        return Err(PhysicsError::OutOfEnvelope {
            reason: "SPK type 19 mini-segment query is outside packet epoch coverage",
        });
    }

    let actual_window = window_size.min(n);
    let start = lagrange_window_start(epochs, et_s, actual_window);
    match subtype {
        0 => esoc_ddid_hermite_state_from_window(packets, epochs, start, actual_window, et_s),
        1 => esoc_ddid_lagrange_state_from_window(packets, epochs, start, actual_window, et_s),
        2 => hermite_state_from_unequal_step_window(packets, epochs, start, actual_window, et_s),
        _ => unreachable!(),
    }
}

fn esoc_ddid_lagrange_state_from_window(
    packets: &[f64],
    epochs: &[f64],
    start: usize,
    window_size: usize,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    let mut state = [0.0_f64; 6];
    for offset in 0..window_size {
        let source_index = start + offset;
        let basis = unequal_step_lagrange_basis(epochs, start, window_size, offset, et_s)?;
        for component in 0..6 {
            state[component] += basis * packets[source_index * 6 + component];
        }
    }
    finite_state_from_components(state, "SPK ESOC/DDID Lagrange interpolation")
}

fn esoc_ddid_hermite_state_from_window(
    packets: &[f64],
    epochs: &[f64],
    start: usize,
    window_size: usize,
    et_s: f64,
) -> Result<SpkStateKmS, PhysicsError> {
    let window_epochs = &epochs[start..start + window_size];
    let mut state = [0.0_f64; 6];
    for component in 0..3 {
        let mut positions = Vec::with_capacity(window_size);
        let mut position_derivatives = Vec::with_capacity(window_size);
        let mut velocities = Vec::with_capacity(window_size);
        let mut velocity_derivatives = Vec::with_capacity(window_size);
        for offset in 0..window_size {
            let packet_index = start + offset;
            let base = packet_index * 12;
            positions.push(packets[base + component]);
            position_derivatives.push(packets[base + 3 + component]);
            velocities.push(packets[base + 6 + component]);
            velocity_derivatives.push(packets[base + 9 + component]);
        }
        let (position, _) = hermite_interpolate_value_derivative(
            window_epochs,
            &positions,
            &position_derivatives,
            et_s,
        )?;
        let (velocity, _) = hermite_interpolate_value_derivative(
            window_epochs,
            &velocities,
            &velocity_derivatives,
            et_s,
        )?;
        state[component] = position;
        state[component + 3] = velocity;
    }
    finite_state_from_components(state, "SPK ESOC/DDID Hermite interpolation")
}

fn type19_interval_index(boundaries: &[f64], et_s: f64, use_later_boundary: bool) -> Option<usize> {
    if boundaries.len() < 2 || et_s < boundaries[0] || et_s > boundaries[boundaries.len() - 1] {
        return None;
    }
    let last_interval = boundaries.len() - 2;
    if et_s <= boundaries[0] {
        return Some(0);
    }
    if et_s >= boundaries[boundaries.len() - 1] {
        return Some(last_interval);
    }
    let insertion = boundaries.partition_point(|boundary| *boundary < et_s);
    if insertion < boundaries.len() && boundaries[insertion] == et_s {
        if use_later_boundary {
            Some(insertion.min(last_interval))
        } else {
            Some(insertion.saturating_sub(1))
        }
    } else {
        Some(insertion.saturating_sub(1).min(last_interval))
    }
}

fn hermite_interpolate_value_derivative(
    epochs: &[f64],
    values: &[f64],
    derivatives: &[f64],
    et_s: f64,
) -> Result<(f64, f64), PhysicsError> {
    let count = epochs.len();
    if count == 0 || values.len() != count || derivatives.len() != count {
        return Err(PhysicsError::InvalidParameter {
            reason: "SPK Hermite interpolation window is invalid",
        });
    }

    let repeated_count = count.checked_mul(2).ok_or(PhysicsError::InvalidParameter {
        reason: "SPK Hermite interpolation window is too large",
    })?;
    let mut z = vec![0.0_f64; repeated_count];
    let mut table = vec![vec![0.0_f64; repeated_count]; repeated_count];

    for index in 0..count {
        let even = 2 * index;
        let odd = even + 1;
        z[even] = epochs[index];
        z[odd] = epochs[index];
        table[even][0] = values[index];
        table[odd][0] = values[index];
        table[odd][1] = derivatives[index];
        if index == 0 {
            table[even][1] = derivatives[index];
        } else {
            let denominator = z[even] - z[even - 1];
            if denominator == 0.0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "SPK Hermite interpolation epochs contain duplicates",
                });
            }
            table[even][1] = (table[even][0] - table[even - 1][0]) / denominator;
        }
    }

    for row in 2..repeated_count {
        for column in 2..=row {
            let denominator = z[row] - z[row - column];
            if denominator == 0.0 {
                return Err(PhysicsError::InvalidParameter {
                    reason: "SPK Hermite interpolation epochs contain duplicates",
                });
            }
            table[row][column] =
                (table[row][column - 1] - table[row - 1][column - 1]) / denominator;
        }
    }

    let mut value = table[repeated_count - 1][repeated_count - 1];
    let mut derivative = 0.0;
    for index in (0..repeated_count - 1).rev() {
        derivative = value + (et_s - z[index]) * derivative;
        value = table[index][index] + (et_s - z[index]) * value;
    }
    if !value.is_finite() || !derivative.is_finite() {
        return Err(PhysicsError::NonFinite {
            reason: "SPK Hermite interpolation produced non-finite state",
        });
    }
    Ok((value, derivative))
}

fn finite_state_from_components(
    state: [f64; 6],
    source: &'static str,
) -> Result<SpkStateKmS, PhysicsError> {
    if !state.iter().all(|value| value.is_finite()) {
        return Err(PhysicsError::NonFinite { reason: source });
    }
    Ok(SpkStateKmS {
        position_km: Vector3::new(state[0], state[1], state[2]),
        velocity_km_s: Vector3::new(state[3], state[4], state[5]),
    })
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
    const TYPE5_TEST_GM_KM3_S2: f64 = 398_600.435_436;
    const TYPE5_TEST_RADIUS_KM: f64 = 7_000.0;
    const TYPE15_TEST_CENTRAL_RADIUS_KM: f64 = 6_378.136_3;

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
    fn low_precision_state_reports_finite_velocity() {
        let ephemeris = LowPrecisionSunMoonEphemeris::j2000();
        let state = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(60.0))
            .unwrap();
        assert!(state.position_eci_m.iter().all(|v| v.is_finite()));
        assert!(state.velocity_eci_m_s.iter().all(|v| v.is_finite()));
        assert!(state.velocity_eci_m_s.norm() > 100.0);
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
    fn spk_ephemeris_reports_state_from_type3_velocity_coefficients() {
        let bytes = synthetic_type3_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::ZERO)
            .unwrap();
        assert_eq!(
            sun.position_eci_m,
            Vector3::new(149_597_870.0e3 - 4_700.0e3, -1_200.0e3, 300.0e3)
        );
        assert_eq!(sun.velocity_eci_m_s, Vector3::new(0.0, 29_780.0, 0.0));
    }

    #[test]
    fn spk_ephemeris_reads_type5_two_body_state() {
        let bytes = synthetic_type5_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(5.0))
            .unwrap();
        let expected = type5_circular_state(5.0);
        assert_vector_near(
            sun.position_eci_m,
            Vector3::new(expected[0], expected[1], expected[2]) * 1_000.0,
            1.0e-5,
        );
        assert_vector_near(
            sun.velocity_eci_m_s,
            Vector3::new(expected[3], expected[4], expected[5]) * 1_000.0,
            1.0e-8,
        );
    }

    #[test]
    fn spk_ephemeris_reads_type9_lagrange_state() {
        let bytes = synthetic_type9_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(3.0))
            .unwrap();
        assert_eq!(
            sun.position_eci_m,
            Vector3::new(30_000.0, 6_000.0, -3_000.0)
        );
        assert_eq!(
            sun.velocity_eci_m_s,
            Vector3::new(10_000.0, 2_000.0, -1_000.0)
        );
    }

    #[test]
    fn spk_ephemeris_reads_type14_generic_chebyshev_state() {
        let bytes = synthetic_type14_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(7.0))
            .unwrap();
        assert_vector_near(
            sun.position_eci_m,
            Vector3::new(70_000.0, 14_000.0, -7_000.0),
            1.0e-10,
        );
        assert_vector_near(
            sun.velocity_eci_m_s,
            Vector3::new(10_000.0, 2_000.0, -1_000.0),
            1.0e-10,
        );
    }

    #[test]
    fn spk_ephemeris_reads_type15_precessing_conic_state() {
        let bytes = synthetic_type15_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(5.0))
            .unwrap();
        let expected = type5_circular_state(5.0);
        assert_vector_near(
            sun.position_eci_m,
            Vector3::new(expected[0], expected[1], expected[2]) * 1_000.0,
            1.0e-5,
        );
        assert_vector_near(
            sun.velocity_eci_m_s,
            Vector3::new(expected[3], expected[4], expected[5]) * 1_000.0,
            1.0e-8,
        );
    }

    #[test]
    fn spk_ephemeris_reads_type18_lagrange_state() {
        let bytes = synthetic_type18_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(3.0))
            .unwrap();
        assert_eq!(
            sun.position_eci_m,
            Vector3::new(30_000.0, 6_000.0, -3_000.0)
        );
        assert_eq!(
            sun.velocity_eci_m_s,
            Vector3::new(10_000.0, 2_000.0, -1_000.0)
        );
    }

    #[test]
    fn spk_ephemeris_reads_type19_piecewise_state() {
        let bytes = synthetic_type19_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(7.0))
            .unwrap();
        assert_eq!(
            sun.position_eci_m,
            Vector3::new(170_000.0, 14_000.0, -7_000.0)
        );
        assert_eq!(
            sun.velocity_eci_m_s,
            Vector3::new(10_000.0, 2_000.0, -1_000.0)
        );
    }

    #[test]
    fn spk_ephemeris_reads_type8_equal_step_lagrange_state() {
        let bytes = synthetic_type8_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(3.0))
            .unwrap();
        assert_eq!(
            sun.position_eci_m,
            Vector3::new(30_000.0, 6_000.0, -3_000.0)
        );
        assert_eq!(
            sun.velocity_eci_m_s,
            Vector3::new(10_000.0, 2_000.0, -1_000.0)
        );
    }

    #[test]
    fn spk_type2_state_uses_chebyshev_position_derivative() {
        let segment = SpkSegment {
            start_et_s: -10.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 2,
            data: vec![
                0.0, 10.0, 100.0, 20.0, 0.0, 30.0, 0.0, 40.0, -10.0, 20.0, 8.0, 1.0,
            ],
        };
        let state = segment.state_km_s(0.0).unwrap();
        assert_eq!(state.position_km, Vector3::new(100.0, 0.0, 0.0));
        assert_eq!(state.velocity_km_s, Vector3::new(2.0, 3.0, 4.0));
    }

    #[test]
    fn spk_type9_state_uses_centered_lagrange_window() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 4.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 9,
            data: type9_linear_segment(&[0.0, 1.0, 2.0, 4.0], 1),
        };
        let state = segment.state_km_s(3.0).unwrap();
        assert_eq!(state.position_km, Vector3::new(30.0, 6.0, -3.0));
        assert_eq!(state.velocity_km_s, Vector3::new(10.0, 2.0, -1.0));
    }

    #[test]
    fn spk_type5_state_uses_two_body_propagation() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 5,
            data: type5_circular_segment(&[0.0, 10.0]),
        };
        let state = segment.state_km_s(5.0).unwrap();
        let expected = type5_circular_state(5.0);
        assert_vector_near(
            state.position_km,
            Vector3::new(expected[0], expected[1], expected[2]),
            1.0e-8,
        );
        assert_vector_near(
            state.velocity_km_s,
            Vector3::new(expected[3], expected[4], expected[5]),
            1.0e-11,
        );
    }

    #[test]
    fn spk_type15_state_uses_precessing_conic_record() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 15,
            data: type15_circular_segment(3.0, 0.0),
        };
        let state = segment.state_km_s(5.0).unwrap();
        let expected = type5_circular_state(5.0);
        assert_vector_near(
            state.position_km,
            Vector3::new(expected[0], expected[1], expected[2]),
            1.0e-8,
        );
        assert_vector_near(
            state.velocity_km_s,
            Vector3::new(expected[3], expected[4], expected[5]),
            1.0e-11,
        );
    }

    #[test]
    fn spk_type15_j2_flag_applies_node_and_apsis_precession() {
        let j2 = 1.0e-3;
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 15,
            data: type15_circular_segment(0.0, j2),
        };
        let state = segment.state_km_s(5.0).unwrap();
        let mean_motion_rad_s = (TYPE5_TEST_GM_KM3_S2 / TYPE5_TEST_RADIUS_KM.powi(3)).sqrt();
        let theta = mean_motion_rad_s * 5.0;
        let radius_ratio = TYPE15_TEST_CENTRAL_RADIUS_KM / TYPE5_TEST_RADIUS_KM;
        let precession = theta * 1.5 * j2 * radius_ratio * radius_ratio;
        let expected_theta = theta + precession;
        let (sin_theta, cos_theta) = expected_theta.sin_cos();
        let speed_km_s = mean_motion_rad_s * TYPE5_TEST_RADIUS_KM;
        assert_vector_near(
            state.position_km,
            Vector3::new(
                TYPE5_TEST_RADIUS_KM * cos_theta,
                TYPE5_TEST_RADIUS_KM * sin_theta,
                0.0,
            ),
            1.0e-8,
        );
        assert_vector_near(
            state.velocity_km_s,
            Vector3::new(-speed_km_s * sin_theta, speed_km_s * cos_theta, 0.0),
            1.0e-11,
        );
    }

    #[test]
    fn spk_type14_state_uses_generic_segment_metadata() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 14,
            data: type14_linear_chebyshev_segment(&[0.0, 3.0, 10.0]),
        };
        let state = segment.state_km_s(7.0).unwrap();
        assert_vector_near(state.position_km, Vector3::new(70.0, 14.0, -7.0), 1.0e-13);
        assert_vector_near(state.velocity_km_s, Vector3::new(10.0, 2.0, -1.0), 1.0e-13);
    }

    #[test]
    fn spk_type18_lagrange_state_uses_packet_window() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 4.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 18,
            data: type18_lagrange_segment(&[0.0, 1.0, 2.0, 4.0], 2),
        };
        let state = segment.state_km_s(3.0).unwrap();
        assert_eq!(state.position_km, Vector3::new(30.0, 6.0, -3.0));
        assert_eq!(state.velocity_km_s, Vector3::new(10.0, 2.0, -1.0));
    }

    #[test]
    fn spk_type19_boundary_flag_selects_later_mini_segment() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 19,
            data: type19_piecewise_lagrange_segment(true),
        };
        let state = segment.state_km_s(5.0).unwrap();
        assert_eq!(state.position_km, Vector3::new(150.0, 10.0, -5.0));
        assert_eq!(state.velocity_km_s, Vector3::new(10.0, 2.0, -1.0));
    }

    #[test]
    fn spk_type19_boundary_flag_selects_earlier_mini_segment() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 19,
            data: type19_piecewise_lagrange_segment(false),
        };
        let state = segment.state_km_s(5.0).unwrap();
        assert_eq!(state.position_km, Vector3::new(50.0, 10.0, -5.0));
        assert_eq!(state.velocity_km_s, Vector3::new(10.0, 2.0, -1.0));
    }

    #[test]
    fn spk_type12_state_uses_hermite_position_derivative() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 4.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 12,
            data: type12_quadratic_segment(0.0, 1.0, 5, 2),
        };
        let state = segment.state_km_s(2.5).unwrap();
        assert_vector_near(state.position_km, Vector3::new(6.25, 12.5, -6.25), 1.0e-12);
        assert_vector_near(state.velocity_km_s, Vector3::new(5.0, 10.0, -5.0), 1.0e-12);
    }

    #[test]
    fn spk_type18_hermite_state_interpolates_velocity_packets() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 4.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 18,
            data: type18_hermite_segment(&[0.0, 1.0, 2.0, 4.0], 2),
        };
        let state = segment.state_km_s(3.0).unwrap();
        assert_vector_near(state.position_km, Vector3::new(9.0, 18.0, -9.0), 1.0e-12);
        assert_vector_near(state.velocity_km_s, Vector3::new(6.0, 12.0, -6.0), 1.0e-12);
    }

    #[test]
    fn spk_type19_hermite_subtype0_evaluates_mini_segment() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 4.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 19,
            data: type19_segment(
                &[type18_hermite_segment(&[0.0, 1.0, 2.0, 4.0], 2)],
                &[0.0, 4.0],
                false,
            ),
        };
        let state = segment.state_km_s(3.0).unwrap();
        assert_vector_near(state.position_km, Vector3::new(9.0, 18.0, -9.0), 1.0e-12);
        assert_vector_near(state.velocity_km_s, Vector3::new(6.0, 12.0, -6.0), 1.0e-12);
    }

    #[test]
    fn spk_type19_coupled_hermite_subtype2_uses_state_derivatives() {
        let segment = SpkSegment {
            start_et_s: 0.0,
            stop_et_s: 4.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 19,
            data: type19_segment(
                &[type19_coupled_hermite_mini_segment(
                    &[0.0, 1.0, 2.0, 4.0],
                    2,
                )],
                &[0.0, 4.0],
                false,
            ),
        };
        let state = segment.state_km_s(3.0).unwrap();
        assert_vector_near(state.position_km, Vector3::new(9.0, 18.0, -9.0), 1.0e-12);
        assert_vector_near(state.velocity_km_s, Vector3::new(6.0, 12.0, -6.0), 1.0e-12);
    }

    #[test]
    fn spk_ephemeris_reads_type13_unequal_step_hermite_state() {
        let bytes = synthetic_type13_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(3.0))
            .unwrap();
        assert_vector_near(
            sun.position_eci_m,
            Vector3::new(9_000.0, 18_000.0, -9_000.0),
            1.0e-9,
        );
        assert_vector_near(
            sun.velocity_eci_m_s,
            Vector3::new(6_000.0, 12_000.0, -6_000.0),
            1.0e-9,
        );
    }

    #[test]
    fn spk_type20_state_integrates_velocity_coefficients() {
        let segment = SpkSegment {
            start_et_s: -10.0,
            stop_et_s: 10.0,
            target: NAIF_SUN,
            center: NAIF_SOLAR_SYSTEM_BARYCENTER,
            frame: SPK_J2000_FRAME_ID,
            data_type: 20,
            data: type20_single_record_segment(
                J2000_JULIAN_DATE,
                20.0 / SECONDS_PER_DAY,
                [[2.0, 4.0], [-1.0, 0.0], [0.0, 2.0]],
                [100.0, 10.0, -5.0],
                1.0,
                1.0,
            ),
        };
        let state = segment.state_km_s(15.0).unwrap();
        assert_vector_near(state.position_km, Vector3::new(115.0, 5.0, -2.5), 1.0e-8);
        assert_vector_near(state.velocity_km_s, Vector3::new(4.0, -1.0, 1.0), 1.0e-9);
    }

    #[test]
    fn spk_ephemeris_reads_type20_velocity_chebyshev_state() {
        let bytes = synthetic_type20_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::from_seconds(10.0))
            .unwrap();
        assert_vector_near(
            sun.position_eci_m,
            Vector3::new(149_597_870.0e3, 0.0, 0.0),
            1.0e-4,
        );
        assert_vector_near(
            sun.velocity_eci_m_s,
            Vector3::new(0.0, 29_780.0, 0.0),
            1.0e-8,
        );
    }

    #[test]
    fn spk_ephemeris_rotates_eclipj2000_segments_to_j2000() {
        let bytes = synthetic_eclipj2000_spk();
        let ephemeris = SpkEphemeris::from_bytes(J2000_JULIAN_DATE, &bytes).unwrap();
        let sun = ephemeris
            .body_state_eci_m_s(CelestialBody::Sun, SimTime::ZERO)
            .unwrap();
        let (sin_eps, cos_eps) = J2000_ECLIPTIC_OBLIQUITY_RAD.sin_cos();
        assert_vector_near(
            sun.position_eci_m,
            Vector3::new(0.0, -sin_eps * 1_000.0, cos_eps * 1_000.0),
            1.0e-12,
        );
        assert_vector_near(
            sun.velocity_eci_m_s,
            Vector3::new(0.0, cos_eps * 1_000.0, sin_eps * 1_000.0),
            1.0e-12,
        );
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

    #[derive(Clone)]
    struct SyntheticSegment {
        target: i32,
        center: i32,
        frame: i32,
        data_type: i32,
        data: Vec<f64>,
    }

    fn synthetic_spk() -> Vec<u8> {
        synthetic_spk_with_sun_x_km(149_597_870.0)
    }

    fn synthetic_spk_with_sun_x_km(sun_x_km: f64) -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: 3,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([4_700.0, 1_200.0, -300.0]),
            },
            SyntheticSegment {
                target: NAIF_EARTH,
                center: 3,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([sun_x_km, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: 3,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_eclipj2000_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_ECLIPJ2000_FRAME_ID,
                data_type: 3,
                data: type3_constant_segment([0.0, 0.0, 1.0], [0.0, 1.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type3_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: 3,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 3,
                data: type3_constant_segment([4_700.0, 1_200.0, -300.0], [0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_EARTH,
                center: 3,
                frame: SPK_J2000_FRAME_ID,
                data_type: 3,
                data: type3_constant_segment([0.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 3,
                data: type3_constant_segment([149_597_870.0, 0.0, 0.0], [0.0, 29.78, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: 3,
                frame: SPK_J2000_FRAME_ID,
                data_type: 3,
                data: type3_constant_segment([384_400.0, 0.0, 0.0], [0.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type5_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 5,
                data: type5_circular_segment(&[0.0, 10.0]),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type8_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 8,
                data: type8_linear_segment(0.0, 1.0, 5, 1),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type9_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 9,
                data: type9_linear_segment(&[0.0, 1.0, 2.0, 4.0], 1),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type14_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 14,
                data: type14_linear_chebyshev_segment(&[0.0, 3.0, 10.0]),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type15_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 15,
                data: type15_circular_segment(3.0, 0.0),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type18_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 18,
                data: type18_lagrange_segment(&[0.0, 1.0, 2.0, 4.0], 2),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type19_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 19,
                data: type19_piecewise_lagrange_segment(true),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type13_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 13,
                data: type13_quadratic_segment(&[0.0, 1.0, 2.0, 4.0], 2),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_type20_spk() -> Vec<u8> {
        let segments = [
            SyntheticSegment {
                target: NAIF_EARTH,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([0.0, 0.0, 0.0]),
            },
            SyntheticSegment {
                target: NAIF_SUN,
                center: NAIF_SOLAR_SYSTEM_BARYCENTER,
                frame: SPK_J2000_FRAME_ID,
                data_type: 20,
                data: type20_single_record_segment(
                    J2000_JULIAN_DATE,
                    20.0 / SECONDS_PER_DAY,
                    [[0.0], [29.78], [0.0]],
                    [149_597_870.0, 0.0, 0.0],
                    1.0,
                    1.0,
                ),
            },
            SyntheticSegment {
                target: NAIF_MOON,
                center: NAIF_EARTH,
                frame: SPK_J2000_FRAME_ID,
                data_type: 2,
                data: type2_constant_segment([384_400.0, 0.0, 0.0]),
            },
        ];
        synthetic_spk_from_segments(&segments)
    }

    fn synthetic_spk_from_segments(segments: &[SyntheticSegment]) -> Vec<u8> {
        let record_count = 4 + segments.len();
        let mut bytes = vec![0_u8; record_count * DAF_RECORD_BYTES];
        write_file_record(&mut bytes);
        write_summary_record(&mut bytes, segments);
        for (index, segment) in segments.iter().enumerate() {
            let address = data_address(index);
            write_f64_data(&mut bytes, address, &segment.data);
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
            write_i32(bytes, offset + 24, segment.frame);
            write_i32(bytes, offset + 28, segment.data_type);
            write_i32(bytes, offset + 32, data_address(index));
            write_i32(
                bytes,
                offset + 36,
                data_address(index) + segment.data.len() as i32 - 1,
            );
        }
    }

    fn data_address(index: usize) -> i32 {
        (DAF_DOUBLE_WORDS_PER_RECORD * (3 + index) + 1) as i32
    }

    fn type2_constant_segment(position_km: [f64; 3]) -> Vec<f64> {
        vec![
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

    fn type3_constant_segment(position_km: [f64; 3], velocity_km_s: [f64; 3]) -> Vec<f64> {
        vec![
            0.0,
            10.0,
            position_km[0],
            position_km[1],
            position_km[2],
            velocity_km_s[0],
            velocity_km_s[1],
            velocity_km_s[2],
            -10.0,
            20.0,
            8.0,
            1.0,
        ]
    }

    fn type5_circular_segment(epochs: &[f64]) -> Vec<f64> {
        let mut data = Vec::new();
        for epoch in epochs {
            data.extend_from_slice(&type5_circular_state(*epoch));
        }
        data.extend_from_slice(epochs);
        append_spk_type5_epoch_directory(&mut data, epochs);
        data.push(TYPE5_TEST_GM_KM3_S2);
        data.push(epochs.len() as f64);
        data
    }

    fn type5_circular_state(epoch_s: f64) -> [f64; 6] {
        let mean_motion_rad_s = (TYPE5_TEST_GM_KM3_S2 / TYPE5_TEST_RADIUS_KM.powi(3)).sqrt();
        let theta = mean_motion_rad_s * epoch_s;
        let (sin_theta, cos_theta) = theta.sin_cos();
        let speed_km_s = mean_motion_rad_s * TYPE5_TEST_RADIUS_KM;
        [
            TYPE5_TEST_RADIUS_KM * cos_theta,
            TYPE5_TEST_RADIUS_KM * sin_theta,
            0.0,
            -speed_km_s * sin_theta,
            speed_km_s * cos_theta,
            0.0,
        ]
    }

    fn type8_linear_segment(
        first_epoch_s: f64,
        step_s: f64,
        count: usize,
        degree: usize,
    ) -> Vec<f64> {
        let mut data = Vec::new();
        for index in 0..count {
            let epoch = first_epoch_s + step_s * index as f64;
            data.extend_from_slice(&[10.0 * epoch, 2.0 * epoch, -epoch, 10.0, 2.0, -1.0]);
        }
        data.push(first_epoch_s);
        data.push(step_s);
        data.push(degree as f64);
        data.push(count as f64);
        data
    }

    fn type9_linear_segment(epochs: &[f64], degree: usize) -> Vec<f64> {
        let mut data = Vec::new();
        for epoch in epochs {
            data.extend_from_slice(&[10.0 * *epoch, 2.0 * *epoch, -*epoch, 10.0, 2.0, -1.0]);
        }
        data.extend_from_slice(epochs);
        append_spk_epoch_directory(&mut data, epochs);
        data.push(degree as f64);
        data.push(epochs.len() as f64);
        data
    }

    fn type14_linear_chebyshev_segment(boundaries: &[f64]) -> Vec<f64> {
        assert!(boundaries.len() >= 2);
        let references = &boundaries[..boundaries.len() - 1];
        let coefficient_count = 2_usize;
        let packet_size = 2 + 6 * coefficient_count;
        let packet_offset = 1_usize;
        let constant_count = 1_usize;
        let mut data = vec![coefficient_count as f64];
        for window in boundaries.windows(2) {
            data.push(window[0]);
            append_type14_linear_packet(&mut data, window[0], window[1]);
        }
        data.extend_from_slice(references);
        append_spk_epoch_directory(&mut data, references);

        let packet_count = references.len();
        let reference_directory_count = (references.len() - 1) / 100;
        let packet_base = constant_count;
        let reference_base = packet_base + packet_count * (packet_size + packet_offset);
        let reference_directory_base = reference_base + references.len();
        let metadata = [
            0.0,
            constant_count as f64,
            reference_directory_base as f64,
            reference_directory_count as f64,
            3.0,
            reference_base as f64,
            references.len() as f64,
            0.0,
            0.0,
            0.0,
            packet_base as f64,
            packet_count as f64,
            0.0,
            0.0,
            packet_size as f64,
            packet_offset as f64,
            17.0,
        ];
        data.extend_from_slice(&metadata);
        data
    }

    fn append_type14_linear_packet(data: &mut Vec<f64>, start_epoch_s: f64, stop_epoch_s: f64) {
        let midpoint_s = 0.5 * (start_epoch_s + stop_epoch_s);
        let radius_s = 0.5 * (stop_epoch_s - start_epoch_s);
        data.extend_from_slice(&[
            midpoint_s,
            radius_s,
            10.0 * midpoint_s,
            10.0 * radius_s,
            2.0 * midpoint_s,
            2.0 * radius_s,
            -midpoint_s,
            -radius_s,
            10.0,
            0.0,
            2.0,
            0.0,
            -1.0,
            0.0,
        ]);
    }

    fn type15_circular_segment(j2_flag: f64, j2: f64) -> Vec<f64> {
        vec![
            0.0,
            0.0,
            0.0,
            1.0,
            1.0,
            0.0,
            0.0,
            TYPE5_TEST_RADIUS_KM,
            0.0,
            j2_flag,
            0.0,
            0.0,
            1.0,
            TYPE5_TEST_GM_KM3_S2,
            j2,
            TYPE15_TEST_CENTRAL_RADIUS_KM,
        ]
    }

    fn type18_lagrange_segment(epochs: &[f64], window_size: usize) -> Vec<f64> {
        type18_lagrange_segment_with_offset(epochs, window_size, 0.0)
    }

    fn type18_lagrange_segment_with_offset(
        epochs: &[f64],
        window_size: usize,
        x_offset_km: f64,
    ) -> Vec<f64> {
        let mut data = Vec::new();
        for epoch in epochs {
            data.extend_from_slice(&[
                x_offset_km + 10.0 * *epoch,
                2.0 * *epoch,
                -*epoch,
                10.0,
                2.0,
                -1.0,
            ]);
        }
        append_type18_directory(&mut data, epochs, 1, window_size);
        data
    }

    fn type18_hermite_segment(epochs: &[f64], window_size: usize) -> Vec<f64> {
        let mut data = Vec::new();
        for epoch in epochs {
            append_type18_quadratic_packet(&mut data, *epoch);
        }
        append_type18_directory(&mut data, epochs, 0, window_size);
        data
    }

    fn type19_piecewise_lagrange_segment(use_later_boundary: bool) -> Vec<f64> {
        type19_segment(
            &[
                type18_lagrange_segment_with_offset(&[0.0, 2.0, 5.0], 2, 0.0),
                type18_lagrange_segment_with_offset(&[5.0, 7.0, 10.0], 2, 100.0),
            ],
            &[0.0, 5.0, 10.0],
            use_later_boundary,
        )
    }

    fn type19_coupled_hermite_mini_segment(epochs: &[f64], window_size: usize) -> Vec<f64> {
        let mut data = Vec::new();
        for epoch in epochs {
            append_quadratic_state(&mut data, *epoch);
        }
        data.extend_from_slice(epochs);
        append_spk_epoch_directory(&mut data, epochs);
        data.push(2.0);
        data.push(window_size as f64);
        data.push(epochs.len() as f64);
        data
    }

    fn type19_segment(
        mini_segments: &[Vec<f64>],
        boundaries: &[f64],
        use_later_boundary: bool,
    ) -> Vec<f64> {
        assert_eq!(boundaries.len(), mini_segments.len() + 1);
        let mut data = Vec::new();
        let mut pointers = Vec::new();
        for mini_segment in mini_segments {
            pointers.push(data.len() + 1);
            data.extend_from_slice(mini_segment);
        }
        pointers.push(data.len() + 1);
        data.extend_from_slice(boundaries);
        append_type19_interval_directory(&mut data, boundaries);
        data.extend(pointers.into_iter().map(|pointer| pointer as f64));
        data.push(if use_later_boundary { 1.0 } else { 0.0 });
        data.push(mini_segments.len() as f64);
        data
    }

    fn append_type18_quadratic_packet(data: &mut Vec<f64>, epoch: f64) {
        data.extend_from_slice(&[
            epoch * epoch,
            2.0 * epoch * epoch,
            -epoch * epoch,
            2.0 * epoch,
            4.0 * epoch,
            -2.0 * epoch,
            2.0 * epoch,
            4.0 * epoch,
            -2.0 * epoch,
            2.0,
            4.0,
            -2.0,
        ]);
    }

    fn append_type18_directory(
        data: &mut Vec<f64>,
        epochs: &[f64],
        subtype: i32,
        window_size: usize,
    ) {
        data.extend_from_slice(epochs);
        append_spk_epoch_directory(data, epochs);
        data.push(f64::from(subtype));
        data.push(window_size as f64);
        data.push(epochs.len() as f64);
    }

    fn type12_quadratic_segment(
        first_epoch_s: f64,
        step_s: f64,
        count: usize,
        window_size: usize,
    ) -> Vec<f64> {
        let mut data = Vec::new();
        for index in 0..count {
            let epoch = first_epoch_s + step_s * index as f64;
            append_quadratic_state(&mut data, epoch);
        }
        data.push(first_epoch_s);
        data.push(step_s);
        data.push((window_size - 1) as f64);
        data.push(count as f64);
        data
    }

    fn type13_quadratic_segment(epochs: &[f64], window_size: usize) -> Vec<f64> {
        let mut data = Vec::new();
        for epoch in epochs {
            append_quadratic_state(&mut data, *epoch);
        }
        data.extend_from_slice(epochs);
        append_spk_epoch_directory(&mut data, epochs);
        data.push((window_size - 1) as f64);
        data.push(epochs.len() as f64);
        data
    }

    fn append_quadratic_state(data: &mut Vec<f64>, epoch: f64) {
        data.extend_from_slice(&[
            epoch * epoch,
            2.0 * epoch * epoch,
            -epoch * epoch,
            2.0 * epoch,
            4.0 * epoch,
            -2.0 * epoch,
        ]);
    }

    fn append_spk_epoch_directory(data: &mut Vec<f64>, epochs: &[f64]) {
        for one_based_index in (100..epochs.len()).step_by(100) {
            data.push(epochs[one_based_index - 1]);
        }
    }

    fn append_spk_type5_epoch_directory(data: &mut Vec<f64>, epochs: &[f64]) {
        for one_based_index in (100..=epochs.len()).step_by(100) {
            data.push(epochs[one_based_index - 1]);
        }
    }

    fn append_type19_interval_directory(data: &mut Vec<f64>, boundaries: &[f64]) {
        let interval_count = boundaries.len() - 1;
        for one_based_index in (100..=interval_count).step_by(100) {
            data.push(boundaries[one_based_index - 1]);
        }
    }

    fn type20_single_record_segment<const N: usize>(
        init_julian_date: f64,
        interval_len_days: f64,
        velocity_coefficients: [[f64; N]; 3],
        midpoint_position: [f64; 3],
        distance_scale_km: f64,
        time_scale_s: f64,
    ) -> Vec<f64> {
        let mut data = Vec::new();
        for component in 0..3 {
            data.extend_from_slice(&velocity_coefficients[component]);
            data.push(midpoint_position[component] / distance_scale_km);
        }
        data.push(distance_scale_km);
        data.push(time_scale_s);
        data.push(init_julian_date);
        data.push(0.0);
        data.push(interval_len_days);
        data.push((3 + 3 * N) as f64);
        data.push(1.0);
        data
    }

    fn assert_vector_near(actual: Vector3<f64>, expected: Vector3<f64>, tolerance: f64) {
        let delta = (actual - expected).norm();
        assert!(
            delta <= tolerance,
            "actual {actual:?} expected {expected:?} delta {delta}"
        );
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
