//! FC-portable control-structure-interaction margin helpers.
//!
//! The functions in this module consume plain continuous-time coefficient
//! arrays and frequency grids. They do not read clocks, allocate output
//! buffers, or depend on simulator, runner, scenario, telemetry, structural, or
//! feed-system crates.

use crate::filters::{BiquadSection, NotchConfig};

#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Complex scalar used for frozen-time frequency responses.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ComplexResponse {
    /// Real component.
    pub re: f64,
    /// Imaginary component.
    pub im: f64,
}

impl ComplexResponse {
    /// Construct a complex response value.
    #[must_use]
    pub const fn new(re: f64, im: f64) -> Self {
        Self { re, im }
    }

    /// Add two complex response values.
    #[must_use]
    pub const fn add(self, rhs: Self) -> Self {
        Self {
            re: self.re + rhs.re,
            im: self.im + rhs.im,
        }
    }

    /// Multiply two complex response values.
    #[must_use]
    pub const fn mul(self, rhs: Self) -> Self {
        Self {
            re: self.re * rhs.re - self.im * rhs.im,
            im: self.re * rhs.im + self.im * rhs.re,
        }
    }

    /// Divide two complex response values.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when the denominator is singular or the quotient is
    /// non-finite.
    pub fn divide_by(self, rhs: Self) -> Result<Self, CsiError> {
        let denom = rhs.norm_sqr();
        if denom <= f64::MIN_POSITIVE {
            return Err(CsiError::InvalidNumber {
                field: "complex.denominator",
            });
        }
        let value = Self {
            re: (self.re * rhs.re + self.im * rhs.im) / denom,
            im: (self.im * rhs.re - self.re * rhs.im) / denom,
        };
        require_finite_complex("complex.quotient", value)?;
        Ok(value)
    }

    /// Squared magnitude.
    #[must_use]
    pub const fn norm_sqr(self) -> f64 {
        self.re * self.re + self.im * self.im
    }

    /// Magnitude.
    #[must_use]
    pub fn magnitude(self) -> f64 {
        self.norm_sqr().sqrt()
    }

    /// Phase in degrees in the usual `[-180, 180]` branch.
    #[must_use]
    pub fn phase_deg(self) -> f64 {
        self.im.atan2(self.re).to_degrees()
    }

    /// Phase in degrees, normalized for negative-feedback margin checks.
    #[must_use]
    pub fn phase_deg_negative_feedback(self) -> f64 {
        let mut phase_deg = self.phase_deg();
        if phase_deg > 0.0 {
            phase_deg -= 360.0;
        }
        phase_deg
    }
}

/// Continuous-time frozen plant represented by coefficient arrays.
///
/// Coefficients are in ascending powers of `s`:
/// `c[0] + c[1] s + c[2] s^2 + ...`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrozenPlant<'a> {
    numerator: &'a [f64],
    denominator: &'a [f64],
}

impl<'a> FrozenPlant<'a> {
    /// Construct a validated frozen open-loop plant.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when coefficient arrays are empty, non-finite, or
    /// all zero.
    pub fn new(numerator: &'a [f64], denominator: &'a [f64]) -> Result<Self, CsiError> {
        validate_coefficients("plant.numerator", numerator)?;
        validate_coefficients("plant.denominator", denominator)?;
        Ok(Self {
            numerator,
            denominator,
        })
    }

    /// Numerator coefficients in ascending powers of `s`.
    #[must_use]
    pub const fn numerator(&self) -> &'a [f64] {
        self.numerator
    }

    /// Denominator coefficients in ascending powers of `s`.
    #[must_use]
    pub const fn denominator(&self) -> &'a [f64] {
        self.denominator
    }

    /// Evaluate the open-loop response at `s = j omega`.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when `omega_rad_s` is invalid or the denominator
    /// evaluates to zero.
    pub fn response_at(&self, omega_rad_s: f64) -> Result<ComplexResponse, CsiError> {
        require_positive_finite("plant.omega_rad_s", omega_rad_s)?;
        let numerator = polynomial_jw(self.numerator, omega_rad_s)?;
        let denominator = polynomial_jw(self.denominator, omega_rad_s)?;
        numerator.divide_by(denominator)
    }
}

/// One frozen-time plant sample in an LTV CSI sweep.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct FrozenTrajectoryPoint<'a> {
    /// Time on the caller-owned trajectory grid, s.
    pub time_s: f64,
    /// Frozen open-loop plant at this time.
    pub plant: FrozenPlant<'a>,
}

impl<'a> FrozenTrajectoryPoint<'a> {
    /// Construct and validate one frozen-time trajectory point.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when `time_s` is negative or non-finite.
    pub fn new(time_s: f64, plant: FrozenPlant<'a>) -> Result<Self, CsiError> {
        require_nonnegative_finite("trajectory.time_s", time_s)?;
        Ok(Self { time_s, plant })
    }
}

/// Minimum margin criteria for sampled CSI analysis.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsiMarginCriteria {
    /// Required gain margin, dB.
    pub min_gain_margin_db: f64,
    /// Required phase margin, deg.
    pub min_phase_margin_deg: f64,
}

impl CsiMarginCriteria {
    /// Construct and validate sampled-margin criteria.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when either threshold is negative or non-finite.
    pub fn new(min_gain_margin_db: f64, min_phase_margin_deg: f64) -> Result<Self, CsiError> {
        require_nonnegative_finite("criteria.min_gain_margin_db", min_gain_margin_db)?;
        require_nonnegative_finite("criteria.min_phase_margin_deg", min_phase_margin_deg)?;
        Ok(Self {
            min_gain_margin_db,
            min_phase_margin_deg,
        })
    }
}

/// Verdict against sampled CSI margin criteria.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CsiMarginVerdict {
    /// Sampled gain and phase margins meet or exceed the requested criteria.
    MeetsCriteria,
    /// One or more sampled margins are missing or below the requested criteria.
    BelowCriteria,
}

/// Sampled CSI margin report for one frozen-time plant.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsiMarginReport {
    /// Gain margin at sampled phase crossover, dB.
    pub gain_margin_db: Option<f64>,
    /// Phase margin at sampled gain crossover, deg.
    pub phase_margin_deg: Option<f64>,
    /// Frequency where the sampled gain crossover was found, rad/s.
    pub gain_crossover_frequency_rad_s: Option<f64>,
    /// Frequency where the sampled phase crossover was found, rad/s.
    pub phase_crossover_frequency_rad_s: Option<f64>,
    /// Minimum sampled distance to the Nyquist critical point `-1 + j0`.
    pub min_distance_to_minus_one: f64,
    /// Frequency for [`Self::min_distance_to_minus_one`], rad/s.
    pub min_distance_frequency_rad_s: f64,
    /// Peak sampled open-loop gain, dB.
    pub peak_gain_db: f64,
    /// Frequency for [`Self::peak_gain_db`], rad/s.
    pub peak_gain_frequency_rad_s: f64,
    /// Verdict against the requested criteria.
    pub verdict: CsiMarginVerdict,
}

/// Worst-case report for a caller-owned frozen-time trajectory sweep.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CsiTrajectoryReport {
    /// Number of frozen-time points evaluated.
    pub point_count: usize,
    /// Worst finite sampled gain margin across the trajectory, dB.
    /// `None` means at least one point had no sampled phase crossover.
    pub worst_gain_margin_db: Option<f64>,
    /// Time of [`Self::worst_gain_margin_db`], s.
    pub worst_gain_margin_time_s: Option<f64>,
    /// Worst finite sampled phase margin across the trajectory, deg.
    /// `None` means at least one point had no sampled gain crossover.
    pub worst_phase_margin_deg: Option<f64>,
    /// Time of [`Self::worst_phase_margin_deg`], s.
    pub worst_phase_margin_time_s: Option<f64>,
    /// Minimum sampled distance to the Nyquist critical point across all
    /// trajectory points.
    pub min_distance_to_minus_one: f64,
    /// Time for [`Self::min_distance_to_minus_one`], s.
    pub min_distance_time_s: f64,
    /// Frequency for [`Self::min_distance_to_minus_one`], rad/s.
    pub min_distance_frequency_rad_s: f64,
    /// Peak sampled open-loop gain across the trajectory, dB.
    pub peak_gain_db: f64,
    /// Time for [`Self::peak_gain_db`], s.
    pub peak_gain_time_s: f64,
    /// Frequency for [`Self::peak_gain_db`], rad/s.
    pub peak_gain_frequency_rad_s: f64,
    /// Verdict against the requested criteria across every frozen-time point.
    pub verdict: CsiMarginVerdict,
}

/// One modal path used to assemble a continuous CSI open-loop plant.
///
/// The modal coordinate follows
/// `q_ddot + 2 zeta omega q_dot + omega^2 q = input_coupling / modal_mass * u`.
/// The sensed output uses `displacement*q + rate*q_dot + acceleration*q_ddot`,
/// which covers structural rate-gyro and accelerometer pickup without tying
/// this FC-portable helper to the vehicle or runner crates.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct ModalCsiMode {
    /// Modal natural frequency, rad/s.
    pub natural_frequency_rad_s: f64,
    /// Modal damping ratio.
    pub damping_ratio: f64,
    /// Generalized modal mass, kg.
    pub modal_mass_kg: f64,
    /// Input-to-modal generalized-force coupling.
    pub input_coupling: f64,
    /// Sensor displacement coupling multiplying `q`.
    pub sensor_displacement_coupling: f64,
    /// Sensor rate coupling multiplying `q_dot`.
    pub sensor_rate_coupling: f64,
    /// Sensor acceleration coupling multiplying `q_ddot`.
    pub sensor_acceleration_coupling: f64,
}

impl ModalCsiMode {
    /// Construct one validated modal CSI path.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when any modal scalar is invalid.
    pub fn new(
        natural_frequency_rad_s: f64,
        damping_ratio: f64,
        modal_mass_kg: f64,
        input_coupling: f64,
        sensor_displacement_coupling: f64,
        sensor_rate_coupling: f64,
        sensor_acceleration_coupling: f64,
    ) -> Result<Self, CsiError> {
        let mode = Self {
            natural_frequency_rad_s,
            damping_ratio,
            modal_mass_kg,
            input_coupling,
            sensor_displacement_coupling,
            sensor_rate_coupling,
            sensor_acceleration_coupling,
        };
        require_modal_csi_mode(mode)?;
        Ok(mode)
    }
}

/// Assemble a common-denominator continuous modal CSI plant.
///
/// Coefficients are written in ascending powers of `s` and can be passed to
/// [`FrozenPlant::new`] as `&numerator_out[..count]` and
/// `&denominator_out[..count]`. The output and scratch buffers must all have at
/// least `2 * modes.len() + 1` entries. The function uses no heap allocation.
///
/// # Errors
///
/// Returns [`CsiError`] when inputs are invalid, output buffers are too short,
/// or every modal path cancels to a zero numerator.
pub fn assemble_modal_csi_plant_coefficients(
    modes: &[ModalCsiMode],
    loop_gain: f64,
    numerator_out: &mut [f64],
    denominator_out: &mut [f64],
    scratch_out: &mut [f64],
) -> Result<usize, CsiError> {
    let coefficient_count = modal_csi_coefficient_count(modes)?;
    require_finite("modal_csi.loop_gain", loop_gain)?;
    require_modal_output_buffer("modal_csi.numerator_out", numerator_out, coefficient_count)?;
    require_modal_output_buffer(
        "modal_csi.denominator_out",
        denominator_out,
        coefficient_count,
    )?;
    require_modal_output_buffer("modal_csi.scratch_out", scratch_out, coefficient_count)?;
    for &mode in modes {
        require_modal_csi_mode(mode)?;
    }

    numerator_out.fill(0.0);
    denominator_out.fill(0.0);
    scratch_out.fill(0.0);

    denominator_out[0] = 1.0;
    let mut denominator_len = 1;
    for &mode in modes {
        multiply_by_modal_denominator(denominator_out, denominator_len, mode);
        denominator_len += 2;
    }

    for (mode_index, &mode) in modes.iter().enumerate() {
        scratch_out.fill(0.0);
        scratch_out[0] = 1.0;
        let mut scratch_len = 1;
        for (other_index, &other) in modes.iter().enumerate() {
            if other_index == mode_index {
                continue;
            }
            multiply_by_modal_denominator(scratch_out, scratch_len, other);
            scratch_len += 2;
        }
        let modal_gain = loop_gain * mode.input_coupling / mode.modal_mass_kg;
        for index in 0..scratch_len {
            let base = modal_gain * scratch_out[index];
            numerator_out[index] += base * mode.sensor_displacement_coupling;
            numerator_out[index + 1] += base * mode.sensor_rate_coupling;
            numerator_out[index + 2] += base * mode.sensor_acceleration_coupling;
        }
    }

    for value in numerator_out[..coefficient_count]
        .iter()
        .chain(denominator_out[..coefficient_count].iter())
    {
        require_finite("modal_csi.coefficient", *value)?;
    }
    if numerator_out[..coefficient_count]
        .iter()
        .all(|value| value.abs() <= 0.0)
    {
        return Err(CsiError::InvalidNumber {
            field: "modal_csi.coupling",
        });
    }
    Ok(coefficient_count)
}

/// Continuous second-order bending notch section.
///
/// `H(s) = (s^2 + 2 zeta_z omega s + omega^2) /
///         (s^2 + 2 zeta_p omega s + omega^2)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BendingNotch {
    /// Shared notch center frequency, rad/s.
    pub center_frequency_rad_s: f64,
    /// Numerator damping ratio.
    pub zero_damping_ratio: f64,
    /// Denominator damping ratio.
    pub pole_damping_ratio: f64,
}

/// Continuous first-order lead/lag section.
///
/// `H(s) = gain * (1 + s / omega_z) / (1 + s / omega_p)`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct LeadLagSection {
    /// Zero frequency, rad/s.
    pub zero_frequency_rad_s: f64,
    /// Pole frequency, rad/s.
    pub pole_frequency_rad_s: f64,
    /// Scalar gain.
    pub gain: f64,
}

impl LeadLagSection {
    /// Evaluate the continuous lead/lag response at `s = j omega`.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when section fields or `omega_rad_s` are invalid.
    pub fn response_at(&self, omega_rad_s: f64) -> Result<ComplexResponse, CsiError> {
        require_positive_finite("lead_lag.omega_rad_s", omega_rad_s)?;
        require_lead_lag_fields(*self)?;
        let numerator = ComplexResponse::new(1.0, omega_rad_s / self.zero_frequency_rad_s);
        let denominator = ComplexResponse::new(1.0, omega_rad_s / self.pole_frequency_rad_s);
        let response = ComplexResponse::new(self.gain, 0.0).mul(numerator.divide_by(denominator)?);
        require_finite_complex("lead_lag.response", response)?;
        Ok(response)
    }

    /// Numerator coefficients in ascending powers of `s`.
    #[must_use]
    pub fn numerator_coefficients(self) -> [f64; 2] {
        [self.gain, self.gain / self.zero_frequency_rad_s]
    }

    /// Denominator coefficients in ascending powers of `s`.
    #[must_use]
    pub fn denominator_coefficients(self) -> [f64; 2] {
        [1.0, 1.0 / self.pole_frequency_rad_s]
    }
}

impl BendingNotch {
    /// Evaluate the continuous notch response at `s = j omega`.
    ///
    /// # Errors
    ///
    /// Returns [`CsiError`] when `omega_rad_s` is invalid or the response is
    /// non-finite.
    pub fn response_at(&self, omega_rad_s: f64) -> Result<ComplexResponse, CsiError> {
        require_positive_finite("notch.omega_rad_s", omega_rad_s)?;
        require_notch_fields(*self)?;
        let omega_n = self.center_frequency_rad_s;
        let real = omega_n * omega_n - omega_rad_s * omega_rad_s;
        let numerator =
            ComplexResponse::new(real, 2.0 * self.zero_damping_ratio * omega_n * omega_rad_s);
        let denominator =
            ComplexResponse::new(real, 2.0 * self.pole_damping_ratio * omega_n * omega_rad_s);
        numerator.divide_by(denominator)
    }

    /// Convert the continuous notch into the existing digital-notch
    /// configuration shape.
    #[must_use]
    pub fn notch_config(self) -> NotchConfig {
        let center_hz = self.center_frequency_rad_s / std::f64::consts::TAU;
        let bandwidth_hz =
            (2.0 * self.pole_damping_ratio * self.center_frequency_rad_s) / std::f64::consts::TAU;
        let depth_db = 20.0 * (self.pole_damping_ratio / self.zero_damping_ratio).log10();
        NotchConfig {
            center_hz,
            bandwidth_hz,
            depth_db,
        }
    }
}

/// Analyze sampled frozen-time CSI gain and phase margins.
///
/// Frequencies must be strictly increasing and positive. The function does not
/// allocate; it records the nearest sampled gain crossover and phase crossover.
///
/// # Errors
///
/// Returns [`CsiError`] when the plant, grid, criteria, or sampled response is
/// invalid.
pub fn analyze_frozen_plant(
    plant: &FrozenPlant<'_>,
    frequencies_rad_s: &[f64],
    criteria: CsiMarginCriteria,
) -> Result<CsiMarginReport, CsiError> {
    analyze_open_loop_response(frequencies_rad_s, criteria, |frequency| {
        plant.response_at(frequency)
    })
}

/// Evaluate one discrete biquad section at a continuous angular
/// frequency using `z = exp(j omega dt)`.
///
/// # Errors
///
/// Returns [`CsiError`] when the section is unstable or unnormalized, the
/// frequency/sample interval is invalid, or the frequency is at/above Nyquist.
pub fn biquad_section_response_at(
    section: BiquadSection,
    omega_rad_s: f64,
    dt_s: f64,
) -> Result<ComplexResponse, CsiError> {
    if !section.has_stable_poles() {
        return Err(CsiError::InvalidNumber {
            field: "filter.section",
        });
    }
    require_positive_finite("filter.omega_rad_s", omega_rad_s)?;
    require_positive_finite("filter.dt_s", dt_s)?;
    if omega_rad_s >= std::f64::consts::PI / dt_s {
        return Err(CsiError::InvalidNumber {
            field: "filter.omega_rad_s",
        });
    }

    let theta = omega_rad_s * dt_s;
    let z_inv = ComplexResponse::new(theta.cos(), -theta.sin());
    let z_inv_sq = ComplexResponse::new((2.0 * theta).cos(), -(2.0 * theta).sin());
    let numerator = complex_scale(z_inv_sq, section.b[2])
        .add(complex_scale(z_inv, section.b[1]))
        .add(ComplexResponse::new(section.b[0], 0.0));
    let denominator = complex_scale(z_inv_sq, section.a[2])
        .add(complex_scale(z_inv, section.a[1]))
        .add(ComplexResponse::new(section.a[0], 0.0));
    let response = numerator.divide_by(denominator)?;
    require_finite_complex("filter.response", response)?;
    Ok(response)
}

/// Analyze sampled margins for a continuous frozen plant with a discrete
/// bending/slosh filter cascade inserted in the open loop.
///
/// This is the FC-portable check used by scheduled structural-filter evidence:
/// the plant remains analytic continuous-time modal dynamics, while each
/// filter section is evaluated at `z = exp(j omega dt)`.
///
/// # Errors
///
/// Returns [`CsiError`] when the plant, filter cascade, sample interval, grid,
/// criteria, or sampled response is invalid.
pub fn analyze_filtered_frozen_plant(
    plant: &FrozenPlant<'_>,
    sections: &[BiquadSection],
    dt_s: f64,
    frequencies_rad_s: &[f64],
    criteria: CsiMarginCriteria,
) -> Result<CsiMarginReport, CsiError> {
    if sections.is_empty() {
        return Err(CsiError::DimensionMismatch {
            field: "filter.sections",
            expected: 1,
            actual: 0,
        });
    }
    require_positive_finite("filter.dt_s", dt_s)?;
    for &section in sections {
        if !section.has_stable_poles() {
            return Err(CsiError::InvalidNumber {
                field: "filter.section",
            });
        }
    }

    analyze_open_loop_response(frequencies_rad_s, criteria, |frequency| {
        let mut response = plant.response_at(frequency)?;
        for &section in sections {
            response = response.mul(biquad_section_response_at(section, frequency, dt_s)?);
        }
        Ok(response)
    })
}

fn analyze_open_loop_response<F>(
    frequencies_rad_s: &[f64],
    criteria: CsiMarginCriteria,
    mut response_at: F,
) -> Result<CsiMarginReport, CsiError>
where
    F: FnMut(f64) -> Result<ComplexResponse, CsiError>,
{
    validate_frequency_grid(frequencies_rad_s)?;
    CsiMarginCriteria::new(criteria.min_gain_margin_db, criteria.min_phase_margin_deg)?;

    let mut previous_gain_db = None;
    let mut previous_phase_offset_deg = None;
    let mut nearest_gain = NearestCrossing::default();
    let mut nearest_phase = NearestCrossing::default();
    let mut has_gain_crossing = false;
    let mut has_phase_crossing = false;
    let mut min_distance_to_minus_one = f64::INFINITY;
    let mut min_distance_frequency_rad_s = frequencies_rad_s[0];
    let mut peak_gain_db = f64::NEG_INFINITY;
    let mut peak_gain_frequency_rad_s = frequencies_rad_s[0];

    for &frequency in frequencies_rad_s {
        let response = response_at(frequency)?;
        let magnitude = response.magnitude();
        require_positive_finite("plant.open_loop.magnitude", magnitude)?;
        let gain_db = 20.0 * magnitude.log10();
        let phase_deg = response.phase_deg_negative_feedback();
        let phase_offset_deg = phase_deg + 180.0;
        let distance_to_minus_one = response.add(ComplexResponse::new(1.0, 0.0)).magnitude();

        for value in [gain_db, phase_deg, phase_offset_deg, distance_to_minus_one] {
            require_finite("plant.open_loop.sample", value)?;
        }

        nearest_gain.consider(frequency, gain_db.abs(), gain_db, phase_deg);
        nearest_phase.consider(frequency, phase_offset_deg.abs(), gain_db, phase_deg);

        if gain_db > peak_gain_db {
            peak_gain_db = gain_db;
            peak_gain_frequency_rad_s = frequency;
        }
        if distance_to_minus_one < min_distance_to_minus_one {
            min_distance_to_minus_one = distance_to_minus_one;
            min_distance_frequency_rad_s = frequency;
        }

        if gain_db.abs() <= CROSSING_EPSILON_DB {
            has_gain_crossing = true;
        }
        if phase_offset_deg.abs() <= CROSSING_EPSILON_DEG {
            has_phase_crossing = true;
        }
        if let Some(previous_gain_db) = previous_gain_db
            && crosses_zero(previous_gain_db, gain_db)
        {
            has_gain_crossing = true;
        }
        if let Some(previous_phase_offset_deg) = previous_phase_offset_deg
            && crosses_zero(previous_phase_offset_deg, phase_offset_deg)
        {
            has_phase_crossing = true;
        }

        previous_gain_db = Some(gain_db);
        previous_phase_offset_deg = Some(phase_offset_deg);
    }

    let gain_margin_db = if has_phase_crossing {
        nearest_phase.gain_db.map(|gain_db| -gain_db)
    } else {
        None
    };
    let phase_margin_deg = if has_gain_crossing {
        nearest_gain.phase_deg.map(|phase_deg| phase_deg + 180.0)
    } else {
        None
    };
    let verdict = if gain_margin_db.is_some_and(|margin| margin >= criteria.min_gain_margin_db)
        && phase_margin_deg.is_some_and(|margin| margin >= criteria.min_phase_margin_deg)
    {
        CsiMarginVerdict::MeetsCriteria
    } else {
        CsiMarginVerdict::BelowCriteria
    };

    Ok(CsiMarginReport {
        gain_margin_db,
        phase_margin_deg,
        gain_crossover_frequency_rad_s: if has_gain_crossing {
            nearest_gain.frequency_rad_s
        } else {
            None
        },
        phase_crossover_frequency_rad_s: if has_phase_crossing {
            nearest_phase.frequency_rad_s
        } else {
            None
        },
        min_distance_to_minus_one,
        min_distance_frequency_rad_s,
        peak_gain_db,
        peak_gain_frequency_rad_s,
        verdict,
    })
}

fn complex_scale(value: ComplexResponse, scalar: f64) -> ComplexResponse {
    ComplexResponse::new(value.re * scalar, value.im * scalar)
}

/// Analyze sampled CSI margins across a frozen-time trajectory.
///
/// The trajectory times must be strictly increasing. `reports_out` must have
/// the same length as `trajectory`; it receives the per-point sampled margin
/// reports without allocating an output buffer.
///
/// # Errors
///
/// Returns [`CsiError`] when trajectory samples, frequency grid, criteria, or
/// output dimensions are invalid.
pub fn analyze_frozen_trajectory(
    trajectory: &[FrozenTrajectoryPoint<'_>],
    frequencies_rad_s: &[f64],
    criteria: CsiMarginCriteria,
    reports_out: &mut [CsiMarginReport],
) -> Result<CsiTrajectoryReport, CsiError> {
    if trajectory.is_empty() {
        return Err(CsiError::DimensionMismatch {
            field: "trajectory",
            expected: 1,
            actual: 0,
        });
    }
    if reports_out.len() != trajectory.len() {
        return Err(CsiError::DimensionMismatch {
            field: "trajectory.reports_out",
            expected: trajectory.len(),
            actual: reports_out.len(),
        });
    }
    validate_frequency_grid(frequencies_rad_s)?;
    CsiMarginCriteria::new(criteria.min_gain_margin_db, criteria.min_phase_margin_deg)?;

    let mut previous_time_s = None;
    let mut worst_gain_margin_db = f64::INFINITY;
    let mut worst_gain_margin_time_s = None;
    let mut worst_phase_margin_deg = f64::INFINITY;
    let mut worst_phase_margin_time_s = None;
    let mut all_gain_margins_present = true;
    let mut all_phase_margins_present = true;
    let mut min_distance_to_minus_one = f64::INFINITY;
    let mut min_distance_time_s = trajectory[0].time_s;
    let mut min_distance_frequency_rad_s = frequencies_rad_s[0];
    let mut peak_gain_db = f64::NEG_INFINITY;
    let mut peak_gain_time_s = trajectory[0].time_s;
    let mut peak_gain_frequency_rad_s = frequencies_rad_s[0];
    let mut verdict = CsiMarginVerdict::MeetsCriteria;

    for (point, report_slot) in trajectory.iter().zip(reports_out.iter_mut()) {
        require_nonnegative_finite("trajectory.time_s", point.time_s)?;
        if let Some(previous) = previous_time_s
            && point.time_s <= previous
        {
            return Err(CsiError::InvalidNumber {
                field: "trajectory.time_s",
            });
        }
        previous_time_s = Some(point.time_s);

        let report = analyze_frozen_plant(&point.plant, frequencies_rad_s, criteria)?;
        *report_slot = report;

        if report.verdict == CsiMarginVerdict::BelowCriteria {
            verdict = CsiMarginVerdict::BelowCriteria;
        }
        match report.gain_margin_db {
            Some(margin) => {
                if margin < worst_gain_margin_db {
                    worst_gain_margin_db = margin;
                    worst_gain_margin_time_s = Some(point.time_s);
                }
            }
            None => {
                all_gain_margins_present = false;
                verdict = CsiMarginVerdict::BelowCriteria;
            }
        }
        match report.phase_margin_deg {
            Some(margin) => {
                if margin < worst_phase_margin_deg {
                    worst_phase_margin_deg = margin;
                    worst_phase_margin_time_s = Some(point.time_s);
                }
            }
            None => {
                all_phase_margins_present = false;
                verdict = CsiMarginVerdict::BelowCriteria;
            }
        }
        if report.min_distance_to_minus_one < min_distance_to_minus_one {
            min_distance_to_minus_one = report.min_distance_to_minus_one;
            min_distance_time_s = point.time_s;
            min_distance_frequency_rad_s = report.min_distance_frequency_rad_s;
        }
        if report.peak_gain_db > peak_gain_db {
            peak_gain_db = report.peak_gain_db;
            peak_gain_time_s = point.time_s;
            peak_gain_frequency_rad_s = report.peak_gain_frequency_rad_s;
        }
    }

    Ok(CsiTrajectoryReport {
        point_count: trajectory.len(),
        worst_gain_margin_db: if all_gain_margins_present {
            Some(worst_gain_margin_db)
        } else {
            None
        },
        worst_gain_margin_time_s: if all_gain_margins_present {
            Some(worst_gain_margin_time_s.ok_or(CsiError::InvalidNumber {
                field: "trajectory.gain_margin",
            })?)
        } else {
            None
        },
        worst_phase_margin_deg: if all_phase_margins_present {
            Some(worst_phase_margin_deg)
        } else {
            None
        },
        worst_phase_margin_time_s: if all_phase_margins_present {
            Some(worst_phase_margin_time_s.ok_or(CsiError::InvalidNumber {
                field: "trajectory.phase_margin",
            })?)
        } else {
            None
        },
        min_distance_to_minus_one,
        min_distance_time_s,
        min_distance_frequency_rad_s,
        peak_gain_db,
        peak_gain_time_s,
        peak_gain_frequency_rad_s,
        verdict,
    })
}

/// Design a continuous bending notch centered on one mode.
///
/// The numerator damping must be lower than the denominator damping to create a
/// notch at the mode frequency.
///
/// # Errors
///
/// Returns [`CsiError`] when frequency or damping values are invalid.
pub fn design_bending_notch(
    mode_frequency_rad_s: f64,
    zero_damping_ratio: f64,
    pole_damping_ratio: f64,
) -> Result<BendingNotch, CsiError> {
    let notch = BendingNotch {
        center_frequency_rad_s: mode_frequency_rad_s,
        zero_damping_ratio,
        pole_damping_ratio,
    };
    require_notch_fields(notch)?;
    Ok(notch)
}

/// Design a unity-magnitude first-order lead/lag section at `center_frequency_rad_s`.
///
/// Positive `phase_shift_deg` creates a lead section; negative values create a
/// lag section. The requested phase must be strictly inside `(-90, 90)` degrees.
///
/// # Errors
///
/// Returns [`CsiError`] when the center frequency or phase shift is invalid.
pub fn design_phase_lead_lag(
    center_frequency_rad_s: f64,
    phase_shift_deg: f64,
) -> Result<LeadLagSection, CsiError> {
    require_positive_finite("lead_lag.center_frequency_rad_s", center_frequency_rad_s)?;
    require_finite("lead_lag.phase_shift_deg", phase_shift_deg)?;
    if phase_shift_deg.abs() >= 90.0 {
        return Err(CsiError::InvalidNumber {
            field: "lead_lag.phase_shift_deg",
        });
    }
    if phase_shift_deg.abs() <= f64::EPSILON {
        return Ok(LeadLagSection {
            zero_frequency_rad_s: center_frequency_rad_s,
            pole_frequency_rad_s: center_frequency_rad_s,
            gain: 1.0,
        });
    }

    let phase_rad = phase_shift_deg.abs() * std::f64::consts::PI / 180.0;
    let alpha = (1.0 - phase_rad.sin()) / (1.0 + phase_rad.sin());
    require_positive_finite("lead_lag.alpha", alpha)?;
    let sqrt_alpha = alpha.sqrt();
    let section = if phase_shift_deg > 0.0 {
        LeadLagSection {
            zero_frequency_rad_s: center_frequency_rad_s * sqrt_alpha,
            pole_frequency_rad_s: center_frequency_rad_s / sqrt_alpha,
            gain: sqrt_alpha,
        }
    } else {
        LeadLagSection {
            zero_frequency_rad_s: center_frequency_rad_s / sqrt_alpha,
            pole_frequency_rad_s: center_frequency_rad_s * sqrt_alpha,
            gain: 1.0 / sqrt_alpha,
        }
    };
    require_lead_lag_fields(section)?;
    Ok(section)
}

/// CSI validation error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CsiError {
    /// A scalar field was non-finite or outside the accepted range.
    InvalidNumber {
        /// Field path.
        field: &'static str,
    },
    /// An input slice had the wrong length.
    DimensionMismatch {
        /// Field path.
        field: &'static str,
        /// Expected minimum or exact length.
        expected: usize,
        /// Actual length.
        actual: usize,
    },
}

#[derive(Clone, Copy, Debug, Default)]
struct NearestCrossing {
    frequency_rad_s: Option<f64>,
    error: f64,
    gain_db: Option<f64>,
    phase_deg: Option<f64>,
}

impl NearestCrossing {
    fn consider(&mut self, frequency_rad_s: f64, error: f64, gain_db: f64, phase_deg: f64) {
        if self.frequency_rad_s.is_none() || error < self.error {
            self.frequency_rad_s = Some(frequency_rad_s);
            self.error = error;
            self.gain_db = Some(gain_db);
            self.phase_deg = Some(phase_deg);
        }
    }
}

const CROSSING_EPSILON_DB: f64 = 1.0e-9;
const CROSSING_EPSILON_DEG: f64 = 1.0e-9;

fn polynomial_jw(coefficients: &[f64], omega_rad_s: f64) -> Result<ComplexResponse, CsiError> {
    let s = ComplexResponse::new(0.0, omega_rad_s);
    let mut out = ComplexResponse::new(0.0, 0.0);
    for &coefficient in coefficients.iter().rev() {
        out = out.mul(s).add(ComplexResponse::new(coefficient, 0.0));
    }
    require_finite_complex("polynomial.response", out)?;
    Ok(out)
}

fn validate_coefficients(field: &'static str, coefficients: &[f64]) -> Result<(), CsiError> {
    if coefficients.is_empty() {
        return Err(CsiError::DimensionMismatch {
            field,
            expected: 1,
            actual: 0,
        });
    }
    let mut has_nonzero = false;
    for &coefficient in coefficients {
        require_finite(field, coefficient)?;
        has_nonzero |= coefficient.abs() > 0.0;
    }
    if !has_nonzero {
        return Err(CsiError::InvalidNumber { field });
    }
    Ok(())
}

fn validate_frequency_grid(frequencies_rad_s: &[f64]) -> Result<(), CsiError> {
    if frequencies_rad_s.len() < 2 {
        return Err(CsiError::DimensionMismatch {
            field: "frequency_grid",
            expected: 2,
            actual: frequencies_rad_s.len(),
        });
    }
    let mut previous = None;
    for &frequency in frequencies_rad_s {
        require_positive_finite("frequency_grid", frequency)?;
        if let Some(previous) = previous
            && frequency <= previous
        {
            return Err(CsiError::InvalidNumber {
                field: "frequency_grid",
            });
        }
        previous = Some(frequency);
    }
    Ok(())
}

fn modal_csi_coefficient_count(modes: &[ModalCsiMode]) -> Result<usize, CsiError> {
    if modes.is_empty() {
        return Err(CsiError::DimensionMismatch {
            field: "modal_csi.modes",
            expected: 1,
            actual: 0,
        });
    }
    Ok((2 * modes.len()) + 1)
}

fn require_modal_output_buffer(
    field: &'static str,
    values: &[f64],
    coefficient_count: usize,
) -> Result<(), CsiError> {
    if values.len() < coefficient_count {
        return Err(CsiError::DimensionMismatch {
            field,
            expected: coefficient_count,
            actual: values.len(),
        });
    }
    Ok(())
}

fn require_modal_csi_mode(mode: ModalCsiMode) -> Result<(), CsiError> {
    require_positive_finite(
        "modal_csi.mode.natural_frequency_rad_s",
        mode.natural_frequency_rad_s,
    )?;
    require_nonnegative_finite("modal_csi.mode.damping_ratio", mode.damping_ratio)?;
    require_positive_finite("modal_csi.mode.modal_mass_kg", mode.modal_mass_kg)?;
    require_finite("modal_csi.mode.input_coupling", mode.input_coupling)?;
    require_finite(
        "modal_csi.mode.sensor_displacement_coupling",
        mode.sensor_displacement_coupling,
    )?;
    require_finite(
        "modal_csi.mode.sensor_rate_coupling",
        mode.sensor_rate_coupling,
    )?;
    require_finite(
        "modal_csi.mode.sensor_acceleration_coupling",
        mode.sensor_acceleration_coupling,
    )
}

fn multiply_by_modal_denominator(coefficients: &mut [f64], active_len: usize, mode: ModalCsiMode) {
    coefficients[active_len] = 0.0;
    coefficients[active_len + 1] = 0.0;
    let omega = mode.natural_frequency_rad_s;
    let modal_denominator = [omega * omega, 2.0 * mode.damping_ratio * omega, 1.0];
    for index in (0..active_len).rev() {
        let coefficient = coefficients[index];
        coefficients[index + 2] += coefficient * modal_denominator[2];
        coefficients[index + 1] += coefficient * modal_denominator[1];
        coefficients[index] = coefficient * modal_denominator[0];
    }
}

fn require_notch_fields(notch: BendingNotch) -> Result<(), CsiError> {
    require_positive_finite("notch.center_frequency_rad_s", notch.center_frequency_rad_s)?;
    require_positive_finite("notch.zero_damping_ratio", notch.zero_damping_ratio)?;
    require_positive_finite("notch.pole_damping_ratio", notch.pole_damping_ratio)?;
    if notch.pole_damping_ratio <= notch.zero_damping_ratio {
        return Err(CsiError::InvalidNumber {
            field: "notch.pole_damping_ratio",
        });
    }
    Ok(())
}

fn require_lead_lag_fields(section: LeadLagSection) -> Result<(), CsiError> {
    require_positive_finite(
        "lead_lag.zero_frequency_rad_s",
        section.zero_frequency_rad_s,
    )?;
    require_positive_finite(
        "lead_lag.pole_frequency_rad_s",
        section.pole_frequency_rad_s,
    )?;
    require_positive_finite("lead_lag.gain", section.gain)
}

fn crosses_zero(previous: f64, current: f64) -> bool {
    (previous < 0.0 && current > 0.0) || (previous > 0.0 && current < 0.0)
}

fn require_positive_finite(field: &'static str, value: f64) -> Result<(), CsiError> {
    if !value.is_finite() || value <= 0.0 {
        return Err(CsiError::InvalidNumber { field });
    }
    Ok(())
}

fn require_nonnegative_finite(field: &'static str, value: f64) -> Result<(), CsiError> {
    if !value.is_finite() || value < 0.0 {
        return Err(CsiError::InvalidNumber { field });
    }
    Ok(())
}

fn require_finite(field: &'static str, value: f64) -> Result<(), CsiError> {
    if value.is_finite() {
        Ok(())
    } else {
        Err(CsiError::InvalidNumber { field })
    }
}

fn require_finite_complex(field: &'static str, value: ComplexResponse) -> Result<(), CsiError> {
    require_finite(field, value.re)?;
    require_finite(field, value.im)
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
    use crate::tables::{
        BendingFilterNode, BendingFilterSectionSpec, BendingFilterTable, GainInterp,
        ScheduleVariable, interpolate_bending_filter_table,
    };
    use toml::Value;

    const BENDING_FILTER_TUSTIN_TOLERANCE_TABLE: &str =
        include_str!("../../../data/gnc/bending-filter-tustin-tolerance-v1.toml");

    fn toml_table<'a>(value: &'a Value, key: &str) -> &'a toml::map::Map<String, Value> {
        value
            .get(key)
            .and_then(Value::as_table)
            .unwrap_or_else(|| panic!("missing TOML table {key}"))
    }

    fn toml_f64(value: &toml::map::Map<String, Value>, key: &str) -> f64 {
        value
            .get(key)
            .and_then(Value::as_float)
            .unwrap_or_else(|| panic!("missing TOML float {key}"))
    }

    fn toml_usize(value: &toml::map::Map<String, Value>, key: &str) -> usize {
        value
            .get(key)
            .and_then(Value::as_integer)
            .and_then(|raw| usize::try_from(raw).ok())
            .unwrap_or_else(|| panic!("missing TOML usize {key}"))
    }

    fn toml_str<'a>(value: &'a toml::map::Map<String, Value>, key: &str) -> &'a str {
        value
            .get(key)
            .and_then(Value::as_str)
            .unwrap_or_else(|| panic!("missing TOML string {key}"))
    }

    fn bending_table_from_fixture(data: &Value) -> BendingFilterTable {
        let schedule = toml_table(data, "schedule");
        let variable = match toml_str(schedule, "variable") {
            "Mach" => ScheduleVariable::Mach,
            other => panic!("unsupported bending schedule variable {other}"),
        };
        let interp = match toml_str(schedule, "interp") {
            "Linear" => GainInterp::Linear,
            "Smoothstep" => GainInterp::Smoothstep,
            other => panic!("unsupported bending schedule interp {other}"),
        };
        let nodes = schedule
            .get("node")
            .and_then(Value::as_array)
            .expect("bending filter nodes")
            .iter()
            .map(|node| {
                let node = node.as_table().expect("node table");
                let sections = node
                    .get("section")
                    .and_then(Value::as_array)
                    .expect("bending filter node sections")
                    .iter()
                    .map(|section| {
                        let section = section.as_table().expect("section table");
                        BendingFilterSectionSpec {
                            mode_frequency_rad_s: toml_f64(section, "mode_frequency_rad_s"),
                            zero_frequency_rad_s: toml_f64(section, "zero_frequency_rad_s"),
                            zero_damping_ratio: toml_f64(section, "zero_damping_ratio"),
                            pole_frequency_rad_s: toml_f64(section, "pole_frequency_rad_s"),
                            pole_damping_ratio: toml_f64(section, "pole_damping_ratio"),
                        }
                    })
                    .collect();
                BendingFilterNode {
                    node_id: toml_str(node, "node_id").to_string(),
                    schedule_value: toml_f64(node, "schedule_value"),
                    sections,
                }
            })
            .collect();
        BendingFilterTable {
            variable,
            interp,
            sample_interval_s: toml_f64(schedule, "dt_s"),
            nodes,
        }
    }

    #[test]
    fn csi_margin_reports_gain_and_phase_margin() {
        let gain = 3.0_f64;
        let numerator = [gain];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant = FrozenPlant::new(&numerator, &denominator).unwrap();
        let omega_gc = (gain.powf(2.0 / 3.0) - 1.0).sqrt();
        let frequencies = [0.25, omega_gc, 3.0_f64.sqrt(), 4.0];
        let report = analyze_frozen_plant(
            &plant,
            &frequencies,
            CsiMarginCriteria::new(6.0, 30.0).unwrap(),
        )
        .unwrap();
        let expected_phase_margin = 180.0 - 3.0 * omega_gc.atan().to_degrees();
        let expected_gain_margin = 20.0 * (8.0 / gain).log10();

        assert_eq!(report.verdict, CsiMarginVerdict::MeetsCriteria);
        assert!(
            (report.phase_margin_deg.unwrap() - expected_phase_margin).abs() < 1.0e-12,
            "phase margin was {:?}",
            report.phase_margin_deg
        );
        assert!(
            (report.gain_margin_db.unwrap() - expected_gain_margin).abs() < 1.0e-12,
            "gain margin was {:?}",
            report.gain_margin_db
        );
        assert_eq!(
            report.gain_crossover_frequency_rad_s.unwrap().to_bits(),
            omega_gc.to_bits()
        );
        assert_eq!(
            report.phase_crossover_frequency_rad_s.unwrap().to_bits(),
            3.0_f64.sqrt().to_bits()
        );
    }

    #[test]
    fn csi_margin_flags_below_threshold_case() {
        let gain = 6.0_f64;
        let numerator = [gain];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant = FrozenPlant::new(&numerator, &denominator).unwrap();
        let omega_gc = (gain.powf(2.0 / 3.0) - 1.0).sqrt();
        let frequencies = [0.25, omega_gc, 3.0_f64.sqrt(), 4.0];
        let report = analyze_frozen_plant(
            &plant,
            &frequencies,
            CsiMarginCriteria::new(6.0, 30.0).unwrap(),
        )
        .unwrap();

        assert_eq!(report.verdict, CsiMarginVerdict::BelowCriteria);
        assert!(report.gain_margin_db.unwrap() < 6.0);
        assert!(report.phase_margin_deg.unwrap() < 30.0);
    }

    #[test]
    fn csi_trajectory_sweep_reports_worst_frozen_time_margins() {
        let numerators = [[2.5_f64], [3.0_f64]];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant_0 = FrozenPlant::new(&numerators[0], &denominator).unwrap();
        let plant_1 = FrozenPlant::new(&numerators[1], &denominator).unwrap();
        let trajectory = [
            FrozenTrajectoryPoint::new(0.0, plant_0).unwrap(),
            FrozenTrajectoryPoint::new(5.0, plant_1).unwrap(),
        ];
        let omega_gc_0 = (2.5_f64.powf(2.0 / 3.0) - 1.0).sqrt();
        let omega_gc_1 = (3.0_f64.powf(2.0 / 3.0) - 1.0).sqrt();
        let frequencies = [0.25, omega_gc_0, omega_gc_1, 3.0_f64.sqrt(), 4.0];
        let mut reports = [empty_report(), empty_report()];
        let summary = analyze_frozen_trajectory(
            &trajectory,
            &frequencies,
            CsiMarginCriteria::new(6.0, 30.0).unwrap(),
            &mut reports,
        )
        .unwrap();

        assert_eq!(summary.verdict, CsiMarginVerdict::MeetsCriteria);
        assert_eq!(summary.point_count, 2);
        assert_eq!(reports[0].verdict, CsiMarginVerdict::MeetsCriteria);
        assert_eq!(reports[1].verdict, CsiMarginVerdict::MeetsCriteria);
        assert_eq!(
            summary.worst_gain_margin_time_s.unwrap().to_bits(),
            5.0_f64.to_bits()
        );
        assert_eq!(
            summary.worst_phase_margin_time_s.unwrap().to_bits(),
            5.0_f64.to_bits()
        );
        assert!(
            (summary.worst_gain_margin_db.unwrap() - reports[1].gain_margin_db.unwrap()).abs()
                < 1.0e-12
        );
        assert!(
            (summary.worst_phase_margin_deg.unwrap() - reports[1].phase_margin_deg.unwrap()).abs()
                < 1.0e-12
        );
        assert!(summary.min_distance_to_minus_one.is_finite());
        assert!(summary.peak_gain_db.is_finite());
    }

    #[test]
    fn csi_trajectory_sweep_flags_below_threshold_point() {
        let numerators = [[3.0_f64], [6.0_f64]];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant_0 = FrozenPlant::new(&numerators[0], &denominator).unwrap();
        let plant_1 = FrozenPlant::new(&numerators[1], &denominator).unwrap();
        let trajectory = [
            FrozenTrajectoryPoint::new(0.0, plant_0).unwrap(),
            FrozenTrajectoryPoint::new(4.0, plant_1).unwrap(),
        ];
        let omega_gc_0 = (3.0_f64.powf(2.0 / 3.0) - 1.0).sqrt();
        let omega_gc_1 = (6.0_f64.powf(2.0 / 3.0) - 1.0).sqrt();
        let frequencies = [0.25, omega_gc_0, omega_gc_1, 3.0_f64.sqrt(), 4.0];
        let mut reports = [empty_report(), empty_report()];
        let summary = analyze_frozen_trajectory(
            &trajectory,
            &frequencies,
            CsiMarginCriteria::new(6.0, 30.0).unwrap(),
            &mut reports,
        )
        .unwrap();

        assert_eq!(summary.verdict, CsiMarginVerdict::BelowCriteria);
        assert_eq!(reports[0].verdict, CsiMarginVerdict::MeetsCriteria);
        assert_eq!(reports[1].verdict, CsiMarginVerdict::BelowCriteria);
        assert_eq!(
            summary.worst_gain_margin_time_s.unwrap().to_bits(),
            4.0_f64.to_bits()
        );
        assert_eq!(
            summary.worst_phase_margin_time_s.unwrap().to_bits(),
            4.0_f64.to_bits()
        );
    }

    #[test]
    fn modal_csi_assembly_builds_single_rate_gyro_path() {
        let mode = ModalCsiMode::new(4.0, 0.1, 2.0, 3.0, 0.0, 5.0, 0.0).unwrap();
        let mut numerator = [0.0; 3];
        let mut denominator = [0.0; 3];
        let mut scratch = [0.0; 3];

        let count = assemble_modal_csi_plant_coefficients(
            &[mode],
            7.0,
            &mut numerator,
            &mut denominator,
            &mut scratch,
        )
        .unwrap();

        assert_eq!(count, 3);
        assert_eq!(numerator[0].to_bits(), 0.0_f64.to_bits());
        assert!((numerator[1] - 52.5).abs() < 1.0e-14);
        assert_eq!(numerator[2].to_bits(), 0.0_f64.to_bits());
        assert_eq!(denominator[0].to_bits(), 16.0_f64.to_bits());
        assert!((denominator[1] - 0.8).abs() < 1.0e-15);
        assert_eq!(denominator[2].to_bits(), 1.0_f64.to_bits());
    }

    #[test]
    fn modal_csi_assembly_matches_direct_two_mode_sum() {
        let modes = [
            ModalCsiMode::new(2.0, 0.05, 4.0, 3.0, 0.2, 0.3, 0.1).unwrap(),
            ModalCsiMode::new(5.0, 0.02, 2.0, -1.0, -0.1, 0.4, 0.0).unwrap(),
        ];
        let loop_gain = 1.7;
        let mut numerator = [0.0; 5];
        let mut denominator = [0.0; 5];
        let mut scratch = [0.0; 5];
        let count = assemble_modal_csi_plant_coefficients(
            &modes,
            loop_gain,
            &mut numerator,
            &mut denominator,
            &mut scratch,
        )
        .unwrap();
        let plant = FrozenPlant::new(&numerator[..count], &denominator[..count]).unwrap();

        for omega in [0.75, 1.25, 3.0] {
            let assembled = plant.response_at(omega).unwrap();
            let direct = direct_modal_csi_response(&modes, loop_gain, omega);
            assert!(
                (assembled.re - direct.re).abs() < 1.0e-12,
                "real mismatch at {omega}: {assembled:?} vs {direct:?}"
            );
            assert!(
                (assembled.im - direct.im).abs() < 1.0e-12,
                "imag mismatch at {omega}: {assembled:?} vs {direct:?}"
            );
        }
    }

    #[test]
    fn bending_notch_attenuates_modal_peak_and_exports_notch_config() {
        let notch = design_bending_notch(20.0, 0.02, 0.20).unwrap();
        let response = notch.response_at(20.0).unwrap();
        let config = notch.notch_config();

        assert!((response.magnitude() - 0.10).abs() < 1.0e-14);
        assert!((config.center_hz - (20.0 / std::f64::consts::TAU)).abs() < 1.0e-15);
        assert!((config.depth_db - 20.0).abs() < 1.0e-12);
        assert!(config.bandwidth_hz > 0.0);
    }

    #[test]
    fn bending_notch_reduces_deliberately_unstable_modal_gain_below_unity() {
        let plant =
            FrozenPlant::new(&[2.0 * 0.02 * 20.0 * 20.0 * 1.5], &[400.0, 0.8, 1.0]).unwrap();
        let notch = design_bending_notch(20.0, 0.02, 0.20).unwrap();
        let baseline_gain = plant.response_at(20.0).unwrap().magnitude();
        let filtered_gain = plant
            .response_at(20.0)
            .unwrap()
            .mul(notch.response_at(20.0).unwrap())
            .magnitude();

        assert!(baseline_gain > 1.0);
        assert!(filtered_gain < 1.0);
    }

    #[test]
    fn csi_filtered_structural_margin_fixture_meets_nodes_and_interior() {
        let data: Value = toml::from_str(BENDING_FILTER_TUSTIN_TOLERANCE_TABLE)
            .expect("bending filter table parses");
        let tolerance = toml_table(&data, "tolerances");
        let margin_epsilon_db = toml_f64(tolerance, "absolute_margin_db");
        let margin_epsilon_deg = toml_f64(tolerance, "absolute_margin_deg");
        let table = bending_table_from_fixture(&data);
        let margin = toml_table(&data, "analytic_margin");
        let frequency_start_rad_s = toml_f64(margin, "frequency_start_rad_s");
        let frequency_step_rad_s = toml_f64(margin, "frequency_step_rad_s");
        let frequency_count = toml_usize(margin, "frequency_count");
        let frequencies: Vec<_> = (0..frequency_count)
            .map(|index| frequency_start_rad_s + frequency_step_rad_s * index as f64)
            .collect();
        let criteria = CsiMarginCriteria::new(
            toml_f64(margin, "min_gain_margin_db"),
            toml_f64(margin, "min_phase_margin_deg"),
        )
        .unwrap();

        for sample in margin
            .get("sample")
            .and_then(Value::as_array)
            .expect("analytic margin samples")
        {
            let sample = sample.as_table().expect("analytic margin sample table");
            let sections =
                interpolate_bending_filter_table(&table, toml_f64(sample, "schedule_value"))
                    .expect("scheduled filter sections discretize");
            let modes = [
                ModalCsiMode::new(
                    toml_f64(sample, "slosh_frequency_rad_s"),
                    toml_f64(sample, "slosh_damping_ratio"),
                    1.0,
                    1.0,
                    1.0,
                    0.0,
                    0.0,
                )
                .unwrap(),
                ModalCsiMode::new(
                    toml_f64(sample, "bending_frequency_rad_s"),
                    toml_f64(sample, "bending_damping_ratio"),
                    1.0,
                    1.0,
                    1.0,
                    0.0,
                    0.0,
                )
                .unwrap(),
            ];
            let mut numerator = [0.0; 5];
            let mut denominator = [0.0; 5];
            let mut scratch = [0.0; 5];
            let coefficient_count = assemble_modal_csi_plant_coefficients(
                &modes,
                toml_f64(margin, "loop_gain"),
                &mut numerator,
                &mut denominator,
                &mut scratch,
            )
            .unwrap();
            let plant = FrozenPlant::new(
                &numerator[..coefficient_count],
                &denominator[..coefficient_count],
            )
            .unwrap();
            let report = analyze_filtered_frozen_plant(
                &plant,
                &sections,
                table.sample_interval_s,
                &frequencies,
                criteria,
            )
            .unwrap();

            assert_eq!(report.verdict, CsiMarginVerdict::MeetsCriteria);
            let gain_margin_db = report.gain_margin_db.unwrap();
            let phase_margin_deg = report.phase_margin_deg.unwrap();
            assert!(gain_margin_db >= criteria.min_gain_margin_db);
            assert!(phase_margin_deg >= criteria.min_phase_margin_deg);
            assert!(
                (gain_margin_db - toml_f64(sample, "expected_gain_margin_db")).abs()
                    <= margin_epsilon_db,
                "gain margin at schedule {} was {gain_margin_db}",
                toml_f64(sample, "schedule_value")
            );
            assert!(
                (phase_margin_deg - toml_f64(sample, "expected_phase_margin_deg")).abs()
                    <= margin_epsilon_deg,
                "phase margin at schedule {} was {phase_margin_deg}",
                toml_f64(sample, "schedule_value")
            );
        }
    }

    #[test]
    fn lead_lag_design_adds_requested_phase_with_unity_center_gain() {
        let lead = design_phase_lead_lag(10.0, 30.0).unwrap();
        let lag = design_phase_lead_lag(10.0, -20.0).unwrap();
        let lead_response = lead.response_at(10.0).unwrap();
        let lag_response = lag.response_at(10.0).unwrap();

        assert!((lead_response.magnitude() - 1.0).abs() < 1.0e-14);
        assert!((lead_response.phase_deg() - 30.0).abs() < 1.0e-12);
        assert!(lead.zero_frequency_rad_s < lead.pole_frequency_rad_s);
        assert!((lag_response.magnitude() - 1.0).abs() < 1.0e-14);
        assert!((lag_response.phase_deg() + 20.0).abs() < 1.0e-12);
        assert!(lag.zero_frequency_rad_s > lag.pole_frequency_rad_s);
    }

    #[test]
    fn lead_lag_coefficients_match_continuous_section_form() {
        let lead = design_phase_lead_lag(12.0, 25.0).unwrap();
        let numerator = lead.numerator_coefficients();
        let denominator = lead.denominator_coefficients();

        assert_eq!(numerator[0].to_bits(), lead.gain.to_bits());
        assert_eq!(
            numerator[1].to_bits(),
            (lead.gain / lead.zero_frequency_rad_s).to_bits()
        );
        assert_eq!(denominator[0].to_bits(), 1.0_f64.to_bits());
        assert_eq!(
            denominator[1].to_bits(),
            (1.0 / lead.pole_frequency_rad_s).to_bits()
        );
    }

    #[test]
    fn lead_lag_phase_lead_improves_cubic_lag_margin_at_crossover() {
        let gain = 6.0_f64;
        let numerator = [gain];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant = FrozenPlant::new(&numerator, &denominator).unwrap();
        let omega_gc = (gain.powf(2.0 / 3.0) - 1.0).sqrt();
        let baseline = plant.response_at(omega_gc).unwrap();
        let lead = design_phase_lead_lag(omega_gc, 20.0).unwrap();
        let compensated = baseline.mul(lead.response_at(omega_gc).unwrap());
        let baseline_margin = baseline.phase_deg_negative_feedback() + 180.0;
        let compensated_margin = compensated.phase_deg_negative_feedback() + 180.0;

        assert!((baseline.magnitude() - 1.0).abs() < 1.0e-12);
        assert!((compensated.magnitude() - 1.0).abs() < 1.0e-12);
        assert!((compensated_margin - baseline_margin - 20.0).abs() < 1.0e-12);
        assert!(compensated_margin > baseline_margin);
    }

    #[test]
    fn csi_helpers_reject_invalid_inputs() {
        let numerator = [3.0];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant = FrozenPlant::new(&numerator, &denominator).unwrap();

        assert!(FrozenPlant::new(&[], &[1.0]).is_err());
        assert!(FrozenPlant::new(&[1.0], &[0.0]).is_err());
        assert!(FrozenPlant::new(&[1.0], &[f64::NAN]).is_err());
        assert!(
            analyze_frozen_plant(
                &plant,
                &[1.0, 1.0],
                CsiMarginCriteria::new(6.0, 30.0).unwrap(),
            )
            .is_err()
        );
        assert!(CsiMarginCriteria::new(-1.0, 30.0).is_err());
        assert!(ModalCsiMode::new(0.0, 0.02, 10.0, 1.0, 0.0, 1.0, 0.0).is_err());
        let zero_coupling_mode = ModalCsiMode::new(2.0, 0.02, 10.0, 1.0, 0.0, 0.0, 0.0).unwrap();
        let mut short = [0.0; 2];
        let mut denominator = [0.0; 3];
        let mut scratch = [0.0; 3];
        assert!(
            assemble_modal_csi_plant_coefficients(
                &[zero_coupling_mode],
                1.0,
                &mut short,
                &mut denominator,
                &mut scratch,
            )
            .is_err()
        );
        let mut numerator = [0.0; 3];
        assert!(
            assemble_modal_csi_plant_coefficients(
                &[zero_coupling_mode],
                1.0,
                &mut numerator,
                &mut denominator,
                &mut scratch,
            )
            .is_err()
        );
        assert!(
            assemble_modal_csi_plant_coefficients(
                &[],
                1.0,
                &mut numerator,
                &mut denominator,
                &mut scratch,
            )
            .is_err()
        );
        assert!(design_bending_notch(20.0, 0.20, 0.02).is_err());
        assert!(design_phase_lead_lag(0.0, 10.0).is_err());
        assert!(design_phase_lead_lag(10.0, 90.0).is_err());
        assert!(
            LeadLagSection {
                zero_frequency_rad_s: 0.0,
                pole_frequency_rad_s: 1.0,
                gain: 1.0,
            }
            .response_at(1.0)
            .is_err()
        );
    }

    fn direct_modal_csi_response(
        modes: &[ModalCsiMode],
        loop_gain: f64,
        omega_rad_s: f64,
    ) -> ComplexResponse {
        let mut out = ComplexResponse::default();
        for mode in modes {
            let modal_gain = loop_gain * mode.input_coupling / mode.modal_mass_kg;
            let omega_n = mode.natural_frequency_rad_s;
            let denominator = ComplexResponse::new(
                (omega_n * omega_n) - (omega_rad_s * omega_rad_s),
                2.0 * mode.damping_ratio * omega_n * omega_rad_s,
            );
            let sensor = ComplexResponse::new(
                mode.sensor_displacement_coupling
                    - (mode.sensor_acceleration_coupling * omega_rad_s * omega_rad_s),
                mode.sensor_rate_coupling * omega_rad_s,
            );
            let term =
                ComplexResponse::new(modal_gain, 0.0).mul(sensor.divide_by(denominator).unwrap());
            out = out.add(term);
        }
        out
    }

    #[test]
    fn csi_trajectory_sweep_rejects_invalid_inputs() {
        let numerator = [3.0];
        let denominator = [1.0, 3.0, 3.0, 1.0];
        let plant = FrozenPlant::new(&numerator, &denominator).unwrap();
        let frequencies = [0.25, 1.0, 3.0_f64.sqrt(), 4.0];
        let mut reports = [empty_report()];

        assert!(
            analyze_frozen_trajectory(
                &[],
                &frequencies,
                CsiMarginCriteria::new(6.0, 30.0).unwrap(),
                &mut [],
            )
            .is_err()
        );
        assert!(
            analyze_frozen_trajectory(
                &[
                    FrozenTrajectoryPoint::new(1.0, plant).unwrap(),
                    FrozenTrajectoryPoint::new(1.0, plant).unwrap(),
                ],
                &frequencies,
                CsiMarginCriteria::new(6.0, 30.0).unwrap(),
                &mut [empty_report(), empty_report()],
            )
            .is_err()
        );
        assert!(
            analyze_frozen_trajectory(
                &[FrozenTrajectoryPoint::new(0.0, plant).unwrap()],
                &frequencies,
                CsiMarginCriteria::new(6.0, 30.0).unwrap(),
                &mut [empty_report(), empty_report()],
            )
            .is_err()
        );
        assert!(
            analyze_frozen_trajectory(
                &[FrozenTrajectoryPoint::new(0.0, plant).unwrap()],
                &[1.0, 1.0],
                CsiMarginCriteria::new(6.0, 30.0).unwrap(),
                &mut reports,
            )
            .is_err()
        );
    }

    fn empty_report() -> CsiMarginReport {
        CsiMarginReport {
            gain_margin_db: None,
            phase_margin_deg: None,
            gain_crossover_frequency_rad_s: None,
            phase_crossover_frequency_rad_s: None,
            min_distance_to_minus_one: 0.0,
            min_distance_frequency_rad_s: 0.0,
            peak_gain_db: 0.0,
            peak_gain_frequency_rad_s: 0.0,
            verdict: CsiMarginVerdict::BelowCriteria,
        }
    }
}
