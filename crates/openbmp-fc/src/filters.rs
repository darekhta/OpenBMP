//! Deterministic controller-side digital filters.
//!
//! Provides a Direct-Form-II Transposed biquad notch primitive
//! for gyro conditioning. The module is pure arithmetic, carries no
//! time source, and is therefore compatible with the lockstep
//! controller clock.

#[cfg(not(feature = "std"))]
use num_traits::Float;

/// Per-axis notch-filter configuration.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct NotchConfig {
    /// Notch centre frequency (Hz).
    pub center_hz: f64,
    /// Approximate -3 dB bandwidth (Hz).
    pub bandwidth_hz: f64,
    /// Requested notch depth (dB). The biquad zeros sit on the unit
    /// circle; this field is retained for configuration provenance
    /// and test acceptance.
    pub depth_db: f64,
}

/// Stateless normalized biquad section coefficients.
///
/// Coefficients are in direct-form order:
/// `H(z) = (b[0] + b[1] z^-1 + b[2] z^-2) /
///         (a[0] + a[1] z^-1 + a[2] z^-2)`, with `a[0] == 1`.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct BiquadSection {
    /// Numerator coefficients.
    pub b: [f64; 3],
    /// Denominator coefficients. `a[0]` must be one.
    pub a: [f64; 3],
}

impl BiquadSection {
    /// Convert stateless coefficients into the stateful runtime filter.
    #[must_use]
    pub fn into_biquad(self) -> Biquad {
        Biquad {
            b0: self.b[0],
            b1: self.b[1],
            b2: self.b[2],
            a1: self.a[1],
            a2: self.a[2],
            z1: 0.0,
            z2: 0.0,
        }
    }

    /// Returns `true` when the section is normalized, finite, and has
    /// denominator roots strictly inside the unit circle.
    #[must_use]
    pub fn has_stable_poles(self) -> bool {
        self.b.iter().all(|value| value.is_finite())
            && self.a.iter().all(|value| value.is_finite())
            && (self.a[0] - 1.0).abs() <= 1.0e-12
            && stable_second_order_denominator(self.a[1], self.a[2])
    }

    /// Magnitude response at a frequency in Hz.
    #[must_use]
    pub fn magnitude_at_hz(self, frequency_hz: f64, sample_rate_hz: f64) -> f64 {
        self.into_biquad()
            .magnitude_at_hz(frequency_hz, sample_rate_hz)
    }

    /// Magnitude response at a frequency in rad/s.
    ///
    /// # Errors
    ///
    /// Returns `Err` when the frequency or sample interval is invalid.
    pub fn magnitude_at_rad_s(self, omega_rad_s: f64, dt_s: f64) -> Result<f64, &'static str> {
        if !omega_rad_s.is_finite()
            || !dt_s.is_finite()
            || omega_rad_s < 0.0
            || dt_s <= 0.0
            || omega_rad_s >= std::f64::consts::PI / dt_s
        {
            return Err("invalid response frequency");
        }
        Ok(self.magnitude_at_hz(omega_rad_s / std::f64::consts::TAU, 1.0 / dt_s))
    }
}

/// Prewarped-Tustin discretization of one continuous second-order
/// bending/slosh notch section.
///
/// Continuous prototype:
/// `H(s) = (s^2 + 2 ζ_n ω_n s + ω_n^2) /
///         (s^2 + 2 ζ_d ω_d s + ω_d^2)`.
/// The bilinear substitution uses `s = K (z - 1) / (z + 1)`,
/// `K = ω_mode / tan(ω_mode dt / 2)`, preserving the requested
/// mode-frequency response.
///
/// # Errors
///
/// Returns `Err` when frequencies, damping ratios, or sample interval
/// are invalid, when the mode is at/above Nyquist, or when the
/// generated section is not stable.
pub fn prewarped_tustin_notch(
    omega_n: f64,
    zeta_n: f64,
    omega_d: f64,
    zeta_d: f64,
    omega_mode: f64,
    dt_s: f64,
) -> Result<BiquadSection, &'static str> {
    if !omega_n.is_finite()
        || !zeta_n.is_finite()
        || !omega_d.is_finite()
        || !zeta_d.is_finite()
        || !omega_mode.is_finite()
        || !dt_s.is_finite()
        || omega_n <= 0.0
        || omega_d <= 0.0
        || omega_mode <= 0.0
        || dt_s <= 0.0
        || zeta_n <= 0.0
        || zeta_d <= 0.0
        || zeta_n >= zeta_d
    {
        return Err("invalid prewarped notch parameters");
    }

    let nyquist_rad_s = std::f64::consts::PI / dt_s;
    if omega_n >= nyquist_rad_s || omega_d >= nyquist_rad_s || omega_mode >= nyquist_rad_s {
        return Err("notch frequencies must be < Nyquist");
    }

    let half_step_mode = 0.5 * omega_mode * dt_s;
    let tan_half_step_mode = half_step_mode.tan();
    if tan_half_step_mode <= f64::MIN_POSITIVE || !tan_half_step_mode.is_finite() {
        return Err("invalid prewarp constant");
    }
    let k = omega_mode / tan_half_step_mode;

    let numerator = bilinear_second_order_coefficients(omega_n, zeta_n, k);
    let denominator = bilinear_second_order_coefficients(omega_d, zeta_d, k);
    if denominator[0].abs() <= f64::MIN_POSITIVE || !denominator[0].is_finite() {
        return Err("singular prewarped notch denominator");
    }

    let section = BiquadSection {
        b: [
            numerator[0] / denominator[0],
            numerator[1] / denominator[0],
            numerator[2] / denominator[0],
        ],
        a: [
            1.0,
            denominator[1] / denominator[0],
            denominator[2] / denominator[0],
        ],
    };
    if !section.has_stable_poles() {
        return Err("prewarped notch generated unstable poles");
    }
    Ok(section)
}

/// Direct-Form-II Transposed biquad.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct Biquad {
    b0: f64,
    b1: f64,
    b2: f64,
    a1: f64,
    a2: f64,
    z1: f64,
    z2: f64,
}

impl Biquad {
    /// Construct a stateful filter from normalized section coefficients.
    ///
    /// # Errors
    ///
    /// Returns `Err` when the section is not normalized, finite, and stable.
    pub fn from_section(section: BiquadSection) -> Result<Self, &'static str> {
        if !section.has_stable_poles() {
            return Err("invalid biquad section");
        }
        Ok(Self {
            b0: section.b[0],
            b1: section.b[1],
            b2: section.b[2],
            a1: section.a[1],
            a2: section.a[2],
            z1: 0.0,
            z2: 0.0,
        })
    }

    /// Replace normalized section coefficients while preserving the
    /// filter's internal delay state.
    ///
    /// # Errors
    ///
    /// Returns `Err` when the replacement section is invalid or unstable.
    pub fn replace_section(&mut self, section: BiquadSection) -> Result<(), &'static str> {
        if !section.has_stable_poles() {
            return Err("invalid biquad section");
        }
        self.b0 = section.b[0];
        self.b1 = section.b[1];
        self.b2 = section.b[2];
        self.a1 = section.a[1];
        self.a2 = section.a[2];
        Ok(())
    }

    /// Construct a notch filter.
    ///
    /// # Errors
    ///
    /// Returns `Err` when frequency, bandwidth, or sample rate are
    /// outside their physically meaningful ranges.
    pub fn notch(config: NotchConfig, sample_rate_hz: f64) -> Result<Self, &'static str> {
        if !config.center_hz.is_finite()
            || !config.bandwidth_hz.is_finite()
            || !sample_rate_hz.is_finite()
            || config.center_hz <= 0.0
            || config.bandwidth_hz <= 0.0
            || sample_rate_hz <= 0.0
            || config.center_hz >= 0.5 * sample_rate_hz
        {
            return Err("invalid notch filter configuration");
        }
        let omega = 2.0 * std::f64::consts::PI * config.center_hz / sample_rate_hz;
        let q = (config.center_hz / config.bandwidth_hz).max(f64::EPSILON);
        let alpha = omega.sin() / (2.0 * q);
        let cos_omega = omega.cos();
        let a0 = 1.0 + alpha;
        Ok(Self {
            b0: 1.0 / a0,
            b1: -2.0 * cos_omega / a0,
            b2: 1.0 / a0,
            a1: -2.0 * cos_omega / a0,
            a2: (1.0 - alpha) / a0,
            z1: 0.0,
            z2: 0.0,
        })
    }

    /// Push one scalar sample through the filter.
    #[must_use]
    pub fn step(&mut self, x: f64) -> f64 {
        let y = self.b0 * x + self.z1;
        self.z1 = self.b1 * x - self.a1 * y + self.z2;
        self.z2 = self.b2 * x - self.a2 * y;
        y
    }

    /// Construct a first-order low-pass filter via the bilinear
    /// transform.
    ///
    /// Continuous-time prototype: `H(s) = ω_c / (s + ω_c)`. The
    /// bilinear transform with prewarping produces a stable digital
    /// filter for any positive cutoff strictly below Nyquist.
    ///
    /// # Errors
    ///
    /// Returns `Err` when `cutoff_rad_s` or `sample_rate_hz` is
    /// non-positive or non-finite, or when the cutoff is at or above
    /// the Nyquist frequency.
    pub fn lowpass_first_order(
        cutoff_rad_s: f64,
        sample_rate_hz: f64,
    ) -> Result<Self, &'static str> {
        if !cutoff_rad_s.is_finite()
            || !sample_rate_hz.is_finite()
            || cutoff_rad_s <= 0.0
            || sample_rate_hz <= 0.0
        {
            return Err("invalid first-order low-pass parameters");
        }
        let nyquist_rad_s = std::f64::consts::PI * sample_rate_hz;
        if cutoff_rad_s >= nyquist_rad_s {
            return Err("first-order low-pass cutoff must be < Nyquist");
        }
        let dt = 1.0 / sample_rate_hz;
        let k = (cutoff_rad_s * dt * 0.5).tan();
        let denom = 1.0 + k;
        Ok(Self {
            b0: k / denom,
            b1: k / denom,
            b2: 0.0,
            a1: (k - 1.0) / denom,
            a2: 0.0,
            z1: 0.0,
            z2: 0.0,
        })
    }

    /// Construct a second-order Butterworth low-pass filter via the
    /// bilinear transform.
    ///
    /// Continuous-time prototype: `H(s) = ω_c² / (s² + √2 · ω_c · s + ω_c²)`.
    /// Maximally flat magnitude in the passband; -40 dB / decade
    /// rolloff above the cutoff.
    ///
    /// # Errors
    ///
    /// Returns `Err` when `cutoff_rad_s` or `sample_rate_hz` is
    /// non-positive or non-finite, or when the cutoff is at or above
    /// the Nyquist frequency.
    pub fn butterworth_lowpass_second_order(
        cutoff_rad_s: f64,
        sample_rate_hz: f64,
    ) -> Result<Self, &'static str> {
        if !cutoff_rad_s.is_finite()
            || !sample_rate_hz.is_finite()
            || cutoff_rad_s <= 0.0
            || sample_rate_hz <= 0.0
        {
            return Err("invalid second-order Butterworth parameters");
        }
        let nyquist_rad_s = std::f64::consts::PI * sample_rate_hz;
        if cutoff_rad_s >= nyquist_rad_s {
            return Err("Butterworth low-pass cutoff must be < Nyquist");
        }
        let dt = 1.0 / sample_rate_hz;
        let k = (cutoff_rad_s * dt * 0.5).tan();
        let k_sq = k * k;
        let sqrt_two = std::f64::consts::SQRT_2;
        let denom = 1.0 + sqrt_two * k + k_sq;
        Ok(Self {
            b0: k_sq / denom,
            b1: 2.0 * k_sq / denom,
            b2: k_sq / denom,
            a1: 2.0 * (k_sq - 1.0) / denom,
            a2: (1.0 - sqrt_two * k + k_sq) / denom,
            z1: 0.0,
            z2: 0.0,
        })
    }

    /// Magnitude response at a frequency in Hz.
    #[must_use]
    pub fn magnitude_at_hz(&self, frequency_hz: f64, sample_rate_hz: f64) -> f64 {
        let omega = 2.0 * std::f64::consts::PI * frequency_hz / sample_rate_hz;
        let z1_re = omega.cos();
        let z1_im = -omega.sin();
        let z2_re = (2.0 * omega).cos();
        let z2_im = -(2.0 * omega).sin();
        let num_re = self.b0 + self.b1 * z1_re + self.b2 * z2_re;
        let num_im = self.b1 * z1_im + self.b2 * z2_im;
        let den_re = 1.0 + self.a1 * z1_re + self.a2 * z2_re;
        let den_im = self.a1 * z1_im + self.a2 * z2_im;
        (num_re.hypot(num_im)) / (den_re.hypot(den_im))
    }
}

fn bilinear_second_order_coefficients(omega_rad_s: f64, zeta: f64, k: f64) -> [f64; 3] {
    let k_sq = k * k;
    let omega_sq = omega_rad_s * omega_rad_s;
    let damping = 2.0 * zeta * omega_rad_s * k;
    [
        k_sq + damping + omega_sq,
        -2.0 * k_sq + 2.0 * omega_sq,
        k_sq - damping + omega_sq,
    ]
}

fn stable_second_order_denominator(a1: f64, a2: f64) -> bool {
    let eps = 1.0e-12;
    (1.0 + a1 + a2) > eps && (1.0 - a1 + a2) > eps && (1.0 - a2) > eps
}

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;
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

    fn toml_f64_array(value: &toml::map::Map<String, Value>, key: &str) -> Vec<f64> {
        value
            .get(key)
            .and_then(Value::as_array)
            .unwrap_or_else(|| panic!("missing TOML array {key}"))
            .iter()
            .map(|entry| {
                entry
                    .as_float()
                    .unwrap_or_else(|| panic!("TOML array {key} contains non-float"))
            })
            .collect()
    }

    fn assert_close_array(actual: &[f64], expected: &[f64], epsilon: f64, label: &str) {
        assert_eq!(actual.len(), expected.len(), "{label} length");
        for (index, (actual, expected)) in actual.iter().zip(expected).enumerate() {
            assert!(
                (*actual - *expected).abs() <= epsilon,
                "{label}[{index}] actual {actual:e}, expected {expected:e}, epsilon {epsilon:e}"
            );
        }
    }

    #[test]
    fn notch_has_deep_null_at_center_frequency() {
        let config = NotchConfig {
            center_hz: 50.0,
            bandwidth_hz: 5.0,
            depth_db: 20.0,
        };
        let filter = Biquad::notch(config, 1_000.0).unwrap();
        let mag = filter.magnitude_at_hz(config.center_hz, 1_000.0);
        assert!(
            20.0 * mag.max(1.0e-12).log10() < -config.depth_db,
            "magnitude at notch center was {mag}"
        );
    }

    #[test]
    fn lowpass_first_order_dc_gain_is_unity() {
        let filter = Biquad::lowpass_first_order(50.0, 1_000.0).unwrap();
        let mag = filter.magnitude_at_hz(0.0, 1_000.0);
        assert!((mag - 1.0).abs() < 1.0e-12, "DC gain was {mag}");
    }

    #[test]
    fn lowpass_first_order_attenuates_high_frequencies() {
        let filter = Biquad::lowpass_first_order(50.0, 1_000.0).unwrap();
        let cutoff_hz = 50.0 / (2.0 * std::f64::consts::PI);
        let mag_at_cutoff = filter.magnitude_at_hz(cutoff_hz, 1_000.0);
        // First-order LPF has -3 dB at the cutoff (≈ 0.707).
        assert!(
            (mag_at_cutoff - 1.0_f64 / std::f64::consts::SQRT_2).abs() < 1.0e-2,
            "magnitude at cutoff was {mag_at_cutoff}"
        );
        // Well above cutoff, attenuation should be substantial.
        let mag_high = filter.magnitude_at_hz(10.0 * cutoff_hz, 1_000.0);
        assert!(
            mag_high < 0.15,
            "magnitude well above cutoff was {mag_high}"
        );
    }

    #[test]
    fn lowpass_first_order_rejects_invalid_inputs() {
        assert!(Biquad::lowpass_first_order(-1.0, 1_000.0).is_err());
        assert!(Biquad::lowpass_first_order(0.0, 1_000.0).is_err());
        assert!(Biquad::lowpass_first_order(50.0, 0.0).is_err());
        assert!(Biquad::lowpass_first_order(f64::NAN, 1_000.0).is_err());
        // Cutoff at Nyquist must fail.
        assert!(Biquad::lowpass_first_order(2.0 * std::f64::consts::PI * 500.0, 1_000.0).is_err());
    }

    #[test]
    fn butterworth_lowpass_dc_gain_is_unity() {
        let filter = Biquad::butterworth_lowpass_second_order(50.0, 1_000.0).unwrap();
        let mag = filter.magnitude_at_hz(0.0, 1_000.0);
        assert!((mag - 1.0).abs() < 1.0e-12, "DC gain was {mag}");
    }

    #[test]
    fn butterworth_lowpass_is_minus_three_db_at_cutoff() {
        let filter = Biquad::butterworth_lowpass_second_order(50.0, 1_000.0).unwrap();
        let cutoff_hz = 50.0 / (2.0 * std::f64::consts::PI);
        let mag = filter.magnitude_at_hz(cutoff_hz, 1_000.0);
        // Maximally flat at -3 dB at the cutoff (≈ 0.707).
        assert!(
            (mag - 1.0_f64 / std::f64::consts::SQRT_2).abs() < 5.0e-3,
            "magnitude at cutoff was {mag}"
        );
    }

    #[test]
    fn butterworth_lowpass_rejects_invalid_inputs() {
        assert!(Biquad::butterworth_lowpass_second_order(-1.0, 1_000.0).is_err());
        assert!(Biquad::butterworth_lowpass_second_order(50.0, 0.0).is_err());
        assert!(
            Biquad::butterworth_lowpass_second_order(2.0 * std::f64::consts::PI * 500.0, 1_000.0)
                .is_err()
        );
    }

    #[test]
    fn prewarped_tustin_notch_matches_continuous_mode_response() {
        let section = prewarped_tustin_notch(20.0, 0.02, 20.0, 0.2, 20.0, 0.01).unwrap();
        assert!(section.has_stable_poles());
        let magnitude = section.magnitude_at_rad_s(20.0, 0.01).unwrap();
        assert!(
            (magnitude - 0.1).abs() < 1.0e-12,
            "mode magnitude was {magnitude}"
        );
    }

    #[test]
    fn prewarped_tustin_notch_rejects_invalid_and_unstable_inputs() {
        assert!(prewarped_tustin_notch(20.0, 0.2, 20.0, 0.02, 20.0, 0.01).is_err());
        assert!(
            prewarped_tustin_notch(20.0, 0.02, 20.0, 0.2, std::f64::consts::PI / 0.01, 0.01)
                .is_err()
        );
        let unstable = BiquadSection {
            b: [1.0, 0.0, 0.0],
            a: [1.0, 0.0, 1.2],
        };
        assert!(!unstable.has_stable_poles());
    }

    #[test]
    fn bending_filter_tustin_tolerance_table_matches_coefficients() {
        let data: Value = toml::from_str(BENDING_FILTER_TUSTIN_TOLERANCE_TABLE)
            .expect("bending filter table parses");
        assert_eq!(
            toml_table(&data, "openbmp")
                .get("gnc_bending_filter_tustin_reference")
                .and_then(Value::as_integer),
            Some(1)
        );
        let tolerance = toml_table(&data, "tolerances");
        let coefficient_epsilon = toml_f64(tolerance, "absolute_coefficient");
        let magnitude_epsilon = toml_f64(tolerance, "absolute_magnitude");
        let query = toml_table(&data, "query");
        let section = prewarped_tustin_notch(
            toml_f64(query, "zero_frequency_rad_s"),
            toml_f64(query, "zero_damping_ratio"),
            toml_f64(query, "pole_frequency_rad_s"),
            toml_f64(query, "pole_damping_ratio"),
            toml_f64(query, "mode_frequency_rad_s"),
            toml_f64(query, "dt_s"),
        )
        .unwrap();
        assert_close_array(
            &section.b,
            &toml_f64_array(query, "expected_b"),
            coefficient_epsilon,
            "prewarped b",
        );
        assert_close_array(
            &section.a,
            &toml_f64_array(query, "expected_a"),
            coefficient_epsilon,
            "prewarped a",
        );
        let mode_magnitude = section
            .magnitude_at_rad_s(
                toml_f64(query, "mode_frequency_rad_s"),
                toml_f64(query, "dt_s"),
            )
            .unwrap();
        assert!(
            (mode_magnitude - toml_f64(query, "expected_mode_magnitude")).abs()
                <= magnitude_epsilon,
            "mode magnitude was {mode_magnitude}"
        );
    }
}
