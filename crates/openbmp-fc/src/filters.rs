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

#[cfg(test)]
#[allow(clippy::unwrap_used)]
mod tests {
    use super::*;

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
}
