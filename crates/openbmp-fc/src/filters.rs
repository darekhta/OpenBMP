//! Deterministic controller-side digital filters.
//!
//! Phase 4.C adds a Direct-Form-II Transposed biquad notch primitive
//! for gyro conditioning. The module is pure arithmetic, carries no
//! time source, and is therefore compatible with the lockstep
//! controller clock.

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
}
