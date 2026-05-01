//! L1-inspired matched-uncertainty augmentation.
//!
//! This feature-gated module wraps the rate loop with a deterministic
//! scalar projection and low-pass correction channel. It borrows the
//! projection/filter shape used by L1 adaptive control literature, but
//! it is not a full Cao-Hovakimyan state-predictor/reference-model
//! implementation; that is Phase-5 work in `docs/phase-5-plan.md`.
//!
//! The feature flag is named `l1-adaptive` for downstream-API
//! stability; the in-tree types use the `L1Inspired*` naming so the
//! Rust API matches what is actually implemented.

/// Tuning for the scalar L1-inspired augmentation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct L1InspiredParams {
    /// Adaptation gain for the matched-uncertainty estimate.
    pub adaptation_gain: f64,
    /// First-order low-pass cutoff in Hz.
    pub low_pass_cutoff_hz: f64,
    /// Absolute projection bound for the matched uncertainty.
    pub sigma_bound: f64,
}

impl Default for L1InspiredParams {
    fn default() -> Self {
        Self {
            adaptation_gain: 25.0,
            low_pass_cutoff_hz: 10.0,
            sigma_bound: 1.0,
        }
    }
}

/// Runtime state for one scalar L1-inspired augmentation channel.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1InspiredChannel {
    sigma_hat: f64,
    filtered_correction: f64,
}

impl L1InspiredChannel {
    /// Constructs a zeroed channel.
    #[must_use]
    pub const fn new() -> Self {
        Self {
            sigma_hat: 0.0,
            filtered_correction: 0.0,
        }
    }

    /// Returns the current matched-uncertainty estimate.
    #[must_use]
    pub const fn sigma_hat(self) -> f64 {
        self.sigma_hat
    }

    /// Returns the low-pass filtered correction.
    #[must_use]
    pub const fn correction(self) -> f64 {
        self.filtered_correction
    }

    /// Advances the channel by one scheduler tick and returns the
    /// correction that should be added to the baseline PID command.
    #[must_use]
    pub fn step(
        &mut self,
        params: L1InspiredParams,
        tracking_error: f64,
        measured_disturbance: f64,
        dt_s: f64,
    ) -> f64 {
        if !dt_s.is_finite() || dt_s <= 0.0 {
            return self.filtered_correction;
        }
        let estimate_dot =
            params.adaptation_gain * (measured_disturbance - self.sigma_hat + tracking_error);
        self.sigma_hat =
            (self.sigma_hat + estimate_dot * dt_s).clamp(-params.sigma_bound, params.sigma_bound);

        let omega = 2.0 * core::f64::consts::PI * params.low_pass_cutoff_hz.max(0.0);
        let alpha = 1.0 - (-omega * dt_s).exp();
        self.filtered_correction += alpha * (-self.sigma_hat - self.filtered_correction);
        self.filtered_correction
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounded_unit_step_disturbance_stays_projected() {
        let params = L1InspiredParams {
            adaptation_gain: 50.0,
            low_pass_cutoff_hz: 8.0,
            sigma_bound: 0.75,
        };
        let mut channel = L1InspiredChannel::new();
        for _ in 0..1_000 {
            let correction = channel.step(params, 0.0, 1.0, 0.001);
            assert!(correction.abs() <= params.sigma_bound + 1.0e-12);
            assert!(channel.sigma_hat().abs() <= params.sigma_bound + 1.0e-12);
        }
        assert!(channel.sigma_hat() > 0.70);
        assert!(channel.correction() < -0.70);
    }
}
