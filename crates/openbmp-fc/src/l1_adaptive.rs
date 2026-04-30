//! L1 adaptive matched-uncertainty augmentation.
//!
//! This feature-gated module wraps the rate loop with the textbook
//! predictor / projection / low-pass correction structure described
//! by Cao & Hovakimyan (2010). The controller remains deterministic:
//! all state advances from the scheduler `dt` supplied by the caller.

/// Tuning for the scalar L1 adaptive augmentation.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct L1AdaptiveParams {
    /// Adaptation gain for the matched-uncertainty estimate.
    pub adaptation_gain: f64,
    /// First-order low-pass cutoff in Hz.
    pub low_pass_cutoff_hz: f64,
    /// Absolute projection bound for the matched uncertainty.
    pub sigma_bound: f64,
}

impl Default for L1AdaptiveParams {
    fn default() -> Self {
        Self {
            adaptation_gain: 25.0,
            low_pass_cutoff_hz: 10.0,
            sigma_bound: 1.0,
        }
    }
}

/// Runtime state for one scalar L1 augmentation channel.
#[derive(Copy, Clone, Debug, Default, PartialEq)]
pub struct L1AdaptiveChannel {
    sigma_hat: f64,
    filtered_correction: f64,
}

impl L1AdaptiveChannel {
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
        params: L1AdaptiveParams,
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
        let params = L1AdaptiveParams {
            adaptation_gain: 50.0,
            low_pass_cutoff_hz: 8.0,
            sigma_bound: 0.75,
        };
        let mut channel = L1AdaptiveChannel::new();
        for _ in 0..1_000 {
            let correction = channel.step(params, 0.0, 1.0, 0.001);
            assert!(correction.abs() <= params.sigma_bound + 1.0e-12);
            assert!(channel.sigma_hat().abs() <= params.sigma_bound + 1.0e-12);
        }
        assert!(channel.sigma_hat() > 0.70);
        assert!(channel.correction() < -0.70);
    }
}
